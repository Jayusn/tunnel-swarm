#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace fast_planner {
namespace passage_centering {

struct Config {
  double step{0.05};
  double section_spacing{0.20};
  double search_radius{0.90};
  double maximum_shift{0.45};
};

// Scan only the connected free interval containing this position. The callback
// checks the vehicle footprint at the SAME height, not a 3-D ESDF dominated by
// the ground. A blocked/unknown strip is never crossed to reach another lane.
template <typename KnownFree>
Eigen::Vector3d sectionCentre(const Eigen::Vector3d& point,
                              const Eigen::Vector3d& tangent, double shift_limit,
                              const Config& cfg, KnownFree&& known_free) {
  if (tangent.head<2>().norm() < 1e-6 || !known_free(point)) return point;
  const Eigen::Vector3d side(-tangent.y() / tangent.head<2>().norm(),
                             tangent.x() / tangent.head<2>().norm(), 0.0);
  double bounds[2]{0.0, 0.0};
  for (int direction = 0; direction < 2; ++direction) {
    const double sign = direction == 0 ? -1.0 : 1.0;
    for (double d = cfg.step; d <= cfg.search_radius + 1e-6; d += cfg.step) {
      if (!known_free(point + sign * d * side)) break;
      bounds[direction] = sign * d;
    }
  }
  const double offset = std::max(-shift_limit, std::min(shift_limit,
                                    0.5 * (bounds[0] + bounds[1])));
  return point + offset * side;
}

template <typename KnownFree>
bool knownSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                  double step, KnownFree&& known_free) {
  const int count = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
  for (int i = 0; i <= count; ++i)
    if (!known_free(a + (b - a) * (static_cast<double>(i) / count))) return false;
  return true;
}

// Keep the true start/goal and original corners. Re-sample before moving inner
// points so even a straight two-point route can acquire a central guide.
template <typename KnownFree, typename PathSafe>
bool centrePath(std::vector<Eigen::Vector3d>& path, double horizon,
                const Config& cfg, KnownFree&& known_free, PathSafe&& path_safe) {
  if (path.size() < 2) return false;
  std::vector<Eigen::Vector3d> guide{path.front()};
  std::vector<double> progress{0.0};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d delta = path[i] - path[i - 1];
    const double length = delta.norm();
    if (length < 1e-3) continue;
    const int count = std::max(1, static_cast<int>(std::ceil(length / cfg.section_spacing)));
    for (int j = 1; j <= count; ++j) {
      guide.push_back(path[i - 1] + delta * (static_cast<double>(j) / count));
      progress.push_back(progress.back() + length / count);
    }
  }
  const auto original = guide;
  bool changed = false;
  for (std::size_t i = 1; i + 1 < guide.size() && progress[i] <= horizon; ++i) {
    // Blend in/out rather than commanding an abrupt lateral jump at a fixed
    // start/goal. Pure vertical sections keep their original XY.
    const double limit = std::min(cfg.maximum_shift,
        0.6 * std::min(progress[i], progress.back() - progress[i]));
    const Eigen::Vector3d candidate = sectionCentre(
        original[i], original[i + 1] - original[i - 1], limit, cfg, known_free);
    if ((candidate - original[i]).norm() < 0.5 * cfg.step) continue;
    // A section can be clear while its connection cuts a corner. Try a smaller
    // correction, retaining the original route if no connected correction fits.
    for (double scale : {1.0, 0.5, 0.25}) {
      const Eigen::Vector3d adjusted = original[i] + scale * (candidate - original[i]);
      if (knownSegment(guide[i - 1], adjusted, 0.5 * cfg.step, known_free) &&
          knownSegment(adjusted, guide[i + 1], 0.5 * cfg.step, known_free) &&
          path_safe(std::vector<Eigen::Vector3d>{guide[i - 1], adjusted, guide[i + 1]})) {
        guide[i] = adjusted;
        changed = true;
        break;
      }
    }
  }
  if (changed && path_safe(guide)) path.swap(guide);
  else changed = false;
  return changed;
}

}  // namespace passage_centering
}  // namespace fast_planner
