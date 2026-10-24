#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {
namespace route_width {

// 分流路线的候选信息；只在轨迹发布后转为现有的绕行侧承诺。
struct SplitRoute {
  int side{0};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d forward{Eigen::Vector3d::UnitX()};
  double entry{0.0}, exit{0.0}, offset{0.0};
};

inline int routeSide(const Eigen::Vector3d& delta, const Eigen::Vector3d& forward) {
  const double offset = delta.dot(Eigen::Vector3d(-forward.y(), forward.x(), 0));
  return offset > 0.10 ? 1 : (offset < -0.10 ? -1 : 0);
}

inline bool sameAlternative(const Eigen::Vector3d& previous, int previous_side,
                            const Eigen::Vector3d& previous_forward,
                            const Eigen::Vector3d& target, int side,
                            const Eigen::Vector3d& forward) {
  return previous_side == side && previous_forward.dot(forward) > 0.95 &&
         (target - previous).head<2>().norm() <= 0.35;
}

inline bool followsSplit(const std::vector<Eigen::Vector3d>& path,
                         const SplitRoute& lane) {
  if (!lane.side || path.size() < 2 ||
      (path.back() - lane.origin).dot(lane.forward) < lane.exit) return false;
  const Eigen::Vector3d lateral(-lane.forward.y(), lane.forward.x(), 0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const int n = std::max(1, static_cast<int>(std::ceil((path[i]-path[i-1]).norm()/0.025)));
    for (int j = 0; j <= n; ++j) {
      const Eigen::Vector3d p = path[i-1] + (path[i]-path[i-1])*(double(j)/n) - lane.origin;
      const double progress = p.dot(lane.forward);
      if (progress >= lane.entry && progress <= lane.exit &&
          lane.side * p.dot(lateral) < lane.offset - 1e-6) return false;
    }
  }
  return true;
}

// Width is horizontal known-free space at the flight height, not 3-D ESDF
// (which can be dominated by the floor). All routes use the same forward axis.
struct Score {
  bool complete{false};
  double minimum{0.0};
  double mean{0.0};
  double clearance{0.0};
};

inline bool prefer(const Score& candidate, const Score& incumbent,
                   double tolerance = 0.05) {
  if (!candidate.complete) return false;
  if (!incumbent.complete) return true;
  if (std::fabs(candidate.minimum - incumbent.minimum) > tolerance)
    return candidate.minimum > incumbent.minimum;
  if (std::fabs(candidate.mean - incumbent.mean) > tolerance)
    return candidate.mean > incumbent.mean;
  return candidate.clearance > incumbent.clearance + tolerance;
}

inline bool materiallyWider(const Score& candidate, const Score& current,
                            double gain) {
  // Lack of map evidence is not evidence that another route is wider.
  return candidate.complete && current.complete &&
         candidate.minimum + 0.01 >= current.minimum &&
         (candidate.minimum >= current.minimum + gain - 1e-6 ||
          candidate.mean >= current.mean + gain - 1e-6) &&
         candidate.clearance + 0.01 >= current.clearance;
}

// The caller supplies a connected path reaching horizon metres of FORWARD
// progress. A 20cm side step cannot compete as a complete one-metre passage.
// Ignore the shared first 20cm in ranking, but validate it with the rest of the
// path before calling this function. Include every later section and endpoint.
template <typename KnownFree>
Score measure(const std::vector<Eigen::Vector3d>& path,
              const Eigen::Vector3d& direction, double horizon,
              double lateral_range, double step, KnownFree&& known_free) {
  Score result;
  if (path.size() < 2 || direction.head<2>().norm() < 1e-6 ||
      horizon <= 0.0 || step <= 0.0) return result;
  const Eigen::Vector3d forward(direction.x() / direction.head<2>().norm(),
                                 direction.y() / direction.head<2>().norm(), 0.0);
  const Eigen::Vector3d side(-forward.y(), forward.x(), 0.0);
  double next_section = std::min(0.20, horizon);
  double minimum = std::numeric_limits<double>::infinity();
  double clearance = minimum, sum = 0.0;
  int count = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double a = (path[i-1] - path.front()).dot(forward);
    const double b = (path[i] - path.front()).dot(forward);
    if (b <= a + 1e-6) continue;
    while (next_section <= b + 1e-6) {
      const double fraction = std::max(0.0, std::min(1.0,
          (next_section - a) / (b - a)));
      const Eigen::Vector3d p = path[i-1] + fraction * (path[i] - path[i-1]);
      if (!known_free(p)) return Score{};
      double extent[2]{0.0, 0.0};
      for (int sign : {-1, 1}) {
        for (double d = step; d <= lateral_range + 1e-6; d += step) {
          if (!known_free(p + sign * d * side)) break;
          extent[sign > 0 ? 1 : 0] = d;
        }
      }
      const double width = extent[0] + extent[1];
      minimum = std::min(minimum, width);
      clearance = std::min(clearance, std::min(extent[0], extent[1]));
      sum += width;
      ++count;
      if (next_section >= horizon - 1e-6) {
        result.complete = true;
        result.minimum = minimum;
        result.mean = sum / count;
        result.clearance = clearance;
        return result;
      }
      next_section = std::min(horizon, next_section + 0.10);
    }
  }
  return result;
}

}  // namespace route_width
}  // namespace fast_planner
