#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace fast_planner {
namespace safe_relay {

// Only measured odometry samples enter this list. No trajectory endpoints,
// interpolated goals or hovering/dwell requirements are used for selection.
struct Sample {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double progress{0.0};
  double clearance{0.0};
  bool hold_free{false};
  bool confirmed{false};
};

struct Config {
  double normal_spacing{1.00};
  double minimum_progress{0.50};
  double minimum_xy_spacing{0.70};
  double release_distance{1.00};
  double selection_window{0.50};
};

enum class Kind { WAIT, NORMAL, AFTER_OBSTACLE, BEFORE_OBSTACLE };
struct Selection {
  std::size_t index{std::numeric_limits<std::size_t>::max()};
  Kind kind{Kind::WAIT};
};

// Qualify new measured endpoints relative to the latest released sample.
// The follower owns connection planning; old route occupancy does not veto B.
inline Selection select(const std::vector<Sample>& route, std::size_t anchor,
                        const Eigen::Vector3d& leader, const Config& cfg) {
  Selection result;
  if (route.size() < 2 || anchor >= route.size() || !leader.allFinite()) return result;
  const double start = route[anchor].progress;
  std::size_t first_narrow = route.size();
  for (std::size_t i = anchor + 1; i < route.size(); ++i) {
    if (!route[i].hold_free) { first_narrow = i; break; }
  }

  std::vector<std::size_t> candidates;
  for (std::size_t begin = anchor; begin < route.size();) {
    if (!route[begin].hold_free || !route[begin].confirmed) { ++begin; continue; }
    std::size_t end = begin;
    while (end + 1 < route.size() && route[end + 1].hold_free &&
           route[end + 1].confirmed) ++end;
    for (std::size_t i = begin; i <= end; ++i) {
      const auto& point = route[i];
      // Only the candidate needs stopping clearance. Other historical samples
      // do not certify or veto the follower's future connection to this point.
      if (point.progress - start + 1e-6 < cfg.minimum_progress ||
          (point.position.head<2>() - route[anchor].position.head<2>()).norm() + 1e-6 <
              cfg.minimum_xy_spacing ||
          route.back().progress - point.progress + 1e-6 < cfg.release_distance ||
          (leader.head<2>() - point.position.head<2>()).norm() + 1e-6 <
              cfg.release_distance)
        continue;
      candidates.push_back(i);
    }
    begin = end + 1;
  }

  auto choose = [&](Kind kind) {
    std::vector<std::size_t> pool;
    for (const auto i : candidates) {
      if (kind == Kind::AFTER_OBSTACLE && i <= first_narrow) continue;
      if (kind == Kind::BEFORE_OBSTACLE && i >= first_narrow) continue;
      if (kind == Kind::NORMAL &&
          route[i].progress - start + 1e-6 < cfg.normal_spacing) continue;
      pool.push_back(i);
    }
    if (pool.empty()) return false;
    // Prefer a clear measured sample near the first available post-obstacle
    // region / ordinary spacing. Waiting points advance as far as safely possible.
    const double preferred = kind == Kind::BEFORE_OBSTACLE
                                 ? route[pool.back()].progress
                                 : route[pool.front()].progress;
    std::stable_sort(pool.begin(), pool.end(), [&](std::size_t a, std::size_t b) {
      auto score = [&](std::size_t i) {
        const double offset = std::fabs(route[i].progress - preferred);
        return (offset <= cfg.selection_window ? 10.0 : 0.0) +
               std::min(1.0, route[i].clearance) - offset;
      };
      return score(a) > score(b);
    });
    result.index = pool.front();
    result.kind = kind;
    return true;
  };

  // Ordinary endpoints have priority; only B itself must qualify.
  if (choose(Kind::NORMAL)) return result;
  if (first_narrow != route.size()) {
    if (choose(Kind::AFTER_OBSTACLE)) return result;
    choose(Kind::BEFORE_OBSTACLE);
  }
  return result;
}

inline const char* name(Kind kind) {
  switch (kind) {
    case Kind::NORMAL: return "NORMAL";
    case Kind::AFTER_OBSTACLE: return "AFTER_OBSTACLE";
    case Kind::BEFORE_OBSTACLE: return "BEFORE_OBSTACLE_WAIT";
    default: return "WAIT_NO_VALID_TARGET";
  }
}
}  // namespace safe_relay
}  // namespace fast_planner
