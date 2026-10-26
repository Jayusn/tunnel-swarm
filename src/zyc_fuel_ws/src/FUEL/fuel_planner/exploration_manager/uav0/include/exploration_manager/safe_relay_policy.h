#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace fast_planner {
namespace safe_relay {

// Candidates follow measured route progress, but their XYZ may be improved
// within nearby known free space. No hovering/dwell evidence is required.
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
  double maximum_gap{1.50};
};

enum class Kind { WAIT, NORMAL, AFTER_OBSTACLE, BEFORE_OBSTACLE, GAP_FALLBACK };
struct Selection {
  std::size_t index{std::numeric_limits<std::size_t>::max()};
  Kind kind{Kind::WAIT};
};

// Qualify spatial endpoints relative to the latest released goal and progress.
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

  std::vector<std::size_t> candidates, gap_candidates;
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
              cfg.minimum_xy_spacing)
        continue;
      gap_candidates.push_back(i);
      if (route.back().progress - point.progress + 1e-6 < cfg.release_distance ||
          (leader.head<2>() - point.position.head<2>()).norm() + 1e-6 <
              cfg.release_distance)
        continue;
      // A transit point just past the spacing threshold must not become a
      // waiting goal if nearby mapped stopping space is appreciably wider.
      // Compare qualified neighbours even when they are too close to A or the
      // leader to release: otherwise a narrow point would win by eligibility
      // alone. Neighbours need not all be safe; isolated safe B still qualifies.
      constexpr double clearance_tolerance = 0.02;
      std::size_t first = i;
      while (first > anchor + 1 &&
             point.progress - route[first - 1].progress <= cfg.selection_window + 1e-6)
        --first;
      bool better_neighbour = false;
      for (std::size_t j = first; j < route.size() &&
           route[j].progress - point.progress <= cfg.selection_window + 1e-6; ++j) {
        if (route[j].hold_free && route[j].confirmed &&
            route[j].clearance > point.clearance + clearance_tolerance + 1e-6) {
          better_neighbour = true;
          break;
        }
      }
      if (better_neighbour) continue;
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
    // First bound selection to a local route window; within that window choose
    // maximum horizontal voxel clearance, not an offset-dominated mixed score.
    const double preferred = kind == Kind::BEFORE_OBSTACLE
                                 ? route[pool.back()].progress
                                 : route[pool.front()].progress;
    std::size_t best = kind == Kind::BEFORE_OBSTACLE ? pool.back() : pool.front();
    for (const auto i : pool) {
      if (std::fabs(route[i].progress - preferred) > cfg.selection_window + 1e-6) continue;
      if (route[i].clearance > route[best].clearance + 1e-6 ||
          (std::fabs(route[i].clearance - route[best].clearance) <= 1e-6 &&
           std::fabs(route[i].progress - preferred) <
               std::fabs(route[best].progress - preferred))) best = i;
    }
    result.index = best;
    result.kind = kind;
    return true;
  };

  // Ordinary endpoints have priority; only B itself must qualify.
  if (choose(Kind::NORMAL)) return result;
  if (first_narrow != route.size()) {
    if (choose(Kind::AFTER_OBSTACLE)) return result;
    if (choose(Kind::BEFORE_OBSTACLE)) return result;
  }
  // A long gap must not wait indefinitely for a local clearance peak or for
  // the leader to move 1m beyond it. Allow a qualified current/nearby endpoint;
  // execution and real-time inter-vehicle separation belong to the follower.
  if (route.back().progress - start + 1e-6 >= cfg.maximum_gap && !gap_candidates.empty()) {
    std::size_t best = gap_candidates.front();
    for (const auto i : gap_candidates) {
      if (route[i].clearance > route[best].clearance + 1e-6 ||
          (std::fabs(route[i].clearance - route[best].clearance) <= 1e-6 &&
           route[i].progress > route[best].progress)) best = i;
    }
    result.index = best;
    result.kind = Kind::GAP_FALLBACK;
  }
  return result;
}

inline const char* name(Kind kind) {
  switch (kind) {
    case Kind::NORMAL: return "NORMAL";
    case Kind::AFTER_OBSTACLE: return "AFTER_OBSTACLE";
    case Kind::BEFORE_OBSTACLE: return "BEFORE_OBSTACLE_WAIT";
    case Kind::GAP_FALLBACK: return "GAP_FALLBACK";
    default: return "WAIT_NO_VALID_TARGET";
  }
}
}  // namespace safe_relay
}  // namespace fast_planner
