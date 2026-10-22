#pragma once
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace mot_mapping {
// Point sets are expressed relative to their measured centers. Aligning their
// centers compares the two observations of this track, never the whole scene.
struct LocalMotionMatch {
  bool valid = false;
  double forward_moving = 0.0, forward_static = 0.0;
  double reverse_moving = 0.0, reverse_static = 0.0;

  bool supports(double match, double advantage) const {
    return valid && forward_moving >= match && reverse_moving >= match &&
        forward_moving - forward_static >= advantage &&
        reverse_moving - reverse_static >= advantage;
  }
};

inline LocalMotionMatch matchLocalMotion(
    const std::vector<Eigen::Vector3d>& previous,
    const std::vector<Eigen::Vector3d>& current,
    const Eigen::Vector3d& displacement) {
  LocalMotionMatch result;
  if (previous.empty() || current.empty() || !displacement.allFinite()) return result;
  for (const auto& point : previous) if (!point.allFinite()) return result;
  for (const auto& point : current) if (!point.allFinite()) return result;
  const double squared_distance = 0.05 * 0.05;
  size_t fm = 0, fs = 0, rm = 0, rs = 0;
  for (const auto& q : current) {
    bool moving = false, stationary = false;
    for (const auto& p : previous) {
      moving |= (q - p).squaredNorm() <= squared_distance;
      stationary |= (q + displacement - p).squaredNorm() <= squared_distance;
      if (moving && stationary) break;
    }
    fm += moving; fs += stationary;
  }
  for (const auto& p : previous) {
    bool moving = false, stationary = false;
    for (const auto& q : current) {
      moving |= (p - q).squaredNorm() <= squared_distance;
      stationary |= (p - displacement - q).squaredNorm() <= squared_distance;
      if (moving && stationary) break;
    }
    rm += moving; rs += stationary;
  }
  result.valid = true;
  result.forward_moving = double(fm) / current.size();
  result.forward_static = double(fs) / current.size();
  result.reverse_moving = double(rm) / previous.size();
  result.reverse_static = double(rs) / previous.size();
  return result;
}
}  // namespace mot_mapping
