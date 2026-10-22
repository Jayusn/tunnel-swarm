#pragma once
#include <Eigen/Core>
#include <cmath>

namespace mot_mapping {
// A real published observation, independent of tracker ID. Geometry is refreshed
// only by strong motion confirmation; weak fragments cannot enlarge it repeatedly.
struct SpatialConfirmation {
  double stamp = 0.0;
  double motion_stamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d confirmed_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();

  bool supports(double now, const Eigen::Vector3d& measured_position,
                const Eigen::Vector3d& measured_size, double max_gap, double radius) const {
    if (!std::isfinite(now) || !std::isfinite(stamp) || stamp <= 0.0 ||
        now <= stamp || now - stamp > max_gap ||
        !measured_position.allFinite() || !measured_size.allFinite() ||
        (measured_size.array() < 0.0).any() ||
        (measured_position - position).norm() > radius ||
        (measured_position - confirmed_position).norm() > radius) return false;
    // A small body may be partly visible, but a newly attached rope/wall is not
    // the same measured body just because its centroid is nearby.
    const Eigen::Vector3d limit = (2.0 * size.array() + 0.05).max(0.30).matrix();
    return (measured_size.array() <= limit.array()).all();
  }
};
}  // namespace mot_mapping
