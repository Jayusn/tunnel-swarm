#pragma once

#include <optimizer/poly_traj_utils.hpp>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace diff_planner {

// A route request contains only ordinary FIFO points. Each polynomial piece
// ends at the corresponding original point; intermediate velocities are free.
// Reject curves leaving that piece's route corridor instead of cutting corners.
struct OrdinaryRouteResult {
  poly_traj::MinJerkOpt polynomial;
  std::size_t count{0};
  double min_separation{std::numeric_limits<double>::infinity()};
  std::string reason{"no_clear_stop"};
};

inline bool buildOrdinaryRoute(
    const Eigen::Vector3d& start, const Eigen::Vector3d& velocity,
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& leader,
    double separation, double max_velocity, double max_acceleration,
    double resolution,
    const std::function<bool(const Eigen::Vector3d&)>& clear_stop,
    const std::function<bool(const Eigen::Vector3d&)>& clear_path,
    OrdinaryRouteResult& result) {
  result.count = 0;
  result.min_separation = std::numeric_limits<double>::infinity();
  result.reason = "no_clear_stop";
  if (!start.allFinite() || !velocity.allFinite() || !leader.allFinite() ||
      max_velocity <= 0.0 || max_acceleration <= 0.0 || resolution <= 0.0 ||
      velocity.norm() > max_velocity + 1e-6) {
    result.reason = "invalid_start";
    return false;
  }
  for (const auto& point : points)
    if (!point.allFinite()) { result.reason = "invalid_route"; return false; }

  for (std::size_t count = 1; count <= points.size(); ++count) {
    if (!clear_stop(points[count - 1])) continue;
    // Subdivide the original legs before fitting: a single long polynomial
    // at a right-angle turn bows far outside the route even when its vertices
    // are exact. These support points keep the smooth turn inside one voxel.
    std::vector<Eigen::Vector3d> support;
    Eigen::Vector3d previous = start;
    for (std::size_t i = 0; i < count; ++i) {
      const Eigen::Vector3d delta = points[i] - previous;
      const int pieces = std::max(1, static_cast<int>(std::ceil(delta.norm() / (2 * resolution))));
      for (int j = 1; j <= pieces; ++j)
        support.push_back(previous + (static_cast<double>(j) / pieces) * delta);
      previous = points[i];
    }
    const std::size_t pieces = support.size();
    Eigen::Matrix3d head, tail;
    head << start, velocity, Eigen::Vector3d::Zero();
    tail << points[count - 1], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    Eigen::MatrixXd inner(3, pieces - 1);
    Eigen::VectorXd times(pieces);
    previous = start;
    for (std::size_t i = 0; i < pieces; ++i) {
      if (i + 1 < pieces) inner.col(i) = support[i];
      times(i) = std::max(0.2, 2.0 * (support[i] - previous).norm() / max_velocity);
      previous = support[i];
    }
    poly_traj::MinJerkOpt polynomial;
    bool bounded = false;
    for (int attempt = 0; attempt < 8; ++attempt) {
      polynomial.reset(head, tail, pieces);
      polynomial.generate(inner, times);
      const auto curve = polynomial.getTraj();
      const double ratio = std::max(curve.getMaxVelRate() / max_velocity,
                                   std::sqrt(curve.getMaxAccRate() / max_acceleration));
      if (!std::isfinite(ratio)) break;
      if (ratio <= 1.0 + 1e-6) { bounded = true; break; }
      times *= std::max(1.1, ratio * 1.02);
    }
    if (!bounded) { result.reason = "kinematics"; continue; }
    const auto curve = polynomial.getTraj();
    bool valid = true;
    double minimum = std::numeric_limits<double>::infinity();
    previous = start;
    for (std::size_t piece = 0; piece < pieces && valid; ++piece) {
      const Eigen::Vector3d delta = support[piece] - previous;
      const double dt = std::min(0.02, 0.5 * resolution / max_velocity);
      const int samples = std::max(1, static_cast<int>(std::ceil(times(piece) / dt)));
      double furthest = 0.0;
      for (int j = 0; j <= samples; ++j) {
        const Eigen::Vector3d p = curve[piece].getPos(times(piece) * j / samples);
        const double projection = delta.squaredNorm() > 1e-12
            ? (p - previous).dot(delta) / delta.squaredNorm() : 0.0;
        const double fraction = std::max(0.0, std::min(1.0, projection));
        // At most one map voxel of geometric deviation; never allow a large
        // detour/shortcut or reversal across the ordered original waypoints.
        if (!p.allFinite() || (p - previous - fraction * delta).norm() > resolution ||
            (furthest - projection) * delta.norm() > resolution) {
          result.reason = "route_corner"; valid = false; break;
        }
        furthest = std::max(furthest, projection);
        const double distance = (p - leader).head<2>().norm();
        minimum = std::min(minimum, distance);
        if (distance + 1e-6 < separation) {
          result.reason = "leader_spacing"; valid = false; break;
        }
        if (!clear_path(p)) { result.reason = "path_occupied"; valid = false; break; }
      }
      previous = support[piece];
    }
    if (!valid) continue;
    result.polynomial = polynomial;
    result.count = count;
    result.min_separation = minimum;
    result.reason = "ready";
    return true;
  }
  return false;
}
}  // namespace diff_planner
