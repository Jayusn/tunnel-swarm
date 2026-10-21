#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Core>

namespace PayloadMPC {
// Search only +/-1 s around the execution time, not unrelated path branches.
// Project onto sampled line segments, avoiding a sample-spacing error floor.
template <class Trajectory>
double localPathDistance(const Trajectory& trajectory, double time,
                         const Eigen::Vector3d& position) {
  const double duration = trajectory.getTotalDuration();
  if (!position.allFinite() || !std::isfinite(time) || duration <= 0.0)
    return std::numeric_limits<double>::infinity();
  time = std::max(0.0, std::min(duration, time));
  const double begin = std::max(0.0, time - 1.0);
  const double end = std::min(duration, time + 1.0);
  const int steps = std::max(1, static_cast<int>(std::ceil((end - begin) / 0.02)));
  Eigen::Vector3d a = trajectory.getPos(begin);
  double distance = (position - a).norm();
  for (int i = 1; i <= steps; ++i) {
    const Eigen::Vector3d b = trajectory.getPos(begin + (end - begin) * i / steps);
    const Eigen::Vector3d d = b - a;
    const double u = d.squaredNorm() > 1e-12
        ? std::max(0.0, std::min(1.0, (position - a).dot(d) / d.squaredNorm())) : 0.0;
    distance = std::min(distance, (position - a - u * d).norm());
    a = b;
  }
  return distance;
}
}
