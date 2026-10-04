#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace fast_planner {
struct OdomReviewSample {
  double stamp{0.0};
  std::array<double, 3> position;
  std::array<double, 3> velocity;
};

inline bool reviewOdometry(const std::deque<OdomReviewSample>& samples,
                          std::array<double, 3>& position,
                          std::array<double, 3>& velocity) {
  if (samples.size() != 5 || samples.front().stamp <= 0.0)
    return false;
  for (size_t i = 0; i < samples.size(); ++i) {
    if (i > 0 && samples[i].stamp <= samples[i - 1].stamp) return false;
    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(samples[i].position[axis]) ||
          !std::isfinite(samples[i].velocity[axis]))
        return false;
    }
  }
  for (int axis = 0; axis < 3; ++axis) {
    std::array<double, 5> positions, velocities;
    for (size_t i = 0; i < samples.size(); ++i) {
      positions[i] = samples[i].position[axis];
      velocities[i] = samples[i].velocity[axis];
    }
    std::sort(positions.begin(), positions.end());
    std::sort(velocities.begin(), velocities.end());
    position[axis] = positions[2];
    velocity[axis] = velocities[2];
  }
  return true;
}
}  // namespace fast_planner
