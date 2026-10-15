#pragma once

#include <algorithm>
#include <cmath>

namespace diff_planner {
namespace entry_point_to_point {

struct Sample {
  double position, velocity, acceleration, jerk;
};

// 静止到静止的五次时间缩放。导数峰值分别为 15/8 和 10/sqrt(3)。
inline double duration(double distance, double max_speed, double max_acceleration) {
  return std::max(0.02, std::max(1.875 * distance / max_speed,
      std::sqrt((10.0 / std::sqrt(3.0)) * distance / max_acceleration)));
}

inline Sample sample(double elapsed, double duration) {
  if (elapsed < 0.0) return {0.0, 0.0, 0.0, 0.0};
  if (elapsed >= duration) return {1.0, 0.0, 0.0, 0.0};
  const double u = elapsed / duration;
  return {u*u*u*(10.0 + u*(-15.0 + 6.0*u)),
          30.0*u*u*(1.0-u)*(1.0-u) / duration,
          60.0*u*(1.0-u)*(1.0-2.0*u) / (duration*duration),
          (60.0 - 360.0*u + 360.0*u*u) / (duration*duration*duration)};
}

}  // namespace entry_point_to_point
}  // namespace diff_planner
