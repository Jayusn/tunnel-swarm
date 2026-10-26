#pragma once

#include <algorithm>
#include <cmath>

namespace diff_planner {
namespace entry_point_to_point {

struct Sample {
  double position, velocity, acceleration, jerk;
};

// 两段四次位置多项式：前半程加速，后半程减速。
// 中点速度最大，起点、中点、终点加速度均为零，段间位置/速度/加速度连续。
inline double duration(double distance, double max_speed, double max_acceleration) {
  const double half_duration = std::max(distance / max_speed,
      std::sqrt(1.5 * distance / max_acceleration));
  return 2.0 * std::max(0.01, half_duration);
}

inline Sample sample(double elapsed, double duration) {
  if (elapsed < 0.0) return {0.0, 0.0, 0.0, 0.0};
  if (elapsed >= duration) return {1.0, 0.0, 0.0, 0.0};
  const double half = duration * 0.5;
  const bool decelerating = elapsed >= half;
  const double u = decelerating ? (duration-elapsed)/half : elapsed/half;
  const double p = u*u*u*(1.0-0.5*u);
  return {decelerating ? 1.0-p : p,
          u*u*(3.0-2.0*u)/half,
          (decelerating ? -1.0 : 1.0)*6.0*u*(1.0-u)/(half*half),
          (6.0-12.0*u)/(half*half*half)};
}

}  // namespace entry_point_to_point
}  // namespace diff_planner
