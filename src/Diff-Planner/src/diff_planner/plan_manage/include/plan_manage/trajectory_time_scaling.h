#pragma once

#include <algorithm>
#include <cmath>
#include <optimizer/poly_traj_utils.hpp>

namespace diff_planner {
// 统一拉长各段时间，保留完整几何路径和段间连续性。
// p_new(t)=p_old(t/scale)，各阶导数由同一多项式自然得到。
inline poly_traj::Trajectory scaleTrajectoryTime(
    const poly_traj::Trajectory &trajectory, double max_velocity,
    double max_acceleration) {
  if (trajectory.getPieceNum() == 0 ||
      (max_velocity <= 0.0 && max_acceleration <= 0.0)) return trajectory;
  double scale = 1.0;
  if (max_velocity > 0.0)
    scale = std::max(scale, trajectory.getMaxVelRate() / max_velocity);
  if (max_acceleration > 0.0)
    scale = std::max(scale, std::sqrt(trajectory.getMaxAccRate() / max_acceleration));
  if (scale <= 1.0) return trajectory;
  scale *= 1.0 + 1.e-6;  // 极值求解及系数序列化的浮点余量。
  poly_traj::Trajectory result;
  for (const auto &piece : trajectory) {
    auto coefficients = piece.getCoeffMat();
    double divisor = scale;
    for (int column = 4; column >= 0; --column) {
      coefficients.col(column) /= divisor;
      divisor *= scale;
    }
    result.emplace_back(piece.getDuration() * scale, coefficients);
  }
  return result;
}
}  // namespace diff_planner
