#ifndef DIFF_PLANNER_TRAJECTORY_SAMPLING_H_
#define DIFF_PLANNER_TRAJECTORY_SAMPLING_H_

#include <optimizer/poly_traj_utils.hpp>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace diff_planner
{
// Sample every polynomial piece including its junctions. The analytical speed
// bound guarantees both the temporal and spatial sampling limits, even when an
// initialization or candidate exceeds the configured flight speed.
inline std::vector<std::pair<double, Eigen::Vector3d>> sampleCollisionTrajectory(
    const poly_traj::Trajectory &trajectory, double begin, double end,
    double max_distance, double max_dt = 0.05)
{
  std::vector<std::pair<double, Eigen::Vector3d>> samples;
  if (trajectory.getPieceNum() == 0 || !std::isfinite(begin) ||
      !std::isfinite(end) || !std::isfinite(max_distance) || max_distance <= 0.0 ||
      !std::isfinite(max_dt) || max_dt <= 0.0)
    return samples;
  begin = std::max(0.0, begin);
  end = std::min(end, trajectory.getTotalDuration());
  if (end < begin) return samples;
  double piece_start = 0.0;
  for (int piece_id = 0; piece_id < trajectory.getPieceNum(); ++piece_id)
  {
    const auto piece = trajectory.getPiece(piece_id);
    const double duration = piece.getDuration();
    if (!std::isfinite(duration) || duration <= 0.0) return {};
    const double from = std::max(begin, piece_start);
    const double to = std::min(end, piece_start + duration);
    if (from <= to)
    {
      const double speed = std::max(piece.getMaxVelRate(),
          std::max(piece.getVel(0.0).norm(), piece.getVel(duration).norm()));
      if (!std::isfinite(speed) || speed < 0.0) return {};
      const double step = std::min(max_dt, max_distance / std::max(1.0e-6, speed));
      const double count = std::max(1.0, std::ceil((to - from) / step));
      if (!std::isfinite(count) || samples.size() + count > 1000000) return {};
      for (size_t i = 0; i <= static_cast<size_t>(count); ++i)
      {
        const double time = from + (to - from) * i / count;
        if (!samples.empty() && std::abs(samples.back().first - (time - begin)) < 1.0e-12)
          continue;
        const Eigen::Vector3d position = piece.getPos(time - piece_start);
        if (!position.allFinite()) return {};
        samples.emplace_back(time - begin, position);
      }
    }
    piece_start += duration;
    if (piece_start >= end) break;
  }
  return samples;
}
} // namespace diff_planner
#endif
