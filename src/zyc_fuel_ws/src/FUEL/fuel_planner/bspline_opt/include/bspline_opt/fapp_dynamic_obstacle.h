#ifndef FUEL_FAPP_DYNAMIC_OBSTACLE_H_
#define FUEL_FAPP_DYNAMIC_OBSTACLE_H_

#include <Eigen/Eigen>
#include <algorithm>

namespace fuel_fapp
{

struct DynamicObstacleState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  // Measured full AABB extents; all-zero or invalid extents use the sphere fallback.
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double stamp{0.0};
};

struct DynamicObstacleConfig
{
  bool enabled{false};
  double vehicle_radius{0.262};
  // Extents above/below the reported vehicle reference point.
  double vehicle_top_height{0.125};
  double vehicle_bottom_height{0.125};
  double ball_radius{0.10};
  double margin{0.10};
  // 下穿时原始障碍底部到飞行参考点的总间距；负值保留旧机体+margin语义。
  double underpass_reference_clearance{-1.0};
  double lowerReferenceClearance() const
  {
    return underpass_reference_clearance >= 0.0
        ? std::max(vehicle_top_height, underpass_reference_clearance)
        : vehicle_top_height + margin;
  }
  double observation_retention{0.80};
  double prediction_horizon{1.0};
  // Legacy launch compatibility only. Passage height now follows observed
  // geometry and static free space, not a fixed ground-relative ball bottom.
  double minimum_ball_bottom{0.80};
};

// CV prediction follows FAPP's timed-object cost:
// https://github.com/arclab-hku/FAPP/blob/main/src/planner/traj_opt/src/poly_traj_optimizer.cpp
// Valid measured extents use an AABB against a finite vertical vehicle cylinder;
// missing or invalid extents retain the configured sphere fallback.
// reference_time checks observation age; query_time is the trajectory sample's
// absolute time and may be in the future. A valid observation stays valid for
// every sample in one planning evaluation. The gradient is with respect to the
// vehicle position; negative clearance means the padded envelopes overlap.
bool evaluateDynamicObstacle(
    const Eigen::Vector3d &vehicle_position,
    const DynamicObstacleState &state, double query_time, double reference_time,
    const DynamicObstacleConfig &config, double *clearance,
    Eigen::Vector3d *clearance_gradient = nullptr,
    Eigen::Vector3d *obstacle_position = nullptr,
    Eigen::Vector3d *obstacle_velocity = nullptr);

} // namespace fuel_fapp

#endif
