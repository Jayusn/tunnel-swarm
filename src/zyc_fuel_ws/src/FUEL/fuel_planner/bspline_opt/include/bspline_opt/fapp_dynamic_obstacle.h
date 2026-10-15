#ifndef FUEL_FAPP_DYNAMIC_OBSTACLE_H_
#define FUEL_FAPP_DYNAMIC_OBSTACLE_H_

#include <Eigen/Eigen>

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
  double vehicle_half_height{0.125};
  double ball_radius{0.10};
  double margin{0.10};
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
