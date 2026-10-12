#ifndef DIFF_PLANNER_DYNAMIC_OBSTACLE_H_
#define DIFF_PLANNER_DYNAMIC_OBSTACLE_H_

#include <Eigen/Eigen>

namespace diff_planner
{

struct DynamicObstacleState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
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
  // Measured minimum ball bottom relative to the ground; used only for the
  // optimizer's underpass preference, never to alter measured ball coordinates.
  double minimum_ball_bottom{0.80};
};

// CV prediction follows FAPP's timed-object cost:
// https://github.com/arclab-hku/FAPP/blob/main/src/planner/traj_opt/src/poly_traj_optimizer.cpp
// The geometry here is a sphere against a finite vertical vehicle cylinder.
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

} // namespace diff_planner

#endif
