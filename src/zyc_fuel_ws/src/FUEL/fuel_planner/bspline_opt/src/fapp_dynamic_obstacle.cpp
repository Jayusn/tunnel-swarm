#include <bspline_opt/fapp_dynamic_obstacle.h>
#include <bspline_opt/fapp_vehicle_geometry.h>

#include <algorithm>
#include <cmath>

namespace fuel_fapp
{

bool evaluateDynamicObstacle(
    const Eigen::Vector3d &vehicle_position,
    const DynamicObstacleState &state, const double query_time,
    const double reference_time, const DynamicObstacleConfig &config,
    double *clearance, Eigen::Vector3d *clearance_gradient,
    Eigen::Vector3d *obstacle_position, Eigen::Vector3d *obstacle_velocity)
{
  const bool measured_box = state.size.allFinite() &&
      (state.size.array() >= 0.0).all() && (state.size.array() > 0.0).any();
  if (!config.enabled || !vehicle_position.allFinite() ||
      !state.position.allFinite() || !state.velocity.allFinite() ||
      !std::isfinite(state.stamp) || !std::isfinite(query_time) ||
      !std::isfinite(reference_time) ||
      !std::isfinite(config.vehicle_radius) || config.vehicle_radius <= 0.0 ||
      !std::isfinite(config.vehicle_top_height) || config.vehicle_top_height <= 0.0 ||
      !std::isfinite(config.vehicle_bottom_height) || config.vehicle_bottom_height <= 0.0 ||
      (!measured_box &&
       (!std::isfinite(config.ball_radius) || config.ball_radius <= 0.0)) ||
      !std::isfinite(config.margin) || config.margin < 0.0 ||
      !std::isfinite(config.underpass_reference_clearance) ||
      !std::isfinite(config.observation_retention) ||
      config.observation_retention < 0.0 ||
      !std::isfinite(config.prediction_horizon) ||
      config.prediction_horizon < 0.0)
    return false;

  const double age = std::max(0.0, reference_time - state.stamp);
  if (age > config.observation_retention)
    return false;

  const double raw_prediction_time = query_time - state.stamp;
  const double prediction_time = std::max(
      0.0, std::min(raw_prediction_time, config.prediction_horizon));
  Eigen::Vector3d predicted_position =
      state.position + prediction_time * state.velocity;
  Eigen::Vector3d predicted_velocity = state.velocity;
  if (raw_prediction_time < 0.0 ||
      raw_prediction_time >= config.prediction_horizon)
    predicted_velocity.setZero();

  if (obstacle_position != nullptr)
    *obstacle_position = predicted_position;
  if (obstacle_velocity != nullptr)
    *obstacle_velocity = predicted_velocity;

  // 下穿下沿统一采用“原始底部-总间距”，不再叠加机体顶部和通用margin。
  // 仅截取旧包络的底面；水平侧面、上方净空以及运动预测保持原计算。
  const auto underpass_clearance = [&](double original) {
    if (config.underpass_reference_clearance < 0.0) return original;
    const double half_z = measured_box ? 0.5 * state.size.z() : config.ball_radius;
    double gap = predicted_position.z() - half_z -
        config.lowerReferenceClearance() - vehicle_position.z();
    if (std::fabs(gap) < 1e-9) gap = 0.0;  // 仅消除边界浮点舍入。
    if (gap > original) {
      if (clearance_gradient != nullptr)
        *clearance_gradient = -Eigen::Vector3d::UnitZ();
      return gap;
    }
    return original;
  };

  const double half_height = 0.5 * (config.vehicle_top_height + config.vehicle_bottom_height);
  const Eigen::Vector3d body_center = vehicle_position + Eigen::Vector3d(
      0.0, 0.0, 0.5 * (config.vehicle_top_height - config.vehicle_bottom_height));
  if (measured_box)
  {
    const double box_clearance = cylinderBoxClearance(
        body_center, predicted_position, state.size, config.vehicle_radius,
        half_height, config.margin, clearance_gradient);
    if (clearance != nullptr)
      *clearance = underpass_clearance(box_clearance);
    else
      underpass_clearance(box_clearance);
    return true;
  }

  const Eigen::Vector3d relative = body_center - predicted_position;
  const double radial_distance = relative.head<2>().norm();
  const double radial_gap = radial_distance - config.vehicle_radius;
  const double vertical_gap =
      std::abs(relative.z()) - half_height;
  const double outside_radial = std::max(0.0, radial_gap);
  const double outside_vertical = std::max(0.0, vertical_gap);
  const double outside_distance = std::hypot(outside_radial, outside_vertical);
  const double cylinder_distance =
      outside_distance + std::min(0.0, std::max(radial_gap, vertical_gap));

  if (clearance_gradient != nullptr)
  {
    Eigen::Vector3d radial_normal = Eigen::Vector3d::UnitX();
    if (radial_distance > 1.0e-12)
      radial_normal = Eigen::Vector3d(relative.x() / radial_distance,
                                     relative.y() / radial_distance, 0.0);
    // At equal height the two caps are equally close. Select the downward
    // subgradient so a centered collision has a finite escape direction.
    const Eigen::Vector3d vertical_normal(
        0.0, 0.0, relative.z() > 0.0 ? 1.0 : -1.0);
    if (outside_distance > 0.0)
      *clearance_gradient =
          (outside_radial * radial_normal +
           outside_vertical * vertical_normal) / outside_distance;
    else if (vertical_gap >= radial_gap)
      *clearance_gradient = vertical_normal;
    else
      *clearance_gradient = radial_normal;
  }
  const double result = underpass_clearance(cylinder_distance - config.ball_radius - config.margin);
  if (clearance != nullptr) *clearance = result;
  return true;
}

} // namespace fuel_fapp
