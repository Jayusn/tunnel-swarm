#include <ldop/dynamic_object_predictor.h>
#include <ldop/utils.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Eigenvalues>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

namespace {

// 预测尾部对规划/RViz 的最小影响权重；前半段保持 1.0，后半段线性衰减到该值。
constexpr double kMinInfluenceWeight = 0.3;
// 时间比较用的小量，避免浮点精度导致最后一个采样点被跳过。
constexpr double kTimeEpsilon = 1e-9;
// 匹配目标和漏检目标的预测轨迹透明度。
constexpr double kMatchedAlpha = 0.45;
constexpr double kCoastingAlpha = 0.18;

// 影响权重：前半段保持 1.0，后半段线性衰减到 kMinInfluenceWeight。
double computeInfluenceWeight(const double time_from_start,
                              const double prediction_horizon) {
  const double half_horizon = 0.5 * prediction_horizon;
  if (time_from_start <= half_horizon + kTimeEpsilon) {
    return 1.0;
  }
  const double tail_duration = std::max(prediction_horizon - half_horizon, kTimeEpsilon);
  const double ratio = std::clamp((time_from_start - half_horizon) / tail_duration, 0.0, 1.0);
  return 1.0 + (kMinInfluenceWeight - 1.0) * ratio;
}

// Eigen 向量转 std::vector，用于填充 ROS 消息字段。
std::vector<double> eigenVectorToStdVector(const Eigen::VectorXd& vector) {
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(vector.size()));
  for (int index = 0; index < vector.size(); ++index) {
    values.push_back(vector(index));
  }
  return values;
}

// Eigen 矩阵转 row-major std::vector，用于填充 ROS 消息字段。
std::vector<double> eigenMatrixToRowMajorStdVector(const Eigen::MatrixXd& matrix) {
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(matrix.rows() * matrix.cols()));
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      values.push_back(matrix(row, col));
    }
  }
  return values;
}

// 3x3 旋转矩阵转四元数，用于 RViz 椭球朝向。
// 协方差特征分解的特征向量矩阵可能因数值精度导致非严格正交，显式归一化避免 RViz 警告。
geometry_msgs::Quaternion eigenMatrixToQuaternion(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  geometry_msgs::Quaternion ros_quaternion;
  ros_quaternion.x = quaternion.x();
  ros_quaternion.y = quaternion.y();
  ros_quaternion.z = quaternion.z();
  ros_quaternion.w = quaternion.w();
  return ros_quaternion;
}

// 从 row-major 协方差矩阵中提取前三维位置协方差块。
Eigen::Matrix3d positionCovarianceFromRowMajor(
    const std::vector<double>& covariance_values,
    const std::size_t state_size) {
  assert(covariance_values.size() == state_size * state_size);
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      covariance(row, col) =
          covariance_values[static_cast<std::size_t>(row) * state_size +
                            static_cast<std::size_t>(col)];
    }
  }
  return 0.5 * (covariance + covariance.transpose());
}

}  // namespace

DynamicObjectPredictorConfig buildPredictorConfig(
    const DynamicObjectPredictorParams& params) {
  const DynamicObjectPredictorParams defaults;
  DynamicObjectPredictorConfig config;
  config.prediction_horizon =
      params.prediction_horizon > 0.0 ? params.prediction_horizon
                                      : defaults.prediction_horizon;
  config.filter_config = buildMultiModelKalmanFilterConfig(params.filter);
  return config;
}

DynamicObjectPredictor::DynamicObjectPredictor(ros::NodeHandle& pnh,
                                               const bool verbose)
    : pnh_(pnh),
      verbose_(verbose) {
  loadParameters();
}

DynamicObjectPredictor::DynamicObjectPredictor(
    const DynamicObjectPredictorConfig& config,
    const bool verbose)
    : config_(config),
      verbose_(verbose) {}

void DynamicObjectPredictor::loadParameters() {
  const DynamicObjectPredictorParams defaults;
  pnh_.param("prediction_horizon", params_.prediction_horizon, defaults.prediction_horizon);
  pnh_.param("tracking_default_dt", params_.filter.default_dt, defaults.filter.default_dt);
  pnh_.param("kalman_filter/adaptive_window_size", params_.filter.adaptive_window_size, defaults.filter.adaptive_window_size);
  pnh_.param("kalman_filter/adaptive_alpha", params_.filter.adaptive_alpha, defaults.filter.adaptive_alpha);
  pnh_.param("kalman_filter/adaptive_r_alpha", params_.filter.adaptive_r_alpha, defaults.filter.adaptive_r_alpha);
  pnh_.param("kalman_filter/adaptive_min_noise_ratio", params_.filter.adaptive_min_noise_ratio, defaults.filter.adaptive_min_noise_ratio);
  pnh_.param("kalman_filter/enable_cov_limit", params_.filter.enable_cov_limit, defaults.filter.enable_cov_limit);
  pnh_.param("kalman_filter/max_pos_cov", params_.filter.max_pos_cov, defaults.filter.max_pos_cov);
  pnh_.param("kalman_filter/max_vel_cov", params_.filter.max_vel_cov, defaults.filter.max_vel_cov);
  pnh_.param("kalman_filter/max_acc_cov", params_.filter.max_acc_cov, defaults.filter.max_acc_cov);
  pnh_.param("kalman_filter/ca_model/human/jerk_sigma", params_.filter.ca_human_jerk_sigma, defaults.filter.ca_human_jerk_sigma);
  pnh_.param("kalman_filter/ca_model/human/init_cov", params_.filter.ca_human_init_cov, defaults.filter.ca_human_init_cov);
  pnh_.param("kalman_filter/ca_model/human/meas_noise", params_.filter.ca_human_meas_noise, defaults.filter.ca_human_meas_noise);
  pnh_.param("kalman_filter/ca_model/human/z_process_noise", params_.filter.ca_human_z_process_noise, defaults.filter.ca_human_z_process_noise);
  pnh_.param("kalman_filter/ca_model/uav/jerk_sigma", params_.filter.ca_uav_jerk_sigma, defaults.filter.ca_uav_jerk_sigma);
  pnh_.param("kalman_filter/ca_model/uav/init_cov", params_.filter.ca_uav_init_cov, defaults.filter.ca_uav_init_cov);
  pnh_.param("kalman_filter/ca_model/uav/meas_noise", params_.filter.ca_uav_meas_noise, defaults.filter.ca_uav_meas_noise);
  pnh_.param("kalman_filter/cv_model/acc_sigma", params_.filter.cv_acc_sigma, defaults.filter.cv_acc_sigma);
  pnh_.param("kalman_filter/cv_model/init_cov", params_.filter.cv_init_cov, defaults.filter.cv_init_cov);
  pnh_.param("kalman_filter/cv_model/meas_noise", params_.filter.cv_meas_noise, defaults.filter.cv_meas_noise);
  pnh_.param("kalman_filter/ctra_model/init_cov", params_.filter.ctra_init_cov, defaults.filter.ctra_init_cov);
  pnh_.param("kalman_filter/ctra_model/process_noise", params_.filter.ctra_process_noise, defaults.filter.ctra_process_noise);
  pnh_.param("kalman_filter/ctra_model/meas_noise", params_.filter.ctra_meas_noise, defaults.filter.ctra_meas_noise);
  config_ = buildPredictorConfig(params_);
}

DynamicObjectPredictorFrameResult DynamicObjectPredictor::predict(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs) const {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectPredictorFrameResult result;
  result.predictions_msg.header = header;
  result.predictions_msg.predictions.reserve(inputs.size());

  for (const TrackPredictionInput& input : inputs) {
    // 为每个目标创建独立的运动模型实例，避免状态互相干扰。
    std::shared_ptr<MotionModel> model =
        createMotionModel(input.motion_model_type, config_.filter_config);
    model->setDt(config_.filter_config.default_dt);

    Eigen::VectorXd state = input.model_state;
    Eigen::MatrixXd covariance = input.model_covariance;
    assert(state.size() == model->stateDim());
    assert(covariance.rows() == state.size());
    assert(covariance.cols() == state.size());

    ldop::DynamicObjectPrediction prediction;
    prediction.id = input.id;
    prediction.size = input.bbox.size;
    prediction.object_class = toRosObjectClass(input.object_class);
    prediction.motion_model_type = toRosMotionModelType(input.motion_model_type);
    prediction.matched_in_current_frame = input.matched_in_current_frame;

    ldop::DynamicObjectPredictionBranch branch;
    branch.behavior_type = ldop::DynamicObjectPredictionBranch::BEHAVIOR_BASELINE;
    branch.behavior_value = 0.0;
    branch.probability = 1.0;

    // 按采样间隔逐步外推状态和协方差。
    for (double elapsed = config_.filter_config.default_dt;
         elapsed <= config_.prediction_horizon + kTimeEpsilon;
         elapsed += config_.filter_config.default_dt) {
      const Eigen::VectorXd previous_state = state;
      const Eigen::MatrixXd transition = model->getTransitionF(previous_state);
      state = model->stateTransition(previous_state);
      model->normalizeYaw(state);
      // 未来传播使用运动模型基础 Q，不读取 tracker filter 内部自适应噪声状态。
      const Eigen::MatrixXd process_noise = model->getProcessNoiseQ();
      covariance = transition * covariance * transition.transpose() + process_noise;
      covariance = 0.5 * (covariance + covariance.transpose());

      ldop::DynamicObjectPredictionPoint point;
      point.time_from_start = ros::Duration(elapsed);
      point.model_state = eigenVectorToStdVector(state);
      point.model_covariance = eigenMatrixToRowMajorStdVector(covariance);
      point.influence_weight = computeInfluenceWeight(elapsed, config_.prediction_horizon);
      branch.points.push_back(std::move(point));
    }

    prediction.branches.push_back(std::move(branch));
    result.predictions_msg.predictions.push_back(std::move(prediction));
  }

  result.timing.predict_dynamic_objects_ms =
      elapsedMs(total_start, std::chrono::steady_clock::now());

  // 构建预测可视化 marker。
  const auto marker_start = std::chrono::steady_clock::now();
  result.prediction_markers_msg = buildPredictionMarkers(header, result.predictions_msg);
  result.timing.build_prediction_markers_ms =
      elapsedMs(marker_start, std::chrono::steady_clock::now());

  result.timing.predict_total_ms = elapsedMs(total_start, std::chrono::steady_clock::now());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Dynamic object predictor timing"
                                       << ": total="
                                       << formatFloatMs(result.timing.predict_total_ms)
                                       << "ms, predictDynamicObjects="
                                       << formatFloatMs(result.timing.predict_dynamic_objects_ms)
                                       << "ms, buildPredictionMarkers="
                                       << formatFloatMs(result.timing.build_prediction_markers_ms)
                                       << "ms, inputs=" << inputs.size()
                                       << ", predictions="
                                       << result.predictions_msg.predictions.size());
  }

  return result;
}

visualization_msgs::MarkerArray DynamicObjectPredictor::buildPredictionMarkers(
    const std_msgs::Header& header,
    const ldop::DynamicObjectPredictionArray& predictions) const {
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(makeDeleteAllMarker(header, "dynamic_predictions"));
  int marker_id = 1;

  for (const auto& prediction : predictions.predictions) {
    if (prediction.branches.empty() || prediction.branches.front().points.empty()) {
      continue;
    }

    // 匹配目标使用更高透明度，漏检目标使用更低透明度。
    const double target_alpha =
        prediction.matched_in_current_frame ? kMatchedAlpha : kCoastingAlpha;

    // 轨迹线：连接所有采样点中心。
    visualization_msgs::Marker line;
    line.header = header;
    line.ns = "dynamic_prediction_line";
    line.id = marker_id++;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.04;
    line.color.r = 0.1F;
    line.color.g = 0.7F;
    line.color.b = 1.0F;
    line.color.a = static_cast<float>(target_alpha);

    const auto& branch = prediction.branches.front();
    for (const auto& point : branch.points) {
      assert(point.model_state.size() >= 3U);
      line.points.push_back(makePoint(point.model_state[0], point.model_state[1], point.model_state[2]));
    }
    markers.markers.push_back(std::move(line));

    // 不确定性椭球：每个采样点用椭球表达前三维位置协方差。
    for (const auto& point : branch.points) {
      assert(point.model_state.size() >= 3U);
      const Eigen::Matrix3d position_covariance =
          positionCovarianceFromRowMajor(point.model_covariance, point.model_state.size());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(position_covariance);
      const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
      const Eigen::Matrix3d eigenvectors = solver.eigenvectors();

      visualization_msgs::Marker ellipsoid;
      ellipsoid.header = header;
      ellipsoid.ns = "dynamic_prediction_uncertainty";
      ellipsoid.id = marker_id++;
      ellipsoid.type = visualization_msgs::Marker::SPHERE;
      ellipsoid.action = visualization_msgs::Marker::ADD;
      ellipsoid.pose.position = makePoint(point.model_state[0], point.model_state[1], point.model_state[2]);
      ellipsoid.pose.orientation = eigenMatrixToQuaternion(eigenvectors);
      // 椭球轴长按 2 * sqrt(lambda) 设置为 RViz SPHERE 直径，显示 1σ 范围。
      ellipsoid.scale.x = 2.0 * std::sqrt(eigenvalues.x());
      ellipsoid.scale.y = 2.0 * std::sqrt(eigenvalues.y());
      ellipsoid.scale.z = 2.0 * std::sqrt(eigenvalues.z());
      ellipsoid.color.r = 0.1F;
      ellipsoid.color.g = 0.7F;
      ellipsoid.color.b = 1.0F;
      ellipsoid.color.a = static_cast<float>(target_alpha * point.influence_weight);
      markers.markers.push_back(std::move(ellipsoid));
    }
  }

  return markers;
}

}  // namespace ldopcore
