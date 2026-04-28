#if !defined(EIGEN_DONT_PARALLELIZE)
#if defined(__has_include)
#if !__has_include(<omp.h>)
// 诊断环境缺少 omp.h 时禁用 Eigen 并行分支，避免编辑期级联误报。
#define EIGEN_DONT_PARALLELIZE
#endif
#endif
#endif

#if defined(__clang__) && !defined(EIGEN_DONT_VECTORIZE)
// clangd 10 + GCC11 头组合下可能出现 _mm_prefetch 冲突，诊断路径禁用 Eigen 向量化。
#define EIGEN_DONT_VECTORIZE
#endif

#include <ldop/dynamic_object_tracker.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <boost/math/distributions/chi_squared.hpp>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

namespace {

struct AssociationCandidate {
  // 记录一次 detection-track 候选匹配；raw_cost 保留 gate 内的原始马氏距离平方。
  std::size_t detection_index{0U};
  std::size_t track_index{0U};
  double raw_cost{0.0};
};

// 行对应 detection，列对应已有 track；invalid_cost 表示该组合被 gate 拒绝。
using CostMatrix = std::vector<std::vector<double>>;

visualization_msgs::Marker makeTrackDeleteAllMarker(const std_msgs::Header& header) {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "dynamic_tracks";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

std_msgs::ColorRGBA makeTrackColor(const bool coasting) {
  std_msgs::ColorRGBA color;
  color.r = 0.10F;
  color.g = 0.80F;
  color.b = 0.95F;
  color.a = coasting ? 0.35F : 1.0F;
  return color;
}

double chiSquare3DThresholdFromConfidence(const double confidence) {
  constexpr double kDegreesOfFreedom = 3.0;
  constexpr double kDefaultConfidence = 0.99;
  constexpr double kMinConfidence = 1e-6;
  constexpr double kMaxConfidence = 1.0 - 1e-6;

  // 使用卡方分布分位数连续计算 gate，避免离散查表导致门限突跳。
  const double sanitized_confidence =
      std::isfinite(confidence)
          ? std::clamp(confidence, kMinConfidence, kMaxConfidence)
          : kDefaultConfidence;
  const boost::math::chi_squared chi_square_distribution(kDegreesOfFreedom);
  return boost::math::quantile(chi_square_distribution, sanitized_confidence);
}

DynamicObjectTrackerConfig sanitizeTrackerParams(const DynamicObjectTrackerParams& params) {
  DynamicObjectTrackerConfig config;
  // 计数类参数统一裁剪到非负/至少 1，避免运行阶段反复做边界兜底。
  config.max_history_size = static_cast<std::size_t>(std::max(params.history_size, 0));
  config.max_coast_frames = static_cast<std::size_t>(std::max(params.max_missed_frames, 0));
  config.min_hits_to_publish = static_cast<std::size_t>(std::max(params.min_hits_to_publish, 1));
  // 外部配置使用更直观的置信度，内部关联直接使用卡方分布的距离平方阈值。
  config.association_gate_threshold = chiSquare3DThresholdFromConfidence(params.association_gate_confidence);
  config.coasting_gate_relax_factor = std::isfinite(params.coasting_gate_relax_factor) &&
      params.coasting_gate_relax_factor > 0.0 ? std::max(params.coasting_gate_relax_factor, 1.0) : 1.0;
  config.box_size_smoothing_alpha = std::clamp(params.box_size_smoothing_alpha, 0.0, 1.0);
  // 这两个阈值优先用于“马氏距离 gate 失败后的回退关联”；如果最终仍未匹配，
  // 后面的建轨抑制也继续复用同一组阈值，避免旋转中心抖动时裂出新 ID。
  config.spawn_suppression_distance = std::isfinite(params.spawn_suppression_distance) && 
      params.spawn_suppression_distance > 0.0 ? params.spawn_suppression_distance : 0.0;
  config.spawn_suppression_iou_threshold = std::isfinite(params.spawn_suppression_iou_threshold) &&
      params.spawn_suppression_iou_threshold > 0.0 ? std::clamp(params.spawn_suppression_iou_threshold, 0.0, 1.0) : 0.0;
  config.max_dt = std::isfinite(params.max_dt) && params.max_dt > 0.0 ? params.max_dt : 0.5;
  // tracker 负责把 ROS 参数语义转换成滤波器配置，滤波器不再自行读取 ROS 参数。
  config.filter_config.dt = std::isfinite(params.default_dt) && params.default_dt > 0.0 ? params.default_dt : 0.1;
  config.filter_config.innovation_buffer = static_cast<std::size_t>(std::max(params.adaptive_window_size, 0));
  config.filter_config.adaptive_q_alpha = std::clamp(params.adaptive_alpha, 0.0, 1.0);
  config.filter_config.adaptive_r_alpha = std::clamp(params.adaptive_r_alpha, 0.0, 1.0);
  config.filter_config.min_noise_ratio = std::isfinite(params.adaptive_min_noise_ratio) &&
      params.adaptive_min_noise_ratio > 0.0 ? params.adaptive_min_noise_ratio : 0.1;
  config.filter_config.max_pos_covariance = std::isfinite(params.max_pos_cov) && 
      params.max_pos_cov > 0.0 ? params.max_pos_cov : 1e3;
  config.filter_config.max_vel_covariance = std::isfinite(params.max_vel_cov) && 
      params.max_vel_cov > 0.0 ? params.max_vel_cov : 1e3;
  config.filter_config.max_acc_covariance = std::isfinite(params.max_acc_cov) && 
      params.max_acc_cov > 0.0 ? params.max_acc_cov : 1e3;
  config.filter_config.process_noise = std::isfinite(params.ca_jerk_sigma) && 
      params.ca_jerk_sigma > 0.0 ? params.ca_jerk_sigma * params.ca_jerk_sigma : 1.0;
  // 对齐 LDOT：这里直接把参数当作位置观测噪声方差，而不是标准差后再平方。
  config.filter_config.measurement_noise = std::isfinite(params.ca_position_noise) && 
      params.ca_position_noise > 0.0 ? params.ca_position_noise : 0.5;
  config.filter_config.initial_pos_covariance = std::isfinite(params.ca_initial_position_cov) &&
      params.ca_initial_position_cov > 0.0 ? params.ca_initial_position_cov : 1.0;
  config.filter_config.initial_vel_covariance = std::isfinite(params.ca_initial_velocity_cov) &&
      params.ca_initial_velocity_cov > 0.0 ? params.ca_initial_velocity_cov : 1.0;
  config.filter_config.initial_acc_covariance = std::isfinite(params.ca_initial_acceleration_cov) &&
      params.ca_initial_acceleration_cov > 0.0 ? params.ca_initial_acceleration_cov : 1.0;
  return config;
}

Eigen::Vector3d pointToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::Point eigenToPoint(const Eigen::Vector3d& point) {
  geometry_msgs::Point ros_point;
  ros_point.x = point.x();
  ros_point.y = point.y();
  ros_point.z = point.z();
  return ros_point;
}

geometry_msgs::Vector3 eigenToVector(const Eigen::Vector3d& vector) {
  geometry_msgs::Vector3 ros_vector;
  ros_vector.x = vector.x();
  ros_vector.y = vector.y();
  ros_vector.z = vector.z();
  return ros_vector;
}

double computeDeltaSeconds(const ros::Time& current_stamp,
                           const ros::Time& previous_stamp,
                           const double fallback_dt,
                           const double max_dt) {
  if (current_stamp.isZero() || previous_stamp.isZero()) {
    return fallback_dt;
  }

  const double dt = (current_stamp - previous_stamp).toSec();
  const double positive_dt =
      std::isfinite(dt) && dt > 0.0 ? dt : fallback_dt;
  const double sanitized_max_dt =
      std::isfinite(max_dt) && max_dt > 0.0 ? max_dt : positive_dt;
  return std::min(positive_dt, sanitized_max_dt);
}

double computeMahalanobisCost(const Eigen::Vector3d& detection_position,
                              const TrackState& track) {
  // 这里的 innovation 是 detection 位置观测与预测轨迹中心的偏差；当前观测来自点簇质心。
  const Eigen::Vector3d innovation = detection_position - track.filter->position();
  Eigen::Matrix3d position_covariance =
      track.filter->covariance().block<3, 3>(0, 0);
  position_covariance = 0.5 * (position_covariance + position_covariance.transpose());

  // 关联代价使用预测位置协方差；轻微正则化避免数值退化时误拒绝所有候选。
  position_covariance += 1e-6 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d inverse_covariance = position_covariance.inverse();
  if (!inverse_covariance.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const double cost = innovation.dot(inverse_covariance * innovation);
  return std::isfinite(cost) && cost >= 0.0 ? cost : std::numeric_limits<double>::infinity();
}

double computeHorizontalIntersectionOverUnion(const BoundingBox3D& lhs,
                                              const BoundingBox3D& rhs) {
  // 只计算水平 AABB IoU；当前检测框 yaw 固定为 0，先不引入有向框几何。
  const double lhs_size_x = std::max(lhs.size.x, 0.0);
  const double lhs_size_y = std::max(lhs.size.y, 0.0);
  const double rhs_size_x = std::max(rhs.size.x, 0.0);
  const double rhs_size_y = std::max(rhs.size.y, 0.0);
  const double lhs_area = lhs_size_x * lhs_size_y;
  const double rhs_area = rhs_size_x * rhs_size_y;
  if (lhs_area <= 0.0 || rhs_area <= 0.0) {
    return 0.0;
  }

  const double lhs_min_x = lhs.center.x - lhs_size_x * 0.5;
  const double lhs_max_x = lhs.center.x + lhs_size_x * 0.5;
  const double lhs_min_y = lhs.center.y - lhs_size_y * 0.5;
  const double lhs_max_y = lhs.center.y + lhs_size_y * 0.5;
  const double rhs_min_x = rhs.center.x - rhs_size_x * 0.5;
  const double rhs_max_x = rhs.center.x + rhs_size_x * 0.5;
  const double rhs_min_y = rhs.center.y - rhs_size_y * 0.5;
  const double rhs_max_y = rhs.center.y + rhs_size_y * 0.5;

  const double intersection_x = std::max(0.0, std::min(lhs_max_x, rhs_max_x) -
                                                  std::max(lhs_min_x, rhs_min_x));
  const double intersection_y = std::max(0.0, std::min(lhs_max_y, rhs_max_y) -
                                                  std::max(lhs_min_y, rhs_min_y));
  const double intersection_area = intersection_x * intersection_y;
  const double union_area = lhs_area + rhs_area - intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

double computeCenterDistance3D(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

double computeFallbackAssociationCost(const DynamicObjectDetection& detection,
                                      const TrackState& track,
                                      const DynamicObjectTrackerConfig& config,
                                      const double rejected_gate_cost,
                                      const double invalid_cost) {
  const bool use_distance_fallback = config.spawn_suppression_distance > 0.0;
  const bool use_iou_fallback = config.spawn_suppression_iou_threshold > 0.0;
  if (!use_distance_fallback && !use_iou_fallback) {
    return invalid_cost;
  }

  double distance_penalty = 0.0;
  if (use_distance_fallback) {
    const double center_distance = computeCenterDistance3D(detection.bbox.center, track.bbox.center);
    if (!(std::isfinite(center_distance) &&
          center_distance <= config.spawn_suppression_distance)) {
      return invalid_cost;
    }
    // 回退关联先过距离门槛；只有这一层通过后，后面的 IoU 判据才有意义。
    // 距离越近，回退代价越低；但整体仍故意高于任何合法马氏距离匹配。
    distance_penalty =
        center_distance / std::max(config.spawn_suppression_distance, 1e-6);
  }

  double iou_penalty = 0.0;
  if (use_iou_fallback) {
    const double iou = computeHorizontalIntersectionOverUnion(detection.bbox, track.bbox);
    if (!(std::isfinite(iou) &&
          iou >= config.spawn_suppression_iou_threshold)) {
      return invalid_cost;
    }
    // IoU 越大，回退代价越低；同样只作为马氏距离失败后的次优候选。
    iou_penalty = 1.0 - std::clamp(iou, 0.0, 1.0);
  }

  const double fallback_penalty =
      use_distance_fallback && use_iou_fallback
          ? 0.5 * (distance_penalty + iou_penalty)
          : (use_distance_fallback ? distance_penalty : iou_penalty);
  return rejected_gate_cost + 1.0 + fallback_penalty;
}

bool shouldSuppressTrackSpawn(const DynamicObjectDetection& detection,
                              const std::vector<TrackState>& tracks,
                              const DynamicObjectTrackerConfig& config) {
  if (config.spawn_suppression_distance <= 0.0 &&
      config.spawn_suppression_iou_threshold <= 0.0) {
    return false;
  }

  for (const auto& track : tracks) {
    // 未匹配 detection 若仍贴近已有轨迹，先作为 tentative detection 挂起，避免中心跳动时立刻裂出新 ID。
    if (config.spawn_suppression_iou_threshold > 0.0 &&
        computeHorizontalIntersectionOverUnion(detection.bbox, track.bbox) >=
            config.spawn_suppression_iou_threshold) {
      return true;
    }
    if (config.spawn_suppression_distance > 0.0 &&
        computeCenterDistance3D(detection.bbox.center, track.bbox.center) <=
            config.spawn_suppression_distance) {
      return true;
    }
  }

  return false;
}

void updateBBoxCenterFromFilter(TrackState& track) {
  track.bbox.center = eigenToPoint(track.filter->position());
}

void appendHistorySample(TrackState& track, const ros::Time& stamp, const bool matched,
                         const std::size_t max_history_size) {
  TrackHistorySample sample;
  sample.stamp = stamp;
  sample.position = track.bbox.center;
  sample.velocity = eigenToVector(track.filter->velocity());
  sample.matched = matched;
  track.history.push_back(std::move(sample));

  if (max_history_size == 0U) {
    track.history.clear();
    return;
  }

  if (track.history.size() > max_history_size) {
    track.history.erase(track.history.begin(),
                        track.history.begin() +
                            static_cast<std::ptrdiff_t>(track.history.size() - max_history_size));
  }
}

CostMatrix buildCostMatrix(const std::vector<DynamicObjectDetection>& detections,
                           const std::vector<TrackState>& tracks,
                           const DynamicObjectTrackerConfig& config,
                           const double gate_threshold,
                           const double coasting_gate_relax_factor,
                           const double invalid_cost) {
  CostMatrix matrix(detections.size(), std::vector<double>(tracks.size(), invalid_cost));

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    const Eigen::Vector3d detection_position = pointToEigen(detections[detection_index].bbox.center);
    for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
      // cost 仍保留原始马氏距离平方；是否可匹配由对应轨迹的 gate 决定。
      const double cost = computeMahalanobisCost(detection_position, tracks[track_index]);
      const double relaxed_gate =
          tracks[track_index].missed_frames > 0U
              ? gate_threshold * coasting_gate_relax_factor
              : gate_threshold;
      if (std::isfinite(cost) && cost <= relaxed_gate) {
        matrix[detection_index][track_index] = cost;
        continue;
      }

      // 主关联仍优先使用马氏距离；只有它被 gate 拒绝时，才用距离/IoU 阈值给这对
      // detection-track 一个“次优回退候选”，专门补原地旋转或小半径旋转时的中心抖动。
      const double fallback_cost = computeFallbackAssociationCost(
          detections[detection_index], tracks[track_index], config, relaxed_gate, invalid_cost);
      if (fallback_cost < invalid_cost) {
        matrix[detection_index][track_index] = fallback_cost;
      }
    }
  }

  return matrix;
}

std::vector<AssociationCandidate> selectAssociations(const CostMatrix& original,
                                                     const double invalid_cost) {
  if (original.empty() || original.front().empty()) {
    return {};
  }

  const std::size_t detection_count = original.size();
  const std::size_t track_count = original.front().size();
  const std::size_t dimension = detection_count + track_count;
  const double unmatched_penalty = invalid_cost * 0.5;

  // 扩展成方阵后用 Hungarian 算法求全局最小匹配；dummy 行/列显式表示未匹配。
  CostMatrix cost(dimension, std::vector<double>(dimension, 0.0));
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t column = 0; column < dimension; ++column) {
      if (row < detection_count && column < track_count) {
        cost[row][column] =
            original[row][column] < invalid_cost ? original[row][column] : invalid_cost;
      } else if ((row < detection_count && column >= track_count) ||
                 (row >= detection_count && column < track_count)) {
        // dummy 行/列表示漏检或未匹配 detection，代价高于任何合法 gate 内匹配。
        cost[row][column] = unmatched_penalty;
      }
    }
  }

  std::vector<double> row_potential(dimension + 1U, 0.0);
  std::vector<double> column_potential(dimension + 1U, 0.0);
  std::vector<std::size_t> assigned_row_for_column(dimension + 1U, 0U);
  std::vector<std::size_t> previous_column(dimension + 1U, 0U);

  for (std::size_t row = 1U; row <= dimension; ++row) {
    assigned_row_for_column[0] = row;
    std::size_t current_column = 0U;
    std::vector<double> min_value(dimension + 1U, std::numeric_limits<double>::infinity());
    std::vector<bool> used_column(dimension + 1U, false);

    do {
      // Hungarian 势能更新：逐步扩展交替树，直到找到可增广的未占用列。
      used_column[current_column] = true;
      const std::size_t current_row = assigned_row_for_column[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0U;

      for (std::size_t column = 1U; column <= dimension; ++column) {
        if (used_column[column]) {
          continue;
        }
        const double candidate =
            cost[current_row - 1U][column - 1U] - row_potential[current_row] -
            column_potential[column];
        if (candidate < min_value[column]) {
          min_value[column] = candidate;
          previous_column[column] = current_column;
        }
        if (min_value[column] < delta) {
          delta = min_value[column];
          next_column = column;
        }
      }

      for (std::size_t column = 0U; column <= dimension; ++column) {
        if (used_column[column]) {
          row_potential[assigned_row_for_column[column]] += delta;
          column_potential[column] -= delta;
        } else {
          min_value[column] -= delta;
        }
      }
      current_column = next_column;
    } while (assigned_row_for_column[current_column] != 0U);

    do {
      const std::size_t next_column = previous_column[current_column];
      assigned_row_for_column[current_column] = assigned_row_for_column[next_column];
      current_column = next_column;
    } while (current_column != 0U);
  }

  std::vector<std::size_t> assigned_column_for_row(dimension, dimension);
  for (std::size_t column = 1U; column <= dimension; ++column) {
    if (assigned_row_for_column[column] != 0U) {
      assigned_column_for_row[assigned_row_for_column[column] - 1U] = column - 1U;
    }
  }

  std::vector<AssociationCandidate> matches;
  matches.reserve(std::min(detection_count, track_count));
  for (std::size_t detection_index = 0; detection_index < detection_count; ++detection_index) {
    const std::size_t track_index = assigned_column_for_row[detection_index];
    if (track_index >= track_count) {
      continue;
    }
    const double raw_cost = original[detection_index][track_index];
    // 只把原始 gate 内的真实 detection-track 组合返回；dummy 匹配在这里被过滤掉。
    if (raw_cost < invalid_cost) {
      matches.push_back({detection_index, track_index, raw_cost});
    }
  }

  return matches;
}

}  // namespace

TrackState::TrackState(const std::uint32_t track_id,
                       const MotionModelType track_model_type,
                       const MultiModelKalmanFilterConfig& filter_config)
    : id(track_id),
      model_type(track_model_type),
      filter(createMultiModelKalmanFilter(track_model_type, filter_config)) {
  if (filter == nullptr) {
    throw std::runtime_error("failed to construct track filter");
  }
}

TrackState::~TrackState() = default;

TrackState::TrackState(TrackState&&) noexcept = default;

TrackState& TrackState::operator=(TrackState&&) noexcept = default;

DynamicObjectTracker::DynamicObjectTracker(
    ros::NodeHandle& pnh,
    const bool verbose)
    : pnh_(pnh),
      config_(),
      params_(),
      tracks_(),
      next_track_id_(0U),
      verbose_(verbose) {
  loadParameters();

  if (verbose_) {
    ROS_INFO_STREAM("Dynamic object tracker is ready. gate threshold: "
                    << config_.association_gate_threshold
                    << ", history size: " << config_.max_history_size
                    << ", max missed frames: " << config_.max_coast_frames
                    << ", min hits to publish: " << config_.min_hits_to_publish);
  }
}

void DynamicObjectTracker::loadParameters() {
  pnh_.param("tracking_history_size", params_.history_size, 100);
  pnh_.param("tracking_max_missed_frames", params_.max_missed_frames, 5);
  pnh_.param("tracking_min_hits_to_publish", params_.min_hits_to_publish, 2);
  pnh_.param("tracking_association_gate_confidence", params_.association_gate_confidence, 0.99);
  pnh_.param("tracking_coasting_gate_relax_factor", params_.coasting_gate_relax_factor, 1.5);
  pnh_.param("tracking_box_size_smoothing_alpha", params_.box_size_smoothing_alpha, 0.5);
  pnh_.param("tracking_spawn_suppression_distance", params_.spawn_suppression_distance, 0.8);
  pnh_.param("tracking_spawn_suppression_iou_threshold", params_.spawn_suppression_iou_threshold, 0.1);
  pnh_.param("tracking_default_dt", params_.default_dt, 0.1);
  pnh_.param("tracking_max_dt", params_.max_dt, 0.5);
  pnh_.param("tracking_adaptive_window_size", params_.adaptive_window_size, 10);
  pnh_.param("tracking_adaptive_alpha", params_.adaptive_alpha, 0.7);
  pnh_.param("tracking_adaptive_r_alpha", params_.adaptive_r_alpha, 0.7);
  pnh_.param("tracking_adaptive_min_noise_ratio", params_.adaptive_min_noise_ratio, 0.2);
  pnh_.param("tracking_max_pos_cov", params_.max_pos_cov, 1.5);
  pnh_.param("tracking_max_vel_cov", params_.max_vel_cov, 9.0);
  pnh_.param("tracking_max_acc_cov", params_.max_acc_cov, 25.0);
  pnh_.param("tracking_ca_jerk_sigma", params_.ca_jerk_sigma, 3.0);
  pnh_.param("tracking_ca_position_noise", params_.ca_position_noise, 0.1);
  pnh_.param("tracking_ca_initial_position_cov", params_.ca_initial_position_cov, 0.1);
  pnh_.param("tracking_ca_initial_velocity_cov", params_.ca_initial_velocity_cov, 1.0);
  pnh_.param("tracking_ca_initial_acceleration_cov", params_.ca_initial_acceleration_cov, 10.0);

  config_ = sanitizeTrackerParams(params_);
}

DynamicObjectTrackerFrameResult DynamicObjectTracker::processDynamicTracks(
    const std_msgs::Header& header,
    const std::vector<DynamicObjectDetection>& detections) {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectTrackerFrameResult result;

  // 先把所有已有轨迹预测到当前帧时间，再用当前 detection 做同一时刻的数据关联。
  predictTracks(header.stamp);

  const auto association_start = std::chrono::steady_clock::now();
  std::vector<bool> matched_detections(detections.size(), false);
  std::vector<bool> matched_tracks(tracks_.size(), false);
  const CostMatrix original_cost =
      buildCostMatrix(detections,
                      tracks_,
                      config_,
                      config_.association_gate_threshold,
                      config_.coasting_gate_relax_factor,
                      config_.invalid_cost);
  const auto matches = selectAssociations(original_cost, config_.invalid_cost);
  result.timing.association_ms = elapsedMs(association_start, std::chrono::steady_clock::now());

  const auto update_tracks_start = std::chrono::steady_clock::now();
  std::size_t suppressed_track_spawns = 0U;
  for (const auto& match : matches) {
    // 匹配成功的轨迹用 detection 更新滤波器，同时刷新 bbox 尺寸和历史样本。
    matched_detections[match.detection_index] = true;
    matched_tracks[match.track_index] = true;
    updateMatchedTrack(tracks_[match.track_index], detections[match.detection_index], header.stamp);
  }

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    if (!matched_detections[detection_index]) {
      // 未匹配 detection 先经过建轨抑制，防止同一目标中心抖动时立即产生新 ID。
      if (shouldSuppressTrackSpawn(detections[detection_index], tracks_, config_)) {
        ++suppressed_track_spawns;
        continue;
      }
      createTrack(detections[detection_index], header.stamp);
    }
  }

  for (std::size_t track_index = 0; track_index < matched_tracks.size(); ++track_index) {
    if (!matched_tracks[track_index]) {
      // 未匹配轨迹进入 coast 状态，保留短期重关联机会，而不是立刻删除。
      coastTrack(tracks_[track_index], header.stamp);
    }
  }

  deleteExpiredTracks();
  result.timing.update_tracks_ms = elapsedMs(update_tracks_start, std::chrono::steady_clock::now());

  const auto build_output_start = std::chrono::steady_clock::now();
  result.dynamic_objects_msg = buildOutput(header);
  result.timing.build_output_ms = elapsedMs(build_output_start, std::chrono::steady_clock::now());

  const auto build_markers_start = std::chrono::steady_clock::now();
  result.dynamic_track_markers_msg = buildTrackMarkers(header);
  result.timing.build_track_markers_ms =
      elapsedMs(build_markers_start, std::chrono::steady_clock::now());

  result.timing.process_dynamic_tracks_ms = elapsedMs(total_start, std::chrono::steady_clock::now());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Dynamic object tracker timing"
                                      << ": processDynamicTracks="
                                      << formatFloatMs(result.timing.process_dynamic_tracks_ms)
                                      << "ms, association="
                                      << formatFloatMs(result.timing.association_ms)
                                      << "ms, updateTracks="
                                      << formatFloatMs(result.timing.update_tracks_ms)
                                      << "ms, buildOutput="
                                      << formatFloatMs(result.timing.build_output_ms)
                                      << "ms, buildTrackMarkers="
                                      << formatFloatMs(result.timing.build_track_markers_ms)
                                      << "ms, detections=" << detections.size()
                                      << ", tracks=" << tracks_.size()
                                      << ", suppressedSpawns=" << suppressed_track_spawns
                                      << ", publishedObjects=" << result.dynamic_objects_msg.objects.size());
  }

  return result;
}

visualization_msgs::MarkerArray DynamicObjectTracker::buildTrackMarkers(
    const std_msgs::Header& header) const {
  visualization_msgs::MarkerArray markers;
  markers.markers.reserve(1U + tracks_.size() * 2U);
  markers.markers.push_back(makeTrackDeleteAllMarker(header));

  for (const auto& track : tracks_) {
    const bool coasting = track.missed_frames > 0U;
    const auto color = makeTrackColor(coasting);

    visualization_msgs::Marker history_marker;
    history_marker.header = header;
    history_marker.ns = "dynamic_track_history";
    history_marker.id = static_cast<int>(track.id * 2U);
    history_marker.type = visualization_msgs::Marker::LINE_STRIP;
    history_marker.action = visualization_msgs::Marker::ADD;
    history_marker.pose.orientation.w = 1.0;
    history_marker.scale.x = 0.04;
    history_marker.color = color;
    history_marker.points.reserve(track.history.size());
    for (const auto& sample : track.history) {
      history_marker.points.push_back(sample.position);
    }
    // RViz 要求 LINE_STRIP 至少有 2 个点；新轨迹首帧只保留头部球体，避免显示状态反复报错。
    if (history_marker.points.size() >= 2U) {
      markers.markers.push_back(std::move(history_marker));
    }

    visualization_msgs::Marker head_marker;
    head_marker.header = header;
    head_marker.ns = "dynamic_track_head";
    head_marker.id = static_cast<int>(track.id * 2U + 1U);
    head_marker.type = visualization_msgs::Marker::SPHERE;
    head_marker.action = visualization_msgs::Marker::ADD;
    head_marker.pose.position = track.bbox.center;
    head_marker.pose.orientation.w = 1.0;
    head_marker.scale.x = 0.18;
    head_marker.scale.y = 0.18;
    head_marker.scale.z = 0.18;
    head_marker.color = color;
    markers.markers.push_back(std::move(head_marker));
  }

  return markers;
}

void DynamicObjectTracker::predictTracks(const ros::Time& stamp) {
  for (auto& track : tracks_) {
    // 丢帧期间也按帧间 dt 外推，避免下一次匹配把多帧时间压成一次大步长。
    // 当时间戳缺失或异常时，这里才回退到 filter_config.dt 这个默认步长。
    const double dt =
        computeDeltaSeconds(stamp, track.last_stamp, config_.filter_config.dt, config_.max_dt);
    // setDt() 这里只更新本次预测要使用的 dt / F；
    // adaptive Q/R 的估计与保留发生在 filter.update() 内部。
    track.filter->setDt(dt);
    track.filter->predict();
    updateBBoxCenterFromFilter(track);
  }
}

void DynamicObjectTracker::updateMatchedTrack(TrackState& track,
                                             const DynamicObjectDetection& detection,
                                             const ros::Time& stamp) {
  // filter 只接收 detection 中心作为 3D 位置观测；bbox 尺寸另行维护，不进入状态向量。
  track.filter->update(pointToEigen(detection.bbox.center));
  const auto previous_size = track.bbox.size;
  track.bbox = detection.bbox;
  // 尺寸变化通常比中心位置更受点云稀疏/视角影响，因此只对 size 做指数平滑。
  track.bbox.size.x = config_.box_size_smoothing_alpha * detection.bbox.size.x +
                      (1.0 - config_.box_size_smoothing_alpha) * previous_size.x;
  track.bbox.size.y = config_.box_size_smoothing_alpha * detection.bbox.size.y +
                      (1.0 - config_.box_size_smoothing_alpha) * previous_size.y;
  track.bbox.size.z = config_.box_size_smoothing_alpha * detection.bbox.size.z +
                      (1.0 - config_.box_size_smoothing_alpha) * previous_size.z;
  updateBBoxCenterFromFilter(track);
  track.age += 1U;
  track.hits += 1U;
  track.missed_frames = 0U;
  track.last_stamp = stamp;
  appendHistorySample(track, stamp, true, config_.max_history_size);
}

void DynamicObjectTracker::createTrack(const DynamicObjectDetection& detection, const ros::Time& stamp) {
  TrackState track(next_track_id_++, config_.model_type, config_.filter_config);
  // 参考 LDOT：新轨迹直接以首帧 detection 中心初始化状态，避免首帧仍被 Kalman 增益
  // 拉向原点/旧参考点，导致蓝色轨迹头落在无人机与目标之间。
  track.filter->initialize(pointToEigen(detection.bbox.center));
  track.bbox = detection.bbox;
  updateBBoxCenterFromFilter(track);
  track.age = 1U;
  track.hits = 1U;
  track.missed_frames = 0U;
  track.last_stamp = stamp;
  appendHistorySample(track, stamp, true, config_.max_history_size);
  tracks_.push_back(std::move(track));
}

void DynamicObjectTracker::coastTrack(TrackState& track, const ros::Time& stamp) {
  track.age += 1U;
  track.missed_frames += 1U;
  track.last_stamp = stamp;
  updateBBoxCenterFromFilter(track);
  // phase 3 固定记录最小历史样本，供后续轨迹状态和调试直接复用。
  appendHistorySample(track, stamp, false, config_.max_history_size);
}

void DynamicObjectTracker::deleteExpiredTracks() {
  // 只删除连续漏检超过阈值的轨迹；短期漏检由 coastTrack() 继续维护。
  tracks_.erase(std::remove_if(tracks_.begin(),
                               tracks_.end(),
                               [this](const TrackState& track) {
                                 return track.missed_frames > config_.max_coast_frames;
                               }),
                tracks_.end());
}

ldop::DynamicObjectArray DynamicObjectTracker::buildOutput(const std_msgs::Header& header) const {
  ldop::DynamicObjectArray object_array;
  object_array.header = header;

  std::vector<const TrackState*> ordered_tracks;
  ordered_tracks.reserve(tracks_.size());
  for (const auto& track : tracks_) {
    ordered_tracks.push_back(&track);
  }
  std::sort(ordered_tracks.begin(),
            ordered_tracks.end(),
            [](const TrackState* lhs, const TrackState* rhs) { return lhs->id < rhs->id; });

  object_array.objects.reserve(ordered_tracks.size());
  for (const TrackState* track : ordered_tracks) {
    // hits 阈值用于过滤只出现一两帧的候选轨迹，降低对外输出抖动。
    if (track->hits < config_.min_hits_to_publish) {
      continue;
    }
    ldop::DynamicObject object;
    object.id = track->id;
    object.center = track->bbox.center;
    object.size = track->bbox.size;
    object.velocity = eigenToVector(track->filter->velocity());
    object_array.objects.push_back(std::move(object));
  }

  return object_array;
}

}  // namespace ldopcore
