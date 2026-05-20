#include <ldop/dynamic_object_predictor.h>
#include <ldop/utils.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>

#include <visualization_msgs/Marker.h>

namespace ldopcore {

namespace {

// 预测尾部对规划/RViz 的最小影响权重；前半段保持 1.0，后半段线性衰减到该值。
constexpr double kMinInfluenceWeight = 0.3;
// 时间比较用的小量，避免浮点精度导致最后一个采样点被跳过。
constexpr double kTimeEpsilon = 1e-9;
// 匹配目标和漏检目标的预测轨迹透明度。
constexpr double kMatchedAlpha = 1.0;
constexpr double kCoastingAlpha = 0.4;
// 预测线宽沿用 tracker 历史线的量级，避免 MAP 分支在 RViz 中压过目标框和历史轨迹。
constexpr double kPredictionLineWidth = 0.04;
constexpr double kMinMarkerAlpha = 0.05;
// 交互修正从当前目标状态向未来逐步生效，避免把整条轨迹刚体平移到离开当前目标的位置。
constexpr double kCorrectionAnchorRampPoints = 2.0;
// 交互控制在时间上做局部平滑，避免单个冲突采样点把 planner-facing 轨迹折成尖角。
constexpr std::size_t kInteractionControlSmoothingRadius = 2U;

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

// 根据分支行为类型和概率设置 marker 颜色；概率越高透明度越大，便于区分主次分支。
void applyBranchColor(const ldop::DynamicObjectPredictionBranch& branch,
                      const double target_alpha,
                      const double min_alpha,
                      visualization_msgs::Marker& marker) {
  marker.color.a = static_cast<float>(
      std::clamp(target_alpha * (0.7 + 0.3 * branch.probability), min_alpha, 1.0));
  switch (branch.behavior_type) {
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL:
      marker.color.r = 0.0F;
      marker.color.g = 1.0F;
      marker.color.b = 1.0F;
      break;
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING:
      marker.color.r = 1.0F;
      marker.color.g = 0.0F;
      marker.color.b = 0.0F;
      break;
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL:
      marker.color.r = 0.0F;
      marker.color.g = 1.0F;
      marker.color.b = 0.0F;
      break;
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL:
      marker.color.r = 1.0F;
      marker.color.g = 0.0F;
      marker.color.b = 1.0F;
      break;
    default:
      marker.color.r = 0.0F;
      marker.color.g = 1.0F;
      marker.color.b = 1.0F;
      break;
  }
}

// --- 预测模式相关类型和辅助函数 ---

// 预测模式类别，用于区分不同意图假设。
enum class PredictionModeKind {
  Keep,
  LongitudinalDecelerate,
  LongitudinalStop,
  TurnLeft,
  TurnRight,
  LateralLeft,
  LateralRight,
  VerticalClimb,
  VerticalDescend,
  VerticalHold,
};

// 从历史轨迹中提取的运动特征，供模式生成使用。
struct MotionHistoryFeatures {
  Eigen::Vector3d heading{1.0, 0.0, 0.0};
  double speed{0.0};
  double previous_speed{0.0};
  double speed_delta{0.0};
  double vertical_speed{0.0};
  double heading_instability{1.0};
  double history_quality{0.0};
  double history_quality_scale{1.0};
};

// 单个预测模式的完整描述。
struct PredictionMode {
  PredictionModeKind kind{PredictionModeKind::Keep};
  std::uint8_t behavior_type{ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL};
  double behavior_value{0.0};
  double score{0.0};
  double probability{0.0};
  double mode_uncertainty_scale{1.0};
};

struct BranchRolloutContext {
  const TrackPredictionInput* input{nullptr};
  MotionHistoryFeatures features;
  std::vector<PredictionMode> modes;
};

struct SmoothInteractionControlSample {
  Eigen::Vector3d position_offset{Eigen::Vector3d::Zero()};
  double deceleration_ratio{0.0};
  Eigen::Vector3d direction_bias{Eigen::Vector3d::Zero()};
};

struct SmoothInteractionControlProfile {
  std::vector<SmoothInteractionControlSample> samples;
};

// 从运动模型状态中提取三维速度向量。
Eigen::Vector3d velocityFromState(const MotionModelType model_type,
                                  const Eigen::VectorXd& state) {
  switch (model_type) {
    case MotionModelType::CA2D:
      assert(state.size() >= 5);
      return Eigen::Vector3d(state(3), state(4), 0.0);
    case MotionModelType::CA3D:
    case MotionModelType::CV3D:
      assert(state.size() >= 6);
      return state.segment<3>(3);
    case MotionModelType::CTRA: {
      assert(state.size() >= 6);
      const double speed = state(3);
      const double yaw = state(5);
      return Eigen::Vector3d(speed * std::cos(yaw), speed * std::sin(yaw), 0.0);
    }
  }
  return Eigen::Vector3d::Zero();
}

// 将向量投影到水平面并归一化；零向量返回默认朝向 (1,0,0)。
Eigen::Vector3d normalizedHorizontal(const Eigen::Vector3d& vector) {
  Eigen::Vector3d horizontal(vector.x(), vector.y(), 0.0);
  const double norm = horizontal.norm();
  if (norm <= 1e-9) {
    return Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  return horizontal / norm;
}

// 从输入的历史轨迹中估计运动特征：速度、朝向、航向不稳定度、历史质量等。
MotionHistoryFeatures estimateHistoryFeatures(const TrackPredictionInput& input,
                                              const DynamicObjectPredictorConfig& config) {
  MotionHistoryFeatures features;
  const Eigen::Vector3d current_velocity =
      velocityFromState(input.motion_model_type, input.model_state);
  features.speed = current_velocity.head<2>().norm();
  features.vertical_speed = current_velocity.z();
  features.heading = normalizedHorizontal(current_velocity);

  const std::size_t history_size = input.history.size();
  const std::size_t used_size = std::min(history_size, config.history_window);
  if (used_size >= 2U) {
    const std::size_t first_index = history_size - used_size;
    const Eigen::Vector3d first_position = input.history[first_index].model_state.segment<3>(0);
    const Eigen::Vector3d last_position = input.history.back().model_state.segment<3>(0);
    const Eigen::Vector3d displacement = last_position - first_position;
    if (displacement.head<2>().norm() > 1e-6) {
      features.heading = normalizedHorizontal(displacement);
    }

    // tracker 传入 predictor 时，当前状态通常已经等于 history.back()；
    // 趋势判断必须回看窗口起点，否则“持续减速”会被当前帧与当前帧相减抵消。
    const Eigen::Vector3d previous_velocity =
        velocityFromState(input.history[first_index].motion_model_type,
                          input.history[first_index].model_state);
    features.previous_speed = previous_velocity.head<2>().norm();
    features.speed_delta = features.speed - features.previous_speed;

    // 计算历史航向与整体位移方向的平均偏差，衡量航向稳定性。
    double angle_sum = 0.0;
    std::size_t angle_count = 0U;
    for (std::size_t index = first_index + 1U; index < history_size; ++index) {
      const Eigen::Vector3d delta =
          input.history[index].model_state.segment<3>(0) -
          input.history[index - 1U].model_state.segment<3>(0);
      if (delta.head<2>().norm() <= 1e-6) {
        continue;
      }
      const double dot = std::clamp(normalizedHorizontal(delta).dot(features.heading), -1.0, 1.0);
      angle_sum += std::acos(dot);
      ++angle_count;
    }
    features.heading_instability =
        angle_count > 0U ? angle_sum / static_cast<double>(angle_count) : 0.0;
    features.history_quality =
        std::min(1.0, static_cast<double>(used_size) / static_cast<double>(config.history_window));
  }

  if (used_size < 2U) {
    features.previous_speed = features.speed;
    features.speed_delta = 0.0;
    features.history_quality = 0.0;
    features.heading_instability = config.heading_stable_angle;
  }

  // 综合漏检率、航向不稳定度和历史不足度计算质量惩罚系数。
  const double missed_ratio = input.age > 0U
                                  ? static_cast<double>(input.missed_frames) /
                                        static_cast<double>(input.age)
                                  : 0.0;
  const double instability_ratio =
      std::clamp(features.heading_instability / config.heading_stable_angle, 0.0, 1.0);
  const double poor_history_ratio = 1.0 - features.history_quality;
  const double quality_penalty =
      std::clamp(0.5 * poor_history_ratio + 0.3 * missed_ratio + 0.2 * instability_ratio,
                 0.0,
                 1.0);
  features.history_quality_scale =
      1.0 + (config.history_quality_noise_scale - 1.0) * quality_penalty;
  return features;
}

// 构造一个预测模式实例。
PredictionMode makeMode(const PredictionModeKind kind,
                        const std::uint8_t behavior_type,
                        const double behavior_value,
                        const double score,
                        const double uncertainty_scale) {
  PredictionMode mode;
  mode.kind = kind;
  mode.behavior_type = behavior_type;
  mode.behavior_value = behavior_value;
  mode.score = score;
  mode.mode_uncertainty_scale = uncertainty_scale;
  return mode;
}

// 对模式列表做 softmax 归一化、按概率排序、剪枝并重新归一化。
void normalizeAndPruneModes(std::vector<PredictionMode>& modes,
                            const DynamicObjectPredictorConfig& config) {
  if (modes.empty()) {
    modes.push_back(makeMode(PredictionModeKind::Keep,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
                             0.0,
                             1.0,
                             1.0));
  }

  const auto max_score_it = std::max_element(
      modes.begin(), modes.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.score < rhs.score;
      });
  const PredictionMode best_mode = *max_score_it;
  const double max_score = max_score_it->score;
  double probability_sum = 0.0;
  for (PredictionMode& mode : modes) {
    mode.probability = std::exp(mode.score - max_score);
    probability_sum += mode.probability;
  }
  assert(probability_sum > 0.0);
  for (PredictionMode& mode : modes) {
    mode.probability /= probability_sum;
  }

  std::sort(modes.begin(), modes.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.probability > rhs.probability;
  });

  modes.erase(std::remove_if(modes.begin(),
                             modes.end(),
                             [&](const auto& mode) {
                               return mode.probability < config.min_branch_probability;
                             }),
              modes.end());
  if (modes.empty()) {
    modes.push_back(best_mode);
  }
  if (modes.size() > config.max_branches) {
    modes.resize(config.max_branches);
  }

  // 剪枝后重新归一化概率之和为 1。
  const double kept_sum = std::accumulate(
      modes.begin(), modes.end(), 0.0, [](const double sum, const auto& mode) {
        return sum + mode.probability;
      });
  assert(kept_sum > 0.0);
  for (PredictionMode& mode : modes) {
    mode.probability /= kept_sum;
  }
}

// 根据输入和历史特征生成所有候选预测模式。
std::vector<PredictionMode> generatePredictionModes(
    const TrackPredictionInput& input,
    const MotionHistoryFeatures& features,
    const DynamicObjectPredictorConfig& config) {
  std::vector<PredictionMode> modes;
  const bool stable_heading = features.heading_instability <= config.heading_stable_angle;
  const bool slowing = features.speed_delta < -0.05;
  const double keep_score = 1.6 + (stable_heading ? 0.6 : 0.0) + 0.4 * features.history_quality;
  const double decel_score = 0.6 + (slowing ? 0.8 : 0.0);
  const double stop_score =
      0.2 + (slowing ? 0.6 : 0.0) + (features.speed <= config.stop_speed ? 0.6 : 0.0);
  const double turn_score = 0.25 + (stable_heading ? 0.0 : 0.25);
  const double lateral_score = 0.15 + (stable_heading ? 0.0 : 0.35);

  modes.push_back(makeMode(PredictionModeKind::Keep,
                           ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
                           0.0,
                           keep_score,
                           1.0));

  // 按类别组装候选模式的 lambda 辅助函数。
  const auto add_deceleration = [&]() {
    modes.push_back(makeMode(PredictionModeKind::LongitudinalDecelerate,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
                             -0.45,
                             decel_score,
                             config.hypothesis_noise_scale));
  };
  const auto add_turn_pair = [&]() {
    modes.push_back(makeMode(PredictionModeKind::TurnLeft,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING,
                             1.0,
                             turn_score,
                             config.hypothesis_noise_scale));
    modes.push_back(makeMode(PredictionModeKind::TurnRight,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING,
                             -1.0,
                             turn_score,
                             config.hypothesis_noise_scale));
  };
  const auto add_lateral_pair = [&]() {
    modes.push_back(makeMode(PredictionModeKind::LateralLeft,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL,
                             1.0,
                             lateral_score,
                             config.hypothesis_noise_scale));
    modes.push_back(makeMode(PredictionModeKind::LateralRight,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL,
                             -1.0,
                             lateral_score,
                             config.hypothesis_noise_scale));
  };
  // 垂直模式集合：根据当前垂直速度判断爬升/下降/保持的先验分数。
  const auto add_vertical_set = [&]() {
    const double climb_score = 0.25 + (features.vertical_speed > 0.05 ? 0.6 : 0.0);
    const double descend_score = 0.25 + (features.vertical_speed < -0.05 ? 0.6 : 0.0);
    const double hold_score = 0.45 + (std::abs(features.vertical_speed) <= 0.05 ? 0.4 : 0.0);
    modes.push_back(makeMode(PredictionModeKind::VerticalClimb,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL,
                             1.0,
                             climb_score,
                             config.hypothesis_noise_scale));
    modes.push_back(makeMode(PredictionModeKind::VerticalDescend,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL,
                             -1.0,
                             descend_score,
                             config.hypothesis_noise_scale));
    modes.push_back(makeMode(PredictionModeKind::VerticalHold,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL,
                             0.0,
                             hold_score,
                             1.0));
  };

  // 按目标类别选择对应的意图分支组合。
  switch (input.object_class) {
    case ObjectClass::Human:
      add_deceleration();
      modes.push_back(makeMode(PredictionModeKind::LongitudinalStop,
                               ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
                               -1.0,
                               stop_score,
                               config.hypothesis_noise_scale));
      add_turn_pair();
      add_lateral_pair();
      break;
    case ObjectClass::Vehicle:
      add_deceleration();
      add_turn_pair();
      break;
    case ObjectClass::Uav:
      add_deceleration();
      add_turn_pair();
      add_vertical_set();
      break;
    case ObjectClass::Other:
    case ObjectClass::Unknown:
      // 保守策略：航向不稳或减速时才添加减速/转向分支。
      if (!stable_heading || slowing) {
        add_deceleration();
      }
      if (!stable_heading && config.max_branches > 2U) {
        add_turn_pair();
      }
      // unknown/other 不主动假设 2.5D 意图；只有当前 z 速度给出明确证据时才补充垂向候选。
      if (std::abs(features.vertical_speed) > 0.05) {
        add_vertical_set();
      }
      break;
  }

  normalizeAndPruneModes(modes, config);
  return modes;
}

// 二维向量旋转。
Eigen::Vector2d rotate2D(const Eigen::Vector2d& vector, const double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return Eigen::Vector2d(c * vector.x() - s * vector.y(),
                         s * vector.x() + c * vector.y());
}

// 从水平朝向向量取左侧法线方向。
Eigen::Vector2d horizontalNormalLeft(const Eigen::Vector3d& heading) {
  return Eigen::Vector2d(-heading.y(), heading.x());
}

// 按模型类型设置水平速度分量。
void setHorizontalVelocity(const MotionModelType model_type,
                           Eigen::VectorXd& state,
                           const Eigen::Vector2d& velocity) {
  switch (model_type) {
    case MotionModelType::CA2D:
    case MotionModelType::CA3D:
    case MotionModelType::CV3D:
      assert(state.size() >= 5);
      state(3) = velocity.x();
      state(4) = velocity.y();
      break;
    case MotionModelType::CTRA: {
      assert(state.size() >= 6);
      state(3) = velocity.norm();
      if (velocity.norm() > 1e-9) {
        state(5) = std::atan2(velocity.y(), velocity.x());
      }
      break;
    }
  }
}

double correctionAnchorScale(const std::size_t point_index) {
  if (point_index == 0U) {
    return 0.0;
  }
  return std::clamp(static_cast<double>(point_index) / kCorrectionAnchorRampPoints,
                    0.0,
                    1.0);
}

// 按模型类型设置垂直速度分量。
void setVerticalVelocity(const MotionModelType model_type,
                         Eigen::VectorXd& state,
                         const double vertical_velocity) {
  if ((model_type == MotionModelType::CA3D || model_type == MotionModelType::CV3D) &&
      state.size() >= 6) {
    state(5) = vertical_velocity;
  }
  if (model_type == MotionModelType::CA3D && state.size() >= 9) {
    state(8) = 0.0;
  }
}

Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d& vector,
                                const double max_norm) {
  if (max_norm <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double norm = vector.norm();
  if (norm <= max_norm || norm <= 1e-9) {
    return vector;
  }
  return vector * (max_norm / norm);
}

double interactionSmoothingWeight(const std::size_t lhs,
                                  const std::size_t rhs) {
  const std::size_t distance = lhs > rhs ? lhs - rhs : rhs - lhs;
  if (distance > kInteractionControlSmoothingRadius) {
    return 0.0;
  }
  return static_cast<double>(kInteractionControlSmoothingRadius + 1U - distance);
}

SmoothInteractionControlProfile buildSmoothInteractionControlProfile(
    const PredictionBranchInteraction& interaction,
    const DynamicObjectPredictorConfig& config) {
  SmoothInteractionControlProfile profile;
  profile.samples.resize(interaction.point_correction_hints.size());

  for (std::size_t point_index = 0U;
       point_index < interaction.point_correction_hints.size();
       ++point_index) {
    double weight_sum = 0.0;
    Eigen::Vector3d position_offset = Eigen::Vector3d::Zero();
    double deceleration_ratio = 0.0;
    Eigen::Vector3d direction_bias = Eigen::Vector3d::Zero();

    for (std::size_t hint_index = 0U;
         hint_index < interaction.point_correction_hints.size();
         ++hint_index) {
      const double weight = interactionSmoothingWeight(point_index, hint_index);
      if (weight <= 0.0) {
        continue;
      }
      weight_sum += weight;
      const auto& hint = interaction.point_correction_hints[hint_index];
      if (hint.has_position_offset) {
        position_offset += weight * hint.position_offset;
      }
      if (hint.has_deceleration) {
        deceleration_ratio += weight * hint.deceleration_ratio;
      }
      if (hint.has_direction_bias) {
        direction_bias += weight * hint.direction_bias;
      }
    }

    if (weight_sum <= 0.0) {
      continue;
    }

    const double anchor_scale = correctionAnchorScale(point_index);
    auto& sample = profile.samples[point_index];
    sample.position_offset = clampVectorNorm(
        anchor_scale * position_offset / weight_sum,
        config.interaction_config.max_correction_distance);
    sample.deceleration_ratio = std::clamp(
        anchor_scale * deceleration_ratio / weight_sum,
        0.0,
        config.interaction_config.max_deceleration_ratio);
    sample.direction_bias = clampVectorNorm(
        anchor_scale * direction_bias / weight_sum,
        1.0);
  }

  return profile;
}

bool hasInteractionControl(const SmoothInteractionControlProfile& profile) {
  for (const auto& sample : profile.samples) {
    if (sample.position_offset.squaredNorm() > 1e-12 ||
        sample.deceleration_ratio > 1e-9 ||
        sample.direction_bias.squaredNorm() > 1e-12) {
      return true;
    }
  }
  return false;
}

Eigen::Vector3d positionVelocityControlDelta(
    const SmoothInteractionControlProfile& profile,
    const std::size_t point_index,
    const double dt) {
  if (dt <= 0.0 || point_index >= profile.samples.size()) {
    return Eigen::Vector3d::Zero();
  }
  const Eigen::Vector3d current_offset = profile.samples[point_index].position_offset;
  const Eigen::Vector3d previous_offset =
      point_index > 0U ? profile.samples[point_index - 1U].position_offset
                       : Eigen::Vector3d::Zero();
  const Eigen::Vector3d before_previous_offset =
      point_index > 1U ? profile.samples[point_index - 2U].position_offset
                       : Eigen::Vector3d::Zero();
  const Eigen::Vector3d current_control_velocity =
      (current_offset - previous_offset) / dt;
  Eigen::Vector3d previous_control_velocity = Eigen::Vector3d::Zero();
  if (point_index > 0U) {
    previous_control_velocity = (previous_offset - before_previous_offset) / dt;
  }
  return current_control_velocity - previous_control_velocity;
}

void applySmoothInteractionControl(const SmoothInteractionControlProfile& profile,
                                   const std::size_t point_index,
                                   const double dt,
                                   const MotionModelType model_type,
                                   Eigen::VectorXd& state) {
  if (point_index >= profile.samples.size()) {
    return;
  }

  Eigen::Vector3d velocity = velocityFromState(model_type, state);
  // 位置 hint 先转成速度控制的变化量，让模型积分生成平滑中心轨迹，而不是逐点改写位置。
  velocity += positionVelocityControlDelta(profile, point_index, dt);

  const double current_ratio = profile.samples[point_index].deceleration_ratio;
  const double previous_ratio =
      point_index > 0U ? profile.samples[point_index - 1U].deceleration_ratio : 0.0;
  const double current_scale = std::max(0.0, 1.0 - current_ratio);
  const double previous_scale = std::max(1e-6, 1.0 - previous_ratio);
  velocity.head<2>() *= current_scale / previous_scale;

  const Eigen::Vector2d bias(profile.samples[point_index].direction_bias.x(),
                             profile.samples[point_index].direction_bias.y());
  Eigen::Vector2d horizontal_velocity(velocity.x(), velocity.y());
  const double speed = horizontal_velocity.norm();
  if (speed > 1e-9 && bias.norm() > 1e-9) {
    const Eigen::Vector2d biased_direction = horizontal_velocity.normalized() + bias;
    if (biased_direction.norm() > 1e-9) {
      horizontal_velocity = biased_direction.normalized() * speed;
    }
  }

  setHorizontalVelocity(model_type, state, horizontal_velocity);
  setVerticalVelocity(model_type, state, velocity.z());
}

std::size_t maxPriorProbabilityBranchIndex(
    const ldop::DynamicObjectPrediction& prediction) {
  std::size_t best_index = 0U;
  double best_probability = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < prediction.branches.size(); ++index) {
    const double probability = prediction.branches[index].probability;
    if (std::isfinite(probability) && probability > best_probability) {
      best_probability = probability;
      best_index = index;
    }
  }
  return best_index;
}

void renormalizeBranchProbabilities(ldop::DynamicObjectPrediction& prediction,
                                    const std::size_t fallback_index) {
  if (prediction.branches.empty()) {
    return;
  }

  double probability_sum = 0.0;
  for (auto& branch : prediction.branches) {
    if (!std::isfinite(branch.probability) || branch.probability < 0.0) {
      branch.probability = 0.0;
    }
    probability_sum += branch.probability;
  }

  if (probability_sum > 1e-12 && std::isfinite(probability_sum)) {
    for (auto& branch : prediction.branches) {
      branch.probability /= probability_sum;
    }
    return;
  }

  // 极端缩放下仍保留一个明确 MAP 分支，避免下游收到全零或 NaN 的 GMM 权重。
  const std::size_t clamped_fallback_index =
      std::min(fallback_index, prediction.branches.size() - 1U);
  for (std::size_t index = 0U; index < prediction.branches.size(); ++index) {
    prediction.branches[index].probability =
        index == clamped_fallback_index ? 1.0 : 0.0;
  }
}

// 按当前预测模式对状态施加控制修正，在模型传播前调用。
void applyModeControl(const PredictionMode& mode,
                      const MotionHistoryFeatures& features,
                      const DynamicObjectPredictorConfig& config,
                      const double dt,
                      const MotionModelType model_type,
                      Eigen::VectorXd& state) {
  Eigen::Vector3d velocity = velocityFromState(model_type, state);
  Eigen::Vector2d horizontal_velocity(velocity.x(), velocity.y());
  const Eigen::Vector2d heading(features.heading.x(), features.heading.y());
  const Eigen::Vector2d normal = horizontalNormalLeft(features.heading);

  switch (mode.kind) {
    case PredictionModeKind::Keep:
    case PredictionModeKind::VerticalHold:
      break;
    case PredictionModeKind::LongitudinalDecelerate: {
      const double speed = horizontal_velocity.norm();
      // decelerate 表达“比 keep 慢一些”，不能把低速目标抬到 stop_speed；
      // stop_speed 只作为减速度尺度使用，真正停止由 stop mode 表达。
      const double next_speed = std::max(0.0, speed - config.stop_speed * dt);
      horizontal_velocity = speed > 1e-9
                                 ? Eigen::Vector2d(horizontal_velocity.normalized() * next_speed)
                                 : Eigen::Vector2d(heading * next_speed);
      setHorizontalVelocity(model_type, state, horizontal_velocity);
      break;
    }
    case PredictionModeKind::LongitudinalStop: {
      const double speed = horizontal_velocity.norm();
      const double next_speed = std::max(0.0, speed - std::max(speed, config.stop_speed) * dt);
      horizontal_velocity = speed > 1e-9
                                 ? Eigen::Vector2d(horizontal_velocity.normalized() * next_speed)
                                 : Eigen::Vector2d::Zero();
      setHorizontalVelocity(model_type, state, horizontal_velocity);
      break;
    }
    case PredictionModeKind::TurnLeft:
    case PredictionModeKind::TurnRight: {
      const double sign = mode.kind == PredictionModeKind::TurnLeft ? 1.0 : -1.0;
      if (model_type == MotionModelType::CTRA && state.size() >= 7) {
        state(6) = sign * config.turn_yaw_rate;
      } else {
        setHorizontalVelocity(model_type,
                              state,
                              rotate2D(horizontal_velocity, sign * config.turn_yaw_rate * dt));
      }
      break;
    }
    case PredictionModeKind::LateralLeft:
    case PredictionModeKind::LateralRight: {
      const double sign = mode.kind == PredictionModeKind::LateralLeft ? 1.0 : -1.0;
      setHorizontalVelocity(model_type,
                            state,
                            horizontal_velocity + sign * config.lateral_speed * normal);
      break;
    }
    case PredictionModeKind::VerticalClimb:
      setVerticalVelocity(model_type, state, config.vertical_speed);
      break;
    case PredictionModeKind::VerticalDescend:
      setVerticalVelocity(model_type, state, -config.vertical_speed);
      break;
  }
}

// 按给定模式滚出一条预测分支；每步传播前根据模式类型和可选交互控制修正状态。
ldop::DynamicObjectPredictionBranch rollOutMode(
    const TrackPredictionInput& input,
    const PredictionMode& mode,
    const MotionHistoryFeatures& features,
    const DynamicObjectPredictorConfig& config,
    const SmoothInteractionControlProfile* interaction_control = nullptr) {
  std::shared_ptr<MotionModel> model =
      createMotionModel(input.motion_model_type, config.filter_config);
  model->setDt(config.filter_config.default_dt);

  Eigen::VectorXd state = input.model_state;
  Eigen::MatrixXd covariance = input.model_covariance;
  assert(state.size() == model->stateDim());
  assert(covariance.rows() == state.size());
  assert(covariance.cols() == state.size());

  ldop::DynamicObjectPredictionBranch branch;
  branch.behavior_type = mode.behavior_type;
  branch.behavior_value = mode.behavior_value;
  branch.probability = mode.probability;

  std::size_t point_index = 0U;
  for (double elapsed = config.filter_config.default_dt;
       elapsed <= config.prediction_horizon + kTimeEpsilon;
       elapsed += config.filter_config.default_dt, ++point_index) {
    applyModeControl(mode, features, config, config.filter_config.default_dt, input.motion_model_type, state);
    if (interaction_control != nullptr) {
      applySmoothInteractionControl(*interaction_control,
                                    point_index,
                                    config.filter_config.default_dt,
                                    input.motion_model_type,
                                    state);
    }
    const Eigen::VectorXd previous_state = state;
    const Eigen::MatrixXd transition = model->getTransitionF(previous_state);
    state = model->stateTransition(previous_state);
    model->normalizeYaw(state);
    // P0 已经携带 tracker 当前滤波不确定性；这里的尺度只表达未来意图假设和历史质量带来的额外不确定性。
    const double noise_scale = features.history_quality_scale * mode.mode_uncertainty_scale;
    const Eigen::MatrixXd process_noise = noise_scale * model->getProcessNoiseQ();
    covariance = transition * covariance * transition.transpose() + process_noise;
    covariance = 0.5 * (covariance + covariance.transpose());

    ldop::DynamicObjectPredictionPoint point;
    point.time_from_start = ros::Duration(elapsed);
    point.model_state = eigenVectorToStdVector(state);
    point.model_covariance = eigenMatrixToRowMajorStdVector(covariance);
    point.influence_weight = computeInfluenceWeight(elapsed, config.prediction_horizon);
    branch.points.push_back(std::move(point));
  }

  return branch;
}

void applyInteractionResult(const PredictionInteractionResult& interaction_result,
                            const DynamicObjectPredictorConfig& config,
                            const std::vector<BranchRolloutContext>& rollout_contexts,
                            ldop::DynamicObjectPredictionArray& predictions) {
  const double interaction_tau =
      config.interaction_config.interaction_energy_tau > 0.0
          ? config.interaction_config.interaction_energy_tau
          : 1.0;
  const std::size_t prediction_count = std::min(
      {predictions.predictions.size(),
       interaction_result.interactions_by_prediction.size(),
       rollout_contexts.size()});
  for (std::size_t prediction_index = 0U;
       prediction_index < prediction_count;
       ++prediction_index) {
    auto& prediction = predictions.predictions[prediction_index];
    const auto& interactions =
        interaction_result.interactions_by_prediction[prediction_index];
    const auto& rollout_context = rollout_contexts[prediction_index];
    const std::size_t branch_count =
        std::min({prediction.branches.size(),
                  interactions.size(),
                  rollout_context.modes.size()});
    const std::size_t prior_fallback_index = maxPriorProbabilityBranchIndex(prediction);
    std::size_t fallback_index = prior_fallback_index;
    double best_log_score = -std::numeric_limits<double>::infinity();

    for (std::size_t branch_index = 0U; branch_index < branch_count; ++branch_index) {
      auto& branch = prediction.branches[branch_index];
      const auto& interaction = interactions[branch_index];
      const double prior_probability = branch.probability;
      if (prior_probability > 0.0 && std::isfinite(prior_probability) &&
          std::isfinite(interaction.total_energy)) {
        // fallback 必须基于缩放前的 log 权重判断；直接用 total_energy/tau 可覆盖
        // probability_scale 本身下溢为 0 的极端情况。
        const double log_score =
            std::log(prior_probability) - interaction.total_energy / interaction_tau;
        if (log_score > best_log_score) {
          best_log_score = log_score;
          fallback_index = branch_index;
        }
      }

      const double scaled_probability = branch.probability * interaction.probability_scale;
      const SmoothInteractionControlProfile control_profile =
          buildSmoothInteractionControlProfile(interaction, config);
      if (rollout_context.input != nullptr && hasInteractionControl(control_profile)) {
        // 交互修正先变成平滑控制 profile，再交回运动模型积分。这样 planner-facing
        // 均值轨迹仍是模型 rollout 结果，而不是互不连续的逐点位置改写。
        branch = rollOutMode(*rollout_context.input,
                             rollout_context.modes[branch_index],
                             rollout_context.features,
                             config,
                             &control_profile);
      }
      branch.probability = scaled_probability;
    }
    renormalizeBranchProbabilities(prediction, fallback_index);
  }
}

}  // namespace

DynamicObjectPredictorConfig buildPredictorConfig(
    const DynamicObjectPredictorParams& params) {
  const DynamicObjectPredictorParams defaults;
  DynamicObjectPredictorConfig config;
  config.prediction_horizon =
      params.prediction_horizon > 0.0 ? params.prediction_horizon
                                      : defaults.prediction_horizon;
  config.max_branches = static_cast<std::size_t>(
      params.max_branches > 0 ? params.max_branches : defaults.max_branches);
  config.min_branch_probability =
      (params.min_branch_probability >= 0.0 && params.min_branch_probability < 1.0)
          ? params.min_branch_probability
          : defaults.min_branch_probability;
  config.history_window = static_cast<std::size_t>(
      params.history_window > 0 ? params.history_window : defaults.history_window);
  config.heading_stable_angle =
      params.heading_stable_angle > 0.0 ? params.heading_stable_angle
                                        : defaults.heading_stable_angle;
  config.stop_speed = params.stop_speed >= 0.0 ? params.stop_speed : defaults.stop_speed;
  config.turn_yaw_rate =
      params.turn_yaw_rate > 0.0 ? params.turn_yaw_rate : defaults.turn_yaw_rate;
  config.lateral_speed =
      params.lateral_speed >= 0.0 ? params.lateral_speed : defaults.lateral_speed;
  config.vertical_speed =
      params.vertical_speed >= 0.0 ? params.vertical_speed : defaults.vertical_speed;
  config.history_quality_noise_scale =
      params.history_quality_noise_scale >= 1.0 ? params.history_quality_noise_scale
                                                : defaults.history_quality_noise_scale;
  config.hypothesis_noise_scale =
      params.hypothesis_noise_scale >= 1.0 ? params.hypothesis_noise_scale
                                           : defaults.hypothesis_noise_scale;
  config.filter_config = buildMultiModelKalmanFilterConfig(params.filter);
  config.interaction_config = buildPredictionInteractionConfig(params.interaction);
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
  pnh_.param("prediction_max_branches", params_.max_branches, defaults.max_branches);
  pnh_.param("prediction_min_branch_probability",
             params_.min_branch_probability,
             defaults.min_branch_probability);
  pnh_.param("prediction_history_window", params_.history_window, defaults.history_window);
  pnh_.param("prediction_heading_stable_angle",
             params_.heading_stable_angle,
             defaults.heading_stable_angle);
  pnh_.param("prediction_stop_speed", params_.stop_speed, defaults.stop_speed);
  pnh_.param("prediction_turn_yaw_rate", params_.turn_yaw_rate, defaults.turn_yaw_rate);
  pnh_.param("prediction_lateral_speed", params_.lateral_speed, defaults.lateral_speed);
  pnh_.param("prediction_vertical_speed", params_.vertical_speed, defaults.vertical_speed);
  pnh_.param("prediction_history_quality_noise_scale",
             params_.history_quality_noise_scale,
             defaults.history_quality_noise_scale);
  pnh_.param("prediction_hypothesis_noise_scale",
             params_.hypothesis_noise_scale,
             defaults.hypothesis_noise_scale);
  pnh_.param("prediction_enable_interaction_context",
             params_.interaction.enabled,
             defaults.interaction.enabled);
  pnh_.param("prediction_pair_interaction_weight",
             params_.interaction.pair_interaction_weight,
             defaults.interaction.pair_interaction_weight);
  pnh_.param("prediction_map_interaction_weight",
             params_.interaction.map_interaction_weight,
             defaults.interaction.map_interaction_weight);
  pnh_.param("prediction_corridor_interaction_weight",
             params_.interaction.corridor_interaction_weight,
             defaults.interaction.corridor_interaction_weight);
  pnh_.param("prediction_interaction_safe_distance",
             params_.interaction.interaction_safe_distance,
             defaults.interaction.interaction_safe_distance);
  pnh_.param("prediction_near_obstacle_distance",
             params_.interaction.near_obstacle_distance,
             defaults.interaction.near_obstacle_distance);
  pnh_.param("prediction_ttc_safe_time",
             params_.interaction.ttc_safe_time,
             defaults.interaction.ttc_safe_time);
  pnh_.param("prediction_free_corridor_query_radius",
             params_.interaction.free_corridor_query_radius,
             defaults.interaction.free_corridor_query_radius);
  pnh_.param("prediction_max_correction_distance",
             params_.interaction.max_correction_distance,
             defaults.interaction.max_correction_distance);
  pnh_.param("prediction_max_deceleration_ratio",
             params_.interaction.max_deceleration_ratio,
             defaults.interaction.max_deceleration_ratio);
  pnh_.param("prediction_interaction_energy_tau",
             params_.interaction.interaction_energy_tau,
             defaults.interaction.interaction_energy_tau);
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
    const std::vector<TrackPredictionInput>& inputs,
    const PredictionMapQuery* map_query) const {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectPredictorFrameResult result;
  result.predictions_msg.header = header;
  result.predictions_msg.predictions.reserve(inputs.size());
  std::vector<BranchRolloutContext> rollout_contexts;
  rollout_contexts.reserve(inputs.size());

  for (const TrackPredictionInput& input : inputs) {
    ldop::DynamicObjectPrediction prediction;
    prediction.id = input.id;
    prediction.size = input.bbox.size;
    prediction.object_class = toRosObjectClass(input.object_class);
    prediction.motion_model_type = toRosMotionModelType(input.motion_model_type);
    prediction.matched_in_current_frame = input.matched_in_current_frame;

    // GMM 意图分支路径：提取历史特征，生成多模式假设，逐模式滚出分支。
    BranchRolloutContext rollout_context;
    rollout_context.input = &input;
    rollout_context.features = estimateHistoryFeatures(input, config_);
    rollout_context.modes = generatePredictionModes(input, rollout_context.features, config_);

    for (const PredictionMode& mode : rollout_context.modes) {
      prediction.branches.push_back(
          rollOutMode(input, mode, rollout_context.features, config_));
    }

    result.predictions_msg.predictions.push_back(std::move(prediction));
    rollout_contexts.push_back(std::move(rollout_context));
  }

  result.timing.predict_dynamic_objects_ms =
      elapsedMs(total_start, std::chrono::steady_clock::now());

  if (config_.interaction_config.enabled && !result.predictions_msg.predictions.empty()) {
    const auto interaction_start = std::chrono::steady_clock::now();
    // P3 交互先重标定分支概率，再把局部修正证据转为平滑控制并重新 rollout；
    // 协方差继续由 motion model rollout 负责，不因交互代价被人为缩小。
    PredictionInteractionContext context(config_.interaction_config);
    const PredictionInteractionResult interaction_result =
        context.evaluate(header, inputs, result.predictions_msg, map_query);
    applyInteractionResult(interaction_result, config_, rollout_contexts, result.predictions_msg);
    result.interaction_diagnostics = interaction_result.diagnostics;
    result.timing.apply_interaction_context_ms =
        elapsedMs(interaction_start, std::chrono::steady_clock::now());
  }

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
                                       << "ms, applyInteractionContext="
                                       << formatFloatMs(result.timing.apply_interaction_context_ms)
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
    if (prediction.branches.empty()) {
      continue;
    }

    // 匹配目标使用更高透明度，漏检目标使用更低透明度。
    const double target_alpha =
        prediction.matched_in_current_frame ? kMatchedAlpha : kCoastingAlpha;

    // 轨迹线：为每个分支生成独立线 marker，颜色按行为类型和概率区分。
    for (const auto& branch : prediction.branches) {
      if (branch.points.empty()) {
        continue;
      }
      visualization_msgs::Marker line;
      line.header = header;
      line.ns = "dynamic_prediction_line";
      line.id = marker_id++;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = kPredictionLineWidth;
      applyBranchColor(branch, target_alpha, kMinMarkerAlpha, line);

      for (const auto& point : branch.points) {
        assert(point.model_state.size() >= 3U);
        line.points.push_back(makePoint(point.model_state[0],
                                        point.model_state[1],
                                        point.model_state[2]));
      }
      markers.markers.push_back(std::move(line));
    }
  }

  return markers;
}

}  // namespace ldopcore
