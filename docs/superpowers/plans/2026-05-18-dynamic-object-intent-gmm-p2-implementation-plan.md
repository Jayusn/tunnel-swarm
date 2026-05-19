# Dynamic Object Intent GMM P2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement LDOP P2 dynamic-object heading/intent trajectory prediction as branch-level trajectory-level GMM, with RViz showing only branch mean trajectories.

**Current Status:** Implemented and verified. This plan is now a historical implementation record plus review checklist; the checked task boxes below preserve the original execution granularity.

**Architecture:** Extend `DynamicObjectPredictor` only: keep tracker inputs and prediction ROS messages stable, add private history-feature extraction, private mode generation, per-mode rollout, probability normalization, and multi-branch mean-line markers. `/ldop/dynamic_object_predictions` remains the planner-facing GMM branch output; no sampled trajectory markers are emitted.

**Tech Stack:** ROS1 catkin, C++17, Eigen, `gtest`, LDOP `MotionModel` / `TrackPredictionInput` / prediction messages.

**Repository Rule Override:** `AGENTS.md` says not to create intermediate commits during a task. This plan intentionally omits per-task commit steps. After all tasks pass verification, ask the user before committing.

---

## File Structure

- Modify: `include/ldop/dynamic_object_predictor.h`
  - Add P2 predictor parameters and runtime config fields.
  - Keep public API unchanged.

- Modify: `src/dynamic_object_predictor.cpp`
  - Add private `MotionHistoryFeatures` and `PredictionMode`.
  - Add history feature extraction, class-aware mode generation, probability normalization, per-mode rollout, process-noise scaling, and multi-branch mean-line markers.

- Modify: `test/test_dynamic_object_predictor.cpp`
  - Add focused unit tests for mode generation, class strategy, rollout divergence, probability normalization, noise scaling, marker generation, and P2 regression cases.

- Modify: `config/ldop.yaml`
  - Add P2 runtime parameters with the current tuned values used by `roslaunch`.

- Modify: `msg/DynamicObjectPredictionBranch.msg`
  - Remove the obsolete `BEHAVIOR_BASELINE` constant so all retained branches use behavior-family enums.

- Modify: `CMakeLists.txt`
  - Link `test_dynamic_object_predictor` with `gtest` because the test binary now owns `ros::init()`.

- Modify: `launch/rviz/ldop_rviz.rviz`
- Keep only the true branch-line marker namespace for P2 prediction visualization; the removed sample marker path had no actual planning or evaluation value.

No new message files or sampled-trajectory fields are added in P2.

---

## Task 1: Add P2 Parameters Without Old P1 Fallback

**Files:**
- Modify: `include/ldop/dynamic_object_predictor.h`
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `config/ldop.yaml`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add parameter and config fields**

In `include/ldop/dynamic_object_predictor.h`, replace the two structs with:

```cpp
// ROS 参数快照；predictor 和 tracker 各自读取，不共享同一个实例。
struct DynamicObjectPredictorParams {
  double prediction_horizon{1.0};
  int max_branches{6};                         // >=1，限制每个目标的 GMM component 数量。
  double min_branch_probability{0.03};         // [0,1)，过滤极低概率分支后重新归一化。
  int history_window{10};                      // >=1，历史特征估计使用的最近样本数。
  double heading_stable_angle{0.35};           // >0，单位 rad，航向变化小于该值时认为趋势稳定。
  double stop_speed{0.2};                      // >=0，停止分支趋近的速度尺度。
  double turn_yaw_rate{0.5};                   // >0，转向假设的角速度幅值。
  double lateral_speed{0.4};                   // >=0，人类横向避让分支的法向速度幅值。
  double vertical_speed{0.3};                  // >=0，UAV 垂向分支的 z 速度幅值。
  double history_quality_noise_scale{1.5};     // >=1，短历史/漏检/不稳定轨迹的未来噪声放大上限。
  double hypothesis_noise_scale{1.3};          // >=1，假设性更强 mode 的未来噪声放大尺度。
  MultiModelKalmanFilterParams filter;
};

// 运行时配置；构造阶段由 ROS 参数快照转换而来。
struct DynamicObjectPredictorConfig {
  double prediction_horizon{1.0};
  std::size_t max_branches{6U};
  double min_branch_probability{0.03};
  std::size_t history_window{10U};
  double heading_stable_angle{0.35};
  double stop_speed{0.2};
  double turn_yaw_rate{0.5};
  double lateral_speed{0.4};
  double vertical_speed{0.3};
  double history_quality_noise_scale{1.5};
  double hypothesis_noise_scale{1.3};
  MultiModelKalmanFilterConfig filter_config;
};
```

In `src/dynamic_object_predictor.cpp`, update `buildPredictorConfig()`:

```cpp
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
  return config;
}
```

In `DynamicObjectPredictor::loadParameters()`, add these parameter reads after `prediction_horizon`:

```cpp
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
```

In `config/ldop.yaml`, add these defaults near `prediction_horizon`:

```yaml
prediction_max_branches: 3  # 每个目标最多保留的 GMM 分支数量，分支内只有一条中心状态轨迹
prediction_min_branch_probability: 0.05  # 低于该概率的 GMM 分支过滤后重新归一化
prediction_history_window: 20  # 航向和速度趋势估计使用的最近历史样本数
prediction_heading_stable_angle: 0.35  # 航向稳定阈值，单位 rad
prediction_stop_speed: 0.2  # 停止意图趋近的速度尺度
prediction_turn_yaw_rate: 0.5  # 左右转意图使用的角速度幅值
prediction_lateral_speed: 0.4  # 横向避让意图使用的法向速度幅值
prediction_vertical_speed: 0.3  # UAV 垂向意图使用的 z 速度幅值
prediction_history_quality_noise_scale: 1.5  # 短历史/漏检/不稳定轨迹的未来噪声放大上限
prediction_hypothesis_noise_scale: 1.3  # 假设性分支相对 keep 分支的未来噪声尺度
```

These two pruning parameters operate on GMM branches/components. They do not control the number of time samples in `DynamicObjectPredictionBranch.points`, and they do not imply multiple raw trajectories inside one branch. The predictor first converts candidate mode scores to probabilities, removes branches below `prediction_min_branch_probability`, keeps at most `prediction_max_branches` highest-probability branches, and then renormalizes the retained probabilities.

- [x] **Step 2: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: existing predictor tests pass. P2 does not provide an old single-branch disable switch.

---

## Task 2: Add History Feature Extraction And Basic Human GMM Branches

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add failing test helpers and human branch test**

Add these helpers in `test/test_dynamic_object_predictor.cpp` inside the anonymous namespace:

```cpp
TrackHistorySample makeHistorySample(const double stamp,
                                     const MotionModelType model_type,
                                     const Eigen::VectorXd& state,
                                     const bool matched = true) {
  TrackHistorySample sample;
  sample.stamp = ros::Time(stamp);
  sample.object_class = ObjectClass::Human;
  sample.motion_model_type = model_type;
  sample.model_state = state;
  sample.model_covariance = 0.1 * Eigen::MatrixXd::Identity(state.size(), state.size());
  sample.matched = matched;
  return sample;
}

void appendCvLinearHistory(TrackPredictionInput& input,
                           const double start_time,
                           const int count,
                           const double vx,
                           const double vy,
                           const double vz = 0.0) {
  input.history.clear();
  for (int index = 0; index < count; ++index) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
    const double t = start_time + static_cast<double>(index) * 0.1;
    state(0) = static_cast<double>(index) * 0.1 * vx;
    state(1) = static_cast<double>(index) * 0.1 * vy;
    state(2) = static_cast<double>(index) * 0.1 * vz;
    state(3) = vx;
    state(4) = vy;
    state(5) = vz;
    input.history.push_back(makeHistorySample(t, MotionModelType::CV3D, state));
  }
}

double probabilitySum(const ldop::DynamicObjectPrediction& prediction) {
  double sum = 0.0;
  for (const auto& branch : prediction.branches) {
    sum += branch.probability;
  }
  return sum;
}

bool hasBehavior(const ldop::DynamicObjectPrediction& prediction,
                 const std::uint8_t behavior_type) {
  return std::any_of(prediction.branches.begin(),
                     prediction.branches.end(),
                     [&](const auto& branch) { return branch.behavior_type == behavior_type; });
}
```

Add this test:

```cpp
TEST(DynamicObjectPredictorTest, GeneratesHumanIntentBranchesWithNormalizedProbabilities) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 20.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(21.0);
  header.frame_id = "map";
  const DynamicObjectPredictorFrameResult result =
      predictor.predict(header, std::vector<TrackPredictionInput>{input});

  ASSERT_EQ(result.predictions_msg.predictions.size(), 1U);
  const auto& prediction = result.predictions_msg.predictions.front();
  EXPECT_GT(prediction.branches.size(), 1U);
  EXPECT_LE(prediction.branches.size(), config.max_branches);
  EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
  EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL));
  EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING));
  EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL));
  for (const auto& branch : prediction.branches) {
    EXPECT_GT(branch.probability, 0.0);
    EXPECT_EQ(branch.points.size(), 3U);
  }
}
```

- [x] **Step 2: Run test command and verify it fails**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: the new test fails because predictor still outputs a single base trajectory instead of class-aware intent branches.

- [x] **Step 3: Add private structs and feature extraction**

In `src/dynamic_object_predictor.cpp`, add these includes:

```cpp
#include <numeric>
```

Inside the anonymous namespace, add:

```cpp
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

struct PredictionMode {
  PredictionModeKind kind{PredictionModeKind::Keep};
  std::uint8_t behavior_type{ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL};
  double behavior_value{0.0};
  double score{0.0};
  double probability{0.0};
  double mode_uncertainty_scale{1.0};
};

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

Eigen::Vector3d normalizedHorizontal(const Eigen::Vector3d& vector) {
  Eigen::Vector3d horizontal(vector.x(), vector.y(), 0.0);
  const double norm = horizontal.norm();
  if (norm <= 1e-9) {
    return Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  return horizontal / norm;
}

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

    const Eigen::Vector3d previous_velocity =
        velocityFromState(input.history.back().motion_model_type, input.history.back().model_state);
    features.previous_speed = previous_velocity.head<2>().norm();
    features.speed_delta = features.speed - features.previous_speed;

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
```

- [x] **Step 4: Add basic mode generation and branch construction**

Add these helpers in the anonymous namespace:

```cpp
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

  const double kept_sum = std::accumulate(
      modes.begin(), modes.end(), 0.0, [](const double sum, const auto& mode) {
        return sum + mode.probability;
      });
  assert(kept_sum > 0.0);
  for (PredictionMode& mode : modes) {
    mode.probability /= kept_sum;
  }
}

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

  if (input.object_class == ObjectClass::Human) {
    modes.push_back(makeMode(PredictionModeKind::LongitudinalDecelerate,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
                             -0.45,
                             decel_score,
                             config.hypothesis_noise_scale));
    modes.push_back(makeMode(PredictionModeKind::LongitudinalStop,
                             ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
                             -1.0,
                             stop_score,
                             config.hypothesis_noise_scale));
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
  }

  normalizeAndPruneModes(modes, config);
  return modes;
}
```

Add this helper in the anonymous namespace. At this task it uses the same model propagation for every mode; Task 4 will add mode-specific state controls to the helper.

```cpp
ldop::DynamicObjectPredictionBranch rollOutMode(
    const TrackPredictionInput& input,
    const PredictionMode& mode,
    const DynamicObjectPredictorConfig& config) {
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

  for (double elapsed = config.filter_config.default_dt;
       elapsed <= config.prediction_horizon + kTimeEpsilon;
       elapsed += config.filter_config.default_dt) {
    const Eigen::VectorXd previous_state = state;
    const Eigen::MatrixXd transition = model->getTransitionF(previous_state);
    state = model->stateTransition(previous_state);
    model->normalizeYaw(state);
    const Eigen::MatrixXd process_noise = model->getProcessNoiseQ();
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
```

Refactor the current single-branch code inside `DynamicObjectPredictor::predict()` so the P2 path is always used. The predictor should call `estimateHistoryFeatures()`, `generatePredictionModes()`, then create one branch per mode with `rollOutMode()`; do not keep an old single-branch fallback path.

The enabled-path branch setup must be:

```cpp
    const MotionHistoryFeatures features = estimateHistoryFeatures(input, config_);
    const std::vector<PredictionMode> modes = generatePredictionModes(input, features, config_);

    for (const PredictionMode& mode : modes) {
      prediction.branches.push_back(rollOutMode(input, mode, config_));
    }
```

- [x] **Step 5: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all predictor tests pass, including the new human multi-branch probability test.

---

## Task 3: Complete Class-Aware Mode Generation

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add failing tests for vehicle, UAV, and unknown strategy**

Append:

```cpp
bool hasBranchWithBehaviorValue(const ldop::DynamicObjectPrediction& prediction,
                                const std::uint8_t behavior_type,
                                const double sign) {
  return std::any_of(prediction.branches.begin(),
                     prediction.branches.end(),
                     [&](const auto& branch) {
                       return branch.behavior_type == behavior_type &&
                              branch.behavior_value * sign > 0.0;
                     });
}

TEST(DynamicObjectPredictorTest, VehicleUsesTurningButNotLateralBranches) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 5U;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Vehicle;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 30.0, 8, 2.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(31.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();

  EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING));
  EXPECT_FALSE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL));
  EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
}

TEST(DynamicObjectPredictorTest, UavUsesVerticalBranchesWhenVerticalEvidenceExists) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Uav;
  input.model_state(5) = 0.5;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 40.0, 8, 1.0, 0.0, 0.5);

  std_msgs::Header header;
  header.stamp = ros::Time(41.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();

  EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL));
  EXPECT_TRUE(hasBranchWithBehaviorValue(
      prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL, 1.0));
  EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
}

TEST(DynamicObjectPredictorTest, UnknownClassKeepsConservativeBranchSet) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Unknown;
  input.hits = 3U;
  input.age = 3U;
  appendCvLinearHistory(input, 50.0, 3, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(51.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();

  EXPECT_GE(prediction.branches.size(), 1U);
  EXPECT_LE(prediction.branches.size(), 3U);
  EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL));
  EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
}
```

- [x] **Step 2: Run test command and verify failures**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: vehicle/UAV/unknown tests fail because only human strategy exists.

- [x] **Step 3: Extend `generatePredictionModes()`**

Replace the class-specific block in `generatePredictionModes()` with:

```cpp
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
      if (!stable_heading || slowing) {
        add_deceleration();
      }
      if (!stable_heading && config.max_branches > 2U) {
        add_turn_pair();
      }
      break;
  }
```

- [x] **Step 4: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: class strategy tests pass.

---

## Task 4: Implement Mode-Specific Rollout For CV, CA, And CTRA

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add failing rollout divergence tests**

Append:

```cpp
const ldop::DynamicObjectPredictionBranch* findBranch(
    const ldop::DynamicObjectPrediction& prediction,
    const std::uint8_t behavior_type,
    const double sign) {
  for (const auto& branch : prediction.branches) {
    if (branch.behavior_type == behavior_type && branch.behavior_value * sign > 0.0) {
      return &branch;
    }
  }
  return nullptr;
}

TrackPredictionInput makeCtraInput() {
  TrackPredictionInput input;
  input.id = 84U;
  input.object_class = ObjectClass::Vehicle;
  input.motion_model_type = MotionModelType::CTRA;
  input.bbox.size.x = 2.0;
  input.bbox.size.y = 1.0;
  input.bbox.size.z = 1.2;
  input.model_state = Eigen::VectorXd::Zero(7);
  input.model_state(3) = 2.0;
  input.model_state(5) = 0.0;
  input.model_covariance = 0.1 * Eigen::MatrixXd::Identity(7, 7);
  input.matched_in_current_frame = true;
  input.hits = 8U;
  input.age = 8U;
  return input;
}

TEST(DynamicObjectPredictorTest, LateralBranchesDivergeInOppositeYDirections) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 60.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(61.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();
  const auto* left = findBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL, 1.0);
  const auto* right = findBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL, -1.0);

  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  ASSERT_FALSE(left->points.empty());
  ASSERT_FALSE(right->points.empty());
  EXPECT_GT(left->points.back().model_state[1], 0.0);
  EXPECT_LT(right->points.back().model_state[1], 0.0);
}

TEST(DynamicObjectPredictorTest, CtraTurningBranchesDivergeInYaw) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCtraInput();

  std_msgs::Header header;
  header.stamp = ros::Time(71.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();
  const auto* left = findBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING, 1.0);
  const auto* right = findBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING, -1.0);

  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  EXPECT_GT(left->points.back().model_state[5], 0.0);
  EXPECT_LT(right->points.back().model_state[5], 0.0);
}

TEST(DynamicObjectPredictorTest, UavVerticalBranchesChangePredictedZ) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Uav;
  input.model_state(5) = 0.1;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 80.0, 8, 1.0, 0.0, 0.1);

  std_msgs::Header header;
  header.stamp = ros::Time(81.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();
  const auto* climb = findBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL, 1.0);
  const auto* descend = findBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL, -1.0);

  ASSERT_NE(climb, nullptr);
  ASSERT_NE(descend, nullptr);
  EXPECT_GT(climb->points.back().model_state[2], descend->points.back().model_state[2]);
}
```

- [x] **Step 2: Run test command and verify failures**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: new tests fail because branches still reuse identical base rollout.

- [x] **Step 3: Add state control helpers**

In `src/dynamic_object_predictor.cpp`, add:

```cpp
Eigen::Vector2d rotate2D(const Eigen::Vector2d& vector, const double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return Eigen::Vector2d(c * vector.x() - s * vector.y(),
                         s * vector.x() + c * vector.y());
}

Eigen::Vector2d horizontalNormalLeft(const Eigen::Vector3d& heading) {
  return Eigen::Vector2d(-heading.y(), heading.x());
}

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
      const double next_speed = std::max(config.stop_speed, speed - config.stop_speed * dt);
      horizontal_velocity = speed > 1e-9 ? horizontal_velocity.normalized() * next_speed
                                         : heading * next_speed;
      setHorizontalVelocity(model_type, state, horizontal_velocity);
      break;
    }
    case PredictionModeKind::LongitudinalStop: {
      const double speed = horizontal_velocity.norm();
      const double next_speed = std::max(0.0, speed - std::max(speed, config.stop_speed) * dt);
      horizontal_velocity = speed > 1e-9 ? horizontal_velocity.normalized() * next_speed
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
```

- [x] **Step 4: Replace rollout helper with mode-aware version**

Replace the `rollOutMode()` helper added in Task 2 with:

```cpp
ldop::DynamicObjectPredictionBranch rollOutMode(
    const TrackPredictionInput& input,
    const PredictionMode& mode,
    const MotionHistoryFeatures& features,
    const DynamicObjectPredictorConfig& config) {
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

  for (double elapsed = config.filter_config.default_dt;
       elapsed <= config.prediction_horizon + kTimeEpsilon;
       elapsed += config.filter_config.default_dt) {
    applyModeControl(mode, features, config, config.filter_config.default_dt, input.motion_model_type, state);
    const Eigen::VectorXd previous_state = state;
    const Eigen::MatrixXd transition = model->getTransitionF(previous_state);
    state = model->stateTransition(previous_state);
    model->normalizeYaw(state);
    const Eigen::MatrixXd process_noise = model->getProcessNoiseQ();
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
```

In the GMM-enabled branch path inside `predict()`, replace the inline rollout loop with:

```cpp
    for (const PredictionMode& mode : modes) {
      prediction.branches.push_back(rollOutMode(input, mode, features, config_));
    }
```

- [x] **Step 5: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: rollout divergence tests pass.

---

## Task 5: Apply Future Process Noise Scaling

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add failing covariance scaling tests**

Append:

```cpp
double positionVarianceTrace(const ldop::DynamicObjectPredictionPoint& point) {
  const std::size_t state_size = point.model_state.size();
  return point.model_covariance[0U * state_size + 0U] +
         point.model_covariance[1U * state_size + 1U] +
         point.model_covariance[2U * state_size + 2U];
}

const ldop::DynamicObjectPredictionBranch* firstBranch(
    const ldop::DynamicObjectPrediction& prediction,
    const std::uint8_t behavior_type) {
  for (const auto& branch : prediction.branches) {
    if (branch.behavior_type == behavior_type) {
      return &branch;
    }
  }
  return nullptr;
}

TEST(DynamicObjectPredictorTest, HypothesisBranchesCarryLargerFutureCovarianceThanKeep) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.hypothesis_noise_scale = 2.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 90.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(91.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();
  const auto* keep = firstBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL);
  const auto* lateral = firstBranch(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL);

  ASSERT_NE(keep, nullptr);
  ASSERT_NE(lateral, nullptr);
  ASSERT_FALSE(keep->points.empty());
  ASSERT_FALSE(lateral->points.empty());
  EXPECT_GT(positionVarianceTrace(lateral->points.back()),
            positionVarianceTrace(keep->points.back()));
}

TEST(DynamicObjectPredictorTest, CoastingShortHistoryIncreasesFutureCovariance) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.history_quality_noise_scale = 2.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput stable = makeCvInput();
  stable.object_class = ObjectClass::Human;
  stable.hits = 10U;
  stable.age = 10U;
  appendCvLinearHistory(stable, 100.0, 10, 1.0, 0.0);

  TrackPredictionInput weak = makeCvInput();
  weak.object_class = ObjectClass::Human;
  weak.hits = 2U;
  weak.age = 5U;
  weak.missed_frames = 2U;
  appendCvLinearHistory(weak, 100.0, 2, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(101.0);
  header.frame_id = "map";
  const auto stable_result = predictor.predict(header, {stable});
  const auto weak_result = predictor.predict(header, {weak});

  const auto& stable_branch = stable_result.predictions_msg.predictions.front().branches.front();
  const auto& weak_branch = weak_result.predictions_msg.predictions.front().branches.front();
  EXPECT_GT(positionVarianceTrace(weak_branch.points.back()),
            positionVarianceTrace(stable_branch.points.back()));
}
```

- [x] **Step 2: Run test command and verify failures**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: covariance tests fail because `Q` is not scaled yet.

- [x] **Step 3: Scale future process noise in rollout**

In `rollOutMode()`, replace:

```cpp
    const Eigen::MatrixXd process_noise = model->getProcessNoiseQ();
```

with:

```cpp
    const double noise_scale = features.history_quality_scale * mode.mode_uncertainty_scale;
    const Eigen::MatrixXd process_noise = noise_scale * model->getProcessNoiseQ();
```

Add this Chinese comment immediately above the new lines:

```cpp
    // P0 已经携带 tracker 当前滤波不确定性；这里的尺度只表达未来意图假设和历史质量带来的额外不确定性。
```

- [x] **Step 4: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: covariance scaling tests pass.

---

## Task 6: Render Multi-Branch Mean Lines Without Covariance Ellipsoids

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add failing marker tests**

Append:

```cpp
std::size_t countMarkersByNamespace(const visualization_msgs::MarkerArray& markers,
                                    const std::string& marker_namespace) {
  return static_cast<std::size_t>(std::count_if(
      markers.markers.begin(),
      markers.markers.end(),
      [&](const auto& marker) { return marker.ns == marker_namespace; }));
}

TEST(DynamicObjectPredictorTest, BuildsLineMarkerForEachPredictionBranch) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 110.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(111.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();

  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg, "dynamic_prediction_line"),
            prediction.branches.size());
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg, "dynamic_prediction_samples"), 0U);
}

TEST(DynamicObjectPredictorTest, NoUncertaintyEllipsoidsWhenIntentGmmEnabled) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 120.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(121.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});

  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg,
                                    "dynamic_prediction_uncertainty"),
            0U);
}
```

- [x] **Step 2: Run test command and verify failures**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: first marker test fails because current marker code uses only `branches.front()`.

- [x] **Step 3: Add marker helpers**

In `src/dynamic_object_predictor.cpp`, add branch color helper only. Do not add MAP-branch covariance helper or ellipsoid orientation helper because P2 visualization intentionally omits covariance ellipsoid markers.

```cpp
void applyBranchColor(const ldop::DynamicObjectPredictionBranch& branch,
                      const double target_alpha,
                      const double min_alpha,
                      visualization_msgs::Marker& marker) {
  marker.color.a = static_cast<float>(
      std::clamp(target_alpha * (0.7 + 0.3 * branch.probability), min_alpha, 1.0));
  switch (branch.behavior_type) {
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL:
      marker.color.r = 0.1F;
      marker.color.g = 0.7F;
      marker.color.b = 1.0F;
      break;
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING:
      marker.color.r = 1.0F;
      marker.color.g = 0.7F;
      marker.color.b = 0.1F;
      break;
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL:
      marker.color.r = 0.2F;
      marker.color.g = 1.0F;
      marker.color.b = 0.35F;
      break;
    case ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL:
      marker.color.r = 0.7F;
      marker.color.g = 0.35F;
      marker.color.b = 1.0F;
      break;
    default:
      marker.color.r = 0.1F;
      marker.color.g = 0.7F;
      marker.color.b = 1.0F;
      break;
  }
}
```

- [x] **Step 4: Update `buildPredictionMarkers()`**

Replace the current line-marker section with a loop over all branches:

```cpp
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
      line.scale.x = 0.04;
      applyBranchColor(branch, target_alpha, 0.05, line);

      for (const auto& point : branch.points) {
        assert(point.model_state.size() >= 3U);
        line.points.push_back(makePoint(point.model_state[0],
                                        point.model_state[1],
                                        point.model_state[2]));
      }
      markers.markers.push_back(std::move(line));
    }
```

- [x] **Step 5: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: marker tests pass.

---

## Task 7: Keep RViz Limited To Branch Mean Lines

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Remove sample marker construction**

Delete the deterministic sample-marker helpers and their call from `buildPredictionMarkers()`. RViz should only receive `dynamic_prediction_line` markers, one per retained GMM branch.

- [x] **Step 2: Add regression coverage**

Keep a test that asserts `dynamic_prediction_samples` marker count is zero. This guards against reintroducing visual-only sampled trajectories that have no actual consumer and can look like extra prediction outputs.

- [x] **Step 3: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: predictor marker tests pass and no sample marker namespace is produced.

---

## Task 8: Fix P2 Regression Cases Found During Review

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Add failing regression tests**

Add tests covering:

- Historical speed drop should increase `longitudinal-decelerate` probability even when `input.model_state` equals `history.back().model_state`.
- `longitudinal-decelerate` must not raise a target whose current speed is below `prediction_stop_speed`.
- `ObjectClass::Unknown` and `ObjectClass::Other` should generate vertical branches when current z velocity provides clear vertical evidence.

- [x] **Step 2: Verify tests fail before fixes**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: the three new regression tests fail for the unfixed implementation.

- [x] **Step 3: Fix root causes**

Implementation requirements:

- Compute speed trend against the start of the configured history window, not `history.back()`, because tracker passes the current filter state both as `input.model_state` and as the last history sample.
- Treat `prediction_stop_speed` as a deceleration scale in `longitudinal-decelerate`; do not clamp low-speed targets upward to that value.
- Add conservative vertical candidates for `Other` and `Unknown` only when `std::abs(vertical_speed)` exceeds the vertical evidence threshold.

- [x] **Step 4: Run verification**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all predictor tests pass, including the new regression tests.

---

## Task 9: Final Verification And Review

**Files:**
- Verify: all modified files

- [x] **Step 1: Run full package build**

Run:

```bash
catkin build ldop --no-status
```

Expected: build completes successfully.

- [x] **Step 2: Run package tests**

Run:

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all `ldop` tests pass and `catkin_test_results` reports no failures.

- [x] **Step 3: Run whitespace check**

Run:

```bash
git diff --check
```

Expected: no output.

- [x] **Step 4: Inspect final diff**

Run:

```bash
git diff -- include/ldop/dynamic_object_predictor.h src/dynamic_object_predictor.cpp test/test_dynamic_object_predictor.cpp config/ldop.yaml docs/superpowers/specs/2026-05-18-dynamic-object-intent-gmm-p2-design.md docs/superpowers/plans/2026-05-18-dynamic-object-intent-gmm-p2-implementation-plan.md
```

Expected: diff only touches P2 predictor params, P2 predictor implementation, P2 predictor tests, P2 YAML defaults, branch-message enum cleanup, RViz namespace sync, test-link ownership, and the P2 design/plan updates.

- [x] **Step 5: Report completion and ask before commit**

Do not commit automatically. Report:

```text
P2 intent GMM implementation complete.
Verified with:
- catkin build ldop --no-status
- catkin run_tests ldop --no-status
- catkin_test_results /home/st/ldop_ws/build/ldop
- git diff --check

No new message files or sampled-trajectory fields were added; the existing branch message enum was cleaned up by removing `BEHAVIOR_BASELINE`. Do you want me to commit these changes?
```

---

## Self-Review

- Spec coverage: tasks cover stable planner-facing branch GMM, history features, class-aware mode generation, probability normalization, per-mode rollout, process-noise scaling, branch mean-line visualization, config defaults, regression tests, and verification.
- Scope boundary: P2 only cleans up the existing prediction-branch enum; it does not modify `DynamicObject.msg`, add sampled-trajectory messages, change tracker semantics, or implement P3 interaction energy, map interaction, or P4 NIS feedback.
- Type consistency: plan uses `DynamicObjectPredictorParams`, `DynamicObjectPredictorConfig`, `MotionHistoryFeatures`, `PredictionMode`, `PredictionModeKind`, `TrackPredictionInput`, `DynamicObjectPredictionBranch`, and existing message fields consistently.
- Verification: each implementation task starts with a failing test and ends with test/result checks. Final verification matches `AGENTS.md` risk guidance for behavior changes.
