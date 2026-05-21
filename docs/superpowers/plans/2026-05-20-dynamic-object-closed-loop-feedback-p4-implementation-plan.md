# Dynamic Object Closed-Loop Feedback P4 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build LDOP trajectory-prediction P4 closed-loop feedback so each tracked object can use the previous prediction residual/NIS to adjust its next-frame branch probability prior and predictor future process-noise scale.

**Architecture:** Keep P4 inside `DynamicObjectPredictor`: it owns a short-lived per-object feedback cache, evaluates the previous prediction against the current tracker observation at the start of `predict()`, applies feedback hints to P2 mode scores and rollout noise scales, then snapshots the final prediction for the next frame. Do not add ROS messages or planner-facing fields; expose only C++ diagnostics and verbose logs.

**Tech Stack:** ROS1 Noetic, catkin, C++20, Eigen, GoogleTest, LDOP prediction messages.

---

## Scope Check

This plan implements one subsystem: P4 closed-loop feedback inside the existing trajectory predictor. It does not implement a planner-facing dynamic risk map, a class/global calibration model, learning-based calibration, or a diagnostic ROS topic.

Repository rule override: the general planning skill suggests frequent commits, but this repository explicitly says not to create intermediate commits during a task. Execute the steps without committing; after verification, ask the user before creating any commit.

## Completion Status

Status: complete as of 2026-05-21 local verification. The implemented scope is the predictor-internal online feedback loop: previous prediction snapshots are matched by `object_id` and behavior key, residual/NIS/log-likelihood update next-frame mode score bias and future `Q` scale, P3 interaction still runs after feedback, and no ROS prediction message or planner-facing risk-map contract was added. This also closes the LDOP stage-3 trajectory-prediction scope; offline multi-horizon evaluation and LDOP stage-4 risk-map consumption remain follow-up work. Verified with `git diff --cached --check`, `catkin build ldop --no-status`, `catkin run_tests ldop --no-status`, and `catkin_test_results /home/st/ldop_ws/build/ldop` reporting `Summary: 106 tests, 0 errors, 0 failures, 0 skipped`.

## Design Anchors

- Keep `/ldop/dynamic_object_predictions` and `/ldop/dynamic_prediction_markers` message structure unchanged.
- Keep feedback scoped to the same `object_id`; do not share feedback across classes or targets.
- Feedback can raise or lower branch probability through P2 mode score bias.
- Feedback can scale predictor future `Q_future` within `[0.3, prediction_feedback_max_noise_scale]`.
- `process_noise_scale = 1.0` means the predictor uses its normal future `Q_future`; `0.3` is a fixed algorithmic floor and is not a YAML parameter.
- P4 must not alter tracker `model_state`, tracker `model_covariance`, or `DynamicObject.msg`.
- `DynamicObjectPredictor::predict()` becomes non-`const` because it reads and updates the feedback cache.
- P3 interaction still runs after P2 mode scoring and P4 feedback bias.

## File Structure

- Modify `include/ldop/dynamic_object_predictor.h`
  - Add `PredictionFeedbackParams`, `PredictionFeedbackConfig`, and `PredictionFeedbackDiagnostics`.
  - Add feedback config and diagnostics to existing predictor params/config/frame result.
  - Change `DynamicObjectPredictor::predict()` from `const` to non-`const`.
  - Add private per-object feedback cache types and `feedback_states_`.

- Modify `src/dynamic_object_predictor.cpp`
  - Load and build feedback config.
  - Add branch-key, snapshot, covariance extraction, NIS/log-likelihood, smoothing, cache cleanup, and diagnostics helpers.
  - Apply feedback bias before P2 softmax pruning.
  - Apply `process_noise_scale` during rollout future process-noise propagation.
  - Snapshot final predictions after P3 interaction has produced the planner-facing branches.

- Modify `test/test_dynamic_object_predictor.cpp`
  - Add closed-loop tests for probability increase, noise increase, noise decrease to fixed 0.3 floor, disabled behavior, unmatched skip, expiry cleanup, branch semantic matching, and P3 ordering.

- Modify `config/ldop.yaml`
  - Add P4 feedback parameters beside existing `prediction_*` parameters, without adding a min-noise-scale parameter.

- Modify planning docs after code passes:
  - `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`
  - `task_plan.md`
  - `progress.md`
  - `findings.md`

## Task 1: Add Feedback Config, Diagnostics, and Non-Const Predictor Entry

**Files:**
- Modify: `include/ldop/dynamic_object_predictor.h`
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `config/ldop.yaml`
- Test: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Write a disabled-feedback equivalence test**

Add this test near the existing predictor behavior tests in `test/test_dynamic_object_predictor.cpp`:

```cpp
TEST(DynamicObjectPredictorTest, FeedbackDisabledMatchesFreshPredictorAcrossFrames) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 6U;
  config.feedback_config.enabled = false;
  config.interaction_config.enabled = false;

  DynamicObjectPredictor cached_predictor(config);
  DynamicObjectPredictor fresh_predictor(config);

  TrackPredictionInput first = makeCvInput();
  first.id = 501U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 300.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(301.0);
  first_header.frame_id = "map";
  static_cast<void>(cached_predictor.predict(first_header, {first}));

  TrackPredictionInput second = first;
  second.stamp = ros::Time(301.1);
  second.model_state(0) += 0.1;
  for (auto& sample : second.history) {
    sample.model_state(0) += 0.1;
    sample.stamp += ros::Duration(0.1);
  }

  std_msgs::Header second_header;
  second_header.stamp = ros::Time(301.1);
  second_header.frame_id = "map";
  const auto cached = cached_predictor.predict(second_header, {second});
  const auto fresh = fresh_predictor.predict(second_header, {second});

  ASSERT_EQ(cached.predictions_msg.predictions.size(),
            fresh.predictions_msg.predictions.size());
  const auto& cached_prediction = cached.predictions_msg.predictions.front();
  const auto& fresh_prediction = fresh.predictions_msg.predictions.front();
  ASSERT_EQ(cached_prediction.branches.size(), fresh_prediction.branches.size());
  for (std::size_t branch_index = 0U;
       branch_index < cached_prediction.branches.size();
       ++branch_index) {
    const auto& cached_branch = cached_prediction.branches[branch_index];
    const auto& fresh_branch = fresh_prediction.branches[branch_index];
    EXPECT_EQ(cached_branch.behavior_type, fresh_branch.behavior_type);
    EXPECT_NEAR(cached_branch.behavior_value, fresh_branch.behavior_value, 1e-12);
    EXPECT_NEAR(cached_branch.probability, fresh_branch.probability, 1e-12);
    ASSERT_EQ(cached_branch.points.size(), fresh_branch.points.size());
    EXPECT_NEAR(positionVarianceTrace(cached_branch.points.back()),
                positionVarianceTrace(fresh_branch.points.back()),
                1e-12);
  }
  EXPECT_EQ(cached.feedback_diagnostics.feedback_updated_branch_count, 0U);
}
```

- [x] **Step 2: Run the focused test and verify it fails to compile**

Run from `/home/st/ldop_ws`:

```bash
catkin build ldop --no-status
```

Expected: compile fails because `DynamicObjectPredictorConfig::feedback_config` and `DynamicObjectPredictorFrameResult::feedback_diagnostics` do not exist yet.

- [x] **Step 3: Add feedback params/config/diagnostics declarations**

In `include/ldop/dynamic_object_predictor.h`, add `<unordered_map>` and the feedback structs near the existing predictor params/config structs:

```cpp
#include <unordered_map>
```

```cpp
struct PredictionFeedbackParams {
  bool enabled{true};                         // P4 主开关；关闭后保持 P3 行为。
  double max_age{1.5};                        // 超过该时间差的缓存不再用于反馈，单位 s。
  double probability_gain{1.0};               // likelihood bias 对下一轮 mode score 的影响。
  double noise_gain{0.25};                    // NIS 偏离期望值时过程噪声尺度的调整速度。
  double max_noise_scale{3.0};                // predictor 未来 Q 的最大闭环放大尺度。
  double smoothing_alpha{0.35};               // NIS/likelihood 的指数平滑权重。
  double behavior_value_bucket{0.25};         // 按行为强度近似匹配分支语义。
  double max_sample_time_error{0.15};         // 预测采样点与当前 dt 的最大允许差，单位 s。
};

struct PredictionFeedbackConfig {
  bool enabled{true};
  double max_age{1.5};
  double probability_gain{1.0};
  double noise_gain{0.25};
  double max_noise_scale{3.0};
  double smoothing_alpha{0.35};
  double behavior_value_bucket{0.25};
  double max_sample_time_error{0.15};
};

struct PredictionFeedbackDiagnostics {
  std::size_t feedback_evaluated_object_count{0U};
  std::size_t feedback_updated_branch_count{0U};
  std::size_t feedback_skipped_object_count{0U};
  std::size_t feedback_skipped_branch_count{0U};
  std::size_t expired_state_count{0U};
  double max_nis{0.0};
  double mean_nis{0.0};
  double min_process_noise_scale{1.0};
  double max_process_noise_scale{1.0};
  double max_abs_probability_bias{0.0};
};
```

Add fields:

```cpp
struct DynamicObjectPredictorParams {
  ...
  PredictionFeedbackParams feedback;
  MultiModelKalmanFilterParams filter;
  PredictionInteractionParams interaction;
};

struct DynamicObjectPredictorConfig {
  ...
  PredictionFeedbackConfig feedback_config;
  MultiModelKalmanFilterConfig filter_config;
  PredictionInteractionConfig interaction_config;
};

struct DynamicObjectPredictorFrameResult {
  ...
  PredictionFeedbackDiagnostics feedback_diagnostics;
  DynamicObjectPredictorTimingStats timing;
};
```

- [x] **Step 4: Add predictor-internal feedback cache declarations**

In `include/ldop/dynamic_object_predictor.h`, add these namespace-scope types after `PredictionFeedbackDiagnostics`. They are C++ predictor internals, not ROS message contracts:

```cpp
struct PredictionFeedbackBranchKey {
  std::uint8_t behavior_type{ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL};
  int behavior_value_bucket{0};

  bool operator==(const PredictionFeedbackBranchKey& other) const {
    return behavior_type == other.behavior_type &&
           behavior_value_bucket == other.behavior_value_bucket;
  }
};

struct PredictionFeedbackBranchKeyHash {
  std::size_t operator()(const PredictionFeedbackBranchKey& key) const {
    return (static_cast<std::size_t>(key.behavior_type) << 32U) ^
           static_cast<std::size_t>(key.behavior_value_bucket);
  }
};

struct PredictionFeedbackSampleSnapshot {
  double time_from_start{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d position_covariance{Eigen::Matrix3d::Identity()};
};

struct PredictionFeedbackBranchSnapshot {
  PredictionFeedbackBranchKey key;
  double probability{0.0};
  std::vector<PredictionFeedbackSampleSnapshot> samples;
};

struct PredictionFeedbackBranchState {
  double smoothed_log_likelihood{0.0};
  double smoothed_nis{3.0};
  double probability_score_bias{0.0};
  double process_noise_scale{1.0};
  ros::Time last_update_stamp;
  bool initialized{false};
};

struct PredictionFeedbackObjectState {
  ros::Time last_stamp;
  std::vector<PredictionFeedbackBranchSnapshot> last_prediction;
  std::unordered_map<PredictionFeedbackBranchKey,
                     PredictionFeedbackBranchState,
                     PredictionFeedbackBranchKeyHash>
      branch_states;
};
```

Inside `class DynamicObjectPredictor` private section, add:

```cpp
  std::unordered_map<std::uint32_t, PredictionFeedbackObjectState> feedback_states_;
```

- [x] **Step 5: Change `predict()` to non-const**

In `include/ldop/dynamic_object_predictor.h`, change:

```cpp
  DynamicObjectPredictorFrameResult predict(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      const PredictionMapQuery* map_query = nullptr);
```

In `src/dynamic_object_predictor.cpp`, update the definition to match:

```cpp
DynamicObjectPredictorFrameResult DynamicObjectPredictor::predict(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs,
    const PredictionMapQuery* map_query) {
```

No change is needed in `Ldop::processFrame()` because the predictor is held through a non-const `std::unique_ptr`.

- [x] **Step 6: Build feedback config from params**

In `src/dynamic_object_predictor.cpp`, add this helper near `buildPredictorConfig()`:

```cpp
PredictionFeedbackConfig buildPredictionFeedbackConfig(
    const PredictionFeedbackParams& params) {
  const PredictionFeedbackParams defaults;
  PredictionFeedbackConfig config;
  config.enabled = params.enabled;
  config.max_age = params.max_age > 0.0 ? params.max_age : defaults.max_age;
  config.probability_gain =
      params.probability_gain >= 0.0 ? params.probability_gain : defaults.probability_gain;
  config.noise_gain = params.noise_gain >= 0.0 ? params.noise_gain : defaults.noise_gain;
  config.max_noise_scale =
      params.max_noise_scale >= 1.0 ? params.max_noise_scale : defaults.max_noise_scale;
  config.smoothing_alpha =
      (params.smoothing_alpha > 0.0 && params.smoothing_alpha <= 1.0)
          ? params.smoothing_alpha
          : defaults.smoothing_alpha;
  config.behavior_value_bucket =
      params.behavior_value_bucket > 0.0 ? params.behavior_value_bucket
                                         : defaults.behavior_value_bucket;
  config.max_sample_time_error =
      params.max_sample_time_error > 0.0 ? params.max_sample_time_error
                                         : defaults.max_sample_time_error;
  return config;
}
```

Inside `buildPredictorConfig()`, add:

```cpp
  config.feedback_config = buildPredictionFeedbackConfig(params.feedback);
```

- [x] **Step 7: Load ROS feedback parameters**

In `DynamicObjectPredictor::loadParameters()`, after the P2 predictor params and before interaction params, add:

```cpp
  pnh_.param("prediction_enable_feedback",
             params_.feedback.enabled,
             defaults.feedback.enabled);
  pnh_.param("prediction_feedback_max_age",
             params_.feedback.max_age,
             defaults.feedback.max_age);
  pnh_.param("prediction_feedback_probability_gain",
             params_.feedback.probability_gain,
             defaults.feedback.probability_gain);
  pnh_.param("prediction_feedback_noise_gain",
             params_.feedback.noise_gain,
             defaults.feedback.noise_gain);
  pnh_.param("prediction_feedback_max_noise_scale",
             params_.feedback.max_noise_scale,
             defaults.feedback.max_noise_scale);
  pnh_.param("prediction_feedback_smoothing_alpha",
             params_.feedback.smoothing_alpha,
             defaults.feedback.smoothing_alpha);
  pnh_.param("prediction_feedback_behavior_value_bucket",
             params_.feedback.behavior_value_bucket,
             defaults.feedback.behavior_value_bucket);
  pnh_.param("prediction_feedback_max_sample_time_error",
             params_.feedback.max_sample_time_error,
             defaults.feedback.max_sample_time_error);
```

- [x] **Step 8: Add YAML parameters without a min-noise-scale parameter**

In `config/ldop.yaml`, add after `prediction_hypothesis_noise_scale`:

```yaml
prediction_enable_feedback: true  # 启用P4闭环反馈：用上一轮预测残差/NIS调整同一目标的下一帧分支概率和未来Q尺度
prediction_feedback_max_age: 1.5  # 超过该时间差的预测缓存不再用于反馈，单位s
prediction_feedback_probability_gain: 1.0  # likelihood bias对下一轮mode score的影响强度
prediction_feedback_noise_gain: 0.25  # NIS偏离期望值时未来过程噪声尺度的调整速度
prediction_feedback_max_noise_scale: 3.0  # predictor未来Q的最大闭环放大尺度；下界固定为0.3不暴露为参数
prediction_feedback_smoothing_alpha: 0.35  # NIS和likelihood反馈的指数平滑权重
prediction_feedback_behavior_value_bucket: 0.25  # 行为强度近似匹配粒度，用于跨帧匹配同语义分支
prediction_feedback_max_sample_time_error: 0.15  # 预测采样点与当前dt的最大允许时间偏差，单位s
```

- [x] **Step 9: Run the focused build check**

Run from `/home/st/ldop_ws`:

```bash
catkin build ldop --no-status
```

Expected: build succeeds, and the disabled-feedback test compiles. The test may still fail at runtime until Task 2 stores snapshots and diagnostics; keep the failure if it is limited to feedback counts.

## Task 2: Add Feedback Snapshot and Cache Lifecycle

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Test: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Write cache expiry and unmatched skip tests**

Add these tests near the disabled-feedback test:

```cpp
TEST(DynamicObjectPredictorTest, FeedbackStateExpiresAfterConfiguredMaxAge) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 1U;
  config.feedback_config.enabled = true;
  config.feedback_config.max_age = 0.2;
  config.interaction_config.enabled = false;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.id = 511U;
  input.object_class = ObjectClass::Other;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 310.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(311.0);
  first_header.frame_id = "map";
  static_cast<void>(predictor.predict(first_header, {input}));

  std_msgs::Header second_header;
  second_header.stamp = ros::Time(311.5);
  second_header.frame_id = "map";
  input.model_state(0) += 0.5;
  const auto result = predictor.predict(second_header, {input});

  EXPECT_EQ(result.feedback_diagnostics.feedback_updated_branch_count, 0U);
  EXPECT_GT(result.feedback_diagnostics.expired_state_count, 0U);
}

TEST(DynamicObjectPredictorTest, FeedbackSkipsUnmatchedCurrentObservation) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 1U;
  config.feedback_config.enabled = true;
  config.interaction_config.enabled = false;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.id = 512U;
  input.object_class = ObjectClass::Other;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 320.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(321.0);
  first_header.frame_id = "map";
  static_cast<void>(predictor.predict(first_header, {input}));

  TrackPredictionInput unmatched = input;
  unmatched.matched_in_current_frame = false;
  unmatched.model_state(0) += 0.1;
  std_msgs::Header second_header;
  second_header.stamp = ros::Time(321.1);
  second_header.frame_id = "map";
  const auto result = predictor.predict(second_header, {unmatched});

  EXPECT_EQ(result.feedback_diagnostics.feedback_updated_branch_count, 0U);
  EXPECT_GT(result.feedback_diagnostics.feedback_skipped_object_count, 0U);
}
```

- [x] **Step 2: Run tests and verify failure**

Run from `/home/st/ldop_ws`:

```bash
catkin run_tests ldop --no-status --make-args tests
```

Expected: `FeedbackStateExpiresAfterConfiguredMaxAge` and `FeedbackSkipsUnmatchedCurrentObservation` fail because predictor does not yet snapshot, expire, or count feedback state.

- [x] **Step 3: Add fixed lower bound and snapshot helpers**

In the anonymous namespace of `src/dynamic_object_predictor.cpp`, add near the other constants:

```cpp
constexpr double kMinFeedbackProcessNoiseScale = 0.3;
constexpr double kNisPositionDof = 3.0;
constexpr double kLogLikelihoodEpsilon = 1e-12;
```

Add helpers after `eigenMatrixToRowMajorStdVector()`:

```cpp
PredictionFeedbackBranchKey makeFeedbackBranchKey(
    const std::uint8_t behavior_type,
    const double behavior_value,
    const DynamicObjectPredictorConfig& config) {
  PredictionFeedbackBranchKey key;
  key.behavior_type = behavior_type;
  const double bucket_size = config.feedback_config.behavior_value_bucket;
  key.behavior_value_bucket =
      static_cast<int>(std::lround(behavior_value / bucket_size));
  return key;
}

std::optional<Eigen::Matrix3d> positionCovarianceFromPoint(
    const ldop::DynamicObjectPredictionPoint& point) {
  const std::size_t state_size = point.model_state.size();
  if (state_size < 3U || point.model_covariance.size() != state_size * state_size) {
    return std::nullopt;
  }
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = 0U; col < 3U; ++col) {
      covariance(static_cast<int>(row), static_cast<int>(col)) =
          point.model_covariance[row * state_size + col];
    }
  }
  return covariance;
}

std::optional<Eigen::Matrix3d> positionCovarianceFromInput(
    const TrackPredictionInput& input) {
  if (input.model_state.size() < 3 || input.model_covariance.rows() < 3 ||
      input.model_covariance.cols() < 3) {
    return std::nullopt;
  }
  return input.model_covariance.topLeftCorner<3, 3>();
}

std::optional<Eigen::Vector3d> positionFromInput(const TrackPredictionInput& input) {
  if (input.model_state.size() < 3) {
    return std::nullopt;
  }
  return input.model_state.segment<3>(0);
}
```

- [x] **Step 4: Add snapshot creation method**

Add this private method definition in `src/dynamic_object_predictor.cpp` after `buildPredictionMarkers()`:

```cpp
void DynamicObjectPredictor::storeFeedbackSnapshots(
    const std_msgs::Header& header,
    const ldop::DynamicObjectPredictionArray& predictions) {
  if (!config_.feedback_config.enabled) {
    return;
  }

  for (const auto& prediction : predictions.predictions) {
    PredictionFeedbackObjectState& state = feedback_states_[prediction.id];
    state.last_stamp = header.stamp;
    state.last_prediction.clear();
    state.last_prediction.reserve(prediction.branches.size());

    for (const auto& branch : prediction.branches) {
      PredictionFeedbackBranchSnapshot snapshot;
      snapshot.key = makeFeedbackBranchKey(
          branch.behavior_type, branch.behavior_value, config_);
      snapshot.probability = branch.probability;
      snapshot.samples.reserve(branch.points.size());

      for (const auto& point : branch.points) {
        if (point.model_state.size() < 3U) {
          continue;
        }
        const auto covariance = positionCovarianceFromPoint(point);
        if (!covariance.has_value()) {
          continue;
        }
        PredictionFeedbackSampleSnapshot sample;
        sample.time_from_start = point.time_from_start.toSec();
        sample.position = Eigen::Vector3d(
            point.model_state[0], point.model_state[1], point.model_state[2]);
        sample.position_covariance = *covariance;
        snapshot.samples.push_back(sample);
      }

      if (!snapshot.samples.empty()) {
        state.last_prediction.push_back(std::move(snapshot));
      }
    }
  }
}
```

Add declaration to the private section in `include/ldop/dynamic_object_predictor.h`:

```cpp
  void storeFeedbackSnapshots(
      const std_msgs::Header& header,
      const ldop::DynamicObjectPredictionArray& predictions);
```

- [x] **Step 5: Add cache cleanup method**

Add this private method definition in `src/dynamic_object_predictor.cpp`:

```cpp
void DynamicObjectPredictor::cleanupExpiredFeedbackStates(
    const ros::Time& stamp,
    const std::vector<TrackPredictionInput>& inputs,
    PredictionFeedbackDiagnostics& diagnostics) {
  std::vector<std::uint32_t> active_ids;
  active_ids.reserve(inputs.size());
  for (const auto& input : inputs) {
    active_ids.push_back(input.id);
  }

  for (auto iter = feedback_states_.begin(); iter != feedback_states_.end();) {
    const bool active =
        std::find(active_ids.begin(), active_ids.end(), iter->first) != active_ids.end();
    const double age = (stamp - iter->second.last_stamp).toSec();
    if (!active || age > config_.feedback_config.max_age) {
      iter = feedback_states_.erase(iter);
      ++diagnostics.expired_state_count;
      continue;
    }
    ++iter;
  }
}
```

Add declaration:

```cpp
  void cleanupExpiredFeedbackStates(
      const ros::Time& stamp,
      const std::vector<TrackPredictionInput>& inputs,
      PredictionFeedbackDiagnostics& diagnostics);
```

- [x] **Step 6: Call cleanup and snapshot from `predict()`**

At the start of `DynamicObjectPredictor::predict()` after result initialization, add:

```cpp
  if (config_.feedback_config.enabled) {
    cleanupExpiredFeedbackStates(header.stamp, inputs, result.feedback_diagnostics);
  }
```

At the end of `predict()`, after markers are built and before verbose logging, add:

```cpp
  storeFeedbackSnapshots(header, result.predictions_msg);
```

- [x] **Step 7: Count unmatched skips**

Add this temporary method so Task 2 tests pass before NIS is implemented:

```cpp
void DynamicObjectPredictor::countFeedbackEligibility(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs,
    PredictionFeedbackDiagnostics& diagnostics) const {
  if (!config_.feedback_config.enabled) {
    return;
  }
  for (const auto& input : inputs) {
    const auto state_iter = feedback_states_.find(input.id);
    if (state_iter == feedback_states_.end()) {
      continue;
    }
    const double age = (header.stamp - state_iter->second.last_stamp).toSec();
    if (age <= 0.0 || age > config_.feedback_config.max_age ||
        !input.matched_in_current_frame) {
      ++diagnostics.feedback_skipped_object_count;
    }
  }
}
```

Declare it in the header and call it after cleanup:

```cpp
    countFeedbackEligibility(header, inputs, result.feedback_diagnostics);
```

- [x] **Step 8: Run tests**

Run from `/home/st/ldop_ws`:

```bash
catkin run_tests ldop --no-status --make-args tests
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: new Task 2 tests pass. Other existing tests should remain green. If `catkin_test_results` reports zero tests immediately after `run_tests`, rerun it once after the test target finishes writing XML.

## Task 3: Compute NIS, Likelihood, and Feedback Hints

**Files:**
- Modify: `include/ldop/dynamic_object_predictor.h`
- Modify: `src/dynamic_object_predictor.cpp`
- Test: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Write probability-feedback test**

Add this test:

```cpp
double branchProbabilityOrZero(const ldop::DynamicObjectPrediction& prediction,
                               const std::uint8_t behavior_type,
                               const double behavior_value) {
  const auto* branch = findBranchWithBehaviorValue(prediction, behavior_type, behavior_value);
  return branch != nullptr ? branch->probability : 0.0;
}

TEST(DynamicObjectPredictorTest, FeedbackRaisesProbabilityForObservedBranch) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 8U;
  config.feedback_config.enabled = true;
  config.feedback_config.probability_gain = 2.0;
  config.feedback_config.noise_gain = 0.0;
  config.interaction_config.enabled = false;

  DynamicObjectPredictor feedback_predictor(config);

  DynamicObjectPredictorConfig baseline_config = config;
  baseline_config.feedback_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  TrackPredictionInput first = makeCvInput();
  first.id = 521U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 330.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(331.0);
  first_header.frame_id = "map";
  const auto first_result = feedback_predictor.predict(first_header, {first});
  const auto& first_prediction = first_result.predictions_msg.predictions.front();
  const auto* lateral_left = findBranchWithBehaviorValue(
      first_prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL, 1.0);
  ASSERT_NE(lateral_left, nullptr);
  ASSERT_FALSE(lateral_left->points.empty());

  TrackPredictionInput second = first;
  second.model_state(0) = lateral_left->points.front().model_state[0];
  second.model_state(1) = lateral_left->points.front().model_state[1];
  second.history.push_back(makeHistorySample(
      331.1, MotionModelType::CV3D, second.model_state));

  std_msgs::Header second_header;
  second_header.stamp = ros::Time(331.1);
  second_header.frame_id = "map";
  const auto feedback = feedback_predictor.predict(second_header, {second});
  const auto baseline = baseline_predictor.predict(second_header, {second});

  const double feedback_probability = branchProbabilityOrZero(
      feedback.predictions_msg.predictions.front(),
      ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL,
      1.0);
  const double baseline_probability = branchProbabilityOrZero(
      baseline.predictions_msg.predictions.front(),
      ldop::DynamicObjectPredictionBranch::BEHAVIOR_LATERAL,
      1.0);

  EXPECT_GT(feedback.feedback_diagnostics.feedback_updated_branch_count, 0U);
  EXPECT_GT(feedback_probability, baseline_probability);
}
```

- [x] **Step 2: Run test and verify failure**

Run:

```bash
catkin run_tests ldop --no-status --make-args tests
```

Expected: `FeedbackRaisesProbabilityForObservedBranch` fails because NIS/likelihood feedback is not computed or applied.

- [x] **Step 3: Add feedback mode hint type**

In the private section of `DynamicObjectPredictor` in the header, add:

```cpp
struct PredictionFeedbackModeHint {
  double score_bias{0.0};
  double process_noise_scale{1.0};
  bool has_feedback{false};
};

using PredictionFeedbackModeHintMap =
    std::unordered_map<PredictionFeedbackBranchKey,
                       PredictionFeedbackModeHint,
                       PredictionFeedbackBranchKeyHash>;
```

Declare:

```cpp
  std::unordered_map<std::uint32_t, PredictionFeedbackModeHintMap> buildFeedbackHints(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      PredictionFeedbackDiagnostics& diagnostics);
```

- [x] **Step 4: Add sample matching and log-likelihood helpers**

In `src/dynamic_object_predictor.cpp`, add:

```cpp
const PredictionFeedbackSampleSnapshot* closestFeedbackSample(
    const PredictionFeedbackBranchSnapshot& branch,
    const double dt,
    const DynamicObjectPredictorConfig& config) {
  const PredictionFeedbackSampleSnapshot* best = nullptr;
  double best_error = std::numeric_limits<double>::infinity();
  for (const auto& sample : branch.samples) {
    const double error = std::abs(sample.time_from_start - dt);
    if (error < best_error) {
      best = &sample;
      best_error = error;
    }
  }
  if (best == nullptr || best_error > config.feedback_config.max_sample_time_error) {
    return nullptr;
  }
  return best;
}

std::optional<std::pair<double, double>> computeNisAndLogLikelihood(
    const Eigen::Vector3d& observed_position,
    const Eigen::Matrix3d& observed_covariance,
    const PredictionFeedbackSampleSnapshot& sample) {
  const Eigen::Matrix3d innovation_covariance =
      sample.position_covariance + observed_covariance;
  const Eigen::LLT<Eigen::Matrix3d> llt(innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return std::nullopt;
  }

  const Eigen::Vector3d innovation = observed_position - sample.position;
  const Eigen::Vector3d solved = llt.solve(innovation);
  const double nis = innovation.dot(solved);
  const Eigen::Matrix3d l = llt.matrixL();
  double log_det = 0.0;
  for (int index = 0; index < 3; ++index) {
    const double diagonal = l(index, index);
    if (diagonal <= 0.0) {
      return std::nullopt;
    }
    log_det += 2.0 * std::log(diagonal);
  }
  constexpr double kLogTwoPi = 1.8378770664093453;
  const double log_likelihood =
      -0.5 * (nis + log_det + kNisPositionDof * kLogTwoPi);
  return std::make_pair(nis, log_likelihood);
}

double smoothValue(const double previous,
                   const double observed,
                   const double alpha,
                   const bool initialized) {
  return initialized ? (1.0 - alpha) * previous + alpha * observed : observed;
}
```

- [x] **Step 5: Implement `buildFeedbackHints()`**

Add:

```cpp
std::unordered_map<std::uint32_t, PredictionFeedbackModeHintMap>
DynamicObjectPredictor::buildFeedbackHints(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs,
    PredictionFeedbackDiagnostics& diagnostics) {
  std::unordered_map<std::uint32_t, PredictionFeedbackModeHintMap> hints_by_object;
  if (!config_.feedback_config.enabled) {
    return hints_by_object;
  }

  double nis_sum = 0.0;
  std::size_t nis_count = 0U;

  for (const auto& input : inputs) {
    const auto state_iter = feedback_states_.find(input.id);
    if (state_iter == feedback_states_.end()) {
      continue;
    }
    PredictionFeedbackObjectState& object_state = state_iter->second;
    const double dt = (header.stamp - object_state.last_stamp).toSec();
    if (dt <= 0.0 || dt > config_.feedback_config.max_age ||
        !input.matched_in_current_frame) {
      ++diagnostics.feedback_skipped_object_count;
      continue;
    }

    const auto observed_position = positionFromInput(input);
    const auto observed_covariance = positionCovarianceFromInput(input);
    if (!observed_position.has_value() || !observed_covariance.has_value()) {
      ++diagnostics.feedback_skipped_object_count;
      continue;
    }

    ++diagnostics.feedback_evaluated_object_count;
    std::vector<std::pair<PredictionFeedbackBranchKey, double>> raw_log_likelihoods;
    raw_log_likelihoods.reserve(object_state.last_prediction.size());

    for (const auto& branch : object_state.last_prediction) {
      const auto* sample = closestFeedbackSample(branch, dt, config_);
      if (sample == nullptr) {
        ++diagnostics.feedback_skipped_branch_count;
        continue;
      }
      const auto nis_and_log_likelihood =
          computeNisAndLogLikelihood(*observed_position, *observed_covariance, *sample);
      if (!nis_and_log_likelihood.has_value()) {
        ++diagnostics.feedback_skipped_branch_count;
        continue;
      }

      const double nis = nis_and_log_likelihood->first;
      const double log_likelihood = nis_and_log_likelihood->second;
      raw_log_likelihoods.emplace_back(branch.key, log_likelihood);

      PredictionFeedbackBranchState& branch_state = object_state.branch_states[branch.key];
      branch_state.smoothed_nis = smoothValue(branch_state.smoothed_nis,
                                              nis,
                                              config_.feedback_config.smoothing_alpha,
                                              branch_state.initialized);
      branch_state.smoothed_log_likelihood =
          smoothValue(branch_state.smoothed_log_likelihood,
                      log_likelihood,
                      config_.feedback_config.smoothing_alpha,
                      branch_state.initialized);

      const double nis_ratio = branch_state.smoothed_nis / kNisPositionDof;
      const double scale_delta =
          std::clamp(config_.feedback_config.noise_gain * (nis_ratio - 1.0),
                     -0.5,
                     0.5);
      branch_state.process_noise_scale = std::clamp(
          branch_state.process_noise_scale * std::exp(scale_delta),
          kMinFeedbackProcessNoiseScale,
          config_.feedback_config.max_noise_scale);
      branch_state.last_update_stamp = header.stamp;
      branch_state.initialized = true;

      diagnostics.max_nis = std::max(diagnostics.max_nis, nis);
      diagnostics.min_process_noise_scale =
          std::min(diagnostics.min_process_noise_scale, branch_state.process_noise_scale);
      diagnostics.max_process_noise_scale =
          std::max(diagnostics.max_process_noise_scale, branch_state.process_noise_scale);
      nis_sum += nis;
      ++nis_count;
      ++diagnostics.feedback_updated_branch_count;
    }

    if (raw_log_likelihoods.empty()) {
      continue;
    }
    const double mean_log_likelihood =
        std::accumulate(raw_log_likelihoods.begin(),
                        raw_log_likelihoods.end(),
                        0.0,
                        [](const double sum, const auto& item) {
                          return sum + item.second;
                        }) /
        static_cast<double>(raw_log_likelihoods.size());

    auto& hints = hints_by_object[input.id];
    for (const auto& item : raw_log_likelihoods) {
      PredictionFeedbackBranchState& branch_state = object_state.branch_states[item.first];
      branch_state.probability_score_bias =
          branch_state.smoothed_log_likelihood - mean_log_likelihood;
      PredictionFeedbackModeHint hint;
      hint.score_bias = branch_state.probability_score_bias;
      hint.process_noise_scale = branch_state.process_noise_scale;
      hint.has_feedback = true;
      hints[item.first] = hint;
      diagnostics.max_abs_probability_bias =
          std::max(diagnostics.max_abs_probability_bias, std::abs(hint.score_bias));
    }
  }

  if (nis_count > 0U) {
    diagnostics.mean_nis = nis_sum / static_cast<double>(nis_count);
  }
  return hints_by_object;
}
```

- [x] **Step 6: Remove temporary eligibility counting**

Remove `countFeedbackEligibility()` and its call from Task 2. `buildFeedbackHints()` now owns skip/evaluate/update diagnostics.

- [x] **Step 7: Run tests**

Run:

```bash
catkin run_tests ldop --no-status --make-args tests
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: probability-feedback test still fails until Task 4 applies hints to modes; cache expiry and unmatched skip tests still pass.

## Task 4: Apply Feedback to P2 Modes and Rollout Noise

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Test: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Write process-noise increase/decrease tests**

Add:

```cpp
TEST(DynamicObjectPredictorTest, FeedbackIncreasesFutureCovarianceForHighNis) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 1U;
  config.feedback_config.enabled = true;
  config.feedback_config.noise_gain = 0.5;
  config.feedback_config.probability_gain = 0.0;
  config.interaction_config.enabled = false;
  DynamicObjectPredictor feedback_predictor(config);

  DynamicObjectPredictorConfig baseline_config = config;
  baseline_config.feedback_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  TrackPredictionInput first = makeCvInput();
  first.id = 531U;
  first.object_class = ObjectClass::Other;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 340.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(341.0);
  first_header.frame_id = "map";
  static_cast<void>(feedback_predictor.predict(first_header, {first}));

  TrackPredictionInput second = first;
  second.model_state(0) += 2.0;
  second.history.push_back(makeHistorySample(
      341.1, MotionModelType::CV3D, second.model_state));

  std_msgs::Header second_header;
  second_header.stamp = ros::Time(341.1);
  second_header.frame_id = "map";
  const auto feedback = feedback_predictor.predict(second_header, {second});
  const auto baseline = baseline_predictor.predict(second_header, {second});

  const auto& feedback_branch = feedback.predictions_msg.predictions.front().branches.front();
  const auto& baseline_branch = baseline.predictions_msg.predictions.front().branches.front();
  EXPECT_GT(feedback.feedback_diagnostics.max_process_noise_scale, 1.0);
  EXPECT_GT(positionVarianceTrace(feedback_branch.points.back()),
            positionVarianceTrace(baseline_branch.points.back()));
}

TEST(DynamicObjectPredictorTest, FeedbackCanReduceFutureCovarianceButKeepsFixedFloor) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 1U;
  config.feedback_config.enabled = true;
  config.feedback_config.noise_gain = 0.5;
  config.feedback_config.probability_gain = 0.0;
  config.feedback_config.max_noise_scale = 3.0;
  config.interaction_config.enabled = false;
  DynamicObjectPredictor feedback_predictor(config);

  DynamicObjectPredictorConfig baseline_config = config;
  baseline_config.feedback_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  TrackPredictionInput input = makeCvInput();
  input.id = 532U;
  input.object_class = ObjectClass::Other;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 350.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(351.0);
  auto previous_feedback = feedback_predictor.predict(header, {input});

  for (int frame = 1; frame <= 6; ++frame) {
    const auto& previous_branch =
        previous_feedback.predictions_msg.predictions.front().branches.front();
    input.model_state(0) = previous_branch.points.front().model_state[0];
    input.model_state(1) = previous_branch.points.front().model_state[1];
    input.model_state(2) = previous_branch.points.front().model_state[2];
    input.history.push_back(makeHistorySample(
        351.0 + 0.1 * static_cast<double>(frame),
        MotionModelType::CV3D,
        input.model_state));
    header.stamp = ros::Time(351.0 + 0.1 * static_cast<double>(frame));
    previous_feedback = feedback_predictor.predict(header, {input});
  }

  const auto baseline = baseline_predictor.predict(header, {input});
  const auto& feedback_branch =
      previous_feedback.predictions_msg.predictions.front().branches.front();
  const auto& baseline_branch = baseline.predictions_msg.predictions.front().branches.front();

  EXPECT_LT(previous_feedback.feedback_diagnostics.min_process_noise_scale, 1.0);
  EXPECT_GE(previous_feedback.feedback_diagnostics.min_process_noise_scale, 0.3);
  EXPECT_LT(positionVarianceTrace(feedback_branch.points.back()),
            positionVarianceTrace(baseline_branch.points.back()));
}
```

- [x] **Step 2: Run tests and verify failure**

Run:

```bash
catkin run_tests ldop --no-status --make-args tests
```

Expected: new covariance tests and Task 3 probability test fail because feedback hints are not wired into mode generation and rollout.

- [x] **Step 3: Extend `PredictionMode`**

In the anonymous namespace `PredictionMode` struct in `src/dynamic_object_predictor.cpp`, add:

```cpp
  double feedback_score_bias{0.0};
  double feedback_noise_scale{1.0};
```

- [x] **Step 4: Apply hints before mode normalization**

Add helper:

```cpp
void applyFeedbackHintsToModes(
    std::vector<PredictionMode>& modes,
    const PredictionFeedbackModeHintMap* hints,
    const DynamicObjectPredictorConfig& config) {
  if (hints == nullptr) {
    return;
  }
  for (PredictionMode& mode : modes) {
    const auto key = makeFeedbackBranchKey(
        mode.behavior_type, mode.behavior_value, config);
    const auto hint_iter = hints->find(key);
    if (hint_iter == hints->end() || !hint_iter->second.has_feedback) {
      continue;
    }
    mode.feedback_score_bias = config.feedback_config.probability_gain *
                               hint_iter->second.score_bias;
    mode.feedback_noise_scale = std::clamp(hint_iter->second.process_noise_scale,
                                           kMinFeedbackProcessNoiseScale,
                                           config.feedback_config.max_noise_scale);
    mode.score += mode.feedback_score_bias;
  }
}
```

Change `generatePredictionModes()` signature:

```cpp
std::vector<PredictionMode> generatePredictionModes(
    const TrackPredictionInput& input,
    const MotionHistoryFeatures& features,
    const DynamicObjectPredictorConfig& config,
    const PredictionFeedbackModeHintMap* feedback_hints) {
```

Before `normalizeAndPruneModes(modes, config);`, add:

```cpp
  applyFeedbackHintsToModes(modes, feedback_hints, config);
```

- [x] **Step 5: Wire hints into `predict()`**

At the start of `predict()`, replace the cleanup-only flow with:

```cpp
  std::unordered_map<std::uint32_t, PredictionFeedbackModeHintMap> feedback_hints_by_object;
  if (config_.feedback_config.enabled) {
    cleanupExpiredFeedbackStates(header.stamp, inputs, result.feedback_diagnostics);
    feedback_hints_by_object =
        buildFeedbackHints(header, inputs, result.feedback_diagnostics);
  }
```

Inside the per-input loop, before generating modes:

```cpp
    const PredictionFeedbackModeHintMap* feedback_hints = nullptr;
    const auto hint_iter = feedback_hints_by_object.find(input.id);
    if (hint_iter != feedback_hints_by_object.end()) {
      feedback_hints = &hint_iter->second;
    }
```

Change mode generation:

```cpp
    rollout_context.modes =
        generatePredictionModes(input, rollout_context.features, config_, feedback_hints);
```

- [x] **Step 6: Apply feedback scale to future process noise**

In `rollOutMode()`, replace:

```cpp
    const double noise_scale = features.history_quality_scale * mode.mode_uncertainty_scale;
```

with:

```cpp
    const double feedback_noise_scale = std::clamp(mode.feedback_noise_scale,
                                                   kMinFeedbackProcessNoiseScale,
                                                   config.feedback_config.max_noise_scale);
    const double noise_scale = features.history_quality_scale *
                               mode.mode_uncertainty_scale *
                               feedback_noise_scale;
```

- [x] **Step 7: Run focused tests**

Run:

```bash
catkin run_tests ldop --no-status --make-args tests
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: probability-feedback and covariance feedback tests pass, along with existing predictor tests.

## Task 5: Preserve Branch-Key Semantics, P3 Ordering, and Diagnostics

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`
- Test: `test/test_dynamic_object_predictor.cpp`

- [x] **Step 1: Write branch semantic matching and P3 ordering tests**

Add:

```cpp
TEST(DynamicObjectPredictorTest, FeedbackMatchesBranchesByBehaviorKeyAfterSorting) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 8U;
  config.feedback_config.enabled = true;
  config.feedback_config.probability_gain = 2.0;
  config.feedback_config.noise_gain = 0.0;
  config.interaction_config.enabled = false;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput first = makeCvInput();
  first.id = 541U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 360.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(361.0);
  first_header.frame_id = "map";
  const auto first_result = predictor.predict(first_header, {first});
  const auto* turn_right = findBranchWithBehaviorValue(
      first_result.predictions_msg.predictions.front(),
      ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING,
      -1.0);
  ASSERT_NE(turn_right, nullptr);

  TrackPredictionInput second = first;
  second.model_state(0) = turn_right->points.front().model_state[0];
  second.model_state(1) = turn_right->points.front().model_state[1];
  second.history.push_back(makeHistorySample(
      361.1, MotionModelType::CV3D, second.model_state));

  std_msgs::Header second_header;
  second_header.stamp = ros::Time(361.1);
  second_header.frame_id = "map";
  const auto result = predictor.predict(second_header, {second});

  const auto* boosted_turn_right = findBranchWithBehaviorValue(
      result.predictions_msg.predictions.front(),
      ldop::DynamicObjectPredictionBranch::BEHAVIOR_TURNING,
      -1.0);
  ASSERT_NE(boosted_turn_right, nullptr);
  EXPECT_GT(boosted_turn_right->probability, turn_right->probability);
  EXPECT_GT(result.feedback_diagnostics.max_abs_probability_bias, 0.0);
}

TEST(DynamicObjectPredictorTest, FeedbackPriorStillAllowsInteractionReweighting) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  config.feedback_config.enabled = true;
  config.feedback_config.probability_gain = 2.0;
  config.interaction_config.enabled = true;
  config.interaction_config.pair_interaction_weight = 3.0;
  config.interaction_config.map_interaction_weight = 0.0;
  config.interaction_config.corridor_interaction_weight = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput first = makeCvInput();
  first.id = 551U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 370.0, 8, 1.0, 0.0);

  std_msgs::Header first_header;
  first_header.stamp = ros::Time(371.0);
  first_header.frame_id = "map";
  static_cast<void>(predictor.predict(first_header, {first}));

  TrackPredictionInput second = first;
  second.model_state(0) += 0.1;
  second.history.push_back(makeHistorySample(
      371.1, MotionModelType::CV3D, second.model_state));

  TrackPredictionInput opposing = makeCvInput();
  opposing.id = 552U;
  opposing.object_class = ObjectClass::Human;
  opposing.model_state(0) = 0.45;
  opposing.model_state(3) = -1.0;
  opposing.hits = 8U;
  opposing.age = 8U;
  appendCvLinearHistory(opposing, 370.0, 8, -1.0, 0.0);
  for (auto& sample : opposing.history) {
    sample.model_state(0) += 0.45;
  }

  std_msgs::Header second_header;
  second_header.stamp = ros::Time(371.1);
  second_header.frame_id = "map";
  const auto result = predictor.predict(second_header, {second, opposing});

  EXPECT_GT(result.feedback_diagnostics.feedback_updated_branch_count, 0U);
  EXPECT_GT(result.interaction_diagnostics.near_pair_branch_count, 0U);
  for (const auto& prediction : result.predictions_msg.predictions) {
    EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
  }
}
```

- [x] **Step 2: Run tests and verify behavior**

Run:

```bash
catkin run_tests ldop --no-status --make-args tests
```

Expected: tests pass if Task 4 matched by semantic key and P3 still runs after feedback.

- [x] **Step 3: Add verbose feedback logging**

In `DynamicObjectPredictor::predict()`, extend the existing `ROS_INFO_STREAM_THROTTLE` block:

```cpp
                                       << "ms, feedbackUpdatedBranches="
                                       << result.feedback_diagnostics.feedback_updated_branch_count
                                       << ", feedbackMaxNis="
                                       << result.feedback_diagnostics.max_nis
                                       << ", feedbackNoiseScale=["
                                       << result.feedback_diagnostics.min_process_noise_scale
                                       << ", "
                                       << result.feedback_diagnostics.max_process_noise_scale
                                       << "]"
```

Keep this in the existing throttled verbose log rather than adding a new ROS topic.

- [x] **Step 4: Run predictor tests**

Run:

```bash
catkin run_tests ldop --no-status --make-args tests
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all predictor and interaction context tests pass.

## Task 6: Documentation and Planning Sync

**Files:**
- Modify: `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`
- Modify: `task_plan.md`
- Modify: `progress.md`
- Modify: `findings.md`

- [x] **Step 1: Update arrangement doc P4 status**

In `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`, update P4 wording from future tense to implemented/active once code passes:

```markdown
### P4：闭环评估与概率反馈（已实现）

当前实现让 `DynamicObjectPredictor` 在内部维护按 `object_id` 索引的短期反馈状态。每帧开始时，predictor 使用上一轮预测快照与当前 tracker 观测对齐，计算 residual、NIS 和 likelihood；反馈只作用于同一目标的下一帧 mode score 和未来过程噪声尺度，不反向修改 tracker 当前状态。

过程噪声反馈只校准 predictor 未来 rollout 的 `Q_future`，不改变 tracker 传入的 `model_covariance`。`process_noise_scale = 1.0` 表示使用基础预测 Q；闭环可在 NIS 偏低或长期稳定时把尺度降到固定下界 0.3，也可在 NIS 偏大时放大到 `prediction_feedback_max_noise_scale`。
```

Keep existing metrics text if it still describes broader evaluation goals; mark multi-horizon offline reports as future evaluation tooling if not implemented in code.

- [x] **Step 2: Update `task_plan.md`**

Add a dated row in the progress/decision section:

```markdown
| 2026-05-20 | 轨迹预测内部第四阶段 P4 闭环反馈实现 | predictor 内部维护同 ID 短期反馈状态，用残差/NIS/likelihood 调整下一帧分支概率先验和未来 Q 尺度；Q 尺度下界固定 0.3，不新增 ROS msg |
```

Mark the P4 checklist entry complete:

```markdown
| 7 | complete | 建立闭环评估与概率反馈，按误差和不确定性更新各轨迹概率 |
```

- [x] **Step 3: Update `progress.md`**

Append:

```markdown
- 已完成 P4 闭环反馈：`DynamicObjectPredictor` 内部按 `object_id` 缓存上一轮预测，用当前 tracker 观测计算 residual、NIS 和 likelihood，并把反馈用于下一帧同语义分支的 score bias 和未来过程噪声尺度。
- P4 不新增 ROS 预测消息字段，不修改 tracker 当前状态；`process_noise_scale` 可在 `[0.3, prediction_feedback_max_noise_scale]` 内校准 predictor 未来 `Q_future`。
```

- [x] **Step 4: Update `findings.md`**

Add implementation facts:

```markdown
- P4 闭环反馈状态位于 `DynamicObjectPredictor` 内部，而不是 `Ldop` 或 tracker。反馈按 `object_id` 生效，分支通过 `behavior_type + behavior_value` 近似桶匹配，避免依赖 branch index。
- P4 的噪声反馈只乘到 predictor 未来 rollout 的 `Q_future`；无反馈时尺度为 1.0，闭环下界固定 0.3，不暴露为 YAML 参数。
- P4 诊断目前通过 `PredictionFeedbackDiagnostics` 和 verbose 日志暴露，不新增默认 ROS 诊断话题。
```

- [x] **Step 5: Run doc whitespace check**

Run from `/home/st/ldop_ws/src/LDOP`:

```bash
git diff --check -- docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md task_plan.md progress.md findings.md
```

Expected: no output.

## Task 7: Full Verification

**Files:**
- Verify all modified files.

- [x] **Step 1: Build package**

Run from `/home/st/ldop_ws`:

```bash
catkin build ldop --no-status
```

Expected: build succeeds.

- [x] **Step 2: Run package tests**

Run:

```bash
catkin run_tests ldop --no-status
```

Expected: test command completes without build/test target errors.

- [x] **Step 3: Read test results**

Run:

```bash
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all LDOP test suites report zero errors and zero failures. If the result initially reports zero tests while test XML files are still being written, rerun the same command after the test process finishes.

- [x] **Step 4: Check changed files**

Run from `/home/st/ldop_ws/src/LDOP`:

```bash
git status --short
```

Expected: changes are limited to the P4 predictor implementation, tests, config, and planning docs. Do not revert unrelated pre-existing changes.

- [x] **Step 5: Summarize verification evidence**

Record in the final response:

```text
catkin build ldop --no-status: passed
catkin run_tests ldop --no-status: passed
catkin_test_results /home/st/ldop_ws/build/ldop: 0 errors, 0 failures
```

If a command fails because of the ROS execution environment, include the exact failing command and the first relevant error line instead of claiming success.
