# Dynamic Object Trajectory Prediction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first trajectory prediction stage: publish model-native baseline trajectory predictions and RViz uncertainty markers for every stable tracked dynamic object.

**Architecture:** Add a focused `DynamicObjectPredictor` module after `DynamicObjectTracker`. The predictor consumes `TrackPredictionInput`, reuses the existing `MotionModel` implementations for state/covariance propagation, publishes independent prediction messages, and leaves tracker output semantics unchanged.

**Tech Stack:** ROS1 Noetic, catkin messages, C++20, Eigen, visualization_msgs MarkerArray, existing LDOP motion model / Kalman support.

---

## Repository Rule Override

The writing-plans skill normally suggests frequent commits, but this repository's `AGENTS.md` says not to create intermediate commits during the task. Execution steps below do not include commit steps. Commit only after the implementation is complete, verified, and the user explicitly asks or approves.

## File Structure

- Create `msg/DynamicObjectPredictionPoint.msg`: one future sample point with relative time, model-native state/covariance, and planning influence weight.
- Create `msg/DynamicObjectPredictionBranch.msg`: one predicted behavior branch with behavior family, normalized behavior value, probability, and sample points.
- Create `msg/DynamicObjectPrediction.msg`: one target prediction with track identity, class/model metadata, bbox size, match status, and branches.
- Create `msg/DynamicObjectPredictionArray.msg`: frame-level prediction array.
- Modify `CMakeLists.txt`: register new messages and compile predictor source/test.
- Modify `include/ldop/multi_model_kalman_filter.h` and `src/multi_model_kalman_filter.cpp`: expose shared Kalman model parameter struct/config builder and `createMotionModel()`, so tracker and predictor use the same motion model parameters.
- Modify `include/ldop/dynamic_object_tracker.h` and `src/dynamic_object_tracker.cpp`: use the shared Kalman parameter struct/config builder, preserving existing parameter keys and behavior.
- Create `include/ldop/dynamic_object_predictor.h`: predictor params/config/result types and public API.
- Create `src/dynamic_object_predictor.cpp`: parameter loading, trajectory propagation, message construction, marker construction, influence weights.
- Modify `include/ldop/ldop.h` and `src/ldop.cpp`: hold predictor, add processed-frame fields, load topics, advertise/publish prediction outputs, and call predictor after tracker.
- Modify `config/ldop.yaml`: add prediction horizon and prediction topic defaults.
- Create `test/test_dynamic_object_predictor.cpp`: focused unit tests for baseline branch generation and influence weighting.

## Task 1: Add Prediction Messages And Build Registration

**Files:**
- Create: `msg/DynamicObjectPredictionPoint.msg`
- Create: `msg/DynamicObjectPredictionBranch.msg`
- Create: `msg/DynamicObjectPrediction.msg`
- Create: `msg/DynamicObjectPredictionArray.msg`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create prediction sample point message**

Create `msg/DynamicObjectPredictionPoint.msg`:

```text
# Relative time from the source frame stamp to this predicted sample.
duration time_from_start

# Future model-native state. Semantics match DynamicObject.model_state.
float64[] model_state

# Future model-native covariance, row-major, len(model_state)^2.
# Semantics match DynamicObject.model_covariance.
float64[] model_covariance

# Planning/RViz influence in [0,1]. This does not change model_covariance.
float64 influence_weight
```

- [ ] **Step 2: Create prediction branch message**

Create `msg/DynamicObjectPredictionBranch.msg`:

```text
uint8 BEHAVIOR_BASELINE=0
uint8 BEHAVIOR_LONGITUDINAL=1
uint8 BEHAVIOR_VERTICAL=2
uint8 BEHAVIOR_TURNING=3
uint8 BEHAVIOR_LATERAL=4

# Behavior family represented by this branch. BEHAVIOR_BASELINE is only
# for the first-stage model-based baseline extrapolation.
uint8 behavior_type

# Normalized signed behavior strength in [-1,1]. Baseline uses 0.
float64 behavior_value

# Branch probability. First-stage baseline uses 1.0.
float64 probability

DynamicObjectPredictionPoint[] points
```

- [ ] **Step 3: Create per-object prediction message**

Create `msg/DynamicObjectPrediction.msg`:

```text
uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4

uint8 MOTION_MODEL_CA2D=0
uint8 MOTION_MODEL_CA3D=1
uint8 MOTION_MODEL_CV3D=2
uint8 MOTION_MODEL_CTRA=3

uint32 id
geometry_msgs/Vector3 size
uint8 object_class

# Motion model used to interpret every points[].model_state and
# points[].model_covariance in this prediction.
uint8 motion_model_type

bool matched_in_current_frame

DynamicObjectPredictionBranch[] branches
```

Keep these class and motion-model constants synchronized with `DynamicObject.msg` so downstream code and tests can compare prediction metadata without maintaining a second numeric table.

- [ ] **Step 4: Create frame-level prediction array message**

Create `msg/DynamicObjectPredictionArray.msg`:

```text
std_msgs/Header header
DynamicObjectPrediction[] predictions
```

- [ ] **Step 5: Register messages in CMake**

Modify `CMakeLists.txt` `add_message_files()` block:

```cmake
add_message_files(
  FILES
  DynamicObject.msg
  DynamicObjectArray.msg
  DynamicObjectPredictionPoint.msg
  DynamicObjectPredictionBranch.msg
  DynamicObjectPrediction.msg
  DynamicObjectPredictionArray.msg
)
```

- [ ] **Step 6: Build to verify message generation**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
```

Expected: build reaches message generation and compiles existing code. If later tasks are not implemented yet, do this step immediately after Task 1 before including generated headers in C++.

## Task 2: Share Motion Model Parameter Construction

**Files:**
- Modify: `include/ldop/multi_model_kalman_filter.h`
- Modify: `src/multi_model_kalman_filter.cpp`
- Modify: `include/ldop/dynamic_object_tracker.h`
- Modify: `src/dynamic_object_tracker.cpp`

- [ ] **Step 1: Add shared parameter snapshot and builders**

In `include/ldop/multi_model_kalman_filter.h`, add this before `MultiModelKalmanFilterConfig`:

```cpp
struct MultiModelKalmanFilterParams {
  double default_dt{0.1};
  int adaptive_window_size{10};
  double adaptive_alpha{0.7};
  double adaptive_r_alpha{0.7};
  double adaptive_min_noise_ratio{0.2};
  bool enable_cov_limit{true};
  double max_pos_cov{1.0};
  double max_vel_cov{9.0};
  double max_acc_cov{25.0};
  double ca_human_jerk_sigma{1.0};
  std::vector<double> ca_human_init_cov{0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 10.0};
  std::vector<double> ca_human_meas_noise{0.1, 0.1, 0.1};
  double ca_human_z_process_noise{0.01};
  double ca_uav_jerk_sigma{1.0};
  std::vector<double> ca_uav_init_cov{0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};
  std::vector<double> ca_uav_meas_noise{0.1, 0.1, 0.1};
  double cv_acc_sigma{3.0};
  std::vector<double> cv_init_cov{0.5, 0.5, 0.5, 2.0, 2.0, 2.0};
  std::vector<double> cv_meas_noise{0.3, 0.3, 0.3};
  std::vector<double> ctra_init_cov{0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0};
  std::vector<double> ctra_process_noise{0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0};
  std::vector<double> ctra_meas_noise{0.1, 0.1, 0.1};
};
```

Add declarations near the existing factory declarations:

```cpp
MultiModelKalmanFilterConfig buildMultiModelKalmanFilterConfig(
    const MultiModelKalmanFilterParams& params);

std::shared_ptr<MotionModel> createMotionModel(
    MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config = {});
```

- [ ] **Step 2: Implement shared config builder**

In `src/multi_model_kalman_filter.cpp`, add:

```cpp
MultiModelKalmanFilterConfig buildMultiModelKalmanFilterConfig(
    const MultiModelKalmanFilterParams& params) {
  const MultiModelKalmanFilterParams defaults;
  MultiModelKalmanFilterConfig config;
  config.default_dt = params.default_dt > 0.0 ? params.default_dt : defaults.default_dt;
  config.adaptive_window_size =
      params.adaptive_window_size > 0 ? params.adaptive_window_size : defaults.adaptive_window_size;
  config.adaptive_alpha = std::clamp(params.adaptive_alpha, 0.0, 1.0);
  config.adaptive_r_alpha = std::clamp(params.adaptive_r_alpha, 0.0, 1.0);
  config.adaptive_min_noise_ratio = std::clamp(params.adaptive_min_noise_ratio, 0.0, 1.0);
  config.max_pos_cov = params.max_pos_cov > 0.0 ? params.max_pos_cov : defaults.max_pos_cov;
  config.max_vel_cov = params.max_vel_cov > 0.0 ? params.max_vel_cov : defaults.max_vel_cov;
  config.max_acc_cov = params.max_acc_cov > 0.0 ? params.max_acc_cov : defaults.max_acc_cov;
  config.enable_cov_limit = params.enable_cov_limit;
  config.ca_human.jerk_sigma = params.ca_human_jerk_sigma;
  config.ca_human.init_cov = params.ca_human_init_cov;
  config.ca_human.meas_noise = params.ca_human_meas_noise;
  config.ca_human.z_process_noise = params.ca_human_z_process_noise;
  config.ca_uav.jerk_sigma = params.ca_uav_jerk_sigma;
  config.ca_uav.init_cov = params.ca_uav_init_cov;
  config.ca_uav.meas_noise = params.ca_uav_meas_noise;
  config.cv.acc_sigma = params.cv_acc_sigma;
  config.cv.init_cov = params.cv_init_cov;
  config.cv.meas_noise = params.cv_meas_noise;
  config.ctra.init_cov = params.ctra_init_cov;
  config.ctra.process_noise = params.ctra_process_noise;
  config.ctra.meas_noise = params.ctra_meas_noise;
  return config;
}
```

- [ ] **Step 3: Implement motion model factory and reuse it**

In `src/multi_model_kalman_filter.cpp`, add before `createKalmanFilter()`:

```cpp
std::shared_ptr<MotionModel> createMotionModel(
    const MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config) {
  switch (model_type) {
    case MotionModelType::CA2D: {
      auto model = std::make_shared<CAMotionModel>(config.ca_human, false);
      model->setDt(config.default_dt);
      return model;
    }
    case MotionModelType::CA3D: {
      auto model = std::make_shared<CAMotionModel>(config.ca_uav, true);
      model->setDt(config.default_dt);
      return model;
    }
    case MotionModelType::CV3D: {
      auto model = std::make_shared<CVMotionModel>(config.cv);
      model->setDt(config.default_dt);
      return model;
    }
    case MotionModelType::CTRA: {
      auto model = std::make_shared<CTRAMotionModel>(config.ctra);
      model->setDt(config.default_dt);
      return model;
    }
  }
  throw std::invalid_argument("unsupported motion model");
}
```

Then replace the model construction inside `createKalmanFilter()` with:

```cpp
const std::shared_ptr<MotionModel> model = createMotionModel(model_type, config);
if (model_type == MotionModelType::CTRA) {
  return configureExtendedFilter(model, config);
}
return configureLinearFilter(model, config);
```

- [ ] **Step 4: Move tracker Kalman fields to the shared param struct**

In `include/ldop/dynamic_object_tracker.h`, replace the Kalman parameter fields in `DynamicObjectTrackerParams` with:

```cpp
MultiModelKalmanFilterParams filter;
```

Keep tracker-specific fields such as association, classification, history, coast, and thresholds in `DynamicObjectTrackerParams`.

- [ ] **Step 5: Update tracker parameter loading**

In `DynamicObjectTracker::loadParameters()`, update Kalman parameter reads to target `params_.filter.*` and default from `defaults.filter.*`, for example:

```cpp
pnh_.param("tracking_default_dt", params_.filter.default_dt, defaults.filter.default_dt);
pnh_.param("kalman_filter/adaptive_window_size",
           params_.filter.adaptive_window_size,
           defaults.filter.adaptive_window_size);
pnh_.param("kalman_filter/ca_model/human/jerk_sigma",
           params_.filter.ca_human_jerk_sigma,
           defaults.filter.ca_human_jerk_sigma);
```

Apply the same pattern for every existing `kalman_filter/...` key so the YAML keys remain unchanged.

- [ ] **Step 6: Update tracker config construction**

In `buildTrackerConfig()`, replace the manual `config.filter_config.*` assignments with:

```cpp
config.filter_config = buildMultiModelKalmanFilterConfig(params.filter);
```

Keep tracker-specific runtime conversions in `buildTrackerConfig()` unchanged.

- [ ] **Step 7: Build to verify tracker behavior still compiles**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
```

Expected: build passes. This confirms the shared parameter refactor did not break tracker construction.

## Task 3: Add DynamicObjectPredictor Interface And Parameters

**Files:**
- Create: `include/ldop/dynamic_object_predictor.h`

- [ ] **Step 1: Create predictor header**

Create `include/ldop/dynamic_object_predictor.h`:

```cpp
#pragma once

#include <vector>

#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/dynamic_object_tracker.h>
#include <ldop/multi_model_kalman_filter.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

struct DynamicObjectPredictorParams {
  double prediction_horizon{1.0};
  MultiModelKalmanFilterParams filter;
};

struct DynamicObjectPredictorConfig {
  double prediction_horizon{1.0};
  double sampling_dt{0.1};
  MultiModelKalmanFilterConfig filter_config;
};

struct DynamicObjectPredictorTimingStats {
  double predict_dynamic_objects_ms{0.0};
  double build_prediction_markers_ms{0.0};
};

struct DynamicObjectPredictorFrameResult {
  ldop::DynamicObjectPredictionArray predictions_msg;
  visualization_msgs::MarkerArray prediction_markers_msg;
  DynamicObjectPredictorTimingStats timing;
};

class DynamicObjectPredictor {
 public:
  explicit DynamicObjectPredictor(ros::NodeHandle& pnh, bool verbose = false);
  explicit DynamicObjectPredictor(const DynamicObjectPredictorConfig& config,
                                  bool verbose = false);

  DynamicObjectPredictorFrameResult predict(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs) const;

 private:
  void loadParameters();

  ros::NodeHandle pnh_;
  DynamicObjectPredictorParams params_;
  DynamicObjectPredictorConfig config_;
  bool verbose_{false};
};

DynamicObjectPredictorConfig buildPredictorConfig(
    const DynamicObjectPredictorParams& params);

}  // namespace ldopcore
```

## Task 4: Implement Prediction Message Generation

**Files:**
- Create: `src/dynamic_object_predictor.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add predictor source to CMake**

In `CMakeLists.txt`, add the source to `ldop_core`:

```cmake
add_library(ldop_core
  src/ldop.cpp
  src/dynamic_object_clusterer.cpp
  src/dynamic_object_tracker.cpp
  src/dynamic_object_predictor.cpp
  src/motion_model.cpp
  src/multi_model_kalman_filter.cpp
  src/ufomap_mapper.cpp
)
```

- [ ] **Step 2: Implement config construction and parameter loading**

In `src/dynamic_object_predictor.cpp`, start with:

```cpp
#include <ldop/dynamic_object_predictor.h>
#include <ldop/utils.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>

namespace ldopcore {

namespace {

constexpr double kMinInfluenceWeight = 0.3;
constexpr double kTimeEpsilon = 1e-9;

}  // namespace

DynamicObjectPredictorConfig buildPredictorConfig(
    const DynamicObjectPredictorParams& params) {
  const DynamicObjectPredictorParams defaults;
  DynamicObjectPredictorConfig config;
  config.prediction_horizon =
      params.prediction_horizon > 0.0 ? params.prediction_horizon
                                      : defaults.prediction_horizon;
  config.filter_config = buildMultiModelKalmanFilterConfig(params.filter);
  config.sampling_dt = config.filter_config.default_dt;
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
  pnh_.param("kalman_filter/adaptive_window_size",
             params_.filter.adaptive_window_size,
             defaults.filter.adaptive_window_size);
  pnh_.param("kalman_filter/adaptive_alpha",
             params_.filter.adaptive_alpha,
             defaults.filter.adaptive_alpha);
  pnh_.param("kalman_filter/adaptive_r_alpha",
             params_.filter.adaptive_r_alpha,
             defaults.filter.adaptive_r_alpha);
  pnh_.param("kalman_filter/adaptive_min_noise_ratio",
             params_.filter.adaptive_min_noise_ratio,
             defaults.filter.adaptive_min_noise_ratio);
  pnh_.param("kalman_filter/enable_cov_limit",
             params_.filter.enable_cov_limit,
             defaults.filter.enable_cov_limit);
  pnh_.param("kalman_filter/max_pos_cov",
             params_.filter.max_pos_cov,
             defaults.filter.max_pos_cov);
  pnh_.param("kalman_filter/max_vel_cov",
             params_.filter.max_vel_cov,
             defaults.filter.max_vel_cov);
  pnh_.param("kalman_filter/max_acc_cov",
             params_.filter.max_acc_cov,
             defaults.filter.max_acc_cov);
  pnh_.param("kalman_filter/ca_model/human/jerk_sigma",
             params_.filter.ca_human_jerk_sigma,
             defaults.filter.ca_human_jerk_sigma);
  pnh_.param("kalman_filter/ca_model/human/init_cov",
             params_.filter.ca_human_init_cov,
             defaults.filter.ca_human_init_cov);
  pnh_.param("kalman_filter/ca_model/human/meas_noise",
             params_.filter.ca_human_meas_noise,
             defaults.filter.ca_human_meas_noise);
  pnh_.param("kalman_filter/ca_model/human/z_process_noise",
             params_.filter.ca_human_z_process_noise,
             defaults.filter.ca_human_z_process_noise);
  pnh_.param("kalman_filter/ca_model/uav/jerk_sigma",
             params_.filter.ca_uav_jerk_sigma,
             defaults.filter.ca_uav_jerk_sigma);
  pnh_.param("kalman_filter/ca_model/uav/init_cov",
             params_.filter.ca_uav_init_cov,
             defaults.filter.ca_uav_init_cov);
  pnh_.param("kalman_filter/ca_model/uav/meas_noise",
             params_.filter.ca_uav_meas_noise,
             defaults.filter.ca_uav_meas_noise);
  pnh_.param("kalman_filter/cv_model/acc_sigma",
             params_.filter.cv_acc_sigma,
             defaults.filter.cv_acc_sigma);
  pnh_.param("kalman_filter/cv_model/init_cov",
             params_.filter.cv_init_cov,
             defaults.filter.cv_init_cov);
  pnh_.param("kalman_filter/cv_model/meas_noise",
             params_.filter.cv_meas_noise,
             defaults.filter.cv_meas_noise);
  pnh_.param("kalman_filter/ctra_model/init_cov",
             params_.filter.ctra_init_cov,
             defaults.filter.ctra_init_cov);
  pnh_.param("kalman_filter/ctra_model/process_noise",
             params_.filter.ctra_process_noise,
             defaults.filter.ctra_process_noise);
  pnh_.param("kalman_filter/ctra_model/meas_noise",
             params_.filter.ctra_meas_noise,
             defaults.filter.ctra_meas_noise);
  config_ = buildPredictorConfig(params_);
}
```

- [ ] **Step 3: Add influence weight and vector copy helpers**

Add in the anonymous namespace:

```cpp
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

std::vector<double> eigenVectorToStdVector(const Eigen::VectorXd& vector) {
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(vector.size()));
  for (int index = 0; index < vector.size(); ++index) {
    values.push_back(vector(index));
  }
  return values;
}

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
```

- [ ] **Step 4: Implement prediction message assembly**

Add this method body:

```cpp
DynamicObjectPredictorFrameResult DynamicObjectPredictor::predict(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs) const {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectPredictorFrameResult result;
  result.predictions_msg.header = header;
  result.predictions_msg.predictions.reserve(inputs.size());

  for (const TrackPredictionInput& input : inputs) {
    std::shared_ptr<MotionModel> model =
        createMotionModel(input.motion_model_type, config_.filter_config);
    model->setDt(config_.sampling_dt);

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

    for (double elapsed = config_.sampling_dt;
         elapsed <= config_.prediction_horizon + kTimeEpsilon;
         elapsed += config_.sampling_dt) {
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
      point.influence_weight = computeInfluenceWeight(elapsed, config_.prediction_horizon);
      branch.points.push_back(std::move(point));
    }

    prediction.branches.push_back(std::move(branch));
    result.predictions_msg.predictions.push_back(std::move(prediction));
  }

  result.timing.predict_dynamic_objects_ms =
      elapsedMs(total_start, std::chrono::steady_clock::now());
  return result;
}

}  // namespace ldopcore
```

This step intentionally uses base `MotionModel::getProcessNoiseQ()` for future propagation. The initial covariance is tracker's current `model_covariance`, which already contains tracker filtering/adaptive effects.

## Task 5: Implement Prediction Markers

**Files:**
- Modify: `src/dynamic_object_predictor.cpp`

- [ ] **Step 1: Add marker helper declarations and includes**

Add includes:

```cpp
#include <Eigen/Eigenvalues>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
```

Add helper constants:

```cpp
constexpr double kMatchedAlpha = 0.45;
constexpr double kCoastingAlpha = 0.18;
```

- [ ] **Step 2: Add covariance-to-ellipsoid helper**

Add in the anonymous namespace:

```cpp
geometry_msgs::Quaternion eigenMatrixToQuaternion(const Eigen::Matrix3d& rotation) {
  const Eigen::Quaterniond quaternion(rotation);
  geometry_msgs::Quaternion ros_quaternion;
  ros_quaternion.x = quaternion.x();
  ros_quaternion.y = quaternion.y();
  ros_quaternion.z = quaternion.z();
  ros_quaternion.w = quaternion.w();
  return ros_quaternion;
}

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
```

- [ ] **Step 3: Add marker construction helper**

Add a private method declaration in `dynamic_object_predictor.h`:

```cpp
visualization_msgs::MarkerArray buildPredictionMarkers(
    const std_msgs::Header& header,
    const ldop::DynamicObjectPredictionArray& predictions) const;
```

Implement it in `src/dynamic_object_predictor.cpp`:

```cpp
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

    const double target_alpha =
        prediction.matched_in_current_frame ? kMatchedAlpha : kCoastingAlpha;
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
```

- [ ] **Step 4: Call marker construction from predict()**

At the end of `DynamicObjectPredictor::predict()`, before setting total timing return, insert:

```cpp
const auto marker_start = std::chrono::steady_clock::now();
result.prediction_markers_msg = buildPredictionMarkers(header, result.predictions_msg);
result.timing.build_prediction_markers_ms =
    elapsedMs(marker_start, std::chrono::steady_clock::now());
```

Then set `predict_dynamic_objects_ms` from the full operation.

## Task 6: Add Focused Predictor Tests

**Files:**
- Create: `test/test_dynamic_object_predictor.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add catkin gtest target**

In `CMakeLists.txt`, after `target_link_libraries(ldop_node ...)`, add:

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dynamic_object_predictor
    test/test_dynamic_object_predictor.cpp
  )
  if(TARGET test_dynamic_object_predictor)
    target_link_libraries(test_dynamic_object_predictor
      ldop_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
```

- [ ] **Step 2: Add baseline prediction test**

Create `test/test_dynamic_object_predictor.cpp`:

```cpp
#include <gtest/gtest.h>

#include <ldop/dynamic_object_predictor.h>

namespace ldopcore {
namespace {

TrackPredictionInput makeCvInput() {
  TrackPredictionInput input;
  input.id = 42U;
  input.object_class = ObjectClass::Other;
  input.motion_model_type = MotionModelType::CV3D;
  input.bbox.size.x = 1.0;
  input.bbox.size.y = 0.5;
  input.bbox.size.z = 0.8;
  input.model_state = Eigen::VectorXd::Zero(6);
  input.model_state(3) = 1.0;
  input.model_covariance = 0.1 * Eigen::MatrixXd::Identity(6, 6);
  input.matched_in_current_frame = true;
  return input;
}

TEST(DynamicObjectPredictorTest, GeneratesBaselineCvPredictionPoints) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.2;
  config.sampling_dt = 0.1;
  config.filter_config.default_dt = 0.1;
  DynamicObjectPredictor predictor(config);

  std_msgs::Header header;
  header.stamp = ros::Time(10.0);
  header.frame_id = "map";
  const DynamicObjectPredictorFrameResult result =
      predictor.predict(header, std::vector<TrackPredictionInput>{makeCvInput()});

  ASSERT_EQ(result.predictions_msg.predictions.size(), 1U);
  const auto& prediction = result.predictions_msg.predictions.front();
  EXPECT_EQ(prediction.id, 42U);
  EXPECT_EQ(prediction.motion_model_type, ldop::DynamicObjectPrediction::MOTION_MODEL_CV3D);
  ASSERT_EQ(prediction.branches.size(), 1U);
  const auto& branch = prediction.branches.front();
  EXPECT_EQ(branch.behavior_type, ldop::DynamicObjectPredictionBranch::BEHAVIOR_BASELINE);
  EXPECT_DOUBLE_EQ(branch.behavior_value, 0.0);
  EXPECT_DOUBLE_EQ(branch.probability, 1.0);
  ASSERT_EQ(branch.points.size(), 2U);

  EXPECT_NEAR(branch.points[0].time_from_start.toSec(), 0.1, 1e-9);
  EXPECT_NEAR(branch.points[0].model_state[0], 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(branch.points[0].influence_weight, 1.0);
  EXPECT_NEAR(branch.points[1].time_from_start.toSec(), 0.2, 1e-9);
  EXPECT_NEAR(branch.points[1].model_state[0], 0.2, 1e-9);
  EXPECT_NEAR(branch.points[1].influence_weight, 0.3, 1e-9);
  EXPECT_EQ(branch.points[1].model_covariance.size(), 36U);
}

}  // namespace
}  // namespace ldopcore
```

This test intentionally uses the motion-model constants defined in `DynamicObjectPrediction.msg`; keep those constants synchronized with `DynamicObject.msg`.

- [ ] **Step 3: Run focused test**

Run:

```bash
cd /home/st/ldop_ws
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: `test_dynamic_object_predictor` passes. If message generation has not completed, run `catkin build ldop --no-status` first.

## Task 7: Integrate Predictor Into Ldop

**Files:**
- Modify: `include/ldop/ldop.h`
- Modify: `src/ldop.cpp`
- Modify: `config/ldop.yaml`

- [ ] **Step 1: Update Ldop includes and fields**

In `include/ldop/ldop.h`, add:

```cpp
#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/dynamic_object_predictor.h>
```

Add to `ProcessedFrame`:

```cpp
ldop::DynamicObjectPredictionArray dynamic_predictions_msg;
visualization_msgs::MarkerArray dynamic_prediction_markers_msg;
```

Add publishers:

```cpp
ros::Publisher dynamic_predictions_pub_;
ros::Publisher dynamic_prediction_markers_pub_;
```

Add topic names:

```cpp
std::string dynamic_predictions_topic_;
std::string dynamic_prediction_markers_topic_;
```

Add module:

```cpp
std::unique_ptr<DynamicObjectPredictor> dynamic_object_predictor_;
```

- [ ] **Step 2: Construct predictor and log topics**

In `Ldop::Ldop()`, after tracker construction:

```cpp
dynamic_object_predictor_ = std::make_unique<DynamicObjectPredictor>(pnh_, verbose_);
```

Add startup logs:

```cpp
ROS_INFO_STREAM("  dynamic predictions: " << dynamic_predictions_topic_);
ROS_INFO_STREAM("  dynamic prediction markers: " << dynamic_prediction_markers_topic_);
```

- [ ] **Step 3: Load and advertise prediction topics**

In `Ldop::loadParameters()`:

```cpp
pnh_.param<std::string>("dynamic_predictions_topic",
                        dynamic_predictions_topic_,
                        "/ldop/dynamic_object_predictions");
pnh_.param<std::string>("dynamic_prediction_markers_topic",
                        dynamic_prediction_markers_topic_,
                        "/ldop/dynamic_prediction_markers");
```

In `Ldop::setupRosInterfaces()`:

```cpp
dynamic_predictions_pub_ =
    nh_.advertise<ldop::DynamicObjectPredictionArray>(dynamic_predictions_topic_, 10);
dynamic_prediction_markers_pub_ =
    nh_.advertise<visualization_msgs::MarkerArray>(dynamic_prediction_markers_topic_, 10);
```

- [ ] **Step 4: Call predictor in processFrame()**

In `Ldop::processFrame()`, after `tracking_result` is available:

```cpp
DynamicObjectPredictorFrameResult prediction_result =
    dynamic_object_predictor_->predict(processed.dynamic_cloud_msg.header,
                                       tracking_result.prediction_inputs);
```

After existing tracker output assignments:

```cpp
processed.dynamic_predictions_msg = std::move(prediction_result.predictions_msg);
processed.dynamic_prediction_markers_msg =
    std::move(prediction_result.prediction_markers_msg);
```

- [ ] **Step 5: Publish prediction outputs**

In `Ldop::publishFrame()`:

```cpp
dynamic_predictions_pub_.publish(frame.dynamic_predictions_msg);
dynamic_prediction_markers_pub_.publish(frame.dynamic_prediction_markers_msg);
```

- [ ] **Step 6: Add config defaults**

In `config/ldop.yaml`, near the dynamic output topic keys:

```yaml
dynamic_predictions_topic: /ldop/dynamic_object_predictions
dynamic_prediction_markers_topic: /ldop/dynamic_prediction_markers
```

Near tracking params:

```yaml
prediction_horizon: 1.0  # 预测未来时长，采样间隔复用 tracking_default_dt
```

## Task 8: Final Build And Verification

**Files:**
- All modified files

- [ ] **Step 1: Build package**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
```

Expected: build succeeds, including generated prediction messages and `ldop_core`.

- [ ] **Step 2: Run tests if Task 6 was implemented**

Run:

```bash
cd /home/st/ldop_ws
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all `ldop` tests pass.

- [ ] **Step 3: Inspect git diff**

Run:

```bash
cd /home/st/ldop_ws/src/LDOP
git diff --check
git status --short
```

Expected: `git diff --check` reports no whitespace errors. `git status --short` only shows planned files.

## Self-Review

- Spec coverage: tasks cover messages, predictor module, motion model reuse, covariance propagation with base `Q`, influence weighting, RViz markers, Ldop integration, params, and verification.
- Placeholder scan: no `TBD`, `TODO`, or vague implementation placeholders remain.
- Type consistency: `DynamicObjectPredictionPoint`, `DynamicObjectPredictionBranch`, `DynamicObjectPrediction`, `DynamicObjectPredictionArray`, `DynamicObjectPredictorFrameResult`, and `DynamicObjectPredictorConfig` are introduced before use.
- Repository rule: no commit steps are included because repository instructions prohibit intermediate commits during the task.
