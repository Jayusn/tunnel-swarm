#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ldop/dynamic_object_predictor.h>
#include <optional>
#include <ros/ros.h>
#include <string>
#include <vector>

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

void appendCvSpeedRampHistory(TrackPredictionInput& input,
                              const double start_time,
                              const std::vector<double>& speeds) {
  input.history.clear();
  double x = 0.0;
  for (std::size_t index = 0U; index < speeds.size(); ++index) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
    const double t = start_time + static_cast<double>(index) * 0.1;
    x += speeds[index] * 0.1;
    state(0) = x;
    state(3) = speeds[index];
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

class PredictorFakeMapQuery final : public PredictionMapQuery {
 public:
  bool available() const override { return available_; }

  std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const override {
    if (!isOccupied(point)) {
      return std::nullopt;
    }
    PredictionMapNodeQuery node;
    node.exists = true;
    node.occupied = true;
    node.center = point;
    node.voxel_size = 0.2;
    return node;
  }

  bool seenFree(const Eigen::Vector3d& /*point*/) const override {
    return available_ && seen_free_;
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      double /*radius*/,
      std::size_t /*max_results*/) const override {
    if (!isOccupied(center)) {
      if (available_ && near_node_enabled_) {
        PredictionLocalOccupiedNode node;
        node.center = center - Eigen::Vector3d(near_node_offset_, 0.0, 0.0);
        node.voxel_size = 0.0;
        node.hits = 1U;
        return {node};
      }
      return {};
    }
    PredictionLocalOccupiedNode node;
    node.center = center;
    node.voxel_size = 0.2;
    node.hits = 1U;
    return {node};
  }

  bool available_{true};
  bool occupied_node_enabled_{false};
  bool near_node_enabled_{false};
  bool seen_free_{false};
  double occupied_min_x_{-std::numeric_limits<double>::infinity()};
  double occupied_max_x_{std::numeric_limits<double>::infinity()};
  double near_node_offset_{0.86};

 private:
  bool isOccupied(const Eigen::Vector3d& point) const {
    return available_ && occupied_node_enabled_ &&
           point.x() >= occupied_min_x_ && point.x() <= occupied_max_x_;
  }
};

TEST(DynamicObjectPredictorTest, GeneratesBaselineCvPredictionPoints) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.2;
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
  ASSERT_GE(prediction.branches.size(), 1U);
  const auto& branch = prediction.branches.front();
  EXPECT_EQ(branch.behavior_type, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL);
  ASSERT_EQ(branch.points.size(), 2U);

  EXPECT_NEAR(branch.points[0].time_from_start.toSec(), 0.1, 1e-9);
  EXPECT_NEAR(branch.points[0].model_state[0], 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(branch.points[0].influence_weight, 1.0);
  EXPECT_NEAR(branch.points[1].time_from_start.toSec(), 0.2, 1e-9);
  EXPECT_NEAR(branch.points[1].model_state[0], 0.2, 1e-9);
  EXPECT_NEAR(branch.points[1].influence_weight, 0.3, 1e-9);
  EXPECT_EQ(branch.points[1].model_covariance.size(), 36U);
}

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

const ldop::DynamicObjectPredictionBranch* findBranchWithBehaviorValue(
    const ldop::DynamicObjectPrediction& prediction,
    const std::uint8_t behavior_type,
    const double behavior_value) {
  for (const auto& branch : prediction.branches) {
    if (branch.behavior_type == behavior_type &&
        std::abs(branch.behavior_value - behavior_value) < 1e-9) {
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
  config.max_branches = 8U;
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
  config.max_branches = 8U;
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

TEST(DynamicObjectPredictorTest, HistoricalSpeedDropIncreasesDecelerationProbability) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 8U;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput constant = makeCvInput();
  constant.object_class = ObjectClass::Human;
  constant.model_state(3) = 0.4;
  constant.hits = 8U;
  constant.age = 8U;
  appendCvLinearHistory(constant, 85.0, 8, 0.4, 0.0);

  TrackPredictionInput slowing = makeCvInput();
  slowing.object_class = ObjectClass::Human;
  slowing.model_state(3) = 0.4;
  slowing.hits = 8U;
  slowing.age = 8U;
  appendCvSpeedRampHistory(slowing, 85.0, {1.2, 1.1, 1.0, 0.9, 0.75, 0.6, 0.5, 0.4});

  std_msgs::Header header;
  header.stamp = ros::Time(86.0);
  header.frame_id = "map";
  const auto constant_result = predictor.predict(header, {constant});
  const auto slowing_result = predictor.predict(header, {slowing});

  const auto* constant_decelerate = findBranchWithBehaviorValue(
      constant_result.predictions_msg.predictions.front(),
      ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
      -0.45);
  const auto* slowing_decelerate = findBranchWithBehaviorValue(
      slowing_result.predictions_msg.predictions.front(),
      ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL,
      -0.45);

  ASSERT_NE(constant_decelerate, nullptr);
  ASSERT_NE(slowing_decelerate, nullptr);
  EXPECT_GT(slowing_decelerate->probability, constant_decelerate->probability);
}

TEST(DynamicObjectPredictorTest, DecelerationBranchDoesNotRaiseLowSpeedTarget) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.2;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 8U;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.model_state(3) = 0.05;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 87.0, 8, 0.05, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(88.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto& prediction = result.predictions_msg.predictions.front();
  const auto* decelerate = findBranchWithBehaviorValue(
      prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL, -0.45);

  ASSERT_NE(decelerate, nullptr);
  ASSERT_FALSE(decelerate->points.empty());
  EXPECT_LE(decelerate->points.back().model_state[3], input.model_state(3));
}

TEST(DynamicObjectPredictorTest, UnknownAndOtherUseVerticalBranchesWhenVerticalEvidenceExists) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);

  for (const ObjectClass object_class : {ObjectClass::Unknown, ObjectClass::Other}) {
    TrackPredictionInput input = makeCvInput();
    input.object_class = object_class;
    input.model_state(5) = 0.5;
    input.hits = 8U;
    input.age = 8U;
    appendCvLinearHistory(input, 89.0, 8, 1.0, 0.0, 0.5);

    std_msgs::Header header;
    header.stamp = ros::Time(90.0);
    header.frame_id = "map";
    const auto result = predictor.predict(header, {input});
    const auto& prediction = result.predictions_msg.predictions.front();

    EXPECT_TRUE(hasBehavior(prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL));
    EXPECT_TRUE(hasBranchWithBehaviorValue(
        prediction, ldop::DynamicObjectPredictionBranch::BEHAVIOR_VERTICAL, 1.0));
  }
}

double positionVarianceTrace(const ldop::DynamicObjectPredictionPoint& point) {
  const std::size_t state_size = point.model_state.size();
  return point.model_covariance[0U * state_size + 0U] +
         point.model_covariance[1U * state_size + 1U] +
         point.model_covariance[2U * state_size + 2U];
}

double terminalPositionDistance(const ldop::DynamicObjectPredictionBranch& lhs,
                                const ldop::DynamicObjectPredictionBranch& rhs) {
  const auto& lhs_state = lhs.points.back().model_state;
  const auto& rhs_state = rhs.points.back().model_state;
  const double dx = lhs_state[0] - rhs_state[0];
  const double dy = lhs_state[1] - rhs_state[1];
  const double dz = lhs_state[2] - rhs_state[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::size_t countMarkersByNamespace(const visualization_msgs::MarkerArray& markers,
                                    const std::string& marker_namespace);

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

TEST(DynamicObjectPredictorTest, InteractionCorrectionKeepsFirstPredictedPointAnchored) {
  DynamicObjectPredictorConfig baseline_config;
  baseline_config.prediction_horizon = 0.5;
  baseline_config.filter_config.default_dt = 0.1;
  baseline_config.max_branches = 1U;
  baseline_config.min_branch_probability = 0.0;
  baseline_config.interaction_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  DynamicObjectPredictorConfig interaction_config = baseline_config;
  interaction_config.interaction_config.enabled = true;
  interaction_config.interaction_config.pair_interaction_weight = 3.0;
  interaction_config.interaction_config.map_interaction_weight = 0.0;
  interaction_config.interaction_config.corridor_interaction_weight = 0.0;
  interaction_config.interaction_config.max_correction_distance = 0.2;
  DynamicObjectPredictor interaction_predictor(interaction_config);

  TrackPredictionInput first = makeCvInput();
  first.id = 31U;
  first.object_class = ObjectClass::Other;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 170.0, 8, 1.0, 0.0);

  TrackPredictionInput second = makeCvInput();
  second.id = 32U;
  second.object_class = ObjectClass::Other;
  second.model_state(0) = 0.45;
  second.model_state(3) = -1.0;
  second.hits = 8U;
  second.age = 8U;
  appendCvLinearHistory(second, 170.0, 8, -1.0, 0.0);
  for (auto& sample : second.history) {
    sample.model_state(0) += 0.45;
  }

  std_msgs::Header header;
  header.stamp = ros::Time(171.0);
  header.frame_id = "map";
  const auto baseline = baseline_predictor.predict(header, {first, second});
  const auto interacted = interaction_predictor.predict(header, {first, second});

  ASSERT_EQ(interacted.predictions_msg.predictions.size(),
            baseline.predictions_msg.predictions.size());
  EXPECT_GT(interacted.interaction_diagnostics.near_pair_branch_count, 0U);

  for (std::size_t prediction_index = 0U;
       prediction_index < baseline.predictions_msg.predictions.size();
       ++prediction_index) {
    const auto& baseline_branch =
        baseline.predictions_msg.predictions[prediction_index].branches.front();
    const auto& interacted_branch =
        interacted.predictions_msg.predictions[prediction_index].branches.front();
    ASSERT_EQ(interacted_branch.points.size(), baseline_branch.points.size());
    ASSERT_FALSE(interacted_branch.points.empty());

    // 交互修正只能改变未来局部趋势，不能把预测线的起点从当前目标附近整段平移出去。
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      EXPECT_NEAR(interacted_branch.points.front().model_state[axis],
                  baseline_branch.points.front().model_state[axis],
                  1e-9);
    }
    EXPECT_GT(terminalPositionDistance(interacted_branch, baseline_branch), 1e-6);
  }
}

TEST(DynamicObjectPredictorTest, InteractionCorrectionRerolloutAvoidsLocalPositionKink) {
  DynamicObjectPredictorConfig baseline_config;
  baseline_config.prediction_horizon = 0.5;
  baseline_config.filter_config.default_dt = 0.1;
  baseline_config.max_branches = 1U;
  baseline_config.interaction_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  DynamicObjectPredictorConfig interaction_config = baseline_config;
  interaction_config.interaction_config.enabled = true;
  interaction_config.interaction_config.pair_interaction_weight = 0.0;
  interaction_config.interaction_config.map_interaction_weight = 3.0;
  interaction_config.interaction_config.corridor_interaction_weight = 0.0;
  interaction_config.interaction_config.max_correction_distance = 0.2;
  DynamicObjectPredictor interaction_predictor(interaction_config);

  TrackPredictionInput input = makeCvInput();
  input.id = 41U;
  input.object_class = ObjectClass::Other;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 180.0, 8, 1.0, 0.0);

  PredictorFakeMapQuery map;
  map.occupied_node_enabled_ = true;
  map.occupied_min_x_ = 0.29;
  map.occupied_max_x_ = 0.31;

  std_msgs::Header header;
  header.stamp = ros::Time(181.0);
  header.frame_id = "map";
  const auto baseline = baseline_predictor.predict(header, {input});
  const auto interaction = interaction_predictor.predict(header, {input}, &map);

  ASSERT_EQ(baseline.predictions_msg.predictions.size(), 1U);
  ASSERT_EQ(interaction.predictions_msg.predictions.size(), 1U);
  const auto& baseline_branch = baseline.predictions_msg.predictions.front().branches.front();
  const auto& interaction_branch =
      interaction.predictions_msg.predictions.front().branches.front();
  ASSERT_EQ(interaction_branch.points.size(), baseline_branch.points.size());
  ASSERT_GE(interaction_branch.points.size(), 4U);
  EXPECT_GT(interaction.interaction_diagnostics.colliding_branch_count, 0U);
  EXPECT_GT(terminalPositionDistance(interaction_branch, baseline_branch), 0.0);

  // 局部地图冲突只应该变成时间上平滑的控制量；planner-facing 中心轨迹不应出现
  // 先前进、再后退、再前进的离散折返。
  for (std::size_t point_index = 1U;
       point_index < interaction_branch.points.size();
       ++point_index) {
    const double step_x =
        interaction_branch.points[point_index].model_state[0] -
        interaction_branch.points[point_index - 1U].model_state[0];
    EXPECT_GT(step_x, 0.0);
  }
}

TEST(DynamicObjectPredictorTest, InteractionContextKeepsBranchProbabilitiesNormalized) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  config.interaction_config.enabled = true;
  config.interaction_config.pair_interaction_weight = 3.0;
  config.interaction_config.map_interaction_weight = 0.0;
  config.interaction_config.corridor_interaction_weight = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput first = makeCvInput();
  first.id = 1U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 140.0, 8, 1.0, 0.0);

  TrackPredictionInput second = makeCvInput();
  second.id = 2U;
  second.object_class = ObjectClass::Human;
  second.model_state(0) = 0.45;
  second.model_state(3) = -1.0;
  second.hits = 8U;
  second.age = 8U;
  appendCvLinearHistory(second, 140.0, 8, -1.0, 0.0);
  for (auto& sample : second.history) {
    sample.model_state(0) += 0.45;
  }

  std_msgs::Header header;
  header.stamp = ros::Time(141.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {first, second});

  ASSERT_EQ(result.predictions_msg.predictions.size(), 2U);
  EXPECT_GT(result.interaction_diagnostics.near_pair_branch_count, 0U);
  for (const auto& prediction : result.predictions_msg.predictions) {
    ASSERT_FALSE(prediction.branches.empty());
    EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
  }
}

TEST(DynamicObjectPredictorTest, InteractionFallbackUsesBestLogScaledBranch) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.5;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 2U;
  config.min_branch_probability = 0.0;
  config.stop_speed = 1.0;
  config.interaction_config.enabled = true;
  config.interaction_config.pair_interaction_weight = 0.0;
  config.interaction_config.map_interaction_weight = 1.0;
  config.interaction_config.corridor_interaction_weight = 0.0;
  config.interaction_config.interaction_energy_tau = 1.0e-9;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 145.0, 8, 1.0, 0.0);

  PredictorFakeMapQuery map;
  map.occupied_node_enabled_ = true;
  map.near_node_enabled_ = true;
  map.occupied_min_x_ = 0.32;

  std_msgs::Header header;
  header.stamp = ros::Time(146.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input}, &map);

  ASSERT_EQ(result.predictions_msg.predictions.size(), 1U);
  const auto& prediction = result.predictions_msg.predictions.front();
  ASSERT_EQ(prediction.branches.size(), 2U);
  ASSERT_FALSE(prediction.branches[0].points.empty());
  ASSERT_FALSE(prediction.branches[1].points.empty());
  EXPECT_EQ(prediction.branches[0].probability, 0.0);
  EXPECT_EQ(prediction.branches[1].probability, 1.0);
}

TEST(DynamicObjectPredictorTest, InteractionContextDoesNotShrinkModelCovariance) {
  DynamicObjectPredictorConfig baseline_config;
  baseline_config.prediction_horizon = 0.5;
  baseline_config.filter_config.default_dt = 0.1;
  baseline_config.max_branches = 6U;
  baseline_config.min_branch_probability = 0.0;
  baseline_config.interaction_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  DynamicObjectPredictorConfig interaction_config = baseline_config;
  interaction_config.interaction_config.enabled = true;
  interaction_config.interaction_config.pair_interaction_weight = 3.0;
  interaction_config.interaction_config.map_interaction_weight = 0.0;
  interaction_config.interaction_config.corridor_interaction_weight = 0.0;
  DynamicObjectPredictor interaction_predictor(interaction_config);

  TrackPredictionInput first = makeCvInput();
  first.id = 11U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 150.0, 8, 1.0, 0.0);

  TrackPredictionInput second = makeCvInput();
  second.id = 12U;
  second.object_class = ObjectClass::Human;
  second.model_state(0) = 0.45;
  second.model_state(3) = -1.0;
  second.hits = 8U;
  second.age = 8U;
  appendCvLinearHistory(second, 150.0, 8, -1.0, 0.0);
  for (auto& sample : second.history) {
    sample.model_state(0) += 0.45;
  }

  std_msgs::Header header;
  header.stamp = ros::Time(151.0);
  header.frame_id = "map";
  const auto baseline = baseline_predictor.predict(header, {first, second});
  const auto interaction = interaction_predictor.predict(header, {first, second});

  ASSERT_EQ(interaction.predictions_msg.predictions.size(),
            baseline.predictions_msg.predictions.size());
  for (std::size_t prediction_index = 0U;
       prediction_index < baseline.predictions_msg.predictions.size();
       ++prediction_index) {
    const auto& baseline_prediction =
        baseline.predictions_msg.predictions[prediction_index];
    const auto& interaction_prediction =
        interaction.predictions_msg.predictions[prediction_index];
    ASSERT_EQ(interaction_prediction.branches.size(), baseline_prediction.branches.size());
    for (std::size_t branch_index = 0U;
         branch_index < baseline_prediction.branches.size();
         ++branch_index) {
      const auto& baseline_branch = baseline_prediction.branches[branch_index];
      const auto& interaction_branch = interaction_prediction.branches[branch_index];
      ASSERT_EQ(interaction_branch.points.size(), baseline_branch.points.size());
      for (std::size_t point_index = 0U;
           point_index < baseline_branch.points.size();
           ++point_index) {
        const auto& baseline_covariance =
            baseline_branch.points[point_index].model_covariance;
        const auto& interaction_covariance =
            interaction_branch.points[point_index].model_covariance;
        ASSERT_EQ(interaction_covariance.size(), baseline_covariance.size());
        for (std::size_t value_index = 0U;
             value_index < baseline_covariance.size();
             ++value_index) {
          EXPECT_NEAR(interaction_covariance[value_index],
                      baseline_covariance[value_index],
                      1e-12);
        }
      }
    }
  }
}

TEST(DynamicObjectPredictorTest, InteractionContextStillBuildsOnlyPredictionLineMarkers) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 6U;
  config.min_branch_probability = 0.0;
  config.interaction_config.enabled = true;
  config.interaction_config.pair_interaction_weight = 2.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput first = makeCvInput();
  first.id = 21U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 160.0, 8, 1.0, 0.0);

  TrackPredictionInput second = makeCvInput();
  second.id = 22U;
  second.object_class = ObjectClass::Human;
  second.model_state(0) = 0.35;
  second.model_state(3) = -1.0;
  second.hits = 8U;
  second.age = 8U;
  appendCvLinearHistory(second, 160.0, 8, -1.0, 0.0);
  for (auto& sample : second.history) {
    sample.model_state(0) += 0.35;
  }

  std_msgs::Header header;
  header.stamp = ros::Time(161.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {first, second});

  std::size_t branch_count = 0U;
  for (const auto& prediction : result.predictions_msg.predictions) {
    branch_count += prediction.branches.size();
  }
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg, "dynamic_prediction_line"),
            branch_count);
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg, "dynamic_prediction_samples"), 0U);
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg,
                                    "dynamic_prediction_uncertainty"),
            0U);
}

TEST(DynamicObjectPredictorTest, InteractionContextMapCorrectionStaysWithinLimit) {
  DynamicObjectPredictorConfig baseline_config;
  baseline_config.prediction_horizon = 0.3;
  baseline_config.filter_config.default_dt = 0.1;
  baseline_config.max_branches = 1U;
  baseline_config.interaction_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);

  DynamicObjectPredictorConfig interaction_config = baseline_config;
  interaction_config.interaction_config.enabled = true;
  interaction_config.interaction_config.pair_interaction_weight = 0.0;
  interaction_config.interaction_config.map_interaction_weight = 3.0;
  interaction_config.interaction_config.corridor_interaction_weight = 0.0;
  interaction_config.interaction_config.max_correction_distance = 0.05;
  DynamicObjectPredictor interaction_predictor(interaction_config);

  TrackPredictionInput input = makeCvInput();
  input.id = 31U;
  input.object_class = ObjectClass::Other;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 170.0, 8, 1.0, 0.0);

  PredictorFakeMapQuery map;
  map.occupied_node_enabled_ = true;

  std_msgs::Header header;
  header.stamp = ros::Time(171.0);
  header.frame_id = "map";
  const auto baseline = baseline_predictor.predict(header, {input});
  const auto interaction = interaction_predictor.predict(header, {input}, &map);

  ASSERT_EQ(baseline.predictions_msg.predictions.size(), 1U);
  ASSERT_EQ(interaction.predictions_msg.predictions.size(), 1U);
  ASSERT_FALSE(baseline.predictions_msg.predictions.front().branches.empty());
  ASSERT_FALSE(interaction.predictions_msg.predictions.front().branches.empty());
  const auto& baseline_branch = baseline.predictions_msg.predictions.front().branches.front();
  const auto& interaction_branch =
      interaction.predictions_msg.predictions.front().branches.front();
  ASSERT_FALSE(baseline_branch.points.empty());
  ASSERT_FALSE(interaction_branch.points.empty());
  EXPECT_GT(interaction.interaction_diagnostics.colliding_branch_count, 0U);
  EXPECT_GT(terminalPositionDistance(interaction_branch, baseline_branch), 0.0);
  EXPECT_LE(terminalPositionDistance(interaction_branch, baseline_branch),
            interaction_config.interaction_config.max_correction_distance + 1e-9);
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

std::size_t countMarkersByNamespace(const visualization_msgs::MarkerArray& markers,
                                    const std::string& marker_namespace) {
  return static_cast<std::size_t>(std::count_if(
      markers.markers.begin(),
      markers.markers.end(),
      [&](const auto& marker) { return marker.ns == marker_namespace; }));
}

const visualization_msgs::Marker* firstMarkerByNamespace(
    const visualization_msgs::MarkerArray& markers,
    const std::string& marker_namespace) {
  const auto iter = std::find_if(
      markers.markers.begin(), markers.markers.end(), [&](const auto& marker) {
        return marker.ns == marker_namespace;
      });
  return iter == markers.markers.end() ? nullptr : &(*iter);
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

TEST(DynamicObjectPredictorTest, PredictionLineMarkersUseSubtleTrackerScale) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 1U;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 115.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(116.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input});
  const auto* line =
      firstMarkerByNamespace(result.prediction_markers_msg, "dynamic_prediction_line");

  ASSERT_NE(line, nullptr);
  EXPECT_NEAR(line->scale.x, 0.04, 1e-9);
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

TEST(DynamicObjectPredictorTest, DoesNotBuildSampleTrajectoryMarkers) {
  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 130.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(131.0);
  header.frame_id = "map";

  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  DynamicObjectPredictor predictor(config);
  const auto result = predictor.predict(header, {input});
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg, "dynamic_prediction_samples"), 0U);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_dynamic_object_predictor");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
