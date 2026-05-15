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
