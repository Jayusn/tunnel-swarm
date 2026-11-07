#include <gtest/gtest.h>
#include "mpc_controller.h"
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace {

class ThrustMappingTest : public testing::Test {
 protected:
  void SetUp() override {
    // 使用固定的本地时间，无需 ROS master，也不创建发布者或服务客户端。
    ros::Time::init();
    ros::Time::setNow(ros::Time(100));
    params.step_T_ = 0.02;
    params.gravity_ = 9.795;
    params.dyn_params_.mass_q = 1.926;
    params.q_gain_ = {1, 1, 1, 1, 1};
    params.r_gain_ = {1, 1, 1, 1, 1};
    params.min_thrust_ = 1;
    params.max_thrust_ = 40;
    params.max_bodyrate_xy_ = params.max_bodyrate_z_ = 1;
    params.max_velocity_xy_ = params.max_velocity_z_ = 1;
    params.max_velocity_slack_xy_ = params.max_velocity_slack_z_ = 1;
    params.force_estimator_param_.sqrt_kf = std::sqrt(1.835e-8);
    params.thr_map_.accurate_thrust_model = 1;
    params.thr_map_.hover_percentage = 0.543;
    params.thr_map_.filter_factor = 0.2;
    params.thr_map_.min_learning_rpm = 7000;
    params.thr_map_.max_normalized_thrust = 0.8;
    params.thr_map_.max_hover_percentage_step = 0.01;
    controller.reset(new PayloadMPC::MpcController(params));
    controller->waitForPreparation();
    controller->resetThrustMapping();
  }

  bool update(double rpm, double command = 0.63) {
    controller->timed_thrust.emplace(ros::Time::now() - ros::Duration(0.04), command);
    return controller->estimateThrustModel(
        Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
        Eigen::Vector4d::Constant(rpm), 13.3, params);
  }

  void expectRejected(double rpm, double command = 0.63) {
    const double scale = controller->thrustscale_;
    const double covariance = controller->P;
    const double hover = controller->debug.hover_percentage;
    EXPECT_FALSE(update(rpm, command));
    EXPECT_DOUBLE_EQ(scale, controller->thrustscale_);
    EXPECT_DOUBLE_EQ(covariance, controller->P);
    EXPECT_DOUBLE_EQ(hover, controller->debug.hover_percentage);
    EXPECT_TRUE(controller->timed_thrust.empty());
  }

  PayloadMPC::MpcParams params{};
  std::unique_ptr<PayloadMPC::MpcController> controller;
};

TEST_F(ThrustMappingTest, LargeIncreaseAdvancesByConfiguredStep) {
  ASSERT_TRUE(update(12000));
  EXPECT_NEAR(0.553, controller->currentHoverPercentage(), 1e-12);
  EXPECT_NEAR(0.553, controller->debug.hover_percentage, 1e-7);
  EXPECT_GT(controller->P, 0);
  EXPECT_LT(controller->P, 1e6);
}

TEST_F(ThrustMappingTest, LargeDecreaseAdvancesByConfiguredStep) {
  ASSERT_TRUE(update(19000));
  EXPECT_NEAR(0.533, controller->currentHoverPercentage(), 1e-12);
}

TEST_F(ThrustMappingTest, SmallUpdateMatchesUnconstrainedRls) {
  const double scale = controller->thrustscale_;
  const double covariance = controller->P;
  const double gain = covariance * 0.63 / (controller->rho2 + 0.63 * covariance * 0.63);
  const double measured_thrust = 4 * 1.835e-8 * 17000 * 17000;
  const double expected_scale = scale + 0.2 * gain * (measured_thrust - 0.63 * scale);
  ASSERT_TRUE(update(17000));
  EXPECT_NEAR(expected_scale, controller->thrustscale_, 1e-12);
  EXPECT_NEAR((1 - gain * 0.63) * covariance / controller->rho2, controller->P, 1e-12);
}

TEST_F(ThrustMappingTest, RepeatedSamplesContinueAdaptingInsteadOfStalling) {
  const double true_hover = 0.65;
  const double rpm = std::sqrt(params.gravity_ * params.dyn_params_.mass_q /
                               true_hover * 0.63 / (4 * 1.835e-8));
  for (int i = 0; i < 1000; ++i) {
    const double previous = controller->currentHoverPercentage();
    ASSERT_TRUE(update(rpm));
    const double current = controller->currentHoverPercentage();
    EXPECT_GT(current, previous);
    EXPECT_LE(current - previous, 0.01 + 1e-12);
    EXPECT_LT(current, true_hover);
  }
  EXPECT_GT(controller->currentHoverPercentage(), 0.62);
}

TEST_F(ThrustMappingTest, InvalidRpmAndCommandsKeepLastMapping) {
  expectRejected(6999);
  expectRejected(std::numeric_limits<double>::quiet_NaN());
  expectRejected(std::numeric_limits<double>::infinity());
  expectRejected(17000, 0);
  expectRejected(17000, 0.81);
  expectRejected(17000, std::numeric_limits<double>::quiet_NaN());
}

TEST_F(ThrustMappingTest, OutOfRangeCandidateIsStillRejected) {
  expectRejected(100000);  // 候选悬停比例低于 0.1。
  params.thr_map_.filter_factor = 1;
  expectRejected(7000);  // 候选悬停比例高于 0.8。
}

TEST_F(ThrustMappingTest, InvalidCovarianceIsStillRejected) {
  controller->P = -1;
  expectRejected(17000);
}

TEST_F(ThrustMappingTest, FixedLandingWeightsAreOptionalAndRestoreOnExit) {
  using namespace PayloadMPC;
  const std::vector<real_t> baseline(acadoVariables.W, acadoVariables.W + ACADO_N*ACADO_NY*ACADO_NY);
  const std::vector<real_t> terminal(acadoVariables.WN, acadoVariables.WN + ACADO_NYN*ACADO_NYN);
  const Eigen::Vector3d position(1., 2., .65), zero = Eigen::Vector3d::Zero();
  ASSERT_TRUE(controller->setPositionCommandReference(position, zero, zero, zero, -.5, 0., true));
  controller->waitForPreparation();
  EXPECT_TRUE(std::equal(baseline.begin(), baseline.end(), acadoVariables.W));
  params.landing_position_weight_xy_ = 400.;
  params.landing_bodyrate_weight_xy_ = 20.;
  for (int cycle = 0; cycle < 3; ++cycle) {
    for (int repeat = 0; repeat < 3; ++repeat) {
      ASSERT_TRUE(controller->setPositionCommandReference(position, zero, zero, zero, -.5, 0., true));
      controller->waitForPreparation();
      EXPECT_NEAR(acadoVariables.W[0], 400., 1.e-6);
      EXPECT_NEAR(acadoVariables.W[kRefSize+1], 400., 1.e-6);
      EXPECT_NEAR(acadoVariables.W[2*(kRefSize+1)], params.q_gain_.Q_pos_z, 1.e-6);
      EXPECT_NEAR(acadoVariables.W[(kRefSize+1)*(kCostSize+kRateX)], 20., 1.e-6);
      EXPECT_NEAR(acadoVariables.W[(kRefSize+1)*(kCostSize+kRateY)], 20., 1.e-6);
    }
    if (cycle == 0) {
      ASSERT_TRUE(controller->setPositionCommandReference(position, zero, zero, zero, -.5, 0.));
    } else if (cycle == 1) {
      controller->setHoverReference(position, -.5);
    } else {
      Eigen::Matrix<real_t,kStateSize,1> state;
      state << 1,2,.65,1,0,0,0,0,0,0;
      ASSERT_TRUE(controller->resetForHover(state, position, -.5));
    }
    controller->waitForPreparation();
    EXPECT_TRUE(std::equal(baseline.begin(), baseline.end(), acadoVariables.W));
    EXPECT_TRUE(std::equal(terminal.begin(), terminal.end(), acadoVariables.WN));
  }
}

}  // namespace
