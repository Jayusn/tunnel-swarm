#include "mpc_fsm.h"
#include <gtest/gtest.h>
#include <limits>
#include <memory>

namespace PayloadMPC {
class VerticalVelocityInputTestAccess {
 public:
  static Eigen::Matrix<real_t, kStateSize, 1> state(MPCFSM& f) {
    f.setEstimateState(f.odom_data, f.force_attitude_odom_data, f.vertical_velocity_odom_data);
    return f.est_state_;
  }
  static bool valid(const MPCFSM& f) { return f.odom_state_valid(); }
};
}

using namespace PayloadMPC;

class VerticalVelocityInput : public testing::Test {
 protected:
  void SetUp() override {
    params.config_from_ros_handle(nh);
    controller.reset(new MpcController(params));
    fsm.reset(new MPCFSM(nh, params, *controller));
    controller->waitForPreparation();
    fsm->odom_data.p = Eigen::Vector3d(1.0, 2.0, .429);
    fsm->odom_data.v = Eigen::Vector3d(.1, -.2, .525);
    fsm->odom_data.q = Eigen::Quaterniond::Identity();
    fsm->force_attitude_odom_data.p = Eigen::Vector3d(9.0, 8.0, 7.0);
    fsm->vertical_velocity_odom_data.v = Eigen::Vector3d(1.2, -.7, -.095);
    fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
  }
  void TearDown() override { ros::Time::init(); }

  ros::NodeHandle nh{"~"};
  MpcParams params;
  std::unique_ptr<MpcController> controller;
  std::unique_ptr<MPCFSM> fsm;
};

TEST_F(VerticalVelocityInput, ChangesOnlyVerticalVelocityAndKeepsPx4Attitude) {
  const Eigen::Quaterniond attitude(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()));
  fsm->force_attitude_odom_data.q = attitude;
  const auto x = VerticalVelocityInputTestAccess::state(*fsm);
  EXPECT_TRUE(VerticalVelocityInputTestAccess::valid(*fsm));
  EXPECT_NEAR(x(kPosX), 1., 1e-6);
  EXPECT_NEAR(x(kPosY), 2., 1e-6);
  EXPECT_NEAR(x(kPosZ), .429, 1e-6);
  EXPECT_NEAR(x(kVelX), .1, 1e-6);
  EXPECT_NEAR(x(kVelY), -.2, 1e-6);
  EXPECT_NEAR(x(kVelZ), -.095, 1e-6);
  EXPECT_NEAR(x(kOriW), attitude.w(), 1e-6);
  EXPECT_NEAR(x(kOriZ), attitude.z(), 1e-6);
}

TEST_F(VerticalVelocityInput, ConvertsBodyVelocityToLioWorldInsteadOfPx4Map) {
  // LIO世界系的+90度俯仰把机体+x映射到世界-z；PX4 map取不同姿态。
  fsm->odom_data.q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitY()));
  fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
  const auto x = VerticalVelocityInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kVelZ), -1.2, 1e-6);
  EXPECT_NEAR(x(kVelX), .1, 1e-6);
  EXPECT_NEAR(x(kVelY), -.2, 1e-6);
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
}

TEST_F(VerticalVelocityInput, QuaternionScaleAndSignDoNotChangeVelocity) {
  fsm->odom_data.q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  fsm->odom_data.q.coeffs() *= -3.;
  fsm->force_attitude_odom_data.q.coeffs() *= 2.;
  const auto x = VerticalVelocityInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kVelZ), -.7, 1e-6);
  EXPECT_NEAR(x.segment<4>(kOriW).norm(), 1., 1e-6);
}

TEST_F(VerticalVelocityInput, HoldsLatestPx4SampleWithoutIntegratingLioAcceleration) {
  ros::Time::setNow(ros::Time(100.06));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time(100.02);
  msg->header.frame_id = "map";
  msg->child_frame_id = "base_link";
  msg->pose.pose.orientation.w = 1.;
  msg->twist.twist.linear.z = -.3;
  fsm->verticalVelocityOdomCallback(msg);
  EXPECT_NEAR(VerticalVelocityInputTestAccess::state(*fsm)(kVelZ), -.3, 1e-6);
  ros::Time::setNow(ros::Time(100.09));
  fsm->odom_data.v.z() = 2.;
  EXPECT_NEAR(VerticalVelocityInputTestAccess::state(*fsm)(kVelZ), -.3, 1e-6);
  EXPECT_EQ(fsm->vertical_velocity_odom_data.msg.header.stamp, ros::Time(100.02));
  msg->header.stamp = ros::Time(100.05);
  msg->twist.twist.linear.z = -.5;
  fsm->verticalVelocityOdomCallback(msg);
  EXPECT_NEAR(VerticalVelocityInputTestAccess::state(*fsm)(kVelZ), -.5, 1e-6);
}

TEST_F(VerticalVelocityInput, RequiresPx4InputAlongsideExistingOdometryInputs) {
  ros::Time::setNow(ros::Time(100.));
  fsm->odom_data.rcv_stamp = ros::Time(100.);
  fsm->force_attitude_odom_data.rcv_stamp = ros::Time(100.);
  EXPECT_FALSE(fsm->odom_is_received(ros::Time(100.)));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time(99.97);
  msg->pose.pose.orientation.w = 1.;
  fsm->verticalVelocityOdomCallback(msg);
  EXPECT_TRUE(fsm->odom_is_received(ros::Time(100.)));
}

TEST_F(VerticalVelocityInput, InvalidNewInputsUseExistingInvalidStatePath) {
  fsm->vertical_velocity_odom_data.v.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(VerticalVelocityInputTestAccess::valid(*fsm));
  auto x = VerticalVelocityInputTestAccess::state(*fsm);
  EXPECT_TRUE(x.allFinite());
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
  EXPECT_NEAR(x(kVelZ), 0., 1e-6);
  fsm->vertical_velocity_odom_data.v.setZero();
  fsm->odom_data.q.coeffs().setZero();
  EXPECT_FALSE(VerticalVelocityInputTestAccess::valid(*fsm));
  x = VerticalVelocityInputTestAccess::state(*fsm);
  EXPECT_TRUE(x.allFinite());
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
}

TEST_F(VerticalVelocityInput, MpcRespondsToPx4DescentDespitePositiveLioVelocity) {
  fsm->odom_data.p = Eigen::Vector3d(0, 0, .429);
  fsm->vertical_velocity_odom_data.v.setZero();
  controller->setHoverReference(Eigen::Vector3d(0, 0, .6), 0.);
  Eigen::Matrix<real_t, kStateSize, kSamples + 1> predicted;
  Eigen::Matrix<real_t, kInputSize, kSamples> inputs;
  fsm->vertical_velocity_odom_data.v.z() = .525;
  auto x = VerticalVelocityInputTestAccess::state(*fsm);
  for (int i = 0; i < 10; ++i) controller->execMPC(x, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  const double upward_thrust = inputs(kThrust, 0);
  fsm->vertical_velocity_odom_data.v.z() = -.095;
  x = VerticalVelocityInputTestAccess::state(*fsm);
  for (int i = 0; i < 10; ++i) controller->execMPC(x, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  const double weight = params.dyn_params_.mass_q * params.gravity_;
  EXPECT_LT(upward_thrust, weight);
  EXPECT_GT(inputs(kThrust, 0), weight);
  EXPECT_GT(inputs(kThrust, 0) - upward_thrust, 3.0);
}
