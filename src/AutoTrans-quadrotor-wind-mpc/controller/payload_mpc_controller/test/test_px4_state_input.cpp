#include "mpc_fsm.h"
#include <gtest/gtest.h>
#include <limits>
#include <memory>

namespace PayloadMPC {
class Px4StateInputTestAccess {
 public:
  static Eigen::Matrix<real_t, kStateSize, 1> state(MPCFSM& f) {
    f.setEstimateState(f.odom_data, f.force_attitude_odom_data);
    return f.est_state_;
  }
  static bool valid(const MPCFSM& f) { return f.odom_state_valid(); }
};
}

using namespace PayloadMPC;

class Px4StateInput : public testing::Test {
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
    fsm->force_attitude_odom_data.v = Eigen::Vector3d(1.2, -.7, -.095);
    fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
  }
  void TearDown() override { ros::Time::init(); }

  ros::NodeHandle nh{"~"};
  MpcParams params;
  std::unique_ptr<MpcController> controller;
  std::unique_ptr<MPCFSM> fsm;
};

TEST_F(Px4StateInput, TakesOnlyPositionFromLioAndVelocityAttitudeFromPx4) {
  const Eigen::Quaterniond attitude(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()));
  fsm->force_attitude_odom_data.q = attitude;
  const auto x = Px4StateInputTestAccess::state(*fsm);
  EXPECT_TRUE(Px4StateInputTestAccess::valid(*fsm));
  EXPECT_NEAR(x(kPosX), 1., 1e-6);
  EXPECT_NEAR(x(kPosY), 2., 1e-6);
  EXPECT_NEAR(x(kPosZ), .429, 1e-6);
  EXPECT_NEAR(x(kVelX), 1.2, 1e-6);
  EXPECT_NEAR(x(kVelY), -.7, 1e-6);
  EXPECT_NEAR(x(kVelZ), -.095, 1e-6);
  EXPECT_NEAR(x(kOriW), attitude.w(), 1e-6);
  EXPECT_NEAR(x(kOriZ), attitude.z(), 1e-6);
}

TEST_F(Px4StateInput, RotatesAllBodyVelocityAxesUsingPx4Attitude) {
  // PX4的+90度俯仰把机体+x映射到世界-z；LIO姿态故意不同。
  fsm->odom_data.q = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitY()));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time::now();
  msg->pose.pose.orientation.w = q.w(); msg->pose.pose.orientation.y = q.y();
  msg->twist.twist.linear.x = 1.2;
  msg->twist.twist.linear.y = -.7;
  msg->twist.twist.linear.z = -.095;
  fsm->forceAttitudeOdomCallback(msg);
  const auto x = Px4StateInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kVelX), -.095, 1e-6);
  EXPECT_NEAR(x(kVelY), -.7, 1e-6);
  EXPECT_NEAR(x(kVelZ), -1.2, 1e-6);
  EXPECT_NEAR(x(kOriW), q.w(), 1e-6);
  EXPECT_NEAR(x(kOriY), q.y(), 1e-6);
}

TEST_F(Px4StateInput, QuaternionScaleAndSignDoNotChangeWorldVelocity) {
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  q.coeffs() *= -3.;
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time::now();
  msg->pose.pose.orientation.w = q.w(); msg->pose.pose.orientation.x = q.x();
  msg->twist.twist.linear.y = -.7;
  fsm->forceAttitudeOdomCallback(msg);
  const auto x = Px4StateInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kVelZ), -.7, 1e-6);
  EXPECT_NEAR(x.segment<4>(kOriW).norm(), 1., 1e-6);
}

TEST_F(Px4StateInput, HoldsLatestPx4SampleWithoutIntegratingLioAcceleration) {
  ros::Time::setNow(ros::Time(100.06));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time(100.02);
  msg->header.frame_id = "map";
  msg->child_frame_id = "base_link";
  msg->pose.pose.orientation.w = 1.;
  msg->twist.twist.linear.z = -.3;
  fsm->forceAttitudeOdomCallback(msg);
  EXPECT_NEAR(Px4StateInputTestAccess::state(*fsm)(kVelZ), -.3, 1e-6);
  ros::Time::setNow(ros::Time(100.09));
  fsm->odom_data.v.z() = 2.;
  EXPECT_NEAR(Px4StateInputTestAccess::state(*fsm)(kVelZ), -.3, 1e-6);
  EXPECT_EQ(fsm->force_attitude_odom_data.msg.header.stamp, ros::Time(100.02));
  msg->header.stamp = ros::Time(100.05);
  msg->twist.twist.linear.z = -.5;
  fsm->forceAttitudeOdomCallback(msg);
  EXPECT_NEAR(Px4StateInputTestAccess::state(*fsm)(kVelZ), -.5, 1e-6);
}

TEST_F(Px4StateInput, RequiresPx4InputAlongsideExistingOdometryInputs) {
  ros::Time::setNow(ros::Time(100.));
  fsm->odom_data.rcv_stamp = ros::Time(100.);
  EXPECT_FALSE(fsm->odom_is_received(ros::Time(100.)));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time(99.97);
  msg->pose.pose.orientation.w = 1.;
  fsm->forceAttitudeOdomCallback(msg);
  EXPECT_TRUE(fsm->odom_is_received(ros::Time(100.)));
}

TEST_F(Px4StateInput, InvalidNewInputsUseExistingInvalidStatePath) {
  fsm->force_attitude_odom_data.v.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Px4StateInputTestAccess::valid(*fsm));
  auto x = Px4StateInputTestAccess::state(*fsm);
  EXPECT_TRUE(x.allFinite());
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
  EXPECT_NEAR(x(kVelZ), 0., 1e-6);
  fsm->force_attitude_odom_data.v.setZero();
  fsm->force_attitude_odom_data.q.coeffs().setZero();
  EXPECT_FALSE(Px4StateInputTestAccess::valid(*fsm));
  x = Px4StateInputTestAccess::state(*fsm);
  EXPECT_TRUE(x.allFinite());
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
}

TEST_F(Px4StateInput, MpcRespondsToPx4DescentDespitePositiveLioVelocity) {
  fsm->odom_data.p = Eigen::Vector3d(0, 0, .429);
  fsm->force_attitude_odom_data.v.setZero();
  controller->setHoverReference(Eigen::Vector3d(0, 0, .6), 0.);
  Eigen::Matrix<real_t, kStateSize, kSamples + 1> predicted;
  Eigen::Matrix<real_t, kInputSize, kSamples> inputs;
  fsm->force_attitude_odom_data.v.z() = .525;
  auto x = Px4StateInputTestAccess::state(*fsm);
  for (int i = 0; i < 10; ++i) controller->execMPC(x, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  const double upward_thrust = inputs(kThrust, 0);
  fsm->force_attitude_odom_data.v.z() = -.095;
  x = Px4StateInputTestAccess::state(*fsm);
  for (int i = 0; i < 10; ++i) controller->execMPC(x, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  const double weight = params.dyn_params_.mass_q * params.gravity_;
  EXPECT_LT(upward_thrust, weight);
  EXPECT_GT(inputs(kThrust, 0), weight);
  EXPECT_GT(inputs(kThrust, 0) - upward_thrust, 3.0);
}

TEST_F(Px4StateInput, LioVelocityAndAttitudeDoNotAffectStateOrPositionAcceptance) {
  ros::Time::setNow(ros::Time(200.));
  nav_msgs::Odometry::Ptr px4(new nav_msgs::Odometry);
  px4->header.stamp = ros::Time(200.);
  px4->pose.pose.orientation.w = 1.;
  px4->twist.twist.linear.x = .1;
  fsm->forceAttitudeOdomCallback(px4);
  nav_msgs::Odometry::Ptr lio(new nav_msgs::Odometry);
  lio->pose.pose.position.z = .5;
  lio->twist.twist.linear.x = 100.;
  lio->pose.pose.orientation.w = 0.;
  fsm->odomCallback(lio);
  ros::Time::setNow(ros::Time(200.01));
  lio->pose.pose.position.x = .001;
  lio->twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  lio->pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  fsm->odomCallback(lio);
  EXPECT_TRUE(Px4StateInputTestAccess::valid(*fsm));
  const auto x = Px4StateInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kPosX), .001, 1e-6);
  EXPECT_NEAR(x(kPosZ), .5, 1e-6);
  EXPECT_NEAR(x(kVelX), .1, 1e-6);
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
}

TEST_F(Px4StateInput, Px4ImuAccelerationAndAngularRateReachExistingInputPath) {
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu);
  msg->header.stamp = ros::Time::now();
  msg->orientation_covariance[0] = -1.;
  msg->linear_acceleration.x = .2;
  msg->linear_acceleration.z = 9.81;
  msg->angular_velocity.y = -.03;
  fsm->imuCallback(msg);
  EXPECT_DOUBLE_EQ(fsm->imu_data.a.x(), .2);
  EXPECT_DOUBLE_EQ(fsm->imu_data.a.z(), 9.81);
  EXPECT_DOUBLE_EQ(fsm->imu_data.w.y(), -.03);
  EXPECT_TRUE(fsm->imu_data.filtered_a.allFinite());
  EXPECT_TRUE(Px4StateInputTestAccess::valid(*fsm));
}
