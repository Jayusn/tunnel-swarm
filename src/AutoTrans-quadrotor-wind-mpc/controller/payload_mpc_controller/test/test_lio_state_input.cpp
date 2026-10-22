#include "mpc_fsm.h"
#include <gtest/gtest.h>
#include <limits>
#include <memory>

namespace PayloadMPC {
class LioStateInputTestAccess {
 public:
  static Eigen::Matrix<real_t, kStateSize, 1> state(MPCFSM& f) {
    f.setEstimateState(f.odom_data, f.force_attitude_odom_data);
    return f.est_state_;
  }
  static bool valid(const MPCFSM& f) { return f.odom_state_valid(); }
  static ForceObserverSyncResult synchronizedInput(
      MPCFSM& f, double stamp, ForceObserverSynchronizedInput& input) {
    f.force_input_synchronizer_.addRpm(stamp, stamp, Eigen::Vector4d::Constant(16000.));
    return f.force_input_synchronizer_.synchronize(stamp, input);
  }
};
}
using namespace PayloadMPC;

class LioStateInput : public testing::Test {
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
    // The attitude subscription must not override accepted LIO translation.
    fsm->force_attitude_odom_data.v = Eigen::Vector3d(1.2, -.7, -.095);
    fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
  }
  void TearDown() override { ros::Time::init(); }
  ros::NodeHandle nh{"~"};
  MpcParams params;
  std::unique_ptr<MpcController> controller;
  std::unique_ptr<MPCFSM> fsm;
};

TEST_F(LioStateInput, TakesPositionAndAllVelocityAxesFromAcceptedLio) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()));
  fsm->force_attitude_odom_data.q = q;
  const auto x = LioStateInputTestAccess::state(*fsm);
  EXPECT_TRUE(LioStateInputTestAccess::valid(*fsm));
  EXPECT_NEAR(x(kPosX), 1., 1e-6);
  EXPECT_NEAR(x(kPosY), 2., 1e-6);
  EXPECT_NEAR(x(kPosZ), .429, 1e-6);
  EXPECT_NEAR(x(kVelX), .1, 1e-6);
  EXPECT_NEAR(x(kVelY), -.2, 1e-6);
  EXPECT_NEAR(x(kVelZ), .525, 1e-6);
  EXPECT_NEAR(x(kOriW), q.w(), 1e-6);
  EXPECT_NEAR(x(kOriZ), q.z(), 1e-6);
}

TEST_F(LioStateInput, DoesNotRotateLioWorldVelocityByBodyAttitude) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitY()));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time::now();
  msg->pose.pose.orientation.w = q.w(); msg->pose.pose.orientation.y = q.y();
  msg->twist.twist.linear.x = 1.2;
  msg->twist.twist.linear.y = -.7;
  msg->twist.twist.linear.z = -.095;
  fsm->odomCallback(msg);
  fsm->forceAttitudeOdomCallback(msg);
  const auto x = LioStateInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kVelX), 1.2, 1e-6);
  EXPECT_NEAR(x(kVelY), -.7, 1e-6);
  EXPECT_NEAR(x(kVelZ), -.095, 1e-6);
  EXPECT_NEAR(x(kOriW), q.w(), 1e-6);
  EXPECT_NEAR(x(kOriY), q.y(), 1e-6);
}

TEST_F(LioStateInput, NormalizesAttitudeWithoutChangingWorldVelocity) {
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  q.coeffs() *= -3.;
  fsm->force_attitude_odom_data.q = q;
  const auto x = LioStateInputTestAccess::state(*fsm);
  EXPECT_NEAR(x(kVelX), .1, 1e-6);
  EXPECT_NEAR(x(kVelY), -.2, 1e-6);
  EXPECT_NEAR(x(kVelZ), .525, 1e-6);
  EXPECT_NEAR(x.segment<4>(kOriW).norm(), 1., 1e-6);
}

TEST_F(LioStateInput, LioVelocityChangesAffectStateWithoutAnotherAttitudeSample) {
  EXPECT_NEAR(LioStateInputTestAccess::state(*fsm)(kVelZ), .525, 1e-6);
  fsm->odom_data.v.z() = -.3;
  EXPECT_NEAR(LioStateInputTestAccess::state(*fsm)(kVelZ), -.3, 1e-6);
  fsm->force_attitude_odom_data.v.z() = 8.;
  EXPECT_NEAR(LioStateInputTestAccess::state(*fsm)(kVelZ), -.3, 1e-6);
}

TEST_F(LioStateInput, SameLioMessageSatisfiesExistingOdometryFreshnessChecks) {
  ros::Time::setNow(ros::Time(100.));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time(99.995);
  msg->pose.pose.orientation.w = 1.;
  fsm->odomCallback(msg);
  EXPECT_FALSE(fsm->odom_is_received(ros::Time(100.)));
  fsm->forceAttitudeOdomCallback(msg);
  EXPECT_TRUE(fsm->odom_is_received(ros::Time(100.)));
}

TEST_F(LioStateInput, InvalidLioVelocityOrAttitudeUsesExistingInvalidStatePath) {
  fsm->odom_data.v.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(LioStateInputTestAccess::valid(*fsm));
  auto x = LioStateInputTestAccess::state(*fsm);
  EXPECT_TRUE(x.allFinite());
  EXPECT_NEAR(x(kOriW), 1., 1e-6);
  EXPECT_NEAR(x(kVelZ), 0., 1e-6);
  fsm->odom_data.v.setZero();
  fsm->force_attitude_odom_data.q.coeffs().setZero();
  EXPECT_FALSE(LioStateInputTestAccess::valid(*fsm));
  EXPECT_TRUE(LioStateInputTestAccess::state(*fsm).allFinite());
}

TEST_F(LioStateInput, MpcThrustRespondsToLioVerticalVelocity) {
  fsm->odom_data.p = Eigen::Vector3d(0, 0, .429);
  fsm->odom_data.v.setZero();
  controller->setHoverReference(Eigen::Vector3d(0, 0, .6), 0.);
  Eigen::Matrix<real_t, kStateSize, kSamples + 1> predicted;
  Eigen::Matrix<real_t, kInputSize, kSamples> inputs;
  fsm->odom_data.v.z() = .525;
  auto x = LioStateInputTestAccess::state(*fsm);
  for (int i = 0; i < 10; ++i) controller->execMPC(x, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  const double upward_thrust = inputs(kThrust, 0);
  fsm->odom_data.v.z() = -.095;
  x = LioStateInputTestAccess::state(*fsm);
  for (int i = 0; i < 10; ++i) controller->execMPC(x, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  const double weight = params.dyn_params_.mass_q * params.gravity_;
  EXPECT_LT(upward_thrust, weight);
  EXPECT_GT(inputs(kThrust, 0), weight);
  EXPECT_GT(inputs(kThrust, 0) - upward_thrust, 3.0);
}

TEST_F(LioStateInput, ExistingSpikeGuardUsesLioVelocity) {
  ros::Time::setNow(ros::Time(200.));
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
  msg->header.stamp = ros::Time(200.);
  msg->pose.pose.orientation.w = 1.;
  msg->twist.twist.linear.x = .1;
  fsm->odomCallback(msg);
  ros::Time::setNow(ros::Time(200.01));
  msg->header.stamp = ros::Time(200.01);
  msg->twist.twist.linear.x = 5.;
  fsm->odomCallback(msg);
  EXPECT_NEAR(fsm->odom_data.v.x(), .1, 1e-9);
  EXPECT_EQ(fsm->lio_rejected_total_, 1U);
}

TEST_F(LioStateInput, LivoxImuAndLioAttitudeReachForceSynchronizer) {
  ros::Time::setNow(ros::Time(100.));
  sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu);
  imu->header.stamp = ros::Time(99.995);
  imu->header.frame_id = "UAV1/body";
  imu->linear_acceleration.x = .2;
  imu->linear_acceleration.z = 9.81;
  imu->angular_velocity.y = -.03;
  fsm->imuCallback(imu);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX()));
  nav_msgs::Odometry::Ptr attitude(new nav_msgs::Odometry);
  attitude->header.stamp = imu->header.stamp;
  attitude->pose.pose.orientation.w = q.w();
  attitude->pose.pose.orientation.x = q.x();
  fsm->forceAttitudeOdomCallback(attitude);
  ForceObserverSynchronizedInput input;
  ASSERT_EQ(LioStateInputTestAccess::synchronizedInput(*fsm, 99.995, input),
            ForceObserverSyncResult::READY);
  EXPECT_DOUBLE_EQ(fsm->imu_data.a.x(), .2);
  EXPECT_DOUBLE_EQ(fsm->imu_data.a.z(), 9.81);
  EXPECT_DOUBLE_EQ(fsm->imu_data.w.y(), -.03);
  EXPECT_LT((input.acceleration - fsm->imu_data.filtered_a).norm(), 1e-12);
  EXPECT_LT(input.attitude.angularDistance(q), 1e-12);
  EXPECT_NEAR(input.rpm.x(), 16000., 1e-9);
}

TEST_F(LioStateInput, DiagnosticsDistinguishRejectedInputFromAcceptedState) {
  ros::Time::setNow(ros::Time(100.));
  nav_msgs::Odometry::Ptr attitude(new nav_msgs::Odometry);
  attitude->header.stamp = ros::Time(99.98);
  attitude->pose.pose.orientation.w = 1.;
  fsm->forceAttitudeOdomCallback(attitude);
  nav_msgs::Odometry::Ptr lio(new nav_msgs::Odometry);
  lio->header.stamp = ros::Time(99.995);
  lio->pose.pose.position.z = .6;
  lio->pose.pose.orientation.w = 1.;
  fsm->odomCallback(lio);
  ros::Time::setNow(ros::Time(100.01));
  lio->header.stamp = ros::Time(100.005);
  lio->pose.pose.position.z = 1.6;
  fsm->odomCallback(lio);
  ros::Time::setNow(ros::Time(100.03));
  fsm->beginInputDiagnostics(1.25);
  const auto& d = fsm->input_diagnostics_;
  EXPECT_NEAR(d.position.z, .6, 1e-9);
  EXPECT_EQ(d.lio_latest_source_stamp, ros::Time(100.005));
  EXPECT_EQ(d.lio_used_source_stamp, ros::Time(99.995));
  EXPECT_NEAR(d.lio_used_age_s, .035, 1e-8);
  EXPECT_TRUE(d.px4_used_source_stamp.isZero());
  EXPECT_TRUE(std::isnan(d.px4_used_age_s));
  EXPECT_NEAR(d.since_lio_accept_s, .03, 1e-8);
  EXPECT_NEAR(d.rejected_hold_s, .02, 1e-8);
  EXPECT_EQ(d.lio_rejected_total, 1U);
  EXPECT_EQ(d.lio_rejected_consecutive, 1U);
  fsm->publishInputDiagnostics(2.0);
  EXPECT_EQ(d.mpc_solve_count, 0U);
  EXPECT_FALSE(d.mpc_solve_success);
  EXPECT_DOUBLE_EQ(d.mpc_exec_wall_ms, 0.0);
  EXPECT_DOUBLE_EQ(d.input_dispatch_ms, 1.25);
  ros::Time::setNow(ros::Time(100.04));
  lio->header.stamp = ros::Time(100.035);
  lio->pose.pose.position.z = .6;
  fsm->odomCallback(lio); // returning from the spike is also a raw-sample jump
  fsm->beginInputDiagnostics(0.);
  EXPECT_EQ(d.lio_rejected_consecutive, 2U);
  ros::Time::setNow(ros::Time(100.05));
  lio->header.stamp = ros::Time(100.045);
  fsm->odomCallback(lio);
  fsm->beginInputDiagnostics(0.);
  EXPECT_EQ(d.lio_rejected_consecutive, 0U);
  EXPECT_DOUBLE_EQ(d.rejected_hold_s, 0.);
  EXPECT_EQ(d.lio_rejected_total, 2U);
  EXPECT_EQ(d.lio_used_source_stamp, ros::Time(100.045));
}

TEST_F(LioStateInput, DiagnosticsReportMissingInputAndOnlyCurrentCycleSolves) {
  ros::Time::setNow(ros::Time(100.));
  fsm->beginInputDiagnostics(0.);
  EXPECT_TRUE(std::isnan(fsm->input_diagnostics_.lio_used_age_s));
  EXPECT_TRUE(std::isnan(fsm->input_diagnostics_.px4_used_age_s));
  fsm->odom_data.v.setZero();
  controller->setHoverReference(fsm->odom_data.p, 0.);
  Eigen::Matrix<real_t, kStateSize, kSamples + 1> predicted;
  Eigen::Matrix<real_t, kInputSize, kSamples> inputs;
  controller->execMPC(LioStateInputTestAccess::state(*fsm), predicted, inputs);
  fsm->publishInputDiagnostics(2.);
  EXPECT_EQ(fsm->input_diagnostics_.mpc_solve_count, 1U);
  EXPECT_GT(fsm->input_diagnostics_.mpc_exec_wall_ms, 0.);
  fsm->beginInputDiagnostics(0.);
  fsm->publishInputDiagnostics(1.);
  EXPECT_EQ(fsm->input_diagnostics_.mpc_solve_count, 0U);
  EXPECT_DOUBLE_EQ(fsm->input_diagnostics_.mpc_exec_wall_ms, 0.);
}

TEST_F(LioStateInput, InputDiagnosticsReachRosSubscriber) {
  bool received = false;
  payload_mpc_controller::InputDiagnostics recorded;
  auto subscriber = nh.subscribe<payload_mpc_controller::InputDiagnostics>(
      "mpc/input_diagnostics", 10,
      [&](const payload_mpc_controller::InputDiagnostics::ConstPtr& message) {
        recorded = *message; received = true;
      });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (fsm->input_diagnostics_pub_.getNumSubscribers() == 0 && ros::WallTime::now() < deadline)
    ros::WallDuration(.01).sleep();
  ASSERT_GT(fsm->input_diagnostics_pub_.getNumSubscribers(), 0U);
  ros::Time::setNow(ros::Time(100.));
  fsm->beginInputDiagnostics(.7);
  fsm->publishInputDiagnostics(2.3);
  while (!received && ros::WallTime::now() < deadline) {
    ros::spinOnce(); ros::WallDuration(.01).sleep();
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(recorded.header.stamp, ros::Time(100.));
  EXPECT_DOUBLE_EQ(recorded.input_dispatch_ms, .7);
  EXPECT_DOUBLE_EQ(recorded.process_wall_ms, 2.3);
  EXPECT_EQ(recorded.mpc_solve_count, 0U);
}
