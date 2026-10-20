#include <gtest/gtest.h>
#include <memory>
#include <limits>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "mpc_params.h"
#include "mpc_input.h"
#include "mpc_controller.h"
#include "odom_spike_guard.h"
#include "recovery_control.h"
#include "force_observer_input_synchronizer.h"
#include "disturbance_slew_limiter.h"
#include "vertical_disturbance_low_pass.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ESCStatus.h>
#include "multi_optimization_based_force_estimator.hpp"

// 生产代码不增加测试接口；只在本翻译单元检查航点与轨迹的绑定。
#define private public
#include "mpc_fsm.h"
#undef private

using namespace PayloadMPC;

class WaypointYaw : public ::testing::Test {
 protected:
  void SetUp() override {
    params.config_from_ros_handle(nh);
    params.use_fix_yaw_ = false;
    params.use_trajectory_ending_pos_ = true;
    controller.reset(new MpcController(params));
    fsm.reset(new MPCFSM(nh, params, *controller));
    controller->waitForPreparation();
  }

  void TearDown() override { ros::Time::init(); }

  Trajectory line(const Eigen::Vector3d& velocity, double duration = 8.0) {
    Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
    coefficients.col(4) = velocity;
    coefficients.col(5) = Eigen::Vector3d(0.0, 0.0, 0.6);
    Trajectory trajectory;
    trajectory.emplace_back(duration, coefficients);
    return trajectory;
  }

  double yaw(int column = 0) const {
    const auto& s = controller->reference_states_;
    const Eigen::Quaterniond q(s(kOriW, column), s(kOriX, column),
                              s(kOriY, column), s(kOriZ, column));
    return std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                     1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
  }

  geometry_msgs::PoseStamped::Ptr goal(const Eigen::Vector3d& position,
                                     double heading, unsigned stamp = 1000) {
    geometry_msgs::PoseStamped::Ptr msg(new geometry_msgs::PoseStamped);
    msg->header.stamp = ros::Time(stamp);
    msg->pose.position.x = position.x();
    msg->pose.position.y = position.y();
    msg->pose.position.z = position.z();
    msg->pose.orientation.w = std::cos(heading/2.0);
    msg->pose.orientation.z = std::sin(heading/2.0);
    return msg;
  }

  oneTraj_Data_t trajectoryInfo() {
    oneTraj_Data_t info;
    info.traj = line(Eigen::Vector3d(0.3, 0.0, 0.0));
    info.traj_start_time = ros::Time(1001);
    info.traj_end_time = ros::Time(1009);
    return info;
  }

  void prepareStop(const oneTraj_Data_t& info) {
    auto& data = fsm->trajectory_data;
    data.traj_queue.push_back(info);
    data.total_traj_start_time = info.traj_start_time;
    data.total_traj_end_time = info.traj_end_time;
    data.exec_traj = 1;
    data.last_end_position = info.traj.getJuncPos(info.traj.getPieceNum());
    data.last_end_position_valid = true;
    fsm->odom_data.p = data.last_end_position;
    fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
    fsm->est_state_.setZero();
    fsm->est_state_.head<3>() = data.last_end_position.cast<real_t>();
    fsm->est_state_(kOriW) = 1.0;
    fsm->exec_traj_state_ = MPCFSM::POLY_TRAJ;
    fsm->rmse_cnt_ = 1;
    ASSERT_TRUE(controller->resetForHover(fsm->est_state_, data.last_end_position, 0.0));
  }

  ros::NodeHandle nh{"~"};
  MpcParams params;
  std::unique_ptr<MpcController> controller;
  std::unique_ptr<MPCFSM> fsm;
};

TEST_F(WaypointYaw, EarlyTangentThenSmoothEndpointWithoutChangingPositionOrSpeed) {
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d(0.0, 0.0, 0.6), 0.0);
  Trajectory trajectory = line(Eigen::Vector3d(0.3, 0.0, 0.0));
  const double end_yaw = M_PI/2.0;
  double previous = 0.0;
  for (int i = 1; i <= 800; ++i) {
    const double t = i*0.01;
    ros::Time::setNow(ros::Time(1000) + ros::Duration(t));
    controller->setTrajectoyReference(trajectory, t, 0.0, &end_yaw);
    EXPECT_LE(std::abs(std::remainder(yaw() - previous, 2.0*M_PI)),
              params.max_bodyrate_z_*0.01 + 1e-5);
    if (t <= 3.0) {
      EXPECT_NEAR(yaw(), 0.0, 1e-5);
    }
    previous = yaw();
    EXPECT_TRUE(controller->reference_states_.allFinite());
    EXPECT_TRUE(controller->reference_inputs_.allFinite());
    EXPECT_NEAR(controller->reference_states_(kPosX, 0), 0.3*std::min(t, 7.999), 1e-5);
    EXPECT_NEAR(controller->reference_states_(kPosY, 0), 0.0, 1e-5);
    EXPECT_NEAR(controller->reference_states_(kVelX, 0), t <= 7.999 ? 0.3 : 0.0, 1e-5);
  }
  EXPECT_NEAR(yaw(), end_yaw, 1e-3);
}

TEST_F(WaypointYaw, NextLegReturnsToItsOwnMotionHeading) {
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.0);
  Trajectory first = line(Eigen::Vector3d(0.3, 0.0, 0.0));
  const double end_yaw = M_PI/2.0;
  for (int i = 1; i <= 800; ++i) {
    ros::Time::setNow(ros::Time(1000) + ros::Duration(i*0.01));
    controller->setTrajectoyReference(first, i*0.01, 0.0, &end_yaw);
  }
  Trajectory next = line(Eigen::Vector3d(0.0, -0.3, 0.0));
  for (int i = 1; i <= 700; ++i) {
    ros::Time::setNow(ros::Time(1008) + ros::Duration(i*0.01));
    controller->setTrajectoyReference(next, i*0.01, end_yaw);
  }
  EXPECT_NEAR(yaw(), -M_PI/2.0, 1e-5);
}

TEST_F(WaypointYaw, WrappedEndpointUsesShortRotation) {
  const double start = 179.0*M_PI/180.0, end = -179.0*M_PI/180.0;
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d::Zero(), start);
  Trajectory trajectory = line(Eigen::Vector3d(0.3*std::cos(start), 0.3*std::sin(start), 0.0));
  for (int i = 1; i <= 800; ++i) {
    ros::Time::setNow(ros::Time(1000) + ros::Duration(i*0.01));
    controller->setTrajectoyReference(trajectory, i*0.01, start, &end);
    EXPECT_LT(std::abs(std::remainder(yaw() - start, 2.0*M_PI)), 3.0*M_PI/180.0);
  }
  EXPECT_NEAR(std::remainder(yaw() - end, 2.0*M_PI), 0.0, 1e-3);
}

TEST_F(WaypointYaw, OnlyMatchingCurrentEndpointReceivesYaw) {
  oneTraj_Data_t info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  fsm->waypointGoalCallback(goal(endpoint, 1.2));
  ASSERT_NE(fsm->trajectoryEndYaw(info), nullptr);
  EXPECT_DOUBLE_EQ(*fsm->trajectoryEndYaw(info), 1.2);
  oneTraj_Data_t intermediate = info;
  intermediate.traj = line(Eigen::Vector3d(0.15, 0.0, 0.0));
  EXPECT_EQ(fsm->trajectoryEndYaw(intermediate), nullptr);
  oneTraj_Data_t old = info;
  old.traj_start_time = ros::Time(999);
  EXPECT_EQ(fsm->trajectoryEndYaw(old), nullptr);
  fsm->waypointGoalCallback(goal(endpoint, -0.8, 1002));
  EXPECT_EQ(fsm->trajectoryEndYaw(info), nullptr);
  info.traj_start_time = ros::Time(1003);
  ASSERT_NE(fsm->trajectoryEndYaw(info), nullptr);
  EXPECT_DOUBLE_EQ(*fsm->trajectoryEndYaw(info), -0.8);
  auto invalid = goal(endpoint, 0.0);
  invalid->pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  fsm->waypointGoalCallback(invalid);
  EXPECT_EQ(fsm->trajectoryEndYaw(info), nullptr);
}

TEST_F(WaypointYaw, NormalFinishKeepsEndpointYawWhileAbortKeepsActualYaw) {
  const auto info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  fsm->waypointGoalCallback(goal(endpoint, 1.2));
  ros::Time::setNow(ros::Time(1010));
  prepareStop(info);
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::HOVER);
  EXPECT_NEAR(fsm->hover_yaw_, 1.2, 1e-5);
  EXPECT_NEAR(yaw(), 1.2, 1e-5);
  EXPECT_TRUE(controller->lastMpcSolveSuccessful());

  fsm->trajectory_data.traj_queue.clear();
  prepareStop(info);
  fsm->trajectory_data.exec_traj = -1;
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::HOVER);
  EXPECT_NEAR(fsm->hover_yaw_, 0.0, 1e-5);
  EXPECT_NEAR(yaw(), 0.0, 1e-5);
  EXPECT_TRUE(controller->lastMpcSolveSuccessful());
}

TEST_F(WaypointYaw, GoalAndEntryTopicsBothBindWaypointYaw) {
  const auto info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  const double normal_velocity = params.max_velocity_xy_;
  for (const std::string topic : {"/test/waypoint_goal", "/test/waypoint_goal/entry",
                                "/test/waypoint_goal"}) {
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(topic, 1);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(pub.getNumSubscribers(), 0U);
    fsm->waypoint_yaw_valid_ = false;
    pub.publish(*goal(endpoint, -1.0));
    while (!fsm->waypoint_yaw_valid_ && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    ASSERT_NE(fsm->trajectoryEndYaw(info), nullptr);
    EXPECT_NEAR(*fsm->trajectoryEndYaw(info), -1.0, 1e-9);
    controller->waitForPreparation();
    const double expected_velocity = topic == "/test/waypoint_goal/entry"
        ? 0.7 : normal_velocity;
    EXPECT_NEAR(params.max_velocity_xy_, expected_velocity, 1e-6);
    for (int stage = 0; stage < kSamples; ++stage) {
      for (int axis = 0; axis < 4; ++axis)
        EXPECT_NEAR(acadoVariables.ubAValues[stage * kVelocitySoftConstraintSize + axis],
                    expected_velocity, 1e-6);
      EXPECT_NEAR(acadoVariables.ubAValues[stage * kVelocitySoftConstraintSize + 4],
                  params.max_velocity_z_, 1e-6);
    }
  }
}

TEST_F(WaypointYaw, InvalidEntryGoalDoesNotChangeControllerVelocityLimit) {
  const double normal_velocity = params.max_velocity_xy_;
  auto message = goal(Eigen::Vector3d(0.0, 0.0, 0.6), 0.0, 0);
  fsm->entryWaypointGoalCallback(message);
  controller->waitForPreparation();
  EXPECT_NEAR(params.max_velocity_xy_, normal_velocity, 1e-6);
  EXPECT_NEAR(acadoVariables.ubAValues[0], normal_velocity, 1e-6);
}

TEST_F(WaypointYaw, RecoveryRestoresVelocityLimitForLatestWaypointType) {
  Eigen::Matrix<real_t, kStateSize, 1> state;
  state.setZero();
  state(kPosZ) = 0.6;
  state(kOriW) = 1.0;
  state(kVelX) = 1.0;
  ASSERT_TRUE(controller->resetForHover(state, Eigen::Vector3d(0.0, 0.0, 0.6), 0.0));
  controller->waitForPreparation();
  ASSERT_TRUE(controller->recoveryVelocityLimitsRelaxed());
  const double relaxed_velocity = acadoVariables.ubAValues[0];
  fsm->entryWaypointGoalCallback(goal(Eigen::Vector3d(1.0, 0.0, 0.6), 0.0));
  EXPECT_NEAR(acadoVariables.ubAValues[0], relaxed_velocity, 1e-6);
  ASSERT_TRUE(controller->restoreNominalVelocityLimits());
  controller->waitForPreparation();
  EXPECT_NEAR(acadoVariables.ubAValues[0], 0.7, 1e-6);
  fsm->waypointGoalCallback(goal(Eigen::Vector3d(2.0, 0.0, 0.6), 0.0));
  controller->waitForPreparation();
  EXPECT_NEAR(acadoVariables.ubAValues[0], fsm->normal_velocity_xy_, 1e-6);
}

TEST_F(WaypointYaw, ExistingExitLockStillHasPriority) {
  ASSERT_TRUE(controller->lockYaw(0.4));
  Trajectory trajectory = line(Eigen::Vector3d(0.3, 0.0, 0.0));
  const double end_yaw = -1.0;
  controller->setTrajectoyReference(trajectory, 7.9, 0.0, &end_yaw);
  for (int i = 0; i <= kSamples; ++i) EXPECT_NEAR(yaw(i), 0.4, 1e-5);
}
