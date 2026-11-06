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

TEST_F(WaypointYaw, RecoveryAbortHoldKeepsActualHeightUntilRecoveryTrajectory) {
  // 18:36:04 录包：恢复轨迹与当前航点不匹配，原锁点Z为0.0358489。
  nh.setParam("test_virtual_ground", 0.05);
  fsm->hover_virtual_ground_param_ = nh.resolveName("test_virtual_ground");
  prepareStop(trajectoryInfo());
  fsm->waypointGoalCallback(goal(Eigen::Vector3d(4.879109, -0.282169, 0.143639), 0.0));
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  ros::Time::setNow(ros::Time(1010));
  fsm->odom_data.rcv_stamp = ros::Time::now();
  const Eigen::Vector3d actual(3.47769451, -0.0162512, 0.0358489);
  fsm->odom_data.p = actual;
  fsm->est_state_.head<3>() = actual.cast<real_t>();
  auto hold = boost::make_shared<std_msgs::Bool>();
  hold->data = true;
  fsm->safetyHoldCallback(hold);
  EXPECT_DOUBLE_EQ(fsm->hover_pose_.x(), actual.x());
  EXPECT_DOUBLE_EQ(fsm->hover_pose_.y(), actual.y());
  EXPECT_NEAR(fsm->hover_pose_.z(), actual.z(), 1e-9);
  fsm->CMD_CTRL_process();
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual.z(), 1e-6);
  EXPECT_TRUE(controller->lastMpcSolveSuccessful());
  fsm->odom_data.p.z() = -0.01;
  fsm->safetyHoldCallback(hold); // 重复锁点不重新取点，也不自行提高目标高度。
  fsm->CMD_CTRL_process();
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual.z(), 1e-6);
  // 明确的脱障轨迹发布并解除HOLD后，按脱障轨迹爬升。
  ros::Time::setNow(ros::Time(1011));
  auto recovery = trajectoryInfo();
  Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
  coefficients.col(5) = actual;
  coefficients(2, 4) = .10;
  recovery.traj.clear(); recovery.traj.emplace_back(4.0, coefficients);
  recovery.traj_start_time = ros::Time(1011);
  recovery.traj_end_time = ros::Time(1015);
  prepareStop(recovery);
  fsm->exec_traj_state_ = MPCFSM::HOVER;
  hold->data = false;
  fsm->safetyHoldCallback(hold);
  ros::Time::setNow(ros::Time(1012));
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::POLY_TRAJ);
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual.z() + .10, 1e-6);
  EXPECT_NEAR(controller->reference_states_(kPosX, 0), actual.x(), 1e-6);
  EXPECT_NEAR(controller->reference_states_(kPosY, 0), actual.y(), 1e-6);
}

TEST_F(WaypointYaw, HoverFloorCoversCompletionPendingAbortAndExistingHover) {
  nh.setParam("test_virtual_ground", 0.05);
  fsm->hover_virtual_ground_param_ = nh.resolveName("test_virtual_ground");
  for (int scenario = 0; scenario < 4; ++scenario) {
    fsm->trajectory_data.traj_queue.clear();
    auto info = trajectoryInfo();
    Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
    coefficients.col(5) = Eigen::Vector3d(1.0, 0.0, 0.04);
    info.traj.clear(); info.traj.emplace_back(8.0, coefficients);
    prepareStop(info);
    fsm->fsm_state = MPCFSM::CMD_CTRL;
    fsm->trajectory_data.replan_pending = false;
    fsm->odom_data.p = Eigen::Vector3d(0.9, 0.0, 0.036);
    if (scenario == 1) fsm->trajectory_data.beginTrackingReplan(ros::Time(1008), false);
    if (scenario == 2) fsm->trajectory_data.exec_traj = -1;
    if (scenario == 3) {
      fsm->trajectory_data.traj_queue.clear();
      fsm->trajectory_data.exec_traj = 0;
      fsm->exec_traj_state_ = MPCFSM::HOVER;
      fsm->hover_pose_ = fsm->odom_data.p;
    }
    ros::Time::setNow(ros::Time(1010));
    fsm->CMD_CTRL_process();
    EXPECT_NEAR(controller->reference_states_(kPosZ, 0), 0.10, 1e-6) << scenario;
    EXPECT_TRUE(controller->lastMpcSolveSuccessful()) << scenario;
  }
}

TEST_F(WaypointYaw, HoverReadsPlannerGroundAndPreservesHigherReference) {
  nh.setParam("test_changed_virtual_ground", 0.20);
  fsm->hover_virtual_ground_param_ = nh.resolveName("test_changed_virtual_ground");
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->hover_pose_ = Eigen::Vector3d(1, 2, 0.036);
  fsm->keepHoverAboveVirtualGround();
  EXPECT_NEAR(fsm->hover_pose_.z(), 0.25, 1e-9);
  fsm->hover_pose_.z() = 0.6;
  fsm->keepHoverAboveVirtualGround();
  EXPECT_DOUBLE_EQ(fsm->hover_pose_.z(), 0.6);
}

TEST_F(WaypointYaw, HoverFloorDoesNotAlterFlightTrajectoryTakeoffOrLanding) {
  nh.setParam("test_virtual_ground", 0.05);
  fsm->hover_virtual_ground_param_ = nh.resolveName("test_virtual_ground");
  auto info = trajectoryInfo();
  Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
  coefficients.col(5) = Eigen::Vector3d(1.0, 0.0, 0.08);
  info.traj.clear(); info.traj.emplace_back(8.0, coefficients);
  prepareStop(info);
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  ros::Time::setNow(ros::Time(1005));
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::POLY_TRAJ);
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), 0.08, 1e-6);
  for (int scenario = 0; scenario < 6; ++scenario) {
    fsm->fsm_state = MPCFSM::CMD_CTRL;
    fsm->rc_land_active_ = fsm->precision_landing_active_ = false;
    fsm->mpc_recovery_active_ = fsm->direct_auto_land_active_ = false;
    if (scenario == 0) fsm->fsm_state = MPCFSM::AUTO_TAKEOFF;
    if (scenario == 1) fsm->fsm_state = MPCFSM::AUTO_LAND;
    if (scenario == 2) fsm->rc_land_active_ = true;
    if (scenario == 3) fsm->precision_landing_active_ = true;
    if (scenario == 4) fsm->mpc_recovery_active_ = true;
    if (scenario == 5) fsm->direct_auto_land_active_ = true;
    fsm->hover_pose_ = Eigen::Vector3d(1, 2, 0.036);
    fsm->keepHoverAboveVirtualGround();
    EXPECT_DOUBLE_EQ(fsm->hover_pose_.z(), 0.036) << scenario;
  }
}

TEST_F(WaypointYaw, TrackingWaitKeepsActualPositionUntilNewTrajectory) {
  // 等待重规划时锁当前位置，不主动从 z=0.016495 升到旧目标 z=0.129884。
  auto info = trajectoryInfo();
  Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
  const Eigen::Vector3d endpoint(4.811481, -0.169411, 0.129884);
  coefficients.col(5) = endpoint;
  info.traj.clear();
  info.traj.emplace_back(8.0, coefficients);
  fsm->waypointGoalCallback(goal(endpoint, 1.2));
  prepareStop(info);
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->trajectory_data.beginTrackingReplan(ros::Time(1008), false);
  const Eigen::Vector3d actual(4.760111, -0.185800, 0.016495);
  fsm->odom_data.p = actual;
  fsm->est_state_.head<3>() = actual.cast<real_t>();
  ros::Time::setNow(ros::Time(1010));

  fsm->CMD_CTRL_process();

  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::HOVER);
  EXPECT_TRUE(fsm->trajectory_data.replan_pending);
  EXPECT_NEAR(fsm->hover_pose_.x(), actual.x(), 1e-9);
  EXPECT_NEAR(fsm->hover_pose_.y(), actual.y(), 1e-9);
  EXPECT_NEAR(fsm->hover_pose_.z(), actual.z(), 1e-9);
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual.z(), 1e-6);
  EXPECT_NEAR(fsm->hover_yaw_, 0.0, 1e-9);
  EXPECT_TRUE(controller->lastMpcSolveSuccessful());
  // 等待期间不重复取点；拿到新轨迹后正常恢复。
  fsm->odom_data.p.z() = -0.01;
  ros::Time::setNow(ros::Time(1011));
  fsm->CMD_CTRL_process();
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual.z(), 1e-6);
  fsm->trajectory_data.traj_queue.clear();
  info.traj_start_time = ros::Time(1012);
  info.traj_end_time = ros::Time(1020);
  prepareStop(info);
  fsm->trajectory_data.replan_pending = false;
  fsm->exec_traj_state_ = MPCFSM::HOVER;
  ros::Time::setNow(ros::Time(1013));
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::POLY_TRAJ);
}

TEST_F(WaypointYaw, TrackingWaitDoesNotApplyUnmatchedEntryOrLandingHeight) {
  const auto info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  for (int scenario = 0; scenario < 7; ++scenario) {
    fsm->trajectory_data.traj_queue.clear();
    fsm->waypointGoalCallback(goal(endpoint, 1.2));
    prepareStop(info);
    fsm->fsm_state = MPCFSM::CMD_CTRL;
    fsm->rc_land_active_ = fsm->precision_landing_active_ = fsm->mpc_recovery_active_ = false;
    fsm->params_.use_trajectory_ending_pos_ = true;
    fsm->trajectory_data.beginTrackingReplan(ros::Time(1008), false);
    if (scenario == 0) fsm->entryWaypointGoalCallback(goal(endpoint, 1.2));
    if (scenario == 1) fsm->trajectory_data.traj_queue.back().traj = line(Eigen::Vector3d(0, 0, 0.05));
    if (scenario == 2) fsm->trajectory_data.exec_traj = -1;
    if (scenario == 3) fsm->rc_land_active_ = true;
    if (scenario == 4) fsm->precision_landing_active_ = true;
    if (scenario == 5) fsm->mpc_recovery_active_ = true;
    if (scenario == 6) fsm->params_.use_trajectory_ending_pos_ = false;
    fsm->odom_data.p = Eigen::Vector3d(2.2, 0.1, 0.2);
    ros::Time::setNow(ros::Time(1010));
    fsm->CMD_CTRL_process();
    EXPECT_TRUE(fsm->hover_pose_.isApprox(fsm->odom_data.p)) << scenario;
  }
}

TEST_F(WaypointYaw, PendingNextGoalRetainsOldEndpointAndYawUntilNewTrajectoryArrives) {
  const auto info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  fsm->waypointGoalCallback(goal(endpoint, 0.3));
  prepareStop(info);
  fsm->odom_data.p = endpoint - Eigen::Vector3d(0.10, 0.05, 0.14);
  fsm->est_state_.head<3>() = fsm->odom_data.p.cast<real_t>();
  const Eigen::Vector3d next_endpoint = endpoint + Eigen::Vector3d(0.2, 0.1, 0.1);

  ros::Time::setNow(ros::Time(1008));
  fsm->waypointGoalCallback(goal(next_endpoint, -0.4, 1008));
  ASSERT_EQ(fsm->trajectory_data.traj_queue.size(), 1U);
  const auto& old = fsm->trajectory_data.traj_queue.front();
  ASSERT_NE(fsm->trajectoryEndYaw(old), nullptr);
  EXPECT_NEAR(*fsm->trajectoryEndYaw(old), 0.3, 1e-9);
  // Repeated requests must not replace the executing trajectory's bound heading.
  fsm->waypointGoalCallback(goal(next_endpoint, -0.4, 1009));
  ASSERT_NE(fsm->trajectoryEndYaw(old), nullptr);
  EXPECT_NEAR(*fsm->trajectoryEndYaw(old), 0.3, 1e-9);
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::POLY_TRAJ);
  EXPECT_NEAR(controller->reference_states_(kPosX, 0), 2.1, 1e-5);

  // The old trajectory ends while planning is pending; hold its full endpoint,
  // not the actual pose with the remaining arrival errors.
  ros::Time::setNow(ros::Time(1010));
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::HOVER);
  EXPECT_TRUE(fsm->hover_pose_.isApprox(endpoint));
  EXPECT_NEAR(fsm->hover_yaw_, 0.3, 1e-9);
  fsm->waypointGoalCallback(goal(next_endpoint, -0.4, 1011));
  ros::Time::setNow(ros::Time(1011));
  fsm->CMD_CTRL_process();
  EXPECT_TRUE(fsm->hover_pose_.isApprox(endpoint));
  EXPECT_NEAR(controller->reference_states_(kPosZ, 0), endpoint.z(), 1e-6);
  EXPECT_NEAR(yaw(), 0.3, 1e-5);

  // Deliver a valid new trajectory through the real trajectory input.
  ros::Time::setNow(ros::Time(1012));
  quadrotor_msgs::PolynomialTraj::Ptr next(new quadrotor_msgs::PolynomialTraj);
  next->header.stamp = ros::Time::now();
  next->action = quadrotor_msgs::PolynomialTraj::ACTION_ADD;
  next->trajectory_id = 2;
  next->trajectory.resize(1);
  auto& piece = next->trajectory.front();
  piece.num_dim = 3;
  piece.num_order = 5;
  piece.duration = 2.0;
  piece.data.assign(18, 0.0);
  for (int axis = 0; axis < 3; ++axis) piece.data[15 + axis] = next_endpoint(axis);
  fsm->trajectory_data.feed(next);
  ASSERT_EQ(fsm->trajectory_data.traj_queue.size(), 1U);
  const auto& ready = fsm->trajectory_data.traj_queue.front();
  ASSERT_NE(fsm->trajectoryEndYaw(ready), nullptr);
  EXPECT_NEAR(*fsm->trajectoryEndYaw(ready), -0.4, 1e-9);
  fsm->CMD_CTRL_process();
  EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::POLY_TRAJ);
  for (int axis = 0; axis < 3; ++axis)
    EXPECT_NEAR(controller->reference_states_(kPosX + axis, 0), next_endpoint(axis), 1e-6);
}

TEST_F(WaypointYaw, PendingGoalDoesNotBindYawToPartialTrajectoryOrOverrideSafetyHold) {
  const auto info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  fsm->waypointGoalCallback(goal(endpoint, 0.3));
  prepareStop(info);
  auto partial = info;
  partial.traj = line(Eigen::Vector3d(0.15, 0.0, 0.0));
  fsm->trajectory_data.traj_queue.push_back(partial);
  fsm->waypointGoalCallback(goal(endpoint + Eigen::Vector3d(0.2, 0.0, 0.2), -0.4, 1008));
  ASSERT_TRUE(fsm->trajectory_data.traj_queue.front().endpoint_yaw_valid);
  EXPECT_EQ(fsm->trajectoryEndYaw(fsm->trajectory_data.traj_queue.back()), nullptr);
  fsm->trajectory_data.traj_queue.pop_back();

  ros::Time::setNow(ros::Time(1008));
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->odom_data.rcv_stamp = ros::Time::now();
  fsm->odom_data.p = endpoint - Eigen::Vector3d(0.1, 0.05, 0.1);
  std_msgs::Bool::Ptr hold(new std_msgs::Bool);
  hold->data = true;
  fsm->safetyHoldCallback(hold);
  EXPECT_TRUE(fsm->safety_hold_active_);
  EXPECT_TRUE(fsm->trajectory_data.traj_queue.empty());
  EXPECT_TRUE(fsm->hover_pose_.isApprox(fsm->odom_data.p));
  EXPECT_NEAR(fsm->hover_yaw_, 0.0, 1e-9);
}

TEST_F(WaypointYaw, SafetyHoldCapturesActualXYZAndDoesNotRelatchOnRepeatedTrue) {
  const auto info = trajectoryInfo();
  const Eigen::Vector3d endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  fsm->waypointGoalCallback(goal(endpoint, 1.2));
  ros::Time::setNow(ros::Time(1010));
  prepareStop(info);
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->CMD_CTRL_process(); // Normal completion retains the full endpoint.
  const double normal_z_speed = params.max_velocity_z_;
  std_msgs::Bool::Ptr hold(new std_msgs::Bool);
  for (double actual_z : {0.091, 0.235, 0.8}) {
    fsm->odom_data.rcv_stamp = ros::Time::now();
    fsm->odom_data.p = Eigen::Vector3d(2.2, 0.1, actual_z);
    fsm->est_state_.head<3>() = fsm->odom_data.p.cast<real_t>();
    hold->data = true;
    fsm->safetyHoldCallback(hold);
    EXPECT_NEAR(fsm->hover_pose_.z(), actual_z, 1e-9);
    EXPECT_NEAR(fsm->hover_pose_.x(), 2.2, 1e-9);
    EXPECT_NEAR(fsm->hover_pose_.y(), 0.1, 1e-9);
    fsm->CMD_CTRL_process();
    EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual_z, 1e-6);
    EXPECT_NEAR(params.max_velocity_z_, normal_z_speed, 1e-6);
    EXPECT_TRUE(controller->lastMpcSolveSuccessful());
    // 同一次HOLD重复true不重新取点；释放后下一次HOLD锁存新的实际位置。
    fsm->odom_data.p.z() -= 0.05;
    fsm->safetyHoldCallback(hold);
    EXPECT_NEAR(fsm->hover_pose_.z(), actual_z, 1e-9);
    hold->data = false;
    fsm->safetyHoldCallback(hold);
    EXPECT_NEAR(params.max_velocity_z_, normal_z_speed, 1e-6);
  }
}

TEST_F(WaypointYaw, ChangedWaypointCannotRaiseOrLowerActiveHold) {
  auto info = trajectoryInfo();
  const auto endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  fsm->waypointGoalCallback(goal(endpoint, 1.2));
  prepareStop(info);
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  ros::Time::setNow(ros::Time(1005));
  fsm->odom_data.rcv_stamp = ros::Time::now();
  const Eigen::Vector3d actual(2.2, .1, .091);
  fsm->odom_data.p = actual;
  fsm->est_state_.head<3>() = actual.cast<real_t>();
  std_msgs::Bool::Ptr hold(new std_msgs::Bool);
  hold->data = true;
  fsm->safetyHoldCallback(hold);
  unsigned stamp = 1006;
  for (double new_height : {.432, .526, .05}) {
    fsm->waypointGoalCallback(goal(Eigen::Vector3d(3, 0, new_height), .3, stamp++));
    fsm->CMD_CTRL_process();
    EXPECT_TRUE(fsm->hover_pose_.isApprox(actual));
    EXPECT_NEAR(controller->reference_states_(kPosZ, 0), actual.z(), 1e-6);
    EXPECT_TRUE(controller->lastMpcSolveSuccessful());
  }
}

TEST_F(WaypointYaw, EntryRecoveryAbortAndLandingDoNotReuseOrdinaryWaypointHeight) {
  auto info = trajectoryInfo();
  const auto endpoint = info.traj.getJuncPos(info.traj.getPieceNum());
  ros::Time::setNow(ros::Time(1005));
  std_msgs::Bool::Ptr hold(new std_msgs::Bool);
  for (int scenario = 0; scenario < 5; ++scenario) {
    fsm->waypointGoalCallback(goal(endpoint, 1.2));
    fsm->trajectory_data.traj_queue.clear();
    prepareStop(info);
    fsm->fsm_state = MPCFSM::CMD_CTRL;
    fsm->precision_landing_active_ = false;
    fsm->rc_land_active_ = false;
    if (scenario == 0) fsm->entryWaypointGoalCallback(goal(endpoint, 1.2));
    if (scenario == 1) {
      fsm->trajectory_data.traj_queue.back().traj = line(Eigen::Vector3d(0.0, 0.0, 0.05));
    }
    if (scenario == 2) fsm->trajectory_data.exec_traj = -1;
    if (scenario == 3) fsm->precision_landing_active_ = true;
    if (scenario == 4) fsm->rc_land_active_ = true;
    fsm->odom_data.rcv_stamp = ros::Time::now();
    fsm->odom_data.p.z() = 0.2;
    hold->data = true;
    fsm->safetyHoldCallback(hold);
    EXPECT_NEAR(fsm->hover_pose_.z(), 0.2, 1e-9);
    hold->data = false;
    fsm->safetyHoldCallback(hold);
  }
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

TEST_F(WaypointYaw, TrackingDeviationDoesNotRequestReplanOrStopTheReference) {
  // 即使旧参数仍留在参数服务器，控制器也不再自行按10/15cm换轨或停止。
  nh.setParam("tracking_replan/enabled", true);
  fsm.reset(new MPCFSM(nh, params, *controller));
  nh.deleteParam("tracking_replan/enabled");
  auto info = trajectoryInfo();
  prepareStop(info);
  const Eigen::Vector3d positions[] = {
      Eigen::Vector3d(.3, .11, .6), // 超过旧重规划阈值。
      Eigen::Vector3d(.3, .16, .6), // 超过旧立即停止阈值。
      Eigen::Vector3d(.05, 0, .6)   // 仍在路径上，但落后当前时刻参考超过15cm。
  };
  double time = 1002.0;
  for (const auto &position : positions) {
    fsm->odom_data.p = position;
    fsm->est_state_.head<3>() = position.cast<real_t>();
    for (int sample = 0; sample < 2; ++sample) {
      ros::Time::setNow(ros::Time(time));
      fsm->CMD_CTRL_process();
      controller->waitForPreparation();
      EXPECT_FALSE(fsm->trajectory_data.replan_pending);
      EXPECT_FALSE(fsm->trajectory_data.replan_stopped);
      EXPECT_EQ(fsm->trajectory_data.replan_request_id, 0U);
      ASSERT_EQ(fsm->trajectory_data.traj_queue.size(), 1U);
      EXPECT_EQ(fsm->exec_traj_state_, MPCFSM::POLY_TRAJ);
      EXPECT_NEAR(controller->reference_states_(kPosX, 0),
                  .3 * (time - 1001.0), 1e-5);
      time += 0.11;
    }
  }
}


TEST_F(WaypointYaw, RelayScanYawChangesOnlyHeadingDuringHoldAndDoesNotLockExit) {
  ros::Time::setNow(ros::Time(1000));
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->safety_hold_active_ = fsm->safety_hold_pose_valid_ = true;
  fsm->hover_pose_ = Eigen::Vector3d(2.0, -1.0, 0.3);
  fsm->hover_yaw_ = 0.0;
  auto command = boost::make_shared<quadrotor_msgs::PositionCommand>();
  command->header.stamp = ros::Time::now();
  command->yaw = M_PI/4;
  fsm->relayScanYawCallback(command);
  EXPECT_NEAR(M_PI/4, fsm->hover_yaw_, 1e-8);
  EXPECT_LT((fsm->hover_pose_ - Eigen::Vector3d(2,-1,0.3)).norm(), 1e-9);
  controller->setHoverReference(fsm->hover_pose_, fsm->hover_yaw_);
  EXPECT_NEAR(M_PI/4, yaw(), 1e-5);
  // 扫描不会永久锁住航向：后续普通参考和出口锁向都仍然可用。
  controller->setHoverReference(fsm->hover_pose_, -0.4);
  EXPECT_NEAR(-0.4, yaw(), 1e-5);
  EXPECT_TRUE(controller->lockYaw(1.2));
  controller->setHoverReference(fsm->hover_pose_, 0.0);
  EXPECT_NEAR(1.2, yaw(), 1e-5);
}

TEST_F(WaypointYaw, RelayScanYawDoesNotOverrideFlightLandingOrRecovery) {
  ros::Time::setNow(ros::Time(1000));
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->hover_yaw_ = 0.0;
  auto command = boost::make_shared<quadrotor_msgs::PositionCommand>();
  command->header.stamp = ros::Time::now(); command->yaw = 0.7;
  fsm->relayScanYawCallback(command);
  EXPECT_DOUBLE_EQ(0.0, fsm->hover_yaw_); // 没有锁点时不能接管轨迹航向。
  fsm->safety_hold_active_ = fsm->safety_hold_pose_valid_ = true;
  fsm->rc_land_active_ = true; fsm->relayScanYawCallback(command);
  EXPECT_DOUBLE_EQ(0.0, fsm->hover_yaw_);
  fsm->rc_land_active_ = false; fsm->precision_landing_active_ = true;
  fsm->relayScanYawCallback(command); EXPECT_DOUBLE_EQ(0.0, fsm->hover_yaw_);
  fsm->precision_landing_active_ = false; fsm->mpc_recovery_active_ = true;
  fsm->relayScanYawCallback(command); EXPECT_DOUBLE_EQ(0.0, fsm->hover_yaw_);
  fsm->mpc_recovery_active_ = false; command->header.stamp = ros::Time(999);
  fsm->relayScanYawCallback(command); EXPECT_DOUBLE_EQ(0.0, fsm->hover_yaw_);
}

TEST_F(WaypointYaw, PolynomialReferencePreservesDerivativesAndLandingDescent) {
  params.waypoint_reference_max_velocity_ = 0.2;
  controller->setExternalForce(Eigen::Vector3d::Zero());
  Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
  coefficients.col(3) = Eigen::Vector3d(0.3, 0.4, 0.2);
  coefficients.col(4) = Eigen::Vector3d(0.4, 0.3, 0.2);
  coefficients.col(5) = Eigen::Vector3d(0., 0., 2.);
  Trajectory trajectory;
  trajectory.emplace_back(8., coefficients);
  controller->setTrajectoyReference(trajectory, 0.2, 0.);
  for (int i = 0; i <= kSamples; ++i) {
    const auto state = controller->reference_states_.col(i);
    const double t = 0.2 + i * controller->getTimeStep();
    EXPECT_LT((state.segment<3>(kPosX).cast<double>() - trajectory.getPos(t)).norm(), 1e-5);
    EXPECT_LT((state.segment<3>(kVelX).cast<double>() - trajectory.getVel(t)).norm(), 1e-5);
    Eigen::Quaterniond q(state(kOriW), state(kOriX), state(kOriY), state(kOriZ));
    const Eigen::Vector3d acceleration = q.normalized() * Eigen::Vector3d::UnitZ() *
        (controller->reference_inputs_(kThrust, i) / params.dyn_params_.mass_q) -
        Eigen::Vector3d(0., 0., params.gravity_);
    EXPECT_LT((acceleration - trajectory.getAcc(t)).norm(), 1e-5);
  }
  ASSERT_TRUE(controller->setPositionCommandReference(Eigen::Vector3d(1., 2., 0.65),
      Eigen::Vector3d(0., 0., -0.25), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 0., 0.));
  for (int i = 0; i <= kSamples; ++i)
    EXPECT_NEAR(controller->reference_states_(kVelZ, i), -0.25, 1e-6);
}

TEST_F(WaypointYaw, BridgeWaypointLimitsRestoreImmediatelyOnPrecisionHandoff) {
  const double original_xy = params.max_velocity_xy_, original_z = params.max_velocity_z_;
  params.waypoint_position_weight_xy_ = 400.0;
  params.waypoint_bodyrate_weight_xy_ = 20.0;
  ASSERT_TRUE(controller->setWaypointTrackingWeights(true));
  params.waypoint_reference_max_velocity_ = 0.2;
  for (bool entry : {false, true}) {
    fsm->handleWaypointGoal(goal(Eigen::Vector3d(3., 0., 2.), 0.), entry);
    controller->waitForPreparation();
    EXPECT_NEAR(params.max_velocity_xy_, 0.2, 1e-6);
    EXPECT_NEAR(params.max_velocity_z_, 0.2, 1e-6);
    EXPECT_NEAR(acadoVariables.ubAValues[0], 0.2, 1e-6);
    EXPECT_NEAR(acadoVariables.ubAValues[4], 0.2, 1e-6);
  }
  fsm->fsm_state = MPCFSM::CMD_CTRL;
  fsm->state_data.current_state.armed = true;
  fsm->state_data.current_state.mode = "OFFBOARD";
  fsm->odom_data.p = Eigen::Vector3d(3., 0., 2.);
  fsm->odom_data.v.setZero();
  fsm->odom_data.q = Eigen::Quaterniond::Identity();
  fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
  fsm->odom_data.rcv_stamp = ros::Time::now();
  fsm->force_attitude_odom_data.rcv_stamp = fsm->odom_data.rcv_stamp;
  auto target = boost::make_shared<mavros_msgs::PositionTarget>();
  target->header.stamp = ros::Time::now();
  target->coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target->type_mask = 2503;
  target->velocity.z = -0.25;
  fsm->landingTargetCallback(target);
  controller->waitForPreparation();
  ASSERT_TRUE(fsm->precision_landing_active_);
  EXPECT_NEAR(params.max_velocity_xy_, original_xy, 1e-6);
  EXPECT_NEAR(params.max_velocity_z_, original_z, 1e-6);
  EXPECT_NEAR(acadoVariables.ubAValues[0], original_xy, 1e-6);
  EXPECT_NEAR(acadoVariables.ubAValues[4], original_z, 1e-6);
  EXPECT_NEAR(acadoVariables.W[0], params.q_gain_.Q_pos_xy, 1e-6);
  EXPECT_NEAR(acadoVariables.W[(kRefSize + 1) * (kCostSize + kRateX)],
              params.r_gain_.R_pitchroll, 1e-6);
}

TEST_F(WaypointYaw, OptionalWaypointWeightsRestoreAllCostsAndSurviveSolverReset) {
  const std::vector<real_t> baseline(acadoVariables.W, acadoVariables.W + ACADO_N * ACADO_NY * ACADO_NY);
  const std::vector<real_t> terminal(acadoVariables.WN, acadoVariables.WN + ACADO_NYN * ACADO_NYN);
  ASSERT_TRUE(controller->setWaypointTrackingWeights(true)); // 多机默认不启用。
  controller->waitForPreparation();
  EXPECT_TRUE(std::equal(baseline.begin(), baseline.end(), acadoVariables.W));
  params.waypoint_position_weight_xy_ = 400.0;
  params.waypoint_bodyrate_weight_xy_ = 20.0;
  for (int cycle = 0; cycle < 2; ++cycle) {
    ASSERT_TRUE(controller->setWaypointTrackingWeights(true));
    controller->waitForPreparation();
    EXPECT_NEAR(acadoVariables.W[0], 400.0, 1e-6);
    EXPECT_NEAR(acadoVariables.W[kRefSize + 1], 400.0, 1e-6);
    EXPECT_NEAR(acadoVariables.W[2 * (kRefSize + 1)], params.q_gain_.Q_pos_z, 1e-6);
    EXPECT_NEAR(acadoVariables.W[(kRefSize + 1) * (kCostSize + kRateX)], 20.0, 1e-6);
    EXPECT_NEAR(acadoVariables.W[(kRefSize + 1) * (kCostSize + kRateY)], 20.0, 1e-6);
    if (cycle == 0) {
      Eigen::Matrix<real_t, kStateSize, 1> state;
      state << 0, 0, 2, 1, 0, 0, 0, 0, 0, 0;
      ASSERT_TRUE(controller->resetForHover(state, Eigen::Vector3d(0, 0, 2), 0));
    } else {
      ASSERT_TRUE(controller->setWaypointTrackingWeights(false));
    }
    controller->waitForPreparation();
    EXPECT_TRUE(std::equal(baseline.begin(), baseline.end(), acadoVariables.W));
    EXPECT_TRUE(std::equal(terminal.begin(), terminal.end(), acadoVariables.WN));
  }
}

TEST_F(WaypointYaw, WaypointWeightsIncreaseCorrectionOfRightwardTrackingError) {
  params.waypoint_position_weight_xy_ = 400.0;
  params.waypoint_bodyrate_weight_xy_ = 20.0;
  Eigen::Matrix<real_t, kStateSize, 1> state;
  const Eigen::Quaterniond attitude(Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitX()));
  state << 0, -0.08, 2, attitude.w(), attitude.x(), attitude.y(), attitude.z(), 0.2, -0.15, 0;
  Eigen::Matrix<real_t, kStateSize, kSamples + 1> predicted;
  Eigen::Matrix<real_t, kInputSize, kSamples> inputs;
  double rates[2];
  for (int active = 0; active < 2; ++active) {
    ASSERT_TRUE(controller->resetForHover(state, Eigen::Vector3d(0, 0, 2), 0));
    ASSERT_TRUE(controller->setWaypointTrackingWeights(active));
    controller->setExternalForce(Eigen::Vector3d(0, 0.6, 0));
    for (int i = 0; i < 8; ++i) {
      ASSERT_TRUE(controller->setPositionCommandReference(Eigen::Vector3d(0, 0, 2),
          Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0));
      ASSERT_TRUE(controller->setWaypointTrackingWeights(active));
      controller->execMPC(state, predicted, inputs);
      controller->waitForPreparation();
      ASSERT_TRUE(controller->lastMpcSolveSuccessful());
    }
    rates[active] = inputs(kRateX, 0);
  }
  EXPECT_LT(rates[0], 0.0); // 航向为0时，负横滚角速度纠正向右偏移。
  EXPECT_LT(rates[1], rates[0] - 0.01);
}

TEST_F(WaypointYaw, TakeoffEndBrakesContinuouslyAndNeverExceedsTarget) {
  fsm->takeoff_start_pose_ = Eigen::Vector3d(0,0,0);
  fsm->takeoff_target_z_ = 1.9;
  params.takeoff_.climb_rate = .15;
  EXPECT_NEAR(fsm->takeoffReferenceZ(12), 1.8, 1e-9); // 默认仍为原线性参考。
  params.takeoff_.slowdown_distance = .30;
  const double braking_start = 1.6/.15, finish = braking_start+4.;
  const double eps = 1e-4;
  auto speed = [&](double t) {
    return (fsm->takeoffReferenceZ(t+eps)-fsm->takeoffReferenceZ(t-eps))/(2*eps);
  };
  EXPECT_NEAR(speed(braking_start), .15, 1e-6);
  EXPECT_NEAR(speed(braking_start+2), .075, 1e-6);
  EXPECT_NEAR(speed(finish), 0, 1e-6);
  EXPECT_DOUBLE_EQ(fsm->takeoffReferenceZ(finish+5),1.9);
  double previous = 0;
  for (double t=0;t<finish+1;t+=.01) {
    const double z=fsm->takeoffReferenceZ(t);
    EXPECT_GE(z,previous);EXPECT_LE(z,1.9);
    previous=z;
  }
  fsm->takeoff_start_pose_.z()=1.8; // 剩余爬升不足30 cm也不越过终点。
  EXPECT_NEAR(fsm->takeoffReferenceZ(.1),1.8149308,1e-5);
  EXPECT_DOUBLE_EQ(fsm->takeoffReferenceZ(3),1.9);
}

TEST_F(WaypointYaw, OptionalModeSwitchPreservesReferenceHeightWithoutMovingXYOrLanding) {
  params.fixed_hover_.enabled=false;
  fsm->odom_data.p=Eigen::Vector3d(2,3,2.05);
  fsm->fsm_state=MPCFSM::AUTO_TAKEOFF;
  fsm->hover_pose_=Eigen::Vector3d(0,0,1.9);
  fsm->update_mode_hover_pose();
  EXPECT_DOUBLE_EQ(fsm->hover_pose_.z(),2.05); // 默认不改变多机模式入口。
  params.takeoff_.preserve_hover_height=true;
  for (auto state : {MPCFSM::AUTO_TAKEOFF,MPCFSM::AUTO_HOVER}) {
    fsm->fsm_state=state;fsm->hover_pose_.z()=1.9;
    fsm->update_mode_hover_pose();
    EXPECT_DOUBLE_EQ(fsm->hover_pose_.z(),1.9);
    EXPECT_DOUBLE_EQ(fsm->hover_pose_.x(),2);
    EXPECT_DOUBLE_EQ(fsm->hover_pose_.y(),3);
  }
  fsm->fsm_state=MPCFSM::AUTO_TAKEOFF;fsm->hover_pose_.z()=.8;
  fsm->update_mode_hover_pose();
  EXPECT_DOUBLE_EQ(fsm->hover_pose_.z(),.8); // 提前切模式不突然拉升至1.9。
  fsm->update_hover_pose(); // 降落和轨迹衔接使用原位置捕获。
  EXPECT_DOUBLE_EQ(fsm->hover_pose_.z(),2.05);
}
