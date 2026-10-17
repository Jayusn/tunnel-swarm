// 在独立ROS master下直接验证跟随器的入口/FIFO放行与间距退让，不启动控制器。
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <bspline/Bspline.h>
#include <bspline/non_uniform_bspline.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ldop/DynamicObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

// 只在测试翻译单元中访问状态；生产节点不增加测试接口。
#define private public
#define main follower_node_main_for_test
#include "../src/leader_safe_path_follower.cpp"
#undef main
#undef private

class FifoEntrySpacing : public ::testing::Test {
 protected:
  void SetUp() override {
    follower.reset(new LeaderSafePathFollower);
    follower->timer_.stop();
    ASSERT_DOUBLE_EQ(follower->min_separation_, 1.0);
    ASSERT_DOUBLE_EQ(follower->separation_release_distance_, 1.0);
    follower->simple_segment_endpoint_following_ = true;
    follower->relay_arrive_radius_ = 0.05;
    follower->follower_alignment_x_ = 0.0;
    follower->follower_alignment_y_ = 0.0;
    follower->have_leader_odom_ = true;
    follower->have_follower_odom_ = true;
    follower->follower_odom_.pose.pose.position.z = 0.6;
    follower->follower_odom_.header.frame_id = "test_local";
    follower->confirmed_door_.position.z = 0.6;
    follower->have_confirmed_door_ = true;
    setLeader(2.0);
    sink = nh.subscribe<geometry_msgs::PoseStamped>(
        "/UAV1/planning/goal", 10,
        +[](const geometry_msgs::PoseStamped::ConstPtr&) {});
    entry_sink = nh.subscribe<geometry_msgs::PoseStamped>(
        "/UAV1/planning/goal/entry", 10,
        [this](const geometry_msgs::PoseStamped::ConstPtr&) { ++entry_goals; });
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while ((follower->diff_goal_pub_.getNumSubscribers() == 0 ||
            follower->diff_entry_goal_pub_.getNumSubscribers() == 0) &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(follower->diff_goal_pub_.getNumSubscribers(), 0U);
    ASSERT_GT(follower->diff_entry_goal_pub_.getNumSubscribers(), 0U);
  }

  void setLeader(double x) {
    follower->leader_odom_.pose.pose.position.x = x;
    follower->leader_odom_stamp_ = ros::Time::now();
  }

  void queuePoint(double x, bool door = false) {
    RoutePoint point;
    point.position.x = x;
    point.position.z = 0.6;
    follower->appendRelayWaypoint(point, door ? "DOOR" : "SEGMENT_ENDPOINT");
  }

  void trackSegment(double x, int id) {
    bspline::Bspline::Ptr trajectory(new bspline::Bspline);
    trajectory->order = 3;
    trajectory->traj_id = id;
    trajectory->start_time = ros::Time::now();
    geometry_msgs::Point endpoint;
    endpoint.x = x;
    endpoint.z = 0.6;
    trajectory->pos_pts.assign(4, endpoint);
    for (int i = 0; i < 8; ++i) trajectory->knots.push_back(i);
    follower->leaderTrajectoryCallback(trajectory);
  }

  void reachSegment(double x, double speed) {
    setLeader(x);
    follower->leader_route_progress_ = x;
    RoutePoint actual;
    actual.position = follower->leader_odom_.pose.pose.position;
    actual.position.z = 0.6;
    geometry_msgs::Twist twist;
    twist.linear.x = speed;
    follower->confirmPendingLeaderSegmentEndpoint(actual, twist, ros::Time::now());
  }

  ros::NodeHandle nh;
  ros::Subscriber sink, entry_sink;
  int entry_goals{0};
  std::unique_ptr<LeaderSafePathFollower> follower;
};

TEST_F(FifoEntrySpacing, OrdinaryPointFilteringAlwaysUsesLastRetainedPoint) {
  RoutePoint point;
  point.position.x = 0.4;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.y = 0.4;
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 0.6;
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 0.8;
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 0.9;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  ASSERT_EQ(follower->relay_waypoints_.size(), 2U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_[0].position.x, 0.4);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_[0].position.y, 0.0);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_[1].position.x, 0.9);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_[1].position.y, 0.4);
}

TEST_F(FifoEntrySpacing, OrdinaryPointSpacingSurvivesConsumptionAndIncludesBoundary) {
  queuePoint(1.0);
  follower->consumeSimpleRelayFront();
  ASSERT_TRUE(follower->relay_waypoints_.empty());
  RoutePoint point;
  point.position.x = 1.499;
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 1.5;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 1.6;
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "INTERNAL"));
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
}

TEST_F(FifoEntrySpacing, ConfirmedLeaderEndpointsApplyOrdinaryPointFilter) {
  follower->door_waypoint_released_ = true;
  trackSegment(1.0, 1);
  reachSegment(1.0, 0.5);
  trackSegment(1.3, 2);
  reachSegment(1.3, 0.5);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_FALSE(follower->pending_leader_segment_endpoint_valid_);
  trackSegment(1.6, 3);
  reachSegment(1.6, 0.5);
  ASSERT_EQ(follower->relay_waypoints_.size(), 2U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.back().position.x, 1.6);
}

TEST_F(FifoEntrySpacing, EitherAxisAtHalfMeterRetainsOrdinaryPoint) {
  RoutePoint point;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 0.4;
  point.position.y = 0.4;
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 0.0;
  point.position.y = -0.5;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = -0.5;
  EXPECT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  EXPECT_EQ(follower->relay_waypoints_.size(), 3U);
}

TEST_F(FifoEntrySpacing, SpecialPointsDoNotParticipateInOrdinaryPointFiltering) {
  RoutePoint point;
  point.position.x = 1.0;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "DOOR"));
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 1.1;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "EXIT"));
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "OUTSIDE_WAIT"));
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "TERMINAL"));
  EXPECT_FALSE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  point.position.x = 1.5;
  EXPECT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));
  EXPECT_EQ(follower->relay_waypoints_.size(), 6U);
}

TEST_F(FifoEntrySpacing, DoorReleasesAtPointSevenWithoutOneMeterSpacing) {
  follower->follower_odom_.pose.pose.position.x = -0.1;
  setLeader(0.69);
  EXPECT_FALSE(follower->tryReleaseDoorWaypointFromLeaderPosition(
      follower->leader_odom_.pose.pose.position));
  setLeader(0.7);
  ASSERT_TRUE(follower->tryReleaseDoorWaypointFromLeaderPosition(
      follower->leader_odom_.pose.pose.position));
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_TRUE(follower->relay_waypoints_.front().is_door);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_separation_hold_active_);
}

TEST_F(FifoEntrySpacing, DoorUsesJointTwoPiecePlanningAndNormalPointUsesDefaultTopic) {
  follower->follower_odom_.pose.pose.position.x = -2.0;
  setLeader(0.7);
  ASSERT_TRUE(follower->tryReleaseDoorWaypointFromLeaderPosition(
      follower->leader_odom_.pose.pose.position));
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ros::WallDuration(0.05).sleep();
  ros::spinOnce();
  EXPECT_EQ(entry_goals, 1);
  EXPECT_TRUE(follower->diff_goal_published_);
  follower->follower_odom_.pose.pose.position.x = 0.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->simple_door_hover_active_);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ros::WallDuration(0.05).sleep();
  ros::spinOnce();
  EXPECT_EQ(entry_goals, 1);
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, NormalPointWaitsForOneMeterLeaderClearance) {
  queuePoint(1.0);
  setLeader(1.99);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(2.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, MovingLeaderArrivalQueuesPointImmediately) {
  follower->door_waypoint_released_ = true;
  trackSegment(1.0, 1);
  reachSegment(0.95, 0.5);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_NEAR(follower->relay_waypoints_.front().position.x, 0.95, 1e-9);
  EXPECT_FALSE(follower->pending_leader_segment_endpoint_valid_);
}

TEST_F(FifoEntrySpacing, UnreachedPlannedEndpointDoesNotEnterFifo) {
  follower->door_waypoint_released_ = true;
  trackSegment(2.0, 1);
  reachSegment(1.7, 0.0);
  EXPECT_TRUE(follower->pending_leader_segment_endpoint_valid_);
  EXPECT_TRUE(follower->relay_waypoints_.empty());
}

TEST_F(FifoEntrySpacing, NewTrajectoryReplacesUnreachedCandidate) {
  follower->door_waypoint_released_ = true;
  trackSegment(1.0, 1);
  trackSegment(2.0, 2);
  reachSegment(1.0, 0.5);
  ASSERT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_EQ(follower->pending_leader_segment_trajectory_id_, 2);
  reachSegment(2.0, 0.5);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_NEAR(follower->relay_waypoints_.front().position.x, 2.0, 1e-9);
}

TEST_F(FifoEntrySpacing, ContinuousNextSegmentKeepsAlreadyReachedPoint) {
  follower->door_waypoint_released_ = true;
  trackSegment(1.0, 1);
  reachSegment(0.95, 0.5);
  trackSegment(2.0, 2);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  reachSegment(1.95, 0.5);
  ASSERT_EQ(follower->relay_waypoints_.size(), 2U);
  EXPECT_NEAR(follower->relay_waypoints_[0].position.x, 0.95, 1e-9);
  EXPECT_NEAR(follower->relay_waypoints_[1].position.x, 1.95, 1e-9);
}

TEST_F(FifoEntrySpacing, MovingArrivalStillWaitsForLeaderToClearOneMeter) {
  follower->door_waypoint_released_ = true;
  trackSegment(1.0, 1);
  reachSegment(1.0, 0.5);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  setLeader(1.99);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(2.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, LegacyModeKeepsItsStopConfirmation) {
  follower->simple_segment_endpoint_following_ = false;
  trackSegment(1.0, 1);
  reachSegment(1.0, 0.5);
  EXPECT_TRUE(follower->leader_segment_endpoints_.empty());
  reachSegment(1.0, 0.0);
  EXPECT_TRUE(follower->leader_segment_endpoints_.empty());
  follower->leader_segment_endpoint_dwell_start_ = ros::Time::now() -
      ros::Duration(follower->leader_segment_endpoint_dwell_ + 0.01);
  reachSegment(1.0, 0.0);
  EXPECT_EQ(follower->leader_segment_endpoints_.size(), 1U);
  EXPECT_TRUE(follower->relay_waypoints_.empty());
}

TEST_F(FifoEntrySpacing, DoorArrivalAndHoverIgnoreOneMeterSpacing) {
  queuePoint(0.0, true);
  setLeader(0.8);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_TRUE(follower->simple_door_hover_active_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_separation_hold_active_);
  EXPECT_FALSE(follower->separation_recovery_active_);
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, EntryFinishesDecelerationBeforeEndpointHold) {
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = -0.3;
  queuePoint(0.0, true);
  setLeader(1.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_TRUE(follower->diff_goal_published_);
  follower->follower_odom_.pose.pose.position.x = 0.01;
  follower->follower_horizontal_speed_ = 0.5;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->follower_horizontal_speed_ = 0.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, DoorHoverDoesNotRetreatAtPointSix) {
  follower->simple_door_hover_active_ = true;
  setLeader(0.6);
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
  EXPECT_FALSE(follower->separation_recovery_active_);
}

TEST_F(FifoEntrySpacing, DoorHoverRetreatsBelowPointSixAndStopsAtPointSeven) {
  follower->simple_door_hover_active_ = true;
  setLeader(0.59);
  EXPECT_TRUE(follower->handleSimpleDiffPlannerExecution(ros::Time::now()));
  EXPECT_TRUE(follower->separation_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_LT(follower->separation_recovery_goal_local_.x, 0.0);
  setLeader(0.7);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->separation_recovery_active_);
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, NormalSpacingHoldsButDoesNotRetreatAtPointSix) {
  queuePoint(1.0);
  setLeader(0.8);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_separation_hold_active_);
  EXPECT_FALSE(follower->separation_recovery_active_);
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(0.6);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->separation_recovery_active_);
}

TEST_F(FifoEntrySpacing, NormalRetreatDoesNotRequireLeaderBacktrackLatch) {
  queuePoint(1.0);
  setLeader(0.59);
  ASSERT_FALSE(follower->relay_route_paused_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->separation_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_LT(follower->separation_recovery_goal_local_.x, 0.0);
  EXPECT_EQ(follower->relay_waypoints_.size(), 1U);
  setLeader(0.8);
  EXPECT_TRUE(follower->handleDiffSeparationSafety(ros::Time::now()));
  EXPECT_TRUE(follower->separation_recovery_active_);
  setLeader(1.0);
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
  EXPECT_FALSE(follower->separation_recovery_active_);
}

TEST_F(FifoEntrySpacing, NextPointStillRequiresOneMeterWhileHoveringAtDoor) {
  follower->simple_door_hover_active_ = true;
  queuePoint(2.0);
  setLeader(0.8);  // 离目标1.2m，但离悬停后机只有0.8m。
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_TRUE(follower->simple_door_hover_active_);
  EXPECT_FALSE(follower->separation_recovery_active_);
  setLeader(3.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->simple_door_hover_active_);
}

TEST_F(FifoEntrySpacing, RetreatResponseUnlocksOnlyMatchingRetreatGoal) {
  follower->simple_door_hover_active_ = true;
  setLeader(0.59);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->separation_recovery_goal_valid_);
  std_msgs::String::Ptr status(new std_msgs::String);
  std::ostringstream text;
  text << "TRAJECTORY_PUBLISHED goal_stamp_ns="
       << follower->diff_active_goal_stamp_ns_ << " "
       << follower->separation_recovery_goal_local_.x << " "
       << follower->separation_recovery_goal_local_.y << " "
       << follower->separation_recovery_goal_local_.z;
  status->data = text.str();
  follower->diffStatusCallback(status);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  EXPECT_TRUE(follower->separation_recovery_active_);
  EXPECT_TRUE(follower->diff_accepted_goal_valid_);
  EXPECT_TRUE(follower->relay_waypoints_.empty());
}

TEST_F(FifoEntrySpacing, OrdinaryArrivalContinuesToNextPointWithoutHold) {
  follower->door_waypoint_released_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = 0.65;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, DistantTurnedEndpointIsRetainedAndSentForPlanning) {
  follower->door_waypoint_released_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_alignment_x_ = -1.2;
  follower->follower_odom_.pose.pose.position.x = 3.79;
  follower->follower_odom_.pose.pose.position.y = 0.117;
  setLeader(10.0);
  RoutePoint point;
  point.position.x = 5.80;
  point.position.y = -3.13;
  point.position.z = 0.6;
  point.yaw = M_PI;
  ASSERT_TRUE(follower->appendRelayWaypoint(point, "SEGMENT_ENDPOINT"));

  follower->handleSimpleDiffPlannerExecution(ros::Time::now());

  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_EQ(follower->diff_goal_index_, 0U);
}

TEST_F(FifoEntrySpacing, HeadingOvershootOutsideArrivalRadiusKeepsEndpoint) {
  follower->door_waypoint_released_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = 2.0;
  setLeader(4.0);
  queuePoint(1.0);

  follower->handleSimpleDiffPlannerExecution(ros::Time::now());

  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, OrdinaryPointRequiresHeightArrivalBeforeConsumption) {
  follower->door_waypoint_released_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = 1.1;
  follower->follower_odom_.pose.pose.position.z = 0.2;
  setLeader(4.0);
  queuePoint(1.0);

  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_TRUE(follower->diff_goal_published_);

  follower->follower_odom_.pose.pose.position.z = 0.6;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, EmptyQueueLetsLastOrdinaryTrajectoryFinish) {
  follower->door_waypoint_released_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = 0.65;
  queuePoint(1.0);
  setLeader(3.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->relay_waypoints_.empty());
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, UnclearedNextPointDoesNotInterruptPreviousSegment) {
  follower->door_waypoint_released_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = 0.65;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(2.5);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  setLeader(3.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  setLeader(2.5);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, SlowPlanningResponseStartsFreshCommandWait) {
  queuePoint(2.0);
  setLeader(4.0);
  follower->setDiffWaitPositionHold(true, "waiting for trajectory");
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  follower->diff_goal_publish_stamp_ = ros::Time::now() - ros::Duration(3.0);
  std_msgs::String::Ptr status(new std_msgs::String);
  status->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" +
      std::to_string(follower->diff_active_goal_stamp_ns_ - 1) + " 2 0 0.6";
  follower->diffStatusCallback(status);
  EXPECT_TRUE(follower->diff_trajectory_response_stamp_.isZero());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  status->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" +
      std::to_string(follower->diff_active_goal_stamp_ns_) + " 2 0 0.6";
  follower->diffStatusCallback(status);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->diff_trajectory_response_stamp_ = ros::Time::now() -
      ros::Duration(follower->diff_command_stale_timeout_ + 0.1);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, DoorUsesIndependentRelaxedSpeedLimits) {
  queuePoint(0.0, true);
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_odom_.pose.pose.position.x = -0.1;
  follower->follower_horizontal_speed_ = 0.151;
  follower->follower_vertical_speed_ = 0.12;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  follower->follower_horizontal_speed_ = 0.15;
  follower->follower_vertical_speed_ = 0.121;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  follower->follower_vertical_speed_ = 0.12;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_TRUE(follower->simple_door_hover_active_);
}

TEST_F(FifoEntrySpacing, RelaxedDoorSpeedsStillRequireFifteenCentimeterDistance) {
  queuePoint(0.0, true);
  follower->relay_arrive_radius_ = 0.4;
  follower->follower_horizontal_speed_ = 0.15;
  follower->follower_vertical_speed_ = 0.12;
  follower->follower_odom_.pose.pose.position.x = -0.151;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  follower->follower_odom_.pose.pose.position.x = -0.15;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
}

TEST_F(FifoEntrySpacing, TerminalArrivalKeepsOriginalStrictSpeedLimits) {
  queuePoint(0.0);
  follower->terminal_mode_active_ = true;
  follower->terminal_waypoint_index_ = 0;
  follower->relay_arrive_dwell_ = 0.0;
  follower->follower_horizontal_speed_ = 0.15;
  follower->follower_vertical_speed_ = 0.12;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->follower_landing_requested_);
  follower->follower_horizontal_speed_ = 0.10;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->follower_landing_requested_);
  follower->follower_vertical_speed_ = 0.08;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->follower_landing_requested_);
}

TEST_F(FifoEntrySpacing, ExitYawStartsAfterEarlierPointsAndSurvivesFifoConsumption) {
  queuePoint(1.0);
  queuePoint(3.0);
  queuePoint(4.0);
  follower->have_final_exit_ = true;
  follower->confirmed_exit_.yaw = 1.2;
  follower->exit_waypoint_released_ = true;
  follower->exit_waypoint_index_ = 1;
  follower->outside_wait_waypoint_index_ = 2;
  EXPECT_FALSE(follower->useLockedExitYaw());
  follower->consumeSimpleRelayFront();
  EXPECT_FALSE(follower->exit_yaw_locked_);
  EXPECT_TRUE(follower->useLockedExitYaw());
  follower->consumeSimpleRelayFront();
  EXPECT_EQ(follower->exit_waypoint_index_, std::numeric_limits<std::size_t>::max());
  EXPECT_TRUE(follower->exit_yaw_locked_);
  EXPECT_TRUE(follower->useLockedExitYaw());
  follower->consumeSimpleRelayFront();
  EXPECT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_TRUE(follower->useLockedExitYaw());
}

TEST_F(FifoEntrySpacing, ExitYawTransformsAndIgnoresLaterLeaderHeading) {
  follower->enable_search_landing_ = true;
  geometry_msgs::PoseStamped::Ptr exit(new geometry_msgs::PoseStamped);
  exit->pose.orientation.z = std::sin(-3.0 / 2.0);
  exit->pose.orientation.w = std::cos(-3.0 / 2.0);
  follower->finalExitPoseCallback(exit);
  follower->follower_alignment_yaw_ = 3.0;
  follower->exit_waypoint_released_ = true;
  follower->exit_waypoint_index_ = 0;
  double yaw = -100.0;
  int received = 0;
  ros::Subscriber yaw_sink = nh.subscribe<quadrotor_msgs::PositionCommand>(
      "/UAV1/planning/yaw", 10,
      [&](const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        yaw = msg->yaw;
        EXPECT_DOUBLE_EQ(msg->yaw_dot, 0.0);
        ++received;
      });
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (follower->diff_yaw_pub_.getNumSubscribers() == 0 &&
         ros::WallTime::now() < deadline) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(follower->diff_yaw_pub_.getNumSubscribers(), 0U);
  follower->publishLockedExitYaw();
  ros::WallDuration(0.05).sleep();
  ros::spinOnce();
  ASSERT_GT(received, 0);
  EXPECT_NEAR(yaw, std::atan2(std::sin(-6.0), std::cos(-6.0)), 1e-9);
  exit->pose.orientation.z = std::sin(0.8);
  exit->pose.orientation.w = std::cos(0.8);
  follower->finalExitPoseCallback(exit);
  follower->leader_odom_.pose.pose.orientation = exit->pose.orientation;
  const int before = received;
  follower->publishLockedExitYaw();
  ros::WallDuration(0.05).sleep();
  ros::spinOnce();
  EXPECT_GT(received, before);
  EXPECT_NEAR(yaw, std::atan2(std::sin(-6.0), std::cos(-6.0)), 1e-9);
}

TEST_F(FifoEntrySpacing, ExitFifoGoalUsesLeaderLockedYaw) {
  queuePoint(1.0);
  follower->relay_waypoints_[0].yaw = -0.5;
  follower->have_final_exit_ = true;
  follower->confirmed_exit_.yaw = 1.2;
  follower->exit_waypoint_released_ = true;
  follower->exit_waypoint_index_ = 0;
  double goal_yaw = -100.0;
  ros::Subscriber goal_sink = nh.subscribe<geometry_msgs::PoseStamped>(
      "/UAV1/planning/goal", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_yaw = yawFromQuaternion(msg->pose.orientation);
      });
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_NEAR(goal_yaw, 1.2, 1e-9);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fifo_entry_spacing",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
