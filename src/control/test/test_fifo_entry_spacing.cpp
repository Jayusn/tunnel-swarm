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
  std::vector<geometry_msgs::PoseStamped> goals;

  void SetUp() override {
    follower.reset(new LeaderSafePathFollower);
    follower->timer_.stop();
    ASSERT_DOUBLE_EQ(follower->ordinary_waypoint_dwell_, 0.0);
    // 默认到达即放行；下方驻留用例仅验证显式启用等待的配置。
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
        [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
          goals.push_back(*msg);
        });
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

  void reportDiffStatus(const std::string& state, uint64_t stamp) {
    std_msgs::String::Ptr status(new std_msgs::String);
    status->data = state + " goal_stamp_ns=" + std::to_string(stamp);
    follower->diffStatusCallback(status);
  }

  void prepareQualifiedPoints() {
    follower->door_waypoint_released_ = true;
    follower->require_waypoint_yaw_ = true;
    follower->relay_arrive_radius_ = 0.4;
    follower->min_separation_ = 1.2;
    follower->waypoint_release_min_separation_ = 1.2;
    follower->separation_release_distance_ = 1.2;
    queuePoint(1.0);
    queuePoint(2.0);
    follower->relay_waypoints_[0].safe_relay_id = 1;
    follower->relay_waypoints_[1].safe_relay_id = 2;
    follower->safe_relay_valid_ids_ = {1, 2};
    follower->safe_relay_live_messages_ = 2;
    follower->safe_relay_received_ = ros::Time::now();
    follower->follower_odom_.pose.pose.orientation.w = 1.0;
    setLeader(4.0);
  }

  void occupyFirstPoint() {
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
    cloud->header.frame_id = "test_local";
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(2);
    sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
    for (int i = 0; i < 2; ++i, ++x, ++y, ++z) {
      *x = 1.0f; *y = i * 0.01f; *z = 0.6f;
    }
    follower->obstacle_check_enabled_ = true;
    follower->follower_cloud_ = cloud;
    follower->cloud_stamp_ = ros::Time::now();
  }

  void reportRecoveryStatus(const std::string& state, uint64_t goal_stamp,
                            uint64_t recovery_stamp) {
    std_msgs::String::Ptr status(new std_msgs::String);
    status->data = state + " goal_stamp_ns=" + std::to_string(goal_stamp) +
                   " recovery_stamp_ns=" + std::to_string(recovery_stamp);
    follower->diffStatusCallback(status);
  }

  uint64_t sendAndConsumeOrdinaryGoal() {
    follower->door_waypoint_released_ = true;
    queuePoint(1.0);
    queuePoint(2.0);
    setLeader(4.0);
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_TRUE(follower->diff_goal_published_);
    const auto stamp = follower->diff_active_goal_stamp_ns_;
    follower->follower_odom_.pose.pose.position.x = 1.0;
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(follower->relay_waypoints_.size(), 1U);
    EXPECT_FALSE(follower->diff_goal_published_);
    return stamp;
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

TEST_F(FifoEntrySpacing, DoorReleasesAtPointSixWithoutOneMeterSpacing) {
  follower->follower_odom_.pose.pose.position.x = -0.1;
  setLeader(0.59);
  EXPECT_FALSE(follower->tryReleaseDoorWaypointFromLeaderPosition(
      follower->leader_odom_.pose.pose.position));
  setLeader(0.6);
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

TEST_F(FifoEntrySpacing, DoorHoverRetreatsBelowPointSixAndStopsAtPointSix) {
  follower->simple_door_hover_active_ = true;
  setLeader(0.59);
  EXPECT_TRUE(follower->handleSimpleDiffPlannerExecution(ros::Time::now()));
  EXPECT_TRUE(follower->separation_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_LT(follower->separation_recovery_goal_local_.x, 0.0);
  setLeader(0.6);
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

TEST_F(FifoEntrySpacing, ConsumedGoalRecoveryPreventsNextGoalUntilFinished) {
  const auto stamp = sendAndConsumeOrdinaryGoal();
  reportDiffStatus("OCCUPIED_RECOVERY_HISTORY", stamp);
  ASSERT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);

  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, stamp);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);

  reportDiffStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_GT(follower->diff_active_goal_stamp_ns_, stamp);
}

TEST_F(FifoEntrySpacing, ConsumedGoalRecoveryAbortRetainsNextGoalAndRetryDelay) {
  const auto stamp = sendAndConsumeOrdinaryGoal();
  reportDiffStatus("OCCUPIED_RECOVERY_LATERAL", stamp);
  ASSERT_TRUE(follower->simple_occupied_recovery_active_);

  reportDiffStatus("OCCUPIED_RECOVERY_ABORTED", stamp);

  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
  EXPECT_GT(follower->simple_diff_retry_not_before_, ros::Time::now());
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, RecoveryFromOlderGoalCannotTakeOverNewGoal) {
  const auto old_stamp = sendAndConsumeOrdinaryGoal();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  const auto new_stamp = follower->diff_active_goal_stamp_ns_;
  ASSERT_GT(new_stamp, old_stamp);

  reportDiffStatus("OCCUPIED_RECOVERY_HISTORY", old_stamp);
  reportDiffStatus("OCCUPIED_RECOVERY_SUCCEEDED", old_stamp);

  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, new_stamp);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
}

TEST_F(FifoEntrySpacing, ActiveGoalRecoveryConsumesOnlyItsOwnUnsafeEndpoint) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("OCCUPIED_RECOVERY_HISTORY", stamp);
  follower->follower_odom_.pose.pose.position.x = 1.0;

  follower->handleSimpleDiffPlannerExecution(ros::Time::now());

  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  reportDiffStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
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

TEST_F(FifoEntrySpacing, WaypointHeadingWaitsForPositionAndYawBeforeNextPoint) {
  follower->require_waypoint_yaw_ = true;
  follower->relay_arrive_radius_ = 0.4;
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  follower->relay_waypoints_[0].yaw = M_PI/2.0;
  follower->relay_waypoints_[1].yaw = -M_PI/2.0;
  setLeader(4.0);
  follower->follower_odom_.pose.pose.position.x = 0.65;
  auto& attitude = follower->follower_odom_.pose.pose.orientation;
  attitude.w = std::cos(M_PI/4.0);
  attitude.z = std::sin(M_PI/4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 2U);

  follower->follower_odom_.pose.pose.position.x = 0.9;
  attitude.w = 1.0;
  attitude.z = 0.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 2U);
  attitude.w = std::cos(M_PI/4.0);
  attitude.z = std::sin(M_PI/4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_NEAR(follower->relay_waypoints_.front().yaw, -M_PI/2.0, 1e-9);
  EXPECT_FALSE(follower->diff_goal_published_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, WaypointHeadingToleranceWrapsAtPi) {
  follower->require_waypoint_yaw_ = true;
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  follower->relay_waypoints_[0].yaw = 179.0*M_PI/180.0;
  setLeader(4.0);
  follower->follower_odom_.pose.pose.position.x = 1.0;
  auto& attitude = follower->follower_odom_.pose.pose.orientation;
  attitude.w = std::cos(158.0*M_PI/360.0);
  attitude.z = std::sin(158.0*M_PI/360.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  attitude.w = std::cos(-179.0*M_PI/360.0);
  attitude.z = std::sin(-179.0*M_PI/360.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
}

TEST_F(FifoEntrySpacing, OrdinaryYawAllowsBothTwentyDegreeBoundaries) {
  follower->require_waypoint_yaw_ = true;
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(5.0);
  for (double sign : {1.0, -1.0}) {
    follower->follower_odom_.pose.pose.position.x = sign > 0 ? 1.0 : 2.0;
    const auto size = follower->relay_waypoints_.size();
    auto& attitude = follower->follower_odom_.pose.pose.orientation;
    attitude.w = std::cos(20.01*M_PI/360.0);
    attitude.z = sign * std::sin(20.01*M_PI/360.0);
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(size, follower->relay_waypoints_.size());
    attitude.w = std::cos(20.0*M_PI/360.0);
    attitude.z = sign * std::sin(20.0*M_PI/360.0);
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(size - 1, follower->relay_waypoints_.size());
  }
}

TEST_F(FifoEntrySpacing, QualifiedOrdinaryPointWithDwellDisabledKeepsFifteenCmWithoutSpeedGate) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  follower->follower_odom_.pose.pose.position.x = 0.65;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size()); // 35cm is not arrival.
  EXPECT_TRUE(follower->relay_arrival_stamp_.isZero());
  follower->follower_odom_.pose.pose.position.x = 0.84;
  follower->follower_horizontal_speed_ = 0.5;
  follower->follower_vertical_speed_ = 0.12;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->follower_odom_.pose.pose.position.z = 0.44; // XY reached, height still 16cm low.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->follower_odom_.pose.pose.position.z = 0.451; // Within 15cm, no dwell required.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_TRUE(follower->relay_arrival_stamp_.isZero()); // No stop or dwell before releasing next.
  EXPECT_FALSE(follower->diff_wait_hold_active_); // Retain original endpoint reference.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_wait_hold_active_); // Planning the next leg must not freeze it.
}

TEST_F(FifoEntrySpacing, OrdinaryDwellWaitsFullTwoSecondsAndKeepsEndpointReference) {
  prepareQualifiedPoints();
  follower->ordinary_waypoint_dwell_ = 2.0;
  const auto now = ros::Time::now();
  follower->handleSimpleDiffPlannerExecution(now);
  const auto first_goal = follower->diff_active_goal_stamp_ns_;
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->follower_horizontal_speed_ = 0.2; // No new low-speed gate.
  follower->handleSimpleDiffPlannerExecution(now);
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  EXPECT_EQ(now, follower->relay_arrival_stamp_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->relay_arrival_stamp_ = now - ros::Duration(1.999);
  follower->handleSimpleDiffPlannerExecution(now);
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  EXPECT_EQ(first_goal, follower->diff_active_goal_stamp_ns_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->relay_arrival_stamp_ = now - ros::Duration(2.0);
  follower->handleSimpleDiffPlannerExecution(now);
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_TRUE(follower->relay_arrival_stamp_.isZero());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->handleSimpleDiffPlannerExecution(now);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_NE(first_goal, follower->diff_active_goal_stamp_ns_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  // Each ordinary point gets its own full dwell, also on the legacy FIFO path.
  follower->relay_waypoints_.front().safe_relay_id = 0;
  follower->follower_odom_.pose.pose.position.x = 1.9;
  follower->handleSimpleDiffPlannerExecution(now);
  EXPECT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(now, follower->relay_arrival_stamp_);
}

TEST_F(FifoEntrySpacing, OrdinaryDwellDoesNotRestartAfterPositionHeightOrYawLeavesTolerance) {
  prepareQualifiedPoints();
  follower->ordinary_waypoint_dwell_ = 2.0;
  const auto now = ros::Time::now();
  follower->handleSimpleDiffPlannerExecution(now);
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->handleSimpleDiffPlannerExecution(now);
  const auto started = follower->relay_arrival_stamp_;
  ASSERT_EQ(now, started);
  for (int axis = 0; axis < 3; ++axis) {
    auto& pose = follower->follower_odom_.pose.pose;
    pose.position.x = 0.9;
    pose.position.z = 0.6;
    pose.orientation.w = 1.0;
    pose.orientation.z = 0.0;
    if (axis == 0) pose.position.x = 0.84;
    if (axis == 1) pose.position.z = 0.44;
    if (axis == 2) {
      pose.orientation.w = std::cos(21.0 * M_PI / 360.0);
      pose.orientation.z = std::sin(21.0 * M_PI / 360.0);
    }
    follower->handleSimpleDiffPlannerExecution(now);
    EXPECT_EQ(2U, follower->relay_waypoints_.size());
    EXPECT_EQ(started, follower->relay_arrival_stamp_);
    EXPECT_FALSE(follower->diff_wait_hold_active_);
  }
  // Still outside the yaw tolerance when time expires: release without restarting.
  follower->relay_arrival_stamp_ = now - ros::Duration(2.0);
  follower->handleSimpleDiffPlannerExecution(now);
  EXPECT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_TRUE(follower->relay_arrival_stamp_.isZero());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, OrdinaryDwellStillAllowsEmergencySeparationHold) {
  prepareQualifiedPoints();
  follower->ordinary_waypoint_dwell_ = 2.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_FALSE(follower->relay_arrival_stamp_.isZero());
  setLeader(1.9);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
}

TEST_F(FifoEntrySpacing, QualifiedArrivalKeepsConvergingWhileNextPointWaitsForLeader) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  follower->follower_odom_.pose.pose.position.x = 0.9;
  setLeader(2.8); // Actual spacing is sufficient, next endpoint is not yet cleared.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  setLeader(1.9); // Emergency actual-distance hold still overrides convergence.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, QualifiedLastPointDoesNotFreezeAtArrivalTolerance) {
  prepareQualifiedPoints();
  follower->relay_waypoints_.pop_back();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, QualifiedHandoffIgnoresOldRepliesAndStillHoldsOnCurrentFailure) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto old_stamp = follower->diff_active_goal_stamp_ns_;
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  const auto new_stamp = follower->diff_active_goal_stamp_ns_;
  ASSERT_NE(new_stamp, old_stamp);
  reportDiffStatus("PLANNING_FAILED", old_stamp);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  reportDiffStatus("PLANNING_FAILED", new_stamp);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  std_msgs::String::Ptr accepted(new std_msgs::String);
  accepted->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" +
      std::to_string(old_stamp) + " 1 0 0.6";
  follower->diffStatusCallback(accepted);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->diff_accepted_goal_valid_);
}

TEST_F(FifoEntrySpacing, QualifiedExitStillRequiresSpeedAndDwell) {
  prepareQualifiedPoints();
  follower->ordinary_waypoint_dwell_ = 2.0; // Exit continues using its original dwell.
  follower->exit_waypoint_index_ = 0;
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->follower_horizontal_speed_ = 0.5;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  EXPECT_TRUE(follower->relay_arrival_stamp_.isZero());
  follower->follower_horizontal_speed_ = 0.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  ASSERT_FALSE(follower->relay_arrival_stamp_.isZero());
  follower->relay_arrival_stamp_ = ros::Time::now() -
      ros::Duration(follower->relay_arrive_dwell_ + 0.01);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(1U, follower->relay_waypoints_.size());
}

TEST_F(FifoEntrySpacing, AcceptedEndpointKeepsConvergingInsteadOfFreezingPositionAndYaw) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  reportDiffStatus("TRAJECTORY_PUBLISHED", follower->diff_active_goal_stamp_ns_); // Invalid endpoint rejected.
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  std_msgs::String::Ptr accepted(new std_msgs::String);
  accepted->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" +
      std::to_string(follower->diff_active_goal_stamp_ns_) + " 1 0 0.6";
  follower->diffStatusCallback(accepted);
  ASSERT_TRUE(follower->diff_accepted_goal_valid_);
  quadrotor_msgs::PositionCommand::Ptr command(new quadrotor_msgs::PositionCommand);
  follower->diffCommandCallback(command);
  const auto goal_stamp = follower->diff_active_goal_stamp_ns_;
  follower->diff_trajectory_response_stamp_ = ros::Time::now() - ros::Duration(5.0);
  follower->diff_command_stamp_ = follower->diff_trajectory_response_stamp_;
  auto& attitude = follower->follower_odom_.pose.pose.orientation;
  attitude.w = std::cos(M_PI/6.0);
  attitude.z = std::sin(M_PI/6.0); // Still 60 degrees away.
  for (double x : {0.65, 0.9}) {
    follower->follower_odom_.pose.pose.position.x = x;
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(2U, follower->relay_waypoints_.size());
    EXPECT_FALSE(follower->diff_wait_hold_active_);
    EXPECT_TRUE(follower->diff_goal_published_);
    EXPECT_EQ(goal_stamp, follower->diff_active_goal_stamp_ns_);
  }
  setLeader(1.9); // Sudden approach still triggers existing actual-distance handling.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
}

TEST_F(FifoEntrySpacing, OccupiedPointIsAttemptedFirstThenSkippedAndOldReplyCannotUnlock) {
  prepareQualifiedPoints();
  follower->ordinary_waypoint_dwell_ = 2.0; // Obstacle skipping must not dwell at the blocked point.
  occupyFirstPoint();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  ASSERT_TRUE(follower->diff_goal_published_);
  const auto old_stamp = follower->diff_active_goal_stamp_ns_;
  // The timer must allow checking local occupancy even if the leader withdraws this point.
  follower->safe_relay_valid_ids_.erase(1);
  follower->leader_started_ = follower->follower_started_ = true;
  follower->follower_odom_stamp_ = ros::Time::now();
  follower->timerCallback(ros::TimerEvent());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(2U, follower->relay_waypoints_.front().safe_relay_id);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  std_msgs::String::Ptr accepted(new std_msgs::String);
  accepted->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" + std::to_string(old_stamp) + " 1 0 0.6";
  follower->diffStatusCallback(accepted);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->diff_accepted_goal_valid_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_NE(old_stamp, follower->diff_active_goal_stamp_ns_);
}

TEST_F(FifoEntrySpacing, OccupiedPointWaitsForLeaderClearanceFreshDataAndValidSuccessor) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  occupyFirstPoint();
  setLeader(2.19); // Actual separation is sufficient, but leader is <1.2m from point.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  setLeader(2.2);
  follower->leader_odom_stamp_ = ros::Time::now() - ros::Duration(10.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  setLeader(2.2);
  follower->cloud_stamp_ = ros::Time::now() - ros::Duration(10.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->cloud_stamp_ = ros::Time::now();
  follower->safe_relay_valid_ids_.erase(2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->safe_relay_valid_ids_.insert(2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size()); // Exact 1.2m boundary allows skipping.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_); // Leader is still near next point.
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, FailureAndWithdrawalAloneDoNotSkipAndSpecialPointsAreNotSkipped) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  reportDiffStatus("PLANNING_FAILED", follower->diff_active_goal_stamp_ns_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->safe_relay_valid_ids_.erase(1);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  occupyFirstPoint();
  follower->exit_waypoint_index_ = 0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->exit_waypoint_index_ = 1;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size()); // Cannot skip directly into exit task.
  follower->relay_waypoints_.pop_back();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(1U, follower->relay_waypoints_.size()); // No successor: retain point.
}

TEST_F(FifoEntrySpacing, DoorAndExitKeepFiveDegreeHeadingTolerance) {
  follower->door_waypoint_released_ = true;
  follower->require_waypoint_yaw_ = true;
  queuePoint(1.0, true);
  queuePoint(2.0);
  follower->exit_waypoint_index_ = 1;
  setLeader(5.0);
  for (double x : {1.0, 2.0}) {
    const auto size = follower->relay_waypoints_.size();
    follower->follower_odom_.pose.pose.position.x = x;
    auto& attitude = follower->follower_odom_.pose.pose.orientation;
    attitude.w = std::cos(10.0*M_PI/360.0);
    attitude.z = std::sin(10.0*M_PI/360.0);
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(size, follower->relay_waypoints_.size());
    attitude.w = std::cos(4.0*M_PI/360.0);
    attitude.z = std::sin(4.0*M_PI/360.0);
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(size - 1, follower->relay_waypoints_.size());
  }
}

TEST_F(FifoEntrySpacing, RecoveryAfterPlanningFailureReleasesHoldAndStopsRetry) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;

  reportDiffStatus("PLANNING_FAILED", stamp);
  ASSERT_FALSE(follower->diff_goal_published_);
  ASSERT_TRUE(follower->diff_wait_hold_active_);
  reportDiffStatus("OCCUPIED_RECOVERY_HISTORY", stamp);
  ASSERT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, stamp);

  reportDiffStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
}

TEST_F(FifoEntrySpacing, RecoveryRetryAfterAbortCanReleaseHoldBeforeFifoResend) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("OCCUPIED_RECOVERY_HISTORY", stamp);
  reportDiffStatus("OCCUPIED_RECOVERY_ABORTED", stamp);
  ASSERT_FALSE(follower->diff_goal_published_);
  ASSERT_TRUE(follower->diff_wait_hold_active_);

  reportDiffStatus("OCCUPIED_RECOVERY_HISTORY", stamp);
  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, stamp);
}

TEST_F(FifoEntrySpacing, SamePointRetryAcceptsRecoveryFromPreviousAttempt) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto old_stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("PLANNING_FAILED", old_stamp);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_GT(follower->diff_active_goal_stamp_ns_, old_stamp);
  ASSERT_TRUE(follower->diff_wait_hold_active_);

  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", old_stamp, 100);

  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  const auto retry_stamp = follower->diff_active_goal_stamp_ns_;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now() + ros::Duration(10));
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, retry_stamp);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", old_stamp, 100);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
}

TEST_F(FifoEntrySpacing, PreviousEndpointRecoveryPreservesAlreadySentNextEndpoint) {
  const auto old_stamp = sendAndConsumeOrdinaryGoal();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto next_stamp = follower->diff_active_goal_stamp_ns_;
  ASSERT_GT(next_stamp, old_stamp);
  setLeader(2.8);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_wait_hold_active_);

  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", old_stamp, 100);
  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", old_stamp, 100);

  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_GT(follower->diff_active_goal_stamp_ns_, next_stamp);
}

TEST_F(FifoEntrySpacing, CompletedRecoveryCannotBeReactivatedByDelayedStart) {
  const auto stamp = sendAndConsumeOrdinaryGoal();
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 100);

  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 100);

  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
}

TEST_F(FifoEntrySpacing, UnknownPreviousSessionRecoveryCannotTakeOver) {
  const auto stamp = sendAndConsumeOrdinaryGoal();
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp - 1, 100);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
}

TEST_F(FifoEntrySpacing, OldRecoveryFinishCannotInterruptNewRecoveryAttempt) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  reportRecoveryStatus("OCCUPIED_RECOVERY_ABORTED", stamp, 100);
  reportRecoveryStatus("OCCUPIED_RECOVERY_LATERAL", stamp, 101);

  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 100);

  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 101);
  EXPECT_EQ(follower->relay_waypoints_.size(), 1U);
}

TEST_F(FifoEntrySpacing, SeparationRecoveryDoesNotConsumeOrdinaryFifoEndpoint) {
  queuePoint(1.0);
  geometry_msgs::PoseStamped separation_goal;
  separation_goal.header.stamp = ros::Time::now();
  follower->stampDiffGoalId(&separation_goal);
  const auto stamp = separation_goal.header.stamp.toNSec();

  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 100);

  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 1.0);
}

TEST_F(FifoEntrySpacing, ControllerCancellationPreservesGoalAndAcceptsFreshNormalPlan) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("PLANNING_FAILED", stamp);
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);

  reportRecoveryStatus("OCCUPIED_RECOVERY_CANCELLED", stamp, 100);

  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  std_msgs::String::Ptr success(new std_msgs::String);
  success->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" + std::to_string(stamp) + " 1 0 0.6";
  follower->diffStatusCallback(success);
  EXPECT_TRUE(follower->diff_plan_response_received_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->follower_odom_.pose.pose.position.x = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
}

TEST_F(FifoEntrySpacing, FreshNormalTrajectoryAlsoEndsRecoveryWait) {
  follower->door_waypoint_released_ = true;
  queuePoint(1.0);
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("PLANNING_FAILED", stamp);
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  std_msgs::String::Ptr success(new std_msgs::String);
  success->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" + std::to_string(stamp) + " 1 0 0.6";

  follower->diffStatusCallback(success);

  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  follower->follower_odom_.pose.pose.position.x = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
}

TEST_F(FifoEntrySpacing, SinglePointFollowingReleasesAtOnePointTwoAndConsumesInOrder) {
  follower->min_separation_ = follower->separation_release_distance_ = 1.2;
  follower->waypoint_release_min_separation_ = 1.2;
  queuePoint(1.0);
  queuePoint(2.0);
  setLeader(2.19);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  setLeader(2.2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  ros::WallDuration(.05).sleep();
  ros::spinOnce();
  ASSERT_EQ(goals.size(), 1U);
  EXPECT_DOUBLE_EQ(goals.back().pose.position.x, 1.0);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);

  follower->follower_odom_.pose.pose.position.x = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_DOUBLE_EQ(follower->relay_waypoints_.front().position.x, 2.0);
  EXPECT_FALSE(follower->diff_goal_published_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(3.2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  ros::WallDuration(.05).sleep();
  ros::spinOnce();
  ASSERT_EQ(goals.size(), 2U);
  EXPECT_DOUBLE_EQ(goals.back().pose.position.x, 2.0);
}

TEST_F(FifoEntrySpacing, OrdinarySpacingChangePreservesExitAndEntryThresholds) {
  follower->min_separation_ = follower->separation_release_distance_ = 1.2;
  follower->waypoint_release_min_separation_ = 1.2;
  follower->task_separation_ = 1.5;
  queuePoint(1.0);
  setLeader(1.3);
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
  EXPECT_TRUE(follower->relayWaypointSeparationReady("INTERNAL"));
  EXPECT_FALSE(follower->relayWaypointSeparationReady("EXIT"));
  EXPECT_FALSE(follower->relayWaypointSeparationReady("OUTSIDE_WAIT"));
  setLeader(1.19);
  EXPECT_TRUE(follower->handleDiffSeparationSafety(ros::Time::now()));
  setLeader(1.2);
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
  follower->exit_waypoint_index_ = 0;
  EXPECT_TRUE(follower->handleDiffSeparationSafety(ros::Time::now()));
  setLeader(1.5);
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
  follower->relay_waypoints_[0].is_door = true;
  setLeader(.6);
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fifo_entry_spacing",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
