// 在独立ROS master下直接验证跟随器的入口/FIFO放行与间距退让，不启动控制器。
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
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
    // 其余FIFO/扫描用例默认已收到出口；入口等待用例单独设为未收到。
    follower->have_final_exit_ = true;
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

  void reportDiffStatus(const std::string& state, uint64_t stamp,
                        const std::string& suffix = "") {
    std_msgs::String::Ptr status(new std_msgs::String);
    status->data = state + " goal_stamp_ns=" + std::to_string(stamp) + suffix;
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
    follower->follower_odom_.pose.pose.orientation.w = 1.0;
    setLeader(4.0);
  }

  nav_msgs::Path::Ptr relaySnapshot() {
    nav_msgs::Path::Ptr path(new nav_msgs::Path);
    path->header.frame_id = follower->world_frame_;
    path->header.stamp = ros::Time::now();
    path->header.seq = 1;
    for (const auto& point : follower->relay_waypoints_) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = path->header.frame_id;
      pose.header.seq = point.safe_relay_id;
      pose.header.stamp = path->header.stamp;
      pose.pose.position = point.position;
      pose.pose.orientation.w = std::cos(point.yaw * .5);
      pose.pose.orientation.z = std::sin(point.yaw * .5);
      path->poses.push_back(pose);
    }
    follower->safe_relay_next_id_ = path->poses.back().header.seq + 1;
    follower->safeRelayCallback(path);
    ++path->header.seq;
    follower->safeRelayCallback(path);
    return path;
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

TEST_F(FifoEntrySpacing, EntryWaitCachesPointsUntilExitThenFliesEntireQueueInOrder) {
  follower->have_final_exit_ = false;
  follower->enable_search_landing_ = true;
  follower->follower_odom_.pose.pose.orientation.w = 1.0;
  follower->follower_odom_.pose.pose.position.x = -2.0;
  setLeader(10.0);
  ASSERT_TRUE(follower->tryReleaseDoorWaypointFromLeaderPosition(
      follower->leader_odom_.pose.pose.position));
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_); // 未收到出口仍先飞入口。
  follower->follower_odom_.pose.pose.position.x = 0.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->simple_door_hover_active_);
  ASSERT_TRUE(follower->relay_waypoints_.empty());
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->diff_goal_published_);

  nav_msgs::Path::Ptr path(new nav_msgs::Path);
  path->header.frame_id = follower->world_frame_;
  path->header.stamp = ros::Time::now();
  path->header.seq = 1;
  for (uint32_t id = 1; id <= 2; ++id) {
    geometry_msgs::PoseStamped pose;
    pose.header = path->header;
    pose.header.seq = id;
    pose.pose.position.x = id;
    pose.pose.position.z = 0.6;
    pose.pose.orientation.w = 1.0;
    path->poses.push_back(pose);
    follower->safeRelayCallback(path);
    ++path->header.seq;
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    ASSERT_EQ(id, follower->relay_waypoints_.size());
    EXPECT_TRUE(follower->simple_door_hover_active_);
    EXPECT_TRUE(follower->diff_wait_hold_active_);
    EXPECT_FALSE(follower->diff_goal_published_);
    EXPECT_EQ(1U, follower->relay_waypoints_.front().safe_relay_id);
  }
  geometry_msgs::PoseStamped::Ptr exit(new geometry_msgs::PoseStamped);
  exit->pose.position.x = 3.0;
  exit->pose.position.z = 0.6;
  exit->pose.orientation.w = 1.0;
  follower->finalExitPoseCallback(exit);
  ASSERT_TRUE(follower->have_final_exit_);
  ASSERT_FALSE(follower->exit_waypoint_released_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_); // 收到出口就放行，不额外等待出门状态。
  EXPECT_FALSE(follower->simple_door_hover_active_);
  EXPECT_DOUBLE_EQ(1.0, follower->relay_waypoints_.front().position.x);

  follower->leader_outside_exit_ = true;
  follower->tryReleaseFinalExitWaypoint("test cached route release");
  ASSERT_EQ(4U, follower->relay_waypoints_.size());
  const std::vector<double> expected = {1.0, 2.0, 3.0,
      3.0 + follower->outside_door_distance_};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    setLeader(10.0);
    ASSERT_EQ(expected.size() - i, follower->relay_waypoints_.size());
    EXPECT_DOUBLE_EQ(expected[i], follower->relay_waypoints_.front().position.x);
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    ASSERT_TRUE(follower->diff_goal_published_);
    follower->follower_odom_.pose.pose.position.x = expected[i];
    follower->handleSimpleDiffPlannerExecution(ros::Time::now());
    EXPECT_EQ(expected.size() - i - 1, follower->relay_waypoints_.size());
  }
  EXPECT_TRUE(follower->outside_wait_arrived_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, ExitReceivedBeforeEntryArrivalReleasesCachedPointsAfterArrival) {
  follower->have_final_exit_ = false;
  follower->enable_search_landing_ = true;
  follower->door_waypoint_released_ = true;
  queuePoint(1.0, true);
  queuePoint(2.0);
  setLeader(5.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  geometry_msgs::PoseStamped::Ptr exit(new geometry_msgs::PoseStamped);
  exit->pose.position.x = 3.0;
  exit->pose.orientation.w = 1.0;
  follower->finalExitPoseCallback(exit);
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  ASSERT_TRUE(follower->relay_waypoints_.front().is_door);
  follower->follower_odom_.pose.pose.position.x = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->simple_door_hover_active_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_FALSE(follower->simple_door_hover_active_);
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_DOUBLE_EQ(2.0, follower->relay_waypoints_.front().position.x);
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
  follower->have_final_exit_ = false;
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
  // Point-cloud hits alone do not authorize a skip; Diff owns this decision.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  follower->follower_cloud_.reset(); // Reproduce the flight's lack of local point hits.
  reportDiffStatus("GOAL_OCCUPIED", old_stamp);
  // The timer must allow the Diff result even if the leader withdraws this point.
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

TEST_F(FifoEntrySpacing, DiffOccupiedPointWaitsForLeaderClearanceFreshOdomAndValidSuccessor) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  reportDiffStatus("GOAL_OCCUPIED", follower->diff_active_goal_stamp_ns_);
  setLeader(2.19); // Actual separation is sufficient, but leader is <1.2m from point.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  setLeader(2.2);
  follower->leader_odom_stamp_ = ros::Time::now() - ros::Duration(10.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  setLeader(2.2);
  follower->safe_relay_valid_ids_.erase(2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  follower->safe_relay_valid_ids_.insert(2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(2U, follower->relay_waypoints_.size()); // Clearance release resent the goal.
  reportDiffStatus("GOAL_OCCUPIED", follower->diff_active_goal_stamp_ns_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size()); // Exact 1.2m boundary allows skipping.
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_); // Leader is still near next point.
  setLeader(4.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, DiffOccupancyRejectsOldRepliesAndClearsAfterSuccessfulPlan) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("GOAL_OCCUPIED", stamp - 1ULL);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  reportDiffStatus("GOAL_OCCUPIED", stamp);
  reportDiffStatus("TRAJECTORY_PUBLISHED", stamp, " 1 0 0.6");
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  reportDiffStatus("PLANNING_FAILED", stamp);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_NE(stamp, follower->diff_active_goal_stamp_ns_);
  reportDiffStatus("GOAL_OCCUPIED", stamp);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  reportDiffStatus("GOAL_OCCUPIED", follower->diff_active_goal_stamp_ns_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(2U, follower->relay_waypoints_.front().safe_relay_id);
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
  follower->safe_relay_valid_ids_.insert(1);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  reportDiffStatus("GOAL_OCCUPIED", follower->diff_active_goal_stamp_ns_);
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

// 复现确认点分支曾提前返回：失败后已锁点，脱障回执仍必须接管执行。
TEST_F(FifoEntrySpacing, QualifiedRecoveryOwnsExecutionEvenWhenPointBecomesInvalid) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("PLANNING_FAILED", stamp);
  ASSERT_TRUE(follower->diff_wait_hold_active_);
  ASSERT_FALSE(follower->diff_goal_published_);
  follower->safe_relay_valid_ids_.erase(1);

  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, stamp);

  reportRecoveryStatus("OCCUPIED_RECOVERY_ABORTED", stamp, 100);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  reportRecoveryStatus("OCCUPIED_RECOVERY_LATERAL", stamp, 101);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 101);
  ASSERT_EQ(follower->relay_waypoints_.size(), 1U);
  EXPECT_EQ(follower->relay_waypoints_.front().safe_relay_id, 2U);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  const auto next_stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("TRAJECTORY_PUBLISHED", next_stamp, " 2 0 0.6");
  EXPECT_FALSE(follower->diff_wait_hold_active_);

  // 已结束执行的延迟回执不能再次夺取下一确认点。
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 101);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_EQ(follower->diff_active_goal_stamp_ns_, next_stamp);
}

TEST_F(FifoEntrySpacing, QualifiedRetryAcceptsRecoveryOwnerAndCancellationKeepsEndpoint) {
  prepareQualifiedPoints();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("PLANNING_FAILED", stamp);
  follower->simple_diff_retry_not_before_ = ros::Time(0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto retry_stamp = follower->diff_active_goal_stamp_ns_;
  ASSERT_GT(retry_stamp, stamp);
  reportRecoveryStatus("OCCUPIED_RECOVERY_VERTICAL", stamp, 100);
  EXPECT_TRUE(follower->simple_occupied_recovery_active_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_CANCELLED", stamp, 100);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  reportDiffStatus("TRAJECTORY_PUBLISHED", retry_stamp, " 1 0 0.6");
  EXPECT_TRUE(follower->diff_accepted_goal_valid_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, QualifiedRecoveryIgnoresUnknownOwnerAndPreservesTerminalPoint) {
  prepareQualifiedPoints();
  follower->terminal_mode_active_ = true;
  follower->terminal_waypoint_index_ = follower->active_relay_index_;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp - 1, 100);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_HISTORY", stamp, 100);
  ASSERT_TRUE(follower->simple_occupied_recovery_active_);
  reportRecoveryStatus("OCCUPIED_RECOVERY_SUCCEEDED", stamp, 100);
  EXPECT_EQ(follower->relay_waypoints_.size(), 2U);
  EXPECT_EQ(follower->relay_waypoints_.front().safe_relay_id, 1U);
  EXPECT_FALSE(follower->diff_goal_published_);
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

TEST_F(FifoEntrySpacing, DynamicPassageReleasesBothActualAndEndpointSpacingAtSixTenths) {
  prepareQualifiedPoints();
  auto passage = boost::make_shared<std_msgs::Bool>();
  passage->data = true;
  follower->dynamicPassageCallback(passage);
  // 两机仅0.65m、前机离目标仅0.60m，均不足普通1.20m门槛。
  follower->follower_odom_.pose.pose.position.x = 0.95;
  follower->relay_arrive_radius_ = 0.01;
  setLeader(1.60);
  EXPECT_TRUE(follower->relayWaypointSeparationReady("INTERNAL"));
  EXPECT_FALSE(follower->handleDiffSeparationSafety(ros::Time::now()));
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_TRUE(follower->dynamicSpacingActive());
  EXPECT_FALSE(follower->separation_recovery_active_);
}

TEST_F(FifoEntrySpacing, DynamicPassageStillWaitsBelowSixTenthsWithoutRetreat) {
  prepareQualifiedPoints();
  auto passage = boost::make_shared<std_msgs::Bool>();
  passage->data = true;
  follower->dynamicPassageCallback(passage);
  setLeader(0.59);
  EXPECT_FALSE(follower->relayWaypointSeparationReady("INTERNAL"));
  EXPECT_TRUE(follower->handleDiffSeparationSafety(ros::Time::now()));
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->separation_recovery_active_);
  setLeader(1.59); // 双机间距够，但离当前目标只有0.59m。
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->simple_waypoint_clearance_hold_active_);
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(1.60);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, DynamicExitContinuesToValidPointThenWaitsForNormalSpacing) {
  prepareQualifiedPoints();
  auto passage = boost::make_shared<std_msgs::Bool>();
  passage->data = true;
  follower->dynamicPassageCallback(passage);
  setLeader(1.8);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  reportDiffStatus("TRAJECTORY_PUBLISHED", follower->diff_active_goal_stamp_ns_, " 1 0 0.6");
  ASSERT_FALSE(follower->diff_wait_hold_active_);
  // 实际已出区但尚未到点：双机0.9m，不得恢复1.2m导致途中锁点。
  passage->data = false;
  follower->dynamicPassageCallback(passage);
  follower->follower_odom_.pose.pose.position.x = 0.9;
  follower->relay_arrive_radius_ = 0.05;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->dynamicSpacingActive());
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  // 到区域外合适点后，原地等待前机；队首保留，不能提前放行下一个点。
  follower->follower_odom_.pose.pose.position.x = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->dynamicSpacingActive());
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_TRUE(follower->diff_separation_hold_active_);
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  setLeader(2.19);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  setLeader(2.21);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_DOUBLE_EQ(2.0, follower->relay_waypoints_.front().position.x);
  EXPECT_FALSE(follower->diff_separation_hold_active_);
  // 下一个点重新执行1.20m放行条件。
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(3.21);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, DynamicSpacingSurvivesInsideArrivalAndInvalidOutsidePoint) {
  prepareQualifiedPoints();
  auto passage = boost::make_shared<std_msgs::Bool>();
  passage->data = true;
  follower->dynamicPassageCallback(passage);
  setLeader(2.8);
  follower->follower_odom_.pose.pose.position.x = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_TRUE(follower->dynamicSpacingActive()); // 区域内到点不能恢复大间距。
  passage->data = false;
  follower->dynamicPassageCallback(passage);
  follower->safe_relay_valid_ids_.erase(2);
  follower->follower_odom_.pose.pose.position.x = 2.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->dynamicSpacingActive()); // 无效前机点不能充当出区等待点。
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  follower->safe_relay_valid_ids_.insert(2);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->dynamicSpacingActive());
  EXPECT_TRUE(follower->diff_separation_hold_active_);
}

TEST_F(FifoEntrySpacing, DynamicSpacingPreservesEntryExitAndDisabledSimpleMode) {
  prepareQualifiedPoints();
  auto passage = boost::make_shared<std_msgs::Bool>();
  passage->data = true;
  follower->dynamicPassageCallback(passage);
  follower->relay_waypoints_.front().is_door = true;
  EXPECT_FALSE(follower->dynamicSpacingActive());
  follower->relay_waypoints_.front().is_door = false;
  follower->exit_waypoint_index_ = 0;
  EXPECT_FALSE(follower->dynamicSpacingActive());
  follower->exit_waypoint_index_ = std::numeric_limits<std::size_t>::max();
  follower->simple_segment_endpoint_following_ = false;
  EXPECT_FALSE(follower->dynamicSpacingActive());
}


TEST_F(FifoEntrySpacing, OutsideRelayScanUsesRelayHeadingAndCompletesBeforeNextGoal) {
  prepareQualifiedPoints();
  follower->ordinary_scan_enabled_ = true;
  // 非零对齐角；前机点的世界航向转换后为 170 度，而非当前机头的 160 度。
  const double center = 170.0 * M_PI / 180.0;
  follower->follower_alignment_yaw_ = 0.2;
  follower->relay_waypoints_[0].yaw = center + 0.2;
  follower->follower_odom_.pose.pose.position.x = 1.0;
  auto setYaw = [&](double yaw) {
    follower->follower_odom_.pose.pose.orientation.w = std::cos(yaw / 2.0);
    follower->follower_odom_.pose.pose.orientation.z = std::sin(yaw / 2.0);
  };
  setYaw(center - 10.0 * M_PI / 180.0);
  const ros::Time start = ros::Time::now();
  follower->handleSimpleDiffPlannerExecution(start);
  ASSERT_NE(0, follower->ordinary_scan_stage_);
  EXPECT_NEAR(center, follower->ordinary_scan_center_, 1e-9);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_EQ(2U, follower->relay_waypoints_.size());
  // 扫描已开始后，晚到的普通轨迹回执不能解除悬停。
  reportDiffStatus("TRAJECTORY_PUBLISHED", follower->diff_active_goal_stamp_ns_, " 1 0 0.6");
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  bool left = false, right = false, completed = false;
  double finish_time = 0.0;
  for (int i = 1; i <= 400; ++i) {
    const ros::Time now = start + ros::Duration(i * 0.05);
    follower->leader_odom_stamp_ = now;
    const double previous = follower->ordinary_scan_yaw_;
    const int phase = follower->ordinary_scan_stage_;
    setYaw(previous); // 理想响应，仍通过实际里程计航向确认。
    follower->handleSimpleDiffPlannerExecution(now);
    const double delta = std::abs(std::remainder(follower->ordinary_scan_yaw_ - previous, 2*M_PI));
    EXPECT_LE(delta, ((phase == 1 || phase == 4) ? 30.0 : 15.0) * M_PI/180.0 * 0.05 + 1e-8);
    const double offset = std::remainder(follower->ordinary_scan_yaw_ - center, 2*M_PI);
    left = left || std::abs(offset - M_PI/4) < 1e-8;
    right = right || std::abs(offset + M_PI/4) < 1e-8;
    if (follower->relay_waypoints_.size() == 1U) {
      completed = true; finish_time = i * 0.05; break;
    }
    EXPECT_EQ(2U, follower->relay_waypoints_.size());
  }
  EXPECT_TRUE(left); EXPECT_TRUE(right); ASSERT_TRUE(completed);
  EXPECT_GE(finish_time, 12.5); EXPECT_LT(finish_time, 14.0);
  EXPECT_NEAR(0, std::remainder(follower->ordinary_scan_yaw_ - center, 2*M_PI), 1e-8);
  EXPECT_TRUE(follower->diff_wait_hold_active_); // 仅下一条新轨迹解除 HOLD。
  follower->handleSimpleDiffPlannerExecution(start + ros::Duration(finish_time + 0.05));
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
  reportDiffStatus("TRAJECTORY_PUBLISHED", follower->diff_active_goal_stamp_ns_, " 2 0 0.6");
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, InsideDynamicPassageDoesNotScan) {
  prepareQualifiedPoints();
  follower->ordinary_scan_enabled_ = true;
  follower->in_dynamic_passage_ = true;
  follower->follower_odom_.pose.pose.position.x = 1;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
}

TEST_F(FifoEntrySpacing, ScanHeadingLagAndVelocityNoiseCannotBlockNextGoal) {
  prepareQualifiedPoints();
  follower->ordinary_scan_enabled_ = true;
  follower->follower_odom_.pose.pose.position.x = 1;
  follower->follower_horizontal_speed_ = 0.4;
  follower->follower_vertical_speed_ = 0.2;
  const ros::Time start = ros::Time::now();
  follower->handleSimpleDiffPlannerExecution(start);
  double finish_time = 0;
  for (int i = 1; i <= 350; ++i) {
    const ros::Time now = start + ros::Duration(i * 0.05);
    follower->leader_odom_stamp_ = now;
    follower->handleSimpleDiffPlannerExecution(now);
    if (follower->relay_waypoints_.size() == 1U) {
      finish_time = i * 0.05;
      break;
    }
  }
  // 实际航向始终不动：两侧各额外等1秒，但不无限卡住当前接力点。
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_GE(finish_time, 14.5);
  EXPECT_LT(finish_time, 15.0);
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
  EXPECT_NEAR(0.0, follower->ordinary_scan_yaw_, 1e-8);
  follower->handleSimpleDiffPlannerExecution(start + ros::Duration(finish_time + 0.05));
  EXPECT_TRUE(follower->diff_goal_published_);
  reportDiffStatus("TRAJECTORY_PUBLISHED", follower->diff_active_goal_stamp_ns_, " 2 0 0.6");
  EXPECT_FALSE(follower->diff_wait_hold_active_);
}

TEST_F(FifoEntrySpacing, ScanYawJitterDoesNotRestartSideDwell) {
  follower->ordinary_scan_stage_ = 2;
  follower->ordinary_scan_yaw_ = M_PI/4;
  const ros::Time now = ros::Time::now();
  follower->ordinary_scan_last_update_ = now;
  auto setYaw = [&](double degrees) {
    const double half = degrees * M_PI/360.0;
    follower->follower_odom_.pose.pose.orientation.w = std::cos(half);
    follower->follower_odom_.pose.pose.orientation.z = std::sin(half);
  };
  setYaw(37); // 8度误差在放宽后的10度容差内。
  EXPECT_FALSE(follower->stepOrdinaryScan(now));
  setYaw(32); // 中间一帧超过容差也不能把驻留计时清零。
  EXPECT_FALSE(follower->stepOrdinaryScan(now + ros::Duration(0.5)));
  EXPECT_EQ(now, follower->ordinary_scan_reached_);
  setYaw(37);
  EXPECT_FALSE(follower->stepOrdinaryScan(now + ros::Duration(1.0)));
  EXPECT_EQ(3, follower->ordinary_scan_stage_);
}

TEST_F(FifoEntrySpacing, ExitWaypointKeepsExistingArrivalWithoutScan) {
  prepareQualifiedPoints();
  follower->ordinary_scan_enabled_ = true;
  follower->exit_waypoint_index_ = 0;
  follower->follower_odom_.pose.pose.position.x = 1;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
}

TEST_F(FifoEntrySpacing, ThirdPointReplacementReplansAndReleasesHold) {
  prepareQualifiedPoints();
  follower->relay_waypoints_[0].safe_relay_id = 3;
  follower->relay_waypoints_[1].safe_relay_id = 4;
  auto& old_point = follower->relay_waypoints_[0].position;
  old_point.x = 4.821482725;
  old_point.y = -0.181985043;
  old_point.z = 0.431510298;
  setLeader(10.0);
  auto path = relaySnapshot();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  const auto old_stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("OCCUPIED_RECOVERY_VERTICAL", old_stamp,
                   " recovery_stamp_ns=" + std::to_string(old_stamp + 1));
  ASSERT_TRUE(follower->simple_occupied_recovery_active_);

  // 复现本次 3 -> 3.1 的实际坐标变化；点时间戳不作为额外接收门槛。
  ++path->header.seq;
  auto& replacement = path->poses[0].pose.position;
  replacement.x = 4.721482725;
  replacement.z = 0.525636228;
  follower->safeRelayCallback(path);
  ASSERT_FALSE(follower->safe_relay_fault_);
  ASSERT_TRUE(follower->activeSafeRelayValid());
  ASSERT_FALSE(follower->simple_occupied_recovery_active_);
  ASSERT_FALSE(follower->diff_goal_published_);
  ASSERT_EQ(2U, follower->relay_waypoints_.size());
  follower->setDiffWaitPositionHold(true, "test pending trajectory");
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  const auto new_stamp = follower->diff_active_goal_stamp_ns_;
  ASSERT_GT(new_stamp, old_stamp);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(1.0);
  while ((goals.empty() || goals.back().header.stamp.toNSec() != new_stamp) &&
         ros::WallTime::now() < deadline) {
    ros::spinOnce(); ros::WallDuration(.01).sleep();
  }
  ASSERT_FALSE(goals.empty());
  EXPECT_EQ(new_stamp, goals.back().header.stamp.toNSec());
  EXPECT_LT(distance3d(follower->worldToFollower(replacement), goals.back().pose.position), 1e-9);
  reportDiffStatus("TRAJECTORY_PUBLISHED", new_stamp, " 4.721482725 -0.181985043 0.525636228");
  EXPECT_TRUE(follower->diff_accepted_goal_valid_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);

  // 旧目标的普通、脱障回执都不能重新锁点、接管执行或消费新点。
  reportDiffStatus("PLANNING_FAILED", old_stamp);
  for (const auto& status : {"OCCUPIED_RECOVERY_VERTICAL", "OCCUPIED_RECOVERY_SUCCEEDED",
                             "OCCUPIED_RECOVERY_CANCELLED", "OCCUPIED_RECOVERY_ABORTED"})
    reportDiffStatus(status, old_stamp, " recovery_stamp_ns=" + std::to_string(old_stamp + 2));
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  EXPECT_TRUE(follower->diff_accepted_goal_valid_);
  EXPECT_EQ(2U, follower->relay_waypoints_.size());

  // 同一快照再次广播不会重复撤销已经接纳的新轨迹。
  ++path->header.seq;
  follower->safeRelayCallback(path);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_TRUE(follower->diff_accepted_goal_valid_);
}

TEST_F(FifoEntrySpacing, QueuedPointReplacementDoesNotInterruptCurrentGoal) {
  prepareQualifiedPoints();
  auto path = relaySnapshot();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("TRAJECTORY_PUBLISHED", stamp, " 1 0 0.6");
  ASSERT_TRUE(follower->diff_accepted_goal_valid_);
  ++path->header.seq;
  path->poses.erase(path->poses.begin()); // 只发布下一点的修订，当前点未出现在快照中。
  path->poses[0].pose.position.x = 2.8;
  follower->safeRelayCallback(path);
  EXPECT_FALSE(follower->safe_relay_fault_);
  EXPECT_DOUBLE_EQ(2.8, follower->relay_waypoints_[1].position.x);
  EXPECT_TRUE(follower->diff_accepted_goal_valid_);
  EXPECT_TRUE(follower->diff_goal_published_);
  EXPECT_EQ(stamp, follower->diff_active_goal_stamp_ns_);
  EXPECT_FALSE(follower->diff_wait_hold_active_);
  --path->header.seq;
  path->poses[0].pose.position.x = 2.0;
  follower->safeRelayCallback(path); // 延迟旧快照不能把更新后的点改回去。
  EXPECT_DOUBLE_EQ(2.8, follower->relay_waypoints_[1].position.x);
}

TEST_F(FifoEntrySpacing, UpdatedHeadingReplacesCurrentScanReference) {
  prepareQualifiedPoints();
  auto path = relaySnapshot();
  follower->ordinary_scan_stage_ = 2;
  follower->ordinary_scan_done_ = false;
  follower->diff_goal_published_ = true;
  ++path->header.seq;
  path->poses[0].pose.orientation.w = std::cos(.3);
  path->poses[0].pose.orientation.z = std::sin(.3);
  follower->safeRelayCallback(path);
  EXPECT_FALSE(follower->safe_relay_fault_);
  EXPECT_NEAR(.6, follower->relay_waypoints_[0].yaw, 1e-9);
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
  EXPECT_FALSE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, PassedQueuedPointSkipsScanAndPlansNextPoint) {
  follower->door_waypoint_released_ = true;
  queuePoint(0.0, true);
  follower->consumeSimpleRelayFront();
  queuePoint(1.0);
  queuePoint(2.0);
  follower->ordinary_scan_enabled_ = true;
  follower->ordinary_waypoint_dwell_ = 2.0;
  follower->require_waypoint_yaw_ = true;
  follower->follower_odom_.pose.pose.position.x = 1.3;
  follower->follower_odom_.pose.pose.orientation.z = 1.0; // 面向后方不改变越过判断。
  setLeader(5.0);

  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_DOUBLE_EQ(2.0, follower->relay_waypoints_[0].position.x);
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
  EXPECT_FALSE(follower->diff_goal_published_);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
}

TEST_F(FifoEntrySpacing, PassedActivePointCancelsOldGoalAndCannotBeRevisitedByRevision) {
  prepareQualifiedPoints();
  // 通过实际消费入口点建立入段方向。
  RoutePoint door;
  door.is_door = true;
  door.position.z = 0.6;
  follower->relay_waypoints_.insert(follower->relay_waypoints_.begin(), door);
  follower->consumeSimpleRelayFront();
  auto path = relaySnapshot();
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  const auto stamp = follower->diff_active_goal_stamp_ns_;
  reportDiffStatus("TRAJECTORY_PUBLISHED", stamp, " 1 0 0.6");
  follower->follower_odom_.pose.pose.position.x = 1.5;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(2U, follower->relay_waypoints_[0].safe_relay_id);
  EXPECT_TRUE(follower->diff_wait_hold_active_);
  EXPECT_EQ(0U, follower->simple_diff_goal_owners_.count(stamp));
  reportDiffStatus("OCCUPIED_RECOVERY_VERTICAL", stamp,
                   " recovery_stamp_ns=" + std::to_string(stamp + 1));
  EXPECT_FALSE(follower->simple_occupied_recovery_active_);
  ++path->header.seq;
  path->poses[0].pose.position.x = 0.8;
  follower->safeRelayCallback(path);
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(2U, follower->relay_waypoints_[0].safe_relay_id);
}

TEST_F(FifoEntrySpacing, PassedCheckKeepsLateralHeightAndUnreachedTurnTargets) {
  follower->door_waypoint_released_ = true;
  queuePoint(0.0, true);
  follower->consumeSimpleRelayFront();
  queuePoint(1.0);
  setLeader(5.0);
  auto& current = follower->follower_odom_.pose.pose.position;
  current.x = 1.3;
  current.y = 0.8; // 在拐弯外侧，不能只用前向投影消费目标。
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  current.y = 0.0;
  current.z = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  current.z = 0.6;
  current.x = 0.8;
  follower->follower_odom_.pose.pose.orientation.z = 1.0;
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_EQ(1U, follower->relay_waypoints_.size());
}

TEST_F(FifoEntrySpacing, PassedCheckUsesReturnSegmentAndFollowerFrameTransform) {
  follower->door_waypoint_released_ = true;
  follower->follower_alignment_x_ = -1.2;
  queuePoint(3.0);
  follower->consumeSimpleRelayFront();
  queuePoint(2.0); // 回程沿负 X，不能把“X 变小”一律当成回退。
  setLeader(6.0);
  follower->follower_odom_.pose.pose.position.x = 3.5; // 世界 X=2.3，尚未到达。
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  follower->follower_odom_.pose.pose.position.x = 2.9; // 世界 X=1.7，确实越过。
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->relay_waypoints_.empty());
  EXPECT_TRUE(follower->diff_wait_hold_active_); // 无后续点时也撤销旧点的回拉参考。
}

TEST_F(FifoEntrySpacing, PassedCheckPreservesDoorExitOutsideAndTerminalTargets) {
  follower->door_waypoint_released_ = true;
  queuePoint(0.0, true);
  follower->consumeSimpleRelayFront();
  queuePoint(1.0, true);
  queuePoint(2.0);
  queuePoint(3.0);
  queuePoint(4.0);
  follower->exit_waypoint_index_ = 1;
  follower->outside_wait_waypoint_index_ = 2;
  follower->terminal_waypoint_index_ = 3;
  geometry_msgs::Point current;
  current.x = 5;
  current.z = 0.6;
  for (std::size_t i = 0; i < 4; ++i) {
    follower->active_relay_index_ = i;
    EXPECT_FALSE(follower->ordinaryRelayPassed(current, 0.0));
  }
}

TEST_F(FifoEntrySpacing, RecordedSixToSevenRevisionSkipsToEight) {
  // 2026-10-28 13:01:01.225～13:01:40.575 的 UAV1 里程计，按现有 8cm 间隔采样。
  const std::string source(__FILE__);
  std::ifstream input(source.substr(0, source.find_last_of('/')) +
                      "/data/relay_6_to_7_1_flown.csv");
  ASSERT_TRUE(input.good());
  follower->door_waypoint_released_ = true;
  follower->follower_alignment_x_ = -1.2;
  follower->relay_arrive_radius_ = 0.4;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    double stamp;
    nav_msgs::Odometry odom;
    ASSERT_TRUE(static_cast<bool>(row >> stamp >> odom.pose.pose.position.x >>
                                 odom.pose.pose.position.y >> odom.pose.pose.position.z));
    follower->appendFollowerExecutedPoint(odom);
  }
  ASSERT_GT(follower->follower_executed_route_.size(), 20U);
  RoutePoint six, seven, eight;
  six.position.x = 6.199207; six.position.y = -2.477319; six.position.z = 0.666167;
  seven.position.x = 6.148647; seven.position.y = -1.393247; seven.position.z = 0.838322;
  eight.position.x = 5.157354; eight.position.y = -3.078487; eight.position.z = 0.877023;
  six.safe_relay_id = 6; seven.safe_relay_id = 7; eight.safe_relay_id = 8;
  follower->relay_waypoints_ = {six, seven, eight};
  follower->safe_relay_valid_ids_ = {6, 7, 8};
  follower->consumeSimpleRelayFront();
  auto& current = follower->follower_odom_.pose.pose.position;
  current.x = 7.186435213; current.y = -2.375418095; current.z = 0.648526727;
  follower->ordinary_scan_enabled_ = true;
  setLeader(10.0);
  ASSERT_FALSE(follower->ordinaryRelayPassed(follower->followerToWorld(current),
                                            std::abs(current.z - seven.position.z)));
  ASSERT_TRUE(follower->ordinaryRelayAlreadyFlown(follower->followerToWorld(current)));
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_EQ(1U, follower->relay_waypoints_.size());
  EXPECT_EQ(8U, follower->relay_waypoints_[0].safe_relay_id);
  EXPECT_EQ(0, follower->ordinary_scan_stage_);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  ASSERT_TRUE(follower->diff_goal_published_);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(1.0);
  while (goals.empty() && ros::WallTime::now() < deadline) {
    ros::spinOnce(); ros::WallDuration(.01).sleep();
  }
  ASSERT_EQ(1U, goals.size());
  EXPECT_NEAR(6.357354, goals[0].pose.position.x, 1e-6);
  EXPECT_NEAR(-3.078487, goals[0].pose.position.y, 1e-6);
}

TEST_F(FifoEntrySpacing, FlownCheckKeepsReturnNewBranchHeightAndSpecialPoints) {
  follower->relay_arrive_radius_ = 0.4;
  for (double x : {0.0, 1.0, 2.0}) {
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = x; odom.pose.pose.position.z = 0.6;
    follower->appendFollowerExecutedPoint(odom);
  }
  queuePoint(1.0);
  queuePoint(3.0);
  geometry_msgs::Point current;
  current.x = 2.0; current.z = 0.6;
  EXPECT_TRUE(follower->ordinaryRelayAlreadyFlown(current));
  follower->relay_waypoints_[1].position.x = 0.0; // 下一点也返回，保留正常回程。
  EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current));
  follower->relay_waypoints_[1].position.x = 3.0;
  follower->relay_waypoints_[0].position.y = 0.8; // 新的拐弯分支没有实飞覆盖。
  EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current));
  follower->relay_waypoints_[0].position.y = 0.0;
  follower->relay_waypoints_[0].position.z = 1.2;
  EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current));
  follower->relay_waypoints_[0].position.z = 0.6;
  current.x = 1.05; // 尚未离开目标附近，不因扫描抖动跳过。
  EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current));
  current.x = 2.0;
  follower->relay_waypoints_[0].is_door = true;
  EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current));
  follower->relay_waypoints_[0].is_door = false;
  for (auto index : {&follower->exit_waypoint_index_, &follower->outside_wait_waypoint_index_,
                     &follower->terminal_waypoint_index_}) {
    *index = 0;
    EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current));
    *index = std::numeric_limits<std::size_t>::max();
  }
  follower->relay_waypoints_.pop_back();
  EXPECT_FALSE(follower->ordinaryRelayAlreadyFlown(current)); // 无下一点不能区分回程意图。
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fifo_entry_spacing",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
