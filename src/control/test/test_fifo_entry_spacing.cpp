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
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (follower->diff_goal_pub_.getNumSubscribers() == 0 &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(follower->diff_goal_pub_.getNumSubscribers(), 0U);
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

  ros::NodeHandle nh;
  ros::Subscriber sink;
  std::unique_ptr<LeaderSafePathFollower> follower;
};

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

TEST_F(FifoEntrySpacing, NormalPointWaitsForOneMeterLeaderClearance) {
  queuePoint(1.0);
  setLeader(1.99);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_FALSE(follower->diff_goal_published_);
  setLeader(2.0);
  follower->handleSimpleDiffPlannerExecution(ros::Time::now());
  EXPECT_TRUE(follower->diff_goal_published_);
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fifo_entry_spacing",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
