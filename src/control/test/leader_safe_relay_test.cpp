// Exercise the actual receiver callbacks and Diff execution adapter.
#define main leader_safe_path_follower_node_main
#include "../src/leader_safe_path_follower.cpp"
#undef main
#include <gtest/gtest.h>

class LeaderSafeRelayTest {
 public:
  static void run() {
    ros::NodeHandle nh;
    nh.setParam("/dual_uav_frame_alignment/mission_frame", std::string("world"));
    nh.setParam("/dual_uav_frame_alignment/follower/x", -1.2);
    nh.setParam("/dual_uav_frame_alignment/follower/y", 0.3);
    nh.setParam("/dual_uav_frame_alignment/follower/z", 0.2);
    nh.setParam("/dual_uav_frame_alignment/follower/yaw_rad", 0.5);
    ros::NodeHandle pnh("~");
    pnh.setParam("obstacle_check_enabled", false);
    LeaderSafePathFollower follower;
    follower.timer_.stop();
    follower.door_waypoint_released_ = true;
    follower.have_leader_odom_ = follower.have_follower_odom_ = true;
    follower.leader_started_ = follower.follower_started_ = true;
    follower.leader_odom_.pose.pose.position.x = 8;
    follower.leader_odom_.pose.pose.position.z = 1;
    follower.follower_odom_.pose.pose.position.z = 0.6;

    ros::Time::setNow(ros::Time(100));
    auto path = boost::make_shared<nav_msgs::Path>();
    path->header.frame_id = "world";
    path->header.stamp = ros::Time(50);
    path->header.seq = 1;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.seq = 1;
    pose.pose.position.x = 4;
    pose.pose.position.y = 1;
    pose.pose.position.z = 1.1;
    pose.pose.orientation.w = 1;
    path->poses.push_back(pose);
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.relay_waypoints_.empty()); // A stale latched singleton cannot move UAV1.
    follower.leader_odom_.pose.pose.position =
        follower.followerToWorld(follower.follower_odom_.pose.pose.position);
    path->header.seq = 2;
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.relay_waypoints_.empty()); // Keep the existing actual 1m dual-UAV gate.
    follower.leader_odom_.pose.pose.position.x = 8;
    follower.leader_odom_.pose.pose.position.y = 0;
    follower.enqueueSafeRelay();
    ASSERT_EQ(1U, follower.relay_waypoints_.size());
    EXPECT_DOUBLE_EQ(4, follower.relay_waypoints_[0].position.x);
    EXPECT_DOUBLE_EQ(1.1, follower.relay_waypoints_[0].position.z);
    EXPECT_TRUE(follower.activeSafeRelayValid());

    geometry_msgs::PoseStamped goal;
    bool received = false;
    auto sub = nh.subscribe<geometry_msgs::PoseStamped>("/UAV1/planning/goal", 1,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) { goal = *msg; received = true; });
    for (int i = 0; i < 30 && follower.diff_goal_pub_.getNumSubscribers() == 0; ++i) {
      ros::spinOnce(); ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(follower.diff_goal_pub_.getNumSubscribers(), 0U);
    follower.handleDiffPlannerExecution(ros::Time::now());
    for (int i = 0; i < 30 && !received; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
    ASSERT_TRUE(received);
    const auto expected = follower.worldToFollower(pose.pose.position);
    EXPECT_LT(distance3d(expected, goal.pose.position), 1e-9); // No yaw offset or height flattening.

    auto status = boost::make_shared<std_msgs::String>();
    status->data = "TRAJECTORY_PUBLISHED 0 0 0";
    follower.diffStatusCallback(status);
    EXPECT_TRUE(follower.diff_wait_hold_active_);
    EXPECT_FALSE(follower.diff_accepted_goal_valid_); // Planner projection cannot change checked B.
    for (int i = 0; i < 4; ++i) {
      follower.diff_goal_published_ = true;
      status->data = "PLANNING_FAILED";
      follower.diffStatusCallback(status);
    }
    EXPECT_EQ(0U, follower.active_relay_index_);
    EXPECT_FALSE(follower.diff_recovery_requested_); // No new unsafe short stopping point.
    EXPECT_TRUE(follower.diff_quarantined_indices_.empty());

    path->header.seq = 3;
    path->poses.clear();
    follower.safeRelayCallback(path);
    EXPECT_FALSE(follower.activeSafeRelayValid());
    follower.leader_odom_stamp_ = follower.follower_odom_stamp_ = ros::Time::now();
    follower.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(follower.diff_wait_hold_active_);
    EXPECT_EQ(0U, follower.active_relay_index_);
    path->header.seq = 4;
    path->poses.push_back(pose);
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.activeSafeRelayValid());
    EXPECT_EQ(1U, follower.relay_waypoints_.size()); // Revalidation does not append duplicates.

    ros::Time::setNow(ros::Time(102));
    EXPECT_FALSE(follower.activeSafeRelayValid());
    path->header.seq = 5;
    follower.safeRelayCallback(path);
    EXPECT_FALSE(follower.activeSafeRelayValid());
    path->header.seq = 6;
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.activeSafeRelayValid());
    // IDs are an ordered queue, not an unordered set of fallback destinations.
    path->header.seq = 7;
    pose.header.seq = 3;
    pose.pose.position.x = 6;
    path->poses.push_back(pose);
    follower.safeRelayCallback(path);
    EXPECT_EQ(1U, follower.relay_waypoints_.size()); // Missing #2 is not silently skipped.
    path->header.seq = 8;
    path->poses[0].pose.position.x += 0.2;
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.safe_relay_fault_); // Published endpoints are immutable within a session.
    EXPECT_FALSE(follower.activeSafeRelayValid());
  }
  static void replacement(bool completed = false, bool too_far = false) {
    ros::NodeHandle pnh("~");
    pnh.setParam("obstacle_check_enabled", false);
    LeaderSafePathFollower follower;
    follower.timer_.stop();
    ros::Time::setNow(ros::Time(200));
    auto path = boost::make_shared<nav_msgs::Path>();
    path->header.frame_id = "world";
    path->header.stamp = ros::Time(150);
    path->header.seq = 1;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.seq = 8;
    pose.header.stamp = ros::Time(199);
    pose.pose.position.x = 3;
    pose.pose.position.z = .56;
    pose.pose.orientation.w = 1;
    path->poses.push_back(pose);
    follower.safeRelayCallback(path);
    RoutePoint point;
    point.position = pose.pose.position;
    point.safe_relay_id = 8;
    follower.relay_waypoints_.push_back(point);
    point.safe_relay_id = 9;
    point.position.x = 2;
    follower.relay_waypoints_.push_back(point);
    follower.active_relay_index_ = completed ? 1 : 0;
    follower.safe_relay_next_id_ = 10;
    follower.diff_goal_published_ = true;
    follower.relay_arrival_stamp_ = ros::Time(199);
    path->header.seq = 2;
    path->poses.front().header.stamp = ros::Time(200);
    path->poses.front().pose.position.x -= too_far ? .61 : .4;
    follower.safeRelayCallback(path);
    EXPECT_EQ(too_far, follower.safe_relay_fault_);
    if (too_far) return;
    EXPECT_TRUE(follower.safe_relay_eighth_revised_);
    EXPECT_EQ(2U, follower.relay_waypoints_.size());
    EXPECT_EQ(10U, follower.safe_relay_next_id_);
    EXPECT_EQ(completed ? 1U : 0U, follower.active_relay_index_);
    EXPECT_DOUBLE_EQ(completed ? 3.0 : 2.6, follower.relay_waypoints_[0].position.x);
    EXPECT_EQ(completed, follower.diff_goal_published_);
    if (!completed) EXPECT_TRUE(follower.relay_arrival_stamp_.isZero());
    path->header.seq = 3;
    follower.safeRelayCallback(path); // 重复8.1快照不重复消耗或产生故障。
    EXPECT_FALSE(follower.safe_relay_fault_);
    path->header.seq = 4;
    path->poses.front().header.stamp = ros::Time(201);
    path->poses.front().pose.position.x -= .1;
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.safe_relay_fault_); // 不允许8.2。
  }

};
TEST(LeaderSafeRelay, SnapshotValidityExactCoordinatesAndNoUnsafeFallback) { LeaderSafeRelayTest::run(); }
TEST(LeaderSafeRelay, EighthRevisionUpdatesActiveGoalWithoutAppending) {
  LeaderSafeRelayTest::replacement();
}
TEST(LeaderSafeRelay, CompletedEighthGoalIsNotRevisited) {
  LeaderSafeRelayTest::replacement(true);
}
TEST(LeaderSafeRelay, EighthRevisionCannotExceedSixtyCentimetres) {
  LeaderSafeRelayTest::replacement(false, true);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "leader_safe_relay_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
