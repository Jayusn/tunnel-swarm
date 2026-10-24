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
    pnh.setParam("use_diff_planner", true);
    pnh.setParam("simple_segment_endpoint_following", true);
    // 本用例验证显式驻留；普通点默认驻留已经为 0，不能依赖旧默认值。
    pnh.setParam("ordinary_waypoint_dwell", 2.0);
    LeaderSafePathFollower follower;
    follower.timer_.stop();
    follower.door_waypoint_released_ = true;
    follower.have_leader_odom_ = follower.have_follower_odom_ = true;
    follower.leader_started_ = follower.follower_started_ = true;
    follower.leader_odom_.pose.pose.position.x = 8;
    follower.leader_odom_.pose.pose.position.z = 1;
    follower.follower_odom_.pose.pose.position.z = 0.6;

    ros::Time::setNow(ros::Time(100));
    follower.leader_odom_stamp_ = follower.follower_odom_stamp_ = ros::Time::now();
    EXPECT_TRUE(follower.leader_trajectory_sub_.getTopic().empty());
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
    status->data = "TRAJECTORY_PUBLISHED goal_stamp_ns=" +
        std::to_string(follower.diff_active_goal_stamp_ns_) + " 0 0 0";
    follower.diffStatusCallback(status);
    EXPECT_TRUE(follower.diff_wait_hold_active_);
    EXPECT_FALSE(follower.diff_accepted_goal_valid_); // Planner projection cannot change checked B.
    for (int i = 0; i < 4; ++i) {
      follower.diff_goal_published_ = true;
      status->data = "PLANNING_FAILED goal_stamp_ns=" +
          std::to_string(follower.diff_active_goal_stamp_ns_);
      follower.diffStatusCallback(status);
    }
    EXPECT_EQ(0U, follower.active_relay_index_);
    EXPECT_FALSE(follower.diff_recovery_requested_); // No new unsafe short stopping point.
    EXPECT_EQ(1U, follower.relay_waypoints_.size());
    follower.diff_goal_published_ = true;
    std::ostringstream accepted_status;
    accepted_status.precision(17);
    accepted_status << "TRAJECTORY_PUBLISHED goal_stamp_ns="
                    << follower.diff_active_goal_stamp_ns_ << " "
                    << expected.x << " " << expected.y << " " << expected.z;
    status->data = accepted_status.str();
    follower.diffStatusCallback(status);
    EXPECT_TRUE(follower.diff_accepted_goal_valid_);
    EXPECT_FALSE(follower.diff_wait_hold_active_);
    status->data = "PLANNING_FAILED goal_stamp_ns=" +
        std::to_string(follower.diff_active_goal_stamp_ns_ - 1);
    follower.diffStatusCallback(status);
    EXPECT_TRUE(follower.diff_accepted_goal_valid_); // Stale replies cannot reset this goal.
    EXPECT_FALSE(follower.diff_wait_hold_active_);

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
    follower.leader_odom_stamp_ = follower.follower_odom_stamp_ = ros::Time::now();
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
    follower.follower_odom_.pose.pose.position = expected;
    follower.follower_horizontal_speed_ = 0.5;
    follower.follower_vertical_speed_ = 0.12;
    follower.handleDiffPlannerExecution(ros::Time::now());
    ASSERT_EQ(1U, follower.relay_waypoints_.size()); // Keep the endpoint throughout the dwell.
    follower.relay_arrival_stamp_ = ros::Time::now() - ros::Duration(2.0);
    follower.handleDiffPlannerExecution(ros::Time::now());
    EXPECT_TRUE(follower.relay_waypoints_.empty());
    EXPECT_TRUE(follower.diff_wait_hold_active_);
    path->header.seq = 8;
    path->poses[0].pose.position.x += 0.2;
    follower.safeRelayCallback(path);
    EXPECT_TRUE(follower.safe_relay_fault_); // Published endpoints are immutable within a session.
    EXPECT_FALSE(follower.safeRelayFresh());
  }
};
TEST(LeaderSafeRelay, SnapshotValidityExactCoordinatesAndNoUnsafeFallback) { LeaderSafeRelayTest::run(); }
int main(int argc, char** argv) {
  ros::init(argc, argv, "leader_safe_relay_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
