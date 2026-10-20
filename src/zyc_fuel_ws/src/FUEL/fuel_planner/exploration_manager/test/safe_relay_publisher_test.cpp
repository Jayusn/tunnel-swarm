#include <exploration_manager/safe_relay_publisher.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <gtest/gtest.h>

namespace fast_planner {
class SafeRelayPublisherTest {
 public:
  static void run() {
    ros::NodeHandle nh("~");
    nh.setParam("sdf_map/resolution", 0.1);
    nh.setParam("sdf_map/map_size_x", 14.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 3.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/max_ray_length", 20.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.1);
    nh.setParam("mission/safe_relay/enabled", true);
    nh.setParam("mission/safe_relay/topic_prefix", std::string("/safe_relay_test"));
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    auto planner = std::make_shared<FastPlannerManager>();
    SafeRelayPublisher publisher;
    publisher.initialize(nh, map, planner);
    publisher.timer_.stop();
    nav_msgs::Path received_route, received_goals;
    auto route_sub = nh.subscribe<nav_msgs::Path>("/safe_relay_test/reference_route", 1,
        [&](const nav_msgs::Path::ConstPtr& msg) { received_route = *msg; });
    auto goals_sub = nh.subscribe<nav_msgs::Path>("/safe_relay_test/relay_waypoints", 1,
        [&](const nav_msgs::Path::ConstPtr& msg) { received_goals = *msg; });
    auto spin = []() {
      for (int i = 0; i < 10; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
    };
    spin();
    ros::Time::setNow(ros::Time(100));
    publisher.observe(Eigen::Vector3d(-0.2, 0, 1), 0, ros::Time(99));
    EXPECT_TRUE(publisher.history_.empty());
    publisher.setEntrance(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::UnitX());
    publisher.observe(Eigen::Vector3d(-0.1, 0, 1), 0, ros::Time(99.1));
    EXPECT_TRUE(publisher.history_.empty());
    // One moving traversal, without stopping or revisiting any point.
    for (int i = 0; i <= 60; ++i)
      publisher.observe(Eigen::Vector3d(0.1 * i, 0, 1), 0, ros::Time(99.2 + 0.01 * i));
    ASSERT_EQ(61U, publisher.history_.size());
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(publisher.goals_.empty());  // No map evidence yet.
    EXPECT_FALSE(publisher.volumeFree(Eigen::Vector3d(3, 0, 1), 0.38, 0.28));

    // Raycast a measured free rectangular corridor. Out-of-map endpoints are
    // free rays, so this exercises the map's real UNKNOWN -> FREE interface.
    auto map_frame = [&]() {
      for (int y = -12; y <= 12; ++y)
        for (int z = 3; z <= 20; ++z) {
          pcl::PointCloud<pcl::PointXYZ> cloud;
          pcl::PointXYZ endpoint;
          endpoint.x = 20.0; endpoint.y = y * 0.1 + 0.05; endpoint.z = z * 0.1 + 0.05;
          cloud.push_back(endpoint);
          map->inputPointCloud(cloud, 1, Eigen::Vector3d(-6.8, endpoint.y, endpoint.z));
        }
    };
    map_frame();
    ASSERT_TRUE(publisher.volumeFree(Eigen::Vector3d(3, 0, 1), 0.38, 0.28));
    EXPECT_FALSE(publisher.volumeFree(Eigen::Vector3d(3, 1.2, 1), 0.45, 0.32));
    EXPECT_FALSE(publisher.volumeFree(Eigen::Vector3d(6.9, 0, 1), 0.45, 0.32));
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(publisher.goals_.empty());
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(publisher.goals_.empty());  // Same map does not count twice.
    map_frame();
    map->setOccupied(Eigen::Vector3d(5.5, 0, 1));
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(1U, publisher.goals_.size());
    EXPECT_EQ("NORMAL", publisher.last_status_);  // A later obstacle must not veto this endpoint.
    const auto first_index = publisher.goals_.front().route_index;
    EXPECT_EQ(10U, first_index);
    spin();
    EXPECT_EQ(61U, received_route.poses.size());
    ASSERT_EQ(1U, received_goals.poses.size());
    EXPECT_DOUBLE_EQ(1.0, received_goals.poses.front().pose.position.z);
    EXPECT_EQ(1U, received_goals.poses.front().header.seq);
    EXPECT_EQ(publisher.session_, received_goals.header.stamp);
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(2U, publisher.goals_.size());  // Ordinary open route keeps releasing.
    EXPECT_DOUBLE_EQ(1.0, publisher.history_[publisher.goals_.back().route_index].sample.progress -
                              publisher.history_[first_index].sample.progress);
    map->setOccupied(Eigen::Vector3d(5.5, 0, 1), 0);
    const auto initial_goals = publisher.goals_;
    const auto initial_history = publisher.history_;
    const auto last_index = publisher.goals_.back().route_index;
    auto restore_issued = [&]() {
      publisher.goals_ = initial_goals;
      publisher.history_ = initial_history;
      publisher.anchor_ = last_index;
      publisher.next_id_ = 3;
    };

    // A currently occupied earlier segment invalidates its destination; a clear
    // endpoint alone cannot certify A -> B, including unknown intervening space.
    map->setOccupied(Eigen::Vector3d(1.0, 0.0, 1.0));
    EXPECT_FALSE(publisher.connectionFree(0, first_index));
    publisher.publish("TEST_BLOCKED_CONNECTION", true);
    spin();
    for (const auto& pose : received_goals.poses) EXPECT_NE(1U, pose.header.seq);
    map->setOccupied(Eigen::Vector3d(1.0, 0.0, 1.0), 0);
    map->setOccupied(publisher.history_[last_index].sample.position);
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_EQ("WAIT_NO_VALID_TARGET", publisher.last_status_);
    EXPECT_EQ(first_index, publisher.anchor_);
    EXPECT_EQ(2U, publisher.goals_.size());  // Recovery cannot bypass a blocked transit path.
    spin();
    ASSERT_EQ(1U, received_goals.poses.size());
    EXPECT_EQ(1U, received_goals.poses.front().header.seq);
    map->setOccupied(publisher.history_[first_index].sample.position);
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_EQ("LAST_TARGET_INVALID", publisher.last_status_);
    EXPECT_EQ(2U, publisher.goals_.size());
    spin();
    EXPECT_TRUE(received_goals.poses.empty());  // No prior valid A: do not invent a new origin.
    map->setOccupied(publisher.history_[first_index].sample.position, 0);
    map->setOccupied(publisher.history_[last_index].sample.position, 0);
    restore_issued();

    // A clear goal can also be invalid because its incoming connection changed.
    // Recheck from the last valid A, including the entire omitted interval.
    map->setOccupied(Eigen::Vector3d(1.5, 0, 1));
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(publisher.history_[last_index].sample.confirmed);
    EXPECT_EQ(first_index, publisher.anchor_);
    EXPECT_EQ("WAIT_NO_VALID_TARGET", publisher.last_status_);
    EXPECT_EQ(2U, publisher.goals_.size());
    spin();
    ASSERT_EQ(1U, received_goals.poses.size());
    EXPECT_EQ(1U, received_goals.poses.front().header.seq);
    map->setOccupied(Eigen::Vector3d(1.5, 0, 1), 0);
    restore_issued();

    // Real occupied voxels at the edge of the holding envelope leave the
    // smaller transit envelope clear, matching the observed flight failure.
    auto occupy_hold_edge = [&](double x) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::PointXYZ endpoint;
      endpoint.x = x + 0.05; endpoint.y = 0.35; endpoint.z = 1.05;
      cloud.push_back(endpoint);
      for (int i = 0; i < 12; ++i)
        map->inputPointCloud(cloud, 1, Eigen::Vector3d(x + 0.05, 1.0, 1.05));
    };
    occupy_hold_edge(2.0);
    ASSERT_FALSE(publisher.volumeFree(Eigen::Vector3d(2, 0, 1), 0.45, 0.32));
    ASSERT_TRUE(publisher.connectionFree(first_index, 50));
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(3U, publisher.goals_.size());
    EXPECT_EQ(3U, publisher.goals_.back().id);
    EXPECT_GT(publisher.goals_.back().route_index, last_index);
    EXPECT_EQ("NORMAL", publisher.last_status_);
    spin();
    ASSERT_EQ(2U, received_goals.poses.size());
    EXPECT_EQ(1U, received_goals.poses.front().header.seq);
    EXPECT_EQ(3U, received_goals.poses.back().header.seq);
    EXPECT_DOUBLE_EQ(1.0, received_goals.poses.front().pose.position.x);
    const auto recovered_index = publisher.goals_.back().route_index;
    EXPECT_EQ(publisher.history_[recovered_index].stamp, received_goals.poses.back().header.stamp);
    EXPECT_DOUBLE_EQ(publisher.history_[recovered_index].sample.position.x(),
                     received_goals.poses.back().pose.position.x);
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(4U, publisher.goals_.size());
    EXPECT_EQ(4U, publisher.goals_.back().id);
    EXPECT_GT(publisher.goals_.back().route_index, recovered_index);

    // Several latest goals may be invalid. A recovery must not reissue a
    // previously passed sample between A and the furthest issued destination.
    restore_issued();
    publisher.goals_.push_back({40, 3});
    publisher.next_id_ = 4;
    publisher.anchor_ = 40;
    occupy_hold_edge(4.0);
    ASSERT_FALSE(publisher.volumeFree(Eigen::Vector3d(4, 0, 1), 0.45, 0.32));
    ASSERT_TRUE(publisher.connectionFree(first_index, 50));
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(4U, publisher.goals_.size());
    EXPECT_EQ(4U, publisher.goals_.back().id);
    EXPECT_GT(publisher.goals_.back().route_index, 40U);
    spin();
    ASSERT_EQ(2U, received_goals.poses.size());
    EXPECT_EQ(1U, received_goals.poses.front().header.seq);
    EXPECT_EQ(4U, received_goals.poses.back().header.seq);

    // Losing known-free evidence must not turn recovery into unconditional release.
    auto unknown_map = std::make_shared<SDFMap>();
    unknown_map->initMap(nh);
    publisher.map_ = unknown_map;
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_EQ("LAST_TARGET_INVALID", publisher.last_status_);
    EXPECT_EQ(4U, publisher.goals_.size());
    spin();
    EXPECT_TRUE(received_goals.poses.empty());
    publisher.map_ = map;
    for (int i = 0; i < 10; ++i) map_frame();
    restore_issued();
    publisher.timerCallback(ros::TimerEvent());

    ros::Time::setNow(ros::Time(102));
    publisher.timerCallback(ros::TimerEvent());
    spin();
    EXPECT_EQ("STALE_ODOMETRY", publisher.last_status_);
    EXPECT_TRUE(received_goals.poses.empty());
    EXPECT_EQ(61U, received_route.poses.size()); // Still publish ordinary reference route.
    publisher.observe(Eigen::Vector3d(6, 0, 1), 0, ros::Time(102));
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_EQ("STALE_MAP", publisher.last_status_);
    // Missing odometry is not replaced by a straight connection across the gap.
    publisher.observe(Eigen::Vector3d(6.8, 0, 1), 0, ros::Time(102.1));
    EXPECT_FALSE(publisher.connectionFree(60, 61));
    const auto count = publisher.history_.size();
    publisher.observe(Eigen::Vector3d(6.9, 0, 1), 0, ros::Time(101));
    EXPECT_EQ(count, publisher.history_.size());

    // A checked bent route remains usable even when the endpoint chord hits a wall.
    publisher.history_.clear();
    publisher.odom_stamp_ = ros::Time(0);
    for (int i = 0; i <= 20; ++i)
      publisher.observe(Eigen::Vector3d(0.1 * i, 0, 1), 0, ros::Time(103 + 0.01 * i));
    for (int i = 1; i <= 10; ++i)
      publisher.observe(Eigen::Vector3d(2, 0.1 * i, 1), 1.57, ros::Time(104 + 0.01 * i));
    map->setOccupied(Eigen::Vector3d(1, 0.5, 1));
    EXPECT_FALSE(publisher.volumeFree(Eigen::Vector3d(1, 0.5, 1), 0.18, 0.10));
    EXPECT_TRUE(publisher.connectionFree(0, publisher.history_.size() - 1));
  }
};
TEST(SafeRelayPublisher, ActualFlightCurrentMapAndInvalidation) { SafeRelayPublisherTest::run(); }
}  // namespace fast_planner
int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_relay_publisher_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
