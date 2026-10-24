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
    nh.setParam("mission/safe_relay/minimum_progress", 0.70);
    nh.setParam("mission/safe_relay/candidate_search_radius", 0.01);
    nh.setParam("mission/safe_relay/maximum_gap", 100.0);
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

    // Occupied issued endpoints and old connections do not withdraw IDs or
    // move the origin backwards. Even an occupied latest A cannot veto new B.
    for (double obstacle_x : {1.0, 1.5, 2.0}) {
      restore_issued();
      map->setOccupied(Eigen::Vector3d(obstacle_x, 0, 1));
      ASSERT_FALSE(publisher.volumeFree(Eigen::Vector3d(obstacle_x, 0, 1), 0.3, 0.1));
      publisher.timerCallback(ros::TimerEvent());
      ASSERT_EQ(3U, publisher.goals_.size());
      EXPECT_EQ("NORMAL", publisher.last_status_);
      EXPECT_EQ(30U, publisher.goals_.back().route_index);
      spin();
      ASSERT_EQ(3U, received_goals.poses.size());
      for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_EQ(i + 1, received_goals.poses[i].header.seq);
        EXPECT_EQ(initial_goals[i].pose.header.stamp, received_goals.poses[i].header.stamp);
        EXPECT_DOUBLE_EQ(initial_history[initial_goals[i].route_index].sample.position.x(),
                         received_goals.poses[i].pose.position.x);
      }
      EXPECT_EQ(publisher.session_, received_goals.header.stamp);
      map->setOccupied(Eigen::Vector3d(obstacle_x, 0, 1), 0);
    }

    // A newly occupied candidate itself is still rejected; choose a different
    // confirmed endpoint without requiring an old route around the obstacle.
    restore_issued();
    map->setOccupied(Eigen::Vector3d(3.0, 0, 1));
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(3U, publisher.goals_.size());
    EXPECT_GT(publisher.goals_.back().route_index, 30U);
    EXPECT_TRUE(publisher.history_[publisher.goals_.back().route_index].candidate.confirmed);
    map->setOccupied(Eigen::Vector3d(3.0, 0, 1), 0);
    restore_issued();

    // No known-free evidence for new candidates means no new point, but old
    // IDs remain available. The producer does not certify their current state.
    auto unknown_map = std::make_shared<SDFMap>();
    unknown_map->initMap(nh);
    publisher.map_ = unknown_map;
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_EQ("WAIT_NO_VALID_TARGET", publisher.last_status_);
    EXPECT_EQ(last_index, publisher.anchor_);
    EXPECT_EQ(2U, publisher.goals_.size());
    spin();
    ASSERT_EQ(2U, received_goals.poses.size());
    EXPECT_EQ(1U, received_goals.poses.front().header.seq);
    EXPECT_EQ(2U, received_goals.poses.back().header.seq);
    publisher.map_ = map;
    // Observe the restored map at t=100 before advancing the test clock;
    // otherwise its first observation at t=102 legitimately refreshes its age.
    publisher.timerCallback(ros::TimerEvent());
    restore_issued();

    // Stale input blocks new releases, not the already issued point catalogue.
    ros::Time::setNow(ros::Time(102));
    publisher.timerCallback(ros::TimerEvent());
    spin();
    EXPECT_EQ("STALE_ODOMETRY", publisher.last_status_);
    EXPECT_EQ(2U, publisher.goals_.size());
    ASSERT_EQ(2U, received_goals.poses.size());
    EXPECT_EQ(61U, received_route.poses.size());
    publisher.observe(Eigen::Vector3d(6, 0, 1), 0, ros::Time(102));
    publisher.timerCallback(ros::TimerEvent());
    spin();
    EXPECT_EQ("STALE_MAP", publisher.last_status_);
    EXPECT_EQ(2U, publisher.goals_.size());
    ASSERT_EQ(2U, received_goals.poses.size());
    EXPECT_EQ(publisher.session_, received_goals.header.stamp);

    // Continue to retain only real, time-ordered odometry observations.
    publisher.observe(Eigen::Vector3d(6.8, 0, 1), 0, ros::Time(102.1));
    const auto count = publisher.history_.size();
    EXPECT_EQ(62U, count);
    publisher.observe(Eigen::Vector3d(6.9, 0, 1), 0, ros::Time(101));
    EXPECT_EQ(count, publisher.history_.size());
  }

  static void entranceSpacing() {
    ros::NodeHandle nh("~entrance_spacing");
    nh.setParam("sdf_map/resolution", 0.1);
    nh.setParam("sdf_map/map_size_x", 14.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 3.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/max_ray_length", 20.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.1);
    nh.setParam("mission/safe_relay/enabled", true);
    nh.setParam("mission/safe_relay/topic_prefix", std::string("/safe_relay_entrance_test"));
    nh.setParam("mission/safe_relay/minimum_progress", 0.70);
    nh.setParam("mission/safe_relay/entrance_minimum_progress", 0.50);
    nh.setParam("mission/safe_relay/minimum_xy_spacing", 0.70);
    nh.setParam("mission/safe_relay/hold_radius", 0.30);
    nh.setParam("mission/safe_relay/hold_half_height", 0.10);
    nh.setParam("mission/safe_relay/candidate_search_radius", 0.01);
    nh.setParam("mission/safe_relay/maximum_gap", 100.0);
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    SafeRelayPublisher publisher;
    publisher.initialize(nh, map, std::make_shared<FastPlannerManager>());
    publisher.timer_.stop();
    EXPECT_DOUBLE_EQ(0.50, publisher.entrance_minimum_progress_);
    EXPECT_DOUBLE_EQ(0.70, publisher.config_.minimum_progress);
    EXPECT_DOUBLE_EQ(0.70, publisher.config_.minimum_xy_spacing);
    EXPECT_DOUBLE_EQ(1.00, publisher.config_.normal_spacing);
    ros::Time::setNow(ros::Time(200));
    publisher.setEntrance(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::UnitX());
    auto observe = [&](int i) {
      publisher.observe(Eigen::Vector3d(i * 0.1, 0, 1), 0,
                        ros::Time(199.8 + i * 0.01));
    };
    auto refresh = [&](double obstacle_x) {
      for (int frame = 0; frame < 2; ++frame) {
        for (int y = -12; y <= 12; ++y)
          for (int z = 3; z <= 20; ++z) {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::PointXYZ endpoint;
            endpoint.x = 20.0; endpoint.y = y * 0.1 + 0.05;
            endpoint.z = z * 0.1 + 0.05;
            cloud.push_back(endpoint);
            map->inputPointCloud(cloud, 1, Eigen::Vector3d(-6.8, endpoint.y, endpoint.z));
          }
        map->setOccupied(Eigen::Vector3d(obstacle_x, 0, 1));
        publisher.timerCallback(ros::TimerEvent());
      }
    };
    for (int i = 0; i <= 14; ++i) observe(i);
    refresh(1.5);
    EXPECT_TRUE(publisher.goals_.empty()); // Only 0.4m is clear of the leader: below entry minimum.
    observe(15);
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(1U, publisher.goals_.size());
    EXPECT_EQ(5U, publisher.goals_.front().route_index); // Entrance -> first fallback: exactly 0.5m.
    EXPECT_EQ("BEFORE_OBSTACLE_WAIT", publisher.last_status_);
    EXPECT_DOUBLE_EQ(0.70, publisher.config_.minimum_progress);
    EXPECT_DOUBLE_EQ(0.70, publisher.config_.minimum_xy_spacing); // The first-release override is local.

    map->setOccupied(Eigen::Vector3d(1.5, 0, 1), 0);
    for (int i = 16; i <= 21; ++i) observe(i);
    refresh(2.1);
    EXPECT_EQ(1U, publisher.goals_.size()); // At most 0.6m after first goal: below later minimum.
    EXPECT_EQ("WAIT_NO_VALID_TARGET", publisher.last_status_);
    observe(22);
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(2U, publisher.goals_.size());
    EXPECT_EQ(12U, publisher.goals_.back().route_index); // First -> second fallback: exactly 0.7m.
    EXPECT_EQ("BEFORE_OBSTACLE_WAIT", publisher.last_status_);
    EXPECT_NEAR(0.7, (publisher.history_[12].sample.position.head<2>() -
                     publisher.history_[5].sample.position.head<2>()).norm(), 1e-9);
    EXPECT_NEAR(0.7, publisher.history_[12].sample.progress -
                         publisher.history_[5].sample.progress, 1e-9);
  }

  static void voxelClearance() {
    ros::NodeHandle nh("~voxel_clearance");
    nh.setParam("sdf_map/resolution", 0.1);
    nh.setParam("sdf_map/map_size_x", 4.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 2.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/max_ray_length", 20.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.1);
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    SafeRelayPublisher publisher;
    publisher.map_ = map;
    publisher.hold_half_height_ = 0.10;
    publisher.hold_radius_ = 0.30;
    const Eigen::Vector3d point(0.05, 0.05, 0.35);
    EXPECT_DOUBLE_EQ(0.0, publisher.horizontalClearance(point)); // UNKNOWN is a boundary.
    for (int y = -10; y <= 10; ++y)
      for (int z = 0; z <= 10; ++z) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ end;
        end.x = 20.0; end.y = 0.1*y + 0.05; end.z = 0.1*z + 0.05;
        cloud.push_back(end);
        map->inputPointCloud(cloud, 1, Eigen::Vector3d(-1.9, end.y, end.z));
      }
    ASSERT_TRUE(publisher.volumeFree(point, 0.3, 0.1));
    EXPECT_DOUBLE_EQ(0.6, publisher.horizontalClearance(point));
    auto occupy = [&](const Eigen::Vector3d& obstacle) {
      // setOccupied changes only inflation; real hits must update raw occupancy.
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.push_back(pcl::PointXYZ(obstacle.x(), obstacle.y(), obstacle.z()));
      for (int i = 0; i < 20; ++i) map->inputPointCloud(cloud, 1, point);
      ASSERT_EQ(SDFMap::OCCUPIED, map->getOccupancy(obstacle));
    };
    occupy(Eigen::Vector3d(0.05, 0.05, 0.05)); // Ground outside +/-0.1m.
    EXPECT_DOUBLE_EQ(0.6, publisher.horizontalClearance(point));
    occupy(Eigen::Vector3d(0.45, 0.05, 0.35));
    EXPECT_NEAR(0.35, publisher.horizontalClearance(point), 1e-9); // BOX face, not centre distance 0.4.
    EXPECT_TRUE(publisher.volumeFree(point, 0.3, 0.1));
    occupy(Eigen::Vector3d(0.25, 0.05, 0.35));
    EXPECT_NEAR(0.15, publisher.horizontalClearance(point), 1e-9);
    EXPECT_FALSE(publisher.volumeFree(point, 0.3, 0.1));
    EXPECT_LT(publisher.horizontalClearance(Eigen::Vector3d(1.85, 0.05, 0.35)), 0.3);
  }

  static void forwardVerticalPocket(bool mirror, bool uniform_floor, bool unknown_ahead,
                                    bool pair_height = false, bool block_paired_height = false) {
    ros::NodeHandle nh("~forward_height_pocket");
    nh.setParam("sdf_map/resolution", 0.05);
    nh.setParam("sdf_map/map_size_x", 4.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 2.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/max_ray_length", 20.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.05);
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    SafeRelayPublisher publisher;
    publisher.map_ = map;
    publisher.planner_ = std::make_shared<FastPlannerManager>();
    publisher.hold_radius_ = 0.30;
    publisher.hold_half_height_ = 0.10;
    const double sign = mirror ? -1.0 : 1.0;
    const Eigen::Vector3d reference(0.025, 0.025, 0.475);
    const Eigen::Vector2d forward(sign, 0.0);
    // 填充真实射线已知空间；未知前方用例只观测参考点及其原停留包络。
    for (int x = -25; x <= 25; ++x)
      for (int z = 0; z <= 23; ++z) {
        if (unknown_ahead && x > 6 && (z < 7 || z > 11)) continue;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        const pcl::PointXYZ end(0.05*x + 0.025, 20.0, 0.05*z + 0.025);
        cloud.push_back(end);
        // 连续观测两次，避免地图char帧计数回绕与未观测初值相同而漏射线。
        for (int repeat = 0; repeat < 2; ++repeat)
          map->inputPointCloud(cloud, 1, Eigen::Vector3d(end.x, -1.9, end.z));
      }
    const auto occupy = [&](double x) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.push_back(pcl::PointXYZ(x, reference.y(), 0.225));
      for (int repeat = 0; repeat < 10; ++repeat)
        map->inputPointCloud(cloud, 1, reference);
    };
    // 障碍低于原上下0.1m的检测带，但落在高度漂移余量中。
    occupy(reference.x());
    if (uniform_floor) {
      pcl::PointCloud<pcl::PointXYZ> floor;
      for (int x = -6; x <= 20; ++x)
        for (int y = -6; y <= 6; ++y)
          floor.push_back(pcl::PointXYZ(reference.x() + sign*0.05*x,
                                       reference.y() + 0.05*y, 0.225));
      // 对每个低面体素竖直入射，避免大量斜射线把相邻端点计为MISS。
      for (const auto& hit : floor) {
        pcl::PointCloud<pcl::PointXYZ> column;
        column.push_back(hit);
        for (int repeat = 0; repeat < 10; ++repeat)
          map->inputPointCloud(column, 1, Eigen::Vector3d(hit.x, hit.y, reference.z()));
        ASSERT_EQ(SDFMap::OCCUPIED, map->getOccupancy(Eigen::Vector3d(hit.x,hit.y,hit.z)));
      }
    }
    ASSERT_TRUE(publisher.volumeFree(reference, 0.30, 0.10));
    ASSERT_NEAR(0.60, publisher.horizontalClearance(reference), 1e-6);
    SafeRelayPublisher::Record record;
    record.sample.position = reference;
    record.candidate = record.sample;
    if (pair_height) {
      publisher.history_.push_back(record);
      auto ahead = record;
      ahead.sample.position += Eigen::Vector3d(0.40, 0.0, 0.10);
      ahead.sample.progress = 0.42;
      publisher.history_.push_back(ahead);
      if (block_paired_height) {
        for (int x = 5; x <= 20; ++x)
          for (int y = -6; y <= 6; ++y) {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.push_back(pcl::PointXYZ(0.05*x + 0.025, 0.05*y + 0.025, 0.675));
            for (int repeat = 0; repeat < 10; ++repeat)
              map->inputPointCloud(cloud, 1, Eigen::Vector3d(cloud[0].x, cloud[0].y, 0.825));
          }
      }
    }
    publisher.qualifyCandidate(record, true, forward);
    ASSERT_TRUE(record.candidate.hold_free);
    EXPECT_FALSE(record.candidate.confirmed);
    const Eigen::Vector3d selected = record.candidate.position;
    EXPECT_NEAR(reference.z() + (pair_height && !block_paired_height ? 0.10 : 0.0),
                selected.z(), 1e-6);
    if (uniform_floor || unknown_ahead || block_paired_height) {
      EXPECT_TRUE(selected.isApprox(reference)) << selected.transpose();
    } else {
      EXPECT_GE((selected.head<2>() - reference.head<2>()).dot(forward), 0.30);
      EXPECT_LE((selected - reference).norm(), 0.60 + 1e-6);
      EXPECT_GT(publisher.verticalClearance(selected), publisher.verticalClearance(reference));
      EXPECT_NEAR(reference.y(), selected.y(), 1e-6);
    }
    // 同一帧不能增加确认；重复更新不从上次候选继续前移。
    publisher.qualifyCandidate(record, false, forward);
    EXPECT_FALSE(record.candidate.confirmed);
    EXPECT_TRUE(record.candidate.position.isApprox(selected));
    for (int i = 0; i < 5; ++i) publisher.qualifyCandidate(record, true, forward);
    EXPECT_TRUE(record.candidate.confirmed);
    EXPECT_TRUE(record.candidate.position.isApprox(selected));
    EXPECT_DOUBLE_EQ(selected.z(), record.candidate.position.z());
  }

  static void routeHeightPairing() {
    SafeRelayPublisher publisher;
    const auto append = [&](double progress, double x, double y, double z) {
      SafeRelayPublisher::Record r;
      r.sample.progress = progress;
      r.sample.position = Eigen::Vector3d(x, y, z);
      r.candidate = r.sample;
      publisher.history_.push_back(r);
    };
    // 本次2号点：后方低样本的XY移回已飞过的较高区域。
    append(2.37, 3.605, 0.088, 0.263);
    append(2.87, 3.976, -0.271, 0.213);
    EXPECT_DOUBLE_EQ(0.263, publisher.routeHeight(publisher.history_[1],
        Eigen::Vector3d(3.576, -0.171, 0.213)));
    // 本次3号点：同路段前方已有实飞证据，不能保留原位置的下降高度。
    append(3.78, 4.726, -0.345, 0.193);
    append(4.16, 5.013, -0.134, 0.301);
    EXPECT_DOUBLE_EQ(0.301, publisher.routeHeight(publisher.history_[2],
        Eigen::Vector3d(5.026, -0.145, 0.193)));
    // 折返的另一段路线即使XY完全重合，也不能借来高度。
    append(8.0, 5.026, -0.145, 1.20);
    EXPECT_DOUBLE_EQ(0.301, publisher.routeHeight(publisher.history_[2],
        Eigen::Vector3d(5.026, -0.145, 0.193)));
    EXPECT_DOUBLE_EQ(0.193, publisher.routeHeight(publisher.history_[2],
        publisher.history_[2].sample.position));
    // 真实低位穿越仍取低高度，不应用统一最低高度。
    EXPECT_DOUBLE_EQ(0.193, publisher.routeHeight(publisher.history_[3],
        Eigen::Vector3d(4.726, -0.345, 0.301)));
    EXPECT_DOUBLE_EQ(0.193, publisher.history_[2].sample.position.z());
  }

  static void lowObstacleHeight(bool ceiling, bool unknown_above, bool obstacle) {
    ros::NodeHandle nh("~low_obstacle_height");
    nh.setParam("sdf_map/resolution", 0.05);
    nh.setParam("sdf_map/map_size_x", 4.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 2.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/max_ray_length", 20.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.05);
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    SafeRelayPublisher publisher;
    publisher.map_ = map;
    publisher.planner_ = std::make_shared<FastPlannerManager>();
    publisher.hold_radius_ = 0.30;
    publisher.hold_half_height_ = 0.10;
    publisher.candidate_search_radius_ = 0.01;
    const Eigen::Vector3d reference(0.025, 0.025, 0.475);
    for (int x = -25; x <= 25; ++x)
      for (int z = 0; z <= (unknown_above ? 11 : 23); ++z) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        const pcl::PointXYZ end(0.05*x + 0.025, 20.0, 0.05*z + 0.025);
        cloud.push_back(end);
        // 连续观测两次，避免地图char帧计数回绕与未观测初值相同而漏射线。
        for (int repeat = 0; repeat < 2; ++repeat)
          map->inputPointCloud(cloud, 1, Eigen::Vector3d(end.x, -1.9, end.z));
      }
    const auto occupy = [&](double z) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.push_back(pcl::PointXYZ(reference.x(), reference.y(), z));
      for (int repeat = 0; repeat < 10; ++repeat)
        map->inputPointCloud(cloud, 1, reference);
    };
    if (obstacle) occupy(0.325);  // 体素顶面0.35，原高度的底部余量不足。
    if (ceiling) occupy(0.625);
    ASSERT_TRUE(publisher.volumeFree(reference, 0.30, 0.10));
    SafeRelayPublisher::Record record;
    record.sample.position = reference;
    record.candidate = record.sample;
    publisher.qualifyCandidate(record, true, Eigen::Vector2d::Zero());
    ASSERT_TRUE(record.candidate.hold_free);
    const Eigen::Vector3d selected = record.candidate.position;
    EXPECT_TRUE(selected.head<2>().isApprox(reference.head<2>()));
    if (obstacle && !ceiling && !unknown_above) {
      EXPECT_NEAR(0.575, selected.z(), 1e-6);  // 最小抬高，不一律抬到上限。
      EXPECT_GE(selected.z() - 0.15 - 0.35, 0.05);
    } else {
      EXPECT_DOUBLE_EQ(reference.z(), selected.z());
    }
    EXPECT_FALSE(record.candidate.confirmed);
    publisher.qualifyCandidate(record, false, Eigen::Vector2d::Zero());
    EXPECT_FALSE(record.candidate.confirmed);
    for (int i = 0; i < 5; ++i)
      publisher.qualifyCandidate(record, true, Eigen::Vector2d::Zero());
    EXPECT_TRUE(record.candidate.confirmed);
    EXPECT_TRUE(selected.isApprox(record.candidate.position));
    EXPECT_TRUE(reference.isApprox(record.sample.position));
  }

  static void unflownPocketAndGap() {
    ros::NodeHandle nh("~unflown_pocket");
    nh.setParam("sdf_map/resolution", 0.1);
    nh.setParam("sdf_map/map_size_x", 4.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 2.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/max_ray_length", 20.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.1);
    nh.setParam("mission/safe_relay/enabled", true);
    nh.setParam("mission/safe_relay/hold_radius", 0.30);
    nh.setParam("mission/safe_relay/hold_half_height", 0.10);
    nh.setParam("mission/safe_relay/topic_prefix", std::string("/safe_relay_pocket_test"));
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    SafeRelayPublisher publisher;
    publisher.initialize(nh, map, std::make_shared<FastPlannerManager>());
    publisher.timer_.stop();
    nav_msgs::Path route, goals;
    auto route_sub = nh.subscribe<nav_msgs::Path>("/safe_relay_pocket_test/reference_route", 1,
        [&](const nav_msgs::Path::ConstPtr& msg) { route = *msg; });
    auto goals_sub = nh.subscribe<nav_msgs::Path>("/safe_relay_pocket_test/relay_waypoints", 1,
        [&](const nav_msgs::Path::ConstPtr& msg) { goals = *msg; });
    auto spin = []() {
      for (int i = 0; i < 10; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
    };
    spin();
    ros::Time::setNow(ros::Time(300));
    publisher.setEntrance(Eigen::Vector3d(-1.0, 0.0, 0.45), Eigen::Vector3d::UnitX());
    for (int i = 0; i <= 15; ++i)
      publisher.observe(Eigen::Vector3d(-1.0 + 0.1*i, 0.0, 0.45), 0,
                        ros::Time(299.8 + 0.01*i));
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(publisher.goals_.empty()); // UNKNOWN must not create an unflown goal.
    const auto map_frame = [&]() {
      // Leader flies near the lower boundary y=-0.30; the wider stopping
      // pocket lies at y=+0.30 and has never been traversed by the leader.
      for (int y = -3; y <= 9; ++y)
        for (int z = 2; z <= 7; ++z) {
          pcl::PointCloud<pcl::PointXYZ> cloud;
          const pcl::PointXYZ end(20.0, 0.1*y + 0.05, 0.1*z + 0.05);
          cloud.push_back(end);
          map->inputPointCloud(cloud, 1, Eigen::Vector3d(-1.9, end.y, end.z));
        }
    };
    map_frame();
    publisher.timerCallback(ros::TimerEvent());
    publisher.timerCallback(ros::TimerEvent());
    EXPECT_TRUE(publisher.goals_.empty()); // Same map cannot confirm a shifted point twice.
    map_frame();
    publisher.timerCallback(ros::TimerEvent());
    ASSERT_EQ(1U, publisher.goals_.size());
    EXPECT_EQ("GAP_FALLBACK", publisher.last_status_);
    const auto released = publisher.goals_.front();
    const auto& p = released.pose.pose.position;
    EXPECT_GE(p.y, 0.29);
    EXPECT_DOUBLE_EQ(0.45, p.z);
    EXPECT_GE(publisher.horizontalClearance(Eigen::Vector3d(p.x, p.y, p.z)), 0.59);
    EXPECT_TRUE(publisher.volumeFree(Eigen::Vector3d(p.x, p.y, p.z), 0.30, 0.10));
    EXPECT_LT((Eigen::Vector3d(p.x, p.y, p.z) - publisher.leader_).norm(), 1.0);
    spin();
    ASSERT_EQ(16U, route.poses.size());
    ASSERT_EQ(1U, goals.poses.size());
    for (const auto& pose : route.poses) EXPECT_DOUBLE_EQ(0.0, pose.pose.position.y);
    EXPECT_DOUBLE_EQ(p.y, goals.poses.front().pose.position.y);
    EXPECT_EQ(ros::Time(300), goals.poses.front().header.stamp);
    // The immutable goal has its own coordinates; it is not reconstructed from
    // the measured reference route when that endpoint is later occupied.
    map->setOccupied(Eigen::Vector3d(p.x, p.y, p.z));
    publisher.timerCallback(ros::TimerEvent());
    spin();
    ASSERT_EQ(1U, goals.poses.size());
    EXPECT_DOUBLE_EQ(p.y, goals.poses.front().pose.position.y);
    EXPECT_EQ(released.id, goals.poses.front().header.seq);
  }
};
TEST(SafeRelayPublisher, NewEndpointsAndRetainedIssuedPoints) { SafeRelayPublisherTest::run(); }
TEST(SafeRelayPublisher, EntranceFallbackUsesHalfMeterThenConfiguredSpacing) {
  SafeRelayPublisherTest::entranceSpacing();
}
TEST(SafeRelayPublisher, RawVoxelBoxesAndLowHeightClearance) {
  SafeRelayPublisherTest::voxelClearance();
}
TEST(SafeRelayPublisher, UnflownKnownFreePocketAndCurrentPositionGapFallback) {
  SafeRelayPublisherTest::unflownPocketAndGap();
}
TEST(SafeRelayPublisher, BetterForwardVerticalPocketKeepsHeightAndDoesNotDrift) {
  SafeRelayPublisherTest::forwardVerticalPocket(false, false, false);
}
TEST(SafeRelayPublisher, NegativeXProgressUsesForwardPocketInsteadOfBodyYaw) {
  SafeRelayPublisherTest::forwardVerticalPocket(true, false, false);
}
TEST(SafeRelayPublisher, NoBetterVerticalSpaceKeepsOriginalPoint) {
  SafeRelayPublisherTest::forwardVerticalPocket(false, true, false);
}
TEST(SafeRelayPublisher, UnknownForwardSpaceCannotMoveHoldPoint) {
  SafeRelayPublisherTest::forwardVerticalPocket(false, false, true);
}
TEST(SafeRelayPublisher, OffsetXYUsesLocalFlownHeightAndIgnoresOtherRouteLeg) {
  SafeRelayPublisherTest::routeHeightPairing();
}
TEST(SafeRelayPublisher, ForwardCandidateUsesPairedHeightAndRemainsConfirmed) {
  SafeRelayPublisherTest::forwardVerticalPocket(false, false, false, true, false);
}
TEST(SafeRelayPublisher, BlockedPairedHeightCannotFallBackToOldLowZAtSameXY) {
  SafeRelayPublisherTest::forwardVerticalPocket(false, false, false, true, true);
}
TEST(SafeRelayPublisher, LowObstacleRaisesOnlyEnoughForBodyAndMargin) {
  SafeRelayPublisherTest::lowObstacleHeight(false, false, true);
}
TEST(SafeRelayPublisher, LowCeilingKeepsOriginalQualifiedHeight) {
  SafeRelayPublisherTest::lowObstacleHeight(true, false, true);
}
TEST(SafeRelayPublisher, UnknownAboveCannotRaiseHoldPoint) {
  SafeRelayPublisherTest::lowObstacleHeight(false, true, true);
}
TEST(SafeRelayPublisher, OpenSpaceDoesNotRaiseHoldPoint) {
  SafeRelayPublisherTest::lowObstacleHeight(false, false, false);
}
}  // namespace fast_planner
int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_relay_publisher_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
