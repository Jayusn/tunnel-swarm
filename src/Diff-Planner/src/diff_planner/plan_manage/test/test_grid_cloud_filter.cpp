#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <limits>
#include <random>
#include <queue>
#include <tuple>

#define private public
#include <plan_env/grid_map.h>
#undef private

namespace
{
using Eigen::Vector3d;

class GridCloudFilter : public testing::Test
{
protected:
  void SetUp() override
  {
    ros::Time::setNow(ros::Time(1000));
    for (const auto &parameter : std::vector<std::pair<std::string, double>>{
        {"grid_map/resolution", 0.05}, {"grid_map/obstacles_inflation", 0.099},
        {"grid_map/local_update_range_x", 2.0},
        {"grid_map/local_update_range_y", 2.0},
        {"grid_map/local_update_range_z", 1.0}, {"grid_map/fading_time", 0.0},
        {"grid_map/near_point_filter_range", 1.0},
        {"grid_map/near_point_filter_radius", 0.05}})
      nh.setParam(parameter.first, parameter.second);
    nh.setParam("grid_map/near_point_filter_min_neighbors", 3);
    nh.setParam("grid_map/cloud_use_init_offset", false);
    nh.setParam("grid_map/lidar_hit_dominates_miss", true);
    map.reset(new GridMap);
    map->initMap(nh);
    map->occ_timer_.stop();
    map->vis_timer_.stop();
    setOrigin(Vector3d(0.0, 0.0, 0.5));
  }

  void TearDown() override
  {
    map.reset();
    ros::Time::init();
  }

  void setOrigin(const Vector3d &position)
  {
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->pose.pose.position.x = position.x();
    odom->pose.pose.position.y = position.y();
    odom->pose.pose.position.z = position.z();
    odom->pose.pose.orientation.w = 1.0;
    map->odomCallback(odom);
    map->moveRingBuffer();
  }

  void fuse(const std::vector<Vector3d> &points, int frames = 3)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto &point : points)
      cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    cloud.is_dense = false;
    sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud, *message);
    for (int i = 0; i < frames; ++i)
      map->cloudCallback(message);
  }

  double occupancyLog(const Vector3d &point)
  {
    return map->md_.occupancy_buffer_[
        map->globalIdx2BufIdx(map->pos2GlobalIdx(point))];
  }

  const std::vector<Vector3d> supported_patch{
      {0.51, 0.015, 0.515}, {0.52, 0.015, 0.515},
      {0.51, 0.025, 0.515}, {0.52, 0.025, 0.515}};
  ros::NodeHandle nh{"~grid_cloud_filter"};
  GridMap::Ptr map;
};

TEST_F(GridCloudFilter, IsolatedNearReturnsDoNotOccupyTheStart)
{
  fuse({Vector3d(0.515, 0.115, 0.515)});
  EXPECT_EQ(map->md_.proj_points_cnt_, 0);
  EXPECT_EQ(map->getInflateOccupancy(Vector3d(0.415, 0.015, 0.515)), 0);
  EXPECT_LT(occupancyLog(Vector3d(0.515, 0.115, 0.515)),
            map->mp_.lidar_min_occupancy_log_);
}

TEST_F(GridCloudFilter, RejectedReturnDoesNotClearAnExistingObstacle)
{
  fuse(supported_patch);
  const Vector3d obstacle(0.515, 0.015, 0.515);
  ASSERT_GT(map->getInflateOccupancy(obstacle), 0);
  const double before = occupancyLog(obstacle);
  // This ray would cross the occupied patch if the rejected endpoint were
  // still used to generate FREE evidence.
  fuse({Vector3d(0.815, 0.015, 0.515)}, 8);
  EXPECT_DOUBLE_EQ(occupancyLog(obstacle), before);
  EXPECT_GT(map->getInflateOccupancy(obstacle), 0);
}

TEST_F(GridCloudFilter, OnlyFusedObservationsAdvancePassageConfirmationRevision)
{
  const auto initial = map->getObservationRevision();
  fuse({}, 2);
  fuse({Vector3d(0.515, 0.115, 0.515)}, 3);
  EXPECT_EQ(map->getObservationRevision(), initial);
  fuse(supported_patch, 1);
  EXPECT_EQ(map->getObservationRevision(), initial + 1);
  setOrigin(Vector3d(0.0, 0.0, 0.5));
  EXPECT_EQ(map->getObservationRevision(), initial + 1);
  fuse(supported_patch, 1);
  EXPECT_EQ(map->getObservationRevision(), initial + 2);
}

TEST_F(GridCloudFilter, ThreeOtherPointsAreRequired)
{
  fuse({supported_patch[0], supported_patch[1], supported_patch[2]});
  EXPECT_EQ(map->md_.proj_points_cnt_, 0);
  EXPECT_EQ(map->getInflateOccupancy(supported_patch[0]), 0);
  fuse(supported_patch);
  EXPECT_EQ(map->md_.proj_points_cnt_, 4);
  EXPECT_GT(map->getInflateOccupancy(supported_patch[0]), 0);
}

TEST_F(GridCloudFilter, SupportedThinPoleRemainsOccupied)
{
  std::vector<Vector3d> pole;
  for (int i = 0; i <= 30; ++i)
    pole.emplace_back(0.565, 0.205, 0.35 + 0.01 * i);
  fuse(pole);
  EXPECT_EQ(map->md_.proj_points_cnt_, static_cast<int>(pole.size()));
  for (const auto &point : pole)
    EXPECT_GT(map->getInflateOccupancy(point), 0);
}

TEST_F(GridCloudFilter, SupportedSmallSuspendedObjectRemainsOccupied)
{
  std::vector<Vector3d> object;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z)
        object.emplace_back(0.65 + 0.02 * x, -0.2 + 0.02 * y,
                            0.85 + 0.02 * z);
  fuse(object);
  EXPECT_EQ(map->md_.proj_points_cnt_, static_cast<int>(object.size()));
  for (const auto &point : object)
    EXPECT_GT(map->getInflateOccupancy(point), 0);
}

TEST_F(GridCloudFilter, FarSparseReturnsKeepTheirOriginalUpdate)
{
  const Vector3d far_point(1.255, 0.015, 0.515);
  fuse({far_point});
  EXPECT_EQ(map->md_.proj_points_cnt_, 1);
  EXPECT_GT(map->getInflateOccupancy(far_point), 0);
}

TEST_F(GridCloudFilter, NeighborsOutsideTheCheckedRangeStillProvideSupport)
{
  const Vector3d near_point(0.995, 0.005, 0.505);
  fuse({near_point, Vector3d(1.005, 0.005, 0.505),
        Vector3d(1.015, 0.005, 0.505), Vector3d(1.025, 0.005, 0.505)});
  EXPECT_EQ(map->md_.proj_points_cnt_, 4);
  EXPECT_GT(map->getInflateOccupancy(near_point), 0);
}

TEST_F(GridCloudFilter, DisabledFilterKeepsLegacyNearReturns)
{
  map->mp_.near_point_filter_range_ = 0.0;
  fuse({supported_patch[0]});
  EXPECT_EQ(map->md_.proj_points_cnt_, 1);
  EXPECT_GT(map->getInflateOccupancy(supported_patch[0]), 0);
}

TEST_F(GridCloudFilter, WorldOffsetAndInvalidReturnsUseTheSameFiniteNeighborhood)
{
  map->mp_.cloud_use_init_offset_ = true;
  map->mp_.init_x_ = 4.0;
  map->mp_.init_y_ = -2.0;
  map->mp_.init_z_ = 0.4;
  setOrigin(Vector3d(4.0, -2.0, 0.9));
  auto input = supported_patch;
  input.emplace_back(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  fuse(input);
  EXPECT_EQ(map->md_.proj_points_cnt_, 4);
  EXPECT_GT(map->getInflateOccupancy(
      supported_patch[0] + Vector3d(4.0, -2.0, 0.4)), 0);
}

} // namespace
