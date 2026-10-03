#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <traj_utils/plan_container.hpp>
#include <optimizer/poly_traj_utils.hpp>

// Seed exact occupied cells and an existing lock without a running sensor stack.
#define private public
#include <optimizer/poly_traj_optimizer.h>
#undef private

namespace diff_planner
{
namespace
{
class ObstacleSearchFallback : public testing::Test
{
protected:
  GridMap::Ptr map{new GridMap};
  PolyTrajOptimizer optimizer;
  const Eigen::Vector3d start{-0.9, 0.0, 0.8}, end{0.9, 0.0, 0.8};

  void SetUp() override
  {
    ros::Time::init();
    map->mp_.resolution_ = 0.1;
    map->mp_.resolution_inv_ = 10.0;
    map->mp_.enable_virtual_wall_ = true;
    map->mp_.virtual_ground_ = 0.05;
    map->mp_.virtual_ceil_ = 1.55;
    map->md_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(-30, -30, 0);
    map->md_.ringbuffer_inf_size3i_ = Eigen::Vector3i(60, 60, 20);
    map->md_.ringbuffer_inf_lowbound3d_ = Eigen::Vector3d(-3.0, -3.0, 0.0);
    map->md_.ringbuffer_inf_upbound3d_ = Eigen::Vector3d(2.99, 2.99, 1.99);
    map->md_.occupancy_buffer_inflate_.assign(60 * 60 * 20, 0);
    optimizer.grid_map_ = map;
    optimizer.a_star_.reset(new AStar);
    optimizer.a_star_->initGridMap(map, Eigen::Vector3i(44, 44, 24));
  }

  void box(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
  {
    for (int x = -30; x < 30; ++x)
      for (int y = -30; y < 30; ++y)
        for (int z = 0; z < 20; ++z)
        {
          const Eigen::Vector3d p((x + 0.5) * 0.1, (y + 0.5) * 0.1, (z + 0.5) * 0.1);
          if (p.x() >= xmin && p.x() <= xmax && p.y() >= ymin && p.y() <= ymax &&
              p.z() >= zmin && p.z() <= zmax)
            map->md_.occupancy_buffer_inflate_[map->globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z))] = 1;
        }
  }

  void lockPositiveSide(bool fallback = true)
  {
    auto &side = optimizer.vertical_obstacle_side_lock_;
    VerticalObstacleSideLock::Config config;
    config.enabled = true;
    config.failure_fallback = fallback;
    config.width_similarity_tolerance = 1.0;
    side.setConfig(config);
    side.lock_.active = true;
    side.lock_.center = Eigen::Vector3d(0, 0, 0.8);
    side.lock_.axis = Eigen::Vector3d::UnitX();
    side.lock_.normal = Eigen::Vector3d::UnitY();
    side.lock_.half_length = 0.2;
    side.lock_.passage_center_offset = 0.4;
  }

  void expectFreePath(const std::vector<Eigen::Vector3d> &path)
  {
    ASSERT_FALSE(path.empty());
    for (const auto &point : path)
      EXPECT_EQ(0, map->getInflateOccupancy(point)) << point.transpose();
  }
};

TEST_F(ObstacleSearchFallback, SwitchesToOppositeSideAfterTwoFailedRounds)
{
  box(-0.16, 0.16, -0.16, 0.16, 0, 1.6);
  box(0.14, 0.45, 0.14, 0.9, 0, 1.6);
  lockPositiveSide();
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(ASTAR_RET::SEARCH_ERR, optimizer.searchObstaclePath(start, end, path));
  EXPECT_EQ(ASTAR_RET::SUCCESS, optimizer.searchObstaclePath(start, end, path));
  expectFreePath(path);
  EXPECT_LT(optimizer.vertical_obstacle_side_lock_.lock_.normal.y(), 0.0);
  EXPECT_EQ(0, optimizer.vertical_obstacle_side_lock_.lock_.failed_rounds);
  EXPECT_EQ(ASTAR_RET::SUCCESS, optimizer.searchObstaclePath(start, end, path));
  EXPECT_LT(optimizer.vertical_obstacle_side_lock_.lock_.normal.y(), 0.0);
}

TEST_F(ObstacleSearchFallback, UsesOrdinaryPathWhenBothConstrainedSidesFail)
{
  box(-0.15, 0.15, -0.95, 0.95, 0, 1.6);
  lockPositiveSide();
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(ASTAR_RET::SEARCH_ERR, optimizer.searchObstaclePath(start, end, path));
  EXPECT_EQ(ASTAR_RET::SUCCESS, optimizer.searchObstaclePath(start, end, path));
  expectFreePath(path);
  EXPECT_FALSE(optimizer.vertical_obstacle_side_lock_.lock_.active);
}

TEST_F(ObstacleSearchFallback, DisabledFallbackPreservesSideLockFailure)
{
  box(-0.15, 0.15, -0.95, 0.95, 0, 1.6);
  lockPositiveSide(false);
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(ASTAR_RET::SEARCH_ERR, optimizer.searchObstaclePath(start, end, path));
  EXPECT_EQ(ASTAR_RET::SEARCH_ERR, optimizer.searchObstaclePath(start, end, path));
  EXPECT_TRUE(optimizer.vertical_obstacle_side_lock_.lock_.active);
}

TEST_F(ObstacleSearchFallback, UnderpassFailureFallsBackToCollisionFreeSideRoute)
{
  box(-0.4, 0.4, -0.25, 0.25, 0.7, 1.1);
  box(-0.7, -0.4, -0.25, 0.25, 0.0, 0.7);
  box(0.4, 0.7, -0.25, 0.25, 0.0, 0.7);
  SuspendedObstacleUnderpass::Config config;
  config.enabled = true;
  config.corridor_half_width = 0.1;
  config.min_vertical_gap = 0.35;
  config.min_height = 0.15;
  config.search_timeout = 0.2;
  optimizer.suspended_underpass_.setConfig(config);
  std::vector<Eigen::Vector3d> path;
  ASSERT_EQ(SuspendedObstacleUnderpass::SearchResult::SEARCH_FAILED,
            optimizer.suspended_underpass_.trySearch(*optimizer.a_star_, map, 0.1, start, end, path));
  EXPECT_EQ(ASTAR_RET::POLICY_ERR, optimizer.searchObstaclePath(start, end, path));
  optimizer.underpass_failure_fallback_ = true;
  EXPECT_EQ(ASTAR_RET::SUCCESS, optimizer.searchObstaclePath(start, end, path));
  expectFreePath(path);
}

TEST_F(ObstacleSearchFallback, ReturnsFailureWhenNoOrdinaryRouteExists)
{
  box(-0.15, 0.15, -3.0, 3.0, 0, 1.6);
  lockPositiveSide();
  optimizer.underpass_failure_fallback_ = true;
  std::vector<Eigen::Vector3d> path;
  EXPECT_NE(ASTAR_RET::SUCCESS, optimizer.searchObstaclePath(start, end, path));
  EXPECT_TRUE(path.empty());
}
} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_search_fallback_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
