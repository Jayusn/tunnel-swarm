#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <random>
#include <queue>
#include <map>
#include <array>
#include <functional>
#include <optimizer/poly_traj_utils.hpp>
#include <optimizer/lbfgs.hpp>

// 仅测试翻译单元访问地图缓冲区，避免传感器回放时序影响选边回归。
#define private public
#include <optimizer/poly_traj_optimizer.h>
#undef private

namespace
{
using Eigen::Vector3d;
using diff_planner::PolyTrajOptimizer;
using diff_planner::VerticalObstacleSideLock;

class StaticSidePlanning : public testing::Test
{
protected:
  void SetUp() override
  {
    map.reset(new GridMap);
    map->mp_.resolution_ = 0.05;
    map->mp_.resolution_inv_ = 20.0;
    map->mp_.enable_virtual_wall_ = true;
    map->mp_.virtual_ground_ = 0.0;
    map->mp_.virtual_ceil_ = 1.5;
    auto &data = map->md_;
    data.ringbuffer_inf_origin3i_ = Eigen::Vector3i(-30, -40, 0);
    data.ringbuffer_inf_size3i_ = Eigen::Vector3i(60, 80, 30);
    data.ringbuffer_inf_lowbound3d_ = Vector3d(-1.5, -2.0, 0.0);
    data.ringbuffer_inf_upbound3d_ = Vector3d(1.5, 2.0, 1.5);
    data.occupancy_buffer_inflate_.resize(60 * 80 * 30);
    fillCorridor(-0.10, 0.25);
  }

  void fillCorridor(double divider_min_y, double divider_max_y)
  {
    for (int z = 0; z < 30; ++z)
      for (int y = -40; y < 40; ++y)
        for (int x = -30; x < 30; ++x)
        {
          const Eigen::Vector3i cell(x, y, z);
          const Vector3d p = map->globalIdx2Pos(cell);
          const bool wall = std::abs(p.y()) >= 0.8;
          const bool divider = p.x() >= -0.25 && p.x() <= 0.25 &&
              p.y() >= divider_min_y && p.y() <= divider_max_y;
          map->md_.occupancy_buffer_inflate_[map->globalIdx2InfBufIdx(cell)] =
              wall || divider;
        }
  }

  void configure(PolyTrajOptimizer &optimizer, bool fapp)
  {
    optimizer.dynamic_obstacle_config_.enabled = fapp;
    optimizer.max_planning_wall_time_ = 5.0;
    optimizer.obs_clearance_ = 0.1;
    optimizer.obs_clearance_soft_ = 0.15;
    optimizer.wei_obs_ = optimizer.wei_obs_soft_ = 10000.0;
    optimizer.wei_feas_ = optimizer.wei_swarm_ = 10000.0;
    optimizer.wei_sqrvar_ = 100.0;
    optimizer.wei_time_ = 5.0;
    optimizer.wei_dynamic_obstacle_ = 10000.0;
    optimizer.max_vel_ = 1.0;
    optimizer.max_acc_ = 1.0;
    optimizer.max_jer_ = 8.0;
    optimizer.vel_tolerance_ = optimizer.acc_tolerance_ = 1.0;
    optimizer.cps_num_prePiece_ = 6;
    optimizer.swarm_clearance_ = 0.15;
    optimizer.setSwarmTrajs(&swarm);
    optimizer.setDroneId(1);
    optimizer.setIfTouchGoal(true);
    optimizer.setEnvironment(map);
    VerticalObstacleSideLock::Config config;
    config.enabled = true;
    config.width_similarity_tolerance = 0.05;
    config.search_timeout = 1.0;
    optimizer.vertical_obstacle_side_lock_.setConfig(config);
  }

  void expectLowerPassage(const std::vector<Vector3d> &path)
  {
    ASSERT_FALSE(path.empty());
    bool crossed = false;
    for (const auto &p : path)
    {
      EXPECT_EQ(map->getInflateOccupancy(p), 0);
      if (std::abs(p.x()) <= 0.25)
      {
        crossed = true;
        EXPECT_LE(p.y(), -0.10 + 1e-6);
      }
    }
    EXPECT_TRUE(crossed);
  }

  GridMap::Ptr map;
  diff_planner::SwarmTrajData swarm;
  const Vector3d start{-0.9, 0.0, 0.6}, end{0.9, 0.0, 0.6};
};

TEST_F(StaticSidePlanning, FappEnabledAndDisabledBothChooseWiderStaticPassage)
{
  for (bool enabled : {false, true})
  {
    PolyTrajOptimizer optimizer;
    configure(optimizer, enabled);
    optimizer.beginPlanningCycle();
    std::vector<Vector3d> path;
    ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
    EXPECT_TRUE(optimizer.vertical_obstacle_side_lock_.lock_.active);
    expectLowerPassage(path);
    optimizer.endPlanningCycle();
  }
}

TEST_F(StaticSidePlanning, SingleUsableSideAndBothBlockedDoNotFallBackToNarrowSide)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  fillCorridor(-0.10, 0.65);
  std::vector<Vector3d> path;
  ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
  expectLowerPassage(path);
  fillCorridor(-0.65, 0.65);
  EXPECT_NE(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
}

TEST_F(StaticSidePlanning, SimilarWidthsKeepSelectedSideAcrossReversedSearch)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  std::vector<Vector3d> path;
  ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
  expectLowerPassage(path);
  fillCorridor(-0.20, 0.20);
  ASSERT_EQ(optimizer.searchObstaclePath(end, start, path), ASTAR_RET::SUCCESS);
  expectLowerPassage(path);
}

TEST_F(StaticSidePlanning, IncompleteObservationRetainsLockForSameObstacle)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  std::vector<Vector3d> path;
  ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
  const double offset = optimizer.vertical_obstacle_side_lock_.lock_.passage_center_offset;
  // 分隔物暂时漏观测，不能因这一帧缺失而改走另一侧。
  fillCorridor(0.1, -0.1);
  ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
  expectLowerPassage(path);
  EXPECT_TRUE(optimizer.vertical_obstacle_side_lock_.lock_.active);
  EXPECT_DOUBLE_EQ(optimizer.vertical_obstacle_side_lock_.lock_.passage_center_offset, offset);
}

TEST_F(StaticSidePlanning, DynamicCandidateRetainsItsOwnRegion)
{
  fillCorridor(-0.20, 0.10);
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  optimizer.beginDynamicCandidate(1.0);
  optimizer.dynamic_candidate_active_ = true;
  auto &region = optimizer.dynamic_candidate_region_;
  region.enabled = region.limit_passage = true;
  region.side_axis = Vector3d::UnitX();
  region.side_normal = Vector3d::UnitY();
  region.side_origin = Vector3d(0.0, 0.0, 0.6);
  region.side_half_length = 0.3;
  region.passage_min_offset = 0.2;
  region.passage_max_offset = 0.65;
  std::vector<Vector3d> path;
  ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
  for (const auto &p : path)
    if (std::abs(p.x()) <= 0.3) {
      EXPECT_GE(p.y(), 0.2 - 1e-6);
    }
  EXPECT_FALSE(optimizer.vertical_obstacle_side_lock_.lock_.active);
}

TEST_F(StaticSidePlanning, OptimizedTrajectoryStaysInSelectedStaticPassageWithFappOnOrOff)
{
  for (bool enabled : {false, true})
  {
    PolyTrajOptimizer optimizer;
    configure(optimizer, enabled);
    optimizer.beginPlanningCycle();
    poly_traj::MinJerkOpt seed;
    Eigen::Matrix3d head, tail;
    head << start, Vector3d::Zero(), Vector3d::Zero();
    tail << end, Vector3d::Zero(), Vector3d::Zero();
    Eigen::MatrixXd midpoint(3, 1);
    midpoint.col(0) = 0.5 * (start + end);
    const Eigen::VectorXd durations = Eigen::Vector2d::Constant(2.0);
    seed.reset(head, tail, 2);
    seed.generate(midpoint, durations);
    std::vector<std::pair<int, int>> segments;
    ASSERT_NE(optimizer.finelyCheckAndSetConstraintPoints(segments, seed, true),
              PolyTrajOptimizer::ERR);
    double cost;
    ASSERT_TRUE(optimizer.optimizeTrajectory(head, tail, midpoint, durations, cost));
    const auto trajectory = optimizer.getMinJerkOpt().getTraj();
    bool crossed = false;
    for (int i = 0; i <= 1000; ++i)
    {
      const auto p = trajectory.getPos(trajectory.getTotalDuration() * i / 1000.0);
      EXPECT_EQ(map->getInflateOccupancy(p), 0);
      if (std::abs(p.x()) <= 0.25)
      {
        crossed = true;
        EXPECT_LT(p.y(), -0.10);
      }
    }
    EXPECT_TRUE(crossed);
    optimizer.endPlanningCycle();
  }
}
class UnderpassEndpointPlanning : public StaticSidePlanning
{
protected:
  void SetUp() override
  {
    StaticSidePlanning::SetUp();
    for (int z = 0; z < 30; ++z)
      for (int y = -40; y < 40; ++y)
        for (int x = -30; x < 30; ++x)
        {
          const Eigen::Vector3i cell(x, y, z);
          const Vector3d p = map->globalIdx2Pos(cell);
          const bool suspended = std::abs(p.x()) <= 0.25 &&
              std::abs(p.y()) <= 0.4 && p.z() >= 0.5 && p.z() <= 0.8;
          map->md_.occupancy_buffer_inflate_[map->globalIdx2InfBufIdx(cell)] =
              std::abs(p.y()) >= 0.8 || suspended;
        }
  }

  void configureUnderpass(PolyTrajOptimizer &optimizer)
  {
    configure(optimizer, true);
    diff_planner::SuspendedObstacleUnderpass::Config config;
    config.enabled = true;
    config.min_height = 0.15;
    config.search_timeout = 1.0;
    optimizer.suspended_underpass_.setConfig(config);
  }
};

TEST_F(UnderpassEndpointPlanning, InvalidLocalEndpointCanRetryWithExpandedEndpoints)
{
  PolyTrajOptimizer optimizer;
  configureUnderpass(optimizer);
  std::vector<Vector3d> path;
  ASSERT_EQ(map->getInflateOccupancy(start), 0);
  ASSERT_EQ(map->getInflateOccupancy(end), 0);
  EXPECT_EQ(optimizer.searchObstaclePath(Vector3d(0.0, 0.0, 0.6), end, path),
            ASTAR_RET::INIT_ERR);
  EXPECT_TRUE(path.empty());
  ASSERT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::SUCCESS);
  ASSERT_FALSE(path.empty());
  bool passed_below = false;
  for (const auto &p : path)
  {
    EXPECT_EQ(map->getInflateOccupancy(p), 0);
    EXPECT_GE(p.z(), 0.15 - 1e-6);
    passed_below = passed_below || (std::abs(p.x()) <= 0.25 && p.z() < 0.5);
  }
  EXPECT_TRUE(passed_below);
}

TEST_F(UnderpassEndpointPlanning, FineCollisionCheckExecutesExistingEndpointExpansion)
{
  PolyTrajOptimizer optimizer;
  configureUnderpass(optimizer);
  optimizer.beginPlanningCycle();
  const auto occupancy_checker = optimizer.a_star_->occupancy_checker_;
  std::vector<Vector3d> attempted_starts;
  int last_round = 0;
  optimizer.a_star_->setOccupancyChecker([&](const Vector3d &p) {
    const int round = optimizer.a_star_->rounds_;
    if (round != last_round)
    {
      attempted_starts.push_back(p);
      last_round = round;
    }
    // 确定性注入第一次局部端点初始化失败，验证真实外层循环会外扩重试。
    return round == 1 || occupancy_checker(p);
  });
  poly_traj::MinJerkOpt seed;
  Eigen::Matrix3d head, tail;
  head << start, Vector3d::Zero(), Vector3d::Zero();
  tail << end, Vector3d::Zero(), Vector3d::Zero();
  Eigen::MatrixXd midpoint(3, 1);
  midpoint.col(0) = 0.5 * (start + end);
  seed.reset(head, tail, 2);
  seed.generate(midpoint, Eigen::Vector2d::Constant(2.0));
  std::vector<std::pair<int, int>> segments;
  EXPECT_EQ(optimizer.finelyCheckAndSetConstraintPoints(segments, seed, true),
            PolyTrajOptimizer::FINISH);
  ASSERT_GE(attempted_starts.size(), 2u);
  EXPECT_GT(attempted_starts[1].x(), attempted_starts[0].x());
  EXPECT_TRUE(optimizer.underpass_floor_active_);
  optimizer.endPlanningCycle();
}

TEST_F(UnderpassEndpointPlanning, ActualSearchFailureStillForbidsOrdinaryFallback)
{
  PolyTrajOptimizer optimizer;
  configureUnderpass(optimizer);
  const auto occupancy_checker = optimizer.a_star_->occupancy_checker_;
  // 下方空间已由地图确认，但碰撞查询拒绝穿越；不能误判为端点错误。
  optimizer.a_star_->setOccupancyChecker([&](const Vector3d &p) {
    return std::abs(p.x()) <= 0.25 || occupancy_checker(p);
  });
  std::vector<Vector3d> path;
  EXPECT_EQ(optimizer.searchObstaclePath(start, end, path), ASTAR_RET::POLICY_ERR);
  EXPECT_TRUE(path.empty());
  EXPECT_EQ(optimizer.a_star_->rounds_, 1);
}
} // namespace
