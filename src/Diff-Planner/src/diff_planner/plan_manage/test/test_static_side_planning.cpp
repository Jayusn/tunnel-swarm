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
#include <set>
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

  poly_traj::Trajectory recoveryLine(const Vector3d &from, const Vector3d &to)
  {
    Eigen::Matrix3d head, tail;
    head << from, Vector3d::Zero(), Vector3d::Zero();
    tail << to, Vector3d::Zero(), Vector3d::Zero();
    poly_traj::MinJerkOpt polynomial;
    polynomial.reset(head, tail, 1);
    polynomial.generate(Eigen::MatrixXd(3, 0), Eigen::VectorXd::Constant(1, 2.0));
    return polynomial.getTraj();
  }

  GridMap::Ptr map;
  diff_planner::SwarmTrajData swarm;
  const Vector3d start{-0.9, 0.0, 0.6}, end{0.9, 0.0, 0.6};
};

TEST_F(StaticSidePlanning, RecoveryValidationAllowsOnlyAShortInitialOccupiedSegment)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  const auto trajectory = recoveryLine(Vector3d(0.225, 0.025, 0.625),
                                        Vector3d(0.8, 0.025, 0.625));
  const double now = ros::Time::now().toSec();
  EXPECT_FALSE(optimizer.trajectoryCollisionFree(trajectory, now, now));
  EXPECT_TRUE(optimizer.trajectoryCollisionFree(trajectory, now, now, 0.20));
  const auto long_prefix = recoveryLine(Vector3d(-0.20, 0.025, 0.625),
                                       Vector3d(0.8, 0.025, 0.625));
  EXPECT_FALSE(optimizer.trajectoryCollisionFree(long_prefix, now, now, 0.20));
}

TEST_F(StaticSidePlanning, NearObstacleAddsOppositeWallWithExistingConstraint)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, false);
  optimizer.obs_clearance_ = 0.18;
  optimizer.obs_clearance_soft_ = 0.30;
  optimizer.cps_.resize_cp(3);
  const Vector3d point(0.025, 0.675, 0.625);
  optimizer.cps_.points.colwise() = point;
  optimizer.cps_.base_point[1].push_back(Vector3d(0.025, 0.25, 0.625));
  optimizer.cps_.direction[1].push_back(Vector3d::UnitY());
  ASSERT_EQ(map->getInflateOccupancy(point), 0);
  ASSERT_TRUE(optimizer.addNearObstacleConstraints(1));
  ASSERT_TRUE(std::any_of(optimizer.cps_.direction[1].begin(),
                          optimizer.cps_.direction[1].end(),
                          [](const Vector3d &normal) { return normal.y() < -0.9; }));
  Vector3d gradient;
  double cost;
  ASSERT_TRUE(optimizer.obstacleGradCostP(1, point, gradient, cost));
  EXPECT_GT(gradient.y(), 0.0); // 梯度下降使轨迹远离上侧墙。
  const size_t count = optimizer.cps_.direction[1].size();
  EXPECT_FALSE(optimizer.addNearObstacleConstraints(1));
  EXPECT_EQ(optimizer.cps_.direction[1].size(), count);
}

TEST_F(StaticSidePlanning, NearObstacleUsesSoftClearanceBeforeHardPenaltyRange)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, false);
  optimizer.obs_clearance_ = 0.18;
  optimizer.obs_clearance_soft_ = 0.30;
  optimizer.cps_.resize_cp(3);
  const Vector3d point(0.625, 0.575, 0.625);
  optimizer.cps_.points.colwise() = point;
  ASSERT_EQ(map->getInflateOccupancy(point), 0);
  ASSERT_TRUE(optimizer.addNearObstacleConstraints(1));
  Vector3d gradient;
  double cost;
  ASSERT_TRUE(optimizer.obstacleGradCostP(1, point, gradient, cost));
  EXPECT_GT(cost, 0.0);
  EXPECT_GT(gradient.y(), 0.0);
}

TEST_F(StaticSidePlanning, NearObstacleKeepsBothSidesOfNarrowCorridor)
{
  for (int z = 0; z < 30; ++z)
    for (int y = -40; y < 40; ++y)
      for (int x = -30; x < 30; ++x)
      {
        const Eigen::Vector3i cell(x, y, z);
        const double wall_y = map->globalIdx2Pos(cell).y();
        map->md_.occupancy_buffer_inflate_[map->globalIdx2InfBufIdx(cell)] =
            wall_y >= 0.25 || wall_y <= -0.20;
      }
  PolyTrajOptimizer optimizer;
  configure(optimizer, false);
  optimizer.obs_clearance_ = 0.18;
  optimizer.obs_clearance_soft_ = 0.30;
  optimizer.cps_.resize_cp(3);
  optimizer.cps_.points.colwise() = Vector3d(0.025, 0.025, 0.625);
  ASSERT_TRUE(optimizer.addNearObstacleConstraints(1));
  bool positive_y = false, negative_y = false;
  for (const auto &normal : optimizer.cps_.direction[1])
  {
    positive_y |= normal.y() > 0.9;
    negative_y |= normal.y() < -0.9;
  }
  EXPECT_TRUE(positive_y);
  EXPECT_TRUE(negative_y);
  EXPECT_FALSE(optimizer.addNearObstacleConstraints(1));
}

TEST_F(StaticSidePlanning, MapVisualizationPreservesWrappedVoxelsAndClipping)
{
  auto &data = map->md_;
  data.ringbuffer_size3i_ = data.ringbuffer_inf_size3i_;
  data.ringbuffer_origin3i_ = data.ringbuffer_inf_origin3i_ = Eigen::Vector3i(7, 9, 5);
  data.ringbuffer_lowbound3i_ = data.ringbuffer_inf_lowbound3i_ = Eigen::Vector3i(-30, -40, 0);
  data.ringbuffer_upbound3i_ = data.ringbuffer_inf_upbound3i_ = Eigen::Vector3i(29, 39, 29);
  data.ringbuffer_lowbound3d_ = data.ringbuffer_inf_lowbound3d_;
  data.ringbuffer_upbound3d_ = data.ringbuffer_inf_upbound3d_;
  data.occupancy_buffer_.assign(60 * 80 * 30, -2.0);
  data.occupancy_buffer_inflate_.assign(60 * 80 * 30, 0);
  data.camera_pos_.setZero();
  data.camera_r_m_.setIdentity();
  data.cam2body_.setIdentity();
  map->mp_.virtual_ground_ = 0.10;
  map->mp_.virtual_ceil_ = 1.40;
  map->mp_.min_occupancy_log_ = 1.0;
  map->mp_.frame_id_ = "visualization_test";
  const std::vector<Eigen::Vector3i> occupied{
      {10, 0, 12}, {-10, 0, 12}, {0, 10, 12}, {10, 0, 1},
      {10, 0, 28}, {-30, -40, 12}};
  for (const auto &cell : occupied)
  {
    data.occupancy_buffer_[map->globalIdx2BufIdx(cell)] = 2.0;
    data.occupancy_buffer_inflate_[map->globalIdx2InfBufIdx(cell)] = 1;
  }
  ros::NodeHandle nh;
  map->map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("visualization_raw_test", 1);
  map->map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("visualization_inflated_test", 1);
  sensor_msgs::PointCloud2ConstPtr raw, inflated;
  const boost::function<void(const sensor_msgs::PointCloud2ConstPtr &)> raw_cb =
      [&](const sensor_msgs::PointCloud2ConstPtr &msg) { raw = msg; };
  const boost::function<void(const sensor_msgs::PointCloud2ConstPtr &)> inflated_cb =
      [&](const sensor_msgs::PointCloud2ConstPtr &msg) { inflated = msg; };
  auto raw_sub = nh.subscribe("visualization_raw_test", 1, raw_cb);
  auto inflated_sub = nh.subscribe("visualization_inflated_test", 1, inflated_cb);
  auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while ((!map->map_pub_.getNumSubscribers() || !map->map_inf_pub_.getNumSubscribers()) &&
         ros::WallTime::now() < deadline)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  const auto voxels = [&](const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::set<std::array<int, 3>> cells;
    for (const auto &point : cloud)
    {
      const auto cell = map->pos2GlobalIdx(Vector3d(point.x, point.y, point.z));
      cells.insert({{cell.x(), cell.y(), cell.z()}});
    }
    return cells;
  };
  for (bool preserve_global : {false, true})
  {
    raw.reset();
    inflated.reset();
    map->mp_.preserve_global_map_ = preserve_global;
    map->publishMap();
    map->publishMapInflate();
    deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while ((!raw || !inflated) && ros::WallTime::now() < deadline)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    ASSERT_TRUE(raw);
    ASSERT_TRUE(inflated);
    std::set<std::array<int, 3>> expected_raw{{{10, 0, 12}}};
    std::set<std::array<int, 3>> expected_inflated{{{10, 0, 12}}, {{0, 10, 12}}};
    if (preserve_global)
      expected_raw = expected_inflated = {
          {{10, 0, 12}}, {{-10, 0, 12}}, {{0, 10, 12}}, {{-30, -40, 12}}};
    EXPECT_EQ(voxels(raw), expected_raw);
    EXPECT_EQ(voxels(inflated), expected_inflated);
    EXPECT_EQ(raw->header.frame_id, "visualization_test");
  }
}

TEST_F(StaticSidePlanning, RecoveryValidationRequiresLeavingOccupiedSpace)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  const auto trajectory = recoveryLine(Vector3d(0.175, 0.025, 0.625),
                                        Vector3d(0.225, 0.025, 0.625));
  const double now = ros::Time::now().toSec();
  EXPECT_FALSE(optimizer.trajectoryCollisionFree(trajectory, now, now, 0.20));
}

TEST_F(StaticSidePlanning, RecoveryValidationRejectsAnotherObstacleAfterFreeSpace)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  const auto trajectory = recoveryLine(Vector3d(0.225, 0.025, 0.625),
                                        Vector3d(0.8, 0.025, 0.625));
  const double now = ros::Time::now().toSec();
  ASSERT_TRUE(optimizer.trajectoryCollisionFree(trajectory, now, now, 0.20));
  map->md_.occupancy_buffer_inflate_[
      map->globalIdx2InfBufIdx(Eigen::Vector3i(9, 0, 12))] = 1;
  EXPECT_FALSE(optimizer.trajectoryCollisionFree(trajectory, now, now, 0.20));
}

TEST_F(StaticSidePlanning, RecoveryValidationStillChecksDynamicObstacles)
{
  PolyTrajOptimizer optimizer;
  configure(optimizer, true);
  const auto trajectory = recoveryLine(Vector3d(0.225, 0.025, 0.625),
                                        Vector3d(0.8, 0.025, 0.625));
  const double now = ros::Time::now().toSec();
  ASSERT_TRUE(optimizer.trajectoryCollisionFree(trajectory, now, now, 0.20));
  diff_planner::DynamicObstacleState obstacle;
  obstacle.position = Vector3d(0.70, 0.025, 0.625);
  obstacle.stamp = now;
  optimizer.setDynamicObstacles({obstacle});
  EXPECT_FALSE(optimizer.trajectoryCollisionFree(trajectory, now, now, 0.20));
}

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

TEST_F(UnderpassEndpointPlanning, FloorCostOnlyActsWhenActiveAndBelowBufferedHeight)
{
  PolyTrajOptimizer optimizer;
  configureUnderpass(optimizer);
  Vector3d gradient;
  double cost;
  EXPECT_FALSE(optimizer.underpassFloorGradCostP(Vector3d(0, 0, 0.1), gradient, cost));
  EXPECT_DOUBLE_EQ(cost, 0.0);
  optimizer.underpass_floor_active_ = true;
  EXPECT_TRUE(optimizer.underpassFloorGradCostP(Vector3d(0, 0, 0.16), gradient, cost));
  EXPECT_GT(cost, 0.0);
  EXPECT_LT(gradient.z(), 0.0);
  EXPECT_FALSE(optimizer.underpassFloorGradCostP(Vector3d(0, 0, 0.2), gradient, cost));
  EXPECT_DOUBLE_EQ(cost, 0.0);
  EXPECT_TRUE(gradient.isZero());
  auto config = optimizer.suspended_underpass_.config();
  config.min_height = 0.0;
  optimizer.suspended_underpass_.setConfig(config);
  EXPECT_FALSE(optimizer.underpassFloorGradCostP(Vector3d(0, 0, -0.1), gradient, cost));
}

TEST_F(UnderpassEndpointPlanning, FloorGradientMatchesPositionAndTimeDifferencesIncludingTail)
{
  PolyTrajOptimizer optimizer;
  configureUnderpass(optimizer);
  optimizer.setIfTouchGoal(false);
  // 隔离高度项，避免高速短轨迹的巨大动力学罚项在相减时损失精度。
  optimizer.wei_obs_ = optimizer.wei_obs_soft_ = optimizer.wei_feas_ = 0.0;
  optimizer.wei_swarm_ = optimizer.wei_dynamic_obstacle_ = 0.0;
  optimizer.wei_sqrvar_ = optimizer.wei_time_ = 0.0;
  optimizer.piece_num_ = 2;
  optimizer.t_now_ = 0.0;
  optimizer.wei_swarm_mod_ = optimizer.wei_swarm_;
  optimizer.cps_.resize_cp(13);
  Eigen::Matrix3d head, tail;
  head << start, Vector3d::Zero(), Vector3d::Zero();
  tail << Vector3d(0.9, 0.0, 0.08), Vector3d::Zero(), Vector3d::Zero();
  optimizer.jerkOpt_.reset(head, tail, 2);
  // 仅尾部低于下限，确保没有沿用普通避障的前 2/3 段裁剪。
  Eigen::VectorXd x(5);
  x << 0.0, 0.0, 0.6, 0.0, 0.0;
  auto floor_cost = [&](const Eigen::VectorXd &variables, Eigen::VectorXd &gradient) {
    Eigen::VectorXd active_gradient(5), inactive_gradient(5);
    optimizer.iter_num_ = 0;
    optimizer.underpass_floor_active_ = true;
    const double active = PolyTrajOptimizer::costFunctionCallback(
        &optimizer, variables.data(), active_gradient.data(), 5);
    optimizer.iter_num_ = 0;
    optimizer.underpass_floor_active_ = false;
    const double inactive = PolyTrajOptimizer::costFunctionCallback(
        &optimizer, variables.data(), inactive_gradient.data(), 5);
    gradient = active_gradient - inactive_gradient;
    return active - inactive;
  };
  Eigen::VectorXd gradient;
  ASSERT_GT(floor_cost(x, gradient), 0.0);
  const double epsilon = 1e-6;
  for (int i = 0; i < x.size(); ++i)
  {
    SCOPED_TRACE(i);
    Eigen::VectorXd plus = x, minus = x, unused;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    const double numerical = (floor_cost(plus, unused) - floor_cost(minus, unused)) /
        (2.0 * epsilon);
    EXPECT_NEAR(gradient(i), numerical, 1e-4 * std::max(1.0, std::abs(numerical)));
  }
}

TEST_F(UnderpassEndpointPlanning, BilateralConstraintsKeepUnderpassAboveFloor)
{
  for (bool enable_cost : {false, true})
  {
    PolyTrajOptimizer optimizer;
    configureUnderpass(optimizer);
    optimizer.obs_clearance_ = 0.18;
    optimizer.obs_clearance_soft_ = 0.35;
    if (!enable_cost) optimizer.wei_underpass_floor_ = 0.0;
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
    ASSERT_TRUE(optimizer.underpass_floor_active_);
    double cost;
    const bool success = optimizer.optimizeTrajectory(head, tail, midpoint, durations, cost);
    const auto trajectory = optimizer.getMinJerkOpt().getTraj();
    double minimum_z = 1e9;
    for (int i = 0; i <= 2000; ++i)
    {
      const auto p = trajectory.getPos(trajectory.getTotalDuration() * i / 2000.0);
      minimum_z = std::min(minimum_z, p.z());
      EXPECT_EQ(map->getInflateOccupancy(p), 0);
    }
    // 双侧约束也能消除旧例中的下探；不再要求关闭独立高度代价必然失败。
    EXPECT_TRUE(success);
    EXPECT_GE(minimum_z, 0.15 - 1e-6);
    RecordProperty(enable_cost ? "minimum_z_with_floor_cost" : "minimum_z_without_floor_cost",
                   std::to_string(minimum_z));
    optimizer.endPlanningCycle();
  }
}

TEST_F(UnderpassEndpointPlanning, NoRouteAboveFloorStillFailsFinalChecks)
{
  // 唯一通道在 z < 0.15 m，侧面和上方均封闭，无法同时满足避障与高度下限。
  for (int z = 0; z < 30; ++z)
    for (int y = -40; y < 40; ++y)
      for (int x = -30; x < 30; ++x)
      {
        const Eigen::Vector3i cell(x, y, z);
        const Vector3d p = map->globalIdx2Pos(cell);
        map->md_.occupancy_buffer_inflate_[map->globalIdx2InfBufIdx(cell)] =
            std::abs(p.y()) >= 0.8 || (std::abs(p.x()) <= 0.25 && p.z() >= 0.15);
      }
  PolyTrajOptimizer optimizer;
  configureUnderpass(optimizer);
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
  // 模拟已选择下穿后地图更新、通道变窄，最终检查不能被软代价替代。
  optimizer.underpass_floor_active_ = true;
  double cost;
  EXPECT_FALSE(optimizer.optimizeTrajectory(head, tail, midpoint, durations, cost));
  EXPECT_FALSE(optimizer.planning_timeout_reported_);
  optimizer.endPlanningCycle();
}
} // namespace

#include <plan_manage/fapp_history.h>

TEST(FappHistory, KeepsRegionAcrossDropoutAndRejectsIsolatedTallBox) {
  diff_planner::FappHistory history;
  diff_planner::DynamicObstacleState ball;
  ball.position = Vector3d(0,0,0.8); ball.size = Vector3d(0.15,0.15,0.15);
  for (int i=0;i<4;++i) { ball.stamp=1+0.1*i; history.update({ball},ball.stamp); }
  history.watch(history.regions(1.3).front(), Vector3d(-1,0,0.6), Vector3d(1,0,0.6),0.4);
  ball.stamp=1.4; ball.size.z()=0.7; ball.position.z()=0.5;
  history.update({ball},1.4);
  history.update({},20.0);
  const auto regions=history.regions(20.0);
  ASSERT_EQ(1u, regions.size());
  EXPECT_NEAR(0.15,regions.front().size.z(),1e-6);
  EXPECT_DOUBLE_EQ(20.0,regions.front().stamp);
  history.observePosition(Vector3d(0.2,0,0.6)); EXPECT_TRUE(history.active());
  history.observePosition(Vector3d(0.6,0,0.6)); EXPECT_FALSE(history.active());
  EXPECT_TRUE(history.regions(20.0).empty());
}

TEST_F(StaticSidePlanning, DynamicCrossingUsesForwardSideDipAndReturnsToGoalHeight) {
  fillCorridor(2.0,3.0);
  PolyTrajOptimizer optimizer; configure(optimizer,true);
  optimizer.dynamic_continuous_crossing_=true;
  optimizer.dynamic_obstacle_config_.vehicle_top_height=0.05;
  optimizer.dynamic_obstacle_config_.vehicle_bottom_height=0.20;
  diff_planner::DynamicObstacleState ball;
  ball.position=Vector3d(-0.2,0,0.8); ball.size=Vector3d(0.15,0.15,0.15);
  ball.stamp=ros::Time::now().toSec();
  optimizer.setDynamicObstacles({ball});optimizer.beginPlanningCycle();
  std::vector<Vector3d> path;
  ASSERT_TRUE(optimizer.searchDynamicCandidate(start,Vector3d(1.3,0,0.6),ball,0,0,path));
  EXPECT_NEAR(0.6,path.back().z(),1e-6);
  bool dipped=false,side=false;
  for(size_t i=1;i<path.size();++i) {
    EXPECT_GT(path[i].x(),path[i-1].x());
    EXPECT_GE(path[i].z(),0.15);
    dipped |= path[i].z()<0.55; side |= std::abs(path[i].y())>0.1;
  }
  EXPECT_TRUE(dipped);EXPECT_TRUE(side);
}

TEST_F(StaticSidePlanning, DynamicLowBallRejectsUnderpassButAllowsSide) {
  fillCorridor(2.0,3.0);
  PolyTrajOptimizer optimizer;configure(optimizer,true);
  optimizer.dynamic_continuous_crossing_=true;
  optimizer.dynamic_obstacle_config_.vehicle_top_height=0.05;
  optimizer.dynamic_obstacle_config_.vehicle_bottom_height=0.20;
  diff_planner::DynamicObstacleState ball;
  ball.position=Vector3d(0,0,0.3);ball.size=Vector3d(0.15,0.15,0.3);
  ball.stamp=ros::Time::now().toSec();optimizer.setDynamicObstacles({ball});
  optimizer.beginPlanningCycle();std::vector<Vector3d> path;
  EXPECT_FALSE(optimizer.searchDynamicCandidate(start,end,ball,0,0,path));
  optimizer.endDynamicCandidate();
  EXPECT_TRUE(optimizer.searchDynamicCandidate(start,end,ball,0,1,path));
  ASSERT_FALSE(path.empty());EXPECT_NEAR(0.6,path.back().z(),1e-6);
}
