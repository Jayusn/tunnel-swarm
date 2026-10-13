#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>

namespace
{
using Eigen::Vector3d;

class AStarVehicleOccupancy : public testing::Test
{
protected:
  void SetUp() override
  {
    ros::Time::init();
    search.initGridMap(nullptr, Eigen::Vector3i(15, 15, 5));
    region.enabled = true;
    region.limit_min_z = region.limit_max_z = true;
    region.min_z = region.max_z = 1.0;
    region.time_limit = 1.0;
    region.report_timeout = false;
  }
  AStar search;
  AStarSearchRegion region;
};

TEST_F(AStarVehicleOccupancy, RejectsOccupiedActualEndpoint)
{
  const Vector3d start(0.35, 0.0, 1.0), end(1.65, 0.0, 1.0);
  search.setOccupancyChecker([start](const Vector3d &position) {
    return (position - start).norm() < 0.04;
  });
  EXPECT_EQ(search.AstarSearch(0.5, start, end, &region), ASTAR_RET::INIT_ERR);
}

TEST_F(AStarVehicleOccupancy, ChecksActualEndpointToGridConnection)
{
  search.setOccupancyChecker([](const Vector3d &position) {
    return std::abs(position.x() + 0.175) < 0.025;
  });
  EXPECT_EQ(search.AstarSearch(0.5, Vector3d(-0.35, 0.0, 1.0),
      Vector3d(1.35, 0.0, 1.0), &region), ASTAR_RET::INIT_ERR);
}

TEST_F(AStarVehicleOccupancy, SamplesBetweenNeighborNodes)
{
  const Vector3d start(-0.75, 0.0, 1.0), end(0.75, 0.0, 1.0);
  region.limit_corridor = true;
  region.line_start = start;
  region.line_end = end;
  region.corridor_half_width = 0.01;
  search.setOccupancyChecker([](const Vector3d &position) {
    return std::abs(position.x() - 0.25) < 0.08;
  });
  EXPECT_EQ(search.AstarSearch(0.5, start, end, &region), ASTAR_RET::SEARCH_ERR);
}

TEST_F(AStarVehicleOccupancy, FindsAvailableSidePassage)
{
  const auto occupied = [](const Vector3d &position) {
    return std::abs(position.x()) < 0.30 && std::abs(position.y()) < 0.40;
  };
  search.setOccupancyChecker(occupied);
  EXPECT_EQ(search.AstarSearch(0.25, Vector3d(-1.0, 0.0, 1.0),
      Vector3d(1.0, 0.0, 1.0), &region), ASTAR_RET::SUCCESS);
  const auto path = search.getPath();
  ASSERT_GT(path.size(), 2U);
  double lateral = 0.0;
  for (size_t index = 0; index < path.size(); ++index)
  {
    EXPECT_FALSE(occupied(path[index]));
    lateral = std::max(lateral, std::abs(path[index].y()));
    if (index == 0)
      continue;
    const Vector3d segment = path[index] - path[index - 1];
    const int intervals = std::max(1, static_cast<int>(
        std::ceil(segment.norm() / 0.125)));
    for (int sample = 0; sample <= intervals; ++sample)
      EXPECT_FALSE(occupied(path[index - 1] + segment *
          (static_cast<double>(sample) / intervals)));
  }
  EXPECT_GE(lateral, 0.5);
}

TEST_F(AStarVehicleOccupancy, RepeatedSearchUsesUpdatedChecker)
{
  search.setOccupancyChecker([](const Vector3d &position) {
    return std::abs(position.x()) < 0.3;
  });
  const Vector3d start(-1.0, 0.0, 1.0), end(1.0, 0.0, 1.0);
  EXPECT_EQ(search.AstarSearch(0.25, start, end, &region), ASTAR_RET::SEARCH_ERR);
  search.setOccupancyChecker([](const Vector3d &) { return false; });
  EXPECT_EQ(search.AstarSearch(0.25, start, end, &region), ASTAR_RET::SUCCESS);
}

TEST_F(AStarVehicleOccupancy, WallTimeoutWorksWithFrozenRosTime)
{
  ros::Time::setNow(ros::Time(42));
  region.time_limit = 0.005;
  search.setOccupancyChecker([](const Vector3d &) {
    ros::WallDuration(0.001).sleep();
    return false;
  });
  const ros::WallTime begin = ros::WallTime::now();
  const ASTAR_RET result = search.AstarSearch(0.25,
      Vector3d(-1.0, 0.0, 1.0), Vector3d(1.0, 0.0, 1.0), &region);
  const double elapsed = (ros::WallTime::now() - begin).toSec();
  ros::Time::init();
  EXPECT_EQ(result, ASTAR_RET::SEARCH_ERR);
  EXPECT_LT(elapsed, 0.25);
}

TEST_F(AStarVehicleOccupancy, SplitPassageRestrictionIsLocalToDivider)
{
  search.setOccupancyChecker([](const Vector3d &position) {
    return std::abs(position.x()) < 0.30 && std::abs(position.y()) < 0.40;
  });
  region.limit_passage = true;
  region.side_origin = Vector3d(0.0, 0.0, 1.0);
  region.side_axis = Vector3d::UnitX();
  region.side_normal = Vector3d::UnitY();
  region.side_half_length = 0.5;
  region.passage_min_offset = 0.5;
  region.passage_max_offset = 1.0;
  // 起终点在所选通路之外，但在分隔物前后允许正常转入/转出。
  EXPECT_EQ(search.AstarSearch(0.25, Vector3d(-1.0, 0.0, 1.0),
      Vector3d(1.0, 0.0, 1.0), &region), ASTAR_RET::SUCCESS);
  for (const auto &point : search.getPath())
    if (std::abs(point.x()) <= 0.5)
    {
      EXPECT_GE(point.y(), 0.5 - 1.0e-6);
      EXPECT_LE(point.y(), 1.0 + 1.0e-6);
    }
}

TEST_F(AStarVehicleOccupancy, SplitPassageFailureDoesNotConstrainOrdinarySearch)
{
  search.setOccupancyChecker([](const Vector3d &position) {
    return std::abs(position.x()) < 0.30 && std::abs(position.y()) < 0.40;
  });
  region.limit_passage = true;
  region.side_origin = Vector3d(0.0, 0.0, 1.0);
  region.side_axis = Vector3d::UnitX();
  region.side_normal = Vector3d::UnitY();
  region.side_half_length = 0.5;
  region.passage_min_offset = 0.0;
  region.passage_max_offset = 0.25;
  const Vector3d start(-1.0, 0.0, 1.0), end(1.0, 0.0, 1.0);
  EXPECT_EQ(search.AstarSearch(0.25, start, end, &region), ASTAR_RET::SEARCH_ERR);
  region.limit_passage = false;
  EXPECT_EQ(search.AstarSearch(0.25, start, end, &region), ASTAR_RET::SUCCESS);
}

} // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
