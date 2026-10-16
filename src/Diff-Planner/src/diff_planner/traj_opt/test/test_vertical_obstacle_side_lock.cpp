#include <gtest/gtest.h>

#include <optimizer/vertical_obstacle_side_lock.h>

namespace diff_planner
{
namespace
{

TEST(VerticalObstacleSideLock, AcceptsSimilarUsableWidths)
{
  EXPECT_TRUE(VerticalObstacleSideLock::widthsAreComparable(
      0.42, 0.50, 0.20, 0.15));
}

TEST(VerticalObstacleSideLock, RejectsClearlyUnequalWidths)
{
  EXPECT_FALSE(VerticalObstacleSideLock::widthsAreComparable(
      0.25, 0.55, 0.20, 0.15));
}

TEST(VerticalObstacleSideLock, RejectsBlockedSide)
{
  EXPECT_FALSE(VerticalObstacleSideLock::widthsAreComparable(
      0.10, 0.12, 0.20, 0.15));
}

TEST(VerticalObstacleSideLock, ChoosesSideUsedByInitialPath)
{
  const Eigen::Vector3d center(1.0, 0.0, 0.6);
  const Eigen::Vector3d lateral(0.0, 1.0, 0.0);
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(1.0, -0.35, 0.6),
      Eigen::Vector3d(2.0, 0.0, 0.6)};
  EXPECT_LT(VerticalObstacleSideLock::chooseSideNormal(
                path, center, lateral, 0.45, 0.45, 0.05).dot(lateral),
            0.0);
}

TEST(VerticalObstacleSideLock, ChoosesWiderSideDespiteInitialPath)
{
  const Eigen::Vector3d center(1.0, 0.0, 0.6);
  const Eigen::Vector3d lateral(0.0, 1.0, 0.0);
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(1.0, -0.35, 0.6),
      Eigen::Vector3d(2.0, 0.0, 0.6)};
  EXPECT_GT(VerticalObstacleSideLock::chooseSideNormal(
                path, center, lateral, 0.40, 0.25, 0.05).dot(lateral),
            0.0);
}

TEST(VerticalObstacleSideLock, UsesWiderSideForFlatPathFallback)
{
  const Eigen::Vector3d center(1.0, 0.0, 0.6);
  const Eigen::Vector3d lateral(0.0, 1.0, 0.0);
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(2.0, 0.0, 0.6)};
  EXPECT_GT(VerticalObstacleSideLock::chooseSideNormal(
                path, center, lateral, 0.50, 0.45, 0.05).dot(lateral),
            0.0);
}

TEST(VerticalObstacleSideLock, UsesMidpointOfObstacleAndWallBoundaries)
{
  EXPECT_DOUBLE_EQ(
      VerticalObstacleSideLock::computePassageCenter(0.10, 0.75), 0.425);
}

TEST(VerticalObstacleSideLock, KeepsLockAcrossLongitudinalCenterDrift)
{
  EXPECT_TRUE(VerticalObstacleSideLock::isSameObstacle(
      Eigen::Vector3d(1.0, 0.0, 0.6), Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(), 0.40,
      Eigen::Vector3d(1.9, 0.05, 0.6), 0.30, 0.75, 0.30));
}

TEST(VerticalObstacleSideLock, RejectsDifferentLateralObstacle)
{
  EXPECT_FALSE(VerticalObstacleSideLock::isSameObstacle(
      Eigen::Vector3d(1.0, 0.0, 0.6), Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(), 0.40,
      Eigen::Vector3d(1.1, 0.80, 0.6), 0.30, 0.75, 0.30));
}

// 0.05m 栅格：两侧墙位于 y=±0.75m，横向采样包含完整外墙。
std::vector<std::vector<int>> corridorPlane()
{
  std::vector<std::vector<int>> plane(21, std::vector<int>(61, 0));
  for (auto &row : plane)
    for (int c = 0; c < 61; ++c)
      if (c <= 15 || c >= 45)
        row[c] = 1;
  return plane;
}

void addDivider(std::vector<std::vector<int>> &plane,
                int first_row, int last_row, int first_col, int last_col)
{
  for (int r = first_row; r <= last_row; ++r)
    for (int c = first_col; c <= last_col; ++c)
      plane[r][c] = 1;
}

TEST(VerticalObstacleSideLock, PlainCorridorDoesNotActivate)
{
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(
      corridorPlane(), 0.05, 0.20).empty());
}

TEST(VerticalObstacleSideLock, OpenFieldDoesNotActivate)
{
  std::vector<std::vector<int>> plane(21, std::vector<int>(61, 0));
  addDivider(plane, 7, 13, 28, 32);
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20).empty());
}

TEST(VerticalObstacleSideLock, InternalDividerCreatesTwoThroughPassages)
{
  auto plane = corridorPlane();
  addDivider(plane, 7, 13, 30, 35);
  const auto passages = VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20);
  ASSERT_EQ(passages.size(), 2U);
  EXPECT_NEAR(passages[0].max_offset - passages[0].min_offset, 0.70, 1.0e-6);
  EXPECT_NEAR(passages[1].max_offset - passages[1].min_offset, 0.45, 1.0e-6);
}

TEST(VerticalObstacleSideLock, MultipleDividersCreateThreePassages)
{
  std::vector<std::vector<int>> plane(21, std::vector<int>(61, 0));
  for (auto &row : plane)
    for (int c = 0; c < 61; ++c)
      if (c <= 7 || c >= 45)
        row[c] = 1;
  addDivider(plane, 7, 13, 13, 15);
  addDivider(plane, 7, 13, 30, 32);
  const auto passages = VerticalObstacleSideLock::findSplitPassages(plane, 0.04, 0.20);
  ASSERT_EQ(passages.size(), 3U);
  EXPECT_NEAR(passages[0].max_offset - passages[0].min_offset, 0.20, 1.0e-6);
  EXPECT_NEAR(passages[1].max_offset - passages[1].min_offset, 0.56, 1.0e-6);
  EXPECT_NEAR(passages[2].max_offset - passages[2].min_offset, 0.48, 1.0e-6);
}

TEST(VerticalObstacleSideLock, BendWallConnectedToOuterWallDoesNotActivate)
{
  auto plane = corridorPlane();
  addDivider(plane, 9, 11, 15, 32);
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20).empty());
}

TEST(VerticalObstacleSideLock, UShapedWallDoesNotActivate)
{
  auto plane = corridorPlane();
  addDivider(plane, 0, 13, 28, 30);
  addDivider(plane, 11, 13, 15, 30);
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20).empty());
}

TEST(VerticalObstacleSideLock, IncompleteLongDividerDoesNotActivate)
{
  auto plane = corridorPlane();
  addDivider(plane, 0, 13, 28, 32);
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20).empty());
}

TEST(VerticalObstacleSideLock, KeepsOnlyUsablePassage)
{
  auto plane = corridorPlane();
  addDivider(plane, 7, 13, 30, 42);
  const auto passages = VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20);
  ASSERT_EQ(passages.size(), 1U);
  EXPECT_LT(passages.front().max_offset, 0.0);
}

TEST(VerticalObstacleSideLock, LocalPocketDoesNotCountAsThroughPassage)
{
  auto plane = corridorPlane();
  addDivider(plane, 7, 13, 28, 32);
  addDivider(plane, 7, 8, 15, 26);
  const auto passages = VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20);
  ASSERT_EQ(passages.size(), 1U);
  EXPECT_GT(passages.front().min_offset, 0.0);
}

TEST(VerticalObstacleSideLock, DistinguishesBlockedDividerFromMissingObservation)
{
  auto plane = corridorPlane();
  addDivider(plane, 7, 13, 18, 42);
  std::pair<int, int> rows;
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20, &rows).empty());
  EXPECT_GE(rows.first, 0);
  EXPECT_GE(rows.second, rows.first);
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(corridorPlane(), 0.05, 0.20, &rows).empty());
  EXPECT_LT(rows.first, 0);
}

TEST(VerticalObstacleSideLock, UnknownDividerBoundaryDoesNotActivate)
{
  auto plane = corridorPlane();
  addDivider(plane, 7, 13, 28, 32);
  plane[6][30] = -1;
  EXPECT_TRUE(VerticalObstacleSideLock::findSplitPassages(plane, 0.05, 0.20).empty());
}

} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
