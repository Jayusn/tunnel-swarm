#include <gtest/gtest.h>
#include <plan_env/vehicle_geometry.h>
#include <vector>

namespace
{
using diff_planner::cylinderBoxClearance;
using Eigen::Vector3d;

TEST(VehicleGeometry, DistinguishesBoxWidthFromFixedSphere)
{
  const Vector3d size(0.20, 1.00, 0.20);
  EXPECT_NEAR(cylinderBoxClearance(Vector3d(0.5, 0.0, 0.0),
      Vector3d::Zero(), size, 0.262, 0.125, 0.1), 0.038, 1e-12);
  EXPECT_NEAR(cylinderBoxClearance(Vector3d(0.0, 0.5, 0.0),
      Vector3d::Zero(), size, 0.262, 0.125, 0.1), -0.325, 1e-12);
}

TEST(VehicleGeometry, LowerObstacleCanBlockUnderpass)
{
  const Vector3d obstacle_center(0.0, 0.0, 0.2);
  const Vector3d size(0.4, 0.4, 0.4);
  EXPECT_NEAR(cylinderBoxClearance(Vector3d(0.0, 0.0, 0.65),
      obstacle_center, size, 0.262, 0.125, 0.10), 0.025, 1e-12);
  EXPECT_LT(cylinderBoxClearance(Vector3d(0.0, 0.0, 0.50),
      obstacle_center, size, 0.262, 0.125, 0.10), 0.0);
}

TEST(VehicleGeometry, CornerDistanceIsEuclidean)
{
  const Vector3d size(0.2, 0.2, 0.2);
  const double horizontal = std::hypot(0.4, 0.3) - 0.262;
  const double vertical = 0.6 - 0.1 - 0.125;
  EXPECT_NEAR(cylinderBoxClearance(Vector3d(0.5, 0.4, 0.6),
      Vector3d::Zero(), size, 0.262, 0.125, 0.10),
      std::hypot(horizontal, vertical) - 0.10, 1e-12);
}

TEST(VehicleGeometry, ZeroAxisSupportsThinPanels)
{
  const Vector3d panel(0.0, 0.8, 0.6);
  EXPECT_NEAR(cylinderBoxClearance(Vector3d(0.4, 0.0, 0.0),
      Vector3d::Zero(), panel, 0.262, 0.125, 0.10), 0.038, 1e-12);
  EXPECT_LT(cylinderBoxClearance(Vector3d(0.2, 0.0, 0.0),
      Vector3d::Zero(), panel, 0.262, 0.125, 0.10), 0.0);
}

TEST(VehicleGeometry, MarginIsAppliedOnce)
{
  const Vector3d position(0.5, 0.4, 0.6), size(0.2, 0.3, 0.4);
  const double unpadded = cylinderBoxClearance(position, Vector3d::Zero(),
      size, 0.262, 0.125, 0.0);
  EXPECT_NEAR(cylinderBoxClearance(position, Vector3d::Zero(), size,
      0.262, 0.125, 0.10), unpadded - 0.10, 1e-12);
}

TEST(VehicleGeometry, TranslationPreservesClearanceAndGradient)
{
  const Vector3d position(0.5, 0.4, 0.6), size(0.2, 0.3, 0.4);
  const Vector3d offset(-2.1, 3.4, 0.6);
  Vector3d gradient, translated_gradient;
  const double original = cylinderBoxClearance(position, Vector3d::Zero(),
      size, 0.262, 0.125, 0.10, &gradient);
  EXPECT_NEAR(cylinderBoxClearance(position + offset, offset, size,
      0.262, 0.125, 0.10, &translated_gradient), original, 1e-12);
  EXPECT_TRUE(gradient.isApprox(translated_gradient, 1e-12));
}

TEST(VehicleGeometry, GradientMatchesFiniteDifference)
{
  const Vector3d size(0.2, 0.5, 0.4);
  const std::vector<Vector3d> positions{
      Vector3d(0.6, 0.4, 0.65), Vector3d(-0.6, -0.4, -0.65),
      Vector3d(0.45, 0.1, 0.01), Vector3d(0.01, 0.1, -0.4),
      Vector3d(0.04, 0.02, 0.05)};
  for (const auto &position : positions)
  {
    Vector3d gradient;
    cylinderBoxClearance(position, Vector3d::Zero(), size,
        0.262, 0.125, 0.10, &gradient);
    for (int axis = 0; axis < 3; ++axis)
    {
      Vector3d delta = Vector3d::Zero();
      delta(axis) = 1e-6;
      const double derivative = (cylinderBoxClearance(position + delta,
          Vector3d::Zero(), size, 0.262, 0.125, 0.10) -
          cylinderBoxClearance(position - delta, Vector3d::Zero(), size,
          0.262, 0.125, 0.10)) / (2e-6);
      EXPECT_NEAR(gradient(axis), derivative, 1e-6);
    }
  }
}

TEST(VehicleGeometry, CenterOverlapHasFiniteEscapeGradient)
{
  Vector3d gradient;
  EXPECT_LT(cylinderBoxClearance(Vector3d::Zero(), Vector3d::Zero(),
      Vector3d(0.2, 0.2, 0.2), 0.262, 0.125, 0.10, &gradient), 0.0);
  EXPECT_TRUE(gradient.allFinite());
  EXPECT_TRUE(gradient.isApprox(-Vector3d::UnitZ()));
}

} // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
