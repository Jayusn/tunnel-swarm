#include <gtest/gtest.h>

#include <bspline_opt/fapp_dynamic_obstacle.h>

#include <cmath>
#include <limits>

namespace fuel_fapp
{
namespace
{

class DynamicObstacleTest : public testing::Test
{
protected:
  void SetUp() override
  {
    config.enabled = true;
    state.position = Eigen::Vector3d(0.0, 0.0, 1.0);
    state.stamp = 10.0;
  }

  double clearanceAt(const Eigen::Vector3d &position,
                     Eigen::Vector3d *gradient = nullptr)
  {
    double result = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(evaluateDynamicObstacle(position, state, 10.0, 10.0, config,
                                       &result, gradient));
    return result;
  }

  DynamicObstacleConfig config;
  DynamicObstacleState state;
};

TEST_F(DynamicObstacleTest, HorizontalClearanceIncludesBallAndVehicle)
{
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.50, 0.0, 1.0)), 0.038, 1.0e-12);
  EXPECT_LT(clearanceAt(Eigen::Vector3d(0.40, 0.0, 1.0)), 0.0);
}

TEST_F(DynamicObstacleTest, UnderpassUsesFiniteVehicleHeight)
{
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.0, 0.0, 0.65)), 0.025, 1.0e-12);
  EXPECT_LT(clearanceAt(Eigen::Vector3d(0.0, 0.0, 0.70)), 0.0);
}

TEST_F(DynamicObstacleTest, UnderpassReferenceUsesTopFivePlusGapFiveOnly)
{
  config.vehicle_top_height = 0.05;
  config.vehicle_bottom_height = 0.20;
  state.position.z() = 0.39;  // 原始底部Z=0.29，目标Z=0.19。
  for (bool measured : {false, true}) {
    state.size = measured ? Eigen::Vector3d::Constant(0.2) : Eigen::Vector3d::Zero();
    config.underpass_reference_clearance = -1.0;
    const double horizontal = clearanceAt(Eigen::Vector3d(.50,0,.39));
    const double above = clearanceAt(Eigen::Vector3d(0,0,.85));
    config.underpass_reference_clearance = 0.10;
    Eigen::Vector3d gradient;
    EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0,0,.19), &gradient), 0.0, 1e-12);
    EXPECT_NEAR(gradient.z(), -1.0, 1e-12);
    EXPECT_GT(clearanceAt(Eigen::Vector3d(0,0,.18)), 0.0);
    EXPECT_LT(clearanceAt(Eigen::Vector3d(0,0,.20)), 0.0);
    EXPECT_NEAR(clearanceAt(Eigen::Vector3d(.50,0,.39)), horizontal, 1e-12);
    EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0,0,.85)), above, 1e-12);
    const Eigen::Vector3d point(0.0, 0.0, 0.195);
    clearanceAt(point, &gradient);
    for (int axis=0; axis<3; ++axis) {
      const Eigen::Vector3d delta=1e-6*Eigen::Vector3d::Unit(axis);
      EXPECT_NEAR(gradient(axis), (clearanceAt(point+delta)-clearanceAt(point-delta))/2e-6, 1e-6);
    }
  }
}

TEST_F(DynamicObstacleTest, AsymmetricBodyUsesTopBelowAndBottomAbove)
{
  config.vehicle_top_height = 0.05;
  config.vehicle_bottom_height = 0.20;
  for (bool measured : {false, true}) {
    state.size = measured ? Eigen::Vector3d::Constant(0.2) : Eigen::Vector3d::Zero();
    Eigen::Vector3d gradient;
    EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0, 0, 0.70), &gradient), 0.05, 1e-12);
    EXPECT_NEAR(gradient.z(), -1.0, 1e-12);
    EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0, 0, 1.45), &gradient), 0.05, 1e-12);
    EXPECT_NEAR(gradient.z(), 1.0, 1e-12);
    EXPECT_LT(clearanceAt(Eigen::Vector3d(0, 0, 1.30)), 0.0);
    const Eigen::Vector3d point(0.45, 0.15, 0.70);
    clearanceAt(point, &gradient);
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector3d offset = 1e-6 * Eigen::Vector3d::Unit(axis);
      EXPECT_NEAR(gradient(axis), (clearanceAt(point + offset) -
          clearanceAt(point - offset)) / 2e-6, 1e-6);
    }
  }
}

TEST_F(DynamicObstacleTest, MeasuredBoxUsesEachObservedExtent)
{
  state.size = Eigen::Vector3d(0.2, 0.8, 0.6);
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.50, 0.0, 1.0)), 0.038, 1.0e-12);
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.0, 0.50, 1.0)), -0.262, 1.0e-12);
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.0, 0.0, 0.40)), 0.075, 1.0e-12);

  state.size.x() = 0.8;
  EXPECT_LT(clearanceAt(Eigen::Vector3d(0.50, 0.0, 1.0)), 0.0);
}

TEST_F(DynamicObstacleTest, FlatMeasuredShapesDoNotUseSphereFallback)
{
  state.size = Eigen::Vector3d(0.6, 0.4, 0.0);
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.0, 0.0, 0.70)), 0.075, 1.0e-12);
  state.size = Eigen::Vector3d(0.0, 0.4, 0.0);
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.0, 0.50, 1.0)), -0.062, 1.0e-12);
}

TEST_F(DynamicObstacleTest, MeasuredBoxCornerUsesEuclideanMarginOnce)
{
  state.size = Eigen::Vector3d(0.2, 0.4, 0.6);
  config.margin = 0.02;
  // The nearest box corner is 0.2 m away in both horizontal axes. After
  // subtracting the cylinder radius, the vertical surface gap is 0.025 m.
  const double expected = std::hypot(std::sqrt(0.08) - 0.262, 0.025) - 0.02;
  EXPECT_NEAR(clearanceAt(Eigen::Vector3d(0.3, 0.4, 1.45)), expected, 1.0e-12);
}

TEST_F(DynamicObstacleTest, MissingOrInvalidMeasuredSizeRetainsSphereGeometry)
{
  const Eigen::Vector3d position(0.45, 0.20, 0.62);
  Eigen::Vector3d original_gradient;
  const double original_clearance = clearanceAt(position, &original_gradient);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d sizes[] = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(-0.01, 0.2, 0.2),
      Eigen::Vector3d(nan, 0.2, 0.2), Eigen::Vector3d(0.2, infinity, 0.2)};
  for (const Eigen::Vector3d &size : sizes)
  {
    state.size = size;
    Eigen::Vector3d gradient;
    EXPECT_DOUBLE_EQ(clearanceAt(position, &gradient), original_clearance);
    EXPECT_TRUE(gradient.isApprox(original_gradient, 1.0e-12));
  }
}

TEST_F(DynamicObstacleTest, MeasuredBoxDoesNotDependOnUnusedSphereRadius)
{
  state.size = Eigen::Vector3d(0.2, 0.4, 0.6);
  const Eigen::Vector3d position(0.5, 0.0, 1.0);
  const double expected = clearanceAt(position);
  config.ball_radius = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(clearanceAt(position), expected);
  state.size.setZero();
  EXPECT_FALSE(evaluateDynamicObstacle(position, state, 10.0, 10.0,
                                      config, nullptr));
}

TEST_F(DynamicObstacleTest, RoundedCornerRejectsGapMissedByAnEllipsoid)
{
  config.margin = 0.0;
  const Eigen::Vector3d position(0.338, 0.0, 0.815);
  const double ellipsoid_value =
      std::pow(position.x() / (config.vehicle_radius + config.ball_radius), 2) +
      std::pow((position.z() - state.position.z()) /
                   (config.vehicle_top_height + config.ball_radius),
               2);
  EXPECT_GT(ellipsoid_value, 1.0);
  EXPECT_LT(clearanceAt(position), 0.0);
}

TEST_F(DynamicObstacleTest, CenteredOverlapChoosesDownwardSubgradient)
{
  Eigen::Vector3d gradient;
  EXPECT_LT(clearanceAt(state.position, &gradient), 0.0);
  EXPECT_TRUE(gradient.allFinite());
  EXPECT_DOUBLE_EQ(gradient.x(), 0.0);
  EXPECT_DOUBLE_EQ(gradient.y(), 0.0);
  EXPECT_DOUBLE_EQ(gradient.z(), -1.0);
  // The distance is not differentiable between equally close top and bottom
  // caps; the selected subgradient increases clearance by descending.
  EXPECT_GT(clearanceAt(state.position - 0.01 * Eigen::Vector3d::UnitZ()),
            clearanceAt(state.position));
}

TEST_F(DynamicObstacleTest, GradientsMatchFiniteDifferencesAwayFromSeams)
{
  const Eigen::Vector3d positions[] = {
      Eigen::Vector3d(0.5, 0.12, 1.03),
      Eigen::Vector3d(0.05, 0.03, 0.6),
      Eigen::Vector3d(0.35, 0.14, 0.72),
      Eigen::Vector3d(0.23, 0.03, 1.01),
      Eigen::Vector3d(0.03, 0.02, 0.96)};
  const double epsilon = 1.0e-6;
  for (const Eigen::Vector3d &position : positions)
  {
    Eigen::Vector3d gradient;
    clearanceAt(position, &gradient);
    for (int axis = 0; axis < 3; ++axis)
    {
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      offset(axis) = epsilon;
      const double numerical =
          (clearanceAt(position + offset) - clearanceAt(position - offset)) /
          (2.0 * epsilon);
      EXPECT_NEAR(gradient(axis), numerical, 1.0e-6);
    }
  }
}

TEST_F(DynamicObstacleTest, MeasuredBoxGradientsMatchFiniteDifferences)
{
  state.size = Eigen::Vector3d(0.2, 0.6, 0.4);
  const Eigen::Vector3d positions[] = {
      Eigen::Vector3d(0.6, 0.5, 1.02), Eigen::Vector3d(0.04, 0.03, 0.45),
      Eigen::Vector3d(0.4, 0.5, 0.55), Eigen::Vector3d(0.3, 0.04, 1.0),
      Eigen::Vector3d(0.04, 0.05, 0.9), Eigen::Vector3d(0.08, 0.05, 1.0)};
  const double epsilon = 1.0e-6;
  for (const Eigen::Vector3d &position : positions)
  {
    Eigen::Vector3d gradient;
    clearanceAt(position, &gradient);
    for (int axis = 0; axis < 3; ++axis)
    {
      const Eigen::Vector3d offset = epsilon * Eigen::Vector3d::Unit(axis);
      const double numerical =
          (clearanceAt(position + offset) - clearanceAt(position - offset)) /
          (2.0 * epsilon);
      EXPECT_NEAR(gradient(axis), numerical, 1.0e-6);
    }
  }
}

TEST_F(DynamicObstacleTest, DisabledOrExpiredObservationsAreIgnored)
{
  config.enabled = false;
  EXPECT_FALSE(evaluateDynamicObstacle(state.position, state, 10.0, 10.0,
                                      config, nullptr));
  config.enabled = true;
  EXPECT_TRUE(evaluateDynamicObstacle(state.position, state, 10.80, 10.79,
                                     config, nullptr));
  EXPECT_FALSE(evaluateDynamicObstacle(state.position, state, 10.81, 10.81,
                                      config, nullptr));
}

TEST_F(DynamicObstacleTest, FutureSamplesDoNotExpireCurrentObservation)
{
  EXPECT_TRUE(evaluateDynamicObstacle(state.position, state, 15.0, 10.50,
                                     config, nullptr));
  EXPECT_FALSE(evaluateDynamicObstacle(state.position, state, 15.0, 10.81,
                                      config, nullptr));
}

TEST_F(DynamicObstacleTest, RetentionBoundaryIsInclusive)
{
  state.stamp = 0.0;
  EXPECT_TRUE(evaluateDynamicObstacle(state.position, state, 1.0, 0.80,
                                     config, nullptr));
  EXPECT_FALSE(evaluateDynamicObstacle(state.position, state, 1.0, 0.8001,
                                      config, nullptr));
}

TEST_F(DynamicObstacleTest, ConstantVelocityPredictionStopsAtHorizon)
{
  state.velocity = Eigen::Vector3d(0.4, -0.2, 0.1);
  Eigen::Vector3d predicted_position, predicted_velocity;
  ASSERT_TRUE(evaluateDynamicObstacle(
      state.position, state, 10.50, 10.0, config, nullptr, nullptr,
      &predicted_position, &predicted_velocity));
  EXPECT_TRUE(predicted_position.isApprox(state.position + 0.50 * state.velocity));
  EXPECT_TRUE(predicted_velocity.isApprox(state.velocity));

  ASSERT_TRUE(evaluateDynamicObstacle(
      state.position, state, 12.0, 10.0, config, nullptr, nullptr,
      &predicted_position, &predicted_velocity));
  EXPECT_TRUE(predicted_position.isApprox(state.position + state.velocity));
  EXPECT_TRUE(predicted_velocity.isZero());
}

TEST_F(DynamicObstacleTest, PredictionBeforeObservationClampsToSnapshot)
{
  state.velocity = Eigen::Vector3d(0.4, -0.2, 0.1);
  Eigen::Vector3d predicted_position, predicted_velocity;
  ASSERT_TRUE(evaluateDynamicObstacle(
      state.position, state, 9.5, 10.0, config, nullptr, nullptr,
      &predicted_position, &predicted_velocity));
  EXPECT_TRUE(predicted_position.isApprox(state.position));
  EXPECT_TRUE(predicted_velocity.isZero());
}

TEST_F(DynamicObstacleTest, GroundRelativeMinimumDoesNotAlterMeasuredHeight)
{
  state.position.z() = -0.05;
  state.velocity = Eigen::Vector3d(0.2, 0.0, -0.3);
  Eigen::Vector3d predicted_position, predicted_velocity;
  ASSERT_TRUE(evaluateDynamicObstacle(
      state.position, state, 10.50, 10.0, config, nullptr, nullptr,
      &predicted_position, &predicted_velocity));
  EXPECT_NEAR(predicted_position.z(), -0.20, 1.0e-12);
  EXPECT_DOUBLE_EQ(predicted_velocity.z(), -0.3);
  EXPECT_DOUBLE_EQ(predicted_velocity.x(), 0.2);
}

TEST_F(DynamicObstacleTest, LegacyMinimumBottomDoesNotGateGeometry)
{
  config.minimum_ball_bottom = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(evaluateDynamicObstacle(state.position, state, 10.0, 10.0,
                                     config, nullptr));
  state.size = Eigen::Vector3d(0.2, 0.4, 0.6);
  EXPECT_TRUE(evaluateDynamicObstacle(state.position, state, 10.0, 10.0,
                                     config, nullptr));
}

TEST_F(DynamicObstacleTest, MeasuredBoxTranslationPreservesPredictedGeometry)
{
  state.size = Eigen::Vector3d(0.2, 0.6, 0.4);
  state.velocity = Eigen::Vector3d(0.2, -0.1, -0.15);
  const Eigen::Vector3d position(0.45, 0.50, 0.60);
  double original_clearance;
  Eigen::Vector3d original_gradient;
  ASSERT_TRUE(evaluateDynamicObstacle(position, state, 10.50, 10.0, config,
                                      &original_clearance, &original_gradient));
  for (const Eigen::Vector3d &translation : {
           Eigen::Vector3d(-1.5, 0.8, -0.6), Eigen::Vector3d(2.0, -1.2, 1.5)})
  {
    DynamicObstacleState translated_state = state;
    translated_state.position += translation;
    double translated_clearance;
    Eigen::Vector3d translated_gradient, predicted_position, predicted_velocity;
    ASSERT_TRUE(evaluateDynamicObstacle(
        position + translation, translated_state, 10.50, 10.0, config,
        &translated_clearance, &translated_gradient, &predicted_position,
        &predicted_velocity));
    EXPECT_NEAR(translated_clearance, original_clearance, 1.0e-12);
    EXPECT_TRUE(translated_gradient.isApprox(original_gradient, 1.0e-12));
    EXPECT_TRUE(predicted_position.isApprox(
        state.position + translation + 0.5 * state.velocity, 1.0e-12));
    EXPECT_TRUE(predicted_velocity.isApprox(state.velocity, 1.0e-12));
  }
}

TEST_F(DynamicObstacleTest, VerticalFrameTranslationPreservesGeometry)
{
  state.velocity = Eigen::Vector3d(0.2, -0.1, -0.15);
  const Eigen::Vector3d position(0.45, 0.20, 0.60);
  double original_clearance;
  Eigen::Vector3d original_gradient;
  ASSERT_TRUE(evaluateDynamicObstacle(
      position, state, 10.50, 10.0, config, &original_clearance,
      &original_gradient));
  for (const double translation : {-1.5, -0.8, 2.0})
  {
    DynamicObstacleState translated_state = state;
    translated_state.position.z() += translation;
    Eigen::Vector3d translated_position = position;
    translated_position.z() += translation;
    double translated_clearance;
    Eigen::Vector3d translated_gradient;
    ASSERT_TRUE(evaluateDynamicObstacle(
        translated_position, translated_state, 10.50, 10.0, config,
        &translated_clearance, &translated_gradient));
    EXPECT_NEAR(translated_clearance, original_clearance, 1.0e-12);
    EXPECT_TRUE(translated_gradient.isApprox(original_gradient, 1.0e-12));
  }
}

TEST_F(DynamicObstacleTest, PredictedVelocityMatchesClearanceTimeDerivative)
{
  state.velocity = Eigen::Vector3d(0.2, -0.1, 0.15);
  const Eigen::Vector3d position(0.45, 0.20, 0.60);
  const double epsilon = 1.0e-6;
  for (const double query_time : {10.5, 12.0})
  {
    double before, after;
    Eigen::Vector3d gradient, predicted_velocity;
    ASSERT_TRUE(evaluateDynamicObstacle(
        position, state, query_time, 10.0, config, nullptr, &gradient,
        nullptr, &predicted_velocity));
    ASSERT_TRUE(evaluateDynamicObstacle(position, state, query_time - epsilon,
                                       10.0, config, &before));
    ASSERT_TRUE(evaluateDynamicObstacle(position, state, query_time + epsilon,
                                       10.0, config, &after));
    EXPECT_NEAR((after - before) / (2.0 * epsilon),
                -gradient.dot(predicted_velocity), 1.0e-6);
  }
}

TEST_F(DynamicObstacleTest, MeasuredBoxVelocityMatchesClearanceTimeDerivative)
{
  state.size = Eigen::Vector3d(0.2, 0.6, 0.4);
  state.velocity = Eigen::Vector3d(0.2, -0.1, 0.15);
  const Eigen::Vector3d position(0.45, 0.50, 0.60);
  const double epsilon = 1.0e-6;
  for (const double query_time : {10.5, 12.0})
  {
    double before, after;
    Eigen::Vector3d gradient, predicted_velocity;
    ASSERT_TRUE(evaluateDynamicObstacle(
        position, state, query_time, 10.0, config, nullptr, &gradient,
        nullptr, &predicted_velocity));
    ASSERT_TRUE(evaluateDynamicObstacle(position, state, query_time - epsilon,
                                       10.0, config, &before));
    ASSERT_TRUE(evaluateDynamicObstacle(position, state, query_time + epsilon,
                                       10.0, config, &after));
    EXPECT_NEAR((after - before) / (2.0 * epsilon),
                -gradient.dot(predicted_velocity), 1.0e-6);
  }
}

TEST_F(DynamicObstacleTest, RejectsNonFiniteState)
{
  state.velocity.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(evaluateDynamicObstacle(state.position, state, 10.0, 10.0,
                                      config, nullptr));
}

} // namespace
} // namespace fuel_fapp

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
