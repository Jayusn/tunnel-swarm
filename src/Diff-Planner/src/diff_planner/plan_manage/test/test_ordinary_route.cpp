#include <gtest/gtest.h>
#include <plan_manage/ordinary_route.h>

namespace {
using Eigen::Vector3d;
using diff_planner::OrdinaryRouteResult;
using diff_planner::buildOrdinaryRoute;
const auto free_space = [](const Vector3d&) { return true; };

TEST(OrdinaryRoute, PassesUnsafeStopAndStopsAtFirstClearEndpoint) {
  OrdinaryRouteResult result;
  ASSERT_TRUE(buildOrdinaryRoute(Vector3d(0,0,.6), Vector3d::Zero(),
      {Vector3d(1,0,.6), Vector3d(2,0,.6), Vector3d(3,0,.6)}, Vector3d(5,0,.6),
      1.3, .4, .5, .05, [](const Vector3d& p) { return p.x() > 1.5; },
      free_space, result));
  EXPECT_EQ(result.count, 2U);
  const auto curve = result.polynomial.getTraj();
  bool passed = false;
  for (int i=1; i<curve.getPieceNum(); ++i)
    if ((curve.getJuncPos(i) - Vector3d(1,0,.6)).norm() < 1e-8) {
      passed = true;
      EXPECT_GT(curve.getJuncVel(i).norm(), .01);
    }
  EXPECT_TRUE(passed);
  EXPECT_LT(curve.getVel(curve.getTotalDuration()).norm(), 1e-8);
  EXPECT_LT(curve.getAcc(curve.getTotalDuration()).norm(), 1e-8);
  EXPECT_LE(curve.getMaxVelRate(), .400001);
  EXPECT_LE(curve.getMaxAccRate(), .500001);
}

TEST(OrdinaryRoute, NoClearStopNeverGeneratesMotion) {
  OrdinaryRouteResult result;
  EXPECT_FALSE(buildOrdinaryRoute(Vector3d(0,0,.6), Vector3d::Zero(),
      {Vector3d(1,0,.6)}, Vector3d(5,0,.6), 1.3, .4, .5, .05,
      [](const Vector3d&) { return false; }, free_space, result));
  EXPECT_EQ(result.count, 0U);
  EXPECT_EQ(result.reason, "no_clear_stop");
}

TEST(OrdinaryRoute, ChecksMiddleOfTrajectoryAndResumesAfterLeaderMoves) {
  OrdinaryRouteResult result;
  const std::vector<Vector3d> points{Vector3d(4,0,.6)};
  EXPECT_FALSE(buildOrdinaryRoute(Vector3d(0,0,.6), Vector3d::Zero(), points,
      Vector3d(2,0,.6), 1.3, .4, .5, .05, free_space, free_space, result));
  EXPECT_EQ(result.reason, "leader_spacing");
  EXPECT_TRUE(buildOrdinaryRoute(Vector3d(0,0,.6), Vector3d::Zero(), points,
      Vector3d(6,0,.6), 1.3, .4, .5, .05, free_space, free_space, result));
}

TEST(OrdinaryRoute, SeparationIncludesTrackingMarginAndBoundary) {
  for (const double gap : {1.19, 1.2, 1.29, 1.3, 1.31}) {
    OrdinaryRouteResult result;
    EXPECT_EQ(buildOrdinaryRoute(Vector3d(0,0,.6), Vector3d::Zero(),
        {Vector3d(1,0,.6)}, Vector3d(1+gap,0,.6), 1.3, .4, .5, .05,
        free_space, free_space, result), gap >= 1.3);
  }
}

TEST(OrdinaryRoute, CornerCannotBeReplacedByDirectChordOrWideCut) {
  OrdinaryRouteResult result;
  const auto corridor = [](const Vector3d& p) {
    return (p.x() <= 1.05 && std::fabs(p.y()) <= .05) ||
           (std::fabs(p.x()-1) <= .05 && p.y() >= -.05);
  };
  const bool ready = buildOrdinaryRoute(Vector3d(0,0,.6), Vector3d::Zero(),
      {Vector3d(1,0,.6), Vector3d(1,1,.6)}, Vector3d(5,5,.6), 1.3, .4, .5, .05,
      [](const Vector3d& p) { return p.y() > .5; }, corridor, result);
  ASSERT_TRUE(ready) << result.reason;
  {
    const auto curve = result.polynomial.getTraj();
    EXPECT_EQ(result.count, 2U);
    bool passed = false;
    for (int i=1; i<curve.getPieceNum(); ++i)
      passed = passed || (curve.getJuncPos(i) - Vector3d(1,0,.6)).norm() < 1e-8;
    EXPECT_TRUE(passed);
    for (double t=0; t<curve.getTotalDuration(); t+=.01) EXPECT_TRUE(corridor(curve.getPos(t)));
  }
}
} // namespace
