#include <gtest/gtest.h>

#include <optimizer/trajectory_sampling.h>

#include <cmath>
#include <limits>
#include <vector>

namespace diff_planner
{
namespace
{

poly_traj::Piece makePiece(double duration, const Eigen::Vector3d &position,
                          const Eigen::Vector3d &velocity,
                          const Eigen::Vector3d &acceleration = Eigen::Vector3d::Zero())
{
  poly_traj::CoefficientMat coefficients = poly_traj::CoefficientMat::Zero();
  coefficients.col(5) = position;
  coefficients.col(4) = velocity;
  coefficients.col(3) = acceleration / 2.0;
  return poly_traj::Piece(duration, coefficients);
}

poly_traj::Trajectory fastMultiPieceTrajectory()
{
  poly_traj::Trajectory trajectory;
  const poly_traj::Piece first =
      makePiece(0.137, Eigen::Vector3d(0.0, 0.0, 1.0),
                Eigen::Vector3d(8.0, 1.0, 0.5), Eigen::Vector3d(70.0, -3.0, 4.0));
  const poly_traj::Piece second =
      makePiece(0.093, first.getPos(first.getDuration()),
                Eigen::Vector3d(-15.0, 20.0, -2.0), Eigen::Vector3d(30.0, 8.0, 1.0));
  const poly_traj::Piece third =
      makePiece(0.22, second.getPos(second.getDuration()),
                Eigen::Vector3d(25.0, -6.0, 3.0), Eigen::Vector3d(-15.0, 2.0, 8.0));
  trajectory.emplace_back(first);
  trajectory.emplace_back(second);
  trajectory.emplace_back(third);
  return trajectory;
}

void expectSamplingLimits(
    const std::vector<std::pair<double, Eigen::Vector3d>> &samples,
    double max_distance, double max_dt)
{
  ASSERT_FALSE(samples.empty());
  for (size_t i = 1; i < samples.size(); ++i)
  {
    EXPECT_GT(samples[i].first, samples[i - 1].first);
    EXPECT_LE(samples[i].first - samples[i - 1].first, max_dt + 1.0e-12);
    EXPECT_LE((samples[i].second - samples[i - 1].second).norm(),
              max_distance + 1.0e-10);
  }
}

size_t countTime(const std::vector<std::pair<double, Eigen::Vector3d>> &samples,
                 double time)
{
  size_t count = 0;
  for (const auto &sample : samples)
    if (std::abs(sample.first - time) < 1.0e-12) ++count;
  return count;
}

TEST(TrajectorySamplingTest, FastPiecesRespectTimeAndHalfVoxelDistance)
{
  const auto trajectory = fastMultiPieceTrajectory();
  const double half_voxel = 0.025;
  const auto samples = sampleCollisionTrajectory(
      trajectory, 0.0, trajectory.getTotalDuration(), half_voxel);
  expectSamplingLimits(samples, half_voxel, 0.05);
  EXPECT_DOUBLE_EQ(samples.front().first, 0.0);
  EXPECT_NEAR(samples.back().first, trajectory.getTotalDuration(), 1.0e-12);
  for (const double junction : {0.0, 0.137, 0.23, 0.45})
    EXPECT_EQ(countTime(samples, junction), 1U);
  for (const auto &sample : samples)
    EXPECT_TRUE(sample.second.isApprox(trajectory.getPos(sample.first), 1.0e-11));
}

TEST(TrajectorySamplingTest, ConstantVelocityStillRespectsSpatialLimit)
{
  poly_traj::Trajectory trajectory;
  trajectory.emplace_back(makePiece(0.2, Eigen::Vector3d::Zero(),
                                   Eigen::Vector3d(40.0, 3.0, 1.0)));
  const auto samples = sampleCollisionTrajectory(trajectory, 0.0, 0.2, 0.025);
  expectSamplingLimits(samples, 0.025, 0.05);
  ASSERT_FALSE(samples.empty());
  EXPECT_TRUE(samples.back().second.isApprox(Eigen::Vector3d(8.0, 0.6, 0.2)));
}

TEST(TrajectorySamplingTest, SlowMotionAndStationarySegmentsRespectTimeLimit)
{
  poly_traj::Trajectory trajectory;
  trajectory.emplace_back(makePiece(0.23, Eigen::Vector3d::Zero(),
                                   Eigen::Vector3d(0.01, 0.0, 0.0)));
  trajectory.emplace_back(makePiece(0.17, Eigen::Vector3d(0.0023, 0.0, 0.0),
                                   Eigen::Vector3d::Zero()));
  const auto samples = sampleCollisionTrajectory(trajectory, 0.0, 0.4, 0.025);
  expectSamplingLimits(samples, 0.025, 0.05);
  EXPECT_EQ(countTime(samples, 0.23), 1U);
  EXPECT_EQ(countTime(samples, 0.4), 1U);
}

TEST(TrajectorySamplingTest, PartialIntervalUsesRelativeTimesAndRetainsJunctions)
{
  const auto trajectory = fastMultiPieceTrajectory();
  const double begin = 0.06;
  const double end = 0.31;
  const auto samples = sampleCollisionTrajectory(trajectory, begin, end, 0.025, 0.02);
  expectSamplingLimits(samples, 0.025, 0.02);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.front().first, 0.0);
  EXPECT_NEAR(samples.back().first, end - begin, 1.0e-12);
  EXPECT_EQ(countTime(samples, 0.137 - begin), 1U);
  EXPECT_EQ(countTime(samples, 0.23 - begin), 1U);
  for (const auto &sample : samples)
    EXPECT_TRUE(sample.second.isApprox(trajectory.getPos(begin + sample.first), 1.0e-11));
}

TEST(TrajectorySamplingTest, FastMotionCannotSkipOneVoxelThickObstacle)
{
  poly_traj::Trajectory trajectory;
  trajectory.emplace_back(makePiece(0.1, Eigen::Vector3d(-1.0, 0.0, 1.0),
                                   Eigen::Vector3d(20.0, 0.0, 0.0)));
  // A 0.05 s time-only sampler visits x=-1, 0, 1 and misses this voxel.
  const auto samples = sampleCollisionTrajectory(trajectory, 0.0, 0.1, 0.025);
  bool hit = false;
  for (const auto &sample : samples)
    hit = hit || (sample.second.x() >= 0.101 && sample.second.x() <= 0.151);
  EXPECT_TRUE(hit);
  expectSamplingLimits(samples, 0.025, 0.05);
}

TEST(TrajectorySamplingTest, IntervalClampsToDurationAndPointIntervalIsSingleSample)
{
  const auto trajectory = fastMultiPieceTrajectory();
  const auto clamped = sampleCollisionTrajectory(trajectory, -1.0, 5.0, 0.025);
  ASSERT_FALSE(clamped.empty());
  EXPECT_DOUBLE_EQ(clamped.front().first, 0.0);
  EXPECT_NEAR(clamped.back().first, trajectory.getTotalDuration(), 1.0e-12);
  const auto point = sampleCollisionTrajectory(trajectory, 0.137, 0.137, 0.025);
  ASSERT_EQ(point.size(), 1U);
  EXPECT_DOUBLE_EQ(point.front().first, 0.0);
  EXPECT_TRUE(point.front().second.isApprox(trajectory.getPos(0.137), 1.0e-12));
}

TEST(TrajectorySamplingTest, InvalidArgumentsAndEmptyTrajectoryReturnEmpty)
{
  const auto trajectory = fastMultiPieceTrajectory();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(sampleCollisionTrajectory(poly_traj::Trajectory(), 0.0, 1.0, 0.025).empty());
  EXPECT_TRUE(sampleCollisionTrajectory(trajectory, nan, 0.2, 0.025).empty());
  EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 0.0, infinity, 0.025).empty());
  EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 0.2, 0.1, 0.025).empty());
  EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 2.0, 3.0, 0.025).empty());
  for (const double invalid : {0.0, -0.01, nan, infinity})
  {
    EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 0.0, 0.2, invalid).empty());
    EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 0.0, 0.2, 0.025, invalid).empty());
  }
  EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 0.0, 0.2, 1.0e-10).empty());
}

TEST(TrajectorySamplingTest, InvalidPieceDurationReturnsEmpty)
{
  for (const double invalid : {0.0, -0.01, std::numeric_limits<double>::infinity()})
  {
    poly_traj::Trajectory trajectory;
    trajectory.emplace_back(makePiece(invalid, Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero()));
    EXPECT_TRUE(sampleCollisionTrajectory(trajectory, 0.0, 0.1, 0.025).empty());
  }
}

} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
