#include <gtest/gtest.h>
#include <diff_planner/entry_point_to_point.h>

namespace motion = diff_planner::entry_point_to_point;

TEST(EntryPointToPoint, StartsAtCurrentPositionAndStopsAtGoal) {
  const double duration = motion::duration(1.66, 0.4, 0.4);
  const auto start = motion::sample(0.0, duration);
  const auto end = motion::sample(duration, duration);
  EXPECT_DOUBLE_EQ(start.position, 0.0);
  EXPECT_DOUBLE_EQ(start.velocity, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(end.position, 1.0);
  EXPECT_DOUBLE_EQ(end.velocity, 0.0);
  EXPECT_DOUBLE_EQ(end.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(end.jerk, 0.0);
  EXPECT_DOUBLE_EQ(motion::sample(duration + 5.0, duration).position, 1.0);
}

TEST(EntryPointToPoint, RespectsVectorSpeedAndAccelerationForShortAndLongMoves) {
  for (double distance : {0.0, 0.001, 0.1, 1.66, 10.0}) {
    const double duration = motion::duration(distance, 0.4, 0.4);
    double previous = 0.0;
    for (int i = 0; i <= 10000; ++i) {
      const auto s = motion::sample(duration * i / 10000.0, duration);
      EXPECT_GE(s.position + 1e-12, previous);
      EXPECT_LE(s.position, 1.0 + 1e-12);
      EXPECT_LE(distance * std::abs(s.velocity), 0.4 + 1e-9);
      EXPECT_LE(distance * std::abs(s.acceleration), 0.4 + 1e-9);
      previous = s.position;
    }
  }
}

TEST(EntryPointToPoint, FeedforwardMatchesPositionDerivatives) {
  const double duration = motion::duration(1.66, 0.4, 0.4);
  const double h = 1e-5;
  for (double fraction : {0.1, 0.3, 0.5, 0.8}) {
    const double t = fraction * duration;
    const auto a = motion::sample(t-h, duration);
    const auto b = motion::sample(t+h, duration);
    const auto s = motion::sample(t, duration);
    EXPECT_NEAR((b.position-a.position)/(2*h), s.velocity, 1e-8);
    EXPECT_NEAR((b.velocity-a.velocity)/(2*h), s.acceleration, 1e-8);
    EXPECT_NEAR((b.acceleration-a.acceleration)/(2*h), s.jerk, 1e-8);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
