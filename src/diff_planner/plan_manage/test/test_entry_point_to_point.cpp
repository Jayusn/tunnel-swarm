#include <gtest/gtest.h>
#include <diff_planner/entry_point_to_point.h>

namespace motion = diff_planner::entry_point_to_point;

TEST(EntryPointToPoint, StartsAtCurrentPositionAndStopsAtGoal) {
  const double duration = motion::duration(1.66, 1.0, 1.0);
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
    const double duration = motion::duration(distance, 1.0, 1.0);
    double previous = 0.0;
    for (int i = 0; i <= 10000; ++i) {
      const auto s = motion::sample(duration * i / 10000.0, duration);
      EXPECT_GE(s.position + 1e-12, previous);
      EXPECT_LE(s.position, 1.0 + 1e-12);
      EXPECT_LE(distance * std::abs(s.velocity), 1.0 + 1e-9);
      EXPECT_LE(distance * std::abs(s.acceleration), 1.0 + 1e-9);
      previous = s.position;
    }
  }
}

TEST(EntryPointToPoint, FeedforwardMatchesPositionDerivatives) {
  const double duration = motion::duration(1.66, 1.0, 1.0);
  const double h = 1e-5;
  for (double fraction : {0.1, 0.3, 0.5, 0.8}) {
    const double t = fraction * duration;
    const auto a = motion::sample(t-h, duration);
    const auto b = motion::sample(t+h, duration);
    const auto s = motion::sample(t, duration);
    EXPECT_NEAR((b.position-a.position)/(2*h), s.velocity, 1e-8);
    EXPECT_NEAR((b.velocity-a.velocity)/(2*h), s.acceleration, 1e-8);
    // 段连接处四阶导数不连续，跨段中心差分的误差为 O(h)。
    EXPECT_NEAR((b.acceleration-a.acceleration)/(2*h), s.jerk,
                fraction == 0.5 ? 1e-5 : 1e-8);
  }
}

TEST(EntryPointToPoint, TwoPiecesMeetAtMidpointAtOneMetrePerSecond) {
  const double duration = motion::duration(1.66, 1.0, 1.0);
  EXPECT_NEAR(duration, 3.32, 1e-12);
  const auto mid = motion::sample(duration/2, duration);
  EXPECT_NEAR(mid.position, 0.5, 1e-12);
  EXPECT_NEAR(1.66*mid.velocity, 1.0, 1e-12);
  EXPECT_NEAR(mid.acceleration, 0.0, 1e-12);
  const auto left = motion::sample(duration/2-1e-7, duration);
  const auto right = motion::sample(duration/2+1e-7, duration);
  EXPECT_NEAR(left.position, right.position, 1e-6);
  EXPECT_NEAR(left.velocity, right.velocity, 1e-6);
  EXPECT_NEAR(left.acceleration, right.acceleration, 1e-6);
  EXPECT_NEAR(left.jerk, right.jerk, 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
