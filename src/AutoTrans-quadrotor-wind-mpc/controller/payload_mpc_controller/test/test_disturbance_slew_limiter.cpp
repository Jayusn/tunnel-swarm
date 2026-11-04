#include <gtest/gtest.h>

#include <limits>

#include "disturbance_slew_limiter.h"

namespace
{
using PayloadMPC::DisturbanceSlewLimiter;

TEST(DisturbanceSlewLimiter, LimitsHorizontalVectorAndVerticalComponent)
{
    DisturbanceSlewLimiter limiter;
    const auto result = limiter.update(Eigen::Vector3d(3.0, 4.0, -2.0),
                                       0.1, 2.0, 3.0);

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.limited_xy);
    EXPECT_TRUE(result.limited_z);
    EXPECT_NEAR(result.value.head<2>().norm(), 0.2, 1.0e-12);
    EXPECT_NEAR(result.value.x(), 0.12, 1.0e-12);
    EXPECT_NEAR(result.value.y(), 0.16, 1.0e-12);
    EXPECT_NEAR(result.value.z(), -0.3, 1.0e-12);
}

TEST(DisturbanceSlewLimiter, ReachesNearbyTargetWithoutOvershoot)
{
    DisturbanceSlewLimiter limiter;
    ASSERT_TRUE(limiter.update(Eigen::Vector3d(1.0, 0.0, 1.0),
                               0.1, 2.0, 3.0).valid);

    const Eigen::Vector3d target(0.25, 0.0, 0.35);
    const auto result = limiter.update(target, 0.1, 2.0, 3.0);

    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.limited_xy);
    EXPECT_FALSE(result.limited_z);
    EXPECT_TRUE(result.value.isApprox(target, 1.0e-12));
}

TEST(DisturbanceSlewLimiter, ResetClearsPreviousCompensation)
{
    DisturbanceSlewLimiter limiter;
    ASSERT_TRUE(limiter.update(Eigen::Vector3d(1.0, -1.0, 1.0),
                               0.1, 2.0, 3.0).valid);
    limiter.reset();

    EXPECT_TRUE(limiter.value().isZero(1.0e-12));
    const auto result = limiter.update(Eigen::Vector3d(1.0, 0.0, 0.0),
                                       0.01, 2.0, 3.0);
    EXPECT_NEAR(result.value.x(), 0.02, 1.0e-12);
}

TEST(DisturbanceSlewLimiter, ZeroRatesBypassAndTrackLatestTarget)
{
    DisturbanceSlewLimiter limiter;
    ASSERT_TRUE(limiter.update(Eigen::Vector3d::Ones(), 0.01, 2.0, 3.0).valid);
    const Eigen::Vector3d target(3.0, 4.0, -2.0);
    const auto bypass = limiter.update(target, 0.01, 0.0, 0.0);
    ASSERT_TRUE(bypass.valid);
    EXPECT_FALSE(bypass.limited_xy);
    EXPECT_FALSE(bypass.limited_z);
    EXPECT_TRUE(bypass.value.isApprox(target, 1.0e-12));
    const auto resumed = limiter.update(Eigen::Vector3d(4.0, 4.0, -1.0),
                                       0.1, 2.0, 3.0);
    ASSERT_TRUE(resumed.valid);
    EXPECT_TRUE(resumed.value.isApprox(Eigen::Vector3d(3.2, 4.0, -1.7), 1.0e-12));
}

TEST(DisturbanceSlewLimiter, ZeroRatesBypassEachAxisIndependently)
{
    DisturbanceSlewLimiter limiter;
    const Eigen::Vector3d target(3.0, 4.0, -2.0);
    const auto xy_off = limiter.update(target, 0.1, 0.0, 3.0);
    ASSERT_TRUE(xy_off.valid);
    EXPECT_FALSE(xy_off.limited_xy);
    EXPECT_TRUE(xy_off.limited_z);
    EXPECT_TRUE(xy_off.value.isApprox(Eigen::Vector3d(3.0, 4.0, -0.3), 1.0e-12));
    limiter.reset();
    const auto z_off = limiter.update(target, 0.1, 2.0, 0.0);
    ASSERT_TRUE(z_off.valid);
    EXPECT_TRUE(z_off.limited_xy);
    EXPECT_FALSE(z_off.limited_z);
    EXPECT_TRUE(z_off.value.isApprox(Eigen::Vector3d(0.12, 0.16, -2.0), 1.0e-12));
}

TEST(DisturbanceSlewLimiter, InvalidInputFailsClosedAndResetsState)
{
    DisturbanceSlewLimiter limiter;
    ASSERT_TRUE(limiter.update(Eigen::Vector3d(1.0, 0.0, 0.0),
                               0.1, 2.0, 3.0).valid);

    Eigen::Vector3d invalid_target = Eigen::Vector3d::Zero();
    invalid_target.x() = std::numeric_limits<double>::quiet_NaN();
    const auto invalid = limiter.update(invalid_target, 0.1, 2.0, 3.0);

    EXPECT_FALSE(invalid.valid);
    EXPECT_TRUE(invalid.value.isZero(1.0e-12));
    EXPECT_TRUE(limiter.value().isZero(1.0e-12));

    EXPECT_FALSE(limiter.update(Eigen::Vector3d::Zero(), 0.0, 2.0, 3.0).valid);
    EXPECT_FALSE(limiter.update(Eigen::Vector3d::Zero(), 0.1, -1.0, 3.0).valid);
    EXPECT_FALSE(limiter.update(Eigen::Vector3d::Zero(), 0.1, 2.0, -1.0).valid);
}
} // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
