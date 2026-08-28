#include <gtest/gtest.h>

#include <limits>

#include "vertical_disturbance_low_pass.h"

namespace
{
using PayloadMPC::VerticalDisturbanceLowPass;

TEST(VerticalDisturbanceLowPass, SmoothsStepInput)
{
    VerticalDisturbanceLowPass filter;
    ASSERT_TRUE(filter.configure(100.0, 0.7));

    const auto first = filter.update(1.0);
    ASSERT_TRUE(first.valid);
    EXPECT_GT(first.value, 0.0);
    EXPECT_LT(first.value, 0.01);

    double value = first.value;
    for (int i = 0; i < 1000; ++i)
        value = filter.update(1.0).value;
    EXPECT_NEAR(value, 1.0, 1.0e-3);
}

TEST(VerticalDisturbanceLowPass, ZeroCutoffBypassesExactly)
{
    VerticalDisturbanceLowPass filter;
    ASSERT_TRUE(filter.configure(100.0, 0.0));
    EXPECT_FALSE(filter.enabled());

    const auto result = filter.update(-0.375);
    ASSERT_TRUE(result.valid);
    EXPECT_DOUBLE_EQ(result.value, -0.375);
}

TEST(VerticalDisturbanceLowPass, ResetClearsFilterHistory)
{
    VerticalDisturbanceLowPass fresh;
    VerticalDisturbanceLowPass used;
    ASSERT_TRUE(fresh.configure(100.0, 0.7));
    ASSERT_TRUE(used.configure(100.0, 0.7));

    for (int i = 0; i < 500; ++i)
        ASSERT_TRUE(used.update(1.0).valid);
    used.reset();

    const auto fresh_result = fresh.update(1.0);
    const auto reset_result = used.update(1.0);
    ASSERT_TRUE(fresh_result.valid);
    ASSERT_TRUE(reset_result.valid);
    EXPECT_DOUBLE_EQ(reset_result.value, fresh_result.value);
}

TEST(VerticalDisturbanceLowPass, InvalidInputFailsClosedAndResets)
{
    VerticalDisturbanceLowPass filter;
    ASSERT_TRUE(filter.configure(100.0, 0.7));
    for (int i = 0; i < 500; ++i)
        ASSERT_TRUE(filter.update(1.0).valid);

    const auto invalid = filter.update(std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(invalid.valid);
    EXPECT_DOUBLE_EQ(invalid.value, 0.0);

    VerticalDisturbanceLowPass fresh;
    ASSERT_TRUE(fresh.configure(100.0, 0.7));
    EXPECT_DOUBLE_EQ(filter.update(1.0).value, fresh.update(1.0).value);
}

TEST(VerticalDisturbanceLowPass, RejectsInvalidConfiguration)
{
    VerticalDisturbanceLowPass filter;
    EXPECT_FALSE(filter.configure(0.0, 0.7));
    EXPECT_FALSE(filter.configure(100.0, -0.1));
    EXPECT_FALSE(filter.configure(100.0, 50.0));
    EXPECT_FALSE(filter.update(1.0).valid);
}
} // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
