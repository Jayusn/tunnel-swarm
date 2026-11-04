#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include "mpc_input.h"

namespace {
void feed(RC_Data_t& rc, uint16_t pwm) {
  auto msg = boost::make_shared<mavros_msgs::RCIn>();
  msg->channels.assign(10, 1500);
  msg->channels[9] = pwm;
  rc.feed(msg);
}

TEST(RcLandTrigger, StartupHighDoesNotTrigger) {
  ros::Time::init();
  RC_Data_t rc;
  feed(rc, 2000);
  feed(rc, 2000);
  EXPECT_FALSE(rc.enter_land_mode);
}

TEST(RcLandTrigger, QueuedHighMessagesCannotEraseUnconsumedEdge) {
  ros::Time::init();
  RC_Data_t rc;
  feed(rc, 1000);
  feed(rc, 2000);
  feed(rc, 2000);
  feed(rc, 1000);
  EXPECT_TRUE(rc.enter_land_mode);
  // 与 MPCFSM 控制周期末尾消费事件的方式一致。
  rc.enter_land_mode = false;
  feed(rc, 1000);
  EXPECT_FALSE(rc.enter_land_mode);
  feed(rc, 2000);
  EXPECT_TRUE(rc.enter_land_mode);
  rc.enter_land_mode = false;
  feed(rc, 2000);
  EXPECT_FALSE(rc.enter_land_mode);
}
void feedMode(RC_Data_t& rc, uint16_t ch8, uint16_t ch9, size_t count = 10) {
  auto msg = boost::make_shared<mavros_msgs::RCIn>();
  msg->channels.assign(count, 1500);
  if (count > 7) msg->channels[7] = ch8;
  if (count > 8) msg->channels[8] = ch9;
  rc.feed(msg);
}

TEST(RcMode, Ch9LowTakeoffHighHoverWithCh8LowOrMiddle) {
  ros::Time::init();
  for (uint16_t ch8 : {1000, 1500}) {
    RC_Data_t rc;
    feedMode(rc, ch8, 1000);
    EXPECT_TRUE(rc.mode_input_valid);
    EXPECT_TRUE(rc.is_takeoff_mode);
    EXPECT_FALSE(rc.is_hover_mode);
    EXPECT_FALSE(rc.is_command_mode);
    feedMode(rc, ch8, 2000);
    EXPECT_FALSE(rc.is_takeoff_mode);
    EXPECT_TRUE(rc.is_hover_mode);
    EXPECT_TRUE(rc.enter_hover_mode);
    EXPECT_FALSE(rc.is_command_mode);
    feedMode(rc, ch8, 2000);
    EXPECT_FALSE(rc.enter_hover_mode);
  }
}

TEST(RcMode, Ch8HighOverridesBothCh9PositionsAndReleasesToCh9) {
  ros::Time::init();
  for (uint16_t ch9 : {1000, 2000}) {
    RC_Data_t rc;
    feedMode(rc, 1000, ch9);
    feedMode(rc, 2000, ch9);
    EXPECT_TRUE(rc.is_command_mode);
    EXPECT_TRUE(rc.enter_command_mode);
    EXPECT_FALSE(rc.is_takeoff_mode);
    EXPECT_FALSE(rc.is_hover_mode);
    feedMode(rc, 2000, ch9);
    EXPECT_FALSE(rc.enter_command_mode);
    feedMode(rc, 1500, ch9);
    EXPECT_FALSE(rc.is_command_mode);
    EXPECT_EQ(rc.is_takeoff_mode, ch9 == 1000);
    EXPECT_EQ(rc.is_hover_mode, ch9 == 2000);
  }
}

TEST(RcMode, Ch9MiddleIsNotAThirdMode) {
  ros::Time::init();
  RC_Data_t rc;
  for (uint16_t pwm : {1300, 1500, 1800}) {
    feedMode(rc, 1000, pwm);
    EXPECT_FALSE(rc.is_takeoff_mode);
    EXPECT_FALSE(rc.is_hover_mode);
    EXPECT_FALSE(rc.is_command_mode);
  }
}

TEST(RcMode, MissingOrInvalidChannelKeepsRequestButMarksInputInvalid) {
  ros::Time::init();
  RC_Data_t rc;
  feedMode(rc, 1000, 2000);
  feedMode(rc, 1000, 0, 8);
  EXPECT_FALSE(rc.mode_input_valid);
  EXPECT_TRUE(rc.is_hover_mode);
  feedMode(rc, 1000, 0);
  EXPECT_FALSE(rc.mode_input_valid);
  EXPECT_TRUE(rc.is_hover_mode);
  feedMode(rc, 0, 1000);
  EXPECT_FALSE(rc.mode_input_valid);
  EXPECT_TRUE(rc.is_hover_mode);
  feedMode(rc, 1000, 1000);
  EXPECT_TRUE(rc.mode_input_valid);
  EXPECT_TRUE(rc.is_takeoff_mode);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
