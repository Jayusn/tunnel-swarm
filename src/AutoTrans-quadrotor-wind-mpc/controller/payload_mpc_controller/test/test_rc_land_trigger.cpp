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
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
