#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/swing_obstacle_guard.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/DataDisp.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <quadrotor_msgs/GoalSet.h>
#include <ldop/DynamicObjectArray.h>
#include <obj_state_msgs/ObjectsStates.h>

#define private public
#include <plan_manage/diff_replan_fsm.h>
#undef private

namespace
{
class PendingMapWork : public ros::CallbackInterface
{
public:
  bool called{false};
  CallResult call() override { called = true; return Success; }
};

TEST(PlannerOdometry, LatestStateIsAvailableBeforeQueuedMapWork)
{
  ros::NodeHandle nh("~");
  diff_planner::DiffReplanFSM planner;
  planner.have_odom_ = false;
  planner.initOdometry(nh);
  auto publisher = nh.advertise<nav_msgs::Odometry>("odom_world", 10);
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (publisher.getNumSubscribers() == 0 && ros::WallTime::now() < deadline)
    ros::WallDuration(0.01).sleep();
  ASSERT_GT(publisher.getNumSubscribers(), 0U);

  auto *pending = new PendingMapWork;
  ros::getGlobalCallbackQueue()->addCallback(ros::CallbackInterfacePtr(pending));
  nav_msgs::Odometry sample;
  sample.header.stamp = ros::Time::now() - ros::Duration(1.8);
  sample.pose.pose.position.x = 0.72;
  publisher.publish(sample);
  ros::WallDuration(0.03).sleep();
  sample.header.stamp = ros::Time::now();
  sample.pose.pose.position.x = 1.15;
  sample.twist.twist.linear.x = 0.28;
  publisher.publish(sample);
  ros::WallDuration(0.03).sleep();

  // A planning callback can refresh the latest state without running queued
  // map work and without a second thread mutating the FSM.
  planner.refreshOdometry();
  ASSERT_TRUE(planner.have_odom_);
  EXPECT_DOUBLE_EQ(planner.odom_pos_.x(), 1.15);
  EXPECT_DOUBLE_EQ(planner.odom_vel_.x(), 0.28);
  EXPECT_EQ(planner.odom_stamp_, sample.header.stamp);
  EXPECT_FALSE(pending->called);
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_planner_odometry");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
