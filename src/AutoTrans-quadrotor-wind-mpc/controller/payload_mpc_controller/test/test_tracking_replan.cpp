#include <gtest/gtest.h>
#include <ros/ros.h>
#include "mpc_input.h"
#include "tracking_deviation.h"

namespace {
quadrotor_msgs::PolynomialTraj::Ptr line(double stamp, unsigned id) {
  quadrotor_msgs::PolynomialTraj::Ptr msg(new quadrotor_msgs::PolynomialTraj);
  msg->header.stamp = ros::Time(stamp);
  msg->trajectory_id = id;
  msg->action = msg->ACTION_ADD;
  quadrotor_msgs::PolynomialMatrix p;
  p.num_order = 1; p.num_dim = 3; p.duration = 8;
  p.data = {0.3, 0, 0, 0, 0, 0.6};
  msg->trajectory.push_back(p);
  return msg;
}
traj_utils::TrackingReplanResult::Ptr response(const std_msgs::Header& request, double stamp) {
  traj_utils::TrackingReplanResult::Ptr r(new traj_utils::TrackingReplanResult);
  r->header = request;
  r->trajectory.traj_id = 9;
  r->trajectory.start_time = ros::Time(stamp);
  r->trajectory.order = 1;
  r->trajectory.duration = {8};
  r->trajectory.coef_x = {0.3, 0};
  r->trajectory.coef_y = {0, 0};
  r->trajectory.coef_z = {0, 0.6};
  return r;
}
class TrackingReplan : public testing::Test {
 protected:
  void SetUp() override { ros::Time::setNow(ros::Time(100.1)); data.feed(line(100, 7)); }
  void TearDown() override { ros::Time::init(); }
  Trajectory_Data_t data;
};
TEST_F(TrackingReplan, GeometrySeparatesAlongRouteLagFromDeviation) {
  const auto& t = data.traj_queue.front().traj;
  EXPECT_NEAR(PayloadMPC::localPathDistance(t, 1.0, Eigen::Vector3d(0.05, 0, .6)), 0, 1e-9);
  EXPECT_NEAR(PayloadMPC::localPathDistance(t, 1.0, Eigen::Vector3d(.3, .11, .6)), .11, 1e-9);
  EXPECT_NEAR(PayloadMPC::localPathDistance(t, 1.0, Eigen::Vector3d(.3, .09, .72)), .15, 1e-9);
  // Never search the far-away section of the same path.
  EXPECT_GT(PayloadMPC::localPathDistance(t, 6.0, Eigen::Vector3d(.3, 0, .6)), 1.0);
}
TEST_F(TrackingReplan, HardRequestInvalidatesSoftResultAndOldQueue) {
  data.beginTrackingReplan(ros::Time(100.1), false);
  const auto soft = data.replan_request;
  ASSERT_EQ(data.traj_queue.size(), 1U);
  ros::Time::setNow(ros::Time(100.3));
  data.feed(line(100.2, 8));
  EXPECT_EQ(data.last_trajectory_id, 7U); // ordinary result cannot replace pending request
  data.beginTrackingReplan(ros::Time(100.3), true);
  EXPECT_TRUE(data.traj_queue.empty());
  EXPECT_FALSE(data.last_end_position_valid);
  EXPECT_EQ(data.exec_traj, -1);
  ros::Time::setNow(ros::Time(100.5));
  data.feedTrackingResult(response(soft, 100.4)); // completed during hard-stop escalation
  EXPECT_TRUE(data.replan_pending);
  EXPECT_TRUE(data.traj_queue.empty());
  auto current = response(data.replan_request, 100.4);
  data.feedTrackingResult(current);
  ASSERT_FALSE(data.replan_pending);
  ASSERT_FALSE(data.replan_stopped);
  ASSERT_EQ(data.traj_queue.size(), 1U);
  EXPECT_EQ(data.exec_traj, 1);
  data.feed(line(100.2, 8)); // delayed old trajectory
  data.feedTrackingResult(response(soft, 100.45));
  EXPECT_EQ(data.last_trajectory_id, 9U);
  EXPECT_EQ(data.last_trajectory_stamp, ros::Time(100.4));
}
TEST_F(TrackingReplan, InvalidOrPreviousSessionResultCannotReleaseStop) {
  data.beginTrackingReplan(ros::Time(100.1), true);
  ros::Time::setNow(ros::Time(100.4));
  auto r = response(data.replan_request, 100.3);
  r->header.stamp = ros::Time(99.0); // same seq, wrong request epoch
  data.feedTrackingResult(r);
  EXPECT_TRUE(data.replan_pending);
  r->header = data.replan_request;
  r->trajectory.coef_z.clear();
  data.feedTrackingResult(r);
  EXPECT_TRUE(data.replan_pending);
  EXPECT_TRUE(data.traj_queue.empty());
  r = response(data.replan_request, 100.3);
  r->trajectory.duration[0] = -1;
  data.feedTrackingResult(r);
  EXPECT_TRUE(data.replan_pending);
  EXPECT_TRUE(data.replan_stopped);
}
TEST_F(TrackingReplan, AbortCannotBeUndoneByPendingResult) {
  data.beginTrackingReplan(ros::Time(100.1), true);
  auto r = response(data.replan_request, 100.3);
  auto abort = line(100.2, 8);
  abort->action = abort->ACTION_ABORT;
  ros::Time::setNow(ros::Time(100.4));
  data.feed(abort);
  data.feedTrackingResult(r);
  EXPECT_FALSE(data.replan_pending);
  EXPECT_TRUE(data.traj_queue.empty());
  EXPECT_EQ(data.exec_traj, -1);
}
TEST_F(TrackingReplan, HigherPriorityHoldOrLandingCancelsPendingResult) {
  data.beginTrackingReplan(ros::Time(100.1), true);
  auto r = response(data.replan_request, 100.3);
  data.allowTrajectoryAcceptanceAfter(ros::Time(100.2));
  data.feedTrackingResult(r);
  EXPECT_TRUE(data.traj_queue.empty());
  data.beginTrackingReplan(ros::Time(100.3), true);
  r = response(data.replan_request, 100.4);
  data.blockTrajectoryAcceptance();
  data.feedTrackingResult(r);
  EXPECT_TRUE(data.traj_queue.empty());
}
}
