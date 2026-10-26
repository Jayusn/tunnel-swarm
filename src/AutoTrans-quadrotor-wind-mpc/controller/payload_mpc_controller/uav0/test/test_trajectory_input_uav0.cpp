#include <gtest/gtest.h>
#include "mpc_input.h"

namespace {
quadrotor_msgs::PolynomialTrajUav0::Ptr trajectory() {
  ros::Time::init();
  ros::Time::setNow(ros::Time(100));
  auto message = boost::make_shared<quadrotor_msgs::PolynomialTrajUav0>();
  message->header.stamp = ros::Time(100);
  message->trajectory_id = 1;
  message->action = message->ACTION_ADD;
  quadrotor_msgs::PolynomialMatrix position;
  position.num_order = 1;
  position.num_dim = 3;
  position.duration = 2.0;
  position.data = {1, 0, 0, 0, 0, 1};
  message->trajectory.push_back(position);
  auto yaw = position;
  yaw.num_dim = 1;
  yaw.data = {0.2, 0.3};
  message->has_yaw = true;
  message->yaw_trajectory.push_back(yaw);
  return message;
}
}

TEST(FrontTrajectoryInput, PreservesYawAndPositionPolynomials) {
  Trajectory_Data_t input;
  input.feed(trajectory());
  ASSERT_EQ(1u, input.traj_queue.size());
  const auto& accepted = input.traj_queue.front();
  ASSERT_TRUE(accepted.has_yaw);
  EXPECT_NEAR(0.4, accepted.yaw_traj.getPos(0.5)(0), 1e-9);
  EXPECT_NEAR(0.2, accepted.yaw_traj.getVel(0.5)(0), 1e-9);
  EXPECT_NEAR(0.5, accepted.traj.getPos(0.5)(0), 1e-9);
  EXPECT_NEAR(1.0, accepted.traj.getPos(0.5)(2), 1e-9);
}

TEST(FrontTrajectoryInput, RejectsMismatchedYawDuration) {
  Trajectory_Data_t input;
  auto message = trajectory();
  message->yaw_trajectory.front().duration = 1.0;
  input.feed(message);
  EXPECT_TRUE(input.traj_queue.empty());
}

TEST(FrontTrajectoryInput, SupportsTrajectoryWithoutYaw) {
  Trajectory_Data_t input;
  auto message = trajectory();
  message->has_yaw = false;
  message->yaw_trajectory.clear();
  input.feed(message);
  ASSERT_EQ(1u, input.traj_queue.size());
  EXPECT_FALSE(input.traj_queue.front().has_yaw);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
