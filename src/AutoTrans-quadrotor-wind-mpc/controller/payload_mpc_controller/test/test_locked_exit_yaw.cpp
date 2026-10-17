#include "mpc_fsm.h"
#include <gtest/gtest.h>
#include <limits>
#include <memory>

using namespace PayloadMPC;

class LockedExitYaw : public ::testing::Test {
 protected:
  void SetUp() override {
    params.config_from_ros_handle(nh);
    params.use_fix_yaw_ = false;
    controller.reset(new MpcController(params));
    fsm.reset(new MPCFSM(nh, params, *controller));
    yaw_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/test/locked_exit_yaw", 1);
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(3.0);
    while (yaw_pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(yaw_pub.getNumSubscribers(), 0U);
    controller->waitForPreparation();
  }

  void TearDown() override {
    ros::Time::init();
  }

  void publishYaw(double yaw) {
    quadrotor_msgs::PositionCommand command;
    command.yaw = yaw;
    yaw_pub.publish(command);
    ros::WallDuration(0.05).sleep();
    ros::spinOnce();
  }

  Trajectory line(const Eigen::Vector3d& velocity) {
    Piece::CoefficientMat coefficients = Piece::CoefficientMat::Zero(3, 6);
    coefficients.col(4) = velocity;
    coefficients.col(5) = Eigen::Vector3d(0.8, -4.27, 1.8);
    Trajectory trajectory;
    trajectory.emplace_back(6.0, coefficients);
    return trajectory;
  }

  double referenceYaw(int column = 0) const {
    const auto& states = controller->reference_states_;
    const Eigen::Quaterniond q(states(kOriW, column), states(kOriX, column),
                                states(kOriY, column), states(kOriZ, column));
    return std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                      1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
  }

  void expectLocked() {
    for (int i = 0; i <= kSamples; ++i) {
      EXPECT_NEAR(std::remainder(referenceYaw(i) - exit_yaw, 2.0*M_PI), 0.0, 1e-5);
      EXPECT_NEAR(controller->reference_inputs_(kRateZ, i), 0.0, 1e-5);
    }
  }

  const double exit_yaw = 178.772 * M_PI / 180.0;
  ros::NodeHandle nh{"~"};
  MpcParams params;
  std::unique_ptr<MpcController> controller;
  std::unique_ptr<MPCFSM> fsm;
  ros::Publisher yaw_pub;
};

TEST_F(LockedExitYaw, CorridorHeadingStillFollowsTrajectoryBeforeExitMessage) {
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d(0.0, 0.0, 0.6), 0.0);
  Trajectory trajectory = line(Eigen::Vector3d(0.0, 0.3, 0.0));
  for (int i = 0; i < 40; ++i) {
    ros::Time::setNow(ros::Time(1000) + ros::Duration(0.01*(i+1)));
    controller->setTrajectoyReference(trajectory, 0.01*i, 0.0);
  }
  EXPECT_GT(referenceYaw(), 0.1);
  EXPECT_GT(std::abs(std::remainder(referenceYaw() - exit_yaw, 2.0*M_PI)), 0.1);
}

TEST_F(LockedExitYaw, CurrentHeadingUsesControlTimeAndPredictionUsesModelTime) {
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.0);
  Trajectory trajectory = line(Eigen::Vector3d(0.0, 0.3, 0.0));
  const double yaw_rate = params.max_bodyrate_z_;
  for (int i = 1; i <= 100; ++i) {
    ros::Time::setNow(ros::Time(1000) + ros::Duration(0.01*i));
    // 重规划将轨迹时间归零，实际偏航仍应按连续的控制时间推进。
    const double trajectory_time = 0.01 * (i <= 50 ? i : i - 50);
    controller->setTrajectoyReference(trajectory, trajectory_time, 0.0);
    ASSERT_NEAR(referenceYaw(), yaw_rate * 0.01*i, 1e-5);
  }
  EXPECT_NEAR(referenceYaw(1) - referenceYaw(), yaw_rate * params.step_T_, 1e-5);
  EXPECT_NEAR(controller->reference_inputs_(kRateZ, 0), yaw_rate, 1e-5);
}

TEST_F(LockedExitYaw, HeadingUsesIrregularElapsedTimeWithoutAdvancingTwice) {
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.0);
  Trajectory trajectory = line(Eigen::Vector3d(0.0, 0.3, 0.0));
  double elapsed = 0.0;
  for (double dt : {0.007, 0.025, 0.012, 0.003, 0.018}) {
    elapsed += dt;
    ros::Time::setNow(ros::Time(1000) + ros::Duration(elapsed));
    controller->setTrajectoyReference(trajectory, elapsed, 0.0);
    EXPECT_NEAR(referenceYaw(), params.max_bodyrate_z_ * elapsed, 1e-5);
    controller->setTrajectoyReference(trajectory, elapsed, 0.0);
    EXPECT_NEAR(referenceYaw(), params.max_bodyrate_z_ * elapsed, 1e-5);
    EXPECT_NEAR(controller->reference_inputs_(kRateZ, 0), 0.0, 1e-5);
    EXPECT_TRUE(controller->reference_states_.allFinite());
    EXPECT_TRUE(controller->reference_inputs_.allFinite());
  }
}

TEST_F(LockedExitYaw, HoverRestartsHeadingClockBeforeAnotherTrajectory) {
  ros::Time::setNow(ros::Time(1000));
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.0);
  Trajectory trajectory = line(Eigen::Vector3d(0.0, 0.3, 0.0));
  ros::Time::setNow(ros::Time(1000, 10000000));
  controller->setTrajectoyReference(trajectory, 0.0, 0.0);
  ros::Time::setNow(ros::Time(1010));
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.2);
  ros::Time::setNow(ros::Time(1010, 10000000));
  controller->setTrajectoyReference(trajectory, 0.0, 0.2);
  EXPECT_NEAR(referenceYaw(), 0.2 + params.max_bodyrate_z_ * 0.01, 1e-5);
}

TEST_F(LockedExitYaw, PlatformApproachAndReplanningKeepPublishedExitYaw) {
  publishYaw(exit_yaw);
  Trajectory platform = line(Eigen::Vector3d(-0.3, -0.18, 0.2));
  controller->setTrajectoyReference(platform, 0.5, -149.5*M_PI/180.0);
  expectLocked();
  publishYaw(-1.0);
  Trajectory replanned = line(Eigen::Vector3d(0.2, -0.4, 0.1));
  controller->setTrajectoyReference(replanned, 1.0, -1.0);
  expectLocked();
}

TEST_F(LockedExitYaw, PrecisionLandingKeepsExitYawAfterPublisherStops) {
  publishYaw(exit_yaw);
  // 复现精降捕获了-149.5度的情况；同一真实参考接口必须继续保持178.772度。
  yaw_pub.shutdown();
  ros::WallDuration(0.6).sleep();
  ASSERT_TRUE(controller->setPositionCommandReference(
      Eigen::Vector3d(0.8, -4.27, 1.5), Eigen::Vector3d(0.0, 0.0, -0.1),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), -149.5*M_PI/180.0, 0.4));
  expectLocked();
  controller->setHoverReference(Eigen::Vector3d(0.8, -4.27, 0.8), -2.6);
  expectLocked();
}

TEST_F(LockedExitYaw, SolverResetKeepsExitYawAndTurnsTowardIt) {
  publishYaw(exit_yaw);
  Eigen::Matrix<real_t, kStateSize, 1> state =
      Eigen::Matrix<real_t, kStateSize, 1>::Zero();
  state(kPosX) = 0.8; state(kPosY) = -4.27; state(kPosZ) = 1.8;
  const double current_yaw = -149.5*M_PI/180.0;
  state(kOriW) = std::cos(current_yaw/2.0);
  state(kOriZ) = std::sin(current_yaw/2.0);
  ASSERT_TRUE(controller->resetForHover(state, state.head<3>().cast<double>(), current_yaw));
  expectLocked();
  Eigen::Matrix<real_t, kStateSize, kSamples + 1> predicted;
  Eigen::Matrix<real_t, kInputSize, kSamples> inputs;
  controller->execMPC(state, predicted, inputs);
  ASSERT_TRUE(controller->lastMpcSolveSuccessful());
  EXPECT_TRUE(inputs.allFinite());
  EXPECT_LT(inputs(kRateZ, 0), 0.0);
}

TEST_F(LockedExitYaw, InvalidMessageDoesNotLatchHeading) {
  publishYaw(std::numeric_limits<double>::quiet_NaN());
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.4);
  EXPECT_NEAR(referenceYaw(), 0.4, 1e-5);
  publishYaw(exit_yaw);
  controller->setHoverReference(Eigen::Vector3d::Zero(), 0.4);
  expectLocked();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_locked_exit_yaw");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
