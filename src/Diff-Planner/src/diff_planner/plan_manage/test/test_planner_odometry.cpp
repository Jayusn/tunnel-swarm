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
#include <pcl_conversions/pcl_conversions.h>

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

class PlannerEndpoint : public testing::Test
{
protected:
  using FSM = diff_planner::DiffReplanFSM;
  void SetUp() override
  {
    ros::Time::setNow(ros::Time(1000));
    // A small real map and optimizer, with the relay's arrival/tracking limits.
    nh.setParam("fsm/flight_type", 1);
    nh.setParam("fsm/waypoint_switch_dist", 0.12);
    nh.setParam("fsm/thresh_replan_time", 1.0);
    nh.setParam("fsm/planning_horizon", 8.0);
    nh.setParam("fsm/enable_stuck_detect", false);
    nh.setParam("fsm/enable_occupied_recovery", true);
    nh.setParam("fsm/require_pre_agent_trajectory", false);
    nh.setParam("optimization/constraint_points_perPiece", 6);
    nh.setParam("optimization/enable_fapp_dynamic_avoidance", true);
    nh.setParam("optimization/fapp_planner_priority", true);
    nh.setParam("manager/drone_id", 1);
    for (const auto &parameter : std::vector<std::pair<std::string, double>>{
        {"grid_map/resolution", 0.05}, {"grid_map/obstacles_inflation", 0.099},
        {"grid_map/local_update_range_x", 3.0},
        {"grid_map/local_update_range_y", 3.0},
        {"grid_map/local_update_range_z", 1.5}, {"grid_map/fading_time", 0.0},
        {"manager/max_vel", 0.5}, {"manager/max_acc", 1.0},
        {"manager/planning_horizon", 8.0}, {"manager/polyTraj_piece_length", 1.5},
        {"optimization/weight_obstacle", 10000.0},
        {"optimization/weight_obstacle_soft", 1000.0},
        {"optimization/weight_swarm", 10000.0},
        {"optimization/weight_feasibility", 10000.0},
        {"optimization/weight_sqrvariance", 100.0},
        {"optimization/weight_time", 5.0},
        {"optimization/obstacle_clearance", 0.18},
        {"optimization/obstacle_clearance_soft", 0.30},
        {"optimization/swarm_clearance", 0.15},
        {"optimization/max_vel", 0.5}, {"optimization/vel_tolerance", 0.2},
        {"optimization/max_acc", 1.0}, {"optimization/acc_tolerance", 1.0},
        {"optimization/max_jer", 8.0}})
      nh.setParam(parameter.first, parameter.second);

    planner.init(nh);
    planner.exec_timer_.stop();
    planner.safety_timer_.stop();
    auto odom_pub = nh.advertise<nav_msgs::Odometry>("grid_map/odom", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map/cloud", 1);
    trajectory_sub = nh.subscribe<traj_utils::PolyTraj>("planning/trajectory", 10,
        [this](const traj_utils::PolyTraj::ConstPtr &) { ++published; });
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while ((odom_pub.getNumSubscribers() == 0 || cloud_pub.getNumSubscribers() == 0 ||
            planner.poly_traj_pub_.getNumSubscribers() == 0) &&
           ros::WallTime::now() < deadline)
      ros::WallDuration(0.01).sleep();
    ASSERT_GT(odom_pub.getNumSubscribers(), 0U);
    ASSERT_GT(cloud_pub.getNumSubscribers(), 0U);
    ASSERT_GT(planner.poly_traj_pub_.getNumSubscribers(), 0U);
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = goal.x() - 0.08;
    odom.pose.pose.position.y = goal.y();
    odom.pose.pose.position.z = goal.z();
    odom.pose.pose.orientation.w = 1.0;
    odom_pub.publish(odom);
    drain();
    publishPoint(goal + Eigen::Vector3d(0.0, 2.0, 0.0));
    planner.odometryCallback(boost::make_shared<nav_msgs::Odometry>(odom));

    Eigen::Matrix3d head, tail;
    head << goal - Eigen::Vector3d(1.0, 0.0, 0.0),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    tail << goal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    poly_traj::MinJerkOpt polynomial;
    polynomial.reset(head, tail, 1);
    polynomial.generate(Eigen::MatrixXd(3, 0), Eigen::VectorXd::Constant(1, 4.815));
    ASSERT_TRUE(planner.planner_manager_->planGlobalTrajWaypoints(
        head.col(0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        {goal}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    planner.planner_manager_->traj_.setLocalTraj(polynomial.getTraj(), {}, 994.485);
    planner.local_target_pt_ = planner.final_goal_ = goal;
    planner.local_target_vel_.setZero();
    planner.have_target_ = planner.have_trigger_ = true;
    planner.have_new_target_ = false;
    planner.exec_state_ = FSM::EXEC_TRAJ;
    initial_id = planner.planner_manager_->traj_.local_traj.traj_id;
  }

  void TearDown() override
  {
    planner.exec_timer_.stop();
    ros::Time::init();
  }
  void drain()
  {
    ros::WallDuration(0.03).sleep();
    ros::spinOnce();
  }
  void publishPoint(const Eigen::Vector3d &point)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.stamp = ros::Time::now();
    // Occupancy requires repeated hits from the initially free log odds.
    for (int i = 0; i < 3; ++i) {
      cloud_pub.publish(message);
      drain();
    }
  }
  void exec()
  {
    planner.execFSMCallback(ros::TimerEvent());
    planner.exec_timer_.stop();
  }
  void expectNoReplacement()
  {
    drain();
    EXPECT_EQ(planner.planner_manager_->traj_.local_traj.traj_id, initial_id);
    EXPECT_EQ(published, 0);
  }

  ros::NodeHandle nh{"~endpoint"};
  FSM planner;
  Eigen::Vector3d goal{3.32939, 0.385744, 0.6};
  ros::Publisher cloud_pub;
  ros::Subscriber trajectory_sub;
  int published{0}, initial_id{0};
};

TEST_F(PlannerEndpoint, CollisionTimerFinishesOldGoalBeforeReplanning)
{
  // A map update occupies the old endpoint, but the measured vehicle remains
  // free and within 8 cm. Previously the safety timer generated a return trip.
  publishPoint(goal + Eigen::Vector3d(0.10, 0.0, 0.0));
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(goal), 0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.checkCollisionCallback(ros::TimerEvent());
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.have_target_);
  for (int i = 0; i < 5; ++i) {
    exec();
    planner.checkCollisionCallback(ros::TimerEvent());
  }
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, MainTimerAlsoFinishesUsingMeasuredArrival)
{
  planner.odom_pos_ = Eigen::Vector3d(3.347, 0.443, 0.576);
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.have_target_);
  EXPECT_FALSE(planner.have_trigger_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, QueuedPeriodicReplanCannotRestartCompletedGoal)
{
  planner.exec_state_ = FSM::REPLAN_TRAJ;
  planner.periodic_replan_pending_ = true;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.periodic_replan_pending_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, ExpirationWaitsForSmallRemainingErrorToConverge)
{
  planner.odom_pos_ = goal - Eigen::Vector3d(0.18, 0.0, 0.0);
  planner.exec_state_ = FSM::REPLAN_TRAJ;
  planner.periodic_replan_pending_ = true;
  for (int i = 0; i < 3; ++i) {
    planner.checkCollisionCallback(ros::TimerEvent());
    exec();
  }
  EXPECT_TRUE(planner.have_target_);
  EXPECT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  EXPECT_FALSE(planner.periodic_replan_pending_);
  expectNoReplacement();
  planner.odom_pos_ = goal - Eigen::Vector3d(0.08, 0.0, 0.0);
  planner.checkCollisionCallback(ros::TimerEvent());
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, ArrivalDoesNotTruncateTrajectoryBeforeItsEnd)
{
  planner.planner_manager_->traj_.local_traj.start_time = 996.0;
  EXPECT_FALSE(planner.handleManualTrajectoryEnd(1000.0));
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, PartialHorizonStillNeedsContinuation)
{
  planner.final_goal_.x() += 4.0;
  EXPECT_FALSE(planner.handleManualTrajectoryEnd(1000.0));
  EXPECT_TRUE(planner.have_target_);
}

TEST_F(PlannerEndpoint, RealDeviationStillReplansFromOdometry)
{
  planner.odom_pos_ = goal - Eigen::Vector3d(0.40, 0.0, 0.0);
  EXPECT_FALSE(planner.handleManualTrajectoryEnd(1000.0));
  // Exercise the real optimizer and publication path, not just the predicate.
  ASSERT_TRUE(planner.planner_manager_->planGlobalTrajWaypoints(
      planner.odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      {goal}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(planner.planFromLocalTraj());
  EXPECT_NEAR((planner.start_pt_ - planner.odom_pos_).norm(), 0.0, 1e-9);
  drain();
  EXPECT_GT(published, 0);
}

TEST_F(PlannerEndpoint, ActualOccupiedPositionRetainsEmergencyPriority)
{
  publishPoint(planner.odom_pos_);
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.checkCollisionCallback(ros::TimerEvent());
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_TRUE(planner.have_target_);
}

TEST_F(PlannerEndpoint, NextGoalStartsNormallyAfterEndpointHover)
{
  planner.checkCollisionCallback(ros::TimerEvent());
  ASSERT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = goal.x() + 1.0;
  message->pose.position.y = goal.y();
  message->pose.position.z = goal.z();
  planner.waypointCallback(message);
  ASSERT_TRUE(planner.have_new_target_);
  EXPECT_FALSE(planner.handleManualTrajectoryEnd(1000.0));
  exec();
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  EXPECT_TRUE(planner.have_target_);
  drain();
  EXPECT_GT(published, 0);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_planner_odometry");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
