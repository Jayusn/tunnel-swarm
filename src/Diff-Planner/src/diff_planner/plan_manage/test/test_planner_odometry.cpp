#include <gtest/gtest.h>
#include <limits>
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

TEST(EntryTwoPiece, ContinuousMidpointAndBoundedSpeedAndAcceleration)
{
  diff_planner::DiffPlannerManager manager;
  manager.two_piece_entry_ = true;
  manager.pp_.max_vel_ = manager.pp_.max_acc_ = 1.0;
  const Eigen::Vector3d start(0.3, -0.2, 0.6);
  const Eigen::Vector3d direction = Eigen::Vector3d(3.0, 4.0, 0.0).normalized();
  for (double distance : {0.0, 0.001, 0.1, 1.66, 10.0})
  {
    const Eigen::Vector3d goal = start + distance * direction;
    poly_traj::MinJerkOpt polynomial;
    ASSERT_TRUE(manager.computeInitState(start, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), goal, Eigen::Vector3d::Zero(), true, false,
        1.5, polynomial));
    const auto trajectory = polynomial.getTraj();
    ASSERT_EQ(trajectory.getPieceNum(), 2);
    const double half = trajectory.getDurations()(0);
    EXPECT_LT((trajectory[0].getPos(half) - trajectory[1].getPos(0.0)).norm(), 1e-8);
    EXPECT_LT((trajectory[0].getVel(half) - trajectory[1].getVel(0.0)).norm(), 1e-8);
    EXPECT_LT((trajectory[0].getAcc(half) - trajectory[1].getAcc(0.0)).norm(), 1e-8);
    EXPECT_LT((trajectory.getJuncPos(1) - 0.5 * (start + goal)).norm(), 1e-8);
    if (distance > 0.0) {
      EXPECT_GT(trajectory.getJuncVel(1).norm(), 0.0);
    }
    const double duration = trajectory.getTotalDuration();
    EXPECT_LT((trajectory.getPos(duration) - goal).norm(), 1e-8);
    EXPECT_LT(trajectory.getVel(duration).norm(), 1e-8);
    EXPECT_LT(trajectory.getAcc(duration).norm(), 1e-8);
    for (int i = 0; i <= 1000; ++i)
    {
      const double t = duration * i / 1000.0;
      EXPECT_LE(trajectory.getVel(t).norm(), 1.0 + 1e-8);
      EXPECT_LE(trajectory.getAcc(t).norm(), 1.0 + 1e-8);
    }
  }
}

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
        [this](const traj_utils::PolyTraj::ConstPtr &message) {
          ++published;
          last_trajectory = *message;
        });
    status_sub = nh.subscribe<std_msgs::String>("planning/status", 20,
        [this](const std_msgs::String::ConstPtr &message) {
          statuses.push_back(message->data);
        });
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while ((odom_pub.getNumSubscribers() == 0 || cloud_pub.getNumSubscribers() == 0 ||
            planner.poly_traj_pub_.getNumSubscribers() == 0 ||
            planner.planning_status_pub_.getNumSubscribers() == 0) &&
           ros::WallTime::now() < deadline)
      ros::WallDuration(0.01).sleep();
    ASSERT_GT(odom_pub.getNumSubscribers(), 0U);
    ASSERT_GT(cloud_pub.getNumSubscribers(), 0U);
    ASSERT_GT(planner.poly_traj_pub_.getNumSubscribers(), 0U);
    ASSERT_GT(planner.planning_status_pub_.getNumSubscribers(), 0U);
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
  ros::Subscriber trajectory_sub, status_sub;
  std::vector<std::string> statuses;
  traj_utils::PolyTraj last_trajectory;
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

TEST_F(PlannerEndpoint, EntryGoalUsesTwoPiecesAndNormalGoalRestoresDefault)
{
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = goal.x();
  message->pose.position.y = goal.y();
  message->pose.position.z = goal.z();
  planner.entryWaypointCallback(message);
  ASSERT_TRUE(planner.planner_manager_->two_piece_entry_);
  EXPECT_EQ(planner.planner_manager_->traj_.global_traj.traj.getPieceNum(), 2);
  planner.waypointCallback(message);
  EXPECT_FALSE(planner.planner_manager_->two_piece_entry_);
  EXPECT_EQ(planner.planner_manager_->traj_.global_traj.traj.getPieceNum(), 1);
}

TEST_F(PlannerEndpoint, EntryPublishesTwoContinuousOptimizedPieces)
{
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = goal.x() + 1.0;
  message->pose.position.y = goal.y();
  message->pose.position.z = goal.z();
  planner.entryWaypointCallback(message);
  exec();
  exec();
  ASSERT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  const auto& trajectory = planner.planner_manager_->traj_.local_traj.traj;
  ASSERT_EQ(trajectory.getPieceNum(), 2);
  const double half = trajectory.getDurations()(0);
  EXPECT_LT((trajectory[0].getPos(half) - trajectory[1].getPos(0.0)).norm(), 1e-8);
  EXPECT_LT((trajectory[0].getVel(half) - trajectory[1].getVel(0.0)).norm(), 1e-8);
  EXPECT_LT((trajectory[0].getAcc(half) - trajectory[1].getAcc(0.0)).norm(), 1e-8);
  drain();
  EXPECT_GT(published, 0);
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

TEST_F(PlannerEndpoint, DisabledGoalModificationRetainsOccupiedTargetAndStops)
{
  publishPoint(goal + Eigen::Vector3d(0.10, 0.0, 0.0));
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(goal), 0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.mondify_final_goal_ = false;
  planner.exec_state_ = FSM::GEN_NEW_TRAJ;

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_NEAR((planner.final_goal_ - goal).norm(), 0.0, 1e-9);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, OccupiedEmergencyKeepsFirstStopWhenPositionBecomesFree)
{
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  const Eigen::Vector3d first_stop = planner.odom_pos_;
  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = true;
  planner.need_hover_stop_ = true;
  planner.occupied_recovery_episode_ = true;
  planner.last_escape_target_search_time_ = ros::Time::now().toSec();

  exec();
  const auto stop_id = planner.planner_manager_->traj_.local_traj.traj_id;
  // Reproduce the recorded drift while the occupancy query is temporarily zero.
  planner.odom_pos_ += Eigen::Vector3d(0.25, 0.08, 0.0);
  planner.odom_vel_ = Eigen::Vector3d(0.30, 0.15, -0.20);
  ros::Time::setNow(ros::Time(1000, 100000000));

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_TRUE(planner.occupied_recovery_episode_);
  EXPECT_TRUE(planner.have_target_);
  EXPECT_TRUE(planner.have_trigger_);
  const auto &local = planner.planner_manager_->traj_.local_traj;
  EXPECT_EQ(local.traj_id, stop_id);
  EXPECT_LT((local.traj.getPos(0.0) - first_stop).norm(), 1e-9);
  EXPECT_LT((local.traj.getPos(local.duration) - first_stop).norm(), 1e-9);
  drain();
  EXPECT_EQ(published, 1);
}

TEST_F(PlannerEndpoint, NonOccupiedEmergencyStillResumesCurrentTarget)
{
  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = false;
  planner.need_hover_stop_ = true;
  planner.occupied_recovery_episode_ = false;

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, LatchedOccupiedEmergencyRetreatsEvenAfterMapClears)
{
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.45, 0.0, 0.0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = false;
  planner.need_hover_stop_ = true;
  planner.occupied_recovery_episode_ = true;
  planner.last_escape_target_search_time_ = 999.0;
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  EXPECT_TRUE(planner.occupied_recovery_from_history_);
  EXPECT_LT((planner.occupied_recovery_target_ - retreat).norm(), 1e-9);
  drain();
  EXPECT_EQ(published, 1);
}

TEST_F(PlannerEndpoint, OccupiedHistoryRetreatPublishesAndReleasesNextRelayGoal)
{
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d retreat = start - Eigen::Vector3d(0.45, 0.0, 0.0);
  publishPoint(start);
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(start), 0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(retreat), 0);

  // 同一条占据起点轨迹，普通发布检查仍必须拒绝。
  Eigen::Matrix3d head, tail;
  head << start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  tail << retreat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  poly_traj::MinJerkOpt polynomial;
  polynomial.reset(head, tail, 1);
  polynomial.generate(Eigen::MatrixXd(3, 0), Eigen::VectorXd::Constant(1, 8.5));
  EXPECT_FALSE(planner.planner_manager_->setLocalTrajFromOpt(polynomial, false));
  EXPECT_EQ(planner.planner_manager_->traj_.local_traj.traj_id, initial_id);

  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = false;
  planner.occupied_recovery_episode_ = true;
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.last_escape_target_search_time_ = 999.0;

  exec();

  ASSERT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_from_history_);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  const auto &trajectory = planner.planner_manager_->traj_.local_traj.traj;
  EXPECT_LT((trajectory.getPos(0.0) - start).norm(), 1e-9);
  EXPECT_LT((trajectory.getPos(trajectory.getTotalDuration()) - retreat).norm(), 1e-9);
  drain();
  EXPECT_EQ(published, 1);

  planner.odom_pos_ = retreat + Eigen::Vector3d(0.03, 0.0, 0.0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_FALSE(planner.have_target_);
}

TEST_F(PlannerEndpoint, RecoveryPublicationUsesConfiguredOccupiedPrefixLimit)
{
  publishPoint(planner.odom_pos_);
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.45, 0.0, 0.0);
  planner.escape_max_occupied_prefix_ = 0.05;
  EXPECT_FALSE(planner.callOccupiedRecovery(retreat));
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, RecordedRecoveryDoesNotFinishAfterFiveMillimeterDrift)
{
  // 14:05:51.647: the recorded query was free, but the 8.55s retreat had only
  // run for 0.118s and the aircraft remained about 46cm from its target.
  planner.odom_pos_ = Eigen::Vector3d(4.097284666, 0.412365316, 0.487896510);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_active_ = planner.occupied_recovery_episode_ = true;
  planner.occupied_recovery_from_history_ = true;
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.occupied_recovery_target_ = Eigen::Vector3d(3.692173513, 0.228377401, 0.6);
  planner.occupied_recovery_deadline_ = ros::Time::now().toSec() + 10.0;
  planner.odom_vel_ = Eigen::Vector3d(-0.20, 0.10, 0.12);

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();

  // Horizontal recovery uses XY arrival; the recorded height lag must not
  // prevent completion once the selected horizontal retreat is achieved.
  planner.odom_pos_ = planner.occupied_recovery_target_ + Eigen::Vector3d(0.03, 0.0, -0.12);
  exec();

  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  EXPECT_FALSE(planner.have_target_);
  EXPECT_FALSE(planner.have_trigger_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, LateralRecoveryReachesTargetBeforeResumingPlanning)
{
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_active_ = planner.occupied_recovery_episode_ = true;
  planner.occupied_recovery_from_history_ = false;
  planner.wait_new_target_after_occupied_recovery_ = false;
  planner.occupied_recovery_target_ = planner.odom_pos_ + Eigen::Vector3d(0.0, 0.45, 0.0);
  planner.occupied_recovery_deadline_ = ros::Time::now().toSec() + 10.0;
  planner.odom_vel_ = Eigen::Vector3d(0.15, 0.20, 0.10);

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  planner.odom_pos_ = planner.occupied_recovery_target_ + Eigen::Vector3d(0.0, 0.03, 0.0);
  exec();

  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, VerticalRecoveryRequiresHeightArrival)
{
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_active_ = planner.occupied_recovery_episode_ = true;
  planner.occupied_recovery_vertical_ = true;
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.occupied_recovery_target_ = planner.odom_pos_ + Eigen::Vector3d(0.0, 0.0, 0.20);
  planner.occupied_recovery_deadline_ = ros::Time::now().toSec() + 10.0;

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  planner.odom_pos_.z() = planner.occupied_recovery_target_.z() - 0.03;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, HistoryRecoveryCannotFinishWhileCurrentPositionIsOccupied)
{
  publishPoint(planner.odom_pos_);
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_active_ = planner.occupied_recovery_episode_ = true;
  planner.occupied_recovery_from_history_ = true;
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.occupied_recovery_target_ = planner.odom_pos_;
  planner.occupied_recovery_deadline_ = ros::Time::now().toSec() + 10.0;
  planner.odom_vel_.setZero();

  for (int i = 0; i < 6; ++i)
    exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
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

TEST_F(PlannerEndpoint, OccupiedEndpointHoverRetreatsWithoutAnotherGoal) {
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  ASSERT_TRUE(planner.handleManualTrajectoryEnd(1000.0));
  ASSERT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  ASSERT_FALSE(planner.have_target_);
  ASSERT_FALSE(planner.have_trigger_);
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.45, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  publishPoint(planner.odom_pos_);

  exec();
  exec();

  ASSERT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  EXPECT_FALSE(planner.have_target_);
  EXPECT_FALSE(planner.have_trigger_);
  EXPECT_EQ(planner.occupied_recovery_goal_stamp_ns_, 999000000000ULL);
  planner.odom_pos_ = retreat;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_active_);
}

TEST_F(PlannerEndpoint, WaitingBeforeFirstExternalGoalDoesNotStartRecovery) {
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.exec_state_ = FSM::WAIT_TARGET;
  planner.have_target_ = planner.have_trigger_ = false;
  planner.active_external_goal_stamp_ns_ = 0ULL;
  publishPoint(planner.odom_pos_);

  exec();
  exec();

  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, MandatoryStopAlsoPreventsWaitingRecovery) {
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.exec_state_ = FSM::WAIT_TARGET;
  planner.have_target_ = planner.have_trigger_ = false;
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  planner.mandatory_stop_ = true;
  publishPoint(planner.odom_pos_);

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, NewGoalCannotRetagCurrentOccupiedRecovery) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.45, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  drain();
  ASSERT_EQ(statuses.size(), 1U);
  const auto execution_stamp = planner.occupied_recovery_stamp_ns_;
  const auto trajectory_id = planner.planner_manager_->traj_.local_traj.traj_id;
  auto next = boost::make_shared<geometry_msgs::PoseStamped>();
  next->header.stamp = ros::Time(1000, 10000000);
  next->pose.position.x = goal.x() + 1.0;
  next->pose.position.y = goal.y();
  next->pose.position.z = goal.z();

  planner.waypointCallback(next);

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_EQ(planner.active_external_goal_stamp_ns_, next->header.stamp.toNSec());
  EXPECT_EQ(planner.planner_manager_->traj_.local_traj.traj_id, trajectory_id);
  planner.odom_pos_ = retreat;
  exec();
  drain();
  ASSERT_EQ(statuses.size(), 2U);
  EXPECT_NE(statuses.back().find("OCCUPIED_RECOVERY_SUCCEEDED goal_stamp_ns=999000000000"),
            std::string::npos);
  EXPECT_NE(statuses.back().find("recovery_stamp_ns=" + std::to_string(execution_stamp)),
            std::string::npos);
}

TEST_F(PlannerEndpoint, ControllerRestartNotifiesRecoveryCancellation) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  drain();
  const auto execution_stamp = planner.occupied_recovery_stamp_ns_;

  planner.planningRestartCallback(std_msgs::Empty());
  drain();

  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  ASSERT_EQ(statuses.size(), 2U);
  EXPECT_NE(statuses.back().find("OCCUPIED_RECOVERY_CANCELLED goal_stamp_ns=999000000000"),
            std::string::npos);
  EXPECT_NE(statuses.back().find("recovery_stamp_ns=" + std::to_string(execution_stamp)),
            std::string::npos);
}

TEST_F(PlannerEndpoint, MandatoryStopNotifiesActiveRecoveryOnlyOnce) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  drain();

  planner.mandatoryStopCallback(std_msgs::Empty());
  planner.mandatoryStopCallback(std_msgs::Empty());
  drain();

  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  ASSERT_EQ(statuses.size(), 2U);
  EXPECT_EQ(statuses.back().find("OCCUPIED_RECOVERY_ABORTED"), 0U);
}

TEST_F(PlannerEndpoint, HistoryGenerationFailureUsesExistingLateralAlternative) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.escape_history_time_ = 0.0;
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  publishPoint(retreat);
  ASSERT_FALSE(planner.callOccupiedRecovery(retreat));
  Eigen::Vector3d alternative;
  double clearance = 0.0;
  ASSERT_TRUE(planner.selectLateralRecoveryTarget(alternative, clearance));

  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));

  EXPECT_TRUE(planner.occupied_recovery_history_failed_);
  EXPECT_FALSE(planner.occupied_recovery_from_history_);
  EXPECT_GT((planner.occupied_recovery_target_ - retreat).norm(), 0.1);
}

TEST_F(PlannerEndpoint, HistoryTimeoutDoesNotSelectTheSameHistoryAgain) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.escape_history_time_ = 0.0;
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  ASSERT_TRUE(planner.occupied_recovery_from_history_);
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_deadline_ = 999.9;

  exec();
  ASSERT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  ros::Time::setNow(ros::Time(1000, 300000000));
  exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_FALSE(planner.occupied_recovery_from_history_);
  EXPECT_GT((planner.occupied_recovery_target_ - retreat).norm(), 0.1);
}

TEST_F(PlannerEndpoint, ClosedHistoryLoopCannotBlockAllAvailableRecovery) {
  planner.escape_max_distance_ = 0.45;
  planner.escape_history_time_ = 0.0;
  const Eigen::Vector3d start = planner.odom_pos_;
  publishPoint(start);
  const Eigen::Vector3d a = start + Eigen::Vector3d(0.15, 0, 0);
  const Eigen::Vector3d b = start + Eigen::Vector3d(0.075, std::sqrt(3.0) * 0.075, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{997, start.x(), start.y(), start.z()});
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{998, b.x(), b.y(), b.z()});
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{999, a.x(), a.y(), a.z()});

  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));

  EXPECT_TRUE(planner.occupied_recovery_active_);
  EXPECT_GT((planner.occupied_recovery_target_ - start).head<2>().norm(),
            planner.escape_reach_tolerance_);
}

TEST_F(PlannerEndpoint, HistoryPolylinePreservesCornerInsteadOfCrossingObstacle) {
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d corner = start + Eigen::Vector3d(0, 0.35, 0);
  const Eigen::Vector3d end = corner + Eigen::Vector3d(0.35, 0, 0);
  // 对角线穿过占据，原来的两条历史线段均自由。
  publishPoint(0.5 * (start + end));
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(start), 0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(corner), 0);
  ASSERT_FALSE(planner.callOccupiedRecovery(end));
  planner.escape_max_distance_ = 0.70;
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{998, end.x(), end.y(), end.z()});
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{999, corner.x(), corner.y(), corner.z()});
  int broadcasts = 0;
  auto broadcast_sub = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10,
      [&broadcasts](const traj_utils::MINCOTraj::ConstPtr &) { ++broadcasts; });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (planner.broadcast_ploytraj_pub_.getNumSubscribers() == 0 &&
         ros::WallTime::now() < deadline)
    ros::WallDuration(0.01).sleep();
  ASSERT_GT(planner.broadcast_ploytraj_pub_.getNumSubscribers(), 0U);

  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));

  const auto &local = planner.planner_manager_->traj_.local_traj;
  const auto &traj = local.traj;
  ASSERT_EQ(traj.getPieceNum(), 2);
  EXPECT_EQ(local.traj_id, initial_id + 1);
  EXPECT_LT((traj.getJuncPos(1) - corner).norm(), 1e-8);
  EXPECT_LT((traj.getPos(local.duration) - end).norm(), 1e-8);
  EXPECT_NEAR(local.duration, 1.875 * 0.70 / planner.escape_speed_, 1e-7);
  for (int piece = 0; piece < 2; ++piece) {
    const double duration = traj[piece].getDuration();
    EXPECT_LT(traj[piece].getVel(0).norm(), 1e-8);
    EXPECT_LT(traj[piece].getVel(duration).norm(), 1e-8);
    EXPECT_LT(traj[piece].getAcc(0).norm(), 1e-8);
    EXPECT_LT(traj[piece].getAcc(duration).norm(), 1e-8);
    for (int i = 0; i <= 200; ++i) {
      const double t = duration * i / 200.0;
      const auto p = traj[piece].getPos(t);
      EXPECT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(p), 0);
      EXPECT_LE(traj[piece].getVel(t).norm(), planner.escape_speed_ + 1e-8);
      if (piece == 0) EXPECT_NEAR(p.x(), start.x(), 1e-8);
      else EXPECT_NEAR(p.y(), corner.y(), 1e-8);
    }
  }
  EXPECT_NEAR(local.start_time + local.duration + 1.0,
              planner.occupied_recovery_deadline_, 1e-8);
  drain();
  EXPECT_EQ(published, 1);
  EXPECT_EQ(broadcasts, 0); // MINCO不能表达中间停转，不能广播一条不同的重建轨迹。
  ASSERT_EQ(last_trajectory.duration.size(), 2U);
  EXPECT_EQ(last_trajectory.traj_id, local.traj_id);
  // 从实际发布的系数重建，确认两个piece没有在序列化时被重新平滑。
  for (int piece = 0; piece < 2; ++piece) {
    poly_traj::CoefficientMat coefficients;
    for (int c = 0; c < 6; ++c)
      coefficients.col(c) << last_trajectory.coef_x[6 * piece + c],
          last_trajectory.coef_y[6 * piece + c], last_trajectory.coef_z[6 * piece + c];
    poly_traj::Piece wire_piece(last_trajectory.duration[piece], coefficients);
    EXPECT_LT((wire_piece.getPos(0.5 * wire_piece.getDuration()) -
               traj[piece].getPos(0.5 * wire_piece.getDuration())).norm(), 1e-6);
  }
}

TEST_F(PlannerEndpoint, HistorySelectionRetainsOnlyPointsUpToChosenEndpoint) {
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d latest = start + Eigen::Vector3d(0, 0.21, 0);
  const Eigen::Vector3d end = latest + Eigen::Vector3d(0.21, 0, 0);
  const Eigen::Vector3d older = end + Eigen::Vector3d(0.18, 0, 0);
  planner.escape_max_distance_ = 0.45;
  planner.free_odom_history_.clear();
  for (const auto &p : {older, end, latest})
    planner.free_odom_history_.push_back(FSM::FreeOdomSample{999, p.x(), p.y(), p.z()});
  planner.odom_pos_.z() = 0.52;
  std::vector<Eigen::Vector3d> path;
  Eigen::Vector3d selected;
  double clearance = 0;

  ASSERT_TRUE(planner.selectHistoryRecoveryTarget(selected, clearance, &path));

  ASSERT_EQ(path.size(), 3U);
  EXPECT_LT((path[0] - planner.odom_pos_).norm(), 1e-9);
  EXPECT_LT((path[1] - latest).norm(), 1e-9);
  EXPECT_LT((path[2] - end).norm(), 1e-9);
  EXPECT_LT((selected - end).norm(), 1e-9);
  planner.occupied_recovery_history_failed_ = true;
  bool from_history = true;
  ASSERT_TRUE(planner.selectOccupiedRecoveryTarget(selected, from_history, clearance, &path));
  EXPECT_FALSE(from_history);
  EXPECT_TRUE(path.empty());
}

TEST_F(PlannerEndpoint, CollinearHistoryMergesWithoutChangingStraightRecovery) {
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d end = start + Eigen::Vector3d(-0.45, 0, 0);
  auto &manager = *planner.planner_manager_;
  ASSERT_TRUE(manager.OccupiedStartRecovery(start, end, 0.10));
  const auto original = manager.traj_.local_traj.traj;

  ASSERT_TRUE(manager.OccupiedHistoryRecovery({start, start,
      start + Eigen::Vector3d(-0.10, 0, 0),
      start + Eigen::Vector3d(-0.30, 0, 0), end, end}, 0.10));

  const auto &traj = manager.traj_.local_traj.traj;
  ASSERT_EQ(traj.getPieceNum(), 1);
  EXPECT_NEAR(traj.getTotalDuration(), original.getTotalDuration(), 1e-9);
  for (int i = 0; i <= 200; ++i) {
    const double t = traj.getTotalDuration() * i / 200.0;
    EXPECT_LT((traj.getPos(t) - original.getPos(t)).norm(), 1e-8);
  }
}

TEST_F(PlannerEndpoint, DenseNearlyStraightHistoryAvoidsRepeatedStops) {
  auto &manager = *planner.planner_manager_;
  manager.pp_.max_acc_ = 0.50;
  const Eigen::Vector3d start = planner.odom_pos_;
  std::vector<Eigen::Vector3d> path{start};
  for (int i = 1; i <= 60; ++i)
    path.push_back(start + Eigen::Vector3d(-0.45 * i / 60.0,
        i == 60 ? 0.0 : (i % 2 ? 0.002 : -0.002), 0.0));

  ASSERT_TRUE(manager.OccupiedHistoryRecovery(path, 0.20));

  const auto &traj = manager.traj_.local_traj.traj;
  ASSERT_EQ(traj.getPieceNum(), 1);
  EXPECT_LT(traj.getTotalDuration(), 5.0);
  EXPECT_LT((traj.getPos(0) - start).norm(), 1e-9);
  EXPECT_LT((traj.getPos(traj.getTotalDuration()) - path.back()).norm(), 1e-9);
  EXPECT_LE(traj.getMaxVelRate(), 0.20 + 1e-8);
  EXPECT_LE(traj.getMaxAccRate(), 0.50 + 1e-8);
}

TEST_F(PlannerEndpoint, CurvedHistorySimplificationBoundsDeviationInBothDirections) {
  auto &manager = *planner.planner_manager_;
  const Eigen::Vector3d start = planner.odom_pos_;
  std::vector<Eigen::Vector3d> path;
  for (int i = 0; i <= 60; ++i) {
    const double angle = std::acos(-1.0) * i / 120.0;
    path.push_back(start + Eigen::Vector3d(-0.30 * std::sin(angle),
        0.30 * (1.0 - std::cos(angle)), 0.0));
  }
  auto distanceToPath = [](const Eigen::Vector3d &p,
                           const std::vector<Eigen::Vector3d> &vertices) {
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < vertices.size(); ++i) {
      const Eigen::Vector3d segment = vertices[i] - vertices[i - 1];
      const double ratio = std::max(0.0, std::min(1.0,
          (p - vertices[i - 1]).dot(segment) / segment.squaredNorm()));
      nearest = std::min(nearest, (p - vertices[i - 1] - ratio * segment).norm());
    }
    return nearest;
  };

  ASSERT_TRUE(manager.OccupiedHistoryRecovery(path, 0.20));

  const auto &traj = manager.traj_.local_traj.traj;
  EXPECT_GT(traj.getPieceNum(), 1); // 不能把弧形通道直接连成起终点直线。
  EXPECT_LT(traj.getPieceNum(), 60);
  std::vector<Eigen::Vector3d> simplified;
  for (int i = 0; i <= traj.getPieceNum(); ++i)
    simplified.push_back(traj.getJuncPos(i));
  for (const auto &p : path)
    EXPECT_LE(distanceToPath(p, simplified), 0.01 + 1e-8);
  for (int i = 0; i <= 1000; ++i)
    EXPECT_LE(distanceToPath(traj.getPos(traj.getTotalDuration() * i / 1000.0), path),
              0.01 + 1e-8);
  EXPECT_LT((simplified.front() - path.front()).norm(), 1e-9);
  EXPECT_LT((simplified.back() - path.back()).norm(), 1e-9);
}

TEST_F(PlannerEndpoint, CollinearBacktrackingInsideEndpointsRetainsTravelOrder) {
  auto &manager = *planner.planner_manager_;
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d first = start + Eigen::Vector3d(-0.30, 0, 0);
  const Eigen::Vector3d back = start + Eigen::Vector3d(-0.10, 0, 0);
  const Eigen::Vector3d end = start + Eigen::Vector3d(-0.45, 0, 0);

  ASSERT_TRUE(manager.OccupiedHistoryRecovery({start, first, back, end}, 0.20));

  const auto &traj = manager.traj_.local_traj.traj;
  ASSERT_EQ(traj.getPieceNum(), 3);
  EXPECT_LT((traj.getJuncPos(1) - first).norm(), 1e-9);
  EXPECT_LT((traj.getJuncPos(2) - back).norm(), 1e-9);
  EXPECT_LT((traj.getJuncPos(3) - end).norm(), 1e-9);
}

TEST_F(PlannerEndpoint, ShortTurnsAndDirectionReversalAreNotMerged) {
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d first = start + Eigen::Vector3d(0.02, 0, 0);
  const Eigen::Vector3d turn = first + Eigen::Vector3d(0, 0.02, 0);
  const Eigen::Vector3d back = turn + Eigen::Vector3d(0, -0.10, 0);
  auto &manager = *planner.planner_manager_;

  ASSERT_TRUE(manager.OccupiedHistoryRecovery({start, first, turn, back}, 0.10));

  const auto &traj = manager.traj_.local_traj.traj;
  ASSERT_EQ(traj.getPieceNum(), 3);
  EXPECT_LT((traj.getJuncPos(1) - first).norm(), 1e-9);
  EXPECT_LT((traj.getJuncPos(2) - turn).norm(), 1e-9);
  for (int i = 0; i < 3; ++i) {
    const double duration = traj[i].getDuration();
    if (i < 2) {
      EXPECT_LT(duration, 1.0);
    }
    for (int j = 0; j <= 100; ++j) {
      EXPECT_LE(traj[i].getVel(duration * j / 100.0).norm(), 0.10 + 1e-8);
      EXPECT_LE(traj[i].getAcc(duration * j / 100.0).norm(), manager.pp_.max_acc_ + 1e-8);
    }
  }
}

TEST_F(PlannerEndpoint, PolylineRecoveryRetainsConfiguredInitialOccupiedLimit) {
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d corner = start - Eigen::Vector3d(0.30, 0, 0);
  const Eigen::Vector3d end = corner + Eigen::Vector3d(0, 0.15, 0);
  publishPoint(start);
  auto &manager = *planner.planner_manager_;

  EXPECT_FALSE(manager.OccupiedHistoryRecovery({start, corner, end}, 0.10, 0.05));
  EXPECT_EQ(manager.traj_.local_traj.traj_id, initial_id);
  ASSERT_TRUE(manager.OccupiedHistoryRecovery({start, corner, end}, 0.10, 0.20));
  EXPECT_EQ(manager.traj_.local_traj.traj.getPieceNum(), 2);
  EXPECT_EQ(manager.traj_.local_traj.traj_id, initial_id + 1);
}

TEST_F(PlannerEndpoint, InvalidHistoryPolylineCannotReplaceCurrentTrajectory) {
  auto &manager = *planner.planner_manager_;
  const Eigen::Vector3d start = planner.odom_pos_;
  Eigen::Vector3d invalid = start;
  invalid.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(manager.OccupiedHistoryRecovery({}, 0.10));
  EXPECT_FALSE(manager.OccupiedHistoryRecovery({start, start}, 0.10));
  EXPECT_FALSE(manager.OccupiedHistoryRecovery(
      {start, start + Eigen::Vector3d(0.002, 0, 0), start}, 0.20));
  EXPECT_FALSE(manager.OccupiedHistoryRecovery({start, invalid}, 0.10));
  EXPECT_FALSE(manager.OccupiedHistoryRecovery({start, start + Eigen::Vector3d(0.2, 0, 0)}, 0));
  EXPECT_EQ(manager.traj_.local_traj.traj_id, initial_id);
}

TEST_F(PlannerEndpoint, HistoryPolylineCannotFinishBeforeReachingItsLastPiece) {
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d first = start + Eigen::Vector3d(0.15, 0, 0);
  const Eigen::Vector3d second = first + Eigen::Vector3d(0, 0.15, 0);
  const Eigen::Vector3d end = second - Eigen::Vector3d(0.15, 0, 0);
  ASSERT_TRUE(planner.planner_manager_->OccupiedHistoryRecovery({start, first, second, end}, 0.10));
  const auto &local = planner.planner_manager_->traj_.local_traj;
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_active_ = planner.occupied_recovery_episode_ = true;
  planner.occupied_recovery_from_history_ = true;
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.occupied_recovery_target_ = end;
  planner.occupied_recovery_deadline_ = local.start_time + local.duration + 1.0;
  planner.odom_pos_ = end;

  exec();

  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  ros::Time::setNow(ros::Time(local.start_time + local.duration));
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  EXPECT_FALSE(planner.occupied_recovery_active_);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_planner_odometry");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
