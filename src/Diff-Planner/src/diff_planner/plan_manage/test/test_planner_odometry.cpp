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
  virtual int routeSegments() const { return 1; }
  virtual double normalAcceleration() const { return 1.0; }
  virtual bool fappMemory() const { return false; }
  virtual bool requireLeaderConfirmation() const { return false; }
  void SetUp() override
  {
    nh.setParam("manager/route_segment_count", routeSegments());
    nh.setParam("manager/fapp_memory_enabled", fappMemory());
    nh.setParam("manager/fapp_require_leader_confirmation", requireLeaderConfirmation());
    nh.setParam("optimization/fapp_continuous_crossing", fappMemory());
    nh.setParam("optimization/fapp_vehicle_top_height", fappMemory() ? 0.05 : 0.125);
    nh.setParam("optimization/fapp_vehicle_bottom_height", fappMemory() ? 0.20 : 0.125);
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
        {"manager/max_vel", 0.5}, {"manager/max_acc", normalAcceleration()},
        {"manager/entry_max_vel", 1.0}, {"manager/entry_max_acc", 1.0},
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
    tracking_sub = nh.subscribe<traj_utils::TrackingReplanResult>("planning/tracking_result", 10,
        [this](const traj_utils::TrackingReplanResult::ConstPtr &message) {
          tracking_results.push_back(*message);
        });
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while ((odom_pub.getNumSubscribers() == 0 || cloud_pub.getNumSubscribers() == 0 ||
            planner.poly_traj_pub_.getNumSubscribers() == 0 ||
            planner.planning_status_pub_.getNumSubscribers() == 0 ||
            planner.tracking_result_pub_.getNumSubscribers() == 0) &&
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
  ros::Subscriber trajectory_sub, status_sub, tracking_sub;
  std::vector<traj_utils::TrackingReplanResult> tracking_results;
  std::vector<std::string> statuses;
  traj_utils::PolyTraj last_trajectory;
  int published{0}, initial_id{0};
};

class RollingPlannerEndpoint : public PlannerEndpoint {
  int routeSegments() const override { return 3; }
};

class GroundHeightRecovery : public PlannerEndpoint {
  void SetUp() override {
    nh.setParam("grid_map/enable_virtual_wall", true);
    nh.setParam("grid_map/virtual_ground", 0.05);
    nh.setParam("grid_map/virtual_ceil", 2.25);
    PlannerEndpoint::SetUp();
    // 15:46:23日志的越界高度和当前接力点高度；XY平移到测试地图中。
    planner.odom_pos_.z() = 0.044;
    planner.final_goal_.z() = 0.169302949;
    planner.escape_reach_tolerance_ = 0.10;
    planner.escape_speed_ = 0.20;
    planner.wait_new_target_after_occupied_recovery_ = true;
    planner.active_external_goal_stamp_ns_ = 999000000000ULL;
    const auto &p = planner.odom_pos_;
    planner.free_odom_history_.push_back(
        FSM::FreeOdomSample{999.0, p.x() - 0.45, p.y(), 0.13});
  }
  void TearDown() override {
    PlannerEndpoint::TearDown();
    nh.deleteParam("grid_map/enable_virtual_wall");
    nh.deleteParam("grid_map/virtual_ground");
    nh.deleteParam("grid_map/virtual_ceil");
  }
};

TEST_F(GroundHeightRecovery, RecordedLowHeightClimbsWithoutHistoryOrSideClearance) {
  const auto start = planner.odom_pos_;
  const Eigen::Vector3d top(start.x(), start.y(), planner.final_goal_.z());
  publishPoint(top + Eigen::Vector3d(0.0, 0.15, 0.0));
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(top), 0);
  ASSERT_LT(planner.estimateInflatedClearance(top), planner.escape_min_clearance_);
  planner.history_only_occupied_recovery_ = true;

  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  EXPECT_TRUE(planner.occupied_recovery_ground_);
  EXPECT_TRUE(planner.occupied_recovery_vertical_);
  EXPECT_FALSE(planner.occupied_recovery_from_history_);
  EXPECT_LT((planner.occupied_recovery_target_ - top).norm(), 1e-9);
  const auto &trajectory = planner.planner_manager_->traj_.local_traj.traj;
  for (int i = 0; i <= 100; ++i) {
    const double t = trajectory.getTotalDuration() * i / 100.0;
    const auto point = trajectory.getPos(t);
    EXPECT_LT((point.head<2>() - start.head<2>()).norm(), 1e-9);
    EXPECT_GE(point.z(), start.z() - 1e-9);
    EXPECT_LE(point.z(), top.z() + 1e-9);
    EXPECT_GE(trajectory.getVel(t).z(), -1e-9);
    EXPECT_LE(trajectory.getVel(t).norm(), 0.20 + 1e-9);
  }
  drain();
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(statuses.back().find("OCCUPIED_RECOVERY_VERTICAL "), 0U);
}

TEST_F(GroundHeightRecovery, HeightArrivalResumesSameRelayWithoutConsumingIt) {
  const auto original_goal = planner.final_goal_;
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.odom_pos_.z() = 0.07;  // 已越过地面线，但不能用10厘米容差过早结束抬升。
  ros::Time::setNow(ros::Time(1000.2));
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  planner.odom_pos_.z() = original_goal.z() + 0.04;  // 上冲也表示已纠正“过低”。
  planner.tracking_replan_pending_ = true;  // 高度已到达时应接续原目标，不生成零长度抬升。
  ros::Time::setNow(ros::Time(1001.2));
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_TRUE(planner.have_target_);
  EXPECT_TRUE(planner.have_trigger_);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  EXPECT_FALSE(planner.occupied_recovery_ground_);
  EXPECT_LT((planner.final_goal_ - original_goal).norm(), 1e-9);
  drain();
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(statuses.back().find("OCCUPIED_RECOVERY_CANCELLED "), 0U);
  for (const auto &status : statuses)
    EXPECT_EQ(status.find("OCCUPIED_RECOVERY_SUCCEEDED"), std::string::npos);
}

TEST_F(GroundHeightRecovery, TrackingReplanContinuesClimbAfterCrossingGround) {
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.odom_pos_.z() = 0.08;
  planner.occupied_recovery_attempt_count_ = planner.escape_max_attempts_;
  planner.occupied_recovery_history_failed_ = true;
  planner.tracking_replan_pending_ = true;
  ros::Time::setNow(ros::Time(1000.2));
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_TRUE(planner.occupied_recovery_active_);
  EXPECT_TRUE(planner.occupied_recovery_vertical_);
  EXPECT_FALSE(planner.occupied_recovery_from_history_);
  EXPECT_NEAR(planner.occupied_recovery_target_.z(), 0.169302949, 1e-9);
}

TEST_F(GroundHeightRecovery, AlreadyRestoredHeightLeavesEmergencyWait) {
  planner.beginOccupiedRecoveryEpisode();
  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = false;
  planner.need_hover_stop_ = true;
  planner.odom_pos_.z() = planner.final_goal_.z();
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  EXPECT_FALSE(planner.occupied_recovery_ground_);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(GroundHeightRecovery, DeepVirtualGroundIsNotLimitedByObstaclePrefixLength) {
  planner.odom_pos_.z() = -0.25;
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  EXPECT_TRUE(planner.occupied_recovery_vertical_);
  EXPECT_NEAR(planner.occupied_recovery_target_.z(), 0.169302949, 1e-9);
}

TEST_F(GroundHeightRecovery, OccupiedAscentDoesNotFallBackToHorizontalRetreat) {
  planner.final_goal_.z() = 0.6;
  publishPoint(planner.odom_pos_ + Eigen::Vector3d(0.0, 0.0, 0.3));
  EXPECT_FALSE(planner.startOccupiedRecovery(1000.0));
  EXPECT_TRUE(planner.occupied_recovery_ground_);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  expectNoReplacement();
}

class FappRollingEndpoint : public PlannerEndpoint {
  int routeSegments() const override { return 3; }
  bool fappMemory() const override { return true; }
};

class FappConfirmedPassage : public FappRollingEndpoint {
 protected:
  bool requireLeaderConfirmation() const override { return true; }
  obj_state_msgs::ObjectsStates localBall(double x = 2.0, double z = 0.8) {
    obj_state_msgs::ObjectsStates msg;
    msg.header.stamp = ros::Time::now(); msg.states.resize(1);
    auto& ball = msg.states[0]; ball.position.x = x; ball.position.z = z;
    ball.size.x = ball.size.y = ball.size.z = 0.15;
    return msg;
  }
  diff_planner::DynamicObstacleState leaderBall(double x = 2.0, double z = 0.8) {
    diff_planner::DynamicObstacleState ball;
    ball.position = {x, 0, z}; ball.size = {0.15,0.15,0.15};
    ball.stamp = ros::Time::now().toSec();
    return ball;
  }
};

TEST_F(FappConfirmedPassage, LocalOnlyStillBlocksCollisionButCannotRelaxSpacing) {
  auto& m = *planner.planner_manager_;
  m.updateDynamicObjects(localBall());
  EXPECT_TRUE(m.dynamicTrajectoryBlocked({{0, {2,0,0.8}}}, 1000));
  EXPECT_FALSE(m.inDynamicPassage({2,0,0.2}, 1000));
  // A later front observation confirms the already observed local region.
  m.updateLeaderDynamicObjects({leaderBall()});
  EXPECT_TRUE(m.inDynamicPassage({2,0,0.2}, 1000));
}

TEST_F(FappConfirmedPassage, EarlierLeaderEvidenceSurvivesEmptyFramesButExpires) {
  auto& m = *planner.planner_manager_;
  auto ball = leaderBall(); ball.stamp = 950;
  m.updateLeaderDynamicObjects({ball});
  EXPECT_FALSE(m.inDynamicPassage({2,0,0.2}, 1000)); // Leader alone is insufficient.
  m.updateDynamicObjects(localBall());
  ASSERT_TRUE(m.inDynamicPassage({2,0,0.2}, 1000));
  m.updateLeaderDynamicObjects({});
  EXPECT_TRUE(m.inDynamicPassage({2,0,0.2}, 1000));
  ros::Time::setNow(ros::Time(1071)); m.updateDynamicObjects(localBall());
  EXPECT_FALSE(m.inDynamicPassage({2,0,0.2}, 1071));
  EXPECT_TRUE(m.dynamicTrajectoryBlocked({{0, {2,0,0.8}}}, 1071));
}

TEST_F(FappConfirmedPassage, OtherHeightOrOnlyInflatedOverlapCannotConfirmRegion) {
  auto& m = *planner.planner_manager_;
  m.updateDynamicObjects(localBall());
  m.updateLeaderDynamicObjects({leaderBall(2,1.4)});
  EXPECT_FALSE(m.inDynamicPassage({2,0,0.2}, 1000));
  // Both padded XY footprints cover this point, but the raw boxes are unrelated.
  m.updateLeaderDynamicObjects({leaderBall(2.5,0.8)});
  EXPECT_FALSE(m.inDynamicPassage({2.25,0,0.2}, 1000));
}

TEST_F(FappConfirmedPassage, LocalExpansionCannotAuthorizeUnobservedPartOfPassage) {
  auto& m = *planner.planner_manager_;
  m.updateLeaderDynamicObjects({leaderBall()});
  m.updateDynamicObjects(localBall());
  m.updateDynamicObjects(localBall(2.5));
  EXPECT_TRUE(m.inDynamicPassage({2,0,0.2}, 1000));
  EXPECT_TRUE(m.dynamicTrajectoryBlocked({{0, {2.65,0,0.8}}}, 1000));
  EXPECT_FALSE(m.inDynamicPassage({2.65,0,0.2}, 1000));
}

TEST_F(FappConfirmedPassage, LeaderBodyInsideConfirmedObstacleDoesNotClearEvidence) {
  auto& m = *planner.planner_manager_;
  m.updateLeaderDynamicObjects({leaderBall()});
  auto msg = localBall();
  auto body = msg.states[0]; body.position.z = 0.5; body.size.z = 0.05;
  msg.states.push_back(body);
  m.updateDynamicObjects(msg, {false,true});
  EXPECT_TRUE(m.inDynamicPassage({2,0,0.2}, 1000));
  m.updateLeaderDynamicObjects({});
  msg.states = {body}; m.updateDynamicObjects(msg, {true});
  EXPECT_TRUE(m.inDynamicPassage({2,0,0.2}, 1000));
}

TEST_F(FappConfirmedPassage, LeaderCallbackTransformsBoxAndRejectsUnknownFrame) {
  auto& m = *planner.planner_manager_;
  auto local = localBall(2.2); local.states[0].position.y = 1;
  local.states[0].size.x = 0.1; local.states[0].size.y = 0.6;
  m.updateDynamicObjects(local);
  auto front = boost::make_shared<obj_state_msgs::ObjectsStates>();
  front->header.frame_id = "leader"; front->header.stamp = ros::Time(1000);
  front->states.resize(1); auto& ball = front->states[0];
  ball.position.x = 1; ball.position.z = 0.8;
  ball.size.x = 0.6; ball.size.y = 0.1; ball.size.z = 0.15;
  planner.fapp_tf_buffer_.reset(new tf2_ros::Buffer);
  planner.fappLeaderObjectsCallback(front);
  EXPECT_FALSE(m.inDynamicPassage({2.2,1,0.2}, 1000));
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "UAV1/camera_init"; transform.child_frame_id = "leader";
  transform.transform.translation.x = 2.2;
  transform.transform.rotation.z = transform.transform.rotation.w = std::sqrt(0.5);
  ASSERT_TRUE(planner.fapp_tf_buffer_->setTransform(transform, "test", true));
  planner.fappLeaderObjectsCallback(front);
  EXPECT_TRUE(m.inDynamicPassage({2.2,1.6,0.2}, 1000)); // Rotated AABB long side is Y.
  EXPECT_FALSE(m.inDynamicPassage({2.8,1,0.2}, 1000));
}

TEST(FappLeaderIdentity, MatchesRecordedBodyBoxesButKeepsNearbyAndMixedObstacles) {
  diff_planner::FappLeaderIdentity identity;
  // Same-time, common-frame observations from the 2026-10-24 bag.
  EXPECT_TRUE(identity.contains({1.314, -0.186, 0.583}, {0.101, 0.333, 0.067},
                               {1.505, -0.123, 0.601}));
  EXPECT_TRUE(identity.contains({5.538, -3.069, 0.412}, {0.109, 0.322, 0.049},
                               {5.462, -3.147, 0.487}));
  EXPECT_FALSE(identity.contains({3.826, -2.624, 0.588}, {0.178, 0.176, 0.121},
                                {2.432, -2.831, 0.505}));
  const Eigen::Vector3d leader(0, 0, 0.5);
  EXPECT_FALSE(identity.contains({0, 0, 0.8}, {0.15, 0.15, 0.15}, leader));
  EXPECT_FALSE(identity.contains({0.3, 0, 0.5}, {0.2, 0.2, 0.2}, leader));
  // Center proximity alone must never erase a merged leader + obstacle box.
  EXPECT_FALSE(identity.contains(leader, {0.8, 0.2, 0.1}, leader));
  EXPECT_FALSE(identity.contains(leader, {0, 0, 0}, leader));
}

TEST(FappLeaderIdentity, UsesDetectionTimeAndRejectsMissingOrStaleOdometry) {
  diff_planner::FappLeaderIdentity identity;
  Eigen::Vector3d position; std::string frame;
  EXPECT_FALSE(identity.positionAt(10, position, frame));
  identity.observe({10.02, {0.02, 0, 0.5}, "leader"});
  identity.observe({9.98, {-0.02, 0, 0.5}, "leader"}); // Out-of-order delivery.
  ASSERT_TRUE(identity.positionAt(10, position, frame));
  EXPECT_LT((position - Eigen::Vector3d(0, 0, 0.5)).norm(), 1e-9);
  EXPECT_EQ(frame, "leader");
  identity.observe({11, {1, 0, 0.5}, "leader"});
  ASSERT_TRUE(identity.positionAt(10, position, frame));
  EXPECT_NEAR(position.x(), 0, 1e-9); // Not the most recent leader position.
  EXPECT_FALSE(identity.positionAt(11.1, position, frame));
  EXPECT_FALSE(identity.positionAt(10.5, position, frame));
  EXPECT_FALSE(identity.positionAt(0, position, frame));
  identity.observe({9.98, {-0.02, 0, 0.5}, "different_frame"});
  EXPECT_FALSE(identity.positionAt(10, position, frame));
}

TEST_F(FappRollingEndpoint, LeaderBodyRetainsCollisionWithoutPassageOrLongHistory) {
  auto& manager = *planner.planner_manager_;
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time(1000); message.states.resize(1);
  auto& body = message.states[0];
  body.position.x = goal.x(); body.position.y = goal.y(); body.position.z = goal.z();
  body.size.x = 0.2; body.size.y = 0.3; body.size.z = 0.05;
  manager.updateDynamicObjects(message, {true});
  EXPECT_FALSE(manager.inDynamicPassage(goal, 1000));
  EXPECT_TRUE(manager.dynamicTrajectoryBlocked({{0, goal}}, 1000));
  message.states.clear(); ros::Time::setNow(ros::Time(1000.2));
  manager.updateDynamicObjects(message);
  EXPECT_TRUE(manager.dynamicTrajectoryBlocked({{0, goal}}, 1000.2));
  EXPECT_FALSE(manager.inDynamicPassage(goal, 1000.2));
  ros::Time::setNow(ros::Time(1001)); manager.updateDynamicObjects(message);
  EXPECT_FALSE(manager.dynamicTrajectoryBlocked({{0, goal}}, 1001));
  // Small sensor-clock lead must retain the original collision-check semantics.
  message.states.resize(1);
  auto& future_body = message.states[0];
  future_body.position.x = goal.x(); future_body.position.y = goal.y(); future_body.position.z = goal.z();
  future_body.size.x = future_body.size.y = future_body.size.z = 0.1;
  message.header.stamp = ros::Time(1001.01);
  manager.updateDynamicObjects(message, {true});
  EXPECT_TRUE(manager.dynamicTrajectoryBlocked({{0, goal}}, 1001));
  EXPECT_FALSE(manager.inDynamicPassage(goal, 1001));
}

TEST_F(FappRollingEndpoint, LeaderInsideRealPassageDoesNotEraseNewOrWatchedObstacle) {
  auto& manager = *planner.planner_manager_;
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time(1000); message.states.resize(1);
  auto& ball = message.states[0];
  ball.position.x = goal.x() - 0.9; ball.position.y = goal.y(); ball.position.z = 0.8;
  ball.size.x = ball.size.y = ball.size.z = 0.15;
  manager.updateDynamicObjects(message);
  Eigen::Vector3d target, velocity; bool final = false;
  ASSERT_TRUE(manager.prepareLocalSegment(goal - Eigen::Vector3d(1.8,0,0),
                                          goal, target, velocity, final));
  ASSERT_TRUE(final); // Real region is now watched through the crossing.
  auto body = ball; body.position.z = 0.5; body.size.z = 0.05;
  message.states.push_back(body);
  manager.updateDynamicObjects(message, {false, true});
  const Eigen::Vector3d below_ball(body.position.x, body.position.y, 0.2);
  EXPECT_TRUE(manager.inDynamicPassage(below_ball, 1000));
  // Leader enters the same XY area later; its observation cannot clear the old region.
  message.states = {body}; message.header.stamp = ros::Time(1020);
  ros::Time::setNow(ros::Time(1020)); manager.updateDynamicObjects(message, {true});
  EXPECT_TRUE(manager.inDynamicPassage(below_ball, 1020));
  EXPECT_TRUE(manager.dynamicTrajectoryBlocked({{0, {below_ball.x(), below_ball.y(), 0.8}}}, 1020));
}

TEST_F(FappRollingEndpoint, LeaderIdentityTransformsSameTimeOdometryAndKeepsUnknownFrames) {
  planner.fapp_distinguish_leader_ = true;
  planner.fapp_tf_buffer_.reset(new tf2_ros::Buffer);
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "follower"; transform.child_frame_id = "leader";
  transform.transform.translation.x = 1.2;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);
  ASSERT_TRUE(planner.fapp_tf_buffer_->setTransform(transform, "test", true));
  auto odom = boost::make_shared<nav_msgs::Odometry>();
  odom->header.frame_id = "leader"; odom->header.stamp = ros::Time(1000);
  odom->pose.pose.position.x = 1; odom->pose.pose.position.z = 0.5;
  planner.fappLeaderOdometryCallback(odom);
  auto message = boost::make_shared<obj_state_msgs::ObjectsStates>();
  message->header.frame_id = "follower"; message->header.stamp = ros::Time(1000);
  message->states.resize(1); auto& body = message->states[0];
  body.position.x = 1.2; body.position.y = 1; body.position.z = 0.5;
  body.size.x = body.size.y = 0.15; body.size.z = 0.05;
  EXPECT_TRUE(planner.fappObservationIsLeader(body, message->header));
  planner.fappObjectsCallback(message);
  EXPECT_FALSE(planner.planner_manager_->inDynamicPassage({1.2,1,0.5}, 1000));
  EXPECT_TRUE(planner.planner_manager_->dynamicTrajectoryBlocked({{0, {1.2,1,0.5}}}, 1000));
  body.header.frame_id = "missing_tf";
  EXPECT_FALSE(planner.fappObservationIsLeader(body, message->header));
  body.header.frame_id.clear(); body.header.stamp = ros::Time(999);
  EXPECT_FALSE(planner.fappObservationIsLeader(body, message->header));
}

TEST(FappHistory, LeaderBodyCannotCreateOrExpandWatchedPassage) {
  diff_planner::FappHistory history;
  diff_planner::DynamicObstacleState body;
  body.position = {1,0,0.5}; body.size = {0.2,0.3,0.05}; body.stamp = 10;
  body.leader_body = true;
  history.update({body}, 10);
  history.watch(body, {0,0,0.5}, {2,0,0.5}, 0.412);
  EXPECT_TRUE(history.regions(10).empty());
  EXPECT_FALSE(history.active());
  auto ball = body; ball.leader_body = false; ball.size = {0.15,0.15,0.15};
  history.update({ball}, 10); history.watch(ball, {0,0,0.5}, {2,0,0.5}, 0.412);
  body.position.x() = 1.4; history.update({body}, 10.1);
  auto regions = history.regions(30);
  ASSERT_EQ(regions.size(), 1U);
  EXPECT_LT((regions[0].position - ball.position).norm(), 1e-9);
  EXPECT_LT((regions[0].size - ball.size).norm(), 1e-9);
}

TEST_F(FappRollingEndpoint, CrossingKeepsReleasedRelayEndpointAndHistoryThroughDropout) {
  auto& m = *planner.planner_manager_;
  const Eigen::Vector3d start = goal - Eigen::Vector3d(1.8,0,0);
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time::now();message.states.resize(1);
  auto& ball = message.states.front();ball.position.x=goal.x()-0.9;
  ball.position.y=goal.y();ball.position.z=0.8;
  ball.size.x=ball.size.y=ball.size.z=0.15;
  m.updateDynamicObjects(message);
  Eigen::Vector3d target,velocity;bool final=false;
  ASSERT_TRUE(m.prepareLocalSegment(start,goal,target,velocity,final));
  EXPECT_TRUE(final); EXPECT_LT((target-goal).norm(),1e-6);
  EXPECT_NEAR(goal.z(),target.z(),1e-6);
  message.states.clear();ros::Time::setNow(ros::Time(1020));m.updateDynamicObjects(message);
  EXPECT_TRUE(m.dynamicTrajectoryBlocked({{0.0,goal-Eigen::Vector3d(0.9,0,0)}},1020));
  EXPECT_FALSE(m.avoidanceTrajectoryBlocked({{0.0,goal-Eigen::Vector3d(0.9,0,0)}},1020));
  m.observeDynamicPosition(goal);m.updateDynamicObjects(message);
  EXPECT_FALSE(m.dynamicTrajectoryBlocked({{0.0,goal-Eigen::Vector3d(0.9,0,0)}},1020));
}

TEST_F(FappRollingEndpoint, LowDynamicPassageKeepsGoalWithoutStartingRetreat)
{
  planner.odom_pos_ = Eigen::Vector3d(4.23084, -0.06418, 0.03674);
  planner.final_goal_ = Eigen::Vector3d(4.84524, -0.09443, 0.10333);
  const auto expected_goal = planner.final_goal_;
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time::now();
  message.states.resize(1);
  auto &ball = message.states.front();
  ball.position.x = 4.268; ball.position.y = -0.136; ball.position.z = 0.598;
  ball.size.x = 0.352; ball.size.y = 1.315; ball.size.z = 0.375;
  auto &manager = *planner.planner_manager_;
  manager.updateDynamicObjects(message);
  // Low altitude is clear in 3D but still below the dynamic passage in XY.
  EXPECT_FALSE(manager.dynamicTrajectoryBlocked({{0.0, planner.odom_pos_}}, 1000));
  ASSERT_TRUE(manager.inDynamicPassage(planner.odom_pos_, 1000));
  EXPECT_FALSE(planner.startOccupiedRecovery(1000));
  EXPECT_FALSE(planner.occupied_recovery_episode_);

  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = false;
  planner.need_hover_stop_ = true;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_LT((planner.final_goal_ - expected_goal).norm(), 1e-9);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  expectNoReplacement();

  message.states.clear();
  ros::Time::setNow(ros::Time(1001));
  manager.updateDynamicObjects(message);
  EXPECT_FALSE(planner.useOccupiedRecovery()); // Empty detection keeps history.
  planner.odom_pos_.x() = 5.1;
  EXPECT_TRUE(planner.useOccupiedRecovery()); // Static recovery outside passage.
  planner.odom_pos_.x() = 4.23;
  ros::Time::setNow(ros::Time(1011));
  manager.updateDynamicObjects(message);
  EXPECT_TRUE(planner.useOccupiedRecovery()); // No indefinite stale suppression.
}

TEST_F(FappRollingEndpoint, PublishesDynamicPassageForLowFlightAndExit) {
  std::vector<bool> received;
  auto sub = nh.subscribe<std_msgs::Bool>("planning/dynamic_passage", 10,
      [&received](const std_msgs::Bool::ConstPtr& msg) { received.push_back(msg->data); });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (planner.dynamic_passage_pub_.getNumSubscribers() == 0 &&
         ros::WallTime::now() < deadline) ros::WallDuration(0.01).sleep();
  ASSERT_GT(planner.dynamic_passage_pub_.getNumSubscribers(), 0U);
  planner.exec_state_ = FSM::WAIT_TARGET;
  planner.have_target_ = false;
  planner.odom_pos_ = Eigen::Vector3d(4.23, -0.06, 0.04);
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time::now();
  message.states.resize(1);
  auto &ball = message.states.front();
  ball.position.x = 4.268; ball.position.y = -0.136; ball.position.z = 0.598;
  ball.size.x = 0.352; ball.size.y = 1.315; ball.size.z = 0.375;
  planner.planner_manager_->updateDynamicObjects(message);
  exec(); drain();
  ASSERT_FALSE(received.empty());
  EXPECT_TRUE(received.back()); // 从球下方通过，也必须通知跟随器降低间距。
  message.states.clear();
  ros::Time::setNow(ros::Time(1001));
  planner.planner_manager_->updateDynamicObjects(message);
  exec(); drain();
  EXPECT_TRUE(received.back()); // 检测空帧不提前退出记忆包络。
  planner.odom_pos_.x() = 5.1;
  exec(); drain();
  EXPECT_FALSE(received.back());
}

TEST_F(FappRollingEndpoint, DynamicPassageCancelsRetreatWithoutDiscardingGoal)
{
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time::now();
  message.states.resize(1);
  auto &ball = message.states.front();
  ball.position.x = planner.odom_pos_.x();
  ball.position.y = planner.odom_pos_.y();
  ball.position.z = 0.8;
  ball.size.x = ball.size.y = ball.size.z = 0.2;
  planner.planner_manager_->updateDynamicObjects(message);
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_active_ = planner.occupied_recovery_episode_ = true;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::GEN_NEW_TRAJ);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_FALSE(planner.occupied_recovery_episode_);
  EXPECT_TRUE(planner.have_target_);
  EXPECT_LT((planner.final_goal_ - goal).norm(), 1e-9);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, LowRelayRecoveryPreservesHeightUntilHigherGoalArrives)
{
  planner.odom_pos_ = Eigen::Vector3d(4.23, -0.06, 0.037);
  planner.final_goal_ = Eigen::Vector3d(4.85, -0.09, 0.103);
  planner.escape_max_distance_ = 0.45;
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{998, 3.78, -0.06, 0.60});
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{999, 4.16, -0.06, 0.08});
  std::vector<Eigen::Vector3d> path;
  Eigen::Vector3d selected;
  double clearance;
  ASSERT_TRUE(planner.selectHistoryRecoveryTarget(selected, clearance, &path));
  ASSERT_EQ(path.size(), 3U);
  EXPECT_NEAR(path[1].z(), 0.08, 1e-9);
  EXPECT_NEAR(selected.z(), 0.103, 1e-9);
  for (const auto &p : path) EXPECT_LE(p.z(), planner.final_goal_.z());
  ASSERT_TRUE(planner.selectVerticalRecoveryTarget(selected, clearance));
  EXPECT_NEAR(selected.z(), 0.103, 1e-9);

  planner.have_target_ = false; // Arrived at a low point, waiting for the next one.
  planner.active_external_goal_stamp_ns_ = 123;
  ASSERT_TRUE(planner.selectHistoryRecoveryTarget(selected, clearance, &path));
  EXPECT_NEAR(selected.z(), 0.103, 1e-9);
  ASSERT_TRUE(planner.selectVerticalRecoveryTarget(selected, clearance));
  EXPECT_NEAR(selected.z(), 0.103, 1e-9);

  // The next higher relay goal, rather than the 0.60 m fallback, permits climbing.
  planner.have_target_ = true;
  planner.final_goal_.z() = 0.45;
  ASSERT_TRUE(planner.selectHistoryRecoveryTarget(selected, clearance, &path));
  EXPECT_NEAR(path[1].z(), 0.08, 1e-9);
  EXPECT_NEAR(selected.z(), 0.45, 1e-9);
  ASSERT_TRUE(planner.selectVerticalRecoveryTarget(selected, clearance));
  EXPECT_NEAR(selected.z(), 0.45, 1e-9);
}

TEST_F(RollingPlannerEndpoint, RollingSegmentsRetainGoalAndNonzeroIntermediateVelocity)
{
  auto& m = *planner.planner_manager_;
  const Eigen::Vector3d start = goal - Eigen::Vector3d(1.8, 0, 0);
  Eigen::Vector3d target, velocity;
  bool final = true;
  ASSERT_TRUE(m.prepareLocalSegment(start, goal, target, velocity, final));
  EXPECT_FALSE(final);
  EXPECT_GT(velocity.norm(), 0.0);
  EXPECT_NEAR((target-start).norm(), 0.60, 0.12);
  const double horizon = (target-start).norm();
  poly_traj::MinJerkOpt seed;
  ASSERT_TRUE(m.computeInitState(start, .1*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero(),
      target, velocity, true, false, 1.0, seed));
  EXPECT_LT((seed.getTraj().getVel(0) - .1*Eigen::Vector3d::UnitX()).norm(), 1e-8);
  EXPECT_LT((seed.getTraj().getVel(seed.getTraj().getTotalDuration()) - velocity).norm(), 1e-8);
  ASSERT_TRUE(m.prepareLocalSegment(goal-Eigen::Vector3d(.30,0,0), goal, target, velocity, final));
  EXPECT_TRUE(final);
  EXPECT_LT((target-goal).norm(), 1e-8);
  EXPECT_LT(velocity.norm(), 1e-8);
  ASSERT_TRUE(m.prepareLocalSegment(start+Eigen::Vector3d(.30,0,0), goal, target, velocity, final));
  EXPECT_FALSE(final);
  EXPECT_NEAR((target-start-Eigen::Vector3d(.30,0,0)).norm(), horizon, .08); // fixed horizon
  m.two_piece_entry_ = true;
  EXPECT_FALSE(m.segmentedPlanning());
}

TEST_F(RollingPlannerEndpoint, RollingSegmentSearchRejectsOccupiedGoal)
{
  auto& m = *planner.planner_manager_;
  publishPoint(goal+Eigen::Vector3d(.10,0,0));
  Eigen::Vector3d target, velocity; bool final;
  EXPECT_FALSE(m.prepareLocalSegment(planner.odom_pos_, goal, target, velocity, final));
  EXPECT_EQ(m.traj_.local_traj.traj_id, initial_id);
}

TEST_F(RollingPlannerEndpoint, RecordedDescendingGoalAdvancesXYZTogether)
{
  auto& m = *planner.planner_manager_;
  const Eigen::Vector3d start(2.28443, -0.0399136, 0.511114);
  const Eigen::Vector3d recorded_goal(3.710437157, -0.081747157, 0.163605022);
  Eigen::Vector3d target, velocity;
  bool final = true;
  ASSERT_TRUE(m.prepareLocalSegment(start, recorded_goal, target, velocity, final));
  EXPECT_FALSE(final);
  const Eigen::Vector3d direction = (recorded_goal - start).normalized();
  EXPECT_LT((target - start - 0.6 * direction).norm(), 1e-8);
  EXPECT_GT(target.z(), 0.36); // 旧栅格导引在第一段已经降到 0.18736。
  EXPECT_LT((velocity.normalized() - direction).norm(), 1e-8);
  planner.start_pt_ = start;
  planner.start_vel_.setZero();
  planner.start_acc_.setZero();
  planner.final_goal_ = recorded_goal;
  ASSERT_TRUE(planner.callReboundReplan(true, false, false));
  const auto& trajectory = m.traj_.local_traj.traj;
  for (int i = 0; i <= 200; ++i) {
    const auto point = trajectory.getPos(trajectory.getTotalDuration() * i / 200.0);
    const double progress = (point.x() - start.x()) / (recorded_goal.x() - start.x());
    const double expected_z = start.z() + progress * (recorded_goal.z() - start.z());
    EXPECT_NEAR(point.z(), expected_z, 0.02);
    EXPECT_GT(point.z(), 0.35);
  }
}

TEST_F(RollingPlannerEndpoint, DescendingGuideChecksBottomObstacleBeyondFirstSegment)
{
  auto& m = *planner.planner_manager_;
  const Eigen::Vector3d start(2.28443, -0.0399136, 0.511114);
  const Eigen::Vector3d end(3.710437157, -0.081747157, 0.163605022);
  const Eigen::Vector3d obstacle = start + 0.72 * (end - start);
  publishPoint(obstacle);
  ASSERT_NE(m.grid_map_->getInflateOccupancy(obstacle), 0);
  Eigen::Vector3d target, velocity;
  bool final;
  ASSERT_TRUE(m.prepareLocalSegment(start, end, target, velocity, final));
  // 后续段的底部占据也必须触发原有绕障搜索，不能只检查眼前 0.6m。
  EXPECT_GT((target - start - 0.6 * (end - start).normalized()).norm(), 0.01);
  EXPECT_EQ(m.grid_map_->getInflateOccupancy(target), 0);
}

TEST_F(RollingPlannerEndpoint, OptimizedSegmentPreservesMovingStartAndDoesNotStopAtBoundary)
{
  auto& m = *planner.planner_manager_;
  planner.start_pt_ = goal - Eigen::Vector3d(1.8, 0, 0);
  planner.start_vel_ = Eigen::Vector3d(.1, 0, 0);
  planner.start_acc_.setZero();
  ASSERT_TRUE(planner.callReboundReplan(true, false, false));
  const auto trajectory = m.traj_.local_traj.traj;
  EXPECT_GT(m.traj_.local_traj.traj_id, initial_id);
  EXPECT_FALSE(planner.touch_goal_);
  EXPECT_LT((trajectory.getPos(0) - planner.start_pt_).norm(), 1e-7);
  EXPECT_LT((trajectory.getVel(0) - planner.start_vel_).norm(), 1e-7);
  EXPECT_GT(trajectory.getVel(trajectory.getTotalDuration()).norm(), .01);
  EXPECT_GT((trajectory.getPos(trajectory.getTotalDuration()) - goal).norm(), .5);
  for (int i = 0; i <= 200; ++i)
    EXPECT_EQ(m.grid_map_->getInflateOccupancy(
        trajectory.getPos(trajectory.getTotalDuration() * i / 200.0)), 0);
  // Replan while moving; continuity must hold through the production FSM path.
  const double switch_time = .6 * trajectory.getTotalDuration();
  const Eigen::Vector3d position = trajectory.getPos(switch_time);
  const Eigen::Vector3d velocity = trajectory.getVel(switch_time);
  const Eigen::Vector3d acceleration = trajectory.getAcc(switch_time);
  const int first_id = m.traj_.local_traj.traj_id;
  ros::Time::setNow(ros::Time(m.traj_.local_traj.start_time + switch_time));
  planner.odom_pos_ = position;
  planner.odom_vel_ = velocity;
  planner.odom_stamp_ = ros::Time::now();
  ASSERT_TRUE(planner.planFromLocalTraj(1, true));
  ASSERT_GT(m.traj_.local_traj.traj_id, first_id);
  const auto& next = m.traj_.local_traj.traj;
  EXPECT_LT((next.getPos(0) - position).norm(), 1e-7);
  EXPECT_LT((next.getVel(0) - velocity).norm(), 1e-7);
  EXPECT_LT((next.getAcc(0) - acceleration).norm(), 1e-7);
  EXPECT_GT(next.getVel(0).norm(), .01);
}

TEST_F(RollingPlannerEndpoint, ShortSegmentOptimizesAroundMappedObstacle)
{
  auto& m = *planner.planner_manager_;
  planner.start_pt_ = goal - Eigen::Vector3d(1.8, 0, 0);
  planner.start_vel_.setZero();
  planner.start_acc_.setZero();
  const Eigen::Vector3d obstacle = planner.start_pt_ + Eigen::Vector3d(.35, 0, 0);
  publishPoint(obstacle);
  ASSERT_NE(m.grid_map_->getInflateOccupancy(obstacle), 0);
  ASSERT_TRUE(planner.callReboundReplan(true, false, false));
  const auto& trajectory = m.traj_.local_traj.traj;
  EXPECT_FALSE(planner.touch_goal_);
  double departure = 0;
  for (int i = 0; i <= 500; ++i) {
    const Eigen::Vector3d p = trajectory.getPos(trajectory.getTotalDuration() * i / 500.0);
    EXPECT_EQ(m.grid_map_->getInflateOccupancy(p), 0);
    departure = std::max(departure, (p - planner.start_pt_).tail<2>().norm());
  }
  EXPECT_GT(departure, .05); // the straight route intersects inflated occupancy
  EXPECT_GT(trajectory.getVel(trajectory.getTotalDuration()).norm(), .01);
}

TEST_F(PlannerEndpoint, CollisionTimerChecksOccupiedEndpointBeforeCompletion)
{
  // 即使当前位置自由且已在到达容差内，也不能跳过终点占据处理。
  planner.mondify_final_goal_ = false;
  publishPoint(goal + Eigen::Vector3d(0.10, 0.0, 0.0));
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(goal), 0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.checkCollisionCallback(ros::TimerEvent());
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, ExpiredEndpointChecksOccupiedGoalInBothPlanningStates)
{
  planner.mondify_final_goal_ = false;
  publishPoint(goal + Eigen::Vector3d(0.10, 0.0, 0.0));
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(goal), 0);
  for (const auto state : {FSM::EXEC_TRAJ, FSM::REPLAN_TRAJ}) {
    for (const double error : {0.08, 0.18}) {
      planner.odom_pos_ = goal - Eigen::Vector3d(error, 0.0, 0.0);
      ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
      planner.exec_state_ = state;
      exec();
      EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
      EXPECT_TRUE(planner.have_target_);
      EXPECT_TRUE(planner.need_hover_stop_);
      expectNoReplacement();
    }
  }
}

class EntrySpeedEndpoint : public PlannerEndpoint {
  double normalAcceleration() const override { return 0.5; }
};

TEST_F(EntrySpeedEndpoint, EntryGoalUsesTwoPiecesAndNormalGoalRestoresDefault)
{
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = goal.x();
  message->pose.position.y = goal.y();
  message->pose.position.z = goal.z();
  planner.entryWaypointCallback(message);
  ASSERT_TRUE(planner.planner_manager_->two_piece_entry_);
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_vel_, 1.0);
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_acc_, 1.0);
  EXPECT_EQ(planner.planner_manager_->traj_.global_traj.traj.getPieceNum(), 2);
  // 门点重复下发时保持独立速度，普通点恢复原有配置。
  planner.entryWaypointCallback(message);
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_vel_, 1.0);
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_acc_, 1.0);
  planner.waypointCallback(message);
  EXPECT_FALSE(planner.planner_manager_->two_piece_entry_);
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_vel_, 0.5);
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_acc_, 0.5);
  EXPECT_EQ(planner.planner_manager_->traj_.global_traj.traj.getPieceNum(), 1);
}

TEST_F(PlannerEndpoint, EntryPublishesTwoContinuousDirectPieces)
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
  EXPECT_DOUBLE_EQ(planner.planner_manager_->pp_.max_vel_, 1.0);
  // 入口直达仍使用独立速度和连续两段轨迹。
  EXPECT_LE(trajectory.getMaxVelRate(), 1.0 + 1e-6);
  EXPECT_LE(trajectory.getMaxAccRate(), 1.0 + 1e-6);
  const double half = trajectory.getDurations()(0);
  EXPECT_LT((trajectory[0].getPos(half) - trajectory[1].getPos(0.0)).norm(), 1e-8);
  EXPECT_LT((trajectory[0].getVel(half) - trajectory[1].getVel(0.0)).norm(), 1e-8);
  EXPECT_LT((trajectory[0].getAcc(half) - trajectory[1].getAcc(0.0)).norm(), 1e-8);
  drain();
  EXPECT_GT(published, 0);
}

TEST_F(PlannerEndpoint, EntryReachesHighSpeedWithoutPeriodicReplacement)
{
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = planner.odom_pos_.x() + 2.87;
  message->pose.position.y = planner.odom_pos_.y();
  message->pose.position.z = planner.odom_pos_.z();
  planner.entryWaypointCallback(message);
  exec(); exec();
  ASSERT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  ASSERT_TRUE(planner.enable_periodic_replan_);
  const auto local = planner.planner_manager_->traj_.local_traj;
  double peak_speed = 0.0;
  // 覆盖本次实飞的2.87m入口及多个1秒周期，实际执行到原轨迹高速段。
  for (double elapsed = 0.05; elapsed < local.duration; elapsed += 0.05) {
    ros::Time::setNow(ros::Time(local.start_time + elapsed));
    planner.odom_pos_ = local.traj.getPos(elapsed);
    planner.odom_vel_ = local.traj.getVel(elapsed);
    exec();
    ASSERT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
    ASSERT_EQ(planner.planner_manager_->traj_.local_traj.traj_id, local.traj_id);
    EXPECT_FALSE(planner.periodic_replan_pending_);
    peak_speed = std::max(peak_speed, planner.odom_vel_.norm());
  }
  EXPECT_GT(peak_speed, 0.9);
  EXPECT_LE(peak_speed, 1.0);
  ros::Time::setNow(ros::Time(local.start_time + local.duration + 0.01));
  planner.odom_pos_ = local.traj.getPos(local.duration);
  planner.odom_vel_.setZero();
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
}

TEST_F(PlannerEndpoint, OrdinaryGoalStillTriggersPeriodicReplanning)
{
  auto& local = planner.planner_manager_->traj_.local_traj;
  local.start_time = ros::Time::now().toSec() - 1.1;
  planner.odom_pos_ = local.traj.getPos(1.1);
  planner.odom_vel_ = local.traj.getVel(1.1);
  ASSERT_FALSE(planner.planner_manager_->two_piece_entry_);
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::REPLAN_TRAJ);
  EXPECT_TRUE(planner.periodic_replan_pending_);
}

TEST_F(PlannerEndpoint, EntryRemainsStraightThroughStaticAndDynamicDetections)
{
  const Eigen::Vector3d start = planner.odom_pos_;
  const Eigen::Vector3d end = start + Eigen::Vector3d(1.5, -0.1, 0.03);
  const Eigen::Vector3d middle = 0.5 * (start + end);
  publishPoint(middle);
  auto& manager = *planner.planner_manager_;
  ASSERT_NE(manager.grid_map_->getInflateOccupancy(middle), 0);
  obj_state_msgs::ObjectsStates balls;
  balls.header.stamp = ros::Time::now(); balls.states.resize(1);
  auto& ball = balls.states.front();
  ball.position.x = middle.x(); ball.position.y = middle.y(); ball.position.z = middle.z();
  ball.size.x = ball.size.y = ball.size.z = 0.3;
  manager.updateDynamicObjects(balls);
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = end.x(); message->pose.position.y = end.y();
  message->pose.position.z = end.z();
  planner.entryWaypointCallback(message);
  exec(); exec();
  ASSERT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  EXPECT_FALSE(planner.useOccupiedRecovery());
  const auto& local = manager.traj_.local_traj;
  const int id = local.traj_id;
  for (int i = 0; i <= 200; ++i) {
    const auto p = local.traj.getPos(local.duration * i / 200.0);
    EXPECT_LT((p - start).cross(end - start).norm(), 1e-8);
    EXPECT_GE(p.z(), start.z() - 1e-8);
    EXPECT_LE(p.z(), end.z() + 1e-8);
  }
  planner.checkCollisionCallback(ros::TimerEvent());
  EXPECT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  EXPECT_EQ(manager.traj_.local_traj.traj_id, id);
}

TEST_F(PlannerEndpoint, EntryIgnoresOccupiedGoalAndNormalGoalRestoresAvoidance)
{
  const Eigen::Vector3d end = goal + Eigen::Vector3d(0.8, 0, 0);
  const Eigen::Vector3d next = end + Eigen::Vector3d(0.5, 0, 0);
  // 在推进模拟时钟前装入两处点云；本夹具不提供深度图输入。
  publishPoint(next);
  publishPoint(end);
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(next), 0);
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(end), 0);
  auto message = boost::make_shared<geometry_msgs::PoseStamped>();
  message->header.stamp = ros::Time::now();
  message->pose.position.x = end.x(); message->pose.position.y = end.y();
  message->pose.position.z = end.z();
  planner.entryWaypointCallback(message);
  exec(); exec();
  ASSERT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  auto request = boost::make_shared<traj_utils::TrackingReplan>();
  request->header.stamp = ros::Time(1000, 100000000);
  request->request_id = 1;
  planner.trackingReplanCallback(request);
  ros::Time::setNow(ros::Time(1000, 200000000));
  exec(); drain();
  ASSERT_EQ(tracking_results.size(), 1U);
  EXPECT_FALSE(planner.tracking_replan_pending_);
  const auto& local = planner.planner_manager_->traj_.local_traj;
  ros::Time::setNow(ros::Time(local.start_time + local.duration + 0.01));
  planner.odom_pos_ = end;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::WAIT_TARGET);
  // 下一普通接力点位于入口之后，避免构造起终点完全重合的零长度全局轨迹。
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(next), 0);
  message->header.stamp = ros::Time::now();
  message->pose.position.x = next.x();
  planner.waypointCallback(message);
  EXPECT_FALSE(planner.planner_manager_->two_piece_entry_);
  EXPECT_TRUE(planner.useOccupiedRecovery());
  exec(); exec();
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
}

TEST_F(PlannerEndpoint, EntryStillHonorsControllerMandatoryStop)
{
  planner.planner_manager_->setEntryMode(true);
  planner.mandatoryStopCallback(std_msgs::Empty());
  EXPECT_TRUE(planner.mandatory_stop_);
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
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

TEST_F(PlannerEndpoint, EndpointConvergenceStillDetectsStuckAfterGrace)
{
  planner.odom_pos_ = goal - Eigen::Vector3d(0.18, 0.0, 0.0);
  planner.enable_stuck_detect_ = true;
  planner.stuck_detect_ignore_until_ = 999.0;
  exec(); // Initialize the existing progress baseline while holding the endpoint.
  ASSERT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  planner.last_target_change_time_ = 1000.0 - planner.TARGET_STUCK_TIME - 1.0;
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_TRUE(planner.need_hover_stop_);
  EXPECT_TRUE(planner.have_target_);
  expectNoReplacement();
}

TEST_F(PlannerEndpoint, EndpointConvergenceKeepsExistingStuckGrace)
{
  planner.odom_pos_ = goal - Eigen::Vector3d(0.18, 0.0, 0.0);
  planner.enable_stuck_detect_ = true;
  planner.stuck_detect_ignore_until_ = 1001.0;
  planner.last_target_change_time_ = 1000.0 - planner.TARGET_STUCK_TIME - 1.0;
  for (int i = 0; i < 3; ++i)
    exec();
  EXPECT_EQ(planner.exec_state_, FSM::EXEC_TRAJ);
  EXPECT_TRUE(planner.have_target_);
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

TEST_F(PlannerEndpoint, TrackingRequestDoesNotBypassOccupiedGoalStop)
{
  publishPoint(goal + Eigen::Vector3d(0.10, 0.0, 0.0));
  ASSERT_NE(planner.planner_manager_->grid_map_->getInflateOccupancy(goal), 0);
  ASSERT_EQ(planner.planner_manager_->grid_map_->getInflateOccupancy(planner.odom_pos_), 0);
  planner.tracking_replan_pending_ = true;
  planner.tracking_request_.header.seq = 3;
  planner.tracking_request_.header.stamp = ros::Time(999.9);
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_FALSE(planner.tracking_replan_pending_); // matched stop result was sent
  EXPECT_TRUE(planner.need_hover_stop_);
  exec();
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP); // no retry of the occupied goal
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

TEST_F(PlannerEndpoint, RecoveryTrackingRequestsReplaceStoppedTrajectoryFromActualPosition) {
  planner.escape_max_attempts_ = 3; // 首次脱障以及两次跟踪请求。
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  const auto original_goal = planner.final_goal_;
  const auto owner = planner.occupied_recovery_goal_stamp_ns_;
  // 同时覆盖软重规划与控制器已停车；无需等待旧轨迹超时。
  for (int i = 1; i <= 2; ++i) {
    const int old_id = planner.planner_manager_->traj_.local_traj.traj_id;
    const auto old_stamp = planner.occupied_recovery_stamp_ns_;
    ros::Time::setNow(ros::Time(1000, i * 100000000));
    planner.odom_pos_.y() += 0.03;
    const auto actual = planner.odom_pos_;
    auto request = boost::make_shared<traj_utils::TrackingReplan>();
    request->header.stamp = ros::Time::now();
    request->request_id = i;
    request->trajectory_id = old_id;
    request->stop_current = i == 2;
    planner.trackingReplanCallback(request);
    ASSERT_TRUE(planner.tracking_replan_pending_);
    ros::Time::setNow(ros::Time::now() + ros::Duration(0.01));
    exec();
    drain();
    EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
    EXPECT_FALSE(planner.tracking_replan_pending_);
    EXPECT_TRUE(planner.occupied_recovery_active_);
    EXPECT_GT(planner.occupied_recovery_stamp_ns_, old_stamp);
    EXPECT_EQ(planner.occupied_recovery_goal_stamp_ns_, owner);
    EXPECT_LT((planner.final_goal_ - original_goal).norm(), 1e-9);
    const auto &local = planner.planner_manager_->traj_.local_traj;
    EXPECT_GT(local.traj_id, old_id);
    EXPECT_LT((local.traj.getPos(0) - actual).norm(), 1e-9);
    EXPECT_GT(planner.occupied_recovery_deadline_, ros::Time::now().toSec());
    ASSERT_EQ(tracking_results.size(), static_cast<std::size_t>(i));
    EXPECT_EQ(tracking_results.back().request_id, request->request_id);
    EXPECT_EQ(tracking_results.back().header.stamp, request->header.stamp);
    EXPECT_EQ(tracking_results.back().trajectory.traj_id, local.traj_id);
    EXPECT_GT(tracking_results.back().trajectory.start_time, request->header.stamp);
  }
}

TEST_F(PlannerEndpoint, WaitingGoalRecoveryStillAcceptsControllerTrackingRequest) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.have_target_ = planner.have_trigger_ = false;
  auto request = boost::make_shared<traj_utils::TrackingReplan>();
  request->header.stamp = ros::Time(1000, 100000000);
  request->request_id = 30;
  request->stop_current = true;
  planner.trackingReplanCallback(request);
  ros::Time::setNow(ros::Time(1000, 200000000));
  ASSERT_TRUE(planner.tracking_replan_pending_);
  exec();
  drain();
  EXPECT_FALSE(planner.tracking_replan_pending_);
  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_FALSE(planner.have_target_);
  ASSERT_EQ(tracking_results.size(), 1U);
  EXPECT_EQ(tracking_results.back().request_id, 30U);
}

TEST_F(PlannerEndpoint, CompletedGoalEmergencyStopAcceptsRequestBeforeRecoveryStarts) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  planner.active_external_goal_stamp_ns_ = 999000000000ULL;
  planner.wait_new_target_after_occupied_recovery_ = true;
  planner.enable_fail_safe_ = true;
  planner.have_target_ = planner.have_trigger_ = false;
  planner.beginOccupiedRecoveryEpisode();
  planner.exec_state_ = FSM::EMERGENCY_STOP;
  planner.flag_escape_emergency_ = false; // 停车轨迹已发出，脱障尚未开始。
  planner.need_hover_stop_ = true;
  auto request = boost::make_shared<traj_utils::TrackingReplan>();
  request->header.stamp = ros::Time(1000, 100000000);
  request->request_id = 3;
  planner.trackingReplanCallback(request);
  ASSERT_TRUE(planner.tracking_replan_pending_);
  ros::Time::setNow(ros::Time(1000, 200000000));
  exec();
  drain();
  EXPECT_EQ(planner.exec_state_, FSM::OCCUPIED_RECOVERY);
  EXPECT_FALSE(planner.tracking_replan_pending_);
  EXPECT_FALSE(planner.have_target_);
  ASSERT_EQ(tracking_results.size(), 1U);
  EXPECT_EQ(tracking_results.back().request_id, 3U);
  EXPECT_EQ(tracking_results.back().header.stamp, request->header.stamp);
  EXPECT_EQ(tracking_results.back().trajectory.traj_id,
            planner.planner_manager_->traj_.local_traj.traj_id);
}

TEST_F(PlannerEndpoint, RecoveryTrackingFailureAcknowledgesStopAndUsesExistingRetry) {
  const Eigen::Vector3d retreat = planner.odom_pos_ - Eigen::Vector3d(0.4, 0, 0);
  planner.free_odom_history_.clear();
  planner.free_odom_history_.push_back(FSM::FreeOdomSample{
      999.0, retreat.x(), retreat.y(), retreat.z()});
  ASSERT_TRUE(planner.startOccupiedRecovery(1000.0));
  planner.exec_state_ = FSM::OCCUPIED_RECOVERY;
  planner.occupied_recovery_attempt_count_ = planner.escape_max_attempts_;
  auto request = boost::make_shared<traj_utils::TrackingReplan>();
  request->header.stamp = ros::Time(1000, 100000000);
  request->request_id = 30;
  request->stop_current = true;
  planner.trackingReplanCallback(request);
  ros::Time::setNow(ros::Time(1000, 200000000));
  exec();
  drain();
  EXPECT_EQ(planner.exec_state_, FSM::EMERGENCY_STOP);
  EXPECT_FALSE(planner.occupied_recovery_active_);
  EXPECT_TRUE(planner.occupied_recovery_episode_);
  EXPECT_FALSE(planner.tracking_replan_pending_);
  ASSERT_EQ(tracking_results.size(), 1U);
  EXPECT_EQ(tracking_results.back().request_id, 30U);
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(statuses.back().find("OCCUPIED_RECOVERY_ABORTED"), 0U);
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

TEST(TrackingRequest, IgnoreOldRequestAndRespectMandatoryStop) {
  diff_planner::DiffReplanFSM planner;
  planner.have_target_ = planner.have_trigger_ = true;
  planner.mandatory_stop_ = false;
  traj_utils::TrackingReplan::Ptr request(new traj_utils::TrackingReplan);
  request->header.stamp = ros::Time(100);
  request->header.seq = 1;
  planner.trackingReplanCallback(request);
  EXPECT_TRUE(planner.tracking_replan_pending_);
  planner.tracking_replan_pending_ = false;
  planner.trackingReplanCallback(request);
  EXPECT_FALSE(planner.tracking_replan_pending_);
  request->header.stamp = ros::Time(101);
  request->header.seq = 2;
  request->stop_current = true;
  planner.trackingReplanCallback(request);
  EXPECT_TRUE(planner.tracking_replan_pending_);
  EXPECT_TRUE(planner.tracking_request_.stop_current);
  planner.tracking_replan_pending_ = false;
  planner.mandatory_stop_ = true;
  request->header.stamp = ros::Time(102);
  planner.trackingReplanCallback(request);
  EXPECT_FALSE(planner.tracking_replan_pending_);
}
