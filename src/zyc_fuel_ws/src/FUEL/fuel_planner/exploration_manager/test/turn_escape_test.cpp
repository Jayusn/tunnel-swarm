#include <cstdlib>
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/task_search_manager.h>
#include <string>
#include <utility>
using std::string;
using std::pair;
#include <exploration_manager/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <plan_env/raycast.h>

namespace fast_planner {
class TurnEscapeTest {
  static void check(bool ok, const char* message) {
    if (!ok) { std::cerr << message << std::endl; std::exit(1); }
  }
  static void initialize(ros::NodeHandle& nh, FastExplorationManager& manager,
                         bool production_weights = false) {
    nh.setParam("sdf_map/resolution", 0.05);
    nh.setParam("sdf_map/map_size_x", 4.0);
    nh.setParam("sdf_map/map_size_y", 4.0);
    nh.setParam("sdf_map/map_size_z", 2.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.05);
    nh.setParam("sdf_map/max_ray_length", 3.0);
    nh.setParam("manager/use_optimization", true);
    nh.setParam("manager/max_vel", 0.4);
    nh.setParam("manager/max_acc", 0.5);
    nh.setParam("manager/control_points_distance", 0.2);
    for (const auto& param : {"ld_smooth", "ld_dist", "ld_feasi", "ld_start", "ld_end",
                              "ld_guide", "ld_waypt", "ld_view", "ld_time"})
      nh.setParam(std::string("optimization/")+param, 1.0);
    nh.setParam("optimization/ld_start", 1000.0);
    nh.setParam("optimization/ld_end", 1000.0);
    nh.setParam("optimization/dist0", 0.2);
    nh.setParam("optimization/max_vel", 0.4);
    nh.setParam("optimization/max_acc", 0.5);
    nh.setParam("optimization/algorithm1", 15);
    nh.setParam("optimization/algorithm2", 11);
    for (int i=1;i<=4;++i) {
      nh.setParam("optimization/max_iteration_num"+std::to_string(i), 200);
      nh.setParam("optimization/max_iteration_time"+std::to_string(i), 0.2);
    }
    if (production_weights) {
      // 绕障采用正式algorithm.xml的优化权重，旧测试保持原单位权重。
      nh.setParam("optimization/ld_smooth", 20.0);
      nh.setParam("optimization/ld_dist", 25.0);
      nh.setParam("optimization/ld_feasi", 2.0);
      nh.setParam("optimization/ld_start", 100.0);
      nh.setParam("optimization/ld_end", 0.5);
      nh.setParam("optimization/ld_guide", 12.0);
      nh.setParam("optimization/dist0", 0.27);
      nh.setParam("optimization/max_iteration_num2", 2000);
    }
    manager.planner_manager_ = std::make_shared<FastPlannerManager>();
    auto& planner = *manager.planner_manager_;
    planner.initPlanModules(nh);
    manager.sdf_map_ = planner.edt_environment_->sdf_map_;
    manager.ep_ = std::make_shared<ExplorationParam>();
    manager.ep_->mission_global_no_return_ = false;
    manager.ep_->relax_time_ = 1.0;
    manager.ed_ = std::make_shared<ExplorationData>();
    manager.task_search_manager_ = std::make_shared<TaskSearchManager>();
    manager.task_search_manager_->initialize(nh);
  }
public:
  static void runPassageCentre() {
    ros::NodeHandle nh("~passage_centre");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    // Raw FREE evidence bounded by UNKNOWN walls. Z is intentionally close
    // to the floor: centre guidance must depend on lateral feasible space.
    for (int y = -14; y <= 14; ++y)
      for (int z = 3; z <= 10; ++z) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ end;
        end.x = 1.9; end.y = y * 0.05 + 0.025; end.z = z * 0.05 + 0.025;
        cloud.push_back(end);
        map.inputPointCloud(cloud, 1, Vector3d(-1.9, end.y, end.z));
        cloud.front().x = -1.9;
        map.inputPointCloud(cloud, 1, Vector3d(1.9, end.y, end.z));
      }
    map.updateESDF3d();
    std::vector<Vector3d> path{{-1.0, -0.35, 0.30}, {0.0, -0.35, 0.30}, {1.0, -0.35, 0.30}};
    ASSERT_TRUE(manager.planner_manager_->isPathSafe(path));
    ASSERT_TRUE(manager.centrePassagePath(path, 3.0));
    EXPECT_TRUE(path.front().isApprox(Vector3d(-1, -0.35, 0.3)));
    EXPECT_TRUE(path.back().isApprox(Vector3d(1, -0.35, 0.3)));
    double max_y = -0.35;
    for (const auto& p : path) {
      EXPECT_DOUBLE_EQ(0.30, p.z());
      max_y = std::max(max_y, p.y());
    }
    EXPECT_GT(max_y, -0.1);
    ASSERT_TRUE(manager.planner_manager_->planExploreTraj(path, Vector3d::Zero(), Vector3d::Zero(), 0));
    ASSERT_TRUE(manager.planner_manager_->isTrajectorySafe());
    auto& trajectory = manager.planner_manager_->local_data_.position_traj_;
    double curve_max_y = -0.35;
    for (int i = 0; i <= 100; ++i)
      curve_max_y = std::max(curve_max_y, trajectory.evaluateDeBoorT(trajectory.getTimeSum()*i/100.0).y());
    EXPECT_GT(curve_max_y, -0.20); // The published spline must actually use the central guide.
    FastExplorationFSM fsm;
    fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(&manager, [](FastExplorationManager*) {});
    fsm.planner_manager_ = manager.planner_manager_;
    fsm.fd_ = std::make_shared<FSMData>();
    fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
    fsm.active_traj_ = manager.planner_manager_->local_data_;
    fsm.active_traj_.start_time_ = ros::Time::now();
    fsm.active_traj_valid_ = true;
    fsm.narrow_corridor_stage_ = true;
    fsm.width_replan_pending_ = true;
    fsm.state_ = PLAN_TRAJ;
    EXPECT_TRUE(fsm.resumeAfterWidthAttempt("test rejected quality alternative"));
    EXPECT_EQ(EXEC_TRAJ, fsm.state_);
    EXPECT_FALSE(fsm.active_traj_braked_);
    fsm.width_replan_pending_ = true;
    fsm.active_traj_braked_ = true;
    EXPECT_FALSE(fsm.resumeAfterWidthAttempt("braked route cannot resume"));
  }

  static void runWidthColumn() {
    ros::NodeHandle nh("~width_column");
    FastExplorationManager manager;
    initialize(nh, manager);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.325, 0.075, 0.425));
    for (int i = 0; i < 8; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), Vector3d(-.5, .075, .425));
    const Vector3d query(.325, .075, .440465);
    EXPECT_FALSE(manager.cameraOccupancyColumn(query, query.z()));
    EXPECT_EQ(SDFMap::OCCUPIED, manager.widthColumnState(query));
    EXPECT_EQ(SDFMap::UNKNOWN, manager.widthColumnState(Vector3d(1, 1, 1)));
  }

  static void runStraightWidth(bool mirror) {
    ros::NodeHandle nh("~straight_width");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 0.1);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    const double side = mirror ? -1.0 : 1.0;
    const Vector3d start(-.4, .40 * side, 1.025);
    // Observe FREE cells from both directions before adding walls/divider.
    for (double y = -1.7; y <= 1.7; y += .05)
      for (double z = .725; z <= 1.325; z += .05) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1.9, y, z));
        map.inputPointCloud(cloud, 1, Vector3d(-1.9, y, z));
        cloud.front().x = -1.9;
        map.inputPointCloud(cloud, 1, Vector3d(1.9, y, z));
      }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (double x = -.3; x <= 1.6; x += .025)
      for (double z = .725; z <= 1.325; z += .025) {
        cloud.push_back(pcl::PointXYZ(x, .80 * side, z));
        cloud.push_back(pcl::PointXYZ(x, -1.4 * side, z));
        if (x >= .4) cloud.push_back(pcl::PointXYZ(x, 0.0, z));
      }
    for (int i = 0; i < 8; ++i) map.inputPointCloud(cloud, cloud.size(), start);
    map.updateESDF3d();
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.ep_->mission_forward_fallback_step_ = .2;
    manager.ep_->mission_forward_fallback_max_ = 1.8;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = start - Vector3d(1, 0, 0);
    manager.mission_workspace_dir_ = Vector3d::UnitX();
    manager.entry_forward_complete_ = true;
    manager.wide_side_bypass_enabled_ = false; // Exercise the generic straight branch.
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    task.exit_detection_enabled_ = false;
    task.cruise_height_ = start.z();
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    task.updateRobotPose(start, 0.0);
    const vector<Vector3d> straight{start, start + Vector3d(1, 0, 0)};
    manager.entry_forward_complete_ = false;
    EXPECT_FALSE(manager.prepareWiderRoute(straight, 0));
    manager.entry_forward_complete_ = true;
    ASSERT_TRUE(manager.planner_manager_->isPathSafe(straight));
    const auto straight_width = manager.evaluateRouteWidth(straight, Vector3d::UnitX());
    ASSERT_TRUE(straight_width.complete);
    Vector3d target; double yaw = 0;
    ASSERT_TRUE(manager.buildMissionForwardFallback(start, 0, target, yaw));
    std::cout << "width recovery: " << target.transpose() << std::endl;
    EXPECT_LT((target.y() - start.y()) * side, -.15);
    // The same physical fixture must offer a quality replan from the executing
    // straight path, and the selected path must retain the original hard checks.
    manager.recovery_side_latched_ = false;
    ASSERT_TRUE(manager.prepareWiderRoute(straight, 0));
    EXPECT_LT((manager.widerRouteTarget().y() - start.y()) * side, -.15);
    EXPECT_TRUE(manager.planner_manager_->isPathSafe(manager.pending_wider_route_));
  }

  static void runHeightReferenceOwnership() {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.task_search_manager_->min_search_height_ = 0.10;
    manager.task_search_manager_->max_search_height_ = 1.90;
    check(std::fabs(manager.horizontalGoalHeight(0.03) - 0.10) < 1e-9,
          "ordinary horizontal goal bypassed the configured minimum height");
    manager.pending_reference_height_ = 0.65;
    check(std::fabs(manager.horizontalGoalHeight(0.55) - 0.55) < 1e-9,
          "unpublished candidate changed the committed height");
    manager.recordPublishedTrajectoryHeight();
    manager.pending_reference_height_ = 0.20;
    check(std::fabs(manager.horizontalGoalHeight(0.50) - 0.65) < 1e-9,
          "unpublished recovery replaced the previous height reference");
    manager.pending_reference_height_ = std::numeric_limits<double>::quiet_NaN();
    manager.recordPublishedTrajectoryHeight();
    check(std::fabs(manager.horizontalGoalHeight(0.45) - 0.65) < 1e-9,
          "turn or escape without a new height erased the previous reference");
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::ADVANCING;
    manager.low_probe_height_ = 0.12;
    check(std::fabs(manager.horizontalGoalHeight(0.08) - 0.12) < 1e-9,
          "active underpass inherited cruise height or accumulated tracking error");
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::IDLE;
    check(std::fabs(manager.horizontalGoalHeight(0.50) - 0.65) < 1e-9,
          "underpass probe overwrote the published reference without publication");
  }

  static void runMovingClimb(int mode, bool height_correction = false) {
    ros::NodeHandle nh("~");
    nh.setParam("mission/task_search/min_height", 0.10);
    nh.setParam("mission/task_search/cruise_height", 0.60);
    nh.setParam("mission/task_search/max_goal_climb", mode == 5 ? 0.06 : 0.30);
    FastExplorationManager manager;
    initialize(nh, manager);
    for (const auto& name : {"min_height", "cruise_height", "max_goal_climb"})
      nh.deleteParam(std::string("mission/task_search/") + name);
    manager.mission_entered_search_region_ = true;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.global_no_return_ = false;
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    const Vector3d start(0.025, 0.025, 0.325);
    const Vector3d end = start + Vector3d(0.6, -0.3, 0.0);
    // 每层双向真实射线建图；未知上方用例只观测当前低高度的三层。
    for (int layer = 5; layer <= (mode == 2 ? 7 : 14); ++layer) {
      const double z = (layer + 0.5) * 0.05;
      for (double x : {-1.525, 1.525}) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (double y = -1.5; y <= 1.5; y += 0.025)
          cloud.push_back(pcl::PointXYZ(x, y, z));
        for (int i = 0; i < 5; ++i)
          map.inputPointCloud(cloud, cloud.size(), Vector3d(-0.85*x, 0.0, z));
      }
    }
    if (mode == 1) {
      pcl::PointCloud<pcl::PointXYZ> roof;
      for (double x : {0.275, 0.325})
        for (double y : {-0.125, -0.075, -0.025})
          roof.push_back(pcl::PointXYZ(x, y, 0.525));
      for (int i = 0; i < 10; ++i)
        map.inputPointCloud(roof, roof.size(), start);
    }
    map.updateESDF3d();
    task.updateRobotPose(start, 0.0);
    if (height_correction) {
      const Vector3d horizontal = end - start;
      const Vector3d correction = horizontal + Vector3d(0, 0, 0.275);
      const double distance = horizontal.head<2>().norm();
      check(manager.isLowProbeCorridorSafe(start, horizontal, distance) &&
            manager.isKnownSafeHorizontalCorridor(start, horizontal, distance),
            "height correction fixture must have a safe horizontal corridor");
      check(manager.isLowProbeCorridorSafe(start, correction, distance) == (mode == 0) &&
            manager.isKnownSafeHorizontalCorridor(start, correction, distance) == (mode == 0),
            "height correction ignored occupied or unknown space above the horizontal route");
      return;
    }
    manager.recovery_side_latched_ = true;
    manager.recovery_side_dir_ = -Vector3d::UnitY();
    manager.ed_->path_next_goal_ = {start, 0.5*(start+end), end};
    manager.ed_->next_goal_ = end;
    check(planner.planExploreTraj(manager.ed_->path_next_goal_, Vector3d::Zero(),
                                  Vector3d::Zero(), 0.0) && planner.isTrajectorySafe(),
          "moving-climb fixture must first have a safe horizontal trajectory");
    LocalTrajData original = planner.local_data_;
    check(manager.hasSafeCruiseContinuation(start, 0.0),
          "clear horizontal route did not defer optional stationary ascent");
    if (mode == 1)
      check(planner.isKnownFreePosition(end + Vector3d(0, 0, 0.275)),
            "roof fixture must leave raised endpoint free to test the swept path");
    if (mode == 2)
      check(map.getOccupancy(Vector3d(end + Vector3d(0, 0, 0.275))) == SDFMap::UNKNOWN,
            "unknown overhead fixture was accidentally observed");
    if (mode == 3) manager.low_probe_phase_ = vertical_detour::LowProbePhase::ADVANCING;
    Vector3d velocity = Vector3d::Zero();
    if (mode == 4) velocity.x() = std::numeric_limits<double>::quiet_NaN();
    const bool raised = manager.tryCruiseHeightRecovery(start, velocity, Vector3d::Zero(), 0.0);
    check(raised == (mode == 0 || mode == 5),
          "moving climb did not match overhead visibility, occupancy or active underpass");
    check(manager.recovery_side_latched_ && manager.recovery_side_dir_.y() < -0.9,
          "height recovery replaced the committed right-hand side");
    if (!raised) {
      check(planner.local_data_.position_traj_.getControlPoint().isApprox(
                original.position_traj_.getControlPoint(), 1e-12) &&
            planner.local_data_.traj_id_ == original.traj_id_ &&
            planner.local_data_.duration_ == original.duration_ &&
            manager.ed_->next_goal_.isApprox(end),
            "rejected climb changed the validated horizontal trajectory");
      return;
    }
    const double rise = mode == 5 ? 0.06 : 0.275;
    check((manager.ed_->next_goal_.head<2>() - end.head<2>()).norm() < 1e-9 &&
          std::fabs(manager.ed_->next_goal_.z() - start.z() - rise) < 1e-6,
          "moving climb changed XY or ignored cruise/per-segment climb limits");
    for (double t = 0.0; t <= planner.local_data_.duration_; t += 0.01) {
      const Vector3d point = planner.local_data_.position_traj_.evaluateDeBoorT(t);
      check(planner.isKnownFreePosition(point), "raised spline left known free body space");
      check(planner.local_data_.velocity_traj_.evaluateDeBoorT(t).cwiseAbs().maxCoeff() <= 0.41 &&
            planner.local_data_.acceleration_traj_.evaluateDeBoorT(t).cwiseAbs().maxCoeff() <= 0.51,
            "raised spline exceeded configured dynamic limits");
    }
    const Vector3d actual_end = planner.local_data_.position_traj_.evaluateDeBoorT(
        planner.local_data_.duration_);
    check(actual_end.x() > start.x()+0.5 && actual_end.z() > start.z()+rise-0.02,
          "generated trajectory did not both advance and regain height");
  }

  static void checkPublishedCorridorExecution(FastExplorationManager& manager) {
    // 使用真正已生成的轨迹，分别经过执行 FSM 和独立安全回调。
    FastExplorationFSM fsm;
    fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(
        &manager, [](FastExplorationManager*) {});
    fsm.planner_manager_ = manager.planner_manager_;
    fsm.fd_ = std::make_shared<FSMData>();
    fsm.fp_ = std::make_shared<FSMParam>();
    fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
    fsm.fp_->replan_thresh1_ = 1.0;
    fsm.fp_->replan_thresh2_ = 0.0;
    fsm.fp_->replan_thresh3_ = 0.0;
    fsm.fp_->tracking_error_grace_time_ = 1000.0;
    fsm.fp_->max_tracking_error_xy_ = 1.0;
    fsm.fp_->max_tracking_error_z_ = 1.0;
    fsm.fp_->emergency_brake_horizon_ = 0.12;
    fsm.classic_ = false;
    fsm.narrow_corridor_stage_ = true;
    fsm.active_traj_valid_ = true;
    fsm.active_traj_ = manager.planner_manager_->local_data_;
    fsm.active_traj_.start_time_ = ros::Time::now() -
        ros::Duration(fsm.active_traj_.duration_ - 0.5);
    fsm.state_ = EXEC_TRAJ;
    fsm.FSMCallback(ros::TimerEvent());
    check(fsm.state_ == EXEC_TRAJ,
          "near-end or periodic replan replaced an unfinished corridor route");

    fsm.fd_->odom_pos_ = fsm.active_traj_.position_traj_.evaluateDeBoorT(
        fsm.active_traj_.duration_ - 0.5);
    fsm.fd_->odom_yaw_ = 0.0;
    auto& task = *manager.task_search_manager_;
    task.curve_heading_active_ = true;
    task.curve_origin_ = fsm.fd_->odom_pos_;
    const double reference_yaw = fsm.active_traj_.yaw_traj_.evaluateDeBoorT(
        fsm.active_traj_.duration_ - 0.5)[0];
    task.curve_forward_ = Eigen::Vector2d(std::cos(reference_yaw + 0.8),
                                        std::sin(reference_yaw + 0.8));
    task.curve_geometry_.center.setZero();
    check(manager.shouldReplanCurveHeading(fsm.fd_->odom_pos_, reference_yaw),
          "fixture did not request a new corridor heading");
    fsm.last_suspended_underpass_check_ = ros::Time::now() + ros::Duration(30.0);
    fsm.safetyCallback(ros::TimerEvent());
    check(fsm.state_ == EXEC_TRAJ && !fsm.active_traj_braked_,
          "heading update replaced a collision-free published corridor route");

    fsm.active_traj_.start_time_ = ros::Time::now() -
        ros::Duration(fsm.active_traj_.duration_ - 0.02);
    fsm.FSMCallback(ros::TimerEvent());
    check(fsm.state_ == PLAN_TRAJ, "completed corridor route never resumed planning");

    // 路线保持不能屏蔽新出现的占据：安全回调必须立刻刹停并交回规划。
    fsm.state_ = EXEC_TRAJ;
    fsm.active_traj_.start_time_ = ros::Time::now();
    fsm.fd_->odom_pos_ = fsm.active_traj_.position_traj_.evaluateDeBoorT(0.0);
    const Vector3d blocked = fsm.active_traj_.position_traj_.evaluateDeBoorT(
        fsm.active_traj_.duration_);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (double x=-0.075; x<=0.075; x+=0.025)
      for (double y=-0.075; y<=0.075; y+=0.025)
        for (double z=-0.075; z<=0.075; z+=0.025)
          cloud.push_back(pcl::PointXYZ(blocked.x()+x, blocked.y()+y, blocked.z()+z));
    for (int i=0; i<8; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), fsm.fd_->odom_pos_);
    manager.sdf_map_->updateESDF3d();
    ros::NodeHandle nh("~");
    fsm.emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("test_route_brake", 1);
    fsm.safetyCallback(ros::TimerEvent());
    check(fsm.active_traj_braked_ && fsm.state_ == PLAN_TRAJ &&
          !fsm.keepExecutingCorridorTrajectory(),
          "committed route ignored a newly observed collision");
  }

  static void runUnderpassClassification(bool pillar, bool known_low_space,
                                         bool missing_low_layer = false,
                                         bool discover_support = false,
                                         bool advancing = false,
                                         int height_limit_case = 0) {
    ros::NodeHandle nh("~");
    nh.setParam("mission/task_search/min_height", 0.10);
    FastExplorationManager manager;
    initialize(nh, manager);
    nh.deleteParam("mission/task_search/min_height");
    manager.mission_entered_search_region_ = true;
    const bool ordinary_probe = height_limit_case == 4;
    if (ordinary_probe) manager.vertical_detour_low_height_ = 0.40;
    const Vector3d pos(0.025, 0.025, 0.625);
    const Vector3d forward(1, 0, 0);
    auto& map = *manager.sdf_map_;

    // 用实际点云射线建立低位自由层，避免直接篡改地图占据缓存。
    if (known_low_space) {
      for (int layer = 4; layer <= 9; ++layer) {
        if (missing_low_layer && layer == 6) continue;
        const double z = (layer + 0.5) * 0.05;
        pcl::PointCloud<pcl::PointXYZ> background;
        for (double y = -1.0; y <= 1.0; y += 0.025)
          background.push_back(pcl::PointXYZ(1.525, y, z));
        for (int repeat = 0; repeat < 5; ++repeat)
          map.inputPointCloud(background, background.size(), Vector3d(pos.x(), pos.y(), z));
      }
    }
    pcl::PointCloud<pcl::PointXYZ> obstacle;
    for (double x = 0.375; x <= 0.525; x += 0.025)
      for (double y = -0.15; y <= 0.20; y += 0.025)
        for (double z = pillar ? 0.225 : 0.525; z <= 1.425; z += 0.025)
          obstacle.push_back(pcl::PointXYZ(x, y, z));
    for (int repeat = 0; repeat < 8; ++repeat)
      map.inputPointCloud(obstacle, obstacle.size(), pos);

    // 原地图的低障碍不满足0.40m普通下探所需的空间，不能退回旧的低位目标。
    const bool expected_suspended = !pillar && known_low_space && !missing_low_layer &&
                                    !ordinary_probe;
    check(manager.hasCurrentHeightForwardObstacle(pos, forward),
          "underpass fixture has no obstacle at flight height");
    double bottom, exit_progress, distance;
    check(manager.detectSuspendedObstacle(pos, forward, bottom, exit_progress, distance) ==
              expected_suspended,
          "pillar, unknown low space or suspended obstacle was misclassified");
    check(manager.hasSuspendedForwardObstacle(pos, forward) == expected_suspended,
          "fallback obstacle classification differs from primary detector");
    if (height_limit_case > 0 && !ordinary_probe) {
      const double requested_height = height_limit_case == 1 ? 0.24
                                      : height_limit_case == 2 ? 0.25 : 0.30;
      manager.underpass_height_tracking_margin_ = bottom -
          manager.underpass_lidar_above_odom_ - manager.underpass_clearance_ -
          requested_height;
      if (height_limit_case == 3) manager.task_search_manager_->min_search_height_ = 0.35;
    }
    check(manager.beginSuspendedUnderpass(pos, 0.0) == expected_suspended,
          "primary underpass trigger ignored obstacle classification");
    if (height_limit_case > 0 && !ordinary_probe) {
      if (height_limit_case == 2) {
        check(manager.suspended_underpass_active_ &&
              std::fabs(manager.low_probe_target_.z() - 0.25) < 1e-9,
              "valid suspended underpass at the 0.25m boundary was rejected");
      } else {
        check(manager.suspended_underpass_blocked_ && !manager.suspended_underpass_active_ &&
              manager.low_probe_phase_ == vertical_detour::LowProbePhase::IDLE,
              "suspended underpass bypassed the 0.25m floor or higher task minimum");
      }
      return;
    }
    if (expected_suspended) {
      check(manager.suspended_underpass_active_ &&
            manager.low_probe_phase_ == vertical_detour::LowProbePhase::DESCENDING,
            "confirmed suspended obstacle did not start descent");
      check(manager.low_probe_target_.z() < pos.z(),
            "confirmed suspended obstacle lost its downward target");
      if (discover_support) {
        const Vector3d lowered(pos.x(), pos.y(), manager.low_probe_height_);
        pcl::PointCloud<pcl::PointXYZ> support;
        for (double x = 0.375; x <= 0.525; x += 0.025)
          for (double y = -0.15; y <= 0.20; y += 0.025)
            for (double z = 0.225; z <= 0.525; z += 0.025)
              support.push_back(pcl::PointXYZ(x, y, z));
        for (int repeat = 0; repeat < 15; ++repeat)
          map.inputPointCloud(support, support.size(), lowered);
        manager.low_probe_phase_ = advancing ? vertical_detour::LowProbePhase::ADVANCING
                                             : vertical_detour::LowProbePhase::VERIFYING;
        Vector3d next = Vector3d::Constant(99.0);
        double yaw;
        bool wait = true;
        check(!manager.handleActiveLowProbe(lowered, next, yaw, wait) && !wait &&
              manager.low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
              !manager.suspended_underpass_active_ && !manager.currentPlanIsVerticalDetour(),
              "newly observed pillar kept the underpass session active");
        check(next == Vector3d::Constant(99.0),
              "supported obstacle released an underpass or side target instead of normal planning");
        return;
      }
      manager.cancelActiveLowProbe("test fallback independently");
    }
    Vector3d next;
    double yaw;
    const bool fallback = manager.buildVerticalDetourFallback(
        pos, 0.0, forward, next, yaw, true);
    check(fallback == expected_suspended,
          "forced fallback descent bypassed confirmed free space below obstacle");
    if (!expected_suspended) {
      check(manager.low_probe_phase_ == vertical_detour::LowProbePhase::IDLE,
            "non-suspended obstacle latched a low probe");
      manager.buildVerticalDetourFallback(pos, 0.0, forward, next, yaw);
      check(manager.low_probe_phase_ == vertical_detour::LowProbePhase::IDLE,
            "generic vertical fallback descended beneath an unconfirmed obstacle");
    }
  }

  static void runDescentEndpointTracking(bool height_error, bool reaches_target) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    const Vector3d anchor(0.0, 0.0, 0.56);
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::DESCENDING;
    manager.low_probe_origin_ = anchor;
    manager.low_probe_target_ = Vector3d(0.0, 0.0, 0.27);
    manager.low_probe_height_ = 0.27;
    manager.low_probe_return_height_ = 0.60;
    manager.low_probe_direction_ = Vector3d::UnitX();
    manager.suspended_underpass_active_ = true;
    manager.vertical_detour_plan_ = true;
    manager.has_last_requested_goal_ = true;
    const Vector3d displaced = height_error ? Vector3d(0.0, 0.0, 0.19)
                                           : Vector3d(-0.082, 0.0, 0.27);
    Vector3d next;
    double yaw;
    bool wait = true;
    // 正在下降不能按总飞行时间判超时；参考到终点后才开始跟踪宽限。
    check(manager.handleActiveLowProbe(displaced, next, yaw, wait) && !wait,
          "descent was abandoned before its reference endpoint");
    manager.completeVerticalDetourReference();
    const ros::Time first_endpoint = manager.low_probe_descent_end_time_;
    manager.completeVerticalDetourReference();
    check(!first_endpoint.isZero() && manager.low_probe_descent_end_time_ == first_endpoint,
          "repeated endpoint notification renewed tracking grace");
    manager.low_probe_descent_end_time_ = ros::Time::now() - ros::Duration(1.9);
    check(manager.handleActiveLowProbe(displaced, next, yaw, wait) &&
          manager.low_probe_phase_ == vertical_detour::LowProbePhase::DESCENDING,
          "short endpoint tracking error cancelled descent prematurely");
    manager.low_probe_descent_end_time_ = ros::Time::now() - ros::Duration(2.1);
    if (reaches_target) {
      check(!manager.handleActiveLowProbe(manager.low_probe_target_, next, yaw, wait) && wait &&
            manager.low_probe_phase_ == vertical_detour::LowProbePhase::VERIFYING,
            "actual arrival at the timeout boundary did not take priority over cancellation");
    } else {
      check(!manager.handleActiveLowProbe(displaced, next, yaw, wait) && !wait &&
            manager.low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
            !manager.suspended_underpass_active_ && !manager.currentPlanIsVerticalDetour() &&
            !manager.has_last_requested_goal_ && manager.low_probe_descent_end_time_.isZero(),
            "persistent descent error retained the stale target or blocked normal planning");
    }
  }

  static void runManualWaypoints(bool blocked) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.manual_path_enabled_ = true;
    manager.task_search_manager_.reset();
    // 不初始化 FrontierFinder：手填路线必须完全独立于前沿生成和任务选点。
    const Vector3d start(0, 0, 1), first(0.5, 0, 1), last(0.5, 0.5, 1);
    const Vector3d zero = Vector3d::Zero();
    check(manager.planExploreMotion(start, zero, zero, zero) == NO_FRONTIER,
          "empty manual list selected an automatic target");
    manager.manual_path_.points = {first, last};
    if (blocked) {
      manager.sdf_map_->setOccupied(Vector3d(0.25, 0, 1));
      check(manager.planExploreMotion(start, zero, zero, zero) == FAIL &&
            manager.manual_path_.next == 0,
            "blocked manual segment changed the target or bypassed collision checks");
      return;
    }
    auto& planner = *manager.planner_manager_;
    const auto planTo = [&](const Vector3d& from, const Vector3d& to) {
      check(manager.planExploreMotion(from, zero, zero, zero) == SUCCEED,
            "manual route failed without frontiers");
      const Vector3d end = planner.local_data_.position_traj_.evaluateDeBoorT(
          planner.local_data_.duration_);
      // 现有样条拟合的终点容差为 0.08m，实际到点仍由里程计的 0.25m 半径判定。
      check((end - to).norm() < 0.08 && manager.ed_->next_goal_ == to,
            "manual trajectory appended or substituted an automatic target");
      check(planner.local_data_.yaw_traj_.getControlPoint().allFinite(),
            "manual route generated invalid yaw for a constant or changing heading");
    };
    planTo(start, first);
    check(manager.planExploreMotion(first, zero, zero, zero) == FAIL &&
          manager.manual_path_.next == 0, "predicted start consumed a waypoint");
    manager.updateMissionOdometry(first, 0.0);
    planTo(first, last);
    manager.updateMissionOdometry(last, 0.0);
    check(manager.planExploreMotion(last, zero, zero, zero) == NO_FRONTIER,
          "finished manual route resumed frontier exploration");
  }

  static void runManualAstar(const std::string& scenario) {
    ros::NodeHandle nh("~manual_astar_" + scenario);
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.10);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    nh.setParam("astar/preferred_clearance", 0.50);
    nh.setParam("astar/clearance_weight", 12.0);
    FastExplorationManager manager;
    initialize(nh, manager, true);
    manager.manual_path_enabled_ = true;
    manager.manual_path_use_astar_ = true;
    manager.task_search_manager_.reset();
    manager.edt_environment_ = manager.planner_manager_->edt_environment_;
    ViewNode::caster_.reset(new RayCaster);
    Eigen::Vector3d origin, size;
    manager.sdf_map_->getRegion(origin, size);
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), origin);
    auto& map = *manager.sdf_map_;
    const Vector3d start(-1.025, 0.025, 1.025);
    const Vector3d target = scenario == "same_voxel"
        ? start + Vector3d(0.04, 0, 0) : Vector3d(1.025, 0.025, 1.025);
    const Vector3d last(1.025, 0.825, 1.025), zero = Vector3d::Zero();
    manager.manual_path_.points = {target, last};
    if (scenario != "unknown") {
      // 用真实射线建立自由空间，不把未观测体素直接标成可走。
      for (double y = -1.725; y < 1.75; y += 0.05)
        for (double z = 0.525; z < 1.55; z += 0.05) {
          pcl::PointCloud<pcl::PointXYZ> cloud;
          cloud.push_back(pcl::PointXYZ(1.9, y, z));
          map.inputPointCloud(cloud, 1, Vector3d(-1.9, y, z));
          cloud.front().x = -1.9;
          map.inputPointCloud(cloud, 1, Vector3d(1.9, y, z));
        }
      // 最后一批点覆盖完整测试盒，避免ESDF只更新最后一条射线的z切片。
      pcl::PointCloud<pcl::PointXYZ> bounds;
      for (double x : {-1.9, 1.9})
        for (double y : {-1.725, 1.725})
          for (double z : {0.525, 1.525}) bounds.push_back(pcl::PointXYZ(x, y, z));
      map.inputPointCloud(bounds, bounds.size(), Vector3d(0, 0, 1));
      if (scenario != "same_voxel") {
        const double width = scenario == "blocked" ? 1.95 : 0.30;
        for (double x = -0.10; x <= 0.10; x += 0.025)
          for (double y = -width; y <= width; y += 0.025)
            for (double z = 0.025; z < 1.975; z += 0.025)
              map.setOccupied(Vector3d(x, y, z));
      }
      map.updateESDF3d();
    }
    auto& planner = *manager.planner_manager_;
    if (scenario == "blocked" || scenario == "unknown") {
      EXPECT_EQ(FAIL, manager.planExploreMotion(start, zero, zero, zero));
      EXPECT_EQ(0u, manager.manual_path_.next);
      EXPECT_TRUE(manager.ed_->next_goal_.isApprox(target));
      return;
    }
    if (scenario == "detour") {
      EXPECT_FALSE(planner.isPathSafe({start, target}));
    }
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(start, zero, zero, zero));
    EXPECT_EQ(0u, manager.manual_path_.next);
    EXPECT_TRUE(manager.ed_->next_goal_.isApprox(target));
    EXPECT_TRUE(manager.ed_->path_next_goal_.front().isApprox(start));
    EXPECT_TRUE(manager.ed_->path_next_goal_.back().isApprox(target));
    EXPECT_TRUE(planner.isPathSafe(manager.ed_->path_next_goal_));
    EXPECT_TRUE(planner.isTrajectorySafe());
    EXPECT_LT((planner.local_data_.position_traj_.evaluateDeBoorT(
                   planner.local_data_.duration_) - target).norm(), 0.08);
    EXPECT_TRUE(planner.local_data_.yaw_traj_.getControlPoint().allFinite());
    EXPECT_EQ(FAIL, manager.planExploreMotion(target, zero, zero, zero));
    EXPECT_EQ(0u, manager.manual_path_.next);  // 预测起点到达不能跳过航点。
    manager.updateMissionOdometry(target, 0.0);
    EXPECT_EQ(1u, manager.manual_path_.next);
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(target, zero, zero, zero));
    EXPECT_TRUE(manager.ed_->next_goal_.isApprox(last));
    manager.updateMissionOdometry(last, 0.0);
    EXPECT_EQ(NO_FRONTIER, manager.planExploreMotion(last, zero, zero, zero));
  }

  static void run(double initial_yaw) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    const double target_yaw = initial_yaw - M_PI/3;
    const Vector3d direction(std::cos(target_yaw), std::sin(target_yaw), 0.0);
    manager.task_search_manager_->commitCorridorTurn(direction);
    // 真实占据图：中心不在膨胀层，机体外圈却接触墙，复现本次飞行的判定组合。
    pcl::PointCloud<pcl::PointXYZ> wall;
    for (double y=-0.5;y<=0.5;y+=0.025)
      for (double z=0.7;z<=1.3;z+=0.025)
        wall.push_back(pcl::PointXYZ(0.8,y,z));
    for (int i=0;i<5;++i)
      manager.sdf_map_->inputPointCloud(wall,wall.size(),Vector3d(0,0,1));
    Vector3d pos;
    bool found=false;
    for (double x=0.5;x<0.8;x+=0.005) {
      pos=Vector3d(x,0,1);
      if (!planner.isPositionInflated(pos) && !planner.isRawPositionSafe(pos) &&
          planner.isControlledEscapePosition(pos)) { found=true; break; }
    }
    check(found,"fixture did not reproduce noninflated footprint contact");
    manager.recovery_odom_history_ = {pos-Vector3d(0,0.1,0), pos};
    const Vector3d zero=Vector3d::Zero();
    const Vector3d yaw(initial_yaw,0,0);
    const auto plan = [&](const Vector3d& point) {
      return manager.buildTurnInPlacePlan(point,zero,zero,yaw,direction);
    };
    // 无可用脱困时不能退化成原地转向，也不能解除转向锁存。
    manager.inflation_history_escape_enabled_=false;
    check(!plan(pos) && !manager.currentPlanIsTurnInPlace() &&
          manager.turnInPlaceAlignmentPending(), "disabled escape allowed unsafe turn");
    manager.inflation_history_escape_enabled_=true;
    check(plan(pos),"contact prevented validated translation escape");
    check(!manager.currentPlanIsTurnInPlace() && manager.turnInPlaceAlignmentPending(),
          "escape was tagged as a turn or lost final yaw");
    check(planner.isTrajectorySafe(0.03,true) &&
          planner.trajectoryClearsInflation(0.8,0.02,true), "escape bypassed release safety");
    const double duration=planner.local_data_.position_traj_.getTimeSum();
    const Vector3d end=planner.local_data_.position_traj_.evaluateDeBoorT(duration);
    check((end-pos).norm()>0.03 && planner.isPositionSafe(end),"escape did not clear contact");
    check(std::abs(planner.local_data_.yaw_traj_.evaluateDeBoorT(duration)[0]-yaw[0])<0.02,
          "escape rotated before clearance");
    manager.turn_in_place_still_confirm_time_=0.0;
    check(plan(end) && manager.currentPlanIsTurnInPlace(),"clear endpoint did not resume turn");
    check(planner.isTrajectorySafe(0.03) && planner.trajectoryClearsInflation(0.8,0.02),
          "resumed turn failed release safety");
    const double turn_duration=planner.local_data_.duration_;
    check((planner.local_data_.position_traj_.evaluateDeBoorT(turn_duration)-end).norm()<1e-6,
          "resumed turn translated");
    check(std::abs(planner.local_data_.yaw_traj_.evaluateDeBoorT(turn_duration)[0]-target_yaw)<0.02,
          "resumed turn lost the original target yaw");
    manager.completeTurnInPlace(end);
    check(!manager.turnInPlaceAlignmentPending(),"completed turn stayed latched");
    std::cout << "turn escape real-map regression passed" << std::endl;
  }

  static void runPendingExitApproach(double exit_yaw, double lateral_offset,
                                     bool block_center) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    manager.edt_environment_ = planner.edt_environment_;
    Eigen::Vector3d origin, size;
    manager.sdf_map_->getRegion(origin, size);
    ViewNode::caster_ = std::make_shared<RayCaster>();
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), origin);
    // 本测试只验证出口接近；不让自由空间边界生成普通探索目标。
    nh.setParam("frontier/cluster_min", 1000000);
    manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
    manager.mission_entered_search_region_ = true;
    const Vector3d forward(std::cos(exit_yaw), std::sin(exit_yaw), 0);
    const Vector3d lateral(-forward.y(), forward.x(), 0);
    const auto point = [&](double x, double y, double z) -> Vector3d {
      return x * forward + y * lateral + Vector3d(0.013, 0.017, z + 0.013);
    };
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (double y=-1.2; y<=1.2; y+=0.025)
      for (double z=0.3; z<=1.7; z+=0.025) {
        const Vector3d p = point(1.5, y, z);
        cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
      }
    for (int i=0; i<5; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), point(-1.3, 0, 1));
    cloud.clear();
    for (double y=-1.2; y<=1.2; y+=0.025)
      for (double z=0.3; z<=1.7; z+=0.025) {
        const Vector3d p = point(-1.5, y, z);
        cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
      }
    for (int i=0; i<5; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), point(1.3, 0, 1));
    manager.sdf_map_->updateESDF3d();
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.exit_detection_enabled_ = true;
    task.exit_auto_crossing_enabled_ = true;
    task.require_stage2_detections_ = false;
    task.exit_candidate_hits_ = 3;
    task.pending_exit_portal_outside_verified_ = true;
    task.pending_exit_outward_direction_ = forward.head<2>();
    task.pending_exit_candidate_ = point(0.4, 0, 1);
    task.pending_exit_portal_center_ = point(0.9, 0, 1);
    task.stable_progress_direction_ = forward.head<2>();
    task.stable_progress_direction_valid_ = true;
    if (block_center) {
      task.failed_goal_radius_ = 0.05;
      task.failed_goals_.push_back({point(0.4, 0, 1), ros::Time::now()});
      task.failed_goals_.push_back({point(0.2, 0, 1), ros::Time::now()});
    }
    const Vector3d start = point(-0.7, lateral_offset, 1);
    // 普通绕柱侧偏好不能抢占出口候选接近。
    manager.recovery_side_latched_ = true;
    manager.recovery_side_origin_ = start;
    manager.recovery_side_dir_ = lateral;
    check(task.exitVerificationActive() && !task.exitTransitActive(),
          "fixture must remain in unconfirmed exit verification");
    check(manager.planExploreMotion(start, Vector3d::Zero(), Vector3d::Zero(),
                                   Vector3d(exit_yaw, 0, 0)) == SUCCEED,
          "pending exit approach failed to generate a real trajectory");
    check(!task.exit_candidate_confirmed_ && !task.exitTransitActive() &&
          !manager.currentPlanIsTurnInPlace(), "pending approach became a confirmed exit or turn");
    const double duration = planner.local_data_.duration_;
    const Vector3d end = planner.local_data_.position_traj_.evaluateDeBoorT(duration);

    check((end-start).head<2>().norm()>0.2 &&
          std::fabs((end-start).dot(lateral))>(block_center ? 0.05 : 0.1),
          "fixture did not generate a translating lateral correction");
    for (int i=0; i<=100; ++i) {
      const double yaw = planner.local_data_.yaw_traj_.evaluateDeBoorT(duration*i/100.0)[0];
      check(std::fabs(std::atan2(std::sin(yaw-exit_yaw), std::cos(yaw-exit_yaw)))<0.02,
            "pending exit lateral correction rotated the nose away from the exit");
    }
  }

  static void runEntryPassage(int open_side, bool height_drift = false) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& task = *manager.task_search_manager_;
    task.min_search_height_ = 0.10;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    task.exit_detection_enabled_ = false;
    const double mirror = open_side < 0 ? -1.0 : 1.0;
    const Vector3d start(0.033, -0.103*mirror, 0.473);
    const double reference_height = start.z() + (height_drift ? 0.05 : 0.0);
    if (height_drift) {
      manager.pending_reference_height_ = reference_height;
      manager.recordPublishedTrajectoryHeight();
    }
    task.updateRobotPose(start, 0.0);
    pcl::PointCloud<pcl::PointXYZ> walls;
    // 采用本次入口的尺寸：左墙+0.60、右墙-0.95，障碍位于前方0.67m，
    // 从右墙延伸到Y=-0.10。镜像用例应自动选择右侧。
    for (double x=-1.7; x<=1.7; x+=0.04)
      for (double z=0.15; z<=1.45; z+=0.04)
        for (double y : {-0.95, 0.60})
          walls.push_back(pcl::PointXYZ(x, mirror*y, z));
    for (double x : {-1.7, 1.7})
      for (double y=-0.95; y<=0.60; y+=0.04)
        for (double z=0.15; z<=1.45; z+=0.04)
          walls.push_back(pcl::PointXYZ(x, mirror*y, z));
    for (const Vector3d& viewpoint : {start, Vector3d(1.3, 0, 0.6)})
      for (int i=0; i<5; ++i)
        manager.sdf_map_->inputPointCloud(walls, walls.size(), viewpoint);
    if (open_side != 0) {
      pcl::PointCloud<pcl::PointXYZ> obstacle;
      const double wall_x = open_side == 2 ? 0.30 : 0.70;
      const double wall_y_end = open_side == 2 ? 0.60 : -0.10;
      for (double y=-0.95; y<=wall_y_end+1e-6; y+=0.025)
        for (double z=0.15; z<=1.45; z+=0.025)
          obstacle.push_back(pcl::PointXYZ(wall_x, mirror*y, z));
      for (int i=0; i<6; ++i)
        manager.sdf_map_->inputPointCloud(obstacle, obstacle.size(), start);
    }
    manager.sdf_map_->updateESDF3d();
    entry_forward::Goal goal;
    const bool selected = manager.selectEntryForwardGoal(start, Vector3d::UnitX(), 0.62, goal);
    if (open_side == 2) {
      check(!selected, "blocked entry generated a segment through the wall");
      return;
    }
    check(selected, "entry fixture has no known-free forward candidate");
    std::cout << "entry fixture side=" << open_side << " advance=" << goal.advance
              << " side=" << goal.side << " passage=" << goal.passage_priority
              << " selected_lane=" << goal.passage_side << std::endl;
    if (open_side == 0) {
      check(std::fabs(goal.side) < 1e-6 && goal.passage_priority == 0,
            "open entry acquired a false obstacle-side preference");
    } else {
      check(goal.passage_priority > 0 && goal.passage_side*open_side > 0.0 &&
            goal.side*open_side > 0.0,
            "entry selected the obstructed side despite a connected passage");
      // 下一轮从已执行短步继续；现有绕行侧仍可用时不能被局部净空改成反侧。
      const Vector3d midway = 0.5 * (start + goal.position);
      task.updateRobotPose(midway, 0.0);
      entry_forward::Goal next;
      const double remaining = 0.62 - 0.5 * goal.advance;
      if (remaining >= 0.20) {
        check(manager.selectEntryForwardGoal(midway, Vector3d::UnitX(), remaining, next),
              "same-side entry could not continue from its selected endpoint");
        check(next.side*open_side >= -1e-6,
              "entry reversed its side while the observed passage remained open");
      }
    }
    check(manager.planner_manager_->planStraightTraj(start, goal.position) &&
          manager.planner_manager_->isTrajectorySafe(),
          "entry passage candidate failed final straight-trajectory footprint checks");
    const double duration = manager.planner_manager_->local_data_.duration_;
    const Vector3d end = manager.planner_manager_->local_data_.position_traj_.evaluateDeBoorT(duration);
    check(end.x()-start.x() >= 0.20-1e-3 && std::fabs(end.z()-reference_height) < 1e-6,
          "entry passage lost actual forward progress or changed height");
    if (height_drift) {
      for (int i = 0; i < 8; ++i) {
        Vector3d measured = start;
        measured.z() = goal.position.z() - 0.04;
        check(manager.selectEntryForwardGoal(measured, Vector3d::UnitX(), 0.62, goal) &&
              std::fabs(goal.position.z() - reference_height) < 1e-9,
              "entry replans accumulated four-centimeter height tracking errors");
      }
    }
  }

  static void runWiderSideRecovery(double wide_side, bool fallback_only,
                                   bool remember_side = true, bool entry = false,
                                   bool expired_descent = false) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    const Vector3d start(0.013, 0.017, 1.013);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // 先双向观测出FREE，再加入柱子和窄侧墙，复现左0.35m、右0.90m的间隙。
    for (double x : {-1.5, 1.5}) {
      cloud.clear();
      for (double y=-1.3; y<=1.3; y+=0.025)
        for (double z=0.25; z<=1.75; z+=0.025)
          cloud.push_back(pcl::PointXYZ(x, y, z));
      for (int i=0; i<5; ++i)
        manager.sdf_map_->inputPointCloud(cloud, cloud.size(),
                                        Vector3d(-0.85*x, 0, 1));
    }
    cloud.clear();
    for (double x=-1.4; x<=1.4; x+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        for (double y : {entry ? -0.85 : -0.45, 1.0})
          cloud.push_back(pcl::PointXYZ(x, wide_side*y, z));
    for (int i=0; i<6; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), start);
    cloud.clear();
    for (double y=-0.10; y<=0.10; y+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        cloud.push_back(pcl::PointXYZ(0.55, y, z));
    for (int i=0; i<6; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), start);
    manager.sdf_map_->updateESDF3d();
    manager.edt_environment_ = planner.edt_environment_;
    Eigen::Vector3d map_origin, size;
    manager.sdf_map_->getRegion(map_origin, size);
    ViewNode::caster_ = std::make_shared<RayCaster>();
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), map_origin);
    nh.setParam("frontier/cluster_min", 1000000);
    manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
    // 动态入口已完成推进：有原绕行侧时先续接，无原侧时才比较宽度。
    manager.mission_entered_search_region_ = fallback_only;
    manager.entry_forward_complete_ = true;
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.ep_->mission_forward_fallback_step_ = 0.20;
    manager.ep_->mission_forward_fallback_max_ = 1.0;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = start - Vector3d(1, 0, 0);
    manager.mission_workspace_dir_ = Vector3d::UnitX();
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.exit_detection_enabled_ = false;
    task.global_no_return_ = false;
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    task.updateRobotPose(start, 0.0);
    manager.vertical_detour_enabled_ = false;
    manager.ground_ascent_enabled_ = false;
    manager.inflation_history_escape_enabled_ = false;
    manager.turn_in_place_enabled_ = false;
    manager.center_gate_enabled_ = false;
    manager.camera_head_sweep_enabled_ = false;
    manager.recovery_side_latched_ = remember_side;
    manager.recovery_side_origin_ = start - Vector3d(1, 0, 0);
    manager.recovery_side_dir_ = Vector3d(0, -wide_side, 0);
    const auto split = manager.findWideSideSplit(start, Vector3d::UnitX());
    check(split.valid && (wide_side > 0 ? split.left_width > split.right_width
                                       : split.right_width > split.left_width),
          "fixture did not provide a wider lane opposite the old detour");
    check(manager.isLowProbeCorridorSafe(start, manager.recovery_side_dir_, 0.20),
          "old narrow-side short step must be feasible to reproduce the priority bug");
    if (entry) {
      entry_forward::Goal goal;
      check(manager.selectEntryForwardGoal(start, Vector3d::UnitX(), 0.62, goal) &&
            goal.passage_side*wide_side > 0.0 && goal.side*wide_side > 0.0,
            "entry preferred the remembered narrower lane over a feasible wider passage");
      return;
    }
    Vector3d next;
    double yaw = 0.10;
    if (expired_descent) {
      manager.low_probe_phase_ = vertical_detour::LowProbePhase::DESCENDING;
      manager.low_probe_origin_ = start + Vector3d(0.082, 0.0, 0.30);
      manager.low_probe_target_ = start + Vector3d(0.082, 0.0, 0.0);
      manager.low_probe_height_ = start.z();
      manager.suspended_underpass_active_ = true;
      manager.low_probe_descent_end_time_ = ros::Time::now() - ros::Duration(2.1);
    }
    if (fallback_only) {
      check(manager.buildMissionForwardFallback(start, yaw, next, yaw),
            "disconnected-frontier recovery failed to find the wider side");
    } else {
      check(manager.planExploreMotion(start, Vector3d::Zero(), Vector3d::Zero(),
                                     Vector3d(yaw, 0, 0)) == SUCCEED,
            "wider-side priority failed to generate a trajectory");
      next = planner.local_data_.position_traj_.evaluateDeBoorT(planner.local_data_.duration_);
      check(planner.isTrajectorySafe(), "wider-side trajectory bypassed footprint checks");
      if (expired_descent)
        check(manager.low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
              !manager.suspended_underpass_active_ && !manager.currentPlanIsVerticalDetour(),
              "expired descent did not release ownership to a new horizontal trajectory");
    }
    std::cout << "wider-side fixture side=" << wide_side << " fallback=" << fallback_only
              << " widths=" << split.left_width << "," << split.right_width
              << " next=" << next.transpose()
              << " remembered=" << manager.recovery_side_dir_.transpose() << std::endl;
    const double expected_side = remember_side ? -wide_side : wide_side;
    check((next-start).y()*expected_side >= 0.10 &&
          std::fabs(next.z()-start.z()) < 0.05 &&
          manager.recovery_side_dir_.y()*expected_side > 0.9,
          "width comparison replaced a feasible remembered side or initial choice ignored width");
    if (!fallback_only && !remember_side) checkPublishedCorridorExecution(manager);
  }

  static void runSideRecovery(double side, bool block_side, bool climb = false,
                              bool height_drift = false) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    if (climb) {
      nh.setParam("mission/task_search/min_height", 0.10);
      nh.setParam("mission/task_search/cruise_height", 0.60);
      nh.setParam("mission/task_search/max_goal_climb", 0.30);
    }
    FastExplorationManager manager;
    initialize(nh, manager);
    if (climb)
      for (const auto& name : {"min_height", "cruise_height", "max_goal_climb"})
        nh.deleteParam(std::string("mission/task_search/") + name);
    auto& planner = *manager.planner_manager_;
    const Vector3d origin(0.013, 0.017, climb ? 0.325 : 1.013);
    // 从两边观测墙面，生成带真实FREE证据的走廊；之后补入挡住直行的立柱。
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (double x : {-1.5, 1.5}) {
      cloud.clear();
      for (double y=-1.3; y<=1.3; y+=0.025)
        for (double z=0.25; z<=1.75; z+=0.025)
          cloud.push_back(pcl::PointXYZ(x, y, z));
      for (int i=0; i<5; ++i)
        manager.sdf_map_->inputPointCloud(cloud, cloud.size(),
                                        Vector3d(-0.85*x, 0, 1));
    }
    if (climb) {
      for (int layer = 4; layer <= 15; ++layer) {
        const double z = (layer + 0.5)*0.05;
        for (double x : {-1.525, 1.525}) {
          cloud.clear();
          for (double y=-1.3; y<=1.3; y+=0.025)
            cloud.push_back(pcl::PointXYZ(x, y, z));
          for (int i=0; i<5; ++i)
            manager.sdf_map_->inputPointCloud(cloud, cloud.size(), Vector3d(-0.85*x, 0, z));
        }
      }
    }
    cloud.clear();
    for (double y=-0.10; y<=0.10; y+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        cloud.push_back(pcl::PointXYZ(climb ? 0.513 : 0.50,
                                             climb ? y+0.013 : y, climb ? z+0.013 : z));
    for (int i=0; i<6; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), origin);
    manager.sdf_map_->updateESDF3d();
    manager.edt_environment_ = planner.edt_environment_;
    Eigen::Vector3d map_origin, size;
    manager.sdf_map_->getRegion(map_origin, size);
    ViewNode::caster_ = std::make_shared<RayCaster>();
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), map_origin);
    nh.setParam("frontier/cluster_min", 1000000);
    manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
    manager.mission_entered_search_region_ = true;
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.ep_->mission_forward_fallback_step_ = 0.20;
    manager.ep_->mission_forward_fallback_max_ = 1.0;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = origin - Vector3d(1, 0, 0);
    manager.mission_workspace_dir_ = Vector3d::UnitX();
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.exit_detection_enabled_ = false;
    task.global_no_return_ = false;
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    manager.vertical_detour_enabled_ = false;
    manager.ground_ascent_enabled_ = climb;
    manager.inflation_history_escape_enabled_ = false;
    manager.turn_in_place_enabled_ = false;
    manager.center_gate_enabled_ = false;
    manager.camera_head_sweep_enabled_ = false;

    const Vector3d original_side_goal = origin + Vector3d(0, side*0.35, 0);
    manager.rememberSideRecovery(origin, 0.0, {origin, original_side_goal,
                                original_side_goal + Vector3d(0.8, 0, 0)});
    check(manager.recovery_side_latched_ && manager.recovery_side_dir_.y()*side > 0.9,
          "safe local detour did not retain its chosen side");
    manager.last_requested_goal_ = original_side_goal;
    manager.has_last_requested_goal_ = true;
    manager.reportTrajectoryCollision();
    check(manager.recovery_side_latched_, "collision feedback erased the detour side");

    Vector3d displaced = origin + Vector3d(0, side*0.22, 0);
    if (height_drift) {
      task.max_search_height_ = 1.90;
      manager.pending_reference_height_ = origin.z();
      manager.recordPublishedTrajectoryHeight();
      displaced.z() -= 0.05;
    }
    task.updateRobotPose(displaced, 0.0);
    check(!manager.isLowProbeCorridorSafe(displaced, Vector3d::UnitX(), 0.40),
          "fixture did not keep the forward footprint blocked after 0.22m side progress");
    if (block_side) {
      cloud.clear();
      for (double x=-0.3; x<=0.3; x+=0.025)
        for (double z=0.25; z<=1.75; z+=0.025)
          cloud.push_back(pcl::PointXYZ(x, displaced.y()+side*0.35, z));
      for (int i=0; i<6; ++i)
        manager.sdf_map_->inputPointCloud(cloud, cloud.size(), displaced);
      manager.sdf_map_->updateESDF3d();
      Vector3d next;
      double yaw = 0.0;
      check(!manager.buildSideRecoveryContinuation(displaced, 0.0, next, yaw) &&
            !manager.recovery_side_latched_, "unavailable same side became a hard lock");
      check(manager.buildMissionForwardFallback(displaced, 0.0, next, yaw),
            "same-side failure prevented the original recovery strategies");
      check(planner.isPositionSafe(next), "fallback selected an unsafe endpoint");
      return;
    }

    check(manager.planExploreMotion(displaced, Vector3d::Zero(), Vector3d::Zero(),
                                   Vector3d(0.10, 0, 0)) == SUCCEED,
          "same-side continuation failed to generate a trajectory");
    check(manager.recovery_side_latched_ && !manager.recovery_side_continuation_pending_,
          "0.22m lateral progress released the side or successful plan stayed pending");
    const double duration = planner.local_data_.duration_;
    const Vector3d end = planner.local_data_.position_traj_.evaluateDeBoorT(duration);
    if (height_drift) {
      // A* 短段可能在到达最终高度前截断；保持的是发布段所继承的目标，
      // 不能把截断处高度或下一次偏低里程计重新提交为目标。
      check(std::fabs(manager.pending_reference_height_ - origin.z()) < 1e-9 &&
            end.z() > displaced.z() + 0.005 && end.z() <= origin.z() + 0.02 &&
            (end - displaced).y()*side >= 0.10 && planner.isTrajectorySafe(),
            "side recovery did not retain and approach the committed height safely");
      manager.recordPublishedTrajectoryHeight();
      for (double error : {0.04, 0.06, 0.03, 0.05}) {
        displaced.z() = origin.z() - error;
        task.updateRobotPose(displaced, 0.0);
        check(manager.planExploreMotion(displaced, Vector3d::Zero(), Vector3d::Zero(),
                                       Vector3d(0.10, 0, 0)) == SUCCEED,
              "height-preserving side replan failed");
        auto& traj = planner.local_data_;
        const double end_z = traj.position_traj_.evaluateDeBoorT(traj.duration_).z();
        check(std::fabs(manager.pending_reference_height_ - origin.z()) < 1e-9 &&
              end_z > displaced.z() + 0.005 && end_z <= origin.z() + 0.02 &&
              planner.isTrajectorySafe(),
              "published side replans ratcheted the target down with measured height");
        manager.recordPublishedTrajectoryHeight();
      }
      return;
    }
    if (climb) {
      std::cout << "moving bypass end=" << end.transpose()
                << " selected=" << manager.ed_->next_goal_.transpose()
                << " path_end=" << manager.ed_->path_next_goal_.back().transpose()
                << " phase=" << static_cast<int>(manager.low_probe_phase_)
                << " entered=" << manager.mission_entered_search_region_ << std::endl;
    }
    // 宽侧规划走A*，既有短路径截断会让首段约横移0.14m，仍须保持所选方向。
    check((end-displaced).y()*side >= 0.10 &&
          (climb ? end.z() > displaced.z()+0.05 : std::fabs(end.z()-displaced.z()) < 0.05),
          "replan switched sides or changed the recovery height");
    check(planner.isTrajectorySafe(), "same-side spline bypassed footprint safety");
    Vector3d next;
    double yaw = 0.0;
    check(manager.isLowProbeCorridorSafe(end, Vector3d::UnitX(), 0.40) &&
          manager.buildSideRecoveryContinuation(end, 0.0, next, yaw) &&
          next.x() > end.x()+0.15 && std::fabs(next.y()-end.y()) <= 0.24 + 1e-6 &&
          manager.isLowProbeCorridorSafe(end, next-end, (next-end).head<2>().norm()),
          "clear same-side lane did not resume forward motion");
    Vector3d passed = end;
    passed.x() = manager.recovery_side_origin_.x() +
                 (manager.wide_side_bypass_active_ ? manager.wide_side_bypass_release_distance_
                                                   : manager.recovery_side_release_distance_) + 0.05;
    // 模拟通过柱子后从当前位置补扫前方，不能用尚未观测的远端 FREE 作断言。
    cloud.clear();
    for (double y=-1.3; y<=1.3; y+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        cloud.push_back(pcl::PointXYZ(1.5, y, z));
    for (int i=0; i<5; ++i)
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), passed);
    manager.sdf_map_->updateESDF3d();
    task.updateRobotPose(passed, 0.0);
    check(!manager.buildSideRecoveryContinuation(passed, 0.0, next, yaw) &&
          !manager.recovery_side_latched_, "passing the obstacle did not release the side");
    manager.recovery_side_latched_ = true;
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::VERIFYING;
    check(!manager.buildSideRecoveryContinuation(displaced, 0.0, next, yaw) &&
          manager.recovery_side_latched_, "side preference took over an active vertical probe");
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::IDLE;
    manager.completeTurnInPlace(displaced);
    check(!manager.recovery_side_latched_, "confirmed turn kept the old obstacle side");
  }

  static void runBypassContinuation(double side, bool change_width, bool block_side,
                                    bool height_drift = false) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.entry_forward_complete_ = true;
    manager.ep_->mission_use_forward_fallback_ = true;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.global_no_return_ = false;
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    Vector3d origin(0.013, 0.017, 1.013);
    const double reference_height = origin.z();
    if (height_drift) {
      task.max_search_height_ = 1.90;
      manager.pending_reference_height_ = reference_height;
      manager.recordPublishedTrajectoryHeight();
    }
    task.updateRobotPose(origin, 0.0);
    auto& map = *manager.sdf_map_;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const auto observe = [&](const Vector3d& camera, int repeats) {
      for (int i = 0; i < repeats; ++i) map.inputPointCloud(cloud, cloud.size(), camera);
    };
    for (double x : {-1.5, 1.5}) {
      cloud.clear();
      for (double y=-1.3; y<=1.3; y+=0.025)
        for (double z=0.25; z<=1.75; z+=0.025)
          cloud.push_back(pcl::PointXYZ(x, y, z));
      observe(Vector3d(-0.85*x, 0, 1), 5);
    }
    cloud.clear();
    for (double x=-1.4; x<=1.4; x+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        for (double y : {-0.45, 1.0})
          cloud.push_back(pcl::PointXYZ(x, side*y, z));
    observe(origin, 6);
    cloud.clear();
    for (double y=-0.10; y<=0.10; y+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        cloud.push_back(pcl::PointXYZ(0.55, y, z));
    observe(origin, 6);
    map.updateESDF3d();
    Vector3d map_origin, map_size;
    map.getRegion(map_origin, map_size);
    ViewNode::caster_ = std::make_shared<RayCaster>();
    ViewNode::caster_->setParams(map.getResolution(), map_origin);

    if (height_drift) {
      origin.z() -= 0.04;
      task.updateRobotPose(origin, 0.0);
    }
    Vector3d next;
    double yaw;
    bool split;
    check(manager.buildWideSideBypass(origin, 0.0, Vector3d::UnitX(), next, yaw, split) &&
          manager.wide_side_bypass_active_ && (next-origin).y()*side > 0.15 &&
          next.x() >= origin.x()+0.19,
          "initial bypass did not choose the wider lane with available forward progress");
    if (height_drift) {
      check(std::fabs(next.z() - reference_height) < 1e-9,
            "wide-side bypass copied the low measured height into its goal");
      return;
    }
    const Vector3d initial_origin = manager.recovery_side_origin_;
    const double release_distance = manager.wide_side_bypass_release_distance_;
    if (change_width) {
      // 新观测改变截面宽度，但原侧仍有已知安全短路，不应立即掉头换边。
      cloud.clear();
      // Keep the obstacle disjoint: the old bounding-box merge incorrectly
      // counted the empty gap as part of the pillar. Narrow the actual free gap.
      for (double y=0.35; y<=0.375; y+=0.025)
        for (double z=0.25; z<=1.75; z+=0.025)
          cloud.push_back(pcl::PointXYZ(0.55, side*y, z));
      observe(origin, 8);
      map.updateESDF3d();
      const auto changed = manager.findWideSideSplit(origin, Vector3d::UnitX());
      std::cout << "changed bypass widths left=" << changed.left_width
                << " right=" << changed.right_width << std::endl;
      check(changed.valid && (side > 0.0 ? changed.right_width > changed.left_width
                                        : changed.left_width > changed.right_width),
            "fixture did not reverse the measured lane widths");
      check(manager.buildWideSideBypass(origin, 0.0, Vector3d::UnitX(), next, yaw, split) &&
            (next-origin).y()*side > 0.15 && manager.recovery_side_origin_ == initial_origin &&
            manager.wide_side_bypass_release_distance_ == release_distance,
            "a width change reversed a safe active bypass or renewed its progress origin");
    }
    if (block_side) {
      cloud.clear();
      for (double x=-0.35; x<=0.35; x+=0.025)
        for (double z=0.25; z<=1.75; z+=0.025)
          cloud.push_back(pcl::PointXYZ(x, side*0.25, z));
      observe(origin, 8);
      map.updateESDF3d();
      check(manager.buildWideSideBypass(origin, 0.0, Vector3d::UnitX(), next, yaw, split) &&
            (next-origin).y()*side < -0.15,
            "a newly blocked committed side prevented switching to the safe opposite lane");
      return;
    }
    if (change_width) return;
    // 以部分短步执行模拟反复临近终点重规划，必须累计前进至障碍后方。
    Vector3d pos = origin;
    for (int step = 0; step < 12 &&
         (pos-origin).x() < release_distance+0.02; ++step) {
      task.updateRobotPose(pos, 0.0);
      check(manager.buildWideSideBypass(pos, 0.0, Vector3d::UnitX(), next, yaw, split),
            "bypass could not continue toward the far side of the pillar");
      check(next.y()*side >= -1e-6 && next.x() >= pos.x()-1e-6,
            "partial-step replanning reversed the side or moved backward");
      pos += 0.75*(next-pos);
    }
    check((pos-origin).x() > release_distance,
          "repeated bypass replans produced no forward progress past the obstacle");
    // 居中会改变到达位置；补扫通过柱子后的前方，避免把测试建图射线的
    // 人工终点墙当成第二个障碍，干扰“首个绕障已完成”的释放检查。
    cloud.clear();
    for (double y=-1.3; y<=1.3; y+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        cloud.push_back(pcl::PointXYZ(20.0, y, z));
    observe(pos, 8);
    map.updateESDF3d();
    task.updateRobotPose(pos, 0.0);
    check(!manager.buildSideRecoveryContinuation(pos, 0.0, next, yaw) &&
          !manager.wide_side_bypass_active_ && !manager.recovery_side_latched_,
          "completed bypass constrained the next obstacle's lane selection");
  }

  static void runRecordedBypass(const char* fixture) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.task_search_manager_->global_no_return_ = false;
    manager.task_search_manager_->setMap(manager.sdf_map_);
    std::ifstream input(fixture);
    int frames;
    Vector3d pos;
    double yaw;
    check(bool(input >> frames >> pos.x() >> pos.y() >> pos.z() >> yaw),
          "recorded bypass fixture header is invalid");
    const Vector3d origin = pos;
    const Vector3d forward(std::cos(yaw), std::sin(yaw), 0.0);
    manager.task_search_manager_->stable_progress_direction_ = forward.head<2>();
    manager.task_search_manager_->stable_progress_direction_valid_ = true;
    for (int frame=0; frame<frames; ++frame) {
      int count;
      Vector3d camera;
      check(bool(input >> count >> camera.x() >> camera.y() >> camera.z()),
            "recorded cloud header is invalid");
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.resize(count);
      for (auto& point : cloud)
        check(bool(input >> point.x >> point.y >> point.z), "recorded cloud is incomplete");
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), camera);
    }
    manager.sdf_map_->updateESDF3d();
    Vector3d map_origin, map_size;
    manager.sdf_map_->getRegion(map_origin, map_size);
    ViewNode::caster_ = std::make_shared<RayCaster>();
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), map_origin);
    const auto obstacle = manager.findWideSideSplit(pos, forward);
    check(obstacle.valid, "recorded map did not reproduce the blocking obstacle");
    const double required_progress = obstacle.forward_distance +
        manager.vertical_detour_footprint_radius_ + manager.sdf_map_->getResolution();
    for (int step=0; step<24 && (pos-origin).dot(forward)<required_progress; ++step) {
      manager.task_search_manager_->updateRobotPose(pos, yaw);
      Vector3d next;
      double next_yaw;
      bool split;
      const bool selected = manager.buildWideSideBypass(pos, yaw, forward, next, next_yaw, split);
      std::cout << "recorded bypass step=" << step << " pos=" << pos.transpose()
                << " selected=" << selected;
      if (!selected) { std::cout << std::endl; break; }
      std::cout << " target=" << next.transpose() << " advance=" << (next-pos).dot(forward)
                << std::endl;
      check(manager.planner_manager_->isPositionSafe(next),
            "recorded-map bypass selected an unsafe endpoint");
      pos += 0.65*(next-pos);
    }
    std::cout << "recorded bypass progress=" << (pos-origin).dot(forward)
              << " required=" << required_progress << std::endl;
    check((pos-origin).dot(forward)>=required_progress,
          "recorded pillar map still trapped bypass in lateral oscillation");
  }
};
}
TEST(CruiseHeightRecovery, OpenSpaceClimbsAlongCommittedRightRoute) {
  fast_planner::TurnEscapeTest::runMovingClimb(0);
}
TEST(CruiseHeightRecovery, MidRouteRoofKeepsOriginalHorizontalTrajectory) {
  fast_planner::TurnEscapeTest::runMovingClimb(1);
}
TEST(CruiseHeightRecovery, UnknownOverheadKeepsOriginalHorizontalTrajectory) {
  fast_planner::TurnEscapeTest::runMovingClimb(2);
}
TEST(CruiseHeightRecovery, ActiveUnderpassRetainsHeightOwnership) {
  fast_planner::TurnEscapeTest::runMovingClimb(3);
}
TEST(CruiseHeightRecovery, RejectedGenerationRestoresOriginalTrajectory) {
  fast_planner::TurnEscapeTest::runMovingClimb(4);
}
TEST(CruiseHeightRecovery, SmallClimbStepStillRecoversHeight) {
  fast_planner::TurnEscapeTest::runMovingClimb(5);
}

TEST(SideBypassProgress, PartialStepsPassPillarOnLeft) {
  fast_planner::TurnEscapeTest::runBypassContinuation(1.0, false, false);
}

TEST(SideBypassProgress, PartialStepsPassPillarOnRight) {
  fast_planner::TurnEscapeTest::runBypassContinuation(-1.0, false, false);
}

TEST(SideBypassProgress, WidthChangesKeepSafeCurrentLane) {
  fast_planner::TurnEscapeTest::runBypassContinuation(1.0, true, false);
  fast_planner::TurnEscapeTest::runBypassContinuation(-1.0, true, false);
}

TEST(SideBypassProgress, BlockedCommittedLaneAllowsSwitch) {
  fast_planner::TurnEscapeTest::runBypassContinuation(1.0, false, true);
  fast_planner::TurnEscapeTest::runBypassContinuation(-1.0, false, true);
}

TEST(SideBypassProgress, RecordedPillarMapMakesForwardProgress) {
  const char* fixture = std::getenv("UAV0_BYPASS_REPLAY_FIXTURE");
  if (!fixture) GTEST_SKIP() << "Set UAV0_BYPASS_REPLAY_FIXTURE to an extracted flight map.";
  fast_planner::TurnEscapeTest::runRecordedBypass(fixture);
}

TEST(UnderpassClassification, FloorToCeilingPillarDoesNotDescend) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(true, false);
}

TEST(UnderpassClassification, UnknownLowerPillarDoesNotDescend) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, false);
}

TEST(UnderpassClassification, ConfirmedSuspendedObstacleDescends) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true);
}

TEST(UnderpassClassification, GapInLowObservationsDoesNotDescend) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, true);
}

TEST(UnderpassHeight, RejectsTargetBelowQuarterMeter) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, false, false, false, 1);
}

TEST(UnderpassHeight, AcceptsQuarterMeterBoundary) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, false, false, false, 2);
}

TEST(UnderpassHeight, PreservesHigherTaskMinimum) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, false, false, false, 3);
}

TEST(UnderpassHeight, OrdinaryProbeRejectsInsufficientVerticalSpace) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, false, false, false, 4);
}

TEST(UnderpassExit, NewlyObservedSupportRestoresNormalPlanning) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, false, true);
}

TEST(UnderpassExit, SupportFoundDuringAdvanceStopsUnderpass) {
  fast_planner::TurnEscapeTest::runUnderpassClassification(false, true, false, true, true);
}

TEST(UnderpassExit, PersistentEightCentimeterDriftReleasesOldTarget) {
  fast_planner::TurnEscapeTest::runDescentEndpointTracking(false, false);
}

TEST(UnderpassExit, PersistentHeightErrorReleasesOldTarget) {
  fast_planner::TurnEscapeTest::runDescentEndpointTracking(true, false);
}

TEST(UnderpassExit, ReachingTargetAtTimeoutStillStartsVerification) {
  fast_planner::TurnEscapeTest::runDescentEndpointTracking(false, true);
}

TEST(UnderpassExit, TimeoutGeneratesFreshHorizontalTrajectoryInSamePlan) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false, true, false, true);
}

TEST(ManualWaypoints, CompletesWithoutFrontierTargets) {
  fast_planner::TurnEscapeTest::runManualWaypoints(false);
}

TEST(ManualWaypoints, BlockedSegmentKeepsSpecifiedTarget) {
  fast_planner::TurnEscapeTest::runManualWaypoints(true);
}

TEST(ManualWaypoints, AstarDetoursWithoutChangingWaypointOrder) {
  fast_planner::TurnEscapeTest::runManualAstar("detour");
}

TEST(ManualWaypoints, AstarBlockedRouteKeepsTarget) {
  fast_planner::TurnEscapeTest::runManualAstar("blocked");
}

TEST(ManualWaypoints, AstarUnknownRouteKeepsTarget) {
  fast_planner::TurnEscapeTest::runManualAstar("unknown");
}

TEST(ManualWaypoints, AstarSameVoxelPreservesExactTarget) {
  fast_planner::TurnEscapeTest::runManualAstar("same_voxel");
}

TEST(TurnEscape, ClearsContactBeforeResumingLatchedYaw) {
  fast_planner::TurnEscapeTest::run(-M_PI/3);
}

TEST(TurnEscape, ZeroYawEscapeRemainsFinite) {
  fast_planner::TurnEscapeTest::run(0.0);
}

TEST(ExitApproach, LateralCorrectionKeepsExitYaw) {
  fast_planner::TurnEscapeTest::runPendingExitApproach(M_PI, 0.35, false);
  fast_planner::TurnEscapeTest::runPendingExitApproach(M_PI, -0.35, false);
}

TEST(ExitApproach, SideObservationKeepsExitYaw) {
  fast_planner::TurnEscapeTest::runPendingExitApproach(M_PI, -0.35, true);
}

TEST(SideRecovery, LeftDetourSurvivesCollisionAndLateralProgress) {
  fast_planner::TurnEscapeTest::runSideRecovery(1.0, false);
}

TEST(SideRecovery, RightDetourSurvivesCollisionAndLateralProgress) {
  fast_planner::TurnEscapeTest::runSideRecovery(-1.0, false);
}

TEST(CruiseHeightRecovery, FullPlannerRaisesCommittedRightBypass) {
  fast_planner::TurnEscapeTest::runSideRecovery(-1.0, false, true);
}

TEST(SideRecovery, BlockedSameSideYieldsToExistingRecovery) {
  fast_planner::TurnEscapeTest::runSideRecovery(1.0, true);
}

TEST(SideRecovery, FeasibleOrdinaryLeftRouteSurvivesWiderRightLane) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, false);
}

TEST(SideRecovery, FeasibleOrdinaryRightRouteSurvivesWiderLeftLane) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false);
}

TEST(SideRecovery, DisconnectedFrontierKeepsFeasibleRememberedSide) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, true);
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, true);
}

TEST(SideRecovery, FirstObstacleChoosesWiderLaneBeforeFrontier) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, false, false);
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false, false);
}

TEST(SideRecovery, FirstObstacleFallbackChoosesWiderLane) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, true, false);
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, true, false);
}

TEST(EntryPassage, CurrentWidthOverridesRememberedNarrowerPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, false, true, true);
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false, true, true);
}

TEST(EntryPassage, ChoosesLeftOfRightWallObstacle) {
  fast_planner::TurnEscapeTest::runEntryPassage(1);
}

TEST(EntryPassage, ChoosesRightOfMirroredObstacle) {
  fast_planner::TurnEscapeTest::runEntryPassage(-1);
}

TEST(EntryPassage, OpenEntryKeepsStraightMotion) {
  fast_planner::TurnEscapeTest::runEntryPassage(0);
}

TEST(EntryPassage, FullyBlockedEntryDoesNotCrossWall) {
  fast_planner::TurnEscapeTest::runEntryPassage(2);
}

TEST(HorizontalHeight, PublishedReferenceAndUnderpassKeepSeparateOwnership) {
  fast_planner::TurnEscapeTest::runHeightReferenceOwnership();
}

TEST(HorizontalHeight, EntryDoesNotAccumulateRepeatedTrackingError) {
  fast_planner::TurnEscapeTest::runEntryPassage(0, true);
}

TEST(HorizontalHeight, LeftBypassRetainsPublishedHeight) {
  fast_planner::TurnEscapeTest::runBypassContinuation(1.0, false, false, true);
}

TEST(HorizontalHeight, RightBypassRetainsPublishedHeight) {
  fast_planner::TurnEscapeTest::runBypassContinuation(-1.0, false, false, true);
}

TEST(HorizontalHeight, LeftSideReplansDoNotRatchetHeightDown) {
  fast_planner::TurnEscapeTest::runSideRecovery(1.0, false, false, true);
}

TEST(HorizontalHeight, RightSideReplansDoNotRatchetHeightDown) {
  fast_planner::TurnEscapeTest::runSideRecovery(-1.0, false, false, true);
}

TEST(HorizontalHeight, KnownFreeSlopeAllowsHeightCorrection) {
  fast_planner::TurnEscapeTest::runMovingClimb(0, true);
}

TEST(HorizontalHeight, MidRouteRoofRejectsHeightCorrection) {
  fast_planner::TurnEscapeTest::runMovingClimb(1, true);
}

TEST(HorizontalHeight, UnknownOverheadRejectsHeightCorrection) {
  fast_planner::TurnEscapeTest::runMovingClimb(2, true);
}
TEST(PassageCentre, LowCorridorGuidesActualSpline) { fast_planner::TurnEscapeTest::runPassageCentre(); }

TEST(RouteWidth, OccupiedLayerBetweenOldColumnSamplesIsNotMissed) {
  fast_planner::TurnEscapeTest::runWidthColumn();
}
TEST(RouteWidth, FeasibleStraightCompetesWithWiderRightRoute) {
  fast_planner::TurnEscapeTest::runStraightWidth(false);
}
TEST(RouteWidth, MirroredWiderLeftRouteUsesSameRanking) {
  fast_planner::TurnEscapeTest::runStraightWidth(true);
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"turn_escape_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
