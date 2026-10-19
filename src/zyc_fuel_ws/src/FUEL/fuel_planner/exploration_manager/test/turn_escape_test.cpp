#include <cstdlib>
#include <gtest/gtest.h>
#include <iostream>
#include <exploration_manager/fast_exploration_manager.h>
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
  static void initialize(ros::NodeHandle& nh, FastExplorationManager& manager) {
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
  static void runUnderpassClassification(bool pillar, bool known_low_space,
                                         bool missing_low_layer = false) {
    ros::NodeHandle nh("~");
    nh.setParam("mission/task_search/min_height", 0.10);
    FastExplorationManager manager;
    initialize(nh, manager);
    nh.deleteParam("mission/task_search/min_height");
    manager.mission_entered_search_region_ = true;
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

    const bool expected_suspended = !pillar && known_low_space && !missing_low_layer;
    check(manager.hasCurrentHeightForwardObstacle(pos, forward),
          "underpass fixture has no obstacle at flight height");
    double bottom, exit_progress, distance;
    check(manager.detectSuspendedObstacle(pos, forward, bottom, exit_progress, distance) ==
              expected_suspended,
          "pillar, unknown low space or suspended obstacle was misclassified");
    check(manager.hasSuspendedForwardObstacle(pos, forward) == expected_suspended,
          "fallback obstacle classification differs from primary detector");
    check(manager.beginSuspendedUnderpass(pos, 0.0) == expected_suspended,
          "primary underpass trigger ignored obstacle classification");
    if (expected_suspended) {
      check(manager.suspended_underpass_active_ &&
            manager.low_probe_phase_ == vertical_detour::LowProbePhase::DESCENDING,
            "confirmed suspended obstacle did not start descent");
      check(manager.low_probe_target_.z() < pos.z(),
            "confirmed suspended obstacle lost its downward target");
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

  static void runEntryPassage(int open_side) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    task.exit_detection_enabled_ = false;
    const double mirror = open_side < 0 ? -1.0 : 1.0;
    const Vector3d start(0.033, -0.103*mirror, 0.473);
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
    check(end.x()-start.x() >= 0.20-1e-3 && std::fabs(end.z()-start.z()) < 1e-6,
          "entry passage lost actual forward progress or changed height");
  }

  static void runWiderSideRecovery(double wide_side, bool fallback_only,
                                   bool remember_side = true, bool entry = false) {
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
    // 动态入口飞行已完成一米推进，可能还未进入静态搜索框；同样必须先比较宽度。
    manager.mission_entered_search_region_ = fallback_only;
    manager.entry_forward_complete_ = !fallback_only;
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
    if (fallback_only) {
      check(manager.buildMissionForwardFallback(start, yaw, next, yaw),
            "disconnected-frontier recovery failed to find the wider side");
    } else {
      check(manager.planExploreMotion(start, Vector3d::Zero(), Vector3d::Zero(),
                                     Vector3d(yaw, 0, 0)) == SUCCEED,
            "wider-side priority failed to generate a trajectory");
      next = planner.local_data_.position_traj_.evaluateDeBoorT(planner.local_data_.duration_);
      check(planner.isTrajectorySafe(), "wider-side trajectory bypassed footprint checks");
    }
    std::cout << "wider-side fixture side=" << wide_side << " fallback=" << fallback_only
              << " widths=" << split.left_width << "," << split.right_width
              << " next=" << next.transpose()
              << " remembered=" << manager.recovery_side_dir_.transpose() << std::endl;
    check((next-start).y()*wide_side >= 0.10 &&
          std::fabs(next.z()-start.z()) < 0.05 &&
          manager.recovery_side_dir_.y()*wide_side > 0.9,
          "old narrow-side preference overrode the current pillar's feasible wider lane");
  }

  static void runSideRecovery(double side, bool block_side) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    const Vector3d origin(0.013, 0.017, 1.013);
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
    cloud.clear();
    for (double y=-0.10; y<=0.10; y+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        cloud.push_back(pcl::PointXYZ(0.50, y, z));
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
    manager.ground_ascent_enabled_ = false;
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

    const Vector3d displaced = origin + Vector3d(0, side*0.22, 0);
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
    // 宽侧规划走A*，既有短路径截断会让首段约横移0.14m，仍须保持所选方向。
    check((end-displaced).y()*side >= 0.10 && std::fabs(end.z()-displaced.z()) < 0.05,
          "replan switched sides or changed the recovery height");
    check(planner.isTrajectorySafe(), "same-side spline bypassed footprint safety");
    Vector3d next;
    double yaw = 0.0;
    check(manager.isLowProbeCorridorSafe(end, Vector3d::UnitX(), 0.40) &&
          manager.buildSideRecoveryContinuation(end, 0.0, next, yaw) &&
          next.x() > end.x()+0.15 && std::fabs(next.y()-end.y()) < 1e-6,
          "clear same-side lane did not resume forward motion");
    check(!manager.buildSideRecoveryContinuation(end+Vector3d(0.25, 0, 0),
                                                 0.0, next, yaw) &&
          !manager.recovery_side_latched_, "clear forward progress did not release the side");
    manager.recovery_side_latched_ = true;
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::VERIFYING;
    check(!manager.buildSideRecoveryContinuation(displaced, 0.0, next, yaw) &&
          manager.recovery_side_latched_, "side preference took over an active vertical probe");
    manager.low_probe_phase_ = vertical_detour::LowProbePhase::IDLE;
    manager.completeTurnInPlace(displaced);
    check(!manager.recovery_side_latched_, "confirmed turn kept the old obstacle side");
  }
};
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

TEST(ManualWaypoints, CompletesWithoutFrontierTargets) {
  fast_planner::TurnEscapeTest::runManualWaypoints(false);
}

TEST(ManualWaypoints, BlockedSegmentKeepsSpecifiedTarget) {
  fast_planner::TurnEscapeTest::runManualWaypoints(true);
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

TEST(SideRecovery, BlockedSameSideYieldsToExistingRecovery) {
  fast_planner::TurnEscapeTest::runSideRecovery(1.0, true);
}

TEST(SideRecovery, CurrentWiderRightLaneOverridesOldLeftBeforeFrontier) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, false);
}

TEST(SideRecovery, CurrentWiderLeftLaneOverridesOldRightBeforeFrontier) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false);
}

TEST(SideRecovery, DisconnectedFrontierUsesWiderLaneBeforeOldSide) {
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

int main(int argc,char** argv) {
  ros::init(argc,argv,"turn_escape_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
