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

  static void runHoverClearance() {
    ros::NodeHandle nh("~");
    nh.setParam("mission/hover_clearance_margin", 0.20);
    nh.setParam("manager/footprint_check_radius", 0.18);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& task = *manager.task_search_manager_;
    auto& planner = *manager.planner_manager_;
    task.setMap(manager.sdf_map_);
    task.global_no_return_ = false;
    const Vector3d start(0.013, 0.017, 1.013);
    task.updateRobotPose(start, 0.0);
    pcl::PointCloud<pcl::PointXYZ> walls;
    for (double t=-1.5; t<=1.5; t+=0.04)
      for (double z=0.15; z<=1.75; z+=0.04) {
        walls.push_back(pcl::PointXYZ(1.5, t, z));
        walls.push_back(pcl::PointXYZ(-1.5, t, z));
        walls.push_back(pcl::PointXYZ(t, 1.5, z));
        walls.push_back(pcl::PointXYZ(t, -1.5, z));
      }
    for (const Vector3d& viewpoint : {start, Vector3d(-0.6, 0.5, 1.013)})
      for (int i=0; i<6; ++i)
        manager.sdf_map_->inputPointCloud(walls, walls.size(), viewpoint);
    check(task.isHoverPositionSafe(start), "open observed space rejected a hover point");
    check(!task.isHoverPositionSafe(Vector3d(1.8, 1.8, 1)),
          "unobserved space was accepted as hover clearance");
    walls.clear();
    for (double y=-1.3; y<=1.3; y+=0.025)
      for (double z=0.15; z<=1.75; z+=0.025)
        walls.push_back(pcl::PointXYZ(0.80, y, z));
    for (int i=0; i<6; ++i)
      manager.sdf_map_->inputPointCloud(walls, walls.size(), start);
    manager.sdf_map_->updateESDF3d();
    const Vector3d near_wall(0.57, 0.017, 1.013), roomy(0.25, -0.45, 1.013);
    check(std::fabs(task.hoverClearanceRadius()-0.38) < 1e-9,
          "hover radius did not add the configured 20cm outside the 18cm body");
    check(planner.isPositionSafe(near_wall) &&
          planner.isPathSafe({start, near_wall}) && !task.isHoverPositionSafe(near_wall),
          "transit-safe point with only 10cm outside the body became a hover target");
    check(task.isHoverPositionSafe(roomy), "roomy alternative was lost");
    task.recordSelectedGoal(near_wall);
    const int chosen = task.selectSearchCandidate({near_wall, roomy}, {0, 0}, {}, start, 0.0);
    check(chosen == 1, "retained near-wall goal bypassed the hover filter");
    vector<Vector3d> route{start, start+Vector3d(0.20, 0, 0), near_wall};
    check(manager.trimPathToHoverPoint(route) && route.back().x() < near_wall.x() &&
          task.isHoverPositionSafe(route.back(), Astar::pathLength(route)) && (route.back()-start).norm() >= 0.20,
          "short-route endpoint did not retreat to a usable hover point");
    manager.hover_clearance_required_ = true;
    check(planner.planStraightTraj(start, near_wall) && planner.isTrajectorySafe() &&
          !manager.currentPlanHasHoverClearance(),
          "optimized endpoint check accepted the original near-wall stop");
    check(planner.planStraightTraj(start, route.back()) &&
          manager.currentPlanHasHoverClearance(), "corrected endpoint failed hover validation");
    // 复现首条轨迹发布前重锚：旧终点安全，实时平移后进入停车余量区。
    Eigen::MatrixXd shifted = planner.local_data_.position_traj_.getControlPoint();
    shifted.col(0).array() += 0.15;
    NonUniformBspline moved(shifted, 3, planner.local_data_.position_traj_.getKnotSpan());
    moved.setKnot(planner.local_data_.position_traj_.getKnot());
    planner.local_data_.position_traj_ = moved;
    check(!manager.currentPlanHasHoverClearance(),
          "release re-anchor moved a safe endpoint near the wall without detection");
    // 同一地图补入窄通道：可通行不等于可以在其中停车，不能偷偷增大通行半径。
    walls.clear();
    for (double x=-1.2; x<=0.6; x+=0.025)
      for (double z=0.15; z<=1.75; z+=0.025)
        for (double y : {-0.30, 0.30})
          walls.push_back(pcl::PointXYZ(x, y, z));
    for (int i=0; i<6; ++i)
      manager.sdf_map_->inputPointCloud(walls, walls.size(), start);
    manager.sdf_map_->updateESDF3d();
    vector<Vector3d> narrow{Vector3d(-0.60, 0, 1.013), Vector3d(0.30, 0, 1.013)};
    vector<Vector3d> short_narrow{Vector3d(0, 0, 1.013), narrow.back()};
    check(planner.isPathSafe(narrow) && !task.isHoverPositionSafe(narrow.back()) &&
          !manager.trimPathToHoverPoint(short_narrow),
          "narrow transit corridor acquired a false short-path hover point or lost passability");
  }

  static void runShortLongHoverRecovery(bool lateral) {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    nh.setParam("mission/hover_clearance_margin", 0.20);
    nh.setParam("mission/long_path_hover_clearance_margin", 0.10);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& task = *manager.task_search_manager_;
    auto& planner = *manager.planner_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    const Vector3d start(0.013, 0.017, 1.013);
    const Vector3d travel = lateral ? Vector3d::UnitY() : Vector3d::UnitX();
    task.updateRobotPose(start, 0.0);
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    pcl::PointCloud<pcl::PointXYZ> walls;
    for (double x=-1.4; x<=1.1; x+=0.025)
      for (double z=0.15; z<=1.75; z+=0.025)
        for (double y : {-0.35, 0.35})
          walls.push_back(pcl::PointXYZ(x, y, z));
    for (double y=-0.35; y<=0.35; y+=0.025)
      for (double z=0.15; z<=1.75; z+=0.025)
        for (double x : {-1.4, 1.1})
          walls.push_back(pcl::PointXYZ(x, y, z));
    if (lateral)
      for (auto& point : walls) std::swap(point.x, point.y);
    for (const Vector3d& viewpoint : {start, Vector3d(start - 0.60*travel),
                                     Vector3d(start + 0.65*travel)})
      for (int i=0; i<6; ++i)
        manager.sdf_map_->inputPointCloud(walls, walls.size(), viewpoint);
    manager.sdf_map_->updateESDF3d();
    check(std::fabs(task.hoverClearanceRadius(0.40)-0.38) < 1e-9 &&
          std::fabs(task.hoverClearanceRadius(0.41)-0.28) < 1e-9,
          "short/long path margin boundary is incorrect");
    const Vector3d short_end = start + 0.30*travel;
    const Vector3d long_end = start + 0.60*travel;
    check(!task.isHoverPositionSafe(short_end, 0.30) &&
          task.isHoverPositionSafe(long_end, 0.60),
          "corridor did not distinguish 20cm short stops from 10cm long stops");
    manager.hover_clearance_required_ = true;
    check(planner.planStraightTraj(start, short_end) &&
          !manager.currentPlanHasHoverClearance(),
          "short spline inherited the long-path clearance");
    check(planner.planStraightTraj(start, long_end) &&
          planner.isTrajectorySafe() && manager.currentPlanHasHoverClearance(),
          "long spline was still required to keep 20cm clearance");
    check(planner.planStraightTraj(start, short_end) &&
          !manager.currentPlanHasHoverClearance(),
          "short replacement inherited the previous long trajectory margin");
    vector<Vector3d> short_path{start, short_end}, long_path{start, long_end};
    check(!manager.trimPathToHoverPoint(short_path) &&
          manager.trimPathToHoverPoint(long_path) && long_path.back() == long_end,
          "route truncation did not reclassify short versus long stops");
    task.recordSelectedGoal(long_end);
    check(task.selectSearchCandidate({long_end}, {0.0}, {}, start, 0.0) == 0,
          "frontier prefilter discarded a valid long-path stop");
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.ep_->mission_forward_fallback_step_ = 0.20;
    manager.ep_->mission_forward_fallback_max_ = 0.40;
    manager.short_backtrack_enabled_ = false;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = start - Vector3d(1, 0, 0);
    manager.mission_workspace_dir_ = Vector3d::UnitX();
    manager.vertical_detour_enabled_ = false;
    manager.wide_side_bypass_enabled_ = false;
    task.min_search_height_ = task.max_search_height_ = start.z();
    Vector3d next;
    double yaw = 0.0;
    check(!manager.buildMissionForwardFallback(start, 0.0, next, yaw),
          "short-only search released a stop with less than 20cm clearance");
    manager.ep_->mission_forward_fallback_max_ = 0.80;
    check(manager.buildMissionForwardFallback(start, 0.0, next, yaw) &&
          (next-start).norm() > 0.40 && task.isHoverPositionSafe(next, (next-start).norm()),
          "unavailable short steps did not yield to a connected long-path stop");
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
            goal.side*open_side >= -1e-6 && task.isHoverPositionSafe(goal.position, (goal.position-start).norm()),
            "entry selected the obstructed side or a stop without hover clearance");
      // 下一轮从已执行短步继续；现有绕行侧仍可用时不能被局部净空改成反侧。
      const Vector3d midway = 0.5 * (start + goal.position);
      task.updateRobotPose(midway, 0.0);
      entry_forward::Goal next;
      const double remaining = 0.62 - 0.5 * goal.advance;
      if (remaining >= 0.20) {
        // 停车余量不足时可交还普通A*绕行，不能为保持入口短步强造近障停点。
        if (manager.selectEntryForwardGoal(midway, Vector3d::UnitX(), remaining, next))
          check(next.side*open_side >= -1e-6 && task.isHoverPositionSafe(next.position, (next.position-midway).norm()),
                "entry reversed sides or forced a near-wall stop on replan");
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
    // 停车余量需要已知FREE证据，补足真实侧墙观测，避免端墙射线留下未知扇区。
    cloud.clear();
    for (double x=-1.5; x<=1.5; x+=0.025)
      for (double z=0.25; z<=1.75; z+=0.025)
        for (double y : {-1.3, 1.3})
          cloud.push_back(pcl::PointXYZ(x, y, z));
    for (const Vector3d& viewpoint : {origin, Vector3d(0, -0.8, 1)})
      for (int i=0; i<5; ++i)
        manager.sdf_map_->inputPointCloud(cloud, cloud.size(), viewpoint);
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
      // 新停车余量下，原有仅可通行的微调点可能已经不适合停车；恢复器可
      // 换到合格位置，也可返回无可用点，但不能重新锁死原侧或发布近墙点。
      if (manager.buildMissionForwardFallback(displaced, 0.0, next, yaw)) {
        check(planner.isPositionSafe(next) && task.isHoverPositionSafe(next, (next-displaced).norm()),
              "fallback selected a transit-only near-wall stop");
      } else {
        check(!manager.recovery_side_latched_,
              "unavailable recovery restored a hard lock on the blocked side");
      }
      return;
    }

    check(manager.planExploreMotion(displaced, Vector3d::Zero(), Vector3d::Zero(),
                                   Vector3d(0.10, 0, 0)) == SUCCEED,
          "same-side continuation failed to generate a trajectory");
    check(manager.recovery_side_latched_ && !manager.recovery_side_continuation_pending_,
          "0.22m lateral progress released the side or successful plan stayed pending");
    const double duration = planner.local_data_.duration_;
    const Vector3d end = planner.local_data_.position_traj_.evaluateDeBoorT(duration);
    check((end-displaced).y()*side >= 0.15 && std::fabs(end.z()-displaced.z()) < 0.05,
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

TEST(HoverClearance, ShortStepsYieldToLongPathsWithTenCentimeterMargin) {
  fast_planner::TurnEscapeTest::runShortLongHoverRecovery(false);
}

TEST(HoverClearance, ShortSideStepsYieldToLongLateralPaths) {
  fast_planner::TurnEscapeTest::runShortLongHoverRecovery(true);
}

TEST(HoverClearance, StopsAwayFromWallAndRechecksActualEndpoint) {
  fast_planner::TurnEscapeTest::runHoverClearance();
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
