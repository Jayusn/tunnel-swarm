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
                         bool production_weights = false, bool production_map = false) {
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
    if (production_map) {
      nh.setParam("sdf_map/ground_height", -1.0);
      nh.setParam("sdf_map/map_size_z", 3.0);
      nh.setParam("sdf_map/obstacles_inflation", 0.099);
      nh.setParam("sdf_map/inflation_noise_filter_enabled", true);
      nh.setParam("sdf_map/inflation_vertical_radius", 4);
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
    const std::weak_ptr<TaskSearchManager> height_task = manager.task_search_manager_;
    if (planner.path_finder_) planner.path_finder_->setHeightLimitProvider([height_task]() {
      const auto task = height_task.lock();
      return task ? task->corridorHeightLimit() : std::numeric_limits<double>::infinity();
    });
  }
public:
  static void runUpperExecution(bool diagonal, bool passed_fapp = false) {
    ros::NodeHandle nh("~upper_execution");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("manager/footprint_bottom_clearance", .20);
    nh.setParam("astar/min_search_height", .10);
    nh.setParam("astar/max_search_height", 1.50);
    nh.setParam("astar/resolution_astar", .05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("fapp/enabled", passed_fapp);
    nh.setParam("fapp/memory_enabled", passed_fapp);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    const Vector3d pos(.025, .025, diagonal ? .375 : .30);
    const Vector3d forward = Vector3d::UnitX(), zero = Vector3d::Zero();
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    task.exit_detection_enabled_ = false;
    task.cruise_height_ = .60;
    task.min_search_height_ = .10;
    task.max_search_height_ = 1.50;
    task.setCorridorFrame(pos - forward, forward);
    task.updateRobotPose(pos, 0);
    manager.ground_ascent_enabled_ = false;
    manager.vertical_detour_upper_max_side_angle_deg_ = 0;
    manager.entry_forward_complete_ = true;
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = pos - forward;
    manager.mission_workspace_dir_ = forward;
    for (int x = 0; x < map.mp_->map_voxel_num_.x(); ++x)
      for (int y = 0; y < map.mp_->map_voxel_num_.y(); ++y)
        for (int z = 0; z < map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i index(x, y, z);
          Vector3d p; map.indexToPos(index, p);
          const bool occupied = p.x() >= (diagonal ? .375 : .275) &&
              p.x() <= .575 && std::fabs(p.y() - pos.y()) <= .60 &&
              p.z() < (diagonal ? .60 : 1.20);
          const int address = map.toAddress(index);
          map.md_->occupancy_buffer_[address] = occupied ? map.mp_->clamp_max_log_
                                                        : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address] = occupied;
        }
    if (passed_fapp) {
      manager.fapp_exploration_crossing_enabled_ = true;
      manager.manual_fapp_active_ = true;
      manager.manual_fapp_forward_ = forward;
      manager.manual_fapp_hold_height_ = pos.z();
      planner.fapp_watched_valid_ = true;
      planner.fapp_watched_lower_ = pos + Vector3d(-1.20, -.30, -.10);
      planner.fapp_watched_upper_ = pos + Vector3d(-.60, .30, .70);
      planner.fapp_watched_state_.stamp = ros::Time::now().toSec();
      EXPECT_FALSE(manager.manualFappObservationMissing(pos, 0, true));
      ASSERT_FALSE(manager.manual_fapp_active_);
      ASSERT_FALSE(planner.fapp_watched_valid_);
    }
    manager.edt_environment_ = planner.edt_environment_;
    nh.setParam("frontier/cluster_min", 1000000);
    manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(pos, zero, zero, zero));
    ASSERT_TRUE(manager.upper_detour_plan_);
    EXPECT_EQ(vertical_detour::LowProbePhase::IDLE, manager.low_probe_phase_);
    const Vector3d goal = manager.ed_->next_goal_;
    EXPECT_GT(goal.z(), pos.z() + .20);
    if (diagonal) {
      // 低层只支持原地升高，更高一层可斜升时必须先执行斜升。
      EXPECT_GT(goal.x(), pos.x() + .15);
      EXPECT_GT(goal.z(), .80);
    } else {
      EXPECT_LT((goal - pos).head<2>().norm(), 1e-6);
      ASSERT_TRUE(manager.currentPlanIsVerticalDetour());
      const Vector3d live = pos + Vector3d(0, .02, -.01);
      ASSERT_TRUE(manager.reanchorVerticalDetourForRelease(live, zero));
      auto& curve = planner.local_data_.position_traj_;
      EXPECT_LT((curve.evaluateDeBoorT(0) - live).norm(), 1e-5);
      for (double t = 0; t <= curve.getTimeSum(); t += .02)
        EXPECT_LT((curve.evaluateDeBoorT(t) - live).head<2>().norm(), 1e-6);
      EXPECT_NEAR(goal.z(), curve.evaluateDeBoorT(curve.getTimeSum()).z(), 1e-5);
      // 实飞中0.374 -> 0.400m的短上升也必须生成，不能再落入IDLE拒绝。
      manager.last_requested_goal_ = live + Vector3d(0, 0, .025);
      ASSERT_TRUE(manager.reanchorVerticalDetourForRelease(live, zero));
      EXPECT_NEAR(live.z() + .025,
                  curve.evaluateDeBoorT(curve.getTimeSum()).z(), 1e-5);
    }
    EXPECT_TRUE(planner.isTrajectorySafe());
    manager.recordPublishedTrajectoryHeight();
    EXPECT_DOUBLE_EQ(.60, manager.horizontalGoalHeight(goal.z()));
  }

  static void runSafeFailureSideEscape(bool inflated_start = false) {
    ros::NodeHandle nh("~safe_failure_side_escape");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", .05);
    nh.setParam("astar/allocate_num", 100000);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    const Vector3d pos(.025, .025, 1.025), zero = Vector3d::Zero();
    for (int x = 0; x < map.mp_->map_voxel_num_.x(); ++x)
      for (int y = 0; y < map.mp_->map_voxel_num_.y(); ++y)
        for (int z = 0; z < map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i index(x, y, z);
          Vector3d p; map.indexToPos(index, p);
          const int address = map.toAddress(index);
          map.md_->occupancy_buffer_[address] = map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address] =
              p.x() >= (inflated_start ? 0.0 : .175) && p.x() <= .60 &&
              std::fabs(p.y() - pos.y()) <= .10;
        }
    manager.task_search_manager_->stable_progress_direction_ = Eigen::Vector2d(1, 0);
    manager.task_search_manager_->stable_progress_direction_valid_ = true;
    manager.recovery_odom_history_ = {pos - Vector3d(.5, 0, 0), pos};
    ASSERT_EQ(!inflated_start, manager.planner_manager_->isPositionSafe(pos));
    ASSERT_EQ(inflated_start, manager.planner_manager_->isControlledEscapePosition(pos));
    manager.upper_detour_plan_ = true; // 上绕候选生成失败后的现场状态。
    manager.pending_reference_height_ = 1.40;
    ASSERT_TRUE(manager.planFailedMotionEscape(pos, zero, zero, zero));
    const Vector3d delta = manager.ed_->next_goal_ - pos;
    EXPECT_GE(delta.x(), -.03);
    EXPECT_GT(std::fabs(delta.y()), .04);
    EXPECT_FALSE(manager.upper_detour_plan_);
    EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
    EXPECT_FALSE(std::isfinite(manager.pending_reference_height_));
    EXPECT_TRUE(manager.planner_manager_->isTrajectorySafe(.03, true));
  }

  static void runEarlyUpperCombination(int mode) {
    ros::NodeHandle nh("~early_upper_combination");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("manager/footprint_bottom_clearance", .20);
    nh.setParam("astar/min_search_height", .10);
    nh.setParam("astar/max_search_height", 1.50);
    nh.setParam("astar/resolution_astar", .05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    const Vector3d pos(.025, .025, mode == 3 ? .975 : .575), forward(1, 0, 0);
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    task.exit_detection_enabled_ = false;
    task.cruise_height_ = pos.z();
    task.max_search_height_ = mode == 5 ? 1.10 : 1.50;
    task.setCorridorFrame(pos - forward, forward);
    task.updateRobotPose(pos, 0);
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = pos - forward;
    manager.mission_workspace_dir_ = forward;
    manager.entry_forward_complete_ = true;
    for (int x=0; x<map.mp_->map_voxel_num_.x(); ++x)
      for (int y=0; y<map.mp_->map_voxel_num_.y(); ++y)
        for (int z=0; z<map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i index(x,y,z);
          Vector3d p; map.indexToPos(index,p);
          // 两个锥桶的保守体素包络，中间低泡沫箱；当前高度中缝仍可通。
          // 整体距机体约0.75m，超出旧上绕0.60m扫描范围。
          const bool cone = p.x() >= .775 && p.x() <= .925 &&
              std::fabs(std::fabs(p.y() - pos.y()) - .40) <= .075 && p.z() < (mode>=6 ? .70 : .65);
          const bool box = p.x() >= .875 && p.x() <= 1.125 &&
              std::fabs(p.y() - pos.y()) <= .20 &&
              p.z() < (mode>=6 ? .70 : (mode==1 || mode==2) ? .65 : .30);
          const bool roof = mode == 1 && p.z() >= .725;
          const bool unknown = mode == 2 && p.z() >= .725;
          const bool high_barrier = (mode == 4 || mode == 5) && p.x() >= .775 && p.x() <= .925 &&
              std::fabs(p.y() - pos.y()) <= .50 && p.z() < 1.05;
          const bool side_wall = (mode == 4 || mode == 5) && std::fabs(p.y() - pos.y()) >= .65;
          // 局部物体顶Z=.70，旁边有Z=1.40的受支持高点（不满足连续墙分类）。
          const bool high_patch = (mode==6 || mode==7) && p.x()>=.775 && p.x()<=.925 &&
              (mode==7 ? std::fabs(p.y()-pos.y())<=.65 : std::fabs(p.y()-pos.y()-.55)<=.025) &&
              p.z()<1.40 && p.z()>=1.25;
          const bool high_wall = mode==7 && p.x()>=.775 && p.x()<=.925 &&
              std::fabs(p.y()-pos.y())<=.65 && p.z()<1.40;
          const bool isolated = mode==8 && (p-Vector3d(.825,.575,1.375)).norm()<.01;
          const bool ceiling = mode==9 && p.z()>=1.375 && p.z()<1.425;
          const bool occupied = cone || box || roof || high_barrier || side_wall ||
                                high_patch || high_wall || isolated || ceiling;
          const int address = map.toAddress(index);
          map.md_->occupancy_buffer_[address] = occupied ? map.mp_->clamp_max_log_
              : unknown ? map.mp_->clamp_min_log_ - map.mp_->unknown_flag_ : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address] = occupied ? 1 : 0;
        }
    double top;
    ASSERT_TRUE(manager.findForwardObstacleTop(pos, forward, top));
    if (mode<6 && mode != 1) EXPECT_NEAR(mode >= 4 ? 1.05 : .65, top, .001);
    if (mode==6 || mode==7) EXPECT_NEAR(1.40,top,.001);
    if (mode==8) EXPECT_NEAR(.70,top,.001);
    Vector3d next; double next_yaw=0;
    if (mode == 0) {
      bool gate = false;
      ASSERT_TRUE(manager.buildCenterGatePassage(pos, 0, forward, next, next_yaw, gate));
      ASSERT_TRUE(gate);
      EXPECT_DOUBLE_EQ(pos.z(), next.z());
      EXPECT_FALSE(manager.buildForwardUpperDetour(pos, 0, forward, next, next_yaw));
      EXPECT_FALSE(manager.buildVerticalDetourFallback(pos, 0, forward, next, next_yaw,
                                                      false, false));
      ASSERT_TRUE(manager.buildMissionForwardFallback(pos, 0, next, next_yaw));
      EXPECT_DOUBLE_EQ(pos.z(), next.z());
      EXPECT_GT(next.x(), pos.x());
      EXPECT_TRUE(manager.planner_manager_->isPathSafe({pos,next}));
    } else if (mode == 4 || mode==6 || mode==8 || mode==9) {
      if (mode==4) ASSERT_GT(top + .20 - pos.z(), manager.vertical_detour_upper_max_rise_);
      manager.edt_environment_ = manager.planner_manager_->edt_environment_;
      nh.setParam("frontier/cluster_min", 1000000);
      manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
      manager.external_selection_only_ = mode==4;
      ASSERT_EQ(SUCCEED, manager.planExploreMotion(pos, Vector3d::Zero(),
                                                   Vector3d::Zero(), Vector3d::Zero()));
      if (mode==4) {
        EXPECT_GE(manager.external_selected_viewpoint_.z(), top + .20 - 1e-6);
        EXPECT_LE(manager.external_selected_viewpoint_.z(), 1.50);
        EXPECT_GT(manager.external_selected_viewpoint_.x(), pos.x());
      } else {
        auto& curve=manager.planner_manager_->local_data_.position_traj_;
        const Vector3d end=curve.evaluateDeBoorT(curve.getTimeSum());
        EXPECT_NEAR(.90,end.z(),1e-4);
        EXPECT_GT(end.x(),pos.x());
        EXPECT_LT(end.x(),.775); // 在物体前完成抬升，不能把上升弧线拖进中缝。
        double previous=pos.z();
        for(double t=0;t<curve.getTimeSum();t+=.02) {
          const Vector3d point=curve.evaluateDeBoorT(t);
          EXPECT_GE(point.z(),previous-1e-6);
          EXPECT_NEAR(0.0,(point-pos).cross(end-pos).norm(),1e-6);
          previous=point.z();
        }
        EXPECT_TRUE(manager.planner_manager_->isTrajectorySafe());
      }
    } else {
      EXPECT_FALSE(manager.buildForwardUpperDetour(pos, 0, forward, next, next_yaw));
    }
  }

  static void runLowObstacleUpperDetour(bool blocked_above) {
    ros::NodeHandle nh("~low_obstacle_upper");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("manager/footprint_bottom_clearance", .20);
    nh.setParam("mission/task_search/min_height", .10);
    nh.setParam("astar/min_search_height", .10);
    nh.setParam("astar/max_search_height", 1.50);
    nh.setParam("astar/resolution_astar", .05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),
              map.md_->occupancy_buffer_inflate_.end(), 0);
    const Vector3d pos(.025, .025, .375), forward(1, 0, 0);
    // 顶部仅0.30m，低于实飞配置的0.40m下探目标，仍应识别并尝试上绕。
    for (double x=.325; x<=.475; x+=.05)
      for (double y=-.175; y<=.225; y+=.05)
        for (double z=.025; z<=.275; z+=.05) {
          Eigen::Vector3i index; map.posToIndex(Vector3d(x,y,z), index);
          map.md_->occupancy_buffer_[map.toAddress(index)] = map.mp_->clamp_max_log_;
        }
    manager.vertical_detour_low_height_ = .40;
    manager.vertical_detour_upper_step_ = .20;
    manager.vertical_detour_upper_max_rise_ = .60;
    manager.vertical_detour_upper_forward_max_distance_ = .60;
    double top = 0;
    ASSERT_TRUE(manager.findForwardObstacleTop(pos, forward, top));
    EXPECT_NEAR(.30, top, .001);
    // 改变下探高度也不能改变上绕看到的障碍高度。
    manager.vertical_detour_low_height_ = .80;
    double same_top = 0;
    ASSERT_TRUE(manager.findForwardObstacleTop(pos, forward, same_top));
    EXPECT_DOUBLE_EQ(top, same_top);
    // 靠近后也不能因为动作步长0.20m而漏掉0.10m前方的已占据障碍。
    double near_top = 0;
    ASSERT_TRUE(manager.findForwardObstacleTop(pos + .20*forward, forward, near_top));
    EXPECT_DOUBLE_EQ(top, near_top);
    if (blocked_above) {
      for (int x=0; x<map.mp_->map_voxel_num_.x(); ++x)
        for (int y=0; y<map.mp_->map_voxel_num_.y(); ++y)
          for (int z=0; z<map.mp_->map_voxel_num_.z(); ++z) {
            const Eigen::Vector3i index(x,y,z);
            Vector3d point; map.indexToPos(index, point);
            if (point.z() < .425) continue;
            const int address = map.toAddress(index);
            map.md_->occupancy_buffer_[address] = map.mp_->clamp_max_log_;
            map.md_->occupancy_buffer_inflate_[address] = 1;
        }
    }
    Vector3d next; double yaw = 0;
    const bool found = manager.buildVerticalDetourFallback(pos, 0, forward, next, yaw, false, false);
    EXPECT_EQ(!blocked_above, found);
    if (found) {
      EXPECT_GT(next.z(), pos.z());
      EXPECT_GE(next.z() - top, manager.vertical_detour_upper_obstacle_clearance_ - 1e-6);
      EXPECT_TRUE(manager.planner_manager_->isPathSafe({pos,next}));
    }
  }
  static void runUpperProgress(int mode) {
    ros::NodeHandle nh("~upper_progress");
    nh.setParam("manager/use_geometric_path",true);
    nh.setParam("manager/footprint_bottom_clearance",.20);
    nh.setParam("astar/resolution_astar",.05);
    nh.setParam("astar/allocate_num",100000);
    nh.setParam("astar/lambda_heu",1.0);
    nh.setParam("astar/max_search_time",1.0);
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& map=*manager.sdf_map_;
    auto& task=*manager.task_search_manager_;
    Vector3d pos(-.975,.025,mode==2 ? .85 : .60), forward(1,0,0);
    task.enabled_=true;
    task.cruise_height_=.60;
    task.min_search_height_=.10;
    task.max_search_height_=1.50;
    task.global_no_return_=false;
    task.setMap(manager.sdf_map_);
    task.setCorridorFrame(pos-forward,forward);
    task.updateRobotPose(pos,0);
    for (int x=0;x<map.mp_->map_voxel_num_.x();++x)
      for (int y=0;y<map.mp_->map_voxel_num_.y();++y)
        for (int z=0;z<map.mp_->map_voxel_num_.z();++z) {
          const Eigen::Vector3i index(x,y,z); Vector3d p; map.indexToPos(index,p);
          const bool blocker=mode!=0 && p.x()>=-.225 && p.x()<=-.075 &&
              std::fabs(p.y()-pos.y())<=.25 && p.z()<.70;
          const bool side_high=p.x()>=-.225 && p.x()<=-.075 &&
              std::fabs(p.y()-pos.y()-.55)<=.075 && p.z()<1.35;
          const int address=map.toAddress(index);
          map.md_->occupancy_buffer_[address]=(blocker || side_high)
              ? map.mp_->clamp_max_log_ : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address]=blocker || side_high;
        }
    Vector3d next; double yaw=0;
    if (mode==0) {
      ASSERT_FALSE(manager.hasForwardBodyObstacle(pos,forward));
      EXPECT_FALSE(manager.buildForwardUpperDetour(pos,0,forward,next,yaw));
      EXPECT_FALSE(manager.buildVerticalDetourFallback(pos,0,forward,next,yaw,false,false));
      return;
    }
    ASSERT_TRUE(manager.hasForwardBodyObstacle(pos,forward));
    ASSERT_TRUE(manager.buildForwardUpperDetour(pos,0,forward,next,yaw));
    EXPECT_NEAR(.90,next.z(),1e-6);
    if (mode==2) {
      EXPECT_LT(next.z()-pos.z(),.20); // 只差5cm净空不能再被强加20cm上升。
      return;
    }
    manager.pending_reference_height_=next.z();
    manager.recordPublishedTrajectoryHeight();
    EXPECT_DOUBLE_EQ(.60,manager.horizontalGoalHeight(next.z()));
    // 到达上绕段末端，前方高层通了，但斜下降仍会碰到原障碍：保持高度往前走。
    pos=next; manager.upper_detour_plan_=false;
    ASSERT_FALSE(manager.hasForwardBodyObstacle(pos,forward));
    ASSERT_TRUE(manager.buildForwardUpperDetour(pos,0,forward,next,yaw));
    EXPECT_NEAR(.90,next.z(),1e-6);
    EXPECT_NEAR(.90,next.x()-pos.x(),1e-6);
    EXPECT_TRUE(manager.planner_manager_->isPathSafe({pos,next}));
    // 通过障碍后，边向前走边回到0.6m，不停在原地继续升高。
    pos=next; manager.upper_detour_plan_=false;
    ASSERT_TRUE(manager.buildForwardUpperDetour(pos,0,forward,next,yaw));
    EXPECT_NEAR(.60,next.z(),1e-6);
    EXPECT_NEAR(.90,next.x()-pos.x(),1e-6);
    EXPECT_TRUE(manager.planner_manager_->isPathSafe({pos,next}));
  }
  static void runRecordedLowCrossing(const char* fixture, bool manual_mission = false) {
    ros::NodeHandle nh("/exploration_node");
    FastExplorationManager manager;
    manager.initialize(nh);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    std::ifstream input(fixture, std::ios::binary);
    const auto read = [&](auto& value) { input.read(reinterpret_cast<char*>(&value), sizeof(value)); };
    int frames; Vector3d pos, lower, upper;
    read(frames);
    for (auto* v : {&pos, &lower, &upper}) for (int k=0;k<3;++k) read((*v)[k]);
    for (int f=0; f<frames; ++f) {
      int count; Vector3d camera; read(count);
      for (int k=0;k<3;++k) read(camera[k]);
      pcl::PointCloud<pcl::PointXYZ> cloud; cloud.resize(count);
      for (auto& p : cloud) { read(p.x); read(p.y); read(p.z); }
      map.inputPointCloud(cloud,count,camera);
      map.clearAndInflateLocalMap();
    }
    ASSERT_TRUE(input.good());
    map.updateESDF3d();
    planner.fapp_watched_valid_=true;
    planner.fapp_watched_lower_=lower; planner.fapp_watched_upper_=upper;
    planner.fapp_watched_state_.stamp=ros::Time::now().toSec();
    manager.manual_fapp_active_=true; manager.manual_fapp_forward_=Vector3d::UnitX();
    manager.manual_fapp_hold_height_=pos.z();
    const Vector3d goal(upper.x()+map.getResolution()+.20,pos.y(),pos.z());
    std::vector<Vector3d> path;
    std::cout << "recorded start=" << pos.transpose() << " bounds=" << lower.transpose() << " / " << upper.transpose() << std::endl;
    if (manual_mission) {
      // 参数必须来自航点launch；测试目标放在录包障碍后方，不改用户航点文件。
      ASSERT_TRUE(manager.manual_path_enabled_);
      ASSERT_TRUE(manager.manual_path_exploration_style_);
      ASSERT_TRUE(manager.manual_path_wait_for_trigger_);
      ASSERT_DOUBLE_EQ(1.20, manager.manual_path_max_segment_length_);
      manager.manual_path_.points = {goal};
      manager.manual_path_.next = 0;
      ASSERT_EQ(FAIL, manager.planExploreMotion(pos,Vector3d::Zero(),
                                               Vector3d::Zero(),Vector3d::Zero()));
      manager.startManualWaypointMission();
      ASSERT_EQ(SUCCEED, manager.planExploreMotion(pos,Vector3d::Zero(),
                                                  Vector3d::Zero(),Vector3d::Zero()));
      path = manager.ed_->path_next_goal_;
      EXPECT_TRUE(manager.manual_fapp_curve_active_);
      EXPECT_GT(Astar::pathLength(path), manager.manual_path_max_segment_length_);
      EXPECT_GT(path.back().x(), upper.x()+map.getResolution()+.10);
      EXPECT_EQ(0u, manager.manual_path_.next);
      ASSERT_EQ(1u, manager.manual_path_.points.size());
      EXPECT_TRUE(manager.manual_path_.points.front().isApprox(goal));
    } else {
      ASSERT_TRUE(manager.buildManualFappUnderpassPath(pos,goal,path));
      ASSERT_TRUE(planner.planExploreTraj(path,Vector3d::Zero(),Vector3d::Zero(),0.0));
    }
    EXPECT_GT(manager.manual_fapp_curve_side_,0.30);
    EXPECT_LT(manager.manual_fapp_curve_side_,0.60);
    EXPECT_NEAR(path.back().z(),lower.z()-manager.underpass_height_tracking_margin_,1e-6);
    for (const auto& point : path) EXPECT_TRUE(planner.isKnownFreePosition(point));
    ASSERT_TRUE(planner.isFappPathSafe(path));
    EXPECT_TRUE(planner.isTrajectorySafe());
    auto& trajectory=planner.local_data_.position_traj_;
    for(double t=0;t<=trajectory.getTimeSum();t+=.02) {
      const Vector3d point=trajectory.evaluateDeBoorT(t);
      EXPECT_TRUE(planner.isWatchedFappPositionSafe(point));
    }
  }

  static void runAdaptiveLowCrossing(double open_side, bool blocked = false,
                                      bool unknown = false) {
    ros::NodeHandle nh("~adaptive_low_crossing");
    nh.setParam("mission/task_search/min_height",0.10);
    nh.setParam("manager/footprint_bottom_clearance",0.20);
    nh.setParam("fapp/enabled",true);
    nh.setParam("fapp/memory_enabled",true);
    FastExplorationManager manager;
    initialize(nh,manager,true);
    auto& planner=*manager.planner_manager_;
    auto& map=*manager.sdf_map_;
    for(int x=0;x<map.mp_->map_voxel_num_.x();++x)
      for(int y=0;y<map.mp_->map_voxel_num_.y();++y)
        for(int z=0;z<map.mp_->map_voxel_num_.z();++z) {
          const Eigen::Vector3i id(x,y,z); Vector3d p; map.indexToPos(id,p);
          const bool barrier=p.x()>-.20&&p.x()<.85&&p.z()<.45&&
              (blocked||p.y()*open_side<.16);
          const int addr=map.toAddress(id);
          map.md_->occupancy_buffer_[addr]=barrier
              ? (unknown ? map.mp_->clamp_min_log_-map.mp_->unknown_flag_ : map.mp_->clamp_max_log_)
              : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[addr]=barrier&&!unknown;
        }
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_=map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    map.updateESDF3d();
    planner.fapp_watched_valid_=true;
    planner.fapp_watched_lower_=Vector3d(-.30,-.40,.40);
    planner.fapp_watched_upper_=Vector3d(.30,.40,.90);
    planner.fapp_watched_state_.stamp=ros::Time::now().toSec();
    manager.manual_fapp_active_=true;
    manager.manual_fapp_forward_=Vector3d::UnitX();
    // 旧的普通侧绕偏好指向受阻侧，不能封死首次低位穿越的另一侧。
    manager.recovery_side_latched_=true;
    manager.recovery_side_dir_=-open_side*Vector3d::UnitY();
    const Vector3d start(-1.,0.,.65),goal(.60,0.,.65);
    std::vector<Vector3d> path;
    if(blocked) {
      EXPECT_FALSE(manager.buildManualFappUnderpassPath(start,goal,path));
      EXPECT_FALSE(manager.manual_fapp_curve_active_);
      return;
    }
    ASSERT_TRUE(manager.buildManualFappUnderpassPath(start,goal,path));
    EXPECT_GT(manager.manual_fapp_curve_side_*open_side,.30);
    EXPECT_GT(manager.recovery_side_dir_.y()*open_side,.9);
    EXPECT_NEAR(path.back().z(),.34,1e-6);
    EXPECT_GT(path.back().x(),.45);
    for(const auto& p:path) EXPECT_TRUE(planner.isKnownFreePosition(p));
    EXPECT_TRUE(planner.isFappPathSafe(path));
    ASSERT_TRUE(planner.planExploreTraj(path,Vector3d::Zero(),Vector3d::Zero(),0.0));
    EXPECT_TRUE(planner.isTrajectorySafe());
    auto trajectory=planner.local_data_.position_traj_;
    for(double t=0;t<=trajectory.getTimeSum();t+=.02)
      EXPECT_TRUE(planner.isWatchedFappPositionSafe(trajectory.evaluateDeBoorT(t)));
    const Vector3d end=path.back(),mid=path[path.size()/2];
    const double side=manager.manual_fapp_curve_side_;
    ASSERT_TRUE(manager.buildManualFappUnderpassPath(mid,goal,path));
    EXPECT_NEAR(manager.manual_fapp_curve_side_,side,1e-9);
    EXPECT_LT((end-path.back()).norm(),1e-9);
  }

  static void runCloseFappDescent(int scenario) {
    ros::NodeHandle nh("~close_fapp_descent");
    nh.setParam("mission/task_search/min_height",0.10);
    nh.setParam("manager/footprint_bottom_clearance",0.20);
    nh.setParam("fapp/enabled",true);
    nh.setParam("fapp/memory_enabled",true);
    nh.setParam("fapp/planner_priority",true);
    FastExplorationManager manager;
    initialize(nh,manager,true);
    auto& planner=*manager.planner_manager_;
    auto& map=*manager.sdf_map_;
    for(int x=0;x<map.mp_->map_voxel_num_.x();++x)
      for(int y=0;y<map.mp_->map_voxel_num_.y();++y)
        for(int z=0;z<map.mp_->map_voxel_num_.z();++z) {
          const Eigen::Vector3i id(x,y,z); Vector3d p; map.indexToPos(id,p);
          // 前方高处横梁挡住斜下降，但起点下降柱及低位水平通道可通。
          const bool beam=scenario==6&&p.x()>-.65&&p.x()<-.50&&p.z()>=.55&&p.z()<=.85;
          const bool blocked=((scenario==1||scenario==2)&&p.z()<.30)||beam;
          map.md_->occupancy_buffer_[map.toAddress(id)]=blocked
              ? (scenario==1 ? map.mp_->clamp_min_log_-map.mp_->unknown_flag_ : map.mp_->clamp_max_log_)
              : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[map.toAddress(id)]=blocked&&(scenario==2||beam);
        }
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_=map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    map.updateESDF3d();
    planner.fapp_watched_valid_=true;
    planner.fapp_watched_lower_=Vector3d(-.30,scenario==8?-1.8:-.40,(scenario==4||scenario==8)?.12:.40);
    planner.fapp_watched_upper_=Vector3d(.30,scenario==8?1.8:.40,.90);
    planner.fapp_watched_state_.stamp=ros::Time::now().toSec();
    manager.manual_path_enabled_=manager.manual_path_exploration_style_=true;
    manager.manual_path_wait_for_trigger_=true;
    manager.startManualWaypointMission();
    manager.manual_path_use_astar_=true;
    manager.manual_path_max_segment_length_=1.2;
    manager.manual_path_.points={Vector3d(scenario==5?.20:.80,0,.65),Vector3d(1.,0,.65)};
    manager.manual_fapp_active_=true;
    manager.manual_fapp_forward_=Vector3d::UnitX();
    manager.manual_fapp_hold_height_=.65;
    const Vector3d start(scenario==3?0.:(scenario==5?-1.:(scenario==6?-.95:-.425)),0,.65),zero=Vector3d::Zero();
    if(scenario>=1&&scenario<=4) {
      EXPECT_FALSE(manager.planFappVerticalDescent(start,zero));
      EXPECT_FALSE(manager.vertical_detour_plan_);
      return;
    }
    if(scenario==8) {
      // 完整穿越暂不可用时，复用探索普通前进回退，不横移等待。
      manager.manual_path_use_astar_=false;
      EXPECT_EQ(SUCCEED,manager.planExploreMotion(start,zero,zero,zero));
      EXPECT_TRUE(manager.fapp_crossing_retry_needed_);
      EXPECT_GT(manager.ed_->next_goal_.x(),start.x()+.20);
      EXPECT_NEAR(manager.ed_->next_goal_.y(),start.y(),1e-6);
      EXPECT_EQ(0u,manager.manual_path_.next);
      return;
    }
    if(scenario==6) {
      std::vector<Vector3d> blocked_path;
      EXPECT_FALSE(manager.buildManualFappUnderpassPath(start,manager.manual_path_.points.front(),blocked_path));
      EXPECT_TRUE(planner.isKnownFreePosition(start));
    }
    if(scenario==7) {
      ASSERT_TRUE(planner.planExploreTraj({start,Vector3d(0,0,.65),Vector3d(.8,0,.65)},
                                         zero,zero,0.,true));
      FastExplorationFSM fsm;
      fsm.expl_manager_=std::shared_ptr<FastExplorationManager>(&manager,[](FastExplorationManager*){});
      fsm.planner_manager_=manager.planner_manager_;
      fsm.fd_=std::make_shared<FSMData>();
      fsm.fp_=std::make_shared<FSMParam>();
      fsm.fd_->state_str_={"INIT","WAIT","PLAN","PUB","EXEC","FINISH"};
      fsm.fd_->odom_pos_=start;
      fsm.fd_->odom_yaw_=0.;
      fsm.fp_->emergency_brake_horizon_=.12;
      fsm.fp_->tracking_error_grace_time_=1000.;
      fsm.fp_->max_tracking_error_xy_=1.;
      fsm.fp_->max_tracking_error_z_=1.;
      fsm.active_traj_=planner.local_data_;
      fsm.active_traj_.start_time_=ros::Time::now();
      fsm.active_traj_valid_=true;
      fsm.state_=EXEC_TRAJ;
      fsm.emergency_brake_pub_=nh.advertise<std_msgs::Int32>("brake",1);
      fsm.safetyCallback(ros::TimerEvent());
      EXPECT_FALSE(fsm.active_traj_braked_);
      EXPECT_EQ(EXEC_TRAJ,fsm.state_);
    }
    ASSERT_EQ(SUCCEED,manager.planExploreMotion(start,zero,zero,zero));
    EXPECT_EQ(0u,manager.manual_path_.next);
    if(scenario==5) {
      EXPECT_TRUE(manager.manual_fapp_curve_active_);
      EXPECT_NEAR(manager.ed_->next_goal_.x(),.20,1e-6);
      EXPECT_NEAR(manager.ed_->next_goal_.z(),.34,1e-6);
      manager.recordPublishedTrajectoryHeight();
      const Vector3d end=manager.ed_->next_goal_;
      EXPECT_FALSE(manager.manualFappObservationMissing(end,0.,true));
      EXPECT_TRUE(manager.manual_fapp_active_); // 到中间航点仍未通过整个包络。
      manager.manual_path_.next=1;
      ASSERT_EQ(SUCCEED,manager.planExploreMotion(end,zero,zero,zero));
      EXPECT_GT(manager.ed_->next_goal_.x(),.45);
      return;
    }
    EXPECT_TRUE(manager.vertical_detour_plan_);
    EXPECT_FALSE(manager.manual_fapp_curve_active_);
    const Vector3d live=start+Vector3d(-.01,.02,.01);
    ASSERT_TRUE(manager.reanchorVerticalDetourForRelease(live,zero));
    auto& curve=planner.local_data_.position_traj_;
    double previous_z=live.z();
    for(double t=0;t<=curve.getTimeSum();t+=.02) {
      const Vector3d p=curve.evaluateDeBoorT(t);
      EXPECT_NEAR(p.x(),live.x(),1e-6); EXPECT_NEAR(p.y(),live.y(),1e-6);
      EXPECT_LE(p.z(),previous_z+1e-6); previous_z=p.z();
      EXPECT_TRUE(planner.isKnownFreePosition(p));
      EXPECT_TRUE(planner.isWatchedFappPositionSafe(p));
    }
    manager.recordPublishedTrajectoryHeight();
    EXPECT_NEAR(manager.manual_fapp_hold_height_,.34,1e-6);
    const Vector3d end=manager.ed_->next_goal_;
    ASSERT_EQ(SUCCEED,manager.planExploreMotion(end,zero,zero,zero));
    EXPECT_FALSE(manager.vertical_detour_plan_);
    EXPECT_TRUE(manager.manual_fapp_curve_active_);
    EXPECT_GT(manager.ed_->next_goal_.x(),.45);
    EXPECT_EQ(0u,manager.manual_path_.next);
    EXPECT_TRUE(planner.isTrajectorySafe());
  }

  static void runManualSharedFappRetry(bool candidate_succeeds) {
    ros::NodeHandle nh("~manual_shared_fapp_retry");
    nh.setParam("mission/task_search/min_height",.10);
    nh.setParam("fapp/enabled",true);
    nh.setParam("fapp/memory_enabled",true);
    nh.setParam("fapp/planner_priority",true);
    FastExplorationManager manager;
    initialize(nh,manager,true);
    auto& planner=*manager.planner_manager_;
    auto& map=*manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_=map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    map.updateESDF3d();
    const Vector3d start(-1.4,0,.65), zero=Vector3d::Zero();
    manager.manual_path_enabled_=manager.manual_path_exploration_style_=true;
    manager.manual_path_use_astar_=false;
    manager.manual_path_.points={Vector3d(.8,0,.65)};
    manager.manual_path_max_segment_length_=1.2;
    manager.manual_actual_pos_=start;
    manager.manual_fapp_active_=true;
    manager.manual_fapp_hold_height_=.65;
    manager.manual_fapp_forward_=Vector3d::UnitX();
    planner.fapp_watched_valid_=true;
    planner.fapp_watched_lower_=Vector3d(-.30,-1.8,.12);
    planner.fapp_watched_upper_=Vector3d(.30,1.8,.90);
    planner.fapp_watched_state_.stamp=ros::Time::now().toSec();
    ASSERT_EQ(SUCCEED,manager.planExploreMotion(start,zero,zero,zero));
    ASSERT_GT(manager.ed_->next_goal_.x(),start.x()+.5);
    EXPECT_NEAR(manager.ed_->next_goal_.y(),0.,1e-6);
    EXPECT_NEAR(manager.ed_->next_goal_.z(),.65,1e-6);
    EXPECT_TRUE(manager.needsExplorationFappRetry(start));
    FastExplorationFSM fsm;
    fsm.expl_manager_=std::shared_ptr<FastExplorationManager>(&manager,[](FastExplorationManager*){});
    fsm.planner_manager_=manager.planner_manager_;
    fsm.fd_=std::make_shared<FSMData>();
    fsm.fp_=std::make_shared<FSMParam>();
    fsm.fd_->state_str_={"INIT","WAIT","PLAN","PUB","EXEC","FINISH"};
    fsm.fp_->replan_time_=.2;
    fsm.fd_->odom_pos_=start;
    fsm.fd_->odom_yaw_=0.;
    fsm.fd_->static_state_=false;
    fsm.active_traj_=planner.local_data_;
    fsm.active_traj_.start_time_=ros::Time::now();
    fsm.active_traj_valid_=true;
    fsm.state_=EXEC_TRAJ;
    fsm.narrow_corridor_stage_=false; // 航点入口不依赖自主探索阶段。
    const auto published=fsm.active_traj_.position_traj_.getControlPoint();
    EXPECT_FALSE(fsm.keepExecutingCorridorTrajectory()); // 不启用无关的宽路替换。
    ASSERT_TRUE(fsm.checkFappExecutionRetry());
    EXPECT_FALSE(fsm.checkFappExecutionRetry()); // 复用0.5秒重试节奏。
    fsm.fapp_replan_pending_=true;
    fsm.fd_->start_pt_=start;
    fsm.fd_->start_vel_=fsm.fd_->start_acc_=fsm.fd_->start_yaw_=zero;
    if(candidate_succeeds) {
      planner.fapp_watched_lower_=Vector3d(-.30,-.40,.40);
      planner.fapp_watched_upper_=Vector3d(.30,.40,.90);
      ASSERT_EQ(SUCCEED,fsm.callExplorationPlanner());
      EXPECT_TRUE(manager.manual_fapp_curve_active_);
      EXPECT_LT(manager.ed_->next_goal_.z(),.40);
      EXPECT_FALSE(manager.needsExplorationFappRetry(start));
    } else {
      EXPECT_EQ(FAIL,fsm.callExplorationPlanner());
    }
    EXPECT_TRUE(published.isApprox(fsm.active_traj_.position_traj_.getControlPoint()));
    ASSERT_TRUE(fsm.resumeAfterWidthAttempt("manual retry rejected before release"));
    EXPECT_EQ(EXEC_TRAJ,fsm.state_);
    EXPECT_FALSE(fsm.active_traj_braked_);
    EXPECT_TRUE(manager.needsExplorationFappRetry(start));
    EXPECT_TRUE(published.isApprox(fsm.active_traj_.position_traj_.getControlPoint()));
    EXPECT_EQ(0u,manager.manual_path_.next);
    EXPECT_TRUE(manager.manual_path_.points.front().isApprox(Vector3d(.8,0,.65)));
  }

  static void runBottomClearance(bool search) {
    ros::NodeHandle nh("~bottom_clearance");
    nh.setParam("manager/footprint_bottom_clearance", 0.20);
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.10);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    nh.setParam("astar/min_search_height", 0.65);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    for (int x=0; x<map.mp_->map_voxel_num_.x(); ++x)
      for (int y=0; y<map.mp_->map_voxel_num_.y(); ++y)
        for (int z=0; z<map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i id(x,y,z);
          Vector3d p; map.indexToPos(id,p);
          // 0.60m高的横梁；点检查时只放在机体右边缘，中心射线完全自由。
          const bool obstacle = std::fabs(p.x()) < .15 && p.z() < .60 &&
              (search || (p.y() > .10 && p.y() < .20));
          const int address = map.toAddress(id);
          map.md_->occupancy_buffer_[address] = obstacle ? map.mp_->clamp_max_log_ : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address] = obstacle;
        }
    if (!search) {
      EXPECT_EQ(map.getOccupancy(Vector3d(.025,.025,.725)), SDFMap::FREE);
      EXPECT_FALSE(planner.isPositionSafe(Vector3d(.025,.025,.725)));
      EXPECT_FALSE(planner.isKnownFreePosition(Vector3d(.025,.025,.775)));
      EXPECT_TRUE(planner.isKnownFreePosition(Vector3d(.025,.025,.825)));
      EXPECT_FALSE(planner.isPathSafe({Vector3d(-.5,.025,.725),Vector3d(.5,.025,.725)}));
      EXPECT_TRUE(planner.isPathSafe({Vector3d(-.5,.025,.825),Vector3d(.5,.025,.825)}));
      return;
    }
    const Vector3d start(-.6,.025,.725), goal(.6,.025,.725);
    const auto begin = ros::WallTime::now();
    ASSERT_EQ(planner.path_finder_->search(start,goal), Astar::REACH_END);
    std::cout << "bottom-clearance A* seconds=" << (ros::WallTime::now()-begin).toSec() << std::endl;
    const auto path = planner.path_finder_->getPath();
    EXPECT_TRUE(planner.isPathSafe(path));
    double crossing_height = 0.0;
    for (const auto& point : path)
      if (std::fabs(point.x()) < .20) crossing_height = std::max(crossing_height,point.z());
    EXPECT_GE(crossing_height,.80);
  }
  static void runRecordedTurn(const char* fixture) {
    ros::NodeHandle nh("~");
    // 与正式fuel_map.yaml的分辨率、融合概率和膨胀配置一致。
    nh.setParam("sdf_map/resolution", 0.05);
    nh.setParam("sdf_map/map_size_x", 8.0);
    nh.setParam("sdf_map/map_size_y", 8.0);
    nh.setParam("sdf_map/map_size_z", 3.0);
    nh.setParam("sdf_map/ground_height", -1.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.099);
    nh.setParam("sdf_map/max_ray_length", 8.0);
    nh.setParam("sdf_map/local_bound_inflate", 0.5);
    nh.setParam("sdf_map/local_map_margin", 100);
    nh.setParam("sdf_map/p_hit", 0.65);
    nh.setParam("sdf_map/p_max", 0.90);
    nh.setParam("sdf_map/inflation_noise_filter_enabled", true);
    nh.setParam("sdf_map/inflation_vertical_radius", 4);
    nh.setParam("sdf_map/virtual_ceil_height", -10.0);
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    std::ifstream input(fixture);
    int frames; Vector3d pos;
    ASSERT_TRUE(bool(input >> frames >> pos.x() >> pos.y() >> pos.z()));
    for (int frame=0; frame<frames; ++frame) {
      int count; Vector3d camera;
      ASSERT_TRUE(bool(input >> count >> camera.x() >> camera.y() >> camera.z()));
      pcl::PointCloud<pcl::PointXYZ> cloud; cloud.resize(count);
      for (auto& p : cloud) ASSERT_TRUE(bool(input >> p.x >> p.y >> p.z));
      map->inputPointCloud(cloud, cloud.size(), camera);
    }
    TaskSearchManager task;
    task.initialize(nh); task.setMap(map);
    task.exit_detection_enabled_ = false;
    task.setCorridorFrame(Vector3d(-4.28,2.69,0.60), Vector3d::UnitX());
    task.updateRobotPose(pos, -72.9*M_PI/180.0);
    task.visited_positions_.clear(); task.visited_positions_.push_back(pos);
    task.latest_turn_anchor_valid_ = true;
    task.latest_turn_anchor_ = Eigen::Vector2d(-0.06,2.22);
    task.latest_turn_incoming_direction_ = Eigen::Vector2d::UnitX();
    const double axis = -66.0*M_PI/180.0;
    const Vector3d travel(std::cos(axis),std::sin(axis),0);
    task.turn_detection_direction_ = travel.head<2>();
    task.segment_direction_ = travel.head<2>();
    task.segment_origin_ = task.latest_turn_anchor_;
    task.pending_turn_probe_origin_valid_ = true;
    task.pending_turn_probe_origin_ = Vector3d(0.20,0.13,pos.z());
    task.recovery_turn_min_angle_deg_ = 45.0;
    Vector3d direction; double old_free, branch_free;
    const bool found = task.inferOccupancyTurnDirection(travel,direction,old_free,branch_free);
    std::cout << "recorded turn found=" << found << " position=" << pos.transpose()
              << " old_free=" << old_free << " branch_free=" << branch_free
              << " direction=" << direction.transpose() << std::endl;
    ASSERT_TRUE(found);
    EXPECT_LT(travel.x()*direction.y()-travel.y()*direction.x(), -0.5);
    EXPECT_LT(direction.x(), -0.5);
  }

  static void runUpperTurn(int mode) {
    ros::NodeHandle nh("~upper_turn");
    nh.setParam("manager/footprint_bottom_clearance",.20);
    nh.setParam("manager/use_geometric_path",true);
    nh.setParam("astar/resolution_astar",.05);
    nh.setParam("astar/allocate_num",100000);
    nh.setParam("astar/lambda_heu",1.0);
    nh.setParam("astar/max_search_time",1.0);
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& task=*manager.task_search_manager_;
    auto& map=*manager.sdf_map_;
    const double side=mode==6 ? 1.0 : -1.0;
    const Vector3d pos(.45,0,.525), zero=Vector3d::Zero();
    task.setMap(manager.sdf_map_);
    task.global_no_return_=false;
    task.setCorridorFrame(Vector3d(-1,0,.525),Vector3d::UnitX());
    task.segment_direction_=Eigen::Vector2d::UnitX();
    task.turn_detection_direction_=Eigen::Vector2d::UnitX();
    task.stable_progress_direction_=Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_=true;
    task.updateRobotPose(pos,0);
    task.visited_positions_={Vector3d(.45,-side*.6,.525),Vector3d(.45,-side*.3,.525),pos};
    task.exit_footprint_extra_radius_=.20;
    task.exit_footprint_half_height_=.15;
    task.turn_upper_enabled_=mode!=4;
    for (int x=0; x<map.mp_->map_voxel_num_.x(); ++x)
      for (int y=0; y<map.mp_->map_voxel_num_.y(); ++y)
        for (int z=0; z<map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i id(x,y,z); Vector3d p; map.indexToPos(id,p);
          const double along=side*p.y();
          const bool hall=p.x()<.9 && std::fabs(p.y())<.55;
          const bool branch=std::fabs(p.x()-.45)<.45;
          const bool low_obstacle=mode!=3 && along>.30 && along<.65 && p.z()<.60;
          const bool roof=mode==2 && along>.30 && p.z()>.70;
          const bool occupied=(!hall && !branch) || low_obstacle || roof;
          const int address=map.toAddress(id);
          map.md_->occupancy_buffer_[address]=occupied?map.mp_->clamp_max_log_:map.mp_->clamp_min_log_;
          if (mode==1 && along>.30 && p.z()>=.60)
            map.md_->occupancy_buffer_[address]=map.mp_->clamp_min_log_-map.mp_->unknown_flag_;
          map.md_->occupancy_buffer_inflate_[address]=occupied;
        }
    Vector3d direction;
    const bool found=task.mappedCorridorDirection(0,direction,false,false,true);
    if (mode==1 || mode==2 || mode==3 || mode==4) {
      EXPECT_FALSE(found);
      EXPECT_FALSE(task.hasConfirmedCorridorTurn());
      if (mode==3) { // 普通可通行的转弯仍走原发布前确认流程。
        EXPECT_TRUE(task.mappedCorridorDirection(0,direction,false,false));
        EXPECT_GT(direction.y()*side,.9);
      }
      return;
    }
    ASSERT_TRUE(found);
    EXPECT_GT(direction.y()*side,.9);
    EXPECT_GE(task.committed_turn_upper_height_,.80);
    Vector3d target; double target_yaw=0;
    EXPECT_FALSE(manager.buildPostTurnUpperRecovery(pos,0,target,target_yaw));
    manager.completeTurnInPlace(pos);
    if (mode==5) { // 转向期间地图更新：原先可用的高层被堵，不能使用旧结论强行上绕。
      for (double x=-1.9; x<1.9; x+=.05)
        for (double y=-1.9; y<1.9; y+=.05)
          for (double z=.65; z<1.6; z+=.05) {
            Eigen::Vector3i id; map.posToIndex(Vector3d(x,y,z),id);
            map.md_->occupancy_buffer_[map.toAddress(id)]=map.mp_->clamp_max_log_;
          }
      EXPECT_FALSE(manager.buildPostTurnUpperRecovery(pos,side*M_PI_2,target,target_yaw));
      return;
    }
    ASSERT_TRUE(manager.buildPostTurnUpperRecovery(pos,side*M_PI_2,target,target_yaw));
    // 障碍仅在0.30m前方：旧A*弧线可绕过，但直接斜升会擦进底部包络。
    // 应保留当前位置先竖升，不能为满足水平位移断言而重新贴近障碍。
    EXPECT_NEAR(0.0,(target-pos).head<2>().norm(),1e-6);
    EXPECT_GE(target.z(),.80);
    EXPECT_TRUE(manager.isKnownSafeVerticalPath(pos,target.z()));
    EXPECT_TRUE(manager.planner_manager_->isPositionSafe(target));
    EXPECT_TRUE(manager.planner_manager_->isPathSafe({pos,target}));
    EXPECT_NEAR(target_yaw,side*M_PI_2,1e-9);
    if (mode == 7) {
      // 复现实飞：Y 远小于旧框的 0.70m，但已确认上层通道并完成转向。
      manager.ep_->mission_search_region_min_ = Vector3d(-1.2, .7, .2);
      manager.ep_->mission_search_region_max_ = Vector3d(6.2, 6.8, 1.8);
      manager.edt_environment_ = manager.planner_manager_->edt_environment_;
      nh.setParam("frontier/cluster_min", 1000000);
      manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
      manager.external_selection_only_ = true;
      ASSERT_FALSE(manager.mission_entered_search_region_);
      ASSERT_EQ(SUCCEED, manager.planExploreMotion(pos, zero, zero, Vector3d(side*M_PI_2,0,0)));
      EXPECT_FALSE(manager.mission_entered_search_region_);
      EXPECT_GE(manager.external_selected_viewpoint_.z(), .80);
      EXPECT_NEAR(0.0,(manager.external_selected_viewpoint_-pos).head<2>().norm(),1e-6);
    }
    EXPECT_FALSE(manager.buildPostTurnUpperRecovery(target,side*M_PI_2,target,target_yaw));
  }

  static void runCorridorHeightScope() {
    ros::NodeHandle nh("~corridor_height");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("mission/task_search/corridor_max_height", 1.50);
    nh.setParam("mission/task_search/max_height", 1.90);
    nh.setParam("mission/task_search/min_height", .10);
    nh.setParam("astar/max_search_height", 1.90);
    nh.setParam("astar/min_search_height", .10);
    nh.setParam("astar/resolution_astar", .05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    auto& finder = *manager.planner_manager_->path_finder_;
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),
              map.md_->occupancy_buffer_inflate_.end(), 0);
    const Vector3d start(.025,.025,1.425), high(.325,.025,1.725), low(.325,.025,.325);
    EXPECT_DOUBLE_EQ(1.50, task.clampSearchHeight(1.7));
    finder.reset();
    EXPECT_EQ(Astar::NO_PATH, finder.search(start, high));
    finder.reset();
    ASSERT_EQ(Astar::REACH_END, finder.search(start, Vector3d(.325,.025,1.475)));
    for (const auto& point : finder.getPath()) EXPECT_LE(point.z(), 1.501);
    // 不重启/重配规划器，阶段切换后原高层搜索和下降都可用。
    for (auto stage : {TaskSearchManager::EXIT_APPROACH_INSIDE, TaskSearchManager::CROSS_EXIT,
                       TaskSearchManager::SEARCH_OUTSIDE_LANDING, TaskSearchManager::APPROACH_LANDING,
                       TaskSearchManager::LANDING}) {
      task.mission_stage_ = stage;
      EXPECT_DOUBLE_EQ(1.7, task.clampSearchHeight(1.7));
      finder.reset();
      EXPECT_EQ(Astar::REACH_END, finder.search(start, high));
      finder.reset();
      EXPECT_EQ(Astar::REACH_END, finder.search(high, low));
      EXPECT_FALSE(manager.shouldStartGroundAscent(low));
    }
    task.mission_stage_ = TaskSearchManager::SEARCH_CORRIDOR;
    finder.reset();
    EXPECT_EQ(Astar::NO_PATH, finder.search(start, high));
    nh.deleteParam("mission");
    nh.deleteParam("astar");
  }

  static void runTurnHistory(int mode) {
    TaskSearchManager task;
    Eigen::Vector2d anchor(0,0), direction(0,1);
    bool expected = true;
    // 来路全部落在原来豁免的70cm内，且只需两个历史点就能确认反向重走。
    task.visited_positions_ = {Vector3d(0,.6,1),Vector3d(0,0,1)};
    if (mode == 1) { direction = Eigen::Vector2d(0,-1); expected = false; }
    if (mode == 2) {
      task.visited_positions_ = {Vector3d(-.6,0,1),Vector3d(0,0,1)};
      expected = false;  // 从共同起点横移，不能仅凭距离近判回头。
    }
    if (mode == 3) {
      anchor.x() = .8; expected = false; // 倒U另一条通道方向虽相反，空间不重合。
    }
    if (mode == 4) {
      task.visited_positions_ = {Vector3d(0,.08,1),Vector3d(0,0,1)};
      expected = false; // 起点处极短重合不等于沿来路返回。
    }
    if (mode == 5) {
      task.visited_positions_ = {Vector3d(0,2,1),Vector3d(0,-1,1)};
      // 稀疏采样点位于探测范围外，但它们之间的实际轨迹穿过候选。
    }
    if (mode == 6) {
      task.visited_positions_ = {Vector3d(0,-.6,1),Vector3d(0,0,1)};
      direction.y() = -1;
    }
    const auto original = task.visited_positions_;
    EXPECT_EQ(task.turnDirectionRetracesVisitedRoute(anchor,direction),expected);
    ASSERT_EQ(task.visited_positions_.size(),original.size());
    for (size_t i=0; i<original.size(); ++i)
      EXPECT_EQ((task.visited_positions_[i]-original[i]).norm(),0.0);
  }

  static void runRecordedTurnHistory(const char* fixture) {
    TaskSearchManager task;
    std::ifstream input(fixture);
    Vector3d point;
    while (input >> point.x() >> point.y() >> point.z()) {
      if (task.visited_positions_.empty() ||
          (point-task.visited_positions_.back()).head<2>().norm() >= .35)
        task.visited_positions_.push_back(point);
    }
    ASSERT_GT(task.visited_positions_.size(),4u);
    const Eigen::Vector2d anchor(5.99,-1.36);
    const double yaw = 87.6*M_PI/180.0;
    EXPECT_TRUE(task.turnDirectionRetracesVisitedRoute(
        anchor,Eigen::Vector2d(std::cos(yaw),std::sin(yaw))));
    EXPECT_FALSE(task.turnDirectionRetracesVisitedRoute(
        anchor,Eigen::Vector2d(-std::cos(yaw),-std::sin(yaw))));
  }

  static void runPassableTurn(int mode) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.recovery_turn_min_angle_deg_ = 45.0;
    task.recovery_turn_max_angle_deg_ = 120.0;
    task.recovery_turn_probe_length_ = 1.5;
    task.exit_footprint_extra_radius_ = 0.22;
    task.exit_footprint_half_height_ = 0.15;
    const bool mirror = mode == 1 || mode == 4;
    // 从入口段沿+Y走到转角，再实飞沿+X推进；镜像场景自动得到反向绕行先验。
    const Vector3d entry(0.0, mirror ? -1.0 : 1.0, 0.0);
    task.setCorridorFrame(Vector3d(-1,-1,1), entry);
    task.latest_turn_anchor_valid_ = mode != 9;
    task.latest_turn_anchor_ = Eigen::Vector2d(-1.1,0);
    task.latest_turn_incoming_direction_ = entry.head<2>();
    task.turn_detection_direction_ = Eigen::Vector2d::UnitX();
    task.segment_direction_ = Eigen::Vector2d::UnitX();
    task.segment_origin_ = Eigen::Vector2d(-1.1,0);
    task.updateRobotPose(Vector3d(-0.1,0,1),0.0);
    task.visited_positions_.clear();
    task.visited_positions_.push_back(Vector3d(-1.1,0,1));
    task.visited_positions_.push_back(Vector3d(-0.1,0,1));
    if (mode >= 10) {
      task.updateRobotPose(Vector3d(.45,0,1),0.0);
      task.visited_positions_ = {Vector3d(.45,.60,1),Vector3d(.45,.30,1),Vector3d(.45,0,1)};
      // 冻结的轮廓观察点不等于实际转弯起点。
      task.pending_turn_probe_origin_valid_ = true;
      task.pending_turn_probe_origin_ = Vector3d(-.1,0,1);
    }
    double right_width = (mode == 2 ? 0.65 : (mode == 11 ? .20 : .90));
    double left_width = (mode == 2 || mode == 3 || mode == 4 || mode >= 10 ? 0.90 : 0.20);
    if (mode == 5) right_width = left_width = 0.20;
    if (mirror) std::swap(right_width, left_width);
    for (int x=0; x<map.mp_->map_voxel_num_.x(); ++x)
      for (int y=0; y<map.mp_->map_voxel_num_.y(); ++y)
        for (int z=0; z<map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i id(x,y,z);
          Vector3d p; map.indexToPos(id,p);
          const bool hall = p.x() < 0.90 && std::fabs(p.y()) < 0.55;
          const double width = p.y() < 0 ? right_width : left_width;
          bool branch = std::fabs(p.x()-0.45) < 0.5*width;
          // 看得到远处宽阔区域，但连接处被墙/低顶/未知体素切断。
          if (mode == 6 && p.y() < -0.50 && p.y() > -0.65) branch = false;
          if (mode == 8 && p.y() < -0.55 && p.z() > 1.05) branch = false;
          const int address = map.toAddress(id);
          double occ = hall || branch ? map.mp_->clamp_min_log_ : map.mp_->clamp_max_log_;
          if (mode == 7 && p.y() < -0.55) occ = map.mp_->clamp_min_log_ - map.mp_->unknown_flag_;
          map.md_->occupancy_buffer_[address] = occ;
          map.md_->occupancy_buffer_inflate_[address] = occ > map.mp_->min_occupancy_log_;
        }
    Vector3d selected; double old_free, branch_free;
    const bool found = task.inferOccupancyTurnDirection(
        Vector3d::UnitX(), selected, old_free, branch_free);
    if ((mode >= 5 && mode <= 8) || mode == 11) {
      EXPECT_FALSE(found) << "slit, disconnected, unknown or low-roof branch was accepted";
      return;
    }
    ASSERT_TRUE(found);
    const bool expect_left = mode == 1 || mode == 2 || mode == 4;
    EXPECT_GT((expect_left ? 1.0 : -1.0)*selected.y(), 0.70)
        << "direction=" << selected.transpose();
    EXPECT_GE(branch_free, 1.0-1e-6);
  }

  static void runFappTallOutlier() {
    FastPlannerManager planner;
    planner.fapp_config_.enabled = true;
    planner.fapp_config_.vehicle_top_height = 0.05;
    planner.fapp_memory_enabled_ = true;
    planner.fapp_frame_ = "UAV0/camera_init";
    auto message = boost::make_shared<obj_state_msgs::ObjectsStates>();
    message->header.frame_id = planner.fapp_frame_;
    obj_state_msgs::State ball;
    ball.position.x = 3.15;
    ball.position.y = 0.32;
    ball.position.z = 0.592;
    ball.size.x = 0.08;
    ball.size.y = 0.16;
    ball.size.z = 0.158;
    const double now = ros::Time::now().toSec();
    for (int i = 0; i < 4; ++i) {
      message->header.stamp.fromSec(now - 0.5 + 0.1*i);
      message->states = {ball};
      planner.fappCallback(message);
    }
    planner.watchFappObstacle(&planner.fapp_states_.front());
    const Vector3d lower = planner.fapp_watched_lower_;
    // 21:31录包中的异常大框：中心在高度范围内，但底部被拉至0.098m。
    ball.position.x = 3.177462;
    ball.position.y = 0.468733;
    ball.position.z = 0.476641;
    ball.size.z = 0.757740;
    message->header.stamp.fromSec(now - 0.15);
    message->states = {ball};
    planner.fappCallback(message);
    ASSERT_EQ(1u, planner.fapp_states_.size());
    EXPECT_EQ(4u, planner.fapp_history_.size());
    EXPECT_TRUE(planner.fapp_watched_lower_.isApprox(lower));
    EXPECT_NEAR(0.158, planner.fapp_states_.front().size.z(), 1e-9);
    // 尺寸正常但明显更低的点保留原始观测，三次独立观测后更新下沿。
    ball.size.z = 0.158;
    message->header.stamp.fromSec(now - 0.10);
    message->states = {ball};
    planner.fappCallback(message);
    EXPECT_EQ(5u, planner.fapp_history_.size());
    EXPECT_NEAR(lower.z(), planner.fapp_watched_lower_.z(), 1e-9);
    for (double dt : {-0.05, 0.0}) {
      message->header.stamp.fromSec(now + dt);
      planner.fappCallback(message);
    }
    EXPECT_LT(planner.fapp_watched_lower_.z(), lower.z() - 0.10);
    // 其他位置的目标不受这个区域的尺寸约束。
    ball.position.x = 5.0;
    ball.size.z = 0.758;
    message->states = {ball};
    planner.fappCallback(message);
    EXPECT_EQ(2u, planner.fapp_states_.size());
  }

  static void runLateHistoricalOutlier() {
    FastPlannerManager planner;
    planner.fapp_config_.enabled = true;
    planner.fapp_config_.vehicle_top_height = 0.05;
    planner.fapp_memory_enabled_ = true;
    planner.fapp_frame_ = "UAV0/camera_init";
    auto message = boost::make_shared<obj_state_msgs::ObjectsStates>();
    message->header.frame_id = planner.fapp_frame_;
    obj_state_msgs::State ball;
    // 14:23:56录包：尺寸证据不足时收入的旧异常框。
    ball.position.x = 3.473151; ball.position.y = 0.582865; ball.position.z = 0.382162;
    ball.size.x = 0.171854; ball.size.y = 0.013825; ball.size.z = 0.421173;
    const double now = ros::Time::now().toSec();
    message->header.stamp.fromSec(now - 0.5);
    message->states = {ball};
    planner.fappCallback(message);
    ASSERT_EQ(1u, planner.fapp_history_.size()); // 不能靠新帧过滤掩盖旧帧合并漏洞。
    ball.position.x = 3.30; ball.position.y = 0.20; ball.position.z = 0.62;
    ball.size.x = 0.08; ball.size.y = 0.16; ball.size.z = 0.158;
    for (int i=0; i<4; ++i) {
      message->header.stamp.fromSec(now - 0.4 + i*0.05);
      message->states = {ball};
      planner.fappCallback(message);
    }
    auto initial = planner.fapp_history_.back();
    initial.position = Vector3d(3.09,-0.02,0.62); // 起初旧异常框还不在关联范围。
    planner.watchFappObstacle(&initial);
    const Vector3d confirmed_lower = planner.fapp_watched_lower_;
    ball.position.y = 0.30; // 摆动后，旧异常框进入新位置的历史关联范围。
    message->header.stamp.fromSec(now-0.15);
    message->states = {ball};
    planner.fappCallback(message);
    EXPECT_NEAR(confirmed_lower.z(), planner.fapp_watched_lower_.z(), 1e-9);
    EXPECT_GT(planner.fapp_watched_lower_.z()-0.06, 0.25);
    // 尺寸正常的低点保留，确认后仍记住真实最低点。
    ball.position.z = 0.44;
    message->header.stamp.fromSec(now-0.10);
    message->states = {ball};
    planner.fappCallback(message);
    EXPECT_NEAR(confirmed_lower.z(), planner.fapp_watched_lower_.z(), 1e-9);
    for (double dt : {-0.05, 0.0}) {
      message->header.stamp.fromSec(now + dt);
      planner.fappCallback(message);
    }
    EXPECT_NEAR(0.44-0.158/2-0.05-0.10, planner.fapp_watched_lower_.z(), 1e-6);
    const double real_low = planner.fapp_watched_lower_.z();
    planner.fapp_history_.clear();
    message->states.clear();
    planner.fappCallback(message);
    EXPECT_DOUBLE_EQ(real_low, planner.fapp_watched_lower_.z());
  }

  static void runBottomConfirmation(bool low_first) {
    FastPlannerManager planner;
    planner.fapp_config_.enabled = true;
    planner.fapp_config_.vehicle_top_height = 0.05;
    planner.fapp_memory_enabled_ = true;
    planner.fapp_frame_ = "UAV0/camera_init";
    auto message = boost::make_shared<obj_state_msgs::ObjectsStates>();
    message->header.frame_id = planner.fapp_frame_;
    obj_state_msgs::State normal, low;
    normal.position.x = 3.2169; normal.position.y = 0.2567; normal.position.z = 0.5385;
    normal.size.x = 0.08; normal.size.y = 0.16; normal.size.z = 0.1432;
    // 17:04录包中尺寸未超大框阈值、却将下穿目标拉至0.009m的旧框。
    low.position.x = 3.664521; low.position.y = 0.590193; low.position.z = 0.358428;
    low.size.x = 0.195341; low.size.y = 0.014231; low.size.z = 0.278143;
    const double now = ros::Time::now().toSec();
    const auto feed = [&](const obj_state_msgs::State& state, double dt) {
      message->header.stamp.fromSec(now + dt);
      message->states = {state};
      planner.fappCallback(message);
    };
    const double normal_lower = normal.position.z - normal.size.z/2 - .15;
    const double low_lower = low.position.z - low.size.z/2 - .15;
    if (low_first) {
      feed(low, -.7);
      planner.watchFappObstacle(&planner.fapp_states_.front());
      ASSERT_NEAR(low_lower, planner.fapp_watched_lower_.z(), 1e-6);
      feed(normal, -.65);
      EXPECT_NEAR(low_lower, planner.fapp_watched_lower_.z(), 1e-6);
    }
    for (int i=0; i<6; ++i) feed(normal, -.6 + .05*i);
    if (!low_first) planner.watchFappObstacle(&planner.fapp_states_.front());
    // 已合入的孤立低点也能撤销，而不是永远保留旧cwiseMin。
    ASSERT_NEAR(normal_lower, planner.fapp_watched_lower_.z(), 1e-6);
    if (low_first) {
      planner.watchFappObstacle(nullptr);
      EXPECT_TRUE(planner.fapp_watched_bottom_history_.empty());
      return;
    }
    feed(low, -.25);
    const size_t source_count = planner.fapp_watched_bottom_history_.size();
    ASSERT_NEAR(normal_lower, planner.fapp_watched_lower_.z(), 1e-6);
    ASSERT_EQ(7u, planner.fapp_history_.size()); // 仍保留该原始观测。
    for (int i=0; i<10; ++i) feed(low, -.25);
    EXPECT_EQ(source_count, planner.fapp_watched_bottom_history_.size());
    EXPECT_NEAR(normal_lower, planner.fapp_watched_lower_.z(), 1e-6);
    feed(low, -.24);
    feed(low, -.23);
    EXPECT_NEAR(normal_lower, planner.fapp_watched_lower_.z(), 1e-6);
    feed(low, -.15);
    EXPECT_NEAR(normal_lower, planner.fapp_watched_lower_.z(), 1e-6);
    feed(low, -.05);
    EXPECT_NEAR(low_lower, planner.fapp_watched_lower_.z(), 1e-6);
    // 已确认的最低点不能因活动历史清空、失观或重新摆高而丢失。
    planner.fapp_history_.clear();
    message->states.clear();
    planner.fappCallback(message);
    feed(normal, 0.0);
    EXPECT_NEAR(low_lower, planner.fapp_watched_lower_.z(), 1e-6);
    auto unrelated = low; unrelated.position.x = 8.0; unrelated.position.z = .1;
    feed(unrelated, .01);
    EXPECT_NEAR(low_lower, planner.fapp_watched_lower_.z(), 1e-6);
  }

  static void checkManualFappExecution(FastExplorationManager& manager, const Vector3d& start) {
    FastExplorationFSM fsm;
    fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(
        &manager, [](FastExplorationManager*) {});
    fsm.planner_manager_ = manager.planner_manager_;
    fsm.fd_ = std::make_shared<FSMData>();
    fsm.fp_ = std::make_shared<FSMParam>();
    fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
    fsm.fp_->tracking_error_grace_time_ = 1000.0;
    fsm.fp_->max_tracking_error_xy_ = 1.0;
    fsm.fp_->max_tracking_error_z_ = 1.0;
    fsm.fp_->emergency_brake_horizon_ = 0.12;
    fsm.active_traj_valid_ = true;
    fsm.active_traj_ = manager.planner_manager_->local_data_;
    fsm.active_traj_.start_time_ = ros::Time::now();
    fsm.state_ = EXEC_TRAJ;
    fsm.fd_->odom_pos_ = start;
    fsm.fd_->odom_yaw_ = 0.0;
    ros::NodeHandle nh("~manual_execution");
    fsm.emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("brake", 1);
    for (int i = 0; i < 3; ++i) {
      fsm.last_suspended_underpass_check_ = ros::Time();
      fsm.safetyCallback(ros::TimerEvent());
      EXPECT_EQ(EXEC_TRAJ, fsm.state_);
      EXPECT_FALSE(fsm.active_traj_braked_);
    }
    // 下一段暂未就绪且里程计掉高时，等待不能覆盖原轨迹终点。
    fsm.state_ = PLAN_TRAJ;
    fsm.active_traj_.start_time_ = ros::Time::now() -
        ros::Duration(fsm.active_traj_.duration_ + 0.1);
    fsm.fd_->odom_pos_ = fsm.active_traj_.position_traj_.evaluateDeBoorT(
        fsm.active_traj_.duration_);
    fsm.fd_->odom_pos_.z() -= 0.10;
    fsm.next_plan_retry_time_ = ros::Time::now() + ros::Duration(10.0);
    fsm.endpoint_hold_pub_ = nh.advertise<std_msgs::Bool>("endpoint_hold", 1);
    const Vector3d endpoint = fsm.active_traj_.position_traj_.evaluateDeBoorT(
        fsm.active_traj_.duration_);
    fsm.FSMCallback(ros::TimerEvent());
    EXPECT_FALSE(fsm.active_traj_braked_);
    EXPECT_FALSE(fsm.endpoint_hold_active_);
    EXPECT_TRUE(endpoint.isApprox(fsm.active_traj_.position_traj_.evaluateDeBoorT(
        fsm.active_traj_.duration_)));
  }

  static void runFappMemory() {
    ros::NodeHandle nh("~memory_history");
    FastPlannerManager planner;
    planner.fapp_config_.enabled = true;
    planner.fapp_memory_enabled_ = true;
    planner.fapp_frame_ = "UAV0/camera_init";
    planner.pp_.max_vel_ = 0.4;
    auto message = boost::make_shared<obj_state_msgs::ObjectsStates>();
    message->header.frame_id = planner.fapp_frame_;
    message->header.stamp = ros::Time::now();
    obj_state_msgs::State ball;
    ball.position.z = 1.0;
    ball.size.x = ball.size.y = ball.size.z = 0.2;
    message->states = {ball, ball};
    message->states[1].position.x = 2.0;
    planner.fappCallback(message);
    ASSERT_EQ(2u, planner.fapp_states_.size());
    const double first_stamp = planner.fapp_states_[0].stamp;
    planner.watchFappObstacle(&planner.fapp_states_[0]);
    message->states.clear();
    planner.fappCallback(message);
    ASSERT_EQ(2u, planner.fapp_states_.size());
    EXPECT_DOUBLE_EQ(first_stamp, planner.fapp_states_[0].stamp);
    EXPECT_FALSE(planner.isFappPathSafe({Vector3d(-1,0,1), Vector3d(1,0,1)}));

    ros::WallDuration(0.01).sleep();
    message->header.stamp = ros::Time::now();
    ball.position.x = 0.1;
    message->states = {ball};
    planner.fappCallback(message);
    ASSERT_EQ(2u, planner.fapp_states_.size());
    EXPECT_DOUBLE_EQ(0.1, planner.fapp_states_[0].position.x());
    EXPECT_DOUBLE_EQ(first_stamp, planner.fapp_states_[1].stamp);
    const double updated_stamp = planner.fapp_states_[0].stamp;
    // 空帧、乱序和错误坐标系都不能给旧障碍续期。
    message->header.stamp.fromSec(first_stamp);
    planner.fappCallback(message);
    EXPECT_DOUBLE_EQ(updated_stamp, planner.fapp_states_[0].stamp);
    message->header.stamp = ros::Time::now();
    message->header.frame_id = "wrong_frame";
    planner.fappCallback(message);
    EXPECT_DOUBLE_EQ(updated_stamp, planner.fapp_watched_state_.stamp);
    message->header.frame_id = planner.fapp_frame_;
    message->states[0].position.x = std::numeric_limits<double>::quiet_NaN();
    planner.fappCallback(message);
    EXPECT_DOUBLE_EQ(updated_stamp, planner.fapp_watched_state_.stamp);

    ros::WallDuration(0.85).sleep();
    message->states.clear();
    message->header.stamp = ros::Time::now();
    planner.fappCallback(message);
    EXPECT_TRUE(planner.fapp_states_.empty());
    Vector3d lower, upper;
    double age;
    ASSERT_TRUE(planner.watchedFappBounds(lower, upper, age));
    EXPECT_GT(age, 0.8); // 新鲜预测过期，历史包络仍约束路径。
    EXPECT_FALSE(planner.isWatchedFappPositionSafe(Vector3d(0,0,1)));
    EXPECT_TRUE(planner.isWatchedFappPositionSafe(Vector3d(0,0,0.3)));
    const double historical_bottom = lower.z();
    ball.position.x = 2.0;
    message->states = {ball};
    planner.fappCallback(message);
    EXPECT_DOUBLE_EQ(updated_stamp, planner.fapp_watched_state_.stamp);
    ball.position.x = 0.15;
    ball.position.z += 0.15;
    message->states = {ball};
    planner.fappCallback(message);
    EXPECT_DOUBLE_EQ(0.15, planner.fapp_watched_state_.position.x());
    EXPECT_GT(planner.fapp_watched_state_.stamp, updated_stamp);
    ASSERT_TRUE(planner.watchedFappBounds(lower, upper, age));
    EXPECT_DOUBLE_EQ(historical_bottom, lower.z()); // 球升高不能抬高历史最低点。
    EXPECT_FALSE(planner.isFappPathSafe({Vector3d(-1,0,1), Vector3d(1,0,1)}));
    planner.watchFappObstacle(nullptr);
    EXPECT_FALSE(planner.watchedFappBounds(lower, upper, age));
  }

  static void runManualSuspended(const std::string& scenario) {
    ros::NodeHandle nh("~manual_suspended_" + scenario);
    const bool disabled = scenario == "disabled";
    const bool recorded = scenario == "recorded_history";
    const bool committed = scenario == "distant_committed";
    const bool inside = scenario == "inside_history" || scenario == "inside_history_strict";
    const bool approach = scenario == "distant_blocked" || committed || inside;
    const bool recovery_roof = scenario == "exploration_recovery_roof";
    const bool prior_side = scenario == "exploration_prior_side";
    const bool prior_right_side = scenario == "exploration_prior_right_side";
    const bool low_minimum = scenario == "exploration_low_minimum";
    const bool entry_retry = scenario == "exploration_entry_retry";
    const bool exploration = scenario == "exploration_low" || scenario == "exploration_side" ||
        recovery_roof || prior_side || prior_right_side || low_minimum || entry_retry;
    const bool asymmetric_side = scenario == "asymmetric_side" || scenario == "exploration_side" || prior_right_side;
    const bool asymmetric = scenario == "asymmetric_low" || scenario == "exploration_low" || asymmetric_side || recovery_roof || prior_side || low_minimum || entry_retry;
    const bool memory = scenario == "memory_below" || scenario == "memory_left" ||
        scenario == "memory_curve_right" || recorded || approach || asymmetric;
    const bool stale = scenario == "stale";
    const bool mapped = scenario == "mapped";
    const bool mapped_ball = scenario == "mapped_ball" || scenario == "mapped_ball_left";
    const bool left_blocked = scenario == "right" || scenario == "blocked" ||
        scenario == "memory_curve_right";
    const bool right_blocked = scenario == "left" || scenario == "blocked" ||
        scenario == "mapped_ball_left" || scenario == "memory_left";
    const bool floor_blocked = (left_blocked || right_blocked) && scenario != "memory_curve_right";
    const bool unknown_below = scenario == "unknown_below";
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.10);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    nh.setParam("mission/task_search/min_height", asymmetric ? 0.15 : 0.25);
    nh.setParam("mission/task_search/max_height", 1.90);
    nh.setParam("fapp/enabled", !disabled && !mapped);
    nh.setParam("fapp/planner_priority", scenario != "inside_history_strict" && scenario != "blocked");
    nh.setParam("fapp/memory_enabled", memory);
    if (approach || asymmetric) {
      nh.setParam("fapp/vehicle_top_height", 0.05);
      nh.setParam("fapp/vehicle_bottom_height", 0.20);
    }
    const std::string topic = nh.resolveName("states");
    nh.setParam("fapp/topic", topic);
    FastExplorationManager manager;
    initialize(nh, manager, true);
    manager.manual_path_enabled_ = true;
    manager.manual_path_use_astar_ = true;
    manager.manual_path_exploration_style_ = true;
    manager.manual_path_max_segment_length_ = scenario == "memory_left" ? 0.15 : 1.2;
    if (asymmetric) manager.underpass_min_height_ = 0.15;
    // 此用例专测水平侧绕的失观续接；低位高度不允许，不能改走新增的宽偏移下穿。
    if (scenario == "memory_left") manager.underpass_min_height_ = 0.70;
    if (low_minimum) {
      manager.underpass_min_height_ = 0.10;
      manager.task_search_manager_->min_search_height_ = 0.10;
    }
    if (prior_side || prior_right_side) {
      manager.recovery_side_latched_ = true;
      manager.recovery_side_dir_ = (prior_right_side ? -1.0 : 1.0) * Vector3d::UnitY();
    }
    if (committed) {
      manager.recovery_side_latched_ = true;
      // 旧侧向已与当前前进方向不符，续接失败也不能提前否决正常规划。
      manager.recovery_side_dir_ = Vector3d::UnitX();
    }
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.task_search_manager_->setMap(manager.sdf_map_);
    manager.task_search_manager_->exit_detection_enabled_ = false;
    manager.edt_environment_ = manager.planner_manager_->edt_environment_;
    ViewNode::caster_.reset(new RayCaster);
    Vector3d origin, size;
    manager.sdf_map_->getRegion(origin, size);
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), origin);
    const Vector3d start(mapped || mapped_ball ? 0.025 : (recorded || approach || asymmetric ? -1.375 : -0.675),
                         0.025, recorded || asymmetric ? 0.582 : 1.025);
    const Vector3d goal(mapped_ball ? 1.650 : 1.425, 0.025, start.z()), zero = Vector3d::Zero();
    manager.manual_path_.points = {goal};
    auto& map = *manager.sdf_map_;
    // 低空用未截断的射线建立自由观测，避免地图忽略Z<0.2的超量程射线。
    const double ray_origin_x = asymmetric ? 0.9 : 1.9;
    for (double y = -1.725; y < 1.75; y += 0.05)
      for (double z = unknown_below ? 0.875 : (asymmetric ? 0.075 : 0.225); z < 1.65; z += 0.05) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1.9, y, z));
        map.inputPointCloud(cloud, 1, Vector3d(-ray_origin_x, y, z));
        cloud.front().x = -1.9;
        map.inputPointCloud(cloud, 1, Vector3d(ray_origin_x, y, z));
      }
    if (floor_blocked || mapped)
      for (double x = mapped ? 0.375 : (mapped_ball ? 0.525 : -0.10);
           x <= (mapped ? 0.525 : (mapped_ball ? 0.725 : 0.15)); x += 0.025)
        for (double y = -0.125; y <= 0.175; y += 0.025)
          for (double z = mapped ? 0.825 : 0.225;
               z <= (mapped ? 1.275 : 0.725); z += 0.025)
            map.setOccupied(Vector3d(x, y, z));
    if (mapped || mapped_ball) {
      // 悬挂分类读取原始占据证据；setOccupied 只修改膨胀层，需真实点云命中。
      pcl::PointCloud<pcl::PointXYZ> hanging;
      for (double x = mapped ? 0.375 : 0.525; x <= (mapped ? 0.525 : 0.725); x += 0.025)
        for (double y = -0.125; y <= 0.175; y += 0.025)
          for (double z = mapped ? 0.825 : 0.925; z <= (mapped ? 1.275 : 1.125); z += 0.025)
            hanging.push_back(pcl::PointXYZ(x, y, z));
      for (int repeat = 0; repeat < 8; ++repeat)
        map.inputPointCloud(hanging, hanging.size(), start);
    }
    for (double sign : {-1.0, 1.0}) {
      if ((sign > 0.0 && !left_blocked) || (sign < 0.0 && !right_blocked)) continue;
      for (double x = -1.0; x <= 1.0; x += 0.05)
        for (double y = 0.275; y <= 1.025; y += 0.05)
          for (double z = 0.225; z <= 1.625; z += 0.05)
            map.setOccupied(Vector3d(x, 0.025 + sign * y, z));
    }
    if (memory && (floor_blocked || scenario == "memory_curve_right")) {
      // 侧下方通路需要真实占据墙/底座，不能仅设置膨胀层而漏掉旋翼足迹。
      pcl::PointCloud<pcl::PointXYZ> occupied;
      if (floor_blocked) {
        for (double x=-0.10; x<=0.15; x+=0.025)
          for (double y=-0.125; y<=0.175; y+=0.025)
            for (double z=0.225; z<=0.725; z+=0.025)
              occupied.push_back(pcl::PointXYZ(x,y,z));
      } else {
        for (double x=-1.0; x<=1.4; x+=0.05)
          for (double z=0.225; z<=1.625; z+=0.05)
            occupied.push_back(pcl::PointXYZ(x,0.325,z));
      }
      for (int repeat=0; repeat<8; ++repeat)
        map.inputPointCloud(occupied, occupied.size(), start);
    }
    if (recovery_roof) {
      // 下穿区自由；只封住旧曲线在障碍后方强制回升的空间。
      for (double x=0.925; x<=1.225; x+=0.05)
        for (double y=-0.45; y<=0.45; y+=0.05)
          for (double z=0.425; z<=1.4; z+=0.05)
            map.setOccupied(Vector3d(x,y,z));
    }
    map.updateESDF3d();
    if (recovery_roof)
      ASSERT_FALSE(manager.planner_manager_->isPositionSafe(Vector3d(1.025,0.025,0.525)));
    auto pub = nh.advertise<obj_state_msgs::ObjectsStates>(topic, 1, true);
    if (!disabled && !mapped) {
      for (int i = 0; i < 50 && pub.getNumSubscribers() == 0; ++i) {
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
      }
      ASSERT_GT(pub.getNumSubscribers(), 0u);
    }
    obj_state_msgs::ObjectsStates message;
    message.header.frame_id = "UAV0/camera_init";
    obj_state_msgs::State ball;
    ball.position.x = mapped_ball ? 0.625 : 0.025;
    ball.position.y = 0.025;
    ball.position.z = scenario == "moving" ? 1.125 : 1.025;
    ball.velocity.z = scenario == "moving" ? -0.10 : 0.0;
    ball.size.x = ball.size.y = ball.size.z = 0.20;
    if (approach) {
      ball.position.x = inside ? start.x() + 0.10 : 0.225;
      ball.position.z = 0.70;
      ball.size.y = 2.0;
      ball.size.z = 1.20;
    }
    if (asymmetric) ball.position.z = asymmetric_side ? 0.40 : (low_minimum ? 0.43 : 0.54);
    message.states.push_back(ball);
    const auto send = [&]() {
      message.header.stamp = ros::Time::now() - ros::Duration(stale ? 2.0 : 0.0);
      pub.publish(message);
      for (int i = 0; i < 3; ++i) {
        ros::WallDuration(0.01).sleep();
        ros::spinOnce();
      }
    };
    if (recorded) {
      // 2026-10-22实飞包16:00:15.900的球高度与尺寸，平移XY放入测试地图。
      message.states[0].position.z = 0.5756615251302719;
      message.states[0].size.z = 0.30854132771492004;
      send();
      for (int i=0; i<2; ++i) {
        ros::WallDuration(0.03).sleep();
        send(); // 用独立时间的低点证据验证真实底部不足时仍走侧绕。
      }
      message.states[0].position.z = 0.60;
      message.states[0].size.z = 0.10;
    }
    send();
    Vector3d lower, upper;
    EXPECT_EQ(!disabled && !stale && !mapped,
              manager.planner_manager_->fappBlockingBounds({start, goal}, lower, upper));
    if (inside) {
      // 历史包络曾覆盖当前位置，之后已无新鲜目标；旧包络不能封死出路。
      manager.planner_manager_->watchFappObstacle(&manager.planner_manager_->fapp_states_.front());
      ros::WallDuration(0.85).sleep();
      message.states.clear();
      send();
    }
    if (exploration) {
      manager.manual_path_enabled_ = false;
      manager.fapp_exploration_crossing_enabled_ = true;
      manager.manual_path_.points.clear(); // 自主探索不能依赖人工航点列表。
    }
    if (entry_retry) {
      manager.ep_->mission_use_workspace_lock_ = true;
      manager.ep_->mission_global_no_return_ = true;
      manager.ep_->mission_door_back_margin_ = 0.0;
      manager.mission_workspace_lock_received_ = true;
      manager.mission_workspace_origin_ = start + Vector3d(0.04,0,0);
      manager.mission_workspace_dir_ = Vector3d::UnitX();
      ASSERT_FALSE(manager.planExplorationFappCrossing(start, zero, zero, zero));
      ASSERT_TRUE(manager.fapp_crossing_retry_needed_);
      auto& planner = *manager.planner_manager_;
      ASSERT_TRUE(planner.planExploreTraj({start, start + Vector3d(0.4,0,0),
                                          start + Vector3d(0.8,0,0)},
                                        zero, zero, 0.0, true));
      FastExplorationFSM fsm;
      fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(&manager, [](FastExplorationManager*) {});
      fsm.planner_manager_ = manager.planner_manager_;
      fsm.fd_ = std::make_shared<FSMData>();
      fsm.fp_ = std::make_shared<FSMParam>();
      fsm.fp_->replan_time_ = 0.2;
      fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
      fsm.active_traj_ = planner.local_data_;
      fsm.active_traj_.start_time_ = ros::Time::now();
      fsm.active_traj_valid_ = true;
      fsm.narrow_corridor_stage_ = true;
      fsm.fd_->odom_pos_ = start;
      fsm.fd_->static_state_ = false;
      fsm.state_ = EXEC_TRAJ;
      const auto published_points = fsm.active_traj_.position_traj_.getControlPoint();
      EXPECT_FALSE(fsm.checkFappExecutionRetry()); // 门外仍沿普通入口路线走。
      fsm.fd_->odom_pos_ = start + Vector3d(0.15,0,0);
      manager.manual_fapp_active_ = false;
      EXPECT_FALSE(fsm.checkFappExecutionRetry()); // 无动态会话不触发。
      manager.manual_fapp_active_ = true;
      manager.fapp_crossing_retry_needed_ = false;
      EXPECT_FALSE(fsm.checkFappExecutionRetry()); // 已成功的穿越不周期换轨。
      manager.fapp_crossing_retry_needed_ = true;
      ASSERT_TRUE(fsm.checkFappExecutionRetry());
      EXPECT_FALSE(fsm.checkFappExecutionRetry()); // 非100Hz连续重算。
      // 预测起点尚在门外时再次失败，仍保留真实已发布的普通轨迹。
      fsm.fapp_replan_pending_ = true;
      fsm.fd_->start_pt_ = start;
      fsm.fd_->start_vel_ = fsm.fd_->start_acc_ = fsm.fd_->start_yaw_ = zero;
      EXPECT_EQ(FAIL, fsm.callExplorationPlanner());
      ASSERT_TRUE(fsm.resumeAfterWidthAttempt("test temporary crossing rejection"));
      EXPECT_EQ(EXEC_TRAJ, fsm.state_);
      EXPECT_FALSE(fsm.active_traj_braked_);
      EXPECT_TRUE(published_points.isApprox(fsm.active_traj_.position_traj_.getControlPoint()));
      // 入口内重新生成，终点应低位且完全越过障碍；失败未永久封死候选。
      fsm.fapp_replan_pending_ = true;
      fsm.fd_->start_pt_ = fsm.fd_->odom_pos_;
      ASSERT_EQ(SUCCEED, fsm.callExplorationPlanner());
      EXPECT_LT(manager.ed_->next_goal_.z(), start.z()-0.10);
      EXPECT_GT(manager.ed_->next_goal_.x(), upper.x()+0.10);
      EXPECT_TRUE(published_points.isApprox(fsm.active_traj_.position_traj_.getControlPoint()));
      ASSERT_TRUE(fsm.resumeAfterWidthAttempt("test replacement rejected before publish"));
      EXPECT_TRUE(manager.needsExplorationFappRetry(fsm.fd_->odom_pos_));
      EXPECT_FALSE(manager.manual_fapp_curve_active_);
      EXPECT_FALSE(fsm.active_traj_braked_);
      fsm.fapp_replan_pending_ = true;
      ASSERT_EQ(SUCCEED, fsm.callExplorationPlanner());
      manager.recordPublishedTrajectoryHeight();
      EXPECT_FALSE(manager.needsExplorationFappRetry(fsm.fd_->odom_pos_));
      EXPECT_NEAR(manager.manual_fapp_hold_height_, manager.ed_->next_goal_.z(), 0.01);
      fsm.fapp_replan_pending_ = true;
      fsm.fd_->static_state_ = true; // 正常跟踪纠偏重规划抢占动态试探。
      ASSERT_EQ(SUCCEED, fsm.callExplorationPlanner());
      EXPECT_FALSE(fsm.fapp_replan_pending_);
      return;
    }
    const int result = manager.planExploreMotion(start, zero, zero, zero);
    if (result == SUCCEED) manager.recordPublishedTrajectoryHeight();
    if (exploration) {
      ASSERT_EQ(SUCCEED, result);
      EXPECT_FALSE(manager.manualWaypointsEnabled());
      EXPECT_TRUE(manager.manual_fapp_active_);
      EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
      EXPECT_EQ(vertical_detour::LowProbePhase::IDLE, manager.low_probe_phase_);
      EXPECT_EQ(!asymmetric_side, manager.manual_fapp_curve_active_);
      EXPECT_TRUE(manager.recovery_side_latched_);
      if (!asymmetric_side) {
        EXPECT_GT(manager.ed_->next_goal_.x(), upper.x() + 0.1);
        EXPECT_GT(Astar::pathLength(manager.ed_->path_next_goal_), 1.2);
        EXPECT_NEAR(manager.low_probe_height_, manager.ed_->next_goal_.z(), 1e-6);
        if (low_minimum) EXPECT_NEAR(0.12, manager.low_probe_height_, 1e-6);
      } else {
        EXPECT_GT(std::fabs(manager.ed_->next_goal_.y() - start.y()), 0.2);
        EXPECT_NEAR(start.z(), manager.ed_->next_goal_.z(), 1e-6);
      }
      // 侧绕也必须一次越过包络，不能把0.4m短步误报为完整穿越。
      EXPECT_GT(manager.ed_->next_goal_.x(), upper.x() + 0.1);
      auto preceding = manager.planner_manager_->local_data_.position_traj_;
      const Vector3d endpoint = preceding.evaluateDeBoorT(preceding.getTimeSum());
      if (!asymmetric_side) {
        Vector3d lower_bound, upper_bound;
        double age;
        ASSERT_TRUE(manager.planner_manager_->watchedFappBounds(lower_bound, upper_bound, age));
        const double descent_start = lower_bound.x()-map.getResolution()-0.50;
        for (const auto& point : manager.ed_->path_next_goal_)
          if (point.x() < descent_start) EXPECT_NEAR(start.z(), point.z(), 1e-9);
        // 同时检查最终B样条，不能只有折线等高、优化后又提前下降。
        for (double t=0; t<preceding.getTimeSum(); t+=0.02) {
          const auto point = preceding.evaluateDeBoorT(t);
          if (point.x() < descent_start-0.10) EXPECT_NEAR(start.z(), point.z(), 0.02);
        }
      }
      EXPECT_GT(endpoint.x(), upper.x() + 0.1);
      EXPECT_GT((endpoint-start).norm(), 1.2);
      EXPECT_NEAR(endpoint.z(), asymmetric_side ? start.z() : manager.low_probe_height_, 0.01);
      for (double t = 0; t <= preceding.getTimeSum(); t += 0.02) {
        const Vector3d point = preceding.evaluateDeBoorT(t);
        EXPECT_TRUE(manager.planner_manager_->isPositionSafe(point));
        EXPECT_TRUE(manager.planner_manager_->isWatchedFappPositionSafe(point));
        if (!asymmetric_side && point.x() >= upper.x())
          EXPECT_NEAR(point.z(), manager.low_probe_height_, 0.02);
        if (low_minimum) EXPECT_GE(point.z(), 0.10);
      }
      checkManualFappExecution(manager, start);
      ros::WallDuration(0.85).sleep();
      message.states.clear();
      send();
      EXPECT_FALSE(manager.manualFappObservationMissing(start, 0.0));
      EXPECT_TRUE(manager.manual_fapp_active_);
      EXPECT_FALSE(manager.beginSuspendedUnderpass(start, 0.0));
      // 已横移到侧面且检测停发后，续接终点仍在同一历史包络后方。
      const double time = preceding.getTimeSum() * 0.4;
      Vector3d replanning_start = preceding.evaluateDeBoorT(time);
      if (asymmetric_side) {
        manager.horizontal_reference_height_ = endpoint.z();
        replanning_start.z() -= 0.08;
      }
      ASSERT_TRUE(manager.planExplorationFappCrossing(replanning_start,
          preceding.getDerivative().evaluateDeBoorT(time),
          preceding.getDerivative().getDerivative().evaluateDeBoorT(time), zero));
      EXPECT_GT(manager.ed_->next_goal_.x(), upper.x() + 0.1);
      EXPECT_NEAR(endpoint.z(), manager.ed_->next_goal_.z(), 0.01);
      if (asymmetric_side) EXPECT_GT((manager.ed_->next_goal_.y() - start.y()) *
          (prior_right_side ? -1.0 : 1.0), 0.0);
      {
        const Vector3d end = manager.ed_->next_goal_;
        EXPECT_FALSE(manager.manualFappObservationMissing(end, 0.0));
        EXPECT_TRUE(manager.manual_fapp_active_); // 预测端点不能释放历史。
        EXPECT_FALSE(manager.manualFappObservationMissing(end, 0.0, true));
        EXPECT_FALSE(manager.manual_fapp_active_);
      }
      return;
    }
    if (inside) {
      auto& planner = *manager.planner_manager_;
      ASSERT_FALSE(planner.isWatchedFappPositionSafe(start));
      if(scenario=="inside_history_strict") EXPECT_EQ(FAIL,result);
      else {
        ASSERT_EQ(SUCCEED,result);
        EXPECT_TRUE(manager.fapp_crossing_retry_needed_);
        EXPECT_GT(manager.ed_->next_goal_.x(),start.x()+.20);
        checkManualFappExecution(manager,start);
      }
      return;
    }
    if (scenario == "blocked") {
      EXPECT_EQ(FAIL, result);
      EXPECT_EQ(0u, manager.manual_path_.next);
      EXPECT_FALSE(manager.suspended_underpass_active_);
      return;
    }
    ASSERT_EQ(SUCCEED, result);
    if (approach || asymmetric) {
      double age = 0.0;
      ASSERT_TRUE(manager.planner_manager_->watchedFappBounds(lower, upper, age));
      EXPECT_EQ(0u, manager.manual_path_.next);
      EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
      if (!asymmetric_side) EXPECT_GT(manager.ed_->next_goal_.x(), start.x() + 0.2);
      if (approach) {
        EXPECT_EQ(committed, manager.recovery_side_latched_);
        EXPECT_FALSE(manager.manual_fapp_curve_active_);
        // 规划优先允许普通前进段进入历史包络，不能被包络边界卡住。
        EXPECT_LE(manager.ed_->next_goal_.x(), start.x() + 1.20 + 1e-6);
        EXPECT_NEAR(start.z(), manager.ed_->next_goal_.z(), 1e-6);
      } else if (asymmetric_side) {
        EXPECT_LT(lower.z() - manager.underpass_height_tracking_margin_, 0.15);
        EXPECT_FALSE(manager.manual_fapp_curve_active_);
        EXPECT_TRUE(manager.recovery_side_latched_);
        EXPECT_GT(std::fabs(manager.ed_->next_goal_.y() - start.y()), 0.2);
        EXPECT_NEAR(start.z(), manager.ed_->next_goal_.z(), 1e-6);
      } else {
        EXPECT_TRUE(manager.manual_fapp_curve_active_);
        EXPECT_NEAR(0.23, manager.low_probe_height_, 1e-6);
        EXPECT_NEAR(0.29, lower.z(), 1e-6);
        EXPECT_NEAR(0.94, upper.z(), 1e-6);
        EXPECT_GT(manager.ed_->next_goal_.x(), upper.x());
      }
      auto& traj = manager.planner_manager_->local_data_.position_traj_;
      for (double t = 0; t <= traj.getTimeSum(); t += 0.02) {
        const Vector3d p = traj.evaluateDeBoorT(t);
        if (!approach) EXPECT_TRUE(manager.planner_manager_->isWatchedFappPositionSafe(p));
        EXPECT_TRUE(manager.planner_manager_->isPositionSafe(p));
        EXPECT_GE(p.z(), 0.15);
      }
      checkManualFappExecution(manager, start);
      return;
    }
    const bool expected_underpass = !floor_blocked && !unknown_below && !disabled && !stale && !recorded;
    EXPECT_EQ(expected_underpass && !memory, manager.currentPlanIsVerticalDetour());
    EXPECT_EQ(expected_underpass, manager.suspended_underpass_active_);
    if (disabled || stale) {
      EXPECT_NEAR(start.z(), manager.ed_->next_goal_.z(), 1e-6);
      EXPECT_FALSE(manager.recovery_side_latched_);
      return;
    }
    if (expected_underpass) {
      if (memory) {
        EXPECT_TRUE(manager.manual_fapp_curve_active_);
        EXPECT_NEAR(0.20, std::fabs(manager.manual_fapp_curve_side_), 1e-9);
        if (scenario == "memory_curve_right") EXPECT_LT(manager.manual_fapp_curve_side_, 0.0);
        EXPECT_GT(manager.ed_->next_goal_.x(), manager.low_probe_exit_progress_);
        EXPECT_GT((manager.ed_->next_goal_ - start).norm(), 1.2);
        EXPECT_NEAR(manager.ed_->next_goal_.z(), manager.low_probe_height_, 0.01);
      } else {
        EXPECT_NEAR(0.0, (manager.ed_->next_goal_ - start).head<2>().norm(), 1e-9);
      }
      if (!memory) EXPECT_LT(manager.ed_->next_goal_.z(), start.z() - 0.25);
      if (!mapped) { EXPECT_NEAR(0.640, manager.low_probe_height_, 0.001); }
      EXPECT_DOUBLE_EQ(start.z(), manager.low_probe_return_height_);
      EXPECT_EQ(memory, manager.recovery_side_latched_);
    } else {
      EXPECT_TRUE(manager.recovery_side_latched_);
      EXPECT_NEAR(start.z(), manager.ed_->next_goal_.z(), 1e-6);
      if (scenario == "left") { EXPECT_GT(manager.ed_->next_goal_.y(), start.y() + 0.20); }
      if (scenario == "right") { EXPECT_LT(manager.ed_->next_goal_.y(), start.y() - 0.20); }
    }
    if (recorded) {
      double age = 0.0;
      ASSERT_TRUE(manager.planner_manager_->watchedFappBounds(lower, upper, age));
      EXPECT_NEAR(0.1363908612728119, lower.z()-manager.underpass_height_tracking_margin_, 1e-6);
      EXPECT_FALSE(manager.manual_fapp_curve_active_);
      EXPECT_TRUE(manager.recovery_side_latched_);
      return;
    }
    if (memory) {
      auto& planner = *manager.planner_manager_;
      ASSERT_TRUE(manager.manual_fapp_active_);
      const auto phase = manager.low_probe_phase_;
      const bool side = manager.recovery_side_latched_;
      const Vector3d side_direction = manager.recovery_side_dir_;
      auto empty = boost::make_shared<obj_state_msgs::ObjectsStates>();
      empty->header.frame_id = "UAV0/camera_init";
      empty->header.stamp = ros::Time::now();
      planner.fappCallback(empty);
      EXPECT_FALSE(manager.manualFappObservationMissing(start, 0.0));
      EXPECT_EQ(phase, manager.low_probe_phase_);
      EXPECT_EQ(side, manager.recovery_side_latched_);
      ros::WallDuration(0.85).sleep(); // 也覆盖消息完全停发，而非仅连续空帧。
      EXPECT_FALSE(manager.manualFappObservationMissing(start, 0.0));
      EXPECT_EQ(SUCCEED, manager.planExploreMotion(start, zero, zero, zero));
      EXPECT_EQ(phase, manager.low_probe_phase_);
      EXPECT_EQ(side, manager.recovery_side_latched_);
      EXPECT_EQ(0u, manager.manual_path_.next);
      // 预测起点不能释放历史；失观时也不能让穿过球的直线通过检查。
      EXPECT_FALSE(manager.manualFappObservationMissing(goal, 0.0));
      EXPECT_TRUE(manager.manual_fapp_active_);
      EXPECT_FALSE(planner.isFappPathSafe({start, goal}));
      if (side) EXPECT_TRUE(manager.recovery_side_dir_.isApprox(side_direction));
      if (expected_underpass) {
        auto preceding = planner.local_data_.position_traj_;
        const double t = preceding.getTimeSum()*0.4;
        const Vector3d p = preceding.evaluateDeBoorT(t);
        const Vector3d v = preceding.getDerivative().evaluateDeBoorT(t);
        const Vector3d a = preceding.getDerivative().getDerivative().evaluateDeBoorT(t);
        ASSERT_GT(v.norm(), 0.01);
        ASSERT_EQ(SUCCEED, manager.planExploreMotion(p, v, a, zero));
        auto& replanned = planner.local_data_.position_traj_;
        EXPECT_LT((replanned.evaluateDeBoorT(0.0)-p).norm(), 0.003);
        EXPECT_LT((replanned.getDerivative().evaluateDeBoorT(0.0)-v).norm(), 1e-3);
        EXPECT_LT((replanned.getDerivative().getDerivative().evaluateDeBoorT(0.0)-a).norm(), 1e-3);
        ASSERT_EQ(SUCCEED, manager.planExploreMotion(start, zero, zero, zero));
        planner.local_data_.start_time_ = ros::Time::now();
        double distance = 0.0;
        EXPECT_TRUE(planner.checkTrajCollision(distance));
      }
      send();
      EXPECT_FALSE(manager.manualFappObservationMissing(start, 0.0));
      EXPECT_EQ(phase, manager.low_probe_phase_);
      EXPECT_EQ(side, manager.recovery_side_latched_);
    }
    Vector3d current = start;
    bool saw_low_advance = false, saw_ascent = false, saw_side_pass = false;
    int planned = result;
    for (int step = 0; step < 75 && !manager.manual_path_.finished(); ++step) {
      if (planned == SUCCEED) {
        manager.recordPublishedTrajectoryHeight();
        auto& trajectory = manager.planner_manager_->local_data_.position_traj_;
        const auto control = trajectory.getControlPoint();
        double length = 0.0;
        for (int i = 1; i < control.rows(); ++i)
          length += (control.row(i) - control.row(i - 1)).norm();
        if (!memory || !manager.manual_fapp_active_) {
          EXPECT_LE(length, manager.manual_path_max_segment_length_ + 1e-6);
        } else if (step == 0) {
          EXPECT_GT(length, manager.manual_path_max_segment_length_);
        }
        std::vector<Vector3d> flown_path;
        for (int i = 0; i <= 50; ++i)
          flown_path.push_back(trajectory.evaluateDeBoorT(trajectory.getTimeSum() * i / 50.0));
        EXPECT_TRUE(manager.planner_manager_->isFappPathSafe(flown_path));
        const Vector3d end = trajectory.evaluateDeBoorT(trajectory.getTimeSum());
        if (expected_underpass && end.x() > start.x() + 0.10 &&
            end.z() < start.z() - 0.20)
          saw_low_advance = true;
        if (memory && expected_underpass && manager.manual_fapp_active_) {
          EXPECT_GT(end.x(), current.x() + 0.01);
          for (const auto& point : flown_path) {
            EXPECT_TRUE(manager.planner_manager_->isWatchedFappPositionSafe(point));
            if (point.x() > start.x()+0.10 && point.z() < start.z()-0.20)
              saw_low_advance = true;
            if (saw_low_advance && point.x() > manager.low_probe_exit_progress_ &&
                point.z() > manager.low_probe_height_+0.20)
              saw_ascent = true;
          }
          EXPECT_GT(end.x(), manager.low_probe_exit_progress_);
          EXPECT_NEAR(end.z(), manager.low_probe_height_, 0.01);
        }
        if (memory && expected_underpass && !manager.manual_fapp_active_ &&
            saw_low_advance && end.z() > current.z() + 0.05) {
          EXPECT_GT(current.x(), manager.low_probe_exit_progress_ + 0.15);
          saw_ascent = true;
        }
        if (saw_low_advance && manager.currentPlanIsVerticalDetour() && end.z() > current.z()) {
          EXPECT_GE(current.x(), manager.low_probe_exit_progress_ - 1e-6);
          EXPECT_NEAR(0.0, (end - current).head<2>().norm(), 1e-6);
          saw_ascent = true;
        }
        if (!expected_underpass && end.x() > 0.40 && std::fabs(end.y() - start.y()) > 0.40)
          saw_side_pass = true;
        current = end;
        manager.updateMissionOdometry(current, 0.0);
      }
      // 首次确认后持续空帧直到完成穿越，不能靠重新检测到球推动流程。
      if (!memory) send();
      if (memory && step % 2 == 0) {
        auto empty = boost::make_shared<obj_state_msgs::ObjectsStates>();
        empty->header.frame_id = "UAV0/camera_init";
        empty->header.stamp = ros::Time::now();
        manager.planner_manager_->fappCallback(empty);
      }
      planned = manager.planExploreMotion(current, zero, zero, zero);
    }
    EXPECT_EQ(1u, manager.manual_path_.next);
    if (memory) EXPECT_FALSE(manager.manual_fapp_active_);
    if (expected_underpass) {
      EXPECT_TRUE(saw_low_advance);
      EXPECT_TRUE(saw_ascent);
      EXPECT_FALSE(manager.suspended_underpass_active_);
      if (memory) {
        // 完成穿越后立即恢复普通航点长度上限。
        manager.manual_path_.points.push_back(Vector3d(current.x(), -1.425, current.z()));
        ASSERT_EQ(SUCCEED, manager.planExploreMotion(current, zero, zero, zero));
        const auto controls = manager.planner_manager_->local_data_.position_traj_.getControlPoint();
        double ordinary_length = 0.0;
        for (int i=1; i<controls.rows(); ++i)
          ordinary_length += (controls.row(i)-controls.row(i-1)).norm();
        EXPECT_LE(ordinary_length, manager.manual_path_max_segment_length_+1e-6);
        EXPECT_FALSE(manager.manual_fapp_active_);
      }
    } else {
      EXPECT_TRUE(saw_side_pass);
      EXPECT_LT(std::fabs(current.y() - goal.y()), manager.manual_path_arrive_distance_);
    }
  }

  static void runDynamicOpening(double side) {
    ros::NodeHandle nh(side > 0 ? "~opening_left" : "~opening_right");
    nh.setParam("fapp/enabled", true);
    nh.setParam("fapp/memory_enabled", true);
    nh.setParam("fapp/planner_priority", true);
    FastExplorationManager manager;
    initialize(nh, manager, true);
    auto& map = *manager.sdf_map_;
    for (double y = -1.5; y < 1.5; y += .05)
      for (double z = .60; z < 1.5; z += .05) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1.9, y, z));
        map.inputPointCloud(cloud, 1, Vector3d(-1.9, y, z));
        cloud.front().x = -1.9;
        map.inputPointCloud(cloud, 1, Vector3d(1.9, y, z));
      }
    map.updateESDF3d();
    auto& planner = *manager.planner_manager_;
    const Vector3d start(-1.0, side*.60, 1.0), zero = Vector3d::Zero();
    manager.task_search_manager_->setMap(manager.sdf_map_);
    manager.task_search_manager_->global_no_return_ = false;
    manager.task_search_manager_->exit_detection_enabled_ = false;
    manager.task_search_manager_->updateRobotPose(start, 0.0);
    manager.manual_fapp_active_ = true;
    manager.manual_fapp_forward_ = Vector3d::UnitX();
    manager.manual_fapp_exit_progress_ = .65;
    manager.manual_fapp_hold_height_ = start.z();
    manager.recovery_side_latched_ = true;
    manager.recovery_side_dir_ = side * Vector3d::UnitY();
    planner.fapp_watched_valid_ = true;
    auto& ball = planner.fapp_watched_state_;
    ball.position = Vector3d(.2, 0, 1);
    ball.velocity = Vector3d(0, -side*.3, 0);
    ball.size = Vector3d::Constant(.2);
    ball.stamp = ros::Time::now().toSec();
    planner.fapp_watched_lower_ = Vector3d(-.2,-.9,.7);
    planner.fapp_watched_upper_ = Vector3d(.6,.9,1.4);
    planner.fapp_states_ = {ball};
    ASSERT_TRUE(planner.fappSideOpening(start, manager.recovery_side_dir_));
    ball.velocity *= -1;
    EXPECT_FALSE(manager.planFappOpeningCrossing(start, zero, zero, zero));
    ball.velocity *= -1;
    ball.stamp -= 2.0;
    EXPECT_FALSE(manager.planFappOpeningCrossing(start, zero, zero, zero));
    ball.stamp = ros::Time::now().toSec();
    planner.fapp_states_ = {ball};
    ASSERT_TRUE(manager.planFappOpeningCrossing(start, zero, zero, zero));
    EXPECT_TRUE(manager.fapp_rush_pending_);
    EXPECT_FALSE(manager.fapp_rush_active_);
    EXPECT_DOUBLE_EQ(.4, planner.pp_.max_vel_);
    EXPECT_DOUBLE_EQ(.5, planner.pp_.max_acc_);
    EXPECT_GT(manager.ed_->next_goal_.x(), manager.manual_fapp_exit_progress_ + .1);
    EXPECT_NEAR(start.y(), manager.ed_->next_goal_.y(), 1e-6);
    ASSERT_FALSE(planner.isFappPathSafe(manager.ed_->path_next_goal_)); // 历史摆幅覆盖该侧。
    ASSERT_TRUE(planner.isFappCrossingTrajectorySafe());
    const Vector3d observed_position = ball.position;
    planner.fapp_states_.clear();
    ball.position = start;
    EXPECT_FALSE(planner.isFappCrossingTrajectorySafe()); // 空帧仍检查尚新鲜的锁定目标。
    ball.position = observed_position;
    planner.fapp_states_ = {ball};
    auto fast = planner.local_data_.position_traj_;
    auto velocity = fast.getDerivative();
    auto acceleration = velocity.getDerivative();
    double vmax=0, amax=0;
    for (double t=0; t<=fast.getTimeSum(); t+=.01) {
      vmax = std::max(vmax, velocity.evaluateDeBoorT(t).cwiseAbs().maxCoeff());
      amax = std::max(amax, acceleration.evaluateDeBoorT(t).cwiseAbs().maxCoeff());
    }
    EXPECT_GT(vmax, .40);
    EXPECT_LE(vmax, 1.001);
    EXPECT_LE(amax, 1.001);
    EXPECT_FALSE(manager.keepsDynamicSide({start, start - .3*manager.recovery_side_dir_}));
    manager.recordPublishedTrajectoryHeight();
    EXPECT_TRUE(manager.fapp_rush_active_);
    const Vector3d end = manager.ed_->next_goal_;
    EXPECT_FALSE(manager.manualFappObservationMissing(end, 0.0));
    EXPECT_TRUE(manager.fapp_rush_active_); // 预测终点不提前恢复。
    manager.fapp_exploration_crossing_enabled_ = true;
    EXPECT_FALSE(manager.manualFappObservationMissing(end, 0.0, true));
    EXPECT_FALSE(manager.fapp_rush_active_);
    EXPECT_FALSE(manager.manual_fapp_active_);
    EXPECT_TRUE(manager.keepsDynamicSide({start, start - .3*side*Vector3d::UnitY()}));
    ASSERT_TRUE(planner.planExploreTraj({start, .5*(start+end), end}, zero, zero, 0.0));
    auto ordinary = planner.local_data_.position_traj_;
    EXPECT_GT(ordinary.getTimeSum(), fast.getTimeSum());
    velocity = ordinary.getDerivative(); acceleration = velocity.getDerivative();
    for (double t=0; t<=ordinary.getTimeSum(); t+=.02) {
      EXPECT_LE(velocity.evaluateDeBoorT(t).cwiseAbs().maxCoeff(), .401);
      EXPECT_LE(acceleration.evaluateDeBoorT(t).cwiseAbs().maxCoeff(), .501);
    }
    EXPECT_FALSE(manager.planFappOpeningCrossing(start, zero, zero, zero));
  }

  static void runPassageCentre(bool dynamic_side = false) {
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
    if (dynamic_side) {
      manager.manual_fapp_active_ = true;
      manager.manual_fapp_hold_height_ = path.front().z();
      manager.task_search_manager_->min_search_height_ = 0.10;
      manager.fapp_exploration_crossing_enabled_ = true;
      manager.recovery_side_latched_ = true;
      manager.recovery_side_dir_ = -Vector3d::UnitY();
      manager.recovery_side_origin_ = path.front();
      manager.ep_->mission_use_forward_fallback_ = true;
      manager.task_search_manager_->enabled_ = true;
      manager.task_search_manager_->setMap(manager.sdf_map_);
      manager.task_search_manager_->stable_progress_direction_ = Eigen::Vector2d::UnitX();
      manager.task_search_manager_->stable_progress_direction_valid_ = true;
      manager.task_search_manager_->exit_detection_enabled_ = false;
      manager.task_search_manager_->global_no_return_ = false;
      manager.task_search_manager_->updateRobotPose(path.front(), 0.0);
      auto& planner = *manager.planner_manager_;
      planner.fapp_config_.enabled = true;
      planner.fapp_memory_enabled_ = true;
      planner.fapp_planner_priority_ = true;
      planner.fapp_watched_valid_ = true;
      planner.fapp_watched_lower_ = Vector3d(-0.9, -0.7, 0.1);
      planner.fapp_watched_upper_ = Vector3d(0.5, 0.7, 0.8);
      ASSERT_FALSE(planner.isFappPathSafe(path)); // 历史摆幅曾覆盖右侧，当前静态通路畅通。
      for (bool wide_side : {false, true}) {
        manager.wide_side_bypass_active_ = wide_side;
        const auto original = path;
        EXPECT_FALSE(manager.centrePassagePath(path, 3.0));
        ASSERT_EQ(original.size(), path.size());
        for (size_t i=0; i<path.size(); ++i) EXPECT_TRUE(path[i].isApprox(original[i]));
        Vector3d next; double yaw = 0.0;
        ASSERT_TRUE(manager.buildSideRecoveryContinuation(path.front(), 0.0, next, yaw));
        EXPECT_GT(next.x(), path.front().x() + 0.3);
        EXPECT_NEAR(path.front().y(), next.y(), 1e-6);
      }
      manager.width_route_plan_ = true;
      manager.recordPublishedTrajectoryHeight();
      EXPECT_TRUE(manager.recovery_side_latched_);
      manager.manual_fapp_active_ = false; // 结束动态穿越后恢复通常的居中。
    }
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

  static void runEarlySplit(bool mirror, bool stepped = false, bool dead_end = false,
                            int generic = 0) {
    ros::NodeHandle nh("~early_split");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager, true);
    auto& map = *manager.sdf_map_;
    const double sign = mirror ? -1.0 : 1.0;
    const Vector3d start(-.4, .4 * sign, 1.025);
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
        cloud.push_back(pcl::PointXYZ(x, 1.20 * sign, z));
        cloud.push_back(pcl::PointXYZ(x, -.70 * sign, z));
        if (x >= .4 && x <= .6)
          for (double y = -.15; y <= .15; y += .025)
            cloud.push_back(pcl::PointXYZ(x, (.4 + y) * sign, z));
      }
    for (int i = 0; i < 8; ++i) map.inputPointCloud(cloud, cloud.size(), start);
    if (dead_end) {
      // 宽侧入口1m，但障碍后段被横墙封死；窄侧仍连通。
      for (double x = .70; x <= 1.05; x += .025)
        for (double y = -.70; y <= .22; y += .025)
          for (double z = .725; z <= 1.325; z += .025)
            map.setOccupied(Vector3d(x, y * sign, z));
    }
    map.updateESDF3d();
    manager.ep_->mission_use_forward_fallback_ = true;
    manager.ep_->mission_use_workspace_lock_ = true;
    manager.mission_workspace_lock_received_ = true;
    manager.mission_workspace_origin_ = start - Vector3d(1, 0, 0);
    manager.mission_workspace_dir_ = Vector3d::UnitX();
    manager.entry_forward_complete_ = true;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.global_no_return_ = false;
    task.exit_detection_enabled_ = false;
    task.cruise_height_ = start.z();
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    task.updateRobotPose(start, 0.0);
    if (dead_end) {
      const auto split = manager.findWideSideSplit(start, Vector3d::UnitX());
      ASSERT_TRUE(split.valid);
      const auto wide = manager.makeSplitLane(start, Vector3d::UnitX(), split, -sign);
      const auto narrow = manager.makeSplitLane(start, Vector3d::UnitX(), split, sign);
      vector<Vector3d> path;
      ASSERT_TRUE(wide.rear_known);
      EXPECT_FALSE(manager.splitPassagePath(start, wide, path));
      ASSERT_TRUE(manager.splitPassagePath(start, narrow, path));
      Vector3d target; double yaw = 0; bool detected = false;
      ASSERT_TRUE(manager.buildWideSideBypass(start, 0, Vector3d::UnitX(), target, yaw, detected));
      EXPECT_GT(sign * (target.y() - start.y()), .1);
      EXPECT_EQ(sign, manager.active_split_lane_.side);
      return;
    }
    if (generic) {
      // 普通规划也必须记住同侧后沿，前进0.20m不能解锁。
      manager.rememberSideRecovery(start, 0, {start, start + Vector3d(.2, -.4 * sign, 0)});
      ASSERT_TRUE(manager.committedSplitRoute());
      const auto lane = manager.active_split_lane_;
      const Vector3d pos = generic == 2 ? start :
          Vector3d(start + Vector3d(.25, -sign*(lane.offset+.08), 0));
      if (generic == 2) {
        // 先有普通横移记录，直到接近障碍才看清后沿，不能先按20cm释放。
        manager.split_route_committed_ = false;
        manager.wide_side_bypass_active_ = false;
        manager.recovery_side_origin_ = start - Vector3d(.30, 0, 0);
      }
      task.updateRobotPose(pos, 0);
      manager.recovery_odom_history_.push_back(pos);
      Vector3d target; double yaw = 0;
      ASSERT_TRUE(manager.buildSideRecoveryContinuation(pos, 0, target, yaw));
      EXPECT_TRUE(manager.committedSplitRoute());
      if (generic != 2) EXPECT_TRUE(route_width::insideLane(target, lane));
      EXPECT_GT(manager.wide_side_bypass_release_distance_, .30);
      return;
    }
    if (stepped) {
      Vector3d target; double yaw = 0; bool split = false;
      ASSERT_TRUE(manager.buildWideSideBypass(start, 0, Vector3d::UnitX(), target, yaw, split));
      ASSERT_TRUE(manager.committedSplitRoute());
      EXPECT_EQ(-sign, manager.active_split_lane_.side);
      const auto lane = manager.active_split_lane_;
      ASSERT_GT(lane.outer_offset, lane.offset);
      // 已经横移到右/左侧，但当前截面还在柱子之前，整个走廊依然连通。
      Vector3d pos = start + Vector3d(.20, -sign*(lane.offset+.08), 0);
      task.updateRobotPose(pos, 0);
      manager.recovery_odom_history_.push_back(pos);
      ASSERT_TRUE(manager.buildSideRecoveryContinuation(pos, 0, target, yaw));
      EXPECT_GT(target.x(), pos.x()+.19);
      EXPECT_TRUE(route_width::insideLane(target, lane));
      vector<Vector3d> path{pos, target};
      manager.centrePassagePath(path, 2.0);
      for (const auto& p : path) EXPECT_TRUE(route_width::insideLane(p, lane));
      EXPECT_TRUE(manager.planner_manager_->isPathSafe(path));
      if (path.size() == 2) path.insert(path.begin()+1, .5*(path.front()+path.back()));
      ASSERT_TRUE(manager.planner_manager_->planExploreTraj(path, Vector3d::Zero(), Vector3d::Zero(), 0));
      ASSERT_TRUE(manager.planner_manager_->isTrajectorySafe());
      auto& curve = manager.planner_manager_->local_data_.position_traj_;
      for (double t = 0; t <= curve.getTimeSum(); t += .05) {
        const Vector3d p = curve.evaluateDeBoorT(t);
        const double offset = lane.side*(p-lane.origin).y();
        EXPECT_GE(offset, lane.offset-.02); // 样条数值容差，不允许回到整条通道中心。
      }
      const Vector3d beyond = start + Vector3d(lane.exit+.1, -sign*(lane.offset+.08), 0);
      task.updateRobotPose(beyond, 0);
      manager.recovery_odom_history_.push_back(beyond);
      EXPECT_FALSE(manager.buildSideRecoveryContinuation(beyond, 0, target, yaw));
      EXPECT_FALSE(manager.committedSplitRoute());
      EXPECT_TRUE(manager.insideRecoveryLane(start));
      return;
    }
    const vector<Vector3d> approach{start, start + Vector3d(.45, 0, 0)};
    // The current short trajectory is safe, but ends BEFORE a divider 0.8m away.
    ASSERT_TRUE(manager.planner_manager_->isPathSafe(approach));
    ASSERT_TRUE(manager.prepareWiderRoute(approach, 0));
    EXPECT_GT(manager.pending_split_route_.entry, .4);
    EXPECT_EQ(-sign, manager.pending_split_route_.side);
    EXPECT_FALSE(manager.recovery_side_latched_); // probes must not alter the live route
    EXPECT_TRUE(route_width::followsSplit(manager.pending_wider_route_, manager.pending_split_route_));
    EXPECT_TRUE(manager.planner_manager_->isPathSafe(manager.pending_wider_route_));
    const auto path = manager.pending_wider_route_;
    const auto lane = manager.pending_split_route_;
    ASSERT_TRUE(manager.planner_manager_->planExploreTraj(
        path, Vector3d(.3, 0, 0), Vector3d::Zero(), 0));
    ASSERT_TRUE(manager.planner_manager_->isTrajectorySafe());
    auto& trajectory = manager.planner_manager_->local_data_.position_traj_;
    vector<Vector3d> samples;
    for (double t = 0; t < trajectory.getTimeSum(); t += .05)
      samples.push_back(trajectory.evaluateDeBoorT(t));
    samples.push_back(trajectory.evaluateDeBoorT(trajectory.getTimeSum()));
    EXPECT_TRUE(route_width::followsSplit(samples, lane));
    manager.clearWiderRoute(); // rejected/unconfirmed attempt leaves no commitment
    EXPECT_FALSE(manager.recovery_side_latched_);
    manager.planned_split_route_ = lane;
    manager.width_route_plan_ = true;
    manager.recordPublishedTrajectoryHeight();
    ASSERT_TRUE(manager.recovery_side_latched_);
    EXPECT_TRUE(manager.wide_side_bypass_active_);
    EXPECT_GT(manager.wide_side_bypass_release_distance_, 1.1);
    EXPECT_FALSE(manager.prepareWiderRoute(approach, 0)); // no reranking across the divider
    auto centered = path;
    manager.centrePassagePath(centered, 2.0);
    EXPECT_TRUE(route_width::followsSplit(centered, lane));
    Vector3d target; double yaw = 0;
    const Vector3d beyond = start + Vector3d(lane.exit + .1, -sign*.5, 0);
    // Predicted start beyond the rear is insufficient: actual aircraft is still alongside it.
    manager.recovery_odom_history_.push_back(start + Vector3d(lane.exit-.1, -sign*.5, 0));
    manager.buildSideRecoveryContinuation(beyond, 0, target, yaw);
    EXPECT_TRUE(manager.recovery_side_latched_);
    manager.recovery_odom_history_.push_back(beyond);
    EXPECT_FALSE(manager.buildSideRecoveryContinuation(beyond, 0, target, yaw));
    EXPECT_FALSE(manager.recovery_side_latched_);
    // 新通道轴已经转弯时，旧障碍的固定通过轴不能阻止正常转弯。
    manager.planned_split_route_ = lane;
    manager.width_route_plan_ = true;
    manager.recordPublishedTrajectoryHeight();
    task.stable_progress_direction_ = Eigen::Vector2d::UnitY();
    EXPECT_FALSE(manager.buildSideRecoveryContinuation(start, 0, target, yaw));
    EXPECT_FALSE(manager.recovery_side_latched_);
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
    manager.manual_fapp_active_ = true;
    manager.recovery_side_latched_ = true;
    manager.recovery_side_dir_ = side * Vector3d::UnitY();
    EXPECT_FALSE(manager.prepareWiderRoute(straight, 0));
    EXPECT_TRUE(manager.pending_wider_route_.empty());
    manager.manual_fapp_active_ = false;
    manager.recovery_side_latched_ = false;
    ASSERT_TRUE(manager.prepareWiderRoute(straight, 0));
    EXPECT_LT((manager.widerRouteTarget().y() - start.y()) * side, -.15);
    EXPECT_TRUE(manager.planner_manager_->isPathSafe(manager.pending_wider_route_));
    const auto chosen = manager.pending_wider_route_;
    ASSERT_TRUE(manager.widthEndpointHasRoom(chosen));
    Vector3d terminal_direction = chosen.back() - chosen[chosen.size() - 2];
    terminal_direction.z() = 0;
    ASSERT_GT(terminal_direction.norm(), 1e-6);
    terminal_direction.normalize();
    // 终点本身足迹可行，但前方不到30cm有障碍；宽度优化不得把它当作好终点。
    const Vector3d obstacle = chosen.back() + .28 * terminal_direction;
    cloud.clear();
    for (double z = .825; z <= 1.225; z += .025)
      for (double y = -.075; y <= .075; y += .025)
        cloud.push_back(pcl::PointXYZ(obstacle.x(), obstacle.y() + y, z));
    for (int i = 0; i < 8; ++i) map.inputPointCloud(cloud, cloud.size(), start);
    ASSERT_TRUE(manager.planner_manager_->isPathSafe(chosen));
    EXPECT_FALSE(manager.widthEndpointHasRoom(chosen));
    EXPECT_FALSE(manager.widthReplacementImproves(straight, chosen, 0));
    if (manager.prepareWiderRoute(straight, 0)) {
      EXPECT_TRUE(manager.widthEndpointHasRoom(manager.pending_wider_route_));
    } else {
      EXPECT_TRUE(manager.pending_wider_route_.empty());
    }
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

  static void runMovingClimb(int mode, bool height_correction = false,
                            bool manual_mission = false) {
    ros::NodeHandle nh("~");
    nh.setParam("mission/task_search/min_height", 0.10);
    nh.setParam("mission/task_search/cruise_height", 0.60);
    nh.setParam("mission/task_search/max_goal_climb", mode == 5 ? 0.06 : 0.30);
    FastExplorationManager manager;
    initialize(nh, manager);
    for (const auto& name : {"min_height", "cruise_height", "max_goal_climb"})
      nh.deleteParam(std::string("mission/task_search/") + name);
    manager.mission_entered_search_region_ = false;
    manager.ep_->mission_search_region_min_ = Vector3d(-1.2, 0.7, 0.2);
    manager.ep_->mission_search_region_max_ = Vector3d(6.2, 6.8, 1.8);
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
    if (manual_mission) {
      // 重现下穿结束后仍沿低位同侧前进的航点入口，不能只直接测试回升工具函数。
      manager.manual_path_enabled_ = true;
      manager.manual_path_started_ = true;
      manager.manual_path_exploration_style_ = true;
      manager.manual_path_use_astar_ = true;
      manager.manual_local_goal_enabled_ = false;
      manager.manual_path_max_segment_length_ = 1.20;
      manager.ep_->mission_use_forward_fallback_ = true;
      manager.edt_environment_ = planner.edt_environment_;
      ViewNode::caster_.reset(new RayCaster);
      Vector3d map_origin, map_size;
      map.getRegion(map_origin, map_size);
      ViewNode::caster_->setParams(map.getResolution(), map_origin);
      const Vector3d goal(1.025, 0.025, mode == 7 ? 0.40 : 0.60);
      const std::vector<Vector3d> waypoints{goal, Vector3d(1.225, 0.025, 0.60)};
      manager.manual_path_.points = waypoints;
      manager.horizontal_reference_height_ = start.z();
      manager.recovery_side_latched_ = true;
      manager.recovery_side_dir_ = -Vector3d::UnitY();
      manager.recovery_side_origin_ = start;
      manager.wide_side_bypass_active_ = true;
      manager.wide_side_bypass_release_distance_ = 1.0;
      // 先取得同一地图、同一侧绕状态下不启用斜升的真实水平轨迹作为比较基线。
      manager.ground_ascent_enabled_ = false;
      ASSERT_EQ(SUCCEED, manager.planExploreMotion(start, Vector3d::Zero(),
                                                   Vector3d::Zero(), Vector3d::Zero()));
      auto original = planner.local_data_;
      const Vector3d horizontal_end = manager.ed_->next_goal_;
      ASSERT_NEAR(start.z(), horizontal_end.z(), 1e-6);
      ASSERT_GT(horizontal_end.x(), start.x() + 0.2);
      if (mode == 8) {
        const auto controls = original.position_traj_.getControlPoint();
        double length = 0.0;
        for (int i = 1; i < controls.rows(); ++i)
          length += (controls.row(i) - controls.row(i-1)).norm();
        manager.manual_path_max_segment_length_ = length + 0.005;
      }
      manager.ground_ascent_enabled_ = true;
      ASSERT_EQ(SUCCEED, manager.planExploreMotion(start, Vector3d::Zero(),
                                                   Vector3d::Zero(), Vector3d::Zero()));
      EXPECT_EQ(0u, manager.manual_path_.next);
      ASSERT_EQ(waypoints.size(), manager.manual_path_.points.size());
      for (size_t i = 0; i < waypoints.size(); ++i)
        EXPECT_TRUE(manager.manual_path_.points[i].isApprox(waypoints[i]));
      EXPECT_TRUE(manager.recovery_side_latched_);
      EXPECT_LT(manager.recovery_side_dir_.y(), -0.9);
      EXPECT_TRUE(manager.ed_->next_goal_.head<2>().isApprox(horizontal_end.head<2>(), 1e-6));
      const bool raised = mode == 0 || mode == 7;
      if (raised) {
        EXPECT_GT(manager.ed_->next_goal_.z(), start.z() + 0.06);
        EXPECT_LE(manager.ed_->next_goal_.z(), goal.z() + 1e-6);
        EXPECT_NEAR(manager.ed_->next_goal_.z(), manager.pending_reference_height_, 1e-9);
        // 回升候选只有真正发布后才更新低位参考。
        EXPECT_DOUBLE_EQ(start.z(), manager.horizontal_reference_height_);
        manager.recordPublishedTrajectoryHeight();
        EXPECT_NEAR(manager.ed_->next_goal_.z(), manager.horizontalGoalHeight(start.z()), 1e-9);
      } else {
        EXPECT_TRUE(manager.ed_->next_goal_.isApprox(horizontal_end, 1e-6));
        EXPECT_TRUE(planner.local_data_.position_traj_.getControlPoint().isApprox(
            original.position_traj_.getControlPoint(), 1e-6));
      }
      const auto controls = planner.local_data_.position_traj_.getControlPoint();
      double length = 0.0;
      for (int i = 1; i < controls.rows(); ++i)
        length += (controls.row(i) - controls.row(i-1)).norm();
      EXPECT_LE(length, manager.manual_path_max_segment_length_ + 1e-6);
      EXPECT_TRUE(planner.isTrajectorySafe());
      if (raised) {
        for (double t = 0.0; t <= planner.local_data_.duration_; t += 0.02)
          EXPECT_TRUE(planner.isKnownFreePosition(planner.local_data_.position_traj_.evaluateDeBoorT(t)));
      }
      return;
    }
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
    if (mode == 9) task.mission_stage_ = TaskSearchManager::LANDING;
    if (mode == 6) manager.manual_fapp_active_ = true;
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
      if (mode == 6) {
        manager.manual_fapp_active_ = false;
        EXPECT_TRUE(manager.tryCruiseHeightRecovery(start, Vector3d::Zero(), Vector3d::Zero(), 0.0));
      }
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
    // 即使已有待转航向，执行中的安全回调也不能以转弯为由结束平移。
    manager.turn_in_place_enabled_ = true;
    manager.mission_workspace_lock_received_ = true;
    manager.entry_forward_complete_ = true;
    task.setCorridorFrame(Vector3d(-3, 0, 1), Vector3d::UnitX());
    task.updateRobotPose(fsm.fd_->odom_pos_, 0.0);
    task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
    fsm.last_suspended_underpass_check_ = ros::Time::now() + ros::Duration(30.0);
    fsm.safetyCallback(ros::TimerEvent());
    check(fsm.state_ == EXEC_TRAJ && !fsm.active_traj_braked_,
          "heading update replaced a collision-free published corridor route");

    fsm.active_traj_.start_time_ = ros::Time::now() -
        ros::Duration(fsm.active_traj_.duration_ - 0.02);
    fsm.FSMCallback(ros::TimerEvent());
    check(fsm.state_ == PLAN_TRAJ, "completed corridor route never resumed planning");
    Vector3d turn_direction;
    EXPECT_TRUE(task.mappedCorridorDirection(0.0, turn_direction));
    EXPECT_TRUE(turn_direction.isApprox(Vector3d::UnitY()));
    task.completeTurnYawAlignment();

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

  static void runManualLocalGoal(int mode) {
    ros::NodeHandle nh("~manual_local_goal");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", .05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", .05);
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.manual_path_enabled_ = mode != 4;
    manager.manual_path_use_astar_ = true;
    manager.manual_local_goal_enabled_ = true;
    manager.manual_exit_latched_ = mode == 2;
    manager.manual_fapp_active_ = mode == 3;
    auto& map = *manager.sdf_map_;
    const Vector3d start(-.6, .25, 1.0), anchor(.60, -.10, 1.0), second(1.3, .4, 1.0);
    manager.manual_path_.points = {anchor, second};
    if (mode != 1) {
      for (int iy = -8; iy <= 29; ++iy)
        for (int iz = 14; iz <= 25; ++iz) {
          const double y = (iy + .5) * .05, z = (iz + .5) * .05;
          pcl::PointCloud<pcl::PointXYZ> cloud;
          cloud.push_back(pcl::PointXYZ(1.9, y, z));
          map.inputPointCloud(cloud, 1, Vector3d(-1.9, y, z));
          cloud.front().x = -1.9;
          map.inputPointCloud(cloud, 1, Vector3d(1.9, y, z));
        }
      if (mode == 5) {
        for (double x = -.1; x <= .1; x += .05)
          for (double y = -1.9; y <= 1.9; y += .05)
            for (double z = .05; z <= 1.95; z += .05)
              map.setOccupied(Vector3d(x, y, z));
      }
      map.updateESDF3d();
    }
    if (mode == 0) {
      ASSERT_TRUE(manager.planner_manager_->isKnownFreePosition(start));
      ASSERT_TRUE(manager.planner_manager_->isKnownFreePosition(Vector3d(.6, .4, 1.0)));
      ASSERT_EQ(SDFMap::FREE, manager.widthColumnState(Vector3d(.6, .4, 1.0)));
    }
    manager.improveManualWaypoint(start);
    if (mode != 0) {
      EXPECT_EQ(anchor, manager.manual_path_.points.front());
      EXPECT_FALSE(manager.manual_local_goal_selected_);
      return;
    }
    ASSERT_TRUE(manager.manual_local_goal_selected_);
    const Vector3d chosen = manager.manual_path_.points.front();
    EXPECT_GT(chosen.y(), anchor.y()+.1);
    EXPECT_LE((chosen-anchor).norm(), .60+1e-6);
    EXPECT_DOUBLE_EQ(anchor.z(), chosen.z());
    EXPECT_EQ(0U, manager.manual_path_.next);
    EXPECT_EQ(second, manager.manual_path_.points.back());
    for (int i=0; i<3; ++i) manager.improveManualWaypoint(start);
    EXPECT_EQ(chosen, manager.manual_path_.points.front());
    EXPECT_EQ(anchor, manager.manual_local_goal_anchor_);
    manager.manual_path_.updateOdometry(anchor, .1);
    EXPECT_EQ(0U, manager.manual_path_.next);
    manager.manual_path_.updateOdometry(chosen, .1);
    EXPECT_EQ(1U, manager.manual_path_.next);
    manager.improveManualWaypoint(chosen);
    EXPECT_EQ(second, manager.manual_local_goal_anchor_); // 下一点仍以其手填坐标为基准。
  }

  static void runManualLowWaypointProgress() {
    ros::NodeHandle nh("~manual_low_waypoint_progress");
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.task_search_manager_.reset();
    manager.manual_path_enabled_ = true;
    manager.external_selection_only_ = true;
    manager.manual_path_arrive_distance_ = .25;
    const Vector3d first(3.8, 0, .6), next(5.6, -.3, .6);
    const Vector3d actual(3.775, -.025, .271), zero = Vector3d::Zero();
    manager.manual_path_.points = {first, next};
    manager.manual_fapp_active_ = true;
    manager.manual_fapp_hold_height_ = .32;
    manager.manual_fapp_exit_progress_ = 3.844;
    manager.manual_fapp_forward_ = Vector3d::UnitX();
    // 预测起点不能推进；水平位置未到不能消耗航点。
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(actual, zero, zero, zero));
    EXPECT_EQ(0U, manager.manual_path_.next);
    manager.updateMissionOdometry(Vector3d(3.4, 0, .32), 0);
    EXPECT_EQ(0U, manager.manual_path_.next);
    manager.updateMissionOdometry(actual, 0);
    ASSERT_EQ(1U, manager.manual_path_.next);
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(actual, zero, zero, zero));
    EXPECT_EQ(Vector3d(5.6, -.3, .32), manager.ed_->next_goal_);
    EXPECT_TRUE(manager.manual_fapp_active_);
    EXPECT_DOUBLE_EQ(3.844, manager.manual_fapp_exit_progress_);
    EXPECT_EQ(first, manager.manual_path_.points.front());
    EXPECT_EQ(next, manager.manual_path_.points.back());
    // 原有通过判定解除动态状态后，继续使用手填高度。
    manager.manual_fapp_active_ = false;
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(actual, zero, zero, zero));
    EXPECT_EQ(next, manager.ed_->next_goal_);
    manager.updateMissionOdometry(Vector3d(5.6, -.3, .27), 0);
    EXPECT_TRUE(manager.manual_path_.finished());
  }

  static void runManualWaypointSkip() {
    ros::NodeHandle nh("~manual_xy_skip");
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.task_search_manager_.reset();
    manager.manual_path_enabled_ = true;
    manager.external_selection_only_ = true;
    manager.manual_path_wait_for_trigger_ = true;
    const Vector3d first(.525, .025, 1.025), second(1.525, .025, 1.025);
    const Vector3d actual(.025, .025, .325), zero=Vector3d::Zero();
    manager.manual_path_.points = {first, second};
    // 入口未交接不计时；开始后未知体素或仅膨胀层不跳点。
    manager.updateMissionOdometry(actual, 0, ros::Time(100));
    EXPECT_TRUE(manager.manual_progress_stamp_.isZero());
    manager.startManualWaypointMission();
    manager.sdf_map_->setOccupied(first);
    manager.updateMissionOdometry(actual, 0, ros::Time(101));
    EXPECT_EQ(0U, manager.manual_path_.next);
    // 明确的目标原始占据立即跳过，并要求FSM撤销旧目标重规划。
    Eigen::Vector3i id; manager.sdf_map_->posToIndex(first, id);
    manager.sdf_map_->md_->occupancy_buffer_[manager.sdf_map_->toAddress(id)] =
        manager.sdf_map_->mp_->clamp_max_log_;
    manager.updateMissionOdometry(actual, 0, ros::Time(102));
    ASSERT_EQ(1U, manager.manual_path_.next);
    EXPECT_TRUE(manager.consumeManualExitReplanRequest());
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(actual, zero, zero, zero));
    EXPECT_EQ(second, manager.ed_->next_goal_);
    // 新点获得独立的5秒观察时间，短暂停留不跳点。
    manager.updateMissionOdometry(actual, 0, ros::Time(106.9));
    EXPECT_EQ(1U, manager.manual_path_.next);
    manager.updateMissionOdometry(actual, 0, ros::Time(107.1));
    EXPECT_EQ(1U, manager.manual_path_.next);
    EXPECT_FALSE(manager.manual_path_.finished());
    EXPECT_TRUE(manager.consumeManualExitReplanRequest());
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(actual, zero, zero, zero));
    EXPECT_EQ(second, manager.ed_->next_goal_);
    // 重规划后重新计时，连续卡住也不能被当成不可达而消耗航点。
    manager.updateMissionOdometry(actual, 0, ros::Time(108));
    EXPECT_FALSE(manager.consumeManualExitReplanRequest());
    manager.updateMissionOdometry(actual, 0, ros::Time(112.2));
    EXPECT_TRUE(manager.consumeManualExitReplanRequest());
    EXPECT_EQ(1U, manager.manual_path_.next);
    // 正常移动及转向分别重置计时；Z变化不能被误认成XY前进。
    manager.manual_path_.points={second}; manager.manual_path_.next=0;
    manager.updateMissionOdometry(actual, 0, ros::Time(110));
    manager.updateMissionOdometry(actual+Vector3d(.2,0,0), 0, ros::Time(114));
    manager.updateMissionOdometry(actual+Vector3d(.21,0,.5), 0, ros::Time(118));
    EXPECT_FALSE(manager.manual_path_.finished());
    manager.turn_in_place_session_active_=true;
    manager.updateMissionOdometry(actual+Vector3d(.21,0,.5), 0, ros::Time(124));
    EXPECT_FALSE(manager.manual_path_.finished());
    manager.turn_in_place_session_active_=false;
    manager.updateMissionOdometry(actual+Vector3d(.21,0,.1), 0, ros::Time(129.1));
    EXPECT_FALSE(manager.manual_path_.finished());
    EXPECT_EQ(0U, manager.manual_path_.next);
    EXPECT_TRUE(manager.consumeManualExitReplanRequest());
    // 无动态障碍时同样只按实际XY到达，预测规划不消耗航点。
    manager.manual_path_.points={second}; manager.manual_path_.next=0;
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(second, zero, zero, zero));
    EXPECT_FALSE(manager.manual_path_.finished());
    manager.updateMissionOdometry(Vector3d(second.x(), second.y(), .2), 0, ros::Time(130));
    EXPECT_TRUE(manager.manual_path_.finished());
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

  static void runManualTriggerQueue() {
    ros::NodeHandle nh("~manual_trigger_queue");
    auto manager = std::make_shared<FastExplorationManager>();
    initialize(nh, *manager);
    manager->manual_path_enabled_ = true;
    manager->manual_path_wait_for_trigger_ = true;
    const Vector3d waypoint(0.5, 0, 1);
    manager->manual_path_.points = {waypoint};
    struct QueuedOdometry : ros::CallbackInterface {
      std::shared_ptr<FastExplorationManager> manager;
      Vector3d position;
      CallResult call() override {
        manager->updateMissionOdometry(position, 0.0);
        return Success;
      }
    };
    auto queued = boost::make_shared<QueuedOdometry>();
    queued->manager = manager;
    queued->position = waypoint;
    FastExplorationFSM fsm;
    fsm.expl_manager_ = manager;
    fsm.fd_ = std::make_shared<FSMData>();
    fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
    fsm.state_ = WAIT_TRIGGER;
    fsm.odom_callback_queue_.addCallback(queued);
    auto trigger = boost::make_shared<nav_msgs::Path>();
    trigger->poses.resize(1);
    trigger->poses[0].pose.position.z = 1.0;
    fsm.triggerCallback(trigger);
    EXPECT_EQ(PLAN_TRAJ, fsm.state_);
    EXPECT_TRUE(manager->manual_path_started_);
    EXPECT_EQ(0u, manager->manual_path_.next);
    manager->updateMissionOdometry(waypoint, 0.0);
    EXPECT_EQ(1u, manager->manual_path_.next);
  }

  static void runManualPortalHandoff(bool with_entry) {
    ros::NodeHandle nh(with_entry ? "~manual_entry_exit" : "~manual_exit_only");
    nh.setParam("mission/task_search/require_stage2_detections", false);
    nh.setParam("mission/task_search/cruise_height", 1.0);
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.manual_path_enabled_ = true;
    manager.manual_path_wait_for_trigger_ = true;
    manager.manual_exit_enabled_ = true;
    manager.external_selection_only_ = true;
    const Vector3d first(0, 0, 1), nearest(0.6, 0, 1), portal(1.0, 0, 1);
    const Vector3d outside(1.2, 0, 1), zero = Vector3d::Zero();
    manager.manual_path_.points = with_entry ? std::vector<Vector3d>{first, nearest, outside}
                                            : std::vector<Vector3d>{first, outside, nearest, outside};
    auto& task = *manager.task_search_manager_;
    task.deferExitCrossing(true);
    task.external_landing_planner_ = true;
    if (with_entry) task.setCorridorFrame(Vector3d(-0.5, 0, 1), Vector3d::UnitX());
    manager.updateMissionOdometry(first, 0.0);
    EXPECT_EQ(0u, manager.manual_path_.next);
    EXPECT_EQ(FAIL, manager.planExploreMotion(first, zero, zero, zero));
    manager.startManualWaypointMission();
    manager.updateMissionOdometry(first, 0.0);
    EXPECT_EQ(1u, manager.manual_path_.next);
    EXPECT_TRUE(task.corridor_entry_crossed_);
    EXPECT_NEAR(task.corridor_origin_.x(), with_entry ? -0.5 : 0.0, 1e-6);
    // 出口检测已由真实地图回归测试覆盖，此处隔离验证航点与出口阶段之间的交接。
    task.exit_candidate_confirmed_ = task.exit_portal_outside_verified_ = true;
    task.exit_portal_center_ = portal;
    task.exit_outward_direction_ = Eigen::Vector2d::UnitX();
    task.lockExitPortal(ros::Time::now());
    manager.updateMissionOdometry(first, 0.0);
    EXPECT_TRUE(manager.manual_exit_latched_);
    EXPECT_EQ(2u, manager.manual_path_.points.size());
    EXPECT_EQ(!with_entry, manager.consumeManualExitReplanRequest());
    EXPECT_FALSE(task.stage3Active());
    EXPECT_EQ(SUCCEED, manager.planExploreMotion(first, zero, zero, zero));
    EXPECT_TRUE(manager.ed_->next_goal_.isApprox(nearest));
    // 预测起点到达最近点/出口不能替代真实到点，也不能提前交接降落。
    manager.planExploreMotion(portal, zero, zero, zero);
    EXPECT_EQ(1u, manager.manual_path_.next);
    EXPECT_FALSE(task.stage3Active());
    manager.updateMissionOdometry(nearest, 0.0);
    EXPECT_TRUE(manager.consumeManualExitReplanRequest());
    EXPECT_FALSE(manager.consumeManualExitReplanRequest());
    // 未给测试地图自由证据：只能重试出口，不能跳回列表或追加frontier。
    EXPECT_EQ(FAIL, manager.planExploreMotion(portal, zero, zero, zero));
    EXPECT_TRUE(task.exitTransitActive());
    manager.updateMissionOdometry(portal, 0.0);
    EXPECT_EQ(NO_FRONTIER, manager.planExploreMotion(portal, zero, zero, zero));
    EXPECT_EQ(TaskSearchManager::SEARCH_OUTSIDE_LANDING, task.mission_stage_);
    EXPECT_FALSE(task.landingRequested());
  }

  static void runManualAstar(const std::string& scenario) {
    ros::NodeHandle nh("~manual_astar_" + scenario);
    nh.setParam("mission/task_search/max_height", 1.90);
    nh.setParam("mission/global_no_return", false);
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
    const bool local_segments = scenario == "short_segments" || scenario == "short_detour" ||
        scenario == "disconnected_short" || scenario == "three_stage_bypass";
    if (local_segments) manager.manual_path_max_segment_length_ = 1.2;
    const bool exploration_style = scenario == "disconnected_short" || scenario == "three_stage_bypass";
    manager.manual_path_exploration_style_ = exploration_style;
    if (!exploration_style) manager.task_search_manager_.reset();
    else {
      manager.task_search_manager_->setMap(manager.sdf_map_);
      manager.task_search_manager_->exit_detection_enabled_ = false;
      manager.ep_->mission_use_forward_fallback_ = true;
    }
    manager.edt_environment_ = manager.planner_manager_->edt_environment_;
    ViewNode::caster_.reset(new RayCaster);
    Eigen::Vector3d origin, size;
    manager.sdf_map_->getRegion(origin, size);
    ViewNode::caster_->setParams(manager.sdf_map_->getResolution(), origin);
    auto& map = *manager.sdf_map_;
    const Vector3d start(scenario == "three_stage_bypass" ? -0.625 : -1.025, 0.025, 1.025);
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
      if (scenario != "same_voxel" && scenario != "short_segments") {
        const double width = scenario == "blocked" || scenario == "disconnected_short" ? 1.95 : 0.30;
        for (double x = -0.10; x <= 0.10; x += 0.025)
          for (double y = -width; y <= width; y += 0.025)
            for (double z = 0.025; z < 1.975; z += 0.025)
              map.setOccupied(Vector3d(x, y, z));
      }
      if (scenario == "three_stage_bypass") {
        // 分流检测读取原始占据，setOccupied只写膨胀层，故这里用真实射线观测立柱。
        pcl::PointCloud<pcl::PointXYZ> obstacle;
        for (double y = -0.30; y <= 0.30; y += 0.025)
          for (double z = 0.25; z <= 1.75; z += 0.025)
            obstacle.push_back(pcl::PointXYZ(-0.10, y, z));
        for (int i = 0; i < 6; ++i) map.inputPointCloud(obstacle, obstacle.size(), start);
      }
      map.updateESDF3d();
    }
    auto& planner = *manager.planner_manager_;
    if (local_segments) {
      Vector3d current = start;
      double largest_side = 0.0;
      bool saw_side = false, saw_pass = false, saw_return = false;
      for (int segment = 0; segment < 20 && manager.manual_path_.next == 0; ++segment) {
        manager.updateMissionOdometry(current, 0.0);
        const int result = manager.planExploreMotion(current, zero, zero, zero);
        if (scenario == "disconnected_short" && segment > 0 && result == FAIL) break;
        ASSERT_EQ(SUCCEED, result) << "segment=" << segment << " start=" << current.transpose();
        auto& trajectory = planner.local_data_.position_traj_;
        EXPECT_LE(trajectory.getLength(trajectory.getTimeSum() / 1000.0), 1.2 + 1e-6);
        EXPECT_TRUE(planner.isTrajectorySafe());
        EXPECT_TRUE(manager.last_requested_goal_.isApprox(target));
        EXPECT_EQ(0u, manager.manual_path_.next);
        const Vector3d end = trajectory.evaluateDeBoorT(trajectory.getTimeSum());
        EXPECT_GT((end - current).norm(), 0.03);
        const double side = std::fabs(end.y() - start.y());
        if (scenario == "three_stage_bypass") {
          if (segment == 0) {
            EXPECT_TRUE(manager.recovery_side_latched_);
            EXPECT_GT(side, 0.15);
            EXPECT_LT(end.x() - current.x(), 0.30);
            saw_side = true;
          }
          if (saw_side && end.x() > 0.25 && side > 0.30) saw_pass = true;
          if (saw_pass && side < largest_side - 0.10) saw_return = true;
          largest_side = std::max(largest_side, side);
        }
        current = end;
        manager.updateMissionOdometry(current, 0.0);
      }
      if (scenario == "disconnected_short") {
        EXPECT_EQ(0u, manager.manual_path_.next);
        EXPECT_GT(current.x() - start.x(), 0.10);
        EXPECT_LT(current.x(), 0.0);  // 局部推进不能穿墙，也不能消耗尚未到达的远点。
      } else {
        EXPECT_EQ(1u, manager.manual_path_.next);
        if (scenario == "three_stage_bypass") {
          EXPECT_TRUE(saw_side && saw_pass && saw_return);
        }
      }
      return;
    }
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

  static void runManualTurnBeforeAstar() {
    ros::NodeHandle nh("~manual_turn_before_astar");
    FastExplorationManager manager;
    initialize(nh, manager);
    manager.manual_path_enabled_ = manager.manual_path_exploration_style_ = true;
    manager.manual_path_max_segment_length_ = 1.2;
    manager.manual_path_use_astar_ = true;
    manager.turn_in_place_still_confirm_time_ = 0.0;
    const Vector3d start(0, 0, 1), target(0, -1.5, 1), zero = Vector3d::Zero();
    manager.manual_path_.points = {target};
    manager.updateMissionOdometry(start, 0.0);
    auto& task = *manager.task_search_manager_;
    task.commitCorridorTurn(-Vector3d::UnitY());
    // 航点之间尚无已知自由连通路径，已确认的转弯仍须先执行。
    ASSERT_EQ(SUCCEED, manager.planExploreMotion(start, zero, zero, zero));
    EXPECT_TRUE(manager.currentPlanIsTurnInPlace());
    auto& trajectory = manager.planner_manager_->local_data_;
    EXPECT_LT(trajectory.position_traj_.getLength(0.01), 1e-6);
    EXPECT_NEAR(-M_PI_2, trajectory.yaw_traj_.evaluateDeBoorT(trajectory.duration_)[0], 1e-3);
    EXPECT_EQ(0u, manager.manual_path_.next);
    manager.updateMissionOdometry(start, -M_PI_2);
    EXPECT_TRUE(task.turnYawAlignmentPending());  // 只能由FSM的实测对准确认释放。
    manager.completeTurnInPlace(start);
    EXPECT_FALSE(task.turnYawAlignmentPending());
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
                                     bool block_center, bool confirmed_return = false,
                                     double heading_offset = 0.0) {
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
    if (confirmed_return) {
      const double body_yaw = exit_yaw + heading_offset;
      const Vector3d portal = point(0, 0, 1);
      const Vector3d outside = point(0.65, lateral_offset, 1);
      task.external_landing_planner_ = true;
      task.cruise_height_ = portal.z();
      task.exit_portal_center_ = portal;
      task.exit_candidate_confirmed_ = task.exit_portal_outside_verified_ = true;
      task.exit_outward_direction_ = forward.head<2>();
      task.setCorridorFrame(point(-1, 0, 1), forward);
      task.global_no_return_ = true;
      task.beginExitCrossing(ros::Time::now());
      manager.updateMissionOdometry(outside, body_yaw);
      Vector3d goal; double goal_yaw;
      // 预测起点已到门心但实际机体仍在外侧，不能提前交接。
      ASSERT_TRUE(task.buildStage3Goal(portal, body_yaw, false, goal, goal_yaw));
      EXPECT_TRUE(goal.isApprox(portal));
      EXPECT_DOUBLE_EQ(body_yaw, goal_yaw);
      ASSERT_TRUE(task.exitTransitActive());
      ASSERT_EQ(SUCCEED, manager.planExploreMotion(outside, Vector3d::Zero(),
                      Vector3d::Zero(), Vector3d(body_yaw, 0, 0)));
      EXPECT_TRUE(manager.ed_->next_goal_.isApprox(portal, 0.03));
      EXPECT_TRUE(planner.isTrajectorySafe());
      const Vector3d end = planner.local_data_.position_traj_.evaluateDeBoorT(
          planner.local_data_.duration_);
      EXPECT_LT((end - outside).dot(forward), -0.4);
      EXPECT_LT((end - portal).norm(), 0.05);
      for (int i = 0; i <= 100; ++i) {
        const double heading = planner.local_data_.yaw_traj_.evaluateDeBoorT(
            planner.local_data_.duration_ * i / 100.0)[0];
        EXPECT_NEAR(0.0, std::atan2(std::sin(heading - body_yaw),
                                   std::cos(heading - body_yaw)), 0.02);
      }

      // 出口接管立即撤销旧探索段；已发布的出口段持续执行。
      FastExplorationFSM fsm;
      fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(&manager,
                                                 [](FastExplorationManager*) {});
      fsm.planner_manager_ = manager.planner_manager_;
      fsm.fd_ = std::make_shared<FSMData>();
      fsm.fp_ = std::make_shared<FSMParam>();
      fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
      fsm.fp_->emergency_brake_horizon_ = 0.12;
      fsm.active_traj_ = planner.local_data_;
      fsm.active_traj_.start_time_ = ros::Time::now();
      fsm.active_traj_valid_ = true;
      fsm.narrow_corridor_stage_ = true;
      fsm.state_ = EXEC_TRAJ;
      fsm.emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("exit_brake", 1);
      fsm.FSMCallback(ros::TimerEvent());
      EXPECT_EQ(PLAN_TRAJ, fsm.state_);
      EXPECT_TRUE(fsm.active_traj_braked_);
      fsm.state_ = EXEC_TRAJ;
      fsm.active_traj_braked_ = false;
      fsm.active_exit_transit_ = true;
      fsm.next_width_check_ = ros::Time::now() + ros::Duration(60.0);
      fsm.FSMCallback(ros::TimerEvent());
      EXPECT_EQ(EXEC_TRAJ, fsm.state_);
      EXPECT_FALSE(fsm.active_traj_braked_);
      // 旧候选等待发布期间锁门，也必须撤销，不能漏发门外终点。
      fsm.state_ = PUB_TRAJ;
      fsm.pending_exit_transit_ = false;
      fsm.FSMCallback(ros::TimerEvent());
      EXPECT_EQ(PLAN_TRAJ, fsm.state_);
      EXPECT_TRUE(fsm.active_traj_braked_);
      manager.updateMissionOdometry(portal, body_yaw);
      EXPECT_FALSE(task.buildStage3Goal(portal, body_yaw, false, goal, goal_yaw));
      EXPECT_EQ(TaskSearchManager::SEARCH_OUTSIDE_LANDING, task.mission_stage_);
      return;
    }
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
                              bool height_drift = false, bool fapp_hold = false) {
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

    if (fapp_hold) {
      manager.fapp_exploration_crossing_enabled_ = true;
      planner.fapp_config_.enabled = true;
      planner.fapp_memory_enabled_ = true;
      planner.fapp_planner_priority_ = true;
      planner.fapp_watched_valid_ = true;
      planner.fapp_watched_state_.stamp = ros::Time::now().toSec() - 20.0;
      // 模拟低框污染、失观且完整侧绕不可用，强制实际普通规划接管。
      planner.fapp_watched_lower_ = Vector3d(0.30, -1.50, 0.17);
      planner.fapp_watched_upper_ = Vector3d(1.40, 1.50, 1.43);
      manager.manual_fapp_active_ = true;
      manager.manual_fapp_hold_height_ = origin.z();
      manager.manual_fapp_forward_ = Vector3d::UnitX();
      ASSERT_FALSE(manager.planExplorationFappCrossing(displaced, Vector3d::Zero(),
                                                       Vector3d::Zero(), Vector3d::Zero()));
    }

    check(manager.planExploreMotion(displaced, Vector3d::Zero(), Vector3d::Zero(),
                                   Vector3d(0.10, 0, 0)) == SUCCEED,
          "same-side continuation failed to generate a trajectory");
    check(manager.recovery_side_latched_ && !manager.recovery_side_continuation_pending_,
          "0.22m lateral progress released the side or successful plan stayed pending");
    const double duration = planner.local_data_.duration_;
    const Vector3d end = planner.local_data_.position_traj_.evaluateDeBoorT(duration);
    if (fapp_hold) {
      EXPECT_GT((end-displaced).y()*side, 0.10);
      EXPECT_NEAR(end.z(), origin.z(), 0.01);
      for (double t=0; t<=duration; t+=0.01)
        EXPECT_NEAR(planner.local_data_.position_traj_.evaluateDeBoorT(t).z(), origin.z(), 0.01);
      manager.recordPublishedTrajectoryHeight();
      for (double error : {0.04, 0.06, 0.03}) {
        displaced.z() = origin.z() - error;
        task.updateRobotPose(displaced, 0.0);
        ASSERT_EQ(SUCCEED, manager.planExploreMotion(displaced, Vector3d::Zero(),
                                                    Vector3d::Zero(), Vector3d(0.10,0,0)));
        EXPECT_NEAR(manager.manual_fapp_hold_height_, origin.z(), 1e-9);
        auto& traj = planner.local_data_.position_traj_;
        for (double t=0; t<=traj.getTimeSum(); t+=0.01)
          EXPECT_LT(traj.evaluateDeBoorT(t).z(), origin.z()+0.02);
        EXPECT_GT((traj.evaluateDeBoorT(traj.getTimeSum())-displaced).head<2>().norm(), 0.10);
        manager.recordPublishedTrajectoryHeight();
      }
      Vector3d passed = displaced;
      passed.x() = manager.manual_fapp_exit_progress_ + 0.01;
      EXPECT_FALSE(manager.manualFappObservationMissing(passed, 0.0));
      EXPECT_TRUE(manager.manual_fapp_active_); // 未来规划端点不等于实际通过。
      Vector3d before = passed;
      before.x() -= 0.02;
      manager.manualFappObservationMissing(before, 0.0, true);
      EXPECT_TRUE(manager.manual_fapp_active_);
      manager.manualFappObservationMissing(passed, 0.0, true);
      EXPECT_FALSE(manager.manual_fapp_active_);
      EXPECT_FALSE(std::isfinite(manager.manual_fapp_hold_height_));
      return;
    }
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

  static void runDirectionalEscape(int scenario) {
    ros::NodeHandle nh("~directional_escape");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/resolution_astar", .05);
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& map=*manager.sdf_map_;
    const Vector3d pos(.025,.025,1.025), zero=Vector3d::Zero();
    for (double y=-1.5; y<=1.5; y+=.05)
      for (double z=.7; z<=1.35; z+=.05) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1.9,y,z));
        map.inputPointCloud(cloud,1,Vector3d(-1.9,y,z));
        cloud.front().x=-1.9; map.inputPointCloud(cloud,1,Vector3d(1.9,y,z));
      }
    // 膨胀层在后方/前方/前后，原始足迹自由，复现方向不同的边缘受阻。
    const double xmin=(scenario==1||scenario>=5)?0.0:-.6, xmax=scenario==0?.05:.6;
    for (double x=xmin; x<=xmax+1e-6; x+=.05)
      for (double y=-.05; y<=.05+1e-6; y+=.05)
        for (double z=.9; z<=1.1+1e-6; z+=.05) map.setOccupied(Vector3d(x+.025,y+.025,z+.025));
    if (scenario==4) {
      // 航点模式前后受阻，只有右侧能离开膨胀层。
      for(double x=-1.5;x<=1.5;x+=.05)
        for(double y=.10;y<=1.5;y+=.05)
          for(double z=.9;z<=1.1;z+=.05) map.setOccupied(Vector3d(x,y,z));
      manager.manual_path_enabled_=manager.manual_path_exploration_style_=true;
      manager.manual_path_wait_for_trigger_=true;
      manager.manual_path_.points={pos+Vector3d(1.,0,0)};
    }
    if (scenario==3) {
      // 接触数可先减少，但20cm末端仍在膨胀层，验证分步发布复核。
      for (double z=.90; z<=1.15; z+=.05) {
        Eigen::Vector3i id; map.posToIndex(Vector3d(.225,.025,z+.025),id);
        map.md_->occupancy_buffer_[map.toAddress(id)]=map.mp_->clamp_max_log_;
      }
    }
    map.updateESDF3d();
    manager.task_search_manager_->stable_progress_direction_=Eigen::Vector2d(scenario==3?-1:1,0);
    manager.task_search_manager_->stable_progress_direction_valid_=true;
    manager.recovery_odom_history_={pos-Vector3d(.5,0,0),pos};
    // 最近历史含向障碍一侧的侧移，不能代替当前高度的正后方候选。
    if (scenario==5) manager.recovery_odom_history_={pos+Vector3d(.3,.15,.1),pos};
    if (scenario==6) manager.recovery_odom_history_.clear();
    manager.task_search_manager_->visited_positions_=manager.recovery_odom_history_;
    if (scenario==1) {
      EXPECT_TRUE(manager.task_search_manager_->turnDirectionRetracesVisitedRoute(
          pos.head<2>(),Eigen::Vector2d(-1,0)));
    }
    manager.manual_fapp_active_=true;
    manager.manual_fapp_hold_height_=pos.z();
    ASSERT_TRUE(manager.planner_manager_->isControlledEscapePosition(pos));
    if(scenario>=5) {
      manager.manual_path_enabled_=manager.manual_path_exploration_style_=true;
      manager.manual_path_wait_for_trigger_=true;
      manager.manual_path_.points={pos+Vector3d(1.,0,0)};
      manager.startManualWaypointMission();
      ASSERT_TRUE(manager.planFailedMotionEscape(pos,zero,zero,zero));
      EXPECT_TRUE(manager.currentPlanIsEscapeStep());
      EXPECT_EQ(0u,manager.manual_path_.next);
      auto& curve=manager.planner_manager_->local_data_.position_traj_;
      double previous_x=pos.x();
      for(double t=0;t<=curve.getTimeSum();t+=.02) {
        const Vector3d point=curve.evaluateDeBoorT(t);
        EXPECT_LE(point.x(),previous_x+1e-6);
        EXPECT_NEAR(point.y(),pos.y(),1e-6);
        EXPECT_NEAR(point.z(),pos.z(),1e-6);
        previous_x=point.x();
      }
    } else if(scenario==4) {
      EXPECT_FALSE(manager.planFailedMotionEscape(pos,zero,zero,zero));
      manager.startManualWaypointMission();
      ASSERT_TRUE(manager.planFailedMotionEscape(pos,zero,zero,zero));
      EXPECT_TRUE(manager.currentPlanIsEscapeStep());
      EXPECT_EQ(0u,manager.manual_path_.next);
      EXPECT_TRUE(manager.manual_path_.points.front().isApprox(pos+Vector3d(1.,0,0)));
      EXPECT_LT(manager.ed_->next_goal_.y(),pos.y()-.04);
    } else {
      ASSERT_TRUE(manager.planInflationHistoryEscape(pos,zero,zero,zero));
    }
    const Vector3d delta=manager.ed_->next_goal_-pos;
    EXPECT_NEAR(delta.z(),0,1e-6);
    if (scenario==0) { EXPECT_GT(delta.x(),.04); EXPECT_NEAR(delta.y(),0,1e-6); }
    if (scenario==1||scenario>=5) {
      EXPECT_LT(delta.x(),-.04); EXPECT_GE(delta.x(),-.20-1e-6);
      EXPECT_NEAR(delta.y(),0,1e-6);
    }
    if (scenario==2) { EXPECT_GT(std::abs(delta.y()),.04); }
    if (scenario==3) {
      EXPECT_LT(delta.x(),-.04);
      EXPECT_TRUE(manager.incremental_escape_plan_);
      EXPECT_TRUE(manager.incrementalEscapePlanSafe());
      EXPECT_TRUE(manager.planner_manager_->isPositionInflated(manager.ed_->next_goal_));
      EXPECT_LT(manager.planner_manager_->rawFootprintCollisionCount(manager.ed_->next_goal_),
                manager.planner_manager_->rawFootprintCollisionCount(pos));
      manager.incremental_escape_plan_=false;
      EXPECT_FALSE(manager.incrementalEscapePlanSafe()); // 普通轨迹不能借用小步发布例外。
      manager.incremental_escape_plan_=true;
    }
    EXPECT_TRUE(manager.planner_manager_->isTrajectorySafe(.03,true));
    EXPECT_TRUE(manager.incrementalEscapePlanSafe() ||
        manager.planner_manager_->trajectoryClearsInflation(.45,.02,true));
    const Vector3d end=manager.ed_->next_goal_;
    if (scenario==7) {
      ASSERT_TRUE(manager.planner_manager_->isPositionSafe(end));
      manager.manual_fapp_active_=false;
      FastExplorationFSM fsm;
      fsm.expl_manager_=std::shared_ptr<FastExplorationManager>(&manager,[](FastExplorationManager*){});
      fsm.planner_manager_=manager.planner_manager_;
      fsm.fd_=std::make_shared<FSMData>(); fsm.fp_=std::make_shared<FSMParam>();
      fsm.fd_->state_str_={"INIT","WAIT","PLAN","PUB","EXEC","FINISH"};
      fsm.fd_->odom_pos_=end; fsm.fd_->odom_yaw_=0;
      fsm.fp_->max_tracking_error_xy_=1; fsm.fp_->max_tracking_error_z_=1;
      fsm.fp_->hard_tracking_error_xy_=1; fsm.fp_->hard_tracking_error_z_=1;
      fsm.fp_->tracking_error_grace_time_=1000;
      fsm.fp_->emergency_brake_horizon_=.12;
      fsm.active_traj_=manager.planner_manager_->local_data_;
      fsm.active_traj_.start_time_=ros::Time::now();
      fsm.active_traj_valid_=fsm.active_escape_step_=fsm.inflation_escape_active_=true;
      fsm.inflation_escape_clear_since_=ros::Time::now()-ros::Duration(1.0);
      fsm.state_=EXEC_TRAJ;
      fsm.safetyCallback(ros::TimerEvent());
      EXPECT_EQ(EXEC_TRAJ,fsm.state_);
      EXPECT_TRUE(fsm.inflation_escape_active_);
      EXPECT_FALSE(fsm.active_traj_braked_);
    }
    if (manager.planner_manager_->isPositionSafe(end)) {
      EXPECT_FALSE(manager.planInflationHistoryEscape(end,zero,zero,zero));
    }
  }

  static void runRecordedEscape(const char* fixture) {
    ros::NodeHandle nh("~recorded_escape");
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("manager/use_geometric_path", true);
    FastExplorationManager manager;
    initialize(nh, manager, false, true);
    std::ifstream input(fixture);
    int frames, count; Vector3d pos; double yaw;
    ASSERT_TRUE(bool(input >> frames >> pos.x() >> pos.y() >> pos.z() >> yaw >> count));
    for (int i=0; i<count; ++i) {
      Vector3d p; ASSERT_TRUE(bool(input >> p.x() >> p.y() >> p.z()));
      manager.recovery_odom_history_.push_back(p);
    }
    for (int f=0; f<frames; ++f) {
      Vector3d camera; ASSERT_TRUE(bool(input >> count >> camera.x() >> camera.y() >> camera.z()));
      pcl::PointCloud<pcl::PointXYZ> cloud; cloud.resize(count);
      for (auto& p : cloud) { ASSERT_TRUE(bool(input >> p.x >> p.y >> p.z)); }
      manager.sdf_map_->inputPointCloud(cloud, cloud.size(), camera);
      manager.sdf_map_->clearAndInflateLocalMap();
    }
    manager.sdf_map_->updateESDF3d();
    auto& planner=*manager.planner_manager_;
    for (double d : {-.4, -.2, 0., .1, .2, .4}) {
      Vector3d probe=pos+Vector3d(-d,0,0);
      std::cout << "escape recorded probe forward=" << d << " pos=" << probe.transpose()
                << " contacts=" << planner.rawFootprintCollisionCount(probe)
                << " inflated=" << planner.isPositionInflated(probe)
                << " clearance=" << manager.sdf_map_->getDistance(probe) << std::endl;
    }
    manager.task_search_manager_->stable_progress_direction_=Eigen::Vector2d(-1,0);
    manager.task_search_manager_->stable_progress_direction_valid_=true;
    EXPECT_FALSE(manager.planInflationHistoryEscape(pos,Vector3d::Zero(),Vector3d::Zero(),Vector3d(yaw,0,0)));
    manager.manual_fapp_active_=true;
    manager.manual_fapp_hold_height_=0.316;
    const bool planned=manager.planFailedMotionEscape(pos,Vector3d::Zero(),Vector3d::Zero(),Vector3d(yaw,0,0));
    ASSERT_TRUE(planned);
    EXPECT_LT(manager.ed_->next_goal_.x(),pos.x()-.04);
    EXPECT_NEAR(manager.ed_->next_goal_.z(),pos.z(),1e-6);
    EXPECT_TRUE(planner.isTrajectorySafe(.03,true));
    std::cout << "recorded escape planned=" << planned << std::endl;
    if (planned) std::cout << "recorded escape goal=" << manager.ed_->next_goal_.transpose() << std::endl;
  }

  static void runRecordedBypass(const char* fixture, bool lane_center = false) {
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
    Vector3d observed_pos;
    if (lane_center) {
      ASSERT_TRUE(bool(input >> observed_pos.x() >> observed_pos.y() >> observed_pos.z()));
    }
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
    if (lane_center) {
      manager.entry_forward_complete_ = true;
      manager.task_search_manager_->exit_detection_enabled_ = false;
      manager.task_search_manager_->cruise_height_ = .60;
      manager.horizontal_reference_height_ = .60;
      manager.task_search_manager_->updateRobotPose(pos, yaw);
      Vector3d next; double next_yaw;
      // 点云覆盖横移前后；恢复日志中已经发布的右侧承诺，不用后续地图重选左右。
      manager.active_split_lane_ = manager.makeSplitLane(pos, forward, obstacle, -1);
      manager.split_route_committed_ = true;
      manager.recovery_side_latched_ = true;
      manager.wide_side_bypass_active_ = true;
      manager.recovery_side_origin_ = pos;
      manager.wide_side_bypass_forward_ = forward;
      manager.recovery_side_dir_ = Vector3d(forward.y(), -forward.x(), 0);
      manager.wide_side_bypass_release_distance_ = manager.active_split_lane_.exit;
      manager.task_search_manager_->updateRobotPose(observed_pos, yaw);
      manager.recovery_odom_history_.push_back(observed_pos);
      ASSERT_TRUE(manager.buildSideRecoveryContinuation(observed_pos, yaw, next, next_yaw));
      std::cout << "recorded lane observed=" << observed_pos.transpose()
                << " next=" << next.transpose() << std::endl;
      EXPECT_GT((next-observed_pos).dot(forward), .19);
      EXPECT_GE(next.y(), observed_pos.y() - .05);
      Vector3d legacy; double legacy_yaw;
      manager.split_route_committed_ = false; // 同一地图、位置与右侧标记，复现旧的全通道居中。
      manager.entry_forward_complete_ = false; // 仅此对照禁用新增的接近后补认，真正复现旧行为。
      ASSERT_TRUE(manager.buildSideRecoveryContinuation(observed_pos, yaw, legacy, legacy_yaw));
      manager.entry_forward_complete_ = true;
      manager.split_route_committed_ = true;
      std::cout << "recorded legacy target=" << legacy.transpose() << std::endl;
      EXPECT_GT(next.y(), legacy.y()+.10);
      vector<Vector3d> path{observed_pos, next};
      manager.centrePassagePath(path, 1.0);
      for (const auto& p : path) EXPECT_GE(p.y(), observed_pos.y()-.05);
      return;
    }
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

TEST(ManualCruiseRecovery, WaypointEntryClimbsAlongExistingSideRoute) {
  fast_planner::TurnEscapeTest::runMovingClimb(0, false, true);
}
TEST(ManualCruiseRecovery, MidRouteRoofKeepsHorizontalTrajectory) {
  fast_planner::TurnEscapeTest::runMovingClimb(1, false, true);
}
TEST(ManualCruiseRecovery, UnknownOverheadKeepsHorizontalTrajectory) {
  fast_planner::TurnEscapeTest::runMovingClimb(2, false, true);
}
TEST(ManualCruiseRecovery, LowerWaypointCapsRecoveredHeight) {
  fast_planner::TurnEscapeTest::runMovingClimb(7, false, true);
}
TEST(ManualCruiseRecovery, RaisedRouteStillObeysWaypointSegmentLimit) {
  fast_planner::TurnEscapeTest::runMovingClimb(8, false, true);
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

TEST(ManualWaypoints, LowCrossingAdvancesIntermediateGoalWithoutReleasingHeight) {
  fast_planner::TurnEscapeTest::runManualLowWaypointProgress();
}

TEST(ManualWaypoints, SkipsOccupiedButReplansStalledGoalAndUsesActualXYArrival) {
  fast_planner::TurnEscapeTest::runManualWaypointSkip();
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

TEST(ManualWaypoints, LimitsEveryPublishedStraightSegment) {
  fast_planner::TurnEscapeTest::runManualAstar("short_segments");
}

TEST(ManualWaypoints, LimitsCurvedDetourSegmentsWithoutSkippingWaypoint) {
  fast_planner::TurnEscapeTest::runManualAstar("short_detour");
}

TEST(ManualWaypoints, DisconnectedRemoteGoalAllowsOnlyKnownFreeLocalProgress) {
  fast_planner::TurnEscapeTest::runManualAstar("disconnected_short");
}

TEST(ManualSuspended, FappBallUsesDescentLowStepsAndAscentBeforeSides) {
  fast_planner::TurnEscapeTest::runManualSuspended("below");
}
TEST(FappMemory, EmptyPartialInvalidAndExpiredObservationsUseSpatialMemory) {
  fast_planner::TurnEscapeTest::runFappMemory();
}
TEST(FappMemory, ConfirmedRegionRejectsSingleTallBoxButKeepsRealLowerObservation) {
  fast_planner::TurnEscapeTest::runFappTallOutlier();
}
TEST(FappMemory, OldSparseTallBoxIsRecheckedBeforeJoiningConfirmedEnvelope) {
  fast_planner::TurnEscapeTest::runLateHistoricalOutlier();
}
TEST(FappMemory, PlannerPriorityReusesExplorationFallbackInsideOldHistory) {
  fast_planner::TurnEscapeTest::runManualSuspended("inside_history");
}
TEST(FappMemory, StrictModeStillRejectsHistoricalEnvelopeOverlap) {
  fast_planner::TurnEscapeTest::runManualSuspended("inside_history_strict");
}
TEST(FappExploration, ContinuousCrossingWithoutManualWaypointsKeepsHistoryAndEndpoint) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_low");
}
TEST(FappExploration, LowerPassageUnavailableUsesSideWithoutStationaryDescent) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_side");
}
TEST(FappMemory, HistoricalUnderpassMovesForwardThroughContinuousDropout) {
  fast_planner::TurnEscapeTest::runManualSuspended("memory_below");
}
TEST(FappMemory, LeftWallSelectsRightBottomCurve) {
  fast_planner::TurnEscapeTest::runManualSuspended("memory_curve_right");
}
TEST(FappMemory, RecordedLowExtentSelectsSideRatherThanVerticalDescent) {
  fast_planner::TurnEscapeTest::runManualSuspended("recorded_history");
}
TEST(FappMemory, SideBypassUsesHistoricalBoundsThroughDropout) {
  fast_planner::TurnEscapeTest::runManualSuspended("memory_left");
}
TEST(ManualSuspended, DownwardPredictionLowersUnderpassHeight) {
  fast_planner::TurnEscapeTest::runManualSuspended("moving");
}
TEST(ManualSuspended, StaticHangingObstacleUsesSameStages) {
  fast_planner::TurnEscapeTest::runManualSuspended("mapped");
}
TEST(ManualSuspended, BallInBothMapAndFappStillPrefersUnderpass) {
  fast_planner::TurnEscapeTest::runManualSuspended("mapped_ball");
}
TEST(ManualSuspended, BallInBothMapAndFappUsesSideWhenBelowBlocked) {
  fast_planner::TurnEscapeTest::runManualSuspended("mapped_ball_left");
}
TEST(ManualSuspended, BlockedLowerPassageUsesLeftThenReturns) {
  fast_planner::TurnEscapeTest::runManualSuspended("left");
}
TEST(ManualSuspended, BlockedLowerPassageUsesRightThenReturns) {
  fast_planner::TurnEscapeTest::runManualSuspended("right");
}
TEST(ManualSuspended, UnknownBelowUsesSideInsteadOfDescending) {
  fast_planner::TurnEscapeTest::runManualSuspended("unknown_below");
}
TEST(ManualSuspended, NoLowerOrSidePassageKeepsWaypoint) {
  fast_planner::TurnEscapeTest::runManualSuspended("blocked");
}
TEST(ManualSuspended, DisabledFappDoesNotTriggerBallAvoidance) {
  fast_planner::TurnEscapeTest::runManualSuspended("disabled");
}
TEST(ManualSuspended, StaleFappDoesNotTriggerBallAvoidance) {
  fast_planner::TurnEscapeTest::runManualSuspended("stale");
}

TEST(ManualWaypoints, ExplorationBypassMovesAsidePassesThenReturns) {
  fast_planner::TurnEscapeTest::runManualAstar("three_stage_bypass");
}

TEST(ManualWaypoints, ConfirmedTurnRunsBeforeDisconnectedWaypointSearch) {
  fast_planner::TurnEscapeTest::runManualTurnBeforeAstar();
}

TEST(ManualWaypoints, EntryGatesWaypointsThenExitWaitsForActualArrival) {
  fast_planner::TurnEscapeTest::runManualPortalHandoff(true);
}

TEST(ManualWaypoints, TriggerDiscardsQueuedPreEntryOdometry) {
  fast_planner::TurnEscapeTest::runManualTriggerQueue();
}

TEST(ManualWaypoints, ExitOnlyUsesInitialHeadingAndSkipsOutsideWaypoints) {
  fast_planner::TurnEscapeTest::runManualPortalHandoff(false);
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
TEST(FappMemory, IsolatedBottomNeedsDistinctObservationsAndConfirmedLowSurvivesDropout) {
  fast_planner::TurnEscapeTest::runBottomConfirmation(false);
}
TEST(FappMemory, LaterEvidenceRevokesAlreadyMergedIsolatedBottom) {
  fast_planner::TurnEscapeTest::runBottomConfirmation(true);
}
TEST(RouteWidth, FeasibleStraightCompetesWithWiderRightRoute) {
  fast_planner::TurnEscapeTest::runStraightWidth(false);
}
TEST(RouteWidth, MirroredWiderLeftRouteUsesSameRanking) {
  fast_planner::TurnEscapeTest::runStraightWidth(true);
}

TEST(SideSelection, WiderDeadEndLosesToConnectedLeft) {
  fast_planner::TurnEscapeTest::runEarlySplit(false, false, true);
}
TEST(SideSelection, WiderDeadEndLosesToConnectedRight) {
  fast_planner::TurnEscapeTest::runEarlySplit(true, false, true);
}
TEST(SideSelection, NewlyObservedDividerIsAdoptedBeforeShortRecoveryRelease) {
  fast_planner::TurnEscapeTest::runEarlySplit(false, false, false, 2);
}
TEST(SideSelection, GenericBypassKeepsSideBeyondTwentyCentimetres) {
  fast_planner::TurnEscapeTest::runEarlySplit(false, false, false, true);
}
TEST(ManualLocalGoal, ImprovesWithinOriginalRadiusKeepsHeightOrderAndActualArrival) {
  fast_planner::TurnEscapeTest::runManualLocalGoal(0);
}
TEST(ManualLocalGoal, UnknownMapRetainsOriginalWaypoint) {
  fast_planner::TurnEscapeTest::runManualLocalGoal(1);
}
TEST(ManualLocalGoal, ConfirmedExitOwnsGoal) {
  fast_planner::TurnEscapeTest::runManualLocalGoal(2);
}
TEST(ManualLocalGoal, DynamicCrossingOwnsGoal) {
  fast_planner::TurnEscapeTest::runManualLocalGoal(3);
}
TEST(ManualLocalGoal, AutonomousExplorationIsUnaffected) {
  fast_planner::TurnEscapeTest::runManualLocalGoal(4);
}
TEST(ManualLocalGoal, UnreachableBetterPocketRetainsOriginalWaypoint) {
  fast_planner::TurnEscapeTest::runManualLocalGoal(5);
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"turn_escape_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(FappMemory, DistantBlockedCrossingStillPlansForwardApproach) {
  fast_planner::TurnEscapeTest::runManualSuspended("distant_blocked");
}
TEST(FappMemory, AsymmetricBodyAllowsCurveBelowOldMinimum) {
  fast_planner::TurnEscapeTest::runManualSuspended("asymmetric_low");
}

TEST(FappMemory, UnavailableCommittedSideStillAllowsForwardApproach) {
  fast_planner::TurnEscapeTest::runManualSuspended("distant_committed");
}
TEST(FappMemory, AsymmetricBodyFallsBackToSideBelowMinimum) {
  fast_planner::TurnEscapeTest::runManualSuspended("asymmetric_side");
}

TEST(FappHeightHold, OrdinaryFallbackKeepsLowRightRouteDespiteBadFrameAndDropout) {
  fast_planner::TurnEscapeTest::runSideRecovery(-1.0, false, true, false, true);
}
TEST(FappHeightHold, OrdinaryFallbackKeepsLowLeftRouteDespiteBadFrameAndDropout) {
  fast_planner::TurnEscapeTest::runSideRecovery(1.0, false, true, false, true);
}
TEST(FappHeightHold, CruiseRecoveryWaitsForActualClearance) {
  fast_planner::TurnEscapeTest::runMovingClimb(6);
}

TEST(FappExploration, BlockedRecoveryBeyondBallDoesNotRejectLowCrossing) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_recovery_roof");
}
TEST(FappExploration, OrdinarySidePreferenceDoesNotSuppressUnderpass) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_prior_side");
}
TEST(FappExploration, ConfiguredTenCentimetreMinimumAllowsTwelveCentimetreCrossing) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_low_minimum");
}
TEST(FappExploration, EntranceRejectionRetriesInsideWithoutReplacingValidRouteOnFailure) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_entry_retry");
}

TEST(FappExploration, CommittedSideAdvancesWithoutRecenteringOrHistorySidePush) {
  fast_planner::TurnEscapeTest::runPassageCentre(true);
}

TEST(FappExploration, CommittedRightSideSurvivesFullCrossingReplans) {
  fast_planner::TurnEscapeTest::runManualSuspended("exploration_prior_right_side");
}

TEST(FappOpening, RightLaneUsesOpeningAndRestoresNominalLimitsAfterActualPassage) {
  fast_planner::TurnEscapeTest::runDynamicOpening(-1.0);
}
TEST(FappOpening, LeftLaneUsesOpeningAndRestoresNominalLimitsAfterActualPassage) {
  fast_planner::TurnEscapeTest::runDynamicOpening(1.0);
}

TEST(ExitPortal, OvershootReturnsToCenterAndReplacesOrdinaryTarget) {
  fast_planner::TurnEscapeTest::runPendingExitApproach(0.0, 0.25, false, true);
}
TEST(ExitPortal, RotatedOvershootReturnsWithoutForwardAstarConstraint) {
  fast_planner::TurnEscapeTest::runPendingExitApproach(2.8, -0.25, false, true, 0.35);
}

TEST(RouteWidth, EarlyRightPassageKeepsSideUntilActualRearClearance) {
  fast_planner::TurnEscapeTest::runEarlySplit(false);
}
TEST(RouteWidth, EarlyLeftPassageMirrorsRight) {
  fast_planner::TurnEscapeTest::runEarlySplit(true);
}

TEST(RouteWidth, SteppedRightBypassCentersOnlyInsideRightLane) {
  fast_planner::TurnEscapeTest::runEarlySplit(false, true);
}
TEST(RouteWidth, SteppedLeftBypassCentersOnlyInsideLeftLane) {
  fast_planner::TurnEscapeTest::runEarlySplit(true, true);
}

TEST(SideBypassProgress, RecordedExitObstacleKeepsRightSubLane) {
  const char* fixture = std::getenv("LANE_CENTER_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires recorded exit-obstacle point clouds";
  fast_planner::TurnEscapeTest::runRecordedBypass(fixture, true);
}

TEST(InflationEscape, RecordedLatestExitContact) {
  const char* fixture=std::getenv("ESCAPE_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires latest exit point clouds";
  fast_planner::TurnEscapeTest::runRecordedEscape(fixture);
}

TEST(InflationEscape, RearBlockedPrefersForwardStep) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(0);
}
TEST(InflationEscape, FrontBlockedUsesRecordedHistory) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(1);
}
TEST(InflationEscape, BothEndsBlockedUsesMappedSideSpace) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(2);
}

TEST(InflationEscape, ImprovingContactAllowsIncrementalInflatedEndpoint) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(3);
}

TEST(InflationEscape, ManualWaypointFailureUsesRightSpaceWithoutConsumingGoal) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(4);
}

TEST(InflationEscape, ManualWaypointBacksStraightWhenHistoryDoesNotPointBackward) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(5);
}

TEST(InflationEscape, ReferenceContactDoesNotBrakeWhenOdometryCrossesFreeBoundary) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(7);
}
TEST(InflationEscape, ManualWaypointBacksStraightWithoutOdometryHistory) {
  fast_planner::TurnEscapeTest::runDirectionalEscape(6);
}

TEST(ManualSuspended, CloseDynamicObstacleUsesVerticalDescentThenCrosses) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(0);
}
TEST(ManualSuspended, BlockedSlopeDescendsBeforeApproachingDynamicEnvelope) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(6);
}
TEST(ManualSuspended, HistoricalEnvelopeUsesExplorationExecutionChecks) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(7);
}
TEST(ManualSuspended, FailedCrossingReusesExplorationForwardFallback) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(8);
}
TEST(ManualSuspended, VerticalDynamicDescentRejectsUnknownBodyColumn) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(1);
}
TEST(ManualSuspended, VerticalDynamicDescentRejectsOccupiedBodyColumn) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(2);
}
TEST(ManualSuspended, VerticalDynamicDescentRejectsHistoricalOverlap) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(3);
}
TEST(ManualSuspended, VerticalDynamicDescentRespectsMinimumHeight) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(4);
}
TEST(ManualSuspended, IntermediateWaypointInsideEnvelopeContinuesLowCrossing) {
  fast_planner::TurnEscapeTest::runCloseFappDescent(5);
}

TEST(PassableTurn, RightPassageBeatsLeftSlit) { fast_planner::TurnEscapeTest::runPassableTurn(0); }
TEST(PassableTurn, MirroredPassageBeatsRightSlit) { fast_planner::TurnEscapeTest::runPassableTurn(1); }
TEST(PassableTurn, WiderLeftPassageOverridesRightHistoryPrior) { fast_planner::TurnEscapeTest::runPassableTurn(2); }
TEST(PassableTurn, ComparableBranchesUseActualUHistory) { fast_planner::TurnEscapeTest::runPassableTurn(3); }
TEST(PassableTurn, MirroredHistoryReversesPrior) { fast_planner::TurnEscapeTest::runPassableTurn(4); }
TEST(PassableTurn, TwoSlitsDoNotConfirmTurn) { fast_planner::TurnEscapeTest::runPassableTurn(5); }
TEST(PassableTurn, WideButDisconnectedBranchIsRejected) { fast_planner::TurnEscapeTest::runPassableTurn(6); }
TEST(PassableTurn, UnknownBranchIsRejected) { fast_planner::TurnEscapeTest::runPassableTurn(7); }
TEST(PassableTurn, LowRoofBranchIsRejected) { fast_planner::TurnEscapeTest::runPassableTurn(8); }
TEST(PassableTurn, FirstTurnUsesMapWithoutShapePrior) { fast_planner::TurnEscapeTest::runPassableTurn(9); }
TEST(PassableTurn, RecordedOctober25CornerSelectsRight) {
  const char* fixture = std::getenv("TURN_PASSABILITY_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires October 25 pre-turn point clouds";
  fast_planner::TurnEscapeTest::runRecordedTurn(fixture);
}

TEST(BottomClearance, SideOfBodyNeedsTwentyCentimetersAboveObstacle) {
  fast_planner::TurnEscapeTest::runBottomClearance(false);
}
TEST(BottomClearance, AstarFindsHigherRouteInsteadOfRepeatedlyRejectingLowRoute) {
  fast_planner::TurnEscapeTest::runBottomClearance(true);
}

TEST(TurnHistory, RecentReturnIsRejectedWithoutDeletingHistory) { fast_planner::TurnEscapeTest::runTurnHistory(0); }
TEST(TurnHistory, ForwardContinuationIsAllowed) { fast_planner::TurnEscapeTest::runTurnHistory(1); }
TEST(TurnHistory, LateralDepartureIsAllowed) { fast_planner::TurnEscapeTest::runTurnHistory(2); }
TEST(TurnHistory, SeparateUShapedCorridorIsAllowed) { fast_planner::TurnEscapeTest::runTurnHistory(3); }
TEST(TurnHistory, SharedStartAloneIsNotReturn) { fast_planner::TurnEscapeTest::runTurnHistory(4); }
TEST(TurnHistory, SparseHistoryChecksConnectingSegments) { fast_planner::TurnEscapeTest::runTurnHistory(5); }
TEST(TurnHistory, MirroredReturnIsRejected) { fast_planner::TurnEscapeTest::runTurnHistory(6); }
TEST(TurnHistory, RecordedCh10ApproachRejectsOldLeftRoute) {
  const char* fixture=std::getenv("TURN_HISTORY_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires October 26 pre-CH10 odometry";
  fast_planner::TurnEscapeTest::runRecordedTurnHistory(fixture);
}
TEST(PassableTurn, RejectsRecentLeftReturnFromActualTurnAnchor) { fast_planner::TurnEscapeTest::runPassableTurn(10); }
TEST(PassableTurn, OnlyOldRouteDoesNotConfirmNewTurn) { fast_planner::TurnEscapeTest::runPassableTurn(11); }

TEST(UpperTurn, LowRightObstacleConfirmsTurnThenTriggersAscent) { fast_planner::TurnEscapeTest::runUpperTurn(0); }
TEST(UpperDetour, ObstacleBelowFortyCentimetresCanTriggerAscent) {
  fast_planner::TurnEscapeTest::runLowObstacleUpperDetour(false);
}
TEST(UpperDetour, FeasibleCenterGapDoesNotTriggerUnnecessaryAscent) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(0);
}
TEST(UpperDetour, EarlyAscentRejectsBlockedRoof) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(1);
}
TEST(UpperDetour, EarlyAscentRejectsUnknownUpperSpace) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(2);
}
TEST(UpperDetour, AlreadyAboveCombinationDoesNotKeepClimbing) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(3);
}
TEST(UpperDetour, TallObstacleUsesEarlyUpperRouteBeyondOldRiseLimit) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(4);
}
TEST(UpperDetour, TallObstacleCannotExceedCorridorHeight) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(5);
}
TEST(UpperDetour, SideHighPatchDoesNotVetoMeasuredObstacleUpperRoute) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(6);
}
TEST(UpperDetour, RealHighBarrierStillRejectsLowerCandidate) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(7);
}
TEST(UpperDetour, IsolatedHighNoiseDoesNotChangeObstacleTop) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(8);
}
TEST(UpperDetour, SeparateCeilingDoesNotReplaceLocalObstacleTop) {
  fast_planner::TurnEscapeTest::runEarlyUpperCombination(9);
}
TEST(UpperDetour, LowObstacleDoesNotAuthorizeBlockedUpperSpace) {
  fast_planner::TurnEscapeTest::runLowObstacleUpperDetour(true);
}
TEST(UpperDetour, SideHighObjectCannotTriggerAscentWhenForwardIsClear) {
  fast_planner::TurnEscapeTest::runUpperProgress(0);
}
TEST(UpperDetour, ClimbThenAdvanceOverObstacleThenReturnToCruise) {
  fast_planner::TurnEscapeTest::runUpperProgress(1);
}
TEST(UpperDetour, NearlyClearedTopDoesNotForceAnotherTwentyCentimetres) {
  fast_planner::TurnEscapeTest::runUpperProgress(2);
}
TEST(UpperDetour, IdlePhaseVerticalAscentExecutesAndReanchorsAtRelease) {
  fast_planner::TurnEscapeTest::runUpperExecution(false);
}
TEST(UpperDetour, FeasibleHigherDiagonalPrecedesLowerStationaryCandidate) {
  fast_planner::TurnEscapeTest::runUpperExecution(true);
}
TEST(UpperDetour, ClearedDynamicRegionDoesNotBlockStationaryAscent) {
  fast_planner::TurnEscapeTest::runUpperExecution(false, true);
}
TEST(InflationEscape, SafeStartAfterPlanFailureUsesSideBeforeBackingTowardFollower) {
  fast_planner::TurnEscapeTest::runSafeFailureSideEscape();
}
TEST(InflationEscape, FailedUpperRouteInInflationUsesSideBeforeBackingTowardFollower) {
  fast_planner::TurnEscapeTest::runSafeFailureSideEscape(true);
}
TEST(UpperTurn, UnknownAboveDoesNotAuthorizeTurn) { fast_planner::TurnEscapeTest::runUpperTurn(1); }
TEST(UpperTurn, RoofBlocksElevatedPassage) { fast_planner::TurnEscapeTest::runUpperTurn(2); }
TEST(UpperTurn, OrdinaryTurnDoesNotUseEarlyTakeover) { fast_planner::TurnEscapeTest::runUpperTurn(3); }
TEST(UpperTurn, DisabledVerticalDetourDoesNotAuthorizeElevatedTurn) { fast_planner::TurnEscapeTest::runUpperTurn(4); }
TEST(UpperTurn, RechecksMapAfterYawCompletes) { fast_planner::TurnEscapeTest::runUpperTurn(5); }
TEST(UpperTurn, MirroredLowObstacleUsesLeftPassage) { fast_planner::TurnEscapeTest::runUpperTurn(6); }

TEST(FappExploration, RecordedLeftLowCrossing) {
  const char* fixture=std::getenv("LOW_CROSSING_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires recorded crossing cloud snapshot";
  fast_planner::TurnEscapeTest::runRecordedLowCrossing(fixture);
}

TEST(ManualSuspended, RecordedLeftLowCrossingWithWaypointLaunch) {
  const char* fixture=std::getenv("WAYPOINT_LOW_CROSSING_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires waypoint launch parameters and recorded crossing cloud snapshot";
  fast_planner::TurnEscapeTest::runRecordedLowCrossing(fixture,true);
}

TEST(FappExploration, RightObstacleSelectsWiderLeftLowCrossing) {
  fast_planner::TurnEscapeTest::runAdaptiveLowCrossing(1.0);
}
TEST(FappExploration, LeftObstacleSelectsWiderRightLowCrossing) {
  fast_planner::TurnEscapeTest::runAdaptiveLowCrossing(-1.0);
}
TEST(FappExploration, AdaptiveLowCrossingRejectsBlockedBodySpace) {
  fast_planner::TurnEscapeTest::runAdaptiveLowCrossing(1.0,true);
}
TEST(FappExploration, AdaptiveLowCrossingRejectsUnknownBodySpace) {
  fast_planner::TurnEscapeTest::runAdaptiveLowCrossing(1.0,true,true);
}

TEST(ManualSuspended, DistantCrossingFailureKeepsPublishedForwardRouteAndRetries) {
  fast_planner::TurnEscapeTest::runManualSharedFappRetry(false);
}
TEST(ManualSuspended, SharedRetryFindsUnderpassWithoutLosingPublishedRoute) {
  fast_planner::TurnEscapeTest::runManualSharedFappRetry(true);
}

TEST(UpperTurn, PlannerRecoversAboveObstacleOutsideObsoleteWorkArea) {
  fast_planner::TurnEscapeTest::runUpperTurn(7);
}
TEST(CorridorHeight, ExitAndLandingRestoreOriginalSearchAndDescent) {
  fast_planner::TurnEscapeTest::runCorridorHeightScope();
}
TEST(CruiseHeightRecovery, LandingDoesNotTriggerCorridorAscent) {
  fast_planner::TurnEscapeTest::runMovingClimb(9);
}
