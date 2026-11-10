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
  }
public:
  static void runReturnStraight(int mode) {
    ros::NodeHandle nh("~return_straight_"+std::to_string(mode));
    FastExplorationManager manager; initialize(nh,manager);
    auto& task=*manager.task_search_manager_;
    auto& map=*manager.sdf_map_;
    task.setMap(manager.sdf_map_);
    task.exit_detection_enabled_=false;
    task.return_straight_enabled_=mode!=1;
    task.corridor_width_=1.3;
    const double entry_yaw=mode==6 ? M_PI/3.0 : 0.0;
    const Vector3d entry(std::cos(entry_yaw),std::sin(entry_yaw),0);
    const Vector3d start(0,0,.6);
    task.setCorridorFrame(start-3*entry,entry);
    task.corridor_entry_crossed_=true;
    const double yaw=entry_yaw+(mode==2 ? -M_PI/3.0 : (mode==5 ? -165.0 : -135.0)*M_PI/180.0);
    task.turn_detection_direction_=Eigen::Vector2d(std::cos(yaw),std::sin(yaw));
    task.stable_progress_direction_=task.turn_detection_direction_;
    task.width_origin_=start.head<2>();
    task.width_direction_=task.turn_detection_direction_;
    task.segment_origin_=start.head<2>();
    task.segment_direction_=task.turn_detection_direction_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    task.updateRobotPose(start,yaw);
    task.visited_positions_={start};
    if (mode==3 || mode==4) {
      Eigen::Vector3i id; map.posToIndex(start-.5*entry,id);
      map.md_->occupancy_buffer_[map.toAddress(id)]=mode==3 ? map.mp_->clamp_max_log_
          : map.mp_->clamp_min_log_-map.mp_->unknown_flag_;
    }
    Vector3d direction;
    if (mode>=1 && mode<=4) {
      EXPECT_FALSE(task.inferReturnStraightDirection(direction));
      EXPECT_FALSE(task.return_straight_active_);
      return;
    }
    ASSERT_TRUE(manager.requestMappedTurn(start,yaw));
    EXPECT_TRUE(task.return_straight_active_);
    EXPECT_TRUE(task.turnYawAlignmentPending());
    EXPECT_TRUE(task.turnDetectionDirection().isApprox(-entry.head<2>()));
    task.completeTurnYawAlignment();
    task.updateRobotPose(start-.7*entry,yaw);
    EXPECT_FALSE(task.mappedCorridorDirection(yaw,direction));
    double commanded_yaw=0.0;
    ASSERT_TRUE(task.corridorCurveYaw(start,commanded_yaw));
    EXPECT_NEAR(std::cos(commanded_yaw),-entry.x(),1e-6);
    EXPECT_NEAR(std::sin(commanded_yaw),-entry.y(),1e-6);
    double old_free,branch_free;
    EXPECT_FALSE(task.inferOccupancyTurnDirection(-entry,direction,old_free,branch_free));
    task.mission_stage_=TaskSearchManager::SEARCH_OUTSIDE_LANDING;
    EXPECT_FALSE(task.corridorCurveYaw(start,commanded_yaw));
  }

  static void runManualExit(int mode) {
    ros::NodeHandle nh("~manual_exit_"+std::to_string(mode));
    nh.setParam("mission/task_search/exit/portal_arrive_distance",.1);
    FastExplorationManager manager; initialize(nh,manager);
    auto& task=*manager.task_search_manager_;
    auto& map=*manager.sdf_map_;
    task.setMap(manager.sdf_map_);
    task.manual_exit_enabled_=mode!=1;
    task.manual_exit_center_=Vector3d(0,0,.6);
    task.manual_exit_yaw_=M_PI;
    task.require_stage2_detections_=false;
    task.external_landing_planner_=true;
    task.corridor_width_=1.3;
    task.setCorridorFrame(Vector3d(-3,0,.6),Vector3d::UnitX());
    task.corridor_entry_crossed_=true;
    task.turn_detection_direction_=task.stable_progress_direction_=-Eigen::Vector2d::UnitX();
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    if (mode==2) {
      for (const auto& p : {Vector3d(.5,.8,.6), Vector3d(.5,0,1.4), Vector3d(1.2,0,.6)}) {
        task.updateRobotPose(p,M_PI);
        EXPECT_FALSE(task.exit_portal_locked_);
        EXPECT_FALSE(task.final_exit_pose_published_);
      }
      return;
    }
    if (mode==3) {
      Eigen::Vector3i id;map.posToIndex(task.manual_exit_center_,id);
      map.md_->occupancy_buffer_[map.toAddress(id)]=map.mp_->clamp_max_log_;
    }
    task.updateRobotPose(Vector3d(.8,0,.6),M_PI);
    if (mode==1 || mode==3) {
      EXPECT_FALSE(task.exit_portal_locked_);
      EXPECT_EQ(task.mission_stage_,TaskSearchManager::SEARCH_CORRIDOR);
      EXPECT_FALSE(task.final_exit_pose_published_);
      return;
    }
    ASSERT_TRUE(task.exit_portal_locked_);
    EXPECT_TRUE(task.exit_portal_center_.isApprox(task.manual_exit_center_));
    EXPECT_FALSE(task.final_exit_pose_published_);
    Vector3d goal;double yaw;
    ASSERT_TRUE(task.buildStage3Goal(Vector3d(0,0,.6),0.7,false,goal,yaw));
    EXPECT_TRUE(goal.isApprox(task.manual_exit_center_));
    EXPECT_NEAR(std::fabs(yaw),M_PI,1e-6);
    EXPECT_FALSE(task.final_exit_pose_published_); // 预测起点到门不等于实机到门。
    task.updateRobotPose(Vector3d(.2,0,.6),M_PI);
    EXPECT_FALSE(task.final_exit_pose_published_);
    task.updateRobotPose(Vector3d(.05,0,.6),M_PI);
    EXPECT_TRUE(task.final_exit_pose_published_);
    EXPECT_FALSE(task.buildStage3Goal(Vector3d(.05,0,.6),M_PI,false,goal,yaw));
    EXPECT_EQ(task.mission_stage_,TaskSearchManager::SEARCH_OUTSIDE_LANDING);
  }

  static void runLowExit(int mode, bool transparent = true, bool solid_wall = false) {
    ros::NodeHandle nh("~low_exit_"+std::to_string(mode));
    FastExplorationManager manager; initialize(nh,manager,true,true);
    auto& task=*manager.task_search_manager_;
    auto& map=*manager.sdf_map_;
    task.setMap(manager.sdf_map_);
    task.transparent_tunnel_mode_=transparent;
    task.exit_outside_length_=.60;
    task.exit_footprint_extra_radius_=.20;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    auto mark=[&](double x,double y,double z,double value) {
      Eigen::Vector3i id;map.posToIndex(Vector3d(x,y,z),id);
      map.md_->occupancy_buffer_[map.toAddress(id)]=value;
    };
    for (double x=-1.8;x<1.8;x+=.05) {
      // 地板在机体范围以下，旧的上下0.4m投影却会把整片通道填成占据。
      if (x<.5)
        for (double y=-1.8;y<1.8;y+=.05) mark(x,y,-.075,map.mp_->clamp_max_log_);
      if (mode!=4 && (x<.5 || mode==5 || (mode==6 && x>.8))) {
        // 透明侧板只有高于飞行高度的细横梁，不要求机体高度有连续实体墙。
        for (int side : {-1,1}) {
          if (mode==3 && side==-1) continue;
          mark(x,side*.75,.625,map.mp_->clamp_max_log_);
          if (solid_wall)
            for (double z=.075;z<.65;z+=.05)
              mark(x,side*.75,z,map.mp_->clamp_max_log_);
        }
      }
      if (mode==2 && x>.55)
        for (double y=-1.8;y<1.8;y+=.05)
          for (double z=.075;z<.8;z+=.05)
            mark(x,y,z,map.mp_->clamp_min_log_-map.mp_->unknown_flag_);
    }
    if (mode==1)
      for (double y=-1.8;y<1.8;y+=.05)
        for (double z=.125;z<.8;z+=.05) mark(.85,y,z,map.mp_->clamp_max_log_);
    const Vector3d start(0,0,.233);
    TaskSearchManager::OpenExitResult result;
    const bool found=task.detectLocalMapExit(start,Eigen::Vector2d::UnitX(),result);
    if (mode!=0 || (!transparent && !solid_wall)) { EXPECT_FALSE(found); return; }
    ASSERT_TRUE(found);
    EXPECT_NEAR(result.portal_center.x(),.5,.16);
    EXPECT_NEAR(result.portal_center.y(),0,.08);
    EXPECT_TRUE(task.mapRelativeColumnOccupied(start,start.z()));
    EXPECT_FALSE(task.mapRelativeColumnOccupied(start,start.z(),true));
    EXPECT_TRUE(task.mapExitFootprintFree(result.portal_center));
  }

  static void runCorridorTurnBoundary(int mode) {
    ros::NodeHandle nh("~corridor_turn_boundary_"+std::to_string(mode));
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& task=*manager.task_search_manager_;
    auto& planner=*manager.planner_manager_;
    auto& map=*manager.sdf_map_;
    task.setMap(manager.sdf_map_);
    task.corridor_width_=1.30;
    task.setCorridorFrame(Vector3d(-1,0,.6),Vector3d::UnitX());
    task.corridor_entry_crossed_=true;
    task.curve_heading_enabled_=false;
    manager.corridor_low_flight_enabled_=true;
    manager.corridor_limit_start_=Vector3d(0,0,.6);
    planner.corridor_point_allowed_=[&](const Vector3d& p) { return manager.corridorPointAllowed(p); };
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    if (mode==2) {
      // 旧段边界外两厘米的实机位置：可识别下一段，但不能直接放行平移。
      const Vector3d actual(0,.49,.6);
      EXPECT_FALSE(planner.isPositionSafe(actual));
      EXPECT_FALSE(planner.isTurnPositionSafe(actual));
      EXPECT_TRUE(planner.isTurnPositionSafe(actual,false));
      task.updateRobotPose(actual,0.0);
      task.commitCorridorTurn(Vector3d::UnitY());
      EXPECT_TRUE(planner.isTurnPositionSafe(actual));
      Eigen::Vector3i index; map.posToIndex(actual,index);
      map.md_->occupancy_buffer_inflate_[map.toAddress(index)]=1;
      EXPECT_FALSE(planner.isTurnPositionSafe(actual,false));
      return;
    }
    if (mode==3) {
      task.recordSelectedGoal(Vector3d(.2,0,.6));
      manager.recovery_side_latched_=true;
      manager.recordPublishedTrajectoryHeight();
      EXPECT_TRUE(task.active_goal_valid_); // 普通发布或未发布的脱困候选不清目标。
      manager.escape_step_plan_=true;
      manager.recordPublishedTrajectoryHeight();
      EXPECT_FALSE(task.active_goal_valid_);
      EXPECT_FALSE(manager.recovery_side_latched_);
      task.recordSelectedGoal(Vector3d(.2,0,.6));
      manager.manual_fapp_active_=true;
      manager.recordPublishedTrajectoryHeight();
      EXPECT_TRUE(task.active_goal_valid_); // 动态下穿仍保留自身会话。
      manager.manual_fapp_active_=false;
      manager.low_probe_phase_=vertical_detour::LowProbePhase::DESCENDING;
      manager.recordPublishedTrajectoryHeight();
      EXPECT_TRUE(task.active_goal_valid_);
      manager.low_probe_phase_=vertical_detour::LowProbePhase::IDLE;
      manager.split_route_committed_=manager.recovery_side_latched_=manager.wide_side_bypass_active_=true;
      manager.recordPublishedTrajectoryHeight();
      EXPECT_FALSE(task.active_goal_valid_);
      EXPECT_TRUE(manager.committedSplitRoute());
      return;
    }
    const Vector3d start(-.2,0,.6);
    task.updateRobotPose(start,0.0);
    task.visited_positions_={start};
    task.recovery_turn_min_angle_deg_=45.0;
    task.recovery_turn_max_angle_deg_=90.0;
    task.recovery_turn_probe_length_=1.5;
    for (int x=0;x<map.mp_->map_voxel_num_.x();++x)
      for (int y=0;y<map.mp_->map_voxel_num_.y();++y)
        for (int z=0;z<map.mp_->map_voxel_num_.z();++z) {
          const Eigen::Vector3i id(x,y,z);
          Vector3d p; map.indexToPos(id,p);
          const bool occupied=p.x()>=.4 && (mode==0 || p.x()<.65) &&
              std::fabs(p.y())< (mode==0 ? .70 : .10);
          map.md_->occupancy_buffer_[map.toAddress(id)]=occupied ? map.mp_->clamp_max_log_ : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[map.toAddress(id)]=occupied;
        }
    Vector3d direction; double old_free,branch_free;
    if (mode==0) {
      task.corridor_width_=0.0;
      EXPECT_FALSE(task.inferOccupancyTurnDirection(Vector3d::UnitX(),direction,old_free,branch_free));
      task.corridor_width_=1.3;
      EXPECT_TRUE(task.inferOccupancyTurnDirection(Vector3d::UnitX(),direction,old_free,branch_free));
      EXPECT_GT(std::fabs(direction.y()),.7);
      task.corridor_origin_.x()=-3.0; // 已经过入口直行段。
      ASSERT_TRUE(manager.requestMappedTurn(start,0.0));
      EXPECT_TRUE(task.turnYawAlignmentPending());
      EXPECT_TRUE(manager.turn_in_place_session_active_);
      EXPECT_TRUE(planner.isTurnPositionSafe(start));
    } else {
      EXPECT_FALSE(task.inferOccupancyTurnDirection(Vector3d::UnitX(),direction,old_free,branch_free));
    }
  }
  static void runCorridorLowFlight(int scenario) {
    ros::NodeHandle nh("~corridor_low_"+std::to_string(scenario));
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 1.0);
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& map=*manager.sdf_map_;
    auto& planner=*manager.planner_manager_;
    auto& task=*manager.task_search_manager_;
    task.corridor_width_=1.30;
    task.setCorridorFrame(Vector3d(0,0,.6),Vector3d::UnitX());
    task.corridor_entry_crossed_=true;
    manager.corridor_low_flight_enabled_=true;
    manager.corridor_floor_z_=0.0;
    manager.corridor_limit_start_=Vector3d(0,0,.60);
    planner.corridor_point_allowed_=[&](const Vector3d& p) { return manager.corridorPointAllowed(p); };
    planner.path_finder_->corridor_point_allowed_=planner.corridor_point_allowed_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    std::fill(map.md_->distance_buffer_.begin(),map.md_->distance_buffer_.end(),1.0);
    // 0: isolated top; 1: suspended box; 2: supported full blockage; 3: right-hand gap;
    // 4: short obstacle; 5: unknown lower support; 6: free corridor and out-of-corridor landing.
    if (scenario<6) {
      for (double x=.425;x<=.575;x+=.05)
        for (double y=-.725;y<=.725;y+=.05)
          for (double z=.025;z<=.675;z+=.05) {
            if (scenario==3 && y<-.025) continue;
            bool occupied=z<=.65;
            if (scenario==0) occupied=z>.60 && z<.65;
            if (scenario==1) occupied=z>.40 && z<.65;
            if (scenario==4) occupied=z<.50;
            Eigen::Vector3i id; map.posToIndex(Vector3d(x,y,z),id);
            const int address=map.toAddress(id);
            map.md_->occupancy_buffer_[address]=scenario==5 && z<.60
                ? map.mp_->clamp_min_log_-map.mp_->unknown_flag_
                : (occupied ? map.mp_->clamp_max_log_ : map.mp_->clamp_min_log_);
            map.md_->occupancy_buffer_inflate_[address]=occupied;
          }
      EXPECT_EQ(manager.corridorBottomBlocked(Vector3d(0,0,.6),Vector3d::UnitX()),scenario==2);
      manager.updateCorridorFlightLimits(Vector3d(0,0,.6),0.0);
      EXPECT_NEAR(manager.corridor_height_limit_,scenario==2 ? 1.1 : .7,1e-6);
      EXPECT_EQ(manager.corridorPointAllowed(Vector3d(.1,0,.9)),scenario==2);
      EXPECT_FALSE(manager.corridorPointAllowed(Vector3d(.1,0,1.101)));
      return;
    }
    manager.updateCorridorFlightLimits(Vector3d(0,0,.6),0.0);
    EXPECT_TRUE(manager.corridorPointAllowed(Vector3d(.5,.2,.7)));
    EXPECT_FALSE(manager.corridorPointAllowed(Vector3d(.5,.2,.71)));
    EXPECT_FALSE(manager.corridorPointAllowed(Vector3d(.5,.60,.6))); // body crosses assumed wall
    EXPECT_FALSE(planner.isPathSafe({Vector3d(0,0,.6),Vector3d(.3,0,.8),Vector3d(.6,0,.6)}));
    ASSERT_TRUE(planner.planStraightTraj(Vector3d(0,0,.6),Vector3d(.5,0,.85)));
    EXPECT_FALSE(planner.isTrajectorySafe());
    planner.path_finder_->reset();
    EXPECT_EQ(planner.path_finder_->search(Vector3d(0,0,.6),Vector3d(.5,0,.8)),Astar::NO_PATH);
    planner.path_finder_->reset();
    EXPECT_EQ(planner.path_finder_->search(Vector3d(0,0,.6),Vector3d(.5,0,.6)),Astar::REACH_END);
    // The geometric corridor, rather than current vehicle XY, defines the lateral bound.
    task.latest_robot_pos_=Vector3d(.2,.3,.6);
    EXPECT_FALSE(manager.corridorPointAllowed(Vector3d(.5,.60,.6)));
    task.width_curve_valid_=true;
    task.width_curve_origin_=Eigen::Vector2d(0,0);
    task.width_curve_forward_=Eigen::Vector2d::UnitX();
    task.width_curve_.center=Vector3d(0,0,.5);
    EXPECT_TRUE(manager.corridorPointAllowed(Vector3d(1,.5,.6)));
    EXPECT_FALSE(manager.corridorPointAllowed(Vector3d(1,-.2,.6)));
    // Initial tracking overshoot may recover but cannot authorize another upward step.
    manager.corridor_limit_start_.z()=.8;
    manager.horizontal_reference_height_=.95;
    EXPECT_NEAR(manager.horizontalGoalHeight(.8),.8,1e-6);
    EXPECT_TRUE(manager.corridorPointAllowed(Vector3d(0,0,.75)));
    EXPECT_FALSE(manager.corridorPointAllowed(Vector3d(0,0,.81)));
    for (const auto stage : {TaskSearchManager::SEARCH_OUTSIDE_LANDING,
                            TaskSearchManager::APPROACH_LANDING, TaskSearchManager::LANDING}) {
      task.mission_stage_=stage;
      EXPECT_TRUE(manager.corridorPointAllowed(Vector3d(2,2,1.5)));
      EXPECT_TRUE(manager.corridorPointAllowed(Vector3d(2,2,.05)));
    }
    task.mission_stage_=TaskSearchManager::SEARCH_CORRIDOR;
    task.corridor_entry_crossed_=false;
    EXPECT_TRUE(manager.corridorPointAllowed(Vector3d(2,2,1.5)));
  }
  static void runIsolatedInflationNearVehicle() {
    ros::NodeHandle nh("~isolated_inflation_near_vehicle");
    FastExplorationManager manager;
    initialize(nh, manager, false, true);
    auto& map = *manager.sdf_map_;
    auto& planner = *manager.planner_manager_;
    const Vector3d noise(.225,.025,.625), sensor(.025,.025,.625);
    pcl::PointCloud<pcl::PointXYZ> hit, background;
    hit.push_back(pcl::PointXYZ(noise.x(),noise.y(),noise.z()));
    background.push_back(pcl::PointXYZ(1.225,.025,.625));
    for (int i=0; i<8; ++i)
      map.inputPointCloud(hit,hit.size(),sensor-Vector3d(.5,0,0));
    Eigen::Vector3i index; map.posToIndex(noise,index);
    const int address = map.toAddress(index);
    ASSERT_EQ(map.getOccupancy(noise),SDFMap::OCCUPIED);
    ASSERT_GE(map.md_->static_hit_evidence_[address],map.mp_->inflation_min_hit_evidence_);
    // 接近后没有近点回波：保留原始证据，但孤点不能重新膨胀封住机体旁通路。
    for (int i=0; i<12; ++i) {
      map.inputPointCloud(background,background.size(),sensor,.25);
      map.clearAndInflateLocalMap();
      EXPECT_EQ(map.getOccupancy(noise),SDFMap::OCCUPIED);
      EXPECT_EQ(map.getInflateOccupancy(noise),0);
      EXPECT_EQ(map.getInflateOccupancy(Vector3d(.325,.125,.625)),0);
      EXPECT_TRUE(planner.isPositionSafe(Vector3d(.325,.125,.625)));
    }
  }

  static void runCurrentInflationSupport() {
    ros::NodeHandle nh("~current_inflation_support");
    FastExplorationManager manager;
    initialize(nh,manager,false,true);
    auto& map = *manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_ = map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    const Vector3d source(.025,.025,.625), adjacent(.125,.025,.625);
    const auto address = [&](const Vector3d& p) {
      Eigen::Vector3i index; map.posToIndex(p,index); return map.toAddress(index);
    };
    map.md_->occupancy_buffer_[address(source)] = map.mp_->clamp_max_log_;
    map.md_->static_hit_evidence_[address(source)] = 2;
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getInflateOccupancy(source),0);
    map.md_->occupancy_buffer_[address(adjacent)] = map.mp_->clamp_max_log_;
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getInflateOccupancy(source),1); // 历史命中加当前邻格仍是有效障碍。
    EXPECT_FALSE(manager.planner_manager_->isPositionSafe(source));
    map.md_->occupancy_buffer_[address(adjacent)] = map.mp_->clamp_min_log_;
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getInflateOccupancy(source),0); // 支撑消失后清掉旧膨胀，不清原始格。
    EXPECT_EQ(map.getOccupancy(source),SDFMap::OCCUPIED);
    map.md_->static_hit_evidence_[address(source)] = 0;
    const Vector3d vertical = source+Vector3d(0,0,.20);
    map.md_->occupancy_buffer_[address(vertical)] = map.mp_->clamp_max_log_;
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getInflateOccupancy(source),1); // 稀疏细柱的竖向支撑保留。
    EXPECT_EQ(map.getInflateOccupancy(vertical),1);
    map.md_->occupancy_buffer_[address(vertical)] = map.mp_->clamp_min_log_;
    const Vector3d neighbors[] = {adjacent,source+Vector3d(0,.10,0)};
    for (const Vector3d& p : neighbors)
      map.md_->occupancy_buffer_[address(p)] = map.mp_->clamp_max_log_;
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getInflateOccupancy(source),1); // 当前多邻格无需等待历史命中。
    for (const Vector3d& p : neighbors)
      map.md_->occupancy_buffer_[address(p)] = map.mp_->clamp_min_log_;
    map.mp_->inflation_noise_filter_enabled_ = false;
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getInflateOccupancy(source),1); // 显式关闭过滤的语义不变。
  }

  static void runInflationEvidenceDecay(bool static_lock) {
    ros::NodeHandle nh(static_lock ? "~locked_static_evidence" : "~decaying_inflation_evidence");
    nh.setParam("sdf_map/static_retention_enabled",static_lock);
    nh.setParam("sdf_map/static_retention_required_hits",2);
    nh.setParam("sdf_map/p_hit",.65);
    nh.setParam("sdf_map/p_miss",.35);
    nh.setParam("sdf_map/p_max",.90);
    FastExplorationManager manager;
    initialize(nh,manager,false,true);
    auto& map = *manager.sdf_map_;
    const Vector3d sensor(-.475,.025,.625), source(.225,.025,.625);
    pcl::PointCloud<pcl::PointXYZ> hit, background;
    hit.push_back(pcl::PointXYZ(source.x(),source.y(),source.z()));
    background.push_back(pcl::PointXYZ(1.225,.025,.625));
    Eigen::Vector3i index; map.posToIndex(source,index);
    const int address = map.toAddress(index);
    for (int i=0; i<8; ++i) map.inputPointCloud(hit,hit.size(),sensor);
    ASSERT_EQ(map.md_->static_hit_evidence_[address],2);
    ASSERT_EQ(bool(map.md_->static_occupancy_locked_[address]),static_lock);
    map.inputPointCloud(background,background.size(),sensor,.25);
    EXPECT_EQ(map.md_->static_hit_evidence_[address],static_lock ? 2 : 1);
    EXPECT_EQ(map.getOccupancy(source),SDFMap::OCCUPIED); // 一次MISS未必清空概率占据。
    map.inputPointCloud(background,background.size(),sensor,.25);
    EXPECT_EQ(map.md_->static_hit_evidence_[address],static_lock ? 2 : 0);
    EXPECT_EQ(map.getOccupancy(source),static_lock ? SDFMap::OCCUPIED : SDFMap::FREE);
    if (!static_lock) {
      map.inputPointCloud(hit,hit.size(),sensor);
      EXPECT_EQ(map.md_->static_hit_evidence_[address],1);
      EXPECT_EQ(map.getOccupancy(source),SDFMap::OCCUPIED);
      map.clearAndInflateLocalMap();
      EXPECT_EQ(map.getInflateOccupancy(source),0);
    }
  }

  static void runRecordedStaticInflation(const char* fixture) {
    ros::NodeHandle nh("~recorded_static_inflation");
    FastExplorationManager manager;
    initialize(nh,manager,false,true);
    auto& map = *manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    std::ifstream input(fixture);
    Vector3d source, probe; int count;
    ASSERT_TRUE(bool(input >> source.x() >> source.y() >> source.z()
                           >> probe.x() >> probe.y() >> probe.z() >> count));
    for (int i=0; i<count; ++i) {
      Vector3d p; ASSERT_TRUE(bool(input >> p.x() >> p.y() >> p.z()));
      if (!map.isInMap(p)) continue;
      Eigen::Vector3i index; map.posToIndex(p,index);
      const int address = map.toAddress(index);
      map.md_->occupancy_buffer_[address] = map.mp_->clamp_max_log_;
      // 对记录中未知的历史命中次数取已达标，复现最容易被旧规则膨胀的情况。
      map.md_->static_hit_evidence_[address] = map.mp_->inflation_min_hit_evidence_;
    }
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_ = map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    map.clearAndInflateLocalMap();
    EXPECT_EQ(map.getOccupancy(source),SDFMap::OCCUPIED);
    EXPECT_FALSE(manager.planner_manager_->isSupportedOccupied(source));
    EXPECT_EQ(map.getInflateOccupancy(probe),0);
    EXPECT_TRUE(manager.planner_manager_->isPositionSafe(probe));
  }

  static void runBlindRangeFusion(bool observed) {
    ros::NodeHandle nh("~blind_range_fusion");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    const Vector3d sensor(.025, .025, .625), near(.225, .025, .625);
    pcl::PointCloud<pcl::PointXYZ> hit, background;
    for (double z : {.575, .625, .675}) {
      hit.push_back(pcl::PointXYZ(near.x(), near.y(), z));
      background.push_back(pcl::PointXYZ(1.225, .025, .625 + 6.0*(z-.625)));
    }
    if (observed) for (int i=0; i<8; ++i) map.inputPointCloud(hit, hit.size(), sensor);
    const int original = observed ? SDFMap::OCCUPIED : SDFMap::UNKNOWN;
    ASSERT_EQ(map.getOccupancy(near), original);
    for (int i=0; i<12; ++i) map.inputPointCloud(background, background.size(), sensor, .25);
    EXPECT_EQ(map.getOccupancy(near), original);
    EXPECT_EQ(map.getOccupancy(Vector3d(.525,.025,.625)), SDFMap::FREE);
    if (observed) EXPECT_FALSE(manager.planner_manager_->isTurnPositionSafe(sensor));
    // 退到可观测距离后，新的自由射线仍可正常消除旧占据，不永久冻结地图。
    for (int i=0; i<12; ++i)
      map.inputPointCloud(background, background.size(), sensor-Vector3d(.5,0,0), .25);
    EXPECT_EQ(map.getOccupancy(near), SDFMap::FREE);
  }

  static void runRetainedTurnHeight(bool blocked) {
    ros::NodeHandle nh("~retained_turn_height");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/max_search_time", 0.1);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& planner = *manager.planner_manager_;
    auto& task = *manager.task_search_manager_;
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.setCorridorFrame(Vector3d(-1,0,0), Vector3d::UnitX());
    task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
    manager.horizontal_reference_height_ = .625;
    const Vector3d zero = Vector3d::Zero();
    if (blocked) {
      for (double x : {-.025,.025,.075})
        for (double y : {-.025,.025,.075}) {
          Eigen::Vector3i index; map.posToIndex(Vector3d(x,y,.575),index);
          map.md_->occupancy_buffer_[map.toAddress(index)] = map.mp_->clamp_max_log_;
        }
    }
    for (double height : {.525, .425, .325, .595}) {
      const Vector3d actual(.025,.025,height);
      for (const Vector3d& velocity : {zero, Vector3d(.151,0,0),
                                      Vector3d(.224,0,0), Vector3d(0,0,.25)}) {
        manager.ground_ascent_plan_ = manager.turn_in_place_plan_ = false;
        const bool planned = manager.buildTurnInPlacePlan(actual,velocity,zero,zero,Vector3d::UnitY());
        if (blocked) {
          EXPECT_FALSE(manager.currentPlanIsGroundAscent());
          EXPECT_FALSE(manager.currentPlanIsTurnInPlace());
          EXPECT_DOUBLE_EQ(manager.horizontal_reference_height_, .625);
          return;
        }
        ASSERT_TRUE(planned);
        EXPECT_FALSE(manager.currentPlanIsGroundAscent());
        EXPECT_FALSE(manager.ground_ascent_active_);
        EXPECT_TRUE(manager.currentPlanIsTurnInPlace());
        EXPECT_TRUE(manager.ed_->next_goal_.isApprox(actual));
        EXPECT_LT(planner.local_data_.position_traj_.getLength(0.01), 1e-6);
        EXPECT_TRUE(planner.isTrajectorySafe());
        manager.recordPublishedTrajectoryHeight();
        EXPECT_DOUBLE_EQ(manager.horizontal_reference_height_, .625);
      }
    }
    manager.ground_ascent_plan_ = false;
    ASSERT_TRUE(manager.buildTurnInPlacePlan(Vector3d(.025,.025,.625),zero,zero,zero,Vector3d::UnitY()));
    EXPECT_TRUE(manager.currentPlanIsTurnInPlace());
    EXPECT_FALSE(manager.currentPlanIsGroundAscent());
  }

  static void runGroundAscentXY(int mode, bool manual = false) {
    ros::NodeHandle nh("~ground_ascent_xy_" + std::to_string(mode));
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& planner = *manager.planner_manager_;
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    const Vector3d anchor(.025, .025, .525), target(.025, .025, .925);
    const Vector3d zero = Vector3d::Zero();
    manager.ground_ascent_active_ = true;
    manager.ground_ascent_anchor_valid_ = true;
    manager.ground_ascent_anchor_ = anchor;
    manager.ground_ascent_target_ = target;
    manager.ground_ascent_still_confirm_time_ = 0.0;
    manager.horizontal_reference_height_ = target.z();
    if (manual) {
      manager.manual_path_enabled_ = manager.manual_path_exploration_style_ = true;
      manager.manual_path_started_ = true;
      manager.manual_path_.points = {Vector3d(1.425,.025,.625)};
    }
    if (mode == 2 || mode == 5) manager.obstacle_clearance_height_ = .85;
    if (mode == 0) {
      // 连续漂移重试不能重置XY；较大偏差先同高度回正，小偏差随升高收回。
      for (const Vector3d& offset : {Vector3d(.12, 0, 0), Vector3d(.16, .08, .03),
                                     Vector3d(.04, 0, .04)}) {
        const Vector3d actual = anchor + offset;
        if (manual) {
          ASSERT_EQ(manager.planExploreMotion(actual, zero, zero, zero), SUCCEED);
        } else {
          ASSERT_TRUE(manager.buildGroundAscentPlan(actual, zero, zero));
        }
        auto& trajectory = planner.local_data_.position_traj_;
        const Vector3d end = trajectory.evaluateDeBoorT(trajectory.getTimeSum());
        EXPECT_LT((end-target).head<2>().norm(), 1e-5);
        EXPECT_TRUE(manager.ground_ascent_target_.isApprox(target));
        EXPECT_TRUE(manager.ground_ascent_anchor_.isApprox(anchor));
        EXPECT_LT((trajectory.evaluateDeBoorT(0.0)-actual).norm(), 1e-5);
        if (offset.head<2>().norm() > manager.vertical_detour_xy_tolerance_) {
          for (double t=0; t<=trajectory.getTimeSum(); t+=.02)
            EXPECT_NEAR(trajectory.evaluateDeBoorT(t).z(), actual.z(), 1e-6);
        } else EXPECT_NEAR(end.z(), target.z(), 1e-5);
        manager.recordPublishedTrajectoryHeight();
        EXPECT_DOUBLE_EQ(manager.horizontal_reference_height_, target.z());
        EXPECT_TRUE(planner.isTrajectorySafe());
      }
      return;
    }
    if (mode == 1) {
      // 即使是可选恢复，回正路径有障碍也不能改锚点后在偏移处硬升。
      manager.ground_ascent_optional_ = true;
      const Vector3d actual = anchor + Vector3d(0, .30, 0);
      Eigen::Vector3i id; map.posToIndex(anchor + Vector3d(0, .15, 0), id);
      map.md_->occupancy_buffer_[map.toAddress(id)] = map.mp_->clamp_max_log_;
      EXPECT_FALSE(manager.buildGroundAscentPlan(actual, zero, zero));
      EXPECT_TRUE(manager.ground_ascent_active_);
      EXPECT_TRUE(manager.ground_ascent_target_.isApprox(target));
      EXPECT_TRUE(manager.ground_ascent_anchor_.isApprox(anchor));
      return;
    }
    const Vector3d actual = anchor + Vector3d(.12, 0, 0);
    if (mode == 3) ASSERT_TRUE(planner.planStraightTraj(actual, anchor));
    else ASSERT_TRUE(planner.planVerticalTraj(anchor, target));
    ASSERT_TRUE(planner.planYawTurnInPlace(zero, 0.0));
    FastExplorationFSM fsm;
    fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(&manager,
        [](FastExplorationManager*){});
    fsm.planner_manager_ = manager.planner_manager_;
    fsm.fd_ = std::make_shared<FSMData>();
    fsm.fp_ = std::make_shared<FSMParam>();
    fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
    fsm.fd_->odom_pos_ = actual;
    fsm.fd_->odom_vel_.setZero();
    fsm.fd_->odom_yaw_ = 0.0;
    fsm.fp_->emergency_brake_horizon_ = .12;
    fsm.fp_->max_tracking_error_xy_ = fsm.fp_->max_tracking_error_z_ = 1.0;
    fsm.fp_->hard_tracking_error_xy_ = .65;
    fsm.fp_->hard_tracking_error_z_ = 1.0;
    fsm.fp_->tracking_error_grace_time_ = 0.0;
    fsm.fp_->tracking_error_confirm_time_ = .30;
    fsm.active_traj_ = planner.local_data_;
    fsm.active_traj_.start_time_ = ros::Time::now();
    fsm.active_traj_valid_ = true;
    fsm.active_ground_ascent_ = mode != 4;
    fsm.active_escape_step_ = mode == 5;
    fsm.state_ = EXEC_TRAJ;
    fsm.last_mapped_turn_check_ = ros::Time::now() + ros::Duration(30.0);
    fsm.last_suspended_underpass_check_ = ros::Time::now() + ros::Duration(30.0);
    fsm.emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("ascent_xy_brake", 1);
    fsm.safetyCallback(ros::TimerEvent());
    EXPECT_FALSE(fsm.active_traj_braked_);
    EXPECT_EQ(fsm.state_, EXEC_TRAJ);
    EXPECT_TRUE(manager.ground_ascent_target_.isApprox(target));
    if (mode==2) {
      // 上升期间偏12cm不打断；完整段结束后才以同高度回正。
      Vector3d end_actual=target+Vector3d(.12,0,0);
      manager.latest_odom_height_=end_actual.z();
      manager.ground_ascent_attempt_pending_=true;
      manager.completeGroundAscentReference(end_actual);
      EXPECT_EQ(manager.ground_ascent_failures_,1);
      ASSERT_TRUE(manager.buildGroundAscentPlan(end_actual,zero,zero));
      auto& correction=planner.local_data_.position_traj_;
      const Vector3d end=correction.evaluateDeBoorT(correction.getTimeSum());
      EXPECT_NEAR(end.z(),end_actual.z(),1e-6);
      EXPECT_LT((end-target).head<2>().norm(),1e-6);
    }
  }

  static void runAscentRetryPolicy(bool manual) {
    ros::NodeHandle nh("~ascent_retry_policy");
    nh.setParam("manager/use_geometric_path",true);
    nh.setParam("astar/resolution_astar",.05);
    nh.setParam("astar/allocate_num",100000);
    FastExplorationManager manager; initialize(nh,manager);
    auto& map=*manager.sdf_map_; auto& task=*manager.task_search_manager_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),map.mp_->clamp_min_log_);
    std::fill(map.md_->occupancy_buffer_inflate_.begin(),map.md_->occupancy_buffer_inflate_.end(),0);
    std::fill(map.md_->distance_buffer_.begin(),map.md_->distance_buffer_.end(),1.0);
    task.max_search_height_=1.50; task.min_search_height_=.10;
    task.global_no_return_=false;
    task.stable_progress_direction_=Eigen::Vector2d(1,0);
    task.stable_progress_direction_valid_=true;
    const Vector3d pos(.025,.025,.60),zero=Vector3d::Zero();
    EXPECT_NEAR(manager.obstacleAscentTargetHeight(.85),1.10,1e-6);
    EXPECT_NEAR(manager.obstacleAscentTargetHeight(.70),1.025,1e-6);
    EXPECT_NEAR(manager.obstacleAscentTargetHeight(.95),1.15,1e-6);
    manager.ground_ascent_active_=true;
    manager.ground_ascent_anchor_valid_=true;
    manager.ground_ascent_target_=Vector3d(pos.x(),pos.y(),1.10);
    manager.obstacle_clearance_height_=1.05;
    manager.ground_ascent_still_confirm_time_=0;
    manager.inflation_history_escape_enabled_=true;
    manager.inflation_wall_pull_enabled_=false;
    manager.recovery_odom_history_={pos-Vector3d(.70,0,0),pos};
    if (manual) {
      manager.manual_path_enabled_=manager.manual_path_exploration_style_=true;
      manager.manual_path_started_=true;
      manager.manual_path_.points={Vector3d(1.4,.025,.60)};
    }
    const auto plan=[&](const Vector3d& p) {
      manager.ground_ascent_plan_=false;
      manager.ground_ascent_alternative_plan_=false;
      manager.escape_step_plan_=false;
      return manual ? manager.planExploreMotion(p,zero,zero,zero)==SUCCEED
                    : manager.buildGroundAscentPlan(p,zero,zero);
    };
    // 等待静止并非失败，不按100Hz计数。
    ASSERT_FALSE(manager.buildGroundAscentPlan(pos,Vector3d(.3,0,0),zero));
    EXPECT_EQ(manager.ground_ascent_failures_,0);
    for (int attempt=1;attempt<=3;++attempt) {
      ASSERT_TRUE(plan(pos));
      manager.recordPublishedTrajectoryHeight();
      manager.completeGroundAscentReference(pos);
      manager.reportGroundAscentFailure("duplicate callback");
      EXPECT_EQ(manager.ground_ascent_failures_,attempt);
      EXPECT_FALSE(manager.ground_ascent_retreated_);
    }
    ASSERT_FALSE(plan(pos));
    ASSERT_TRUE(manager.planFailedMotionEscape(pos,zero,zero,zero));
    EXPECT_TRUE(manager.currentPlanIsEscapeStep());
    const Vector3d retreat=manager.ed_->next_goal_;
    EXPECT_LE(retreat.x(),pos.x()-.30+1e-6);
    EXPECT_NEAR(retreat.z(),pos.z(),1e-6);
    EXPECT_FALSE(manager.ground_ascent_retreated_);
    manager.recordPublishedTrajectoryHeight();
    EXPECT_TRUE(manager.ground_ascent_retreated_);
    for (int attempt=4;attempt<=6;++attempt) {
      ASSERT_TRUE(plan(retreat));
      const Vector3d target=manager.ed_->next_goal_;
      EXPECT_GT(target.x(),retreat.x()+.20);
      EXPECT_NEAR(target.z(),1.10,1e-6);
      auto& curve=manager.planner_manager_->local_data_.position_traj_;
      const Vector3d middle=curve.evaluateDeBoorT(curve.getTimeSum()*.5);
      EXPECT_GT(middle.x(),retreat.x()); EXPECT_GT(middle.z(),retreat.z());
      manager.recordPublishedTrajectoryHeight();
      manager.completeGroundAscentReference(retreat);
      EXPECT_EQ(manager.ground_ascent_failures_,attempt);
    }
    ASSERT_TRUE(plan(retreat));
    const Vector3d side=manager.ed_->next_goal_;
    EXPECT_GT(std::fabs(side.y()-retreat.y()),.20);
    EXPECT_NEAR(side.z(),1.10,1e-6);
    manager.recordPublishedTrajectoryHeight();
    manager.latest_odom_height_=side.z();
    manager.completeGroundAscentReference(side);
    EXPECT_FALSE(manager.buildGroundAscentPlan(side,zero,zero));
    EXPECT_FALSE(manager.ground_ascent_active_);
    EXPECT_EQ(manager.ground_ascent_failures_,0);
    if (manual) EXPECT_EQ(manager.manual_path_.next,0u);

    // 20cm只约束原地升高；前进/侧向斜升只差10cm时不能再额外抬高。
    const Vector3d high_start(.025,.025,1.0);
    manager.ground_ascent_active_=true;
    manager.ground_ascent_target_=Vector3d(.025,.025,1.10);
    manager.obstacle_clearance_height_=1.05;
    manager.latest_odom_height_=high_start.z();
    manager.ground_ascent_retreated_=true;
    for (int failures:{3,6}) {
      manager.ground_ascent_failures_=failures;
      ASSERT_TRUE(plan(high_start));
      auto& trajectory=manager.planner_manager_->local_data_.position_traj_;
      const Vector3d end=trajectory.evaluateDeBoorT(trajectory.getTimeSum());
      EXPECT_NEAR(end.z()-high_start.z(),.10,1e-6);
      EXPECT_GT((end-high_start).head<2>().norm(),.20);
    }
  }

  static void runBlockedAscentRetreat(bool rear_blocked, bool manual = false) {
    ros::NodeHandle nh("~blocked_ascent_retreat");
    nh.setParam("manager/use_geometric_path",true);
    nh.setParam("astar/resolution_astar",.05);
    nh.setParam("astar/allocate_num",100000);
    nh.setParam("astar/lambda_heu",1.0);
    nh.setParam("astar/max_search_time",1.0);
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& map=*manager.sdf_map_;
    auto& planner=*manager.planner_manager_;
    const Vector3d pos(.025,.025,.525), target(.025,.025,.95), zero=Vector3d::Zero();
    if (manual) {
      manager.manual_path_enabled_ = manager.manual_path_exploration_style_ = true;
      manager.manual_path_started_ = true;
      manager.manual_path_.points = {Vector3d(1.425,.025,.625)};
    }
    manager.ground_ascent_active_=true;
    manager.ground_ascent_anchor_valid_=true;
    manager.ground_ascent_target_=target;
    manager.ground_ascent_anchor_=pos;
    manager.obstacle_clearance_height_=.85;
    manager.ground_ascent_still_confirm_time_=0.0;
    manager.inflation_history_escape_enabled_=true;
    manager.inflation_wall_pull_enabled_=false;
    manager.task_search_manager_->global_no_return_=false;
    manager.task_search_manager_->stable_progress_direction_=Eigen::Vector2d(1,0);
    manager.task_search_manager_->stable_progress_direction_valid_=true;
    manager.recovery_odom_history_={pos-Vector3d(.70,0,0),pos};
    for (int x=0;x<map.mp_->map_voxel_num_.x();++x)
      for (int y=0;y<map.mp_->map_voxel_num_.y();++y)
        for (int z=0;z<map.mp_->map_voxel_num_.z();++z) {
          const Eigen::Vector3i id(x,y,z); Vector3d p;map.indexToPos(id,p);
          // 当前位置前后均能平移，但上升柱被阻挡；退20cm仍会擦到上方障碍。
          const bool roof=p.x()>=-.15 && p.x()<=.20 &&
              std::fabs(p.y()-.025)<.25 && p.z()>=.75 && p.z()<=.85;
          const bool rear=rear_blocked && p.x()>=-.30 && p.x()<=-.20 &&
              std::fabs(p.y()-.025)<.25 && p.z()<.70;
          const int address=map.toAddress(id);
          map.md_->occupancy_buffer_[address]=(roof||rear)?map.mp_->clamp_max_log_:map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address]=roof||rear;
          map.md_->distance_buffer_[address]=1.0;
        }
    ASSERT_TRUE(planner.isPositionSafe(pos));
    ASSERT_FALSE(manager.buildGroundAscentPlan(pos,zero,zero));
    const bool escaped=manager.planFailedMotionEscape(pos,zero,zero,zero);
    if (rear_blocked) { EXPECT_FALSE(escaped); return; }
    ASSERT_TRUE(escaped);
    ASSERT_TRUE(manager.currentPlanIsEscapeStep());
    EXPECT_FALSE(manager.currentPlanIsGroundAscent());
    Vector3d end=manager.ed_->next_goal_;
    EXPECT_LT(end.x(),pos.x()-.30);
    EXPECT_NEAR(end.z(),pos.z(),1e-6);
    EXPECT_TRUE(planner.isTrajectorySafe(.03,true));
    EXPECT_TRUE(manager.obstacleAscentPending());
    EXPECT_TRUE(manager.ground_ascent_anchor_valid_);
    EXPECT_TRUE(manager.ground_ascent_target_.isApprox(target));
    manager.recordPublishedTrajectoryHeight();
    EXPECT_FALSE(manager.ground_ascent_anchor_valid_);
    EXPECT_FALSE(manager.groundAscentXYCorrectionNeeded(end));
    EXPECT_DOUBLE_EQ(manager.ground_ascent_target_.z(),target.z());
    Vector3d elevated=end; elevated.z()=target.z();
    EXPECT_TRUE(manager.isKnownSafeGroundAscentPath(end,elevated));
    manager.latest_odom_height_=end.z();
    ASSERT_TRUE(manager.buildGroundAscentPlan(end,zero,zero));
    EXPECT_TRUE(manager.currentPlanIsGroundAscent());
    EXPECT_TRUE(manager.ground_ascent_anchor_valid_);
    EXPECT_LT((manager.ground_ascent_anchor_-end).norm(),1e-6);
    auto& curve=planner.local_data_.position_traj_;
    EXPECT_NEAR(curve.evaluateDeBoorT(curve.getTimeSum()).z(),target.z(),1e-5);
    EXPECT_TRUE(planner.isTrajectorySafe());
  }

  static void runCruiseRecoveryTolerance() {
    ros::NodeHandle nh("~cruise_recovery_tolerance");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    manager.ground_ascent_still_confirm_time_ = 0.0;
    EXPECT_DOUBLE_EQ(manager.ground_ascent_height_tolerance_, 0.10);
    const Vector3d zero = Vector3d::Zero();
    for (double height : {.535, .515}) {
      const Vector3d actual(.025, .025, height);
      manager.ground_ascent_active_ = manager.ground_ascent_optional_ = true;
      manager.ground_ascent_anchor_valid_ = true;
      manager.ground_ascent_target_ = Vector3d(.025, .025, .625);
      manager.ground_ascent_plan_ = false;
      const bool planned = manager.buildGroundAscentPlan(actual, zero, zero);
      if (height > .525) {
        EXPECT_FALSE(planned);
        EXPECT_FALSE(manager.ground_ascent_active_);
        EXPECT_FALSE(manager.currentPlanIsGroundAscent());
      } else {
        EXPECT_TRUE(planned);
        EXPECT_TRUE(manager.currentPlanIsGroundAscent());
        EXPECT_NEAR(manager.ed_->next_goal_.z(), height+.20,1e-6);
      }
    }
  }

  static void runFappReferenceClearance(int mode) {
    ros::NodeHandle nh("~fapp_reference_clearance_" + std::to_string(mode));
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/max_search_time", 0.1);
    nh.setParam("sdf_map/virtual_ceil_height", -10.0);
    nh.setParam("fapp/enabled", true);
    nh.setParam("fapp/memory_enabled", true);
    nh.setParam("fapp/planner_priority", true);
    nh.setParam("fapp/vehicle_top_height", 0.05);
    nh.setParam("fapp/vehicle_bottom_height", 0.20);
    nh.setParam("mission/vertical_detour/underpass/clearance", 0.10);
    FastExplorationManager manager;
    initialize(nh, manager, false, true);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    manager.task_search_manager_->setMap(manager.sdf_map_);
    manager.task_search_manager_->min_search_height_ = manager.underpass_min_height_ = .05;
    manager.fapp_exploration_crossing_enabled_ = true;
    manager.manual_fapp_active_ = true;
    manager.manual_fapp_forward_ = Vector3d::UnitX();
    manager.manual_fapp_hold_height_ = .60;
    manager.manual_fapp_exit_progress_ = 1.0;
    // 2026-10-28 00:09:04日志中的原始最低观测；旧目标约0.08，新目标约0.19。
    const double bottom = .2901487946510315, expected = bottom-.10;
    fuel_fapp::DynamicObstacleState state;
    state.position = Vector3d(.45,0,bottom+.10);
    state.size = Vector3d::Constant(.20);
    state.stamp = ros::Time::now().toSec();
    planner.fapp_states_ = {state};
    planner.watchFappObstacle(&state);
    Vector3d lower, upper; double age;
    ASSERT_TRUE(planner.watchedFappBounds(lower,upper,age));
    EXPECT_NEAR(lower.z(), expected, 1e-12);
    EXPECT_NEAR(upper.z(), bottom+.20+.20+.10, 1e-12);
    EXPECT_NEAR(planner.fappUnderpassTargetHeight(lower.z(),.06),expected,1e-12);
    EXPECT_TRUE(planner.isWatchedFappPositionSafe(Vector3d(.45,0,expected)));
    EXPECT_FALSE(planner.isWatchedFappPositionSafe(Vector3d(.45,0,expected+.001)));
    EXPECT_TRUE(planner.fappPositionSafe(Vector3d(.45,0,expected),state.stamp,state.stamp));
    EXPECT_FALSE(planner.fappPositionSafe(Vector3d(.45,0,expected+.001),state.stamp,state.stamp));
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    if (mode == 2) {
      // 真实静态层仍阻止下降；不能因下穿高度修正而绕过静态碰撞检查。
      for (double x=-1.975; x<2.0; x+=.05)
        for (double y=-1.975; y<2.0; y+=.05) {
          Eigen::Vector3i index; map.posToIndex(Vector3d(x,y,.375),index);
          map.md_->occupancy_buffer_[map.toAddress(index)] = map.mp_->clamp_max_log_;
        }
    }
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_ = map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    map.clearAndInflateLocalMap(); map.updateESDF3d();
    const Vector3d start(-.50,.025,.60), zero=Vector3d::Zero();
    if (mode == 1) {
      std::vector<Vector3d> path;
      ASSERT_TRUE(manager.buildManualFappUnderpassPath(start,Vector3d(1.5,.025,.60),path));
      EXPECT_NEAR(path.back().z(),expected,1e-12);
      EXPECT_TRUE(planner.isFappPathSafe(path));
      EXPECT_TRUE(planner.isPathSafe(path));
      return;
    }
    const bool planned = manager.planLocalFappUnderpass(start,zero,zero,zero);
    if (mode == 2) {
      EXPECT_FALSE(planned);
      EXPECT_EQ(manager.low_probe_phase_,vertical_detour::LowProbePhase::IDLE);
      return;
    }
    ASSERT_TRUE(planned);
    EXPECT_TRUE(manager.currentPlanIsVerticalDetour());
    EXPECT_NEAR(manager.ed_->next_goal_.z(),expected,1e-12);
    EXPECT_TRUE(planner.isTrajectorySafe());
    Vector3d reached = manager.ed_->next_goal_;
    if (mode == 3 || mode == 8 || mode == 9) reached.z() = .15;
    if (mode == 9) reached.y() += .12;
    if (mode == 4) reached.z() = .05;
    if (mode == 5) reached.z() = .049;
    if (mode == 6) reached.z() = expected+.001;
    if (mode == 7) reached.z() = .10;
    manager.recordPublishedTrajectoryHeight();
    manager.completeVerticalDetourReference();
    if (mode == 5 || mode == 6) {
      Vector3d next; double next_yaw; bool wait;
      EXPECT_TRUE(manager.handleActiveLowProbe(reached,next,next_yaw,wait));
      EXPECT_EQ(manager.low_probe_phase_,vertical_detour::LowProbePhase::DESCENDING);
      EXPECT_NEAR(next.z(),expected,1e-12);
      return;
    }
    if (mode >= 3 && mode <= 7) {
      manager.recovery_side_latched_ = true;
      manager.recovery_side_obstacle_.side = 1;
      manager.recovery_side_dir_ = Vector3d::UnitY();
    }
    ASSERT_TRUE(manager.planExplorationFappCrossing(reached,zero,zero,zero));
    EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
    EXPECT_GT(manager.ed_->next_goal_.x(),reached.x()+.20);
    EXPECT_NEAR(manager.ed_->next_goal_.z(),reached.z(),1e-12);
    EXPECT_TRUE(planner.isFappPathSafe(manager.ed_->path_next_goal_));
    if (mode >= 3 && mode <= 7) {
      manager.recordPublishedTrajectoryHeight();
      reached.x() += .10;
      ASSERT_TRUE(manager.planExplorationFappCrossing(reached,zero,zero,zero));
      EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
      EXPECT_GT(manager.ed_->next_goal_.x(),reached.x()+.20);
      EXPECT_NEAR(manager.ed_->next_goal_.z(),reached.z(),1e-12);
    }
  }

  static void runLocalFappStep(int mode, bool manual = false) {
    ros::NodeHandle nh("~local_fapp_step_" + std::to_string(mode));
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 0.1);
    nh.setParam("sdf_map/virtual_ceil_height", -10.0);
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("fapp/enabled", true);
    nh.setParam("fapp/memory_enabled", true);
    nh.setParam("fapp/planner_priority", true);
    nh.setParam("fapp/vehicle_top_height", 0.05);
    nh.setParam("fapp/vehicle_bottom_height", 0.06);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    manager.task_search_manager_->setMap(manager.sdf_map_);
    manager.task_search_manager_->min_search_height_ = 0.10;
    manager.task_search_manager_->max_search_height_ = 1.90;
    manager.underpass_min_height_ = 0.10;
    manager.fapp_exploration_crossing_enabled_ = true;
    manager.manual_fapp_active_ = true;
    manager.manual_fapp_forward_ = Vector3d::UnitX();
    manager.manual_fapp_hold_height_ = 0.60;
    manager.manual_fapp_exit_progress_ = 0.95;
    if (manual) {
      manager.manual_path_enabled_ = manager.manual_path_exploration_style_ = true;
      manager.manual_path_started_ = true;
      manager.manual_path_use_astar_ = true;
      manager.manual_path_.points = {Vector3d(mode == 7 ? .725 : 1.425,.025,.60)};
      manager.task_search_manager_->enabled_ = true;
      manager.task_search_manager_->exit_detection_enabled_ = false;
      manager.turn_in_place_enabled_ = false;
    }
    planner.fapp_watched_valid_ = true;
    planner.fapp_watched_state_.position = Vector3d(0.45, 0.0, 0.80);
    planner.fapp_watched_state_.size = Vector3d(0.2,0.2,0.2);
    planner.fapp_watched_state_.stamp = ros::Time::now().toSec();
    planner.fapp_watched_lower_ = Vector3d(0.0,-0.4, mode == 3 ? 0.12 : 0.40);
    planner.fapp_watched_upper_ = Vector3d(0.9,0.4,1.0);
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(), map.mp_->clamp_min_log_);
    const auto occupy = [&](const Vector3d& point, bool unknown = false) {
      Eigen::Vector3i index; map.posToIndex(point, index);
      map.md_->occupancy_buffer_[map.toAddress(index)] = unknown
          ? map.mp_->clamp_min_log_ - map.mp_->unknown_flag_ : map.mp_->clamp_max_log_;
    };
    Vector3d start(mode == 4 ? -1.175 : -0.425, 0.025, 0.60);
    if (mode == 5 || mode == 7) start = Vector3d(0.425,0.025,0.34);
    const double wall_x = mode == 5 ? 0.725 : 0.825;
    if (mode != 7)
      for (double y=-1.725; y<1.75; y+=0.05)
        for (double z=0.125; z<1.5; z+=0.05) occupy(Vector3d(wall_x,y,z));
    if (mode == 1) occupy(Vector3d(start.x(),start.y(),0.375));
    if (mode == 2)
      for (double x=-1.975; x<2.0; x+=0.05)
        for (double y=-1.975; y<2.0; y+=0.05) occupy(Vector3d(x,y,0.375),true);
    if (mode == 8 || mode == 9) {
      manager.recovery_side_latched_ = true;
      manager.recovery_side_origin_ = start;
      manager.recovery_side_dir_ = Vector3d::UnitY();
      manager.recovery_side_obstacle_.side = 1;
    }
    if (mode == 9)
      for (double x=-1.975; x<2.0; x+=0.05)
        for (double y=-1.975; y<2.0; y+=0.05) occupy(Vector3d(x,y,0.375));
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_ = map.mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
    map.clearAndInflateLocalMap(); map.updateESDF3d();
    const Vector3d zero = Vector3d::Zero();
    if (mode == 9) {
      // 首次下降被真实静态层挡住；后续地图清除后必须重新检查，而非被旧选侧跳过。
      ASSERT_FALSE(manager.planExplorationFappCrossing(start,zero,zero,zero));
      EXPECT_TRUE(manager.fapp_crossing_retry_needed_);
      EXPECT_EQ(manager.low_probe_phase_, vertical_detour::LowProbePhase::IDLE);
      EXPECT_DOUBLE_EQ(manager.manual_fapp_hold_height_, 0.60);
      for (double x=-1.975; x<2.0; x+=0.05)
        for (double y=-1.975; y<2.0; y+=0.05) {
          Eigen::Vector3i index; map.posToIndex(Vector3d(x,y,0.375),index);
          map.md_->occupancy_buffer_[map.toAddress(index)] = map.mp_->clamp_min_log_;
        }
      map.clearAndInflateLocalMap(); map.updateESDF3d();
    }
    if (manual) {
      manager.manual_path_wait_for_trigger_ = true;
      manager.manual_path_started_ = false;
      ASSERT_EQ(manager.planExploreMotion(start,zero,zero,zero), FAIL);
      EXPECT_EQ(manager.low_probe_phase_, vertical_detour::LowProbePhase::IDLE);
      manager.manual_path_started_ = true;
    }
    const bool planned = manual
        ? manager.planExploreMotion(start,zero,zero,zero) == SUCCEED
        : (mode == 0 || mode == 6 || mode == 8 || mode == 9
          ? manager.planExplorationFappCrossing(start,zero,zero,zero)
          : manager.planLocalFappUnderpass(start,zero,zero,zero));
    if (manual) {
      EXPECT_EQ(manager.manual_path_.next, 0u);
      EXPECT_EQ(manager.manual_path_.points.front(), Vector3d(mode == 7 ? .725 : 1.425,.025,.60));
      if (planned) EXPECT_LE(manager.ed_->next_goal_.x(), manager.manual_path_.points.front().x()+1e-6);
    }
    if (mode == 2 || mode == 3) {
      EXPECT_FALSE(planned);
      EXPECT_EQ(manager.low_probe_phase_, vertical_detour::LowProbePhase::IDLE);
      EXPECT_DOUBLE_EQ(manager.manual_fapp_hold_height_, 0.60);
      return;
    }
    ASSERT_TRUE(planned);
    if (mode == 8 || mode == 9) {
      EXPECT_EQ(manager.recovery_side_obstacle_.side, 1);
      EXPECT_FALSE(manager.fapp_crossing_retry_needed_);
    }
    EXPECT_DOUBLE_EQ(manager.manual_fapp_hold_height_, 0.60); // 未发布候选不能提交高度。
    EXPECT_TRUE(planner.isTrajectorySafe());
    EXPECT_TRUE(planner.isFappPathSafe(manager.ed_->path_next_goal_));
    if (mode == 1 || mode == 4) {
      EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
      EXPECT_NEAR(manager.ed_->next_goal_.z(), start.z(), 1e-6);
      EXPECT_TRUE(manager.isKnownSafeVerticalPath(manager.ed_->next_goal_,0.34,false));
      if (mode == 4) {
        EXPECT_GT(manager.ed_->next_goal_.x(),start.x()+0.2);
        const Vector3d reached = manager.ed_->next_goal_ - Vector3d(0.02,0,0);
        manager.recordPublishedTrajectoryHeight();
        ASSERT_TRUE(manager.planLocalFappUnderpass(reached,zero,zero,zero));
        EXPECT_TRUE(manager.currentPlanIsVerticalDetour());
        EXPECT_NEAR((manager.ed_->next_goal_-reached).head<2>().norm(),0.0,1e-6);
      } else EXPECT_GT((manager.ed_->next_goal_-start).head<2>().norm(),0.03);
      return;
    }
    if (mode == 5 || mode == 7) {
      EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
      const double advance = manager.ed_->next_goal_.x()-start.x();
      if (mode == 5) { EXPECT_GT(advance,0.03); EXPECT_LT(advance,0.20); }
      else if (manual) EXPECT_NEAR(manager.ed_->next_goal_.x(), .725, 1e-6);
      else EXPECT_GT(manager.ed_->next_goal_.x(),manager.manual_fapp_exit_progress_+0.20);
      manager.recordPublishedTrajectoryHeight();
      EXPECT_FALSE(manager.manualFappObservationMissing(start,0.0,true));
      EXPECT_TRUE(manager.manual_fapp_active_); // 计划终点过了远端不等于机体已经通过。
      return;
    }
    ASSERT_TRUE(manager.currentPlanIsVerticalDetour());
    EXPECT_NEAR(manager.ed_->next_goal_.z(),0.34,1e-6);
    for (double t=0; t<planner.local_data_.duration_; t+=0.02) {
      const Vector3d p = planner.local_data_.position_traj_.evaluateDeBoorT(t);
      EXPECT_NEAR((p-start).head<2>().norm(),0.0,1e-6);
    }
    if (mode == 6) {
      occupy(Vector3d(start.x(),start.y(),0.375));
      map.clearAndInflateLocalMap(); map.updateESDF3d();
      EXPECT_FALSE(manager.reanchorVerticalDetourForRelease(start,zero));
      return;
    }
    const Vector3d live = start + Vector3d(-0.025,0.025,0.0);
    ASSERT_TRUE(manager.reanchorVerticalDetourForRelease(live,zero));
    EXPECT_NEAR((manager.ed_->next_goal_-live).head<2>().norm(),0.0,1e-6);
    manager.recordPublishedTrajectoryHeight();
    EXPECT_NEAR(manager.manual_fapp_hold_height_,0.34,1e-6);
    manager.completeVerticalDetourReference();
    ASSERT_TRUE(manager.planExplorationFappCrossing(live,zero,zero,zero));
    EXPECT_TRUE(manager.currentPlanIsVerticalDetour()); // 参考结束但实际未下降，不准向前。
    Vector3d low = live; low.z() = 0.34;
    ASSERT_TRUE(manager.planExplorationFappCrossing(low,zero,zero,zero));
    EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
    EXPECT_GT(manager.ed_->next_goal_.x(), low.x()+0.2);
    EXPECT_LT(manager.ed_->next_goal_.x(), wall_x-0.1);
    EXPECT_NEAR(manager.ed_->next_goal_.z(), low.z(), 1e-6);
  }

  static void runRecordedTurn(const char* fixture, int corridor_case = 0) {
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
    if (corridor_case==3) {
      task.exit_outside_length_=.60;
      task.exit_footprint_extra_radius_=.20;
      TaskSearchManager::OpenExitResult result;
      const double old_yaw=150.1*M_PI/180.0;
      const bool old_found=task.detectLocalMapExit(pos,Eigen::Vector2d(std::cos(old_yaw),std::sin(old_yaw)),result);
      const bool found=task.detectLocalMapExit(pos,-Eigen::Vector2d::UnitX(),result);
      std::cout << "Recorded automatic exit old=" << old_found << " aligned=" << found
                << " position=" << pos.transpose() << " portal=" << result.portal_center.transpose() << std::endl;
      ASSERT_TRUE(found);
      EXPECT_NEAR(result.portal_center.x()+3,1.8,.25);
      EXPECT_NEAR(result.portal_center.y()-2,-3,.15);
      EXPECT_TRUE(result.outward_direction.isApprox(-Eigen::Vector2d::UnitX()));
      return;
    }
    if (corridor_case==2) {
      const double entry=.1*M_PI/180.0, heading=-134.9*M_PI/180.0;
      const Vector3d initial(std::cos(entry),std::sin(entry),0);
      task.setCorridorFrame(Vector3d(-3.2,2,.6),initial);
      task.corridor_width_=1.3;
      task.corridor_entry_crossed_=true;
      task.return_straight_enabled_=true;
      task.curve_heading_enabled_=false;
      task.turn_detection_direction_=Eigen::Vector2d(std::cos(heading),std::sin(heading));
      task.stable_progress_direction_=task.turn_detection_direction_;
      task.updateRobotPose(pos,heading);
      task.visited_positions_={pos};
      Vector3d direction;
      ASSERT_TRUE(task.mappedCorridorDirection(heading,direction));
      EXPECT_TRUE(task.return_straight_active_);
      EXPECT_TRUE(direction.isApprox(-initial));
      std::cout << "Recorded return heading=" << std::atan2(direction.y(),direction.x())*180/M_PI
                << " at " << pos.transpose() << std::endl;
      return;
    }
    if (corridor_case) {
      // 输入为平移到局部坐标的实测雷达帧；由日志给出旧轴和最后双墙切线。
      task.setCorridorFrame(Vector3d(-4.2,1,.6),Vector3d::UnitX());
      task.corridor_width_=1.3;
      task.corridor_entry_crossed_=true;
      task.curve_heading_enabled_=false;
      const double axis=-80.3*M_PI/180.0, tangent=-59.8*M_PI/180.0;
      const Vector3d travel(std::cos(axis),std::sin(axis),0);
      task.width_origin_=Eigen::Vector2d(-.64,.63);
      task.width_direction_=Eigen::Vector2d(std::cos(tangent),std::sin(tangent));
      task.turn_detection_direction_=travel.head<2>();
      task.segment_direction_=travel.head<2>();
      task.segment_origin_=Eigen::Vector2d(-.51,1.26);
      task.updateRobotPose(pos,axis);
      task.visited_positions_={pos};
      task.recovery_turn_min_angle_deg_=45.0;
      task.recovery_turn_probe_length_=1.5;
      Vector3d direction=Vector3d::Zero(); double old_free,branch_free;
      task.corridor_width_=0.0;
      const bool old_found=task.inferOccupancyTurnDirection(travel,direction,old_free,branch_free);
      std::cout << "October30 without corridor-aware bypass: found=" << old_found << std::endl;
      task.corridor_width_=1.3;
      const bool found=task.inferOccupancyTurnDirection(travel,direction,old_free,branch_free);
      std::cout << "October30 reconstructed map: found=" << found
                << " old_free=" << old_free << " branch_free=" << branch_free
                << " direction=" << direction.transpose() << std::endl;
      ASSERT_TRUE(found);
      EXPECT_LT(direction.x(),-.5);
      EXPECT_LT(travel.x()*direction.y()-travel.y()*direction.x(),-.5);
      return;
    }
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

  static void runLocalObstacleKeepsCorridor(double heading) {
    ros::NodeHandle nh("~local_obstacle_keeps_corridor");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    const Vector3d forward(std::cos(heading), std::sin(heading), 0.0);
    const Vector3d lateral(-forward.y(), forward.x(), 0.0);
    const Vector3d start = -0.60 * forward + Vector3d(0, 0, 1.0);
    task.setMap(manager.sdf_map_);
    task.setCorridorFrame(start - 3.0 * forward, forward);
    task.updateRobotPose(start, heading);
    task.visited_positions_ = {start};
    task.recovery_turn_min_angle_deg_ = 45.0;
    task.recovery_turn_probe_length_ = 1.50;
    task.curve_heading_enabled_ = false;
    for (int x = 0; x < map.mp_->map_voxel_num_.x(); ++x)
      for (int y = 0; y < map.mp_->map_voxel_num_.y(); ++y)
        for (int z = 0; z < map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i id(x, y, z);
          Vector3d p; map.indexToPos(id, p);
          const double along = p.dot(forward), across = p.dot(lateral);
          // 两侧通道墙未转向，中间障碍的绕行空间超出旧1.5m审查窗口。
          const bool occupied = std::fabs(across) >= 1.65 ||
              (along >= 0.20 && along <= 0.55 && std::fabs(across) <= 0.55);
          const int address = map.toAddress(id);
          map.md_->occupancy_buffer_[address] = occupied
              ? map.mp_->clamp_max_log_ : map.mp_->clamp_min_log_;
          map.md_->occupancy_buffer_inflate_[address] = occupied;
        }
    Vector3d direction;
    double old_free, branch_free;
    EXPECT_FALSE(task.inferOccupancyTurnDirection(forward, direction, old_free, branch_free));
    EXPECT_FALSE(task.mappedCorridorDirection(heading, direction, false));
    EXPECT_FALSE(task.turnYawAlignmentPending());
    EXPECT_GT(task.turnDetectionDirection().dot(forward.head<2>()), 0.999);
  }

  static void runPassableTurn(int mode) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.recovery_turn_min_angle_deg_ = mode >= 10 ? 90.0 : 45.0;
    task.recovery_turn_max_angle_deg_ = mode >= 10 ? 90.0 : 120.0;
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
    task.updateRobotPose(Vector3d(-.1,0,1),0.0);
    task.visited_positions_.clear();
    task.visited_positions_.push_back(Vector3d(-1.1,0,1));
    task.visited_positions_.push_back(Vector3d(-.1,0,1));
    double right_width = (mode == 2 ? 0.65 : 0.90);
    double left_width = (mode == 2 || mode == 3 || mode == 4 || mode == 11 ? 0.90 : 0.20);
    if (mode == 5) right_width = left_width = 0.20;
    if (mirror) std::swap(right_width, left_width);
    for (int x=0; x<map.mp_->map_voxel_num_.x(); ++x)
      for (int y=0; y<map.mp_->map_voxel_num_.y(); ++y)
        for (int z=0; z<map.mp_->map_voxel_num_.z(); ++z) {
          const Eigen::Vector3i id(x,y,z);
          Vector3d p; map.indexToPos(id,p);
          const bool hall = p.x() < 0.90 && std::fabs(p.y()) < (mode == 11 ? .35 : .55);
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
    if (mode >= 5 && mode <= 8) {
      EXPECT_FALSE(found) << "slit, disconnected, unknown or low-roof branch was accepted";
      return;
    }
    ASSERT_TRUE(found);
    if (mode >= 10) {
      ASSERT_TRUE(manager.planner_manager_->isTurnPositionSafe(task.latest_robot_pos_));
      if (mode == 10) {
        ASSERT_FALSE(task.mapExitFootprintFree(task.latest_robot_pos_ + selected));
        EXPECT_TRUE(task.inferOccupancyTurnDirection(
            Vector3d::UnitX(), selected, old_free, branch_free, true));
        EXPECT_TRUE(task.mappedCorridorDirection(0.0, selected, false));
        EXPECT_TRUE(task.turnYawAlignmentPending());
        return;
      }
      ASSERT_LT(task.knownHorizontalClearance(task.latest_robot_pos_)+map.getResolution(),
                task.knownHorizontalClearance(Vector3d(.45,0,1)));
      EXPECT_FALSE(task.inferOccupancyTurnDirection(
          Vector3d::UnitX(), selected, old_free, branch_free, true));
      EXPECT_FALSE(task.mappedCorridorDirection(0.0, selected, false));
      EXPECT_FALSE(task.turnYawAlignmentPending());
      task.updateRobotPose(Vector3d(.45,0,1),0.0);
      EXPECT_TRUE(task.inferOccupancyTurnDirection(
          Vector3d::UnitX(), selected, old_free, branch_free, true));
      return;
    }
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
    const bool manual_retry = scenario == "manual_entry_retry";
    const bool entry_retry = scenario == "exploration_entry_retry" || manual_retry;
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
    if (manual_retry) manager.fapp_exploration_crossing_enabled_ = true;
    if (exploration && !manual_retry) {
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
      EXPECT_GE(manager.ed_->next_goal_.x(), upper.x()+map.getResolution()-1e-6);
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
        EXPECT_GE(manager.ed_->next_goal_.x(), upper.x()+map.getResolution()-1e-6);
        EXPECT_GT(Astar::pathLength(manager.ed_->path_next_goal_), 1.2);
        EXPECT_NEAR(manager.low_probe_height_, manager.ed_->next_goal_.z(), 1e-6);
        if (low_minimum) EXPECT_NEAR(0.12, manager.low_probe_height_, 1e-6);
      } else {
        EXPECT_GT(std::fabs(manager.ed_->next_goal_.y() - start.y()), 0.2);
        EXPECT_NEAR(start.z(), manager.ed_->next_goal_.z(), 1e-6);
      }
      // 此处为完整可通场景；终点越过已膨胀包络即可，不强制额外20cm。
      EXPECT_GE(manager.ed_->next_goal_.x(), upper.x()+map.getResolution()-1e-6);
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
      EXPECT_GT(endpoint.x(), upper.x()+map.getResolution()-0.01);
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
      EXPECT_GE(manager.ed_->next_goal_.x(), upper.x()+map.getResolution()-1e-6);
      EXPECT_NEAR(endpoint.z(), manager.ed_->next_goal_.z(), 0.01);
      if (asymmetric_side) EXPECT_GT((manager.ed_->next_goal_.y() - start.y()) *
          (prior_right_side ? -1.0 : 1.0), 0.0);
      {
        const Vector3d end = manager.ed_->next_goal_;
        EXPECT_FALSE(manager.manualFappObservationMissing(end, 0.0));
        EXPECT_TRUE(manager.manual_fapp_active_); // 预测端点不能释放历史。
        EXPECT_FALSE(manager.manualFappObservationMissing(end, 0.0, true));
        if (manager.manual_fapp_active_) {
          // 端点数值误差尚未真正过线时，继续规划低位段，不伪报已穿过。
          ASSERT_TRUE(manager.planExplorationFappCrossing(end, zero, zero, zero));
          EXPECT_GT(manager.ed_->next_goal_.x(),end.x()+0.03);
          EXPECT_FALSE(manager.manualFappObservationMissing(manager.ed_->next_goal_,0.0,true));
        }
        EXPECT_FALSE(manager.manual_fapp_active_);
      }
      return;
    }
    if (inside) {
      auto& planner = *manager.planner_manager_;
      ASSERT_FALSE(planner.isWatchedFappPositionSafe(start));
      if (scenario == "inside_history_strict") {
        EXPECT_EQ(FAIL, result);
      } else {
        ASSERT_EQ(SUCCEED, result);
        auto& trajectory = planner.local_data_.position_traj_;
        const Vector3d end = trajectory.evaluateDeBoorT(trajectory.getTimeSum());
        EXPECT_GT(end.x(), start.x() + 0.3);
        EXPECT_GT(end.z(), start.z() - 0.05);
        EXPECT_FALSE(manager.currentPlanIsVerticalDetour());
        checkManualFappExecution(manager, start);
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
    // 远离球的短段可换侧；真正穿过球的路径仍须拒绝。
    EXPECT_TRUE(manager.keepsDynamicSide({start, start - .3*manager.recovery_side_dir_}));
    EXPECT_FALSE(manager.keepsDynamicSide({start, Vector3d(.2, 0, 1)}));
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

  static void runRaisedStaticSide(int mode, bool manual = false) {
    ros::NodeHandle nh("~raised_static_side");
    nh.setParam("sdf_map/virtual_ceil_height", -10.0);
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 10000.0);
    nh.setParam("astar/max_search_time", 0.1);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& map = *manager.sdf_map_;
    auto& planner = *manager.planner_manager_;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.min_search_height_ = 0.10;
    task.max_search_height_ = mode == 5 ? 1.25 : 1.90;
    const Vector3d start(0.025, 0.025, 0.525);
    const double top = mode == 3 ? 1.15 : 1.10;
    const double wall_side = mode == 1 ? 1.0 : -1.0;
    // 确定性已知自由地图，障碍仍通过生产膨胀和ESDF更新处理。
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    const auto occupy = [&](double x, double y, double z) {
      Eigen::Vector3i index;
      map.posToIndex(Vector3d(x, y, z), index);
      map.md_->occupancy_buffer_[map.toAddress(index)] = map.mp_->clamp_max_log_;
    };
    for (double x : {0.425, 0.475, 0.525})
      for (double y : {-0.075, -0.025, 0.025, 0.075, 0.125})
        for (double z = 0.125; z < top; z += 0.05) occupy(x, y, z);
    // 右/左侧连续墙让同距离的左右候选具有不同净空。
    for (double x = -1.025; x <= 1.475; x += 0.05)
      for (double z = 0.125; z <= 1.825; z += 0.05)
        occupy(x, 0.025 + wall_side * 0.325, z);
    if (mode == 2) {
      // 起点上方的小横梁挡住最低+20cm层，侧方可以绕开横梁升到更高层。
      for (double x : {-0.025, 0.025, 0.075})
        for (double y : {-0.025, 0.025, 0.075}) occupy(x, y, 1.325);
    }
    if (mode == 4) {
      for (int x = 0; x < map.mp_->map_voxel_num_.x(); ++x)
        for (int y = 0; y < map.mp_->map_voxel_num_.y(); ++y)
          for (int z = 24; z < map.mp_->map_voxel_num_.z(); ++z)
            map.md_->occupancy_buffer_[map.toAddress(x,y,z)] =
                map.mp_->clamp_min_log_ - map.mp_->unknown_flag_;
    }
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_ = map.mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
    map.clearAndInflateLocalMap();
    map.updateESDF3d();
    Vector3d next;
    double yaw = 0.0;
    const bool selected = manager.buildRaisedSideBypass(
        start, 0.0, Vector3d::UnitX(), top, next, yaw, mode == 6);
    if (mode == 6) {
      ASSERT_TRUE(selected);
      EXPECT_NEAR(next.y(), start.y(), 1e-9);
      EXPECT_GT(next.x(), start.x());
      EXPECT_GE(next.z(), top + .20 - 1e-6);
      EXPECT_TRUE(manager.isKnownSafeVerticalPath(start, next.z()));
      manager.edt_environment_ = planner.edt_environment_;
      Vector3d origin, size; map.getRegion(origin, size);
      ViewNode::caster_ = std::make_shared<RayCaster>();
      ViewNode::caster_->setParams(map.getResolution(), origin);
      nh.setParam("frontier/cluster_min", 1000000);
      manager.frontier_finder_ = std::make_shared<FrontierFinder>(manager.edt_environment_, nh);
      manager.entry_forward_complete_ = manager.mission_entered_search_region_ = true;
      manager.ep_->mission_use_forward_fallback_ = true;
      manager.ep_->mission_max_route_segment_length_ = 1.2;
      manager.ground_ascent_enabled_ = false;
      manager.inflation_history_escape_enabled_ = false;
      manager.turn_in_place_enabled_ = false;
      manager.center_gate_enabled_ = false;
      manager.camera_head_sweep_enabled_ = false;
      manager.ground_ascent_still_confirm_time_ = 0.0;
      task.enabled_ = true;
      task.exit_detection_enabled_ = false;
      task.global_no_return_ = false;
      task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
      task.stable_progress_direction_valid_ = true;
      task.updateRobotPose(start,0.0);
      if (manual) {
        manager.manual_path_enabled_ = manager.manual_path_exploration_style_ = true;
        manager.manual_path_started_ = manager.manual_path_use_astar_ = true;
        manager.manual_path_.points = {Vector3d(1.425,.025,.60)};
      }
      ASSERT_EQ(manager.planExploreMotion(start,Vector3d::Zero(),Vector3d::Zero(),Vector3d(.1,0,0)), SUCCEED);
      EXPECT_GE(manager.ed_->next_goal_.z(), top+.20-1e-6);
      EXPECT_TRUE(manager.currentPlanIsGroundAscent());
      EXPECT_GE(manager.obstacle_clearance_height_, top+.20-1e-6);
      EXPECT_NEAR((manager.ed_->next_goal_-start).head<2>().norm(), 0.0, 1e-9);
      EXPECT_TRUE(planner.isTrajectorySafe());
      if (manual) {
        const Vector3d elevated = manager.ed_->next_goal_;
        manager.recordPublishedTrajectoryHeight();
        manager.updateMissionOdometry(elevated, 0.0, ros::Time::now());
        ASSERT_EQ(manager.planExploreMotion(elevated,Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero()), SUCCEED);
        EXPECT_GT(manager.ed_->next_goal_.x(), elevated.x());
        EXPECT_GE(manager.ed_->next_goal_.z(), top+.20-1e-6);
        EXPECT_EQ(manager.manual_path_.points.front(), Vector3d(1.425,.025,.60));
      }
      return;
    }
    if (mode >= 3) {
      EXPECT_FALSE(selected);
      EXPECT_EQ(manager.low_probe_phase_, vertical_detour::LowProbePhase::IDLE);
      return;
    }
    ASSERT_TRUE(selected);
    EXPECT_GE(next.z(), top + 0.20 - 1e-6);
    EXPECT_LE(next.z(), task.max_search_height_);
    EXPECT_GT(next.x(), start.x());
    EXPECT_GT((next.y() - start.y()) * -wall_side, 0.05);
    EXPECT_LE((next - start).head<2>().norm(), 0.60 + 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
    if (mode == 2) EXPECT_GT(next.z(), top + 0.20 + 0.10);
    else EXPECT_NEAR(next.z(), top + 0.20, 1e-6);
    planner.path_finder_->reset();
    ASSERT_EQ(planner.path_finder_->search(start, next), Astar::REACH_END);
    EXPECT_TRUE(planner.isPathSafe(planner.path_finder_->getPath()));
    if (mode < 2) {
      ASSERT_TRUE(planner.planExploreTraj(planner.path_finder_->getPath(),
                                          Vector3d::Zero(), Vector3d::Zero()));
      EXPECT_TRUE(planner.isTrajectorySafe());
      // 达到抬高侧向点后，原直行短段可以从顶面以上20cm接着通过障碍。
      ASSERT_TRUE(planner.planStraightTraj(next, next + Vector3d(0.8, 0.0, 0.0)));
      EXPECT_TRUE(planner.isTrajectorySafe());
      double crossed_top;
      EXPECT_FALSE(manager.trajectoryNeedsObstacleClearance(crossed_top));
      EXPECT_NEAR(crossed_top, top, 1e-6);
    }
    if (mode <= 2) {
      double detected_top;
      ASSERT_TRUE(manager.findForwardObstacleTop(start, Vector3d::UnitX(), detected_top));
      EXPECT_NEAR(detected_top, top, 1e-6);
      // 从0.525m到1.30m超过旧0.60m升幅，正式垂直恢复入口仍能选到侧向候选。
      ASSERT_TRUE(manager.buildVerticalDetourFallback(
          start, 0.0, Vector3d::UnitX(), next, yaw, false, false));
      EXPECT_GE(next.z(), top + 0.20 - 1e-6);
      if (mode == 2) EXPECT_GT(next.z(), top + 0.20 + 0.10);
      EXPECT_GT((next.y() - start.y()) * -wall_side, 0.05);
    }
  }

  static void runObstacleClearance(bool roof, bool high_reference = false) {
    ros::NodeHandle nh("~");
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.max_search_height_ = 1.90;
    manager.ground_ascent_still_confirm_time_ = 0.0;
    manager.vertical_detour_low_height_ = 0.10;
    const Vector3d start(-0.525,0.025,0.625), end(1.025,0.025,0.625);
    // 用真实射线构建已观测的上升柱和路线，再添加顶部为0.55m的障碍。
    for (int layer=2;layer<=24;++layer) {
      const double z=(layer+0.5)*0.05;
      for (double x : {-1.525,1.525}) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (double y=-1.5;y<=1.5;y+=0.025)
          cloud.push_back(pcl::PointXYZ(x,y,z));
        for (int i=0;i<5;++i) map.inputPointCloud(cloud,cloud.size(),Vector3d(-0.85*x,0.0,z));
      }
    }
    pcl::PointCloud<pcl::PointXYZ> obstacle;
    for (double x : {0.425,0.475,0.525,0.575})
      for (double y : {-0.075,-0.025,0.025,0.075})
        for (double z : {0.425,0.475,0.525})
          obstacle.push_back(pcl::PointXYZ(x,y,z));
    for (int i=0;i<8;++i) map.inputPointCloud(obstacle,obstacle.size(),start);
    if (roof) {
      pcl::PointCloud<pcl::PointXYZ> ceiling;
      for (double x : {-0.575,-0.525,-0.475})
        for (double y : {-0.025,0.025,0.075})
          ceiling.push_back(pcl::PointXYZ(x,y,0.775));
      for (int i=0;i<8;++i) map.inputPointCloud(ceiling,ceiling.size(),start);
    }
    map.updateESDF3d();
    ASSERT_TRUE(planner.planStraightTraj(start,end));
    ASSERT_TRUE(planner.isTrajectorySafe());
    manager.ed_->next_goal_=end;
    double top;
    ASSERT_TRUE(manager.trajectoryNeedsObstacleClearance(top));
    EXPECT_NEAR(top,0.55,1e-6);
    if (!roof) {
      const Vector3d high_start(start.x(),start.y(),0.75), high_end(end.x(),end.y(),0.75);
      ASSERT_TRUE(planner.planStraightTraj(high_start,high_end));
      EXPECT_FALSE(manager.trajectoryNeedsObstacleClearance(top));
      ASSERT_TRUE(planner.planStraightTraj(start+Vector3d(0,.7,0),end+Vector3d(0,.7,0)));
      EXPECT_FALSE(manager.trajectoryNeedsObstacleClearance(top));
      ASSERT_TRUE(planner.planStraightTraj(start,end));
    }
    manager.manual_fapp_active_=true;
    manager.manual_fapp_curve_active_=false;
    manager.manual_fapp_hold_height_=0.625;
    Vector3d predicted_start=start;
    if (high_reference) {
      manager.updateMissionOdometry(start,0,ros::Time::now());
      predicted_start.z()=0.75;
      ASSERT_TRUE(planner.planStraightTraj(Vector3d(start.x(),start.y(),0.75),
                                          Vector3d(end.x(),end.y(),0.75)));
      ASSERT_FALSE(manager.trajectoryNeedsObstacleClearance(top));
    }
    bool needed=false;
    const bool planned=manager.ensureObstacleTopClearance(predicted_start,Vector3d::Zero(),Vector3d::Zero(),needed);
    ASSERT_TRUE(needed);
    if (roof) {
      EXPECT_FALSE(planned);
      EXPECT_FALSE(manager.obstacleAscentPending());
      return;
    }
    ASSERT_TRUE(planned);
    ASSERT_TRUE(manager.currentPlanIsGroundAscent());
    EXPECT_NEAR(manager.obstacle_clearance_height_,0.75,1e-6);
    auto& trajectory=planner.local_data_.position_traj_;
    for (double t=0;t<trajectory.getTimeSum();t+=.02)
      EXPECT_LT((trajectory.evaluateDeBoorT(t)-start).head<2>().norm(),1e-6);
    const Vector3d target=trajectory.evaluateDeBoorT(trajectory.getTimeSum());
    EXPECT_GE(target.z(),0.75);
    manager.recordPublishedTrajectoryHeight();
    EXPECT_GE(manager.manual_fapp_hold_height_,0.75);
    Vector3d actual=start;
    actual.z()=.74;
    manager.updateMissionOdometry(actual,0,ros::Time::now());
    manager.buildGroundAscentPlan(actual,Vector3d::Zero(),Vector3d::Zero());
    EXPECT_TRUE(manager.obstacleAscentPending());
    actual.z()=.751;
    manager.updateMissionOdometry(actual,0,ros::Time::now());
    EXPECT_FALSE(manager.buildGroundAscentPlan(actual,Vector3d::Zero(),Vector3d::Zero()));
    EXPECT_FALSE(manager.obstacleAscentPending());
  }

  static void runTurnRadiusIndependentOfTranslation(bool inside_turn_radius) {
    ros::NodeHandle nh("~turn_radius_independent");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    const Vector3d start(0.025, 0.025, 1.025);
    // 障碍体素近边距机心22.5cm或27.5cm；均在原平移18cm检测之外。
    const double lateral_distance = inside_turn_radius ? 0.25 : 0.30;
    pcl::PointCloud<pcl::PointXYZ> obstacle;
    for (double z : {0.975, 1.025, 1.075})
      obstacle.push_back(pcl::PointXYZ(start.x(), start.y() + lateral_distance, z));
    for (int i = 0; i < 8; ++i)
      manager.sdf_map_->inputPointCloud(obstacle, obstacle.size(), start);
    manager.sdf_map_->updateESDF3d();
    EXPECT_DOUBLE_EQ(planner.footprintRadius(), 0.18);
    ASSERT_TRUE(planner.isPositionSafe(start));
    EXPECT_EQ(planner.isTurnPositionSafe(start), !inside_turn_radius);
    ASSERT_TRUE(planner.planStraightTraj(start, start - 0.4 * Vector3d::UnitX()));
    EXPECT_TRUE(planner.isTrajectorySafe());
  }

  static void runTurnMinimumHeight() {
    ros::NodeHandle nh("~turn_minimum_height");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.setCorridorFrame(Vector3d(-2.0, 0.025, 0.20), Vector3d::UnitX());
    const Vector3d zero = Vector3d::Zero();
    for (double z : {0.10, 0.15, 0.199999, 0.20, 0.25}) {
      SCOPED_TRACE(z);
      const Vector3d position(0.025, 0.025, z);
      task.updateRobotPose(position, 0.0);
      task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
      manager.turn_in_place_session_active_ = false;
      manager.turn_in_place_plan_ = false;
      ASSERT_TRUE(planner.isPositionSafe(position));
      ASSERT_TRUE(planner.isTurnPositionSafe(position));
      EXPECT_EQ(manager.turnInPlaceHeightAllowed(z), z >= 0.20);
      // 低位不得抢占当前平移，也不得从规划入口发布转弯；20cm边界允许。
      EXPECT_EQ(manager.requestMappedTurn(position, 0.0), z >= 0.20);
      EXPECT_EQ(manager.buildTurnInPlacePlan(position, Vector3d(0.1, 0.0, -0.1),
                                            zero, zero, Vector3d::UnitY()), z >= 0.20);
      EXPECT_EQ(manager.currentPlanIsTurnInPlace(), z >= 0.20);
      EXPECT_EQ(manager.turnInPlaceAlignmentPending(), z >= 0.20);
      EXPECT_FALSE(manager.currentPlanIsEscapeStep());
      ASSERT_TRUE(planner.planStraightTraj(position, position + 0.4 * Vector3d::UnitX()));
      EXPECT_TRUE(planner.isTrajectorySafe());
    }
  }

  static void runStaticTurnCollision(int stage, bool low_height = false) {
    ros::NodeHandle nh("~static_turn_safety");
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& task = *manager.task_search_manager_;
    const Vector3d start(0.025, 0.025, low_height ? 0.21 : 1.025), zero = Vector3d::Zero();
    task.setMap(manager.sdf_map_);
    task.enabled_ = true;
    task.setCorridorFrame(start - 2.0 * Vector3d::UnitX(), Vector3d::UnitX());
    task.updateRobotPose(start, 0.0);
    task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
    if (stage == 0) {
      // 实测位置比正在执行的参考靠墙10cm；旧轨迹仍朝远离墙面的安全终点移动。
      ASSERT_TRUE(planner.planStraightTraj(start - 0.1 * Vector3d::UnitX(),
                                          start - 0.6 * Vector3d::UnitX()));
      ASSERT_TRUE(planner.planYawTurnInPlace(zero, 0.0));
    } else {
      ASSERT_TRUE(manager.buildTurnInPlacePlan(start, zero, zero, zero, Vector3d::UnitY()));
    }

    FastExplorationFSM fsm;
    fsm.expl_manager_ = std::shared_ptr<FastExplorationManager>(&manager, [](FastExplorationManager*){});
    fsm.planner_manager_ = manager.planner_manager_;
    fsm.fd_ = std::make_shared<FSMData>();
    fsm.fp_ = std::make_shared<FSMParam>();
    fsm.fd_->state_str_ = {"INIT", "WAIT", "PLAN", "PUB", "EXEC", "FINISH"};
    fsm.fd_->odom_pos_ = start;
    fsm.fd_->odom_vel_.setZero();
    fsm.fd_->odom_yaw_ = 0.0;
    fsm.fd_->start_acc_.setZero();
    fsm.fp_->emergency_brake_horizon_ = 0.12;
    fsm.fp_->max_tracking_error_xy_ = fsm.fp_->max_tracking_error_z_ = 1.0;
    fsm.fp_->trajectory_release_max_start_error_ = 0.1;
    fsm.fp_->trajectory_release_max_velocity_error_ = 0.1;
    fsm.fp_->trajectory_release_max_acceleration_error_ = 1.0;
    fsm.active_traj_ = planner.local_data_;
    fsm.active_traj_.start_time_ = ros::Time::now();
    fsm.active_traj_valid_ = true;
    fsm.active_turn_in_place_ = stage == 2;
    fsm.state_ = EXEC_TRAJ;
    fsm.last_suspended_underpass_check_ = ros::Time::now() + ros::Duration(30.0);
    fsm.emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("static_turn_brake", 1);

    if (stage < 2 && !low_height) {
      // 有重复邻域支撑的细障碍落在旧12点圆环间隙内，中心与旧采样仍安全。
      pcl::PointCloud<pcl::PointXYZ> obstacle;
      for (double z : {0.975, 1.025, 1.075})
        obstacle.push_back(pcl::PointXYZ(0.175, 0.025, z));
      for (int i = 0; i < 8; ++i)
        manager.sdf_map_->inputPointCloud(obstacle, obstacle.size(), start);
      manager.sdf_map_->updateESDF3d();
      ASSERT_TRUE(planner.isPositionSafe(start));
      ASSERT_FALSE(planner.isTurnPositionSafe(start));
      ASSERT_TRUE(planner.isTrajectorySafe());
    }
    if (stage == 0) {
      fsm.safetyCallback(ros::TimerEvent());
      EXPECT_EQ(fsm.state_, EXEC_TRAJ);
      EXPECT_FALSE(fsm.active_traj_braked_);
      EXPECT_FALSE(manager.turn_in_place_session_active_);
      EXPECT_FALSE(manager.buildTurnInPlacePlan(start, zero, zero, zero, Vector3d::UnitY()));
      EXPECT_FALSE(manager.currentPlanIsTurnInPlace());
      // 旧安全轨迹的终点移出旋转碰撞范围后，原锁存转向仍然可执行。
      const Vector3d end = start - 0.6 * Vector3d::UnitX();
      task.updateRobotPose(end, 0.0);
      ASSERT_TRUE(planner.isTurnPositionSafe(end));
      EXPECT_TRUE(manager.buildTurnInPlacePlan(end, zero, zero, zero, Vector3d::UnitY()));
      return;
    }
    if (stage == 1) {
      // 候选已生成后才出现障碍，发布门控仍须拦截，而不是只相信规划时的地图。
      fsm.state_ = PUB_TRAJ;
      fsm.active_traj_braked_ = true;
      fsm.pending_turn_in_place_ = true;
      if (low_height) fsm.fd_->odom_pos_.z() = 0.19;
      for (int i = 0; i < 5; ++i) {
        OdomReviewSample sample;
        sample.stamp = 1.0 + 0.01 * i;
        sample.position = {{fsm.fd_->odom_pos_.x(), fsm.fd_->odom_pos_.y(),
                            fsm.fd_->odom_pos_.z()}};
        sample.velocity = {{0.0, 0.0, 0.0}};
        fsm.odom_review_window_.push_back(sample);
      }
      fsm.FSMCallback(ros::TimerEvent());
      EXPECT_EQ(fsm.state_, PLAN_TRAJ);
      EXPECT_TRUE(fsm.pending_traj_safe_since_.isZero());
      return;
    }
    // 实际机体偏离参考点碰到静态障碍，参考轨迹本身仍安全，也必须停止转向。
    if (low_height) {
      fsm.fd_->odom_pos_.z() = 0.19;
      ASSERT_TRUE(planner.isPositionSafe(fsm.fd_->odom_pos_));
    } else {
      fsm.fd_->odom_pos_ = start + Vector3d(0.30, 0.0, 0.0);
      manager.sdf_map_->setOccupied(fsm.fd_->odom_pos_);
    }
    ASSERT_TRUE(planner.isTurnPositionSafe(start));
    ASSERT_EQ(planner.isTurnPositionSafe(fsm.fd_->odom_pos_), low_height);
    if (low_height) EXPECT_FALSE(manager.turnInPlaceHeightAllowed(fsm.fd_->odom_pos_.z()));
    double collision_distance;
    ASSERT_TRUE(planner.checkTrajCollision(fsm.active_traj_, collision_distance));
    fsm.safetyCallback(ros::TimerEvent());
    EXPECT_EQ(fsm.state_, PLAN_TRAJ);
    EXPECT_TRUE(fsm.active_traj_braked_);
    EXPECT_FALSE(fsm.active_turn_in_place_);
    EXPECT_TRUE(manager.turnInPlaceAlignmentPending());
  }

  static void runTurnRetreatOrEscapeRelease(bool release_check) {
    ros::NodeHandle nh("~turn_retreat_"+std::to_string(release_check));
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& planner=*manager.planner_manager_;
    auto& task=*manager.task_search_manager_;
    auto& map=*manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    const Vector3d start(.025,.025,1.025),zero=Vector3d::Zero();
    for (double z : {.975,1.025,1.075}) {
      Eigen::Vector3i id;
      map.posToIndex(Vector3d(release_check?.225:.275,.025,z),id);
      map.md_->occupancy_buffer_[map.toAddress(id)]=map.mp_->clamp_max_log_;
    }
    task.setMap(manager.sdf_map_);
    task.setCorridorFrame(start-2.0*Vector3d::UnitX(),Vector3d::UnitX());
    task.updateRobotPose(start,0.0);
    manager.recovery_odom_history_={start-.5*Vector3d::UnitX(),start};
    if (!release_check) {
      ASSERT_TRUE(planner.isPositionSafe(start));
      ASSERT_FALSE(planner.isTurnPositionSafe(start));
      task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
      ASSERT_TRUE(manager.buildTurnInPlacePlan(start,zero,zero,zero,Vector3d::UnitY()));
      EXPECT_TRUE(manager.currentPlanIsEscapeStep());
      EXPECT_FALSE(manager.currentPlanIsTurnInPlace());
      EXPECT_LT(manager.ed_->next_goal_.x(),start.x()-.03);
      EXPECT_TRUE(planner.isTurnPositionSafe(manager.ed_->next_goal_));
      EXPECT_TRUE(task.turnYawAlignmentPending());
      const Vector3d end=manager.ed_->next_goal_;
      task.updateRobotPose(end,0.0);
      EXPECT_TRUE(manager.buildTurnInPlacePlan(end,zero,zero,zero,Vector3d::UnitY()));
      EXPECT_TRUE(manager.currentPlanIsTurnInPlace());
      return;
    }
    ASSERT_TRUE(planner.isControlledEscapePosition(start));
    ASSERT_FALSE(planner.isRawPositionSafe(start));
    Vector3d actual=start;
    for (double d=.001;d<=.10;d+=.001) {
      actual=start-d*Vector3d::UnitX();
      if (planner.isPositionSafe(actual)) break;
    }
    ASSERT_TRUE(planner.isPositionSafe(actual));
    ASSERT_FALSE(planner.isControlledEscapePosition(actual));
    ASSERT_TRUE(planner.planExploreTraj({start,start-.15*Vector3d::UnitX(),
        start-.30*Vector3d::UnitX()},zero,zero,0.0));
    ASSERT_TRUE(planner.isTrajectorySafe(.03,true));
    ASSERT_FALSE(planner.isTrajectorySafe(.03,false));
    FastExplorationFSM fsm;
    fsm.expl_manager_=std::shared_ptr<FastExplorationManager>(&manager,[](FastExplorationManager*){});
    fsm.planner_manager_=manager.planner_manager_;
    fsm.fd_=std::make_shared<FSMData>();
    fsm.fp_=std::make_shared<FSMParam>();
    fsm.fd_->state_str_={"INIT","WAIT","PLAN","PUB","EXEC","FINISH"};
    fsm.fd_->odom_pos_=actual;fsm.fd_->odom_vel_.setZero();fsm.fd_->odom_yaw_=0.0;
    fsm.fp_->trajectory_release_max_start_error_=.11;
    fsm.fp_->trajectory_release_max_velocity_error_=.1;
    fsm.fp_->trajectory_release_max_acceleration_error_=1.0;
    fsm.fp_->trajectory_release_confirm_time_=10.0;
    fsm.active_traj_valid_=true;fsm.active_traj_braked_=true;
    fsm.pending_escape_step_=true;fsm.state_=PUB_TRAJ;
    for (int i=0;i<5;++i) {
      OdomReviewSample sample;sample.stamp=1.0+.01*i;
      sample.position={{actual.x(),actual.y(),actual.z()}};
      sample.velocity={{0,0,0}};fsm.odom_review_window_.push_back(sample);
    }
    fsm.FSMCallback(ros::TimerEvent());
    EXPECT_EQ(fsm.state_,PUB_TRAJ);
    EXPECT_FALSE(fsm.pending_traj_safe_since_.isZero());
  }

  static void runClimbBeforeMappedTurn(int mode) {
    ros::NodeHandle nh("~climb_before_turn_" + std::to_string(mode));
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& task = *manager.task_search_manager_;
    auto& map = *manager.sdf_map_;
    std::fill(map.md_->occupancy_buffer_.begin(), map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    const Vector3d start(.025,.025,.125), end(.465,.025,.345), zero=Vector3d::Zero();
    task.setMap(manager.sdf_map_);
    task.setCorridorFrame(start-2.0*Vector3d::UnitX(),Vector3d::UnitX());
    task.updateRobotPose(start,0.0);
    task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
    ASSERT_TRUE(planner.planStraightTraj(start,end));
    ASSERT_TRUE(planner.planYawTurnInPlace(zero,0.0));
    FastExplorationFSM fsm;
    fsm.expl_manager_=std::shared_ptr<FastExplorationManager>(&manager,[](FastExplorationManager*){});
    fsm.planner_manager_=manager.planner_manager_;
    fsm.fd_=std::make_shared<FSMData>();
    fsm.fp_=std::make_shared<FSMParam>();
    fsm.fd_->state_str_={"INIT","WAIT","PLAN","PUB","EXEC","FINISH"};
    fsm.fd_->odom_pos_=start;
    fsm.fd_->odom_vel_=Vector3d(.1,0,.05);
    fsm.fd_->odom_yaw_=0.0;
    fsm.fp_->tracking_error_grace_time_=100.0;
    fsm.fp_->max_tracking_error_xy_=fsm.fp_->max_tracking_error_z_=1.0;
    fsm.fp_->emergency_brake_horizon_=.12;
    fsm.active_traj_=planner.local_data_;
    fsm.active_traj_.start_time_=ros::Time::now();
    fsm.active_traj_valid_=true;
    fsm.narrow_corridor_stage_=true;
    fsm.state_=mode==1 ? PUB_TRAJ : EXEC_TRAJ;
    fsm.last_suspended_underpass_check_=ros::Time::now()+ros::Duration(30.0);
    fsm.emergency_brake_pub_=nh.advertise<std_msgs::Int32>("brake",1);
    if (mode==2) map.setOccupied(end);
    fsm.safetyCallback(ros::TimerEvent());
    if (mode==2) {
      EXPECT_TRUE(fsm.active_traj_braked_);
      EXPECT_EQ(fsm.state_,PLAN_TRAJ);
      EXPECT_FALSE(manager.turn_in_place_session_active_);
      return;
    }
    EXPECT_FALSE(fsm.active_traj_braked_);
    EXPECT_FALSE(manager.turn_in_place_session_active_);
    EXPECT_EQ(fsm.state_,mode==1 ? PUB_TRAJ : EXEC_TRAJ);
    if (mode==0) {
      fsm.FSMCallback(ros::TimerEvent());
      EXPECT_EQ(fsm.state_,EXEC_TRAJ);
    }
    // 只完成本段0.345m目标就允许重评转弯，不等0.6m，也不以速度判完成。
    fsm.active_traj_.start_time_=ros::Time::now()-ros::Duration(fsm.active_traj_.duration_+.1);
    fsm.fd_->odom_pos_=end;
    task.updateRobotPose(end,0.0);
    fsm.safetyCallback(ros::TimerEvent());
    EXPECT_TRUE(fsm.active_traj_braked_);
    EXPECT_TRUE(manager.turn_in_place_session_active_);
  }

  static void runLateralFappEnvelope(bool rotated) {
    ros::NodeHandle nh("~lateral_fapp_" + std::to_string(rotated));
    nh.setParam("fapp/enabled",true);
    nh.setParam("fapp/memory_enabled",true);
    nh.setParam("fapp/lateral_motion_only",true);
    FastExplorationManager manager;
    initialize(nh,manager);
    auto& planner=*manager.planner_manager_;
    const Vector3d forward=rotated ? Vector3d(1,1,0).normalized() : Vector3d::UnitX();
    const Vector3d side(-forward.y(),forward.x(),0);
    planner.setFappCorridorDirection(forward);
    auto message=boost::make_shared<obj_state_msgs::ObjectsStates>();
    message->header.frame_id="UAV0/camera_init";
    const double now=ros::Time::now().toSec();
    for (int i=0;i<7;++i) {
      obj_state_msgs::State ball;
      const Vector3d p=(i==6?.25:0.0)*forward + (.1*(i%5)-.2)*side +
          Vector3d(0,0,i==6?1.0:.6);
      const Vector3d v=.8*forward+.4*side+Vector3d(0,0,.7);
      ball.position.x=p.x();ball.position.y=p.y();ball.position.z=p.z();
      ball.velocity.x=v.x();ball.velocity.y=v.y();ball.velocity.z=v.z();
      ball.size.x=ball.size.y=ball.size.z=.10;
      message->header.stamp.fromSec(now-.65+.1*i);
      message->states={ball};
      planner.fappCallback(message);
    }
    ASSERT_FALSE(planner.fapp_states_.empty());
    const auto ball=planner.fapp_states_.back();
    EXPECT_NEAR(ball.velocity.dot(forward),0.0,1e-9);
    EXPECT_NEAR(ball.velocity.z(),0.0,1e-9);
    EXPECT_NEAR(ball.velocity.dot(side),.4,1e-9);
    planner.watchFappObstacle(&ball);
    Vector3d lower,upper;double age;
    ASSERT_TRUE(planner.watchedFappBounds(lower,upper,age));
    EXPECT_LT(upper.z(),.95); // 单帧高点不能把整个历史包络拉高。
    EXPECT_LT(planner.fapp_watched_along_bounds_.y(),.45);
    EXPECT_FALSE(planner.isWatchedFappPositionSafe(.2*side+Vector3d(0,0,.6)));
    EXPECT_TRUE(planner.isWatchedFappPositionSafe(.55*forward+Vector3d(0,0,.6)));
    EXPECT_TRUE(planner.isWatchedFappPositionSafe(Vector3d(0,0,1.0)));
    // 未锁定障碍时的历史查询也必须裁去旋转AABB多出的前后角落。
    planner.watchFappObstacle(nullptr);
    planner.fapp_states_.clear();
    const Vector3d beyond=.55*forward+Vector3d(0,0,.6);
    EXPECT_TRUE(planner.isFappPathSafe({beyond-.02*side,beyond+.02*side}));
    EXPECT_FALSE(planner.isFappPathSafe({Vector3d(0,0,.6),.2*side+Vector3d(0,0,.6)}));
    planner.watchFappObstacle(&ball);
    const auto old_axis=ball.motion_axis;
    planner.setFappCorridorDirection(side); // 飞机拐弯不能改变原障碍摆动方向。
    message->header.stamp.fromSec(now);
    planner.fappCallback(message);
    EXPECT_TRUE(planner.fapp_watched_state_.motion_axis.isApprox(old_axis));
  }

  static void runTurnPreemptsSpecialMotion() {
    ros::NodeHandle nh("~");
    nh.setParam("manager/use_geometric_path", true);
    nh.setParam("astar/resolution_astar", 0.05);
    nh.setParam("astar/allocate_num", 100000);
    nh.setParam("astar/lambda_heu", 1.0);
    nh.setParam("astar/max_search_time", 0.1);
    FastExplorationManager manager;
    initialize(nh,manager);
    const Vector3d start(0.025,0.025,0.625);
    auto& task=*manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.enabled_=true;
    task.setCorridorFrame(Vector3d(-2,0,.625),Vector3d::UnitX());
    task.updateRobotPose(start,0);
    task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
    manager.turn_in_place_enabled_=true;
    manager.manual_fapp_active_=true;
    manager.ground_ascent_active_=true;
    manager.obstacle_clearance_height_=0.8;
    manager.low_probe_phase_=vertical_detour::LowProbePhase::DESCENDING;
    ASSERT_TRUE(manager.planner_manager_->planStraightTraj(start,start+Vector3d(1,0,0)));
    FastExplorationFSM fsm;
    fsm.expl_manager_=std::shared_ptr<FastExplorationManager>(&manager,[](FastExplorationManager*){});
    fsm.planner_manager_=manager.planner_manager_;
    fsm.fd_=std::make_shared<FSMData>();
    fsm.fp_=std::make_shared<FSMParam>();
    fsm.fd_->state_str_={"INIT","WAIT","PLAN","PUB","EXEC","FINISH"};
    fsm.fp_->emergency_brake_horizon_=.12;
    fsm.fd_->odom_pos_=start;
    fsm.fd_->odom_yaw_=0;
    fsm.active_traj_=manager.planner_manager_->local_data_;
    fsm.active_traj_.start_time_=ros::Time::now();
    fsm.active_traj_valid_=true;
    fsm.active_vertical_detour_=true;
    fsm.active_ground_ascent_=true;
    fsm.fapp_replan_pending_=true;
    fsm.state_=EXEC_TRAJ;
    fsm.emergency_brake_pub_=nh.advertise<std_msgs::Int32>("test_turn_brake",1);
    fsm.safetyCallback(ros::TimerEvent());
    EXPECT_EQ(fsm.state_,PLAN_TRAJ);
    EXPECT_TRUE(fsm.active_traj_braked_);
    EXPECT_TRUE(manager.turnInPlaceAlignmentPending());
    EXPECT_FALSE(fsm.active_ground_ascent_);
    EXPECT_FALSE(fsm.active_vertical_detour_);
    EXPECT_FALSE(fsm.fapp_replan_pending_);
    EXPECT_FALSE(manager.obstacleAscentPending());
    EXPECT_EQ(manager.low_probe_phase_,vertical_detour::LowProbePhase::IDLE);
    EXPECT_NEAR(manager.turn_in_place_final_yaw_,M_PI/2,1e-6);
    // 转弯会话优先于旧FAPP，残余速度不再阻止直接生成原地转向。
    EXPECT_EQ(manager.planExploreMotion(start,Vector3d(.4,0,0),Vector3d::Zero(),Vector3d::Zero()),SUCCEED);
    EXPECT_TRUE(manager.currentPlanIsTurnInPlace());
    EXPECT_FALSE(manager.currentPlanIsGroundAscent());
    EXPECT_TRUE(manager.turnInPlaceAlignmentPending());
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
    auto& planner = *manager.planner_manager_;
    manager.fapp_exploration_crossing_enabled_ = true;
    manager.task_search_manager_->enabled_ = true;
    planner.fapp_config_.enabled = planner.fapp_memory_enabled_ = true;
    fuel_fapp::DynamicObstacleState observation;
    observation.position = Vector3d(.6,0,.5);
    observation.size = Vector3d::Constant(.2);
    observation.stamp = ros::Time::now().toSec();
    planner.fapp_states_ = {observation};
    manager.manualFappObservationMissing(Vector3d(0,0,.45),0.0);
    ASSERT_TRUE(manager.manual_fapp_active_);
    EXPECT_DOUBLE_EQ(manager.manual_fapp_hold_height_, .65);
  }

  static void runMovingClimb(int mode, bool height_correction = false, bool manual = false) {
    ros::NodeHandle nh("~");
    nh.setParam("mission/task_search/min_height", 0.10);
    nh.setParam("mission/task_search/cruise_height", mode == 8 ? 0.625 : 0.60);
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
    const Vector3d end = start + (mode == 8 ? Vector3d(.4, 0, 0) :
        Vector3d(0.6, -0.3, mode == 7 ? .10 : 0.0));
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
    if (mode == 7) manager.ed_->path_next_goal_[1].z() = end.z();
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
    if (mode == 6) manager.manual_fapp_active_ = true;
    if (mode == 3) manager.low_probe_phase_ = vertical_detour::LowProbePhase::ADVANCING;
    Vector3d velocity = Vector3d::Zero();
    if (mode == 4) velocity.x() = std::numeric_limits<double>::quiet_NaN();
    bool raised;
    if (manual) {
      manager.manual_path_enabled_ = manager.manual_path_exploration_style_ = true;
      manager.manual_path_started_ = true;
      manager.manual_path_use_astar_ = false;
      manager.manual_path_.points = {end};
      manager.turn_in_place_enabled_ = false;
      task.exit_detection_enabled_ = false;
      raised = manager.planExploreMotion(start, velocity, Vector3d::Zero(), Vector3d::Zero()) == SUCCEED;
      EXPECT_EQ(manager.manual_path_.points.front(), end);
      EXPECT_EQ(manager.manual_path_.next, 0u);
    } else {
      raised = manager.tryCruiseHeightRecovery(start, velocity, Vector3d::Zero(), 0.0);
    }
    check(raised == (mode == 0 || mode == 1 || mode == 5 || mode == 7 || mode == 8),
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
    const double rise = mode == 1 ? 0.075 : (mode == 5 ? 0.06 :
        (mode == 8 ? 0.30 : 0.275));
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
    check(actual_end.x() > start.x()+(mode == 8 ? 0.3 : 0.5) &&
          actual_end.z() > start.z()+rise-0.02,
          "generated trajectory did not both advance and regain height");
  }

  static void runPostFappClearanceRecovery(int mode) {
    ros::NodeHandle nh("~post_fapp_clearance_" + std::to_string(mode));
    nh.setParam("sdf_map/virtual_ceil_height", -10.0);
    nh.setParam("mission/task_search/min_height", .10);
    nh.setParam("mission/task_search/cruise_height", .60);
    nh.setParam("mission/task_search/max_goal_climb", .30);
    nh.setParam("fapp/enabled", true);
    nh.setParam("fapp/memory_enabled", true);
    nh.setParam("fapp/planner_priority", true);
    FastExplorationManager manager;
    initialize(nh, manager);
    auto& planner = *manager.planner_manager_;
    auto& map = *manager.sdf_map_;
    auto& task = *manager.task_search_manager_;
    task.setMap(manager.sdf_map_);
    task.global_no_return_ = false;
    task.stable_progress_direction_ = Eigen::Vector2d::UnitX();
    task.stable_progress_direction_valid_ = true;
    manager.entry_forward_complete_ = true;
    manager.mission_entered_search_region_ = false; // 动态入口无需命中旧固定搜索框。
    manager.fapp_exploration_crossing_enabled_ = true;
    const Vector3d start(.025,.025,.125), end(.825,-.275,.125), zero=Vector3d::Zero();
    task.updateRobotPose(start,0.0);
    std::fill(map.md_->occupancy_buffer_.begin(),map.md_->occupancy_buffer_.end(),
              map.mp_->clamp_min_log_);
    for (int ix=-39; ix<40; ++ix) for (int iy=-39; iy<40; ++iy) {
      const auto fill = [&](double z, double value) {
        Eigen::Vector3i index;
        map.posToIndex(Vector3d(ix*.05+.025,iy*.05+.025,z),index);
        map.md_->occupancy_buffer_[map.toAddress(index)] = value;
      };
      fill(.025,map.mp_->clamp_max_log_);
      if (mode == 1) fill(.475,map.mp_->clamp_max_log_);
      if (mode == 2) fill(.225,map.mp_->clamp_max_log_);
      if (mode == 3) for (int iz=4; iz<39; ++iz)
        fill(iz*.05+.025,map.mp_->clamp_min_log_-1.0);
    }
    map.md_->local_bound_min_.setZero();
    map.md_->local_bound_max_ = map.mp_->map_voxel_num_-Eigen::Vector3i::Ones();
    map.clearAndInflateLocalMap(); map.updateESDF3d();
    manager.ed_->path_next_goal_ = {start,.5*(start+end),end};
    manager.ed_->next_goal_ = end;
    ASSERT_TRUE(planner.planExploreTraj(manager.ed_->path_next_goal_,zero,zero,0.0));
    ASSERT_TRUE(planner.isTrajectorySafe());
    LocalTrajData original = planner.local_data_;
    fuel_fapp::DynamicObstacleState ball;
    ball.position=Vector3d(-.65,0,.65); ball.size=Vector3d::Constant(.20);
    ball.stamp=ros::Time::now().toSec();
    planner.fapp_states_={ball}; planner.watchFappObstacle(&ball);
    manager.manual_fapp_active_=true;
    manager.manual_fapp_forward_=Vector3d::UnitX();
    manager.manual_fapp_exit_progress_=-1.0;
    manager.manual_fapp_hold_height_=manager.horizontal_reference_height_=start.z();
    // 未来轨迹端点过线不释放低位；必须由实际位置确认已经通过。
    EXPECT_FALSE(manager.manualFappObservationMissing(start,0.0));
    ASSERT_TRUE(manager.manual_fapp_active_);
    EXPECT_FALSE(manager.tryCruiseHeightRecovery(start,zero,zero,0.0));
    EXPECT_FALSE(manager.manualFappObservationMissing(start,0.0,true));
    ASSERT_FALSE(manager.manual_fapp_active_);
    const bool raised=manager.tryCruiseHeightRecovery(start,zero,zero,0.0);
    ASSERT_EQ(raised,mode<2);
    if (!raised) {
      EXPECT_TRUE(original.position_traj_.getControlPoint().isApprox(
          planner.local_data_.position_traj_.getControlPoint()));
      EXPECT_TRUE(manager.ed_->next_goal_.isApprox(end));
      return;
    }
    const Vector3d selected=manager.ed_->next_goal_;
    EXPECT_GT(selected.z(),start.z()+.05);
    EXPECT_LT(selected.z(),.60); // 可行点不必固定在巡航0.6m。
    EXPECT_TRUE(selected.head<2>().isApprox(end.head<2>()));
    EXPECT_TRUE(planner.isTrajectorySafe());
    if (mode==1) {
      EXPECT_LT(selected.z(),.40); // 高候选被顶棚挡住，改走更低且净空更好的点。
      EXPECT_GE(map.getDistance(selected),map.getDistance(Vector3d(end.x(),end.y(),.325)));
    }
    for (double t=0; t<=planner.local_data_.duration_; t+=.02)
      EXPECT_TRUE(planner.isKnownFreePosition(planner.local_data_.position_traj_.evaluateDeBoorT(t)));
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
    // 普通弧形航向变化不抢占；地图已确认的离散转弯应立即打断旧平移。
    manager.turn_in_place_enabled_ = true;
    manager.mission_workspace_lock_received_ = true;
    manager.entry_forward_complete_ = true;
    task.setCorridorFrame(Vector3d(-3, 0, 1), Vector3d::UnitX());
    task.updateRobotPose(fsm.fd_->odom_pos_, 0.0);
    task.turn_yaw_follow_latch_.arm(Eigen::Vector2d::UnitY());
    fsm.last_suspended_underpass_check_ = ros::Time::now() + ros::Duration(30.0);
    ros::NodeHandle nh("~");
    fsm.emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("test_route_brake", 1);
    fsm.safetyCallback(ros::TimerEvent());
    check(fsm.state_ == PLAN_TRAJ && fsm.active_traj_braked_ &&
          manager.turnInPlaceAlignmentPending(),
          "confirmed mapped turn waited for the old translation endpoint");
    Vector3d turn_direction;
    EXPECT_TRUE(task.mappedCorridorDirection(0.0, turn_direction));
    EXPECT_TRUE(turn_direction.isApprox(Vector3d::UnitY()));
    manager.completeTurnInPlace(fsm.fd_->odom_pos_);

    // 路线保持不能屏蔽新出现的占据：安全回调必须立刻刹停并交回规划。
    fsm.state_ = EXEC_TRAJ;
    fsm.active_traj_braked_ = false;
    fsm.active_traj_ = manager.planner_manager_->local_data_;
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
                                   bool expired_descent = false, int previous_obstacle = 0) {
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
    if (remember_side)
      manager.recovery_side_obstacle_ = manager.makeSplitLane(
          start, Vector3d::UnitX(), split, wide_side > 0 ? -1 : 1);
    if (previous_obstacle == 1) {
      // 前一个静态障碍尚在保持侧向的尾部余量内，但当前挡路的是另一个柱子。
      manager.recovery_side_obstacle_.origin.x() -= 0.65;
      manager.wide_side_bypass_active_ = true;
      manager.wide_side_bypass_forward_ = Vector3d::UnitX();
      manager.wide_side_bypass_release_distance_ = 2.0;
      ASSERT_FALSE(manager.sameRecoveryObstacle(start, Vector3d::UnitX(), split));
    } else if (previous_obstacle >= 2) {
      manager.recovery_side_obstacle_ = {};
      manager.manual_fapp_active_ = true;
      manager.manual_fapp_forward_ = Vector3d::UnitX();
      manager.manual_fapp_exit_progress_ = 0.30;
      manager.manual_fapp_hold_height_ = start.z();
      planner.fapp_config_.enabled = planner.fapp_memory_enabled_ = true;
      planner.fapp_planner_priority_ = true;
      planner.fapp_watched_valid_ = true;
      auto& ball = planner.fapp_watched_state_;
      ball.position = previous_obstacle == 2 ? Vector3d(-.2, 0, 1.6) :
          (previous_obstacle >= 4 ? Vector3d(.55, 0, start.z())
                                  : Vector3d(.1, wide_side*.5, start.z()));
      ball.size = Vector3d::Constant(.10);
      ball.stamp = ros::Time::now().toSec();
      planner.fapp_watched_lower_ = ball.position - Vector3d(.412, .412, .275);
      planner.fapp_watched_upper_ = ball.position + Vector3d(.412, .412, .275);
      planner.fapp_states_ = {ball};
      ASSERT_FALSE(manager.sameRecoveryObstacle(start, Vector3d::UnitX(), split));
    }
    if (previous_obstacle >= 4) {
      if (previous_obstacle == 4) {
        manager.recovery_side_obstacle_ = manager.makeSplitLane(
            start, Vector3d::UnitX(), split, wide_side > 0 ? -1 : 1);
        manager.recovery_side_obstacle_.origin.x() -= .65;
      }
      manager.fapp_exploration_crossing_enabled_ = true;
      Vector3d target; double target_yaw; bool detected = false;
      vector<Vector3d> route;
      ASSERT_TRUE(manager.buildFappSideBypass(start, 0, Vector3d::UnitX(),
          target, target_yaw, detected, &route));
      EXPECT_TRUE(detected);
      EXPECT_GT((target.y()-start.y())*wide_side, .4);
      EXPECT_TRUE(planner.isPathSafe(route));
      EXPECT_TRUE(planner.isFappPathSafe(route));
      EXPECT_EQ(manager.recovery_side_obstacle_.side, 0);
      return;
    }
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
    const double expected_side = previous_obstacle == 1 || previous_obstacle == 2
        ? wide_side : (remember_side ? -wide_side : wide_side);
    check((next-start).y()*expected_side >= 0.10 &&
          std::fabs(next.z()-start.z()) < 0.05 &&
          manager.recovery_side_dir_.y()*expected_side > 0.9,
          "width comparison replaced a feasible remembered side or initial choice ignored width");
    if (previous_obstacle) {
      EXPECT_TRUE(planner.isPathSafe(manager.ed_->path_next_goal_.empty()
          ? vector<Vector3d>{start, next} : manager.ed_->path_next_goal_));
      if (previous_obstacle >= 2) {
        EXPECT_TRUE(planner.isFappPathSafe(manager.ed_->path_next_goal_.empty()
            ? vector<Vector3d>{start, next} : manager.ed_->path_next_goal_));
      }
      if (previous_obstacle == 2) {
        // 旧动态会话结束，也不能清掉已为新静态障碍选好的另一侧。
        manager.recordPublishedTrajectoryHeight();
        manager.fapp_exploration_crossing_enabled_ = true;
        const auto selected_obstacle = manager.recovery_side_obstacle_;
        ASSERT_NE(selected_obstacle.side, 0);
        const auto chosen_controls = planner.local_data_.position_traj_.getControlPoint();
        EXPECT_FALSE(manager.planExplorationFappCrossing(
            start, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero()));
        EXPECT_TRUE(chosen_controls.isApprox(planner.local_data_.position_traj_.getControlPoint()));
        EXPECT_FALSE(manager.manualFappObservationMissing(
            start + Vector3d(.5, 0, 0), 0.0, true));
        EXPECT_FALSE(manager.manual_fapp_active_);
        EXPECT_TRUE(manager.recovery_side_latched_);
        EXPECT_EQ(manager.recovery_side_obstacle_.side, selected_obstacle.side);
      }
    }
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
                                    bool height_drift = false, bool prefer_near_obstacle = false) {
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
    if (prefer_near_obstacle) manager.preferred_obstacle_distance_ = 0.20;
    check(manager.buildWideSideBypass(origin, 0.0, Vector3d::UnitX(), next, yaw, split) &&
          manager.wide_side_bypass_active_ && (next-origin).y()*side > 0.15 &&
          next.x() >= origin.x()+0.19,
          "initial bypass did not choose the wider lane with available forward progress");
    if (prefer_near_obstacle) {
      const auto evidence=manager.findWideSideSplit(origin,Vector3d::UnitX());
      const double edge=side>0 ? evidence.obstacle_max_offset : evidence.obstacle_min_offset;
      EXPECT_NEAR(std::fabs((next-origin).y()-edge),0.20,0.026);
      EXPECT_TRUE(manager.planner_manager_->isPositionSafe(next));
      return;
    }
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
    const double xmin=scenario==1?0.0:-.6, xmax=scenario==0?.05:.6;
    for (double x=xmin; x<=xmax+1e-6; x+=.05)
      for (double y=-.05; y<=.05+1e-6; y+=.05)
        for (double z=.9; z<=1.1+1e-6; z+=.05) map.setOccupied(Vector3d(x+.025,y+.025,z+.025));
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
    manager.manual_fapp_active_=true;
    manager.manual_fapp_hold_height_=pos.z();
    ASSERT_TRUE(manager.planner_manager_->isControlledEscapePosition(pos));
    ASSERT_TRUE(manager.planInflationHistoryEscape(pos,zero,zero,zero));
    const Vector3d delta=manager.ed_->next_goal_-pos;
    EXPECT_NEAR(delta.z(),0,1e-6);
    if (scenario==0) { EXPECT_GT(delta.x(),.04); EXPECT_NEAR(delta.y(),0,1e-6); }
    if (scenario==1) { EXPECT_LT(delta.x(),-.04); EXPECT_NEAR(delta.y(),0,1e-6); }
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
TEST(UpperClearance, HighestVoxelPlusTwentyBeforeForward) {
  fast_planner::TurnEscapeTest::runObstacleClearance(false);
}

TEST(ManualStrategyAlignment, ShortClearWaypointRouteRecoversThirtyCentimetresInOneSegment) {
  fast_planner::TurnEscapeTest::runMovingClimb(8, false, true);
}
TEST(ManualStrategyAlignment, TemporaryDynamicFailureRetriesWithoutReplacingValidTrajectory) {
  fast_planner::TurnEscapeTest::runManualSuspended("manual_entry_retry");
}
TEST(ManualStrategyAlignment, LowDynamicPrefixStopsAtCurrentWaypoint) {
  fast_planner::TurnEscapeTest::runLocalFappStep(7, true);
}
TEST(ManualStrategyAlignment, DynamicDescentAndLowPrefixUseSharedPlanner) {
  fast_planner::TurnEscapeTest::runLocalFappStep(0, true);
}
TEST(ManualStrategyAlignment, StaticBypassRetriesDynamicDescentAfterMapClears) {
  fast_planner::TurnEscapeTest::runLocalFappStep(9, true);
}
TEST(ManualStrategyAlignment, LowObstacleStartsStationaryAscentBeforeWaypoint) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(6, true);
}
TEST(ManualStrategyAlignment, AscentDriftCorrectsOriginalXYThroughManualEntry) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(0, true);
}
TEST(ManualStrategyAlignment, BlockedAscentRetreatsThenClimbs) {
  fast_planner::TurnEscapeTest::runBlockedAscentRetreat(false, true);
}
TEST(ManualStrategyAlignment, RearObstacleStillRejectsAscentRetreat) {
  fast_planner::TurnEscapeTest::runBlockedAscentRetreat(true, true);
}

TEST(GroundAscentXY, RepeatedDriftReturnsToOriginalXYBeforeClimbing) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(0);
}
TEST(GroundAscentXY, BlockedCorrectionDoesNotMoveAnchorOrClimb) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(1);
}
TEST(GroundAscentXY, DriftWaitsForAscentEndpointBeforeCorrection) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(2);
}
TEST(GroundAscentXY, HorizontalCorrectionIsNotInterruptedByAscentCheck) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(3);
}
TEST(GroundAscentXY, OrdinaryTrajectoryKeepsExistingTrackingRules) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(4);
}
TEST(GroundAscentXY, ActiveObstacleRetreatIsExemptFromOldAscentAnchor) {
  fast_planner::TurnEscapeTest::runGroundAscentXY(5);
}
TEST(GroundAscentXY, BlockedAscentRetreatsBeyondShortStepThenClimbs) {
  fast_planner::TurnEscapeTest::runBlockedAscentRetreat(false);
}
TEST(GroundAscentXY, BlockedAscentCannotRetreatThroughRearObstacle) {
  fast_planner::TurnEscapeTest::runBlockedAscentRetreat(true);
}
TEST(FappLocalStep, RemoteStaticObstacleAllowsStationaryDescentThenPartialCrossing) {
  fast_planner::TurnEscapeTest::runLocalFappStep(0);
}
TEST(FappReferenceClearance, RecordedBottomAllowsNineteenCentimetreDescentAndCrossing) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(0);
}
TEST(FappReferenceClearance, ContinuousUnderpassUsesSameRawBottomMinusTen) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(1);
}
TEST(FappReferenceClearance, StaticObstacleStillRejectsDescent) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(2);
}
TEST(FappReferenceClearance, StaticBypassContinuesAtFifteenWithoutReturningToNineteen) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(3);
}
TEST(FappReferenceClearance, FiveCentimetreMinimumContinuesForward) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(4);
}
TEST(FappReferenceClearance, BelowFiveDoesNotCompleteDescent) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(5);
}
TEST(FappReferenceClearance, AboveObstacleClearanceDoesNotCompleteDescent) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(6);
}
TEST(FappReferenceClearance, TenCentimetresContinuesForward) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(7);
}
TEST(FappReferenceClearance, ContinuousCrossingKeepsActualFifteenCentimetres) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(8);
}
TEST(FappReferenceClearance, HorizontalTrackingOffsetDoesNotRaiseAllowedLowHeight) {
  fast_planner::TurnEscapeTest::runFappReferenceClearance(9);
}
TEST(FappLocalStep, OccupiedColumnMovesToNearbyFreeDescentPoint) {
  fast_planner::TurnEscapeTest::runLocalFappStep(1);
}
TEST(FappLocalStep, UnknownColumnDoesNotAuthorizeStationaryDescent) {
  fast_planner::TurnEscapeTest::runLocalFappStep(2);
}
TEST(FappLocalStep, InsufficientHeightDoesNotStartDescent) {
  fast_planner::TurnEscapeTest::runLocalFappStep(3);
}
TEST(FappLocalStep, DistantObstacleApproachesBeforeDescending) {
  fast_planner::TurnEscapeTest::runLocalFappStep(4);
}
TEST(FappLocalStep, SafePrefixShorterThanTwentyCentimetresIsAllowed) {
  fast_planner::TurnEscapeTest::runLocalFappStep(5);
}
TEST(FappLocalStep, ChangedStaticMapRejectsDescentBeforeRelease) {
  fast_planner::TurnEscapeTest::runLocalFappStep(6);
}
TEST(FappLocalStep, SafeEndpointMayExtendMoreThanTwentyCentimetresBeyondEnvelope) {
  fast_planner::TurnEscapeTest::runLocalFappStep(7);
}
TEST(FappLocalStep, StaticBypassStillRechecksStationaryDescent) {
  fast_planner::TurnEscapeTest::runLocalFappStep(8);
}
TEST(FappLocalStep, RejectedStaticColumnRetriesAfterMapClears) {
  fast_planner::TurnEscapeTest::runLocalFappStep(9);
}
TEST(StaticInflationNoise, RepeatedIsolatedReturnsStayOutOfInflationNearVehicle) {
  fast_planner::TurnEscapeTest::runIsolatedInflationNearVehicle();
}
TEST(StaticInflationNoise, CurrentSupportRequiredAndThinPostsPreserved) {
  fast_planner::TurnEscapeTest::runCurrentInflationSupport();
}
TEST(StaticInflationNoise, FreeRaysDecayPreviouslySaturatedEvidence) {
  fast_planner::TurnEscapeTest::runInflationEvidenceDecay(false);
}
TEST(StaticInflationNoise, ExplicitStaticLockStillRetainsConfirmedObstacles) {
  fast_planner::TurnEscapeTest::runInflationEvidenceDecay(true);
}
TEST(StaticInflationNoise, RecordedOctober27IsolatedVoxelDoesNotClosePassage) {
  const char* fixture = std::getenv("STATIC_INFLATION_FIXTURE");
  if (!fixture) GTEST_SKIP();
  fast_planner::TurnEscapeTest::runRecordedStaticInflation(fixture);
}
TEST(BlindRangeFusion, NearbyObstacleSurvivesMissingReturnsAndBlocksTurn) {
  fast_planner::TurnEscapeTest::runBlindRangeFusion(true);
}
TEST(BlindRangeFusion, UnobservedBlindRegionIsNotCarvedFree) {
  fast_planner::TurnEscapeTest::runBlindRangeFusion(false);
}
TEST(TurnHeight, TrackingDropAndResidualVelocityDoNotDelayYawOrTriggerAscent) {
  fast_planner::TurnEscapeTest::runRetainedTurnHeight(false);
}
TEST(TurnHeight, BelowTwentyCentimetresBlocksTurnButNotTranslation) {
  fast_planner::TurnEscapeTest::runTurnMinimumHeight();
}
TEST(TurnHeight, DropBelowTwentyCentimetresRejectsPendingPublication) {
  fast_planner::TurnEscapeTest::runStaticTurnCollision(1, true);
}
TEST(TurnHeight, DropBelowTwentyCentimetresStopsExecutingTurn) {
  fast_planner::TurnEscapeTest::runStaticTurnCollision(2, true);
}
TEST(TurnHeight, RealObstacleStillBlocksRotationWithoutForcingAscent) {
  fast_planner::TurnEscapeTest::runRetainedTurnHeight(true);
}
TEST(CruiseHeightRecovery, TenCentimetreToleranceAvoidsTinyRecoveryButKeepsRealAscent) {
  fast_planner::TurnEscapeTest::runCruiseRecoveryTolerance();
}
TEST(RaisedStaticSide, PreferredLowObstacleRouteRisesAndContinuesForward) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(6);
}
TEST(CruiseHeightRecovery, TrackingHeightCorrectionDoesNotSuppressFurtherClimb) {
  fast_planner::TurnEscapeTest::runMovingClimb(7);
}
TEST(RaisedStaticSide, LowObstacleUsesSaferLeftSideAboveTop) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(0);
}
TEST(RaisedStaticSide, MirroredSceneUsesSaferRightSideAboveTop) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(1);
}
TEST(RaisedStaticSide, TwentyCentimetersIsMinimumAndHigherRouteRemainsAvailable) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(2);
}
TEST(RaisedStaticSide, TallerObstacleDoesNotUseLowObstacleCandidate) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(3);
}
TEST(RaisedStaticSide, UnknownUpperSpaceDoesNotProduceRoute) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(4);
}
TEST(RaisedStaticSide, InsufficientTaskHeightDoesNotLowerRequiredClearance) {
  fast_planner::TurnEscapeTest::runRaisedStaticSide(5);
}
TEST(UpperClearance, BlockedAscentLeavesOtherRoutesAvailable) {
  fast_planner::TurnEscapeTest::runObstacleClearance(true);
}
TEST(UpperClearance, SafeHighReferenceStillWaitsForAircraftHeight) {
  fast_planner::TurnEscapeTest::runObstacleClearance(false,true);
}
TEST(MappedTurn, InterruptsFlatCrossingWithStaleVerticalFlags) {
  fast_planner::TurnEscapeTest::runTurnPreemptsSpecialMotion();
}
TEST(MappedTurn, PublishedDiagonalClimbFinishesBeforeYaw) {
  fast_planner::TurnEscapeTest::runClimbBeforeMappedTurn(0);
}
TEST(MappedTurn, PendingReplacementCannotLetYawCancelPublishedClimb) {
  fast_planner::TurnEscapeTest::runClimbBeforeMappedTurn(1);
}
TEST(MappedTurn, RealCollisionStillStopsClimb) {
  fast_planner::TurnEscapeTest::runClimbBeforeMappedTurn(2);
}
TEST(MappedTurn, ClearTurnSiteAllowsConnectedBentDeparture) {
  fast_planner::TurnEscapeTest::runPassableTurn(10);
}
TEST(MappedTurn, NarrowTurnSiteWaitsForBetterPositionClearance) {
  fast_planner::TurnEscapeTest::runPassableTurn(11);
}
TEST(FappEnvelope, HistoryExpandsLaterallyWithoutForeAftOrUpperOutlierGrowth) {
  fast_planner::TurnEscapeTest::runLateralFappEnvelope(false);
}
TEST(FappEnvelope, RotatedCorridorKeepsOriginalObstacleAxis) {
  fast_planner::TurnEscapeTest::runLateralFappEnvelope(true);
}
TEST(TurnStaticSafety, BlockedRotationKeepsSafeTranslationUntilClearEndpoint) {
  fast_planner::TurnEscapeTest::runStaticTurnCollision(0);
}
TEST(TurnStaticSafety, BlockedRotationRetreatsAlongValidatedHistoryBeforeContact) {
  fast_planner::TurnEscapeTest::runTurnRetreatOrEscapeRelease(false);
}
TEST(InflationEscape, ReleaseUsesEscapeTrajectoryStartAcrossVoxelBoundary) {
  fast_planner::TurnEscapeTest::runTurnRetreatOrEscapeRelease(true);
}
TEST(TurnStaticSafety, NewObstacleRejectsAlreadyGeneratedTurnBeforePublish) {
  fast_planner::TurnEscapeTest::runStaticTurnCollision(1);
}
TEST(TurnStaticSafety, ExpandedRadiusBlocksTurnButAllowsHorizontalTranslation) {
  fast_planner::TurnEscapeTest::runTurnRadiusIndependentOfTranslation(true);
}
TEST(TurnStaticSafety, ObstacleOutsideTwentyFiveCentimetresAllowsTurn) {
  fast_planner::TurnEscapeTest::runTurnRadiusIndependentOfTranslation(false);
}
TEST(TurnStaticSafety, ActualFootprintCollisionStopsTurnDespiteSafeReference) {
  fast_planner::TurnEscapeTest::runStaticTurnCollision(2);
}

TEST(CruiseHeightRecovery, OpenSpaceClimbsAlongCommittedRightRoute) {
  fast_planner::TurnEscapeTest::runMovingClimb(0);
}
TEST(CruiseHeightRecovery, MidRouteRoofUsesLowerClearClimb) {
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
TEST(CruiseHeightRecovery, ShortClearRouteClimbsThirtyCentimetersInOneTrajectory) {
  fast_planner::TurnEscapeTest::runMovingClimb(8);
}
TEST(PostFappRecovery, ActualPassageSelectsRaisedPointAfterDynamicEntrance) {
  fast_planner::TurnEscapeTest::runPostFappClearanceRecovery(0);
}
TEST(PostFappRecovery, LowCeilingSelectsClearerPointBelowCruiseHeight) {
  fast_planner::TurnEscapeTest::runPostFappClearanceRecovery(1);
}
TEST(PostFappRecovery, BlockedAscentRetainsValidatedRoute) {
  fast_planner::TurnEscapeTest::runPostFappClearanceRecovery(2);
}
TEST(PostFappRecovery, UnknownAscentRetainsValidatedRoute) {
  fast_planner::TurnEscapeTest::runPostFappClearanceRecovery(3);
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
TEST(FappMemory, PlannerPriorityAdvancesFromInsideHistoricalEnvelope) {
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

TEST(SideRecoveryScope, PreviousStaticObstacleDoesNotChooseNextRightPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, false, true, false, false, 1);
}
TEST(SideRecoveryScope, PreviousStaticObstacleDoesNotChooseNextLeftPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false, true, false, false, 1);
}
TEST(SideRecoveryScope, PreviousDynamicObstacleDoesNotChooseNextRightPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, false, true, false, false, 2);
}
TEST(SideRecoveryScope, PreviousDynamicObstacleDoesNotChooseNextLeftPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, false, true, false, false, 2);
}
TEST(SideRecoveryScope, WiderRightPassageStillRejectsActualDynamicCollision) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, true, true, false, false, 3);
}
TEST(SideRecoveryScope, WiderLeftPassageStillRejectsActualDynamicCollision) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, true, true, false, false, 3);
}
TEST(SideRecoveryScope, PreviousStaticSideDoesNotChooseNextDynamicRightPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, true, true, false, false, 4);
}
TEST(SideRecoveryScope, PreviousStaticSideDoesNotChooseNextDynamicLeftPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, true, true, false, false, 4);
}
TEST(SideRecoveryScope, PreviousDynamicSideDoesNotChooseNextDynamicRightPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(-1.0, true, true, false, false, 5);
}
TEST(SideRecoveryScope, PreviousDynamicSideDoesNotChooseNextDynamicLeftPassage) {
  fast_planner::TurnEscapeTest::runWiderSideRecovery(1.0, true, true, false, false, 5);
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
TEST(PassableTurn, LocalObstacleKeepsStraightCorridorDirection) {
  fast_planner::TurnEscapeTest::runLocalObstacleKeepsCorridor(0.0);
}
TEST(PassableTurn, RotatedLocalObstacleKeepsStraightCorridorDirection) {
  fast_planner::TurnEscapeTest::runLocalObstacleKeepsCorridor(M_PI / 2.0);
}
TEST(PassableTurn, RecordedOctober25CornerSelectsRight) {
  const char* fixture = std::getenv("TURN_PASSABILITY_FIXTURE");
  if (!fixture) GTEST_SKIP() << "requires October 25 pre-turn point clouds";
  fast_planner::TurnEscapeTest::runRecordedTurn(fixture);
}

TEST(AscentRetry, ExplorationRetreatsAfterThreeThenUsesDiagonalAndSideAscent) {
  fast_planner::TurnEscapeTest::runAscentRetryPolicy(false);
}
TEST(AscentRetry, WaypointsUseSameRetryPolicyWithoutAdvancingWaypoint) {
  fast_planner::TurnEscapeTest::runAscentRetryPolicy(true);
}

TEST(ReturnStraight, ExactReverseRemainsStableThroughLocalAvoidance) { fast_planner::TurnEscapeTest::runReturnStraight(0); }
TEST(ReturnStraight, DisabledPreservesExistingDirection) { fast_planner::TurnEscapeTest::runReturnStraight(1); }
TEST(ReturnStraight, EntryAndFirstLegCannotTurnBack) { fast_planner::TurnEscapeTest::runReturnStraight(2); }
TEST(ReturnStraight, OccupiedReturnLegDoesNotAlign) { fast_planner::TurnEscapeTest::runReturnStraight(3); }
TEST(ReturnStraight, UnknownReturnLegDoesNotAlign) { fast_planner::TurnEscapeTest::runReturnStraight(4); }
TEST(ReturnStraight, SmallRemainingAngleStillAligns) { fast_planner::TurnEscapeTest::runReturnStraight(5); }
TEST(ReturnStraight, RotatedEntranceUsesRelativeReverse) { fast_planner::TurnEscapeTest::runReturnStraight(6); }
TEST(ReturnStraight, RecordedOctober30ExitLeg) {
  const char* fixture=std::getenv("RETURN_STRAIGHT_FIXTURE");
  if (!fixture) GTEST_SKIP() << "RETURN_STRAIGHT_FIXTURE not provided";
  fast_planner::TurnEscapeTest::runRecordedTurn(fixture,2);
}
TEST(ExitPortal, RecordedOctober30AutomaticExit) {
  const char* fixture=std::getenv("AUTO_EXIT_FIXTURE");
  if (!fixture) GTEST_SKIP() << "AUTO_EXIT_FIXTURE not provided";
  fast_planner::TurnEscapeTest::runRecordedTurn(fixture,3);
}
TEST(AutomaticExit, LowFlightUsesRailsExcludesFloor) { fast_planner::TurnEscapeTest::runLowExit(0); }
TEST(AutomaticExit, BlockedDoorRejected) { fast_planner::TurnEscapeTest::runLowExit(1); }
TEST(AutomaticExit, UnknownOutsideRejected) { fast_planner::TurnEscapeTest::runLowExit(2); }
TEST(AutomaticExit, SingleSideOpeningRejected) { fast_planner::TurnEscapeTest::runLowExit(3); }
TEST(AutomaticExit, FloorAloneIsNotCorridor) { fast_planner::TurnEscapeTest::runLowExit(4); }
TEST(AutomaticExit, ContinuousCorridorIsNotExit) { fast_planner::TurnEscapeTest::runLowExit(5); }
TEST(AutomaticExit, ShortFrameSeamIsNotExit) { fast_planner::TurnEscapeTest::runLowExit(6); }
TEST(ManualExit, ApproachYawAndActualArrivalBeforeSharing) { fast_planner::TurnEscapeTest::runManualExit(0); }
TEST(ManualExit, DisabledCoordinatesDoNotAffectPlanning) { fast_planner::TurnEscapeTest::runManualExit(1); }
TEST(ManualExit, OtherPartsOfPlaneAreNotDoor) { fast_planner::TurnEscapeTest::runManualExit(2); }
TEST(ManualExit, OccupiedDoorCannotTakeOver) { fast_planner::TurnEscapeTest::runManualExit(3); }
TEST(CorridorTurnBoundary, OutsideContinuationDoesNotVetoBend) { fast_planner::TurnEscapeTest::runCorridorTurnBoundary(0); }
TEST(CorridorTurnBoundary, InsideBypassStillKeepsDirection) { fast_planner::TurnEscapeTest::runCorridorTurnBoundary(1); }
TEST(CorridorTurnBoundary, PhysicalTurnCanUpdateStaleBoundary) { fast_planner::TurnEscapeTest::runCorridorTurnBoundary(2); }
TEST(CorridorTurnBoundary, PublishedEscapeReleasesStaleGoal) { fast_planner::TurnEscapeTest::runCorridorTurnBoundary(3); }
TEST(CorridorTurnBoundary, RecordedOctober30ReconstructedBend) {
  const char* fixture=std::getenv("CORRIDOR_TURN_FIXTURE");
  if (!fixture) GTEST_SKIP() << "CORRIDOR_TURN_FIXTURE not provided";
  fast_planner::TurnEscapeTest::runRecordedTurn(fixture,true);
}
TEST(CorridorLowFlight, IsolatedTopDoesNotUnlockAscent) { fast_planner::TurnEscapeTest::runCorridorLowFlight(0); }

TEST(CorridorLowFlight, SuspendedObstacleDoesNotSealBottom) { fast_planner::TurnEscapeTest::runCorridorLowFlight(1); }

TEST(CorridorLowFlight, SupportedFullBlockageUnlocksOnlyUpToOnePointOne) { fast_planner::TurnEscapeTest::runCorridorLowFlight(2); }

TEST(CorridorLowFlight, SideGapKeepsLowFlight) { fast_planner::TurnEscapeTest::runCorridorLowFlight(3); }

TEST(CorridorLowFlight, HalfMeterObstacleDoesNotUnlockAscent) { fast_planner::TurnEscapeTest::runCorridorLowFlight(4); }

TEST(CorridorLowFlight, UnknownSupportDoesNotUnlockAscent) { fast_planner::TurnEscapeTest::runCorridorLowFlight(5); }

TEST(CorridorLowFlight, SearchTrajectoryCurveAndOutsideLandingScope) { fast_planner::TurnEscapeTest::runCorridorLowFlight(6); }

TEST(CorridorLowFlight, LeftBypassPrefersTwentyCentimetersFromCenter) { fast_planner::TurnEscapeTest::runBypassContinuation(1,true,false,false,true); }
TEST(CorridorLowFlight, RightBypassPrefersTwentyCentimetersFromCenter) { fast_planner::TurnEscapeTest::runBypassContinuation(-1,true,false,false,true); }

TEST(AutomaticExit, OpaqueModeIgnoresOverheadRails) { fast_planner::TurnEscapeTest::runLowExit(0,false); }
TEST(AutomaticExit, OpaqueModeAcceptsSolidWalls) { fast_planner::TurnEscapeTest::runLowExit(0,false,true); }
TEST(AutomaticExit, TransparentModeAlsoAcceptsSolidWalls) { fast_planner::TurnEscapeTest::runLowExit(0,true,true); }
