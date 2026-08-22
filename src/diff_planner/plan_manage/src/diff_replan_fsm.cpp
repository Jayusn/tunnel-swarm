
#include <plan_manage/diff_replan_fsm.h>

#include <cmath>
#include <iomanip>

namespace
{
  bool sameScalar(double lhs, double rhs, double tolerance = 1e-9)
  {
    return std::isfinite(lhs) && std::isfinite(rhs) &&
           std::fabs(lhs - rhs) <= tolerance;
  }

  bool samePoint(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs)
  {
    return sameScalar(lhs.x, rhs.x) && sameScalar(lhs.y, rhs.y) &&
           sameScalar(lhs.z, rhs.z);
  }

  bool sameVector(const geometry_msgs::Vector3 &lhs, const geometry_msgs::Vector3 &rhs)
  {
    return sameScalar(lhs.x, rhs.x) && sameScalar(lhs.y, rhs.y) &&
           sameScalar(lhs.z, rhs.z);
  }

  bool sameQuaternion(const geometry_msgs::Quaternion &lhs,
                      const geometry_msgs::Quaternion &rhs)
  {
    return sameScalar(lhs.x, rhs.x) && sameScalar(lhs.y, rhs.y) &&
           sameScalar(lhs.z, rhs.z) && sameScalar(lhs.w, rhs.w);
  }
}

namespace diff_planner
{

  void DiffReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    // 2026-07-07: 允许按场景单独指定 waypoint 切换距离，避免沿默认经验值在窄通道里提前切段。
    nh.param("fsm/waypoint_switch_dist", no_replan_thresh_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/min_goal_distance", min_goal_distance_, 0.05);
    // SEARCH_TARGET 的上层会周期性重发当前子目标。只有目标位置发生实质变化时才重建全局轨迹，
    // 避免保活消息打断局部重规划并造成参考轨迹导数跳变。
    nh.param("fsm/search_subgoal_update_distance", search_subgoal_update_distance_, 0.15);
    // 手动/接力目标默认不去重；UAV1 接力实例显式开启，避免重发同一请求时重建轨迹。
    nh.param("fsm/manual_goal_update_distance", manual_goal_update_distance_, 0.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);
    nh.param("fsm/mondify_final_goal", mondify_final_goal_, true);
    nh.param("fsm/enable_stuck_detect", enable_stuck_detect_, true);
    // 2026-07-28: 保留原生Diff编队默认行为；异构FUEL->Diff接力由launch显式关闭此前序轨迹门槛。
    nh.param("fsm/require_pre_agent_trajectory", require_pre_agent_trajectory_, true);
    // 2026-07-28: 默认保留Diff原行为；UAV1接力实例关闭随机初始化，避免不可行目标让规划回调阻塞并丢心跳。
    nh.param("fsm/enable_random_global_init", enable_random_global_init_, true);
    // 2026-07-07: 允许首飞阶段或外部触发后的短时间内跳过 stuck detect，避免 indoor1 首段轨迹因局部目标变化慢被误判卡死。
    nh.param("fsm/stuck_detect_grace_time", stuck_detect_grace_time_, 6.0);
    // 2026-07-08: 为“RViz 只触发、自动搜索单独下发子目标”模式新增独立子目标话题，避免和 /goal 的触发语义混用。
    nh.param("fsm/search_subgoal_topic", search_subgoal_topic_, std::string("/corridor_search/subgoal"));
    nh.param("fsm/search_trigger_topic", search_trigger_topic_, std::string("/traj_start_trigger"));
    nh.param("fsm/search_goal_trigger_topic", search_goal_trigger_topic_, std::string("/goal"));
    // 2026-07-28: 外部目标话题参数化；双机时后机只接收接力管理器发布的/UAV1/planning/goal。
    nh.param("fsm/manual_goal_topic", manual_goal_topic_, std::string("/goal"));
    nh.param("fsm/exploration_goal_topic", exploration_goal_topic_,
             std::string("/UAV0/fuel_diff/goal"));
    nh.param("fsm/exploration_status_topic", exploration_status_topic_,
             std::string("/UAV0/fuel_diff/status"));
    nh.param("fsm/exploration_cancel_topic", exploration_cancel_topic_,
             std::string("/UAV0/fuel_diff/cancel"));
    nh.param("fsm/exploration_trigger_topic", exploration_trigger_topic_,
             std::string("/UAV0/corridor_search/exploration_trigger"));
    nh.param("fsm/mission_stage_topic", mission_stage_topic_,
             std::string("/UAV0/mission/task_status"));
    nh.param("fsm/exploration_yaw_topic", exploration_yaw_topic_,
             std::string("/UAV0/fuel_diff/yaw"));
    nh.param("fsm/exploration_arrival_xy", exploration_arrival_xy_, 0.30);
    nh.param("fsm/exploration_arrival_z", exploration_arrival_z_, 0.20);
    nh.param("fsm/exploration_arrival_velocity", exploration_arrival_velocity_, 0.10);
    nh.param("fsm/exploration_arrival_dwell", exploration_arrival_dwell_, 0.30);
    nh.param("fsm/exploration_arrival_yaw", exploration_arrival_yaw_, 0.15);
    nh.param("fsm/exploration_tracking_timeout", exploration_tracking_timeout_, 2.0);
    nh.param("fsm/exploration_trajectory_sample_time", exploration_trajectory_sample_time_, 0.03);
    nh.param("fsm/exploration_trajectory_sample_distance", exploration_trajectory_sample_distance_, 0.05);

    if (!std::isfinite(min_goal_distance_) || min_goal_distance_ <= 0.0)
    {
      ROS_WARN("Invalid fsm/min_goal_distance=%.3f; use 0.05 m.",
               min_goal_distance_);
      min_goal_distance_ = 0.05;
    }
    if (!std::isfinite(search_subgoal_update_distance_) || search_subgoal_update_distance_ <= 0.0)
    {
      ROS_WARN("Invalid fsm/search_subgoal_update_distance=%.3f; use 0.15 m.",
               search_subgoal_update_distance_);
      search_subgoal_update_distance_ = 0.15;
    }
    if (!std::isfinite(manual_goal_update_distance_) || manual_goal_update_distance_ < 0.0)
    {
      ROS_WARN("Invalid fsm/manual_goal_update_distance=%.3f; disable manual goal deduplication.",
               manual_goal_update_distance_);
      manual_goal_update_distance_ = 0.0;
    }
    if (!std::isfinite(exploration_arrival_xy_) || exploration_arrival_xy_ <= 0.0)
      exploration_arrival_xy_ = 0.30;
    if (!std::isfinite(exploration_arrival_z_) || exploration_arrival_z_ <= 0.0)
      exploration_arrival_z_ = 0.20;
    if (!std::isfinite(exploration_arrival_velocity_) || exploration_arrival_velocity_ <= 0.0)
      exploration_arrival_velocity_ = 0.10;
    if (!std::isfinite(exploration_arrival_dwell_) || exploration_arrival_dwell_ <= 0.0)
      exploration_arrival_dwell_ = 0.30;
    if (!std::isfinite(exploration_arrival_yaw_) || exploration_arrival_yaw_ <= 0.0)
      exploration_arrival_yaw_ = 0.15;
    if (!std::isfinite(exploration_tracking_timeout_) || exploration_tracking_timeout_ <= 0.0)
      exploration_tracking_timeout_ = 2.0;
    if (!std::isfinite(exploration_trajectory_sample_time_) ||
        exploration_trajectory_sample_time_ <= 0.0)
      exploration_trajectory_sample_time_ = 0.03;
    if (!std::isfinite(exploration_trajectory_sample_distance_) ||
        exploration_trajectory_sample_distance_ <= 0.0)
      exploration_trajectory_sample_distance_ = 0.05;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }


    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new DiffPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    have_trigger_ = !flag_realworld_experiment_;
    // 2026-07-07: 未显式给 waypoint_switch_dist 时保持原有经验公式，避免影响其他 launch 的默认行为。
    if (no_replan_thresh_ < 0.0)
      no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;

    /* initialize  Anomaly Detection Parameters */
    last_local_target_pos_.setZero();
    last_target_change_time_ = ros::Time::now().toSec();
    // 2026-07-07: 默认无豁免，仅在首段规划成功或外部触发后临时打开 stuck detect 豁免窗口。
    stuck_detect_ignore_until_ = 0.0;
    replan_fail_count_ = 0;
    TARGET_STUCK_TIME = 1.5 * planning_horizen_ / planner_manager_->pp_.max_vel_;
    need_hover_stop_ = false;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &DiffReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &DiffReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &DiffReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &DiffReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &DiffReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    // 2026-07-28: 后机接力不能仅凭目标已发布判断Diff正在执行，增加锁存状态反馈。
    planning_status_pub_ = nh.advertise<std_msgs::String>("planning/status", 10, true);
    exploration_status_pub_ =
        nh.advertise<quadrotor_msgs::ExplorationGoalStatus>(
            exploration_status_topic_, 10, true);
    exploration_yaw_pub_ = nh.advertise<std_msgs::Float64>(exploration_yaw_topic_, 2, true);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    exploration_goal_sub_ = nh.subscribe(
        exploration_goal_topic_, 10, &DiffReplanFSM::explorationGoalCallback, this);
    exploration_cancel_sub_ = nh.subscribe(
        exploration_cancel_topic_, 10, &DiffReplanFSM::explorationCancelCallback, this);
    exploration_trigger_sub_ = nh.subscribe(
        exploration_trigger_topic_, 2, &DiffReplanFSM::explorationTriggerCallback, this);
    mission_stage_sub_ = nh.subscribe(
        mission_stage_topic_, 10, &DiffReplanFSM::missionStageCallback, this);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      // 2026-07-28: 不再把手动目标硬编码到全局/goal，防止UAV0的RViz目标误触发UAV1规划器。
      waypoint_sub_ = nh.subscribe(manual_goal_topic_, 1, &DiffReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // 2026-07-07: 预设航点模式额外接受 RViz /goal 作为“开始执行”的触发信号，
      // 只解锁 FSM，不改写已配置好的 waypoint 序列，方便起飞后点一次 2D Goal 进入规划阶段。
      waypoint_sub_ = nh.subscribe("/goal", 1, &DiffReplanFSM::triggerCallback, this);
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &DiffReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else if (target_type_ == TARGET_TYPE::SEARCH_TARGET)
    {
      // 2026-07-08: 搜索模式沿用 RViz /goal 仅作“允许开始”的触发语义，真实自动子目标改由独立 search_subgoal_topic 输入。
      waypoint_sub_ = nh.subscribe(search_goal_trigger_topic_, 1, &DiffReplanFSM::triggerCallback, this);
      trigger_sub_ = nh.subscribe(search_trigger_topic_, 1, &DiffReplanFSM::triggerCallback, this);
      subgoal_sub_ = nh.subscribe(search_subgoal_topic_, 1, &DiffReplanFSM::searchSubgoalCallback, this);
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void DiffReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      // 2026-07-28: UAV1由FUEL离散目标驱动时，前机不会广播Diff轨迹；关闭门槛后应立即基于自身地图规划。
      if (!require_pre_agent_trajectory_ || planner_manager_->pp_.drone_id <= 0 ||
          (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (!mondify_final_goal_ && planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
        {
          ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
          need_hover_stop_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
        }
        else
        {
          // 2026-07-28: 每个10ms FSM周期只做一次规划尝试，避免单回调内十连试阻塞心跳和新目标回调。
          bool success = planFromGlobalTraj(1);
          if (success)
          {
            replan_fail_count_ = 0;
            publishPlanningStatus("TRAJECTORY_PUBLISHED");  // 2026-07-28: 首轨迹生成成功反馈。
            changeFSMExecState(EXEC_TRAJ, "FSM");
          }
          else
          {
            ROS_WARN("Failed to generate the first trajectory, keep trying");
            replan_fail_count_++;
            if (exploration_goal_active_ && replan_fail_count_ == 3)
              publishExplorationStatus(
                  quadrotor_msgs::ExplorationGoalStatus::STATE_PLANNING_FAILED,
                  quadrotor_msgs::ExplorationGoalStatus::REASON_LOCAL_TRAJECTORY_FAILED,
                  "DIFF local trajectory generation failed");
            // 2026-07-28: 三次失败即通知上层切换已验证路线短子目标，避免原目标被机械重发到卡死。
            if (replan_fail_count_ == 3 || replan_fail_count_ % 25 == 0)
              publishPlanningStatus("PLANNING_FAILED");
            changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
          }
        }
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (!mondify_final_goal_ && planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
      {
        ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
        need_hover_stop_ = true;
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
      }
      else
      {
        // 2026-07-28: 全局重规划同样单周期单次尝试，保证FSM能继续发心跳并接收替代目标。
        bool success = planFromGlobalTraj(1);
        if (success)
        {
          replan_fail_count_ = 0;
          publishPlanningStatus("TRAJECTORY_PUBLISHED");  // 2026-07-28: 全局重规划成功反馈。
          changeFSMExecState(EXEC_TRAJ, "FSM");
          flag_escape_emergency_ = true;
        }
        else
        {
          replan_fail_count_++;
          if (exploration_goal_active_ && replan_fail_count_ == 3)
            publishExplorationStatus(
                quadrotor_msgs::ExplorationGoalStatus::STATE_PLANNING_FAILED,
                quadrotor_msgs::ExplorationGoalStatus::REASON_GLOBAL_PATH_FAILED,
                "DIFF global trajectory generation failed");
          // 2026-07-28: 全局轨迹连续失败时显式反馈，而不是让接力管理器无限重发同一点。
          if (replan_fail_count_ == 3 || replan_fail_count_ % 25 == 0)
            publishPlanningStatus("PLANNING_FAILED");
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        if (exploration_goal_active_ && replan_fail_count_ == 10)
          publishExplorationStatus(
              quadrotor_msgs::ExplorationGoalStatus::STATE_PLANNING_FAILED,
              quadrotor_msgs::ExplorationGoalStatus::REASON_REPLAN_FAILED,
              "DIFF local replan failed");
        // 2026-07-28: 局部重规划持续失败同样触发安全子目标恢复。
        if (replan_fail_count_ == 10 || replan_fail_count_ % 50 == 0)
          publishPlanningStatus("PLANNING_FAILED");
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);
      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

      if (exploration_goal_active_ && exploration_goal_executing_)
      {
        updateExplorationArrival();
        if (!exploration_goal_active_ || exec_state_ != EXEC_TRAJ)
          break;
        if (ros::Time::now().toSec() > exploration_execution_deadline_)
        {
          publishExplorationStatus(
              quadrotor_msgs::ExplorationGoalStatus::STATE_ABORTED,
              quadrotor_msgs::ExplorationGoalStatus::REASON_TRACKING_TIMEOUT,
              "trajectory ended without reaching the exploration viewpoint");
          clearExplorationGoal(true);
          have_target_ = false;
          need_hover_stop_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "EXPLORATION_TRACKING_TIMEOUT");
          break;
        }
      }

      const PtsChk_t *chk_ptr = &planner_manager_->traj_.local_traj.pts_chk;
      bool close_to_current_traj_end = (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1) ? chk_ptr->back().back().first - t_cur < emergency_time_ : 0; // In case of empty vector

      if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
      {
        if (!mondify_final_goal_)
        {
          ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
          need_hover_stop_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
        }
        else if (mondifyInCollisionFinalGoal())
        {
          ROS_WARN("Successfully modified final_goal in EXEC_TRAJ !!!");
          changeFSMExecState(REPLAN_TRAJ, "mondify_FSM");
        }
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) // case 2: assign the next waypoint
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_], true);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // case 3: the final waypoint reached
      {
        have_target_ = false;
        // 2026-07-08: SEARCH_TARGET 模式下 RViz 触发语义应在整轮搜索期间持续有效，
        // 不能像单次手动/预设任务那样在到达一个局部子目标后就清掉 trigger，
        // 否则 corridor_search_manager 后续发布的子目标会全部被忽略，表现为卡在通道口不再前进。
        if (target_type_ != TARGET_TYPE::SEARCH_TARGET)
          have_trigger_ = false;
        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          // prepare for next round
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_], true);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) // case 3: time to perform next replan
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // ROS_ERROR("AAAA");
      // 2026-07-07: 首飞阶段和 RViz 2D 触发后的豁免窗口内不做 stuck detect，避免首段轨迹刚启动就进入 EMERGENCY_STOP。
      const double now_sec = ros::Time::now().toSec();
      if (enable_stuck_detect_ && now_sec >= stuck_detect_ignore_until_)
      {
        /* Avoid getting stuck wandering around large obstacles */
        static bool baseline_initialized = false;
        if (touch_the_goal)
        {
          static double last_proj_len = 0.0;
          static Eigen::Vector3d baseline_origin = odom_pos_;
          static Eigen::Vector3d last_goal_when_baseline = final_goal_;
          if (!baseline_initialized || (last_goal_when_baseline - final_goal_).norm() > 0.1)
          {
            baseline_origin = odom_pos_;
            last_goal_when_baseline = final_goal_;
            last_proj_len = 0.0;
            baseline_initialized = true;
          }
          Eigen::Vector3d cur_pos = odom_pos_;
          Eigen::Vector3d global2cur = cur_pos - baseline_origin;
          Eigen::Vector3d proj_pos = projectPointToLineSegment(baseline_origin, final_goal_, cur_pos);
          double proj_len = (proj_pos - baseline_origin).norm();
          if (proj_len - last_proj_len < TARGET_STUCK_THRESH)
          {
            if (now_sec - last_target_change_time_ > TARGET_STUCK_TIME)
            {
              ROS_WARN("Drone stuck! Obstacle too large and near final goal. Emergency stop.");
              need_hover_stop_ = true;
              flag_escape_emergency_ = true;
              changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
            }
          }
          else
          {
            last_proj_len = proj_len;
            last_target_change_time_ = now_sec;
          }

          if (global2cur.norm() > planning_horizen_ * M_SQRT2)
          {
            ROS_WARN("Drone stuck! The drone flew too far out of its way . Emergency stop.");
            need_hover_stop_ = true;
            flag_escape_emergency_ = true;
            changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
          }
        }
        else
        {
          baseline_initialized = false;
          if ((local_target_pt_ - last_local_target_pos_).norm() < TARGET_STUCK_THRESH)
          {
            if (now_sec - last_target_change_time_ > TARGET_STUCK_TIME)
            {
              ROS_WARN("Drone stuck! Obstacle too large. Emergency stop.");
              need_hover_stop_ = true;
              flag_escape_emergency_ = true;
              changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
            }
          }
          else
          {
            last_local_target_pos_ = local_target_pt_;
            last_target_change_time_ = now_sec;
          }
        }
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          last_target_change_time_ = ros::Time::now().toSec();
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          // 2026-07-07: 对非 mandatory stop 的中途急停优先尝试自动恢复规划，避免录像里那种停住后直接丢失当前任务。
          if (have_target_ && have_trigger_ && !mandatory_stop_)
          {
            ROS_INFO("Exiting EMERGENCY_STOP. Resume current target with GEN_NEW_TRAJ.");
            need_hover_stop_ = false;
            last_target_change_time_ = ros::Time::now().toSec();
            stuck_detect_ignore_until_ = last_target_change_time_ + stuck_detect_grace_time_;
            changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_RESUME");
          }
          else
          {
            ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point !!!");
            need_hover_stop_ = false;
            have_target_ = false;
            have_trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
          }
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }
    finishProcess();
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }
  void DiffReplanFSM::finishProcess()
  {
    if (replan_fail_count_ > MAX_REPLAN_FAIL_COUNT)
    {
      ROS_WARN("replan fail too much. Emergency stop.");
      replan_fail_count_ = 0; 
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  void DiffReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void DiffReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    // some warnings
    // 2026-07-28: 仅原生Diff顺序编队需要显示/等待前序轨迹，异构接力不再误报prev traj。
    const bool waiting_pre_agent = require_pre_agent_trajectory_ &&
        planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_;
    if (!have_odom_ || !have_target_ || !have_trigger_ || waiting_pre_agent)
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (waiting_pre_agent)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }

  std::pair<int, DiffReplanFSM::FSM_EXEC_STATE> DiffReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void DiffReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    const double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || exec_state_ ==  EMERGENCY_STOP || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    double t_temp = t_cur; // t_temp will be changed in the next function!
    int i_start = info->traj.locatePieceIdx(t_temp);

    if (i_start >= (int)pts_chk.size())
    {
      return;
    }
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;
            if (dist < allowed_dist)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool DiffReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);
    return true;
  }

  bool DiffReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (mondify_final_goal_ && mondifyInCollisionFinalGoal()) 
    {
      ROS_WARN("Successfully modified final_goal in callReboundReplan !!!");
    }
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      if (exploration_goal_active_)
      {
        std::string trajectory_error;
        if (!validateExplorationTrajectory(trajectory_error))
        {
          ROS_ERROR("Reject DIFF exploration trajectory: %s", trajectory_error.c_str());
          plan_success = false;
        }
      }
    }

    if (plan_success)
    {
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
      if (exploration_goal_active_)
      {
        exploration_goal_executing_ = true;
        exploration_execution_deadline_ = ros::Time::now().toSec() +
            planner_manager_->traj_.local_traj.duration +
            exploration_tracking_timeout_;
        publishExplorationStatus(
            quadrotor_msgs::ExplorationGoalStatus::STATE_EXECUTING,
            quadrotor_msgs::ExplorationGoalStatus::REASON_NONE,
            "DIFF trajectory published",
            static_cast<uint32_t>(planner_manager_->traj_.local_traj.traj_id));
      }
    }

    return plan_success;
  }

  bool DiffReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    // 2026-07-28: 随机初始化由实例参数控制；UAV1失败时交给接力层换短目标，不进入曾导致节点失去心跳的阻塞分支。
    const bool flag_random_poly_init =
        enable_random_global_init_ && timesOfConsecutiveStateCalls().first > 1;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool DiffReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

    bool DiffReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan)
  {
    if (!have_odom_)
    {
      ROS_WARN("Reject goal before receiving odometry.");
      publishPlanningStatus("GOAL_REJECTED_NO_ODOM");
      return false;
    }
    if (!odom_pos_.allFinite() || !odom_vel_.allFinite() || !next_wp.allFinite())
    {
      ROS_ERROR_STREAM("Reject non-finite planning input: start="
                       << odom_pos_.transpose() << " velocity="
                       << odom_vel_.transpose() << " goal="
                       << next_wp.transpose());
      publishPlanningStatus("GOAL_REJECTED_NONFINITE");
      return false;
    }

    const int start_occupancy =
        planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
    const int goal_occupancy =
        planner_manager_->grid_map_->getInflateOccupancy(next_wp);
    if (start_occupancy == -1)
    {
      ROS_ERROR_STREAM("Reject goal because current odometry is outside the "
                       "Diff safety fence: start=" << odom_pos_.transpose());
      publishPlanningStatus("START_REJECTED_OUTSIDE_MAP");
      return false;
    }
    if (goal_occupancy == -1)
    {
      ROS_WARN_STREAM("Reject goal outside the Diff safety fence: goal="
                      << next_wp.transpose());
      publishPlanningStatus("GOAL_REJECTED_OUTSIDE_MAP");
      return false;
    }

    const double goal_distance = (next_wp - odom_pos_).norm();
    if (!std::isfinite(goal_distance) || goal_distance < min_goal_distance_)
    {
      ROS_WARN_STREAM("Skip near-zero Diff goal: distance=" << goal_distance
                      << " m, start=" << odom_pos_.transpose()
                      << " goal=" << next_wp.transpose());
      publishPlanningStatus("GOAL_SKIPPED_TOO_CLOSE");
      return false;
    }

    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
    if (success)
    {
      final_goal_ = next_wp;
      // 2026-07-07: 预设航点模式在 WAIT_TARGET 里可能等待很久才触发，
      // 这里统一刷新目标变更时间，避免一进入 EXEC_TRAJ 就被 stuck detect 误判为“长期没推进”。
      last_target_change_time_ = ros::Time::now().toSec();
      // 2026-07-07: 每次预设航点成功切换后重新给 stuck detect 一个短豁免窗口，避免切段初期局部目标尚未明显推进就被误判。
      stuck_detect_ignore_until_ = last_target_change_time_ + stuck_detect_grace_time_;
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }
      have_target_ = true;
      have_new_target_ = true;
      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET && flag_2replan && exec_state_ != EMERGENCY_STOP)
      {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(0.5); 
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
          if (ros::Time::now() - start_time > timeout)
          {
            ROS_WARN("Timeout waiting for state to change to EXEC_TRAJ.");
            return false; 
          }
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      else if(exec_state_ == EMERGENCY_STOP)
      {
        return true;
      }
      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
       visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
    return success;
  }

  bool DiffReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      Eigen::Vector3d orig_goal = final_goal_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step)
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          for (int i = 6; i > 0; i--)
          {
            if (t - i * t_step > 0)
            {
              Eigen::Vector3d pt_tmp = planner_manager_->traj_.global_traj.traj.getPos(t - i * t_step);
              if (!planner_manager_->grid_map_->getInflateOccupancy(pt_tmp))
              {
                pt = pt_tmp;
                break;
              }
            }
          }
          if (planNextWaypoint(pt, false)) // final_goal_=pt inside if success
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }

        if (t <= t_step)
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;
  }

  void DiffReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (planner_manager_->grid_map_->getInflateOccupancy(end_wp) == -1)
    {
      ROS_WARN("The goal is outside the safe fence, ignore this goal!");
      publishPlanningStatus("GOAL_REJECTED_OUTSIDE_MAP");  // 2026-07-28: 区分目标接收与目标可规划。
      return;
    }

    // UAV1 上层会在规划响应丢失时重发同一接力目标。执行中的重复请求只补发成功状态，
    // 不重建全局轨迹；WAIT_TARGET/EMERGENCY_STOP 时仍允许原位置重新规划以恢复失活轨迹。
    if (manual_goal_update_distance_ > 0.0 && have_target_ &&
        exec_state_ != WAIT_TARGET && exec_state_ != EMERGENCY_STOP)
    {
      const double goal_delta = (end_wp - final_goal_).norm();
      if (goal_delta < manual_goal_update_distance_)
      {
        ROS_INFO_THROTTLE(1.0,
                          "Acknowledge duplicate manual goal without replanning: delta=%.3f m, threshold=%.3f m.",
                          goal_delta, manual_goal_update_distance_);
        publishPlanningStatus("TRAJECTORY_PUBLISHED");
        return;
      }
    }

    ROS_INFO("Received goal: %f, %f, %f", end_wp(0), end_wp(1), end_wp(2));
    if (planNextWaypoint(end_wp, true))
    {
      last_target_change_time_ = ros::Time::now().toSec();
      have_trigger_ = true;
    }
  }

  void DiffReplanFSM::publishPlanningStatus(const std::string &status)
  {
    // 2026-07-28: 使用String保持现有消息依赖不变；成功时附带Diff实际接受的
    // final_goal，避免上层仍等待已被占据的原始接力点而永久卡住。
    std_msgs::String msg;
    if (status == "TRAJECTORY_PUBLISHED")
    {
      std::ostringstream stream;
      stream << status << " " << final_goal_.x() << " " << final_goal_.y() << " "
             << final_goal_.z();
      msg.data = stream.str();
    }
    else
      msg.data = status;
    planning_status_pub_.publish(msg);
  }

  double DiffReplanFSM::odometryYaw() const
  {
    return odom_yaw_;
  }

  bool DiffReplanFSM::explorationGoalPayloadEqual(
      const quadrotor_msgs::ExplorationGoal &lhs,
      const quadrotor_msgs::ExplorationGoal &rhs) const
  {
    if (lhs.session_id != rhs.session_id || lhs.goal_id != rhs.goal_id ||
        lhs.target_type != rhs.target_type || lhs.motion_type != rhs.motion_type ||
        lhs.enforce_yaw != rhs.enforce_yaw ||
        lhs.header.frame_id != rhs.header.frame_id)
      return false;
    if (!samePoint(lhs.target_pose.position, rhs.target_pose.position) ||
        !sameQuaternion(lhs.target_pose.orientation, rhs.target_pose.orientation))
      return false;

    const auto &a = lhs.motion_constraint;
    const auto &b = rhs.motion_constraint;
    if (a.enabled != b.enabled ||
        !sameScalar(a.exploration_initial_yaw, b.exploration_initial_yaw) ||
        !sameScalar(a.entry_inside_yaw, b.entry_inside_yaw) ||
        a.entry_plane_active != b.entry_plane_active ||
        !samePoint(a.entry_origin, b.entry_origin) ||
        !sameVector(a.entry_direction, b.entry_direction) ||
        !sameScalar(a.entry_back_margin, b.entry_back_margin) ||
        !samePoint(a.segment_origin, b.segment_origin) ||
        !sameVector(a.segment_direction, b.segment_direction) ||
        !sameScalar(a.segment_high_water, b.segment_high_water) ||
        !sameScalar(a.segment_regression_margin, b.segment_regression_margin) ||
        a.transition_active != b.transition_active ||
        !samePoint(a.transition_anchor, b.transition_anchor) ||
        !sameVector(a.transition_incoming_direction, b.transition_incoming_direction) ||
        !sameVector(a.transition_outgoing_direction, b.transition_outgoing_direction) ||
        !sameScalar(a.transition_old_high_water, b.transition_old_high_water) ||
        !sameScalar(a.transition_regression_margin, b.transition_regression_margin) ||
        !sameScalar(a.transition_radius, b.transition_radius) ||
        !sameScalar(a.transition_corridor_half_width, b.transition_corridor_half_width) ||
        a.completed_gates.size() != b.completed_gates.size())
      return false;
    for (size_t i = 0; i < a.completed_gates.size(); ++i)
    {
      const auto &ga = a.completed_gates[i];
      const auto &gb = b.completed_gates[i];
      if (ga.sequence != gb.sequence || !samePoint(ga.center, gb.center) ||
          !sameVector(ga.normal, gb.normal) ||
          !sameScalar(ga.left_extent, gb.left_extent) ||
          !sameScalar(ga.right_extent, gb.right_extent) ||
          !sameScalar(ga.thickness, gb.thickness))
        return false;
    }
    return true;
  }

  bool DiffReplanFSM::convertExplorationConstraint(
      const quadrotor_msgs::ExplorationMotionConstraint &msg,
      exploration_constraint::State &state, std::string &error) const
  {
    state = exploration_constraint::State();
    state.enabled = msg.enabled;
    state.exploration_initial_yaw = msg.exploration_initial_yaw;
    state.entry_inside_yaw = msg.entry_inside_yaw;
    state.entry_plane_active = msg.entry_plane_active;
    state.entry_origin = Eigen::Vector2d(msg.entry_origin.x, msg.entry_origin.y);
    state.entry_direction = Eigen::Vector2d(msg.entry_direction.x, msg.entry_direction.y);
    state.entry_back_margin = msg.entry_back_margin;
    state.segment_origin = Eigen::Vector2d(msg.segment_origin.x, msg.segment_origin.y);
    state.segment_direction = Eigen::Vector2d(msg.segment_direction.x, msg.segment_direction.y);
    state.segment_high_water = msg.segment_high_water;
    state.segment_regression_margin = msg.segment_regression_margin;
    state.transition_active = msg.transition_active;
    state.transition_anchor = Eigen::Vector2d(msg.transition_anchor.x, msg.transition_anchor.y);
    state.transition_incoming_direction =
        Eigen::Vector2d(msg.transition_incoming_direction.x,
                        msg.transition_incoming_direction.y);
    state.transition_outgoing_direction =
        Eigen::Vector2d(msg.transition_outgoing_direction.x,
                        msg.transition_outgoing_direction.y);
    state.transition_old_high_water = msg.transition_old_high_water;
    state.transition_regression_margin = msg.transition_regression_margin;
    state.transition_radius = msg.transition_radius;
    state.transition_corridor_half_width = msg.transition_corridor_half_width;
    for (const auto &gate_msg : msg.completed_gates)
    {
      exploration_constraint::Gate gate;
      gate.sequence = gate_msg.sequence;
      gate.center = Eigen::Vector2d(gate_msg.center.x, gate_msg.center.y);
      gate.normal = Eigen::Vector2d(gate_msg.normal.x, gate_msg.normal.y);
      gate.left_extent = gate_msg.left_extent;
      gate.right_extent = gate_msg.right_extent;
      gate.thickness = gate_msg.thickness;
      state.completed_gates.push_back(gate);
    }
    if (!exploration_constraint::validState(state))
    {
      error = "invalid or non-finite exploration motion constraint";
      return false;
    }
    if (state.enabled && state.completed_gates.size() > 128)
    {
      error = "too many completed exploration gates";
      return false;
    }
    return true;
  }

  void DiffReplanFSM::publishExplorationStatusFor(
      const quadrotor_msgs::ExplorationGoal &goal, uint8_t state,
      uint8_t reason, const std::string &detail)
  {
    quadrotor_msgs::ExplorationGoalStatus status;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = goal.header.frame_id;
    status.session_id = goal.session_id;
    status.goal_id = goal.goal_id;
    status.source = quadrotor_msgs::ExplorationGoalStatus::SOURCE_FUEL_EXPLORATION;
    status.state = state;
    status.reason = reason;
    status.trajectory_id = planner_manager_ ?
        static_cast<uint32_t>(planner_manager_->traj_.local_traj.traj_id) : 0U;
    status.requested_pose = goal.target_pose;
    status.accepted_pose = exploration_accepted_pose_;
    status.detail = detail;
    last_exploration_status_ = status;
    exploration_status_pub_.publish(status);
  }

  void DiffReplanFSM::publishExplorationStatus(uint8_t state, uint8_t reason,
                                                const std::string &detail,
                                                uint32_t trajectory_id)
  {
    if (!exploration_goal_active_ && !have_last_exploration_goal_)
      return;
    quadrotor_msgs::ExplorationGoalStatus status = last_exploration_status_;
    status.header.stamp = ros::Time::now();
    status.session_id = exploration_session_id_;
    status.goal_id = exploration_goal_id_;
    status.source = quadrotor_msgs::ExplorationGoalStatus::SOURCE_FUEL_EXPLORATION;
    status.state = state;
    status.reason = reason;
    status.trajectory_id = trajectory_id != 0 ? trajectory_id :
        static_cast<uint32_t>(planner_manager_->traj_.local_traj.traj_id);
    status.requested_pose = exploration_requested_pose_;
    status.accepted_pose = exploration_accepted_pose_;
    status.detail = detail;
    last_exploration_status_ = status;
    exploration_status_pub_.publish(status);
  }

  void DiffReplanFSM::clearExplorationGoal(bool clear_constraint)
  {
    exploration_goal_active_ = false;
    exploration_goal_executing_ = false;
    exploration_dwell_active_ = false;
    exploration_execution_deadline_ = std::numeric_limits<double>::infinity();
    active_target_source_ = SOURCE_NONE;
    if (clear_constraint)
      planner_manager_->grid_map_->clearExplorationMotionConstraint();
  }

  void DiffReplanFSM::preemptExplorationForLanding(const std::string &detail)
  {
    landing_phase_latched_ = true;
    active_target_source_ = SOURCE_LANDING;
    if (exploration_goal_active_)
      publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_PREEMPTED,
                               quadrotor_msgs::ExplorationGoalStatus::REASON_LANDING_HANDOFF,
                               detail);
    clearExplorationGoal(true);
    have_target_ = false;
    have_new_target_ = false;
    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
        exec_state_ == GEN_NEW_TRAJ || exec_state_ == SEQUENTIAL_START)
    {
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "LANDING_HANDOFF");
    }
  }

  void DiffReplanFSM::updateExplorationArrival()
  {
    if (!exploration_goal_active_ || !exploration_goal_executing_ || !have_odom_)
      return;
    const Eigen::Vector3d target(exploration_accepted_pose_.position.x,
                                 exploration_accepted_pose_.position.y,
                                 exploration_accepted_pose_.position.z);
    const Eigen::Vector3d delta = odom_pos_ - target;
    bool reached = delta.head<2>().norm() <= exploration_arrival_xy_ &&
                   std::fabs(delta.z()) <= exploration_arrival_z_ &&
                   odom_vel_.norm() <= exploration_arrival_velocity_;
    if (exploration_goal_enforce_yaw_)
    {
      double yaw_error = std::atan2(std::sin(odometryYaw() - exploration_goal_yaw_),
                                    std::cos(odometryYaw() - exploration_goal_yaw_));
      reached = reached && std::fabs(yaw_error) <= exploration_arrival_yaw_;
    }
    const double now = ros::Time::now().toSec();
    if (reached)
    {
      if (!exploration_dwell_active_)
      {
        exploration_dwell_active_ = true;
        exploration_dwell_start_ = now;
      }
      if (now - exploration_dwell_start_ >= exploration_arrival_dwell_)
      {
        publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_REACHED,
                                 quadrotor_msgs::ExplorationGoalStatus::REASON_NONE,
                                 "viewpoint reached and settled");
        clearExplorationGoal(true);
        have_target_ = false;
        if (target_type_ != TARGET_TYPE::SEARCH_TARGET)
          have_trigger_ = false;
        changeFSMExecState(WAIT_TARGET, "EXPLORATION_REACHED");
      }
    }
    else
      exploration_dwell_active_ = false;
  }

  bool DiffReplanFSM::validateExplorationTrajectory(std::string &error) const
  {
    if (!exploration_goal_active_ || !planner_manager_->traj_.local_traj.traj_id)
      return true;
    const auto &traj = planner_manager_->traj_.local_traj.traj;
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0)
    {
      error = "trajectory has no positive duration";
      return false;
    }
    const double dt = std::max(1e-3, exploration_trajectory_sample_time_);
    const Eigen::Vector3d first = traj.getPos(0.0);
    Eigen::Vector3d previous = first;
    if (!planner_manager_->grid_map_->isExplorationSegmentAllowed(first, first))
    {
      error = "trajectory start violates exploration constraint";
      return false;
    }
    for (double t = dt; t <= duration + 1e-9; t += dt)
    {
      const double sample_t = std::min(duration, t);
      const Eigen::Vector3d current = traj.getPos(sample_t);
      const double length = (current - previous).norm();
      const int subdivisions = std::max(1, static_cast<int>(std::ceil(
          length / exploration_trajectory_sample_distance_)));
      for (int i = 1; i <= subdivisions; ++i)
      {
        const Eigen::Vector3d point = previous +
            (current - previous) * (static_cast<double>(i) / subdivisions);
        const Eigen::Vector3d prior = previous +
            (current - previous) * (static_cast<double>(i - 1) / subdivisions);
        if (planner_manager_->grid_map_->getInflateOccupancy(point) != 0 ||
            !planner_manager_->grid_map_->isExplorationSegmentAllowed(prior, point))
        {
          error = "trajectory sample violates map or exploration gate";
          return false;
        }
      }
      previous = current;
      if (sample_t >= duration)
        break;
    }
    return true;
  }

  void DiffReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_], true);
  }

  void DiffReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void DiffReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    const auto &q = msg->pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    have_odom_ = true;
    if (planner_manager_ && planner_manager_->grid_map_)
      planner_manager_->grid_map_->advanceExplorationHighWater(odom_pos_);
    updateExplorationArrival();
  }

  void DiffReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    // 2026-07-07: 外部触发（/traj_start_trigger 或 RViz /goal）后临时忽略 stuck detect，给首段轨迹和控制器起步留时间。
    stuck_detect_ignore_until_ = ros::Time::now().toSec() + stuck_detect_grace_time_;
    // 2026-07-07: 统一记录触发来源，便于区分是 /traj_start_trigger 还是 RViz /goal 把 flight_type=2 解锁。
    cout << "Triggered! frame=" << msg->header.frame_id
         << " pos=(" << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << ")"
         << endl;
  }

  void DiffReplanFSM::searchSubgoalCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (planner_manager_->grid_map_->getInflateOccupancy(end_wp) == -1)
    {
      ROS_WARN("The search subgoal is outside the safe fence, ignore this goal!");
      return;
    }

    // 2026-07-08: 搜索子目标只负责“去哪”，起飞许可仍由 triggerCallback 控制；未触发前先缓存/忽略自动子目标，避免地面阶段误规划。
    if (!have_trigger_)
    {
      ROS_WARN_THROTTLE(1.0, "Search subgoal received before trigger, ignore until RViz trigger arrives.");
      return;
    }

    // landing_diff_search_manager 每秒重发当前目标用于保活。已成功规划的目标在阈值内没有
    // 实质变化时直接忽略；不要刷新 last_target_change_time_，否则会掩盖 stuck detection。
    if (have_target_)
    {
      const double goal_delta = (end_wp - final_goal_).norm();
      if (goal_delta < search_subgoal_update_distance_)
      {
        ROS_INFO_THROTTLE(1.0,
                          "Ignore duplicate search subgoal: delta=%.3f m, threshold=%.3f m.",
                          goal_delta, search_subgoal_update_distance_);
        return;
      }
    }

    ROS_INFO("Received search subgoal: %f, %f, %f", end_wp(0), end_wp(1), end_wp(2));
    if (planNextWaypoint(end_wp, true))
    {
      last_target_change_time_ = ros::Time::now().toSec();
    }
  }

  void DiffReplanFSM::explorationTriggerCallback(
      const geometry_msgs::PoseStampedConstPtr &msg)
  {
    (void)msg;
    if (landing_phase_latched_)
      return;
    exploration_triggered_ = true;
    have_trigger_ = true;
    active_target_source_ = SOURCE_FUEL;
    stuck_detect_ignore_until_ = ros::Time::now().toSec() + stuck_detect_grace_time_;
  }

  void DiffReplanFSM::missionStageCallback(const std_msgs::StringConstPtr &msg)
  {
    const std::string stage = msg->data.substr(0, msg->data.find_first_of(" \t\r\n"));
    if (stage == "SEARCH_OUTSIDE_LANDING" || stage == "SEARCH_OUTSIDE_QR" ||
        stage == "APPROACH_LANDING" || stage == "LANDING")
      preemptExplorationForLanding("mission stage changed to " + stage);
  }

  void DiffReplanFSM::explorationCancelCallback(
      const quadrotor_msgs::ExplorationCancelConstPtr &msg)
  {
    if (landing_phase_latched_ || !exploration_goal_active_ ||
        msg->session_id != exploration_session_id_ ||
        msg->goal_id != exploration_goal_id_)
      return;
    publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_CANCELED,
                             quadrotor_msgs::ExplorationGoalStatus::REASON_NONE,
                             msg->reason.empty() ? "cancelled by FUEL" : msg->reason);
    clearExplorationGoal(true);
    have_target_ = false;
    need_hover_stop_ = true;
    flag_escape_emergency_ = true;
    changeFSMExecState(EMERGENCY_STOP, "FUEL_CANCEL");
  }

  void DiffReplanFSM::explorationGoalCallback(
      const quadrotor_msgs::ExplorationGoalConstPtr &msg)
  {
    const auto reject = [&](uint8_t reason, const std::string &detail) {
      publishExplorationStatusFor(*msg,
          quadrotor_msgs::ExplorationGoalStatus::STATE_REJECTED, reason, detail);
    };
    if (have_last_exploration_goal_ && msg->session_id == last_exploration_goal_.session_id &&
        msg->goal_id == last_exploration_goal_.goal_id)
    {
      if (explorationGoalPayloadEqual(*msg, last_exploration_goal_))
      {
        exploration_status_pub_.publish(last_exploration_status_);
        return;
      }
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_ID_CONFLICT,
             "same session and goal id carries different payload");
      return;
    }
    if (landing_phase_latched_)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_LANDING_HANDOFF,
             "exploration is permanently preempted by landing");
      return;
    }
    if ((msg->session_id < highest_exploration_session_id_) ||
        (msg->session_id == highest_exploration_session_id_ &&
         msg->goal_id < highest_exploration_goal_id_))
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_STALE_ID,
             "goal id is older than the accepted exploration sequence");
      return;
    }
    if (!exploration_triggered_)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_NOT_TRIGGERED,
             "exploration trigger has not been latched");
      return;
    }
    if (!have_odom_)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_NO_ODOM,
             "no odometry has been received");
      return;
    }
    const auto &pose = msg->target_pose;
    const bool finite_pose = std::isfinite(pose.position.x) &&
        std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
        std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
        std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
    if (!finite_pose || (msg->motion_type != quadrotor_msgs::ExplorationGoal::MOTION_MOVE &&
                         msg->motion_type != quadrotor_msgs::ExplorationGoal::MOTION_YAW_ONLY) ||
        msg->target_type > quadrotor_msgs::ExplorationGoal::TARGET_TURN_ALIGNMENT)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_INVALID_GOAL,
             "invalid target pose, motion type, or target type");
      return;
    }
    if (msg->enforce_yaw &&
        std::sqrt(pose.orientation.x * pose.orientation.x +
                  pose.orientation.y * pose.orientation.y +
                  pose.orientation.z * pose.orientation.z +
                  pose.orientation.w * pose.orientation.w) < 1e-6)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_INVALID_GOAL,
             "enforced yaw requires a valid quaternion");
      return;
    }

    exploration_constraint::State constraint;
    std::string constraint_error;
    if (!convertExplorationConstraint(msg->motion_constraint, constraint,
                                      constraint_error))
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_MISSION_CONSTRAINT,
             constraint_error);
      return;
    }
    const Eigen::Vector3d target(pose.position.x, pose.position.y, pose.position.z);
    const int occupancy = planner_manager_->grid_map_->getInflateOccupancy(target);
    if (occupancy == -1)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_OUTSIDE_MAP,
             "goal is outside Diff safety fence");
      return;
    }
    if (occupancy != 0)
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_GOAL_OCCUPIED,
             "goal is occupied after inflation");
      return;
    }
    if (constraint.enabled &&
        !exploration_constraint::pointAllowed(constraint, target.head<2>()))
    {
      reject(quadrotor_msgs::ExplorationGoalStatus::REASON_MISSION_CONSTRAINT,
             "goal violates the exploration motion constraint");
      return;
    }
    if (exploration_goal_active_)
      publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_PREEMPTED,
                               quadrotor_msgs::ExplorationGoalStatus::REASON_NONE,
                               "superseded by a newer FUEL exploration goal");

    last_exploration_goal_ = *msg;
    have_last_exploration_goal_ = true;
    exploration_session_id_ = msg->session_id;
    exploration_goal_id_ = msg->goal_id;
    highest_exploration_session_id_ = msg->session_id;
    highest_exploration_goal_id_ = msg->goal_id;
    exploration_requested_pose_ = msg->target_pose;
    exploration_accepted_pose_ = msg->target_pose;
    exploration_goal_enforce_yaw_ = msg->enforce_yaw;
    exploration_goal_yaw_ = std::atan2(
        2.0 * (pose.orientation.w * pose.orientation.z +
               pose.orientation.x * pose.orientation.y),
        1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                      pose.orientation.z * pose.orientation.z));
    exploration_goal_yaw_only_ =
        msg->motion_type == quadrotor_msgs::ExplorationGoal::MOTION_YAW_ONLY;
    exploration_goal_active_ = true;
    exploration_goal_executing_ = false;
    exploration_dwell_active_ = false;
    active_target_source_ = SOURCE_FUEL;
    planner_manager_->grid_map_->setExplorationMotionConstraint(constraint);
    have_trigger_ = true;

    publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_ACCEPTED,
                             quadrotor_msgs::ExplorationGoalStatus::REASON_NONE,
                             "goal accepted; waiting for DIFF trajectory");

    const double distance = (target - odom_pos_).norm();
    if (exploration_goal_yaw_only_ || distance < min_goal_distance_)
    {
      final_goal_ = target;
      have_target_ = true;
      have_new_target_ = false;
      exploration_goal_executing_ = true;
      exploration_execution_deadline_ = ros::Time::now().toSec() +
          exploration_tracking_timeout_;
      publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_EXECUTING,
                               quadrotor_msgs::ExplorationGoalStatus::REASON_NONE,
                               "zero-distance or yaw-only viewpoint held without MINCO");
      updateExplorationArrival();
      return;
    }

    if (!planNextWaypoint(target, true))
    {
      publishExplorationStatus(quadrotor_msgs::ExplorationGoalStatus::STATE_PLANNING_FAILED,
                               quadrotor_msgs::ExplorationGoalStatus::REASON_GLOBAL_PATH_FAILED,
                               "DIFF global path generation failed");
      clearExplorationGoal(true);
      have_target_ = false;
    }
  }

  void DiffReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    if ( msg->start_time.toSec() <= planner_manager_->traj_.swarm_traj[recv_id].start_time ) // This must be called after buffer fill-up
    {
      ROS_WARN("Old traj received, ignored.");
      return;
    }

    /* Parse and store data */

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd cps_chk = MJO.getInitConstraintPoints(5); // K = 5, such accuracy is sufficient
    bool far_away = true;
    for (int i = 0; i < cps_chk.cols(); ++i)
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) // close to me that can not be ignored
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !have_recv_pre_agent_) // Accept a far traj if no previous agent received
    {
      poly_traj::Trajectory trajectory = MJO.getTraj();
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      /* Check Collision */
      if (planner_manager_->checkCollision(recv_id))
      {
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
      }

      /* Check if receive agents have lower drone id */
      if (!have_recv_pre_agent_)
      {
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
        {
          for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
            {
              break;
            }

            have_recv_pre_agent_ = true;
          }
        }
      }
    }
    else
    {
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = -1; // Means this trajectory is invalid
    }
  }

  void DiffReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    MINCO_msg.des_clearance = planner_manager_->getSwarmClearance();
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool DiffReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
  Eigen::Vector3d DiffReplanFSM::projectPointToLineSegment(const Eigen::Vector3d& a,
                                                          const Eigen::Vector3d& b,
                                                          const Eigen::Vector3d& p)
  {
      double t = 0.0;
      Eigen::Vector3d ab = b - a;
      double ab2 = ab.squaredNorm();   
      double ab_norm = ab.norm();
      if (ab2 < 1e-8)                  
      {
        t = 0.0;
        return a;
      }
      t = (p - a).dot(ab) / ab2;       
      if (t < 0.0)                     
      {
        t = 0.0;
        return a;
      }
      else if (t > 1.0)                
      {
        t = 1.0;
        return b;
      }
      return a + t * ab;
  }
} // namespace diff_planner
