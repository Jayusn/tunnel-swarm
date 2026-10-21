
#include <plan_manage/diff_replan_fsm.h>
#include <cmath>
#include <limits>

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
    controller_restart_pending_ = false;
    swing_wait_active_ = false;
    fapp_wait_active_ = false;
    swing_clear_since_ = 0.0;
    swing_wait_obstacle_id_ = 0;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/enable_periodic_replan", enable_periodic_replan_, true);
    periodic_replan_pending_ = false;
    last_periodic_replan_attempt_time_ = 0.0;
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/max_tracking_error", max_tracking_error_, 0.30);
    if (!std::isfinite(max_tracking_error_) || max_tracking_error_ <= 0.0)
    {
      ROS_WARN("fsm/max_tracking_error must be a positive finite value; using 0.30 m.");
      max_tracking_error_ = 0.30;
    }
    // 2026-07-07: 允许按场景单独指定 waypoint 切换距离，避免沿默认经验值在窄通道里提前切段。
    nh.param("fsm/waypoint_switch_dist", no_replan_thresh_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/enable_occupied_recovery", enable_occupied_recovery_, false);
    nh.param("fsm/escape_max_distance", escape_max_distance_, 0.40);
    nh.param("fsm/escape_history_time", escape_history_time_, 1.50);
    nh.param("fsm/escape_speed", escape_speed_, 0.10);
    nh.param("fsm/escape_reach_tolerance", escape_reach_tolerance_, 0.06);
    nh.param("fsm/escape_min_clearance", escape_min_clearance_, 0.20);
    nh.param("fsm/escape_clearance_search_radius", escape_clearance_search_radius_, 0.60);
    nh.param("fsm/escape_max_occupied_prefix", escape_max_occupied_prefix_, 0.20);
    nh.param("fsm/escape_recovery_height", escape_recovery_height_, 0.60);
    nh.param("fsm/escape_max_attempts", escape_max_attempts_, 2);
    nh.param("fsm/history_only_occupied_recovery",
             history_only_occupied_recovery_, false);
    nh.param("fsm/wait_new_target_after_occupied_recovery",
             wait_new_target_after_occupied_recovery_, false);
    if (!std::isfinite(escape_max_distance_) || escape_max_distance_ <= 0.0)
      escape_max_distance_ = 0.40;
    if (!std::isfinite(escape_history_time_) || escape_history_time_ < 0.0)
      escape_history_time_ = 1.50;
    if (!std::isfinite(escape_speed_) || escape_speed_ <= 0.0)
      escape_speed_ = 0.10;
    if (!std::isfinite(escape_reach_tolerance_) || escape_reach_tolerance_ <= 0.0)
      escape_reach_tolerance_ = 0.06;
    if (!std::isfinite(escape_min_clearance_) || escape_min_clearance_ < 0.0)
      escape_min_clearance_ = 0.20;
    if (!std::isfinite(escape_clearance_search_radius_) || escape_clearance_search_radius_ <= 0.0)
      escape_clearance_search_radius_ = 0.60;
    if (!std::isfinite(escape_max_occupied_prefix_) || escape_max_occupied_prefix_ <= 0.0)
      escape_max_occupied_prefix_ = 0.20;
    if (!std::isfinite(escape_recovery_height_) || escape_recovery_height_ <= 0.0)
      escape_recovery_height_ = 0.60;
    escape_max_attempts_ = std::max(1, escape_max_attempts_);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);
    nh.param("fsm/mondify_final_goal", mondify_final_goal_, true);
    nh.param("fsm/preserve_modified_goal_height", preserve_modified_goal_height_, false);
    nh.param("fsm/enable_stuck_detect", enable_stuck_detect_, true);
    // 2026-07-28: 保留原生Diff编队默认行为；异构FUEL->Diff接力由launch显式关闭此前序轨迹门槛。
    nh.param("fsm/require_pre_agent_trajectory", require_pre_agent_trajectory_, true);
    // 2026-07-28: 默认保留Diff原行为；UAV1接力实例关闭随机初始化，避免不可行目标让规划回调阻塞并丢心跳。
    nh.param("fsm/enable_random_global_init", enable_random_global_init_, true);
    nh.param("fsm/enable_random_local_init", enable_random_local_init_, true);
    // 2026-07-07: 允许首飞阶段或外部触发后的短时间内跳过 stuck detect，避免 indoor1 首段轨迹因局部目标变化慢被误判卡死。
    nh.param("fsm/stuck_detect_grace_time", stuck_detect_grace_time_, 6.0);
    // 2026-07-08: 为“RViz 只触发、自动搜索单独下发子目标”模式新增独立子目标话题，避免和 /goal 的触发语义混用。
    nh.param("fsm/search_subgoal_topic", search_subgoal_topic_, std::string("/corridor_search/subgoal"));
    // 2026-07-28: 外部目标话题参数化；双机时后机只接收接力管理器发布的/UAV1/planning/goal。
    nh.param("fsm/manual_goal_topic", manual_goal_topic_, std::string("/goal"));
    nh.param("fsm/enable_swing_obstacle_guard", enable_swing_obstacle_guard_, false);
    nh.param("fsm/swing_obstacle_topic", swing_obstacle_topic_,
             std::string("/UAV1/ldop/dynamic_objects"));
    nh.param("fsm/enable_fapp_dynamic_avoidance", enable_fapp_dynamic_avoidance_, false);
    nh.param("fsm/fapp_obstacle_topic", fapp_obstacle_topic_,
             std::string("/UAV1/fapp/states"));
    nh.param("fsm/swing_prediction_horizon", swing_prediction_horizon_, 5.0);
    nh.param("fsm/swing_prediction_dt", swing_prediction_dt_, 0.05);
    nh.param("fsm/swing_release_clear_time", swing_release_clear_time_, 0.35);
    nh.param("fsm/swing_release_speed", swing_release_speed_, 0.20);
    if (!std::isfinite(swing_prediction_horizon_) || swing_prediction_horizon_ <= 0.0)
      swing_prediction_horizon_ = 5.0;
    if (!std::isfinite(swing_prediction_dt_) || swing_prediction_dt_ <= 0.0)
      swing_prediction_dt_ = 0.05;
    swing_prediction_dt_ = std::min(swing_prediction_dt_, swing_prediction_horizon_);
    if (!std::isfinite(swing_release_clear_time_) || swing_release_clear_time_ < 0.0)
      swing_release_clear_time_ = 0.35;
    if (!std::isfinite(swing_release_speed_) || swing_release_speed_ <= 0.0)
      swing_release_speed_ = 0.20;

    SwingObstacleGuard::Config swing_config;
    nh.param("fsm/swing_corridor_width", swing_config.corridor_width, 1.5);
    nh.param("fsm/swing_corridor_boundary_margin",
             swing_config.corridor_boundary_margin, 0.02);
    nh.param("fsm/swing_vehicle_radius", swing_config.vehicle_radius, 0.25);
    nh.param("fsm/swing_vehicle_half_height", swing_config.vehicle_half_height, 0.15);
    nh.param("fsm/swing_horizontal_margin", swing_config.horizontal_margin, 0.10);
    nh.param("fsm/swing_vertical_margin", swing_config.vertical_margin, 0.10);
    nh.param("fsm/swing_observation_retention", swing_config.observation_retention,
             0.80);
    nh.param("fsm/swing_underpass_learning_time",
             swing_config.underpass_learning_time, 3.0);
    nh.param("fsm/swing_velocity_deadband", swing_config.velocity_deadband, 0.08);
    nh.param("fsm/swing_minimum_speed", swing_config.minimum_swing_speed, 0.25);
    nh.param("fsm/swing_enable_harmonic_prediction",
             swing_config.enable_harmonic_prediction, false);
    nh.param("fsm/swing_harmonic_min_samples",
             swing_config.harmonic_min_samples, 12);
    nh.param("fsm/swing_harmonic_max_history_samples",
             swing_config.harmonic_max_history_samples, 80);
    nh.param("fsm/swing_harmonic_min_motion_span",
             swing_config.harmonic_min_motion_span, 0.35);
    nh.param("fsm/swing_harmonic_reversal_velocity_epsilon",
             swing_config.harmonic_reversal_velocity_epsilon, 0.04);
    nh.param("fsm/swing_harmonic_min_half_period",
             swing_config.harmonic_min_half_period, 0.3);
    nh.param("fsm/swing_harmonic_max_half_period",
             swing_config.harmonic_max_half_period, 4.0);
    swing_obstacle_guard_.setConfig(swing_config);

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
    occupied_recovery_target_.setZero();
    occupied_recovery_deadline_ = 0.0;
    last_free_history_record_time_ = 0.0;
    last_escape_path_check_time_ = 0.0;
    last_escape_target_search_time_ = 0.0;
    occupied_recovery_attempt_count_ = 0;
    occupied_recovery_active_ = false;
    occupied_recovery_episode_ = false;
    occupied_recovery_from_history_ = false;
    occupied_recovery_vertical_ = false;
    occupied_recovery_failure_reported_ = false;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &DiffReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &DiffReplanFSM::checkCollisionCallback, this);

    initOdometry(nh);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &DiffReplanFSM::mandatoryStopCallback, this);
    planning_restart_sub_ = nh.subscribe("/planning_restart_trigger", 1,
                                         &DiffReplanFSM::planningRestartCallback, this);
    if (enable_swing_obstacle_guard_)
    {
      swing_obstacle_sub_ = nh.subscribe(swing_obstacle_topic_, 10,
                                         &DiffReplanFSM::dynamicObjectsCallback, this,
                                         ros::TransportHints().tcpNoDelay());
      ROS_INFO("Swing obstacle guard enabled: topic=%s, corridor=%.2fm, horizon=%.2fs, harmonic=%s.",
               swing_obstacle_topic_.c_str(),
               swing_obstacle_guard_.config().corridor_width,
               swing_prediction_horizon_,
               swing_obstacle_guard_.config().enable_harmonic_prediction ? "on" : "off");
    }
    if (enable_fapp_dynamic_avoidance_)
    {
      fapp_obstacle_sub_ = nh.subscribe(fapp_obstacle_topic_, 10,
                                        &DiffReplanFSM::fappObjectsCallback, this,
                                        ros::TransportHints().tcpNoDelay());
      ROS_INFO("FAPP dynamic avoidance enabled: topic=%s.",
               fapp_obstacle_topic_.c_str());
    }

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &DiffReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    tracking_replan_sub_ = nh.subscribe("planning/tracking_replan", 10,
        &DiffReplanFSM::trackingReplanCallback, this, ros::TransportHints().tcpNoDelay());
    tracking_result_pub_ = nh.advertise<traj_utils::TrackingReplanResult>("planning/tracking_result", 10);
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    // 2026-07-28: 后机接力不能仅凭目标已发布判断Diff正在执行，增加锁存状态反馈。
    planning_status_pub_ = nh.advertise<std_msgs::String>("planning/status", 10, true);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      // 2026-07-28: 不再把手动目标硬编码到全局/goal，防止UAV0的RViz目标误触发UAV1规划器。
      waypoint_sub_ = nh.subscribe(manual_goal_topic_, 1, &DiffReplanFSM::waypointCallback, this);
      entry_waypoint_sub_ = nh.subscribe(manual_goal_topic_ + "/entry", 1,
                                         &DiffReplanFSM::entryWaypointCallback, this);
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
        refreshOdometry();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else if (target_type_ == TARGET_TYPE::SEARCH_TARGET)
    {
      // 2026-07-08: 搜索模式沿用 RViz /goal 仅作“允许开始”的触发语义，真实自动子目标改由独立 search_subgoal_topic 输入。
      waypoint_sub_ = nh.subscribe("/goal", 1, &DiffReplanFSM::triggerCallback, this);
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &DiffReplanFSM::triggerCallback, this);
      subgoal_sub_ = nh.subscribe(search_subgoal_topic_, 1, &DiffReplanFSM::searchSubgoalCallback, this);
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void DiffReplanFSM::trackingReplanCallback(const traj_utils::TrackingReplan::ConstPtr& request)
  {
    if (request->header.stamp.isZero() ||
        request->header.stamp <= tracking_request_.header.stamp ||
        !have_target_ || !have_trigger_ || mandatory_stop_) return;
    tracking_request_ = *request;
    tracking_replan_pending_ = true;
    tracking_retry_after_ = 0.0;
    periodic_replan_pending_ = false;
    ROS_WARN("[跟踪重规划] 收到控制器 request=%u, stop=%d；从最新里程计重算。",
             request->header.seq, request->stop_current);
  }

  void DiffReplanFSM::publishTrackingResult(const traj_utils::PolyTraj& trajectory)
  {
    if (!tracking_replan_pending_) return;
    traj_utils::TrackingReplanResult result;
    result.header = tracking_request_.header;
    result.trajectory = trajectory;
    tracking_result_pub_.publish(result);
    tracking_replan_pending_ = false;
  }

  void DiffReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    refreshOdometry();
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    const double now_sec = ros::Time::now().toSec();
    if (have_odom_)
      updateFreeOdomHistory(now_sec);

    // Actual occupied odometry has priority over every normal planning state.
    // Enter recovery before collision replanning can feed the occupied start
    // back into the optimizer repeatedly.
    const bool normal_planning_state =
        exec_state_ == SEQUENTIAL_START || exec_state_ == GEN_NEW_TRAJ ||
        exec_state_ == REPLAN_TRAJ || exec_state_ == EXEC_TRAJ;
    // 接力任务的末点悬停仍受地图更新影响；首次外部任务前及强停期间不从等待态启动。
    const bool waiting_after_manual_goal = exec_state_ == WAIT_TARGET &&
        wait_new_target_after_occupied_recovery_ &&
        target_type_ == TARGET_TYPE::MANUAL_TARGET &&
        active_external_goal_stamp_ns_ != 0ULL &&
        planner_manager_->traj_.local_traj.traj_id > 0 && !mandatory_stop_;
    if (enable_occupied_recovery_ && have_odom_ &&
        (normal_planning_state || waiting_after_manual_goal))
    {
      const int current_occ =
          planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
      if (current_occ != 0)
      {
        ROS_ERROR("[局部脱障] 实际里程计位置进入膨胀占据区：occ=%d, "
                  "pos=(%.3f, %.3f, %.3f)。停止重规划并沿历史轨迹后退。",
                  current_occ, odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
        need_hover_stop_ = true;
        flag_escape_emergency_ = true;
        occupied_recovery_active_ = false;
        beginOccupiedRecoveryEpisode();
        changeFSMExecState(EMERGENCY_STOP, "OCCUPIED_START");
      }
    }

    if (tracking_replan_pending_ && have_target_ && normal_planning_state &&
        exec_state_ != EMERGENCY_STOP &&
        planner_manager_->grid_map_->getInflateOccupancy(final_goal_) != 0)
    {
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "TRACKING_TARGET_OCCUPIED");
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    if (tracking_replan_pending_ && have_target_ && have_trigger_ && !mandatory_stop_ &&
        (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ || exec_state_ == GEN_NEW_TRAJ))
    {
      if (now_sec >= tracking_retry_after_)
      {
        tracking_retry_after_ = now_sec + std::max(0.1, replan_thresh_);
        // Occupied goals must not be retried as if they were executable.
        if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_) == 0 &&
            planFromGlobalTraj(1)) {
          periodic_replan_pending_ = false;
          changeFSMExecState(EXEC_TRAJ, "CONTROLLER_TRACKING_REPLAN");
        }
      }
      finishProcess();
      exec_timer_.start();
      return;
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
            SwingCollisionResult collision;
            if (planner_manager_->dynamicCandidatesFailed(&collision))
            {
              startSwingWait(collision, true);
              break;
            }
            ROS_WARN("Failed to generate the first trajectory, keep trying");
            replan_fail_count_++;
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
          controller_restart_pending_ = false;
          replan_fail_count_ = 0;
          publishPlanningStatus("TRAJECTORY_PUBLISHED");  // 2026-07-28: 全局重规划成功反馈。
          changeFSMExecState(EXEC_TRAJ, "FSM");
          flag_escape_emergency_ = true;
        }
        else
        {
          SwingCollisionResult collision;
          if (planner_manager_->dynamicCandidatesFailed(&collision))
          {
            startSwingWait(collision, true);
            break;
          }
          replan_fail_count_++;
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
      if (handleManualTrajectoryEnd(now_sec))
        break;
      const bool periodic_replan = periodic_replan_pending_;
      if (planFromLocalTraj(1, periodic_replan))
      {
        periodic_replan_pending_ = false;
        replan_fail_count_ = 0;
        if (external_goal_modified_)
          publishPlanningStatus("TRAJECTORY_PUBLISHED");
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else if (periodic_replan)
      {
        periodic_replan_pending_ = false;
        ROS_WARN("周期重规划未生成可接受候选，继续执行当前轨迹。");
        changeFSMExecState(EXEC_TRAJ, "PERIODIC_REPLAN");
      }
      else
      {
        SwingCollisionResult collision;
        if (planner_manager_->dynamicCandidatesFailed(&collision))
        {
          startSwingWait(collision, true);
          break;
        }
        replan_fail_count_++;
        // 2026-07-28: 局部重规划持续失败同样触发安全子目标恢复。
        if (replan_fail_count_ == 10 || replan_fail_count_ % 50 == 0)
          publishPlanningStatus("PLANNING_FAILED");
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      const bool holding_manual_endpoint = handleManualTrajectoryEnd(now_sec);
      if (holding_manual_endpoint && exec_state_ != EXEC_TRAJ)
        break;
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);
      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

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
      else if (target_type_ != TARGET_TYPE::MANUAL_TARGET &&
               (t_cur > info->duration - 1e-2) && touch_the_goal) // case 3: the final waypoint reached
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
      else if (!holding_manual_endpoint && enable_periodic_replan_ && t_cur > replan_thresh_ &&
               now_sec - last_periodic_replan_attempt_time_ > replan_thresh_)
      {
        periodic_replan_pending_ = true;
        last_periodic_replan_attempt_time_ = now_sec;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      else if (!touch_the_goal && close_to_current_traj_end) // continue the local trajectory horizon
      {
        periodic_replan_pending_ = false;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // ROS_ERROR("AAAA");
      // 2026-07-07: 首飞阶段和 RViz 2D 触发后的豁免窗口内不做 stuck detect，避免首段轨迹刚启动就进入 EMERGENCY_STOP。
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
            // 距离基准与计时基准同步重置，避免把等待起飞时间计入卡住检测。
            last_target_change_time_ = now_sec;
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
        occupied_recovery_active_ = false;
        callEmergencyStop(odom_pos_);
      }
      else if (enable_fail_safe_ || depth_timeout_emergency_)
      {
        if (depth_timeout_emergency_)
        {
          if (planner_manager_->grid_map_->getOdomDepthTimeout())
          {
            ROS_WARN_THROTTLE(0.5, "Depth still unavailable; keep the short safety stop and wait for map recovery.");
            break;
          }
          depth_timeout_emergency_ = false;
          need_hover_stop_ = false;
          replan_fail_count_ = 0;
          last_target_change_time_ = now_sec;
          stuck_detect_ignore_until_ = now_sec + stuck_detect_grace_time_;
          ROS_WARN("Depth recovered; automatically resume the current target from measured odometry.");
          changeFSMExecState(GEN_NEW_TRAJ, "DEPTH_RECOVERED");
          break;
        }
        if (swing_wait_active_)
        {
          if (fapp_wait_active_)
          {
            if (mandatory_stop_)
              break;
            if (swing_clear_since_ > 0.0 &&
                now_sec - swing_clear_since_ < swing_release_clear_time_)
              break;
            swing_clear_since_ = now_sec;
            // Resume only after a real trajectory from odometry passes the
            // manager's static and dynamic checks, including an underpass.
            if (!planFromGlobalTraj(1))
            {
              ROS_WARN_THROTTLE(0.5, "[FAPP避障] 暂无可行绕行或下穿轨迹，保持悬停。");
              break;
            }
            swing_wait_active_ = false;
            fapp_wait_active_ = false;
            swing_clear_since_ = 0.0;
            swing_wait_obstacle_id_ = 0;
            need_hover_stop_ = false;
            periodic_replan_pending_ = false;
            replan_fail_count_ = 0;
            last_target_change_time_ = now_sec;
            stuck_detect_ignore_until_ = now_sec + stuck_detect_grace_time_;
            flag_escape_emergency_ = true;
            publishPlanningStatus("TRAJECTORY_PUBLISHED");
            publishPlanningStatus("SWING_OBSTACLE_RELEASED");
            changeFSMExecState(EXEC_TRAJ, "FAPP_RELEASE");
            break;
          }
          SwingCollisionResult collision;
          const bool blocked = swingTrajectoryBlocked(sampleReleaseTrajectory(),
                                                       now_sec, &collision);
          if (blocked)
          {
            swing_clear_since_ = 0.0;
            swing_wait_obstacle_id_ = collision.obstacle_id;
            ROS_WARN_THROTTLE(
                0.5,
                "[摆球避障] 保持悬停：id=%u，若现在放行将在 %.2fs 后相交，球预测=(%.2f, %.2f, %.2f)。",
                collision.obstacle_id, collision.time_from_now,
                collision.obstacle_position.x(), collision.obstacle_position.y(),
                collision.obstacle_position.z());
            break;
          }

          if (swing_clear_since_ <= 0.0)
            swing_clear_since_ = now_sec;
          if (now_sec - swing_clear_since_ < swing_release_clear_time_)
            break;

          ROS_INFO("[摆球避障] 穿越窗口已连续安全 %.2fs，恢复原目标并重新规划。",
                   now_sec - swing_clear_since_);
          publishPlanningStatus("SWING_OBSTACLE_RELEASED");
          swing_wait_active_ = false;
          swing_clear_since_ = 0.0;
          swing_wait_obstacle_id_ = 0;
          need_hover_stop_ = false;
          replan_fail_count_ = 0;
          last_target_change_time_ = now_sec;
          stuck_detect_ignore_until_ = now_sec + stuck_detect_grace_time_;
          changeFSMExecState(GEN_NEW_TRAJ, "SWING_RELEASE");
          break;
        }

        const int current_occ = planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
        if (enable_occupied_recovery_ && current_occ != 0)
        {
          beginOccupiedRecoveryEpisode();
        }
        // 占据事件持续到退让完成；单帧清空不能恢复原目标、再按漂移位置急停。
        if (enable_occupied_recovery_ && occupied_recovery_episode_ &&
            ((have_target_ && have_trigger_) ||
             (wait_new_target_after_occupied_recovery_ &&
              active_external_goal_stamp_ns_ != 0ULL)) && !mandatory_stop_)
        {
          if (occupied_recovery_attempt_count_ >= escape_max_attempts_)
          {
            if (now_sec - last_escape_target_search_time_ >= 1.0)
            {
              occupied_recovery_attempt_count_ = 0;
              occupied_recovery_failure_reported_ = false;
              last_escape_target_search_time_ = now_sec;
              ROS_WARN("[局部脱障] 本轮%d个候选均失败；地图刷新后自动开始下一轮候选搜索。",
                       escape_max_attempts_);
            }
          }
          else if (now_sec - last_escape_target_search_time_ >= 0.20)
          {
            last_escape_target_search_time_ = now_sec;
            if (startOccupiedRecovery(now_sec))
              changeFSMExecState(OCCUPIED_RECOVERY, "OCCUPIED_RECOVERY_START");
            else
              ROS_ERROR_THROTTLE(1.0, "[局部脱障] 当前地图中没有满足约束的安全点，继续保持固定悬停并等待地图更新。");
          }
        }
        else if (!need_hover_stop_)
        {
          last_target_change_time_ = now_sec;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        else if (need_hover_stop_)
        {
          // 2026-07-07: 对非 mandatory stop 的中途急停优先尝试自动恢复规划，避免录像里那种停住后直接丢失当前任务。
          if (have_target_ && have_trigger_ && !mandatory_stop_)
          {
            if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_) != 0)
              break; // Keep stopped while the same goal is still occupied.
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

    case OCCUPIED_RECOVERY:
    {
      if (!enable_occupied_recovery_ || mandatory_stop_ || !occupied_recovery_active_)
      {
        abortOccupiedRecovery("脱障被禁用、收到强制停止，或脱障状态无效");
        break;
      }

      const int current_occ = planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
      const Eigen::Vector3d remaining = occupied_recovery_target_ - odom_pos_;
      const bool reached_target = occupied_recovery_vertical_
                                      ? std::abs(remaining.z()) <= escape_reach_tolerance_
                                      : remaining.head<2>().norm() <= escape_reach_tolerance_;
      const auto &local = planner_manager_->traj_.local_traj;
      // 回环/近邻折线可能提前经过终点附近，至少执行到最后一段再判到达。
      const int piece_count = local.traj.getPieceNum();
      const bool final_history_piece = !occupied_recovery_from_history_ || piece_count <= 1 ||
          now_sec >= local.start_time + local.duration -
                     local.traj[piece_count - 1].getDuration();
      if (current_occ == 0 && reached_target && final_history_piece)
      {
        ROS_INFO("[局部脱障] 已到达退让点并离开膨胀占据区：pos=(%.3f, %.3f, %.3f)。",
                 odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
        publishPlanningStatus("OCCUPIED_RECOVERY_SUCCEEDED");
        occupied_recovery_active_ = false;
        occupied_recovery_episode_ = false;
        occupied_recovery_attempt_count_ = 0;
        occupied_recovery_failure_reported_ = false;
        replan_fail_count_ = 0;
        last_target_change_time_ = now_sec;
        stuck_detect_ignore_until_ = now_sec + stuck_detect_grace_time_;
        if (wait_new_target_after_occupied_recovery_)
        {
          // Relay mode discards the unsafe FIFO endpoint in the follower. Do not
          // immediately drive back to the same point before that status is handled.
          need_hover_stop_ = true;
          have_target_ = false;
          have_trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "OCCUPIED_RECOVERY_WAIT_NEXT_TARGET");
        }
        else
        {
          need_hover_stop_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "OCCUPIED_RECOVERY_DONE");
        }
        break;
      }

      if (now_sec >= occupied_recovery_deadline_)
      {
        abortOccupiedRecovery("低速脱障轨迹执行超时");
        break;
      }

      if (!occupied_recovery_from_history_ &&
          now_sec - last_escape_path_check_time_ >= 0.10)
      {
        last_escape_path_check_time_ = now_sec;
        if (!validateRecoverySegment(odom_pos_, occupied_recovery_target_,
                                     true, nullptr))
        {
          abortOccupiedRecovery("实时地图更新后剩余脱障路径不再安全");
          break;
        }
      }
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
    // 正常完成/超时会先结束脱障；其余离开路径也必须通知跟随器交还控制。
    if (exec_state_ == OCCUPIED_RECOVERY && new_state != OCCUPIED_RECOVERY)
      cancelOccupiedRecovery("OCCUPIED_RECOVERY_ABORTED");

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START", "OCCUPIED_RECOVERY"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void DiffReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START", "OCCUPIED_RECOVERY"};

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

  bool DiffReplanFSM::handleManualTrajectoryEnd(double now)
  {
    // Relay FIFO goals use MANUAL_TARGET. A consumed waypoint may still have
    // a trajectory finishing in the controller while the next goal waits.
    if (target_type_ != TARGET_TYPE::MANUAL_TARGET ||
        (exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ) ||
        !have_target_ || !have_odom_ || have_new_target_)
      return false;

    const LocalTrajData &info = planner_manager_->traj_.local_traj;
    if (info.traj_id <= 0 || info.duration <= 0.0 ||
        info.traj.getPieceNum() == 0 || !std::isfinite(now) ||
        !std::isfinite(info.start_time) || !std::isfinite(info.duration) ||
        now < info.start_time + info.duration ||
        (local_target_pt_ - final_goal_).norm() >= 1e-2 ||
        (info.traj.getPos(info.duration) - final_goal_).norm() >= 1e-2)
      return false;

    const double error = (odom_pos_ - final_goal_).norm();
    if (!std::isfinite(error) || error > max_tracking_error_)
      return false; // A real tracking deviation still needs replanning.

    // 终点收敛也执行原有目标占据处理，不能先判完成或直接跳过检查。
    // 三个调用入口（执行、重规划、碰撞定时器）沿用同一处理。
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      if (!mondify_final_goal_)
      {
        ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
        need_hover_stop_ = true;
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "STUCK_DETECT");
        return true;
      }
      if (mondifyInCollisionFinalGoal())
      {
        ROS_WARN("Successfully modified final_goal at trajectory end !!!");
        changeFSMExecState(REPLAN_TRAJ, "mondify_FSM");
        return true;
      }
      return false;
    }

    periodic_replan_pending_ = false;
    replan_fail_count_ = 0;
    if (error <= no_replan_thresh_)
    {
      have_target_ = false;
      have_trigger_ = false;
      ROS_INFO("[TRAJECTORY_COMPLETED] goal=(%.3f, %.3f, %.3f), error=%.3f m; "
               "keep endpoint hover and wait for the next goal.",
               final_goal_.x(), final_goal_.y(), final_goal_.z(), error);
      changeFSMExecState(WAIT_TARGET, "TRAJECTORY_COMPLETED");
    }
    else if (exec_state_ == REPLAN_TRAJ)
      changeFSMExecState(EXEC_TRAJ, "ENDPOINT_CONVERGING");

    // Expiration alone is not a tracking fault. Keep the controller's terminal
    // reference while it converges; skip periodic replanning, not stuck checks.
    return true;
  }

  void DiffReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    refreshOdometry();
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

    if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP ||
        exec_state_ == OCCUPIED_RECOVERY || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      depth_timeout_emergency_ = true;
      need_hover_stop_ = true;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return;
    }

    if (enable_occupied_recovery_ &&
        map->getInflateOccupancy(odom_pos_) != 0)
    {
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      occupied_recovery_active_ = false;
      beginOccupiedRecoveryEpisode();
      changeFSMExecState(EMERGENCY_STOP, "OCCUPIED_SAFETY_PREEMPT");
      return;
    }

    // This timer can run before execFSMCallback after the trajectory expires.
    // Actual-position occupancy and depth checks above retain their priority.
    if (handleManualTrajectoryEnd(ros::Time::now().toSec()))
      return;

    if (enable_swing_obstacle_guard_ && !swing_wait_active_)
    {
      SwingCollisionResult collision;
      const double swing_query_time = ros::Time::now().toSec();
      if (swingTrajectoryBlocked(sampleCurrentTrajectory(swing_query_time),
                                 swing_query_time, &collision))
      {
        startSwingWait(collision);
        return;
      }
    }

    if (planner_manager_->dynamicAvoidanceEnabled() && !swing_wait_active_)
    {
      const double query_time = ros::Time::now().toSec();
      std::vector<std::pair<double, Eigen::Vector3d>> dynamic_samples;
      for (const auto &sample : sampleCurrentTrajectory(query_time))
        dynamic_samples.emplace_back(sample.time_from_now, sample.position);
      SwingCollisionResult collision;
      if (planner_manager_->avoidanceTrajectoryBlocked(dynamic_samples, query_time,
                                                      &collision))
      {
        if (planFromLocalTraj(1))
        {
          periodic_replan_pending_ = false;
          replan_fail_count_ = 0;
          changeFSMExecState(EXEC_TRAJ, "FAPP_REPLAN");
          return;
        }
        startSwingWait(collision, true);
        return;
      }
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

  void DiffReplanFSM::fappObjectsCallback(
      const obj_state_msgs::ObjectsStatesConstPtr &msg)
  {
    planner_manager_->updateDynamicObjects(*msg);
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "world" &&
        msg->header.frame_id != "UAV1/camera_init" &&
        msg->header.frame_id != "camera_init")
    {
      ROS_WARN_THROTTLE(2.0,
                       "[FAPP避障] frame_id=%s；请确认状态与规划里程计使用同一坐标系。",
                       msg->header.frame_id.c_str());
    }
  }

  void DiffReplanFSM::dynamicObjectsCallback(
      const ldop::DynamicObjectArrayConstPtr &msg)
  {
    std::vector<SwingObstacleObservation> observations;
    observations.reserve(msg->objects.size());
    for (const ldop::DynamicObject &object : msg->objects)
    {
      if (object.motion_model_type != ldop::DynamicObject::MOTION_MODEL_CV3D ||
          object.model_state.size() < 6)
      {
        ROS_WARN_THROTTLE(2.0,
                          "[摆球避障] 跳过非CV3D或状态长度不足的LDOP目标 id=%u。",
                          object.id);
        continue;
      }

      SwingObstacleObservation observation;
      observation.id = object.id;
      observation.position = Eigen::Vector3d(object.model_state[0],
                                             object.model_state[1],
                                             object.model_state[2]);
      observation.velocity = Eigen::Vector3d(object.model_state[3],
                                             object.model_state[4],
                                             object.model_state[5]);
      observation.size = Eigen::Vector3d(std::max(0.0, object.size.x),
                                         std::max(0.0, object.size.y),
                                         std::max(0.0, object.size.z));
      observations.push_back(observation);
    }

    const double stamp = msg->header.stamp.isZero()
                             ? ros::Time::now().toSec()
                             : msg->header.stamp.toSec();
    swing_obstacle_guard_.update(observations, stamp);

    if (!msg->header.frame_id.empty() && msg->header.frame_id != "world" &&
        msg->header.frame_id != "UAV1/camera_init" &&
        msg->header.frame_id != "camera_init")
    {
      ROS_WARN_THROTTLE(
          2.0,
          "[摆球避障] LDOP frame_id=%s；当前启动文件只保证 world 与 UAV1/camera_init 重合，请确认坐标系。",
          msg->header.frame_id.c_str());
    }
  }

  std::vector<SwingTrajectorySample> DiffReplanFSM::sampleCurrentTrajectory(
      double now) const
  {
    std::vector<SwingTrajectorySample> samples;
    const LocalTrajData &local = planner_manager_->traj_.local_traj;
    if (local.traj_id <= 0 || local.duration <= 0.0 || !std::isfinite(now))
      return samples;

    const double current_time =
        std::max(0.0, std::min(local.duration, now - local.start_time));
    const double remaining = local.duration - current_time;
    const double horizon = std::min(swing_prediction_horizon_, remaining);
    if (planner_manager_->dynamicAvoidanceEnabled())
    {
      for (const auto &sample : sampleCollisionTrajectory(local.traj, current_time,
          current_time + horizon, 0.5 * planner_manager_->grid_map_->getResolution(),
          std::min(0.05, swing_prediction_dt_)))
        samples.push_back({sample.first, sample.second});
      return samples;
    }
    for (double dt = 0.0; dt < horizon; dt += swing_prediction_dt_)
      samples.push_back({dt, local.traj.getPos(current_time + dt)});
    samples.push_back({horizon, local.traj.getPos(current_time + horizon)});
    return samples;
  }

  std::vector<SwingTrajectorySample> DiffReplanFSM::sampleReleaseTrajectory() const
  {
    std::vector<SwingTrajectorySample> samples;
    Eigen::Vector3d target = local_target_pt_;
    Eigen::Vector3d displacement = target - odom_pos_;
    displacement.z() = 0.0;
    if (!displacement.allFinite() || displacement.head<2>().norm() < 0.05)
    {
      target = final_goal_;
      displacement = target - odom_pos_;
      displacement.z() = 0.0;
    }
    if (!displacement.allFinite() || displacement.head<2>().norm() < 0.05)
      return samples;

    const double horizontal_distance = displacement.head<2>().norm();
    const Eigen::Vector3d direction = displacement / horizontal_distance;
    for (double time = 0.0; time < swing_prediction_horizon_;
         time += swing_prediction_dt_)
    {
      const double progress =
          std::min(horizontal_distance, swing_release_speed_ * time);
      Eigen::Vector3d position = odom_pos_ + direction * progress;
      const double ratio = horizontal_distance > 1.0e-6
                               ? progress / horizontal_distance
                               : 0.0;
      position.z() = odom_pos_.z() + ratio * (target.z() - odom_pos_.z());
      samples.push_back({time, position});
    }
    const double final_progress = std::min(
        horizontal_distance, swing_release_speed_ * swing_prediction_horizon_);
    Eigen::Vector3d final_position = odom_pos_ + direction * final_progress;
    final_position.z() = odom_pos_.z() +
                         (final_progress / horizontal_distance) *
                             (target.z() - odom_pos_.z());
    samples.push_back({swing_prediction_horizon_, final_position});
    return samples;
  }

  bool DiffReplanFSM::swingTrajectoryBlocked(
      const std::vector<SwingTrajectorySample> &samples, double now,
      SwingCollisionResult *result) const
  {
    return enable_swing_obstacle_guard_ &&
           swing_obstacle_guard_.findCollision(samples, now, result);
  }

  void DiffReplanFSM::startSwingWait(const SwingCollisionResult &collision,
                                    const bool from_fapp)
  {
    swing_wait_active_ = true;
    fapp_wait_active_ = from_fapp;
    swing_clear_since_ = 0.0;
    swing_wait_obstacle_id_ = collision.obstacle_id;
    need_hover_stop_ = true;
    flag_escape_emergency_ = true;
    cancelOccupiedRecovery("OCCUPIED_RECOVERY_ABORTED");
    publishPlanningStatus("SWING_OBSTACLE_WAIT");
    ROS_ERROR(
        "[摆球避障] 轨迹将在 %.2fs 后与 id=%u 相交，停车等待；球预测=(%.2f, %.2f, %.2f)，轨迹点=(%.2f, %.2f, %.2f)。",
        collision.time_from_now, collision.obstacle_id,
        collision.obstacle_position.x(), collision.obstacle_position.y(),
        collision.obstacle_position.z(), collision.vehicle_position.x(),
        collision.vehicle_position.y(), collision.vehicle_position.z());
    changeFSMExecState(EMERGENCY_STOP, "SWING_GUARD");
  }

  bool DiffReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    const LocalTrajData &local_traj = planner_manager_->traj_.local_traj;
    const double duration = local_traj.traj.getTotalDuration();
    if (local_traj.traj.getPieceNum() > 0 && std::isfinite(duration) && duration > 0.0)
    {
      const double elapsed = ros::Time::now().toSec() - local_traj.start_time;
      const double sample_time = std::max(0.0, std::min(elapsed, duration));
      const double planned_height = local_traj.traj.getPos(sample_time).z();
      if (std::isfinite(planned_height))
        stop_pos.z() = planned_height;
    }

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    publishTrackingResult(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);
    return true;
  }

  void DiffReplanFSM::updateFreeOdomHistory(double now)
  {
    if (!enable_occupied_recovery_)
      return;

    // Freeze the pre-fault safe trail while braking/recovering. Otherwise a
    // slow emergency stop can age every useful sample out of the history.
    if (occupied_recovery_episode_ || exec_state_ == OCCUPIED_RECOVERY ||
        exec_state_ == EMERGENCY_STOP)
      return;

    while (escape_history_time_ > 0.0 && !free_odom_history_.empty() &&
           now - free_odom_history_.front().stamp > escape_history_time_)
      free_odom_history_.pop_front();

    if (!odom_pos_.allFinite() ||
        planner_manager_->grid_map_->getInflateOccupancy(odom_pos_) != 0)
      return;

    const double resolution = planner_manager_->grid_map_->getResolution();
    if (!free_odom_history_.empty())
    {
      const FreeOdomSample &last = free_odom_history_.back();
      const Eigen::Vector3d last_pos(last.x, last.y, last.z);
      if (now - last_free_history_record_time_ < 0.05 &&
          (odom_pos_ - last_pos).norm() < 0.5 * resolution)
        return;
    }

    free_odom_history_.push_back(
        FreeOdomSample{now, odom_pos_.x(), odom_pos_.y(), odom_pos_.z()});
    last_free_history_record_time_ = now;
    occupied_recovery_episode_ = false;
    occupied_recovery_attempt_count_ = 0;
    occupied_recovery_failure_reported_ = false;
  }

  double DiffReplanFSM::estimateInflatedClearance(const Eigen::Vector3d &pos)
  {
    auto map = planner_manager_->grid_map_;
    if (!pos.allFinite() || map->getInflateOccupancy(pos) != 0)
      return 0.0;

    const double resolution = map->getResolution();
    const double max_radius = std::max(resolution, escape_clearance_search_radius_);
    for (double radius = resolution; radius <= max_radius + 1.0e-6;
         radius += resolution)
    {
      const int sample_count = std::max(
          12, static_cast<int>(std::ceil(2.0 * M_PI * radius / resolution)));
      for (int i = 0; i < sample_count; ++i)
      {
        const double angle = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(sample_count);
        Eigen::Vector3d probe = pos;
        probe.x() += radius * std::cos(angle);
        probe.y() += radius * std::sin(angle);
        if (map->getInflateOccupancy(probe) != 0)
          return radius;
      }

      Eigen::Vector3d probe_up = pos;
      Eigen::Vector3d probe_down = pos;
      probe_up.z() += radius;
      probe_down.z() -= radius;
      if (map->getInflateOccupancy(probe_up) != 0 ||
          map->getInflateOccupancy(probe_down) != 0)
        return radius;
    }

    return max_radius;
  }

  bool DiffReplanFSM::validateRecoverySegment(const Eigen::Vector3d &start,
                                              const Eigen::Vector3d &end,
                                              bool allow_initial_occupied,
                                              double *occupied_prefix)
  {
    if (occupied_prefix != nullptr)
      *occupied_prefix = 0.0;
    if (!start.allFinite() || !end.allFinite())
      return false;

    auto map = planner_manager_->grid_map_;
    const double distance = (end - start).norm();
    if (map->getInflateOccupancy(end) != 0)
      return false;
    if (distance < 1.0e-3)
      return true;

    const double step = std::max(0.5 * map->getResolution(), 0.02);
    const int sample_count = std::max(1, static_cast<int>(std::ceil(distance / step)));
    bool reached_free_space = false;
    double prefix = 0.0;
    for (int i = 0; i <= sample_count; ++i)
    {
      const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
      const Eigen::Vector3d sample = start + ratio * (end - start);
      const bool occupied = map->getInflateOccupancy(sample) != 0;
      if (!occupied)
      {
        reached_free_space = true;
        continue;
      }

      if (!allow_initial_occupied || reached_free_space)
        return false;
      prefix = ratio * distance;
      if (prefix > escape_max_occupied_prefix_ + 1.0e-6)
        return false;
    }

    if (occupied_prefix != nullptr)
      *occupied_prefix = prefix;
    return reached_free_space;
  }

  bool DiffReplanFSM::selectHistoryRecoveryTarget(Eigen::Vector3d &target,
                                                  double &clearance,
                                                  std::vector<Eigen::Vector3d> *history_path)
  {
    const double resolution = planner_manager_->grid_map_->getResolution();
    const double min_distance = std::max(resolution, 2.0 * escape_reach_tolerance_);
    double history_distance = 0.0;
    double best_distance_error = std::numeric_limits<double>::infinity();
    double selected_history_distance = 0.0;
    double selected_direct_distance = 0.0;
    bool found = false;
    Eigen::Vector3d previous = odom_pos_;
    std::vector<Eigen::Vector3d> reverse_path{odom_pos_};
    std::size_t selected_path_size = 0;
    for (auto it = free_odom_history_.rbegin(); it != free_odom_history_.rend(); ++it)
    {
      const Eigen::Vector3d candidate(it->x, it->y, escape_recovery_height_);
      if (!candidate.allFinite())
        continue;
      history_distance += std::hypot(candidate.x() - previous.x(),
                                     candidate.y() - previous.y());
      previous = candidate;
      reverse_path.push_back(candidate);
      const double direct_distance = std::hypot(candidate.x() - odom_pos_.x(),
                                                candidate.y() - odom_pos_.y());
      const double distance_error = std::abs(history_distance - escape_max_distance_);
      // 悬停摆动/回环的累计路程不等于实际退让量，不能选择几乎仍在原地的点。
      if (history_distance >= min_distance && direct_distance >= min_distance &&
          distance_error < best_distance_error)
      {
        target = candidate;
        clearance = 0.0;
        best_distance_error = distance_error;
        selected_history_distance = history_distance;
        selected_direct_distance = direct_distance;
        selected_path_size = reverse_path.size();
        found = true;
      }

      if (history_distance >= escape_max_distance_)
        break;
    }
    if (history_path != nullptr)
    {
      reverse_path.resize(selected_path_size);
      *history_path = std::move(reverse_path);
    }
    if (found)
    {
      ROS_INFO("[局部脱障] 信任后机自身实飞历史，沿轨迹后退 %.3f m，直线位移 %.3f m，目标后退量 %.3f m；不使用当前膨胀地图否决。",
               selected_history_distance, selected_direct_distance,
               escape_max_distance_);
    }
    return found;
  }

  bool DiffReplanFSM::selectVerticalRecoveryTarget(Eigen::Vector3d &target,
                                                   double &clearance)
  {
    if (odom_pos_.z() >= escape_recovery_height_ - escape_reach_tolerance_)
      return false;

    Eigen::Vector3d candidate = odom_pos_;
    candidate.z() = escape_recovery_height_;
    double occupied_prefix = 0.0;
    if (!validateRecoverySegment(odom_pos_, candidate, true, &occupied_prefix))
      return false;

    const double candidate_clearance = estimateInflatedClearance(candidate);
    if (candidate_clearance + 1.0e-6 < escape_min_clearance_)
      return false;

    target = candidate;
    clearance = candidate_clearance;
    ROS_WARN("[局部脱障] 当前低于名义高度，优先固定XY上升至 %.3f m；"
             "占据前缀 %.3f m，终点净空 %.3f m。",
             escape_recovery_height_, occupied_prefix, clearance);
    return true;
  }

  bool DiffReplanFSM::selectLateralRecoveryTarget(Eigen::Vector3d &target,
                                                  double &clearance)
  {
    Eigen::Vector3d forward = final_goal_ - odom_pos_;
    forward.z() = 0.0;
    if (!forward.allFinite() || forward.head<2>().norm() < 0.05)
    {
      forward = odom_vel_;
      forward.z() = 0.0;
    }
    if (!forward.allFinite() || forward.head<2>().norm() < 0.05)
      forward = Eigen::Vector3d::UnitX();
    else
      forward.normalize();
    const Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);

    // Search only side/forward-side sectors. No candidate with negative mission
    // progress is allowed in this fallback, so it cannot choose another retreat.
    const double angles_deg[] = {90.0, -90.0, 75.0, -75.0,
                                 60.0, -60.0, 45.0, -45.0};
    const double resolution = planner_manager_->grid_map_->getResolution();
    const double min_radius = std::max(2.0 * resolution, 0.15);
    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;

    for (double radius = min_radius; radius <= escape_max_distance_ + 1.0e-6;
         radius += resolution)
    {
      for (double angle_deg : angles_deg)
      {
        const double angle = angle_deg * M_PI / 180.0;
        const Eigen::Vector3d direction =
            std::cos(angle) * forward + std::sin(angle) * lateral;
        const double forward_progress = direction.dot(forward);
        if (forward_progress < -1.0e-6)
          continue;

        Eigen::Vector3d candidate = odom_pos_ + radius * direction;
        candidate.z() = odom_pos_.z();
        if (planner_manager_->grid_map_->getInflateOccupancy(candidate) != 0)
          continue;

        double occupied_prefix = 0.0;
        if (!validateRecoverySegment(odom_pos_, candidate, true, &occupied_prefix))
          continue;

        const double candidate_clearance = estimateInflatedClearance(candidate);
        if (candidate_clearance + 1.0e-6 < escape_min_clearance_)
          continue;

        const double lateral_preference = std::abs(direction.dot(lateral));
        const double score = 10.0 * candidate_clearance +
                             0.50 * lateral_preference +
                             0.10 * forward_progress - 0.20 * radius;
        if (score > best_score)
        {
          best_score = score;
          target = candidate;
          clearance = candidate_clearance;
          found = true;
        }
      }
    }

    if (found)
    {
      const Eigen::Vector3d displacement = target - odom_pos_;
      ROS_WARN("[局部脱障] 历史点不可用，选择实时地图侧向安全点：位移=(%.3f, %.3f, %.3f) m，"
               "前向分量=%.3f m，净空=%.3f m。",
               displacement.x(), displacement.y(), displacement.z(),
               displacement.dot(forward), clearance);
    }
    return found;
  }

  bool DiffReplanFSM::selectOccupiedRecoveryTarget(Eigen::Vector3d &target,
                                                   bool &from_history,
                                                   double &clearance,
                                                   std::vector<Eigen::Vector3d> *history_path)
  {
    if (history_path != nullptr)
      history_path->clear();
    occupied_recovery_vertical_ = false;
    if (!occupied_recovery_history_failed_ &&
        selectHistoryRecoveryTarget(target, clearance, history_path))
    {
      from_history = true;
      return true;
    }
    if (history_only_occupied_recovery_)
      return false;
    if (selectVerticalRecoveryTarget(target, clearance))
    {
      from_history = false;
      occupied_recovery_vertical_ = true;
      return true;
    }
    if (selectLateralRecoveryTarget(target, clearance))
    {
      from_history = false;
      return true;
    }
    return false;
  }

  bool DiffReplanFSM::callOccupiedRecovery(
      const Eigen::Vector3d &target,
      const std::vector<Eigen::Vector3d> &history_path)
  {
    const bool success = history_path.empty()
        ? planner_manager_->OccupiedStartRecovery(
              odom_pos_, target, escape_speed_, escape_max_occupied_prefix_)
        : planner_manager_->OccupiedHistoryRecovery(
              history_path, escape_speed_, escape_max_occupied_prefix_);
    if (!success)
      return false;

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj minco_msg;
    polyTraj2ROSMsg(poly_msg, minco_msg);
    poly_traj_pub_.publish(poly_msg);
    publishTrackingResult(poly_msg);
    // MINCO只包含端点PVA和中间位置，接收端会重新求解，无法表达拐点停转。
    // 异构接力由完整PolyTraj驱动；不能把多段退让广播成一条重新切角的MINCO。
    if (history_path.empty() || planner_manager_->traj_.local_traj.traj.getPieceNum() == 1)
      broadcast_ploytraj_pub_.publish(minco_msg);
    return true;
  }

  bool DiffReplanFSM::startOccupiedRecovery(double now)
  {
    beginOccupiedRecoveryEpisode();
    Eigen::Vector3d target;
    double clearance = 0.0;
    bool from_history = false;
    std::vector<Eigen::Vector3d> history_path;
    if (!selectOccupiedRecoveryTarget(target, from_history, clearance, &history_path))
      return false;
    if (!callOccupiedRecovery(target, history_path))
    {
      if (!from_history)
        return false;
      // 历史点能选中但轨迹生成失败，也必须给现有垂直/侧向备选一次机会。
      occupied_recovery_history_failed_ = true;
      if (!selectOccupiedRecoveryTarget(target, from_history, clearance, &history_path) ||
          !callOccupiedRecovery(target, history_path))
        return false;
    }

    occupied_recovery_target_ = target;
    occupied_recovery_from_history_ = from_history;
    occupied_recovery_active_ = true;
    occupied_recovery_episode_ = true;
    occupied_recovery_stamp_ns_ = std::max<uint64_t>(
        occupied_recovery_stamp_ns_ + 1ULL,
        static_cast<uint64_t>(ros::Time::now().toNSec()));
    ++occupied_recovery_attempt_count_;
    replan_fail_count_ = 0;
    last_escape_path_check_time_ = now;
    occupied_recovery_deadline_ = planner_manager_->traj_.local_traj.start_time +
                                  planner_manager_->traj_.local_traj.duration + 1.0;

    visualization_->displayGoalPoint(
        target,
        occupied_recovery_vertical_
            ? Eigen::Vector4d(1.0, 0.7, 0.1, 1.0)
            : (from_history ? Eigen::Vector4d(0.0, 1.0, 0.2, 1.0)
                            : Eigen::Vector4d(0.1, 0.8, 1.0, 1.0)),
        0.18, 1000 + planner_manager_->pp_.drone_id);
    publishPlanningStatus(occupied_recovery_vertical_
                              ? "OCCUPIED_RECOVERY_VERTICAL"
                              : (from_history ? "OCCUPIED_RECOVERY_HISTORY"
                                              : "OCCUPIED_RECOVERY_LATERAL"));
    ROS_WARN("[局部脱障] 开始第 %d/%d 次低速脱障，target=(%.3f, %.3f, %.3f)，"
             "来源=%s，净空=%.3f m，截止时间=%.3f。",
             occupied_recovery_attempt_count_, escape_max_attempts_, target.x(),
             target.y(), target.z(), occupied_recovery_vertical_
                                           ? "固定XY恢复名义高度"
                                           : (from_history ? "历史自由轨迹"
                                                           : "实时侧向搜索"),
             clearance, occupied_recovery_deadline_);
    return true;
  }

  void DiffReplanFSM::abortOccupiedRecovery(const char *reason)
  {
    ROS_ERROR("[局部脱障] %s；在当前位置重新急停。", reason);
    if (occupied_recovery_from_history_)
      occupied_recovery_history_failed_ = true;
    publishPlanningStatus("OCCUPIED_RECOVERY_ABORTED");
    occupied_recovery_active_ = false;
    need_hover_stop_ = true;
    callEmergencyStop(odom_pos_);
    flag_escape_emergency_ = false;
    last_escape_target_search_time_ = ros::Time::now().toSec();
    changeFSMExecState(EMERGENCY_STOP, "OCCUPIED_RECOVERY_ABORT");
  }

  void DiffReplanFSM::beginOccupiedRecoveryEpisode()
  {
    if (!occupied_recovery_episode_)
    {
      occupied_recovery_goal_stamp_ns_ = active_external_goal_stamp_ns_;
      occupied_recovery_history_failed_ = false;
    }
    occupied_recovery_episode_ = true;
  }

  void DiffReplanFSM::cancelOccupiedRecovery(const std::string &status)
  {
    if (!occupied_recovery_active_)
      return;
    publishPlanningStatus(status);
    occupied_recovery_active_ = false;
  }

  bool DiffReplanFSM::callReboundReplan(bool flag_use_poly_init,
                                        bool flag_randomPolyTraj,
                                        bool require_improvement)
  {
    if (mondify_final_goal_ && mondifyInCollisionFinalGoal()) 
    {
      ROS_WARN("Successfully modified final_goal in callReboundReplan !!!");
    }
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool trajectory_replaced = false;
    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_, require_improvement,
        &trajectory_replaced);

    have_new_target_ = false;

    if (plan_success && trajectory_replaced)
    {
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      publishTrackingResult(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool DiffReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    refreshOdometry();

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

  bool DiffReplanFSM::planFromLocalTraj(const int trial_times /*=1*/,
                                        const bool require_improvement /*=false*/)
  {
    refreshOdometry();

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    const double t_cur_raw = ros::Time::now().toSec() - info->start_time;
    const double duration = info->traj.getTotalDuration();
    const bool trajectory_time_valid = std::isfinite(t_cur_raw) &&
                                       std::isfinite(duration) && duration > 0.0 &&
                                       t_cur_raw >= 0.0 && t_cur_raw <= duration;
    const double t_cur = std::isfinite(t_cur_raw) && std::isfinite(duration) && duration > 0.0
                             ? std::max(0.0, std::min(t_cur_raw, duration))
                             : 0.0;

    Eigen::Vector3d predicted_pos = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d predicted_vel = predicted_pos;
    Eigen::Vector3d predicted_acc = predicted_pos;
    if (info->traj.getPieceNum() > 0 && std::isfinite(duration) && duration > 0.0)
    {
      predicted_pos = info->traj.getPos(t_cur);
      predicted_vel = info->traj.getVel(t_cur);
      predicted_acc = info->traj.getAcc(t_cur);
    }

    const bool prediction_finite = predicted_pos.allFinite() &&
                                   predicted_vel.allFinite() && predicted_acc.allFinite();
    const bool odom_finite = odom_pos_.allFinite() && odom_vel_.allFinite();
    if (!odom_finite)
    {
      ROS_ERROR_THROTTLE(1.0, "Cannot replan: odometry position or velocity is non-finite.");
      return false;
    }

    const double tracking_error = prediction_finite
                                      ? (predicted_pos - odom_pos_).norm()
                                      : std::numeric_limits<double>::infinity();
    const bool replan_from_odom = tracking_replan_pending_ ||
                                  !trajectory_time_valid || !prediction_finite ||
                                  !std::isfinite(tracking_error) ||
                                  tracking_error > max_tracking_error_;

    if (replan_from_odom)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      ROS_WARN("[TRACKING_DEVIATION_REPLAN] error=%.3f m, limit=%.3f m, "
               "trajectory_time=%.3f/%.3f s, predicted=(%.3f, %.3f, %.3f), "
               "odom=(%.3f, %.3f, %.3f), odom_age=%.3f s. Replanning from odometry.",
               tracking_error, max_tracking_error_, t_cur_raw, duration,
               predicted_pos.x(), predicted_pos.y(), predicted_pos.z(),
               odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
               odom_stamp_.isZero() ? 0.0 : (ros::Time::now() - odom_stamp_).toSec());
      publishPlanningStatus("TRACKING_DEVIATION_REPLAN");
    }
    else
    {
      start_pt_ = predicted_pos;
      start_vel_ = predicted_vel;
      start_acc_ = predicted_acc;
    }

    // A tracking deviation must use polynomial initialization from the actual
    // measured state; never retry the already-detached local prediction first.
    const bool apply_quality_gate = require_improvement && !replan_from_odom;
    bool success = callReboundReplan(replan_from_odom, false,
                                     apply_quality_gate);

    if (!success)
    {
      if (planner_manager_->dynamicCandidatesFailed(nullptr))
        return false;
      if (!replan_from_odom)
        success = callReboundReplan(true, false, apply_quality_gate);
      if (!success && enable_random_local_init_)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true, apply_quality_gate);
          if (success)
            break;
        }
      }
      if (!success)
        return false;
    }

    return true;
  }

    bool DiffReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan)
  {
    refreshOdometry();
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    if (planner_manager_->two_piece_entry_)
      one_pt_wps.push_back(0.5 * (odom_pos_ + next_wp));
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
      if (exec_state_ != WAIT_TARGET && flag_2replan &&
          exec_state_ != EMERGENCY_STOP && exec_state_ != OCCUPIED_RECOVERY)
      {
        // 新目标回调只切状态，不在回调内部嵌套 spin 等待状态机。否则碰撞重规划
        // 期间会阻塞订阅队列，表现为规划器存活却再也收不到后续替换目标。
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_EXTERNAL_GOAL");
      }
      else if(exec_state_ == EMERGENCY_STOP || exec_state_ == OCCUPIED_RECOVERY)
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
        if (preserve_modified_goal_height_)
          pt.z() = orig_goal.z();
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          for (int i = 6; i > 0; i--)
          {
            if (t - i * t_step > 0)
            {
              Eigen::Vector3d pt_tmp = planner_manager_->traj_.global_traj.traj.getPos(t - i * t_step);
              if (preserve_modified_goal_height_)
                pt_tmp.z() = orig_goal.z();
              if (!planner_manager_->grid_map_->getInflateOccupancy(pt_tmp))
              {
                pt = pt_tmp;
                break;
              }
            }
          }
          if (planNextWaypoint(pt, false)) // final_goal_=pt inside if success
          {
            external_goal_modified_ = true;
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
    handleWaypoint(msg, false);
  }

  void DiffReplanFSM::entryWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    handleWaypoint(msg, true);
  }

  void DiffReplanFSM::handleWaypoint(const geometry_msgs::PoseStampedPtr &msg,
                                    const bool two_piece_entry)
  {
    refreshOdometry();
    active_external_goal_stamp_ns_ = msg->header.stamp.toNSec();
    external_goal_modified_ = false;
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (planner_manager_->grid_map_->getInflateOccupancy(end_wp) == -1)
    {
      ROS_WARN("The goal is outside the safe fence, ignore this goal!");
      publishPlanningStatus("GOAL_REJECTED_OUTSIDE_MAP");  // 2026-07-28: 区分目标接收与目标可规划。
      return;
    }
    ROS_INFO("Received goal: %f, %f, %f", end_wp(0), end_wp(1), end_wp(2));
    planner_manager_->setEntryMode(two_piece_entry);
    if (planNextWaypoint(end_wp, true))
    {
      last_target_change_time_ = ros::Time::now().toSec();
      have_trigger_ = true;
    }
  }

  void DiffReplanFSM::publishPlanningStatus(const std::string &status)
  {
    // 使用String保持现有消息依赖不变。所有状态回传当前外部目标时间戳，成功时再
    // 附带Diff实际接受的final_goal，避免延迟/锁存回执污染刚重发的新目标。
    std_msgs::String msg;
    std::ostringstream stream;
    if (status.find("OCCUPIED_RECOVERY_") == 0)
      stream << status << " goal_stamp_ns=" << occupied_recovery_goal_stamp_ns_
             << " recovery_stamp_ns=" << occupied_recovery_stamp_ns_;
    else
      stream << status << " goal_stamp_ns=" << active_external_goal_stamp_ns_;
    if (status == "TRAJECTORY_PUBLISHED")
    {
      stream << " " << final_goal_.x() << " " << final_goal_.y() << " "
             << final_goal_.z();
      external_goal_modified_ = false;
    }
    msg.data = stream.str();
    planning_status_pub_.publish(msg);
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
    (void)msg;
    mandatory_stop_ = true;
    controller_restart_pending_ = false;
    cancelOccupiedRecovery("OCCUPIED_RECOVERY_ABORTED");
    occupied_recovery_episode_ = false;
    flag_escape_emergency_ = true;
    ROS_ERROR("Received controller safety stop; pause planning until restart signal.");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
  }

  void DiffReplanFSM::planningRestartCallback(const std_msgs::Empty &msg)
  {
    (void)msg;
    if (mandatory_stop_)
    {
      mandatory_stop_ = false;
      enable_fail_safe_ = true;
      need_hover_stop_ = false;
      ROS_WARN("[控制器恢复] 解除控制器临时停止，基于最新里程计恢复当前任务。");
    }
    if (!have_odom_ ||
        ((!have_target_ || !have_trigger_) && !occupied_recovery_episode_))
    {
      ROS_WARN("[控制器恢复] 暂不能重新规划：odom=%d, target=%d, trigger=%d。",
               have_odom_, have_target_, have_trigger_);
      return;
    }

    controller_restart_pending_ = true;
    if (fapp_wait_active_)
    {
      swing_wait_active_ = false;
      fapp_wait_active_ = false;
      swing_clear_since_ = 0.0;
      swing_wait_obstacle_id_ = 0;
    }
    replan_fail_count_ = 0;
    need_hover_stop_ = false;
    cancelOccupiedRecovery("OCCUPIED_RECOVERY_CANCELLED");
    const double now = ros::Time::now().toSec();
    last_target_change_time_ = now;
    stuck_detect_ignore_until_ = now + stuck_detect_grace_time_;

    if (planner_manager_->grid_map_->getInflateOccupancy(odom_pos_) != 0)
    {
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      beginOccupiedRecoveryEpisode();
      changeFSMExecState(EMERGENCY_STOP, "CONTROLLER_RESTART_OCCUPIED");
      ROS_WARN("[控制器恢复] 最新里程计仍在膨胀障碍内，先进入现有局部脱障流程。");
      return;
    }

    occupied_recovery_episode_ = false;
    if (!have_target_ || !have_trigger_)
    {
      changeFSMExecState(WAIT_TARGET, "CONTROLLER_RESTART_WAIT_TARGET");
      return;
    }
    changeFSMExecState(GEN_NEW_TRAJ, "CONTROLLER_RESTART");
    ROS_WARN("[控制器恢复] 保留当前目标和航点，从最新里程计位置、速度强制生成新轨迹。");
  }

  void DiffReplanFSM::initOdometry(ros::NodeHandle &nh)
  {
    odom_node_ = nh;
    odom_node_.setCallbackQueue(&odom_callback_queue_);
    odom_sub_ = odom_node_.subscribe("odom_world", 1,
        &DiffReplanFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());
  }

  void DiffReplanFSM::refreshOdometry()
  {
    // Drain only odometry on this same thread, ahead of queued map/timer work.
    odom_callback_queue_.callAvailable(ros::WallDuration(0));
  }

  void DiffReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_stamp_ = msg->header.stamp;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void DiffReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    last_target_change_time_ = ros::Time::now().toSec();
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

    ROS_INFO("Received search subgoal: %f, %f, %f", end_wp(0), end_wp(1), end_wp(2));
    if (planNextWaypoint(end_wp, true))
    {
      last_target_change_time_ = ros::Time::now().toSec();
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
