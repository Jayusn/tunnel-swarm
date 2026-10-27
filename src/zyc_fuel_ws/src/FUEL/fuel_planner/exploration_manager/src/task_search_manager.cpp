// 2026-07-13: 实现比赛第二阶段任务搜索、重复站点抑制、目标登记和多方向搜索恢复排序。
#include <exploration_manager/task_search_manager.h>
#include <exploration_manager/motion_direction_rules.h>
#include <exploration_manager/frontier_clearance_policy.h>
#include <exploration_manager/open_space_exit_detection.h>
#include <exploration_manager/entry_forward_policy.h>
#include <exploration_manager/route_width_policy.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <plan_env/sdf_map.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>

namespace fast_planner {

void TaskSearchManager::initialize(ros::NodeHandle& nh) {
  nh.param("mission/task_search/enabled", enabled_, true);
  nh.param("mission/task_search/curve_heading/enabled", curve_heading_enabled_, false);
  nh.param("fsm/use_diff_for_fuel_exploration",
           hybrid_constraints_enabled_, false);
  // 2026-07-16: false只关闭三类识别对任务状态切换的阻塞，不删除话题订阅和后续接入接口。
  nh.param("mission/task_search/require_stage2_detections", require_stage2_detections_, true);
  nh.param("mission/task_search/world_frame", world_frame_, std::string("world"));
  nh.param("mission/task_search/visit_spacing", visit_spacing_, 0.35);
  nh.param("mission/task_search/revisit_radius", revisit_radius_, 0.65);
  nh.param("mission/task_search/repeat_goal_radius", repeat_goal_radius_, 0.45);
  nh.param("mission/task_search/max_goal_repeats", max_goal_repeats_, 2);
  nh.param("mission/task_search/max_history_size", max_history_size_, 120);
  nh.param("mission/task_search/cruise_height", cruise_height_, 0.75);
  nh.param("mission/task_search/min_height", min_search_height_, 0.55);
  nh.param("mission/task_search/max_height", max_search_height_, 1.00);
  nh.param("mission/task_search/corridor_max_height", corridor_max_height_,
           std::numeric_limits<double>::infinity());
  // 2026-07-13: 比赛窄道采用水平优先搜索，目标高度不能随 frontier 一次跳到顶层。
  nh.param("mission/task_search/max_goal_climb", max_goal_climb_, 0.06);
  nh.param("mission/task_search/max_goal_descent", max_goal_descent_, 0.10);
  nh.param("mission/task_search/novelty_weight", novelty_weight_, 2.5);
  nh.param("mission/task_search/travel_weight", travel_weight_, 0.65);
  nh.param("mission/task_search/preferred_distance_min", preferred_distance_min_, 1.0);
  nh.param("mission/task_search/preferred_distance_max", preferred_distance_max_, 2.0);
  nh.param("mission/task_search/near_distance_weight", near_distance_weight_, 1.5);
  nh.param("mission/task_search/far_distance_weight", far_distance_weight_, 1.2);
  double direction_sector_deg = 30.0;
  nh.param("mission/task_search/direction_sector_deg", direction_sector_deg, 30.0);
  preferred_distance_min_ = std::max(0.0, preferred_distance_min_);
  preferred_distance_max_ =
      std::max(preferred_distance_min_, preferred_distance_max_);
  near_distance_weight_ = std::max(0.0, near_distance_weight_);
  far_distance_weight_ = std::max(0.0, far_distance_weight_);
  direction_sector_rad_ =
      std::max(5.0, std::min(180.0, direction_sector_deg)) * M_PI / 180.0;
  nh.param("mission/task_search/yaw_weight", yaw_weight_, 0.20);
  nh.param("mission/task_search/height_weight", height_weight_, 2.0);
  nh.param("mission/task_search/repeat_penalty", repeat_penalty_, 6.0);
  nh.param("mission/task_search/frontier_gain_weight", frontier_gain_weight_, 0.10);
  nh.param("mission/task_search/clearance_reward_enabled",
           clearance_reward_enabled_, true);
  nh.param("mission/task_search/clearance_reward_start",
           clearance_reward_start_, 0.30);
  nh.param("mission/task_search/clearance_reward_full",
           clearance_reward_full_, 0.60);
  nh.param("mission/task_search/clearance_reward_max",
           clearance_reward_max_, 5.0);
  clearance_reward_start_ = std::max(0.0, clearance_reward_start_);
  clearance_reward_full_ =
      std::max(clearance_reward_start_ + 0.05, clearance_reward_full_);
  clearance_reward_max_ = std::max(0.0, clearance_reward_max_);
  nh.param("mission/task_search/entry_forward_distance", entry_forward_distance_, 2.0);
  nh.param("mission/task_search/entry_forward_weight", entry_forward_weight_, 1.5);
  nh.param("mission/task_search/forward_viewpoint_bonus", forward_viewpoint_bonus_, 5.0);
  nh.param("mission/task_search/backward_viewpoint_penalty",
           backward_viewpoint_penalty_, 5.0);
  // 2026-07-21: 单通道任务不能把单帧frontier耗尽当成死路；默认只接受前向/侧向候选。
  nh.param("mission/task_search/prefer_motion_forward", prefer_motion_forward_, true);
  nh.param("mission/task_search/allow_search_backtrack", allow_search_backtrack_, false);
  nh.param("mission/global_no_return", global_no_return_, true);
  nh.param("mission/task_search/backward_cos_threshold", backward_cos_threshold_, -0.15);
  // 2026-07-13: 选点保持和失败冷却抑制柱体附近每 0.25s 来回切换目标。
  nh.param("mission/task_search/min_goal_hold_time", min_goal_hold_time_, 1.2);
  // 2026-07-14: 活动目标未到达且未失败时惩罚远距离换点，防止前机突然倒车撞向后机。
  nh.param("mission/task_search/goal_switch_weight", goal_switch_weight_, 1.2);
  nh.param("mission/task_search/failed_goal_radius", failed_goal_radius_, 0.55);
  nh.param("mission/task_search/failed_goal_cooldown", failed_goal_cooldown_, 2.0);
  nh.param("mission/task_search/failed_direction_threshold",
           failed_direction_threshold_, 2);
  nh.param("mission/task_search/failed_direction_pause",
           failed_direction_pause_, 3.0);
  failed_direction_threshold_ = std::max(2, failed_direction_threshold_);
  failed_direction_pause_ = std::max(0.0, failed_direction_pause_);
  nh.param("mission/task_search/inside_return_margin", inside_return_margin_, 0.10);
  nh.param("mission/task_search/entry_path_direction_grace_distance",
           entry_path_direction_grace_distance_, 1.50);
  // 2026-07-28: 正常拐弯不能继续把最近实飞切线当作唯一“前方”；累计占据地图负责识别新通道轴线。
  // 复用现有上绕高度范围和机体底部净空，不为转弯引入另一套高度配置。
  nh.param("mission/vertical_detour/enabled", turn_upper_enabled_, true);
  nh.param("mission/vertical_detour/upper_step", turn_upper_step_, 0.20);
  nh.param("mission/vertical_detour/upper_max_rise", turn_upper_max_rise_, 0.60);
  nh.param("astar/max_search_height", turn_upper_max_height_, 1.90);
  nh.param("manager/footprint_bottom_clearance", turn_bottom_clearance_, 0.20);
  // 上绕目标距障碍顶面的间距不是机体尺寸；转弯沿用实际配置的下方包络。
  turn_bottom_clearance_ = std::max(0.06, turn_bottom_clearance_);
  turn_upper_step_ = std::max(0.10,turn_upper_step_);
  turn_upper_max_rise_ = std::max(turn_upper_step_,turn_upper_max_rise_);
  nh.param("mission/task_search/recovery/occupancy_turn_enabled",
           recovery_occupancy_turn_enabled_, true);
  nh.param("mission/task_search/recovery/turn_probe_length",
           recovery_turn_probe_length_, 1.50);
  nh.param("mission/task_search/recovery/turn_probe_step",
           recovery_turn_probe_step_, 0.10);
  nh.param("mission/task_search/recovery/turn_min_angle_deg",
           recovery_turn_min_angle_deg_, 30.0);
  nh.param("mission/task_search/recovery/turn_max_angle_deg",
           recovery_turn_max_angle_deg_, 120.0);
  nh.param("mission/task_search/recovery/turn_confirmation_count",
           recovery_turn_confirmation_count_, 1);
  nh.param("mission/task_search/recovery/turn_confirmation_min_interval",
           recovery_turn_confirmation_min_interval_, 0.15);
  nh.param("mission/task_search/recovery/turn_confirmation_angle_deg",
           recovery_turn_confirmation_angle_deg_, 15.0);
  nh.param("mission/task_search/recovery/turn_confirmation_accumulation_window",
           recovery_turn_confirmation_accumulation_window_, 8.0);
  nh.param("mission/task_search/recovery/turn_no_return_margin",
           recovery_turn_no_return_margin_, 0.20);
  nh.param("mission/task_search/recovery/turn_yaw_release_angle_deg",
           recovery_turn_yaw_release_angle_deg_, 15.0);
  recovery_turn_confirmation_count_ =
      std::max(1, recovery_turn_confirmation_count_);
  recovery_turn_confirmation_min_interval_ =
      std::max(0.05, recovery_turn_confirmation_min_interval_);
  recovery_turn_confirmation_angle_deg_ =
      std::max(3.0, std::min(30.0, recovery_turn_confirmation_angle_deg_));
  recovery_turn_confirmation_accumulation_window_ =
      std::max(0.5, recovery_turn_confirmation_accumulation_window_);

  // 出口只根据局部受限空间向双侧开阔区域的过渡判定。
  nh.param("mission/task_search/exit/enabled", exit_detection_enabled_, true);
  nh.param("mission/task_search/exit/auto_crossing_enabled", exit_auto_crossing_enabled_, true);
  nh.param("mission/task_search/exit/inference_period", exit_inference_period_, 0.50);
  nh.param("mission/task_search/exit/stability_radius", exit_candidate_stability_radius_, 0.30);
  nh.param("mission/task_search/exit/confirmation_count", exit_confirmation_count_, 3);
  nh.param("mission/task_search/exit/confirmation_max_distance",
           exit_confirmation_max_distance_, 1.00);
  nh.param("mission/task_search/exit/confirmation_angle_deg",
           exit_candidate_angle_tolerance_deg_, 15.0);
  nh.param("mission/task_search/exit/search_max_forward", exit_search_max_forward_, 2.0);
  nh.param("mission/task_search/exit/min_open_width", exit_min_open_width_, 2.0);
  nh.param("mission/task_search/exit/min_open_ratio", exit_min_open_ratio_, 0.80);
  nh.param("mission/task_search/exit/sector_ray_count", exit_sector_ray_count_, 11);
  nh.param("mission/task_search/exit/sector_half_angle_deg", exit_sector_half_angle_deg_, 20.0);
  nh.param("mission/task_search/exit/inside_length", exit_inside_length_, 0.90);
  nh.param("mission/task_search/exit/outside_length", exit_outside_length_, 1.20);
  nh.param("mission/task_search/exit/footprint_half_height", exit_footprint_half_height_, 0.15);
  nh.param("mission/task_search/exit/verification_yaw_hold_time",
           exit_verification_yaw_hold_time_, 2.5);
  nh.param("mission/task_search/exit/standoff", exit_standoff_, 0.45);
  nh.param("mission/task_search/exit/arrive_distance", exit_arrive_distance_, 0.50);
  nh.param("mission/task_search/exit/portal_arrive_distance", exit_portal_arrive_distance_, 0.10);
  nh.param("mission/task_search/exit/candidate_max_behind_distance",
           exit_candidate_max_behind_distance_, 0.35);
  // 推进方向与转弯共用，短回撤不能改变出口的内外方向。
  nh.param("mission/task_search/exit/progress_reverse_reject_deg",
           exit_progress_reverse_reject_deg_, 120.0);
  nh.param("mission/task_search/exit/pending_observation_min_hits",
           exit_pending_observation_min_hits_, 3);
  nh.param("mission/task_search/exit/pending_observation_lateral_offset",
           exit_pending_observation_lateral_offset_, 0.25);
  nh.param("mission/task_search/exit/pending_observation_min_move",
           exit_pending_observation_min_move_, 0.22);
  nh.param("mission/task_search/exit/cross_step", exit_cross_step_, 0.35);
  nh.param("mission/task_search/exit/cross_target_distance", exit_cross_target_distance_, 0.90);
  nh.param("mission/task_search/exit/cross_confirm_distance", exit_cross_confirm_distance_, 0.45);
  nh.param("mission/task_search/exit/footprint_extra_radius", exit_footprint_extra_radius_, 0.22);
  nh.param("mission/task_search/exit/footprint_samples", exit_footprint_samples_, 8);
  nh.param("mission/task_search/exit/scan_radius", scan_radius_, 0.35);
  nh.param("mission/task_search/exit/scan_dwell_time", scan_dwell_time_, 1.2);
  nh.param("mission/task_search/exit/external_landing_planner",
           external_landing_planner_, false);
  nh.param("mission/task_search/exit/require_final_landing_marker",
           require_final_landing_marker_, true);
  // 2026-07-20: 最终出口确认且路径足够远后，普通frontier不得把机体明显拉离出口；
  // 出口附近保留局部绕障和姿态调整空间。
  nh.param("mission/task_search/exit/frontier_local_adjust_radius",
           exit_frontier_local_adjust_radius_, 1.20);
  nh.param("mission/task_search/exit/frontier_max_distance_increase",
           exit_frontier_max_distance_increase_, 0.45);
  nh.param("mission/task_search/exit/final_landing_marker_confirmation_count",
           final_landing_marker_confirmation_count_, 3);
  nh.param("mission/task_search/exit/final_landing_marker_consistency_radius",
           final_landing_marker_consistency_radius_, 0.40);
  nh.param("mission/task_search/exit/landing_approach_height", landing_approach_height_, 0.70);
  nh.param("mission/task_search/exit/landing_trigger_distance", landing_trigger_distance_, 0.35);
  nh.param("mission/task_search/exit/landing_column_bottom_clearance",
           landing_column_bottom_clearance_, 0.20);
  nh.param("mission/task_search/exit/landing_column_step", landing_column_step_, 0.10);

  nh.param("mission/task_search/exit/rc_trigger/enabled",
           rc_search_landing_enabled_, false);
  nh.param("mission/task_search/exit/rc_trigger/topic",
           rc_search_landing_topic_, std::string("/UAV0/mavros/rc/in"));
  nh.param("mission/task_search/exit/rc_trigger/channel_index",
           rc_search_landing_channel_, 8);
  nh.param("mission/task_search/exit/rc_trigger/low_pwm",
           rc_search_landing_low_pwm_, 1300);
  nh.param("mission/task_search/exit/rc_trigger/high_pwm",
           rc_search_landing_high_pwm_, 1800);
  nh.param("mission/task_search/exit/rc_trigger/hold_sec",
           rc_search_landing_hold_sec_, 0.5);
  rc_search_landing_channel_ = std::max(0, rc_search_landing_channel_);
  rc_search_landing_hold_sec_ = std::max(0.1, rc_search_landing_hold_sec_);

  std::string color_topic, qrcode_topic, thermal_topic, final_landing_marker_topic;
  nh.param("mission/task_search/color_topic", color_topic, std::string("/mission/detection/color"));
  nh.param("mission/task_search/qrcode_topic", qrcode_topic,
           std::string("/mission/detection/qrcode"));
  nh.param("mission/task_search/thermal_topic", thermal_topic,
           std::string("/mission/detection/thermal"));
  nh.param("mission/task_search/final_landing_marker_topic", final_landing_marker_topic,
           std::string("/mission/detection/final_aruco"));
  color_detection_sub_ = nh.subscribe(color_topic, 5, &TaskSearchManager::colorDetectionCallback, this);
  qrcode_detection_sub_ =
      nh.subscribe(qrcode_topic, 5, &TaskSearchManager::qrcodeDetectionCallback, this);
  thermal_detection_sub_ =
      nh.subscribe(thermal_topic, 5, &TaskSearchManager::thermalDetectionCallback, this);
  final_landing_marker_sub_ = nh.subscribe(
      final_landing_marker_topic, 5,
      &TaskSearchManager::finalLandingMarkerCallback, this);
  // 2026-07-23: 直接订阅FAST-LIO已外参校正的IMU-body点云，避免再经漂移世界z筛选墙体。
  if (rc_search_landing_enabled_) {
    rc_search_landing_sub_ = nh.subscribe(
        rc_search_landing_topic_, 10,
        &TaskSearchManager::rcSearchLandingCallback, this);
  }
  rc_land_active_sub_ = nh.subscribe("/mission/rc_land_active", 1,
      &TaskSearchManager::rcLandActiveCallback, this);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/mission/search_markers", 10, true);
  status_pub_ = nh.advertise<std_msgs::String>("/mission/task_status", 2, true);
  exit_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mission/exit_candidate", 2, true);
  // 2026-07-28: 与可撤销exit_candidate分离；后机只能接收最终锁存门，禁止追第一弯墙端候选。
  final_exit_pose_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("/mission/final_exit", 1, true);
  landing_target_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("/mission/landing_target", 2, true);
  landing_request_pub_ = nh.advertise<std_msgs::Bool>("/mission/landing_request", 2, true);
  publishLandingRequest(false);

  ROS_WARN("[task_search] stage-2 task search enabled=%d detections_required=%d, "
           "z=[%.2f, %.2f], revisit=%.2f, route_constraints=%s.",
           static_cast<int>(enabled_), static_cast<int>(require_stage2_detections_),
           min_search_height_, max_search_height_, revisit_radius_,
           hybrid_constraints_enabled_ ? "FUEL_DIFF" : "NATIVE_FUEL");
  ROS_WARN("[exit_mission] open-space detector enabled=%d auto_crossing=%d ahead_width>%.2fm ratio>=%.2f "
           "confirm=%d period=%.2fs near=%.2fm; fresh map only, turn exclusion enabled.",
           static_cast<int>(exit_detection_enabled_), static_cast<int>(exit_auto_crossing_enabled_),
           exit_min_open_width_, exit_min_open_ratio_,
           exit_confirmation_count_, exit_inference_period_, exit_confirmation_max_distance_);
  ROS_WARN("[exit_mission] RC search-landing trigger enabled=%d topic=%s CH%d "
           "low<=%d high>=%d hold=%.2fs; startup requires LOW before HIGH.",
           static_cast<int>(rc_search_landing_enabled_),
           rc_search_landing_topic_.c_str(), rc_search_landing_channel_ + 1,
           rc_search_landing_low_pwm_, rc_search_landing_high_pwm_,
           rc_search_landing_hold_sec_);
}

void TaskSearchManager::rcLandActiveCallback(const std_msgs::BoolConstPtr& msg) {
  if (!msg->data || rc_land_active_) return;
  rc_land_active_ = true;
  rc_search_landing_start_requested_ = false;
  rc_search_landing_armed_ = false;
  active_goal_valid_ = false;
  publishSearchState();
  ROS_ERROR("[RC_LAND] CH10 cancelled exit approach/crossing and landing search.");
}

void TaskSearchManager::rcSearchLandingCallback(
    const mavros_msgs::RCInConstPtr& msg) {
  if (rc_land_active_ || !enabled_ || !rc_search_landing_enabled_ ||
      rc_search_landing_triggered_) {
    return;
  }
  if (msg->channels.size() <=
      static_cast<std::size_t>(rc_search_landing_channel_)) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] RC trigger waiting for CH%d; received only %zu channels.",
                      rc_search_landing_channel_ + 1, msg->channels.size());
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }

  const int pwm = static_cast<int>(msg->channels[rc_search_landing_channel_]);
  if (pwm <= rc_search_landing_low_pwm_) {
    if (!rc_search_landing_armed_) {
      ROS_WARN("[exit_mission] RC CH%d LOW observed (%d us); search-landing trigger armed.",
               rc_search_landing_channel_ + 1, pwm);
    }
    rc_search_landing_armed_ = true;
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }

  if (pwm < rc_search_landing_high_pwm_) {
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }
  if (!rc_search_landing_armed_) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] ignore RC CH%d HIGH (%d us): move switch LOW first.",
                      rc_search_landing_channel_ + 1, pwm);
    return;
  }

  const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now()
                                                    : msg->header.stamp;
  if (rc_search_landing_high_since_.isZero()) {
    rc_search_landing_high_since_ = now;
    return;
  }
  if ((now - rc_search_landing_high_since_).toSec() <
      rc_search_landing_hold_sec_) {
    return;
  }
  activateRcSearchLanding(now);
}

void TaskSearchManager::activateRcSearchLanding(const ros::Time& stamp) {
  if (mission_stage_ != SEARCH_CORRIDOR) {
    ROS_WARN("[exit_mission] ignore RC search-landing trigger: mission stage is already %d.",
             static_cast<int>(mission_stage_));
    rc_search_landing_triggered_ = true;
    rc_search_landing_armed_ = false;
    return;
  }
  if (!latest_robot_pose_valid_) {
    ROS_ERROR_THROTTLE(1.0,
                       "[exit_mission] reject RC search-landing trigger: robot pose unavailable.");
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }

  const ros::Time now = stamp.isZero() ? ros::Time::now() : stamp;
  if (exit_candidate_confirmed_ && exit_portal_outside_verified_) {
    // 已有确认出口时，CH9复用自动穿越；自动开关关闭也允许人工启动。
    beginExitCrossing(now);
    rc_search_landing_triggered_ = true;
    rc_search_landing_start_requested_ = true;
    rc_search_landing_armed_ = false;
    rc_search_landing_high_since_ = ros::Time(0);
    publishSearchState();
    ROS_ERROR("[exit_mission] RC CH%d TRIGGERED crossing through the confirmed exit; "
              "landing search starts only after crossing completes.",
              rc_search_landing_channel_ + 1);
    return;
  }
  const Eigen::Vector2d outward(std::cos(latest_robot_yaw_),
                               std::sin(latest_robot_yaw_));
  mission_stage_ = SEARCH_OUTSIDE_LANDING;
  mission_stage_start_ = now;
  active_goal_valid_ = false;
  search_exhausted_since_ = ros::Time(0);
  outside_search_anchor_ = latest_robot_pos_;
  outside_search_anchor_.z() = cruise_height_;

  // 人工拨杆表示 UAV0 已经位于出口外：当前位置兼作双机接力的最终出口锚点。
  exit_candidate_ = latest_robot_pos_;
  exit_portal_center_ = latest_robot_pos_;
  exit_outward_direction_ = outward;
  exit_candidate_confirmed_ = true;
  exit_portal_locked_ = true;
  exit_portal_outside_verified_ = true;

  geometry_msgs::PoseStamped final_exit_pose;
  final_exit_pose.header.stamp = now;
  final_exit_pose.header.frame_id = world_frame_;
  final_exit_pose.pose.position.x = latest_robot_pos_.x();
  final_exit_pose.pose.position.y = latest_robot_pos_.y();
  final_exit_pose.pose.position.z = latest_robot_pos_.z();
  final_exit_pose.pose.orientation.w = std::cos(0.5 * latest_robot_yaw_);
  final_exit_pose.pose.orientation.z = std::sin(0.5 * latest_robot_yaw_);
  final_exit_pose_pub_.publish(final_exit_pose);
  exit_pose_pub_.publish(final_exit_pose);

  rc_search_landing_triggered_ = true;
  rc_search_landing_start_requested_ = true;
  rc_search_landing_armed_ = false;
  rc_search_landing_high_since_ = ros::Time(0);
  publishSearchState();
  ROS_ERROR("[exit_mission] RC CH%d TRIGGERED search landing at current outside pose "
            "(%.2f, %.2f, %.2f), yaw=%.1fdeg; automatic exit detection bypassed.",
            rc_search_landing_channel_ + 1, latest_robot_pos_.x(),
            latest_robot_pos_.y(), latest_robot_pos_.z(),
            latest_robot_yaw_ * 180.0 / M_PI);
}

bool TaskSearchManager::consumeRcSearchLandingStartRequest() {
  if (!rc_search_landing_start_requested_) return false;
  rc_search_landing_start_requested_ = false;
  return true;
}

void TaskSearchManager::setMap(const std::shared_ptr<SDFMap>& map) {
  // 2026-07-13: 与 exploration_manager 共用同一累计 SDFMap，避免另订阅实时点云造成无记忆判断。
  sdf_map_ = map;
}

void TaskSearchManager::setCorridorFrame(const Eigen::Vector3d& origin,
                                         const Eigen::Vector3d& inside_dir) {
  // 入口内外是任务开始时的一次性定义：初始机头前方为门内，后方为门外。
  // corridor_search_manager 会周期性重发锁存的 workspace_lock；如果每次都覆盖这里，
  // 通道内已经随转弯更新的 stable_progress_direction_ 会被不断拉回初始航向。
  if (corridor_frame_received_) return;
  corridor_origin_ = origin;
  corridor_dir_ = inside_dir;
  corridor_dir_.z() = 0.0;
  if (corridor_dir_.head<2>().norm() < 1e-3) corridor_dir_ = Eigen::Vector3d::UnitX();
  corridor_dir_.normalize();
  entry_inside_yaw_ = std::atan2(corridor_dir_.y(), corridor_dir_.x());
  segment_origin_ = corridor_origin_.head<2>();
  segment_direction_ = corridor_dir_.head<2>().normalized();
  segment_high_water_ = 0.0;
  stable_progress_direction_ = corridor_dir_.head<2>().normalized();
  stable_progress_direction_valid_ = true;
  turn_detection_direction_ = stable_progress_direction_;
  corridor_frame_received_ = true;
  ROS_WARN("[task_search] entry frame frozen once: origin=(%.2f,%.2f) inside_yaw=%.1fdeg; "
           "initial heading forward is permanently INSIDE.",
           corridor_origin_.x(), corridor_origin_.y(),
           std::atan2(corridor_dir_.y(), corridor_dir_.x()) * 180.0 / M_PI);
}

bool TaskSearchManager::updateExitDetectionOnly(
    const Eigen::Vector3d& pos, double yaw,
    geometry_msgs::PoseStamped& detected_exit, bool observe_turns) {
  if (!corridor_frame_received_) {
    setCorridorFrame(pos, Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0));
    corridor_entry_crossed_ = true;
    ROS_INFO("[exit_detection_test] start INSIDE_CORRIDOR at first odometry; "
             "forward yaw=%.1fdeg, detection only.", yaw * 180.0 / M_PI);
  }
  updateRobotPose(pos, yaw);
  // 手持全程测试由真实地图更新通道轴线；无需执行入口阶段或原地转向轨迹。
  // 正式航点飞行由规划器/FSM执行转向并确认完成，不能等手持者先转机头，
  // 也不能在里程计回调中提前清除仍在执行的转向锁存。
  if (observe_turns) {
    Eigen::Vector3d mapped_direction;
    mappedCorridorDirection(yaw, mapped_direction, false, true);
    if (turnYawAlignmentPending() &&
        !corridorYawCorrectionDirection(yaw, mapped_direction))
      completeTurnYawAlignment();
  }
  // 新转弯会撤销旧出口证据；待机头对准且走出既有过渡区后恢复出口检测。
  updateExitCandidate(pos);
  if (!exit_candidate_confirmed_) return false;
  const double exit_yaw = std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
  detected_exit.header.stamp = ros::Time::now();
  detected_exit.header.frame_id = world_frame_;
  detected_exit.pose.position.x = exit_portal_center_.x();
  detected_exit.pose.position.y = exit_portal_center_.y();
  detected_exit.pose.position.z = exit_portal_center_.z();
  detected_exit.pose.orientation.x = detected_exit.pose.orientation.y = 0.0;
  detected_exit.pose.orientation.z = std::sin(0.5 * exit_yaw);
  detected_exit.pose.orientation.w = std::cos(0.5 * exit_yaw);
  return true;
}

void TaskSearchManager::updateRobotPose(const Eigen::Vector3d& pos, double yaw) {
  if (rc_land_active_ || !enabled_) return;
  {
    // 2026-07-23: 点云回调只借用连续里程计的XY和yaw落入短时局部平面；世界z有意不参与。
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    latest_robot_pos_ = pos;
    latest_robot_yaw_ = yaw;
    latest_robot_pose_valid_ = true;
    ++latest_robot_pose_sequence_;
  }
  if (segment_direction_.norm() > 1e-3) {
    const double progress =
        (pos.head<2>() - segment_origin_).dot(segment_direction_.normalized());
    if (!transition_active_)
      segment_high_water_ = std::max(segment_high_water_, progress);
    else if ((pos.head<2>() - transition_anchor_).dot(transition_outgoing_direction_) >= 0.60) {
      transition_active_ = false;
      segment_origin_ = transition_anchor_;
      segment_direction_ = transition_outgoing_direction_;
      segment_high_water_ = std::max(
          segment_high_water_,
          (pos.head<2>() - segment_origin_).dot(segment_direction_));
      ROS_INFO("[task_search] turn transition released after 0.60m forward progress.");
    }
  }
  // 2026-07-23: 用入口门平面只做一次“起点外 -> 通道内”锁存；之后不再根据局部几何
  // 反复重定义内外。0.35m滞回避免刚过门平面时的里程计抖动。
  if (corridor_frame_received_ && !corridor_entry_crossed_) {
    const double entry_progress = (pos - corridor_origin_).dot(corridor_dir_);
    if (entry_progress >= std::max(0.35, inside_return_margin_ + 0.25)) {
      corridor_entry_crossed_ = true;
      ROS_ERROR("[mission_region] START_OUTSIDE -> INSIDE_CORRIDOR, "
                "entry plane crossed by %.2fm; region is now latched INSIDE and "
                "the entry workspace lock is retired.",
                entry_progress);
    }
  }
  if (visited_positions_.empty() ||
      (pos.head<2>() - visited_positions_.back().head<2>()).norm() >= visit_spacing_) {
    // 实际位移方向不能重定义通道正方向。向右前方斜飞只是局部避障动作；若在这里
    // 跟随里程计切线，恢复器会把斜向误当成新的“直行”，随后无法回到原通道轴线。
    // 通道方向仅由地图墙体更新，不使用实际位移或绕障切线。
    visited_positions_.push_back(pos);
    while (static_cast<int>(visited_positions_.size()) > max_history_size_)
      visited_positions_.pop_front();
    publishSearchState();
  }
  updateCurveHeading();
  // 2026-07-13: 出口候选在第二阶段持续后台更新，frontier 耗尽时无需临时等待重新建图。
  updateExitCandidate(pos);

  // 2026-07-21: 拓扑远端和门框确认只允许触发穿门状态机；通道内任何位置均不得直接发布降落。

  if (mission_stage_ == APPROACH_LANDING && final_landing_marker_.found) {
    const Eigen::Vector3d marker(final_landing_marker_.pose.pose.position.x,
                                 final_landing_marker_.pose.pose.position.y,
                                 final_landing_marker_.pose.pose.position.z);
    const Eigen::Vector3d approach(marker.x(), marker.y(),
                                   marker.z() + landing_approach_height_);
    if ((pos.head<2>() - marker.head<2>()).norm() <= landing_trigger_distance_ &&
        landingColumnSafe(marker, approach)) {
      mission_stage_ = LANDING;
      mission_stage_start_ = ros::Time::now();
      publishSearchState();
      publishLandingRequest(true);
      ROS_ERROR("[exit_mission] landing ArUco aligned at %.2f %.2f; "
                "landing request published.", marker.x(), marker.y());
    }
  }
  (void)yaw;
}

void TaskSearchManager::freezeExplorationInitialYaw(double yaw) {
  if (exploration_initial_yaw_frozen_) return;
  if (!std::isfinite(yaw)) return;
  exploration_initial_yaw_ = yaw;
  exploration_initial_yaw_frozen_ = true;
  ROS_INFO("[fuel_diff] exploration initial yaw frozen at %.1fdeg.",
           exploration_initial_yaw_ * 180.0 / M_PI);
}

void TaskSearchManager::fillExplorationConstraint(
    quadrotor_msgs::ExplorationMotionConstraint& msg) const {
  msg.enabled = corridor_frame_received_ || exploration_initial_yaw_frozen_;
  msg.exploration_initial_yaw = exploration_initial_yaw_;
  msg.entry_inside_yaw = entry_inside_yaw_;
  msg.entry_plane_active = corridor_frame_received_ && !corridor_entry_crossed_;
  msg.entry_origin.x = corridor_origin_.x();
  msg.entry_origin.y = corridor_origin_.y();
  msg.entry_origin.z = 0.0;
  msg.entry_direction.x = corridor_dir_.x();
  msg.entry_direction.y = corridor_dir_.y();
  msg.entry_direction.z = 0.0;
  msg.entry_back_margin = std::max(0.0, inside_return_margin_);
  msg.segment_origin.x = segment_origin_.x();
  msg.segment_origin.y = segment_origin_.y();
  msg.segment_origin.z = 0.0;
  msg.segment_direction.x = segment_direction_.x();
  msg.segment_direction.y = segment_direction_.y();
  msg.segment_direction.z = 0.0;
  msg.segment_high_water = segment_high_water_;
  msg.segment_regression_margin = 0.25;
  msg.transition_active = transition_active_;
  msg.transition_anchor.x = transition_anchor_.x();
  msg.transition_anchor.y = transition_anchor_.y();
  msg.transition_anchor.z = 0.0;
  msg.transition_incoming_direction.x = transition_incoming_direction_.x();
  msg.transition_incoming_direction.y = transition_incoming_direction_.y();
  msg.transition_incoming_direction.z = 0.0;
  msg.transition_outgoing_direction.x = transition_outgoing_direction_.x();
  msg.transition_outgoing_direction.y = transition_outgoing_direction_.y();
  msg.transition_outgoing_direction.z = 0.0;
  msg.transition_old_high_water = transition_old_high_water_;
  msg.transition_regression_margin = 0.45;
  msg.transition_radius = 0.80;
  msg.transition_corridor_half_width = 1.25;
  msg.completed_gates.clear();
  for (const auto& gate : completed_gates_) {
    quadrotor_msgs::ExplorationGate out;
    out.sequence = gate.sequence;
    out.center.x = gate.center.x();
    out.center.y = gate.center.y();
    out.center.z = 0.0;
    out.normal.x = gate.normal.x();
    out.normal.y = gate.normal.y();
    out.normal.z = 0.0;
    out.left_extent = gate.left_extent;
    out.right_extent = gate.right_extent;
    out.thickness = gate.thickness;
    msg.completed_gates.push_back(out);
  }
}

void TaskSearchManager::recordExplorationReached(const Eigen::Vector3d& goal) {
  recordSelectedGoal(goal);
  clearActiveGoal();
}

void TaskSearchManager::clearPendingTurnEvidence() {
  pending_turn_confirmations_ = 0;
  pending_turn_probe_origin_valid_ = false;
  pending_turn_last_evidence_ = ros::Time(0);
  pending_turn_pose_sequence_ = 0;
}

bool TaskSearchManager::confirmCorridorTurnEvidence(
    const Eigen::Vector3d& turn_direction) {
  const ros::Time now = ros::Time::now();
  if (turn_direction.head<2>().norm() < 1e-3) return false;

  const Eigen::Vector2d observed = turn_direction.head<2>().normalized();
  const double elapsed = pending_turn_last_evidence_.isZero()
                             ? std::numeric_limits<double>::infinity()
                             : (now - pending_turn_last_evidence_).toSec();
  const double angular_tolerance =
      recovery_turn_confirmation_angle_deg_ * M_PI / 180.0;
  const bool direction_consistent =
      pending_turn_confirmations_ > 0 &&
      std::acos(std::max(-1.0, std::min(1.0,
                                      pending_turn_direction_.dot(observed)))) <=
          angular_tolerance;
  // 在时间窗内累计同方向证据，不要求相邻规划周期连续命中。中间地图暂时缺失
  // 不清票；只有方向明显改变或时间窗过期才重新计数。
  if (!direction_consistent ||
      elapsed > recovery_turn_confirmation_accumulation_window_) {
    pending_turn_direction_ = observed;
    pending_turn_probe_origin_ = visited_positions_.empty()
                                     ? latest_robot_pos_
                                     : visited_positions_.back();
    pending_turn_probe_origin_valid_ = true;
    pending_turn_confirmations_ = 1;
    pending_turn_last_evidence_ = now;
    pending_turn_pose_sequence_ = latest_robot_pose_sequence_;
  } else if (elapsed >= recovery_turn_confirmation_min_interval_ &&
             latest_robot_pose_sequence_ != pending_turn_pose_sequence_) {
    pending_turn_direction_ =
        (pending_turn_direction_ * pending_turn_confirmations_ + observed).normalized();
    ++pending_turn_confirmations_;
    pending_turn_last_evidence_ = now;
    pending_turn_pose_sequence_ = latest_robot_pose_sequence_;
  }

  if (pending_turn_confirmations_ < recovery_turn_confirmation_count_) {
    ROS_WARN_THROTTLE(0.3,
                      "[task_search] mapped turn pending %d/%d yaw=%.1fdeg; "
                      "freeze probe=(%.2f,%.2f) and keep current corridor direction.",
                      pending_turn_confirmations_, recovery_turn_confirmation_count_,
                      std::atan2(observed.y(), observed.x()) * 180.0 / M_PI,
                      pending_turn_probe_origin_.x(), pending_turn_probe_origin_.y());
    return false;
  }
  clearPendingTurnEvidence();
  return true;
}

Eigen::Vector2d TaskSearchManager::stableProgressDirection() const {
  if (stable_progress_direction_valid_ && stable_progress_direction_.norm() > 1e-3)
    return stable_progress_direction_.normalized();
  if (corridor_frame_received_ && corridor_dir_.head<2>().norm() > 1e-3)
    return corridor_dir_.head<2>().normalized();
  return Eigen::Vector2d(std::cos(latest_robot_yaw_), std::sin(latest_robot_yaw_));
}

Eigen::Vector2d TaskSearchManager::turnDetectionDirection() const {
  if (turn_detection_direction_.norm() > 1e-3)
    return turn_detection_direction_.normalized();
  return stableProgressDirection();
}

void TaskSearchManager::updateCurveHeading() {
  if (!curve_heading_enabled_ || !sdf_map_ || !corridor_entry_crossed_ ||
      entry_forward::needsAdvance((latest_robot_pos_-corridor_origin_).dot(corridor_dir_)) ||
      mission_stage_ != SEARCH_CORRIDOR ||
      transition_active_ || turn_yaw_follow_latch_.active()) {
    curve_heading_active_ = false;
    return;
  }
  const Eigen::Vector3d origin = latest_robot_pos_;
  const auto sequence = sdf_map_->getUpdateSequence();
  if (curve_sample_valid_ && sequence == curve_map_sequence_ &&
      (origin - curve_origin_).norm() < 0.10) return;
  const bool following_curve = curve_heading_active_;
  curve_map_sequence_ = sequence;
  curve_sample_valid_ = true;
  curve_origin_ = origin;
  curve_forward_ = stableProgressDirection();
  const Eigen::Vector2d lateral(-curve_forward_.y(), curve_forward_.x());
  const double resolution = sdf_map_->getResolution();
  const double step = std::max(0.04, 0.5*resolution);
  std::vector<Eigen::Vector2d> left, right;
  // 内弯墙在旧轴前向切面上的长度短，近后方墙体也参与连续轮廓拟合。
  for (int section=0; section<=12; ++section) {
    const double x=-0.60+0.15*section;
    for (double sign : {1.0,-1.0}) {
      for (double distance=step; distance<=2.10; distance+=step) {
        Eigen::Vector3d point = origin;
        point.head<2>() += x*curve_forward_ + sign*distance*lateral;
        int hits=0;
        double min_z=1e9, max_z=-1e9;
        for (double dz=-0.45; dz<=0.30+1e-6; dz+=std::max(0.05,resolution)) {
          point.z()=origin.z()+dz;
          if (sdf_map_->isInMap(point) &&
              sdf_map_->getOccupancy(point)==SDFMap::OCCUPIED) {
            ++hits; min_z=std::min(min_z,dz); max_z=std::max(max_z,dz);
          }
        }
        // 看竖向墙体而非单个水平切面，悬挂物仍交给位置避障处理。
        if (hits>=3 && max_z-min_z>=0.30-1e-6) {
          (sign>0 ? left : right).emplace_back(x,sign*distance);
          break;
        }
      }
    }
  }
  corridor_curve::Result geometry;
  const bool fitted = corridor_curve::infer(
      left,right,std::max(0.06,0.8*resolution),geometry);
  // 首次局部曲线拟合不改变旧基准；连续双墙证据才用当前位置切线交接。
  // 前视0.35m的机头参考独立计算，不把前视角度当成旧通道方向。
  if (fitted && following_curve) {
    const double local_yaw=std::atan2(curve_forward_.y(),curve_forward_.x())+
                           geometry.yaw(0.0);
    turn_detection_direction_=Eigen::Vector2d(std::cos(local_yaw),std::sin(local_yaw));
  }
  curve_heading_active_ = fitted &&
      (geometry.curved || (following_curve &&
                          std::fabs(geometry.yaw(0.35))>M_PI/180.0));
  if (!curve_heading_active_) return;
  curve_geometry_=geometry;
  const double heading=std::atan2(curve_forward_.y(),curve_forward_.x())+
                       geometry.yaw(0.35);
  const Eigen::Vector2d direction(std::cos(heading),std::sin(heading));
  // 滚动前进轴不能把已发生的回退清零；保留原有0.25m回退容差。
  const double regression=std::max(0.0,segment_high_water_-
      (origin.head<2>()-segment_origin_).dot(segment_direction_));
  segment_origin_=origin.head<2>();
  segment_direction_=direction;
  segment_high_water_=regression;
  stable_progress_direction_=direction;
  stable_progress_direction_valid_=true;
  // 弧形只滚动任务推进方向；离散转弯另有已确认通道基准。
  if (hybrid_constraints_enabled_) exploration_initial_yaw_=heading;
  clearPendingTurnEvidence();
  ROS_INFO_THROTTLE(0.3,
      "[corridor_curve] bilateral walls at (%.2f,%.2f), yaw=%.1fdeg; "
      "advance while following channel, no turn-in-place.",
      origin.x(),origin.y(),heading*180.0/M_PI);
}

bool TaskSearchManager::corridorCurveYaw(
    const Eigen::Vector3d& position,double& yaw) const {
  if (!curve_heading_active_) return false;
  const double forward=(position.head<2>()-curve_origin_.head<2>()).dot(curve_forward_);
  // 只使用已拟合的前方墙体范围；横向绕障位移不改变通道朝向。
  yaw=std::atan2(curve_forward_.y(),curve_forward_.x())+
      curve_geometry_.yaw(std::max(0.0,std::min(1.0,forward+0.35)));
  return true;
}

bool TaskSearchManager::shouldReplanCurveHeading(
    const Eigen::Vector3d& position,double reference_yaw) const {
  double heading;
  return corridorCurveYaw(position,heading) &&
      std::fabs(wrapYaw(heading-reference_yaw)) >
          recovery_turn_yaw_release_angle_deg_*M_PI/180.0;
}

bool TaskSearchManager::corridorYawCorrectionDirection(
    double cur_yaw, Eigen::Vector3d& direction) const {
  if (!stable_progress_direction_valid_ && !corridor_frame_received_) return false;
  const Eigen::Vector2d stable = curve_heading_active_
                                    ? stableProgressDirection()
                                    : turnDetectionDirection();
  const double correction_threshold =
      std::max(0.0, recovery_turn_yaw_release_angle_deg_) * M_PI / 180.0;
  if (!task_search::corridorYawCorrectionNeeded(
          stable, cur_yaw, correction_threshold))
    return false;
  direction = Eigen::Vector3d(stable.x(), stable.y(), 0.0);
  return true;
}

bool TaskSearchManager::corridorMotionDirectionMatches(
    const Eigen::Vector2d& motion, Eigen::Vector3d& corridor_direction) const {
  if (motion.norm() < 1e-3 || !corridor_frame_received_) return false;
  const Eigen::Vector2d observed = motion.normalized();
  const double tolerance =
      std::max(3.0, std::min(30.0, recovery_turn_yaw_release_angle_deg_)) *
      M_PI / 180.0;
  const auto aligned = [&](const Eigen::Vector2d& direction) {
    return direction.norm() > 1e-3 &&
           std::acos(std::max(-1.0, std::min(1.0,
                                               direction.normalized().dot(observed)))) <=
               tolerance;
  };

  const Eigen::Vector2d stable = stableProgressDirection();
  if (aligned(stable)) {
    corridor_direction = Eigen::Vector3d(stable.x(), stable.y(), 0.0);
    return true;
  }

  // stableProgressDirection 仍指向旧段时，复用累计地图的无副作用拐弯探测；
  // 只有确认旧轴终止且新分支延伸方向与轨迹一致，才开放 yaw 跟随。
  Eigen::Vector3d mapped_direction;
  double forward_free_length = 0.0;
  double turn_free_length = 0.0;
  const Eigen::Vector2d turn_base = turnDetectionDirection();
  if (!inferOccupancyTurnDirection(
          Eigen::Vector3d(turn_base.x(), turn_base.y(), 0.0), mapped_direction,
          forward_free_length, turn_free_length) ||
      !aligned(mapped_direction.head<2>()))
    return false;
  corridor_direction = mapped_direction;
  return true;
}




double TaskSearchManager::minDistance2D(
    const Eigen::Vector3d& point, const std::deque<Eigen::Vector3d>& history) const {
  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto& item : history)
    min_dist = std::min(min_dist, (point.head<2>() - item.head<2>()).norm());
  return history.empty() ? 100.0 : min_dist;
}

double TaskSearchManager::wrapYaw(double yaw) const {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

double TaskSearchManager::knownHorizontalClearance(
    const Eigen::Vector3d& point) const {
  if (!sdf_map_ || !sdf_map_->isInMap(point) ||
      sdf_map_->getOccupancy(point) != SDFMap::FREE)
    return 0.0;
  constexpr int kDirections = 16;
  const double step = std::max(0.05, sdf_map_->getResolution());
  return frontier_clearance::minimumKnownRadialClearance(
      step, clearance_reward_full_, kDirections,
      [&](int direction, double distance) {
        const double angle = 2.0 * M_PI * static_cast<double>(direction) /
                             static_cast<double>(kDirections);
        Eigen::Vector3d probe = point;
        probe.x() += distance * std::cos(angle);
        probe.y() += distance * std::sin(angle);
        for (double z_offset : {-0.06, 0.0, 0.06}) {
          probe.z() = point.z() + z_offset;
          if (!sdf_map_->isInMap(probe) ||
              sdf_map_->getOccupancy(probe) != SDFMap::FREE)
            return false;
        }
        return true;
      });
}

int TaskSearchManager::selectSearchCandidate(
    const std::vector<Eigen::Vector3d>& points, const std::vector<double>& yaws,
    const std::vector<std::vector<Eigen::Vector3d>>& frontiers,
    const Eigen::Vector3d& cur_pos, double cur_yaw) {
  if (!enabled_ || points.empty()) return -1;

  const double current_progress = corridor_frame_received_
                                      ? (cur_pos - corridor_origin_).dot(corridor_dir_)
                                      : 0.0;
  const bool entry_forward_phase =
      corridor_frame_received_ && current_progress < entry_forward_distance_;
  // frontier的前后语义与恢复器使用同一条持久通道轴线。机头转动和避障斜飞
  // 都不能让候选排序跟着偏转；真实拐弯由累计地图确认后才更新该轴线。
  Eigen::Vector2d motion_forward = stableProgressDirection();
  if (hybrid_constraints_enabled_ && exploration_initial_yaw_frozen_ &&
      !corridor_frame_received_)
    motion_forward = Eigen::Vector2d(std::cos(exploration_initial_yaw_),
                                     std::sin(exploration_initial_yaw_));
  if (motion_forward.norm() < 1e-3)
    motion_forward = Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));
  motion_forward.normalize();
  int best_idx = -1;
  int rejected_revisit = 0;
  int rejected_failed = 0;
  int rejected_door_return = 0;
  int rejected_exit_regression = 0;
  struct RankedCandidate {
    int index{-1};
    int direction_sector{0};
    bool backward{false};
    double direction_score{0.0};
    double score{0.0};
    double travel{0.0};
    double clearance{0.0};
    double clearance_reward{0.0};
  };
  std::vector<RankedCandidate> ranked_candidates;
  ranked_candidates.reserve(points.size());
  double best_score = std::numeric_limits<double>::infinity();
  double best_clearance = 0.0;
  double best_clearance_reward = 0.0;
  double best_travel = 0.0;
  int selected_sector = -1;

  // 2026-07-20: 终点未确认（或拓扑距离不足）时该保护完全关闭，下面原有的全体最优回退
  // 继续允许U形通道掉头；只有可信最终出口出现后才开始约束普通frontier。
  const bool final_exit_guard = finalExitFrontierGuardActive();
  const double current_distance_to_exit = final_exit_guard
      ? (cur_pos.head<2>() - exit_candidate_.head<2>()).norm()
      : -1.0;

  // 2026-07-13: 当前目标仍存在且尚未到达时至少保持一段时间，避免碰撞检查触发后在柱子两侧来回跳点。
  const Eigen::Vector2d active_delta = active_goal_.head<2>() - cur_pos.head<2>();
  const bool active_goal_is_backward =
      active_delta.norm() > 0.35 &&
      active_delta.normalized().dot(motion_forward) < backward_cos_threshold_;
  const double active_goal_distance_to_exit = final_exit_guard
      ? (active_goal_.head<2>() - exit_candidate_.head<2>()).norm()
      : 0.0;
  const bool active_goal_is_exit_regression = final_exit_guard &&
      active_goal_distance_to_exit > exit_frontier_local_adjust_radius_ &&
      active_goal_distance_to_exit >
          current_distance_to_exit + exit_frontier_max_distance_increase_;
  // 2026-07-23: 最终出口锁存后，普通FUEL活动目标不得越过门平面；穿门只由CROSS_EXIT发布。
  const bool active_goal_crosses_locked_exit =
      exit_portal_locked_ && exit_outward_direction_.norm() > 1e-3 &&
      (active_goal_.head<2>() - exit_portal_center_.head<2>())
              .dot(exit_outward_direction_.normalized()) >= -0.05;
  if (active_goal_valid_ && !active_goal_is_backward && !active_goal_is_exit_regression &&
      !active_goal_crosses_locked_exit &&
      (ros::Time::now() - active_goal_stamp_).toSec() < min_goal_hold_time_ &&
      (cur_pos.head<2>() - active_goal_.head<2>()).norm() > 0.35 &&
      !goalTemporarilyBlocked(active_goal_)) {
    int matched = -1;
    double matched_dist = 0.55;
    for (size_t i = 0; i < points.size(); ++i) {
      const double dist = (points[i].head<2>() - active_goal_.head<2>()).norm();
      if (dist < matched_dist) {
        matched_dist = dist;
        matched = static_cast<int>(i);
      }
    }
    if (matched >= 0 && isTaskMotionAllowed(points[matched])) {
      ROS_WARN("[task_search] keep active goal idx=%d for %.2fs, match=%.2fm.", matched,
               (ros::Time::now() - active_goal_stamp_).toSec(), matched_dist);
      return matched;
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d& point = points[i];
    // 与选中后的任务进度检查一致，提前剔除，空集合当轮进入前向恢复。
    if (!isTaskMotionAllowed(point)) {
      ++rejected_revisit;
      continue;
    }
    if (goalTemporarilyBlocked(point) || goalDirectionTemporarilyBlocked(point)) {
      ++rejected_failed;
      continue;
    }

    if (corridor_frame_received_) {
      const double candidate_progress = (point - corridor_origin_).dot(corridor_dir_);
      // 门后初始推进阶段只准保持或增加入口法向进度。外部空间更开阔时，不能仅靠代价
      // 惩罚期待FUEL自觉向前，否则大frontier收益仍会诱导其掉头。
      if (entry_forward_phase && candidate_progress < current_progress - 0.10) {
        ++rejected_door_return;
        continue;
      }
      // 2026-07-13: 深入通道后禁止再选入口门平面附近的点，修复日志中重新选择 (1.70,-0.19) 的回头轨迹。
      if (current_progress >= entry_forward_distance_ &&
          candidate_progress < inside_return_margin_) {
        ++rejected_door_return;
        continue;
      }
    }

    // 2026-07-20: 最终出口已经确认且拓扑距离达到直降门槛后，普通frontier若既不在
    // 出口局部调整区，又会让distance_to_exit显著增加，则拒绝该回头候选。
    if (final_exit_guard) {
      // 2026-07-23: 锁门后先关掉跨门的普通frontier，避免无人机先飞到门外再把门框方向带乱；
      // 门内局部调整仍保留，真正穿门由出口状态机单独执行。
      if (exit_portal_locked_ && exit_outward_direction_.norm() > 1e-3) {
        const double candidate_exit_side =
            (point.head<2>() - exit_portal_center_.head<2>())
                .dot(exit_outward_direction_.normalized());
        if (candidate_exit_side >= -0.05) {
          ++rejected_exit_regression;
          continue;
        }
      }
      const double candidate_distance_to_exit =
          (point.head<2>() - exit_candidate_.head<2>()).norm();
      const bool local_exit_adjustment =
          candidate_distance_to_exit <= exit_frontier_local_adjust_radius_;
      if (!local_exit_adjustment &&
          candidate_distance_to_exit >
              current_distance_to_exit + exit_frontier_max_distance_increase_) {
        ++rejected_exit_regression;
        continue;
      }
    }
    const double travel = (point - cur_pos).norm();
    const double novelty = minDistance2D(point, visited_positions_);
    const double goal_distance = minDistance2D(point, selected_goals_);
    int repeat_count = 0;
    for (const auto& goal : selected_goals_)
      if ((point.head<2>() - goal.head<2>()).norm() < repeat_goal_radius_) ++repeat_count;

    // 2026-07-13: 已搜索且反复选中的局部 frontier 直接失去任务资格，防止在通道拐弯处生成零长度轨迹。
    if (novelty < revisit_radius_ && repeat_count >= max_goal_repeats_) {
      ++rejected_revisit;
      continue;
    }

    const double candidate_yaw = i < yaws.size() ? yaws[i] : cur_yaw;
    const double yaw_cost = std::fabs(wrapYaw(candidate_yaw - cur_yaw));
    const double height_cost = std::fabs(clampSearchHeight(point.z()) - cruise_height_);
    const double frontier_gain =
        i < frontiers.size() ? std::log1p(static_cast<double>(frontiers[i].size())) : 0.0;
    const double clearance = clearance_reward_enabled_
                                 ? knownHorizontalClearance(point)
                                 : 0.0;
    const double clearance_reward = clearance_reward_enabled_
        ? frontier_clearance::clearanceReward(
              clearance, clearance_reward_start_, clearance_reward_full_,
              clearance_reward_max_)
        : 0.0;
    // 方向评分有意不含距离：远frontier仍然可以提供正确的探索方向。
    double direction_score = yaw_weight_ * yaw_cost + height_weight_ * height_cost -
                             novelty_weight_ * std::min(2.0, novelty) -
                             frontier_gain_weight_ * frontier_gain - clearance_reward;
    // 前向只作为软偏好：正前方获得完整加分，斜前方按投影加分，横向和后方不加分也不拒绝。
    // 使用稳定通道方向而非瞬时机头角，避免避障转头时把合法横移误判成回头。
    const Eigen::Vector2d candidate_delta = point.head<2>() - cur_pos.head<2>();
    if (prefer_motion_forward_ && candidate_delta.norm() > 1e-3) {
      const double forward_alignment =
          candidate_delta.normalized().dot(motion_forward.normalized());
      direction_score += task_search::viewpointDirectionScoreAdjustment(
          forward_alignment, forward_viewpoint_bonus_, backward_viewpoint_penalty_);
    }
    // 2026-07-14: 仅对仍有效且尚未到达的活动目标施加连续性代价；失败目标已由
    // reportGoalFailure 失效，不会阻止规划器绕开真正不可达的位置。
    if (active_goal_valid_ && !goalTemporarilyBlocked(active_goal_) &&
        (cur_pos.head<2>() - active_goal_.head<2>()).norm() > 0.35) {
      direction_score += goal_switch_weight_ *
                         (point.head<2>() - active_goal_.head<2>()).norm();
    }
    if (goal_distance < repeat_goal_radius_) direction_score += repeat_penalty_;

    // 2026-07-13: 固定门方向只用于刚穿门后的短距离；进入通道深处后由未搜索覆盖决定方向，允许正常拐弯。
    if (entry_forward_phase) {
      const double progress = (point - corridor_origin_).dot(corridor_dir_);
      direction_score -=
          entry_forward_weight_ * std::max(0.0, progress - current_progress);
      if (progress < current_progress - 0.10) direction_score += repeat_penalty_;
    }
    const bool is_backward =
        candidate_delta.norm() > 0.35 &&
        candidate_delta.normalized().dot(motion_forward) < backward_cos_threshold_;
    const double distance_cost =
        travel_weight_ * travel + task_search::preferredDistanceCost(
                                      travel, preferred_distance_min_,
                                      preferred_distance_max_, near_distance_weight_,
                                      far_distance_weight_);
    ranked_candidates.push_back(
        {static_cast<int>(i),
         task_search::directionSector(candidate_delta, motion_forward,
                                      direction_sector_rad_),
         is_backward, direction_score, direction_score + distance_cost, travel,
         clearance, clearance_reward});
  }

  const bool have_non_backward = std::any_of(
      ranked_candidates.begin(), ranked_candidates.end(),
      [](const RankedCandidate& candidate) { return !candidate.backward; });
  if (prefer_motion_forward_ && !have_non_backward && !ranked_candidates.empty() &&
      !allow_search_backtrack_) {
    ROS_ERROR_THROTTLE(1.0,
                       "[task_search] reject backward-only frontier set; hold for forward map "
                       "update instead of returning through completed corridor.");
  } else {
    const bool require_non_backward = prefer_motion_forward_ && have_non_backward;
    double best_direction_score = std::numeric_limits<double>::infinity();
    for (const auto& candidate : ranked_candidates) {
      if (require_non_backward && candidate.backward) continue;
      if (candidate.direction_score < best_direction_score) {
        best_direction_score = candidate.direction_score;
        selected_sector = candidate.direction_sector;
      }
    }

    // 方向由全部远近frontier决定；具体目标只在胜出扇区内按1-2m距离带排序。
    for (const auto& candidate : ranked_candidates) {
      if ((require_non_backward && candidate.backward) ||
          candidate.direction_sector != selected_sector)
        continue;
      if (candidate.score < best_score) {
        best_score = candidate.score;
        best_idx = candidate.index;
        best_travel = candidate.travel;
        best_clearance = candidate.clearance;
        best_clearance_reward = candidate.clearance_reward;
      }
    }
  }

  ROS_WARN("[task_search] candidates=%zu selected=%d rejected_revisit=%d rejected_failed=%d "
           "rejected_door=%d rejected_exit_regression=%d exit_guard=%d "
           "current_exit_distance=%.2f entry_forward=%d sector=%d travel=%.2f score=%.2f "
           "clearance=%.2f reward=%.2f.",
           points.size(), best_idx, rejected_revisit, rejected_failed, rejected_door_return,
           rejected_exit_regression, static_cast<int>(final_exit_guard),
           current_distance_to_exit, static_cast<int>(entry_forward_phase),
           selected_sector, best_travel, best_score, best_clearance, best_clearance_reward);
  return best_idx;
}

// 2026-07-28: 用累计占据地图识别“旧前向封闭、侧向通道连续”的正常弯道；
// 允许一侧墙在拐角中断，但另一侧必须有连续地图轮廓，避免孤立障碍物误触发。
bool TaskSearchManager::inferOccupancyTurnDirection(
    const Eigen::Vector3d& travel_direction, Eigen::Vector3d& turn_direction,
    double& forward_free_length, double& turn_free_length, double* upper_height) const {
  if (upper_height) *upper_height = std::numeric_limits<double>::quiet_NaN();
  forward_free_length = 0.0;
  turn_free_length = 0.0;
  if (!recovery_occupancy_turn_enabled_ || !sdf_map_ || visited_positions_.empty() ||
      travel_direction.head<2>().norm() < 1e-3)
    return false;

  // 第一票出现后冻结当时看到拐角的地图观察点。无人机随后斜移/进入新通道时，
  // 若继续用最新位置发射旧前向射线，前墙会落到身后，可靠的第一票反而无法复核。
  const Eigen::Vector3d origin =
      pending_turn_probe_origin_valid_ ? pending_turn_probe_origin_
                                       : visited_positions_.back();
  const Eigen::Vector2d travel = travel_direction.head<2>().normalized();
  const double probe_step = std::max(0.05, recovery_turn_probe_step_);
  const double min_angle = recovery_turn_min_angle_deg_ * M_PI / 180.0;

  auto knownFreeLength = [&](const Eigen::Vector3d& ray_origin,
                             const Eigen::Vector2d& direction) {
    double free_length = 0.0;
    for (double distance = probe_step; distance <= recovery_turn_probe_length_ + 1e-6;
         distance += probe_step) {
      Eigen::Vector3d probe = ray_origin;
      probe.head<2>() += distance * direction;
      // 这里只识别地图轮廓，不签发飞行许可。UNKNOWN和占据都终止射线，足迹安全仍由A*复核。
      if (!sdf_map_->isInMap(probe) ||
          sdf_map_->getOccupancy(probe) != SDFMap::FREE)
        break;
      free_length = distance;
    }
    return free_length;
  };
  forward_free_length = knownFreeLength(origin, travel);
  Eigen::Vector3d forward_end = origin;
  forward_end.head<2>() +=
      std::min(recovery_turn_probe_length_, forward_free_length + probe_step) * travel;
  // 旧通道前方必须有雷达已建图的实际占据，UNKNOWN不算死路。
  // 确认旧轴已终止后，不再用新旧自由长度差或固定最小延伸距离否决转弯。
  const bool old_direction_blocked =
      forward_free_length + probe_step < recovery_turn_probe_length_ + 1e-6 &&
      mapRelativeColumnOccupied(forward_end, origin.z());
  if (!old_direction_blocked) return false;

  // 中心射线第一次碰到占据只说明前方有障碍，不等于旧通道已经结束。
  // 九宫格只用来把障碍水平面分区；真正判定在整个平面上搜索能容纳
  // 0.5m×0.5m足迹的连续路径，并对相邻采样中点再做足迹检查。
  const Eigen::Vector2d lateral(-travel.y(), travel.x());
  constexpr double kBypassWindowSize = 0.50;
  constexpr double kAuditPlaneSize = 1.50;
  const double map_resolution = std::max(0.01, sdf_map_->getResolution());
  auto knownFreeWindow = [&](const Eigen::Vector3d& center) {
    const double half = 0.5 * kBypassWindowSize;
    const int samples = std::max(
        1, static_cast<int>(std::ceil(kBypassWindowSize / map_resolution)));
    const double sample_step = kBypassWindowSize / samples;
    for (int forward_sample = 0; forward_sample <= samples; ++forward_sample) {
      for (int lateral_sample = 0; lateral_sample <= samples; ++lateral_sample) {
        Eigen::Vector3d sample = center;
        sample.head<2>() +=
            (-half + forward_sample * sample_step) * travel +
            (-half + lateral_sample * sample_step) * lateral;
        if (!sdf_map_->isInMap(sample) ||
            sdf_map_->getOccupancy(sample) != SDFMap::FREE)
          return false;
      }
    }
    return true;
  };

  Eigen::Vector3d grid_center = forward_end;
  const Eigen::Vector2d corridor_axis_origin =
      segment_direction_.norm() > 1e-3 ? segment_origin_ : origin.head<2>();
  grid_center.head<2>() = task_search::corridorGridCenter(
      forward_end.head<2>(), corridor_axis_origin, travel);

  const int lattice_steps = std::max(
      3, static_cast<int>(std::ceil(kAuditPlaneSize / map_resolution)));
  const int lattice_size = lattice_steps + 1;
  const double lattice_step = kAuditPlaneSize / lattice_steps;
  const double plane_half = 0.5 * kAuditPlaneSize;
  std::vector<std::vector<uint8_t>> footprint_free(
      lattice_size, std::vector<uint8_t>(lattice_size, 0));
  std::array<std::array<bool, 3>, 3> region_has_free{};
  auto latticeCenter = [&](int forward_index, int lateral_index) {
    Eigen::Vector3d center = grid_center;
    const double forward_offset = -plane_half + forward_index * lattice_step;
    const double lateral_offset = -plane_half + lateral_index * lattice_step;
    center.head<2>() += forward_offset * travel + lateral_offset * lateral;
    return center;
  };
  for (int forward_index = 0; forward_index < lattice_size; ++forward_index) {
    for (int lateral_index = 0; lateral_index < lattice_size; ++lateral_index) {
      if (!knownFreeWindow(latticeCenter(forward_index, lateral_index))) continue;
      footprint_free[forward_index][lateral_index] = 1;
      const int region_forward = std::min(
          2, static_cast<int>(forward_index * 3 / lattice_size));
      const int region_lateral = std::min(
          2, static_cast<int>(lateral_index * 3 / lattice_size));
      region_has_free[region_forward][region_lateral] = true;
    }
  }

  std::vector<std::vector<uint8_t>> reachable(
      lattice_size, std::vector<uint8_t>(lattice_size, 0));
  std::queue<std::pair<int, int>> open;
  for (int lateral_index = 0; lateral_index < lattice_size; ++lateral_index) {
    if (!footprint_free[0][lateral_index]) continue;
    reachable[0][lateral_index] = 1;
    open.emplace(0, lateral_index);
  }
  bool has_bypass = false;
  while (!open.empty() && !has_bypass) {
    const auto current = open.front();
    open.pop();
    if (current.first == lattice_size - 1) {
      has_bypass = true;
      break;
    }
    for (int df = -1; df <= 1; ++df) {
      for (int dl = -1; dl <= 1; ++dl) {
        if (df == 0 && dl == 0) continue;
        const int next_forward = current.first + df;
        const int next_lateral = current.second + dl;
        if (next_forward < 0 || next_forward >= lattice_size ||
            next_lateral < 0 || next_lateral >= lattice_size ||
            reachable[next_forward][next_lateral] ||
            !footprint_free[next_forward][next_lateral])
          continue;
        const Eigen::Vector3d connection_center =
            0.5 * (latticeCenter(current.first, current.second) +
                   latticeCenter(next_forward, next_lateral));
        if (!knownFreeWindow(connection_center)) continue;
        reachable[next_forward][next_lateral] = 1;
        open.emplace(next_forward, next_lateral);
      }
    }
  }
  Eigen::Vector3d robot_pos;
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    robot_pos = latest_robot_pos_;
  }
  const double goal_x = active_goal_valid_
                            ? active_goal_.x()
                            : std::numeric_limits<double>::quiet_NaN();
  const double goal_y = active_goal_valid_
                            ? active_goal_.y()
                            : std::numeric_limits<double>::quiet_NaN();
  ROS_WARN_THROTTLE(
      0.5,
      "[task_search] obstacle-plane audit robot=(%.2f,%.2f) origin=(%.2f,%.2f) "
      "ray_hit=(%.2f,%.2f) grid_center=(%.2f,%.2f) active_goal=(%.2f,%.2f) "
      "free_regions=[%d%d%d/%d%d%d/%d%d%d] connected=%d decision=%s.",
      robot_pos.x(), robot_pos.y(), origin.x(), origin.y(),
      forward_end.x(), forward_end.y(), grid_center.x(), grid_center.y(),
      goal_x, goal_y,
      static_cast<int>(region_has_free[0][0]),
      static_cast<int>(region_has_free[0][1]),
      static_cast<int>(region_has_free[0][2]),
      static_cast<int>(region_has_free[1][0]),
      static_cast<int>(region_has_free[1][1]),
      static_cast<int>(region_has_free[1][2]),
      static_cast<int>(region_has_free[2][0]),
      static_cast<int>(region_has_free[2][1]),
      static_cast<int>(region_has_free[2][2]),
      static_cast<int>(has_bypass),
      has_bypass ? "LOCAL_BYPASS" : "TEST_MAPPED_TURN");
  if (has_bypass) {
    return false;
  }

  // 轮廓射线只用于发现前墙。新通道必须从真实机位连通，整个机体包络在
  // 累计地图中已知可通行；不能把穿过窄缝的中心射线提升为通道方向。
  const Eigen::Vector3d start = latest_robot_pose_valid_ ? robot_pos : origin;
  const auto bodyFree = [&](const Eigen::Vector3d& p) {
    return mapExitFootprintFree(p,turn_bottom_clearance_);
  };
  const double body_step = std::min(0.05, sdf_map_->getResolution());
  const double horizon = std::min(1.0, recovery_turn_probe_length_);
  if (horizon <= 0.0 || !bodyFree(start)) return false;
  std::vector<Eigen::Vector3d> anchors{start};
  // 射线采样的最后FREE点不等于墙面；接近距离由足迹检查实际截断，
  // 不再提前减一次机体半径，避免漏掉窄弯唯一可通行的转角中心。
  const double approach_length = std::min(
      recovery_turn_probe_length_, forward_free_length + probe_step);
  for (double distance = body_step; distance <= approach_length + 1e-6;
       distance += body_step) {
    Eigen::Vector3d point = start;
    point.head<2>() += distance * travel;
    if (!bodyFree(point)) break;
    anchors.push_back(point);
  }

  // 历史实飞已形成入口段和横向段时，倒U形延伸倾向于入口反向。
  // 只在实际离开上个转角后启用，不预设左/右，也不靠刚锁存的航向自证。
  double u_shape_confidence = 0.0;
  Eigen::Vector2d return_direction = -corridor_dir_.head<2>().normalized();
  if (corridor_frame_received_ && latest_turn_anchor_valid_) {
    const Eigen::Vector2d flown = start.head<2>() - latest_turn_anchor_;
    if (flown.norm() >= 0.60 && flown.normalized().dot(travel) > 0.85 &&
        latest_turn_incoming_direction_.dot(-return_direction) > 0.85 &&
        std::fabs(travel.dot(return_direction)) < 0.50)
      u_shape_confidence = 1.0;
  }
  double best_score = -std::numeric_limits<double>::infinity();
  double side_score[2] = {best_score, best_score};
  double side_width[2] = {0.0, 0.0};
  double best_width = 0.0;
  Eigen::Vector3d best_anchor = start;
  Eigen::Vector2d best_direction = travel;
  const double max_angle = recovery_turn_max_angle_deg_ * M_PI / 180.0;
  double best_rise = 0.0;
  const double maximum_rise = turn_upper_enabled_ ? turn_upper_max_rise_ : 0.0;
  // 当前高度有新通路就保持原选择；只有这些候选均不可用时才查低障碍上方。
  for (double rise = 0.0; rise <= maximum_rise + 1e-6; rise += turn_upper_step_) {
    if (rise > 0.0 && start.z()+rise > std::min(turn_upper_max_height_, corridorHeightLimit())+1e-6) break;
    for (double angle = min_angle; angle <= max_angle + 1e-6; angle += M_PI / 12.0) {
      for (double sign : {-1.0, 1.0}) {
        const double signed_angle = sign * angle;
        const Eigen::Vector2d direction(
            std::cos(signed_angle) * travel.x() - std::sin(signed_angle) * travel.y(),
            std::sin(signed_angle) * travel.x() + std::cos(signed_angle) * travel.y());
        // 新通道不能落到当前已确认通道的后方。这里必须比较当前travel，不能比较初始
        // 入口方向；否则北向通道中的东南旧路仍会被误当成合法新分支。
        if (!task_search::insideForwardHalfPlane(direction, travel))
          continue;
        // U形通道的新段可以与入口绝对航向相反；回头只按空间上是否
        // 重新压回已飞历史轨迹判断，不再用初始入口航向一刀切。
        const Eigen::Vector3d forward(direction.x(), direction.y(), 0.0);
        // 每个转弯起点都通过了从当前机位到该点的连续足迹检查。
        // 所有左右候选共用同一米级观察长度和宽度评分，最高分胜出。
        for (std::size_t a = 0; a < anchors.size(); ++a) {
          const Eigen::Vector3d& base = anchors[a];
          if (turnDirectionRetracesVisitedRoute(base.head<2>(), direction)) continue;
          Eigen::Vector3d anchor = base;
          anchor.z() += rise;
          if (rise > 0.0) {
            // 原地转向发生在当前机位，高层分支必须从此处可上升进入，不能隔墙看宽处。
            if (a != 0) continue;
            bool low_obstacle = false;
            for (double d = body_step; d <= horizon+1e-6; d += body_step)
              if (mapRelativeColumnOccupied(base+d*forward,base.z())) { low_obstacle=true; break; }
            if (!low_obstacle) continue;
            bool column_free = true;
            for (double dz = body_step; dz <= rise+1e-6; dz += body_step)
              if (!bodyFree(base+Eigen::Vector3d(0,0,dz))) { column_free=false; break; }
            if (!column_free || !bodyFree(anchor)) continue;
          }
          double free_length = 0.0;
          for (double distance = body_step;
               distance <= recovery_turn_probe_length_ + 1e-6; distance += body_step) {
            if (!bodyFree(anchor + distance * forward)) break;
            free_length = distance;
          }
          if (free_length + 1e-6 < horizon) continue;
          const std::vector<Eigen::Vector3d> branch{anchor, anchor + horizon * forward};
          const auto width = route_width::measure(
              branch, forward, horizon, 0.75, body_step,
              bodyFree);
          if (!width.complete) continue;
          // 宽度为可容纳机体的中心移动范围；包络本身已在上面逐点验证。
          const double shape_bonus = 0.05 * u_shape_confidence *
                                     direction.dot(return_direction);
          const double score = width.minimum + 0.25 * width.mean +
                               0.10 * free_length - 0.02 * std::fabs(angle) + shape_bonus;
          const int side = sign > 0.0 ? 1 : 0;
          if (score > side_score[side]) {
            side_score[side] = score;
            side_width[side] = width.minimum + 2.0 * exit_footprint_extra_radius_;
          }
          if (score > best_score) {
            best_score = score;
            best_rise = rise;
            best_width = width.minimum + 2.0 * exit_footprint_extra_radius_;
            best_anchor = anchor;
            best_direction = direction;
            turn_free_length = free_length;
          }
        }
      }
    }
    if (std::isfinite(best_score)) break;
  }
  if (!std::isfinite(best_score)) return false;
  if (upper_height && best_rise > 0.0) *upper_height = start.z()+best_rise;
  turn_direction = Eigen::Vector3d(best_direction.x(), best_direction.y(), 0.0);
  ROS_ERROR_THROTTLE(
      0.5,
      "[task_search] PASSABLE TURN selected yaw=%.1fdeg old_free=%.2fm "
      "branch_free=%.2fm body_width=%.2fm anchor=(%.2f,%.2f) "
      "right=(score=%.3f,width=%.2f) left=(score=%.3f,width=%.2f) "
      "u_history=%.0f rise=%.2fm; highest body-passable branch score wins.",
      std::atan2(best_direction.y(), best_direction.x()) * 180.0 / M_PI,
      forward_free_length, turn_free_length, best_width, best_anchor.x(), best_anchor.y(),
      side_score[0], side_width[0], side_score[1], side_width[1], u_shape_confidence, best_rise);
  return true;
}

bool TaskSearchManager::turnDirectionRetracesVisitedRoute(
    const Eigen::Vector2d& anchor,
    const Eigen::Vector2d& direction) const {
  if (visited_positions_.size() < 2 || direction.norm() < 1e-3)
    return false;
  const Eigen::Vector2d unit_direction = direction.normalized();
  constexpr double kRouteOverlapRadius = 0.30;
  constexpr double kProbeLength = 1.50;
  constexpr double kProbeStep = 0.05;
  double overlap_start = -1.0;
  // 只用于识别正常探索的新通道，历史回退脱障不调用本检查。
  // 保留全部历史；按相邻实飞点之间的线段判断反向重走，不再豁免附近70cm。
  for (double distance = 0.10; distance <= kProbeLength + 1e-6; distance += kProbeStep) {
    const Eigen::Vector2d probe = anchor + distance * unit_direction;
    bool reversing_history = false;
    for (size_t i = 1; i < visited_positions_.size(); ++i) {
      const Eigen::Vector2d from = visited_positions_[i-1].head<2>();
      const Eigen::Vector2d flown = visited_positions_[i].head<2>() - from;
      if (flown.norm() < 0.03 || flown.normalized().dot(unit_direction) > -0.5) continue;
      const double projection = (probe-from).dot(flown) / flown.squaredNorm();
      // 不把轨迹端点周围的圆形邻域当成已飞路段；横移离开共同起点应正常放行。
      if (projection < 0.0 || projection > 1.0) continue;
      const Eigen::Vector2d history = from + projection * flown;
      if ((history-anchor).dot(unit_direction) < 0.10-1e-6) continue;
      if ((probe-history).norm() <= kRouteOverlapRadius) {
        reversing_history = true;
        break;
      }
    }
    if (!reversing_history) { overlap_start = -1.0; continue; }
    if (overlap_start < 0.0) overlap_start = distance;
    if (distance-overlap_start >= 0.20-1e-6) {
      ROS_ERROR_THROTTLE(0.5,
          "[task_search] reject mapped branch yaw=%.1fdeg at (%.2f,%.2f): "
          "reverses %.2fm of flown route; history retained for dedicated escape.",
          std::atan2(unit_direction.y(),unit_direction.x())*180.0/M_PI,
          anchor.x(),anchor.y(),distance-overlap_start);
      return true;
    }
  }
  return false;
}

void TaskSearchManager::commitCorridorTurn(
    const Eigen::Vector3d& turn_direction) {
  if (turn_direction.head<2>().norm() < 1e-3) return;
  Eigen::Vector2d incoming = turnDetectionDirection();
  if (incoming.norm() < 1e-3) incoming = corridor_dir_.head<2>();
  incoming.normalize();
  const Eigen::Vector2d outgoing = turn_direction.head<2>().normalized();
  const double signed_angle = std::atan2(
      incoming.x() * outgoing.y() - incoming.y() * outgoing.x(),
      incoming.dot(outgoing));

  Eigen::Vector2d anchor = visited_positions_.empty()
                               ? corridor_origin_.head<2>()
                               : visited_positions_.back().head<2>();
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    if (latest_robot_pose_valid_) anchor = latest_robot_pos_.head<2>();
  }
  latest_turn_anchor_ = anchor;
  latest_turn_incoming_direction_ = incoming;
  latest_turn_anchor_valid_ = true;
  committed_turn_upper_height_ = std::numeric_limits<double>::quiet_NaN();
  transition_active_ = true;
  transition_anchor_ = anchor;
  transition_incoming_direction_ = incoming;
  transition_outgoing_direction_ = outgoing;
  transition_old_high_water_ = segment_high_water_;
  if (hybrid_constraints_enabled_) {
    CompletedGate gate;
    gate.sequence = next_gate_sequence_++;
    gate.center = anchor;
    gate.normal = incoming;
    gate.left_extent = 1.45;
    gate.right_extent = 1.45;
    gate.thickness = 0.30;
    completed_gates_.push_back(gate);
    if (completed_gates_.size() > 64)
      completed_gates_.erase(completed_gates_.begin());
  }
  // Native FUEL同样按已确认的通道段记录前进进度。它不使用混合模式的历史门，
  // 但必须防止恢复轨迹沿当前段长距离倒退。
  segment_origin_ = anchor;
  segment_direction_ = outgoing;
  segment_high_water_ = 0.0;
  if (!exit_portal_locked_) clearExitEvidence();
  stable_progress_direction_ = outgoing;
  stable_progress_direction_valid_ = true;
  turn_detection_direction_ = outgoing;
  // A confirmed turn starts a new FUEL segment. Keep the yaw reference tied to
  // that segment instead of letting the instantaneous body yaw redefine
  // which frontier is considered "in front".
  if (hybrid_constraints_enabled_) {
    exploration_initial_yaw_ = std::atan2(outgoing.y(), outgoing.x());
    exploration_initial_yaw_frozen_ = true;
  }
  turn_yaw_follow_latch_.arm(outgoing);
  ROS_ERROR("[task_search] confirmed %s turn %.1fdeg at (%.2f,%.2f); "
            "new wall-parallel yaw=%.1fdeg and latest completed direction is no-return.",
            signed_angle >= 0.0 ? "LEFT" : "RIGHT",
            std::fabs(signed_angle) * 180.0 / M_PI, anchor.x(), anchor.y(),
            std::atan2(outgoing.y(), outgoing.x()) * 180.0 / M_PI);
}

bool TaskSearchManager::mappedCorridorDirection(
    double cur_yaw, Eigen::Vector3d& direction, bool check_entry_forward_phase,
    bool require_observed_yaw, bool upper_route_only) {
  // 入口后的第一段直线尚未走完时，门外开放区和门框遮挡不是拐弯。
  if (check_entry_forward_phase && entryForwardPhaseActive()) {
    clearPendingTurnEvidence();
    return false;
  }

  Eigen::Vector2d latched_direction;
  if (turn_yaw_follow_latch_.lockedDirection(latched_direction)) {
    if (upper_route_only && !std::isfinite(committed_turn_upper_height_)) return false;
    direction = Eigen::Vector3d(latched_direction.x(), latched_direction.y(), 0.0);
    return true;
  }

  // 已确认的转弯尚未沿新通道离开转角时，不得把同一片转角地图再次解释成
  // 另一条新通道。保持刚提交的通道方向，走出既有0.60m过渡区后再恢复检测。
  if (transition_active_) {
    clearPendingTurnEvidence();
    ROS_INFO_THROTTLE(
        0.5,
        "[task_search] turn transition active; keep committed corridor direction "
        "and suppress repeated turn detection."
    );
    return false;
  }

  updateCurveHeading();
  if (curve_heading_active_) {
    // 渐变双墙轮廓只更新滚动方向，不锁存一次离散转弯；直角弯仍走下方原流程。
    clearPendingTurnEvidence();
    return false;
  }

  Eigen::Vector2d stable = turnDetectionDirection();
  if (stable.norm() < 1e-3)
    stable = Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));
  const Eigen::Vector3d travel(stable.x(), stable.y(), 0.0);
  double forward_free_length = 0.0;
  double turn_free_length = 0.0;
  double upper_height;
  if (!inferOccupancyTurnDirection(
          travel, direction, forward_free_length, turn_free_length, &upper_height))
    return false;
  if (upper_route_only && !std::isfinite(upper_height)) return false;
  // 手持模式没有控制器去执行地图建议的转向。实际朝向尚未与该分支一致时，
  // 保留旧段，避免提前锁定一条未走入的支路后永远等待对准。
  // 正式飞行调用默认不启用此项，仍先确认地图转弯再由FSM执行转向。
  if (require_observed_yaw && task_search::corridorYawCorrectionNeeded(
          direction.head<2>(), cur_yaw,
          recovery_turn_yaw_release_angle_deg_ * M_PI / 180.0)) {
    clearPendingTurnEvidence();
    ROS_INFO_THROTTLE(1.0,
                     "[exit_detection_test] mapped branch awaits observed yaw: "
                     "branch=%.1fdeg body=%.1fdeg.",
                     std::atan2(direction.y(), direction.x()) * 180.0 / M_PI,
                     cur_yaw * 180.0 / M_PI);
    return false;
  }
  if (!confirmCorridorTurnEvidence(direction)) return false;
  commitCorridorTurn(direction);
  committed_turn_upper_height_ = upper_height;
  if (std::isfinite(upper_height))
    ROS_WARN("[upper_turn] confirmed mapped branch at z=%.2fm; align yaw first, "
             "then recheck ascent and forward route on the current map.",upper_height);
  // 新确认的这一周期立即生效。锁存方向只能由实际yaw连续达标后的FSM显式释放，
  // 这样30度分段之间不会重新做地图转弯检测。
  if (!turn_yaw_follow_latch_.lockedDirection(latched_direction))
    return false;
  direction = Eigen::Vector3d(latched_direction.x(), latched_direction.y(), 0.0);
  return true;
}

bool TaskSearchManager::entryForwardPhaseActive() const {
  if (!corridor_frame_received_) return true;
  Eigen::Vector3d position = visited_positions_.empty()
                                 ? corridor_origin_
                                 : visited_positions_.back();
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    if (latest_robot_pose_valid_) position = latest_robot_pos_;
  }
  return (position - corridor_origin_).dot(corridor_dir_) <
         entry_forward_distance_;
}

// 局部恢复只读取已确认的通道轴线，不在候选排序过程中检测或提交转弯。
Eigen::Vector3d TaskSearchManager::recoveryForwardDirection(double cur_yaw) {
  Eigen::Vector2d latched_direction;
  if (turn_yaw_follow_latch_.lockedDirection(latched_direction))
    return Eigen::Vector3d(latched_direction.x(), latched_direction.y(), 0.0);

  Eigen::Vector2d stable = stableProgressDirection();
  if (stable.norm() < 1e-3)
    stable = Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));
  return Eigen::Vector3d(stable.x(), stable.y(), 0.0);
}

// 2026-07-28: 与普通候选使用相同角度阈值，避免恢复器和frontier对“后退”的定义不一致。
bool TaskSearchManager::isRecoveryDirectionBackward(
    const Eigen::Vector3d& direction, double cur_yaw) {
  const Eigen::Vector3d forward = recoveryForwardDirection(cur_yaw);
  return direction.head<2>().norm() > 1e-3 &&
         direction.head<2>().normalized().dot(forward.head<2>()) <
             backward_cos_threshold_;
}

std::vector<Eigen::Vector3d> TaskSearchManager::recoveryDirections(double cur_yaw) {
  const Eigen::Vector3d forward = recoveryForwardDirection(cur_yaw);
  const double travel_yaw = std::atan2(forward.y(), forward.x());

  // 2026-07-23: 最新视频中直行短路径被膨胀栅格拒绝后，旧恢复器只试0/±45/±90度，
  // 会漏掉仍在前方半平面内的可行绕障方向；改为每15度扫描前方半圆，仍不生成后退方向。
  std::vector<double> offsets = {
      0.0,
      M_PI / 12.0, -M_PI / 12.0,
      M_PI / 6.0, -M_PI / 6.0,
      M_PI_4, -M_PI_4,
      M_PI / 3.0, -M_PI / 3.0,
      5.0 * M_PI / 12.0, -5.0 * M_PI / 12.0,
      M_PI_2, -M_PI_2};
  // 2026-07-28: 始终把±135度和180度交给“短回撤”候选池；上层将其严格限制为一次、最多0.45m，
  // 不会让普通FUEL frontier获得向后探索资格，也不会形成连续直线倒车。
  offsets.push_back(3.0 * M_PI_4);
  offsets.push_back(-3.0 * M_PI_4);
  offsets.push_back(M_PI);
  std::vector<std::pair<double, Eigen::Vector3d>> scored;
  for (double offset : offsets) {
    const double yaw = wrapYaw(travel_yaw + offset);
    Eigen::Vector3d dir(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d probe =
        visited_positions_.empty() ? dir : visited_positions_.back() + 1.2 * dir;
    const double novelty = minDistance2D(probe, visited_positions_);
    // 2026-07-22: fallback首先保持真实运动方向，只有同一转角级别的左右候选才比较新颖度；
    // 旧权重会让“更陌生的±90度”压过可行直行，并在通道末端主动拐回刚走过的支路。
    scored.emplace_back(-4.0 * std::fabs(offset) +
                            0.10 * std::min(2.0, novelty),
                        dir);
  }
  std::sort(scored.begin(), scored.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });

  std::vector<Eigen::Vector3d> directions;
  for (const auto& item : scored) directions.push_back(item.second);
  return directions;
}

bool TaskSearchManager::isRecoveryCandidateUseful(const Eigen::Vector3d& candidate) const {
  // 恢复点只受入口和最终出口边界约束；历史航迹距离不能否决窄通道中的侧移绕障。
  if (!hybrid_constraints_enabled_ && corridor_frame_received_ &&
      !visited_positions_.empty()) {
    const Eigen::Vector2d motion =
        candidate.head<2>() - visited_positions_.back().head<2>();
    if (!task_search::insideForwardHalfPlane(motion, stableProgressDirection()))
      return false;
  }
  return isTaskMotionAllowed(candidate);
}

bool TaskSearchManager::astarNoReturnDirection(Eigen::Vector3d& direction) const {
  if (!global_no_return_ || !corridor_frame_received_) return false;
  const Eigen::Vector2d progress = stableProgressDirection();
  if (progress.norm() < 1e-6) return false;
  direction = Eigen::Vector3d(progress.x(), progress.y(), 0.0);
  return true;
}

bool TaskSearchManager::isMissionBoundaryMotionAllowed(
    const Eigen::Vector3d& candidate) const {
  if (!global_no_return_ || !corridor_frame_received_) return true;
  // 普通搜索永远不能重新穿回初始入口外侧。最终出口的专用状态由上层
  // exit_transit_active 绕过普通任务路径过滤，不受这里影响。
  const double door_progress = (candidate - corridor_origin_).dot(corridor_dir_);
  // 2026-07-23: 最终出口只允许穿越一次。CROSS_EXIT确认完成后，出口外的所有区域
  // 统一视为通道外；候选若重新落到门内侧就直接拒绝，不再做任何复杂区域分类。
  const bool final_exit_guard_active =
      mission_stage_ >= SEARCH_OUTSIDE_LANDING && exit_candidate_confirmed_ &&
      exit_outward_direction_.norm() > 1e-3;
  double exit_side = 0.0;
  if (final_exit_guard_active) {
    exit_side =
        (candidate.head<2>() - exit_portal_center_.head<2>())
            .dot(exit_outward_direction_.normalized());
  }
  if (!task_search::passesMissionBoundaryNoReturn(
          door_progress, inside_return_margin_, final_exit_guard_active, exit_side))
    return false;
  return true;
}

bool TaskSearchManager::isTaskMotionAllowed(const Eigen::Vector3d& candidate) const {
  if (!isMissionBoundaryMotionAllowed(candidate)) return false;
  if (!global_no_return_ || !corridor_frame_received_) return true;
  if (!hybrid_constraints_enabled_) {
    if (latest_turn_anchor_valid_ &&
        !task_search::passesLatestTurnNoReturn(
            candidate.head<2>(), latest_turn_anchor_,
            latest_turn_incoming_direction_, stableProgressDirection(),
            recovery_turn_no_return_margin_))
      return false;
    const Eigen::Vector2d direction = segment_direction_.norm() > 1e-3
                                          ? segment_direction_.normalized()
                                          : stableProgressDirection();
    const double progress = (candidate.head<2>() - segment_origin_).dot(direction);
    double minimum_progress = segment_high_water_ - 0.25;
    {
      std::lock_guard<std::mutex> lock(body_cloud_mutex_);
      if (latest_robot_pose_valid_) {
        const double current_progress =
            (latest_robot_pos_.head<2>() - segment_origin_).dot(direction);
        // 被动回漂已发生时，允许从实际位置向前恢复；不把历史最高进度
        // 清零，也不额外授予从当前位置继续后退的 0.25m 容差。
        minimum_progress = std::min(minimum_progress, current_progress);
      }
    }
    return progress >= minimum_progress - 1e-6;
  }
  if (transition_active_ &&
      (candidate.head<2>() - transition_anchor_).norm() <= 0.80 + 1e-6)
    return true;
  for (const auto& gate : completed_gates_) {
    const Eigen::Vector2d normal = gate.normal.normalized();
    const Eigen::Vector2d lateral(-normal.y(), normal.x());
    const Eigen::Vector2d relative = candidate.head<2>() - gate.center;
    const double side = relative.dot(normal);
    const double across = relative.dot(lateral);
    if (std::fabs(side) <= 0.5 * gate.thickness + 1e-6 &&
        across <= gate.left_extent + 1e-6 && across >= -gate.right_extent - 1e-6)
      return false;
  }
  if (transition_active_) {
    const Eigen::Vector2d incoming = transition_incoming_direction_.normalized();
    const Eigen::Vector2d outgoing = transition_outgoing_direction_.normalized();
    const Eigen::Vector2d in_lateral(-incoming.y(), incoming.x());
    const Eigen::Vector2d out_lateral(-outgoing.y(), outgoing.x());
    const Eigen::Vector2d from_segment = candidate.head<2>() - segment_origin_;
    const Eigen::Vector2d from_anchor = candidate.head<2>() - transition_anchor_;
    const bool in_corridor = from_segment.dot(incoming) >=
                                 transition_old_high_water_ - 0.45 &&
                             std::fabs(from_segment.dot(in_lateral)) <= 1.25;
    const bool out_corridor = from_anchor.dot(outgoing) >= -0.45 &&
                              std::fabs(from_anchor.dot(out_lateral)) <= 1.25;
    return (from_anchor.norm() <= 0.80 + 1e-6) || in_corridor || out_corridor;
  }
  const Eigen::Vector2d direction = segment_direction_.norm() > 1e-3
                                        ? segment_direction_.normalized()
                                        : stableProgressDirection();
  const double progress = (candidate.head<2>() - segment_origin_).dot(direction);
  return progress >= segment_high_water_ - 0.25 - 1e-6;
}

bool TaskSearchManager::isTaskPathAllowed(
    const std::vector<Eigen::Vector3d>& path) const {
  if (!global_no_return_ || !corridor_frame_received_ || path.empty()) return true;

  // 完整路径逐点检查入口、最近已完成转角和最终出口边界，避免目标合法但中途越界。
  for (const auto& point : path) {
    if (!isTaskMotionAllowed(point)) return false;
  }

  if (!hybrid_constraints_enabled_) return true;

  // 解析检查每一段，保证端点合法但中途穿过历史门的路径仍被拒绝。
  // During the short turn transition the outgoing segment is allowed to leave
  // the anchor through the completed gate; the corridor envelope above is the
  // active restriction until the new segment is established.
  if (transition_active_) return true;
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d start = path[i - 1].head<2>();
    const Eigen::Vector2d end = path[i].head<2>();
    for (const auto& gate : completed_gates_) {
      const Eigen::Vector2d normal = gate.normal.normalized();
      const Eigen::Vector2d lateral(-normal.y(), normal.x());
      const double s0 = (start - gate.center).dot(normal);
      const double s1 = (end - gate.center).dot(normal);
      if (std::fabs(s0) <= 0.5 * gate.thickness ||
          std::fabs(s1) <= 0.5 * gate.thickness || s0 * s1 < 0.0) {
        const double denom = s0 - s1;
        const double ratio = std::fabs(denom) < 1e-9 ? 0.0 : s0 / denom;
        const Eigen::Vector2d crossing = start +
            std::max(0.0, std::min(1.0, ratio)) * (end - start);
        const double across = (crossing - gate.center).dot(lateral);
        if (across <= gate.left_extent + 1e-6 &&
            across >= -gate.right_extent - 1e-6)
          return false;
      }
    }
  }
  return true;
}

bool TaskSearchManager::isRecoveryPathAllowed(
    const std::vector<Eigen::Vector3d>& path, bool allow_initial_reverse) const {
  if (!hybrid_constraints_enabled_) {
    if (corridor_frame_received_ && path.size() >= 2) {
      const Eigen::Vector2d progress_direction = stableProgressDirection();
      const double start_progress =
          path.front().head<2>().dot(progress_direction);
      for (const auto& point : path) {
        if (point.head<2>().dot(progress_direction) < start_progress - 1e-3)
          return false;
      }
    }
    if (!allow_initial_reverse) return isTaskPathAllowed(path);
    if (!global_no_return_ || !corridor_frame_received_ || path.empty())
      return true;
    for (const auto& point : path) {
      if (!isTaskMotionAllowed(point)) return false;
    }
    return true;
  }
  if (!allow_initial_reverse) return isTaskPathAllowed(path);
  if (!global_no_return_ || !corridor_frame_received_ || path.empty()) return true;

  return isTaskPathAllowed(path);
}

double TaskSearchManager::clampSearchHeight(double z) const {
  return std::max(min_search_height_, std::min(std::min(max_search_height_, corridorHeightLimit()), z));
}

double TaskSearchManager::preferredSearchHeight() const {
  return clampSearchHeight(cruise_height_);
}

double TaskSearchManager::projectSearchHeight(double candidate_z, double current_z) const {
  // 2026-07-13: frontier 的 z 只说明观测点来源，不应直接成为窄道飞行高度；按当前高度缓慢回到巡航层。
  (void)candidate_z;
  const double current = clampSearchHeight(current_z);
  const double target = clampSearchHeight(cruise_height_);
  const double delta = target - current;
  if (delta > 0.0) return current + std::min(delta, std::max(0.0, max_goal_climb_));
  return current + std::max(delta, -std::max(0.0, max_goal_descent_));
}

void TaskSearchManager::recordSelectedGoal(const Eigen::Vector3d& goal) {
  // 2026-07-13: 同一目标的重规划不再重复写满历史，否则几次安全重规划就会被误判为已搜索完成。
  if (selected_goals_.empty() ||
      (selected_goals_.back().head<2>() - goal.head<2>()).norm() > 0.15)
    selected_goals_.push_back(goal);
  while (static_cast<int>(selected_goals_.size()) > max_history_size_)
    selected_goals_.pop_front();
  active_goal_ = goal;
  active_goal_stamp_ = ros::Time::now();
  active_goal_valid_ = true;
}

void TaskSearchManager::reportGoalFailure(const Eigen::Vector3d& goal) {
  // 2026-07-13: A*/足迹检查失败的目标短时拉黑，规划器不能以 100Hz 重试同一点。
  const ros::Time now = ros::Time::now();
  failed_goals_.emplace_back(goal, now);
  while (failed_goals_.size() > 20) failed_goals_.pop_front();
  // 任一实际规划/安全失败都必须解除活动目标，否则3s目标保持会把下一轮重新拉回旧点。
  active_goal_valid_ = false;

  Eigen::Vector3d robot_pos;
  bool robot_pose_valid = false;
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    robot_pos = latest_robot_pos_;
    robot_pose_valid = latest_robot_pose_valid_;
  }
  const Eigen::Vector2d failed_delta = goal.head<2>() - robot_pos.head<2>();
  if (!robot_pose_valid || failed_delta.norm() < 0.20) return;

  const Eigen::Vector2d failed_direction = failed_delta.normalized();
  const bool same_recent_direction =
      recent_failed_direction_.norm() > 1e-3 &&
      (now - recent_failed_direction_stamp_).toSec() <= failed_goal_cooldown_ &&
      failed_direction.dot(recent_failed_direction_) >= std::cos(direction_sector_rad_);
  recent_failed_direction_count_ = same_recent_direction
                                       ? recent_failed_direction_count_ + 1
                                       : 1;
  recent_failed_direction_ = failed_direction;
  recent_failed_direction_stamp_ = now;
  if (recent_failed_direction_count_ >= failed_direction_threshold_) {
    paused_failed_direction_ = failed_direction;
    failed_direction_pause_until_ = now + ros::Duration(failed_direction_pause_);
    recent_failed_direction_count_ = 0;
    ROS_WARN("[task_search] pause repeatedly failed direction for %.1fs.",
             failed_direction_pause_);
  }
}

bool TaskSearchManager::goalTemporarilyBlocked(const Eigen::Vector3d& goal) const {
  const ros::Time now = ros::Time::now();
  for (const auto& failed : failed_goals_) {
    if ((now - failed.second).toSec() <= failed_goal_cooldown_ &&
        (failed.first.head<2>() - goal.head<2>()).norm() < failed_goal_radius_)
      return true;
  }
  return false;
}

bool TaskSearchManager::goalDirectionTemporarilyBlocked(
    const Eigen::Vector3d& goal) const {
  const ros::Time now = ros::Time::now();
  if (paused_failed_direction_.norm() < 1e-3 ||
      now > failed_direction_pause_until_)
    return false;

  Eigen::Vector3d robot_pos;
  bool robot_pose_valid = false;
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    robot_pos = latest_robot_pos_;
    robot_pose_valid = latest_robot_pose_valid_;
  }
  const Eigen::Vector2d candidate_delta = goal.head<2>() - robot_pos.head<2>();
  if (!robot_pose_valid || candidate_delta.norm() < 0.20) return false;
  return candidate_delta.normalized().dot(paused_failed_direction_) >=
         std::cos(0.5 * direction_sector_rad_);
}

void TaskSearchManager::colorDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  registerDetection(0, *msg);
}
void TaskSearchManager::qrcodeDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  registerDetection(1, *msg);
}
void TaskSearchManager::thermalDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  registerDetection(2, *msg);
}

void TaskSearchManager::finalLandingMarkerCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // 通道内看到相似图案也不得触发降落；只有确认穿过出口后才登记平台 ArUco。
  if (mission_stage_ != SEARCH_OUTSIDE_LANDING &&
      mission_stage_ != APPROACH_LANDING) {
    ROS_WARN_THROTTLE(
        1.0, "[exit_mission] ignore landing ArUco before confirmed exit crossing.");
    return;
  }
  const ros::Time now = ros::Time::now();
  const Eigen::Vector2d point(msg->pose.position.x, msg->pose.position.y);
  const Eigen::Vector2d previous(final_landing_marker_.pose.pose.position.x,
                                 final_landing_marker_.pose.pose.position.y);
  const bool continuous = final_landing_marker_hits_ > 0 &&
                          (now - final_landing_marker_.stamp).toSec() <= 1.5 &&
                          (point - previous).norm() <=
                              final_landing_marker_consistency_radius_;
  final_landing_marker_hits_ = continuous ? final_landing_marker_hits_ + 1 : 1;
  final_landing_marker_.pose = *msg;
  final_landing_marker_.stamp = now;

  if (final_landing_marker_hits_ >=
      final_landing_marker_confirmation_count_) {
    final_landing_marker_.found = true;
    ROS_ERROR("[exit_mission] LANDING ARUCO confirmed %d/%d at "
              "world=(%.2f, %.2f, %.2f).",
              final_landing_marker_hits_,
              final_landing_marker_confirmation_count_, msg->pose.position.x,
              msg->pose.position.y, msg->pose.position.z);
  } else {
    ROS_WARN("[exit_mission] landing ArUco confirmation %d/%d at "
             "(%.2f, %.2f).",
             final_landing_marker_hits_,
             final_landing_marker_confirmation_count_, point.x(), point.y());
  }
  publishSearchState();
}

void TaskSearchManager::registerDetection(int type, const geometry_msgs::PoseStamped& msg) {
  if (type < 0 || type >= 3) return;
  targets_[type].found = true;
  targets_[type].pose = msg;
  targets_[type].stamp = ros::Time::now();

  const char* names[] = {"COLOR", "QRCODE", "THERMAL"};
  Eigen::Vector3d point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Vector3d corridor_point = point;
  if (corridor_frame_received_) {
    const Eigen::Vector3d lateral(-corridor_dir_.y(), corridor_dir_.x(), 0.0);
    const Eigen::Vector3d rel = point - corridor_origin_;
    corridor_point = Eigen::Vector3d(rel.dot(corridor_dir_), rel.dot(lateral), rel.z());
  }
  ROS_WARN("[task_search] target %s registered world=(%.2f, %.2f, %.2f) corridor=(%.2f, %.2f, %.2f).",
           names[type], point.x(), point.y(), point.z(), corridor_point.x(), corridor_point.y(),
           corridor_point.z());
  publishSearchState();
}

bool TaskSearchManager::allStage2TargetsFound() const {
  return targets_[0].found && targets_[1].found && targets_[2].found;
}

bool TaskSearchManager::mapPointSafe(const Eigen::Vector3d& point) const {
  if (!sdf_map_ || !sdf_map_->isInMap(point) ||
      sdf_map_->getOccupancy(point) != SDFMap::FREE ||
      sdf_map_->getInflateOccupancy(point) != 0)
    return false;

  // 2026-07-13: 与轨迹安全层一致，中心查 0.15m 膨胀、机体圆周查原始占据，禁止双重膨胀堵死通道。
  for (int sample = 0; sample < exit_footprint_samples_; ++sample) {
    const double angle = 2.0 * M_PI * static_cast<double>(sample) /
                         static_cast<double>(exit_footprint_samples_);
    Eigen::Vector3d probe = point;
    probe.x() += exit_footprint_extra_radius_ * std::cos(angle);
    probe.y() += exit_footprint_extra_radius_ * std::sin(angle);
    if (!sdf_map_->isInMap(probe) || sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
      return false;
  }
  return true;
}

bool TaskSearchManager::landingColumnSafe(const Eigen::Vector3d& marker,
                                          const Eigen::Vector3d& approach) const {
  if (approach.z() <= marker.z()) return false;
  const double start_z =
      marker.z() + std::max(0.05, landing_column_bottom_clearance_);
  const double step = std::max(0.05, landing_column_step_);
  for (double z = start_z; z <= approach.z() + 1e-6; z += step) {
    if (!mapPointSafe(Eigen::Vector3d(marker.x(), marker.y(), z))) return false;
  }
  return mapPointSafe(approach);
}

// 2026-07-23: 只在当前里程计高度上下各0.40m内投影占据，地图和里程计共同漂移时仍保持相对一致；
// 不再用固定世界高度找“门柱”，同时避开明显低于机体的地面和明显高于机体的顶棚。
bool TaskSearchManager::mapRelativeColumnOccupied(const Eigen::Vector3d& point,
                                                  double reference_z) const {
  if (!sdf_map_) return false;
  Eigen::Vector3d box_min, box_max;
  sdf_map_->getBox(box_min, box_max);
  const double min_z = std::max(box_min.z() + 0.05, reference_z - 0.40);
  const double max_z = std::min(box_max.z() - 0.05, reference_z + 0.40);
  if (max_z < min_z) return false;
  const double xy_offsets[][2] = {{0.0, 0.0}, {0.06, 0.0}, {-0.06, 0.0},
                                  {0.0, 0.06}, {0.0, -0.06}};
  int occupied_layers = 0;
  for (double z = min_z; z <= max_z + 1e-6; z += 0.10) {
    bool occupied = false;
    for (const auto& offset : xy_offsets) {
      Eigen::Vector3d probe(point.x() + offset[0], point.y() + offset[1], z);
      if (sdf_map_->isInMap(probe) &&
          sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED) {
        occupied = true;
        break;
      }
    }
    if (occupied) ++occupied_layers;
  }
  return occupied_layers >= 1;
}

// 在当前飞行高度附近判定双侧开阔；原始占据优先，UNKNOWN不能冒充净空。
bool TaskSearchManager::detectLocalMapExit(const Eigen::Vector3d& cur_pos,
                                           const Eigen::Vector2d& forward_hint,
                                           OpenExitResult& result) const {
  if (!sdf_map_ || forward_hint.norm() < 1e-3) return false;
  open_space_exit::Config config;
  config.search_max = exit_search_max_forward_;
  config.min_open_width = exit_min_open_width_;
  config.min_open_ratio = exit_min_open_ratio_;
  config.sector_ray_count = exit_sector_ray_count_;
  config.sector_half_angle_deg = exit_sector_half_angle_deg_;
  config.inside_length = exit_inside_length_;
  config.outside_length = exit_outside_length_;
  const Eigen::Vector2d incoming = forward_hint.normalized();
  const double correction = std::min(15.0, std::max(0.0,
      exit_candidate_angle_tolerance_deg_)) * M_PI / 180.0;
  open_space_exit::Diagnostics total;
  for (double angle : {0.0, correction, -correction}) {
    const Eigen::Vector2d forward(
        std::cos(angle) * incoming.x() - std::sin(angle) * incoming.y(),
        std::sin(angle) * incoming.x() + std::cos(angle) * incoming.y());
    const Eigen::Vector2d lateral(-forward.y(), forward.x());
    auto pointAt = [&](double along, double across) {
      Eigen::Vector3d point = cur_pos;
      point.head<2>() += along * forward + across * lateral;
      return point;
    };
    auto occupied = [&](double along, double across) {
      return mapRelativeColumnOccupied(pointAt(along, across), cur_pos.z());
    };
    auto known_free = [&](double along, double across) {
      Eigen::Vector3d point = pointAt(along, across);
      for (double offset : {-0.10, 0.0, 0.10}) {
        point.z() = cur_pos.z() + offset;
        if (sdf_map_->isInMap(point) && sdf_map_->getOccupancy(point) == SDFMap::FREE)
          return true;
      }
      return false;
    };
    open_space_exit::Result detection;
    open_space_exit::Diagnostics diagnostics;
    if (!open_space_exit::detect(config, occupied, known_free,
        [&](double along, double across) { return mapExitFootprintFree(pointAt(along, across)); },
        detection, &diagnostics)) {
      total.supported_sections += diagnostics.supported_sections;
      total.widened_sections += diagnostics.widened_sections;
      total.clear_sections += diagnostics.clear_sections;
      continue;
    }
    result.portal_center = pointAt(detection.forward, detection.lateral);
    result.outward_direction = forward;
    result.left_open_ratio = detection.left_open_ratio;
    result.right_open_ratio = detection.right_open_ratio;
    result.confidence = detection.confidence;
    ROS_INFO_THROTTLE(1.0, "[exit_local_map] open transition=(%.2f,%.2f) "
                      "yaw=%.1fdeg ahead_width=%.3fm width_ratio=%.2f.",
                      result.portal_center.x(), result.portal_center.y(),
                      std::atan2(forward.y(), forward.x()) * 180.0 / M_PI,
                      detection.open_width, result.left_open_ratio);
    return true;
  }
  const char* reason = total.supported_sections == 0 ? "inside_walls_missing" :
      total.widened_sections == 0 ? "bilateral_widening_unobserved_or_blocked" :
      total.clear_sections == 0 ? "body_path_unobserved_or_blocked" :
      "outside_clearance_unobserved_or_blocked";
  ROS_INFO_THROTTLE(1.0, "[exit_local_map] no exit: reason=%s inside=%d widened=%d body_clear=%d.",
                    reason, total.supported_sections, total.widened_sections, total.clear_sections);
  return false;
}

bool TaskSearchManager::mapExitFootprintFree(const Eigen::Vector3d& center,
                                             double bottom_height) const {
  if (!sdf_map_ || !sdf_map_->isInMap(center) ||
      sdf_map_->getInflateOccupancy(center) != 0) return false;
  const double step = std::max(0.05, sdf_map_->getResolution());
  const double radius = std::max(0.0, exit_footprint_extra_radius_);
  const double half_height = std::max(0.0, exit_footprint_half_height_);
  const int xy_bins = std::max(1, static_cast<int>(std::ceil(radius / step)));
  const int z_bins = std::max(1, static_cast<int>(std::ceil(half_height / step)));
  const double bottom = bottom_height < 0.0 ? half_height : bottom_height;
  const int bottom_bins = std::max(1,static_cast<int>(std::ceil(bottom/step)));
  for (int x = -xy_bins; x <= xy_bins; ++x) {
    for (int y = -xy_bins; y <= xy_bins; ++y) {
      const double dx = radius * x / xy_bins, dy = radius * y / xy_bins;
      if (dx * dx + dy * dy > radius * radius + 1e-9) continue;
      for (int z = -bottom_bins; z <= z_bins; ++z) {
        const Eigen::Vector3d point = center +
            Eigen::Vector3d(dx, dy, z < 0 ? bottom*z/bottom_bins : half_height*z/z_bins);
        if (!sdf_map_->isInMap(point) || sdf_map_->getOccupancy(point) != SDFMap::FREE)
          return false;
      }
    }
  }
  return true;
}

// 出口结构已经确认后，沿锁定法向做短步穿越。已知FREE优先；若门外仍是UNKNOWN，
// 只要目标足迹和整段轴线都没有原始OCCUPIED，也允许一步受控探测，打破“必须先看到FREE
// 才肯出门、但不出门又看不到FREE”的死锁。该放宽仅存在于 CROSS_EXIT。
bool TaskSearchManager::buildSafeExitCrossingGoal(const Eigen::Vector3d& cur_pos,
                                                  Eigen::Vector3d& goal) const {
  if (!exit_candidate_confirmed_ || exit_outward_direction_.norm() < 1e-3) return false;
  const Eigen::Vector2d direction = exit_outward_direction_.normalized();
  const auto raw_footprint_clear = [this](const Eigen::Vector3d& point) {
    if (!sdf_map_ || !sdf_map_->isInMap(point) ||
        sdf_map_->getOccupancy(point) == SDFMap::OCCUPIED)
      return false;
    for (int sample = 0; sample < exit_footprint_samples_; ++sample) {
      const double angle = 2.0 * M_PI * static_cast<double>(sample) /
                           static_cast<double>(exit_footprint_samples_);
      Eigen::Vector3d probe = point;
      probe.x() += exit_footprint_extra_radius_ * std::cos(angle);
      probe.y() += exit_footprint_extra_radius_ * std::sin(angle);
      if (!sdf_map_->isInMap(probe) ||
          sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
        return false;
    }
    return true;
  };
  const auto raw_segment_clear = [&raw_footprint_clear](const Eigen::Vector3d& start,
                                                        const Eigen::Vector3d& end) {
    const double length = (end - start).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.08)));
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(samples);
      if (!raw_footprint_clear(start + ratio * (end - start))) return false;
    }
    return true;
  };
  const double current_side =
      (cur_pos.head<2>() - exit_portal_center_.head<2>()).dot(direction);
  const double desired_side = std::min(exit_cross_target_distance_,
                                       std::max(0.0, current_side) + exit_cross_step_);
  bool found = false;
  for (double side = desired_side; side >= -0.05; side -= 0.10) {
    Eigen::Vector3d candidate = exit_portal_center_;
    candidate.head<2>() += side * direction;
    candidate.z() = cruise_height_;
    const bool known_free = mapPointSafe(candidate);
    if ((!known_free &&
         (!raw_footprint_clear(candidate) || !raw_segment_clear(cur_pos, candidate))) ||
        goalTemporarilyBlocked(candidate))
      continue;
    goal = candidate;
    if (!known_free)
      ROS_WARN_THROTTLE(1.0,
                        "[exit_mission] CROSS_EXIT uses occupied-free UNKNOWN probe "
                        "side=%.2fm step=%.2fm.",
                        side, (candidate.head<2>() - cur_pos.head<2>()).norm());
    found = true;
    break;
  }
  return found;
}

// 候选开阔区域已有证据但尚未近场确认时，保留门内小范围观察目标。
// 观察本身不确认出口、不穿入UNKNOWN，也不调用旧frontier回头。
bool TaskSearchManager::buildPendingExitObservationGoal(
    const Eigen::Vector3d& cur_pos, Eigen::Vector3d& goal, double& goal_yaw) const {
  if (exit_candidate_confirmed_ ||
      exit_candidate_hits_ < std::max(1, exit_pending_observation_min_hits_) ||
      !pending_exit_portal_outside_verified_ ||
      pending_exit_outward_direction_.norm() < 1e-3)
    return false;

  const Eigen::Vector2d direction = pending_exit_outward_direction_.normalized();
  const Eigen::Vector2d progress_direction = stableProgressDirection();
  const double direction_dot = std::max(
      -1.0, std::min(1.0, direction.dot(progress_direction)));
  const double direction_change_deg = std::acos(direction_dot) * 180.0 / M_PI;
  const double portal_forward =
      (pending_exit_portal_center_.head<2>() - cur_pos.head<2>())
          .dot(progress_direction);
  if (direction_change_deg > exit_candidate_angle_tolerance_deg_ ||
      portal_forward < -exit_candidate_max_behind_distance_) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[exit_mission] reject pending observation behind progress: "
        "direction_jump=%.1f/%.1fdeg portal_forward=%.2f/%.2fm.",
        direction_change_deg, exit_candidate_angle_tolerance_deg_, portal_forward,
        -exit_candidate_max_behind_distance_);
    return false;
  }
  const Eigen::Vector2d lateral(-direction.y(), direction.x());
  const double lateral_offset = std::max(0.10, exit_pending_observation_lateral_offset_);
  const double min_move = std::max(0.10, exit_pending_observation_min_move_);
  std::vector<Eigen::Vector3d> candidates;
  for (double inward_offset : {0.0, 0.20}) {
    for (double side : {0.0, 1.0, -1.0}) {
      Eigen::Vector3d candidate = pending_exit_candidate_;
      candidate.head<2>() -= inward_offset * direction;
      candidate.head<2>() += side * lateral_offset * lateral;
      candidate.z() = clampSearchHeight(pending_exit_candidate_.z());
      candidates.push_back(candidate);
    }
  }

  double best_travel = std::numeric_limits<double>::infinity();
  double best_clearance = -1.0;
  bool best_center = false;
  bool found = false;
  for (const auto& candidate : candidates) {
    const double travel = (candidate.head<2>() - cur_pos.head<2>()).norm();
    if (travel + 1e-6 < min_move || !mapPointSafe(candidate) ||
        goalTemporarilyBlocked(candidate))
      continue;
    // 中心安全时不能因无人机已偏向一侧而继续选择更近的侧点。
    // 两个中心点按距离排序；中心不可用时侧点先比净空，再比距离。
    // 可达性及整条轨迹的碰撞检查仍由后续A*和轨迹发布流程执行。
    const bool center = std::fabs(
        (candidate.head<2>() - pending_exit_candidate_.head<2>()).dot(lateral)) < 1e-6;
    const double clearance = center ? 0.0 : knownHorizontalClearance(candidate);
    const bool better_same_class = center
        ? travel < best_travel
        : clearance > best_clearance + 1e-6 ||
          (std::fabs(clearance - best_clearance) <= 1e-6 && travel < best_travel);
    if (!found || (center && !best_center) ||
        (center == best_center && better_same_class)) {
      best_travel = travel;
      best_clearance = clearance;
      best_center = center;
      goal = candidate;
      found = true;
    }
  }
  if (!found) return false;
  // 观察点可横移避障，但机头始终沿出口法向，不能随侧向候选点偏转。
  goal_yaw = std::atan2(direction.y(), direction.x());
  return true;
}

void TaskSearchManager::clearExitEvidence() {
  exit_candidate_hits_ = 0;
  exit_candidate_confirmed_ = false;
  pending_exit_portal_outside_verified_ = false;
  exit_portal_outside_verified_ = false;
  exit_verification_hold_until_ = ros::Time(0);
}

bool TaskSearchManager::consumeExitMapUpdate(std::uint64_t sequence, const ros::Time& now) {
  if (sequence == last_exit_map_sequence_ ||
      (!last_exit_inference_.isZero() &&
       (now - last_exit_inference_).toSec() < exit_inference_period_)) return false;
  last_exit_map_sequence_ = sequence;
  last_exit_inference_ = now;
  return true;
}

bool TaskSearchManager::recordExitEvidence(const OpenExitResult& detection,
                                           const Eigen::Vector3d& inside,
                                           const Eigen::Vector3d& cur_pos,
                                           const ros::Time& now) {
  const Eigen::Vector2d direction = detection.outward_direction.normalized();
  const bool same_candidate = exit_candidate_hits_ > 0 &&
      (detection.portal_center.head<2>() - pending_exit_portal_center_.head<2>()).norm() <=
          exit_candidate_stability_radius_ &&
      direction.dot(pending_exit_outward_direction_) + 1e-9 >=
          std::cos(exit_candidate_angle_tolerance_deg_ * M_PI / 180.0);
  if (same_candidate) {
    ++exit_candidate_hits_;
    pending_exit_candidate_ = 0.65 * pending_exit_candidate_ + 0.35 * inside;
    // 两个有效观察点之间的平均点可能落入墙尾或未知体素，不能因此丢掉有效票。
    // inside 已由本次调用方完成机体范围检查，平均点不安全时使用该实测有效点。
    if (sdf_map_ && !mapExitFootprintFree(pending_exit_candidate_))
      pending_exit_candidate_ = inside;
    pending_exit_portal_center_ =
        0.65 * pending_exit_portal_center_ + 0.35 * detection.portal_center;
    pending_exit_outward_direction_ =
        (0.65 * pending_exit_outward_direction_ + 0.35 * direction).normalized();
  } else {
    clearExitEvidence();
    exit_candidate_hits_ = 1;
    pending_exit_candidate_ = inside;
    pending_exit_portal_center_ = detection.portal_center;
    pending_exit_outward_direction_ = direction;
  }
  pending_exit_portal_outside_verified_ = true;
  exit_verification_hold_until_ =
      now + ros::Duration(std::max(0.0, exit_verification_yaw_hold_time_));
  const double distance = (cur_pos.head<2>() - pending_exit_portal_center_.head<2>()).norm();
  exit_candidate_confirmed_ = exit_candidate_hits_ >= std::max(1, exit_confirmation_count_) &&
                              distance <= exit_confirmation_max_distance_;
  exit_portal_outside_verified_ = exit_candidate_confirmed_;
  if (exit_candidate_confirmed_) {
    exit_candidate_ = pending_exit_candidate_;
    exit_portal_center_ = pending_exit_portal_center_;
    exit_outward_direction_ = pending_exit_outward_direction_;
    exit_portal_outside_verified_ = true;
  }
  ROS_INFO("[exit_mission] open exit pending=%d/%d distance=%.2f/%.2fm confirmed=%d.",
           exit_candidate_hits_, exit_confirmation_count_, distance,
           exit_confirmation_max_distance_, static_cast<int>(exit_candidate_confirmed_));
  return exit_candidate_confirmed_;
}

void TaskSearchManager::lockExitPortal(const ros::Time& now) {
  exit_portal_locked_ = true;
  active_goal_valid_ = false;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = world_frame_;
  pose.pose.position.x = exit_portal_center_.x();
  pose.pose.position.y = exit_portal_center_.y();
  pose.pose.position.z = exit_portal_center_.z();
  const double yaw = std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
  pose.pose.orientation.w = std::cos(0.5 * yaw);
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  if (final_exit_pose_pub_) final_exit_pose_pub_.publish(pose);
  if (exit_pose_pub_) exit_pose_pub_.publish(pose);
  ROS_ERROR("[exit_mission] FINAL OPEN EXIT LOCKED center=(%.2f,%.2f) yaw=%.1fdeg.",
            exit_portal_center_.x(), exit_portal_center_.y(), yaw * 180.0 / M_PI);
}

void TaskSearchManager::beginExitCrossing(const ros::Time& now) {
  if (!exit_portal_locked_) lockExitPortal(now);
  mission_stage_ = EXIT_APPROACH_INSIDE;
  mission_stage_start_ = now;
  active_goal_valid_ = false;
  ROS_ERROR("[exit_mission] SEARCH_CORRIDOR -> EXIT_APPROACH_INSIDE: "
            "confirmed exit latched; approach and cross before landing search.");
}

bool TaskSearchManager::confirmedExitPortal(
    Eigen::Vector3d& portal, Eigen::Vector2d& outward) const {
  if (!enabled_ || !exit_detection_enabled_ || !exit_portal_locked_ ||
      !exit_candidate_confirmed_ || !exit_portal_outside_verified_) return false;
  portal = exit_portal_center_;
  outward = exit_outward_direction_;
  return true;
}

bool TaskSearchManager::resumeDeferredExitCrossing() {
  Eigen::Vector3d portal;
  Eigen::Vector2d outward;
  if (rc_land_active_ || !exit_crossing_deferred_ ||
      mission_stage_ != SEARCH_CORRIDOR || !confirmedExitPortal(portal, outward)) return false;
  exit_crossing_deferred_ = false;
  beginExitCrossing(ros::Time::now());
  publishSearchState();
  return true;
}

void TaskSearchManager::updateExitCandidate(const Eigen::Vector3d& cur_pos) {
  if (rc_land_active_ || !enabled_ || !exit_detection_enabled_ || !sdf_map_ ||
      !corridor_frame_received_ || !corridor_entry_crossed_ ||
      mission_stage_ != SEARCH_CORRIDOR || exit_portal_locked_) return;
  // 复用现有转弯流程，过弯期间不能锁定出口。
  if (transition_active_ || pending_turn_confirmations_ > 0 || turn_yaw_follow_latch_.active()) {
    ROS_INFO_THROTTLE(1.0,
                     "[exit_local_map] no exit: turn_transition=%d yaw_alignment=%d pending_turn=%d.",
                     static_cast<int>(transition_active_),
                     static_cast<int>(turn_yaw_follow_latch_.active()), pending_turn_confirmations_);
    clearExitEvidence();
    publishSearchState();
    return;
  }
  const ros::Time now = ros::Time::now();
  if (!consumeExitMapUpdate(sdf_map_->getUpdateSequence(), now)) return;
  const Eigen::Vector2d forward = stableProgressDirection();
  Eigen::Vector3d turn_direction;
  double forward_free = 0.0, turn_free = 0.0;
  if (inferOccupancyTurnDirection(Eigen::Vector3d(forward.x(), forward.y(), 0.0),
                                 turn_direction, forward_free, turn_free)) {
    clearExitEvidence();
    publishSearchState();
    return;
  }
  OpenExitResult detection;
  if (!detectLocalMapExit(cur_pos, forward, detection) ||
      (detection.portal_center.head<2>() - cur_pos.head<2>()).dot(forward) <
          -exit_candidate_max_behind_distance_) {
    clearExitEvidence();
    publishSearchState();
    return;
  }
  Eigen::Vector3d inside;
  bool safe_inside = false;
  for (double standoff = exit_standoff_; standoff >= 0.15; standoff -= 0.10) {
    inside = detection.portal_center;
    inside.head<2>() -= standoff * detection.outward_direction;
    if (mapExitFootprintFree(inside)) {
      safe_inside = true;
      break;
    }
  }
  if (!safe_inside) {
    clearExitEvidence();
    publishSearchState();
    return;
  }
  recordExitEvidence(detection, inside, cur_pos, now);
  if (!mapExitFootprintFree(pending_exit_candidate_)) clearExitEvidence();
  // 本次地图确认有效后立即锁存，下一轮规划前的地图更新不能撤销或替换出口。
  if (exit_auto_crossing_enabled_ && exit_candidate_confirmed_ &&
      (!require_stage2_detections_ || allStage2TargetsFound())) {
    if (exit_crossing_deferred_) lockExitPortal(now);
    else beginExitCrossing(now);
  }
  publishSearchState();
}

bool TaskSearchManager::buildStage3Goal(const Eigen::Vector3d& cur_pos, double cur_yaw,
                                        bool search_exhausted, Eigen::Vector3d& goal,
                                        double& goal_yaw) {
  // 出口总开关关闭时，保持普通通道探索，不推断出口、不切换出口任务阶段。
  if (rc_land_active_ || !enabled_ || !exit_detection_enabled_ || landing_requested_) return false;
  updateExitCandidate(cur_pos);

  if (mission_stage_ == SEARCH_CORRIDOR) {
    // 完整飞行观察时继续更新出口证据和显示，但不生成接近/穿越/补扫目标。
    // CH9人工入口不受此自动接管开关影响。
    if (!exit_auto_crossing_enabled_ || exit_crossing_deferred_) return false;
    // 任务允许接管时才锁定出口和冻结frontier，确认本身不阻止剩余目标搜索。
    if (require_stage2_detections_ && !allStage2TargetsFound()) return false;
    if (!exit_candidate_confirmed_) {
      // 稳定候选优先接近到近场确认范围，不等待普通frontier耗尽。
      // 这里只生成门内观察目标，1米确认距离和穿越条件不变。
      const bool stable_pending_exit =
          exit_candidate_hits_ >= std::max(1, exit_confirmation_count_);
      return (search_exhausted || stable_pending_exit) &&
          buildPendingExitObservationGoal(cur_pos, goal, goal_yaw);
    }
    if (!exit_portal_outside_verified_) return false;
    beginExitCrossing(ros::Time::now());
  }

  if (mission_stage_ == EXIT_APPROACH_INSIDE) {
    const double signed_side =
        (cur_pos.head<2>() - exit_portal_center_.head<2>()).dot(exit_outward_direction_);
    // 外部Diff必须到锁定门心附近才接管；越过门平面但偏离门心时仍返回门心。
    Eigen::Vector3d approach =
        external_landing_planner_ ? exit_portal_center_ : exit_candidate_;
    if (external_landing_planner_) approach.z() = cruise_height_;
    const double arrive_distance =
        external_landing_planner_ ? exit_portal_arrive_distance_ : exit_arrive_distance_;
    Eigen::Vector3d arrival_pos = cur_pos;
    if (external_landing_planner_) {
      // 预测规划起点不能代替真实里程计完成出口交接。
      std::lock_guard<std::mutex> lock(body_cloud_mutex_);
      if (latest_robot_pose_valid_) arrival_pos = latest_robot_pos_;
    }
    const bool height_ready = !external_landing_planner_ ||
        std::fabs(arrival_pos.z() - cruise_height_) <= exit_portal_arrive_distance_;
    if (height_ready &&
        ((arrival_pos.head<2>() - approach.head<2>()).norm() <= arrive_distance ||
         (!external_landing_planner_ && signed_side >= exit_cross_confirm_distance_))) {
      if (external_landing_planner_) {
        mission_stage_ = SEARCH_OUTSIDE_LANDING;
        mission_stage_start_ = ros::Time::now();
        active_goal_valid_ = false;
        outside_search_anchor_ = cur_pos;
        outside_search_anchor_.z() = cruise_height_;
        search_exhausted_since_ = ros::Time(0);
        publishSearchState();
        ROS_ERROR("[exit_mission] EXIT_APPROACH_INSIDE -> SEARCH_OUTSIDE_LANDING: "
                  "actual exit center reached; Diff owns the 1m outward crossing.");
        return false;
      }
      mission_stage_ = CROSS_EXIT;
      mission_stage_start_ = ros::Time::now();
      active_goal_valid_ = false;
      ROS_ERROR("[exit_mission] stage EXIT_APPROACH_INSIDE -> CROSS_EXIT, portal=(%.2f, %.2f).",
                exit_portal_center_.x(), exit_portal_center_.y());
    } else {
      goal = approach;
      if (!mapPointSafe(goal) || goalTemporarilyBlocked(goal)) {
        ROS_WARN_THROTTLE(1.0, "[exit_mission] verified exit approach temporarily blocked.");
        return false;
      }
      // 回到门心只修正位置，不因门法向或倒退方向而改变当前机头朝向。
      goal_yaw = external_landing_planner_ ? cur_yaw :
          std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
      return true;
    }
  }

  if (mission_stage_ == CROSS_EXIT) {
    const double signed_side =
        (cur_pos.head<2>() - exit_portal_center_.head<2>()).dot(exit_outward_direction_);
    if (signed_side >= exit_cross_confirm_distance_) {
      mission_stage_ = SEARCH_OUTSIDE_LANDING;
      mission_stage_start_ = ros::Time::now();
      active_goal_valid_ = false;
      outside_search_anchor_ = cur_pos;
      outside_search_anchor_.z() = cruise_height_;
      search_exhausted_since_ = ros::Time(0);
      publishSearchState();
      ROS_ERROR("[exit_mission] stage CROSS_EXIT -> SEARCH_OUTSIDE_LANDING, "
                "signed_side=%.2fm.",
                signed_side);
    } else {
      if (!buildSafeExitCrossingGoal(cur_pos, goal)) {
        ROS_WARN_THROTTLE(1.0,
                          "[exit_mission] CROSS_EXIT has no OCCUPIED-free footprint/axis step.");
        return false;
      }
      goal_yaw = std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
      return true;
    }
  }

  if (mission_stage_ == SEARCH_OUTSIDE_LANDING) {
    // 出口只是FUEL与Diff的职责分界。启用外部链路后，FUEL不再选择任何门外
    // 搜索/接近目标，也不再发布降落请求；其残余轨迹输出由规划器仲裁器隔离。
    if (external_landing_planner_) return false;
    if (final_landing_marker_.found) {
      mission_stage_ = APPROACH_LANDING;
      mission_stage_start_ = ros::Time::now();
      active_goal_valid_ = false;
      publishSearchState();
      ROS_ERROR("[exit_mission] stage SEARCH_OUTSIDE_LANDING -> APPROACH_LANDING.");
    } else {
      // 门外仍有 frontier 时继续覆盖；耗尽后围绕出口锚点做局部平台搜索。
      if (!search_exhausted) return false;
      const int phase = static_cast<int>(
          std::floor((ros::Time::now() - mission_stage_start_).toSec() /
                     std::max(0.3, scan_dwell_time_))) % 8;
      for (int attempt = 0; attempt < 8; ++attempt) {
        const double angle = (phase + attempt) * M_PI_4;
        Eigen::Vector3d scan_goal = outside_search_anchor_;
        scan_goal.x() += scan_radius_ * std::cos(angle);
        scan_goal.y() += scan_radius_ * std::sin(angle);
        if (!mapPointSafe(scan_goal) || goalTemporarilyBlocked(scan_goal)) continue;
        goal = scan_goal;
        goal_yaw = wrapYaw(cur_yaw + M_PI_2);
        return true;
      }
      goal = outside_search_anchor_;
      goal_yaw = wrapYaw(cur_yaw + M_PI_2);
      return mapPointSafe(goal);
    }
  }

  if (mission_stage_ == APPROACH_LANDING) {
    const Eigen::Vector3d marker(final_landing_marker_.pose.pose.position.x,
                                 final_landing_marker_.pose.pose.position.y,
                                 final_landing_marker_.pose.pose.position.z);
    goal = Eigen::Vector3d(marker.x(), marker.y(),
                           marker.z() + landing_approach_height_);
    if (!landingColumnSafe(marker, goal) || goalTemporarilyBlocked(goal)) {
      ROS_WARN_THROTTLE(1.0,
                        "[exit_mission] landing safety cylinder is not clear; "
                        "continue mapping without handoff.");
      return false;
    }
    goal_yaw = std::atan2(goal.y() - cur_pos.y(), goal.x() - cur_pos.x());
    geometry_msgs::PoseStamped landing_target = final_landing_marker_.pose;
    landing_target.header.stamp = ros::Time::now();
    landing_target.header.frame_id = world_frame_;
    landing_target.pose.position.z = goal.z();
    landing_target_pub_.publish(landing_target);
    return true;
  }

  return false;
}

void TaskSearchManager::reportSearchCoverageAvailable() {
  if (!search_exhausted_since_.isZero()) {
    // 2026-07-16: 任意可覆盖viewpoint恢复即证明地图仍可继续探索，取消临时终点切换倒计时。
    ROS_WARN("[exit_mission] coverable search target recovered; cancel exhaustion confirmation.");
    search_exhausted_since_ = ros::Time(0);
  }
}

void TaskSearchManager::publishLandingRequest(bool active) {
  std_msgs::Bool request;
  request.data = active;
  landing_request_pub_.publish(request);
  landing_requested_ = active;
}

void TaskSearchManager::publishSearchState() {
  visualization_msgs::Marker visited;
  visited.header.stamp = ros::Time::now();
  visited.header.frame_id = world_frame_;
  visited.ns = "task_search_coverage";
  visited.id = 0;
  visited.type = visualization_msgs::Marker::SPHERE_LIST;
  visited.action = visualization_msgs::Marker::ADD;
  visited.scale.x = visited.scale.y = visited.scale.z = 0.12;
  visited.color.r = 0.10;
  visited.color.g = 0.85;
  visited.color.b = 0.25;
  visited.color.a = 0.85;
  for (const auto& pos : visited_positions_) {
    geometry_msgs::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z();
    visited.points.push_back(point);
  }
  marker_pub_.publish(visited);

  const float colors[3][3] = {{1.0f, 0.85f, 0.1f}, {0.1f, 0.4f, 1.0f}, {1.0f, 0.1f, 0.1f}};
  for (int i = 0; i < 3; ++i) {
    if (!targets_[i].found) continue;
    visualization_msgs::Marker marker;
    marker.header = targets_[i].pose.header;
    if (marker.header.frame_id.empty()) marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "task_targets";
    marker.id = 10 + i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = targets_[i].pose.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.28;
    marker.color.r = colors[i][0];
    marker.color.g = colors[i][1];
    marker.color.b = colors[i][2];
    marker.color.a = 1.0;
    marker_pub_.publish(marker);
  }

  // 2026-07-23: 单通道工程模式不再在RViz显示紫色拓扑预测点；它只保留为内部找远端的
  // 辅助量，不能被误认为门、终点或门外区域。持续发送DELETE清理旧锁存Marker。
  visualization_msgs::Marker delete_endpoint;
  delete_endpoint.header.stamp = ros::Time::now();
  delete_endpoint.header.frame_id = world_frame_;
  delete_endpoint.ns = "exit_mission";
  delete_endpoint.action = visualization_msgs::Marker::DELETE;
  delete_endpoint.id = 33;
  marker_pub_.publish(delete_endpoint);
  delete_endpoint.id = 34;
  marker_pub_.publish(delete_endpoint);

  // 矩形仅标出通道向开阔区域的过渡平面；不表示检测到了实体门框。
  const bool has_exit_portal_visual = exit_candidate_confirmed_ || exit_candidate_hits_ > 0;
  if (has_exit_portal_visual) {
    const Eigen::Vector3d& visual_portal =
        exit_candidate_confirmed_ ? exit_portal_center_ : pending_exit_portal_center_;
    Eigen::Vector2d visual_outward = exit_candidate_confirmed_
        ? exit_outward_direction_ : pending_exit_outward_direction_;
    if (visual_outward.norm() < 1e-3) visual_outward = Eigen::Vector2d::UnitX();
    visual_outward.normalize();
    const Eigen::Vector2d visual_lateral(-visual_outward.y(), visual_outward.x());
    const bool confirmed = exit_candidate_confirmed_;

    visualization_msgs::Marker exit_marker;
    exit_marker.header.stamp = ros::Time::now();
    exit_marker.header.frame_id = world_frame_;
    exit_marker.ns = "exit_mission";
    exit_marker.id = 30;
    exit_marker.type = visualization_msgs::Marker::CYLINDER;
    exit_marker.action = visualization_msgs::Marker::ADD;
    exit_marker.pose.position.x = visual_portal.x();
    exit_marker.pose.position.y = visual_portal.y();
    exit_marker.pose.position.z = visual_portal.z();
    exit_marker.pose.orientation.w = 1.0;
    exit_marker.scale.x = exit_marker.scale.y = 0.45;
    exit_marker.scale.z = 0.10;
    exit_marker.color.r = confirmed ? 0.05 : 1.0;
    exit_marker.color.g = confirmed ? 1.0 : 0.48;
    exit_marker.color.b = confirmed ? 0.20 : 0.05;
    exit_marker.color.a = 0.95;
    marker_pub_.publish(exit_marker);

    visualization_msgs::Marker exit_text = exit_marker;
    exit_text.id = 31;
    exit_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    exit_text.pose.position.z += 0.45;
    exit_text.scale.z = 0.20;
    exit_text.color.r = confirmed ? 0.10 : 1.0;
    exit_text.color.g = 1.0;
    exit_text.color.b = confirmed ? 0.25 : 0.10;
    std::ostringstream exit_stream;
    if (confirmed && mission_stage_ != SEARCH_CORRIDOR) {
      static const char* stage_names[] = {
          "SEARCH", "APPROACH_INSIDE", "CROSS_EXIT",
          "SEARCH_OUTSIDE_LANDING", "APPROACH_LANDING", "LANDING"};
      const int stage_index = std::max(
          0, std::min(static_cast<int>(mission_stage_),
                      static_cast<int>(sizeof(stage_names) / sizeof(stage_names[0])) - 1));
      exit_stream << "OPEN-SPACE EXIT / " << stage_names[stage_index];
    } else if (confirmed && exit_portal_locked_) {
      // 2026-07-23: 明确区分可撤销墙端与已冻结最终门，方便RViz确认检测已经停止。
      exit_stream << "OPEN-SPACE EXIT LOCKED";
    } else if (confirmed) {
      exit_stream << "OPEN-SPACE READY / STILL INSIDE";
    } else {
      exit_stream << "OPEN-SPACE CANDIDATE / INSIDE "
                  << exit_candidate_hits_ << "/" << exit_confirmation_count_;
    }
    exit_text.text = exit_stream.str();
    marker_pub_.publish(exit_text);

    visualization_msgs::Marker exit_direction = exit_marker;
    exit_direction.id = 32;
    exit_direction.type = visualization_msgs::Marker::ARROW;
    exit_direction.points.clear();
    geometry_msgs::Point arrow_start, arrow_end;
    arrow_start = exit_marker.pose.position;
    arrow_end = arrow_start;
    arrow_end.x += 0.85 * visual_outward.x();
    arrow_end.y += 0.85 * visual_outward.y();
    exit_direction.points.push_back(arrow_start);
    exit_direction.points.push_back(arrow_end);
    exit_direction.scale.x = 0.08;
    exit_direction.scale.y = 0.16;
    exit_direction.scale.z = 0.16;
    exit_direction.color.r = confirmed ? 0.10 : 1.00;
    exit_direction.color.g = confirmed ? 0.90 : 0.55;
    exit_direction.color.b = confirmed ? 1.00 : 0.05;
    marker_pub_.publish(exit_direction);

    visualization_msgs::Marker door_frame = exit_marker;
    door_frame.id = 35;
    door_frame.type = visualization_msgs::Marker::LINE_LIST;
    // 2026-07-23: LINE_LIST顶点使用world绝对坐标，Marker位姿必须归零，避免门框被中心坐标二次平移。
    door_frame.pose.position.x = 0.0;
    door_frame.pose.position.y = 0.0;
    door_frame.pose.position.z = 0.0;
    door_frame.pose.orientation.x = 0.0;
    door_frame.pose.orientation.y = 0.0;
    door_frame.pose.orientation.z = 0.0;
    door_frame.pose.orientation.w = 1.0;
    door_frame.points.clear();
    door_frame.scale.x = 0.10;
    door_frame.color.r = confirmed ? 0.05 : 1.0;
    door_frame.color.g = confirmed ? 1.0 : 0.48;
    door_frame.color.b = confirmed ? 0.20 : 0.05;
    door_frame.color.a = 1.0;
    const double half_width = 0.55;
    const double bottom_z = std::max(0.05, visual_portal.z() - 0.55);
    const double top_z = visual_portal.z() + 0.75;
    const Eigen::Vector2d left_xy =
        visual_portal.head<2>() + half_width * visual_lateral;
    const Eigen::Vector2d right_xy =
        visual_portal.head<2>() - half_width * visual_lateral;
    auto frame_point = [](const Eigen::Vector2d& xy, double z) {
      geometry_msgs::Point point;
      point.x = xy.x();
      point.y = xy.y();
      point.z = z;
      return point;
    };
    const geometry_msgs::Point left_bottom = frame_point(left_xy, bottom_z);
    const geometry_msgs::Point left_top = frame_point(left_xy, top_z);
    const geometry_msgs::Point right_bottom = frame_point(right_xy, bottom_z);
    const geometry_msgs::Point right_top = frame_point(right_xy, top_z);
    door_frame.points = {
        left_bottom, left_top, right_bottom, right_top,
        left_top, right_top, left_bottom, right_bottom};
    marker_pub_.publish(door_frame);

    // 2026-07-23: 只有任务状态机正式升级为最终出口后才显示IN/OUT；普通墙端候选
    // 全部仍属于通道内，禁止局部几何提前制造“门外”语义。
    if (confirmed && mission_stage_ != SEARCH_CORRIDOR) {
      visualization_msgs::Marker side_text = exit_text;
      side_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      side_text.scale.z = 0.18;
      side_text.pose.position.z = visual_portal.z() + 0.18;
      side_text.id = 36;
      side_text.pose.position.x = visual_portal.x() - 0.65 * visual_outward.x();
      side_text.pose.position.y = visual_portal.y() - 0.65 * visual_outward.y();
      side_text.color.r = 0.20;
      side_text.color.g = 0.65;
      side_text.color.b = 1.00;
      side_text.text = "IN / CORRIDOR";
      marker_pub_.publish(side_text);
      side_text.id = 37;
      side_text.pose.position.x = visual_portal.x() + 0.65 * visual_outward.x();
      side_text.pose.position.y = visual_portal.y() + 0.65 * visual_outward.y();
      side_text.color.r = 0.20;
      side_text.color.g = 1.00;
      side_text.color.b = 0.30;
      side_text.text = "OUT / ARUCO-LANDING";
      marker_pub_.publish(side_text);
    } else {
      visualization_msgs::Marker delete_side = exit_marker;
      delete_side.action = visualization_msgs::Marker::DELETE;
      for (int marker_id : {36, 37}) {
        delete_side.id = marker_id;
        marker_pub_.publish(delete_side);
      }
    }
  } else {
    // 2026-07-23: 候选消失时清理锁存的中心、文字、箭头和门框，避免RViz继续显示旧出口。
    visualization_msgs::Marker delete_exit;
    delete_exit.header.stamp = ros::Time::now();
    delete_exit.header.frame_id = world_frame_;
    delete_exit.ns = "exit_mission";
    delete_exit.action = visualization_msgs::Marker::DELETE;
    for (int marker_id : {30, 31, 32, 35, 36, 37}) {
      delete_exit.id = marker_id;
      marker_pub_.publish(delete_exit);
    }
  }

  // 稳定 ArUco 世界位置确认后单独显示降落平台；未确认时禁止接近和降落。
  if (final_landing_marker_.found) {
    visualization_msgs::Marker landing_marker;
    landing_marker.header = final_landing_marker_.pose.header;
    landing_marker.header.stamp = ros::Time::now();
    if (landing_marker.header.frame_id.empty()) landing_marker.header.frame_id = world_frame_;
    landing_marker.ns = "exit_mission";
    landing_marker.id = 40;
    landing_marker.type = visualization_msgs::Marker::CUBE;
    landing_marker.action = visualization_msgs::Marker::ADD;
    landing_marker.pose = final_landing_marker_.pose.pose;
    landing_marker.scale.x = landing_marker.scale.y = 0.60;
    landing_marker.scale.z = 0.08;
    landing_marker.color.r = 0.05;
    landing_marker.color.g = 0.95;
    landing_marker.color.b = 0.95;
    landing_marker.color.a = 0.95;
    marker_pub_.publish(landing_marker);
  }

  std_msgs::String status;
  std::ostringstream stream;
  // 2026-07-13: 状态话题输出完整任务阶段，供后续第二架无人机和比赛任务面板直接订阅。
  // 状态话题明确区分门内接近、穿门和门外降落平台搜索。
  const char* stage_names[] = {"SEARCH_CORRIDOR", "EXIT_APPROACH_INSIDE", "CROSS_EXIT",
                               "SEARCH_OUTSIDE_LANDING", "APPROACH_LANDING", "LANDING"};
  stream << (rc_land_active_ ? "RC_LAND" : stage_names[static_cast<int>(mission_stage_)])
         << " color=" << targets_[0].found << " qrcode=" << targets_[1].found
         << " thermal=" << targets_[2].found << " exit=" << exit_candidate_confirmed_
         << " open_space_hits=" << exit_candidate_hits_
         << " auto_crossing=" << exit_auto_crossing_enabled_
         << " landing_aruco=" << final_landing_marker_.found
         << " landing=" << landing_requested_
         << " visited=" << visited_positions_.size();
  status.data = stream.str();
  status_pub_.publish(status);
}

}  // namespace fast_planner
