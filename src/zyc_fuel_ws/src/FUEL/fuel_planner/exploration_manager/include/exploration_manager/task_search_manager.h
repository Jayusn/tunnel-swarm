// 2026-07-13: 新增比赛第二阶段任务搜索模块，使用搜索覆盖和目标状态主导选点，frontier 仅作为候选来源。
#ifndef TASK_SEARCH_MANAGER_H_
#define TASK_SEARCH_MANAGER_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <exploration_manager/motion_direction_rules.h>
#include <exploration_manager/corridor_curve_heading.h>
#include <quadrotor_msgs/ExplorationGoal.h>

#include <deque>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace fast_planner {

class SDFMap;

class TaskSearchManager {
public:
  void initialize(ros::NodeHandle& nh);
  // 2026-07-13: 第三阶段直接读取 FUEL 累计占据地图，不依赖缺陷较大的单帧雷达点云。
  void setMap(const std::shared_ptr<SDFMap>& map);
  void setCorridorFrame(const Eigen::Vector3d& origin, const Eigen::Vector3d& inside_dir);
  void updateRobotPose(const Eigen::Vector3d& pos, double yaw);
  // 独立观察入口：首帧假定已在通道内，只更新检测，不生成接近、穿越或降落目标。
  bool updateExitDetectionOnly(const Eigen::Vector3d& pos, double yaw,
                               geometry_msgs::PoseStamped& detected_exit);
  // 连续新地图证据次数；独立入口据此保留历史标记，里程计回调不额外计票。
  int exitEvidenceCount() const { return exit_candidate_hits_; }
  int selectSearchCandidate(const std::vector<Eigen::Vector3d>& points,
                            const std::vector<double>& yaws,
                            const std::vector<std::vector<Eigen::Vector3d>>& frontiers,
                            const Eigen::Vector3d& cur_pos, double cur_yaw);
  std::vector<Eigen::Vector3d> recoveryDirections(double cur_yaw);
  // 2026-07-28: 仅供局部脱困判断真实最近航向和短回撤方向；普通frontier仍执行全局禁回头。
  Eigen::Vector3d recoveryForwardDirection(double cur_yaw);
  // 向A*提供当前通道的单调前进轴；地图确认转弯后自动切换到新通道方向。
  bool astarNoReturnDirection(Eigen::Vector3d& direction) const;
  // 查询并锁存累计占据地图确认的真实转弯，直到机头对准新通道，防止位置恢复先消费结果。
  bool mappedCorridorDirection(double cur_yaw, Eigen::Vector3d& direction,
                               bool check_entry_forward_phase = true,
                               bool require_observed_yaw = false);
  // 已建立分段后，机头偏离实际通道轴线时只校正yaw，不重复建立分段或禁回门。
  bool corridorYawCorrectionDirection(double cur_yaw,
                                      Eigen::Vector3d& direction) const;
  // 通道尚未完成转弯锁存时，若累计地图确认的延伸方向与本次轨迹方向一致，
  // 允许机头跟随轨迹，避免沿新通道平移数米仍保持入口航向。
  bool corridorMotionDirectionMatches(const Eigen::Vector2d& motion,
                                      Eigen::Vector3d& corridor_direction) const;
  // 弧形朝向按双侧墙体拟合，查询位置只取通道前向进度，不使用避障路径切线。
  bool corridorCurveYaw(const Eigen::Vector3d& position, double& yaw) const;
  bool shouldReplanCurveHeading(const Eigen::Vector3d& position,
                               double reference_yaw) const;
  bool curveHeadingActive() const { return curve_heading_active_; }
  bool turnYawAlignmentPending() const { return turn_yaw_follow_latch_.active(); }
  bool hasConfirmedCorridorTurn() const { return latest_turn_anchor_valid_; }
  void completeTurnYawAlignment() { turn_yaw_follow_latch_.clear(); }
  bool isRecoveryDirectionBackward(const Eigen::Vector3d& direction,
                                   double cur_yaw);
  bool isRecoveryCandidateUseful(const Eigen::Vector3d& candidate) const;
  bool isTaskMotionAllowed(const Eigen::Vector3d& candidate) const;
  bool isMissionBoundaryMotionAllowed(const Eigen::Vector3d& candidate) const;
  double entryStartBackMargin() const { return inside_return_margin_; }
  bool isTaskPathAllowed(const std::vector<Eigen::Vector3d>& path) const;
  // 2026-07-28: 短回撤只豁免路径开头方向，门平面、旧通道和逐点障碍约束仍全部保留。
  bool isRecoveryPathAllowed(const std::vector<Eigen::Vector3d>& path,
                             bool allow_initial_reverse) const;
  void freezeExplorationInitialYaw(double yaw);
  bool explorationInitialYawFrozen() const { return exploration_initial_yaw_frozen_; }
  void fillExplorationConstraint(quadrotor_msgs::ExplorationMotionConstraint& msg) const;
  void recordExplorationReached(const Eigen::Vector3d& goal);
  double clampSearchHeight(double z) const;
  double preferredSearchHeight() const;
  // 2026-07-13: 窄通道任务点优先保持水平飞行，仅以有限步长向巡航高度收敛。
  double projectSearchHeight(double candidate_z, double current_z) const;
  void recordSelectedGoal(const Eigen::Vector3d& goal);
  void reportGoalFailure(const Eigen::Vector3d& goal);
  void clearActiveGoal() { active_goal_valid_ = false; }
  // 双侧开阔确认且任务允许接管后切入穿越；耗尽只用于候选补扫。
  bool buildStage3Goal(const Eigen::Vector3d& cur_pos, double cur_yaw, bool search_exhausted,
                       Eigen::Vector3d& goal, double& goal_yaw);
  // 2026-07-16: 仍有frontier/viewpoint时明确撤销“搜索耗尽”计时，失败冷却不能被误当成终点条件。
  void reportSearchCoverageAvailable();
  bool landingRequested() const { return landing_requested_; }
  // CH9 已完成低->高确认时，允许 FUEL 在没有外部航点的情况下启动一次规划。
  bool consumeRcSearchLandingStartRequest();
  bool stage3Active() const { return mission_stage_ != SEARCH_CORRIDOR; }
  // 普通FUEL搜索期间入口始终是单向门，防止开阔的起飞区frontier把飞机吸回门外；
  // 只有最终出口状态机生成的专用穿门轨迹可以临时绕过入口半平面。
  bool entryWorkspaceLockActive() const { return !exitTransitActive(); }
  // 2026-07-23: 只有“到门内点/正在穿门”允许跨越最近历史航迹；一旦进入门外搜索，
  // 出口立即成为单向门，后续目标和路径必须继续留在外侧。
  bool exitTransitActive() const {
    return mission_stage_ == EXIT_APPROACH_INSIDE || mission_stage_ == CROSS_EXIT;
  }
  bool enabled() const { return enabled_; }
  // 出口正式接管后冻结普通frontier；仅检测确认不会冻结剩余任务搜索。
  bool finalExitFrontierGuardActive() const {
    return exit_portal_locked_ ||
           mission_stage_ == EXIT_APPROACH_INSIDE || mission_stage_ == CROSS_EXIT;
  }
  // 2026-07-24: 出口候选从第一票开始就属于连续雷达验证窗口；前机相机摆头必须让路，
  // 否则门框会被当作新障碍触发环扫，并把下一帧双墙终止切面打乱。
  bool exitVerificationActive() const {
    return exit_auto_crossing_enabled_ && mission_stage_ == SEARCH_CORRIDOR &&
           (exit_candidate_hits_ > 0 ||
            (!exit_verification_hold_until_.isZero() &&
             ros::Time::now() < exit_verification_hold_until_));
  }

private:
  friend class TaskSearchManagerTest;

  struct TargetRecord {
    bool found{false};
    geometry_msgs::PoseStamped pose;
    ros::Time stamp;
  };

  enum MissionStage {
    SEARCH_CORRIDOR = 0,
    EXIT_APPROACH_INSIDE = 1,
    CROSS_EXIT = 2,
    SEARCH_OUTSIDE_LANDING = 3,
    APPROACH_LANDING = 4,
    LANDING = 5
  };

  void colorDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void qrcodeDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void thermalDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void finalLandingMarkerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void rcSearchLandingCallback(const mavros_msgs::RCInConstPtr& msg);
  void activateRcSearchLanding(const ros::Time& stamp);
  void registerDetection(int type, const geometry_msgs::PoseStamped& msg);
  void updateExitCandidate(const Eigen::Vector3d& cur_pos);
  void clearExitEvidence();
  bool consumeExitMapUpdate(std::uint64_t sequence, const ros::Time& now);
  struct OpenExitResult {
    Eigen::Vector3d portal_center{0.0, 0.0, 0.0};
    Eigen::Vector2d outward_direction{1.0, 0.0};
    double confidence{0.0};
    double left_open_ratio{0.0};
    double right_open_ratio{0.0};
  };
  bool detectLocalMapExit(const Eigen::Vector3d& cur_pos,
                          const Eigen::Vector2d& forward_hint,
                          OpenExitResult& result) const;
  bool recordExitEvidence(const OpenExitResult& detection,
                          const Eigen::Vector3d& inside,
                          const Eigen::Vector3d& cur_pos, const ros::Time& now);
  void lockExitPortal(const ros::Time& now);
  void beginExitCrossing(const ros::Time& now);
  bool mapExitFootprintFree(const Eigen::Vector3d& point) const;
  bool mapRelativeColumnOccupied(const Eigen::Vector3d& point, double reference_z) const;
  // 2026-07-21: 穿门只发布已成为FREE的短滚动目标，UNKNOWN只用于证明开口延伸，不能直接飞入。
  bool buildSafeExitCrossingGoal(const Eigen::Vector3d& cur_pos, Eigen::Vector3d& goal) const;
  // 2026-07-22: 出口候选接近确认且普通frontier耗尽时，发布门内局部补扫点，避免停在门口无法补齐地图证据。
  bool buildPendingExitObservationGoal(const Eigen::Vector3d& cur_pos,
                                       Eigen::Vector3d& goal, double& goal_yaw) const;
  bool mapPointSafe(const Eigen::Vector3d& point) const;
  bool landingColumnSafe(const Eigen::Vector3d& marker,
                         const Eigen::Vector3d& approach) const;
  // 2026-07-28: 从累计占据地图的前墙和至少一侧连续墙轮廓推断正常拐弯轴线，
  // 供恢复器在90度弯道替换已经失效的历史直行切线。
  bool inferOccupancyTurnDirection(const Eigen::Vector3d& travel_direction,
                                   Eigen::Vector3d& turn_direction,
                                   double& forward_free_length,
                                   double& turn_free_length) const;
  bool turnDirectionRetracesVisitedRoute(
      const Eigen::Vector2d& anchor,
      const Eigen::Vector2d& direction) const;
  bool entryForwardPhaseActive() const;
  void commitCorridorTurn(const Eigen::Vector3d& turn_direction);
  bool confirmCorridorTurnEvidence(const Eigen::Vector3d& turn_direction);
  void clearPendingTurnEvidence();
  void updateCurveHeading();
  bool allStage2TargetsFound() const;
  bool goalTemporarilyBlocked(const Eigen::Vector3d& goal) const;
  bool goalDirectionTemporarilyBlocked(const Eigen::Vector3d& goal) const;
  void publishLandingRequest(bool active);
  void publishSearchState();
  double minDistance2D(const Eigen::Vector3d& point,
                       const std::deque<Eigen::Vector3d>& history) const;
  double knownHorizontalClearance(const Eigen::Vector3d& point) const;
  // 独立于瞬时速度维护任务推进方向：正常转弯可逐段更新，短回撤不能把前后语义翻转。
  Eigen::Vector2d stableProgressDirection() const;
  double wrapYaw(double yaw) const;

  ros::Subscriber color_detection_sub_;
  ros::Subscriber qrcode_detection_sub_;
  ros::Subscriber thermal_detection_sub_;
  ros::Subscriber final_landing_marker_sub_;
  ros::Subscriber rc_search_landing_sub_;
  ros::Subscriber rc_land_active_sub_;
  bool rc_land_active_{false};
  void rcLandActiveCallback(const std_msgs::BoolConstPtr& msg);
  ros::Publisher marker_pub_;
  ros::Publisher status_pub_;
  ros::Publisher exit_pose_pub_;
  ros::Publisher final_exit_pose_pub_;  // 出口接管时发布锁存出口，供后机接力。
  ros::Publisher landing_target_pub_;
  ros::Publisher landing_request_pub_;

  std::shared_ptr<SDFMap> sdf_map_;

  bool enabled_{true};
  // Hybrid route history must not alter native FUEL exploration behavior.
  bool hybrid_constraints_enabled_{false};
  // 2026-07-16: 颜色、普通二维码、温度识别接口始终保留；无摄像头时可关闭其任务完成门槛。
  bool require_stage2_detections_{true};
  bool corridor_frame_received_{false};
  std::string world_frame_{"world"};
  Eigen::Vector3d corridor_origin_{0.0, 0.0, 0.0};
  Eigen::Vector3d corridor_dir_{1.0, 0.0, 0.0};
  Eigen::Vector2d stable_progress_direction_{1.0, 0.0};
  bool stable_progress_direction_valid_{false};
  bool curve_heading_enabled_{false};
  bool curve_heading_active_{false};
  std::uint64_t curve_map_sequence_{0};
  bool curve_sample_valid_{false};
  Eigen::Vector3d curve_origin_{0.0, 0.0, 0.0};
  Eigen::Vector2d curve_forward_{1.0, 0.0};
  corridor_curve::Result curve_geometry_;
  task_search::TurnYawFollowLatch turn_yaw_follow_latch_;
  Eigen::Vector2d pending_turn_direction_{1.0, 0.0};
  Eigen::Vector3d pending_turn_probe_origin_{0.0, 0.0, 0.0};
  bool pending_turn_probe_origin_valid_{false};
  ros::Time pending_turn_last_evidence_;
  int pending_turn_confirmations_{0};
  std::uint64_t pending_turn_pose_sequence_{0};
  Eigen::Vector2d latest_turn_anchor_{0.0, 0.0};
  Eigen::Vector2d latest_turn_incoming_direction_{1.0, 0.0};
  bool latest_turn_anchor_valid_{false};
  struct CompletedGate {
    std::uint32_t sequence{0};
    Eigen::Vector2d center{0.0, 0.0};
    Eigen::Vector2d normal{1.0, 0.0};
    double left_extent{1.45};
    double right_extent{1.45};
    double thickness{0.30};
  };
  std::vector<CompletedGate> completed_gates_;
  std::uint32_t next_gate_sequence_{1};
  bool exploration_initial_yaw_frozen_{false};
  double exploration_initial_yaw_{0.0};
  double entry_inside_yaw_{0.0};
  Eigen::Vector2d segment_origin_{0.0, 0.0};
  Eigen::Vector2d segment_direction_{1.0, 0.0};
  double segment_high_water_{0.0};
  bool transition_active_{false};
  Eigen::Vector2d transition_anchor_{0.0, 0.0};
  Eigen::Vector2d transition_incoming_direction_{1.0, 0.0};
  Eigen::Vector2d transition_outgoing_direction_{1.0, 0.0};
  double transition_old_high_water_{0.0};
  std::deque<Eigen::Vector3d> visited_positions_;
  std::deque<Eigen::Vector3d> selected_goals_;
  mutable std::mutex body_cloud_mutex_;
  Eigen::Vector3d latest_robot_pos_{0.0, 0.0, 0.0};
  double latest_robot_yaw_{0.0};
  bool latest_robot_pose_valid_{false};
  std::uint64_t latest_robot_pose_sequence_{0};
  // 2026-07-23: 任务区域只允许“起点外 -> 入口 -> 通道内 -> 最终出口 -> 终点外”
  // 一次性状态推进；入口穿越一旦锁存，任何局部墙端/拐角都不能把状态重新解释成门外。
  bool corridor_entry_crossed_{false};
  TargetRecord targets_[3];
  TargetRecord final_landing_marker_;
  MissionStage mission_stage_{SEARCH_CORRIDOR};

  // CH9 仅由 UAV0 读取。必须先见低位再持续高位，避免上电时拨杆已在高位而误触发。
  bool rc_search_landing_enabled_{false};
  std::string rc_search_landing_topic_{"/UAV0/mavros/rc/in"};
  int rc_search_landing_channel_{8};
  int rc_search_landing_low_pwm_{1300};
  int rc_search_landing_high_pwm_{1800};
  double rc_search_landing_hold_sec_{0.5};
  bool rc_search_landing_armed_{false};
  bool rc_search_landing_triggered_{false};
  bool rc_search_landing_start_requested_{false};
  ros::Time rc_search_landing_high_since_;

  // 2026-07-13: 出口候选需要跨多次地图更新稳定，不能由单帧噪声直接触发第三阶段。
  bool exit_candidate_confirmed_{false};
  // 与检测确认分离；任务允许穿越时才永久锁定出口中心和方向。
  bool exit_portal_locked_{false};
  Eigen::Vector3d exit_candidate_{0.0, 0.0, 0.75};
  Eigen::Vector3d pending_exit_candidate_{0.0, 0.0, 0.75};
  // 2026-07-21: 区分门内接近点、门框中心和穿出方向；拓扑远端不再与降落点共用一个变量。
  Eigen::Vector3d exit_portal_center_{0.0, 0.0, 0.75};
  Eigen::Vector3d pending_exit_portal_center_{0.0, 0.0, 0.75};
  Eigen::Vector2d exit_outward_direction_{1.0, 0.0};
  Eigen::Vector2d pending_exit_outward_direction_{1.0, 0.0};
  bool exit_portal_outside_verified_{false};
  bool pending_exit_portal_outside_verified_{false};
  Eigen::Vector3d outside_search_anchor_{0.0, 0.0, 0.75};
  int exit_candidate_hits_{0};
  // 2026-07-24: 单帧门证据短暂丢失后仍保持前视一小段时间，避免立刻重启环扫形成1->0循环。
  ros::Time exit_verification_hold_until_;
  double exit_verification_yaw_hold_time_{2.5};
  int final_landing_marker_hits_{0};
  bool landing_requested_{false};
  ros::Time last_exit_inference_;
  std::uint64_t last_exit_map_sequence_{0};
  ros::Time mission_stage_start_;
  ros::Time search_exhausted_since_;
  ros::Time active_goal_stamp_;
  Eigen::Vector3d active_goal_{0.0, 0.0, 0.0};
  bool active_goal_valid_{false};
  std::deque<std::pair<Eigen::Vector3d, ros::Time>> failed_goals_;

  double visit_spacing_{0.35};
  double revisit_radius_{0.65};
  double repeat_goal_radius_{0.45};
  int max_goal_repeats_{2};
  int max_history_size_{120};
  double cruise_height_{0.75};
  double min_search_height_{0.55};
  double max_search_height_{1.00};
  // 2026-07-13: 限制单个搜索目标的升降量，避免 frontier 高度把无人机持续拉向天花板。
  double max_goal_climb_{0.06};
  double max_goal_descent_{0.10};
  double novelty_weight_{2.5};
  double travel_weight_{0.65};
  double preferred_distance_min_{1.0};
  double preferred_distance_max_{2.0};
  double near_distance_weight_{1.5};
  double far_distance_weight_{1.2};
  double direction_sector_rad_{30.0 * M_PI / 180.0};
  double yaw_weight_{0.20};
  double height_weight_{2.0};
  double repeat_penalty_{6.0};
  double frontier_gain_weight_{0.10};
  bool clearance_reward_enabled_{true};
  double clearance_reward_start_{0.30};
  double clearance_reward_full_{0.60};
  double clearance_reward_max_{5.0};
  double entry_forward_distance_{2.0};
  double entry_forward_weight_{1.5};
  double forward_viewpoint_bonus_{5.0};
  double backward_viewpoint_penalty_{5.0};
  // 2026-07-21: 普通单通道搜索默认禁止把“暂无前向候选”解释成掉头；显式故障回撤才允许后向恢复。
  bool prefer_motion_forward_{true};
  bool allow_search_backtrack_{false};
  bool global_no_return_{true};
  double backward_cos_threshold_{-0.15};
  double min_goal_hold_time_{1.2};
  // 2026-07-14: 对尚未到达的活动目标增加空间连续性代价，避免相邻重规划周期跨数米反向换点。
  double goal_switch_weight_{1.2};
  double failed_goal_radius_{0.55};
  double failed_goal_cooldown_{2.0};
  int failed_direction_threshold_{2};
  double failed_direction_pause_{3.0};
  Eigen::Vector2d recent_failed_direction_{0.0, 0.0};
  ros::Time recent_failed_direction_stamp_;
  int recent_failed_direction_count_{0};
  Eigen::Vector2d paused_failed_direction_{0.0, 0.0};
  ros::Time failed_direction_pause_until_;
  double inside_return_margin_{0.10};
  double entry_path_direction_grace_distance_{1.50};
  // 单通道转弯：旧轴前方终止后，直接从占据图选择仍在延伸的自由分支。
  bool recovery_occupancy_turn_enabled_{true};
  double recovery_turn_probe_length_{1.50};
  double recovery_turn_probe_step_{0.10};
  double recovery_turn_min_angle_deg_{30.0};
  double recovery_turn_max_angle_deg_{120.0};
  int recovery_turn_confirmation_count_{1};
  double recovery_turn_confirmation_min_interval_{0.15};
  double recovery_turn_confirmation_angle_deg_{15.0};
  double recovery_turn_confirmation_accumulation_window_{8.0};
  double recovery_turn_no_return_margin_{0.20};
  double recovery_turn_yaw_release_angle_deg_{15.0};


  // 局部双侧开阔出口检测与自动任务接管分开；只检测时保持人工流程。
  bool exit_detection_enabled_{true};
  bool exit_auto_crossing_enabled_{true};
  double exit_inference_period_{0.50};
  double exit_candidate_stability_radius_{0.30};
  int exit_confirmation_count_{3};
  double exit_candidate_angle_tolerance_deg_{15.0};
  double exit_confirmation_max_distance_{1.00};
  double exit_search_max_forward_{2.0};
  double exit_min_open_width_{2.0};
  double exit_min_open_ratio_{0.80};
  int exit_sector_ray_count_{11};
  double exit_sector_half_angle_deg_{20.0};
  double exit_inside_length_{0.90};
  double exit_outside_length_{1.20};
  double exit_footprint_half_height_{0.15};
  double exit_standoff_{0.45};
  double exit_arrive_distance_{0.50};
  double exit_portal_arrive_distance_{0.10};
  double exit_candidate_max_behind_distance_{0.35};
  double exit_progress_reverse_reject_deg_{120.0};
  int exit_pending_observation_min_hits_{3};
  double exit_pending_observation_lateral_offset_{0.25};
  double exit_pending_observation_min_move_{0.22};
  double exit_cross_step_{0.35};
  double exit_cross_target_distance_{0.90};
  double exit_cross_confirm_distance_{0.45};
  double exit_footprint_extra_radius_{0.22};
  int exit_footprint_samples_{8};
  double scan_radius_{0.35};
  double scan_dwell_time_{1.2};
  // true时FUEL只发布出口切换阶段，门外搜索、平台接近和降落请求全部交给外部Diff链路。
  bool external_landing_planner_{false};
  bool require_final_landing_marker_{true};
  // 2026-07-20: 最终出口确认后仍允许出口附近绕障微调，远处普通frontier只能有限增加
  // 到出口的距离，防止重新发布整段回头长路径。
  double exit_frontier_local_adjust_radius_{1.20};
  double exit_frontier_max_distance_increase_{0.45};
  int final_landing_marker_confirmation_count_{3};
  double final_landing_marker_consistency_radius_{0.40};
  double landing_approach_height_{0.70};
  double landing_trigger_distance_{0.35};
  double landing_column_bottom_clearance_{0.20};
  double landing_column_step_{0.10};
};

}  // namespace fast_planner

#endif
