#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace {

std::string firstToken(const std::string& text) {
  const std::size_t end = text.find_first_of(" \t\r\n");
  return text.substr(0, end);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double shortestYawDifference(double from, double to) {
  return std::remainder(to - from, 2.0 * M_PI);
}

class LandingDiffSearchManager {
 public:
  LandingDiffSearchManager() : nh_(), private_nh_("~") {
    private_nh_.param("search_height", search_height_, 2.0);
    private_nh_.param("front_approach_distance", front_approach_distance_, 1.0);
    private_nh_.param("front_approach_arrive_xy", front_approach_arrive_xy_, 0.15);
    private_nh_.param("lane_spacing", lane_spacing_, 1.0);
    private_nh_.param("search_width", search_width_, 3.0);
    private_nh_.param("search_depth", search_depth_, 4.0);
    private_nh_.param("arrive_xy", arrive_xy_, 0.35);
    private_nh_.param("arrive_z", arrive_z_, 0.20);
    private_nh_.param("goal_reissue_sec", goal_reissue_sec_, 1.0);
    private_nh_.param("sweep_stall_timeout_sec", sweep_stall_timeout_sec_, 4.0);
    private_nh_.param("sweep_motion_reset_distance", sweep_motion_reset_distance_, 0.10);
    private_nh_.param("front_hint_dwell_sec", front_hint_dwell_sec_, 2.0);
    private_nh_.param("front_hint_initial_wait_sec",
                      front_hint_initial_wait_sec_, 1.0);
    private_nh_.param("front_yaw_scan_half_angle_rad",
                      front_yaw_scan_half_angle_rad_, 0.78539816339);
    private_nh_.param("front_yaw_scan_rate_rad_s",
                      front_yaw_scan_rate_rad_s_, 0.35);
    private_nh_.param("front_yaw_arrive_tolerance_rad",
                      front_yaw_arrive_tolerance_rad_, 0.12);
    private_nh_.param("front_yaw_return_dwell_sec",
                      front_yaw_return_dwell_sec_, 0.50);
    private_nh_.param("front_hint_forward_margin", front_hint_forward_margin_, 1.0);
    private_nh_.param("front_hint_lateral_margin", front_hint_lateral_margin_, 1.0);
    private_nh_.param("front_hint_min_z", front_hint_min_z_, -0.50);
    private_nh_.param("front_hint_max_z", front_hint_max_z_, 1.50);
    private_nh_.param("marker_timeout_sec", marker_timeout_sec_, 0.50);
    private_nh_.param("front_hint_timeout_sec", front_hint_timeout_sec_, 0.50);
    private_nh_.param("assigned_target_timeout_sec",
                      assigned_target_timeout_sec_, 0.50);
    private_nh_.param("enable_single_front_hint_approach",
                      enable_single_front_hint_approach_, true);
    private_nh_.param("max_header_future_sec", max_header_future_sec_, 0.05);

    std::string odom_topic("/UAV0/fast_lio/Odometry");
    std::string stage_topic("/UAV0/mission/task_status");
    std::string rc_land_topic("/UAV0/mission/rc_land_active");
    std::string marker_topic("/UAV0/mission/detection/final_aruco");
    std::string front_hint_topic("/UAV0/landing/front_aruco_hint");
    std::string assigned_target_topic("/UAV0/landing/assigned_target");
    std::string subgoal_topic("/UAV0/landing_diff/subgoal");
    std::string trigger_topic("/UAV0/landing_diff/trigger");
    std::string yaw_topic("/UAV0/landing_diff/yaw");
    std::string front_scan_anchor_topic("/UAV0/landing/front_scan_anchor");
    std::string request_topic("/UAV0/mission/landing_request");
    std::string target_frame("world");
    private_nh_.param("odom_topic", odom_topic, odom_topic);
    private_nh_.param("stage_topic", stage_topic, stage_topic);
    private_nh_.param("rc_land_active_topic", rc_land_topic, rc_land_topic);
    private_nh_.param("marker_topic", marker_topic, marker_topic);
    private_nh_.param("front_hint_topic", front_hint_topic, front_hint_topic);
    private_nh_.param("assigned_target_topic", assigned_target_topic,
                      assigned_target_topic);
    private_nh_.param("subgoal_topic", subgoal_topic, subgoal_topic);
    private_nh_.param("trigger_topic", trigger_topic, trigger_topic);
    private_nh_.param("yaw_topic", yaw_topic, yaw_topic);
    private_nh_.param("front_scan_anchor_topic", front_scan_anchor_topic,
                      front_scan_anchor_topic);
    private_nh_.param("landing_request_topic", request_topic, request_topic);
    private_nh_.param("target_frame", target_frame, target_frame);
    target_frame_ = target_frame;

    if (target_frame_.empty() || marker_timeout_sec_ <= 0.0 ||
        front_hint_timeout_sec_ <= 0.0 || assigned_target_timeout_sec_ <= 0.0 ||
        front_hint_initial_wait_sec_ < 0.0 || front_approach_distance_ < 0.0 ||
        front_approach_arrive_xy_ <= 0.0 ||
        front_yaw_scan_half_angle_rad_ < 0.0 ||
        front_yaw_scan_rate_rad_s_ <= 0.0 ||
        front_yaw_arrive_tolerance_rad_ <= 0.0 ||
        front_yaw_arrive_tolerance_rad_ > M_PI ||
        front_yaw_return_dwell_sec_ < 0.0 ||
        !std::isfinite(sweep_stall_timeout_sec_) || sweep_stall_timeout_sec_ <= 0.0 ||
        !std::isfinite(sweep_motion_reset_distance_) || sweep_motion_reset_distance_ <= 0.0 ||
        max_header_future_sec_ < 0.0) {
      throw std::runtime_error("landing_diff_search: invalid target freshness parameters");
    }

    odom_sub_ = nh_.subscribe(odom_topic, 20, &LandingDiffSearchManager::odomCallback, this);
    stage_sub_ = nh_.subscribe(stage_topic, 10, &LandingDiffSearchManager::stageCallback, this);
    rc_land_sub_ = nh_.subscribe(rc_land_topic, 1,
                                 &LandingDiffSearchManager::rcLandCallback, this);
    marker_sub_ = nh_.subscribe(marker_topic, 10, &LandingDiffSearchManager::markerCallback, this);
    front_hint_sub_ = nh_.subscribe(front_hint_topic, 10,
                                    &LandingDiffSearchManager::frontHintCallback, this);
    assigned_target_sub_ = nh_.subscribe(
        assigned_target_topic, 10,
        &LandingDiffSearchManager::assignedTargetCallback, this);
    subgoal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(subgoal_topic, 2);
    trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(trigger_topic, 2, true);
    yaw_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(yaw_topic, 2);
    front_scan_anchor_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(front_scan_anchor_topic, 1, true);
    landing_request_pub_ = nh_.advertise<std_msgs::Bool>(request_topic, 2, true);
    state_pub_ = private_nh_.advertise<std_msgs::String>("state", 2, true);
    timer_ = nh_.createTimer(ros::Duration(0.10), &LandingDiffSearchManager::timerCallback, this);
    publishRequest(false);
    publishState("WAIT_EXIT_SWITCH");
  }

 private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_ = *msg;
    have_odom_ = true;
    if (switch_pending_ && !active_) activate();
  }

  void rcLandCallback(const std_msgs::BoolConstPtr& msg) {
    if (!msg->data || rc_land_active_) return;
    rc_land_active_ = true;
    active_ = false;
    switch_pending_ = false;
    landing_requested_ = false;
    publishRequest(false);
    publishState("RC_LAND_ABORTED");
    ROS_ERROR("CH10 landing latched: stopped Diff landing search goals/yaw/triggers");
  }

  void stageCallback(const std_msgs::StringConstPtr& msg) {
    if (rc_land_active_) return;
    const std::string stage = firstToken(msg->data);
    if (!switched_ && (stage == "SEARCH_OUTSIDE_LANDING" ||
                       stage == "SEARCH_OUTSIDE_QR")) {
      switch_pending_ = true;
      if (have_odom_) activate();
    }
    if (switched_ && stage == "LANDING") {
      active_ = false;
      // 通知 AutoTrans 搜索航向锁定已结束；后续由精降控制链接管。
      publishState("LANDING_HANDOFF");
    }
  }

  void markerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (rc_land_active_) return;
    if (!switched_ || !front_approach_complete_) return;
    if (!validTarget(*msg, marker_timeout_sec_, "final ArUco")) return;
    marker_ = *msg;
    have_marker_ = true;
    if (front_yaw_scan_complete_) {
      publishState("ARUCO_LOCKED_DIFF_APPROACH");
    } else {
      ROS_INFO_THROTTLE(
          1.0,
          "landing_diff_search: final ArUco stored; wait for full yaw scan and return");
    }
  }

  void frontHintCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (rc_land_active_) return;
    if (!enable_single_front_hint_approach_ || !switched_ ||
        !front_approach_complete_ ||
        front_hint_exhausted_ || have_marker_ || have_assigned_target_) return;
    if (!validTarget(*msg, front_hint_timeout_sec_, "front ArUco hint")) return;
    const geometry_msgs::Point& point = msg->pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || point.z < front_hint_min_z_ ||
        point.z > front_hint_max_z_) {
      ROS_WARN_THROTTLE(1.0, "landing_diff_search: reject invalid front ArUco hint");
      return;
    }
    const double dx = point.x - anchor_x_;
    const double dy = point.y - anchor_y_;
    const double forward = dx * forward_x_ + dy * forward_y_;
    const double lateral = dx * lateral_x_ + dy * lateral_y_;
    if (forward < -front_hint_forward_margin_ ||
        forward > search_depth_ + front_hint_forward_margin_ ||
        std::fabs(lateral) > 0.5 * search_width_ + front_hint_lateral_margin_) {
      ROS_WARN_THROTTLE(
          1.0,
          "landing_diff_search: reject front hint outside search fence forward=%.2f lateral=%.2f",
          forward, lateral);
      return;
    }
    if (!have_front_hint_ ||
        std::hypot(point.x - front_hint_.pose.position.x,
                   point.y - front_hint_.pose.position.y) > 0.20) {
      front_hint_reached_since_ = ros::Time(0);
    }
    front_hint_ = *msg;
    have_front_hint_ = true;
    if (front_yaw_scan_complete_) {
      publishState("FRONT_ARUCO_HINT_DIFF_APPROACH");
    } else {
      ROS_INFO_THROTTLE(
          1.0,
          "landing_diff_search: front hint stored; wait for full yaw scan and return");
    }
  }

  void assignedTargetCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (rc_land_active_) return;
    if (!switched_ || !front_approach_complete_ || landing_requested_) return;
    if (!validTarget(*msg, assigned_target_timeout_sec_,
                     "assigned landing target")) return;
    // 协调器会周期性重发同一个分配目标以保持时间戳新鲜。目标保活只能更新
    // 目标数据，不能反复重启回正流程，否则每条消息都会清零稳定驻留计时。
    const bool start_yaw_return =
        !front_yaw_scan_complete_ && !front_hint_returning_;
    assigned_target_ = *msg;
    have_assigned_target_ = true;
    have_front_hint_ = false;
    // 双码分配可以立即中断余下扫描，但只能先原地回到 CH9 锁定航向。
    // 回正且稳定前不发布远平台状态，避免 Diff 提前生成平移轨迹。
    if (front_yaw_scan_complete_) {
      publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM");
    } else if (start_yaw_return) {
      front_hint_returning_ = true;
      front_yaw_return_stable_since_ = ros::Time(0);
      publishState("FRONT_ARUCO_YAW_SCAN_RETURN");
      ROS_INFO("landing_diff_search: two-ArUco assignment stored; "
               "interrupt scan and return to CH9 yaw before approach");
    } else {
      ROS_DEBUG_THROTTLE(
          1.0,
          "landing_diff_search: assigned target keepalive updated without "
          "resetting CH9 yaw return dwell");
    }
  }

  void activate() {
    switched_ = true;
    active_ = true;
    switch_pending_ = false;
    anchor_z_ = odom_.pose.pose.position.z;
    const double yaw = yawFromQuaternion(odom_.pose.pose.orientation);
    locked_yaw_ = yaw;
    forward_x_ = std::cos(yaw);
    forward_y_ = std::sin(yaw);
    lateral_x_ = -forward_y_;
    lateral_y_ = forward_x_;
    anchor_x_ = odom_.pose.pose.position.x + front_approach_distance_ * forward_x_;
    anchor_y_ = odom_.pose.pose.position.y + front_approach_distance_ * forward_y_;
    front_approach_goal_.x = anchor_x_;
    front_approach_goal_.y = anchor_y_;
    front_approach_goal_.z = anchor_z_;
    front_yaw_scan_yaw_ = locked_yaw_;
    buildSweepGoals();
    publishTrigger();
    if (front_approach_distance_ <= front_approach_arrive_xy_) {
      beginFrontScan();
    } else {
      publishState("FRONT_ARUCO_FORWARD_APPROACH");
    }
    ROS_ERROR("landing_diff_search: FUEL -> Diff switch latched at "
              "(%.2f, %.2f, %.2f), keep yaw=%.3f and move %.2fm forward to "
              "front-scan anchor (%.2f, %.2f, %.2f)",
              odom_.pose.pose.position.x, odom_.pose.pose.position.y, anchor_z_,
              locked_yaw_, front_approach_distance_, anchor_x_, anchor_y_, anchor_z_);
  }

  void beginFrontScan() {
    front_approach_complete_ = true;
    activated_at_ = ros::Time::now();
    initial_front_wait_complete_ = front_hint_initial_wait_sec_ <= 0.0;
    front_yaw_scan_yaw_ = locked_yaw_;
    front_yaw_scan_phase_ = 0;
    front_yaw_scan_complete_ = front_yaw_scan_half_angle_rad_ <= 0.0;
    front_yaw_return_stable_since_ = ros::Time(0);
    last_front_yaw_scan_update_ = activated_at_;
    last_goal_time_ = ros::Time(0);
    publishFrontScanAnchor();
    publishState("FRONT_ARUCO_INITIAL_WAIT");
    ROS_ERROR("landing_diff_search: reached front-scan anchor "
              "(%.2f, %.2f, %.2f); start front ArUco scan at locked yaw %.3f",
              anchor_x_, anchor_y_, anchor_z_, locked_yaw_);
  }

  void buildSweepGoals() {
    sweep_goals_.clear();
    geometry_msgs::Point climb;
    climb.x = anchor_x_;
    climb.y = anchor_y_;
    climb.z = search_height_;
    sweep_goals_.push_back(climb);
    const int rows = std::max(1, static_cast<int>(std::ceil(search_depth_ / lane_spacing_)));
    const double half_width = 0.5 * search_width_;
    for (int row = 0; row <= rows; ++row) {
      const double forward = std::min(search_depth_, row * lane_spacing_);
      const double first_lateral = (row % 2 == 0) ? -half_width : half_width;
      const double second_lateral = -first_lateral;
      for (double lateral : {first_lateral, second_lateral}) {
        geometry_msgs::Point point;
        point.x = anchor_x_ + forward * forward_x_ + lateral * lateral_x_;
        point.y = anchor_y_ + forward * forward_y_ + lateral * lateral_y_;
        point.z = search_height_;
        sweep_goals_.push_back(point);
      }
    }
    sweep_index_ = 0;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!active_ || !have_odom_) return;
    // The precision-landing handoff is pending; do not resume sweep goals.
    if (landing_requested_) {
      publishSearchYaw();
      publishTrigger();
      return;
    }
    if (!front_approach_complete_) {
      publishSearchYaw();
      publishTrigger();
      if (frontApproachArrived()) {
        beginFrontScan();
      } else {
        publishGoalIfDue(front_approach_goal_);
      }
      return;
    }
    expireTargets();
    if (holdingForInitialFrontSearch()) {
      publishSearchYaw();
      publishTrigger();
      // 控制器在没有Diff轨迹时会自行保持当前位置。这里不能把当前位置
      // 作为子目标交给MINCO，否则零长度轨迹会产生零时长并触发断言。
      return;
    }
    publishSearchYaw();
    publishTrigger();
    geometry_msgs::Point goal;
    if (have_marker_) {
      sweep_tracking_ = false;
      goal = marker_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        if (!landing_requested_) {
          publishRequest(true);
          publishState("DIFF_APPROACH_REACHED_LANDING_REQUESTED");
        }
        return;
      }
    } else if (have_assigned_target_) {
      sweep_tracking_ = false;
      goal = assigned_target_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        publishState("ASSIGNED_TARGET_REACHED_WAIT_DOWN_CONFIRMATION");
      }
    } else if (have_front_hint_) {
      sweep_tracking_ = false;
      goal = front_hint_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        if (front_hint_reached_since_.isZero()) {
          front_hint_reached_since_ = ros::Time::now();
          publishState("FRONT_HINT_REACHED_WAIT_DOWN_CAMERA");
        } else if ((ros::Time::now() - front_hint_reached_since_).toSec() >=
                   front_hint_dwell_sec_) {
          have_front_hint_ = false;
          front_hint_exhausted_ = true;
          front_hint_reached_since_ = ros::Time(0);
          selectNearestSweepGoal();
          publishState("FRONT_HINT_TIMEOUT_FALLBACK_DOWN_SWEEP");
        }
      } else {
        front_hint_reached_since_ = ros::Time(0);
      }
      if (!have_front_hint_) {
        if (!selectSweepGoal(goal)) return;
      }
    } else {
      if (!selectSweepGoal(goal)) return;
    }
    publishGoalIfDue(goal);
  }

  bool holdingForInitialFrontSearch() {
    if (front_hint_returning_) {
      return returnFrontYawToExitHeading();
    }
    if (!initial_front_wait_complete_) {
      if ((ros::Time::now() - activated_at_).toSec() <
          front_hint_initial_wait_sec_) {
        return true;
      }
      initial_front_wait_complete_ = true;
      if (!front_yaw_scan_complete_) {
        publishState("FRONT_ARUCO_YAW_SCAN_LEFT");
      }
    }
    return !front_yaw_scan_complete_ && advanceFrontYawScan();
  }

  bool advanceFrontYawScan() {
    if (front_yaw_scan_complete_) {
      return false;
    }
    const ros::Time now = ros::Time::now();
    const double elapsed = std::max(
        0.0, (now - last_front_yaw_scan_update_).toSec());
    last_front_yaw_scan_update_ = now;

    double target_yaw = locked_yaw_;
    if (front_yaw_scan_phase_ == 0) {
      // ENU 中 yaw 正方向为逆时针（从上往下看向左转）。
      target_yaw = std::remainder(
          locked_yaw_ + front_yaw_scan_half_angle_rad_, 2.0 * M_PI);
    } else if (front_yaw_scan_phase_ == 1) {
      target_yaw = std::remainder(
          locked_yaw_ - front_yaw_scan_half_angle_rad_, 2.0 * M_PI);
    }
    const double remaining =
        shortestYawDifference(front_yaw_scan_yaw_, target_yaw);
    const double step = front_yaw_scan_rate_rad_s_ * elapsed;
    if (std::fabs(remaining) <= step) {
      front_yaw_scan_yaw_ = target_yaw;
      // 不能只按指令时间假定偏航已经完成。AutoTrans 必须把机体实际航向
      // 转到当前端点附近后，状态机才进入下一段或开始下视蛇形搜索。
      const double actual_yaw = yawFromQuaternion(odom_.pose.pose.orientation);
      const double actual_error =
          std::fabs(shortestYawDifference(actual_yaw, target_yaw));
      if (!std::isfinite(actual_yaw) || !std::isfinite(actual_error) ||
          actual_error > front_yaw_arrive_tolerance_rad_) {
        front_yaw_return_stable_since_ = ros::Time(0);
        ROS_WARN_THROTTLE(
            1.0,
            "landing_diff_search: waiting actual yaw at scan endpoint "
            "target=%.3f actual=%.3f error=%.3f tolerance=%.3f",
            target_yaw, actual_yaw, actual_error,
            front_yaw_arrive_tolerance_rad_);
        return true;
      }
      // 最后一段回到 CH9 航向后持续稳定一段时间，才能释放平移目标。
      if (front_yaw_scan_phase_ == 2 && front_yaw_return_dwell_sec_ > 0.0) {
        if (front_yaw_return_stable_since_.isZero()) {
          front_yaw_return_stable_since_ = now;
          return true;
        }
        if ((now - front_yaw_return_stable_since_).toSec() <
            front_yaw_return_dwell_sec_) {
          return true;
        }
      }
      front_yaw_return_stable_since_ = ros::Time(0);
      ++front_yaw_scan_phase_;
      if (front_yaw_scan_phase_ >= 3) {
        front_yaw_scan_complete_ = true;
        if (have_marker_) {
          publishState("ARUCO_LOCKED_DIFF_APPROACH");
        } else if (have_assigned_target_) {
          publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM");
        } else if (have_front_hint_) {
          publishState("FRONT_ARUCO_HINT_DIFF_APPROACH");
        } else {
          publishState("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP");
        }
        return false;
      }
      if (front_yaw_scan_phase_ == 1) {
        publishState("FRONT_ARUCO_YAW_SCAN_RIGHT");
      } else {
        publishState("FRONT_ARUCO_YAW_SCAN_RETURN");
      }
    } else {
      front_yaw_return_stable_since_ = ros::Time(0);
      front_yaw_scan_yaw_ = std::remainder(
          front_yaw_scan_yaw_ + std::copysign(step, remaining),
          2.0 * M_PI);
    }
    return true;
  }

  bool returnFrontYawToExitHeading() {
    const ros::Time now = ros::Time::now();
    const double elapsed = std::max(
        0.0, (now - last_front_yaw_scan_update_).toSec());
    last_front_yaw_scan_update_ = now;
    const double remaining =
        shortestYawDifference(front_yaw_scan_yaw_, locked_yaw_);
    const double step = front_yaw_scan_rate_rad_s_ * elapsed;
    if (std::fabs(remaining) <= step) {
      front_yaw_scan_yaw_ = locked_yaw_;
      const double actual_yaw = yawFromQuaternion(odom_.pose.pose.orientation);
      const double actual_error =
          std::fabs(shortestYawDifference(actual_yaw, locked_yaw_));
      if (!std::isfinite(actual_yaw) || !std::isfinite(actual_error) ||
          actual_error > front_yaw_arrive_tolerance_rad_) {
        front_yaw_return_stable_since_ = ros::Time(0);
        ROS_WARN_THROTTLE(
            1.0,
            "landing_diff_search: assignment pending; waiting return yaw "
            "target=%.3f actual=%.3f error=%.3f tolerance=%.3f",
            locked_yaw_, actual_yaw, actual_error,
            front_yaw_arrive_tolerance_rad_);
        return true;
      }
      if (front_yaw_return_dwell_sec_ > 0.0) {
        if (front_yaw_return_stable_since_.isZero()) {
          front_yaw_return_stable_since_ = now;
          return true;
        }
        if ((now - front_yaw_return_stable_since_).toSec() <
            front_yaw_return_dwell_sec_) {
          return true;
        }
      }
      front_yaw_return_stable_since_ = ros::Time(0);
      front_hint_returning_ = false;
      front_yaw_scan_complete_ = true;
      if (have_marker_) {
        publishState("ARUCO_LOCKED_DIFF_APPROACH");
      } else if (have_assigned_target_) {
        publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM");
      } else if (have_front_hint_) {
        publishState("FRONT_ARUCO_HINT_RETURN_COMPLETE_APPROACH");
      } else {
        publishState("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP");
      }
      return false;
    }
    front_yaw_return_stable_since_ = ros::Time(0);
    front_yaw_scan_yaw_ = std::remainder(
        front_yaw_scan_yaw_ + std::copysign(step, remaining),
        2.0 * M_PI);
    return true;
  }

  void selectNearestSweepGoal() {
    if (sweep_goals_.empty() || !have_odom_) return;
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0U;
    for (std::size_t index = 0U; index < sweep_goals_.size(); ++index) {
      const geometry_msgs::Point& candidate = sweep_goals_[index];
      const double distance = std::hypot(candidate.x - current.x,
                                         candidate.y - current.y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }
    sweep_index_ = best_index;
    sweep_tracking_ = false;
  }

  bool selectSweepGoal(geometry_msgs::Point& goal) {
    if (sweep_goals_.empty()) return false;
    for (std::size_t skipped = 0; arrived(sweep_goals_[sweep_index_]); ++skipped) {
      if (skipped >= sweep_goals_.size()) return false;
      advanceSweepGoal();
    }
    goal = sweep_goals_[sweep_index_];
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    // 只处理下视水平扫描；上升途中及已锁定平台的接近不能按横向尽头跳点。
    if (sweep_index_ == 0 || std::fabs(current.z - search_height_) > arrive_z_) {
      sweep_tracking_ = false;
      return true;
    }
    const ros::Time now = ros::Time::now();
    if (!sweep_tracking_ ||
        std::hypot(current.x - sweep_motion_checkpoint_.x,
                   current.y - sweep_motion_checkpoint_.y) >= sweep_motion_reset_distance_) {
      sweep_motion_checkpoint_ = current;
      sweep_last_motion_time_ = now;
      sweep_tracking_ = true;
    } else if ((now - sweep_last_motion_time_).toSec() >= sweep_stall_timeout_sec_) {
      // 固定端点在墙后时，Diff 会停在可达区域。用实际停点缩短这一侧的
      // 后续航点，保留原蛇形顺序：反扫本行或沿边前进一行后反扫。
      const double remaining_forward =
          (goal.x - current.x) * forward_x_ + (goal.y - current.y) * forward_y_;
      const double remaining_lateral =
          (goal.x - current.x) * lateral_x_ + (goal.y - current.y) * lateral_y_;
      const bool lateral_edge = std::fabs(remaining_lateral) >= std::fabs(remaining_forward);
      const double axis_x = lateral_edge ? lateral_x_ : forward_x_;
      const double axis_y = lateral_edge ? lateral_y_ : forward_y_;
      const double remaining = lateral_edge ? remaining_lateral : remaining_forward;
      for (std::size_t i = 1; i < sweep_goals_.size(); ++i) {
        geometry_msgs::Point& candidate = sweep_goals_[i];
        const double beyond = (candidate.x - current.x) * axis_x +
                              (candidate.y - current.y) * axis_y;
        if (beyond * remaining > 0.0) {
          candidate.x -= beyond * axis_x;
          candidate.y -= beyond * axis_y;
        }
      }
      ROS_WARN("[DOWN_SWEEP_BOUNDARY] goal=%zu no horizontal motion for %.1fs; "
               "clip %s boundary at (%.2f, %.2f), advance serpentine search",
               sweep_index_, sweep_stall_timeout_sec_, lateral_edge ? "side" : "forward",
               current.x, current.y);
      advanceSweepGoal();
      return selectSweepGoal(goal);
    }
    return true;
  }

  void advanceSweepGoal() {
    sweep_index_ = (sweep_index_ + 1) % sweep_goals_.size();
    sweep_tracking_ = false;
    last_goal_time_ = ros::Time(0);
  }

  bool arrived(const geometry_msgs::Point& goal) const {
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    return std::hypot(current.x - goal.x, current.y - goal.y) <= arrive_xy_ &&
           std::fabs(current.z - goal.z) <= arrive_z_;
  }

  bool frontApproachArrived() const {
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    return std::hypot(current.x - front_approach_goal_.x,
                      current.y - front_approach_goal_.y) <=
               front_approach_arrive_xy_ &&
           std::fabs(current.z - front_approach_goal_.z) <= arrive_z_;
  }

  bool validTarget(const geometry_msgs::PoseStamped& target,
                   double timeout_sec,
                   const char* label) const {
    const geometry_msgs::Point& point = target.pose.position;
    if (target.header.frame_id != target_frame_ || target.header.stamp.isZero()) {
      ROS_WARN_THROTTLE(1.0,
                        "landing_diff_search: reject %s with invalid frame or stamp",
                        label);
      return false;
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      ROS_WARN_THROTTLE(1.0,
                        "landing_diff_search: reject %s with non-finite position",
                        label);
      return false;
    }
    const double age_sec = (ros::Time::now() - target.header.stamp).toSec();
    if (age_sec < -max_header_future_sec_ || age_sec > timeout_sec) {
      ROS_WARN_THROTTLE(1.0,
                        "landing_diff_search: reject stale %s age=%.3fs",
                        label, age_sec);
      return false;
    }
    return true;
  }

  void expireTargets() {
    if (have_marker_ && !validTarget(marker_, marker_timeout_sec_, "final ArUco")) {
      have_marker_ = false;
      if (front_yaw_scan_complete_) {
        publishState("FINAL_ARUCO_TIMEOUT_FALLBACK_DOWN_SWEEP");
      }
    }
    // 前视相机提示在接收时已经过时间戳、世界系和搜索边界校验。
    // 平台在世界系中静止，飞近后离开 D435 视野是正常现象，因此粗提示
    // 必须锁存到到达或被下视 final ArUco 取代，不按消息年龄中途清除。
  }

  void publishGoal(const geometry_msgs::Point& point) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position = point;
    goal.pose.orientation.w = std::cos(0.5 * commandedYaw());
    goal.pose.orientation.z = std::sin(0.5 * commandedYaw());
    subgoal_pub_.publish(goal);
    last_goal_time_ = goal.header.stamp;
  }

  void publishGoalIfDue(const geometry_msgs::Point& point) {
    if (last_goal_time_.isZero() ||
        (ros::Time::now() - last_goal_time_).toSec() >= goal_reissue_sec_) {
      publishGoal(point);
    }
  }

  void publishTrigger() {
    if (!last_trigger_time_.isZero() &&
        (ros::Time::now() - last_trigger_time_).toSec() < 1.0) return;
    geometry_msgs::PoseStamped trigger;
    trigger.header.stamp = ros::Time::now();
    trigger.header.frame_id = "world";
    trigger.pose = odom_.pose.pose;
    trigger_pub_.publish(trigger);
    last_trigger_time_ = trigger.header.stamp;
  }

  void publishFrontScanAnchor() {
    geometry_msgs::PoseStamped anchor;
    anchor.header.stamp = ros::Time::now();
    anchor.header.frame_id = target_frame_;
    anchor.pose.position = front_approach_goal_;
    anchor.pose.orientation.w = std::cos(0.5 * locked_yaw_);
    anchor.pose.orientation.z = std::sin(0.5 * locked_yaw_);
    front_scan_anchor_pub_.publish(anchor);
  }

  void publishRequest(bool active) {
    std_msgs::Bool request;
    request.data = active;
    landing_request_pub_.publish(request);
    landing_requested_ = active;
  }

  double commandedYaw() const {
    if (front_hint_returning_) return front_yaw_scan_yaw_;
    // 未检测齐两个码时完成左扫、右扫和回正；双码分配后则走上面的回正路径。
    return front_yaw_scan_complete_ ? locked_yaw_ : front_yaw_scan_yaw_;
  }

  void publishSearchYaw() {
    quadrotor_msgs::PositionCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "world";
    command.yaw = commandedYaw();
    command.yaw_dot = 0.0;
    yaw_pub_.publish(command);
  }

  void publishState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odom_sub_, stage_sub_, marker_sub_, front_hint_sub_,
      assigned_target_sub_, rc_land_sub_;
  ros::Publisher subgoal_pub_, trigger_pub_, yaw_pub_, front_scan_anchor_pub_,
      landing_request_pub_, state_pub_;
  ros::Timer timer_;
  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped marker_;
  geometry_msgs::PoseStamped front_hint_;
  geometry_msgs::PoseStamped assigned_target_;
  geometry_msgs::Point front_approach_goal_;
  std::vector<geometry_msgs::Point> sweep_goals_;
  std::size_t sweep_index_{0};
  bool sweep_tracking_{false};
  geometry_msgs::Point sweep_motion_checkpoint_;
  ros::Time sweep_last_motion_time_;
  double sweep_stall_timeout_sec_{4.0}, sweep_motion_reset_distance_{0.10};
  bool have_odom_{false}, switch_pending_{false}, switched_{false}, active_{false};
  bool front_approach_complete_{false};
  bool rc_land_active_{false};
  bool have_marker_{false}, landing_requested_{false};
  bool have_front_hint_{false}, front_hint_exhausted_{false};
  bool have_assigned_target_{false};
  bool enable_single_front_hint_approach_{true};
  double search_height_{2.0}, lane_spacing_{1.0}, search_width_{3.0}, search_depth_{4.0};
  double front_approach_distance_{1.0}, front_approach_arrive_xy_{0.15};
  double arrive_xy_{0.35}, arrive_z_{0.20}, goal_reissue_sec_{1.0};
  double front_hint_dwell_sec_{2.0}, front_hint_forward_margin_{1.0};
  double front_hint_initial_wait_sec_{1.0};
  double front_yaw_scan_half_angle_rad_{0.78539816339};
  double front_yaw_scan_rate_rad_s_{0.35};
  double front_yaw_arrive_tolerance_rad_{0.12};
  double front_yaw_return_dwell_sec_{0.50};
  double front_hint_lateral_margin_{1.0}, front_hint_min_z_{-0.50};
  double front_hint_max_z_{1.50};
  double marker_timeout_sec_{0.50}, front_hint_timeout_sec_{0.50};
  double assigned_target_timeout_sec_{0.50};
  double max_header_future_sec_{0.05};
  double anchor_x_{0.0}, anchor_y_{0.0}, anchor_z_{0.0};
  double forward_x_{1.0}, forward_y_{0.0};
  double lateral_x_{0.0}, lateral_y_{1.0};
  double locked_yaw_{0.0};
  std::string target_frame_{"world"};
  bool initial_front_wait_complete_{false}, front_yaw_scan_complete_{false};
  ros::Time front_yaw_return_stable_since_{0};
  bool front_hint_returning_{false};
  int front_yaw_scan_phase_{0};
  double front_yaw_scan_yaw_{0.0};
  ros::Time activated_at_, last_goal_time_, last_trigger_time_,
      front_hint_reached_since_, last_front_yaw_scan_update_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_diff_search_manager");
  LandingDiffSearchManager manager;
  ros::spin();
  return 0;
}
