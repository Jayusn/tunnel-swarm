#include <cmath>
#include <sstream>
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

class LandingDiffSearchManager {
 public:
  LandingDiffSearchManager() : nh_(), private_nh_("~") {
    private_nh_.param("search_height", search_height_, 2.0);
    private_nh_.param("lane_spacing", lane_spacing_, 1.0);
    private_nh_.param("search_width", search_width_, 3.0);
    private_nh_.param("search_depth", search_depth_, 4.0);
    private_nh_.param("arrive_xy", arrive_xy_, 0.35);
    private_nh_.param("arrive_z", arrive_z_, 0.20);
    private_nh_.param("goal_reissue_sec", goal_reissue_sec_, 1.0);

    std::string odom_topic("/UAV0/fast_lio/Odometry");
    std::string stage_topic("/UAV0/mission/task_status");
    std::string marker_topic("/UAV0/mission/detection/final_aruco");
    std::string subgoal_topic("/UAV0/landing_diff/subgoal");
    std::string trigger_topic("/UAV0/landing_diff/trigger");
    std::string yaw_topic("/UAV0/landing_diff/yaw");
    std::string request_topic("/UAV0/mission/landing_request");
    private_nh_.param("odom_topic", odom_topic, odom_topic);
    private_nh_.param("stage_topic", stage_topic, stage_topic);
    private_nh_.param("marker_topic", marker_topic, marker_topic);
    private_nh_.param("subgoal_topic", subgoal_topic, subgoal_topic);
    private_nh_.param("trigger_topic", trigger_topic, trigger_topic);
    private_nh_.param("yaw_topic", yaw_topic, yaw_topic);
    private_nh_.param("landing_request_topic", request_topic, request_topic);

    odom_sub_ = nh_.subscribe(odom_topic, 20, &LandingDiffSearchManager::odomCallback, this);
    stage_sub_ = nh_.subscribe(stage_topic, 10, &LandingDiffSearchManager::stageCallback, this);
    marker_sub_ = nh_.subscribe(marker_topic, 10, &LandingDiffSearchManager::markerCallback, this);
    subgoal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(subgoal_topic, 2);
    trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(trigger_topic, 2, true);
    yaw_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(yaw_topic, 2);
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

  void stageCallback(const std_msgs::StringConstPtr& msg) {
    const std::string stage = firstToken(msg->data);
    if (!switched_ && (stage == "SEARCH_OUTSIDE_LANDING" ||
                       stage == "SEARCH_OUTSIDE_QR")) {
      switch_pending_ = true;
      if (have_odom_) activate();
    }
    if (switched_ && stage == "LANDING") active_ = false;
  }

  void markerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (!switched_) return;
    marker_ = *msg;
    have_marker_ = true;
    publishState("ARUCO_LOCKED_DIFF_APPROACH");
  }

  void activate() {
    switched_ = true;
    active_ = true;
    switch_pending_ = false;
    anchor_x_ = odom_.pose.pose.position.x;
    anchor_y_ = odom_.pose.pose.position.y;
    const double yaw = yawFromQuaternion(odom_.pose.pose.orientation);
    locked_yaw_ = yaw;
    forward_x_ = std::cos(yaw);
    forward_y_ = std::sin(yaw);
    lateral_x_ = -forward_y_;
    lateral_y_ = forward_x_;
    buildSweepGoals();
    publishTrigger();
    publishState("DIFF_SEARCH_ACTIVE");
    ROS_ERROR("landing_diff_search: FUEL -> Diff switch latched at %.2f %.2f", anchor_x_, anchor_y_);
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
    publishLockedYaw();
    publishTrigger();
    geometry_msgs::Point goal;
    if (have_marker_) {
      goal = marker_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        if (!landing_requested_) {
          publishRequest(true);
          publishState("DIFF_APPROACH_REACHED_LANDING_REQUESTED");
        }
        return;
      }
    } else {
      if (sweep_goals_.empty()) return;
      if (arrived(sweep_goals_[sweep_index_])) {
        sweep_index_ = (sweep_index_ + 1) % sweep_goals_.size();
      }
      goal = sweep_goals_[sweep_index_];
    }
    if (last_goal_time_.isZero() ||
        (ros::Time::now() - last_goal_time_).toSec() >= goal_reissue_sec_) {
      publishGoal(goal);
    }
  }

  bool arrived(const geometry_msgs::Point& goal) const {
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    return std::hypot(current.x - goal.x, current.y - goal.y) <= arrive_xy_ &&
           std::fabs(current.z - goal.z) <= arrive_z_;
  }

  void publishGoal(const geometry_msgs::Point& point) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position = point;
    goal.pose.orientation.w = std::cos(0.5 * locked_yaw_);
    goal.pose.orientation.z = std::sin(0.5 * locked_yaw_);
    subgoal_pub_.publish(goal);
    last_goal_time_ = goal.header.stamp;
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

  void publishRequest(bool active) {
    std_msgs::Bool request;
    request.data = active;
    landing_request_pub_.publish(request);
    landing_requested_ = active;
  }

  void publishLockedYaw() {
    quadrotor_msgs::PositionCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "world";
    command.yaw = locked_yaw_;
    command.yaw_dot = 0.0;
    yaw_pub_.publish(command);
  }

  void publishState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odom_sub_, stage_sub_, marker_sub_;
  ros::Publisher subgoal_pub_, trigger_pub_, yaw_pub_, landing_request_pub_, state_pub_;
  ros::Timer timer_;
  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped marker_;
  std::vector<geometry_msgs::Point> sweep_goals_;
  std::size_t sweep_index_{0};
  bool have_odom_{false}, switch_pending_{false}, switched_{false}, active_{false};
  bool have_marker_{false}, landing_requested_{false};
  double search_height_{2.0}, lane_spacing_{1.0}, search_width_{3.0}, search_depth_{4.0};
  double arrive_xy_{0.35}, arrive_z_{0.20}, goal_reissue_sec_{1.0};
  double anchor_x_{0.0}, anchor_y_{0.0}, forward_x_{1.0}, forward_y_{0.0};
  double lateral_x_{0.0}, lateral_y_{1.0};
  double locked_yaw_{0.0};
  ros::Time last_goal_time_, last_trigger_time_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_diff_search_manager");
  LandingDiffSearchManager manager;
  ros::spin();
  return 0;
}
