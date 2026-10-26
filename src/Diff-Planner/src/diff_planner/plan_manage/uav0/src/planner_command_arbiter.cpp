#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ldop/DynamicObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/ExplorationGoal.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace {
std::string firstToken(const std::string& text) {
  const std::size_t end = text.find_first_of(" \t\r\n");
  return text.substr(0, end);
}

class PlannerCommandArbiter {
 public:
  PlannerCommandArbiter() : nh_(), private_nh_("~") {
    std::string fuel_topic("/UAV0/fuel/planning/pos_cmd");
    std::string diff_topic("/UAV0/diff/planning/pos_cmd");
    std::string output_topic("/UAV0/planning/pos_cmd");
    std::string stage_topic("/UAV0/mission/task_status");
    std::string odom_topic("/UAV0/fast_lio/Odometry");
    std::string dynamic_objects_topic("/UAV0/ldop/dynamic_objects");
    std::string emergency_brake_topic("/UAV0/planning/emergency_brake");
    std::string exploration_goal_topic("/UAV0/fuel_diff/exploration_goal");
    private_nh_.param("fuel_topic", fuel_topic, fuel_topic);
    private_nh_.param("diff_topic", diff_topic, diff_topic);
    private_nh_.param("output_topic", output_topic, output_topic);
    private_nh_.param("stage_topic", stage_topic, stage_topic);
    private_nh_.param("odom_topic", odom_topic, odom_topic);
    private_nh_.param("dynamic_objects_topic", dynamic_objects_topic,
                      dynamic_objects_topic);
    private_nh_.param("emergency_brake_topic", emergency_brake_topic,
                      emergency_brake_topic);
    private_nh_.param("exploration_goal_topic", exploration_goal_topic,
                      exploration_goal_topic);
    private_nh_.param("dynamic_avoidance_enabled", dynamic_avoidance_enabled_, true);
    private_nh_.param("dynamic_object_timeout", dynamic_object_timeout_, 0.60);
    private_nh_.param("dynamic_safety_radius", dynamic_safety_radius_, 0.45);
    private_nh_.param("dynamic_vertical_margin", dynamic_vertical_margin_, 0.25);
    private_nh_.param("dynamic_prediction_horizon", dynamic_prediction_horizon_, 1.0);
    private_nh_.param("dynamic_prediction_step", dynamic_prediction_step_, 0.20);
    private_nh_.param("diff_command_timeout", diff_command_timeout_, 0.25);
    output_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(output_topic, 10);
    owner_pub_ = private_nh_.advertise<std_msgs::String>("owner", 2, true);
    dynamic_hold_pub_ = private_nh_.advertise<std_msgs::Bool>("dynamic_hold", 2, true);
    emergency_brake_pub_ = nh_.advertise<std_msgs::Int32>(emergency_brake_topic, 2);
    fuel_sub_ = nh_.subscribe(fuel_topic, 20, &PlannerCommandArbiter::fuelCallback, this);
    diff_sub_ = nh_.subscribe(diff_topic, 20, &PlannerCommandArbiter::diffCallback, this);
    stage_sub_ = nh_.subscribe(stage_topic, 10, &PlannerCommandArbiter::stageCallback, this);
    exploration_goal_sub_ = nh_.subscribe(
        exploration_goal_topic, 10, &PlannerCommandArbiter::explorationGoalCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 20, &PlannerCommandArbiter::odomCallback, this);
    dynamic_objects_sub_ = nh_.subscribe(dynamic_objects_topic, 10,
                                         &PlannerCommandArbiter::dynamicObjectsCallback, this);
    hold_timer_ = nh_.createTimer(ros::Duration(0.02),
                                  &PlannerCommandArbiter::holdTimerCallback, this);
    publishOwner();
    publishDynamicHold(false);
  }

 private:
  struct DynamicSample {
    uint32_t id{0U};
    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 size;
    bool have_velocity{false};
  };

  void stageCallback(const std_msgs::StringConstPtr& msg) {
    const std::string stage = firstToken(msg->data);
    if (!diff_owned_ && (stage == "SEARCH_OUTSIDE_LANDING" ||
                         stage == "SEARCH_OUTSIDE_QR" ||
                         stage == "APPROACH_LANDING" || stage == "LANDING")) {
      diff_owned_ = true;
      publishOwner();
      have_diff_command_ = false;
      publishStaticHold(0U);
      ROS_ERROR("planner_command_arbiter: control ownership latched FUEL -> DIFF");
    }
  }
  void explorationGoalCallback(const quadrotor_msgs::ExplorationGoalConstPtr& msg) {
    if (msg->target_type > quadrotor_msgs::ExplorationGoal::TARGET_TURN_ALIGNMENT) {
      return;
    }
    if (!diff_owned_) {
      diff_owned_ = true;
      publishOwner();
      have_diff_command_ = false;
      publishStaticHold(0U);
      ROS_WARN("planner_command_arbiter: DIFF owns control for FUEL viewpoint session=%llu id=%llu",
               static_cast<unsigned long long>(msg->session_id),
               static_cast<unsigned long long>(msg->goal_id));
    }
  }
  void fuelCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (diff_owned_) return;
    uint32_t blocked_id = 0U;
    if (fuelCommandBlocked(*msg, &blocked_id)) {
      quadrotor_msgs::PositionCommand hold = *msg;
      hold.header.stamp = ros::Time::now();
      hold.position = odom_.pose.pose.position;
      hold.velocity.x = hold.velocity.y = hold.velocity.z = 0.0;
      hold.acceleration.x = hold.acceleration.y = hold.acceleration.z = 0.0;
      hold.jerk.x = hold.jerk.y = hold.jerk.z = 0.0;
      output_pub_.publish(hold);
      if (!dynamic_hold_) {
        dynamic_hold_ = true;
        publishDynamicHold(true);
        std_msgs::Int32 brake;
        brake.data = static_cast<int32_t>(msg->trajectory_id);
        emergency_brake_pub_.publish(brake);
        ROS_ERROR("planner_command_arbiter: UAV0 dynamic hold, LDOP object=%u traj=%u",
                  blocked_id, msg->trajectory_id);
      }
      return;
    }
    if (dynamic_hold_) {
      dynamic_hold_ = false;
      publishDynamicHold(false);
      ROS_WARN("planner_command_arbiter: UAV0 dynamic obstacle cleared");
    }
    output_pub_.publish(*msg);
  }
  void diffCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (!diff_owned_) return;
    output_pub_.publish(*msg);
    have_diff_command_ = true;
    last_diff_command_time_ = ros::Time::now();
    last_diff_trajectory_id_ = msg->trajectory_id;
  }
  void publishOwner() {
    std_msgs::String msg;
    msg.data = diff_owned_ ? "DIFF" : "FUEL";
    owner_pub_.publish(msg);
  }

  void publishDynamicHold(bool active) {
    std_msgs::Bool msg;
    msg.data = active;
    dynamic_hold_pub_.publish(msg);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_ = *msg;
    have_odom_ = true;
  }

  void holdTimerCallback(const ros::TimerEvent&) {
    if (!diff_owned_ || !have_odom_) return;
    if (!have_diff_command_ ||
        (ros::Time::now() - last_diff_command_time_).toSec() > diff_command_timeout_)
      publishStaticHold(last_diff_trajectory_id_);
  }

  void publishStaticHold(uint32_t trajectory_id) {
    if (!have_odom_) return;
    quadrotor_msgs::PositionCommand hold;
    hold.header.stamp = ros::Time::now();
    hold.header.frame_id = odom_.header.frame_id;
    hold.position = odom_.pose.pose.position;
    hold.velocity.x = hold.velocity.y = hold.velocity.z = 0.0;
    hold.acceleration.x = hold.acceleration.y = hold.acceleration.z = 0.0;
    hold.jerk.x = hold.jerk.y = hold.jerk.z = 0.0;
    const auto& q = odom_.pose.pose.orientation;
    hold.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    hold.yaw_dot = 0.0;
    hold.trajectory_id = trajectory_id;
    hold.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    output_pub_.publish(hold);
  }

  void dynamicObjectsCallback(const ldop::DynamicObjectArrayConstPtr& msg) {
    std::vector<DynamicSample> samples;
    samples.reserve(msg->objects.size());
    for (const auto& object : msg->objects) {
      if (object.model_state.size() < 3U) continue;
      DynamicSample sample;
      sample.id = object.id;
      sample.position.x = object.model_state[0];
      sample.position.y = object.model_state[1];
      sample.position.z = object.model_state[2];
      sample.size = object.size;
      if (!finitePoint(sample.position)) continue;
      if (object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CA2D &&
          object.model_state.size() >= 5U) {
        sample.velocity.x = object.model_state[3];
        sample.velocity.y = object.model_state[4];
        sample.have_velocity = true;
      } else if ((object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CA3D ||
                  object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CV3D) &&
                 object.model_state.size() >= 6U) {
        sample.velocity.x = object.model_state[3];
        sample.velocity.y = object.model_state[4];
        sample.velocity.z = object.model_state[5];
        sample.have_velocity = true;
      } else if (object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CTRA &&
                 object.model_state.size() >= 6U) {
        const double speed = object.model_state[3];
        const double yaw = object.model_state[5];
        sample.velocity.x = speed * std::cos(yaw);
        sample.velocity.y = speed * std::sin(yaw);
        sample.have_velocity = true;
      }
      sample.have_velocity = sample.have_velocity &&
                             std::isfinite(sample.velocity.x) &&
                             std::isfinite(sample.velocity.y) &&
                             std::isfinite(sample.velocity.z);
      samples.push_back(sample);
    }
    dynamic_samples_.swap(samples);
    dynamic_objects_stamp_ = ros::Time::now();
  }

  static bool finitePoint(const geometry_msgs::Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
  }

  bool fuelCommandBlocked(const quadrotor_msgs::PositionCommand& command,
                          uint32_t* blocked_id) const {
    if (!dynamic_avoidance_enabled_ || !have_odom_ || dynamic_samples_.empty() ||
        dynamic_objects_stamp_.isZero() ||
        (ros::Time::now() - dynamic_objects_stamp_).toSec() > dynamic_object_timeout_) {
      return false;
    }
    const geometry_msgs::Point& start = odom_.pose.pose.position;
    const geometry_msgs::Point& end = command.position;
    for (const auto& sample : dynamic_samples_) {
      const double max_time = sample.have_velocity ? dynamic_prediction_horizon_ : 0.0;
      const double step = std::max(0.05, dynamic_prediction_step_);
      for (double time = 0.0; time <= max_time + 1e-6; time += step) {
        geometry_msgs::Point obstacle = sample.position;
        obstacle.x += sample.velocity.x * time;
        obstacle.y += sample.velocity.y * time;
        obstacle.z += sample.velocity.z * time;
        if (segmentIntersects(start, end, obstacle, sample.size)) {
          if (blocked_id != nullptr) *blocked_id = sample.id;
          return true;
        }
        if (max_time == 0.0) break;
      }
    }
    return false;
  }

  bool segmentIntersects(const geometry_msgs::Point& start,
                         const geometry_msgs::Point& end,
                         const geometry_msgs::Point& obstacle,
                         const geometry_msgs::Vector3& size) const {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_sq = dx * dx + dy * dy;
    double ratio = 0.0;
    if (length_sq > 1e-8) {
      ratio = ((obstacle.x - start.x) * dx + (obstacle.y - start.y) * dy) /
              length_sq;
      ratio = std::max(0.0, std::min(1.0, ratio));
    }
    const double closest_x = start.x + ratio * dx;
    const double closest_y = start.y + ratio * dy;
    const double closest_z = start.z + ratio * (end.z - start.z);
    const double horizontal_radius =
        dynamic_safety_radius_ + 0.5 * std::max(size.x, size.y);
    const double horizontal_distance =
        std::hypot(obstacle.x - closest_x, obstacle.y - closest_y);
    const double vertical_limit = dynamic_vertical_margin_ + 0.5 * size.z;
    return horizontal_distance <= horizontal_radius &&
           std::abs(obstacle.z - closest_z) <= vertical_limit;
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber fuel_sub_, diff_sub_, stage_sub_, exploration_goal_sub_, odom_sub_,
      dynamic_objects_sub_;
  ros::Timer hold_timer_;
  ros::Publisher output_pub_, owner_pub_, dynamic_hold_pub_, emergency_brake_pub_;
  nav_msgs::Odometry odom_;
  std::vector<DynamicSample> dynamic_samples_;
  ros::Time dynamic_objects_stamp_;
  bool diff_owned_{false};
  bool have_odom_{false};
  bool have_diff_command_{false};
  ros::Time last_diff_command_time_;
  uint32_t last_diff_trajectory_id_{0U};
  bool dynamic_hold_{false};
  bool dynamic_avoidance_enabled_{true};
  double dynamic_object_timeout_{0.60};
  double dynamic_safety_radius_{0.45};
  double dynamic_vertical_margin_{0.25};
  double dynamic_prediction_horizon_{1.0};
  double dynamic_prediction_step_{0.20};
  double diff_command_timeout_{0.25};
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_command_arbiter");
  PlannerCommandArbiter arbiter;
  ros::spin();
  return 0;
}
