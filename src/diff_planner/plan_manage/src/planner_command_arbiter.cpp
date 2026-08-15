#include <string>

#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
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
    private_nh_.param("fuel_topic", fuel_topic, fuel_topic);
    private_nh_.param("diff_topic", diff_topic, diff_topic);
    private_nh_.param("output_topic", output_topic, output_topic);
    private_nh_.param("stage_topic", stage_topic, stage_topic);
    output_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(output_topic, 10);
    owner_pub_ = private_nh_.advertise<std_msgs::String>("owner", 2, true);
    fuel_sub_ = nh_.subscribe(fuel_topic, 20, &PlannerCommandArbiter::fuelCallback, this);
    diff_sub_ = nh_.subscribe(diff_topic, 20, &PlannerCommandArbiter::diffCallback, this);
    stage_sub_ = nh_.subscribe(stage_topic, 10, &PlannerCommandArbiter::stageCallback, this);
    publishOwner();
  }

 private:
  void stageCallback(const std_msgs::StringConstPtr& msg) {
    const std::string stage = firstToken(msg->data);
    if (!diff_owned_ && (stage == "SEARCH_OUTSIDE_LANDING" ||
                         stage == "SEARCH_OUTSIDE_QR" ||
                         stage == "APPROACH_LANDING" || stage == "LANDING")) {
      diff_owned_ = true;
      publishOwner();
      ROS_ERROR("planner_command_arbiter: control ownership latched FUEL -> DIFF");
    }
  }
  void fuelCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (!diff_owned_) output_pub_.publish(*msg);
  }
  void diffCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (diff_owned_) output_pub_.publish(*msg);
  }
  void publishOwner() {
    std_msgs::String msg;
    msg.data = diff_owned_ ? "DIFF" : "FUEL";
    owner_pub_.publish(msg);
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber fuel_sub_, diff_sub_, stage_sub_;
  ros::Publisher output_pub_, owner_pub_;
  bool diff_owned_{false};
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_command_arbiter");
  PlannerCommandArbiter arbiter;
  ros::spin();
  return 0;
}
