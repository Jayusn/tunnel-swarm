#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <quadrotor_msgs/PolynomialTrajUav0.h>

#include <string>

namespace {

std::string firstToken(const std::string& text) {
  const std::size_t end = text.find_first_of(" \t\r\n");
  return text.substr(0, end);
}

bool isDiffSearchStage(const std::string& stage) {
  return stage == "SEARCH_OUTSIDE_LANDING" ||
         stage == "SEARCH_OUTSIDE_QR" ||
         stage == "APPROACH_LANDING" || stage == "LANDING";
}

class AutoTransTrajectoryArbiter {
 public:
  AutoTransTrajectoryArbiter() : nh_(), private_nh_("~") {
    std::string fuel_topic("/UAV0/fuel/autotrans_trajectory");
    std::string diff_topic("/UAV0/diff/autotrans_trajectory");
    std::string output_topic("/UAV0/planning/autotrans_trajectory");
    std::string stage_topic("/UAV0/mission/task_status");
    std::string rc_land_topic("/UAV0/mission/rc_land_active");
    private_nh_.param("fuel_topic", fuel_topic, fuel_topic);
    private_nh_.param("diff_topic", diff_topic, diff_topic);
    private_nh_.param("output_topic", output_topic, output_topic);
    private_nh_.param("stage_topic", stage_topic, stage_topic);
    private_nh_.param("rc_land_active_topic", rc_land_topic, rc_land_topic);

    output_pub_ =
        nh_.advertise<quadrotor_msgs::PolynomialTrajUav0>(output_topic, 2, false);
    owner_pub_ = private_nh_.advertise<std_msgs::String>("owner", 1, true);
    fuel_sub_ = nh_.subscribe(fuel_topic, 4,
                              &AutoTransTrajectoryArbiter::fuelCallback, this);
    diff_sub_ = nh_.subscribe(diff_topic, 4,
                              &AutoTransTrajectoryArbiter::diffCallback, this);
    stage_sub_ = nh_.subscribe(stage_topic, 4,
                               &AutoTransTrajectoryArbiter::stageCallback, this);
    rc_land_sub_ = nh_.subscribe(rc_land_topic, 1,
                                &AutoTransTrajectoryArbiter::rcLandCallback, this);
    publishOwner();
    ROS_WARN("autotrans_trajectory_arbiter: FUEL owns AutoTrans trajectory input");
  }

 private:
  void rcLandCallback(const std_msgs::BoolConstPtr& message) {
    if (!message->data || rc_land_active_) return;
    rc_land_active_ = true;
    quadrotor_msgs::PolynomialTrajUav0 abort;
    abort.header.stamp = ros::Time::now();
    abort.trajectory_id = last_forwarded_trajectory_id_;
    abort.action = quadrotor_msgs::PolynomialTrajUav0::ACTION_ABORT;
    output_pub_.publish(abort);
    publishOwner();
    ROS_ERROR("CH10 landing latched: FUEL and Diff trajectory forwarding stopped");
  }

  void stageCallback(const std_msgs::StringConstPtr& message) {
    if (rc_land_active_) return;
    const std::string stage = firstToken(message->data);
    if (diff_owned_ || !isDiffSearchStage(stage)) return;

    // CH9 后立即清除 AutoTrans 中尚未执行完的 FUEL 轨迹；新的 Diff 轨迹
    // 到达前由 AutoTrans 自身的无轨迹悬停保护维持当前位置。
    diff_owned_ = true;
    quadrotor_msgs::PolynomialTrajUav0 abort_message;
    abort_message.header.stamp = ros::Time::now();
    abort_message.trajectory_id = last_forwarded_trajectory_id_;
    abort_message.action = quadrotor_msgs::PolynomialTrajUav0::ACTION_ABORT;
    output_pub_.publish(abort_message);
    publishOwner();
    ROS_ERROR("autotrans_trajectory_arbiter: ownership latched FUEL -> DIFF, "
              "stale FUEL trajectory aborted");
  }

  void fuelCallback(const quadrotor_msgs::PolynomialTrajUav0ConstPtr& message) {
    if (rc_land_active_) return;
    if (diff_owned_) return;
    forward(*message);
  }

  void diffCallback(const quadrotor_msgs::PolynomialTrajUav0ConstPtr& message) {
    if (rc_land_active_) return;
    if (!diff_owned_) return;
    forward(*message);
  }

  void forward(const quadrotor_msgs::PolynomialTrajUav0& message) {
    output_pub_.publish(message);
    last_forwarded_trajectory_id_ = message.trajectory_id;
  }

  void publishOwner() {
    std_msgs::String owner;
    owner.data = rc_land_active_ ? "RC_LAND" : (diff_owned_ ? "DIFF" : "FUEL");
    owner_pub_.publish(owner);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber fuel_sub_;
  ros::Subscriber diff_sub_;
  ros::Subscriber stage_sub_;
  ros::Subscriber rc_land_sub_;
  bool rc_land_active_{false};
  ros::Publisher output_pub_;
  ros::Publisher owner_pub_;
  bool diff_owned_{false};
  uint32_t last_forwarded_trajectory_id_{0U};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "autotrans_trajectory_arbiter");
  AutoTransTrajectoryArbiter arbiter;
  ros::spin();
  return 0;
}
