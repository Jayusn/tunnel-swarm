#pragma once

#include <exploration_manager/safe_relay_policy.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace fast_planner {
class SDFMap;
class FastPlannerManager;
class Astar;

// Front vehicle owns waypoint qualification; the follower owns execution.
// Path snapshots contain only currently valid, immutable released goals.
// header.stamp identifies the producer session; pose.header.seq is the goal ID.
class SafeRelayPublisher {
 public:
  void initialize(ros::NodeHandle& nh, std::shared_ptr<SDFMap> map,
                  std::shared_ptr<FastPlannerManager> planner);
  void setEntrance(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);
  void observe(const Eigen::Vector3d& position, double yaw, const ros::Time& stamp);

 private:
  friend class SafeRelayPublisherTest;
  struct Record {
    safe_relay::Sample sample;
    ros::Time stamp;
    double yaw{0.0};
    int confirmations{0};
    bool connected{true};
  };
  struct Goal {
    std::size_t route_index;
    uint32_t id;
    std::size_t connection_from{0};
    // Empty means the measured route; a detour is only connectivity evidence,
    // never a source of new stopping points or fabricated measured samples.
    std::vector<Eigen::Vector3d> connection;
  };
  void timerCallback(const ros::TimerEvent&);
  void missionCallback(const std_msgs::String::ConstPtr& message);
  bool volumeFree(const Eigen::Vector3d& point, double radius, double height) const;
  bool segmentFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
  bool pathFree(const std::vector<Eigen::Vector3d>& path) const;
  bool connectionFree(std::size_t from, std::size_t to) const;
  bool replanConnection(std::size_t from, std::size_t to, double time_limit,
                        std::vector<Eigen::Vector3d>& path);
  std::vector<Goal> validGoals() const;
  void publish(const std::string& status, bool evidence_fresh);

  std::shared_ptr<SDFMap> map_;
  std::shared_ptr<FastPlannerManager> planner_;
  std::shared_ptr<Astar> connection_search_;
  ros::Publisher goals_pub_, route_pub_, status_pub_;
  ros::Subscriber mission_sub_;
  ros::Timer timer_;
  safe_relay::Config config_;
  std::vector<Record> history_;
  std::vector<Goal> goals_;
  Eigen::Vector3d leader_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d entrance_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d entrance_direction_{Eigen::Vector3d::UnitX()};
  ros::Time session_, odom_stamp_, map_seen_at_;
  std::string frame_, last_status_;
  uint64_t map_sequence_{0};
  uint32_t next_id_{1};
  std::size_t anchor_{0};
  // Resume a budget-limited candidate sweep on the next timer tick. Only failed
  // attempts are remembered; successful paths are always checked on the live map.
  std::size_t replan_anchor_{0};
  std::set<std::size_t> replan_tried_;
  bool enabled_{false}, entrance_received_{false}, exited_{false};
  double sample_spacing_{0.08}, hold_radius_{0.45}, hold_half_height_{0.32};
  double transit_radius_{0.18}, transit_half_height_{0.10};
  double map_timeout_{1.0}, odom_timeout_{0.5}, maximum_sample_gap_{0.50};
  double dynamic_wait_horizon_{2.0};
  double replan_time_budget_{0.05};
  int confirmations_required_{2}, maximum_samples_{10000};
};
}  // namespace fast_planner
