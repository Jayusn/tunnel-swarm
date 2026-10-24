#pragma once

#include <exploration_manager/safe_relay_policy.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <vector>

namespace fast_planner {
class SDFMap;
class FastPlannerManager;

// Front vehicle owns waypoint qualification; the follower owns execution.
// Path snapshots retain qualified goals; goal 8 permits one local improvement (8.1).
// Later obstacle changes and route planning belong to the follower.
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
    safe_relay::Sample candidate;
    ros::Time stamp;
    double yaw{0.0};
    int confirmations{0};
  };
  struct Goal {
    std::size_t route_index;
    uint32_t id;
    geometry_msgs::PoseStamped pose;
    bool revised{false};
    Record revision;
  };
  void timerCallback(const ros::TimerEvent&);
  void missionCallback(const std_msgs::String::ConstPtr& message);
  bool volumeFree(const Eigen::Vector3d& point, double radius, double height) const;
  double horizontalClearance(const Eigen::Vector3d& point) const;
  std::vector<Eigen::Vector2d> horizontalBoundaries(const Eigen::Vector3d& point,
                                                  double radius) const;
  double verticalClearance(const Eigen::Vector3d& point) const;
  double routeHeight(const Record& reference, const Eigen::Vector3d& point) const;
  void qualifyCandidate(Record& record, bool new_map, const Eigen::Vector2d& forward,
                         double search_radius = -1.0);
  void improveEighthGoal(bool new_map);
  void publish(const std::string& status);

  std::shared_ptr<SDFMap> map_;
  std::shared_ptr<FastPlannerManager> planner_;
  ros::Publisher goals_pub_, route_pub_, status_pub_, labels_pub_;
  ros::Subscriber mission_sub_;
  ros::Timer timer_;
  safe_relay::Config config_;
  double entrance_minimum_progress_{0.50};
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
  bool enabled_{false}, entrance_received_{false}, exited_{false};
  double sample_spacing_{0.08}, hold_radius_{0.45}, hold_half_height_{0.32};
  double clearance_search_radius_{0.60};
  double candidate_search_radius_{0.60};
  double map_timeout_{1.0}, odom_timeout_{0.5};
  double dynamic_wait_horizon_{2.0};
  int confirmations_required_{2}, maximum_samples_{10000};
};
}  // namespace fast_planner
