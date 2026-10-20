#include <exploration_manager/safe_relay_publisher.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <stdexcept>

namespace fast_planner {
void SafeRelayPublisher::initialize(ros::NodeHandle& nh, std::shared_ptr<SDFMap> map,
                                     std::shared_ptr<FastPlannerManager> planner) {
  map_ = std::move(map);
  planner_ = std::move(planner);
  nh.param("mission/safe_relay/enabled", enabled_, false);
  if (!enabled_) return;
  ros::NodeHandle params(nh, "mission/safe_relay");
  std::string prefix;
  params.param<std::string>("topic_prefix", prefix, "/UAV0/mission");
  params.param<std::string>("frame_id", frame_, "world");
  params.param("normal_spacing", config_.normal_spacing, 1.00);
  params.param("minimum_progress", config_.minimum_progress, 0.50);
  params.param("entrance_minimum_progress", entrance_minimum_progress_, 0.50);
  params.param("release_distance", config_.release_distance, 1.00);
  params.param("selection_window", config_.selection_window, 0.50);
  params.param("sample_spacing", sample_spacing_, 0.08);
  params.param("hold_radius", hold_radius_, 0.45);
  params.param("hold_half_height", hold_half_height_, 0.32);
  params.param("map_timeout", map_timeout_, 1.0);
  params.param("odom_timeout", odom_timeout_, 0.5);
  params.param("dynamic_wait_horizon", dynamic_wait_horizon_, 2.0);
  params.param("map_confirmations", confirmations_required_, 2);
  params.param("maximum_samples", maximum_samples_, 10000);
  for (double value : {config_.normal_spacing, config_.minimum_progress, entrance_minimum_progress_,
       config_.release_distance, config_.selection_window,
       sample_spacing_, hold_radius_, hold_half_height_,
       map_timeout_, odom_timeout_,
       dynamic_wait_horizon_}) {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::runtime_error("safe_relay: all distances and timeouts must be finite and positive");
  }
  if (confirmations_required_ < 1 || maximum_samples_ < 3 ||
      config_.normal_spacing < config_.minimum_progress ||
      config_.normal_spacing < entrance_minimum_progress_)
    throw std::runtime_error("safe_relay: inconsistent spacing/history settings");
  const auto wall = ros::WallTime::now();
  session_ = ros::Time(wall.sec, wall.nsec);
  goals_pub_ = nh.advertise<nav_msgs::Path>(prefix + "/relay_waypoints", 1, true);
  route_pub_ = nh.advertise<nav_msgs::Path>(prefix + "/reference_route", 1, true);
  status_pub_ = nh.advertise<std_msgs::String>(prefix + "/relay_status", 1, true);
  mission_sub_ = nh.subscribe(prefix + "/task_status", 1,
                              &SafeRelayPublisher::missionCallback, this);
  timer_ = nh.createTimer(ros::Duration(0.25), &SafeRelayPublisher::timerCallback, this);
  publish("WAIT_ENTRANCE");
}

void SafeRelayPublisher::setEntrance(const Eigen::Vector3d& origin,
                                      const Eigen::Vector3d& direction) {
  if (!enabled_ || entrance_received_ || !origin.allFinite() ||
      !direction.allFinite() || direction.head<2>().norm() < 1e-6) return;
  entrance_ = origin;
  entrance_direction_ = direction.normalized();
  entrance_received_ = true;
}

void SafeRelayPublisher::observe(const Eigen::Vector3d& position, double yaw,
                                  const ros::Time& stamp) {
  if (!enabled_ || !position.allFinite() || !std::isfinite(yaw)) return;
  const ros::Time measured_at = stamp.isZero() ? ros::Time::now() : stamp;
  if (!odom_stamp_.isZero() && measured_at <= odom_stamp_) return;
  odom_stamp_ = measured_at;
  leader_ = position;
  if (!entrance_received_) return;
  // Begin at the actual entrance crossing, not at the geometric door centre.
  if (history_.empty() && (position - entrance_).dot(entrance_direction_) < 0.0) return;
  if (history_.size() >= static_cast<std::size_t>(maximum_samples_)) return;
  double step = 0.0;
  if (!history_.empty()) {
    step = (position - history_.back().sample.position).norm();
    if (step < sample_spacing_) return;
  }
  Record point;
  point.sample.position = position;  // Keep measured XYZ; never flatten to follower height.
  point.sample.progress = history_.empty() ? 0.0 : history_.back().sample.progress + step;
  point.stamp = measured_at;
  point.yaw = yaw;
  history_.push_back(point);
}

void SafeRelayPublisher::missionCallback(const std_msgs::String::ConstPtr& message) {
  const auto& state = message->data;
  exited_ = state.find("SEARCH_OUTSIDE") == 0 || state.find("APPROACH_LANDING") == 0 ||
            state.find("LANDING") == 0;
}

bool SafeRelayPublisher::volumeFree(const Eigen::Vector3d& point, double radius,
                                     double height) const {
  if (!point.allFinite() || !map_->isInMap(point) || map_->getInflateOccupancy(point) != 0)
    return false;
  const double half_cell = 0.5 * map_->getResolution();
  const Eigen::Vector3d extent(radius, radius, height);
  if (!map_->isInMap(Eigen::Vector3d(point - extent)) ||
      !map_->isInMap(Eigen::Vector3d(point + extent))) return false;
  Eigen::Vector3i lower, upper;
  map_->posToIndex(Eigen::Vector3d(point - extent), lower);
  map_->posToIndex(Eigen::Vector3d(point + extent), upper);
  for (int x = lower.x(); x <= upper.x(); ++x)
    for (int y = lower.y(); y <= upper.y(); ++y)
      for (int z = lower.z(); z <= upper.z(); ++z) {
        const Eigen::Vector3i index(x, y, z);
        Eigen::Vector3d centre;
        map_->indexToPos(index, centre);
        // Test all voxels intersecting the cylinder, not just sparse radial rays.
        const double dx = std::max(0.0, std::fabs(centre.x() - point.x()) - half_cell);
        const double dy = std::max(0.0, std::fabs(centre.y() - point.y()) - half_cell);
        if (dx * dx + dy * dy > radius * radius + 1e-9) continue;
        if (!map_->isInMap(index) || map_->getOccupancy(index) != SDFMap::FREE) return false;
      }
  return true;
}

void SafeRelayPublisher::timerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  const uint64_t sequence = map_->getUpdateSequence();
  const bool new_map = sequence != map_sequence_;
  if (new_map) { map_sequence_ = sequence; map_seen_at_ = now; }
  if (odom_stamp_.isZero() || (now - odom_stamp_).toSec() > odom_timeout_ ||
      odom_stamp_ > now + ros::Duration(0.1)) {
    publish("STALE_ODOMETRY"); return;
  }
  if (map_seen_at_.isZero() || (now - map_seen_at_).toSec() > map_timeout_) {
    publish("STALE_MAP"); return;
  }
  // Issued points are history, not live validity certificates. Never move the
  // spacing origin backwards when a follower or obstacle occupies an old point.
  anchor_ = goals_.empty() ? 0 : goals_.back().route_index;
  std::vector<safe_relay::Sample> samples;
  samples.reserve(history_.size());
  for (std::size_t i = 0; i < history_.size(); ++i) {
    auto& record = history_[i];
    auto& sample = record.sample;
    if (i <= anchor_) {
      samples.push_back(sample);
      continue;  // Neither the entrance nor issued points need requalification.
    }
    sample.hold_free = volumeFree(sample.position, hold_radius_, hold_half_height_) &&
                      planner_->isRelayHoldDynamicallySafe(sample.position, hold_radius_,
                                                          hold_half_height_, dynamic_wait_horizon_);
    if (!sample.hold_free) record.confirmations = 0;
    else if (new_map) record.confirmations = std::min(confirmations_required_, record.confirmations + 1);
    sample.confirmed = record.confirmations >= confirmations_required_;
    sample.clearance = sample.hold_free ? map_->getDistance(sample.position) : 0.0;
    samples.push_back(sample);
  }
  if (history_.empty()) { publish("WAIT_ENTRANCE_CROSSING"); return; }
  if (exited_) { publish("EXIT_HANDOFF"); return; }
  if (history_.size() >= static_cast<std::size_t>(maximum_samples_)) {
    publish("HISTORY_CAPACITY_REACHED"); return;
  }
  // Only the first release uses the entrance fallback spacing.
  auto selection_config = config_;
  if (goals_.empty()) selection_config.minimum_progress = entrance_minimum_progress_;
  const auto selected = safe_relay::select(samples, anchor_, leader_, selection_config);
  if (selected.kind != safe_relay::Kind::WAIT) {
    anchor_ = selected.index;
    goals_.push_back({anchor_, next_id_++});
    const auto& p = history_[anchor_].sample.position;
    ROS_WARN("[safe_relay] RELEASE id=%u type=%s actual=(%.2f,%.2f,%.2f) s=%.2f",
             goals_.back().id, safe_relay::name(selected.kind), p.x(), p.y(), p.z(),
             history_[anchor_].sample.progress);
  }
  publish(safe_relay::name(selected.kind));
}

void SafeRelayPublisher::publish(const std::string& status) {
  nav_msgs::Path route;
  route.header.frame_id = frame_;
  route.header.stamp = odom_stamp_;
  for (std::size_t i = 0; i < history_.size(); ++i) {
    const auto& record = history_[i];
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_;
    pose.header.stamp = record.stamp;
    pose.header.seq = static_cast<uint32_t>(i);
    pose.pose.position.x = record.sample.position.x();
    pose.pose.position.y = record.sample.position.y();
    pose.pose.position.z = record.sample.position.z();
    pose.pose.orientation.w = std::cos(0.5 * record.yaw);
    pose.pose.orientation.z = std::sin(0.5 * record.yaw);
    route.poses.push_back(pose);
  }
  route_pub_.publish(route);  // Ordinary routes are always published, including while waiting.
  nav_msgs::Path goals;
  goals.header.frame_id = frame_;
  goals.header.stamp = session_;
  // Keep all issued IDs/XYZ even if map or odometry evidence is now stale.
  // Freshness gates new releases; the follower owns reuse, planning and discard.
  for (const auto& goal : goals_) {
    auto pose = route.poses[goal.route_index];
    pose.header.seq = goal.id;
    goals.poses.push_back(pose);
  }
  goals_pub_.publish(goals);
  std_msgs::String message;
  message.data = status + " released=" + std::to_string(goals_.size()) +
                 " published=" + std::to_string(goals.poses.size());
  status_pub_.publish(message);
  if (status != last_status_) {
    ROS_INFO("[safe_relay] %s", message.data.c_str());
    last_status_ = status;
  }
}
}  // namespace fast_planner
