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
  params.param("minimum_xy_spacing", config_.minimum_xy_spacing, 0.70);
  params.param("entrance_minimum_progress", entrance_minimum_progress_, 0.50);
  params.param("release_distance", config_.release_distance, 1.00);
  params.param("selection_window", config_.selection_window, 0.50);
  params.param("maximum_gap", config_.maximum_gap, 1.50);
  params.param("sample_spacing", sample_spacing_, 0.08);
  params.param("hold_radius", hold_radius_, 0.45);
  params.param("hold_half_height", hold_half_height_, 0.32);
  params.param("clearance_search_radius", clearance_search_radius_, 0.60);
  params.param("candidate_search_radius", candidate_search_radius_, 0.60);
  params.param("map_timeout", map_timeout_, 1.0);
  params.param("odom_timeout", odom_timeout_, 0.5);
  params.param("dynamic_wait_horizon", dynamic_wait_horizon_, 2.0);
  params.param("map_confirmations", confirmations_required_, 2);
  params.param("maximum_samples", maximum_samples_, 10000);
  for (double value : {config_.normal_spacing, config_.minimum_progress, config_.minimum_xy_spacing,
       entrance_minimum_progress_,
       config_.release_distance, config_.selection_window, config_.maximum_gap,
       sample_spacing_, hold_radius_, hold_half_height_, clearance_search_radius_,
       candidate_search_radius_,
       map_timeout_, odom_timeout_,
       dynamic_wait_horizon_}) {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::runtime_error("safe_relay: all distances and timeouts must be finite and positive");
  }
  if (confirmations_required_ < 1 || maximum_samples_ < 3 ||
      clearance_search_radius_ < hold_radius_ ||
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
  point.candidate = point.sample;
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

std::vector<Eigen::Vector2d> SafeRelayPublisher::horizontalBoundaries(
    const Eigen::Vector3d& point, double radius) const {
  // Project all non-free voxel boxes intersecting the stopping-height band.
  // Collect once per search instead of querying the 3-D map for every option.
  const Eigen::Vector3d extent(radius, radius, hold_half_height_);
  Eigen::Vector3i lower, upper;
  map_->posToIndex(Eigen::Vector3d(point - extent), lower);
  map_->posToIndex(Eigen::Vector3d(point + extent), upper);
  std::vector<Eigen::Vector2d> boundaries;
  for (int x = lower.x(); x <= upper.x(); ++x)
    for (int y = lower.y(); y <= upper.y(); ++y) {
      for (int z = lower.z(); z <= upper.z(); ++z) {
        const Eigen::Vector3i index(x, y, z);
        if (!map_->isInMap(index) || map_->getOccupancy(index) != SDFMap::FREE) {
          Eigen::Vector3d centre;
          map_->indexToPos(index, centre);
          boundaries.push_back(centre.head<2>());
          break;
        }
      }
    }
  return boundaries;
}

namespace {
double clearanceToBoxes(const Eigen::Vector3d& point,
                        const std::vector<Eigen::Vector2d>& boundaries,
                        double half_cell, double cap) {
  double squared_distance = cap * cap;
  for (const auto& centre : boundaries) {
    const double dx = std::max(0.0, std::fabs(centre.x() - point.x()) - half_cell);
    const double dy = std::max(0.0, std::fabs(centre.y() - point.y()) - half_cell);
    squared_distance = std::min(squared_distance, dx * dx + dy * dy);
  }
  return std::sqrt(squared_distance);
}
}  // namespace

double SafeRelayPublisher::horizontalClearance(const Eigen::Vector3d& point) const {
  return clearanceToBoxes(point, horizontalBoundaries(point, clearance_search_radius_),
                          0.5 * map_->getResolution(), clearance_search_radius_);
}

double SafeRelayPublisher::verticalClearance(const Eigen::Vector3d& point) const {
  // 只用于合格悬停点的优选，不扩大原有硬性停留包络，也不改变目标高度。
  // 在既有上下余量外最多比较0.20m，未知空间不算可用高度余量。
  double height = hold_half_height_;
  const double step = std::max(0.05, map_->getResolution());
  for (double extra = step; extra <= 0.20 + 1e-6; extra += step) {
    if (!volumeFree(point, hold_radius_, hold_half_height_ + extra)) break;
    height = hold_half_height_ + extra;
  }
  return height;
}

void SafeRelayPublisher::qualifyCandidate(
    Record& record, bool new_map, const Eigen::Vector2d& forward) {
  const auto& reference = record.sample.position;
  const auto boundaries = horizontalBoundaries(
      reference, candidate_search_radius_ + clearance_search_radius_);
  const double half_cell = 0.5 * map_->getResolution();
  struct Option { Eigen::Vector3d position; double clearance; double offset; };
  std::vector<Option> options;
  const double step = std::max(0.10, map_->getResolution());
  const int cells = static_cast<int>(std::floor(candidate_search_radius_ / step + 1e-6));
  for (int x = -cells; x <= cells; ++x)
    for (int y = -cells; y <= cells; ++y) {
      const double offset = step * std::hypot(x, y);
      if (offset > candidate_search_radius_ + 1e-6) continue;
      const Eigen::Vector3d p = reference + Eigen::Vector3d(x * step, y * step, 0.0);
      const double gap = clearanceToBoxes(p, boundaries, half_cell, clearance_search_radius_);
      if (gap + 1e-9 >= hold_radius_) options.push_back({p, gap, offset});
    }
  std::sort(options.begin(), options.end(), [](const Option& a, const Option& b) {
    if (a.clearance != b.clearance) return a.clearance > b.clearance;
    return a.offset < b.offset;
  });
  const auto safe = [&](const Eigen::Vector3d& p) {
    return volumeFree(p, hold_radius_, hold_half_height_) &&
        planner_->isRelayHoldDynamicallySafe(p, hold_radius_, hold_half_height_, dynamic_wait_horizon_);
  };
  auto& candidate = record.candidate;
  candidate.hold_free = candidate.confirmed = false;
  for (const auto& option : options) {
    if (!safe(option.position)) continue;
    Eigen::Vector3d selected = option.position;
    double gap = option.clearance;
    const double previous_gap = std::fabs(candidate.position.z() - reference.z()) < 1e-6
        ? clearanceToBoxes(candidate.position, boundaries, half_cell, clearance_search_radius_)
        : horizontalClearance(candidate.position);
    // Keep a still-valid choice within 2cm of the best clearance, so map-cell
    // ties do not repeatedly restart the two-map confirmation.
    if (previous_gap + 0.02 + 1e-9 >= gap && safe(candidate.position)) {
      selected = candidate.position;
      gap = previous_gap;
    }
    // 同等水平净空下，前方已知空地若能容忍更大的Z误差，优先把悬停点
    // 放过去。以原始实飞样本为搜索基准，避免每帧从上次候选继续前移。
    if (forward.norm() > 0.5 && candidate_search_radius_ >= 0.30) {
      double best_height = verticalClearance(selected);
      const double original_gap = gap;
      const double minimum_gain = std::max(0.05, map_->getResolution());
      for (double distance = 0.30; distance <= candidate_search_radius_ + 1e-6;
           distance += step) {
        Eigen::Vector3d p = reference;
        p.head<2>() += distance * forward;
        const double forward_gap = clearanceToBoxes(
            p, boundaries, half_cell, clearance_search_radius_);
        if (forward_gap + 1e-9 < hold_radius_ || forward_gap + 0.02 + 1e-9 < original_gap ||
            !safe(p)) continue;
        const double height = verticalClearance(p);
        if (height + 1e-9 < best_height + minimum_gain) continue;
        selected = p;
        gap = forward_gap;
        best_height = height;
      }
    }
    // 定位原点到机体底部约0.15m，悬停优选再留0.05m余量。
    // 只在下方余量不足时尝试最小幅度抬高；受限位置保留原合格点，
    // 不改变普通规划/下穿，也不把所有历史点统一抬到固定高度。
    const double below = std::max(hold_half_height_, 0.15 + 0.05);
    const auto bottom_free = [&](const Eigen::Vector3d& p) {
      Eigen::Vector3d centre = p;
      centre.z() += 0.5 * (hold_half_height_ - below);
      return volumeFree(centre, hold_radius_, 0.5 * (hold_half_height_ + below));
    };
    if (!bottom_free(selected)) {
      const double height_step = std::max(0.05, map_->getResolution());
      for (double z = selected.z() + height_step; z <= reference.z() + 0.20 + 1e-6;
           z += height_step) {
        Eigen::Vector3d p = selected;
        p.z() = z;
        if (!bottom_free(p) || !safe(p)) continue;
        const double raised_gap = horizontalClearance(p);
        if (raised_gap + 0.02 + 1e-9 < gap) continue;
        selected = p;
        gap = raised_gap;
        break;
      }
    }
    if ((selected - candidate.position).norm() > 1e-6) record.confirmations = 0;
    candidate.position = selected;
    candidate.clearance = gap;
    candidate.hold_free = true;
    if (new_map) record.confirmations = std::min(confirmations_required_, record.confirmations + 1);
    candidate.confirmed = record.confirmations >= confirmations_required_;
    return;
  }
  record.confirmations = 0;
  candidate.clearance = 0.0;
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
    // 沿实飞路线前进方向寻找空地，不用可能仍保持旧通道朝向的机头yaw。
    std::size_t previous = i - 1;
    while (previous > 0 &&
           sample.progress - history_[previous].sample.progress < 0.30) --previous;
    Eigen::Vector2d forward = sample.position.head<2>() -
        history_[previous].sample.position.head<2>();
    if (forward.norm() >= 0.10) forward.normalize();
    else forward.setZero();
    qualifyCandidate(record, new_map, forward);
    samples.push_back(record.candidate);
  }
  if (!goals_.empty()) {
    const auto& p = goals_.back().pose.pose.position;
    samples[anchor_].position = Eigen::Vector3d(p.x, p.y, p.z);
  }
  if (history_.empty()) { publish("WAIT_ENTRANCE_CROSSING"); return; }
  if (exited_) { publish("EXIT_HANDOFF"); return; }
  if (history_.size() >= static_cast<std::size_t>(maximum_samples_)) {
    publish("HISTORY_CAPACITY_REACHED"); return;
  }
  // The entrance is not a previously issued goal. Only the first release
  // keeps its entrance progress threshold without the inter-goal XY gate.
  auto selection_config = config_;
  if (goals_.empty()) {
    selection_config.minimum_progress = entrance_minimum_progress_;
    selection_config.minimum_xy_spacing = 0.0;
  }
  const auto selected = safe_relay::select(samples, anchor_, leader_, selection_config);
  if (selected.kind != safe_relay::Kind::WAIT) {
    anchor_ = selected.index;
    const auto& record = history_[anchor_];
    const auto& p = record.candidate.position;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_;
    pose.header.stamp = now;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.w = std::cos(0.5 * record.yaw);
    pose.pose.orientation.z = std::sin(0.5 * record.yaw);
    goals_.push_back({anchor_, next_id_++, pose});
    ROS_WARN("[safe_relay] RELEASE id=%u type=%s target=(%.2f,%.2f,%.2f) s=%.2f horizontal_clearance=%.3fm offset=%.3fm vertical_half_clearance=%.3fm reference=(%.2f,%.2f,%.2f)",
             goals_.back().id, safe_relay::name(selected.kind), p.x(), p.y(), p.z(),
             record.sample.progress, record.candidate.clearance, (p - record.sample.position).norm(),
             verticalClearance(p), record.sample.position.x(), record.sample.position.y(),
             record.sample.position.z());
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
    auto pose = goal.pose;
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
