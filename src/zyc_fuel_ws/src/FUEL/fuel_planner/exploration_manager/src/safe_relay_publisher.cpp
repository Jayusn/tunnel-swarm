#include <exploration_manager/safe_relay_publisher.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <plan_manage/planner_manager.h>
#include <stdexcept>
#include <visualization_msgs/MarkerArray.h>

namespace fast_planner {
void SafeRelayPublisher::initialize(ros::NodeHandle& nh, std::shared_ptr<SDFMap> map,
                                     std::shared_ptr<FastPlannerManager> planner) {
  map_ = std::move(map);
  planner_ = std::move(planner);
  nh.param("mission/safe_relay/enabled", enabled_, false);
  if (!enabled_) return;
  nh.param("mission/manual_path/enabled", manual_waypoints_only_, false);
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
  labels_pub_ = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/relay_labels", 1, true);
  route_pub_ = nh.advertise<nav_msgs::Path>(prefix + "/reference_route", 1, true);
  status_pub_ = nh.advertise<std_msgs::String>(prefix + "/relay_status", 1, true);
  mission_sub_ = nh.subscribe(prefix + "/task_status", 1,
                              &SafeRelayPublisher::missionCallback, this);
  exit_sub_ = nh.subscribe(prefix + "/final_exit", 1,
                           &SafeRelayPublisher::exitCallback, this);
  execution_sub_ = nh.subscribe("/UAV1/mission/relay_execution_progress", 1,
                                &SafeRelayPublisher::executionCallback, this);
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
                                  const ros::Time& stamp,
                                  const Eigen::Vector3d& corridor_forward) {
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
  // 保存采样时探索已确认的通道方向；短退、机头转动及后续转弯不改写旧段。
  point.forward = corridor_forward.allFinite() && corridor_forward.head<2>().norm() > 1e-6
      ? corridor_forward.head<2>().normalized().eval()
      : entrance_direction_.head<2>().normalized().eval();
  history_.push_back(point);
}

// 手填模式只接收已由真实里程计完成的航点，不沿途自动增加探索接力点。
void SafeRelayPublisher::recordManualWaypoint(
    const Eigen::Vector3d& point, double yaw, const ros::Time& stamp) {
  if (!enabled_ || !manual_waypoints_only_ || exited_ || history_.empty() ||
      !point.allFinite() || !std::isfinite(yaw)) return;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_;
  pose.header.stamp = stamp.isZero() ? odom_stamp_ : stamp;
  pose.pose.position.x = point.x();
  pose.pose.position.y = point.y();
  pose.pose.position.z = point.z();
  pose.pose.orientation.w = std::cos(0.5 * yaw);
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  manual_goals_.push_back({history_.size()-1,
                          static_cast<uint32_t>(manual_goals_.size()+1), pose});
}

void SafeRelayPublisher::missionCallback(const std_msgs::String::ConstPtr& message) {
  const auto& state = message->data;
  exited_ = state.find("SEARCH_OUTSIDE") == 0 || state.find("APPROACH_LANDING") == 0 ||
            state.find("LANDING") == 0;
}

void SafeRelayPublisher::exitCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (exit_received_ || message->header.frame_id != frame_) return;
  const auto& p = message->pose.position;
  const auto& q = message->pose.orientation;
  const Eigen::Vector3d center(p.x, p.y, p.z);
  const double norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (!center.allFinite() || !std::isfinite(norm) || norm < 1e-12) return;
  const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                                norm - 2.0 * (q.y*q.y + q.z*q.z));
  exit_center_ = center;
  exit_outward_ = Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
  exit_received_ = true;
}

bool SafeRelayPublisher::insideCorridor(
    const Eigen::Vector3d& reference, const Eigen::Vector3d& point) const {
  // 内部悬停包络必须留在入口内侧、已确认出口内侧，门外等待点不由这里生成。
  if (!point.allFinite() || !reference.allFinite()) return false;
  if (entrance_received_ &&
      (point - entrance_).head<2>().dot(entrance_direction_.head<2>().normalized()) <
          hold_radius_ - 1e-6) return false;
  if (exit_received_ &&
      (point - exit_center_).head<2>().dot(exit_outward_) > -hold_radius_ + 1e-6)
    return false;

  // 只检查这次局部偏移是否穿墙/未知体素，防止隔墙跳到另一侧空地。
  // 接力点之间的实际连接路径仍由后机规划；目标本身另做完整悬停包络检查。
  if (!map_->isInMap(reference) || !map_->isInMap(point) ||
      map_->getOccupancy(reference) != SDFMap::FREE ||
      map_->getOccupancy(point) != SDFMap::FREE) return false;
  Eigen::Vector3d origin, size;
  map_->getRegion(origin, size);
  RayCaster ray;
  ray.setInput((reference - origin) / map_->getResolution(),
               (point - origin) / map_->getResolution());
  Eigen::Vector3d cell;
  while (ray.step(cell)) {
    const Eigen::Vector3i index = cell.cast<int>();
    if (!map_->isInMap(index) || map_->getOccupancy(index) != SDFMap::FREE) return false;
  }
  return true;
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

double SafeRelayPublisher::routeHeight(const Record& reference,
                                      const Eigen::Vector3d& point) const {
  // XY移动后重新匹配同一小段实飞路线的高度，不能把另一位置的下降Z搬过来。
  // 以原样本累计路程限定范围，避免折返/相邻通道在XY靠近时串用高度。
  const double window = candidate_search_radius_ + sample_spacing_;
  double best_distance = (point.head<2>() - reference.sample.position.head<2>()).squaredNorm();
  double height = reference.sample.position.z();
  auto first = std::lower_bound(history_.begin(), history_.end(),
      reference.sample.progress - window, [](const Record& r, double progress) {
        return r.sample.progress < progress;
      });
  for (auto it = first; it != history_.end() &&
       it->sample.progress <= reference.sample.progress + window; ++it) {
    const double distance = (point.head<2>() - it->sample.position.head<2>()).squaredNorm();
    if (distance + 1e-9 >= best_distance) continue;
    best_distance = distance;
    height = it->sample.position.z();
  }
  return height;
}

void SafeRelayPublisher::executionCallback(const std_msgs::UInt32MultiArray::ConstPtr& message) {
  // 后机队列回报：[会话sec,nsec,回报sec,nsec,已完成最大ID,尚未开始的ID...]
  // 状态只用于前机合并队尾；缺失回报不阻止普通新点发布。
  const auto& data = message->data;
  if (data.size() < 5 || data[0] != session_.sec || data[1] != session_.nsec) return;
  const ros::Time stamp(data[2], data[3]);
  if (stamp <= execution_stamp_) return;
  for (const auto& goal : goals_)
    if (goal.id <= data[4]) completed_ids_.insert(goal.id);
  queued_ids_ = std::set<uint32_t>(data.begin() + 5, data.end());
  execution_stamp_ = stamp;
}

Eigen::Vector2d SafeRelayPublisher::routeForward(std::size_t index) const {
  return history_[index].forward;
}

bool SafeRelayPublisher::newPointFits(
    const Eigen::Vector3d& point, const Eigen::Vector2d& forward) const {
  if (goals_.empty()) return true;
  // .1/.2 已替换原点：新编号必须接在最新版本前方，不能用原始实飞样本作基准。
  const auto& last = goals_.back().pose.pose.position;
  return (point.head<2>() - Eigen::Vector2d(last.x, last.y)).dot(forward) > 1e-6;
}

bool SafeRelayPublisher::revisionFits(std::size_t index, const Eigen::Vector3d& point) const {
  const auto& goal = goals_[index];
  const auto& old = goal.pose.pose.position;
  const Eigen::Vector2d delta = point.head<2>() - Eigen::Vector2d(old.x, old.y);
  if (delta.dot(routeForward(goal.route_index)) < -1e-6) return false;
  for (std::size_t neighbour : {index == 0 ? goals_.size() : index - 1, index + 1}) {
    if (neighbour >= goals_.size()) continue;
    const auto& p = goals_[neighbour].pose.pose.position;
    const Eigen::Vector2d old_segment(old.x - p.x, old.y - p.y);
    const Eigen::Vector2d segment = point.head<2>() - Eigen::Vector2d(p.x, p.y);
    if (segment.norm() + 1e-6 < config_.minimum_xy_spacing ||
        segment.dot(old_segment) < -1e-6) return false;
  }
  return true;
}

void SafeRelayPublisher::qualifyCandidate(
    Record& record, bool new_map, const Eigen::Vector2d& forward, double search_radius,
    std::size_t goal_index) {
  if (search_radius < 0.0) search_radius = candidate_search_radius_;
  const auto& reference = record.sample.position;
  const auto boundaries = horizontalBoundaries(
      reference, search_radius + clearance_search_radius_);
  const double half_cell = 0.5 * map_->getResolution();
  struct Option { Eigen::Vector3d position; double clearance; double offset; };
  std::vector<Option> options;
  const double step = std::max(0.10, map_->getResolution());
  const int cells = static_cast<int>(std::floor(search_radius / step + 1e-6));
  for (int x = -cells; x <= cells; ++x)
    for (int y = -cells; y <= cells; ++y) {
      const double offset = step * std::hypot(x, y);
      if (offset > search_radius + 1e-6) continue;
      const Eigen::Vector3d p = reference + Eigen::Vector3d(x * step, y * step, 0.0);
      const double gap = clearanceToBoxes(p, boundaries, half_cell, clearance_search_radius_);
      if (gap + 1e-9 >= hold_radius_) options.push_back({p, gap, offset});
    }
  std::sort(options.begin(), options.end(), [](const Option& a, const Option& b) {
    if (a.clearance != b.clearance) return a.clearance > b.clearance;
    return a.offset < b.offset;
  });
  const auto safe = [&](const Eigen::Vector3d& p) {
    return (goal_index < goals_.size() ? revisionFits(goal_index, p)
                                      : newPointFits(p, forward)) &&
        insideCorridor(reference, p) && volumeFree(p, hold_radius_, hold_half_height_) &&
        planner_->isRelayHoldDynamicallySafe(p, hold_radius_, hold_half_height_, dynamic_wait_horizon_);
  };
  const auto paired = [&](Eigen::Vector3d p) {
    p.z() = routeHeight(record, p);
    return p;
  };
  const auto clearance = [&](const Eigen::Vector3d& p) {
    return std::fabs(p.z() - reference.z()) < 1e-6
        ? clearanceToBoxes(p, boundaries, half_cell, clearance_search_radius_)
        : horizontalClearance(p);
  };
  auto& candidate = record.candidate;
  candidate.hold_free = candidate.confirmed = false;
  for (const auto& option : options) {
    Eigen::Vector3d selected = paired(option.position);
    if (!safe(selected)) continue;
    double gap = clearance(selected);
    const Eigen::Vector3d previous = paired(candidate.position);
    const double previous_gap = clearance(previous);
    // Keep a still-valid choice within 2cm of the best clearance, so map-cell
    // ties do not repeatedly restart the two-map confirmation.
    if (previous_gap + 0.02 + 1e-9 >= gap && safe(previous)) {
      selected = previous;
      gap = previous_gap;
    }
    // 同等净空和上下余量时也优先向前，不再要求前方必须额外增加Z余量。
    // 搜索仍固定在实飞样本附近，避免每帧从上次候选继续漂移。
    if (forward.norm() > 0.5 && search_radius >= 0.30) {
      double best_height = verticalClearance(selected);
      const double original_gap = gap;
      double best_progress = (selected.head<2>() - reference.head<2>()).dot(forward);
      const auto consider_forward = [&](Eigen::Vector3d p) {
        const double distance = (p - reference).head<2>().dot(forward);
        if (distance <= best_progress + 1e-6) return;
        p = paired(p);
        const double forward_gap = clearance(p);
        if (forward_gap + 1e-9 < hold_radius_ || forward_gap + 0.02 + 1e-9 < original_gap ||
            !safe(p)) return;
        const double height = verticalClearance(p);
        if (height + 1e-9 < best_height) return;
        selected = p;
        gap = forward_gap;
        best_height = height;
        best_progress = distance;
      };
      for (double distance = 0.30; distance <= search_radius + 1e-6;
           distance += step) {
        Eigen::Vector3d p = reference;
        p.head<2>() += distance * forward;
        consider_forward(p);
      }
      // 正前方贴障时也比较侧前方空地；推进不能牺牲横向净空或上下余量。
      for (const auto& option : options) consider_forward(option.position);
    }
    // 定位原点到机体底部约0.15m，悬停优选再留0.05m余量。
    // 只在下方余量不足时尝试最小幅度抬高；受限位置保留原合格点，
    // 不改变普通规划/下穿，也不把所有历史点统一抬到固定高度。
    const double below = std::max(hold_half_height_, 0.15 + 0.05);
    const double paired_height = selected.z();
    const auto bottom_free = [&](const Eigen::Vector3d& p) {
      Eigen::Vector3d centre = p;
      centre.z() += 0.5 * (hold_half_height_ - below);
      return volumeFree(centre, hold_radius_, 0.5 * (hold_half_height_ + below));
    };
    if (!bottom_free(selected)) {
      const double height_step = std::max(0.05, map_->getResolution());
      for (double z = selected.z() + height_step; z <= paired_height + 0.20 + 1e-6;
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

void SafeRelayPublisher::improveInternalGoals(bool new_map) {
  if (!new_map) return;
  // 每个编号最多优化一次；已完成点由后机回报后保留原位置。
  for (std::size_t i = 0; i < goals_.size(); ++i) {
    auto& goal = goals_[i];
    if (goal.revisions != 0 || completed_ids_.count(goal.id) ||
        odom_stamp_ <= goal.pose.header.stamp) continue;
    const auto& old = goal.pose.pose.position;
    const Eigen::Vector3d original(old.x, old.y, old.z);
    auto& record = goal.revision;
    // 局部搜索固定在本段发布位置附近；向后续路段推进由队尾替换处理。
    if (record.stamp.isZero()) {
      record = history_[goal.route_index];
      record.sample.position = original;
      record.candidate = record.sample;
      record.confirmations = 0;
      record.stamp = goal.pose.header.stamp;
    }
    const Eigen::Vector2d forward = routeForward(goal.route_index);
    qualifyCandidate(record, true, forward, candidate_search_radius_, i);
    const auto& candidate = record.candidate;
    const double offset = (candidate.position - original).norm();
    if (!candidate.confirmed || offset + 1e-6 < 0.05) continue;
    const double old_gap = horizontalClearance(original);
    if (candidate.clearance + 0.02 < old_gap ||
        verticalClearance(candidate.position) + 1e-6 < verticalClearance(original)) continue;
    const double gain = std::max(0.05, map_->getResolution());
    const bool forward_gain = (candidate.position - original).head<2>().dot(forward) >=
                                  sample_spacing_ - 1e-6 &&
        candidate.clearance + 0.02 >= old_gap &&
        verticalClearance(candidate.position) + 1e-6 >= verticalClearance(original);
    const bool better = forward_gain || candidate.clearance >= old_gap + gain - 1e-6 ||
        (candidate.clearance + 0.02 >= old_gap &&
         verticalClearance(candidate.position) >= verticalClearance(original) + gain - 1e-6);
    if (!better) continue;
    updateGoal(i, candidate);
  }
}

void SafeRelayPublisher::updateGoal(std::size_t index, const safe_relay::Sample& candidate) {
  auto& goal = goals_[index];
  const auto& p = candidate.position;
  goal.pose.pose.position.x = p.x();
  goal.pose.pose.position.y = p.y();
  goal.pose.pose.position.z = p.z();
  // 替换版本时刻严格晚于原点，供后机识别更新。
  goal.pose.header.stamp = std::max(ros::Time::now(), goal.pose.header.stamp + ros::Duration(0, 1));
  ++goal.revisions;
  ROS_WARN("[safe_relay] REPLACE id=%u label=%u.%u target=(%.2f,%.2f,%.2f); same queue slot, next ID unchanged.",
           goal.id, goal.id, goal.revisions, p.x(), p.y(), p.z());
}

bool SafeRelayPublisher::replaceQueuedTail(std::size_t route_index) {
  if (goals_.empty() || execution_stamp_.isZero() ||
      (ros::Time::now() - execution_stamp_).toSec() > odom_timeout_ ||
      execution_stamp_ > ros::Time::now()) return false;
  auto& tail = goals_.back();
  if (tail.revisions != 0 || !queued_ids_.count(tail.id) ||
      completed_ids_.count(tail.id)) return false;
  const auto& candidate = history_[route_index].candidate;
  const auto& old = tail.pose.pose.position;
  const Eigen::Vector3d original(old.x, old.y, old.z);
  const Eigen::Vector2d delta = (candidate.position - original).head<2>();
  // 间隔不足一个常规接力段时，前方合格点优先替换排队中的末点。
  // 入口/出口不在此列表；已执行或正在执行的点不会被当作冗余队尾吞掉。
  if (!candidate.hold_free || !candidate.confirmed ||
      candidate.clearance + 0.02 < horizontalClearance(original) ||
      verticalClearance(candidate.position) + 1e-6 < verticalClearance(original) ||
      delta.norm() >= config_.normal_spacing ||
      delta.dot(routeForward(tail.route_index)) < sample_spacing_ ||
      !revisionFits(goals_.size() - 1, candidate.position) ||
      !insideCorridor(original, candidate.position)) return false;
  updateGoal(goals_.size() - 1, candidate);
  tail.route_index = route_index;
  tail.revision = Record();
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
  if (manual_waypoints_only_) {
    // 已到达的门内点仍可在出口交接后释放；出口外航点由手填任务层剔除。
    if (goals_.size() < manual_goals_.size()) {
      const auto& goal = manual_goals_[goals_.size()];
      const auto& p = goal.pose.pose.position;
      const Eigen::Vector3d target(p.x, p.y, p.z);
      if ((leader_ - target).head<2>().norm() >= config_.release_distance &&
          history_.back().sample.progress - history_[goal.route_index].sample.progress >=
              config_.release_distance) {
        goals_.push_back(goal);
        ROS_WARN("[safe_relay] RELEASE id=%u type=MANUAL_WAYPOINT target=(%.2f,%.2f,%.2f)",
                 goal.id, p.x, p.y, p.z);
      }
    }
    publish("MANUAL_WAYPOINT");
    return;
  }
  if (map_seen_at_.isZero() || (now - map_seen_at_).toSec() > map_timeout_) {
    publish("STALE_MAP"); return;
  }
  improveInternalGoals(new_map);
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
    // 新点和版本更新均沿该样本所属的已确认通道方向选点，回退不翻转前方。
    qualifyCandidate(record, new_map, routeForward(i));
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
    if (replaceQueuedTail(selected.index)) {
      publish("REPLACE_QUEUED_TAIL");
      return;
    }
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
  // Keep issued IDs and their last accepted XYZ even if input is now stale.
  // Freshness gates new releases; the follower owns reuse, planning and discard.
  visualization_msgs::MarkerArray labels;
  for (const auto& goal : goals_) {
    auto pose = goal.pose;
    pose.header.seq = goal.id;
    goals.poses.push_back(pose);
    visualization_msgs::Marker label;
    label.header.frame_id = frame_;
    label.ns = "safe_relay_ids";
    label.id = goal.id;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose = pose.pose;
    label.pose.position.z += 0.20;
    label.scale.z = 0.16;
    label.color.r = 1.0; label.color.g = 0.6; label.color.a = 1.0;
    label.text = std::to_string(goal.id) + (goal.revisions ? "." + std::to_string(goal.revisions) : "");
    labels.markers.push_back(label);
  }
  goals_pub_.publish(goals);
  labels_pub_.publish(labels);
  std_msgs::String message;
  message.data = status + " released=" + std::to_string(goals_.size()) +
                 " published=" + std::to_string(goals.poses.size());
  for (const auto& goal : goals_)
    if (goal.revisions) message.data += " revised=" + std::to_string(goal.id) + "." +
                                       std::to_string(goal.revisions);
  status_pub_.publish(message);
  if (status != last_status_) {
    ROS_INFO("[safe_relay] %s", message.data.c_str());
    last_status_ = status;
  }
}
}  // namespace fast_planner
