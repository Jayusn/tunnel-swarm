// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/safe_relay_publisher.h>
#include <exploration_manager/inflation_history_escape_policy.h>
#include <exploration_manager/entry_forward_policy.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <std_msgs/Bool.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>
#include <exploration_manager/motion_direction_rules.h>
#include <exploration_manager/path_shortening_policy.h>
#include <exploration_manager/passage_centering_policy.h>
#include <exploration_manager/task_search_manager.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
using namespace Eigen;

namespace fast_planner {
namespace {
// 远处悬挂物只用于预见；进入半米接近段后才刹停并开始固定 XY 下穿。
constexpr double kUnderpassStartDistance = 0.50;
}
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::recordPublishedTrajectoryHeight() {
  if (manual_fapp_active_ && fapp_rush_pending_) {
    fapp_rush_active_ = true;
    fapp_rush_pending_ = false;
    if (fapp_rush_pub_) { std_msgs::Bool mode; mode.data = true; fapp_rush_pub_.publish(mode); }
  }
  if (width_route_plan_) {
    // Only a published replacement releases the old side. A rejected width
    // probe must not change the commitment of the still-executing trajectory.
    if (!manual_fapp_active_) {
      recovery_side_latched_ = false;
      wide_side_bypass_active_ = false;
      recovery_side_continuation_pending_ = false;
    }
    width_route_plan_ = false;
  }
  if (manual_fapp_active_ && manual_fapp_curve_active_ &&
      std::isfinite(pending_reference_height_)) {
    manual_fapp_hold_height_ = low_probe_height_; // 仅已发布的下穿轨迹可更新低位参考。
    // 下穿回退为普通规划时仍保留已发布曲线的侧向，不能重新默认选左侧。
    if (!recovery_side_latched_) recovery_side_origin_ = manual_fapp_curve_origin_;
    recovery_side_latched_ = true;
    recovery_side_dir_ = (manual_fapp_curve_side_ < 0.0 ? -1.0 : 1.0) *
        Vector3d(-manual_fapp_forward_.y(), manual_fapp_forward_.x(), 0.0);
  }
  if (std::isfinite(pending_reference_height_))
    horizontal_reference_height_ = pending_reference_height_;
  pending_reference_height_ = std::numeric_limits<double>::quiet_NaN();
}

double FastExplorationManager::horizontalGoalHeight(double current_height) const {
  // 下穿会话拥有专用低位目标，不能被旧巡航参考抬回障碍物内。
  if (low_probe_phase_ != vertical_detour::LowProbePhase::IDLE)
    return low_probe_height_;
  if (!manual_path_enabled_ && manual_fapp_active_ && std::isfinite(manual_fapp_hold_height_))
    return manual_fapp_hold_height_;
  const double height = std::isfinite(horizontal_reference_height_)
      ? horizontal_reference_height_ : current_height;
  return task_search_manager_ ? task_search_manager_->clampSearchHeight(height) : height;
}

bool FastExplorationManager::pointInBox(const Vector3d& pt, const Vector3d& box_min,
                                        const Vector3d& box_max) const {
  return pt.x() >= box_min.x() && pt.x() <= box_max.x() && pt.y() >= box_min.y() &&
         pt.y() <= box_max.y() && pt.z() >= box_min.z() && pt.z() <= box_max.z();
}

bool FastExplorationManager::cameraOccupancyColumn(const Vector3d& point,
                                                   double reference_z) const {
  if (!sdf_map_) return false;
  // 2026-07-24: 相机障碍物触发只看相对当前高度的短柱，避免FAST-LIO绝对z漂移导致漏检。
  const double step = std::max(0.08, sdf_map_->getResolution());
  int occupied_layers = 0;
  for (double dz = -0.30; dz <= 0.30 + 1e-6; dz += step) {
    Vector3d probe(point.x(), point.y(), reference_z + dz);
    if (sdf_map_->isInMap(probe) &&
        sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
      ++occupied_layers;
  }
  return occupied_layers >= 2;
}

bool FastExplorationManager::cameraWallSupported(
    const Vector3d& point, const Eigen::Vector2d& travel_dir) const {
  if (!sdf_map_ || travel_dir.norm() < 1e-3) return false;
  // 2026-07-24: 沿局部通道方向持续占据的是墙；允许一个栅格横向误差以适应弯曲/离散墙面。
  const double step = std::max(0.10, sdf_map_->getResolution());
  const Eigen::Vector2d dir = travel_dir.normalized();
  const Eigen::Vector2d lateral(-dir.y(), dir.x());
  const double half_length = 0.5 * std::max(0.50, camera_wall_support_length_);
  int supported = 0;
  int total = 0;
  for (double along = -half_length; along <= half_length + 1e-6; along += step) {
    ++total;
    bool hit = false;
    for (double offset : { -step, 0.0, step }) {
      Vector3d probe = point;
      probe.head<2>() += along * dir + offset * lateral;
      if (cameraOccupancyColumn(probe, point.z())) {
        hit = true;
        break;
      }
    }
    if (hit) ++supported;
  }
  return total > 0 &&
         static_cast<double>(supported) / static_cast<double>(total) >=
             camera_wall_support_ratio_;
}

bool FastExplorationManager::cameraObstacleAlreadyScanned(
    const Vector3d& obstacle_center) const {
  if (camera_obstacle_scan_active_ &&
      (camera_active_obstacle_.head<2>() - obstacle_center.head<2>()).norm() <
          camera_obstacle_dedup_radius_)
    return true;
  for (const auto& scanned : camera_scanned_obstacles_) {
    if ((scanned.head<2>() - obstacle_center.head<2>()).norm() <
        camera_obstacle_dedup_radius_)
      return true;
  }
  return false;
}

bool FastExplorationManager::detectCameraObstacle(
    const Vector3d& pos, const Eigen::Vector2d& travel_dir,
    Vector3d& obstacle_center) const {
  if (!sdf_map_ || travel_dir.norm() < 1e-3) return false;
  const Eigen::Vector2d dir = travel_dir.normalized();
  const Eigen::Vector2d lateral(-dir.y(), dir.x());
  const double step = std::max(0.10, sdf_map_->getResolution());
  vector<Vector3d> residual_points;

  // 2026-07-24: 只检查前方近场；连续墙带被剔除，剩余柱子/箱体/向通道内突出的占据簇才触发相机环扫。
  for (double forward = camera_obstacle_min_forward_;
       forward <= camera_obstacle_max_forward_ + 1e-6; forward += step) {
    for (double side = -camera_obstacle_lateral_range_;
         side <= camera_obstacle_lateral_range_ + 1e-6; side += step) {
      Vector3d sample = pos;
      sample.head<2>() += forward * dir + side * lateral;
      if (!cameraOccupancyColumn(sample, pos.z())) continue;
      if (cameraWallSupported(sample, dir)) continue;
      residual_points.push_back(sample);
    }
  }
  if (residual_points.empty()) return false;

  // 2026-07-24: 以最近剩余点为种子做轻量连通聚合，至少三个栅格才接受，过滤零散点云噪声。
  auto nearest_it = std::min_element(
      residual_points.begin(), residual_points.end(),
      [&pos](const Vector3d& lhs, const Vector3d& rhs) {
        return (lhs.head<2>() - pos.head<2>()).squaredNorm() <
               (rhs.head<2>() - pos.head<2>()).squaredNorm();
      });
  const Vector3d seed = *nearest_it;
  Vector3d sum = Vector3d::Zero();
  int cluster_count = 0;
  const double cluster_radius = std::max(0.30, 2.5 * step);
  for (const auto& point : residual_points) {
    if ((point.head<2>() - seed.head<2>()).norm() <= cluster_radius) {
      sum += point;
      ++cluster_count;
    }
  }
  if (cluster_count < 3) return false;
  obstacle_center = sum / static_cast<double>(cluster_count);
  obstacle_center.z() = pos.z();
  return !cameraObstacleAlreadyScanned(obstacle_center);
}

void FastExplorationManager::updateMissionRegionState(const Vector3d& pos) {
  if (mission_entered_search_region_) return;
  if (pointInBox(pos, ep_->mission_search_region_min_, ep_->mission_search_region_max_)) {
    // 2026-07-08 19:26: 一旦机体真正进入比赛搜索区，就锁存状态，后续不再退回“起飞区引导”模式。
    mission_entered_search_region_ = true;
    ROS_INFO("[mission_exploration] entered search region at %.2f %.2f %.2f.", pos.x(), pos.y(),
             pos.z());
  }
}

bool FastExplorationManager::shouldUseMissionEntryTransit(const Vector3d& pos) const {
  if (!ep_->mission_use_entry_transit_ || mission_entered_search_region_) return false;
  // 2026-07-10: 动态门平面锁只会在 corridor_search_manager 完成找门/穿门阶段后发布。
  // 此时 FUEL 已经处于门后搜索阶段，禁止继续使用旧的静态 entry_goal，避免穿门后又被拉回起飞区。
  if (ep_->mission_use_workspace_lock_ && mission_workspace_lock_received_) return false;
  return (pos - ep_->mission_entry_goal_).norm() > ep_->mission_entry_arrive_dist_;
}

void FastExplorationManager::workspaceLockCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  mission_workspace_origin_ =
      Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  const auto& q = msg->pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  mission_workspace_dir_ = Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  mission_workspace_lock_received_ = true;
  // 2026-07-13: 入口坐标只作为通道坐标系和初始推进参考，深处拐弯由任务覆盖历史决定。
  if (task_search_manager_)
    task_search_manager_->setCorridorFrame(mission_workspace_origin_, mission_workspace_dir_);
  ROS_WARN_THROTTLE(1.0,
                    "[workspace_lock] received door plane origin=(%.2f, %.2f) yaw=%.1fdeg.",
                    mission_workspace_origin_.x(), mission_workspace_origin_.y(),
                    yaw * 180.0 / M_PI);
}

bool FastExplorationManager::pointInsideWorkspaceLock(const Vector3d& pt) const {
  if (!ep_->mission_global_no_return_ || !ep_->mission_use_workspace_lock_ ||
      !mission_workspace_lock_received_)
    return true;
  const Vector3d rel = pt - mission_workspace_origin_;
  const double progress = rel.x() * mission_workspace_dir_.x() + rel.y() * mission_workspace_dir_.y();
  return progress >= -ep_->mission_door_back_margin_;
}

bool FastExplorationManager::pathInsideWorkspaceLock(
    const vector<Vector3d>& path, bool allow_entry_start) const {
  if (!ep_->mission_global_no_return_ || !ep_->mission_use_workspace_lock_ ||
      !mission_workspace_lock_received_)
    return true;
  for (const auto& pt : path) {
    if (!pointInsideWorkspaceLock(pt)) {
      if (allow_entry_start && task_search_manager_ &&
          entry_forward::pathStartsTowardInside(
              path, mission_workspace_origin_, mission_workspace_dir_,
              task_search_manager_->entryStartBackMargin())) {
        ROS_WARN_THROTTLE(
            1.0, "[entry_forward] allow inward launch from %.3fm outside entrance "
                 "within %.2fm start margin; goal remains inside.",
            -(path.front() - mission_workspace_origin_).dot(mission_workspace_dir_),
            task_search_manager_->entryStartBackMargin());
        return true;
      }
      return false;
    }
  }
  return true;
}

void FastExplorationManager::applyMissionFrontierFilter() {
  const auto old_frontiers = ed_->frontiers_;
  const auto old_boxes = ed_->frontier_boxes_;
  const auto old_points = ed_->points_;
  const auto old_yaws = ed_->yaws_;
  const auto old_averages = ed_->averages_;

  vector<vector<Vector3d>> filtered_frontiers;
  vector<pair<Vector3d, Vector3d>> filtered_boxes;
  vector<Vector3d> filtered_points;
  vector<double> filtered_yaws;
  vector<Vector3d> filtered_averages;
  int rejected_workspace = 0;
  int rejected_takeoff = 0;
  int rejected_region = 0;
  // 入口是整段任务的永久禁回头边界：进入后仍持续比较候选点和完整A*路径，
  // 防止后续地图更新再次把入口外侧frontier选回来。
  const bool workspace_lock_active =
      ep_->mission_use_workspace_lock_ && mission_workspace_lock_received_;

  // 2026-07-10: 收到动态门平面后，门后搜索方向由 workspace_lock 决定；旧的静态 search_region
  // 只适合早期固定地图验证，不能再误杀当前门方向上的候选点。
  const bool use_static_search_region =
      ep_->mission_prefer_search_region_frontiers_ && !mission_workspace_lock_received_;

  bool has_search_region_candidate = false;
  if (use_static_search_region) {
    for (size_t i = 0; i < old_points.size(); ++i) {
      if (pointInBox(old_points[i], ep_->mission_search_region_min_,
                     ep_->mission_search_region_max_)) {
        has_search_region_candidate = true;
        break;
      }
    }
  }

  for (size_t i = 0; i < old_points.size(); ++i) {
    const bool in_takeoff_box = ep_->mission_use_takeoff_exclusion_box_ &&
                                pointInBox(old_points[i], ep_->mission_takeoff_exclusion_min_,
                                           ep_->mission_takeoff_exclusion_max_);
    const bool in_search_region =
        pointInBox(old_points[i], ep_->mission_search_region_min_, ep_->mission_search_region_max_);
    // 2026-07-10: 正常情况下 averages 与 viewpoints 等长；这里兜底防止异常日志/旧数据导致越界。
    const Vector3d frontier_ref = i < old_averages.size() ? old_averages[i] : old_points[i];
    const bool inside_workspace =
        pointInsideWorkspaceLock(old_points[i]) && pointInsideWorkspaceLock(frontier_ref);

    if (!inside_workspace) {
      ++rejected_workspace;
      continue;
    }
    if (in_takeoff_box && (mission_entered_search_region_ || has_search_region_candidate)) {
      ++rejected_takeoff;
      continue;
    }
    if (use_static_search_region && has_search_region_candidate && !in_search_region) {
      ++rejected_region;
      continue;
    }

    filtered_frontiers.push_back(old_frontiers[i]);
    filtered_boxes.push_back(old_boxes[i]);
    filtered_points.push_back(old_points[i]);
    filtered_yaws.push_back(old_yaws[i]);
    filtered_averages.push_back(frontier_ref);
  }

  if (!filtered_points.empty() || workspace_lock_active) {
    // 2026-07-10: workspace_lock 是硬约束，即使过滤后为空也不能回退到门外 frontier；
    // 空集合由上层 hold_on_no_frontier 处理，避免 FUEL 为补起飞区地图掉头出门。
    ed_->frontiers_.swap(filtered_frontiers);
    ed_->frontier_boxes_.swap(filtered_boxes);
    ed_->points_.swap(filtered_points);
    ed_->yaws_.swap(filtered_yaws);
    ed_->averages_.swap(filtered_averages);
  }

  ROS_WARN_THROTTLE(1.0,
                    "[workspace_lock] frontier kept=%zu rejected_workspace=%d rejected_takeoff=%d "
                    "rejected_region=%d lock=%d received=%d static_region=%d.",
                    ed_->points_.size(), rejected_workspace, rejected_takeoff, rejected_region,
                    static_cast<int>(workspace_lock_active),
                    static_cast<int>(mission_workspace_lock_received_),
                    static_cast<int>(use_static_search_region));
}

bool FastExplorationManager::isKnownSafeHorizontalCorridor(
    const Vector3d& start, const Vector3d& direction, double distance) const {
  if (!sdf_map_ || !planner_manager_ || direction.head<2>().norm() < 1e-3) return false;
  // distance 是水平距离；保留高度差，检查实际起点到定高目标的完整斜段。
  const Vector3d unit = direction / direction.head<2>().norm();
  const double sample_step = std::max(0.05, 0.5 * sdf_map_->getResolution());
  const int ring_samples = 12;
  for (double along = 0.0; along <= distance + 1e-6; along += sample_step) {
    Vector3d center = start + along * unit;
    if (!planner_manager_->isPositionSafe(center)) return false;
    for (double z_offset : {-0.06, 0.0, 0.06}) {
      for (int sample = 0; sample < ring_samples; ++sample) {
        const double angle = 2.0 * M_PI * static_cast<double>(sample) /
                             static_cast<double>(ring_samples);
        Vector3d probe = center;
        probe.x() += vertical_detour_footprint_radius_ * std::cos(angle);
        probe.y() += vertical_detour_footprint_radius_ * std::sin(angle);
        probe.z() += z_offset;
        if (!sdf_map_->isInMap(probe) ||
            sdf_map_->getOccupancy(probe) != SDFMap::FREE)
          return false;
      }
    }
  }
  return true;
}

bool FastExplorationManager::isKnownSafeVerticalPath(const Vector3d& start,
                                                      double target_z,
                                                      bool allow_unknown) const {
  if (!sdf_map_ || !planner_manager_) return false;
  vector<Vector3d> path{start};
  const double distance = std::fabs(target_z - start.z());
  const int samples = std::max(2, static_cast<int>(std::ceil(distance / 0.05)));
  for (int sample = 1; sample <= samples; ++sample) {
    Vector3d point = start;
    point.z() = start.z() + (target_z - start.z()) * static_cast<double>(sample) /
                                static_cast<double>(samples);
    // 向下试探允许进入UNKNOWN补图；下穿结束后的上升释放则要求整根中心柱
    // 已知自由，不能把球体后方尚未重扫的UNKNOWN误认为已经穿出障碍。
    const int occupancy = sdf_map_->isInMap(point)
                              ? sdf_map_->getOccupancy(point)
                              : SDFMap::OCCUPIED;
    if (!sdf_map_->isInMap(point) || occupancy == SDFMap::OCCUPIED ||
        (!allow_unknown && occupancy != SDFMap::FREE))
      return false;
    path.push_back(point);
  }
  return planner_manager_->isPathSafe(path) &&
      (!(manual_path_enabled_ && manual_path_exploration_style_) ||
       planner_manager_->isFappPathSafe(path));
}

bool FastExplorationManager::isKnownSafeGroundAscentPath(
    const Vector3d& start, const Vector3d& target) const {
  if (!sdf_map_ || !planner_manager_ || target.z() <= start.z() + 0.02 ||
      (target - start).head<2>().norm() > 1e-6)
    return false;
  vector<Vector3d> path{start};
  const int samples = std::max(
      2, static_cast<int>(std::ceil((target.z() - start.z()) / 0.05)));
  for (int sample = 1; sample <= samples; ++sample) {
    Vector3d point = start;
    point.z() = start.z() + (target.z() - start.z()) *
                                static_cast<double>(sample) /
                                static_cast<double>(samples);
    if (!sdf_map_->isInMap(point) ||
        sdf_map_->getOccupancy(point) != SDFMap::FREE)
      return false;
    path.push_back(point);
  }
  const bool allow_contact_escape =
      planner_manager_->isControlledEscapePosition(start);
  return planner_manager_->isPathSafe(path, allow_contact_escape);
}

bool FastExplorationManager::hasSafeCruiseContinuation(
    const Vector3d& pos, double yaw) const {
  if (!task_search_manager_ || !planner_manager_) return false;
  vector<Vector3d> directions{task_search_manager_->recoveryForwardDirection(yaw)};
  if (recovery_side_latched_) directions.push_back(recovery_side_dir_);
  for (const auto& direction : directions) {
    if (direction.head<2>().norm() < 1e-3) continue;
    const Vector3d target = pos + 0.20 * Vector3d(
        direction.x(), direction.y(), 0.0).normalized();
    const vector<Vector3d> path{pos, target};
    if (pointInsideWorkspaceLock(target) && pathInsideWorkspaceLock(path) &&
        task_search_manager_->isRecoveryCandidateUseful(target) &&
        task_search_manager_->isRecoveryPathAllowed(path, false) &&
        isLowProbeCorridorSafe(pos, direction, 0.20))
      return true;
  }
  return false;
}

bool FastExplorationManager::tryCruiseHeightRecovery(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, double time_lb) {
  if (manual_fapp_active_ || !ground_ascent_enabled_ || !mission_entered_search_region_ ||
      !task_search_manager_ || !planner_manager_ || ground_ascent_active_ ||
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE ||
      suspended_underpass_active_ || ed_->path_next_goal_.size() < 2)
    return false;
  const double target_z = task_search_manager_->projectSearchHeight(pos.z(), pos.z());
  if (task_search_manager_->preferredSearchHeight() - pos.z() <=
          ground_ascent_height_tolerance_ || target_z - pos.z() <= 0.02)
    return false;

  auto path = ed_->path_next_goal_;
  const Vector3d trajectory_end =
      planner_manager_->local_data_.position_traj_.evaluateDeBoorT(
          planner_manager_->local_data_.duration_);
  // 只提升已验证的水平路线；专用上下绕行、已有爬升及截断后的不同终点不重写。
  if ((path.front() - pos).norm() > 0.05 ||
      (path.back() - trajectory_end).norm() > 0.05 ||
      std::any_of(path.begin(), path.end(), [&](const Vector3d& point) {
        return std::fabs(point.z() - pos.z()) > 0.05;
      }))
    return false;
  vector<double> progress(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i)
    progress[i] = progress[i-1] + (path[i] - path[i-1]).head<2>().norm();
  if (progress.back() < 0.10) return false;
  // 沿原水平折线渐升：每米水平路程最多抬高0.5m，且不超过任务单段爬升量。
  const double rise = std::min(target_z - pos.z(), 0.5 * progress.back());
  if (rise <= 0.02) return false;
  for (size_t i = 0; i < path.size(); ++i)
    path[i].z() = pos.z() + rise * progress[i] / progress.back();
  if (path.size() == 2)
    path.insert(path.begin() + 1, 0.5 * (path.front() + path.back()));
  if (!pathInsideWorkspaceLock(path) ||
      !task_search_manager_->isRecoveryPathAllowed(path, false))
    return false;
  // 先检查斜升整段的三维机体空间；抬高后的终点自由并不足以授权爬升。
  for (size_t i = 1; i < path.size(); ++i) {
    const Vector3d delta = path[i] - path[i-1];
    const int count = std::max(1, static_cast<int>(std::ceil(delta.norm() / 0.025)));
    for (int j = 0; j <= count; ++j) {
      if (!planner_manager_->isKnownFreePosition(path[i-1] + delta * (double(j) / count))) {
        ROS_INFO_THROTTLE(1.0, "[cruise_recovery] upper swept space unavailable; "
                         "keep the validated horizontal route and observe while moving.");
        return false;
      }
    }
  }
  // 高度恢复是原路线的可选改进。优化或动态边界失败时恢复整条原轨迹，不能换边。
  const LocalTrajData original = planner_manager_->local_data_;
  bool safe = planner_manager_->planExploreTraj(path, vel, acc, time_lb) &&
              planner_manager_->isTrajectorySafe();
  if (safe) {
    auto& trajectory = planner_manager_->local_data_.position_traj_;
    const double duration = planner_manager_->local_data_.duration_;
    const int count = std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
    vector<Vector3d> swept;
    swept.reserve(count + 1);
    for (int j = 0; j <= count; ++j) {
      const Vector3d point = trajectory.evaluateDeBoorT(duration * double(j) / count);
      if (!planner_manager_->isKnownFreePosition(point) ||
          point.z() < pos.z() - 0.02 || point.z() > target_z + 0.02) {
        safe = false;
        break;
      }
      swept.push_back(point);
    }
    safe = safe && pathInsideWorkspaceLock(swept) &&
           task_search_manager_->isRecoveryPathAllowed(swept, false);
  }
  if (!safe) {
    planner_manager_->local_data_ = original;
    ROS_WARN("[cruise_recovery] raised trajectory rejected; retain original horizontal trajectory.");
    return false;
  }
  ed_->path_next_goal_ = path;
  ed_->next_goal_ = path.back();
  last_requested_goal_ = path.back();
  has_last_requested_goal_ = true;
  task_search_manager_->recordSelectedGoal(path.back());
  ROS_INFO("[cruise_recovery] retain horizontal route and climb %.2f -> %.2fm "
           "over %.2fm toward cruise height %.2fm.",
           pos.z(), path.back().z(), progress.back(), task_search_manager_->preferredSearchHeight());
  return true;
}

bool FastExplorationManager::shouldStartGroundAscent(
    const Vector3d& pos) const {
  if (!ground_ascent_enabled_ || !mission_entered_search_region_ ||
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE)
    return false;
  const double cruise_height = task_search_manager_
                                   ? std::max(ground_ascent_cruise_height_,
                                              task_search_manager_->preferredSearchHeight())
                                   : ground_ascent_cruise_height_;
  Vector3d target = pos;
  target.z() = cruise_height;
  const int lower_obstacles = planner_manager_->lowerFootprintObstacleCount(
      pos, ground_ascent_lower_probe_depth_);
  const bool path_safe = pointInsideWorkspaceLock(target) &&
                         isKnownSafeGroundAscentPath(pos, target);
  const bool needed = vertical_detour::groundAscentRecoveryNeeded(
      pos.z(), cruise_height, ground_ascent_height_tolerance_,
      lower_obstacles, ground_ascent_min_lower_obstacles_, path_safe);
  if (!needed && lower_obstacles >= ground_ascent_min_lower_obstacles_ &&
      pos.z() < cruise_height - ground_ascent_height_tolerance_) {
    ROS_WARN_THROTTLE(
        0.5,
        "[ground_ascent] lower obstacle detected (%d samples), but known-safe "
        "vertical column to %.2fm is unavailable; vertical recovery is not selected.",
        lower_obstacles, cruise_height);
  }
  return needed;
}

bool FastExplorationManager::buildGroundAscentPlan(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw) {
  const ros::Time now = ros::Time::now();
  if (!ground_ascent_active_) {
    if (!shouldStartGroundAscent(pos)) return false;
    ground_ascent_active_ = true;
    // 0.6 m remains the preferred cruise layer, but low-altitude recovery must
    // not monopolize planning when the vehicle cannot settle or climb safely.
    ground_ascent_optional_ = true;
    ground_ascent_yaw_ = yaw[0];
    ground_ascent_target_ = pos;
    ground_ascent_target_.z() = task_search_manager_
                                    ? std::max(ground_ascent_cruise_height_,
                                               task_search_manager_->preferredSearchHeight())
                                    : ground_ascent_cruise_height_;
    ground_ascent_anchor_valid_ = false;
    ground_ascent_still_since_ = ros::Time(0);
    cancelActiveLowProbe("ground ascent recovery owns xy/yaw");
    ROS_ERROR("[ground_ascent] try before horizontal escape: z=%.2f -> %.2f, "
              "resume normal planning if stationary or climb conditions are unavailable.",
              pos.z(), ground_ascent_target_.z());
  }

  const double height_error = ground_ascent_target_.z() - pos.z();
  const double xy_error = (pos - ground_ascent_target_).head<2>().norm();
  const double speed = vel.norm();
  if (height_error <= ground_ascent_height_tolerance_ &&
      xy_error <= vertical_detour_xy_tolerance_) {
    if (!std::isfinite(speed) || speed > ground_ascent_max_stationary_speed_) {
      ground_ascent_still_since_ = ros::Time(0);
      return false;
    }
    if (ground_ascent_still_since_.isZero()) ground_ascent_still_since_ = now;
    if ((now - ground_ascent_still_since_).toSec() <
        ground_ascent_still_confirm_time_)
      return false;
    ground_ascent_active_ = false;
    ground_ascent_optional_ = false;
    ground_ascent_anchor_valid_ = false;
    ground_ascent_still_since_ = ros::Time(0);
    ROS_ERROR("[ground_ascent] cruise height %.2fm confirmed; resume horizontal planning.",
              pos.z());
    return false;
  }

  if (!std::isfinite(speed) || speed > ground_ascent_max_stationary_speed_) {
    if (ground_ascent_optional_) {
      ground_ascent_active_ = false;
      ground_ascent_optional_ = false;
      ground_ascent_anchor_valid_ = false;
      ground_ascent_still_since_ = ros::Time(0);
      ROS_WARN("[ground_ascent] optional recovery to fixed %.2fm yielded: "
               "speed %.3fm/s is not stationary; keep the configured cruise "
               "height and resume normal planning.",
               ground_ascent_target_.z(), speed);
      return false;
    }
    ground_ascent_still_since_ = ros::Time(0);
    ROS_WARN_THROTTLE(0.5,
                      "[ground_ascent] wait for brake: speed=%.3fm/s limit=%.3fm/s.",
                      speed, ground_ascent_max_stationary_speed_);
    return false;
  }

  // 首条垂直轨迹从最新静止里程计重新锁定 XY，避免把检测瞬间的旧锚点
  // 带入发布门控。安全重规划同样从当时的静止点重新锁定，绝不生成斜升段。
  if (!ground_ascent_anchor_valid_) {
    if (ground_ascent_still_since_.isZero()) ground_ascent_still_since_ = now;
    if ((now - ground_ascent_still_since_).toSec() <
        ground_ascent_still_confirm_time_)
      return false;
    ground_ascent_anchor_ = pos;
    ground_ascent_anchor_valid_ = true;
    ground_ascent_target_.x() = pos.x();
    ground_ascent_target_.y() = pos.y();
  }

  Vector3d start = pos;
  ground_ascent_anchor_ = pos;
  ground_ascent_target_.x() = pos.x();
  ground_ascent_target_.y() = pos.y();
  const bool controlled_escape =
      planner_manager_->isControlledEscapePosition(start);
  const bool ascent_ready =
      isKnownSafeGroundAscentPath(start, ground_ascent_target_) &&
      planner_manager_->planVerticalTraj(start, ground_ascent_target_) &&
      planner_manager_->isTrajectorySafe(0.03, controlled_escape) &&
      (!controlled_escape || planner_manager_->trajectoryClearsInflation(
                                 std::fabs(ground_ascent_target_.z() - start.z()) + 0.15,
                                 0.02, true)) &&
      planner_manager_->planYawTurnInPlace(yaw, ground_ascent_yaw_);
  if (!ascent_ready) {
    if (ground_ascent_optional_) {
      ground_ascent_active_ = false;
      ground_ascent_optional_ = false;
      ground_ascent_anchor_valid_ = false;
      ground_ascent_still_since_ = ros::Time(0);
      ROS_WARN("[ground_ascent] optional recovery to fixed %.2fm is unavailable; "
               "keep the configured cruise height and resume normal planning.",
               ground_ascent_target_.z());
    }
    return false;
  }

  ed_->path_next_goal_ = {start, 0.5 * (start + ground_ascent_target_),
                          ground_ascent_target_};
  ed_->next_goal_ = ground_ascent_target_;
  last_requested_goal_ = ground_ascent_target_;
  has_last_requested_goal_ = true;
  ground_ascent_plan_ = true;
  pending_reference_height_ = ground_ascent_target_.z();
  ground_ascent_still_since_ = ros::Time(0);
  ROS_ERROR("[ground_ascent] publish fixed-xy/yaw recovery at (%.2f,%.2f), "
            "z %.2f -> %.2f.",
            start.x(), start.y(), start.z(), ground_ascent_target_.z());
  return true;
}

bool FastExplorationManager::isLowProbeCorridorSafe(
    const Vector3d& start, const Vector3d& direction, double distance) const {
  if (!sdf_map_ || !planner_manager_ || direction.head<2>().norm() < 1e-3)
    return false;
  const Vector3d unit = direction / direction.head<2>().norm();
  const double sample_step = std::max(0.05, 0.5 * sdf_map_->getResolution());
  vector<Vector3d> path{start};
  for (double along = sample_step; along <= distance + 1e-6;
       along += sample_step) {
    const Vector3d point = start + along * unit;
    // 中心线必须由低位传感器确认为自由；机体半径和膨胀只交给统一路径安全检查，
    // 不再重复要求0.23m圆环的每个栅格都必须是已知FREE。
    if (!sdf_map_->isInMap(point) ||
        sdf_map_->getOccupancy(point) != SDFMap::FREE)
      return false;
    path.push_back(point);
  }
  const Vector3d endpoint = start + distance * unit;
  if (!sdf_map_->isInMap(endpoint) ||
      sdf_map_->getOccupancy(endpoint) != SDFMap::FREE)
    return false;
  if ((path.back() - endpoint).norm() > 1e-3) path.push_back(endpoint);
  return planner_manager_->isPathSafe(path) &&
      (!(manual_path_enabled_ && manual_path_exploration_style_) ||
       planner_manager_->isFappPathSafe(path));
}

bool FastExplorationManager::buildVerticalDetourFallback(
    const Vector3d& pos, double cur_yaw, const Vector3d& forward,
    Vector3d& next_pos, double& next_yaw, bool force_descend,
    bool allow_descend) {
  if (!vertical_detour_enabled_ || forward.head<2>().norm() < 1e-3 ||
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE)
    return false;

  const Vector3d direction(forward.head<2>().normalized().x(),
                           forward.head<2>().normalized().y(), 0.0);

  auto startLowProbe = [&]() {
    // 通用绕障也必须确认悬空，不能因侧绕失败或低位 UNKNOWN 直接下降补图。
    if (!hasSuspendedForwardObstacle(pos, direction)) return false;
    const double low_z = vertical_detour_low_height_;
    low_probe_height_ = low_z;
    if (pos.z() <= low_z + vertical_detour_height_tolerance_ ||
        !isKnownSafeVerticalPath(pos, low_z))
      return false;
    low_probe_phase_ = vertical_detour::LowProbePhase::DESCENDING;
    low_probe_confirmations_ = 0;
    low_probe_release_confirmations_ = 0;
    low_probe_advanced_distance_ = 0.0;
    low_probe_origin_ = pos;
    low_probe_direction_ = direction;
    // 低于正常巡航层开始下穿时，必须一直检查到巡航高度的整根上升柱；否则
    // 只回到原来的低高度后普通frontier会立刻抬升，仍可能在球体正下方撞上去。
    low_probe_return_height_ = pos.z();
    if (task_search_manager_)
      low_probe_return_height_ = std::max(
          low_probe_return_height_, task_search_manager_->preferredSearchHeight());
    low_probe_yaw_ = cur_yaw;
    low_probe_target_ = pos;
    low_probe_target_.z() = low_z;
    low_probe_verify_start_ = ros::Time();
    low_probe_descent_end_time_ = ros::Time();
    next_pos = low_probe_target_;
    next_yaw = cur_yaw;
    ROS_WARN("[vertical_detour] start vertical-only low probe z=%.2f -> %.2f at fixed "
             "xy=(%.2f,%.2f), yaw=%.1fdeg.",
             pos.z(), low_z, pos.x(), pos.y(), cur_yaw * 180.0 / M_PI);
    return true;
  };

  // 已经确认悬挂障碍下方走廊安全时，必须真正进入下降会话；不能再让
  // 通用的上绕候选抢先返回，否则日志虽显示“优先下穿”却不会下降。
  if (force_descend) return allow_descend && startLowProbe();

  const bool descend_first = vertical_detour::preferDescending(
      pos.z(), vertical_detour_down_first_height_);
  if (allow_descend && descend_first && startLowProbe()) {
    ROS_WARN("[vertical_detour] z=%.2f > %.2f; choose downward detour before upward.",
             pos.z(), vertical_detour_down_first_height_);
    return true;
  }

  // 高层按正前及左右前斜向组成扇形搜索，并逐段扩大前进距离；始终不生成后向候选。
  const double max_side_angle =
      std::min(89.0, std::max(0.0, vertical_detour_upper_max_side_angle_deg_)) *
      M_PI / 180.0;
  const vector<double> side_angles{
      0.0, 0.5 * max_side_angle, -0.5 * max_side_angle,
      max_side_angle, -max_side_angle};
  double obstacle_top = pos.z();
  if (!findForwardObstacleTop(pos, direction, obstacle_top)) {
    ROS_WARN_THROTTLE(
        0.5,
        "[vertical_detour] no occupied obstacle top found in the forward prism; "
        "skip unbounded upper detour.");
    return false;
  }
  const double minimum_upper_z =
      obstacle_top + vertical_detour_upper_obstacle_clearance_;
  for (double rise = vertical_detour_upper_step_;
       rise <= vertical_detour_upper_max_rise_ + 1e-6;
       rise += vertical_detour_upper_step_) {
    if (pos.z() + rise + 1e-6 < minimum_upper_z) continue;
    for (double upper_distance = vertical_detour_forward_check_distance_;
         upper_distance <= vertical_detour_upper_forward_max_distance_ + 1e-6;
         upper_distance += vertical_detour_forward_check_distance_) {
      for (const double angle : side_angles) {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        const Vector3d upper_direction(c * direction.x() - s * direction.y(),
                                       s * direction.x() + c * direction.y(), 0.0);
        Vector3d elevated_start = pos;
        elevated_start.z() += rise;
        const Vector3d candidate = elevated_start + upper_distance * upper_direction;
        if (!pointInsideWorkspaceLock(candidate) ||
            !isKnownSafeHorizontalCorridor(elevated_start, upper_direction,
                                           upper_distance))
          continue;
        planner_manager_->path_finder_->reset();
        if (planner_manager_->path_finder_->search(pos, candidate) != Astar::REACH_END)
          continue;
        const auto path = planner_manager_->path_finder_->getPath();
        if (!planner_manager_->isPathSafe(path) ||
            (task_search_manager_ &&
             !task_search_manager_->isRecoveryPathAllowed(path, false)))
          continue;
        next_pos = candidate;
        next_yaw = cur_yaw;
        ROS_WARN("[vertical_detour] use upper diagonal corridor rise=%.2fm "
                 "distance=%.2fm side=%.1fdeg target=(%.2f,%.2f,%.2f), "
                 "obstacle_top=%.2fm clearance=%.2fm.",
                 rise, upper_distance, angle * 180.0 / M_PI, next_pos.x(),
                 next_pos.y(), next_pos.z(), obstacle_top,
                 next_pos.z() - obstacle_top);
        return true;
      }
    }
  }

  // 低位优先上绕；上绕不可行时仍允许下降作为第二选择。
  if (allow_descend && !descend_first && startLowProbe()) return true;
  return false;
}

bool FastExplorationManager::handleActiveLowProbe(
    const Vector3d& pos, Vector3d& next_pos, double& next_yaw,
    bool& wait_for_confirmation) {
  wait_for_confirmation = false;
  if (low_probe_phase_ == vertical_detour::LowProbePhase::IDLE) return false;

  const auto previous_phase = low_probe_phase_;
  if (sdf_map_ &&
      (previous_phase == vertical_detour::LowProbePhase::DESCENDING ||
      previous_phase == vertical_detour::LowProbePhase::VERIFYING ||
       previous_phase == vertical_detour::LowProbePhase::ADVANCING)) {
    // 下降补图后，若发现前方障碍有落地支撑，就退出整个下穿会话。
    // UNKNOWN 不代表非悬空；这里只使用明确的占据和连续低位支撑证据。
    const Vector3d travel = low_probe_direction_.normalized();
    const Vector3d lateral(-travel.y(), travel.x(), 0.0);
    const double step = std::max(0.05, sdf_map_->getResolution());
    const double reference_height =
        std::max(low_probe_origin_.z(), low_probe_return_height_);
    for (double ahead = step; ahead <= kUnderpassStartDistance + 1e-6; ahead += step) {
      for (double side = -vertical_detour_footprint_radius_;
           side <= vertical_detour_footprint_radius_ + 1e-6; side += step) {
        const Vector3d probe = pos + ahead * travel + side * lateral;
        if (cameraOccupancyColumn(probe, reference_height) &&
            hasLowVerticalSupport(probe, reference_height)) {
          cancelActiveLowProbe("updated map confirms supported obstacle; resume normal planning");
          return false;
        }
      }
    }
  }
  if (previous_phase == vertical_detour::LowProbePhase::ASCENDING) {
    const double xy_error =
        (pos - low_probe_ascent_origin_).head<2>().norm();
    const double elapsed = low_probe_ascent_start_.isZero()
                               ? 0.0
                               : (ros::Time::now() - low_probe_ascent_start_).toSec();
    if (vertical_detour::ascentShouldAbort(
            xy_error, vertical_detour_xy_tolerance_, elapsed,
            vertical_detour_ascent_timeout_)) {
      ROS_ERROR("[vertical_detour] cancel stale ascent: xy_error=%.2fm "
                "elapsed=%.2fs; release old target and resume live planning.",
                xy_error, elapsed);
      cancelActiveLowProbe("ascent xy drift or timeout");
      return false;
    }
  }
  vertical_detour::LowProbeObservation observation;
  observation.at_probe_height =
      std::fabs(pos.z() - low_probe_height_) <=
      vertical_detour_height_tolerance_;
  observation.xy_stable =
      (pos - low_probe_origin_).head<2>().norm() <= vertical_detour_xy_tolerance_;
  if (previous_phase == vertical_detour::LowProbePhase::DESCENDING &&
      !low_probe_descent_end_time_.isZero() &&
      (!observation.at_probe_height || !observation.xy_stable) &&
      (ros::Time::now() - low_probe_descent_end_time_).toSec() >=
          vertical_detour_descent_tracking_timeout_) {
    ROS_WARN("[vertical_detour] descent tracking timed out: xy_error=%.3fm "
             "z_error=%.3fm; abandon old target and replan from live position.",
             (pos - low_probe_origin_).head<2>().norm(),
             std::fabs(pos.z() - low_probe_height_));
    cancelActiveLowProbe("descent endpoint tracking timeout");
    return false;
  }
  observation.low_corridor_safe =
      previous_phase == vertical_detour::LowProbePhase::VERIFYING &&
      isLowProbeCorridorSafe(pos, low_probe_direction_,
                             vertical_detour_forward_check_distance_);
  observation.verification_timed_out =
      previous_phase == vertical_detour::LowProbePhase::VERIFYING &&
      !low_probe_verify_start_.isZero() &&
      (ros::Time::now() - low_probe_verify_start_).toSec() >=
          vertical_detour_verification_timeout_;
  observation.advance_complete =
      previous_phase == vertical_detour::LowProbePhase::ADVANCING &&
      (pos - low_probe_target_).norm() <= 0.10;
  observation.ascent_complete =
      previous_phase == vertical_detour::LowProbePhase::ASCENDING &&
      (pos - low_probe_ascent_origin_).head<2>().norm() <=
          vertical_detour_xy_tolerance_ &&
      std::fabs(pos.z() - low_probe_return_height_) <=
          vertical_detour_height_tolerance_;

  // 低位停住验证或正在短步前进时，只要当前高度的前方走廊被新静态障碍
  // 挡住，就直接以实时 xyz 做左右绕障，不先退出球下方或回到巡航高度。
  const bool low_forward_blocked =
      (previous_phase == vertical_detour::LowProbePhase::VERIFYING &&
       !observation.low_corridor_safe) ||
      (previous_phase == vertical_detour::LowProbePhase::ADVANCING &&
       !isLowProbeCorridorSafe(pos, low_probe_direction_,
                               vertical_detour_forward_check_distance_));
  if (low_forward_blocked) {
    bool split_obstacle_detected = false;
    Vector3d side_target;
    double side_yaw = low_probe_yaw_;
    bool use_side_target = false;
    bool dynamic_obstacle = false;
    if (manual_path_enabled_ && manual_path_exploration_style_) {
      use_side_target = buildFappSideBypass(pos, low_probe_yaw_, low_probe_direction_,
                                           side_target, side_yaw, dynamic_obstacle);
    }
    if (!use_side_target && !dynamic_obstacle)
      use_side_target = buildWideSideBypass(
          pos, low_probe_yaw_, low_probe_direction_, side_target, side_yaw,
          split_obstacle_detected);
    split_obstacle_detected = split_obstacle_detected || dynamic_obstacle;
    if (split_obstacle_detected && (use_side_target || !suspended_underpass_active_)) {
      cancelActiveLowProbe("low corridor blocked; use horizontal lane");
      ROS_WARN("[vertical_detour] low-altitude forward path is blocked; keep current z "
               "and switch directly to the wider horizontal lane.");
      if (use_side_target) {
        next_pos = side_target;
        next_yaw = side_yaw;
        return true;
      }
      return false;
    }
  }

  // 每个0.20m低位短步结束后，必须验证当前位置到恢复高度的整根垂直柱，
  // 以及恢复高度正前方短走廊。球仍在头顶时只会继续低位推进，绝不会提前抬升。
  const bool check_ascent =
      observation.advance_complete ||
      previous_phase == vertical_detour::LowProbePhase::EXIT_VERIFYING ||
      observation.verification_timed_out;
  if (check_ascent) {
    const bool past_obstacle = !suspended_underpass_active_ ||
        (!manual_fapp_active_ &&
         pos.head<2>().dot(low_probe_direction_.head<2>()) >= low_probe_exit_progress_);
    observation.ascent_path_safe = past_obstacle &&
        isKnownSafeVerticalPath(pos, low_probe_return_height_, false);
    Vector3d return_level_start = pos;
    return_level_start.z() = low_probe_return_height_;
    observation.release_corridor_safe =
        observation.ascent_path_safe &&
        isKnownSafeHorizontalCorridor(return_level_start, low_probe_direction_,
                                      vertical_detour_forward_check_distance_);
  }

  low_probe_phase_ = vertical_detour::advanceLowProbe(
      previous_phase, observation, vertical_detour_required_confirmations_,
      vertical_detour_release_confirmations_, low_probe_confirmations_,
      low_probe_release_confirmations_);

  if (previous_phase == vertical_detour::LowProbePhase::DESCENDING &&
      low_probe_phase_ == vertical_detour::LowProbePhase::VERIFYING) {
    low_probe_verify_start_ = ros::Time::now();
    ROS_WARN("[vertical_detour] reached low probe height %.2fm; hold xy/yaw and verify "
             "%.2fm forward corridor %d consecutive times.",
             pos.z(), vertical_detour_forward_check_distance_,
             vertical_detour_required_confirmations_);
  } else if (previous_phase == vertical_detour::LowProbePhase::VERIFYING &&
             low_probe_phase_ == vertical_detour::LowProbePhase::ADVANCING) {
    low_probe_origin_ = pos;
    low_probe_target_ = pos + vertical_detour_forward_check_distance_ *
                                  low_probe_direction_;
    low_probe_target_.z() = low_probe_height_;
    ROS_WARN("[vertical_detour] low corridor confirmed %d/%d; release next %.2fm "
             "centered forward target.",
             low_probe_confirmations_, vertical_detour_required_confirmations_,
             vertical_detour_forward_check_distance_);
  } else if (previous_phase == vertical_detour::LowProbePhase::VERIFYING &&
             low_probe_phase_ == vertical_detour::LowProbePhase::ASCENDING) {
    low_probe_target_ = pos;
    low_probe_target_.z() = low_probe_return_height_;
    ROS_WARN("[vertical_detour] low corridor verification timed out and vertical return "
             "column is safe; ascend in place to %.2fm.",
             low_probe_return_height_);
  } else if (previous_phase == vertical_detour::LowProbePhase::ADVANCING &&
             observation.advance_complete) {
    low_probe_advanced_distance_ +=
        (low_probe_target_ - low_probe_origin_).head<2>().norm();
    low_probe_origin_ = pos;
    low_probe_target_ = pos;
    low_probe_target_.z() = low_probe_height_;
    if (low_probe_phase_ == vertical_detour::LowProbePhase::EXIT_VERIFYING) {
      ROS_WARN("[vertical_detour] advanced %.2fm below obstacle; ascent column and "
               "return-level corridor safe %d/%d, hold low altitude for release check.",
               low_probe_advanced_distance_, low_probe_release_confirmations_,
               vertical_detour_release_confirmations_);
    } else if (low_probe_phase_ == vertical_detour::LowProbePhase::ASCENDING) {
      low_probe_target_.z() = low_probe_return_height_;
      ROS_WARN("[vertical_detour] obstacle fully cleared after %.2fm low advance; "
               "ascend vertically to %.2fm.",
               low_probe_advanced_distance_, low_probe_return_height_);
    } else {
      low_probe_verify_start_ = ros::Time::now();
      ROS_WARN("[vertical_detour] obstacle still overhead after %.2fm; keep z=%.2fm and "
               "verify the next %.2fm forward segment.",
               low_probe_advanced_distance_, low_probe_height_,
               vertical_detour_forward_check_distance_);
    }
  } else if (previous_phase == vertical_detour::LowProbePhase::EXIT_VERIFYING &&
             low_probe_phase_ == vertical_detour::LowProbePhase::ASCENDING) {
    low_probe_target_ = pos;
    low_probe_target_.z() = low_probe_return_height_;
    ROS_WARN("[vertical_detour] underpass exit confirmed %d/%d after %.2fm; ascend "
             "vertically to %.2fm before normal planning.",
             low_probe_release_confirmations_, vertical_detour_release_confirmations_,
             low_probe_advanced_distance_, low_probe_return_height_);
  } else if (previous_phase == vertical_detour::LowProbePhase::EXIT_VERIFYING &&
             low_probe_phase_ == vertical_detour::LowProbePhase::VERIFYING) {
    low_probe_origin_ = pos;
    low_probe_target_ = pos;
    low_probe_target_.z() = low_probe_height_;
    low_probe_verify_start_ = ros::Time::now();
    ROS_WARN("[vertical_detour] underpass exit clearance changed; remain low and verify "
             "the next forward segment instead of climbing.");
  }

  if (previous_phase != vertical_detour::LowProbePhase::ASCENDING &&
      low_probe_phase_ == vertical_detour::LowProbePhase::ASCENDING) {
    low_probe_ascent_origin_ = pos;
    low_probe_ascent_start_ = ros::Time::now();
  }

  if (low_probe_phase_ == vertical_detour::LowProbePhase::IDLE) {
    suspended_underpass_active_ = false;
    ROS_WARN("[vertical_detour] staged low detour finished after %.2fm; resume normal planning.",
             low_probe_advanced_distance_);
    return false;
  }
  if (low_probe_phase_ == vertical_detour::LowProbePhase::VERIFYING ||
      low_probe_phase_ == vertical_detour::LowProbePhase::EXIT_VERIFYING) {
    wait_for_confirmation = true;
    if (low_probe_phase_ == vertical_detour::LowProbePhase::EXIT_VERIFYING) {
      ROS_INFO_THROTTLE(
          0.5,
          "[vertical_detour] exit clearance confirmation %d/%d safe=%d; holding low "
          "xy/z/yaw before ascent.",
          low_probe_release_confirmations_, vertical_detour_release_confirmations_,
          static_cast<int>(observation.release_corridor_safe));
    } else {
      ROS_INFO_THROTTLE(0.5,
                        "[vertical_detour] low corridor confirmation %d/%d safe=%d; "
                        "holding fixed xy/yaw.",
                        low_probe_confirmations_, vertical_detour_required_confirmations_,
                        static_cast<int>(observation.low_corridor_safe));
    }
    return false;
  }

  next_pos = low_probe_target_;
  next_yaw = low_probe_yaw_;
  return true;
}

bool FastExplorationManager::buildStationaryVerticalDetourPlan(
    const Vector3d& pos, const Vector3d& yaw) {
  const bool descending =
      low_probe_phase_ == vertical_detour::LowProbePhase::DESCENDING;
  const bool ascending =
      low_probe_phase_ == vertical_detour::LowProbePhase::ASCENDING;
  if (!descending && !ascending) return false;

  // 目标高度属于低空探测会话，但 XY 始终以本次调用的实时位置为准，杜绝斜向
  // 下降/上升。下降允许沿 UNKNOWN 补图，上升仍要求整根柱已知自由。
  Vector3d target = low_probe_target_;
  target.x() = pos.x();
  target.y() = pos.y();
  if (manual_path_enabled_ && manual_path_max_segment_length_ > 0.0)
    target.z() = pos.z() + std::max(-manual_path_max_segment_length_,
        std::min(manual_path_max_segment_length_, target.z() - pos.z()));
  if (!isKnownSafeVerticalPath(pos, target.z(), descending)) return false;
  if (!planner_manager_->planVerticalTraj(pos, target)) return false;
  const bool controlled_escape =
      planner_manager_->isControlledEscapePosition(pos);
  if (!planner_manager_->isTrajectorySafe(0.03, controlled_escape)) return false;
  if (!planner_manager_->planYawTurnInPlace(yaw, low_probe_yaw_)) return false;

  // 竖直短段可以截断，低位会话仍保留最终高度，下一段继续下降/回升。
  low_probe_target_.x() = target.x();
  low_probe_target_.y() = target.y();
  if (descending) {
    low_probe_origin_.x() = pos.x();
    low_probe_origin_.y() = pos.y();
  } else {
    low_probe_ascent_origin_.x() = pos.x();
    low_probe_ascent_origin_.y() = pos.y();
  }
  ed_->path_next_goal_ = {pos, 0.5 * (pos + target), target};
  ed_->next_goal_ = target;
  last_requested_goal_ = target;
  has_last_requested_goal_ = true;
  vertical_detour_plan_ = true;
  pending_reference_height_ = target.z();
  ROS_ERROR("[vertical_detour] build live fixed-xy %s at (%.2f,%.2f), "
            "z %.2f -> %.2f.",
            descending ? "descent" : "ascent", pos.x(), pos.y(), pos.z(),
            target.z());
  return true;
}

bool FastExplorationManager::reanchorVerticalDetourForRelease(
    const Vector3d& pos, const Vector3d& yaw) {
  if (!vertical_detour_plan_) return false;
  return buildStationaryVerticalDetourPlan(pos, yaw);
}

void FastExplorationManager::completeVerticalDetourReference() {
  // 重试生成竖直段不能刷新首次终点后的跟踪宽限，避免旧会话无限续期。
  if (low_probe_phase_ == vertical_detour::LowProbePhase::DESCENDING &&
      low_probe_descent_end_time_.isZero())
    low_probe_descent_end_time_ = ros::Time::now();
}

void FastExplorationManager::cancelActiveLowProbe(const char* reason) {
  if (low_probe_phase_ == vertical_detour::LowProbePhase::IDLE) return;
  low_probe_phase_ = vertical_detour::LowProbePhase::IDLE;
  low_probe_confirmations_ = 0;
  low_probe_release_confirmations_ = 0;
  low_probe_advanced_distance_ = 0.0;
  suspended_underpass_active_ = false;
  suspended_underpass_blocked_ = false;
  low_probe_verify_start_ = ros::Time();
  low_probe_ascent_start_ = ros::Time();
  low_probe_descent_end_time_ = ros::Time();
  vertical_detour_plan_ = false;
  has_last_requested_goal_ = false;
  ROS_WARN("[vertical_detour] active low-probe target cancelled: %s.", reason);
}

bool FastExplorationManager::occupiedNearHeight(const Vector3d& point,
                                                double height) const {
  if (!sdf_map_) return false;
  const double horizontal_step = std::max(0.04, 0.5 * sdf_map_->getResolution());
  for (double dx : {-horizontal_step, 0.0, horizontal_step}) {
    for (double dy : {-horizontal_step, 0.0, horizontal_step}) {
      Vector3d probe(point.x() + dx, point.y() + dy, height);
      if (sdf_map_->isInMap(probe) &&
          sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED) {
        return true;
      }
    }
  }
  return false;
}

bool FastExplorationManager::hasCurrentHeightForwardObstacle(
    const Vector3d& pos, const Vector3d& forward) const {
  if (!sdf_map_ || forward.head<2>().norm() < 1e-3) return false;
  const Vector3d travel(forward.head<2>().normalized().x(),
                        forward.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double sample_step = std::max(0.05, sdf_map_->getResolution());
  const double max_distance = std::max(vertical_detour_forward_check_distance_,
                                       wide_side_bypass_max_lookahead_);
  for (double ahead = sample_step; ahead <= max_distance + 1e-6;
       ahead += sample_step) {
    for (double side : {-vertical_detour_footprint_radius_, 0.0,
                        vertical_detour_footprint_radius_}) {
      const Vector3d probe = pos + ahead * travel + side * lateral;
      if (cameraOccupancyColumn(probe, pos.z())) return true;
    }
  }
  return false;
}

bool FastExplorationManager::detectSuspendedObstacle(
    const Vector3d& pos, const Vector3d& forward, double& bottom_z,
    double& exit_progress, double& first_hit_distance) const {
  if (!sdf_map_ || forward.head<2>().norm() < 1e-3) return false;
  const Vector3d travel(forward.head<2>().normalized().x(),
                        forward.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double step = std::max(0.05, sdf_map_->getResolution());
  bool found = false;
  double last_hit = 0.0;
  int occupied_columns = 0;
  bottom_z = std::numeric_limits<double>::infinity();
  first_hit_distance = std::numeric_limits<double>::infinity();
  for (double ahead = step; ahead <= underpass_lookahead_ + 1e-6; ahead += step) {
    bool column_hit = false;
    for (double side = -vertical_detour_footprint_radius_;
         side <= vertical_detour_footprint_radius_ + 1e-6; side += step) {
      const Vector3d xy = pos + ahead * travel + side * lateral;
      for (double z = vertical_detour_low_height_ + 0.25;
           z <= pos.z() + underpass_clearance_ + 0.15 + 1e-6; z += step) {
        Vector3d probe(xy.x(), xy.y(), z);
        if (!sdf_map_->isInMap(probe) ||
            sdf_map_->getOccupancy(probe) != SDFMap::OCCUPIED)
          continue;
        // 只用该列最低占据层判断底部；下方已知自由才是悬空。
        // UNKNOWN/地图外不能作为没有落地支撑的证据，柱子低位缺图也不下穿。
        const double column_bottom = z - 0.5 * step;
        if (!cameraWallSupported(probe, travel.head<2>()) &&
            !hasLowVerticalSupport(probe, pos.z()) &&
            hasKnownFreeSpaceBelowObstacle(probe, column_bottom)) {
          column_hit = true;
          bottom_z = std::min(bottom_z, column_bottom);
        }
        break;
      }
    }
    if (column_hit) {
      if (!found) first_hit_distance = ahead;
      found = true;
      last_hit = ahead;
      ++occupied_columns;
    } else if (found && ahead - last_hit > 0.15) {
      break;
    }
  }
  if (occupied_columns < 2 ||
      bottom_z < vertical_detour_low_height_ + 0.25 ||
      bottom_z > pos.z() + underpass_lidar_above_odom_ +
                     underpass_clearance_ + underpass_height_tracking_margin_)
    return false;
  exit_progress = (pos + last_hit * travel).head<2>().dot(travel.head<2>()) +
                  vertical_detour_footprint_radius_ + step;
  return true;
}

bool FastExplorationManager::beginSuspendedUnderpass(const Vector3d& pos,
                                                      double yaw) {
  if (explorationFappCrossingActive()) return false;
  const bool manual = manual_path_enabled_ && manual_path_exploration_style_;
  if (manual_path_enabled_ && (!manual || manual_path_.finished() ||
      (manual_path_wait_for_trigger_ && !manual_path_started_)))
    return false;
  if (!vertical_detour_enabled_ || !task_search_manager_ ||
      !task_search_manager_->enabled() ||
      (!manual && !mission_entered_search_region_ &&
       !task_search_manager_->hasConfirmedCorridorTurn()) ||
      task_search_manager_->stage3Active() ||
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE ||
      turn_in_place_session_active_)
    return false;
  if (suspended_underpass_active_ || (manual && recovery_side_latched_)) return false;
  const Vector3d direction = task_search_manager_->recoveryForwardDirection(yaw);
  if (manual && (manual_path_.points[manual_path_.next] - pos).dot(direction) <= 0.15)
    return false;
  double bottom_z, exit_progress, first_hit_distance;
  const bool mapped = detectSuspendedObstacle(pos, direction, bottom_z,
                                               exit_progress, first_hit_distance);
  Vector3d lower, upper;
  const double lookahead = manual
      ? std::min(underpass_lookahead_,
                 (manual_path_.points[manual_path_.next] - pos).dot(direction))
      : underpass_lookahead_;
  const bool dynamic = manual && planner_manager_->fappBlockingBounds(
      {pos, pos + lookahead * direction}, lower, upper);
  double dynamic_target_z = std::numeric_limits<double>::infinity();
  if (dynamic) {
    const Vector3d center = 0.5 * (lower + upper);
    const Vector3d half = 0.5 * (upper - lower);
    const double extent = half.dot(direction.cwiseAbs());
    const double near_distance = (center - pos).dot(direction) - extent;
    const double far_progress = center.dot(direction) + extent + sdf_map_->getResolution();
    first_hit_distance = mapped ? std::min(first_hit_distance, near_distance) : near_distance;
    exit_progress = mapped ? std::max(exit_progress, far_progress) : far_progress;
    // 包围盒已扣除机体半高和动态净空，再留现有高度跟踪余量。
    dynamic_target_z = lower.z() - underpass_height_tracking_margin_;
    if (!mapped) bottom_z = std::numeric_limits<double>::infinity();
  }
  if (!mapped && !dynamic) {
    suspended_underpass_blocked_ = false;
    return false;
  }
  if (first_hit_distance > kUnderpassStartDistance && !dynamic) {
    suspended_underpass_blocked_ = false;
    ROS_INFO_THROTTLE(2.0, "[suspended_underpass] obstacle %.2fm ahead; "
                      "continue horizontal route until %.2fm.",
                      first_hit_distance, kUnderpassStartDistance);
    return false;
  }
  // 悬挂物下方已由检测确认自由；这里另外排除接近段的墙和低位支撑。
  const Vector3d travel(direction.head<2>().normalized().x(),
                        direction.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double step = std::max(0.05, sdf_map_->getResolution());
  for (double ahead = step; ahead < first_hit_distance - step; ahead += step) {
    for (double side : {-vertical_detour_footprint_radius_, 0.0,
                        vertical_detour_footprint_radius_}) {
      const Vector3d probe = pos + ahead * travel + side * lateral;
      if (cameraOccupancyColumn(probe, pos.z()) &&
          (cameraWallSupported(probe, travel.head<2>()) ||
           hasLowVerticalSupport(probe, pos.z())))
        return false;
    }
  }
  // 狭窄通道按雷达到悬挂障碍底部0.10m净空下穿，并额外留出高度跟踪余量。
  const double return_height = manual ? pos.z() : task_search_manager_->preferredSearchHeight();
  const double target_z = std::min(dynamic_target_z, vertical_detour::underpassTargetHeight(
      bottom_z, underpass_lidar_above_odom_, underpass_clearance_,
      underpass_height_tracking_margin_, return_height));
  // 下穿最低参考点Z使用独立参数，不将定位原点当作地面。
  const double minimum_height =
      std::max(underpass_min_height_, task_search_manager_->clampSearchHeight(-1e3));
  Vector3d low_start = pos;
  low_start.z() = std::min(target_z, pos.z());
  const double crossing_distance = std::max(step, exit_progress - pos.dot(direction));
  if (target_z < minimum_height ||
      (pos.z() > target_z + vertical_detour_height_tolerance_ &&
       !isKnownSafeVerticalPath(pos, target_z, !manual)) ||
      (manual && !isLowProbeCorridorSafe(low_start, direction, crossing_distance))) {
    suspended_underpass_blocked_ = true;
    ROS_ERROR_THROTTLE(1.0, "[suspended_underpass] confirmed obstacle (map=%d FAPP=%d) "
                       "but lower passage z=%.2f is unavailable; use side selection in waypoint mode.",
                       static_cast<int>(mapped), static_cast<int>(dynamic), target_z);
    return true;
  }
  suspended_underpass_blocked_ = false;
  suspended_underpass_active_ = true;
  low_probe_exit_progress_ = exit_progress;
  low_probe_height_ = std::min(target_z, pos.z());
  low_probe_direction_ = direction;
  const bool already_low = pos.z() <= target_z + vertical_detour_height_tolerance_;
  low_probe_phase_ = already_low ? vertical_detour::LowProbePhase::VERIFYING
                                 : vertical_detour::LowProbePhase::DESCENDING;
  low_probe_confirmations_ = 0;
  low_probe_release_confirmations_ = 0;
  low_probe_advanced_distance_ = 0.0;
  low_probe_origin_ = pos;
  low_probe_target_ = pos;
  low_probe_target_.z() = low_probe_height_;
  low_probe_return_height_ = return_height;
  low_probe_yaw_ = yaw;
  low_probe_verify_start_ = already_low ? ros::Time::now() : ros::Time();
  low_probe_descent_end_time_ = ros::Time();
  ROS_WARN("[suspended_underpass] ACTIVE map=%d FAPP=%d target_z=%.2f exit_progress=%.2f "
           "lidar_clearance=%.2f; %s before ordinary frontier planning.",
           static_cast<int>(mapped), static_cast<int>(dynamic),
           low_probe_height_, exit_progress, underpass_clearance_,
           already_low ? "verify low corridor" : "descend");
  return true;
}

bool FastExplorationManager::hasSuspendedForwardObstacle(
    const Vector3d& pos, const Vector3d& forward) const {
  double bottom_z, exit_progress, first_hit_distance;
  return detectSuspendedObstacle(pos, forward, bottom_z, exit_progress,
                                  first_hit_distance) &&
         first_hit_distance <= kUnderpassStartDistance;
}

bool FastExplorationManager::hasKnownFreeSpaceBelowObstacle(
    const Vector3d& point, double bottom_z) const {
  if (!sdf_map_) return false;
  const double step = std::max(0.05, sdf_map_->getResolution());
  const double low_height = std::max(vertical_detour_low_height_ + 0.05,
                                     wide_side_bypass_low_support_height_);
  int free_layers = 0;
  for (double height = low_height; height < bottom_z - 1e-6; height += step) {
    const Vector3d probe(point.x(), point.y(), height);
    if (!sdf_map_->isInMap(probe) ||
        sdf_map_->getOccupancy(probe) != SDFMap::FREE ||
        occupiedNearHeight(point, height))
      return false;
    ++free_layers;
  }
  return free_layers >= 2;
}

bool FastExplorationManager::hasLowVerticalSupport(
    const Vector3d& point, double current_height) const {
  if (!sdf_map_) return false;
  const double low_height = std::max(vertical_detour_low_height_ + 0.05,
                                     wide_side_bypass_low_support_height_);
  // 已经下降到低位时，当前层仍被挡就足以说明这不是只悬在头顶的障碍。
  if (current_height <= low_height + 0.12)
    return occupiedNearHeight(point, current_height);

  const double upper_limit = std::min(current_height - 0.12, low_height + 0.35);
  int consecutive_low_support = 0;
  for (double height = low_height; height <= upper_limit + 1e-6; height += 0.10) {
    Vector3d center(point.x(), point.y(), height);
    if (!sdf_map_->isInMap(center)) return false;
    const int occupancy = sdf_map_->getOccupancy(center);
    if (occupancy == SDFMap::UNKNOWN || !occupiedNearHeight(point, height))
      return false;
    ++consecutive_low_support;
    if (consecutive_low_support >= wide_side_bypass_min_vertical_support_layers_)
      return true;
  }
  // 没有连续支撑也可能只是低位缺图；调用方不能把 false 直接当成悬空。
  // 必须从最低检查层开始连续占据，不能把悬空球底部的两层占据算成落地支撑。
  return false;
}

bool FastExplorationManager::findForwardObstacleTop(
    const Vector3d& pos, const Vector3d& forward, double& obstacle_top) const {
  if (!sdf_map_ || forward.head<2>().norm() < 1e-3) return false;
  const Vector3d travel(forward.head<2>().normalized().x(),
                        forward.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double step = std::max(0.05, sdf_map_->getResolution());
  const double max_side_angle =
      std::min(89.0, std::max(0.0, vertical_detour_upper_max_side_angle_deg_)) *
      M_PI / 180.0;
  const double lateral_range =
      vertical_detour_upper_forward_max_distance_ * std::sin(max_side_angle) +
      vertical_detour_footprint_radius_;
  Vector3d map_origin;
  Vector3d map_size;
  sdf_map_->getRegion(map_origin, map_size);
  const double min_z = std::max(map_origin.z(), vertical_detour_low_height_);
  const double max_z = map_origin.z() + map_size.z() - 0.5 * step;

  bool found = false;
  obstacle_top = -std::numeric_limits<double>::infinity();
  for (double ahead = vertical_detour_forward_check_distance_;
       ahead <= vertical_detour_upper_forward_max_distance_ + 1e-6;
       ahead += step) {
    for (double side = -lateral_range; side <= lateral_range + 1e-6;
         side += step) {
      for (double z = min_z; z <= max_z + 1e-6; z += step) {
        const Vector3d probe = pos + ahead * travel + side * lateral +
                               Vector3d(0.0, 0.0, z - pos.z());
        // 连续通道墙不应决定“障碍物最高点”；这里只统计局部物体/门框，
        // 否则墙顶会把上绕高度推到地图上限。
        if (z >= pos.z() - 0.30 && cameraWallSupported(probe, travel.head<2>()))
          continue;
        if (sdf_map_->isInMap(probe) &&
            sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED) {
          obstacle_top = std::max(obstacle_top, z + 0.5 * step);
          found = true;
        }
      }
    }
  }
  return found;
}

bool FastExplorationManager::buildCenterGatePassage(
    const Vector3d& pos, double cur_yaw, const Vector3d& forward,
    Vector3d& next_pos, double& next_yaw, bool& gate_detected) {
  gate_detected = false;
  if (!center_gate_enabled_ || !sdf_map_ || !planner_manager_ ||
      forward.head<2>().norm() < 1e-3)
    return false;

  const Vector3d travel(forward.head<2>().normalized().x(),
                        forward.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double sample_step = std::max(0.05, sdf_map_->getResolution());
  const int side_samples = std::max(
      1, static_cast<int>(std::floor(wide_side_bypass_lateral_range_ /
                                    sample_step)));
  double gate_ahead = 0.0;
  double gate_center_offset = 0.0;
  double gate_width = 0.0;

  // 只把两侧局部物体识别为门框；通道两边连续墙体不能让该规则永久抢占
  // frontier。门中心净宽先做几何筛选，最终通行仍由已知自由足迹走廊确认。
  for (double ahead = wide_side_bypass_min_lookahead_;
       ahead <= wide_side_bypass_max_lookahead_ + 1e-6;
       ahead += sample_step) {
    bool left_found = false;
    bool right_found = false;
    double left_offset = 0.0;
    double right_offset = 0.0;
    for (int side_idx = 1; side_idx <= side_samples; ++side_idx) {
      const double offset = side_idx * sample_step;
      if (!left_found) {
        const Vector3d probe = pos + ahead * travel + offset * lateral;
        if (cameraOccupancyColumn(probe, pos.z()) &&
            !cameraWallSupported(probe, travel.head<2>())) {
          left_found = true;
          left_offset = offset;
        }
      }
      if (!right_found) {
        const Vector3d probe = pos + ahead * travel - offset * lateral;
        if (cameraOccupancyColumn(probe, pos.z()) &&
            !cameraWallSupported(probe, travel.head<2>())) {
          right_found = true;
          right_offset = -offset;
        }
      }
      if (left_found && right_found) break;
    }
    if (!left_found || !right_found) continue;

    gate_detected = true;
    const double width = left_offset - right_offset - sample_step;
    if (width < center_gate_min_width_) continue;
    gate_ahead = ahead;
    gate_center_offset = 0.5 * (left_offset + right_offset);
    gate_width = width;
  }

  if (!gate_detected || gate_ahead <= 0.0) {
    if (gate_detected) {
      ROS_WARN_THROTTLE(
          0.5,
          "[center_gate] two-sided obstacle detected, but center width is below %.2fm; "
          "try horizontal bypass or clearance-constrained upper passage.",
          center_gate_min_width_);
    }
    return false;
  }

  const double pass_distance = gate_ahead + center_gate_pass_margin_;
  Vector3d candidate = pos + pass_distance * travel + gate_center_offset * lateral;
  candidate.z() = horizontalGoalHeight(pos.z());
  const Vector3d direct_delta = candidate - pos;
  if (!pointInsideWorkspaceLock(candidate) ||
      !planner_manager_->isPositionSafe(candidate) ||
      sdf_map_->getOccupancy(candidate) != SDFMap::FREE ||
      !isKnownSafeHorizontalCorridor(pos, direct_delta, direct_delta.head<2>().norm()) ||
      (task_search_manager_ &&
       !task_search_manager_->isRecoveryCandidateUseful(candidate))) {
    ROS_WARN_THROTTLE(
        0.5,
        "[center_gate] center lane width=%.2fm at %.2fm, but the current-height "
        "footprint corridor to the far side is not fully known-safe; try horizontal or upper passage.",
        gate_width, gate_ahead);
    return false;
  }

  next_pos = candidate;
  next_yaw = cur_yaw;
  if (task_search_manager_) task_search_manager_->recordSelectedGoal(next_pos);
  ROS_WARN(
      "[center_gate] prefer known-free center passage width=%.2fm, gate=%.2fm ahead, "
      "center_offset=%.2fm, target=(%.2f,%.2f,%.2f), retain reference height.",
      gate_width, gate_ahead, gate_center_offset, next_pos.x(), next_pos.y(),
      next_pos.z());
  return true;
}

int FastExplorationManager::widthColumnState(const Vector3d& point) const {
  // Use voxel indices so an occupied layer cannot fall between height samples.
  // Same +/-6cm vehicle band for obstacle edges AND free-width measurement.
  if (!sdf_map_ || !point.allFinite()) return SDFMap::UNKNOWN;
  Eigen::Vector3i low, high;
  sdf_map_->posToIndex(point - Vector3d(0, 0, 0.06), low);
  sdf_map_->posToIndex(point + Vector3d(0, 0, 0.06), high);
  bool unknown = false;
  for (int z = low.z(); z <= high.z(); ++z) {
    const Eigen::Vector3i index(low.x(), low.y(), z);
    if (!sdf_map_->isInMap(index)) { unknown = true; continue; }
    const int state = sdf_map_->getOccupancy(index);
    if (state == SDFMap::OCCUPIED) return SDFMap::OCCUPIED;
    if (state != SDFMap::FREE) unknown = true;
  }
  return unknown ? SDFMap::UNKNOWN : SDFMap::FREE;
}

route_width::Score FastExplorationManager::evaluateRouteWidth(
    const vector<Vector3d>& path, const Vector3d& direction) const {
  if (path.size() < 2 || direction.head<2>().norm() < 1e-6) return {};
  const Vector3d forward(direction.x() / direction.head<2>().norm(),
                          direction.y() / direction.head<2>().norm(), 0.0);
  vector<Vector3d> lookahead{path.front()};
  for (size_t i = 1; i < path.size(); ++i) {
    const double a = (lookahead.back() - path.front()).dot(forward);
    const double b = (path[i] - path.front()).dot(forward);
    if (b >= route_width_horizon_ && b > a + 1e-6) {
      lookahead.push_back(lookahead.back() + (path[i] - lookahead.back()) *
                          ((route_width_horizon_ - a) / (b - a)));
      break;
    }
    lookahead.push_back(path[i]);
  }
  // A short lateral step needs a verified forward continuation before it can
  // compete with a complete route. Do not reward a wide but blind endpoint.
  const double remaining = route_width_horizon_ -
      (lookahead.back() - path.front()).dot(forward);
  if (remaining > 1e-6) lookahead.push_back(lookahead.back() + remaining * forward);
  if (!pathInsideWorkspaceLock(lookahead) ||
      !planner_manager_->isPathSafe(lookahead) ||
      (task_search_manager_ && !task_search_manager_->isRecoveryPathAllowed(lookahead, false)))
    return {};
  const double step = std::max(0.02, 0.5 * sdf_map_->getResolution());
  for (size_t i = 1; i < lookahead.size(); ++i) {
    if (!passage_centering::knownSegment(lookahead[i-1], lookahead[i], step,
            [&](const Vector3d& p) { return widthColumnState(p) == SDFMap::FREE; }))
      return {};
  }
  return route_width::measure(lookahead, forward, route_width_horizon_,
      wide_side_bypass_lateral_range_, sdf_map_->getResolution(),
      [&](const Vector3d& p) {
        return pointInsideWorkspaceLock(p) && widthColumnState(p) == SDFMap::FREE;
      });
}

bool FastExplorationManager::widthReplacementImproves(
    const vector<Vector3d>& current_path, const vector<Vector3d>& replacement,
    double yaw) const {
  if (!task_search_manager_) return false;
  const Vector3d forward = task_search_manager_->recoveryForwardDirection(yaw);
  return route_width::materiallyWider(evaluateRouteWidth(replacement, forward),
      evaluateRouteWidth(current_path, forward), route_width_replan_gain_);
}

bool FastExplorationManager::prepareWiderRoute(
    const vector<Vector3d>& current_path, double yaw) {
  pending_wider_route_.clear();
  // 动态穿越原侧由续接检查决定是否可行，不用宽度评分反复换边。
  if (manual_fapp_active_ && recovery_side_latched_) return false;
  // Entry owns the first metre after the doorway; open approach and entry
  // advancement are explicitly exempt from width reranking and early switching.
  if (!mission_workspace_lock_received_ || !entry_forward_complete_ ||
      current_path.size() < 2 || !task_search_manager_ ||
      !task_search_manager_->enabled() || task_search_manager_->stage3Active() ||
      task_search_manager_->finalExitFrontierGuardActive() ||
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE ||
      turn_in_place_session_active_ || ground_ascent_active_ ||
      (!mission_entered_search_region_ && !entry_forward_complete_)) return false;
  const Vector3d forward = task_search_manager_->recoveryForwardDirection(yaw).normalized();
  const Vector3d side(-forward.y(), forward.x(), 0.0);
  const Vector3d start = current_path.front();
  const auto current = evaluateRouteWidth(current_path, forward);
  if (!current.complete) return false;
  route_width::Score best = current;
  Eigen::Vector3d progress;
  planner_manager_->path_finder_->clearProgressConstraint();
  if (task_search_manager_->astarNoReturnDirection(progress))
    planner_manager_->path_finder_->setProgressConstraint(progress, 0.0);
  for (double offset : {0.0, -0.30, 0.30, -0.60, 0.60, -0.90, 0.90}) {
    Vector3d target = start + route_width_horizon_ * forward + offset * side;
    target.z() = current_path.back().z();
    if (!pointInsideWorkspaceLock(target) || !planner_manager_->isPositionSafe(target) ||
        widthColumnState(target) != SDFMap::FREE ||
        !task_search_manager_->isRecoveryCandidateUseful(target)) continue;
    // Include early lateral transitions as well as A*: its shortest path may
    // hug the obstacle corner even when a centred connected route is available.
    vector<vector<Vector3d>> paths{{start, target}};
    for (double advance : {0.0, 0.20, 0.40}) {
      Vector3d via = start + advance * forward + offset * side;
      via.z() = target.z();
      paths.push_back({start, via, target});
    }
    planner_manager_->path_finder_->reset();
    if (planner_manager_->path_finder_->search(start, target) == Astar::REACH_END) {
      auto path = planner_manager_->path_finder_->getPath();
      if (!path.empty()) {
        if ((path.front() - start).norm() > 1e-6) path.insert(path.begin(), start);
        if ((path.back() - target).norm() > 1e-6) path.push_back(target);
        paths.push_back(path);
      }
    }
    for (const auto& path : paths) {
      const auto score = evaluateRouteWidth(path, forward);
      if (route_width::materiallyWider(score, current, route_width_replan_gain_) &&
          route_width::prefer(score, best)) {
        best = score;
        pending_wider_route_ = path;
      }
    }
  }
  if (pending_wider_route_.empty()) return false;
  ROS_WARN("[route_width] route alternative: width %.2f -> %.2fm, mean %.2f -> %.2fm, "
           "clearance %.2f -> %.2fm, horizon=%.2fm target=(%.2f,%.2f,%.2f).",
           current.minimum, best.minimum, current.mean, best.mean, current.clearance, best.clearance,
           route_width_horizon_, pending_wider_route_.back().x(),
           pending_wider_route_.back().y(), pending_wider_route_.back().z());
  return true;
}

FastExplorationManager::SplitEvidence FastExplorationManager::findWideSideSplit(
    const Vector3d& pos, const Vector3d& forward, bool entry_exempt) const {
  const bool width_priority = entry_forward_complete_ && !entry_exempt;
  SplitEvidence split;
  if (!wide_side_bypass_enabled_ || !sdf_map_ || forward.head<2>().norm() < 1e-3)
    return split;
  const Vector3d travel(forward.head<2>().normalized().x(),
                        forward.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double sample_step = std::max(0.05, sdf_map_->getResolution());
  const int side_samples = std::max(
      1, static_cast<int>(std::floor(wide_side_bypass_lateral_range_ / sample_step)));

  auto knownFree = [&](double forward_distance, double side_offset) {
    const Vector3d probe = pos + forward_distance * travel + side_offset * lateral;
    return width_priority ? widthColumnState(probe) == SDFMap::FREE
                          : sdf_map_->isInMap(probe) && sdf_map_->getOccupancy(probe) == SDFMap::FREE;
  };

  // 找第一条包含“非连续墙占据”的横截面。墙本身只作为左右边界，柱子/箱体
  // 等局部占据才视为把通道分成两条可选通路的障碍物。
  const double max_lookahead = width_priority
      ? std::max(wide_side_bypass_max_lookahead_, route_width_horizon_)
      : wide_side_bypass_max_lookahead_;
  for (double ahead = wide_side_bypass_min_lookahead_;
       ahead <= max_lookahead + 1e-6;
       ahead += sample_step) {
    int obstacle_min_idx = side_samples + 1;
    int obstacle_max_idx = -side_samples - 1;
    bool blocks_forward_footprint = false;
    for (int side_idx = -side_samples; side_idx <= side_samples; ++side_idx) {
      const double offset = side_idx * sample_step;
      Vector3d probe = pos + ahead * travel + offset * lateral;
      if ((width_priority ? widthColumnState(probe) != SDFMap::OCCUPIED
                          : !cameraOccupancyColumn(probe, pos.z())) ||
          cameraWallSupported(probe, travel.head<2>()))
        continue;
      // 低位支撑只决定是否可以从障碍下方穿过，不能阻止普通的水平绕障。
      // 如果下方通道尚未确认安全，先把障碍作为当前高度的竖直障碍，
      // 交给左右分流；buildMissionForwardFallback() 会在此之前优先尝试已确认的下穿。
      if (!hasLowVerticalSupport(probe, pos.z())) split.suspended_obstacle_seen = true;
      if (width_priority && std::fabs(offset) > vertical_detour_footprint_radius_ + 0.5 * sample_step)
        continue;
      obstacle_min_idx = std::min(obstacle_min_idx, side_idx);
      obstacle_max_idx = std::max(obstacle_max_idx, side_idx);
      // 横截面两侧的占据不是前方挡路证据；至少一个实际占据栅格必须
      // 与前进机体包络相交，不能把两侧占据的外接区间当作中心障碍。
      if (std::fabs(offset) <= vertical_detour_footprint_radius_ +
                                  0.5 * sample_step)
        blocks_forward_footprint = true;
    }
    if (obstacle_max_idx < obstacle_min_idx || !blocks_forward_footprint) continue;

    while (width_priority && obstacle_min_idx > -side_samples && widthColumnState(
        pos + ahead * travel + (obstacle_min_idx - 1) * sample_step * lateral) == SDFMap::OCCUPIED)
      --obstacle_min_idx;
    while (width_priority && obstacle_max_idx < side_samples && widthColumnState(
        pos + ahead * travel + (obstacle_max_idx + 1) * sample_step * lateral) == SDFMap::OCCUPIED)
      ++obstacle_max_idx;
    split.valid = true;
    split.forward_distance = ahead;
    split.obstacle_min_offset = obstacle_min_idx * sample_step;
    split.obstacle_max_offset = obstacle_max_idx * sample_step;

    // 从障碍物边缘分别向左右数连续的已知自由栅格。未知区域不增加宽度，
    // 但也不会直接否决另一侧；最终能否飞仍由统一A*和足迹检查决定。
    // lateral 指向行进方向的几何左侧：正偏移是左，负偏移是右。
    int right_count = 0;
    for (int idx = obstacle_min_idx - 1; idx >= -side_samples; --idx) {
      if (!knownFree(ahead, idx * sample_step)) break;
      ++right_count;
    }
    int left_count = 0;
    for (int idx = obstacle_max_idx + 1; idx <= side_samples; ++idx) {
      if (!knownFree(ahead, idx * sample_step)) break;
      ++left_count;
    }
    split.left_width = left_count * sample_step;
    split.right_width = right_count * sample_step;
    split.left_free_min = split.obstacle_max_offset + sample_step;
    split.left_free_max = split.left_free_min +
                          std::max(0, left_count - 1) * sample_step;
    split.right_free_max = split.obstacle_min_offset - sample_step;
    split.right_free_min = split.right_free_max -
                           std::max(0, right_count - 1) * sample_step;
    break;
  }

  return split;
}

bool FastExplorationManager::buildWideSideBypass(
    const Vector3d& pos, double cur_yaw, const Vector3d& forward,
    Vector3d& next_pos, double& next_yaw, bool& split_obstacle_detected) {
  split_obstacle_detected = false;
  if (!wide_side_bypass_enabled_ || !sdf_map_ || !planner_manager_ ||
      forward.head<2>().norm() < 1e-3)
    return false;
  // 普通规划与宽侧模块选出的绕行都先续接原侧，不能因另一侧测得更宽就换边。
  // 通过障碍或原侧短路不可行时，continuation 释放侧向，本轮才重新比较宽度。
  if (recovery_side_latched_ &&
      buildSideRecoveryContinuation(pos, cur_yaw, next_pos, next_yaw)) {
    split_obstacle_detected = true;
    return true;
  }
  wide_side_bypass_active_ = false;
  const Vector3d travel(forward.head<2>().normalized().x(),
                        forward.head<2>().normalized().y(), 0.0);
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const SplitEvidence split = findWideSideSplit(pos, forward);
  split_obstacle_detected = split.valid;
  if (!split.valid && split.suspended_obstacle_seen) {
    ROS_INFO_THROTTLE(
        0.5,
        "[wide_side_bypass] low support is unconfirmed; keep horizontal lane selection "
        "unless free space below the obstacle confirms an underpass.");
  }
  const double widest_lane = std::max(split.left_width, split.right_width);
  // 立柱前即使还没有完整的0.25m侧向通道，也先允许一个0.20m的安全拉出，
  // 给足迹和下一帧深度图留出观察位置；只有小于0.20m才交给上下绕障。
  const bool narrow_pullout = split.valid && widest_lane >= 0.20;
  if (!split.valid || (!narrow_pullout && widest_lane < wide_side_bypass_min_lane_width_))
    return false;

  struct BypassChoice {
    bool valid{false};
    Vector3d target{0.0, 0.0, 0.0};
    double endpoint_clearance{-1.0};
    double path_clearance{-1.0};
    double lateral_offset{0.0};
    double forward_step{0.0};
    double lane_width{0.0};
    double centre_error{0.0};
    route_width::Score width;
  } best;

  auto minimumPathClearance = [&](const vector<Vector3d>& path) {
    double clearance = std::numeric_limits<double>::infinity();
    for (size_t segment = 1; segment < path.size(); ++segment) {
      const Vector3d delta = path[segment] - path[segment - 1];
      const int samples =
          std::max(1, static_cast<int>(std::ceil(delta.norm() / 0.05)));
      for (int sample = 0; sample <= samples; ++sample) {
        const Vector3d point = path[segment - 1] +
            delta * static_cast<double>(sample) / static_cast<double>(samples);
        clearance = std::min(clearance, sdf_map_->getDistance(point));
      }
    }
    return std::isfinite(clearance) ? clearance : -1.0;
  };

  // 先横移到较宽通路的中部，并只附带最多0.20m前进量；不要求一次轨迹
  // 穿完整个障碍，下一轮地图更新后会继续沿宽侧推进。
  vector<double> forward_steps{
      std::min(wide_side_bypass_forward_step_,
               std::max(0.0, split.forward_distance - 0.10)),
      0.10, 0.0};
  const double minimum_shift = narrow_pullout ? 0.20 : 0.15;
  // 首次选侧先比较通路间距；同侧优先采用可行的前进量，再比较净空。
  // 纯横移只在前进候选不可用时采用，避免净空评分持续选中零前进量。
  const double remembered_sign = recovery_side_latched_ &&
      recovery_side_dir_.dot(lateral) < -0.5 ? -1.0 : 1.0;
  const bool keep_dynamic_side = manual_fapp_active_ && recovery_side_latched_ &&
      std::fabs(recovery_side_dir_.dot(lateral)) > 0.5;
  const double first_sign = keep_dynamic_side ? remembered_sign :
      (split.right_width > split.left_width + 1e-6 ? -1.0 :
       (split.left_width > split.right_width + 1e-6 ? 1.0 : remembered_sign));
  for (const double side_sign : {first_sign, -first_sign}) {
    if (keep_dynamic_side && side_sign != first_sign) break;
    const bool right_side = side_sign < 0.0;
    const double lane_width = right_side ? split.right_width : split.left_width;
    if (lane_width < wide_side_bypass_min_lane_width_ && lane_width < 0.20) continue;
    const double lane_min = right_side ? split.right_free_min : split.left_free_min;
    const double lane_max = right_side ? split.right_free_max : split.left_free_max;
    const double lane_center = 0.5 * (lane_min + lane_max);
    const double max_shift = std::min(wide_side_bypass_max_lateral_step_,
                                      std::fabs(lane_center));
    for (double shift = max_shift; shift >= minimum_shift - 1e-6; shift -= 0.05) {
      const double lateral_offset = side_sign * shift;
      for (const double forward_step : forward_steps) {
        Vector3d candidate = pos + forward_step * travel + lateral_offset * lateral;
        // 继承已发布目标而非偏低实测值，避免连续侧绕逐段降低高度。
        candidate.z() = horizontalGoalHeight(pos.z());
        if (!pointInsideWorkspaceLock(candidate) ||
            !planner_manager_->isPositionSafe(candidate) ||
            sdf_map_->getOccupancy(candidate) == SDFMap::UNKNOWN ||
            (task_search_manager_ &&
             !task_search_manager_->isRecoveryCandidateUseful(candidate)))
          continue;

        planner_manager_->path_finder_->reset();
        if (planner_manager_->path_finder_->search(pos, candidate) != Astar::REACH_END)
          continue;
        const auto path = planner_manager_->path_finder_->getPath();
        if (!pathInsideWorkspaceLock(path) || !planner_manager_->isPathSafe(path) ||
            (task_search_manager_ &&
             !task_search_manager_->isRecoveryPathAllowed(path, false)))
          continue;

        const double path_clearance = minimumPathClearance(path);
        const double endpoint_clearance = sdf_map_->getDistance(candidate);
        const double centre_error = passage_centering_enabled_
            ? std::fabs(lateral_offset - lane_center) : 0.0;
        const auto width = entry_forward_complete_ ? evaluateRouteWidth(path, travel)
                                                   : route_width::Score{};
        const bool comparable = width.complete == best.width.complete;
        const bool width_tied = comparable &&
            !route_width::prefer(width, best.width) && !route_width::prefer(best.width, width);
        if (!best.valid || route_width::prefer(width, best.width) ||
            (width_tied && (lane_width > best.lane_width + 1e-6 ||
            (std::fabs(lane_width - best.lane_width) <= 1e-6 &&
             (forward_step > best.forward_step + 1e-6 ||
              (std::fabs(forward_step - best.forward_step) <= 1e-6 &&
               (centre_error < best.centre_error - 1e-6 ||
                (std::fabs(centre_error - best.centre_error) <= 1e-6 &&
                 (path_clearance > best.path_clearance + 0.01 ||
                  (std::fabs(path_clearance - best.path_clearance) <= 0.01 &&
                   endpoint_clearance > best.endpoint_clearance + 0.01)))))))))) {
          best.width = width;
          best.valid = true;
          best.target = candidate;
          best.endpoint_clearance = endpoint_clearance;
          best.path_clearance = path_clearance;
          best.lateral_offset = lateral_offset;
          best.forward_step = forward_step;
          best.lane_width = lane_width;
          best.centre_error = centre_error;
        }
      }
    }
    // 动态场景保持原侧，等球让开后前进；普通静态绕障仍可比较另一侧。
    if (keep_dynamic_side && best.valid) break;
  }

  if (!best.valid) {
    ROS_WARN_THROTTLE(
        0.5,
        "[wide_side_bypass] split obstacle at %.2fm; widths left=%.2fm right=%.2fm, "
        "neither side has a safe horizontal step yet.",
        split.forward_distance, split.left_width, split.right_width);
    return false;
  }

  next_pos = best.target;
  next_yaw = cur_yaw;
  recovery_side_latched_ = true;
  recovery_side_origin_ = pos;
  wide_side_bypass_active_ = true;
  // 从首次选侧的位置累计通过障碍截面及机体半径，不能每次重规划重置进度。
  wide_side_bypass_release_distance_ = split.forward_distance +
      vertical_detour_footprint_radius_ + sdf_map_->getResolution();
  const bool choose_right = best.lateral_offset < 0.0;
  recovery_side_dir_ = (choose_right ? -1.0 : 1.0) * lateral;
  if (task_search_manager_) task_search_manager_->recordSelectedGoal(next_pos);
  ROS_WARN(
      "[wide_side_bypass] split obstacle at %.2fm, widths left=%.2fm right=%.2fm; "
      "choose %s lane (%.2fm vs %.2fm), horizontal shift=%.2fm forward=%.2fm "
      "target=(%.2f,%.2f,%.2f), path_clearance=%.2fm, keep yaw=%.1fdeg.",
      split.forward_distance, split.left_width, split.right_width,
      choose_right ? "right" : "left", best.lane_width,
      choose_right ? split.left_width : split.right_width,
      std::fabs(best.lateral_offset), best.forward_step, next_pos.x(), next_pos.y(),
      next_pos.z(), best.path_clearance, next_yaw * 180.0 / M_PI);
  return true;
}

bool FastExplorationManager::buildFappSideBypass(
    const Vector3d& pos, double cur_yaw, const Vector3d& forward,
    Vector3d& next_pos, double& next_yaw, bool& obstacle_detected,
    std::vector<Vector3d>* crossing_path) {
  obstacle_detected = false;
  const bool manual = manual_path_enabled_ && manual_path_exploration_style_;
  if ((!manual && !fapp_exploration_crossing_enabled_) ||
      !wide_side_bypass_enabled_ || !planner_manager_ || !sdf_map_ ||
      forward.head<2>().norm() < 1e-3 || (manual && manual_path_.finished()))
    return false;
  const Vector3d travel = forward.normalized();
  const Vector3d lateral(-travel.y(), travel.x(), 0.0);
  const double horizon = manual ? std::min(underpass_lookahead_,
      (manual_path_.points[manual_path_.next] - pos).dot(travel)) : underpass_lookahead_;
  if (horizon <= 0.15) return false;
  Vector3d lower, upper;
  double age;
  // 已经横移到侧面时前向线可能不再撞框，仍需沿锁定历史范围完成穿越。
  obstacle_detected = crossing_path &&
      planner_manager_->watchedFappBounds(lower, upper, age);
  if (!obstacle_detected)
    obstacle_detected = planner_manager_->fappBlockingBounds(
        {pos, pos + horizon * travel}, lower, upper);
  if (!obstacle_detected) return false;
  const Vector3d center = 0.5 * (lower + upper);
  const Vector3d half = 0.5 * (upper - lower);
  const double margin = sdf_map_->getResolution();
  const double pass_distance = (center - pos).dot(travel) +
      half.dot(travel.cwiseAbs()) + (crossing_path ? std::max(0.15, margin) : margin);
  if (crossing_path && pass_distance <= 0.03) return false;
  const double side_extent = half.dot(lateral.cwiseAbs()) + margin;
  bool found = false;
  double best_clearance = -1.0;
  double best_shift = 0.0;
  double best_sign = 0.0;
  vector<Vector3d> best_route;
  const bool keep_side = manual_fapp_active_ && recovery_side_latched_ &&
      std::fabs(recovery_side_dir_.dot(lateral)) > 0.5;
  const double first_sign = keep_side && recovery_side_dir_.dot(lateral) < 0.0 ? -1.0 : 1.0;
  for (double sign : {first_sign, -first_sign}) {
    if (keep_side && sign != first_sign) break;
    double shift = (center - pos).dot(lateral) + sign * side_extent;
    if (crossing_path) shift = sign * std::max(0.0, sign * shift);
    if ((!crossing_path && sign * shift <= 0.0) ||
        std::fabs(shift) > wide_side_bypass_lateral_range_)
      continue;
    const double approach = crossing_path ? std::max(0.0,
        (center - pos).dot(travel) - half.dot(travel.cwiseAbs()) - margin) : 0.0;
    Vector3d side = pos + approach * travel + shift * lateral;
    Vector3d passed = pos + shift * lateral + std::max(0.0, pass_distance) * travel;
    if (crossing_path) {
      // 延用已发布目标高度，不能把重规划时的实际掉高逐次写成新目标。
      passed.z() = horizontalGoalHeight(pos.z());
      side.z() += (passed.z() - pos.z()) * approach / pass_distance;
    }
    vector<Vector3d> route{pos};
    if ((side - pos).norm() > 1e-3) route.push_back(side);
    route.push_back(passed);
    if (route.size() == 2) route.insert(route.begin() + 1, 0.5 * (pos + passed));
    // 完整穿越直接保留到包络后方的路径；普通恢复调用仍沿用原短步。
    if (((side - pos).norm() > 1e-3 &&
         !isLowProbeCorridorSafe(pos, side - pos, (side - pos).head<2>().norm())) ||
        !isLowProbeCorridorSafe(side, passed - side,
            std::max(margin, (passed - side).head<2>().norm())) ||
        !pathInsideWorkspaceLock(route) || !planner_manager_->isFappPathSafe(route))
      continue;
    double clearance = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < route.size(); ++i) {
      const int samples = std::max(1, static_cast<int>(std::ceil(
          (route[i] - route[i - 1]).norm() / margin)));
      for (int j = 0; j <= samples; ++j) {
        const Vector3d point = route[i - 1] +
            static_cast<double>(j) / samples * (route[i] - route[i - 1]);
        clearance = std::min(clearance, sdf_map_->getDistance(point));
      }
    }
    if (!found || clearance > best_clearance + 0.01 ||
        (std::fabs(clearance - best_clearance) <= 0.01 &&
         std::fabs(shift) < std::fabs(best_shift))) {
      found = true;
      best_clearance = clearance;
      best_shift = shift;
      best_sign = sign;
      best_route = route;
    }
    if (keep_side && found) break;
  }
  if (!found) return false;
  const double sign = best_sign;
  next_pos = crossing_path ? best_route.back() : Vector3d(pos + sign *
      std::min(std::fabs(best_shift), wide_side_bypass_max_lateral_step_) * lateral);
  if (crossing_path) *crossing_path = best_route;
  next_yaw = cur_yaw;
  recovery_side_latched_ = true;
  recovery_side_origin_ = pos;
  recovery_side_dir_ = sign * lateral;
  wide_side_bypass_active_ = true;
  wide_side_bypass_release_distance_ = pass_distance;
  if (crossing_path)
    ROS_WARN("[exploration_fapp] complete %s side path: forward=%.2fm lateral=%.2fm",
        sign > 0.0 ? "left" : "right", pass_distance, best_shift);
  else ROS_WARN("[manual_path] FAPP lower passage unavailable; choose %s side, "
           "translate %.2fm then pass %.2fm before returning to waypoint.",
           sign > 0.0 ? "left" : "right", (next_pos - pos).norm(), pass_distance);
  return true;
}

bool FastExplorationManager::selectEntryForwardGoal(
    const Vector3d& pos, const Vector3d& forward, double remaining,
    entry_forward::Goal& selected) {
  const Vector3d lateral(-forward.y(), forward.x(), 0.0);
  const double sample_step = std::max(0.05, sdf_map_->getResolution());
  const SplitEvidence split = findWideSideSplit(pos, forward, true);
  const auto evaluate = [&](entry_forward::Goal& goal) {
    goal.position.z() = horizontalGoalHeight(pos.z());
    const Vector3d& candidate = goal.position;
    double& clearance = goal.clearance;
    double& continuation = goal.continuation;
    const Vector3d delta = candidate - pos;
    if (!pointInsideWorkspaceLock(candidate) ||
        !isLowProbeCorridorSafe(pos, delta, delta.head<2>().norm()) ||
        !task_search_manager_->isTaskMotionAllowed(candidate))
      return false;
    clearance = std::max(0.0, sdf_map_->getDistance(candidate));
    continuation = 0.0;
    for (double angle : {0.0, -M_PI / 6.0, M_PI / 6.0, -M_PI_4, M_PI_4}) {
      const Vector3d direction(
          forward.x() * std::cos(angle) - forward.y() * std::sin(angle),
          forward.x() * std::sin(angle) + forward.y() * std::cos(angle), 0.0);
      for (double distance : {0.40, 0.20}) {
        if (distance <= continuation + 1e-6) continue;
        const Vector3d next = candidate + distance * direction;
        if (pointInsideWorkspaceLock(next) &&
            task_search_manager_->isTaskMotionAllowed(next) &&
            isLowProbeCorridorSafe(candidate, direction, distance))
          continuation = distance;
      }
      if (continuation >= 0.40) break;
    }
    // 跨过已观测的阻挡截面后仍有足迹可行通路，才提升排序。
    // 第一段不能先走到通路反侧，第二段也不能折返抵消已做的侧移。
    if (split.valid) {
      for (double side : {1.0, -1.0}) {
        const double width = side > 0.0 ? split.left_width : split.right_width;
        if (width < 0.20) continue;
        const double offset = side > 0.0
            ? 0.5 * (split.left_free_min + split.left_free_max)
            : 0.5 * (split.right_free_min + split.right_free_max);
        if (goal.side * offset < -1e-6 ||
            (std::fabs(goal.side) > 1e-3 &&
             goal.side * (offset - goal.side) < -std::fabs(goal.side) * sample_step))
          continue;
        const double crossing_progress = split.forward_distance +
            std::max(0.15, 2.0 * sample_step);
        Vector3d gate = pos + crossing_progress * forward + offset * lateral;
        gate.z() = candidate.z();
        const Vector3d delta = gate - candidate;
        if (delta.dot(forward) < -1e-6 || !pointInsideWorkspaceLock(gate) ||
            !task_search_manager_->isTaskMotionAllowed(gate) ||
            !isLowProbeCorridorSafe(candidate, delta, delta.head<2>().norm()))
          continue;
        const int priority = width + 1e-6 >=
            std::max(split.left_width, split.right_width) ? 2 : 1;
        if (priority > goal.passage_priority) {
          goal.passage_priority = priority;
          goal.passage_side = side;
        }
      }
    }
    return true;
  };
  if (!entry_forward::selectGoal(pos, forward, remaining, evaluate, selected))
    return false;
  if (selected.passage_priority > 0) {
    wide_side_bypass_active_ = false;
    if (!recovery_side_latched_ || recovery_side_dir_.dot(lateral) * selected.passage_side < 0.5)
      recovery_side_origin_ = pos;
    recovery_side_latched_ = true;
    recovery_side_dir_ = selected.passage_side * lateral;
  } else if (split.valid) {
    recovery_side_latched_ = false;
  }
  if (split.valid)
    ROS_WARN("[entry_forward] obstacle %.2fm ahead, lanes left=%.2fm right=%.2fm; "
             "connected passage=%s priority=%d.", split.forward_distance,
             split.left_width, split.right_width,
             selected.passage_side > 0.0 ? "left" :
                 (selected.passage_side < 0.0 ? "right" : "unconfirmed"),
             selected.passage_priority);
  return true;
}

void FastExplorationManager::rememberSideRecovery(
    const Vector3d& pos, double cur_yaw, const vector<Vector3d>& path) {
  if (recovery_side_latched_ || !task_search_manager_ ||
      !ep_->mission_use_forward_fallback_) return;
  const Vector3d forward = task_search_manager_->recoveryForwardDirection(cur_yaw);
  if (!hasCurrentHeightForwardObstacle(pos, forward)) return;
  const Vector3d lateral(-forward.y(), forward.x(), 0.0);
  // 记住已通过检查的近段绕行侧，不用远端frontier的位置代替实际绕柱方向。
  double length = 0.0;
  Vector3d previous = pos;
  for (const auto& point : path) {
    const Vector3d delta = point - previous;
    const double remaining = 0.60 - length;
    const Vector3d probe = delta.norm() > remaining
                              ? Vector3d(previous + delta.normalized() * remaining)
                              : point;
    const double side = (probe - pos).dot(lateral);
    if (std::fabs(side) >= 0.10) {
      wide_side_bypass_active_ = false;
      recovery_side_latched_ = true;
      recovery_side_origin_ = pos;
      recovery_side_dir_ = (side > 0.0 ? 1.0 : -1.0) * lateral;
      return;
    }
    length += delta.norm();
    if (length >= 0.60) break;
    previous = point;
  }
}

bool FastExplorationManager::buildSideRecoveryContinuation(
    const Vector3d& pos, double cur_yaw, Vector3d& next_pos, double& next_yaw) {
  if (!recovery_side_latched_ || !task_search_manager_ || !sdf_map_ ||
      !planner_manager_ || !ep_->mission_use_forward_fallback_ ||
      task_search_manager_->stage3Active() ||
      task_search_manager_->finalExitFrontierGuardActive() ||
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE)
    return false;
  const Vector3d forward =
      task_search_manager_->recoveryForwardDirection(cur_yaw).normalized();
  const Vector3d lateral(-forward.y(), forward.x(), 0.0);
  const double side = recovery_side_dir_.dot(lateral);
  // 确认新通道后旧侧向不再有效，不让旧绕柱偏好约束正常转弯。
  if (std::fabs(side) < 0.5) {
    if (manual_fapp_active_) return false;
    recovery_side_latched_ = false;
    wide_side_bypass_active_ = false;
    return false;
  }
  const double release_distance = wide_side_bypass_active_
      ? wide_side_bypass_release_distance_ : recovery_side_release_distance_;
  const double remaining = std::max(0.40,
      release_distance - (pos - recovery_side_origin_).dot(forward));
  const bool manual_side = ((manual_path_enabled_ && manual_path_exploration_style_) ||
      explorationFappCrossingActive()) &&
      wide_side_bypass_active_;
  // 普通规划优先模式仍交给动态代价和最终轨迹检查；历史整段包络不能
  // 单独把已经可行的同侧前进改成继续往墙上横移。
  const bool forward_clear = isLowProbeCorridorSafe(pos, forward, 0.40) &&
      (!manual_side || (manual_fapp_active_ && planner_manager_->fappPlannerPriority()) ||
       planner_manager_->isFappPathSafe({pos, pos + remaining * forward}));
  if (!manual_fapp_active_ && forward_clear && (pos - recovery_side_origin_).dot(forward) >=
                           release_distance) {
    recovery_side_latched_ = false;
    wide_side_bypass_active_ = false;
    ROS_WARN("[side_recovery] forward passage clear after %.2fm advance; "
             "resume normal selection.", release_distance);
    return false;
  }
  const Vector3d direction = forward_clear
      ? forward : Vector3d((side > 0.0 ? 1.0 : -1.0) * lateral);
  for (double distance : {0.40, 0.30, 0.20}) {
    Vector3d candidate = pos + distance * direction;
    candidate.z() = horizontalGoalHeight(pos.z());
    const auto usable = [&](const Vector3d& point) {
      const vector<Vector3d> path{pos, point};
      return pointInsideWorkspaceLock(point) &&
          task_search_manager_->isRecoveryCandidateUseful(point) &&
          isLowProbeCorridorSafe(pos, point - pos, (point - pos).head<2>().norm()) &&
          pathInsideWorkspaceLock(path) &&
          task_search_manager_->isRecoveryPathAllowed(path, false);
    };
    if (passage_centering_enabled_ && forward_clear && !manual_side && !manual_fapp_active_) {
      passage_centering::Config cfg;
      cfg.step = std::min(0.05, 0.5 * sdf_map_->getResolution());
      cfg.search_radius = passage_centering_search_radius_;
      const Vector3d centred = passage_centering::sectionCentre(
          candidate, forward, std::min(passage_centering_maximum_shift_, 0.6 * distance),
          cfg, [&](const Vector3d& p) {
            return pointInsideWorkspaceLock(p) && planner_manager_->isKnownFreePosition(p);
          });
      if (usable(centred)) candidate = centred;
    }
    if (!usable(candidate)) continue;
    next_pos = candidate;
    next_yaw = cur_yaw;
    recovery_side_continuation_pending_ = true;
    task_search_manager_->recordSelectedGoal(next_pos);
    ROS_WARN("[side_recovery] keep %s side; %s %.2fm from current z=%.2fm "
             "target=(%.2f,%.2f,%.2f).",
             side > 0.0 ? "left" : "right", forward_clear ? "advance" : "translate",
             distance, pos.z(), next_pos.x(), next_pos.y(), next_pos.z());
    return true;
  }
  // 只是先尝试原侧；同侧短路不可行时本轮直接交还原有换边/上下绕行。
  if (manual_fapp_active_) return false;
  recovery_side_latched_ = false;
  wide_side_bypass_active_ = false;
  ROS_WARN("[side_recovery] continuation unavailable; yield to existing recovery.");
  return false;
}

bool FastExplorationManager::buildMissionForwardFallback(const Vector3d& pos, double cur_yaw,
                                                         Vector3d& next_pos,
                                                         double& next_yaw) {
  if (!ep_->mission_use_forward_fallback_ || !ep_->mission_use_workspace_lock_ ||
      !mission_workspace_lock_received_) {
    return false;
  }
  const Vector3d recovery_forward = task_search_manager_
                                        ? task_search_manager_->recoveryForwardDirection(cur_yaw)
                                        : Vector3d(std::cos(cur_yaw), std::sin(cur_yaw), 0.0);
  bool current_split_detected = false;
  if (task_search_manager_ && !task_search_manager_->stage3Active() &&
      !task_search_manager_->finalExitFrontierGuardActive() &&
      low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
      buildWideSideBypass(pos, cur_yaw, recovery_forward, next_pos, next_yaw,
                          current_split_detected))
    return true;
  if (buildSideRecoveryContinuation(pos, cur_yaw, next_pos, next_yaw)) return true;

  const double step = std::max(0.2, ep_->mission_forward_fallback_step_);
  const double max_dist = std::max(step, ep_->mission_forward_fallback_max_);
  // 2026-07-13: 在通道深处按实际运动方向向前、侧向和转弯方向尝试，解决固定门方向在折线通道末端卡死。
  auto directions = task_search_manager_
                              ? task_search_manager_->recoveryDirections(cur_yaw)
                              : vector<Vector3d>{mission_workspace_dir_};
  // 2026-07-28: 所有前向方向仍保持任务层顺序；短回撤候选改按目标点ESDF净空降序，
  // 让无人机优先退向通道中部，而不是因为±135度枚举顺序固定地退向某一侧墙。
  std::stable_sort(directions.begin(), directions.end(), [&](const Vector3d& lhs,
                                                              const Vector3d& rhs) {
    const bool lhs_back = task_search_manager_ &&
        task_search_manager_->isRecoveryDirectionBackward(lhs, cur_yaw);
    const bool rhs_back = task_search_manager_ &&
        task_search_manager_->isRecoveryDirectionBackward(rhs, cur_yaw);
    if (lhs_back != rhs_back) return !lhs_back;
    const Vector3d forward = recovery_forward.normalized();
    const bool lhs_straight = lhs.normalized().dot(forward) >= std::cos(10.0 * M_PI / 180.0);
    const bool rhs_straight = rhs.normalized().dot(forward) >= std::cos(10.0 * M_PI / 180.0);
    if (lhs_straight != rhs_straight) return lhs_straight;
    if (!lhs_back && recovery_side_latched_ && !lhs_straight) {
      return lhs.normalized().dot(recovery_side_dir_) >
             rhs.normalized().dot(recovery_side_dir_);
    }
    if (!lhs_back) return false;
    const double probe_dist = short_backtrack_max_distance_;
    const Vector3d lhs_probe = pos + lhs.normalized() * probe_dist;
    const Vector3d rhs_probe = pos + rhs.normalized() * probe_dist;
    return sdf_map_->getDistance(lhs_probe) > sdf_map_->getDistance(rhs_probe);
  });
  // 2026-07-28: 正常向前恢复后完全解锁；若仍被困，冷却后最多再尝试一次且必须换方向，
  // 既允许视频中的“小退再绕”，又不能把多个0.45m回撤串成同方向持续倒飞。
  if (short_backtrack_latched_ &&
      (pos - short_backtrack_release_origin_).dot(short_backtrack_forward_dir_) >=
          short_backtrack_forward_release_) {
    short_backtrack_latched_ = false;
    short_backtrack_last_dir_.setZero();
    ROS_WARN("[mission_exploration] forward recovery reached %.2fm; the one-time short "
             "backtrack remains consumed.", short_backtrack_forward_release_);
  }
  const bool timed_direction_change_retry =
      short_backtrack_latched_ && short_backtrack_chain_count_ < short_backtrack_max_chain_ &&
      !short_backtrack_last_time_.isZero() &&
      (ros::Time::now() - short_backtrack_last_time_).toSec() >= short_backtrack_retry_cooldown_;
  // 2026-07-23: 记录前向扇区各级拒绝原因，区分“方向没试到”和真实无安全路径，
  // 避免再次只看到笼统的no safe fallback而无法判断卡在哪一层。
  int rejected_workspace = 0;
  int rejected_progress = 0;
  int rejected_position = 0;
  int rejected_astar = 0;
  int rejected_path = 0;
  // 普通水平恢复始终回到任务巡航高度，不能把转弯期间的实际掉高固化成新目标。
  // 明确的上下避障仍由 vertical_detour 独立选择临时高度。
  const double recovery_z =
      task_search_manager_ ? task_search_manager_->preferredSearchHeight() : pos.z();
  struct RecoveryChoice {
    bool valid{false};
    Vector3d position{0.0, 0.0, 0.0};
    Vector3d direction{1.0, 0.0, 0.0};
    double distance{0.0};
    double clearance{-1.0};
    double minimum_path_clearance{-1.0};
    bool short_backtrack{false};
    route_width::Score width;
  };
  RecoveryChoice best_width, straight_fallback;
  RecoveryChoice best_micro_adjustment;
  RecoveryChoice long_side_fallback;
  const double current_clearance = sdf_map_->getDistance(pos);
  const double micro_max_distance = 0.40;
  const double micro_min_clearance_gain = 0.02;

  auto minimumPathClearance = [&](const vector<Vector3d>& path) {
    if (path.empty()) return -1.0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    for (size_t segment = 1; segment < path.size(); ++segment) {
      const Vector3d delta = path[segment] - path[segment - 1];
      const int samples =
          std::max(1, static_cast<int>(std::ceil(delta.norm() / 0.05)));
      for (int sample = 0; sample <= samples; ++sample) {
        const Vector3d point = path[segment - 1] +
            delta * static_cast<double>(sample) / static_cast<double>(samples);
        minimum_clearance =
            std::min(minimum_clearance, sdf_map_->getDistance(point));
      }
    }
    return std::isfinite(minimum_clearance) ? minimum_clearance : -1.0;
  };

  auto commitRecoveryChoice = [&](const RecoveryChoice& choice, bool micro_adjustment) {
    next_pos = choice.position;
    const bool straight_forward =
        choice.direction.normalized().dot(recovery_forward.normalized()) >=
        std::cos(10.0 * M_PI / 180.0);
    // 正常直行恢复才让航向跟随运动方向；所有横向/斜向恢复均保持当前机头。
    next_yaw = (!choice.short_backtrack && straight_forward)
                   ? std::atan2(choice.direction.y(), choice.direction.x())
                   : cur_yaw;
    if (!choice.short_backtrack && straight_forward && !manual_fapp_active_) {
      recovery_side_latched_ = false;
    } else if (!choice.short_backtrack && !straight_forward && !recovery_side_latched_) {
      recovery_side_latched_ = true;
      recovery_side_origin_ = pos;
      recovery_side_dir_ = choice.direction.normalized();
      ROS_WARN("[mission_exploration] side recovery selected; keep side translation "
               "until forward passage is clear and %.2fm forward progress is reached.",
               recovery_side_release_distance_);
    }
    if (task_search_manager_ && !external_selection_only_)
      task_search_manager_->recordSelectedGoal(next_pos);
    if (choice.short_backtrack) {
      pending_short_backtrack_ = true;
      pending_short_backtrack_target_ = next_pos;
      pending_short_backtrack_dir_ = choice.direction.normalized();
      pending_short_backtrack_forward_dir_ = recovery_forward.normalized();
      ROS_WARN("[mission_exploration] one-time short backtrack candidate target=(%.2f, %.2f, %.2f) "
               "dist=%.2fm; consume only after trajectory generation succeeds.",
               next_pos.x(), next_pos.y(), next_pos.z(), choice.distance);
    }
    if (micro_adjustment) {
      ROS_WARN("[clearance_micro_adjust] choose %.2fm side/diagonal translation; clearance "
               "%.2f -> %.2fm, full-path minimum=%.2fm, keep yaw=%.1fdeg.",
               choice.distance, current_clearance, choice.clearance,
               choice.minimum_path_clearance,
               next_yaw * 180.0 / M_PI);
    }
    ROS_WARN_THROTTLE(0.5,
                      "[task_search] 3D recovery target=(%.2f, %.2f, %.2f) dist=%.2f yaw=%.1fdeg.",
                      next_pos.x(), next_pos.y(), next_pos.z(), choice.distance,
                      next_yaw * 180.0 / M_PI);
    return true;
  };

  bool split_obstacle_detected = false;
  auto tryObstacleRecovery = [&]() {
    // 竖直障碍物把通道切成左右两路时，先直接去占据地图中净宽更大的一侧。
    // 该场景不参与普通frontier总分，也不等待上下绕行接管。
    // 只有底部以下已知自由的悬空障碍才触发固定 XY 下降。
    // 贯穿障碍或低位 UNKNOWN 继续由左右分流处理，不下降试探。
    const bool suspended_forward_obstacle =
        hasSuspendedForwardObstacle(pos, recovery_forward);
    if (suspended_forward_obstacle &&
        pos.z() > vertical_detour_low_height_ + vertical_detour_height_tolerance_) {
      if (buildVerticalDetourFallback(pos, cur_yaw, recovery_forward, next_pos,
                                      next_yaw, true)) {
        ROS_WARN("[vertical_detour] suspended obstacle confirmed with free space below; "
                 "descend in place before any forward release.");
        return true;
      }
      ROS_WARN_THROTTLE(
          0.5,
          "[vertical_detour] suspended-obstacle candidate cannot start a safe vertical "
          "probe; keep horizontal/updated-map fallback.");
    }

    if (buildWideSideBypass(pos, cur_yaw, recovery_forward, next_pos, next_yaw,
                            split_obstacle_detected))
      return true;
    return false;
  };

  bool entry_obstacle_checked = false;
  for (const auto& direction : directions) {
    if (manual_fapp_active_ && recovery_side_latched_ &&
        direction.dot(recovery_side_dir_) < -1e-3) continue;
    // Keep the original entry ordering; width ranking starts only after entry.
    if (!entry_forward_complete_ && !entry_obstacle_checked &&
        direction.normalized().dot(recovery_forward.normalized()) <
            std::cos(10.0 * M_PI / 180.0)) {
      entry_obstacle_checked = true;
      if (tryObstacleRecovery()) return true;
    }
    // Compare all feasible forward directions over the same known-free horizon.
    const bool short_backtrack = task_search_manager_ &&
        task_search_manager_->isRecoveryDirectionBackward(direction, cur_yaw);
    // All forward directions have been compared before optional backtracking.
    if (short_backtrack && best_width.valid) return commitRecoveryChoice(best_width, false);
    if (short_backtrack && straight_fallback.valid) return commitRecoveryChoice(straight_fallback, false);
    if (short_backtrack && !short_backtrack_enabled_) continue;
    if (short_backtrack && short_backtrack_chain_count_ >= short_backtrack_max_chain_) continue;
    if (short_backtrack && short_backtrack_latched_) {
      if (!timed_direction_change_retry) continue;
      const double min_change_rad = short_backtrack_min_direction_change_deg_ * M_PI / 180.0;
      // 2026-07-28: 第二次回撤与上一次夹角不足阈值时拒绝，硬性防止沿一条线继续往后退。
      if (short_backtrack_last_dir_.norm() > 1e-3 &&
          direction.normalized().dot(short_backtrack_last_dir_.normalized()) >
              std::cos(min_change_rad))
        continue;
    }
    // 新通道确认后，大偏角方向只能用于局部横移。否则 ±60~90 度候选会沿旧通道
    // 探测完整 1.8m，在转角外生成看似“旧方向前冲”的恢复目标。
    const double direction_max_dist = short_backtrack
        ? short_backtrack_max_distance_
        : task_search::recoveryDirectionMaxDistance(
              direction, recovery_forward, max_dist, 0.40);
    const double direction_min_dist =
        short_backtrack ? short_backtrack_min_distance_ : step;
    // 正常恢复也按10cm采样，使0.20/0.30/0.40m微调都能成为候选。
    const double direction_step = 0.10;
    for (double dist = direction_max_dist; dist >= direction_min_dist - 1e-3;
         dist -= direction_step) {
      Vector3d candidate = pos + direction * dist;
      candidate.z() = recovery_z;
      if (!pointInsideWorkspaceLock(candidate)) {
        ++rejected_workspace;
        continue;
      }
      if (task_search_manager_ && !task_search_manager_->isRecoveryCandidateUseful(candidate)) {
        ++rejected_progress;
        continue;
      }
      // 2026-07-13: 恢复点不能只检查中心栅格，必须容纳 0.15m 膨胀外的完整 Iris 足迹。
      if (!planner_manager_->isPositionSafe(candidate) ||
          sdf_map_->getOccupancy(candidate) == SDFMap::UNKNOWN) {
        ++rejected_position;
        continue;
      }
      // 短回撤终点不得比当前位置更靠近障碍物。
      if (short_backtrack && sdf_map_->getDistance(candidate) + 0.005 <
                                 sdf_map_->getDistance(pos)) {
        ++rejected_position;
        continue;
      }

      planner_manager_->path_finder_->reset();
      if (planner_manager_->path_finder_->search(pos, candidate) != Astar::REACH_END) {
        ++rejected_astar;
        continue;
      }
      const auto candidate_path = planner_manager_->path_finder_->getPath();
      // 2026-07-13: A* 中心线可达不代表旋翼包络可达，危险路径直接换下一个恢复方向。
      bool backtrack_clearance_safe = true;
      if (short_backtrack) {
        double previous_clearance = sdf_map_->getDistance(pos);
        for (const auto& point : candidate_path) {
          const double clearance = sdf_map_->getDistance(point);
          if (clearance + 0.005 < previous_clearance) {
            backtrack_clearance_safe = false;
            break;
          }
          previous_clearance = std::max(previous_clearance, clearance);
        }
      }
      if (!backtrack_clearance_safe || !pathInsideWorkspaceLock(candidate_path) ||
          !planner_manager_->isPathSafe(candidate_path) ||
          (task_search_manager_ &&
           !task_search_manager_->isRecoveryPathAllowed(candidate_path, short_backtrack))) {
        ++rejected_path;
        continue;
      }

      const bool straight_forward =
          direction.normalized().dot(recovery_forward.normalized()) >=
          std::cos(10.0 * M_PI / 180.0);
      RecoveryChoice choice;
      choice.valid = true;
      choice.position = candidate;
      choice.direction = direction;
      choice.distance = dist;
      choice.clearance = sdf_map_->getDistance(candidate);
      choice.minimum_path_clearance = minimumPathClearance(candidate_path);
      choice.short_backtrack = short_backtrack;

      if (short_backtrack || (straight_forward && !entry_forward_complete_))
        return commitRecoveryChoice(choice, false);
      if (entry_forward_complete_)
        choice.width = evaluateRouteWidth(candidate_path, recovery_forward);
      if (choice.width.complete &&
          (!best_width.valid || route_width::prefer(choice.width, best_width.width)))
        best_width = choice;
      if (straight_forward) {
        if (!straight_fallback.valid) straight_fallback = choice;
        if (choice.width.complete) break;
        continue;  // A feasible straight route must also compete on width.
      }

      // 直行不可用后不再立刻取首个远距离斜向点。先比较20~40cm候选，选择净空增益最大者；
      // 相同净空优先更短、更接近原前进方向的平移。
      if (dist <= micro_max_distance + 1e-3 &&
          choice.clearance >= current_clearance + micro_min_clearance_gain) {
        const double alignment = direction.normalized().dot(recovery_forward.normalized());
        const double best_alignment = best_micro_adjustment.valid
                                          ? best_micro_adjustment.direction.normalized().dot(
                                                recovery_forward.normalized())
                                          : -2.0;
        if (!best_micro_adjustment.valid ||
            vertical_detour::preferPathByMinimumClearance(
                choice.minimum_path_clearance, choice.clearance,
                best_micro_adjustment.minimum_path_clearance,
                best_micro_adjustment.clearance, choice.distance,
                best_micro_adjustment.distance, alignment, best_alignment)) {
          best_micro_adjustment = choice;
        }
      } else {
        // 较长侧绕同样按整条A*路径的最低净空排序。终点很空但中途贴墙的路线
        // 不能再因为方向枚举靠前而直接获胜。
        const double alignment = direction.normalized().dot(recovery_forward.normalized());
        const double best_alignment = long_side_fallback.valid
                                          ? long_side_fallback.direction.normalized().dot(
                                                recovery_forward.normalized())
                                          : -2.0;
        if (!long_side_fallback.valid ||
            vertical_detour::preferPathByMinimumClearance(
                choice.minimum_path_clearance, choice.clearance,
                long_side_fallback.minimum_path_clearance,
                long_side_fallback.clearance, choice.distance,
                long_side_fallback.distance, alignment, best_alignment))
          long_side_fallback = choice;
      }
      if (choice.width.complete) break;
    }
  }

  if (best_width.valid) {
    ROS_WARN("[route_width] recovery selected width=%.2fm mean=%.2fm clearance=%.2fm "
             "horizon=%.2fm straight=%d target=(%.2f,%.2f,%.2f).",
             best_width.width.minimum, best_width.width.mean, best_width.width.clearance,
             route_width_horizon_, static_cast<int>(best_width.direction.normalized().dot(
                 recovery_forward.normalized()) >= std::cos(10.0 * M_PI / 180.0)),
             best_width.position.x(), best_width.position.y(), best_width.position.z());
    return commitRecoveryChoice(best_width, false);
  }
  if (straight_fallback.valid) return commitRecoveryChoice(straight_fallback, false);
  if (tryObstacleRecovery()) return true;
  // 专用分流检查已经确认左右均无安全落点时，通用微调不能再用一个局部净空
  // 更高的小步冒充完整侧绕；此时应交给后面的上下绕行。
  if (vertical_detour::acceptGenericHorizontalFallback(
          split_obstacle_detected, best_micro_adjustment.valid))
    return commitRecoveryChoice(best_micro_adjustment, true);
  if (vertical_detour::acceptGenericHorizontalFallback(
          split_obstacle_detected, long_side_fallback.valid))
    return commitRecoveryChoice(long_side_fallback, false);
  if (split_obstacle_detected) {
    ROS_WARN_THROTTLE(
        0.5,
        "[wide_side_bypass] horizontal split remains, but no safe side step is available; "
        "try vertical recovery after horizontal candidates are exhausted.");
  }
  // 上下绕行严格保持最后尝试，并且必须有当前飞行高度的前方占据证据。
  // 远目标断连或未知地图本身不能触发下降，避免在空旷道路误判下穿。
  const bool current_height_obstacle =
      hasCurrentHeightForwardObstacle(pos, recovery_forward);
  if (vertical_detour::allowVerticalRecovery(current_height_obstacle) &&
      buildVerticalDetourFallback(pos, cur_yaw, recovery_forward, next_pos, next_yaw))
    return true;
  if (!current_height_obstacle) {
    ROS_WARN_THROTTLE(
        1.0,
        "[vertical_detour] skip vertical recovery: horizontal candidates exhausted but "
        "no current-height forward obstacle is confirmed.");
  }

  // 2026-07-28: 日志同时给出短回撤锁状态，区分“没有安全回撤点”和“已回撤、等待重新前进”。
  ROS_WARN_THROTTLE(
      1.0,
      "[mission_exploration] no safe recovery fallback: directions=%zu short_backtrack_locked=%d "
      "workspace=%d progress=%d position=%d astar=%d path=%d.",
      directions.size(), static_cast<int>(short_backtrack_latched_), rejected_workspace,
      rejected_progress, rejected_position, rejected_astar, rejected_path);
  next_yaw = cur_yaw;
  return false;
}

bool FastExplorationManager::planInflationHistoryEscape(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
    const Vector3d& yaw) {
  if (!inflation_history_escape_enabled_ || recovery_odom_history_.size() < 2 ||
      !planner_manager_->isControlledEscapePosition(pos))
    return false;

  const double max_distance = inflation_history_escape_max_distance_;
  const double sample_step = inflation_history_escape_sample_step_;
  const double start_clearance = sdf_map_->getDistance(pos);
  const int start_contacts = planner_manager_->rawFootprintCollisionCount(pos);
  struct EscapeCandidate {
    vector<Vector3d> path;
    double length{0.0};
    double end_clearance{-1.0};
    int priority{1};
    bool backward{false};
    const char* label{"history-tangent"};
  };
  vector<EscapeCandidate> candidates;

  auto validateAndTruncate = [&](const vector<Vector3d>& source, bool backward,
                                 int priority, const char* label) {
    if (source.size() < 2) return;
    vector<Vector3d> checked{pos};
    double traveled = 0.0;
    bool reached_clear = false;
    double clear_run = 0.0;
    double previous_clearance = start_clearance;
    int previous_contacts = start_contacts;
    for (size_t segment = 1; segment < source.size() && traveled < max_distance - 1e-6;
         ++segment) {
      const Vector3d delta = source[segment] - source[segment - 1];
      const double segment_length = delta.norm();
      if (segment_length < 1e-6) continue;
      const int samples =
          std::max(1, static_cast<int>(std::ceil(segment_length / sample_step)));
      for (int sample = 1; sample <= samples; ++sample) {
        Vector3d point = source[segment - 1] +
                         delta * static_cast<double>(sample) /
                             static_cast<double>(samples);
        double advance = (point - checked.back()).norm();
        if (traveled + advance > max_distance) {
          const double remaining = max_distance - traveled;
          if (remaining <= 1e-6) break;
          point = checked.back() + (point - checked.back()).normalized() * remaining;
          advance = remaining;
        }
        if (!pointInsideWorkspaceLock(point)) return;
        const int contacts = planner_manager_->rawFootprintCollisionCount(point);
        const double clearance = sdf_map_->getDistance(point);
        if (!std::isfinite(clearance) || contacts > previous_contacts ||
            clearance + inflation_history_escape_clearance_tolerance_ <
                previous_clearance)
          return;
        previous_contacts = std::min(previous_contacts, contacts);
        previous_clearance = std::max(previous_clearance, clearance);
        checked.push_back(point);
        traveled += advance;
        const bool fully_clear =
            !planner_manager_->isPositionInflated(point) && contacts == 0;
        if (fully_clear) {
          if (!reached_clear) {
            reached_clear = true;
            clear_run = 0.0;
          } else {
            clear_run += advance;
          }
          if (clear_run + 1e-6 >= inflation_escape_clear_margin_) break;
        } else if (reached_clear) {
          // 离开膨胀层后禁止为了凑路径长度再次擦回墙面。
          return;
        }
        if (traveled >= max_distance - 1e-6) break;
      }
      if ((reached_clear && clear_run + 1e-6 >= inflation_escape_clear_margin_) ||
          traveled >= max_distance - 1e-6)
        break;
    }
    if (!reached_clear || traveled < 0.03) return;
    EscapeCandidate candidate;
    candidate.path = std::move(checked);
    candidate.length = traveled;
    candidate.end_clearance = sdf_map_->getDistance(candidate.path.back());
    candidate.priority = priority;
    candidate.backward = backward;
    candidate.label = label;
    candidates.push_back(std::move(candidate));
  };

  // 先从当前位置向16个水平方向探测，选择原始足迹占据采样减少最快、ESDF净空
  // 增长最大的方向。它直接把受风贴墙的飞机横向拉回通道，而不是继续沿旧航迹蹭墙。
  if (inflation_wall_pull_enabled_) {
    Vector3d recent_forward = recovery_odom_history_.back() -
                              recovery_odom_history_[recovery_odom_history_.size() - 2];
    recent_forward.z() = 0.0;
    if (recent_forward.head<2>().norm() > 1e-3) recent_forward.normalize();
    double best_score = -std::numeric_limits<double>::infinity();
    Vector3d best_direction = Vector3d::Zero();
    const double probe_distance = std::min(0.20, max_distance);
    for (int sample = 0; sample < 16; ++sample) {
      const double angle = 2.0 * M_PI * static_cast<double>(sample) / 16.0;
      const Vector3d direction(std::cos(angle), std::sin(angle), 0.0);
      if (recent_forward.head<2>().norm() > 1e-3 &&
          direction.dot(recent_forward) < -0.10)
        continue;
      const Vector3d probe = pos + probe_distance * direction;
      if (!pointInsideWorkspaceLock(probe)) continue;
      const int contacts = planner_manager_->rawFootprintCollisionCount(probe);
      const double clearance = sdf_map_->getDistance(probe);
      if (!std::isfinite(clearance) || contacts > start_contacts ||
          clearance + inflation_history_escape_clearance_tolerance_ < start_clearance)
        continue;
      const double lateral_bonus =
          recent_forward.head<2>().norm() > 1e-3
              ? 0.10 * (1.0 - std::fabs(direction.dot(recent_forward)))
              : 0.0;
      const double score = 2.0 * static_cast<double>(start_contacts - contacts) +
                           clearance - start_clearance + lateral_bonus;
      if (score > best_score) {
        best_score = score;
        best_direction = direction;
      }
    }
    if (best_direction.head<2>().norm() > 1e-3 && best_score > 1e-3) {
      validateAndTruncate(inflation_escape::buildDirectionalPath(
                              pos, best_direction, max_distance, sample_step),
                          false, 0, "wall-normal");
    }
  }

  validateAndTruncate(inflation_escape::buildForwardTangentPath(
                          pos, recovery_odom_history_, max_distance, sample_step),
                      false, 1, "history-tangent");
  validateAndTruncate(inflation_escape::buildBackwardHistoryPath(
                          pos, recovery_odom_history_, max_distance,
                          recovery_history_spacing_),
                      true, 2, "history-backtrack");
  if (candidates.empty()) {
    ROS_WARN_THROTTLE(0.5,
                      "[inflation_history_escape] no monotonic wall-pull/history exit "
                      "within %.2fm (start_contacts=%d).",
                      max_distance, start_contacts);
    return false;
  }

  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const EscapeCandidate& lhs, const EscapeCandidate& rhs) {
    if (lhs.priority != rhs.priority) return lhs.priority < rhs.priority;
    if (std::fabs(lhs.length - rhs.length) > 0.02) return lhs.length < rhs.length;
    return lhs.end_clearance > rhs.end_clearance;
  });
  for (auto& candidate : candidates) {
    if (candidate.path.size() == 2)
      candidate.path.insert(candidate.path.begin() + 1,
                            0.5 * (candidate.path.front() + candidate.path.back()));
    if (!planner_manager_->isPathSafe(candidate.path, true)) continue;
    Vector3d escape_vel = Vector3d::Zero();
    const Vector3d escape_direction =
        (candidate.path[1] - candidate.path.front()).normalized();
    const double outward_speed = std::max(0.0, vel.dot(escape_direction));
    escape_vel = std::min(0.20, outward_speed) * escape_direction;
    if (!planner_manager_->planExploreTraj(candidate.path, escape_vel, acc, 0.0) ||
        !planner_manager_->isTrajectorySafe(0.03, true) ||
        !planner_manager_->trajectoryClearsInflation(max_distance, 0.02, true))
      continue;

    ed_->path_next_goal_ = candidate.path;
    ed_->next_goal_ = candidate.path.back();
    last_requested_goal_ = ed_->next_goal_;
    has_last_requested_goal_ = true;
    // 转向前的脱困只保持当前航向，复用硬约束yaw样条，避免零航向常量
    // 进入普通yaw优化器时产生退化问题；平移轨迹及其安全检查保持不变。
    if (turn_in_place_session_active_) {
      if (!planner_manager_->planYawTurnInPlace(yaw, yaw[0])) continue;
    } else {
      planner_manager_->planYawExplore(yaw, yaw[0], false, ep_->relax_time_);
    }
    ROS_ERROR("[inflation_history_escape] publish %s %s escape %.2fm "
              "to (%.2f %.2f %.2f), yaw held at %.1fdeg, start_contacts=%d.",
              candidate.backward ? "BACKWARD" : "FORWARD", candidate.label,
              candidate.length,
              ed_->next_goal_.x(), ed_->next_goal_.y(), ed_->next_goal_.z(),
              yaw[0] * 180.0 / M_PI, start_contacts);
    return true;
  }

  ROS_WARN_THROTTLE(0.5,
                    "[inflation_history_escape] geometric exits exist but optimized "
                    "trajectories failed monotonic contact/inflation clearance validation.");
  return false;
}

bool FastExplorationManager::buildTurnInPlacePlan(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
    const Vector3d& yaw, const Vector3d& turn_direction) {
  if (!turn_in_place_enabled_ || !task_search_manager_ ||
      !task_search_manager_->turnYawAlignmentPending() ||
      turn_direction.head<2>().norm() < 1e-3)
    return false;

  const double detected_target_yaw =
      std::atan2(turn_direction.y(), turn_direction.x());
  if (!turn_in_place_session_active_) {
    turn_in_place_session_active_ = true;
    turn_in_place_final_yaw_ = detected_target_yaw;
    turn_in_place_still_since_ = ros::Time(0);
    skip_straight_extension_after_turn_ = true;
  }
  // 原地转向无法退出足迹接触/膨胀区域；先复用经过完整路径和轨迹检查的
  // 脱困平移，保持最终转向目标锁存，离开接触区域后再从实时位置转向。
  if (!planner_manager_->isPositionSafe(pos)) {
    turn_in_place_plan_ = false;
    turn_in_place_still_since_ = ros::Time(0);
    ROS_WARN_THROTTLE(0.5,
                      "[turn_in_place] clear footprint/inflation contact before yaw "
                      "alignment; final yaw remains %.1fdeg.",
                      turn_in_place_final_yaw_ * 180.0 / M_PI);
    return !external_selection_only_ && planInflationHistoryEscape(pos, vel, acc, yaw);
  }
  const ros::Time now = ros::Time::now();
  const double speed = vel.norm();
  if (!std::isfinite(speed) || speed > turn_in_place_max_stationary_speed_) {
    turn_in_place_still_since_ = ros::Time(0);
    ROS_WARN_THROTTLE(0.5,
                      "[turn_in_place] wait for brake before planning: speed=%.3fm/s "
                      "limit=%.3fm/s.",
                      speed, turn_in_place_max_stationary_speed_);
    return false;
  }
  if (turn_in_place_still_since_.isZero()) turn_in_place_still_since_ = now;
  const double stable_elapsed = (now - turn_in_place_still_since_).toSec();
  if (!task_search::stationaryTurnReady(
          speed, turn_in_place_max_stationary_speed_, stable_elapsed,
          turn_in_place_still_confirm_time_)) {
    ROS_WARN_THROTTLE(0.5,
                      "[turn_in_place] stationary confirmation %.2f/%.2fs at "
                      "speed=%.3fm/s.",
                      stable_elapsed, turn_in_place_still_confirm_time_, speed);
    return false;
  }

  // 最终yaw在整个转弯会话中保持锁存；真正准备发布时从最新静止里程计
  // 重新取位置锚点，避免首次检测位置与严格轨迹交接门槛永久冲突。
  turn_in_place_anchor_ = pos;
  const double turn_delta = std::atan2(
      std::sin(turn_in_place_final_yaw_ - yaw[0]),
      std::cos(turn_in_place_final_yaw_ - yaw[0]));
  const double yaw_rate =
      std::max(5.0, turn_in_place_yaw_rate_deg_) * M_PI / 180.0;
  constexpr double kTurnRampFraction = 0.15;
  const double duration = std::max(
      turn_in_place_min_duration_,
      std::min(turn_in_place_max_duration_,
               std::fabs(turn_delta) /
                   (yaw_rate * (1.0 - kTurnRampFraction))));
  if (!planner_manager_->planStationaryTraj(turn_in_place_anchor_, duration)) return false;

  ed_->path_next_goal_ = {turn_in_place_anchor_};
  ed_->next_goal_ = turn_in_place_anchor_;
  if (!planner_manager_->planYawTurnInPlace(yaw, turn_in_place_final_yaw_))
    return false;
  turn_in_place_plan_ = true;
  turn_in_place_still_since_ = ros::Time(0);
  ROS_ERROR("[turn_in_place] direct candidate uses live anchor "
            "(%.2f,%.2f,%.2f), yaw %.1fdeg -> final %.1fdeg over %.2fs.",
            turn_in_place_anchor_.x(),
            turn_in_place_anchor_.y(), turn_in_place_anchor_.z(),
            yaw[0] * 180.0 / M_PI,
            turn_in_place_final_yaw_ * 180.0 / M_PI,
            planner_manager_->local_data_.duration_);
  return true;
}

double FastExplorationManager::turnInPlaceFinalYawError(
    double current_yaw) const {
  if (!turn_in_place_session_active_) return 0.0;
  return std::atan2(std::sin(turn_in_place_final_yaw_ - current_yaw),
                    std::cos(turn_in_place_final_yaw_ - current_yaw));
}

double FastExplorationManager::turnInPlaceCompletionTolerance() const {
  return turn_in_place_completion_tolerance_deg_ * M_PI / 180.0;
}

void FastExplorationManager::completeTurnInPlace(const Vector3d&) {
  if (task_search_manager_) task_search_manager_->completeTurnYawAlignment();
  // 转向完成后直接恢复平移规划；爬升和上绕由正常避障选路决定。
  turn_in_place_session_active_ = false;
  recovery_side_latched_ = false;
  wide_side_bypass_active_ = false;
  turn_in_place_still_since_ = ros::Time(0);
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  nh.param("fapp/exploration_crossing_enabled", fapp_exploration_crossing_enabled_, false);
  fapp_rush_pub_ = nh.advertise<std_msgs::Bool>("fapp/dynamic_crossing", 1, true);
  std_msgs::Bool crossing_mode; crossing_mode.data = false; fapp_rush_pub_.publish(crossing_mode);
  nh.param("mission/manual_path/enabled", manual_path_enabled_, false);
  nh.param("mission/manual_path/use_astar", manual_path_use_astar_, false);
  nh.param("mission/manual_path/wait_for_trigger", manual_path_wait_for_trigger_, false);
  nh.param("mission/manual_path/exit_enabled", manual_exit_enabled_, false);
  nh.param("mission/manual_path/arrive_distance", manual_path_arrive_distance_, 0.25);
  nh.param("mission/manual_path/max_segment_length", manual_path_max_segment_length_, 0.0);
  nh.param("mission/manual_path/exploration_style", manual_path_exploration_style_, false);
  if (!std::isfinite(manual_path_max_segment_length_) || manual_path_max_segment_length_ < 0.0)
    throw std::runtime_error("manual_path max_segment_length must be finite and nonnegative");
  if (manual_path_enabled_) {
    XmlRpc::XmlRpcValue coordinates;
    if (!nh.getParam("mission/manual_path/waypoints", coordinates) ||
        coordinates.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        !std::isfinite(manual_path_arrive_distance_) || manual_path_arrive_distance_ <= 0.0)
      throw std::runtime_error("manual_path requires a waypoint array and positive arrive_distance");
    manual_path_.points.clear();
    manual_path_.next = 0;
    for (int i = 0; i < coordinates.size(); ++i) {
      if (coordinates[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
          coordinates[i].size() != 3)
        throw std::runtime_error("manual_path waypoint must be [x, y, z]");
      Vector3d point;
      for (int axis = 0; axis < 3; ++axis) {
        auto& value = coordinates[i][axis];
        if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
          point[axis] = static_cast<int>(value);
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
          point[axis] = static_cast<double>(value);
        else
          throw std::runtime_error("manual_path coordinates must be numbers");
      }
      if (!point.allFinite()) throw std::runtime_error("manual_path coordinates must be finite");
      manual_path_.points.push_back(point);
    }
    ROS_WARN("[manual_path] enabled with %zu ordered waypoints in the odometry world frame.",
             manual_path_.points.size());
  }
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  safe_relay_ = std::make_shared<SafeRelayPublisher>();
  safe_relay_->initialize(nh, sdf_map_, planner_manager_);
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // 2026-07-13: 初始化比赛第二阶段任务搜索层，颜色/二维码/温度接口与 frontier 生命周期解耦。
  task_search_manager_.reset(new TaskSearchManager);
  task_search_manager_->initialize(nh);
  task_search_manager_->deferExitCrossing(manual_path_enabled_ && manual_exit_enabled_);
  nh.param("mission/short_backtrack_enabled", short_backtrack_enabled_, false);
  // 短回撤启用时的距离、冷却、次数和换向角均可在launch调节。
  nh.param("mission/short_backtrack_min_distance", short_backtrack_min_distance_, 0.10);
  nh.param("mission/short_backtrack_max_distance", short_backtrack_max_distance_, 0.20);
  nh.param("mission/short_backtrack_forward_release", short_backtrack_forward_release_, 0.60);
  nh.param("mission/short_backtrack_retry_cooldown", short_backtrack_retry_cooldown_, 1.5);
  nh.param("mission/short_backtrack_max_chain", short_backtrack_max_chain_, 1);
  nh.param("mission/short_backtrack_min_direction_change_deg",
           short_backtrack_min_direction_change_deg_, 35.0);
  nh.param("mission/inflation_history_escape/enabled",
           inflation_history_escape_enabled_, true);
  nh.param("mission/inflation_history_escape/max_distance",
           inflation_history_escape_max_distance_, 0.45);
  nh.param("mission/inflation_history_escape/sample_step",
           inflation_history_escape_sample_step_, 0.05);
  nh.param("mission/inflation_history_escape/clearance_tolerance",
           inflation_history_escape_clearance_tolerance_, 0.02);
  nh.param("mission/inflation_history_escape/wall_pull_enabled",
           inflation_wall_pull_enabled_, true);
  nh.param("mission/inflation_history_escape/clear_margin",
           inflation_escape_clear_margin_, 0.10);
  nh.param("mission/inflation_history_escape/history_spacing",
           recovery_history_spacing_, 0.03);
  nh.param("mission/inflation_history_escape/history_max_size",
           recovery_history_max_size_, 200);
  nh.param("mission/task_search/recovery/turn_in_place_enabled",
           turn_in_place_enabled_, true);
  nh.param("mission/task_search/recovery/turn_in_place_yaw_rate_deg",
           turn_in_place_yaw_rate_deg_, 30.0);
  nh.param("mission/task_search/recovery/turn_in_place_completion_tolerance_deg",
           turn_in_place_completion_tolerance_deg_, 8.0);
  nh.param("mission/task_search/recovery/turn_in_place_completion_confirm_time",
           turn_in_place_completion_confirm_time_, 0.15);
  nh.param("mission/task_search/recovery/turn_in_place_tracking_grace_time",
           turn_in_place_tracking_grace_time_, 1.0);
  nh.param("mission/task_search/recovery/turn_in_place_max_stationary_speed",
           turn_in_place_max_stationary_speed_, 0.20);
  nh.param("mission/task_search/recovery/turn_in_place_still_confirm_time",
           turn_in_place_still_confirm_time_, 0.15);
  nh.param("mission/task_search/recovery/turn_in_place_min_duration",
           turn_in_place_min_duration_, 1.0);
  nh.param("mission/task_search/recovery/turn_in_place_max_duration",
           turn_in_place_max_duration_, 4.0);
  nh.param("mission/ground_ascent/enabled", ground_ascent_enabled_, true);
  nh.param("mission/ground_ascent/cruise_height",
           ground_ascent_cruise_height_, 0.60);
  nh.param("mission/ground_ascent/lower_probe_depth",
           ground_ascent_lower_probe_depth_, 0.20);
  nh.param("mission/ground_ascent/min_lower_obstacles",
           ground_ascent_min_lower_obstacles_, 2);
  nh.param("mission/ground_ascent/max_stationary_speed",
           ground_ascent_max_stationary_speed_, 0.12);
  nh.param("mission/ground_ascent/still_confirm_time",
           ground_ascent_still_confirm_time_, 0.15);
  nh.param("mission/ground_ascent/height_tolerance",
           ground_ascent_height_tolerance_, 0.06);
  nh.param("mission/wide_side_bypass/enabled", wide_side_bypass_enabled_, true);
  nh.param("mission/wide_side_bypass/min_lookahead",
           wide_side_bypass_min_lookahead_, 0.20);
  nh.param("mission/wide_side_bypass/max_lookahead",
           wide_side_bypass_max_lookahead_, 0.90);
  nh.param("mission/wide_side_bypass/lateral_range",
           wide_side_bypass_lateral_range_, 0.90);
  nh.param("mission/wide_side_bypass/min_lane_width",
           wide_side_bypass_min_lane_width_, 0.25);
  nh.param("mission/wide_side_bypass/max_lateral_step",
           wide_side_bypass_max_lateral_step_, 0.45);
  nh.param("mission/wide_side_bypass/forward_step",
           wide_side_bypass_forward_step_, 0.20);
  nh.param("mission/wide_side_bypass/low_support_height",
           wide_side_bypass_low_support_height_, 0.20);
  nh.param("mission/wide_side_bypass/min_vertical_support_layers",
           wide_side_bypass_min_vertical_support_layers_, 2);
  nh.param("mission/center_gate/enabled", center_gate_enabled_, true);
  nh.param("mission/center_gate/min_width", center_gate_min_width_, 0.55);
  nh.param("mission/center_gate/pass_margin", center_gate_pass_margin_, 0.20);
  inflation_history_escape_max_distance_ =
      std::max(0.10, std::min(0.45, inflation_history_escape_max_distance_));
  inflation_history_escape_sample_step_ =
      std::max(0.02, std::min(0.10, inflation_history_escape_sample_step_));
  inflation_history_escape_clearance_tolerance_ =
      std::max(0.0, inflation_history_escape_clearance_tolerance_);
  inflation_escape_clear_margin_ =
      std::max(0.0, std::min(0.20, inflation_escape_clear_margin_));
  recovery_history_spacing_ = std::max(0.01, recovery_history_spacing_);
  recovery_history_max_size_ = std::max(20, recovery_history_max_size_);
  turn_in_place_yaw_rate_deg_ =
      std::max(5.0, std::min(90.0, turn_in_place_yaw_rate_deg_));
  turn_in_place_completion_tolerance_deg_ =
      std::max(1.0, std::min(15.0, turn_in_place_completion_tolerance_deg_));
  turn_in_place_completion_confirm_time_ =
      std::max(0.0, turn_in_place_completion_confirm_time_);
  turn_in_place_tracking_grace_time_ =
      std::max(0.0, turn_in_place_tracking_grace_time_);
  turn_in_place_max_stationary_speed_ =
      std::max(0.01, turn_in_place_max_stationary_speed_);
  turn_in_place_still_confirm_time_ =
      std::max(0.0, turn_in_place_still_confirm_time_);
  turn_in_place_min_duration_ =
      std::max(0.30, turn_in_place_min_duration_);
  turn_in_place_max_duration_ =
      std::max(turn_in_place_min_duration_, turn_in_place_max_duration_);
  ground_ascent_cruise_height_ = std::max(0.10, ground_ascent_cruise_height_);
  ground_ascent_lower_probe_depth_ =
      std::max(0.08, ground_ascent_lower_probe_depth_);
  ground_ascent_min_lower_obstacles_ =
      std::max(1, ground_ascent_min_lower_obstacles_);
  ground_ascent_max_stationary_speed_ =
      std::max(0.01, ground_ascent_max_stationary_speed_);
  ground_ascent_still_confirm_time_ =
      std::max(0.0, ground_ascent_still_confirm_time_);
  ground_ascent_height_tolerance_ =
      std::max(0.01, ground_ascent_height_tolerance_);
  wide_side_bypass_min_lookahead_ =
      std::max(0.10, wide_side_bypass_min_lookahead_);
  wide_side_bypass_max_lookahead_ =
      std::max(wide_side_bypass_min_lookahead_, wide_side_bypass_max_lookahead_);
  wide_side_bypass_lateral_range_ =
      std::max(0.30, wide_side_bypass_lateral_range_);
  wide_side_bypass_min_lane_width_ =
      std::max(0.15, wide_side_bypass_min_lane_width_);
  wide_side_bypass_max_lateral_step_ =
      std::max(0.15, wide_side_bypass_max_lateral_step_);
  wide_side_bypass_forward_step_ =
      std::max(0.0, std::min(0.30, wide_side_bypass_forward_step_));
  wide_side_bypass_low_support_height_ =
      std::max(0.10, wide_side_bypass_low_support_height_);
  wide_side_bypass_min_vertical_support_layers_ =
      std::max(1, wide_side_bypass_min_vertical_support_layers_);
  center_gate_min_width_ = std::max(0.40, center_gate_min_width_);
  center_gate_pass_margin_ =
      std::max(0.10, std::min(0.40, center_gate_pass_margin_));
  nh.param("mission/vertical_detour/enabled", vertical_detour_enabled_, true);
  nh.param("mission/vertical_detour/low_height", vertical_detour_low_height_, 0.10);
  nh.param("mission/vertical_detour/forward_check_distance",
           vertical_detour_forward_check_distance_, 0.20);
  nh.param("mission/vertical_detour/upper_step", vertical_detour_upper_step_, 0.20);
  nh.param("mission/vertical_detour/upper_max_rise", vertical_detour_upper_max_rise_, 0.60);
  nh.param("mission/vertical_detour/upper_forward_max_distance",
           vertical_detour_upper_forward_max_distance_, 0.60);
  nh.param("mission/vertical_detour/upper_max_side_angle_deg",
           vertical_detour_upper_max_side_angle_deg_, 45.0);
  nh.param("mission/vertical_detour/upper_obstacle_clearance",
           vertical_detour_upper_obstacle_clearance_, 0.20);
  nh.param("mission/vertical_detour/down_first_height",
           vertical_detour_down_first_height_, 0.75);
  nh.param("mission/vertical_detour/height_tolerance",
           vertical_detour_height_tolerance_, 0.06);
  nh.param("mission/vertical_detour/xy_tolerance", vertical_detour_xy_tolerance_, 0.08);
  nh.param("mission/vertical_detour/verification_timeout",
           vertical_detour_verification_timeout_, 2.0);
  nh.param("mission/vertical_detour/descent_tracking_timeout",
           vertical_detour_descent_tracking_timeout_, 2.0);
  nh.param("mission/vertical_detour/ascent_timeout",
           vertical_detour_ascent_timeout_, 4.0);
  nh.param("mission/vertical_detour/required_confirmations",
           vertical_detour_required_confirmations_, 1);
  nh.param("mission/vertical_detour/release_confirmations",
           vertical_detour_release_confirmations_, 2);
  nh.param("mission/vertical_detour/footprint_radius",
           vertical_detour_footprint_radius_, 0.17);
  nh.param("mission/vertical_detour/underpass/lidar_above_odom",
           underpass_lidar_above_odom_, 0.044);
  nh.param("mission/vertical_detour/underpass/clearance",
           underpass_clearance_, 0.10);
  nh.param("mission/vertical_detour/underpass/height_tracking_margin",
           underpass_height_tracking_margin_, 0.06);
  nh.param("mission/vertical_detour/underpass/min_height", underpass_min_height_, 0.25);
  nh.param("mission/vertical_detour/underpass/lookahead",
           underpass_lookahead_, 2.00);
  vertical_detour_low_height_ = std::max(0.10, vertical_detour_low_height_);
  vertical_detour_forward_check_distance_ =
      std::max(0.20, vertical_detour_forward_check_distance_);
  vertical_detour_upper_step_ = std::max(0.10, vertical_detour_upper_step_);
  vertical_detour_upper_max_rise_ =
      std::max(vertical_detour_upper_step_, vertical_detour_upper_max_rise_);
  vertical_detour_upper_forward_max_distance_ =
      std::max(vertical_detour_forward_check_distance_,
               vertical_detour_upper_forward_max_distance_);
  vertical_detour_upper_obstacle_clearance_ =
      std::max(0.20, vertical_detour_upper_obstacle_clearance_);
  vertical_detour_footprint_radius_ =
      std::max(0.05, vertical_detour_footprint_radius_);
  underpass_lidar_above_odom_ = std::max(0.0, underpass_lidar_above_odom_);
  underpass_clearance_ = std::max(0.10, underpass_clearance_);
  underpass_height_tracking_margin_ = std::max(0.0, underpass_height_tracking_margin_);
  underpass_lookahead_ = std::max(0.80, underpass_lookahead_);
  vertical_detour_ascent_timeout_ =
      std::max(1.0, vertical_detour_ascent_timeout_);
  vertical_detour_descent_tracking_timeout_ =
      std::max(0.1, vertical_detour_descent_tracking_timeout_);
  vertical_detour_required_confirmations_ =
      std::max(1, vertical_detour_required_confirmations_);
  vertical_detour_release_confirmations_ =
      std::max(1, vertical_detour_release_confirmations_);
  // 2026-07-13: 出口推断必须读取与规划器完全相同的累计 SDFMap，不能退化为实时雷达点云判断。
  task_search_manager_->setMap(sdf_map_);
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);
  nh.param("cmu_exploration", cmu_exploration, false);
  // 2026-07-08 19:26: 为比赛 2.1 增加最小任务层参数。
  // 这层不替代后续二维码/颜色/温度识别，但先把“20s 进搜索区、起飞区禁搜、不能因 frontier 清空就结束”落实。
  nh.param("mission/use_entry_transit", ep_->mission_use_entry_transit_, false);
  nh.param("mission/prefer_entry_heading", ep_->mission_prefer_entry_heading_, true);
  nh.param("mission/use_takeoff_exclusion_box", ep_->mission_use_takeoff_exclusion_box_, false);
  nh.param("mission/prefer_search_region_frontiers", ep_->mission_prefer_search_region_frontiers_,
           false);
  nh.param("mission/hold_on_no_frontier", ep_->mission_hold_on_no_frontier_, false);
  // 2026-07-10: FUEL 接管后锁在门内作业区，门外起飞区地图只保留避障用途，不再贡献探索收益。
  nh.param("mission/use_workspace_lock", ep_->mission_use_workspace_lock_, true);
  nh.param("mission/global_no_return", ep_->mission_global_no_return_, true);
  // 2026-07-16: 相机接入后可打开该接口恢复viewpoint航向；当前Mid360模式关闭，优先实际平移。
  nh.param("mission/use_camera_viewpoint_yaw", use_camera_viewpoint_yaw_, false);
  // 2026-07-24: 全程连续环扫改为调试回退项；比赛默认清晰直段轻摆头、新障碍物触发快速环扫。
  double camera_head_sweep_amplitude_deg = 30.0;
  double camera_clear_sweep_amplitude_deg = 35.0;
  double camera_turn_threshold_deg = 30.0;
  double camera_head_scan_direction = 1.0;
  nh.param("mission/camera_head_sweep/enabled", camera_head_sweep_enabled_, true);
  nh.param("mission/camera_head_sweep/full_rotation", camera_head_full_rotation_, false);
  nh.param("mission/camera_head_sweep/amplitude_deg", camera_head_sweep_amplitude_deg, 30.0);
  nh.param("mission/camera_head_sweep/period", camera_head_sweep_period_, 10.0);
  nh.param("mission/camera_head_sweep/direction", camera_head_scan_direction, 1.0);
  nh.param("mission/camera_obstacle_scan/enabled", camera_obstacle_scan_enabled_, true);
  nh.param("mission/camera_obstacle_scan/clear_sweep_amplitude_deg",
           camera_clear_sweep_amplitude_deg, 35.0);
  nh.param("mission/camera_obstacle_scan/clear_sweep_period",
           camera_clear_sweep_period_, 3.5);
  nh.param("mission/camera_obstacle_scan/full_rotation_period",
           camera_obstacle_scan_period_, 6.0);
  nh.param("mission/camera_obstacle_scan/turn_threshold_deg",
           camera_turn_threshold_deg, 30.0);
  nh.param("mission/camera_obstacle_scan/turn_hold_time",
           camera_turn_hold_time_, 1.2);
  nh.param("mission/camera_obstacle_scan/min_forward",
           camera_obstacle_min_forward_, 0.55);
  nh.param("mission/camera_obstacle_scan/max_forward",
           camera_obstacle_max_forward_, 1.60);
  nh.param("mission/camera_obstacle_scan/lateral_range",
           camera_obstacle_lateral_range_, 0.90);
  nh.param("mission/camera_obstacle_scan/dedup_radius",
           camera_obstacle_dedup_radius_, 0.55);
  nh.param("mission/camera_obstacle_scan/wall_support_length",
           camera_wall_support_length_, 0.90);
  nh.param("mission/camera_obstacle_scan/wall_support_ratio",
           camera_wall_support_ratio_, 0.68);
  camera_head_sweep_amplitude_rad_ =
      std::max(0.0, std::min(75.0, camera_head_sweep_amplitude_deg)) * M_PI / 180.0;
  camera_head_sweep_period_ = std::max(6.0, camera_head_sweep_period_);
  camera_head_scan_direction_ = camera_head_scan_direction < 0.0 ? -1.0 : 1.0;
  camera_clear_sweep_amplitude_rad_ =
      std::max(0.0, std::min(65.0, camera_clear_sweep_amplitude_deg)) * M_PI / 180.0;
  camera_clear_sweep_period_ = std::max(2.5, camera_clear_sweep_period_);
  camera_obstacle_scan_period_ = std::max(4.0, camera_obstacle_scan_period_);
  camera_turn_threshold_rad_ =
      std::max(15.0, std::min(70.0, camera_turn_threshold_deg)) * M_PI / 180.0;
  camera_turn_hold_time_ = std::max(0.5, camera_turn_hold_time_);
  camera_obstacle_min_forward_ = std::max(0.30, camera_obstacle_min_forward_);
  camera_obstacle_max_forward_ =
      std::max(camera_obstacle_min_forward_ + 0.30, camera_obstacle_max_forward_);
  camera_obstacle_lateral_range_ = std::max(0.35, camera_obstacle_lateral_range_);
  camera_obstacle_dedup_radius_ = std::max(0.30, camera_obstacle_dedup_radius_);
  camera_wall_support_length_ = std::max(0.50, camera_wall_support_length_);
  camera_wall_support_ratio_ =
      std::max(0.45, std::min(0.90, camera_wall_support_ratio_));
  camera_head_sweep_start_ = ros::Time(0);
  camera_turning_until_ = ros::Time(0);
  camera_obstacle_scan_start_ = ros::Time(0);
  // 2026-07-10: 门后第二阶段以“向通道/作业区内部推进”为主，frontier 只作为建图收益参考。
  nh.param("mission/use_forward_progress_bias", ep_->mission_use_forward_progress_bias_, true);
  nh.param("mission/use_forward_fallback", ep_->mission_use_forward_fallback_, true);
  // 2026-07-22: 远端frontier只提供方向，实际轨迹每次最多发布一段短安全路线点。
  nh.param("mission/max_route_segment_length", ep_->mission_max_route_segment_length_, 0.80);
  nh.param("mission/straight_run_extension/enabled",
           straight_run_extension_enabled_, true);
  nh.param("mission/straight_run_extension/distance",
           straight_run_extension_distance_, 0.30);
  nh.param("mission/straight_run_extension/min_turn_deg",
           straight_run_extension_min_turn_deg_, 35.0);
  nh.param("mission/route_width/horizon", route_width_horizon_, 1.0);
  nh.param("mission/route_width/replan_gain", route_width_replan_gain_, 0.10);
  if (!std::isfinite(route_width_horizon_) || route_width_horizon_ < 0.4 ||
      !std::isfinite(route_width_replan_gain_) || route_width_replan_gain_ < 0.05)
    throw std::runtime_error("route_width: invalid horizon or replan gain");
  nh.param("mission/passage_centering/enabled", passage_centering_enabled_, true);
  nh.param("mission/passage_centering/search_radius", passage_centering_search_radius_, 0.90);
  nh.param("mission/passage_centering/maximum_shift", passage_centering_maximum_shift_, 0.45);
  if (!std::isfinite(passage_centering_search_radius_) ||
      !std::isfinite(passage_centering_maximum_shift_) ||
      passage_centering_search_radius_ <= 0.0 || passage_centering_maximum_shift_ <= 0.0)
    throw std::runtime_error("passage_centering: search radius and shift must be positive");
  straight_run_extension_distance_ =
      std::max(0.0, straight_run_extension_distance_);
  straight_run_extension_min_turn_deg_ =
      std::max(10.0, std::min(90.0, straight_run_extension_min_turn_deg_));
  nh.param("mission/entry_arrive_dist", ep_->mission_entry_arrive_dist_, 0.6);
  nh.param("mission/entry_yaw", ep_->mission_entry_yaw_, 0.0);
  nh.param("mission/door_back_margin", ep_->mission_door_back_margin_, 0.25);
  nh.param("mission/forward_progress_weight", ep_->mission_forward_progress_weight_, 1.8);
  nh.param("mission/forward_min_gain", ep_->mission_forward_min_gain_, 0.25);
  nh.param("mission/forward_fallback_step", ep_->mission_forward_fallback_step_, 0.45);
  nh.param("mission/forward_fallback_max", ep_->mission_forward_fallback_max_, 1.8);
  std::string workspace_lock_topic;
  nh.param("mission/workspace_lock_topic", workspace_lock_topic,
           std::string("/corridor_search/workspace_lock"));
  std::vector<double> mission_entry_goal{0.0, 0.0, 0.8};
  std::vector<double> takeoff_exclusion_min{-100.0, -100.0, -100.0};
  std::vector<double> takeoff_exclusion_max{100.0, 100.0, 100.0};
  std::vector<double> search_region_min{-100.0, -100.0, -100.0};
  std::vector<double> search_region_max{100.0, 100.0, 100.0};
  nh.getParam("mission/entry_goal", mission_entry_goal);
  nh.getParam("mission/takeoff_exclusion_min", takeoff_exclusion_min);
  nh.getParam("mission/takeoff_exclusion_max", takeoff_exclusion_max);
  nh.getParam("mission/search_region_min", search_region_min);
  nh.getParam("mission/search_region_max", search_region_max);
  if (mission_entry_goal.size() != 3 || takeoff_exclusion_min.size() != 3 ||
      takeoff_exclusion_max.size() != 3 || search_region_min.size() != 3 ||
      search_region_max.size() != 3) {
    ROS_ERROR("[mission_exploration] mission box/vector params must all have exactly 3 elements.");
    mission_entry_goal = {0.0, 0.0, 0.8};
    takeoff_exclusion_min = {-100.0, -100.0, -100.0};
    takeoff_exclusion_max = {100.0, 100.0, 100.0};
    search_region_min = {-100.0, -100.0, -100.0};
    search_region_max = {100.0, 100.0, 100.0};
  }
  ep_->mission_entry_goal_ =
      Vector3d(mission_entry_goal[0], mission_entry_goal[1], mission_entry_goal[2]);
  ep_->mission_takeoff_exclusion_min_ =
      Vector3d(takeoff_exclusion_min[0], takeoff_exclusion_min[1], takeoff_exclusion_min[2]);
  ep_->mission_takeoff_exclusion_max_ =
      Vector3d(takeoff_exclusion_max[0], takeoff_exclusion_max[1], takeoff_exclusion_max[2]);
  ep_->mission_search_region_min_ =
      Vector3d(search_region_min[0], search_region_min[1], search_region_min[2]);
  ep_->mission_search_region_max_ =
      Vector3d(search_region_max[0], search_region_max[1], search_region_max[2]);
  workspace_lock_sub_ =
      nh.subscribe(workspace_lock_topic, 1, &FastExplorationManager::workspaceLockCallback, this);

  endpose_pub_ = nh.advertise<geometry_msgs::Point>("/chen/plan_pose", 10);
  cmu_pose_pub_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 10);
  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

bool FastExplorationManager::manualFappObservationMissing(
    const Vector3d& pos, double yaw, bool actual_odometry) {
  const bool manual = manual_path_enabled_ && manual_path_exploration_style_;
  if ((!manual && !fapp_exploration_crossing_enabled_) ||
      (manual && manual_path_wait_for_trigger_ && !manual_path_started_) ||
      !planner_manager_ || !planner_manager_->fappMemoryEnabled() || !task_search_manager_)
    return false;
  if (!manual && (!task_search_manager_->enabled() || task_search_manager_->stage3Active()))
    return false;
  Vector3d lower, upper;
  if (!manual_fapp_active_) {
    if (manual && manual_path_.finished()) return false;
    const Vector3d forward = task_search_manager_->recoveryForwardDirection(yaw).normalized();
    const double distance = manual ? std::min(underpass_lookahead_,
        (manual_path_.points[manual_path_.next] - pos).dot(forward)) : underpass_lookahead_;
    fuel_fapp::DynamicObstacleState observation;
    if (distance <= 0.15 || !planner_manager_->fappBlockingBounds(
            {pos, pos + distance * forward}, lower, upper, &observation))
      return false;
    manual_fapp_hold_height_ = task_search_manager_->clampSearchHeight(
        std::min(pos.z(), horizontalGoalHeight(pos.z())));
    manual_fapp_active_ = true;
    planner_manager_->watchFappObstacle(&observation);
    manual_fapp_forward_ = forward;
    manual_fapp_exit_progress_ = -std::numeric_limits<double>::infinity();
    ROS_WARN("[manual_fapp] watch region=(%.2f,%.2f,%.2f) before choosing underpass or side",
             observation.position.x(), observation.position.y(), observation.position.z());
  }
  double age = std::numeric_limits<double>::infinity();
  const bool known = planner_manager_->watchedFappBounds(lower, upper, age);
  if (known) {
    const double exit = (0.5 * (lower + upper)).dot(manual_fapp_forward_) +
        (0.5 * (upper - lower)).dot(manual_fapp_forward_.cwiseAbs()) +
        sdf_map_->getResolution();
    manual_fapp_exit_progress_ = std::max(manual_fapp_exit_progress_, exit);
  }
  const bool passed = known && actual_odometry &&
      pos.dot(manual_fapp_forward_) >= manual_fapp_exit_progress_ &&
      (!manual_fapp_curve_active_ ||
       pos.dot(manual_fapp_forward_) >= manual_fapp_curve_end_ - 0.02);
  // 失去新鲜帧不等于失去历史障碍：保留历史位置与最低点用于选路。
  // 不再仅因观测超过0.8秒而刹停并无限等待。
  const bool missing = !passed && !known;
  if (missing) {
    if (manual_fapp_wait_since_.isZero()) manual_fapp_wait_since_ = ros::Time::now();
    ROS_WARN_THROTTLE(1.0, "[manual_fapp] WAIT age=%.2fs underpass=%d phase=%d "
        "side=%d progress=%.2f exit=%.2f; retain waypoint and detour",
        age,
        static_cast<int>(suspended_underpass_active_), static_cast<int>(low_probe_phase_),
        static_cast<int>(recovery_side_latched_), pos.dot(manual_fapp_forward_),
        manual_fapp_exit_progress_);
    return true;
  }
  if (!manual_fapp_wait_since_.isZero()) {
    // 等待失观确认的时间不算作下穿执行/验证超时。
    const ros::Duration paused = ros::Time::now() - manual_fapp_wait_since_;
    for (ros::Time* timer : {&low_probe_verify_start_, &low_probe_descent_end_time_,
                            &low_probe_ascent_start_})
      if (!timer->isZero()) *timer += paused;
    manual_fapp_wait_since_ = ros::Time();
    ROS_WARN("[manual_fapp] RESUME reason=%s age=%.2fs",
        passed ? "passed" : "reobserved", age);
  }
  if (passed) {
    ROS_WARN("[manual_fapp] PASSED actual_progress=%.2f exit=%.2f",
        pos.dot(manual_fapp_forward_),
        manual_fapp_exit_progress_);
    planner_manager_->watchFappObstacle(nullptr);
    manual_fapp_active_ = false;
    fapp_rush_active_ = fapp_rush_pending_ = false;
    if (fapp_rush_pub_) { std_msgs::Bool mode; mode.data = false; fapp_rush_pub_.publish(mode); }
    recovery_side_latched_ = false;
    wide_side_bypass_active_ = false;
    recovery_side_continuation_pending_ = false;
    manual_fapp_hold_height_ = std::numeric_limits<double>::quiet_NaN();
    if (manual_fapp_curve_active_) {
      manual_fapp_curve_active_ = false;
      suspended_underpass_active_ = false;
    }
  }
  return false;
}

bool FastExplorationManager::buildManualFappUnderpassPath(
    const Vector3d& pos, const Vector3d& waypoint, std::vector<Vector3d>& path) {
  Vector3d lower, upper;
  double age;
  if (!planner_manager_->watchedFappBounds(lower, upper, age)) return false;
  Vector3d forward = manual_fapp_forward_;
  forward.z() = 0.0;
  if (forward.norm() < 1e-6) return false;
  forward.normalize();
  const Vector3d origin = manual_fapp_curve_active_ ? manual_fapp_curve_origin_ : pos;
  const double begin = origin.dot(forward);
  const double current = pos.dot(forward);
  const double center = (0.5 * (lower + upper)).dot(forward);
  const double extent = (0.5 * (upper - lower)).dot(forward.cwiseAbs());
  const double near = center - extent - sdf_map_->getResolution();
  const double far = center + extent + sdf_map_->getResolution();
  const double low = std::min(origin.z(), lower.z() - underpass_height_tracking_margin_);
  const double minimum = std::max(underpass_min_height_, task_search_manager_->clampSearchHeight(-1e3));
  if (low < minimum || (near - begin < 0.10 && origin.z() > low + 0.02)) {
    ROS_WARN_THROTTLE(1.0, "[fapp_underpass_check] reject geometry low=%.3f min=%.3f "
        "start_z=%.3f approach=%.3f lower_z=%.3f age=%.2f",
        low, minimum, origin.z(), near-begin, lower.z(), age);
    return false;
  }
  // 只生成前进下降和低位穿越，末端仍低位且超过已膨胀远端0.20m。
  // 回升交给实际越过后的下一次规划，后方回升受阻不能否决眼前可行的下穿。
  const double end = std::min(far + 0.20, waypoint.dot(forward));
  if (end <= far + 0.10 || end <= current + 0.03) {
    ROS_WARN_THROTTLE(1.0, "[fapp_underpass_check] reject endpoint current=%.3f far=%.3f end=%.3f",
        current, far, end);
    return false;
  }
  const auto smooth = [](double u) {
    u = std::max(0.0, std::min(1.0, u));
    return u*u*u*(10.0 + u*(-15.0 + 6.0*u));
  };
  // 先等高接近，在已扣除机体净空的障碍前沿约0.5m开始前进下降。
  const double descent_begin = std::max(begin, near - 0.50);
  const Vector3d lateral(-forward.y(), forward.x(), 0.0);
  const int count = std::max(2, static_cast<int>(std::ceil((end-current)/0.10)));
  const auto side_path = [&](double side, std::vector<Vector3d>& candidate) {
    candidate = {pos};
    for (int i = 1; i <= count; ++i) {
      const double progress = current + (end-current)*i/count;
      double dip = 1.0;
      if (progress < near) dip = smooth((progress-descent_begin)/(near-descent_begin));
      // 左下/右下穿越：接近时同时向侧下方移动，保持侧下方直到实际通过。
      Vector3d point = origin + (progress-begin)*forward + side*dip*lateral;
      point.z() = origin.z() + (low-origin.z())*dip;
      if (!planner_manager_->isKnownFreePosition(point)) {
        ROS_WARN_THROTTLE(1.0, "[fapp_underpass_check] reject known-free side=%.2f "
            "sample=%d/%d xyz=(%.3f,%.3f,%.3f) low=%.3f static_safe=%d center_occ=%d "
            "near=%.3f far=%.3f end=%.3f",
            side, i, count, point.x(), point.y(), point.z(), low,
            static_cast<int>(planner_manager_->isPositionSafe(point)),
            sdf_map_->getOccupancy(point), near, far, end);
        return false;
      }
      candidate.push_back(point);
    }
    if (!planner_manager_->isPathSafe(candidate)) {
      ROS_WARN_THROTTLE(1.0, "[fapp_underpass_check] reject static path side=%.2f low=%.3f", side, low);
      return false;
    }
    if (!planner_manager_->isFappPathSafe(candidate)) {
      ROS_WARN_THROTTLE(1.0, "[fapp_underpass_check] reject dynamic path side=%.2f low=%.3f", side, low);
      return false;
    }
    return true;
  };
  double side = manual_fapp_curve_side_;
  if (manual_fapp_curve_active_) {
    if (!side_path(side, path)) return false;
  } else {
    // 只尝试左右两个侧下方通路；选定后保持方向，不逐帧反复换边。
    side = recovery_side_latched_ && recovery_side_dir_.dot(lateral) < 0.0 ? -0.20 : 0.20;
    if (!side_path(side, path)) {
      if (manual_fapp_active_ && recovery_side_latched_) return false;
      side = -side;
      if (!side_path(side, path)) return false;
    }
  }
  if (!manual_fapp_curve_active_)
    ROS_WARN("[manual_fapp] continuous underpass from=(%.2f,%.2f,%.2f) "
             "clearance_z=%.2f low=%.2f near=%.2f far=%.2f side=%.2f age=%.2f descent_begin=%.2f",
        pos.x(), pos.y(), pos.z(), lower.z(), low, near, far, side, age, descent_begin);
  manual_fapp_curve_active_ = true;
  manual_fapp_curve_origin_ = origin;
  manual_fapp_curve_end_ = end;
  manual_fapp_curve_side_ = side;
  suspended_underpass_active_ = true;
  suspended_underpass_blocked_ = false;
  low_probe_height_ = low;
  low_probe_return_height_ = origin.z();
  low_probe_exit_progress_ = far;
  return true;
}

bool FastExplorationManager::keepsDynamicSide(const std::vector<Vector3d>& path) const {
  if (!manual_fapp_active_ || !recovery_side_latched_ || path.empty()) return true;
  const Vector3d lateral(-manual_fapp_forward_.y(), manual_fapp_forward_.x(), 0.0);
  if (std::fabs(recovery_side_dir_.dot(lateral)) < 0.5) return true;
  const Vector3d side = (recovery_side_dir_.dot(lateral) < 0.0 ? -1.0 : 1.0) * lateral;
  for (const auto& point : path)
    if (point.dot(manual_fapp_forward_) <= manual_fapp_exit_progress_ &&
        (point - path.front()).dot(side) < -sdf_map_->getResolution())
      return false;
  return true;
}

bool FastExplorationManager::planFappOpeningCrossing(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  fapp_rush_pending_ = false;
  if (!manual_fapp_active_ || !recovery_side_latched_) return false;
  const Vector3d forward = manual_fapp_forward_;
  const Vector3d lateral(-forward.y(), forward.x(), 0.0);
  if (std::fabs(recovery_side_dir_.dot(lateral)) < 0.5) return false;
  const Vector3d side = (recovery_side_dir_.dot(lateral) < 0.0 ? -1.0 : 1.0) * lateral;
  if (!fapp_rush_active_ && !planner_manager_->fappSideOpening(pos, side)) return false;
  Vector3d lower, upper; double age;
  if (!planner_manager_->watchedFappBounds(lower, upper, age)) return false;
  double end = manual_fapp_exit_progress_ + 0.20;
  if (manual_path_enabled_) {
    if (manual_path_.finished()) return false;
    end = std::min(end, manual_path_.points[manual_path_.next].dot(forward));
    if (end < manual_fapp_exit_progress_) return false;
  }
  if (end - pos.dot(forward) <= 0.10) return false;
  Vector3d target = pos + (end - pos.dot(forward)) * forward;
  target.z() = horizontalGoalHeight(pos.z());
  vector<Vector3d> path{pos, 0.5 * (pos + target), target};
  if (!pathInsideWorkspaceLock(path) || !task_search_manager_->isRecoveryPathAllowed(path, false) ||
      !planner_manager_->isPathSafe(path)) return false;
  // 同侧直穿到历史包络后方，保持高度；不用整段摆幅否决当前已让开的通路。
  for (int sample=0; sample<=40; ++sample)
    if (!planner_manager_->isKnownFreePosition(pos + (sample/40.0)*(target-pos))) return false;
  if (!planner_manager_->planExploreTraj(path, vel, acc, 0.0, true, 1.0, 1.0) ||
      !planner_manager_->isTrajectorySafe() || !planner_manager_->isFappCrossingTrajectorySafe() ||
      !planner_manager_->planYawTurnInPlace(yaw, yaw[0])) return false;
  manual_fapp_curve_active_ = false;
  suspended_underpass_active_ = false;
  ed_->path_next_goal_ = path;
  ed_->global_tour_ = {pos, target};
  ed_->next_goal_ = target;
  last_requested_goal_ = target;
  has_last_requested_goal_ = true;
  pending_reference_height_ = target.z();
  recovery_side_continuation_pending_ = false;
  fapp_rush_pending_ = true; // 仅发布成功后通知执行端，候选失败不改变限速。
  ROS_WARN("[fapp_rush] keep selected side and cross opening: target=(%.2f,%.2f,%.2f) "
           "max_vel=1.00m/s max_acc=1.00m/s^2; restore after actual passage.", target.x(), target.y(), target.z());
  return true;
}

bool FastExplorationManager::planExplorationFappCrossing(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  if (!fapp_exploration_crossing_enabled_ || manual_path_enabled_ || external_selection_only_ ||
      !task_search_manager_ || !task_search_manager_->enabled() ||
      task_search_manager_->stage3Active() || task_search_manager_->landingRequested() ||
      turn_in_place_session_active_ || low_probe_phase_ != vertical_detour::LowProbePhase::IDLE)
    return false;
  manualFappObservationMissing(pos, yaw[0]);
  if (!manual_fapp_active_) return false;
  // 临时失败不消费穿越机会；执行普通入口/前进段时，FSM 仍可重新尝试。
  fapp_crossing_retry_needed_ = true;
  if (planFappOpeningCrossing(pos, vel, acc, yaw)) {
    fapp_crossing_retry_needed_ = false;
    return true;
  }
  Vector3d lower, upper;
  double age;
  if (!planner_manager_->watchedFappBounds(lower, upper, age)) return false;
  const Vector3d forward = manual_fapp_forward_;
  const double far = (0.5 * (lower + upper)).dot(forward) +
      (0.5 * (upper - lower)).dot(forward.cwiseAbs()) + sdf_map_->getResolution();
  // 自动探索先低位越过历史障碍；无需提前把后方回升空间纳入同一条曲线。
  const double end = far + 0.20;
  Vector3d target = pos + (end - pos.dot(forward)) * forward;
  std::vector<Vector3d> path;
  double target_yaw = yaw[0];
  // 普通绕障的左右偏好不能封死下穿：每轮仍先检查侧下方通路。
  bool crossing = buildManualFappUnderpassPath(pos, target, path);
  if (!crossing) {
    manual_fapp_curve_active_ = false;
    suspended_underpass_active_ = false;
    bool obstacle = false;
    crossing = buildFappSideBypass(pos, yaw[0], forward, target, target_yaw, obstacle, &path);
  }
  if (!crossing || !pathInsideWorkspaceLock(path) ||
      !task_search_manager_->isRecoveryPathAllowed(path, false) ||
      !planner_manager_->isPathSafe(path) ||
      !planner_manager_->isFappPathSafe(path) ||
      !planner_manager_->planExploreTraj(path, vel, acc, 0.0, false, fapp_rush_active_ ? 1.0 : 0.0, fapp_rush_active_ ? 1.0 : 0.0) ||
      !planner_manager_->isTrajectorySafe() ||
      !planner_manager_->planYawTurnInPlace(yaw, target_yaw)) {
    // 避让候选不可用时继续原探索选路，不能封死普通前进规划。
    manual_fapp_curve_active_ = false;
    suspended_underpass_active_ = false;
    return false;
  }
  // 完整动态穿越沿用航点版长度豁免，不进入普通探索的1.2m截断。
  ed_->path_next_goal_ = path;
  ed_->global_tour_ = {pos, path.back()};
  ed_->next_goal_ = path.back();
  last_requested_goal_ = path.back();
  has_last_requested_goal_ = true;
  pending_reference_height_ = path.back().z();
  recovery_side_continuation_pending_ = false;
  fapp_crossing_retry_needed_ = false;
  ROS_WARN("[exploration_fapp] continuous crossing target=(%.2f,%.2f,%.2f) "
           "curve=%d side=%d; complete crossing is exempt from route length limit.",
           path.back().x(), path.back().y(), path.back().z(),
           static_cast<int>(manual_fapp_curve_active_), static_cast<int>(recovery_side_latched_));
  return true;
}

int FastExplorationManager::planManualWaypointMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  if (manual_path_wait_for_trigger_ && !manual_path_started_) return FAIL;
  if (task_search_manager_ && task_search_manager_->landingRequested()) return NO_FRONTIER;
  if (manualFappObservationMissing(pos, yaw[0])) return FAIL;
  if (planFappOpeningCrossing(pos, vel, acc, yaw)) return SUCCEED;
  Vector3d target;
  double target_yaw = yaw[0];
  const bool exit_motion = manual_exit_enabled_ && manual_exit_latched_ && manual_path_.finished();
  if (exit_motion) {
    task_search_manager_->resumeDeferredExitCrossing();
    // 出口阶段交接只根据真实里程计，不能用预测起点提前把控制权交给降落。
    if (!task_search_manager_->buildStage3Goal(manual_actual_pos_, yaw[0], false,
                                               target, target_yaw))
      return task_search_manager_->exitTransitActive() ? FAIL : NO_FRONTIER;
    ed_->path_next_goal_ = {pos, target};
    ed_->global_tour_ = {pos, target};
  } else if (manual_path_.finished()) {
    if (manual_exit_enabled_) {
      ROS_INFO_THROTTLE(1.0, "[manual_path] route complete; waiting for confirmed exit.");
      return FAIL;
    }
    ROS_INFO_THROTTLE(1.0, "[manual_path] all specified waypoints completed; finish route.");
    return NO_FRONTIER;
  } else {
    target = manual_path_.points[manual_path_.next];
    ed_->path_next_goal_ = manual_path_.path(pos);
    ed_->global_tour_ = {pos};
    ed_->global_tour_.insert(ed_->global_tour_.end(),
        manual_path_.points.begin() + manual_path_.next, manual_path_.points.end());
    const Eigen::Vector2d direction = target.head<2>() - pos.head<2>();
    if (direction.norm() > 0.15) target_yaw = std::atan2(direction.y(), direction.x());
  }
  const bool hold_fapp_height = !exit_motion && manual_fapp_active_ &&
      std::isfinite(manual_fapp_hold_height_);
  if (hold_fapp_height) {
    target.z() = manual_fapp_hold_height_;
    for (size_t i=1; i<ed_->path_next_goal_.size(); ++i)
      ed_->path_next_goal_[i].z() = target.z();
  }
  ed_->next_goal_ = target;
  last_requested_goal_ = target;
  has_last_requested_goal_ = true;
  if (external_selection_only_) {
    external_selected_viewpoint_ = target;
    external_selected_yaw_ = target_yaw;
    return SUCCEED;
  }
  bool dynamic_crossing_length_exempt = false;
  const auto trajectory_within_limit = [&]() {
    if (dynamic_crossing_length_exempt || manual_path_max_segment_length_ <= 0.0) return true;
    // 控制多边形长度是B样条弧长上界，包含优化后的实际曲线。
    const auto controls = planner_manager_->local_data_.position_traj_.getControlPoint();
    double bound = 0.0;
    for (int i = 1; i < controls.rows(); ++i)
      bound += (controls.row(i) - controls.row(i - 1)).norm();
    return bound <= manual_path_max_segment_length_ + 1e-6;
  };
  const Vector3d waypoint = target;
  bool underpass_motion = false;
  if (manual_path_exploration_style_ && !exit_motion && task_search_manager_) {
    if (manual_fapp_active_) {
      underpass_motion = buildManualFappUnderpassPath(pos, waypoint, ed_->path_next_goal_);
      if (!underpass_motion) {
        // 动态下穿不再退回原地竖直下探；沿用已有的侧绕选择。
        manual_fapp_curve_active_ = false;
        suspended_underpass_active_ = false;
        suspended_underpass_blocked_ = true;
      }
    } else if (!manual_fapp_active_) {
      if (low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
          !recovery_side_latched_)
        beginSuspendedUnderpass(pos, yaw[0]);
      bool waiting = false;
      underpass_motion = handleActiveLowProbe(pos, target, target_yaw, waiting);
      if (waiting) return FAIL;
      if (underpass_motion &&
          (low_probe_phase_ == vertical_detour::LowProbePhase::DESCENDING ||
           low_probe_phase_ == vertical_detour::LowProbePhase::ASCENDING))
        return buildStationaryVerticalDetourPlan(pos, yaw) && trajectory_within_limit()
            ? SUCCEED : FAIL;
      if (underpass_motion) ed_->path_next_goal_ = {pos, target};
    }
  }
  // 与探索相同：地图确认转弯后先停转，不能等远航点A*成功后才允许转机头。
  if (manual_path_exploration_style_ && !exit_motion && !underpass_motion && task_search_manager_) {
    Vector3d direction;
    if (turn_in_place_session_active_)
      direction = Vector3d(std::cos(turn_in_place_final_yaw_),
                           std::sin(turn_in_place_final_yaw_), 0.0);
    if (turn_in_place_session_active_ ||
        task_search_manager_->mappedCorridorDirection(yaw[0], direction, false))
      return buildTurnInPlacePlan(pos, vel, acc, yaw, direction) &&
             trajectory_within_limit() ? SUCCEED : FAIL;
  }
  if (ed_->path_next_goal_.size() < 2 || (target - pos).norm() < 0.03)
    return FAIL;  // 等待实际里程计到点，预测规划起点不能推进列表。
  bool bypass_motion = false;
  if (manual_path_use_astar_ && !underpass_motion) {
    auto& finder = planner_manager_->path_finder_;
    if (!finder) return FAIL;
    finder->clearProgressConstraint();
    if (manual_path_exploration_style_ && !exit_motion && task_search_manager_) {
      const Vector3d forward = task_search_manager_->recoveryForwardDirection(yaw[0]);
      // 复用探索原有“选侧横移 -> 同侧短步通过 -> 释放后回正”。
      // 绕障只改变局部端点，不记录为用户航点，也不绕到远目标后面。
      if ((target - pos).dot(forward) > 0.15 &&
          (recovery_side_latched_ || suspended_underpass_blocked_ ||
           !planner_manager_->isPathSafe({pos, target}) ||
           !planner_manager_->isFappPathSafe({pos, target}))) {
        bool obstacle_detected = false;
        bool dynamic_obstacle = false;
        if (recovery_side_latched_)
          bypass_motion = buildSideRecoveryContinuation(pos, yaw[0], target, target_yaw);
        if (!bypass_motion)
          bypass_motion = buildFappSideBypass(pos, yaw[0], forward,
                                              target, target_yaw, dynamic_obstacle);
        // 摆球也可能留在静态图内，先按动态包围盒选侧，避免较小静态足迹的候选反复被否决。
        if (!bypass_motion && !dynamic_obstacle)
          bypass_motion = buildWideSideBypass(pos, yaw[0], forward,
                                              target, target_yaw, obstacle_detected);
        if (!bypass_motion && (dynamic_obstacle || suspended_underpass_blocked_)) {
          // 完整穿越暂未找到，仍允许下面的A*与短前缀生成接近轨迹。
          // 候选前缀和实际样条继续检查历史障碍，不提前否决整条航点路线。
          ROS_WARN_THROTTLE(1.0, "[manual_path] crossing unavailable; try normal path and short approach.");
        }
      }
      if (!bypass_motion) {
        target_yaw = std::atan2(forward.y(), forward.x());
        task_search_manager_->corridorCurveYaw(pos, target_yaw);
      }
    }
    const auto find_path = [&](const Vector3d& goal) {
      if (!planner_manager_->isPositionSafe(goal)) return false;
      finder->reset();
      if (finder->search(pos, goal, hold_fapp_height) != Astar::REACH_END) return false;
      // A*同栅格可能省略真实端点；保留精确XYZ并去除零长分段。
      ed_->path_next_goal_ = {pos};
      for (const auto& point : finder->getPath())
        if ((point - ed_->path_next_goal_.back()).norm() >= 1e-3)
          ed_->path_next_goal_.push_back(point);
      if ((goal - ed_->path_next_goal_.back()).norm() >= 1e-3)
        ed_->path_next_goal_.push_back(goal);
      else
        ed_->path_next_goal_.back() = goal;
      shortenPath(ed_->path_next_goal_);
      return true;
    };
    bool found = find_path(target);
    // 远点尚未连通时，只尝试朝该点的已知自由短段；不生成frontier探索目标。
    if (!found && manual_path_exploration_style_ && !bypass_motion &&
        manual_path_max_segment_length_ > 0.0) {
      const Vector3d delta = waypoint - pos;
      const double horizon = std::min(manual_path_max_segment_length_, delta.norm());
      for (double scale : {1.0, 2.0 / 3.0, 1.0 / 3.0}) {
        const Vector3d local = pos + delta.normalized() * horizon * scale;
        if (!planner_manager_->isKnownFreePosition(local) || !find_path(local)) continue;
        target = local;
        found = true;
        ROS_WARN("[manual_path] remote waypoint disconnected; use known-free short approach.");
        break;
      }
    }
    if (!found) {
      ROS_WARN_THROTTLE(1.0, "[manual_path] no known free path to waypoint or local segment; keep target.");
      return FAIL;
    }
  }
  // 动态穿越完整覆盖障碍区域；普通航点、出口和静态绕行仍遵守局部段上限。
  dynamic_crossing_length_exempt = !exit_motion && manual_fapp_active_ &&
      ((underpass_motion && manual_fapp_curve_active_) || bypass_motion);
  if (!keepsDynamicSide(ed_->path_next_goal_)) return FAIL;
  const auto full_path = ed_->path_next_goal_;
  double budget = !dynamic_crossing_length_exempt && manual_path_max_segment_length_ > 0.0
      ? std::min(manual_path_max_segment_length_, Astar::pathLength(full_path))
      : Astar::pathLength(full_path);
  bool planned = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    ed_->path_next_goal_ = dynamic_crossing_length_exempt
        ? full_path : manualPathPrefix(full_path, budget);
    if (ed_->path_next_goal_.size() < 2) break;
    // 曲线按弧长截取时，末点可能离前一点不足毫米，合并以满足样条输入要求。
    if (ed_->path_next_goal_.size() > 2 &&
        (ed_->path_next_goal_.back() - ed_->path_next_goal_[ed_->path_next_goal_.size()-2]).norm() < 1e-3)
      ed_->path_next_goal_.erase(ed_->path_next_goal_.end()-2);
    target = ed_->path_next_goal_.back();
    if (ed_->path_next_goal_.size() == 2)
      ed_->path_next_goal_.insert(ed_->path_next_goal_.begin() + 1, 0.5 * (pos + target));
    if (planner_manager_->isPositionSafe(target) &&
        planner_manager_->isPathSafe(ed_->path_next_goal_) &&
        (!(manual_path_exploration_style_ && !exit_motion) ||
         planner_manager_->fappPlannerPriority() ||
         planner_manager_->isFappPathSafe(ed_->path_next_goal_)) &&
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, 0.0,
                                          hold_fapp_height && !underpass_motion, fapp_rush_active_ ? 1.0 : 0.0, fapp_rush_active_ ? 1.0 : 0.0) &&
        planner_manager_->isTrajectorySafe()) {
      if (trajectory_within_limit()) {
        planned = true;
        break;
      }
    }
    // 同探索的短前缀重规划：不发布切墙曲线，缩短后仍走同一条已知路径。
    if (dynamic_crossing_length_exempt || manual_path_max_segment_length_ <= 0.0) break;
    budget *= 2.0 / 3.0;
  }
  if (!planned) {
    ROS_ERROR_THROTTLE(1.0, "[manual_path] trajectory rejected; keep the specified waypoint.");
    return FAIL;
  }
  ed_->next_goal_ = target;
  pending_reference_height_ = target.z();
  // 恒定航向复用插值器，避免全零控制点进入旧航向优化器的归一化计算。
  if (std::fabs(target_yaw - yaw[0]) < 1e-6 && yaw.tail<2>().norm() < 1e-6) {
    if (!planner_manager_->planYawTurnInPlace(yaw, target_yaw)) return FAIL;
  } else {
    planner_manager_->planYawExplore(yaw, target_yaw, !manual_path_exploration_style_, ep_->relax_time_);
  }
  recovery_side_continuation_pending_ = false;
  ROS_INFO("[manual_path] %s local=(%.2f, %.2f, %.2f), waypoint=(%.2f, %.2f, %.2f), "
           "bypass=%d, completed=%zu/%zu.",
           exit_motion ? "exit approach" : "waypoint", target.x(), target.y(), target.z(),
           waypoint.x(), waypoint.y(), waypoint.z(), static_cast<int>(bypass_motion),
           manual_path_.next, manual_path_.points.size());
  return SUCCEED;
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  width_route_plan_ = false;
  fapp_rush_pending_ = false;
  pending_reference_height_ = std::numeric_limits<double>::quiet_NaN();
  // 上轮同侧短步若未生成成功，不在下一轮反复强选同一点。
  if (recovery_side_continuation_pending_ && !manual_fapp_active_) {
    recovery_side_latched_ = false;
    wide_side_bypass_active_ = false;
  }
  recovery_side_continuation_pending_ = false;
  turn_in_place_plan_ = false;
  ground_ascent_plan_ = false;
  vertical_detour_plan_ = false;
  pending_short_backtrack_ = false;
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  if (manual_path_enabled_) return planManualWaypointMotion(pos, vel, acc, yaw);

  updateMissionRegionState(pos);
  // 2026-07-22: 真实航迹已由FSM里程计回调连续写入；这里的pos可能是未来重规划起点，禁止重复记入历史。

  Vector3d next_pos;
  double next_yaw = yaw[0];
  bool use_forced_entry_target = false;
  bool use_stage3_target = false;
  bool use_vertical_detour_target = false;
  bool use_mission_forward_fallback = false;
  bool use_center_gate_target = false;
  bool use_entry_forward_target = false;
  if (task_search_manager_ && task_search_manager_->landingRequested()) {
    // 2026-07-13: AUTO.LAND 已由控制器接管后，探索器不再发布新的平移轨迹。
    ROS_WARN_THROTTLE(1.0, "[exit_mission] landing is active; exploration planning stopped.");
    return NO_FRONTIER;
  }
  planner_manager_->path_finder_->clearProgressConstraint();
  if (planExplorationFappCrossing(pos, vel, acc, yaw)) return SUCCEED;
  const bool hold_fapp_height = manual_fapp_active_ && std::isfinite(manual_fapp_hold_height_);
  if (hold_fapp_height)
    ROS_WARN_THROTTLE(1.0, "[exploration_fapp] ordinary fallback holds z=%.3f until actual "
        "progress clears %.3f; continue forward/side planning.",
        manual_fapp_hold_height_, manual_fapp_exit_progress_);
  // 已确认的悬挂障碍优先于普通前沿选点、地面恢复和远目标 A*。
  if (!hold_fapp_height && low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
      beginSuspendedUnderpass(pos, yaw[0]) && suspended_underpass_blocked_)
    return FAIL;
  // 地图确认的转弯拥有最高运动优先级。若此前刚启动低位探测，立即撤销其
  // 固定XY/高度目标，避免下穿状态阻塞已经确认的新通道转向。
  if (turn_in_place_session_active_ && !suspended_underpass_active_) {
    cancelActiveLowProbe("confirmed mapped turn has higher motion priority");
    const Vector3d locked_direction(std::cos(turn_in_place_final_yaw_),
                                    std::sin(turn_in_place_final_yaw_), 0.0);
    return buildTurnInPlacePlan(pos, vel, acc, yaw, locked_direction) ? SUCCEED : FAIL;
  }
  // 先生成并验证本轮平移目标，再在轨迹发布前检查是否为真实弯道。局部左右绕障
  // 必须先拥有解释权，不能在A*尚未选路时被独立占据射线提前改写通道方向。
  // 低空探测状态具有XY所有权。状态结束或被显式取消前，膨胀层历史逃逸不得
  // 沿旧航迹横移，从而避免原地上升目标被带到数米之外仍继续生效。
  bool wait_for_low_confirmation = false;
  use_vertical_detour_target = handleActiveLowProbe(
      pos, next_pos, next_yaw, wait_for_low_confirmation);
  const bool low_probe_active =
      low_probe_phase_ != vertical_detour::LowProbePhase::IDLE;
  if (wait_for_low_confirmation) return FAIL;
  if (suspended_underpass_active_ && use_vertical_detour_target &&
      low_probe_phase_ == vertical_detour::LowProbePhase::ADVANCING)
    use_center_gate_target = true;
  // 有安全前进/同侧短路时先沿路恢复高度；不可继续水平推进时保留原地上升。
  // 已经启动的竖直会话仍执行完，不在中途重新抢占位置目标。
  if (!hold_fapp_height && !use_vertical_detour_target && !low_probe_active &&
      (ground_ascent_active_ || (shouldStartGroundAscent(pos) &&
                                !hasSafeCruiseContinuation(pos, yaw[0])))) {
    if (buildGroundAscentPlan(pos, vel, yaw)) return SUCCEED;
    if (ground_ascent_active_) return FAIL;
  }
  if (!hold_fapp_height && !external_selection_only_ && !use_vertical_detour_target && !low_probe_active &&
      vertical_detour::inflationEscapeAllowed(low_probe_phase_) &&
      planInflationHistoryEscape(pos, vel, acc, yaw))
    return SUCCEED;
  // 刚穿入口时按门法向累计至少 1m 的前进位移；1.05m仅是选点余量，已知自由段可分多次
  // 发布，但不能把 A* 弯折弧长或优化器的往返路程当成向前推进距离。
  if (!use_vertical_detour_target && !low_probe_active &&
      mission_workspace_lock_received_ && !entry_forward_complete_ &&
      !mission_entered_search_region_ &&
      task_search_manager_ && task_search_manager_->enabled() &&
      !task_search_manager_->hasConfirmedCorridorTurn() &&
      mission_workspace_dir_.head<2>().norm() > 1e-3) {
    const Vector3d forward = mission_workspace_dir_.normalized();
    const double entry_progress = (pos - mission_workspace_origin_).dot(forward);
    if (entry_forward::needsAdvance(entry_progress)) {
      const double remaining = 1.05 - entry_progress;
      entry_forward::Goal selected;
      if (selectEntryForwardGoal(pos, forward, remaining, selected)) {
        next_pos = selected.position;
        next_yaw = std::atan2(forward.y(), forward.x());
        use_mission_forward_fallback = true;
        use_center_gate_target = true;
        use_entry_forward_target = true;
        ed_->global_tour_ = {pos, next_pos};
        ROS_WARN("[entry_forward] known-free advance %.2fm side=%.2fm, total forward=%.2fm "
                 "clearance=%.2fm continuation=%.2fm target=(%.2f,%.2f,%.2f).",
                 selected.advance, selected.side, entry_progress + selected.advance,
                 selected.clearance, selected.continuation,
                 next_pos.x(), next_pos.y(), next_pos.z());
      }
    }
  }
  // 除专用膨胀层逃逸外，普通搜索把禁回头直接放进A*展开。这样A*会寻找横移/斜前
  // 绕障路线，不会先返回一条略向后的宽路、再被任务层整条丢弃。
  Eigen::Vector3d astar_progress_direction;
  if (!use_vertical_detour_target && task_search_manager_ &&
      task_search_manager_->astarNoReturnDirection(astar_progress_direction)) {
    planner_manager_->path_finder_->setProgressConstraint(astar_progress_direction, 0.0);
  }
  if (!use_vertical_detour_target && !use_entry_forward_target &&
      shouldUseMissionEntryTransit(pos)) {
    // 2026-07-08 19:26: 在真正进入搜索区前，先强制把无人机送到门口/作业区入口，不再允许 frontier 收益把它绑在起飞区。
    use_forced_entry_target = true;
    next_pos = ep_->mission_entry_goal_;
    next_yaw = ep_->mission_prefer_entry_heading_ ? atan2(next_pos.y() - pos.y(), next_pos.x() - pos.x())
                                                  : ep_->mission_entry_yaw_;
    ROS_WARN_THROTTLE(1.0,
                      "[mission_exploration] force entry transit to %.2f %.2f %.2f before frontier search.",
                      next_pos.x(), next_pos.y(), next_pos.z());
  }
  if (!use_vertical_detour_target && !use_forced_entry_target && task_search_manager_) {
    // 双侧开阔出口允许接管时，出口目标优先于普通frontier和通道中门目标。
    use_stage3_target =
        task_search_manager_->buildStage3Goal(pos, yaw[0], false, next_pos, next_yaw);
  }

  vector<Vector3d> width_selected_path;
  if (!pending_wider_route_.empty()) {
    width_selected_path.swap(pending_wider_route_);
    if (!use_vertical_detour_target && !use_forced_entry_target && !use_stage3_target &&
        !use_entry_forward_target && !low_probe_active) {
      // Trim the already-traversed prefix before reconnecting at the predicted
      // start; the entire connection is checked again on the current map.
      const Vector3d forward = task_search_manager_->recoveryForwardDirection(yaw[0]);
      while (width_selected_path.size() > 1 &&
             (width_selected_path[1] - pos).dot(forward) <= 0.0)
        width_selected_path.erase(width_selected_path.begin());
      width_selected_path.front() = pos;
      if (width_selected_path.size() < 2 ||
          !pathInsideWorkspaceLock(width_selected_path) ||
          !planner_manager_->isPathSafe(width_selected_path) ||
          !task_search_manager_->isRecoveryPathAllowed(width_selected_path, false)) return FAIL;
      next_pos = width_selected_path.back();
      width_route_plan_ = true;
      next_yaw = yaw[0];
      use_mission_forward_fallback = true;
      ed_->global_tour_ = {pos, next_pos};
      task_search_manager_->recordSelectedGoal(next_pos);
    } else width_selected_path.clear();
  }

  // 优先延续当前可行绕行侧；原侧不可行或已通过后，才按当前通路间距重新选侧。
  // 不抢占入口、出口和已启动的垂直避障。
  if (!use_vertical_detour_target && !use_forced_entry_target &&
      !use_stage3_target && !use_mission_forward_fallback &&
      (mission_entered_search_region_ || entry_forward_complete_) && task_search_manager_ &&
      task_search_manager_->enabled()) {
    bool current_split_detected = false;
    const Vector3d forward = task_search_manager_->recoveryForwardDirection(yaw[0]);
    if (ep_->mission_use_forward_fallback_ && !task_search_manager_->stage3Active() &&
        !task_search_manager_->finalExitFrontierGuardActive() &&
        low_probe_phase_ == vertical_detour::LowProbePhase::IDLE &&
        buildWideSideBypass(pos, yaw[0], forward, next_pos, next_yaw,
                            current_split_detected)) {
      use_mission_forward_fallback = true;
      // buildWideSideBypass 内部也可能返回已检查的同侧直线续接。
      use_center_gate_target = recovery_side_continuation_pending_;
    } else if (buildSideRecoveryContinuation(pos, yaw[0], next_pos, next_yaw)) {
      use_mission_forward_fallback = true;
      use_center_gate_target = true;  // 仅同侧直线短步复用中门路径入口。
    }
  }

  // 两侧局部障碍已经形成中门时，先在当前高度穿过已知安全的门中心；
  // 中门不满足足迹通行条件时先尝试当前高度的侧绕，再尝试带顶部净空约束的上绕。
  // 专用通路均不可用时继续普通选点和A*，不能用局部中缝失败否决其他绕行路线。
  bool center_gate_detected = false;
  if (!use_vertical_detour_target && !use_forced_entry_target &&
      !use_stage3_target && !use_mission_forward_fallback &&
      mission_entered_search_region_ &&
      task_search_manager_ && task_search_manager_->enabled()) {
    const Vector3d recovery_forward =
        task_search_manager_->recoveryForwardDirection(yaw[0]);
    if (buildCenterGatePassage(pos, yaw[0], recovery_forward, next_pos,
                               next_yaw, center_gate_detected)) {
      use_mission_forward_fallback = true;
      use_center_gate_target = true;
    } else if (center_gate_detected) {
      bool side_obstacle_detected = false;
      if (buildWideSideBypass(pos, yaw[0], recovery_forward, next_pos,
                              next_yaw, side_obstacle_detected)) {
        use_mission_forward_fallback = true;
      } else if (buildVerticalDetourFallback(pos, yaw[0], recovery_forward, next_pos,
                                      next_yaw, false, false)) {
        use_vertical_detour_target = true;
      } else {
        ROS_WARN_THROTTLE(
            0.5,
            "[center_gate] dedicated center/horizontal passages are unavailable and "
            "no upper route keeps at least %.2fm above the obstacle top; "
            "continue frontier and A* route search.",
            vertical_detour_upper_obstacle_clearance_);
      }
    }
  }

  // 任务搜索每轮都从当前位置和本轮更新地图重新生成frontier。旧位置缓存不能继续参与
  // 选点；暂时不可达点仍只由任务层短时冷却，不永久删除地图区域。
  if (!use_vertical_detour_target && !use_forced_entry_target && !use_stage3_target &&
      !use_mission_forward_fallback &&
      task_search_manager_) {
    frontier_finder_->clearFrontierHistory();
    // 保留活动目标的短时保持；只有到达或实际失败时才清除，避免每次重规划都抖动选点。
  }

  double frontier_time = 0.0;
  // 入口正式触发后才搜索frontier；等待入口期间不预计算或缓存候选。
  frontier_finder_->searchFrontiers();
  frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  // 2026-07-20: 不能仅凭原始frontier非空就取消搜索耗尽计时；全部候选都被重复访问、
  // 失败冷却或门平面约束过滤时并没有可执行路径，旧逻辑会让FSM原地空转十几秒。
  if (!use_vertical_detour_target && !use_forced_entry_target && !use_stage3_target &&
      !use_mission_forward_fallback &&
      task_search_manager_ &&
      task_search_manager_->buildStage3Goal(pos, yaw[0], false, next_pos, next_yaw)) {
    // 本轮地图确认双侧开阔且任务允许接管时切入穿越，无需等待frontier耗尽。
    use_stage3_target = true;
  }
  const bool use_mission_filtered_direct_view =
      !use_forced_entry_target && !use_stage3_target &&
      (ep_->mission_use_takeoff_exclusion_box_ || ep_->mission_prefer_search_region_frontiers_ ||
       ep_->mission_use_workspace_lock_);
  if (!use_vertical_detour_target && !use_forced_entry_target && !use_stage3_target) {
    applyMissionFrontierFilter();
  }

  if (!use_vertical_detour_target && !use_forced_entry_target && !use_stage3_target &&
      !use_mission_forward_fallback &&
      (ed_->frontiers_.empty() || ed_->points_.empty())) {
    ROS_WARN("No coverable frontier after mission filter.");
    // 2026-07-13: 搜索候选耗尽时优先进入地图出口阶段，不能继续在通道内盲目前探。
    if (task_search_manager_ &&
        task_search_manager_->buildStage3Goal(pos, yaw[0], true, next_pos, next_yaw)) {
      use_stage3_target = true;
      ed_->global_tour_ = {pos, next_pos};
      ed_->refined_tour_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
    // 2026-07-20: 已确认远端终点时，无frontier分支同样不能用无约束前探绕过
    // distance_to_exit保护；终点未确认时仍完整保留原前探和U形掉头能力。
    } else if ((!task_search_manager_ ||
                !task_search_manager_->finalExitFrontierGuardActive()) &&
               buildMissionForwardFallback(pos, yaw[0], next_pos, next_yaw)) {
      // 2026-07-10: 门后 frontier 被过滤为空时，先沿动态门方向短距离前探，给雷达制造新的可见地图和搜索空间。
      use_mission_forward_fallback = true;
      ed_->global_tour_ = {pos, next_pos};
      ed_->refined_tour_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
    } else {
      if (ep_->mission_hold_on_no_frontier_) {
        // 2026-07-08 19:26: 比赛任务尚未接入“目标已全部找到”的外部完成信号前，先禁止 exploration 因 frontier 见底而直接 FINISH。
        return FAIL;
      }
      return NO_FRONTIER;
    }
  }

  // 2026-07-13: 门后由任务覆盖和重复访问选点；现有 frontier 全是已搜索局部点时主动切换多方向恢复。
  int task_candidate_idx = -1;
  if (!use_vertical_detour_target && !use_forced_entry_target && !use_stage3_target &&
      use_mission_filtered_direct_view &&
      !use_mission_forward_fallback && task_search_manager_ && task_search_manager_->enabled()) {
    task_candidate_idx = task_search_manager_->selectSearchCandidate(
        ed_->points_, ed_->yaws_, ed_->frontiers_, pos, yaw[0]);
    if (task_candidate_idx < 0) {
      // 2026-07-20: 原始frontier存在但没有任何可执行候选时也启动配置时长的耗尽确认；
      // 优先使用已经稳定确认并锁存的最终出口，确认期间才允许安全短步恢复。
      if (task_search_manager_->buildStage3Goal(pos, yaw[0], true, next_pos, next_yaw)) {
        use_stage3_target = true;
        ed_->global_tour_ = {pos, next_pos};
        ed_->refined_tour_.clear();
        ed_->refined_views1_.clear();
        ed_->refined_views2_.clear();
      // 2026-07-20: 最终出口保护开启后，通用前探可能绕过frontier距离约束并再次远离终点；
      // 2026-07-21: 此时只等待耗尽确认切入门内接近阶段，不再发布无约束恢复步。
      } else if (!task_search_manager_->finalExitFrontierGuardActive() &&
                 buildMissionForwardFallback(pos, yaw[0], next_pos, next_yaw)) {
        use_mission_forward_fallback = true;
        ed_->global_tour_ = {pos, next_pos};
      } else {
        // 2026-07-14: FSM 会以高频重试，节流该诊断避免一次卡点刷出数千行并拖慢规划回调。
        ROS_WARN_THROTTLE(1.0,
                          "[task_search] searched frontiers exhausted and no safe recovery target exists.");
        return FAIL;
      }
    } else {
      // 2026-07-20: 只有真正选出了可执行候选，才证明搜索覆盖已经恢复并取消耗尽确认。
      task_search_manager_->reportSearchCoverageAvailable();
    }
  }

  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  double view_time = (ros::Time::now() - t1).toSec();
  // 2026-07-16: 规划失败时FSM会高频重试，frontier数量诊断节流到1Hz，避免日志I/O反过来拖慢脱困搜索。
  ROS_WARN_THROTTLE(
      1.0,
      "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
      ed_->points_.size(), view_time);

  // Do global and local tour planning and retrieve the next viewpoint
  if (use_vertical_detour_target) {
    ed_->global_tour_ = {pos, next_pos};
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    ROS_INFO("[vertical_detour] staged target=(%.2f, %.2f, %.2f), fixed yaw=%.1fdeg.",
             next_pos.x(), next_pos.y(), next_pos.z(), next_yaw * 180.0 / M_PI);
  } else if (use_stage3_target) {
    // 2026-07-13: 出口接近、二维码环扫和降落接近都走同一条安全 A*/轨迹生成后端。
    if (task_search_manager_ && !external_selection_only_)
      task_search_manager_->recordSelectedGoal(next_pos);
    ed_->global_tour_ = {pos, next_pos};
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    // 2026-07-14: 第三阶段目标是正常任务状态而非错误；节流输出，真正的 A*/足迹失败仍保留 ERROR。
    ROS_INFO_THROTTLE(1.0, "[exit_mission] stage-3 goal=(%.2f, %.2f, %.2f), yaw=%.1fdeg.",
                      next_pos.x(), next_pos.y(), next_pos.z(), next_yaw * 180.0 / M_PI);
  } else if (use_mission_forward_fallback) {
    ROS_INFO("[mission_exploration] use forward fallback view pos=(%.2f, %.2f, %.2f).",
             next_pos.x(), next_pos.y(), next_pos.z());
  } else if (use_mission_filtered_direct_view) {
    // 2026-07-08 19:26: 任务过滤后的候选集合只存在于 exploration_manager 的副本里，
    // 不能再继续走 FrontierFinder 内部那套基于原始 frontiers_ 链表的 cost matrix / TSP / viewpoint-id 索引，
    // 否则会在 frontier 数量被压缩后出现索引错位，最终在 updateFrontierCostMatrix() 里触发 invalid free。
    // 2026-07-13: 任务搜索启用时使用覆盖评分；关闭时保留第一个 FUEL 视点作为兼容回退。
    const int best_idx = task_search_manager_ && task_search_manager_->enabled()
                             ? task_candidate_idx
                             : (ed_->points_.empty() ? -1 : 0);
    if (best_idx < 0) {
      ROS_WARN("[mission_exploration] no valid filtered candidate after mission selection.");
      return FAIL;
    }
    next_pos = ed_->points_[best_idx];
    if (task_search_manager_ && task_search_manager_->enabled()) {
      // 2026-07-13: 将高处 frontier 投影到当前巡航层，优先横向绕障而不是不断爬高。
      next_pos.z() = task_search_manager_->projectSearchHeight(next_pos.z(), pos.z());
    }
    next_yaw = ed_->yaws_[best_idx];
    if (task_search_manager_) task_search_manager_->recordSelectedGoal(next_pos);
    ed_->global_tour_ = {pos, next_pos};
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    ROS_INFO("[task_search] task view selected idx=%d pos=(%.2f, %.2f, %.2f).",
             best_idx, next_pos.x(), next_pos.y(), next_pos.z());
  } else if (!use_forced_entry_target && ed_->points_.size() > 1) {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
      // Do refinement for the next few viewpoints in the global tour
      // Idx of the first K frontier in optimal tour
      t1 = ros::Time::now();

      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      int knum = min(int(indices.size()), ep_->refined_num_);
      for (int i = 0; i < knum; ++i) {
        auto tmp = ed_->points_[indices[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(indices[i]);
        if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
      }

      // Get top N viewpoints for the next K frontiers
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      vector<double> refined_yaws;
      refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
      next_pos = ed_->refined_points_[0];
      next_yaw = refined_yaws[0];

      // Get marker for view visualization
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        Vector3d view =
            ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
        ed_->refined_views_.push_back(view);
      }
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        vector<Vector3d> v1, v2;
        frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
        frontier_finder_->percep_utils_->getFOV(v1, v2);
        ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
        ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
      }
      double local_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("Local refine time: %lf", local_time);

    } else {
      // Choose the next viewpoint from global tour
      next_pos = ed_->points_[indices[0]];
      next_yaw = ed_->yaws_[indices[0]];
    }
  } else if (!use_forced_entry_target && ed_->points_.size() == 1) {
    // Only 1 destination, no need to find global tour through TSP
    // frontier_finder_->updateFrontierCostMatrix();该这里
    frontier_finder_->updateFrontierCostMatrix();
    ed_->global_tour_ = { pos, ed_->points_[0] };
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

    if (ep_->refine_local_) {
      // Find the min cost viewpoint for next frontier
      ed_->refined_ids_ = { 0 };
      ed_->unrefined_points_ = { ed_->points_[0] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      double min_cost = 100000;
      int min_cost_id = -1;
      vector<Vector3d> tmp_path;
      for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
        auto tmp_cost = ViewNode::computeCost(
            pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
        if (tmp_cost < min_cost) {
          min_cost = tmp_cost;
          min_cost_id = i;
        }
      }
      next_pos = ed_->n_points_[0][min_cost_id];
      next_yaw = n_yaws[0][min_cost_id];
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      next_pos = ed_->points_[0];
      next_yaw = ed_->yaws_[0];
    }
  } else if (!use_forced_entry_target)
    ROS_ERROR("Empty destination.");

  if (hold_fapp_height) next_pos.z() = manual_fapp_hold_height_;

  // 2026-07-16: 比赛使用360度Mid360时航向直接对准运动方向；相机模式保留原始viewpoint航向接口。
  const Eigen::Vector2d motion_delta = next_pos.head<2>() - pos.head<2>();
  // 出口候选观察、确认后的接近/穿越均保持出口朝向，横向修正不能带着机头转。
  const bool use_locked_exit_yaw = use_stage3_target && task_search_manager_ &&
      (task_search_manager_->exitTransitActive() ||
       task_search_manager_->exitVerificationActive());
  if (!use_locked_exit_yaw && !use_camera_viewpoint_yaw_ && motion_delta.norm() > 0.15)
    next_yaw = std::atan2(motion_delta.y(), motion_delta.x());

  if (external_selection_only_) {
    external_selected_viewpoint_ = next_pos;
    external_selected_yaw_ = next_yaw;
    last_requested_goal_ = next_pos;
    has_last_requested_goal_ = true;
    recovery_side_continuation_pending_ = false;
    ROS_INFO("[fuel_diff] selected complete viewpoint (%.2f, %.2f, %.2f), yaw=%.1fdeg.",
             next_pos.x(), next_pos.y(), next_pos.z(), next_yaw * 180.0 / M_PI);
    return SUCCEED;
  }

  // 2026-07-24: 只在前机正常SEARCH_CORRIDOR平移阶段启用相机扫描；入口/出口任务朝向优先。
  // 默认直段小幅快扫、转弯关闭附加扫描、新障碍物才做一次6秒全向扫描。
  bool camera_head_sweep_active = false;
  bool camera_continuous_rotation_active = false;
  const bool corridor_yaw_lock_scope =
      !use_forced_entry_target && !use_stage3_target && task_search_manager_ &&
      task_search_manager_->enabled() && !task_search_manager_->stage3Active();
  const bool mapped_turn_yaw_eligible = corridor_yaw_lock_scope;
  double camera_rotation_rate =
      camera_head_scan_direction_ * 2.0 * M_PI / camera_head_sweep_period_;
  if (corridor_yaw_lock_scope) {
    if (camera_obstacle_scan_active_)
      ROS_WARN("[corridor_yaw_lock] cancel camera yaw scan; only mapped turns may change yaw.");
    camera_obstacle_scan_active_ = false;
    camera_route_heading_valid_ = false;
  }
  // 2026-07-24: 最新实测中出口候选1/5后，门框被相机障碍扫描误触发60deg/s环扫，
  // 下一帧雷达切面随即center_blocked并清票。候选验证期间立即取消环扫并沿路径朝前。
  const bool exit_verification_active =
      task_search_manager_ && task_search_manager_->exitVerificationActive();
  if (exit_verification_active && camera_obstacle_scan_active_) {
    camera_obstacle_scan_active_ = false;
    camera_route_heading_valid_ = false;
    ROS_WARN("[camera_scan] CANCEL obstacle rotation: exit portal verification has priority.");
  }
  if (camera_head_sweep_enabled_ && !corridor_yaw_lock_scope &&
      !use_forced_entry_target && !use_stage3_target &&
      task_search_manager_ && task_search_manager_->enabled() &&
      !task_search_manager_->stage3Active() &&
      !task_search_manager_->finalExitFrontierGuardActive() &&
      !exit_verification_active && motion_delta.norm() > 0.15) {
    const ros::Time now = ros::Time::now();
    const double base_yaw = std::atan2(motion_delta.y(), motion_delta.x());
    if (camera_route_heading_valid_) {
      const double heading_change =
          std::atan2(std::sin(base_yaw - camera_last_route_heading_),
                     std::cos(base_yaw - camera_last_route_heading_));
      if (std::fabs(heading_change) >= camera_turn_threshold_rad_)
        camera_turning_until_ = now + ros::Duration(camera_turn_hold_time_);
    }
    camera_last_route_heading_ = base_yaw;
    camera_route_heading_valid_ = true;
    const bool turning =
        !camera_turning_until_.isZero() && now < camera_turning_until_;

    if (camera_head_full_rotation_) {
      // 2026-07-24: 仅供调试的旧全程环扫回退项，正常比赛launch默认关闭。
      camera_head_sweep_active = true;
      camera_continuous_rotation_active = true;
      ROS_INFO_THROTTLE(
          1.0,
          "[camera_full_scan] current=%.1fdeg route_heading=%.1fdeg rate=%+.1fdeg/s "
          "period=%.1fs.",
          yaw[0] * 180.0 / M_PI, base_yaw * 180.0 / M_PI,
          camera_head_scan_direction_ * 360.0 / camera_head_sweep_period_,
          camera_head_sweep_period_);
    } else {
      // 2026-07-24: 活动障碍物完成一圈后按中心去重，后续地图更新不能反复触发同一物体。
      if (camera_obstacle_scan_active_ &&
          (now - camera_obstacle_scan_start_).toSec() >= camera_obstacle_scan_period_) {
        camera_scanned_obstacles_.push_back(camera_active_obstacle_);
        if (camera_scanned_obstacles_.size() > 64)
          camera_scanned_obstacles_.erase(camera_scanned_obstacles_.begin());
        ROS_WARN("[camera_obstacle_scan] COMPLETE center=(%.2f,%.2f) duration=%.1fs.",
                 camera_active_obstacle_.x(), camera_active_obstacle_.y(),
                 camera_obstacle_scan_period_);
        camera_obstacle_scan_active_ = false;
      }

      if (camera_obstacle_scan_enabled_ && !camera_obstacle_scan_active_ && !turning) {
        Vector3d detected_obstacle;
        if (detectCameraObstacle(pos, motion_delta.head<2>(), detected_obstacle)) {
          camera_active_obstacle_ = detected_obstacle;
          camera_obstacle_scan_start_ = now;
          camera_obstacle_scan_active_ = true;
          ROS_WARN("[camera_obstacle_scan] NEW center=(%.2f,%.2f,%.2f) "
                   "range=%.2fm; start one %.1fs rotation.",
                   detected_obstacle.x(), detected_obstacle.y(), detected_obstacle.z(),
                   (detected_obstacle.head<2>() - pos.head<2>()).norm(),
                   camera_obstacle_scan_period_);
        }
      }

      if (camera_obstacle_scan_active_) {
        // 2026-07-24: 障碍物扫描用60deg/s级连续yaw轨迹；无人机仍沿原安全位置轨迹移动，
        // 通过接近到越过的视点变化覆盖障碍物正面、侧面和背面。
        camera_head_sweep_active = true;
        camera_continuous_rotation_active = true;
        camera_rotation_rate =
            camera_head_scan_direction_ * 2.0 * M_PI / camera_obstacle_scan_period_;
        ROS_INFO_THROTTLE(
            1.0,
            "[camera_obstacle_scan] ACTIVE center=(%.2f,%.2f) elapsed=%.1f/%.1fs "
            "rate=%+.1fdeg/s.",
            camera_active_obstacle_.x(), camera_active_obstacle_.y(),
            (now - camera_obstacle_scan_start_).toSec(), camera_obstacle_scan_period_,
            camera_rotation_rate * 180.0 / M_PI);
      } else if (turning) {
        // 2026-07-24: A/B/C之间的弯道只看运动方向，不把正常转弯计入扫描任务。
        next_yaw = base_yaw;
        ROS_INFO_THROTTLE(
            1.0, "[camera_scan] TURNING route_heading=%.1fdeg; extra scan disabled.",
            base_yaw * 180.0 / M_PI);
      } else {
        // 2026-07-24: 无障碍直段只做小幅快速摆头，兼顾两侧墙面目标而不再持续背向飞行。
        if (camera_head_sweep_start_.isZero()) camera_head_sweep_start_ = now;
        const double elapsed = std::max(0.0, (now - camera_head_sweep_start_).toSec());
        const double phase = 2.0 * M_PI * elapsed / camera_clear_sweep_period_;
        const double sweep_offset =
            camera_clear_sweep_amplitude_rad_ * std::sin(phase);
        next_yaw = std::atan2(std::sin(base_yaw + sweep_offset),
                              std::cos(base_yaw + sweep_offset));
        camera_head_sweep_active = true;
        ROS_INFO_THROTTLE(
            1.0,
            "[camera_scan] CLEAR_STRAIGHT base=%.1fdeg offset=%+.1fdeg "
            "target=%.1fdeg amplitude=%.1fdeg period=%.1fs.",
            base_yaw * 180.0 / M_PI, sweep_offset * 180.0 / M_PI,
            next_yaw * 180.0 / M_PI,
            camera_clear_sweep_amplitude_rad_ * 180.0 / M_PI,
            camera_clear_sweep_period_);
      }
    }
  } else if (!use_locked_exit_yaw && !corridor_yaw_lock_scope && exit_verification_active &&
             motion_delta.norm() > 0.15) {
    // 2026-07-24: 连续5帧门验证期间只看飞行方向，不保持上一环扫角，也不叠加左右摆头。
    next_yaw = std::atan2(motion_delta.y(), motion_delta.x());
    camera_route_heading_valid_ = false;
    ROS_INFO_THROTTLE(
        1.0, "[camera_scan] EXIT_VERIFY forward yaw=%.1fdeg; all camera sweeps suspended.",
        next_yaw * 180.0 / M_PI);
  }

  // 2026-07-14: 记录任务层原始目的地，后续路径截断不能覆盖碰撞失败应冷却的目标。
  last_requested_goal_ = next_pos;
  has_last_requested_goal_ = true;
  std::cout << "Next view: " << next_pos.transpose() << ", " << "next_yaw:" << next_yaw << std::endl;
  // Plan trajectory (position and yaw) to the next viewpoint
  t1 = ros::Time::now();

  // 2026-07-16: Mid360模式不让转头时间拖慢位置轨迹；相机接口打开后才恢复观测航向时间下界。
  const double yaw_diff = std::fabs(next_yaw - yaw[0]);
  const double wrapped_yaw_diff = std::min(yaw_diff, 2.0 * M_PI - yaw_diff);
  const double time_lb = use_camera_viewpoint_yaw_ ? wrapped_yaw_diff / ViewNode::yd_ : 0.0;

  // Generate trajectory of x,y,z
  // 2026-07-23: 已验证出口的门内观察/穿门目标由任务状态机沿锁定门法向生成，不能再被
  // 面向普通FUEL frontier的“旧航迹禁回头”过滤器误杀；占据、A*和机体足迹安全检查仍完整保留。
  const bool exit_transit_active =
      task_search_manager_ && task_search_manager_->exitTransitActive();
  // 低位探测原地只改高度，不应被“最近拐角禁止退回旧通道”的XY半平面误杀。
  // 这里只豁免最近拐角规则，入口/最终出口单向边界、占据、A*和机体足迹仍保留。
  const bool stationary_vertical_detour =
      use_vertical_detour_target && task_search_manager_ &&
      task_search::isStationaryVerticalMotion(
          pos, next_pos, vertical_detour_xy_tolerance_) &&
      task_search_manager_->isMissionBoundaryMotionAllowed(next_pos);
  if (stationary_vertical_detour) {
    if (buildStationaryVerticalDetourPlan(pos, yaw)) return SUCCEED;
    ROS_ERROR_THROTTLE(1.0,
                       "[vertical_detour] failed to build fixed-xy vertical trajectory.");
    return FAIL;
  }
  if (task_search_manager_ && !exit_transit_active &&
      !stationary_vertical_detour &&
      !task_search_manager_->isTaskMotionAllowed(next_pos)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[task_progress] reject goal in completed route %.2f %.2f %.2f.",
                       next_pos.x(), next_pos.y(), next_pos.z());
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 2026-07-13: 所有任务点先做机体足迹检查，避免合法中心点贴柱、贴墙导致旋翼碰撞。
  if (!planner_manager_->isPositionSafe(next_pos)) {
    // 2026-07-14: 保留安全拒绝，但限制重复目标的错误输出频率。
    ROS_ERROR_THROTTLE(1.0, "[footprint_safety] reject unsafe goal %.2f %.2f %.2f before A*.",
                       next_pos.x(), next_pos.y(), next_pos.z());
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  if (!width_selected_path.empty()) {
    ed_->path_next_goal_ = width_selected_path;
  } else if (use_center_gate_target) {
    // buildCenterGatePassage() 已按当前高度验证整段机体包络。直接保留这条线段，
    // 避免通用3D A*为了净空评分重新从门上方绕到同一个终点。
    ed_->path_next_goal_ = {pos, next_pos};
  } else {
    planner_manager_->path_finder_->reset();
  }
  if (width_selected_path.empty() && !use_center_gate_target &&
      planner_manager_->path_finder_->search(pos, next_pos, hold_fapp_height) != Astar::REACH_END) {
    // 2026-07-27: 远frontier跨越尚未建成的已知连通域时，先在当前前向扇区找安全短步；
    // 不再立即切换另一个远目标并触发长时间safety hold，让雷达随局部推进逐段补齐拐弯地图。
    Vector3d local_recovery_pos;
    double local_recovery_yaw = next_yaw;
    const bool allow_local_recovery =
        !use_forced_entry_target && !use_stage3_target && !exit_transit_active;
    if (allow_local_recovery &&
        buildMissionForwardFallback(pos, yaw[0], local_recovery_pos, local_recovery_yaw)) {
      ROS_WARN("[task_route] remote goal (%.2f,%.2f,%.2f) disconnected; use connected "
               "local step (%.2f,%.2f,%.2f).",
               next_pos.x(), next_pos.y(), next_pos.z(), local_recovery_pos.x(),
               local_recovery_pos.y(), local_recovery_pos.z());
      next_pos = local_recovery_pos;
      if (hold_fapp_height) next_pos.z() = manual_fapp_hold_height_;
      next_yaw = local_recovery_yaw;
      // 2026-07-27: 碰撞反馈必须冷却实际发布的局部目标，不能继续指向已被替换的远frontier。
      last_requested_goal_ = next_pos;
      planner_manager_->path_finder_->reset();
      if (planner_manager_->path_finder_->search(pos, next_pos, hold_fapp_height) != Astar::REACH_END) {
        ROS_ERROR_THROTTLE(
            1.0, "Connected local recovery became unavailable before trajectory generation");
        if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
        return FAIL;
      }
    } else {
      // 2026-07-16: Astar内部会区分TIMEOUT和DISCONNECTED；这里只保留1Hz汇总，禁止失败时百Hz刷屏。
      ROS_ERROR_THROTTLE(1.0, "No path to next viewpoint");
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
  }
  if (width_selected_path.empty() && !use_center_gate_target)
    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  // 2026-07-14: 起点和目标落在同一 A* 栅格时 getPath() 可能只返回一个点；去重并显式保留真实目标。
  vector<Vector3d> normalized_path;
  normalized_path.reserve(ed_->path_next_goal_.size() + 1);
  for (const auto& point : ed_->path_next_goal_) {
    if (normalized_path.empty() || (point - normalized_path.back()).norm() >= 1e-3)
      normalized_path.push_back(point);
  }
  // 2026-07-14: 同栅格回溯可能只含目标栅格中心，不含真实起点；显式补起点才能保留实际短距离。
  if (normalized_path.empty())
    normalized_path.push_back(pos);
  else if ((normalized_path.front() - pos).norm() >= 1e-3)
    normalized_path.insert(normalized_path.begin(), pos);
  if ((next_pos - normalized_path.back()).norm() >= 1e-3) normalized_path.push_back(next_pos);
  ed_->path_next_goal_.swap(normalized_path);
  if (hold_fapp_height) {
    for (size_t i = 1; i < ed_->path_next_goal_.size(); ++i)
      ed_->path_next_goal_[i].z() = manual_fapp_hold_height_;
  }

  const bool stationary_vertical_path =
      stationary_vertical_detour &&
      std::all_of(
          ed_->path_next_goal_.begin(), ed_->path_next_goal_.end(),
          [&](const Vector3d& point) {
            return (point - pos).head<2>().norm() <=
                       vertical_detour_xy_tolerance_ + 1e-6 &&
                   task_search_manager_->isMissionBoundaryMotionAllowed(point);
          });

  // 2026-07-23: 普通搜索路径继续执行全局禁回头；出口任务路径允许跨越最近实飞航迹和门平面，
  // 否则CROSS_EXIT会在门口被“completed route”永久拒绝。
  if (task_search_manager_ && !exit_transit_active &&
      !stationary_vertical_path &&
      !task_search_manager_->isRecoveryPathAllowed(ed_->path_next_goal_,
                                                   pending_short_backtrack_)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[entrance_plane_reject] A* path crosses the locked entrance boundary; "
                       "cool down goal and select another frontier.");
    // 坐标仍在workspace内不代表与当前门后通道拓扑连通。入口边界拒绝后必须解除活动目标，
    // 否则keep-active会永久重试建图早期遗留在通道外的frontier。
    if (task_search::boundaryPathRejectRequiresGoalSwitch())
      task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }

  // Normal frontier paths and generic recovery share the same horizontal
  // width comparison. Do this before the local cut, not only after execution
  // has already carried the vehicle toward a narrow passage.
  if (!use_forced_entry_target && !use_stage3_target && !exit_transit_active &&
      !use_entry_forward_target && !use_vertical_detour_target &&
      !pending_short_backtrack_ && width_selected_path.empty() &&
      !use_center_gate_target &&
      prepareWiderRoute(ed_->path_next_goal_, yaw[0])) {
    ed_->path_next_goal_.swap(pending_wider_route_);
    pending_wider_route_.clear();
    width_route_plan_ = true;
    next_pos = ed_->path_next_goal_.back();
    next_yaw = yaw[0];
    last_requested_goal_ = next_pos;
    has_last_requested_goal_ = true;
    if (!external_selection_only_) task_search_manager_->recordSelectedGoal(next_pos);
    ROS_WARN("[route_width] replace narrow planned route before publication; "
             "target=(%.2f,%.2f,%.2f).", next_pos.x(), next_pos.y(), next_pos.z());
  }

  // 2026-07-22: FUEL远端frontier只决定前进方向；沿A*路径截取短安全段作为本轮路线点，
  // 避免刚穿门就发布跨越数米未知/窄区的整条轨迹，并让新点云在每段之间更新地图。
  const double route_horizon = std::max(0.20, ep_->mission_max_route_segment_length_);
  const bool centre_guidance = passage_centering_enabled_ &&
      !(manual_fapp_active_ && recovery_side_latched_) &&
      !use_forced_entry_target && !use_stage3_target && !exit_transit_active && !use_entry_forward_target &&
      !use_vertical_detour_target &&
      (!use_center_gate_target || recovery_side_continuation_pending_);
  bool has_central_guides = false;
  // Simplify first, then retain central cross-section points as spline guides.
  // Centre before the 1.2m cut so its endpoint can lie in the lane centre.
  if (centre_guidance) {
    shortenPath(ed_->path_next_goal_);
    extendSafeStraightRuns(ed_->path_next_goal_);
    has_central_guides = centrePassagePath(ed_->path_next_goal_, route_horizon + 0.50);
  }
  // 只有原始前沿点本身在前方超过该距离时才截断；近前沿即使因绕障导致A*路径变长，
  // 也必须保留其完整路径，不能把“路径长”误当成“前沿点远”。
  const bool far_frontier_goal =
      !use_forced_entry_target && !use_stage3_target &&
      (next_pos - pos).norm() > route_horizon + 1e-6;
  if (far_frontier_goal && Astar::pathLength(ed_->path_next_goal_) > route_horizon) {
    vector<Vector3d> local_segment{ed_->path_next_goal_.front()};
    double accumulated = 0.0;
    for (size_t i = 1; i < ed_->path_next_goal_.size(); ++i) {
      const Vector3d delta = ed_->path_next_goal_[i] - ed_->path_next_goal_[i - 1];
      const double segment_length = delta.norm();
      if (segment_length < 1e-6) continue;
      if (accumulated + segment_length >= route_horizon) {
        const double ratio = (route_horizon - accumulated) / segment_length;
        local_segment.push_back(ed_->path_next_goal_[i - 1] +
                                std::max(0.0, std::min(1.0, ratio)) * delta);
        break;
      }
      local_segment.push_back(ed_->path_next_goal_[i]);
      accumulated += segment_length;
    }
    ed_->path_next_goal_.swap(local_segment);
    ROS_WARN("[task_route] truncate remote goal to %.2fm safe segment endpoint=(%.2f, %.2f, %.2f).",
             route_horizon, ed_->path_next_goal_.back().x(), ed_->path_next_goal_.back().y(),
             ed_->path_next_goal_.back().z());
  }
  // 2026-07-23: 截断后的出口任务短步同样豁免旧航迹门控，但不豁免后续障碍物/足迹检查。
  if (task_search_manager_ && !exit_transit_active &&
      !stationary_vertical_path &&
      !task_search_manager_->isTaskMotionAllowed(ed_->path_next_goal_.back())) {
    ROS_ERROR_THROTTLE(1.0,
                       "[entrance_plane_reject] route segment ends outside the locked entrance; "
                       "cool down goal and select another frontier.");
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  if (Astar::pathLength(ed_->path_next_goal_) < 0.03) {
    // 2026-07-14: 三厘米内视为已到达，冷却该视点并换点，不能生成零时长多项式轨迹。
    ROS_WARN_THROTTLE(1.0, "[trajectory_input] goal is already reached; select another viewpoint.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 缩短、居中和截断后的最终几何路径仍按完整机体足迹复核。
  if (!planner_manager_->isPathSafe(ed_->path_next_goal_)) {
    // 2026-07-14: 路径失败会反馈给任务层更换出口接近点，不再以 100 Hz 输出同一错误。
    ROS_ERROR_THROTTLE(1.0,
                       "[footprint_safety] reject A* path whose Iris footprint intersects obstacles.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 固定入口、出口、上下穿以及关闭居中时仍按原顺序先截断再缩短。
  if (!centre_guidance) {
    shortenPath(ed_->path_next_goal_);
    extendSafeStraightRuns(ed_->path_next_goal_);
    if (!planner_manager_->isPathSafe(ed_->path_next_goal_)) {
      ROS_ERROR_THROTTLE(
          1.0, "[footprint_safety] reject shortened path whose Iris footprint intersects obstacles.");
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
  }
  // 2026-07-23: shortenPath后只对普通探索路径再次做禁回头判定，任务出口路径由门状态机约束方向。
  if (task_search_manager_ && !exit_transit_active &&
      !stationary_vertical_path &&
      !task_search_manager_->isRecoveryPathAllowed(ed_->path_next_goal_,
                                                   pending_short_backtrack_)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[entrance_plane_reject] shortened path crosses the locked entrance boundary; "
                       "cool down goal and select another frontier.");
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 已确认出口的专用接近/穿越目标可能位于原入口平面的另一侧（U形通道）。
  // 仅豁免该任务路径的旧入口平面检查，普通frontier仍受约束；前面的目标
  // 足迹、A*占据检查以及后续轨迹碰撞检查继续执行。
  if (!use_forced_entry_target && !(use_stage3_target && exit_transit_active) &&
      !pathInsideWorkspaceLock(ed_->path_next_goal_, use_entry_forward_target)) {
    // 2026-07-10: 即使 viewpoint 在门内，A* 也可能绕回起飞区；轨迹生成前必须拒绝穿门外路径。
    ROS_ERROR("[workspace_lock] reject path to next viewpoint because it crosses outside the door plane.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }

  // 低位普通回退截断后可能只剩两个端点；补共线中点，避免可行侧路因样条输入不足而停住。
  if (hold_fapp_height && ed_->path_next_goal_.size() == 2)
    ed_->path_next_goal_.insert(ed_->path_next_goal_.begin()+1,
        0.5*(ed_->path_next_goal_.front()+ed_->path_next_goal_.back()));
  if (!keepsDynamicSide(ed_->path_next_goal_)) return FAIL;
  const double radius_far = 5.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (use_entry_forward_target) {
    if (!planner_manager_->planStraightTraj(pos, next_pos)) return FAIL;
    ed_->next_goal_ = next_pos;
  } else if (hold_fapp_height || len < radius_close || (has_central_guides && len <= radius_far)) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    if (!planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb, hold_fapp_height, fapp_rush_active_ ? 1.0 : 0.0, fapp_rush_active_ ? 1.0 : 0.0)) {
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
    ed_->next_goal_ = next_pos;

  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    if (!planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb)) {
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
    // if (!planner_manager_->kinodynamicReplan(
    //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
    //   return FAIL;
    // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb)) {
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
    ROS_ERROR("Lower bound not satified!");

  // 2026-07-14: 优化后的 B-spline 可能切入柱体/墙面；发布前拒绝并切换任务目标，而不是交给控制器反复急停。
  if (!planner_manager_->isTrajectorySafe()) {
    // 2026-07-16: 远目标平滑轨迹切弯失败时，沿已经通过足迹检查的A*路径先走0.8m安全前缀。
    // 比赛中宁可分段推进并重规划，也不能在宽通道对同一远目标永久FAIL/悬停。
    constexpr double conservative_advance = 0.80;
    vector<Vector3d> safe_prefix;
    if (!ed_->path_next_goal_.empty()) safe_prefix.push_back(ed_->path_next_goal_.front());
    double prefix_length = 0.0;
    for (size_t i = 1; i < ed_->path_next_goal_.size() && prefix_length < conservative_advance;
         ++i) {
      const Vector3d delta = ed_->path_next_goal_[i] - safe_prefix.back();
      const double remaining = conservative_advance - prefix_length;
      if (delta.norm() > remaining && remaining > 1e-3) {
        safe_prefix.push_back(safe_prefix.back() + delta.normalized() * remaining);
        prefix_length = conservative_advance;
      } else {
        safe_prefix.push_back(ed_->path_next_goal_[i]);
        prefix_length += delta.norm();
      }
    }
    if (safe_prefix.size() == 2)
      safe_prefix.insert(safe_prefix.begin() + 1, 0.5 * (safe_prefix[0] + safe_prefix[1]));

    const bool prefix_generated = safe_prefix.size() >= 3 &&
                                  planner_manager_->isPathSafe(safe_prefix) &&
                                  planner_manager_->planExploreTraj(safe_prefix, vel, acc, 0.0, hold_fapp_height, fapp_rush_active_ ? 1.0 : 0.0, fapp_rush_active_ ? 1.0 : 0.0);
    if (!prefix_generated || !planner_manager_->isTrajectorySafe()) {
      ROS_ERROR_THROTTLE(1.0,
                         "[stuck_recovery] full trajectory and 0.8m safe-prefix trajectory both failed.");
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
    ed_->next_goal_ = safe_prefix.back();
    ROS_WARN("[stuck_recovery] full trajectory unsafe; publish %.2fm A* safe prefix to (%.2f %.2f %.2f).",
             prefix_length, ed_->next_goal_.x(), ed_->next_goal_.y(), ed_->next_goal_.z());
  }

  // 入口后第一米按实际轨迹终点的门法向位移累计，不能用弧长或侧向绕圈凑数。
  if (!use_entry_forward_target && mission_workspace_lock_received_ &&
      !entry_forward_complete_ &&
      !mission_entered_search_region_ && task_search_manager_ &&
      !task_search_manager_->hasConfirmedCorridorTurn() &&
      mission_workspace_dir_.head<2>().norm() > 1e-3) {
    const Vector3d forward = mission_workspace_dir_.normalized();
    const double entry_progress = (pos - mission_workspace_origin_).dot(forward);
    if (entry_forward::needsAdvance(entry_progress)) {
      const Vector3d actual_end = planner_manager_->local_data_.position_traj_.evaluateDeBoorT(
          planner_manager_->local_data_.position_traj_.getTimeSum());
      const double advance = (actual_end - pos).dot(forward);
      if (advance + 1e-3 < std::min(0.15, 1.0 - entry_progress)) {
        ROS_WARN("[entry_forward] reject side-only trajectory: actual advance %.2fm, "
                 "accumulated %.2fm.", advance, entry_progress);
        task_search_manager_->reportGoalFailure(next_pos);
        return FAIL;
      }
    }
  }

  // 水平路线已生成并通过足迹检查后，再可选地恢复巡航高度；失败保留原路线。
  // 当前段的左右选择与人工、入口/出口、下穿等专用高度所有权不受影响。
  if (!use_forced_entry_target && !use_stage3_target && !use_entry_forward_target &&
      !use_vertical_detour_target && !pending_short_backtrack_ &&
      tryCruiseHeightRecovery(pos, vel, acc, time_lb))
    next_pos = ed_->next_goal_;

  // 2026-07-24: 扫描段关闭“整段始终看前方”的约束；全向模式在位置轨迹全程插入单方向
  // 连续yaw约束，非扫描阶段完全保留原有沿路径看前方或任务固定朝向的行为。
  bool look_forward_along_trajectory = !camera_head_sweep_active && !use_locked_exit_yaw;
  Eigen::Vector3d mapped_direction;
  const bool mapped_turn_detected =
      mapped_turn_yaw_eligible && task_search_manager_ &&
      task_search_manager_->mappedCorridorDirection(yaw[0], mapped_direction);
  const bool corridor_yaw_correction =
      !mapped_turn_detected && corridor_yaw_lock_scope && task_search_manager_ &&
      task_search_manager_->corridorYawCorrectionDirection(yaw[0], mapped_direction);
  Eigen::Vector3d motion_corridor_direction;
  const bool corridor_motion_yaw_exempt =
      !mapped_turn_detected && !corridor_yaw_correction && corridor_yaw_lock_scope &&
      task_search_manager_ && motion_delta.norm() > 0.15 &&
      task_search_manager_->corridorMotionDirectionMatches(
          motion_delta.head<2>(), motion_corridor_direction);
  double curve_yaw=0.0;
  const bool corridor_curve_heading = !mapped_turn_detected && !use_vertical_detour_target &&
      corridor_yaw_lock_scope &&
      task_search_manager_ && task_search_manager_->corridorCurveYaw(pos,curve_yaw);
  if (corridor_curve_heading) {
    next_yaw=curve_yaw;
    look_forward_along_trajectory=false;
    camera_head_sweep_active=false;
    camera_continuous_rotation_active=false;
  } else if (mapped_turn_detected || corridor_yaw_correction) {
    // 地图墙体轮廓独立于本次轨迹方向触发：yaw只朝向双墙确认的新通道轴线，
    // 不能跟随局部A*绕柱切线，否则一次普通侧绕也会带着机头误转。
    next_yaw = std::atan2(mapped_direction.y(), mapped_direction.x());
    look_forward_along_trajectory = false;
    camera_head_sweep_active = false;
    camera_continuous_rotation_active = false;
    if (mapped_turn_detected) {
      ROS_WARN("[turn_yaw_follow] mapped bilateral-wall turn latched; hold "
               "wall-parallel target yaw=%.1fdeg.",
               next_yaw * 180.0 / M_PI);
    } else {
      ROS_WARN("[corridor_yaw_correction] body yaw drifted from the current "
               "segment axis; correct to %.1fdeg without creating a new segment.",
               next_yaw * 180.0 / M_PI);
    }
    // 新通道首次确认时丢弃刚生成的平移轨迹并先对准机头；普通轴线校正继续平移。
    if (mapped_turn_detected) {
      // 本轮前半段可能已经生成了下穿目标；转弯确认后该目标不得跨周期保留。
      cancelActiveLowProbe("new mapped turn has higher motion priority");
      if (buildTurnInPlacePlan(pos, vel, acc, yaw, mapped_direction)) return SUCCEED;
      // 转弯已经锁存但仍在刹停/静止确认时，禁止回落到刚生成的普通平移轨迹。
      return FAIL;
    }
  } else if (corridor_motion_yaw_exempt) {
    // 地图确认的通道延伸方向与本次轨迹一致：允许沿轨迹微调机头，
    // 但不提交新的转弯段，也不触发原地转向。
    next_yaw = std::atan2(motion_delta.y(), motion_delta.x());
    look_forward_along_trajectory = false;
    camera_head_sweep_active = false;
    camera_continuous_rotation_active = false;
    ROS_INFO_THROTTLE(
        0.5,
        "[corridor_yaw_lock] allow trajectory yaw=%.1fdeg; channel extension "
        "matches motion (map_dir=%.1fdeg).",
        next_yaw * 180.0 / M_PI,
        std::atan2(motion_corridor_direction.y(), motion_corridor_direction.x()) *
            180.0 / M_PI);
  } else if (task_search::holdYawInCorridor(
                 corridor_yaw_lock_scope, mapped_turn_detected)) {
    // 通道内除累计地图确认的真实拐弯外，任何位置运动都不能改变机头方向。
    next_yaw = yaw[0];
    look_forward_along_trajectory = false;
    camera_head_sweep_active = false;
    camera_continuous_rotation_active = false;
    ROS_INFO_THROTTLE(0.5,
                      "[corridor_yaw_lock] hold yaw=%.1fdeg; no mapped turn confirmed.",
                      yaw[0] * 180.0 / M_PI);
  }
  std::function<double(const Eigen::Vector3d&)> channel_yaw;
  if (corridor_curve_heading) {
    channel_yaw=[this,next_yaw](const Eigen::Vector3d& position) {
      double heading=next_yaw;
      task_search_manager_->corridorCurveYaw(position,heading);
      return heading;
    };
  }
  planner_manager_->planYawExplore(
      yaw, next_yaw, look_forward_along_trajectory, ep_->relax_time_,
      camera_continuous_rotation_active, camera_rotation_rate, channel_yaw);

  double traj_plan_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  double total = (ros::Time::now() - t2).toSec();
  ROS_WARN("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  if (pending_short_backtrack_) {
    short_backtrack_latched_ = true;
    short_backtrack_release_origin_ = pending_short_backtrack_target_;
    short_backtrack_forward_dir_ = pending_short_backtrack_forward_dir_;
    short_backtrack_last_dir_ = pending_short_backtrack_dir_;
    short_backtrack_last_time_ = ros::Time::now();
    ++short_backtrack_chain_count_;
    ROS_ERROR("[mission_exploration] COMMIT ONE-TIME SHORT BACKTRACK %d/%d after trajectory "
              "generation target=(%.2f, %.2f, %.2f).",
              short_backtrack_chain_count_, short_backtrack_max_chain_,
              pending_short_backtrack_target_.x(), pending_short_backtrack_target_.y(),
              pending_short_backtrack_target_.z());
  }

  if (!use_forced_entry_target && !use_stage3_target &&
      !use_vertical_detour_target && !pending_short_backtrack_ &&
      mission_entered_search_region_ && task_search_manager_ &&
      task_search_manager_->enabled())
    rememberSideRecovery(pos, yaw[0], ed_->path_next_goal_);
  recovery_side_continuation_pending_ = false;
  // 保留本段的目标高度，不能把里程计误差或 A* 截断处的中间高度当成新定高。
  // 原地转向、人工航点和膨胀层脱困提前返回，不改写普通水平段的高度参考。
  pending_reference_height_ = next_pos.z();
  return SUCCEED;
}

bool FastExplorationManager::selectExplorationViewpoint(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {
  external_selection_only_ = true;
  const int result = planExploreMotion(pos, vel, acc, yaw);
  external_selection_only_ = false;
  if (result != SUCCEED) return false;
  next_pos = external_selected_viewpoint_;
  next_yaw = external_selected_yaw_;
  return next_pos.allFinite() && std::isfinite(next_yaw);
}

void FastExplorationManager::freezeExplorationInitialYaw(double yaw) {
  if (task_search_manager_) task_search_manager_->freezeExplorationInitialYaw(yaw);
}

void FastExplorationManager::fillExplorationConstraint(
    quadrotor_msgs::ExplorationMotionConstraint& msg) const {
  if (task_search_manager_) task_search_manager_->fillExplorationConstraint(msg);
  else msg = quadrotor_msgs::ExplorationMotionConstraint();
}

void FastExplorationManager::recordExplorationReached(const Vector3d& goal) {
  if (task_search_manager_) task_search_manager_->recordExplorationReached(goal);
}

void FastExplorationManager::reportTrajectoryCollision() {
  // 2026-07-14: 冷却任务层原始目标而非远路径的截断中间点，阻断同一出口路线反复发布/急停。
  if (task_search_manager_ && has_last_requested_goal_)
    task_search_manager_->reportGoalFailure(last_requested_goal_);
}

void FastExplorationManager::updateMissionOdometry(const Vector3d& pos, double yaw,
                                                   const ros::Time& stamp) {
  if (safe_relay_) {
    if (mission_workspace_lock_received_)
      safe_relay_->setEntrance(mission_workspace_origin_, mission_workspace_dir_);
    safe_relay_->observe(pos, yaw, stamp);
  }
  if (!manual_path_enabled_ && fapp_exploration_crossing_enabled_ && manual_fapp_active_)
    manualFappObservationMissing(pos, yaw, true);
  if (manual_path_enabled_) {
    if (manual_path_wait_for_trigger_ && !manual_path_started_) return;
    manual_actual_pos_ = pos;
    if (manual_fapp_active_) manualFappObservationMissing(pos, yaw, true);
    if ((manual_exit_enabled_ || manual_path_exploration_style_) && task_search_manager_) {
      // 入口关闭时以开始任务的位置/航向建立通道参考；入口开启时复用已发布的门平面。
      geometry_msgs::PoseStamped detected_exit;
      task_search_manager_->updateExitDetectionOnly(
          pos, yaw, detected_exit, !manual_path_exploration_style_);
      Vector3d portal;
      Eigen::Vector2d outward;
      if (manual_exit_enabled_ && !manual_exit_latched_ &&
          task_search_manager_->confirmedExitPortal(portal, outward)) {
        const Vector3d previous = manual_path_.finished() ? pos : manual_path_.points[manual_path_.next];
        const int nearest = manual_path_.truncateAtExit(portal, outward);
        manual_exit_latched_ = true;
        manual_exit_replan_pending_ = manual_path_.finished() ||
            (manual_path_.points[manual_path_.next] - previous).norm() > 1e-6;
        ROS_WARN("[manual_path] exit locked; finish original waypoint %d then approach exit; "
                 "outside and later waypoints discarded.", nearest + 1);
      }
    }
    const bool was_finished = manual_path_.finished();
    manual_path_.updateOdometry(pos, manual_path_arrive_distance_);
    if (manual_exit_latched_ && !was_finished && manual_path_.finished())
      manual_exit_replan_pending_ = true;
  }
  if (!entry_forward_complete_ && mission_workspace_lock_received_ &&
      (pos - mission_workspace_origin_).dot(mission_workspace_dir_) >= 1.0 - 1e-6) {
    entry_forward_complete_ = true;
    ROS_WARN("[entry_forward] actual net advance reached 1m; release entry target ownership.");
  }
  // 2026-07-22: 连续里程计航迹是任务完成路段的唯一依据；规划器预测起点不再污染真实历史。
  if (recovery_odom_history_.empty() ||
      (pos - recovery_odom_history_.back()).norm() >= recovery_history_spacing_) {
    recovery_odom_history_.push_back(pos);
    while (static_cast<int>(recovery_odom_history_.size()) > recovery_history_max_size_)
      recovery_odom_history_.pop_front();
  }
  if (task_search_manager_ &&
      !(manual_path_enabled_ && (manual_exit_enabled_ || manual_path_exploration_style_)))
    task_search_manager_->updateRobotPose(pos, yaw);
}

bool FastExplorationManager::consumeRcSearchLandingStartRequest() {
  return task_search_manager_ &&
         task_search_manager_->consumeRcSearchLandingStartRequest();
}

bool FastExplorationManager::detectMappedTurnDuringExecution(
    double yaw, Vector3d& direction) {
  if (suspended_underpass_active_ || !turn_in_place_enabled_ ||
      (!manual_path_enabled_ && !mission_entered_search_region_) ||
      (manual_path_enabled_ && (!manual_path_exploration_style_ ||
       (manual_path_wait_for_trigger_ && !manual_path_started_))) || !task_search_manager_ ||
      !task_search_manager_->enabled() || task_search_manager_->stage3Active())
    return false;
  if (!task_search_manager_->mappedCorridorDirection(yaw, direction, !manual_path_enabled_))
    return false;
  return task_search_manager_->turnYawAlignmentPending();
}

bool FastExplorationManager::shouldReplanCurveHeading(
    const Vector3d& position,double reference_yaw) const {
  return task_search_manager_ && !suspended_underpass_active_ &&
         task_search_manager_->shouldReplanCurveHeading(position,reference_yaw);
}

bool FastExplorationManager::shouldStartInflationHistoryEscape(
    const Vector3d& odom_pos) const {
  return inflation_history_escape_enabled_ && recovery_odom_history_.size() >= 2 &&
         inflation_escape::shouldStartFromOdometry(
             planner_manager_->isControlledEscapePosition(odom_pos));
}

bool FastExplorationManager::centrePassagePath(vector<Vector3d>& path, double horizon) {
  if (manual_fapp_active_ && recovery_side_latched_) return false;
  passage_centering::Config cfg;
  cfg.step = std::min(0.05, 0.5 * sdf_map_->getResolution());
  cfg.search_radius = passage_centering_search_radius_;
  cfg.maximum_shift = passage_centering_maximum_shift_;
  const bool changed = passage_centering::centrePath(path, horizon, cfg,
      [&](const Vector3d& p) {
        return pointInsideWorkspaceLock(p) && planner_manager_->isKnownFreePosition(p);
      }, [&](const vector<Vector3d>& candidate) {
        return planner_manager_->isPathSafe(candidate) &&
            pathInsideWorkspaceLock(candidate) &&
            (!task_search_manager_ || task_search_manager_->isRecoveryPathAllowed(
                candidate, pending_short_backtrack_));
      });
  if (changed)
    ROS_WARN("[passage_centering] preserve %zu centre-guided route points before local cut.",
             path.size());
  return changed;
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  const double clearance_loss_tolerance = 0.05;
  vector<Vector3d> short_tour = { path.front() };
  size_t anchor_index = 0;
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh) {
      short_tour.push_back(path[i]);
      anchor_index = static_cast<size_t>(i);
    } else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      bool preserve_waypoint = false;
      double shortcut_min_clearance = std::min(
          edt_environment_->sdf_map_->getDistance(short_tour.back()),
          edt_environment_->sdf_map_->getDistance(path[i + 1]));
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          preserve_waypoint = true;
          break;
        }
        shortcut_min_clearance = std::min(
            shortcut_min_clearance, edt_environment_->sdf_map_->getDistance(idx));
      }

      // The A* route already contains a soft clearance cost. Do not erase a turn when the
      // line-of-sight shortcut gives up noticeably more clearance just to save distance.
      if (!preserve_waypoint) {
        double original_min_clearance = std::numeric_limits<double>::infinity();
        for (size_t j = anchor_index; j <= static_cast<size_t>(i + 1); ++j) {
          original_min_clearance = std::min(
              original_min_clearance, edt_environment_->sdf_map_->getDistance(path[j]));
        }
        preserve_waypoint = path_shortening::preserveWaypointForClearance(
            original_min_clearance, shortcut_min_clearance,
            clearance_loss_tolerance) ||
            path_shortening::preserveWaypointForObstacleTurn(
                path[i - 1], path[i], path[i + 1], original_min_clearance,
                0.50, 30.0 * M_PI / 180.0);
      }
      if (preserve_waypoint) {
        short_tour.push_back(path[i]);
        anchor_index = static_cast<size_t>(i);
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::extendSafeStraightRuns(vector<Vector3d>& path) {
  if (!straight_run_extension_enabled_ ||
      straight_run_extension_distance_ < 0.05 || path.size() < 3)
    return;
  // 地图已经确认真实通道转弯后，流程必须是“刹停→原地对准→再前进”；
  // 禁止再插入旧方向0.20m直行点拖延转向。
  if (task_search_manager_ && task_search_manager_->turnYawAlignmentPending()) {
    ROS_WARN_THROTTLE(0.5,
                      "[straight_run_extension] skipped: mapped turn alignment owns motion.");
    return;
  }
  if (skip_straight_extension_after_turn_) {
    skip_straight_extension_after_turn_ = false;
    ROS_WARN("[straight_run_extension] skip the first translation after a confirmed "
             "in-place turn; move directly along the new corridor.");
    return;
  }

  const double minimum_turn =
      straight_run_extension_min_turn_deg_ * M_PI / 180.0;
  vector<Vector3d> extended;
  extended.reserve(path.size() * 2);
  extended.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    extended.push_back(path[i]);
    const Vector3d incoming_3d = path[i] - path[i - 1];
    const Vector3d outgoing_3d = path[i + 1] - path[i];
    const Eigen::Vector2d incoming = incoming_3d.head<2>();
    const Eigen::Vector2d outgoing = outgoing_3d.head<2>();
    if (incoming.norm() < 0.10 || outgoing.norm() < 0.10 ||
        std::fabs(incoming_3d.z()) > 0.08 ||
        std::fabs(outgoing_3d.z()) > 0.08 ||
        sdf_map_->getDistance(path[i]) > 0.50 ||
        path_shortening::turnAngleRadians(path[i - 1], path[i], path[i + 1]) <
            minimum_turn ||
        incoming.normalized().dot(outgoing.normalized()) < -0.10)
      continue;

    Vector3d extension = path[i];
    extension.head<2>() +=
        straight_run_extension_distance_ * incoming.normalized();
    if ((extension - path[i + 1]).norm() < 0.08) continue;

    const vector<Vector3d> delayed_turn{path[i], extension, path[i + 1]};
    if (!planner_manager_->isPathSafe(delayed_turn)) continue;

    extended.push_back(extension);
    ROS_WARN("[straight_run_extension] delay %.1fdeg turn by %.2fm at "
             "(%.2f,%.2f)->(%.2f,%.2f); full footprint path is safe.",
             path_shortening::turnAngleRadians(
                 path[i - 1], path[i], path[i + 1]) * 180.0 / M_PI,
             straight_run_extension_distance_, path[i].x(), path[i].y(),
             extension.x(), extension.y());
  }
  if ((path.back() - extended.back()).norm() > 1e-3)
    extended.push_back(path.back());
  path.swap(extended);
}

void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

}  // namespace fast_planner
