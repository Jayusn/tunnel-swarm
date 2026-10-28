// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_env/metric_voxel_policy.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fast_planner {
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {
}

FastPlannerManager::~FastPlannerManager() {
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
  nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
  nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
  nh.param("manager/min_time", pp_.min_time_, false);
  // 2026-07-23: 0.15m膨胀图用于A*搜索引导；路径和轨迹最终安全按此真实机体圆盘半径复核，
  // 两者不再叠成硬性的0.35m净空要求。
  nh.param("manager/footprint_check_radius", footprint_check_radius_, 0.18);
  nh.param("manager/turn_check_radius", turn_check_radius_, 0.25);
  nh.param("manager/footprint_check_samples", footprint_check_samples_, 12);
  nh.param("manager/footprint_min_occupied_support",
           footprint_min_occupied_support_, 2);
  nh.param("manager/footprint_support_radius", footprint_support_radius_, 0.10);
  nh.param("manager/supported_occupancy_hard_reject_enabled",
           supported_occupancy_hard_reject_enabled_, true);
  nh.param("manager/escape_max_initial_occupied_samples",
           escape_max_initial_occupied_samples_, 6);
  nh.param("manager/turn_slowdown_angle_deg", turn_slowdown_angle_deg_, 30.0);
  nh.param("manager/turn_time_scale", turn_time_scale_, 1.5);
  footprint_check_radius_ = std::max(0.0, footprint_check_radius_);
  turn_check_radius_ = std::max(0.0, turn_check_radius_);
  footprint_check_samples_ = std::max(4, footprint_check_samples_);
  footprint_min_occupied_support_ = std::max(1, footprint_min_occupied_support_);
  footprint_support_radius_ = std::max(0.0, footprint_support_radius_);
  escape_max_initial_occupied_samples_ =
      std::max(0, escape_max_initial_occupied_samples_);
  turn_slowdown_angle_deg_ =
      std::max(5.0, std::min(90.0, turn_slowdown_angle_deg_));
  turn_time_scale_ = std::max(1.0, turn_time_scale_);

  nh.param("fapp/enabled", fapp_config_.enabled, false);
  nh.param("fapp/lateral_motion_only", fapp_lateral_motion_only_, false);
  nh.param("fapp/vehicle_top_height", fapp_config_.vehicle_top_height, 0.125);
  nh.param("fapp/vehicle_bottom_height", fapp_config_.vehicle_bottom_height, 0.125);
  nh.param("mission/vertical_detour/underpass/clearance",
           fapp_config_.underpass_reference_clearance, -1.0);
  if (!std::isfinite(fapp_config_.vehicle_top_height) || fapp_config_.vehicle_top_height <= 0.0 ||
      !std::isfinite(fapp_config_.vehicle_bottom_height) || fapp_config_.vehicle_bottom_height <= 0.0)
    throw std::runtime_error("FAPP vehicle top/bottom heights must be positive");
  nh.param("fapp/planner_priority", fapp_planner_priority_, false);
  nh.param("fapp/memory_enabled", fapp_memory_enabled_, false);
  nh.param("fapp/observation_retention", fapp_config_.observation_retention, 0.8);
  nh.param("fapp/history_duration", fapp_history_duration_, 10.0);
  nh.param("fapp/association_distance", fapp_association_distance_, 0.6);
  if (!std::isfinite(fapp_config_.observation_retention) ||
      fapp_config_.observation_retention <= 0.0 ||
      !std::isfinite(fapp_association_distance_) || fapp_association_distance_ <= 0.0 ||
      !std::isfinite(fapp_history_duration_) || fapp_history_duration_ <= 0.0)
    throw std::runtime_error("FAPP retention, history duration and association distance must be positive");
  nh.param<std::string>("fapp/frame_id", fapp_frame_, "UAV0/camera_init");
  if (fapp_config_.enabled) {
    std::string topic;
    nh.param<std::string>("fapp/topic", topic, "/UAV0/fapp/states");
    fapp_sub_ = nh.subscribe(topic, 10, &FastPlannerManager::fappCallback, this);
    if (fapp_memory_enabled_) {
      fapp_memory_pub_ = nh.advertise<obj_state_msgs::ObjectsStates>("fapp/memory", 1);
      fapp_memory_status_pub_ = nh.advertise<std_msgs::String>("fapp/memory_status", 1);
      fapp_memory_timer_ = nh.createTimer(ros::Duration(0.1),
          &FastPlannerManager::publishFappMemory, this);
    }
  }

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization,
      use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_active_perception", use_active_perception, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    path_finder_.reset(new Astar);
    // path_finder_->setParam(nh);
    // path_finder_->setEnvironment(edt_environment_);
    // path_finder_->init();
    path_finder_->init(nh, edt_environment_);
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }

  if (use_active_perception) {
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    heading_planner_.reset(new HeadingPlanner(nh));
    heading_planner_->setMap(sdf_map_);
    visib_util_.reset(new VisibilityUtil(nh));
    visib_util_->setEDTEnvironment(edt_environment_);
    plan_data_.view_cons_.idx_ = -1;
  }
}

void FastPlannerManager::fappCallback(const obj_state_msgs::ObjectsStates::ConstPtr& message) {
  if (!message->header.frame_id.empty() && message->header.frame_id != fapp_frame_) {
    ROS_WARN_THROTTLE(2.0, "[FAPP] verify state frame %s against %s",
                      message->header.frame_id.c_str(), fapp_frame_.c_str());
    if (fapp_memory_enabled_) return;
  }
  const double now = ros::Time::now().toSec();
  std::vector<fuel_fapp::DynamicObstacleState> observations;
  for (const auto& state : message->states) {
    fuel_fapp::DynamicObstacleState obstacle;
    obstacle.position = Eigen::Vector3d(state.position.x,state.position.y,state.position.z);
    obstacle.velocity = Eigen::Vector3d(state.velocity.x,state.velocity.y,state.velocity.z);
    obstacle.size = Eigen::Vector3d(state.size.x,state.size.y,state.size.z);
    const ros::Time stamp = state.header.stamp.isZero() ? message->header.stamp : state.header.stamp;
    obstacle.stamp = stamp.isZero() ? ros::Time::now().toSec() : stamp.toSec();
    if (fapp_lateral_motion_only_) {
      obstacle.motion_axis = fapp_lateral_axis_;
      // 同一障碍沿其原通道摆动，飞机转向不能旋转旧障碍的运动轴。
      double nearest = fapp_association_distance_;
      for (const auto& old : fapp_memory_) {
        const double distance = (old.position - obstacle.position).norm();
        if (now - old.stamp <= fapp_config_.observation_retention &&
            old.motion_axis.squaredNorm() > 0.5 && distance < nearest) {
          nearest = distance;
          obstacle.motion_axis = old.motion_axis;
        }
      }
      obstacle.velocity = obstacle.motion_axis * obstacle.velocity.dot(obstacle.motion_axis);
    }
    if (fapp_memory_enabled_ &&
        (stamp.isZero() || !obstacle.position.allFinite() || !obstacle.velocity.allFinite() ||
         (!state.header.frame_id.empty() && state.header.frame_id != fapp_frame_) ||
         obstacle.stamp > now + 0.05 || now - obstacle.stamp > fapp_config_.observation_retention))
      continue;
    observations.push_back(obstacle);
  }
  if (!fapp_memory_enabled_) {
    fapp_states_ = observations;
  } else {
    // 新鲜运动状态与真实观测历史分开；空帧不能抹掉已经看到的摆动最低点。
    fapp_history_.erase(std::remove_if(fapp_history_.begin(), fapp_history_.end(),
        [&](const fuel_fapp::DynamicObstacleState& state) {
          return now - state.stamp > fapp_history_duration_;
        }), fapp_history_.end());
    // 同一位置已有多帧观测时，以历史尺寸为准。单帧竖直大框不能
    // 将已确认的摆球底部拉到地面；正常尺寸的更低观测仍保留。
    observations.erase(std::remove_if(observations.begin(), observations.end(),
        [&](const fuel_fapp::DynamicObstacleState& fresh) {
          return fappSizeOutlier(fresh);
        }), observations.end());
    for (const auto& fresh : observations) {
      const bool duplicate = std::any_of(fapp_history_.begin(), fapp_history_.end(),
          [&](const fuel_fapp::DynamicObstacleState& old) {
            return old.stamp == fresh.stamp && old.position.isApprox(fresh.position);
          });
      if (!duplicate) fapp_history_.push_back(fresh);
    }
    fapp_memory_.erase(std::remove_if(fapp_memory_.begin(), fapp_memory_.end(),
        [&](const fuel_fapp::DynamicObstacleState& state) {
          return now - state.stamp > fapp_config_.observation_retention;
        }), fapp_memory_.end());
    const auto distance_to = [&](const fuel_fapp::DynamicObstacleState& old,
                                 const fuel_fapp::DynamicObstacleState& fresh) {
      const double dt = std::max(0.0, std::min(fresh.stamp - old.stamp,
                                              fapp_config_.prediction_horizon));
      // 只关心同一片空间内是否仍有障碍，兼顾摆球换向，不维护身份ID。
      return std::min((fresh.position - old.position).norm(),
                     (fresh.position - old.position - dt * old.velocity).norm());
    };
    if (fapp_watched_valid_) {
      double nearest = fapp_association_distance_;
      const fuel_fapp::DynamicObstacleState* update = nullptr;
      for (const auto& fresh : observations) {
        const double distance = distance_to(fapp_watched_state_, fresh);
        if (fresh.stamp > fapp_watched_state_.stamp && distance <= nearest) {
          nearest = distance;
          update = &fresh;
        }
      }
      if (update) {
        fapp_watched_state_ = *update;
        includeFappHistory(*update, fapp_watched_lower_, fapp_watched_upper_,
                           &fapp_watched_bottom_history_, &fapp_watched_along_bounds_);
      }
    }
    // 每个已有区域每帧最多更新一次，避免不同位置的障碍互相覆盖。
    std::vector<bool> updated(fapp_memory_.size(), false);
    for (const auto& fresh : observations) {
      double nearest = fapp_association_distance_;
      size_t index = fapp_memory_.size();
      for (size_t i = 0; i < updated.size(); ++i) {
        const double distance = distance_to(fapp_memory_[i], fresh);
        if (!updated[i] && distance <= nearest) {
          nearest = distance;
          index = i;
        }
      }
      if (index == fapp_memory_.size()) {
        fapp_memory_.push_back(fresh);
      } else {
        updated[index] = true;
        // 乱序/重发不能回退位置或延长有效期。
        if (fresh.stamp > fapp_memory_[index].stamp) fapp_memory_[index] = fresh;
      }
    }
    fapp_states_ = fapp_memory_;
    ROS_INFO_THROTTLE(1.0, "[FAPP memory] observed=%zu retained=%zu watched=%d",
        observations.size(), fapp_states_.size(), static_cast<int>(fapp_watched_valid_));
  }
  for (auto& optimizer : bspline_optimizers_)
    optimizer->setFappObstacles(fapp_states_, fapp_config_);
}

bool FastPlannerManager::fappSizeOutlier(
    const fuel_fapp::DynamicObstacleState& candidate) const {
  std::vector<double> heights;
  for (const auto& reference : fapp_history_)
    if (!(reference.stamp == candidate.stamp &&
          reference.position.isApprox(candidate.position)) &&
        ros::Time::now().toSec() - reference.stamp <= fapp_history_duration_ &&
        reference.size.allFinite() && reference.size.z() > 0.0 &&
        (reference.position - candidate.position).norm() <= fapp_association_distance_)
      heights.push_back(reference.size.z());
  if (heights.size() < 3) return false;
  auto middle = heights.begin() + heights.size() / 2;
  std::nth_element(heights.begin(), middle, heights.end());
  if (candidate.size.z() <= 2.0 * *middle + 0.05) return false;
  ROS_WARN_THROTTLE(1.0, "[FAPP memory] ignore isolated tall box z_size=%.3f "
      "historical_median=%.3f samples=%zu; retain confirmed region.",
      candidate.size.z(), *middle, heights.size());
  return true;
}

void FastPlannerManager::includeFappHistory(
    const fuel_fapp::DynamicObstacleState& observation,
    Eigen::Vector3d& lower, Eigen::Vector3d& upper,
    std::vector<fuel_fapp::DynamicObstacleState>* bottom_history,
    Eigen::Vector2d* along_bounds) const {
  std::vector<fuel_fapp::DynamicObstacleState> local_bottom_history;
  auto& sources = bottom_history ? *bottom_history : local_bottom_history;
  const auto include = [&](const fuel_fapp::DynamicObstacleState& state) {
    // 旧帧进入历史时可能尚无足够尺寸证据；合并时必须用后来确认的尺寸复核。
    // 同尺寸的真实低点仍保留，不能因旧竖直大框永久拉低锁定包络。
    if (fappSizeOutlier(state)) return;
    if (std::none_of(sources.begin(), sources.end(), [&](const auto& old) {
          return old.stamp == state.stamp && old.position.isApprox(state.position);
        })) sources.push_back(state);
    Eigen::Vector3d half = Eigen::Vector3d::Constant(fapp_config_.ball_radius);
    if (state.size.allFinite() && (state.size.array() >= 0.0).all() &&
        (state.size.array() > 0.0).any()) half = 0.5 * state.size;
    half += Eigen::Vector3d(fapp_config_.vehicle_radius + fapp_config_.margin,
                           fapp_config_.vehicle_radius + fapp_config_.margin,
                           fapp_config_.lowerReferenceClearance());
    lower = lower.cwiseMin(state.position - half);
    Eigen::Vector3d upper_extent = half;
    upper_extent.z() += fapp_config_.vehicle_bottom_height + fapp_config_.margin -
                        fapp_config_.lowerReferenceClearance();
    upper = upper.cwiseMax(state.position + upper_extent);
  };
  include(observation);
  for (const auto& state : fapp_history_)
    if (ros::Time::now().toSec() - state.stamp <= fapp_history_duration_ &&
        (state.position - observation.position).norm() <= fapp_association_distance_)
      include(state);

  // 下沿仍按独立帧证据复核；横向模式随后单独约束前后和上沿。
  // 保留来源而非只保存min，后续正常证据才能撤销早期混入的孤立低点。
  std::map<double, double> frame_bottoms;
  for (const auto& state : sources) {
    if (fappSizeOutlier(state)) continue;
    double half_z = fapp_config_.ball_radius;
    if (state.size.allFinite() && (state.size.array() >= 0.0).all() &&
        (state.size.array() > 0.0).any()) half_z = 0.5 * state.size.z();
    const double bottom = state.position.z() - half_z;
    auto inserted = frame_bottoms.emplace(state.stamp, bottom);
    if (!inserted.second) inserted.first->second = std::min(inserted.first->second, bottom);
  }
  if (frame_bottoms.empty()) return;
  std::vector<double> sorted;
  double median_stamp = -std::numeric_limits<double>::infinity();
  for (const auto& frame : frame_bottoms) {
    if (frame.first - median_stamp < 0.05 - 1e-6) continue;
    sorted.push_back(frame.second);
    median_stamp = frame.first;
  }
  std::sort(sorted.begin(), sorted.end());
  const double median = sorted[sorted.size() / 2];
  // 初始证据不足时不靠一帧更高观测抬高包络；有足够独立观测才复核异常。
  double trusted = sorted.size() < 3 ? sorted.front() : median;
  for (const auto& candidate : frame_bottoms) {
    const double bottom = candidate.second;
    if (bottom >= trusted) continue;
    if (bottom < median - 0.10) {
      int support = 0;
      double last_stamp = -std::numeric_limits<double>::infinity();
      for (const auto& frame : frame_bottoms) {
        if (std::fabs(frame.second - bottom) <= 0.05 + 1e-6 &&
            frame.first - last_stamp >= 0.05 - 1e-6) {
          ++support;
          last_stamp = frame.first;
        }
      }
      if (support < 3) {
        ROS_WARN_THROTTLE(1.0, "[FAPP bottom] defer isolated low bottom=%.3f "
            "median=%.3f support=%d/3; keep historical observations for confirmation.",
            bottom, median, support);
        continue;
      }
    }
    trusted = bottom;
  }
  if (observation.motion_axis.squaredNorm() > 0.5) {
    const Eigen::Vector3d side = observation.motion_axis.normalized();
    const Eigen::Vector3d forward(side.y(), -side.x(), 0.0);
    std::vector<double> along, tops;
    double side_min = std::numeric_limits<double>::infinity();
    double side_max = -side_min, along_half = 0.0;
    for (const auto& state : sources) {
      if (fappSizeOutlier(state)) continue;
      Eigen::Vector3d half = Eigen::Vector3d::Constant(fapp_config_.ball_radius);
      if (state.size.allFinite() && (state.size.array() >= 0.0).all() &&
          (state.size.array() > 0.0).any()) half = 0.5 * state.size;
      along.push_back(state.position.dot(forward));
      tops.push_back(state.position.z() + half.z());
      along_half = std::max(along_half, half.dot(forward.cwiseAbs()));
      const double side_half = half.dot(side.cwiseAbs());
      side_min = std::min(side_min, state.position.dot(side) - side_half);
      side_max = std::max(side_max, state.position.dot(side) + side_half);
    }
    if (!along.empty()) {
      std::sort(along.begin(), along.end());
      std::sort(tops.begin(), tops.end());
      const double center_along = along[along.size()/2];
      const double padding = fapp_config_.vehicle_radius + fapp_config_.margin;
      along_half += padding;
      side_min -= padding;
      side_max += padding;
      // 历史只扩展左右摆幅；前后中心及上沿取观测中值，不累计抖动极值。
      const Eigen::Vector3d center = center_along * forward + 0.5*(side_min+side_max)*side;
      const Eigen::Vector3d extent = along_half*forward.cwiseAbs() +
                                    0.5*(side_max-side_min)*side.cwiseAbs();
      lower.head<2>() = (center-extent).head<2>();
      upper.head<2>() = (center+extent).head<2>();
      upper.z() = std::max(trusted, tops[tops.size()/2]) +
                  fapp_config_.vehicle_bottom_height + fapp_config_.margin;
      if (along_bounds) *along_bounds = Eigen::Vector2d(center_along-along_half,
                                                        center_along+along_half);
    }
  }
  lower.z() = trusted - fapp_config_.lowerReferenceClearance();
}

void FastPlannerManager::watchFappObstacle(
    const fuel_fapp::DynamicObstacleState* observation) {
  fapp_watched_valid_ = observation != nullptr;
  fapp_watched_bottom_history_.clear();
  if (!observation) return;
  fapp_watched_state_ = *observation;
  fapp_watched_lower_.setConstant(std::numeric_limits<double>::infinity());
  fapp_watched_upper_.setConstant(-std::numeric_limits<double>::infinity());
  includeFappHistory(*observation, fapp_watched_lower_, fapp_watched_upper_,
                     &fapp_watched_bottom_history_, &fapp_watched_along_bounds_);
}

bool FastPlannerManager::watchedFappBounds(
    Eigen::Vector3d& lower, Eigen::Vector3d& upper, double& age) const {
  if (!fappMemoryEnabled() || !fapp_watched_valid_) return false;
  age = std::max(0.0, ros::Time::now().toSec() - fapp_watched_state_.stamp);
  // 返回已按运动轴整理的历史包络，下沿保留独立帧确认规则。
  lower = fapp_watched_lower_;
  upper = fapp_watched_upper_;
  return true;
}

bool FastPlannerManager::isWatchedFappPositionSafe(const Eigen::Vector3d& point) const {
  if (!fappMemoryEnabled() || !fapp_watched_valid_) return true;
  if (fapp_watched_state_.motion_axis.squaredNorm() > 0.5) {
    const Eigen::Vector3d side = fapp_watched_state_.motion_axis.normalized();
    const double along = point.dot(Eigen::Vector3d(side.y(), -side.x(), 0.0));
    if (along < fapp_watched_along_bounds_.x() || along > fapp_watched_along_bounds_.y())
      return true;
  }
  if (fapp_config_.underpass_reference_clearance >= 0.0 &&
      point.z() <= fapp_watched_lower_.z() + 1e-9) return true;
  return (point.array() < fapp_watched_lower_.array()).any() ||
         (point.array() > fapp_watched_upper_.array()).any();
}

double FastPlannerManager::fappUnderpassTargetHeight(
    double lower_z, double tracking_margin) const {
  // 已按原始底部扣过总间距的下沿就是目标高度，不能再扣跟踪余量。
  return lower_z - (fapp_config_.underpass_reference_clearance >= 0.0
                       ? 0.0 : tracking_margin);
}

void FastPlannerManager::publishFappMemory(const ros::TimerEvent&) {
  const double now = ros::Time::now().toSec();
  obj_state_msgs::ObjectsStates message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = fapp_frame_;
  std::ostringstream detail;
  detail << "watched=" << fapp_watched_valid_;
  std::vector<fuel_fapp::DynamicObstacleState> visible;
  for (const auto& obstacle : fapp_memory_)
    if (now - obstacle.stamp <= fapp_config_.observation_retention) visible.push_back(obstacle);
  if (fapp_watched_valid_ && now - fapp_watched_state_.stamp > fapp_config_.observation_retention)
    visible.push_back(fapp_watched_state_);
  for (const auto& obstacle : visible) {
    const double age = std::max(0.0, now - obstacle.stamp);
    const bool fresh = age <= fapp_config_.observation_retention;
    obj_state_msgs::State state;
    state.header.frame_id = fapp_frame_;
    state.header.stamp.fromSec(obstacle.stamp);
    state.position.x = obstacle.position.x(); state.position.y = obstacle.position.y();
    state.position.z = obstacle.position.z();
    state.velocity.x = obstacle.velocity.x(); state.velocity.y = obstacle.velocity.y();
    state.velocity.z = obstacle.velocity.z();
    state.size.x = obstacle.size.x(); state.size.y = obstacle.size.y(); state.size.z = obstacle.size.z();
    message.states.push_back(state);
    detail << "; age=" << age
           << " status=" << (fresh ? "retained" : "historical")
           << " xyz=" << obstacle.position.transpose();
  }
  std_msgs::String status;
  if (fapp_watched_valid_) {
    detail << "; historical_lower=" << fapp_watched_lower_.transpose()
           << " historical_upper=" << fapp_watched_upper_.transpose();
    if (fapp_watched_state_.motion_axis.squaredNorm() > 0.5)
      detail << " lateral_axis=" << fapp_watched_state_.motion_axis.transpose()
             << " along_bounds=" << fapp_watched_along_bounds_.transpose();
  }
  status.data = detail.str();
  fapp_memory_pub_.publish(message);
  fapp_memory_status_pub_.publish(status);
}

bool FastPlannerManager::fappPositionSafe(const Eigen::Vector3d& point,
                                         double query, double reference) const {
  for (const auto& obstacle : fapp_states_) {
    double clearance;
    if (fuel_fapp::evaluateDynamicObstacle(point,obstacle,query,reference,
                                          fapp_config_,&clearance) && clearance < 0.0)
      return false;
  }
  return true;
}

bool FastPlannerManager::isRelayHoldDynamicallySafe(
    const Eigen::Vector3d& point, double radius, double half_height, double horizon) const {
  if (!point.allFinite() || !std::isfinite(horizon) || horizon <= 0.0 ||
      !std::isfinite(radius) || radius <= 0.0 ||
      !std::isfinite(half_height) || half_height <= 0.0) return false;
  if (!fapp_config_.enabled) return true;
  auto config = fapp_config_;
  // 中继悬停仍使用自身扩展机体模型，不套用飞行下穿的参考点间距。
  config.underpass_reference_clearance = -1.0;
  config.vehicle_radius = std::max(config.vehicle_radius, radius);
  config.vehicle_top_height = std::max(config.vehicle_top_height, half_height);
  config.vehicle_bottom_height = std::max(config.vehicle_bottom_height, half_height);
  const double reference = ros::Time::now().toSec();
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon / 0.1)));
  const double dt = horizon / steps;
  for (const auto& obstacle : fapp_states_) {
    auto swept_config = config;
    // Cover motion between time samples as well as the sampled obstacle centres.
    swept_config.margin += obstacle.velocity.norm() * dt * 0.5;
    for (int i = 0; i <= steps; ++i) {
      double clearance;
      if (fuel_fapp::evaluateDynamicObstacle(point, obstacle, reference + i * dt,
                                             reference, swept_config, &clearance) &&
          clearance <= 0.0) return false;
    }
  }
  return true;
}

bool FastPlannerManager::fappBlockingBounds(
    const vector<Eigen::Vector3d>& path, Eigen::Vector3d& lower,
    Eigen::Vector3d& upper, fuel_fapp::DynamicObstacleState* observation) const {
  if (!fapp_config_.enabled || path.size() < 2) return false;
  const double reference = ros::Time::now().toSec();
  const double speed = std::max(0.1, pp_.max_vel_ * 0.5);
  double elapsed = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const double length = (path[i] - path[i - 1]).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
    for (int j = 0; j <= samples; ++j) {
      const double u = static_cast<double>(j) / samples;
      const Eigen::Vector3d point = path[i - 1] + u * (path[i] - path[i - 1]);
      if (!isWatchedFappPositionSafe(point)) {
        lower = fapp_watched_lower_;
        upper = fapp_watched_upper_;
        if (observation) *observation = fapp_watched_state_;
        return true;
      }
      for (const auto& obstacle : fapp_states_) {
        double clearance;
        Eigen::Vector3d center;
        if (!fuel_fapp::evaluateDynamicObstacle(
                point, obstacle, reference + elapsed + u * length / speed,
                reference, fapp_config_, &clearance, nullptr, &center) ||
            clearance >= 0.0)
          continue;
        Eigen::Vector3d half = Eigen::Vector3d::Constant(fapp_config_.ball_radius);
        if (obstacle.size.allFinite() && (obstacle.size.array() >= 0.0).all() &&
            (obstacle.size.array() > 0.0).any())
          half = 0.5 * obstacle.size;
        half += Eigen::Vector3d(fapp_config_.vehicle_radius + fapp_config_.margin,
                               fapp_config_.vehicle_radius + fapp_config_.margin,
                               fapp_config_.lowerReferenceClearance());
        lower = center - half;
        upper = center + half;
        upper.z() += fapp_config_.vehicle_bottom_height + fapp_config_.margin -
                     fapp_config_.lowerReferenceClearance();
        if (observation) *observation = obstacle;
        return true;
      }
    }
    elapsed += length / speed;
  }
  if (fappMemoryEnabled() && !fapp_watched_valid_) {
    for (const auto& state : fapp_history_) {
      if (reference - state.stamp > fapp_history_duration_) continue;
      Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d hi = -lo;
      Eigen::Vector2d along_bounds(-std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity());
      includeFappHistory(state, lo, hi, nullptr, &along_bounds);
      for (size_t i = 1; i < path.size(); ++i) {
        const int count = std::max(1, static_cast<int>(std::ceil((path[i] - path[i-1]).norm()/0.05)));
        for (int j = 0; j <= count; ++j) {
          const Eigen::Vector3d p = path[i-1] + (path[i]-path[i-1]) * (double(j)/count);
          if (state.motion_axis.squaredNorm() > 0.5) {
            const Eigen::Vector3d side = state.motion_axis.normalized();
            const double along = p.dot(Eigen::Vector3d(side.y(), -side.x(), 0.0));
            if (along < along_bounds.x() || along > along_bounds.y()) continue;
          }
          if ((p.array() >= lo.array()).all() && (p.array() <= hi.array()).all() &&
              (fapp_config_.underpass_reference_clearance < 0.0 || p.z() > lo.z()+1e-9)) {
            lower = lo; upper = hi;
            if (observation) *observation = state;
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool FastPlannerManager::fappSideOpening(
    const Eigen::Vector3d& position, const Eigen::Vector3d& side) const {
  if (!fappMemoryEnabled() || !fapp_watched_valid_ || side.head<2>().norm() < 0.5)
    return false;
  const auto& ball = fapp_watched_state_;
  const double age = ros::Time::now().toSec() - ball.stamp;
  if (age < -0.05 || age > fapp_config_.observation_retention ||
      !ball.velocity.allFinite() || ball.velocity.dot(side) >= -1e-3) return false;
  const double half = ball.size.allFinite() && (ball.size.array() > 0.0).any()
      ? 0.5 * ball.size.dot(side.cwiseAbs()) : fapp_config_.ball_radius;
  // 已选右侧只认球向左摆且横向让出机体净空；历史摆幅仍用于确定通过终点。
  return (position - ball.position).dot(side) >
      half + fapp_config_.vehicle_radius + fapp_config_.margin;
}

bool FastPlannerManager::isFappCrossingTrajectorySafe() const {
  auto trajectory = local_data_.position_traj_;
  const double duration = trajectory.getTimeSum();
  if (!(duration > 0.0)) return false;
  const double now = ros::Time::now().toSec();
  const int samples = std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
  for (int i = 0; i <= samples; ++i) {
    const double t = duration * i / samples;
    const auto point = trajectory.evaluateDeBoorT(t);
    // 此分支按新鲜观测和实际样条到达时间验球，不能用整段历史摆幅封死空隙。
    if (!fappPositionSafe(point, now + t, now)) return false;
    double clearance;
    if (fapp_watched_valid_ && fuel_fapp::evaluateDynamicObstacle(
        point, fapp_watched_state_, now + t, now, fapp_config_, &clearance) &&
        clearance < 0.0) return false;
  }
  return true;
}

bool FastPlannerManager::isFappPathSafe(const vector<Eigen::Vector3d>& path) const {
  Eigen::Vector3d lower, upper;
  return !fappBlockingBounds(path, lower, upper);
}

bool FastPlannerManager::fappGuidePath(std::vector<Eigen::Vector3d>& path) const {
  // Planner priority retains the B-spline dynamic cost without requiring a clear guide.
  if (fapp_planner_priority_ || !fapp_config_.enabled || fapp_states_.empty() || path.size()<2) return true;
  const double reference=ros::Time::now().toSec();
  const double speed=std::max(0.1,pp_.max_vel_*0.5);
  auto clear=[&](const std::vector<Eigen::Vector3d>& candidate,
                 fuel_fapp::DynamicObstacleState* hit, Eigen::Vector3d* center) {
    double elapsed=0;
    for(size_t i=1;i<candidate.size();++i) {
      const double length=(candidate[i]-candidate[i-1]).norm();
      const int count=std::max(1,static_cast<int>(std::ceil(length/0.05)));
      for(int j=0;j<=count;++j) {
        const double u=double(j)/count;
        const Eigen::Vector3d point=candidate[i-1]+u*(candidate[i]-candidate[i-1]);
        for(const auto& obstacle:fapp_states_) {
          double clearance; Eigen::Vector3d predicted;
          if(fuel_fapp::evaluateDynamicObstacle(point,obstacle,
              reference+elapsed+u*length/speed,reference,fapp_config_,&clearance,
              nullptr,&predicted) && clearance<0.0) {
            if(hit)*hit=obstacle;if(center)*center=predicted;return false;
          }
        }
      }
      elapsed+=length/speed;
    }
    return true;
  };
  fuel_fapp::DynamicObstacleState hit;Eigen::Vector3d center;
  if(clear(path,&hit,&center))return true;
  Eigen::Vector3d axis=path.back()-path.front();axis.z()=0;
  if(axis.norm()<1e-6)return false;
  axis.normalize();const Eigen::Vector3d side(-axis.y(),axis.x(),0);
  Eigen::Vector3d half=Eigen::Vector3d::Constant(fapp_config_.ball_radius);
  if(hit.size.allFinite()&&(hit.size.array()>=0).all()&&(hit.size.array()>0).any())half=.5*hit.size;
  // The same three geometric choices as the rear planner, checked in FUEL's map.
  const double along=half.dot(axis.cwiseAbs())+fapp_config_.vehicle_radius+fapp_config_.margin+0.15;
  const double lateral=half.dot(side.cwiseAbs())+fapp_config_.vehicle_radius+fapp_config_.margin+0.15;
  const double below=half.z()+fapp_config_.vehicle_top_height+fapp_config_.margin+0.15;
  std::vector<Eigen::Vector3d> best;double best_length=std::numeric_limits<double>::infinity();
  for(int direction=0;direction<3;++direction) {
    const Eigen::Vector3d offset=direction==0?Eigen::Vector3d(0,0,-below):
        Eigen::Vector3d((direction==1?1.0:-1.0)*lateral*side);
    std::vector<Eigen::Vector3d> candidate{path.front(),center-along*axis+offset,
                                          center+along*axis+offset,path.back()};
    double length=0;bool distinct=true;
    for(size_t i=1;i<candidate.size();++i){double d=(candidate[i]-candidate[i-1]).norm();
      distinct=distinct&&d>1e-3;length+=d;}
    if(distinct&&length<best_length&&isPathSafe(candidate)&&clear(candidate,nullptr,nullptr)) {
      best=candidate;best_length=length;
    }
  }
  if(best.empty())return false;
  path.swap(best);return true;
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance, bool allow_inflation_escape) {
  return checkTrajCollision(local_data_, distance, allow_inflation_escape);
}

bool FastPlannerManager::checkTrajCollision(LocalTrajData& trajectory, double& distance,
                                            bool allow_inflation_escape) {
  const double t_now = std::max(
      0.0, std::min((ros::Time::now() - trajectory.start_time_).toSec(), trajectory.duration_));

  const double reference = ros::Time::now().toSec();
  Eigen::Vector3d cur_pt = trajectory.position_traj_.evaluateDeBoorT(t_now);
  if (!fapp_planner_priority_ && !fappPositionSafe(cur_pt, trajectory.start_time_.toSec()+t_now, reference)) {
    distance=0.0; return false;
  }
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;
  double previous_clearance = sdf_map_->getDistance(cur_pt);
  int previous_contacts = rawFootprintCollisionCount(cur_pt);
  // 机体半径大于建图膨胀时，中心可能尚未进入膨胀层但旋翼边缘已经触墙；
  // 这两种起点都使用同一套“接触不增加、净空不下降”逃逸规则。
  bool escaping_inflation =
      allow_inflation_escape &&
      (sdf_map_->getInflateOccupancy(cur_pt) != 0 || previous_contacts > 0);
  if (escaping_inflation &&
      previous_contacts > escape_max_initial_occupied_samples_) {
    distance = 0.0;
    return false;
  }
  if (!escaping_inflation && !isPositionSafe(cur_pt)) {
    distance = 0.0;
    ROS_WARN_THROTTLE(0.5,
                      "[footprint_safety] active trajectory point is occupied at %.2f %.2f %.2f, "
                      "radius=%.2f.",
                      cur_pt.x(), cur_pt.y(), cur_pt.z(), footprint_check_radius_);
    return false;
  }

  while (radius < 6.0 && t_now + fut_t < trajectory.duration_) {
    fut_pt = trajectory.position_traj_.evaluateDeBoorT(t_now + fut_t);
    if (!fapp_planner_priority_ && !fappPositionSafe(fut_pt, trajectory.start_time_.toSec()+t_now+fut_t, reference)) {
      distance=radius; return false;
    }
    // double dist = edt_environment_->sdf_map_->getDistance(fut_pt);
    // 2026-07-13: 不能只检查轨迹中心点；橙色柱旁中心安全时，Iris 旋翼仍可能已经接触障碍物。
    bool safe = false;
    const int contacts = rawFootprintCollisionCount(fut_pt);
    const bool still_needs_escape =
        sdf_map_->getInflateOccupancy(fut_pt) != 0 || contacts > 0;
    if (escaping_inflation && still_needs_escape) {
      const double clearance = sdf_map_->getDistance(fut_pt);
      safe = contacts <= previous_contacts &&
             clearance + 0.02 >= previous_clearance;
      previous_contacts = std::min(previous_contacts, contacts);
      previous_clearance = std::max(previous_clearance, clearance);
    } else {
      escaping_inflation = false;
      safe = isPositionSafe(fut_pt);
    }
    if (!safe) {
      distance = radius;
      ROS_WARN_THROTTLE(0.5, "[footprint_safety] future footprint collision at %.2f %.2f %.2f, "
                             "path_dist=%.2f radius=%.2f.",
                        fut_pt.x(), fut_pt.y(), fut_pt.z(), radius, footprint_check_radius_);
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

bool FastPlannerManager::isSupportedOccupied(const Eigen::Vector3d& position) const {
  if (!sdf_map_ || sdf_map_->getOccupancy(position) != SDFMap::OCCUPIED)
    return false;
  if (!supported_occupancy_hard_reject_enabled_) {
    ROS_WARN_THROTTLE(0.5,
                      "[footprint_safety] supported occupancy hard rejection disabled; "
                      "ignore raw occupied footprint contacts.");
    return false;
  }
  Eigen::Vector3i center;
  sdf_map_->posToIndex(position, center);
  const int support_radius = std::max(
      1, metric_voxel::radiusInVoxels(
             footprint_support_radius_, sdf_map_->getResolution()));
  const bool supported = metric_voxel::hasMinimumSupport(
      support_radius, footprint_min_occupied_support_, true,
      [&](int dx, int dy, int dz) {
        const Eigen::Vector3i neighbor = center + Eigen::Vector3i(dx, dy, dz);
        return sdf_map_->isInMap(neighbor) &&
               sdf_map_->getOccupancy(neighbor) == SDFMap::OCCUPIED;
      });
  if (supported) return true;
  ROS_WARN_THROTTLE(0.5,
                    "[footprint_safety] ignore isolated occupied voxel at %.2f %.2f %.2f "
                    "support_radius=%.2fm min_support=%d.",
                    position.x(), position.y(), position.z(),
                    footprint_support_radius_, footprint_min_occupied_support_);
  return false;
}

bool FastPlannerManager::isRawFootprintSafe(const Eigen::Vector3d& position) const {
  return rawFootprintCollisionCount(position) == 0;
}

int FastPlannerManager::rawFootprintCollisionCount(
    const Eigen::Vector3d& position) const {
  if (!sdf_map_ || !sdf_map_->isInMap(position))
    return std::numeric_limits<int>::max();

  // 2026-07-23: 不再只采最外圆周；按中心、半径一半和最外沿组成真实圆盘，
  // 防止放宽膨胀层硬拒绝后漏掉位于旋翼圆盘内部、但恰好不在圆周采样点上的体素。
  int occupied_samples = 0;
  const double z_offsets[] = {-0.06, 0.0, 0.06};
  for (double z_offset : z_offsets) {
    for (int ring = 0; ring <= 2; ++ring) {
      const double radius = 0.5 * static_cast<double>(ring) * footprint_check_radius_;
      const int samples = ring == 0 ? 1 : footprint_check_samples_;
      for (int sample = 0; sample < samples; ++sample) {
        const double angle =
            2.0 * M_PI * static_cast<double>(sample) / static_cast<double>(samples);
        Eigen::Vector3d probe = position;
        probe.x() += radius * std::cos(angle);
        probe.y() += radius * std::sin(angle);
        probe.z() += z_offset;
        if (!sdf_map_->isInMap(probe)) return std::numeric_limits<int>::max();
        if (isSupportedOccupied(probe)) ++occupied_samples;
      }
    }
  }
  return occupied_samples;
}

int FastPlannerManager::lowerFootprintObstacleCount(
    const Eigen::Vector3d& position, double probe_depth) const {
  if (!sdf_map_ || !sdf_map_->isInMap(position))
    return -1;
  probe_depth = std::max(0.08, probe_depth);
  const double z_offsets[] = {-probe_depth, -0.5 * probe_depth, -0.06};
  int occupied_samples = 0;
  for (double z_offset : z_offsets) {
    for (int ring = 0; ring <= 2; ++ring) {
      const double radius = 0.5 * static_cast<double>(ring) * footprint_check_radius_;
      const int samples = ring == 0 ? 1 : footprint_check_samples_;
      for (int sample = 0; sample < samples; ++sample) {
        const double angle = 2.0 * M_PI * static_cast<double>(sample) /
                             static_cast<double>(samples);
        Eigen::Vector3d probe = position;
        probe.x() += radius * std::cos(angle);
        probe.y() += radius * std::sin(angle);
        probe.z() += z_offset;
        if (!sdf_map_->isInMap(probe)) return -1;
        if (isSupportedOccupied(probe)) ++occupied_samples;
      }
    }
  }
  return occupied_samples;
}

bool FastPlannerManager::isTurnPositionSafe(const Eigen::Vector3d& position) const {
  if (!position.allFinite()) return false;
  // 以里程计/地图Z坐标为准：低于20cm禁止转向，边界不向下放宽。
  // 规划抢占、候选发布和执行期均复用此检查；普通平移不受影响。
  if (position.z() < 0.20) {
    ROS_WARN_THROTTLE(0.5,
                      "[turn_in_place] reject turn below minimum Z: %.3fm < 0.200m.",
                      position.z());
    return false;
  }
  if (!isPositionSafe(position)) return false;
  // 转弯使用独立检测半径，不改变平移足迹；枚举整个yaw包络相交的体素。
  const double radius = turn_check_radius_;
  constexpr double half_height = 0.06;
  const double half_voxel = 0.5 * sdf_map_->getResolution();
  Eigen::Vector3i lower, upper;
  sdf_map_->posToIndex(position - Eigen::Vector3d(radius, radius, half_height), lower);
  sdf_map_->posToIndex(position + Eigen::Vector3d(radius, radius, half_height), upper);
  for (int x = lower.x(); x <= upper.x(); ++x)
    for (int y = lower.y(); y <= upper.y(); ++y)
      for (int z = lower.z(); z <= upper.z(); ++z) {
        const Eigen::Vector3i index(x, y, z);
        Eigen::Vector3d center;
        sdf_map_->indexToPos(index, center);
        const double dx = std::max(0.0, std::fabs(center.x() - position.x()) - half_voxel);
        const double dy = std::max(0.0, std::fabs(center.y() - position.y()) - half_voxel);
        if (dx * dx + dy * dy > radius * radius + 1e-9) continue;
        if (!sdf_map_->isInMap(index) || isSupportedOccupied(center)) return false;
      }
  return true;
}

bool FastPlannerManager::isPositionSafe(const Eigen::Vector3d& position) const {
  if (!isRawFootprintSafe(position)) return false;

  // 2026-07-27: 膨胀层接触对普通目标/路径点恢复为硬拒绝；起点脱困例外只由路径和轨迹
  // 首采样单独处理，禁止“allow forward escape”扩散到整条贴墙轨迹。
  if (sdf_map_->getInflateOccupancy(position) != 0) {
    ROS_WARN_THROTTLE(1.0,
                      "[footprint_safety] reject inflated-layer contact at %.2f %.2f %.2f.",
                      position.x(), position.y(), position.z());
    return false;
  }
  return true;
}

// 2026-07-28: 对外只暴露只读安全查询，真实圆盘采样实现仍集中在isRawFootprintSafe中。
bool FastPlannerManager::isRawPositionSafe(const Eigen::Vector3d& position) const {
  return isRawFootprintSafe(position);
}

bool FastPlannerManager::isKnownFreePosition(const Eigen::Vector3d& position) const {
  if (!position.allFinite() || !isPositionSafe(position)) return false;
  for (double z_offset : {-0.06, 0.0, 0.06}) {
    for (int ring = 0; ring <= 2; ++ring) {
      const double radius = 0.5 * ring * footprint_check_radius_;
      const int samples = ring == 0 ? 1 : footprint_check_samples_;
      for (int sample = 0; sample < samples; ++sample) {
        const double angle = 2.0 * M_PI * sample / samples;
        const Eigen::Vector3d probe = position + Eigen::Vector3d(
            radius * std::cos(angle), radius * std::sin(angle), z_offset);
        if (!sdf_map_->isInMap(probe) ||
            sdf_map_->getOccupancy(probe) != SDFMap::FREE)
          return false;
      }
    }
  }
  return true;
}

bool FastPlannerManager::isControlledEscapePosition(
    const Eigen::Vector3d& position) const {
  const int contacts = rawFootprintCollisionCount(position);
  return contacts <= escape_max_initial_occupied_samples_ &&
         (isPositionInflated(position) || contacts > 0);
}

// 2026-07-28: 不把地图外误当成普通膨胀接触；地图外仍由原始足迹检查直接拒绝。
bool FastPlannerManager::isPositionInflated(const Eigen::Vector3d& position) const {
  return sdf_map_ && sdf_map_->isInMap(position) &&
         sdf_map_->getInflateOccupancy(position) != 0;
}

bool FastPlannerManager::isPathSafe(const vector<Eigen::Vector3d>& path,
                                    bool allow_contact_escape) const {
  if (path.empty()) return false;
  // 2026-07-13: A* 使用中心栅格；按 0.05m 插值复核完整足迹，不能让稀疏路径点跨过细柱。
  // 2026-07-27: 起点位于膨胀层时，仅允许净空不下降的连续脱困段；一旦离开后严禁重新进入。
  int previous_contacts = rawFootprintCollisionCount(path.front());
  if ((!allow_contact_escape && previous_contacts != 0) ||
      previous_contacts > escape_max_initial_occupied_samples_)
    return false;
  bool escaping_inflation =
      sdf_map_->getInflateOccupancy(path.front()) != 0 ||
      (allow_contact_escape && previous_contacts > 0);
  double previous_clearance = sdf_map_->getDistance(path.front());
  if (escaping_inflation) {
    ROS_WARN_THROTTLE(1.0,
                      "[footprint_safety] path starts in inflated layer; require monotonic escape.");
  }
  for (size_t segment = 1; segment < path.size(); ++segment) {
    const Eigen::Vector3d delta = path[segment] - path[segment - 1];
    const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / 0.05)));
    for (int sample = 1; sample <= samples; ++sample) {
      const Eigen::Vector3d point =
          path[segment - 1] + delta * static_cast<double>(sample) / static_cast<double>(samples);
      const int contacts = rawFootprintCollisionCount(point);
      const bool still_needs_escape =
          sdf_map_->getInflateOccupancy(point) != 0 || contacts > 0;
      if (escaping_inflation && still_needs_escape) {
        const double clearance = sdf_map_->getDistance(point);
        if (contacts > previous_contacts || clearance + 0.02 < previous_clearance)
          return false;
        previous_contacts = std::min(previous_contacts, contacts);
        previous_clearance = std::max(previous_clearance, clearance);
      } else {
        escaping_inflation = false;
        if (!isPositionSafe(point)) return false;
      }
    }
  }
  return true;
}

bool FastPlannerManager::isTrajectorySafe(double sample_dt,
                                          bool allow_contact_escape) {
  // 2026-07-14: 此检查不依赖 start_time_，用于新轨迹写入 local_data_ 后、发布给控制器前的静态复核。
  const double duration = local_data_.position_traj_.getTimeSum();
  if (duration <= 0.0) return false;

  const double fapp_reference = ros::Time::now().toSec();
  sample_dt = std::max(0.01, sample_dt);
  const int samples = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
  bool escaping_inflation = false;
  double previous_clearance = 0.0;
  int previous_contacts = 0;
  for (int sample = 0; sample <= samples; ++sample) {
    const double t = duration * static_cast<double>(sample) / static_cast<double>(samples);
    const Eigen::Vector3d point = local_data_.position_traj_.evaluateDeBoorT(t);
    if (!fapp_planner_priority_ && !fappPositionSafe(point, fapp_reference+t, fapp_reference)) return false;
    // 2026-07-27: 与几何路径一致，轨迹起点若已在膨胀层只能沿ESDF净空不下降方向连续退出。
    bool safe = false;
    if (sample == 0) {
      previous_clearance = sdf_map_->getDistance(point);
      previous_contacts = rawFootprintCollisionCount(point);
      escaping_inflation = sdf_map_->getInflateOccupancy(point) != 0 ||
                           (allow_contact_escape && previous_contacts > 0);
      safe = previous_contacts == 0 ||
             (allow_contact_escape && escaping_inflation &&
              previous_contacts <= escape_max_initial_occupied_samples_);
    } else {
      const double clearance = sdf_map_->getDistance(point);
      const int contacts = rawFootprintCollisionCount(point);
      const bool still_needs_escape =
          sdf_map_->getInflateOccupancy(point) != 0 || contacts > 0;
      if (escaping_inflation && still_needs_escape) {
        safe = contacts <= previous_contacts &&
               clearance + 0.02 >= previous_clearance;
        previous_contacts = std::min(previous_contacts, contacts);
        previous_clearance = std::max(previous_clearance, clearance);
      } else {
        escaping_inflation = false;
        safe = isPositionSafe(point);
      }
    }
    if (!safe) {
      ROS_WARN_THROTTLE(0.5,
                        "[footprint_safety] reject generated trajectory before publish at "
                        "t=%.2f/%.2f pos=(%.2f, %.2f, %.2f).",
                        t, duration, point.x(), point.y(), point.z());
      return false;
    }
  }
  return true;
}

bool FastPlannerManager::trajectoryClearsInflation(double max_path_distance,
                                                   double sample_dt,
                                                   bool allow_contact_escape) {
  const double duration = local_data_.position_traj_.getTimeSum();
  if (duration <= 0.0) return false;
  const Eigen::Vector3d start = local_data_.position_traj_.evaluateDeBoorT(0.0);
  int previous_contacts = rawFootprintCollisionCount(start);
  if (!isPositionInflated(start) && previous_contacts == 0) return true;
  if (previous_contacts != 0 &&
      (!allow_contact_escape ||
       previous_contacts > escape_max_initial_occupied_samples_))
    return false;

  sample_dt = std::max(0.01, sample_dt);
  max_path_distance = std::max(0.20, max_path_distance);
  Eigen::Vector3d previous = start;
  double traveled = 0.0;
  double previous_clearance = sdf_map_->getDistance(start);
  for (double t = sample_dt; t <= duration + 1e-6; t += sample_dt) {
    const Eigen::Vector3d point =
        local_data_.position_traj_.evaluateDeBoorT(std::min(t, duration));
    traveled += (point - previous).norm();
    previous = point;
    const int contacts = rawFootprintCollisionCount(point);
    if (contacts > previous_contacts) return false;
    previous_contacts = std::min(previous_contacts, contacts);
    if (!isPositionInflated(point) && contacts == 0) return true;
    const double clearance = sdf_map_->getDistance(point);
    if (clearance + 0.02 < previous_clearance) return false;
    previous_clearance = std::max(previous_clearance, clearance);
    if (traveled > max_path_distance) break;
  }
  ROS_WARN_THROTTLE(0.5,
                    "[footprint_safety] reject escape trajectory: still inflated after %.2fm.",
                    traveled);
  return false;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d& start_pt,
    const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
    const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double& time_lb) {
  std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
            << ", " << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", "
            << end_vel.transpose() << endl;

  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // Kinodynamic path searching

  auto t1 = ros::Time::now();

  kino_path_finder_->reset();
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
  if (status == KinodynamicAstar::NO_PATH) {
    ROS_ERROR("search 1 fail");
    // Retry
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[Kino replan]: Can't find path." << endl;
      return false;
    }
  }
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  double t_search = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Parameterize path to B-spline
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  // std::cout << "point set:" << std::endl;
  // for (auto pt : point_set) std::cout << pt.transpose() << std::endl;
  // std::cout << "derivative:" << std::endl;
  // for (auto dr : start_end_derivatives) std::cout << dr.transpose() << std::endl;

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

  // B-spline-based optimization
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;
  vector<Eigen::Vector3d> start, end;
  init.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

  vector<Eigen::Vector3d> start2, end2;
  local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
  std::cout << "State error: (" << (start2[0] - start[0]).norm() << ", "
            << (start2[1] - start[1]).norm() << ", " << (start2[2] - start[2]).norm() << ")"
            << std::endl;

  double t_opt = (ros::Time::now() - t1).toSec();
  ROS_WARN("Kino t: %lf, opt: %lf", t_search, t_opt);

  // t1 = ros::Time::now();

  // // Adjust time and refine

  // double dt;
  // for (int i = 0; i < 2; ++i)
  // {
  //   NonUniformBspline pos = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);
  //   pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  //   pos.lengthenTime(min(1.01, pos.checkRatio()));
  //   double duration = pos.getTimeSum();
  //   dt = duration / double(pos.getControlPoint().rows() - pp_.bspline_degree_);

  //   point_set.clear();
  //   for (double time = 0.0; time <= duration + 1e-4; time += dt)
  //     point_set.push_back(pos.evaluateDeBoorT(time));
  //   NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivatives,
  //   pp_.bspline_degree_, ctrl_pts);
  //   bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_function, 1, 1);
  // }
  // local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  // iterative time adjustment

  // double to = pos.getTimeSum();
  // pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  // bool feasible = pos.checkFeasibility(false);

  // int iter_num = 0;
  // while (!feasible && ros::ok()) {

  //   feasible = pos.reallocateTime();

  //   if (++iter_num >= 3) break;
  // }

  // // pos.checkFeasibility(true);
  // // cout << "[Main]: iter num: " << iter_num << endl;

  // double tn = pos.getTimeSum();

  // cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  // if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  // t_adjust = (ros::Time::now() - t1).toSec();

  // // save planned results

  // local_data_.position_traj_ = pos;

  // double t_total = t_search + t_opt + t_adjust;
  // cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ",
  // optimize: " << t_opt
  //      << ", adjust time:" << t_adjust << endl;

  // pp_.time_search_   = t_search;
  // pp_.time_optimize_ = t_opt;
  // pp_.time_adjust_   = t_adjust;

  // int rd = rand() % 2;
  // if (rd == 0) {
  //   updateTrajInfo();
  //   return true;
  // } else
  //   return false;

  updateTrajInfo();
  return true;
}

bool FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d>& input_tour,
    const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc, const double& time_lb,
    bool preserve_height_profile, double crossing_max_velocity, double crossing_max_acceleration) {
  const double plan_max_velocity = crossing_max_velocity > 0.0
      ? crossing_max_velocity : pp_.max_vel_;
  const double plan_max_acceleration = crossing_max_acceleration > 0.0
      ? crossing_max_acceleration : pp_.max_acc_;
  std::vector<Eigen::Vector3d> tour=input_tour;

  // 2026-07-14: PolynomialTraj 至少需要两个有效分段；单点/两点及重复点会构造零维矩阵并触发 Eigen 越界。
  if (tour.size() < 3) {
    ROS_ERROR("[trajectory_input] reject exploration path with only %zu waypoint(s).", tour.size());
    return false;
  }
  if (!cur_vel.allFinite() || !cur_acc.allFinite()) {
    ROS_ERROR("[trajectory_input] reject non-finite boundary velocity or acceleration.");
    return false;
  }
  for (size_t i = 1; i < tour.size(); ++i) {
    if (!tour[i].allFinite() || (tour[i] - tour[i - 1]).norm() < 1e-3) {
      ROS_ERROR("[trajectory_input] reject invalid or duplicate waypoint at index %zu.", i);
      return false;
    }
  }

  if (!fappGuidePath(tour)) return false;

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (plan_max_velocity * 0.5);

  // 急弯两侧分段额外留时间，让简单控制器在进入横向段前先完成减速，
  // 避免参考点已经转弯而机体中后部仍在障碍物侧面的情况下斜切追点。
  const double slowdown_angle = turn_slowdown_angle_deg_ * M_PI / 180.0;
  for (int i = 1; i < pt_num - 1; ++i) {
    Eigen::Vector3d incoming = pos.row(i).transpose() - pos.row(i - 1).transpose();
    Eigen::Vector3d outgoing = pos.row(i + 1).transpose() - pos.row(i).transpose();
    incoming.z() = 0.0;
    outgoing.z() = 0.0;
    if (incoming.norm() < 1e-6 || outgoing.norm() < 1e-6) continue;
    const double cosine = std::max(
        -1.0, std::min(1.0, incoming.normalized().dot(outgoing.normalized())));
    const double angle = std::acos(cosine);
    if (angle < slowdown_angle) continue;
    const double normalized_angle = std::min(
        1.0, (angle - slowdown_angle) /
                 std::max(1e-6, M_PI_2 - slowdown_angle));
    const double scale = 1.0 + (turn_time_scale_ - 1.0) * normalized_angle;
    times(i - 1) *= scale;
    times(i) *= scale;
  }

  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  const double control_spacing = fappMemoryEnabled() && fapp_watched_valid_
      ? std::min(pp_.ctrl_pt_dist, 0.05) : pp_.ctrl_pt_dist;
  int seg_num = std::max(8, static_cast<int>(init_traj.getLength() / control_spacing));
  double dt = duration / double(seg_num);

  std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt << std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  // 2026-07-20: 将已通过完整机体足迹检查的A*折线按弧长采样为优化引导点，避免B样条在窄道中
  // 为追求平滑而切弯、绕回起飞区。GUIDE代价需要与可优化的内部控制点一一对应。
  const int guide_count = std::max(0, static_cast<int>(ctrl_pts.rows()) -
                                          2 * pp_.bspline_degree_);
  vector<Vector3d> guide_pts;
  guide_pts.reserve(guide_count);
  vector<double> cumulative_length(tour.size(), 0.0);
  for (size_t i = 1; i < tour.size(); ++i)
    cumulative_length[i] = cumulative_length[i - 1] + (tour[i] - tour[i - 1]).norm();
  const double total_length = cumulative_length.back();
  for (int guide_id = 0; guide_id < guide_count; ++guide_id) {
    const double ratio = static_cast<double>(guide_id + 1) /
                         static_cast<double>(guide_count + 1);
    const double target_length = ratio * total_length;
    size_t segment = 1;
    while (segment + 1 < cumulative_length.size() &&
           cumulative_length[segment] < target_length)
      ++segment;
    const double segment_length = cumulative_length[segment] - cumulative_length[segment - 1];
    const double alpha = segment_length > 1e-6
                             ? (target_length - cumulative_length[segment - 1]) / segment_length
                             : 0.0;
    guide_pts.push_back(tour[segment - 1] + alpha * (tour[segment] - tour[segment - 1]));
  }

  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  if (!guide_pts.empty()) {
    bspline_optimizers_[0]->setGuidePath(guide_pts);
    cost_func |= BsplineOptimizer::GUIDE;
  }
  if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

  vector<Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // 2026-07-20: START/END在原优化器中只是软代价；保存两端控制点并在优化后恢复，把起终点及其
  // 一、二阶边界状态变成硬约束，防止日志中轨迹末端从前方目标甩回(0,-3.5)附近。
  const Eigen::MatrixXd boundary_ctrl_pts = ctrl_pts;
  const double optimizer_velocity = crossing_max_velocity > 0.0
      ? bspline_optimizers_[0]->setVelocityLimit(plan_max_velocity) : 0.0;
  const double optimizer_acceleration = crossing_max_acceleration > 0.0
      ? bspline_optimizers_[0]->setAccelerationLimit(plan_max_acceleration) : 0.0;
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  if (crossing_max_velocity > 0.0) bspline_optimizers_[0]->setVelocityLimit(optimizer_velocity);
  if (crossing_max_acceleration > 0.0) bspline_optimizers_[0]->setAccelerationLimit(optimizer_acceleration);
  const int boundary_count = std::min(pp_.bspline_degree_, static_cast<int>(ctrl_pts.rows() / 2));
  for (int i = 0; i < boundary_count; ++i) {
    ctrl_pts.row(i) = boundary_ctrl_pts.row(i);
    ctrl_pts.row(ctrl_pts.rows() - 1 - i) =
        boundary_ctrl_pts.row(boundary_ctrl_pts.rows() - 1 - i);
  }
  // 保留引导曲线的高度与初始竖直速度/加速度，优化器仅调整水平绕行；
  // 防止自由Z优化将已锁定的低位路线重新拱到摆球区域。
  if (preserve_height_profile) ctrl_pts.col(2) = boundary_ctrl_pts.col(2);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  // 2026-07-20: 发布链路增加终点一致性兜底；若数值异常仍破坏目标，恢复未经优化但严格经过
  // waypoint的初始B样条，后续isTrajectorySafe仍会执行完整碰撞复核。
  const Vector3d optimized_end =
      local_data_.position_traj_.evaluateDeBoorT(local_data_.position_traj_.getTimeSum());
  if (!optimized_end.allFinite() || (optimized_end - tour.back()).norm() > 0.08) {
    ROS_ERROR("[trajectory_boundary] optimized endpoint drifted %.2fm; restore guided initial spline.",
              optimized_end.allFinite() ? (optimized_end - tour.back()).norm() : -1.0);
    local_data_.position_traj_.setUniformBspline(
        boundary_ctrl_pts, pp_.bspline_degree_, duration / double(seg_num));
  }

  // 路线截断距离不能被样条在小范围内反复绕行凑出：检查实际曲线的
  // 弧长、反向位移和终点，优化结果不合格时回到原始样条再验一次。
  auto trajectoryShapeAllowed = [&]() {
    auto& trajectory = local_data_.position_traj_;
    const double flight_time = trajectory.getTimeSum();
    const Eigen::Vector3d displacement = tour.back() - tour.front();
    const double chord = displacement.norm();
    if (!std::isfinite(flight_time) || flight_time <= 0.0 || chord < 0.02)
      return false;
    const Eigen::Vector3d forward = displacement / chord;
    const int samples = std::max(20, static_cast<int>(std::ceil(flight_time / 0.05)));
    Eigen::Vector3d previous = trajectory.evaluateDeBoorT(0.0);
    double arc = 0.0;
    double reverse = 0.0;
    for (int sample = 1; sample <= samples; ++sample) {
      const Eigen::Vector3d point = trajectory.evaluateDeBoorT(
          flight_time * static_cast<double>(sample) / samples);
      if (!point.allFinite() ||
          (!fapp_planner_priority_ && !isWatchedFappPositionSafe(point))) return false;
      const Eigen::Vector3d step = point - previous;
      arc += step.norm();
      reverse += std::max(0.0, -step.dot(forward));
      previous = point;
    }
    const double endpoint_error = (previous - tour.back()).norm();
    if (arc > std::max(1.50 * chord, chord + 0.30) ||
        reverse > 0.10 || endpoint_error > 0.08) {
      ROS_WARN("[trajectory_shape] reject arc=%.2f chord=%.2f reverse=%.2f "
               "endpoint_error=%.2f.", arc, chord, reverse, endpoint_error);
      return false;
    }
    return true;
  };
  if (!trajectoryShapeAllowed()) {
    local_data_.position_traj_.setUniformBspline(
        boundary_ctrl_pts, pp_.bspline_degree_, duration / double(seg_num));
    if (!trajectoryShapeAllowed()) return false;
  }

  // 优化器中的 FEASI 是软代价，不能保证发布轨迹满足物理约束。发布前对最终样条执行
  // 控制点导数硬检查；通过拉长内部 knot 降低速度和加速度，同时保留起终端边界状态。
  // 若当前边界状态已经略高于配置上限，则只把该不可瞬时改变的边界值作为本次上限，
  // 禁止轨迹内部在此基础上继续放大。
  auto& final_traj = local_data_.position_traj_;
  const double velocity_limit = std::max(
      plan_max_velocity, cur_vel.cwiseAbs().maxCoeff() + 1e-3);
  const double acceleration_limit = std::max(
      plan_max_acceleration, cur_acc.cwiseAbs().maxCoeff() + 1e-3);
  if (!(velocity_limit > 0.0) || !(acceleration_limit > 0.0) ||
      !std::isfinite(velocity_limit) || !std::isfinite(acceleration_limit)) {
    ROS_ERROR("[trajectory_dynamics] invalid limits velocity=%.3f acceleration=%.3f.",
              velocity_limit, acceleration_limit);
    return false;
  }
  final_traj.setPhysicalLimits(velocity_limit, acceleration_limit);
  const double duration_before_reallocation = final_traj.getTimeSum();
  int reallocation_iterations = 0;
  while (!final_traj.checkFeasibility(false) && reallocation_iterations < 30) {
    final_traj.reallocateTime(false);
    ++reallocation_iterations;
  }
  if (!final_traj.checkFeasibility(false)) {
    ROS_ERROR("[trajectory_dynamics] reject infeasible final spline after %d time "
              "reallocation iteration(s), limits=(%.3fm/s %.3fm/s^2).",
              reallocation_iterations, velocity_limit, acceleration_limit);
    return false;
  }
  const double duration_after_reallocation = final_traj.getTimeSum();
  if (duration_after_reallocation > duration_before_reallocation + 1e-3) {
    ROS_WARN("[trajectory_dynamics] lengthened final spline %.3fs -> %.3fs in %d "
             "local adjustment iteration(s), limits=(%.3fm/s %.3fm/s^2).",
             duration_before_reallocation, duration_after_reallocation,
             reallocation_iterations, velocity_limit, acceleration_limit);
  }

  updateTrajInfo();
  return true;
}

bool FastPlannerManager::planStationaryTraj(
    const Eigen::Vector3d& position, double duration) {
  if (!position.allFinite() || !std::isfinite(duration)) return false;
  duration = std::max(0.30, duration);
  const int degree = std::max(1, pp_.bspline_degree_);
  const int control_point_count = degree + 5;
  Eigen::MatrixXd control_points(control_point_count, 3);
  for (int row = 0; row < control_point_count; ++row)
    control_points.row(row) = position.transpose();
  const double knot_span =
      duration / static_cast<double>(control_point_count - degree);
  local_data_.position_traj_.setUniformBspline(
      control_points, degree, knot_span);
  updateTrajInfo();
  ROS_ERROR("[turn_in_place] stationary position trajectory %.2fs at "
            "(%.2f %.2f %.2f).",
            local_data_.duration_, position.x(), position.y(), position.z());
  return true;
}

bool FastPlannerManager::planVerticalTraj(
    const Eigen::Vector3d& start, const Eigen::Vector3d& target) {
  if ((target - start).head<2>().norm() > 1e-6 ||
      !planStraightTraj(start, target)) return false;
  ROS_ERROR("[ground_ascent] fixed-xy vertical spline z=%.2f -> %.2f over %.2fs.",
            start.z(), target.z(), local_data_.duration_);
  return true;
}

bool FastPlannerManager::planStraightTraj(
    const Eigen::Vector3d& start, const Eigen::Vector3d& target) {
  if (!start.allFinite() || !target.allFinite() ||
      (target - start).norm() < 0.02) return false;
  const double distance = (target - start).norm();
  if (!(pp_.max_vel_ > 0.0) || !(pp_.max_acc_ > 0.0)) return false;

  // 五次 smoothstep 的峰值速度和峰值加速度分别约为 1.875*d/T、
  // 5.774*d/T^2；据此直接给出满足动力学上限的时长。
  const double velocity_duration = 1.875 * distance / pp_.max_vel_;
  const double acceleration_duration =
      std::sqrt(5.774 * distance / pp_.max_acc_);
  // 样条控制点导数界比 smoothstep 峰值更保守，留出插值余量。
  const double duration =
      std::max(1.0, 1.25 * std::max(velocity_duration, acceleration_duration));
  constexpr int kSegments = 24;
  const double dt = duration / static_cast<double>(kSegments);
  vector<Eigen::Vector3d> samples;
  samples.reserve(kSegments + 1);
  for (int i = 0; i <= kSegments; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(kSegments);
    const double smooth = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    samples.push_back(start + smooth * (target - start));
  }
  const vector<Eigen::Vector3d> boundary_derivatives(4, Eigen::Vector3d::Zero());
  Eigen::MatrixXd control_points;
  NonUniformBspline::parameterizeToBspline(
      dt, samples, boundary_derivatives, 3, control_points);
  if (control_points.rows() < 4 || !control_points.allFinite()) return false;
  local_data_.position_traj_.setUniformBspline(control_points, 3, dt);
  const double reference = ros::Time::now().toSec();
  for (int i=0;i<=kSegments;++i) {
    const double t=duration*i/kSegments;
    if (!fappPositionSafe(local_data_.position_traj_.evaluateDeBoorT(t), reference+t, reference)) {
      // Vertical recovery must retain its fixed-XY contract.
      if ((target-start).head<2>().norm()<1e-6) {
        if (fapp_planner_priority_) break;
        return false;
      }
      return planExploreTraj({start, 0.5*(start+target), target},
                             Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
  }
  local_data_.position_traj_.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  if (!local_data_.position_traj_.checkFeasibility(false)) {
    ROS_ERROR("[straight_trajectory] spline exceeds velocity/acceleration limit for %.2fm.",
              distance);
    return false;
  }
  updateTrajInfo();
  const Eigen::Vector3d actual_end =
      local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);
  if ((actual_end - target).norm() > 1e-4) {
    ROS_ERROR("[straight_trajectory] endpoint error %.6fm.", (actual_end - target).norm());
    return false;
  }
  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // Generate global reference trajectory
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // Insert intermediate points if two waypoints are too far
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();
    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;
      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }
  inter_points.push_back(points.back());

  // At least 3 waypoints are required to solve the problem
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  time(0) += pp_.max_vel_ / (2 * pp_.max_acc_);
  time(time.rows() - 1) += pp_.max_vel_ / (2 * pp_.max_acc_);

  PolynomialTraj gl_traj;
  PolynomialTraj::waypointsTraj(pos, zero, zero, zero, zero, time, gl_traj);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double dt, duration;
  Eigen::MatrixXd ctrl_pts = paramLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, pp_.bspline_degree_, dt);

  std::cout << "ctrl pt: " << ctrl_pts.rows() << std::endl;

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_ = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /* truncate a new local segment for replanning */
  ros::Time time_now = ros::Time::now();
  double t_now = (time_now - global_data_.global_start_time_).toSec();
  double local_traj_dt, local_traj_duration;

  Eigen::MatrixXd ctrl_pts = paramLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, pp_.bspline_degree_, local_traj_dt);
  local_data_.start_time_ = time_now;

  std::cout << "dt: " << local_traj_dt << ", dur: " << local_traj_duration << std::endl;

  if (!collide) {
    // No collision detected, but we can further refine the trajectory
    refineTraj(init_traj);
    double time_change = init_traj.getTimeSum() - local_traj_duration;
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(
        local_data_.position_traj_, t_now, local_traj_duration + time_change + t_now, time_change);
    // local_data_.position_traj_ = init_traj;
    // global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
  } else {
    // Find topologically distinctive path and guide optimization in parallel
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
    } else {
      // Call topological replanning when local segment is in collision
      /* Search topological distinctive paths */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr> graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
          raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* Optimize trajectory using different topo guiding paths */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
            local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;

      NonUniformBspline best_traj;
      selectBestTraj(best_traj);
      refineTraj(best_traj);
      double time_change = best_traj.getTimeSum() - local_traj_duration;

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
          local_traj_duration + time_change + t_now, time_change);
    }
  }
  updateTrajInfo();

  double tr = (ros::Time::now() - time_now).toSec();
  ROS_WARN("Replan time: %lf", tr);

  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
      [](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj) {
  ros::Time t1 = ros::Time::now();
  plan_data_.no_visib_traj_ = best_traj;

  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;

  // ViewConstraint view_cons;
  // visib_util_->calcViewConstraint(best_traj, view_cons);
  // plan_data_.view_cons_ = view_cons;
  // if (view_cons.idx_ >= 0)
  // {
  //   cost_function |= BsplineOptimizer::VIEWCONS;
  //   bspline_optimizers_[0]->setViewConstraint(view_cons);
  // }

  // Refine selected best traj
  Eigen::MatrixXd ctrl_pts = best_traj.getControlPoint();
  double dt = best_traj.getKnotSpan();
  vector<Eigen::Vector3d> start1, end1;
  best_traj.getBoundaryStates(2, 0, start1, end1);

  bspline_optimizers_[0]->setBoundaryStates(start1, end1);
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_function, 2, 2);
  best_traj.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  vector<Eigen::Vector3d> start2, end2;
  best_traj.getBoundaryStates(2, 2, start2, end2);
  for (int i = 0; i < 3; ++i)
    std::cout << "error start: " << (start1[i] - start2[i]).norm() << std::endl;
  for (int i = 0; i < 1; ++i)
    std::cout << "error end  : " << (end1[i] - end2[i]).norm() << std::endl;
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
    Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int prev_num = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();

  int seg_num = bspline.getControlPoint().rows() - pp_.bspline_degree_;
  ratio = min(1.01, ratio);

  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt)
    point_set.push_back(bspline.evaluateDeBoorT(time));
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, plan_data_.local_start_end_derivative_, pp_.bspline_degree_, ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(
    double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id) {
  auto t1 = ros::Time::now();

  // Re-parameterize B-spline according to the length of guide path
  int seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  seg_num = max(6, seg_num);  // Min number required for optimizing
  double dt = duration / double(seg_num);
  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(start_t, duration, dt);

  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);
  vector<Eigen::Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);

  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // Discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> tmp_pts, guide_pts;
  if (pp_.bspline_degree_ == 3 || pp_.bspline_degree_ == 5) {
    topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2, tmp_pts);
    guide_pts.insert(guide_pts.end(), tmp_pts.begin() + 2, tmp_pts.end() - 2);
    if (guide_pts.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("Incorrect guide for 3 degree");
  } else if (pp_.bspline_degree_ == 4) {
    topo_prm_->pathToGuidePts(guide_path, int(2 * ctrl_pts.rows()) - 7, tmp_pts);
    for (int i = 0; i < tmp_pts.size(); ++i) {
      if (i % 2 == 1 && i >= 5 && i <= tmp_pts.size() - 6) guide_pts.push_back(tmp_pts[i]);
    }
    if (guide_pts.size() != int(ctrl_pts.rows()) - 8) ROS_WARN("Incorrect guide for 4 degree");
  }

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;

  double tm1 = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // First phase, path-guided optimization
  bspline_optimizers_[traj_id]->setBoundaryStates(start, end);
  bspline_optimizers_[traj_id]->setGuidePath(guide_pts);
  bspline_optimizers_[traj_id]->optimize(ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);
  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  double tm2 = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Second phase, smooth+safety+feasibility
  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  // if (pp_.min_time_)
  //   cost_func |= BsplineOptimizer::MINTIME;
  bspline_optimizers_[traj_id]->setBoundaryStates(start, end);
  bspline_optimizers_[traj_id]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  double tm3 = (ros::Time::now() - t1).toSec();
  // ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::paramLocalTraj(double start_t, double& dt, double& duration) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;
  global_data_.getTrajInfoInSphere(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
      start_end_derivative, dt, duration);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(
    const double& start_t, const double& duration, const double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajInfoInDuration(start_t, duration, dt, point_set, start_end_derivative);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
    vector<Eigen::Vector3d>& colli_end, vector<Eigen::Vector3d>& start_pts,
    vector<Eigen::Vector3d>& end_pts) {
  bool last_safe = true, safe;
  double t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {
    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getKnotSpan();
  int sn = ceil((t_s - t_m) / dt);
  dt = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getKnotSpan();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto& pos = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                  BsplineOptimizer::START | BsplineOptimizer::END;

  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw[0], 0, 0),
    Eigen::Vector3d(start_yaw[1], 0, 0), Eigen::Vector3d(start_yaw[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw[0], 0, 0),
    Eigen::Vector3d(end_yaw[1], 0, 0), Eigen::Vector3d(end_yaw[2], 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, pp_.bspline_degree_, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_ = path_yaw;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "yaw time: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw,
    bool lookfwd, const double& relax_time, bool continuous_scan, double scan_yaw_rate,
    const std::function<double(const Eigen::Vector3d&)>& channel_yaw) {
  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / seg_num;  // time of B-spline segment
  Eigen::Vector3d start_yaw3d = start_yaw;
  std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d.transpose()
            << ", end: " << end_yaw << std::endl;

  while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // Initial state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Add waypoint constraints if look forward is enabled
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  if (continuous_scan) {
    // 2026-07-24: 全向相机扫描必须在整段轨迹上连续展开yaw，不能只给一个归一化终点，
    // 否则360度与0度等价，优化器会选择“不转”或在重规划处反向走最短角。
    for (int i = 1; i < seg_num; ++i) {
      const double tc = i * dt_yaw;
      Eigen::Vector3d waypt(start_yaw3d[0] + scan_yaw_rate * tc, scan_yaw_rate, 0.0);
      waypts.push_back(waypt);
      waypt_idx.push_back(i);
    }
    last_yaw = start_yaw3d[0] + scan_yaw_rate * (seg_num - 1) * dt_yaw;
  } else if (channel_yaw) {
    // 墙体给出沿通道进度的朝向；不使用位置路径的绕障切线。
    for (int i=1; i<seg_num; ++i) {
      double heading=channel_yaw(local_data_.position_traj_.evaluateDeBoorT(i*dt_yaw));
      calcNextYaw(last_yaw,heading);
      waypts.emplace_back(heading,0.0,0.0);
      waypt_idx.push_back(i);
      last_yaw=heading;
    }
  } else if (lookfwd) {
    const double forward_t = 2.0;
    const int relax_num = relax_time / dt_yaw;
    for (int i = 1; i < seg_num - relax_num; ++i) {
      double tc = i * dt_yaw;
      Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
      double tf = min(local_data_.duration_, tc + forward_t);
      Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
      Eigen::Vector3d pd = pf - pc;
      Eigen::Vector3d waypt;
      if (pd.norm() > 1e-6) {
        waypt(0) = atan2(pd(1), pd(0));
        waypt(1) = waypt(2) = 0.0;
        calcNextYaw(last_yaw, waypt(0));
      } else
        waypt = waypts.back();

      last_yaw = waypt(0);
      waypts.push_back(waypt);
      waypt_idx.push_back(i);
    }
  }
  // Final state
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  if (channel_yaw && !continuous_scan)
    end_yaw3d[0]=channel_yaw(local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_));
  if (continuous_scan) {
    // 2026-07-24: 端点保留扫描角速度，使相邻重规划段衔接时不会每段减速到零。
    end_yaw3d =
        Eigen::Vector3d(start_yaw3d[0] + scan_yaw_rate * local_data_.duration_,
                        scan_yaw_rate, 0.0);
  } else {
    calcNextYaw(last_yaw, end_yaw3d(0));
  }
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // 2026-07-14: yaw 是环形量，不能用展开后的端点直接相减判断“突变”；跨越 +/-PI
  // 或沿路径累计转向会被旧逻辑误报。这里只记录规范化最短角差，真实限速由 yaw B-spline 时长保证。
  const double shortest_yaw_delta =
      std::atan2(std::sin(end_yaw3d[0] - start_yaw3d[0]),
                 std::cos(end_yaw3d[0] - start_yaw3d[0]));
  ROS_DEBUG_THROTTLE(
      1.0,
      "[yaw_plan] start=%.1fdeg target=%.1fdeg shortest_delta=%.1fdeg duration=%.2fs "
      "continuous_scan=%d rate=%+.1fdeg/s.",
      start_yaw3d[0] * 180.0 / M_PI, end_yaw3d[0] * 180.0 / M_PI,
      shortest_yaw_delta * 180.0 / M_PI, dt_yaw * seg_num, continuous_scan,
      scan_yaw_rate * 180.0 / M_PI);

  // // Interpolate start and end value for smoothness
  // for (int i = 1; i < seg_num; ++i)
  // {
  //   double tc = i * dt_yaw;
  //   Eigen::Vector3d waypt = (1 - double(i) / seg_num) * start_yaw3d + double(i) / seg_num *
  //   end_yaw3d;
  //   std::cout << "i: " << i << ", wp: " << waypt[0] << ", ";
  //   calcNextYaw(last_yaw, waypt(0));
  // }
  // std::cout << "" << std::endl;

  auto t1 = ros::Time::now();

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::WAYPOINTS;
  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw3d[0], 0, 0),
    Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0) };
  // 2026-07-24: 连续扫描的末端导数使用设定角速度；普通航向规划仍以零角速度到达。
  vector<Eigen::Vector3d> end = {
    Eigen::Vector3d(end_yaw3d[0], 0, 0),
    Eigen::Vector3d(continuous_scan ? scan_yaw_rate : 0.0, 0, 0)
  };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // std::cout << "2: " << (ros::Time::now() - t1).toSec() << std::endl;

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;

  // plan_data_.path_yaw_ = path;
  // plan_data_.dt_yaw_path_ = dt_yaw * subsp;
}

bool FastPlannerManager::planYawTurnInPlace(
    const Eigen::Vector3d& start_yaw, double end_yaw) {
  if (!start_yaw.allFinite() || !std::isfinite(end_yaw) ||
      local_data_.duration_ <= 0.0)
    return false;

  double start = start_yaw[0];
  while (start < -M_PI) start += 2.0 * M_PI;
  while (start > M_PI) start -= 2.0 * M_PI;
  calcNextYaw(start, end_yaw);

  // 两端各用15%时长平滑加减速，中间保持匀速。相比整段五次 smoothstep，
  // 峰值/平均角速度比从1.875降到1/(1-0.15)=1.176，既保证零端点导数，
  // 又避免为了完整到达目标而把转弯拖得过久。
  constexpr int kSegments = 24;
  const double dt = local_data_.duration_ / static_cast<double>(kSegments);
  const double delta = end_yaw - start;
  vector<Eigen::Vector3d> samples;
  samples.reserve(kSegments + 1);
  for (int i = 0; i <= kSegments; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(kSegments);
    constexpr double kRampFraction = 0.15;
    constexpr double kArea = 1.0 - kRampFraction;
    double progress = 0.0;
    if (u < kRampFraction) {
      const double x = u / kRampFraction;
      const double ramp_integral =
          2.5 * std::pow(x, 4) - 3.0 * std::pow(x, 5) + std::pow(x, 6);
      progress = kRampFraction * ramp_integral / kArea;
    } else if (u <= 1.0 - kRampFraction) {
      progress = (0.5 * kRampFraction + u - kRampFraction) / kArea;
    } else {
      const double remaining = 1.0 - u;
      const double x = remaining / kRampFraction;
      const double ramp_integral =
          2.5 * std::pow(x, 4) - 3.0 * std::pow(x, 5) + std::pow(x, 6);
      progress = 1.0 - kRampFraction * ramp_integral / kArea;
    }
    samples.emplace_back(start + delta * progress, 0.0, 0.0);
  }
  const vector<Eigen::Vector3d> boundary_derivatives(4, Eigen::Vector3d::Zero());
  Eigen::MatrixXd interpolated_control_points;
  NonUniformBspline::parameterizeToBspline(
      dt, samples, boundary_derivatives, 3, interpolated_control_points);
  if (interpolated_control_points.rows() < 4 ||
      !interpolated_control_points.allFinite())
    return false;

  Eigen::MatrixXd yaw_control_points(interpolated_control_points.rows(), 1);
  yaw_control_points.col(0) = interpolated_control_points.col(0);
  local_data_.yaw_traj_.setUniformBspline(yaw_control_points, 3, dt);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt;

  const double duration = local_data_.yaw_traj_.getTimeSum();
  const double actual_end = local_data_.yaw_traj_.evaluateDeBoorT(duration)[0];
  const double endpoint_error = std::fabs(actual_end - end_yaw);
  constexpr double kEndpointTolerance = M_PI / 180.0;
  if (!std::isfinite(actual_end) || endpoint_error > kEndpointTolerance) {
    ROS_ERROR("[turn_in_place] hard yaw endpoint verification failed: "
              "target=%.6f actual=%.6f error=%.6f.",
              end_yaw, actual_end, endpoint_error);
    return false;
  }
  ROS_ERROR("[turn_in_place] hard yaw spline %.1fdeg -> %.1fdeg over %.2fs "
            "(endpoint error %.6fdeg).",
            start * 180.0 / M_PI, end_yaw * 180.0 / M_PI, duration,
            endpoint_error * 180.0 / M_PI);
  return true;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
