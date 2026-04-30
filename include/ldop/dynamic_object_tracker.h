#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include <ldop/DynamicObjectArray.h>
#include <ldop/multi_model_kalman_filter.h>
#include <ldop/utils.h>

namespace ldopcore {

enum class ObjectClass : std::uint8_t {
  Unknown = 0U,
  Human = 1U,
  Vehicle = 2U,
  Uav = 3U,
  Other = 4U,
};

// 跟踪模块内部使用的合法化配置：构造阶段由 ROS 参数转换而来，运行过程中不再反复补默认值。
struct DynamicObjectTrackerConfig {
  // 新轨迹先用最稳妥的 CV3D 承接未知类别，等证据足够后再切到类别模型。
  MotionModelType model_type{MotionModelType::CV3D};
  // 传给每条 TrackState 内部 Kalman filter 的滤波参数。
  MultiModelKalmanFilterConfig filter_config{};
  // 数据关联使用的卡方 gate 阈值，已经由置信度转换为 3 自由度马氏距离平方阈值。
  double association_gate_threshold{11.34};
  // 漏检轨迹重关联时的 gate 放宽倍数，用于给 coasting 目标更大的回收窗口。
  double coasting_gate_relax_factor{1.0};
  // bbox 尺寸只做指数平滑，中心位置仍以滤波器状态为准。
  double box_size_smoothing_alpha{0.3};
  // 马氏距离 gate 失败后的回退关联先看三维中心距离：只有距离判据先通过，才会继续看 IoU；
  // 若关闭了 IoU 判据（阈值为 0），则距离通过后即可回退匹配。最终未匹配 detection 的建轨抑制也复用它。
  double spawn_suppression_distance{0.8};
  // IoU 是距离判据通过后的第二层回退条件，不是与距离并列抢先判断的独立入口。
  // 0 表示关闭该判据。未匹配 detection 的建轨抑制同样复用该阈值。
  double spawn_suppression_iou_threshold{0.1};
  // 单帧预测 dt 上限，避免时间戳跳变把轨迹一次性外推过远。
  double max_dt{0.5};
  double invalid_cost{1e9};
  std::size_t max_coast_frames{2U};
  std::size_t max_history_size{30U};
  std::size_t min_hits_to_publish{2U};
  std::size_t classification_start_frame{10U};
  double classification_score_decay{0.85};
  double classification_score_increment{1.0};
  double classification_confirm_score{3.0};
  double classification_switch_margin{1.5};
  double classification_size_change_ratio{0.5};
  double classification_point_count_change_ratio{0.5};
  std::size_t classification_size_change_confirm_frames{5U};
  std::vector<double> classify_human_threshold{1.1, 0.8};
  std::vector<double> classify_vehicle_threshold{1.5, 0.7};
  std::vector<double> classify_uav_threshold{0.6, 1.5};
};

// ROS 参数读取后的原始参数快照；具体合法化规则集中在 sanitizeTrackerParams()。
struct DynamicObjectTrackerParams {
  int history_size{100};
  int max_missed_frames{5};
  int min_hits_to_publish{2};
  double association_gate_confidence{0.99};
  double coasting_gate_relax_factor{1.5};
  double box_size_smoothing_alpha{0.5};
  double spawn_suppression_distance{0.8};
  double spawn_suppression_iou_threshold{0.1};
  double default_dt{0.1};
  double max_dt{0.5};
  int adaptive_window_size{10};
  double adaptive_alpha{0.7};
  double adaptive_r_alpha{0.7};
  double adaptive_min_noise_ratio{0.2};
  double max_pos_cov{1.5};
  double max_vel_cov{9.0};
  double max_acc_cov{25.0};
  double ca_jerk_sigma{3.0};
  double ca_position_noise{0.1};
  double ca_initial_position_cov{0.1};
  double ca_initial_velocity_cov{1.0};
  double ca_initial_acceleration_cov{10.0};
  int classification_start_frame{10};
  double classification_score_decay{0.85};
  double classification_score_increment{1.0};
  double classification_confirm_score{3.0};
  double classification_switch_margin{1.5};
  double classification_size_change_ratio{0.5};
  double classification_point_count_change_ratio{0.5};
  int classification_size_change_confirm_frames{5};
  std::vector<double> classify_human_threshold{1.1, 0.8};
  std::vector<double> classify_vehicle_threshold{1.5, 0.7};
  std::vector<double> classify_uav_threshold{0.6, 1.5};
  bool enable_cov_limit{true};
  double ca_human_jerk_sigma{1.0};
  std::vector<double> ca_human_init_cov{0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 10.0};
  std::vector<double> ca_human_meas_noise{0.1, 0.1, 0.1};
  double ca_human_z_process_noise{0.01};
  double ca_uav_jerk_sigma{1.0};
  std::vector<double> ca_uav_init_cov{0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};
  std::vector<double> ca_uav_meas_noise{0.1, 0.1, 0.1};
  double cv_acc_sigma{3.0};
  std::vector<double> cv_init_cov{0.5, 0.5, 0.5, 2.0, 2.0, 2.0};
  std::vector<double> cv_meas_noise{0.3, 0.3, 0.3};
  std::vector<double> ctra_init_cov{0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0};
  std::vector<double> ctra_process_noise{0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0};
  std::vector<double> ctra_meas_noise{0.1, 0.1, 0.1};
};

struct DynamicObjectTrackerTimingStats {
  // 单帧跟踪主流程总耗时，单位为毫秒。
  double process_dynamic_tracks_ms{0.0};
  // 跟踪主流程分阶段耗时，单位为毫秒。
  double association_ms{0.0};
  double update_tracks_ms{0.0};
  double build_output_ms{0.0};
  double build_track_markers_ms{0.0};
};

struct DynamicObjectTrackerFrameResult {
  ldop::DynamicObjectArray dynamic_objects_msg;
  visualization_msgs::MarkerArray dynamic_track_markers_msg;
  DynamicObjectTrackerTimingStats timing;
};

// 单条轨迹的最小运行状态：稳定 ID、滤波器、bbox、生命周期计数和调试历史。
struct TrackState {
  TrackState(std::uint32_t track_id,
             MotionModelType track_model_type,
             const MultiModelKalmanFilterConfig& filter_config);
  ~TrackState();

  TrackState(const TrackState&) = delete;
  TrackState& operator=(const TrackState&) = delete;
  TrackState(TrackState&&) noexcept;
  TrackState& operator=(TrackState&&) noexcept;

  std::uint32_t id{0U};
  MotionModelType model_type{MotionModelType::CV3D};
  // 每条轨迹独占一个 KF/EKF 实例；基类接口对 tracker 隐藏具体运动模型。
  std::shared_ptr<KalmanFilterBase> filter;
  // bbox.center 会被滤波器位置覆盖，bbox.size 保留检测框尺寸并做平滑。
  BoundingBox3D bbox;
  std::vector<TrackHistorySample> history;
  // age 表示轨迹经历的总帧数，hits 表示成功关联次数，missed_frames 表示连续漏检帧数。
  std::size_t age{0U};
  std::size_t hits{0U};
  std::size_t missed_frames{0U};
  // 上次处理该轨迹的时间戳，用于下一帧计算预测 dt。
  ros::Time last_stamp;
  // 对外类别和内部证据分开：新轨迹先保持 Unknown，避免把 CV3D 默认模型误报为 Other。
  ObjectClass object_class{ObjectClass::Unknown};
  // 只给 Human/Vehicle/UAV/Other 四个明确类别积分；Unknown 表示证据尚未达标。
  std::array<double, 4U> classification_scores{0.0, 0.0, 0.0, 0.0};
  // 这里的 max_observed_size 是分类用的历史稳定尺寸，不是轨迹 history 的容量。
  geometry_msgs::Vector3 max_observed_size;
  std::size_t last_point_count{0U};
  std::size_t large_size_counter{0U};
  std::size_t small_size_counter{0U};
};

class DynamicObjectTracker {
 public:
  // 构造时直接从 ROS 参数服务器读取跟踪参数，并在模块内部完成合法化。
  explicit DynamicObjectTracker(ros::NodeHandle& pnh, bool verbose = false);

  // 跟踪模块自己的主流程：顺序执行关联、状态维护、输出与可视化构建。
  DynamicObjectTrackerFrameResult processDynamicTracks(
      const std_msgs::Header& header,
      const std::vector<DynamicObjectDetection>& detections);

 private:
  // 构造阶段集中读取并合法化 tracker 参数，保持构造函数主体简洁。
  void loadParameters();

  // 历史线和轨迹头 marker 只在模块内部生成，避免测试代码反向塑形公共接口。
  visualization_msgs::MarkerArray buildTrackMarkers(const std_msgs::Header& header) const;
  void predictTracks(const ros::Time& stamp);
  void updateMatchedTrack(TrackState& track,
                          const DynamicObjectDetection& detection,
                          const ros::Time& stamp);
  geometry_msgs::Vector3 prepareMatchedTrackForUpdate(TrackState& track,
                                                      const DynamicObjectDetection& detection,
                                                      std::size_t matched_hits);
  void switchTrackModel(TrackState& track, ObjectClass target_class);
  // 未匹配且未被 spawn suppression 挡住的 detection 才会创建新稳定 ID。
  void createTrack(const DynamicObjectDetection& detection, const ros::Time& stamp);
  // 漏检轨迹继续按预测状态维护，给后续帧重关联留窗口。
  void coastTrack(TrackState& track, const ros::Time& stamp);
  void deleteExpiredTracks();
  // 对外只发布命中次数达到阈值的稳定轨迹，减少一帧噪声 detection 直接暴露给下游。
  ldop::DynamicObjectArray buildOutput(const std_msgs::Header& header) const;

  ros::NodeHandle pnh_;
  DynamicObjectTrackerConfig config_;
  DynamicObjectTrackerParams params_;
  std::vector<TrackState> tracks_;
  std::uint32_t next_track_id_{0U};
  bool verbose_{false};
};

}  // namespace ldopcore
