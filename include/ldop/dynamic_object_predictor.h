#pragma once

#include <vector>

#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/dynamic_object_tracker.h>
#include <ldop/multi_model_kalman_filter.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

// ROS 参数快照；predictor 和 tracker 各自读取，不共享同一个实例。
struct DynamicObjectPredictorParams {
  double prediction_horizon{1.0};
  int max_branches{6};                         // >=1，限制每个目标的 GMM component 数量。
  double min_branch_probability{0.03};         // [0,1)，过滤极低概率分支后重新归一化。
  int history_window{10};                      // >=1，历史特征估计使用的最近样本数。
  double heading_stable_angle{0.35};           // >0，单位 rad，航向变化小于该值时认为趋势稳定。
  double stop_speed{0.2};                      // >=0，停止分支趋近的速度尺度。
  double turn_yaw_rate{0.5};                   // >0，转向假设的角速度幅值。
  double lateral_speed{0.4};                   // >=0，人类横向避让分支的法向速度幅值。
  double vertical_speed{0.3};                  // >=0，UAV 垂向分支的 z 速度幅值。
  double history_quality_noise_scale{1.5};     // >=1，短历史/漏检/不稳定轨迹的未来噪声放大上限。
  double hypothesis_noise_scale{1.3};          // >=1，假设性更强 mode 的未来噪声放大尺度。
  MultiModelKalmanFilterParams filter;
};

// 运行时配置；构造阶段由 ROS 参数快照转换而来。
struct DynamicObjectPredictorConfig {
  double prediction_horizon{1.0};
  std::size_t max_branches{6U};
  double min_branch_probability{0.03};
  std::size_t history_window{10U};
  double heading_stable_angle{0.35};
  double stop_speed{0.2};
  double turn_yaw_rate{0.5};
  double lateral_speed{0.4};
  double vertical_speed{0.3};
  double history_quality_noise_scale{1.5};
  double hypothesis_noise_scale{1.3};
  MultiModelKalmanFilterConfig filter_config;
};

struct DynamicObjectPredictorTimingStats {
  double predict_dynamic_objects_ms{0.0};
  double build_prediction_markers_ms{0.0};
  double predict_total_ms{0.0};
};

// 帧级预测结果：包含预测消息、预测 marker 和耗时统计。
struct DynamicObjectPredictorFrameResult {
  ldop::DynamicObjectPredictionArray predictions_msg;
  visualization_msgs::MarkerArray prediction_markers_msg;
  DynamicObjectPredictorTimingStats timing;
};

class DynamicObjectPredictor {
 public:
  // 从 ROS 参数服务器读取预测参数，并在模块内部构建运行时配置。
  explicit DynamicObjectPredictor(ros::NodeHandle& pnh, bool verbose = false);
  // 直接使用外部配置构造，用于测试。
  explicit DynamicObjectPredictor(const DynamicObjectPredictorConfig& config,
                                  bool verbose = false);

  // 预测模块主流程：对 tracker 输出的稳定目标做轨迹外推，返回预测消息和 marker。
  DynamicObjectPredictorFrameResult predict(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs) const;

 private:
  void loadParameters();

  // 构建预测可视化 marker：只显示真实 GMM 分支的均值轨迹线，避免额外抽样线误导调试。
  visualization_msgs::MarkerArray buildPredictionMarkers(
      const std_msgs::Header& header,
      const ldop::DynamicObjectPredictionArray& predictions) const;

  ros::NodeHandle pnh_;
  DynamicObjectPredictorParams params_;
  DynamicObjectPredictorConfig config_;
  bool verbose_{false};
};

// 从 ROS 参数快照构建运行时配置。
DynamicObjectPredictorConfig buildPredictorConfig(
    const DynamicObjectPredictorParams& params);

}  // namespace ldopcore
