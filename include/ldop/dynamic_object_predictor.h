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
  MultiModelKalmanFilterParams filter;
};

// 运行时配置；构造阶段由 ROS 参数快照转换而来。
struct DynamicObjectPredictorConfig {
  double prediction_horizon{1.0};
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

  // 构建预测可视化 marker：轨迹线和不确定性椭球。
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
