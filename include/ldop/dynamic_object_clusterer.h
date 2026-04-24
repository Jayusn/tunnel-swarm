#pragma once

#include <cstddef>
#include <vector>

#include <ldop/DynamicObjectArray.h>
#include <ldop/ufomap_mapper.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <ufo/map/types.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

struct DynamicObjectClustererParams {
  std::size_t min_points{3U};
  double max_extent{3.0};
  int connectivity{6};
};

struct DynamicObjectClustererTimingStats {
  // 单帧动态聚类主流程整体耗时，单位为毫秒。
  double process_dynamic_objects_ms{0.0};
  // 聚类与 marker 构建的分阶段耗时，单位为毫秒。
  double build_dynamic_object_array_ms{0.0};
  double build_object_markers_ms{0.0};
};

struct DynamicObjectClustererFrameResult {
  ldop::DynamicObjectArray dynamic_objects_msg;
  visualization_msgs::MarkerArray dynamic_object_markers_msg;
  DynamicObjectClustererTimingStats timing;
};

class DynamicObjectClusterer {
 public:
  // 构造时直接从 ROS 参数服务器读取聚类参数，并在内部完成合法化。
  // verbose 只控制模块自己的 timing 日志，不影响聚类结果。
  DynamicObjectClusterer(ros::NodeHandle& pnh, bool verbose = false);

  const DynamicObjectClustererParams& params() const;

  // 聚类模块自己的主流程：统一生成当前帧动态目标与对应可视化，并在模块内记录日志。
  DynamicObjectClustererFrameResult processDynamicObjects(
      const std_msgs::Header& header,
      const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const;

  ldop::DynamicObjectArray buildDynamicObjectArray(
      const std_msgs::Header& header,
      const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const;

  // 为当前帧动态目标生成 RViz 线框包围盒；每帧先发 DELETEALL 清理旧框。
  visualization_msgs::MarkerArray buildObjectMarkers(const ldop::DynamicObjectArray& object_array) const;

 private:
  DynamicObjectClustererParams params_;
  bool verbose_{false};
};

}  // namespace ldopcore
