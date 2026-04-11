#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <ufo/geometry/point.hpp>
#include <ufo/geometry/sphere.hpp>
#include <ufo/map/types.hpp>

namespace ldopcore {

struct UfomapMapperParams {
  // UFOMap 基础分辨率与深度配置。
  double resolution{0.2};
  int depth_levels{17};
  double min_range{0.0};
  double max_range{50.0};
  int insert_hit_depth{0};
  int insert_miss_depth{0};
  int ray_casting_depth{0};
  int num_threads{0};
  bool only_valid{false};
  int inflate_unknown{0};
  bool inflate_unknown_compensation{false};
  bool ray_passthrough_hits{true};
  double inflate_hits_dist{0.0};
  std::string down_sampling_method{"center"};
  bool simple_ray_casting{false};
  double simple_ray_casting_factor{1.0};
  bool parallel{true};
  bool propagate{true};
  int map_reserve_nodes{0};
  // 仅供低频 clean static map 导出使用的历史点缓存窗口。
  int ufomap_accumulation_max_points{0};
};

UfomapMapperParams sanitizeUfomapParams(const UfomapMapperParams& params);

struct UfomapPublishData {
};

struct UfomapRuntimeStats {
  // runtime_ready 表示运行时地图对象是否已构造完成。
  bool runtime_ready{false};
  std::uint64_t update_count{0};
  std::size_t node_count{0};
};

struct UfomapTimingStats {
  // 单帧 processInputCloud() 整体耗时，单位为毫秒。
  double process_input_cloud_ms{0.0};
  // 点云提取、地图更新、旁路累计缓存维护与当前帧分类的分阶段耗时，单位为毫秒。
  double extract_finite_points_ms{0.0};
  double update_ufomap_ms{0.0};
  double accumulate_cache_ms{0.0};
  double classify_current_frame_ms{0.0};
};

struct UfomapClassificationResult {
  // static_cloud_msg / dynamic_cloud_msg 的语义由调用方定义，
  // 当前在 LDOP 主流程中分别用于“当前帧静态点云 / 当前帧动态点云”。
  sensor_msgs::PointCloud2 static_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  std::size_t static_point_count{0};
  std::size_t dynamic_point_count{0};
  std::size_t input_point_count{0};
};

struct UfomapFrameResult {
  // 当前阶段 static_map_cloud_msg / dynamic_cloud_msg 分别用于发布当前帧静态/动态点云。
  sensor_msgs::PointCloud2 static_map_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  UfomapClassificationResult classification;
  UfomapRuntimeStats runtime_stats;
  UfomapPublishData publish_data;
  UfomapTimingStats timing;
};

struct UfomapNodeQueryResult {
  bool exists{false};
  bool occupied{false};
  ufo::Point center{};
  ufo::depth_t depth{0};
};

struct UfomapCandidateQuery {
  // depth=0 保持 UFOMap Leaf(0) 的默认语义。
  ufo::depth_t depth{0};
  std::uint32_t min_hits{1};
  std::uint32_t label{0};
  bool require_seen_free{true};
  std::size_t max_results{0};
  std::vector<ufo::Sphere> intersects;
};

struct UfomapCandidateNode {
  ufo::Point center{};
  ufo::depth_t depth{0};
  std::uint32_t hits{0};
  std::uint32_t label{0};
  bool seen_free{false};
};

class UfomapMapper {
 public:
  // 独立负责处理输入点云、更新 UFOMap，并输出动静态点云与静态地图云。
  UfomapMapper(ros::NodeHandle& nh,
               ros::NodeHandle& pnh,
               const std::string& world_frame,
               bool verbose);
  ~UfomapMapper();

  UfomapMapperParams effectiveParams() const;
  UfomapRuntimeStats runtimeStats() const;
  void reserveMap(std::size_t node_capacity);
  void clearMap();
  void propagateModified();
  bool seenFree(const ufo::Point& point) const;
  std::optional<UfomapNodeQueryResult> queryNode(const ufo::Point& point) const;
  void setNodeLabel(const ufo::Point& point, std::uint32_t label, bool propagate = true);
  std::uint32_t nodeLabel(const ufo::Point& point) const;
  void clearAllLabels();
  std::size_t accumulatedPointCount() const;
  void clearAccumulatedCloud();
  // 低频导出接口：按当前地图状态重新过滤累计历史点，供 clean static map 导出使用。
  UfomapClassificationResult classifyAccumulatedCloud(const std_msgs::Header& header) const;

  // 暴露 UFOMap 中 seenFree / label 等基础查询能力，供上层构建更高层逻辑。
  std::vector<UfomapCandidateNode> queryDynamicCandidates(
      const UfomapCandidateQuery& query) const;
  UfomapFrameResult processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                      const nav_msgs::Odometry& odom_msg);
  void publish(const UfomapPublishData& publish_data);

 private:
  void loadParameters();
  void configureUfomap();
  std::vector<ufo::Point> extractFinitePoints(const sensor_msgs::PointCloud2& cloud_msg,
                                              void* ufomap_cloud,
                                              bool* has_invalid_points) const;
  UfomapClassificationResult classifyPoints(const std_msgs::Header& header,
                                            const std::vector<ufo::Point>& points) const;
  void updateUfomap(void* ufomap_cloud,
                    const ufo::Point& sensor_origin,
                    const nav_msgs::Odometry& odom_msg,
                    UfomapFrameResult& frame_result);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string world_frame_;
  bool verbose_;
  UfomapMapperParams params_;

  std::atomic<std::uint64_t> ufomap_update_count_;
  mutable std::mutex runtime_mutex_;

  struct UfomapRuntime;
  std::unique_ptr<UfomapRuntime> runtime_;
  std::vector<ufo::Point> accumulated_points_;
};

}  // namespace ldopcore
