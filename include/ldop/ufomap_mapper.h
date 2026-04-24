#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <ufo/geometry/point.hpp>
#include <ufo/geometry/sphere.hpp>
#include <ufo/map/types.hpp>

namespace ufo {
// 这里只需要在私有类型别名里声明 CloudElement；完整定义放在 .cpp 中引入。
// 直接 include ufo/map/point_cloud.hpp 会触发 UFOMap 安装头的包含顺序问题。
template <class... Types>
struct CloudElement;
}  // namespace ufo

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
  // UFOMap 会在该值大于 1 时合并最近 N 帧 hits/misses 生成 seenFree。
  int sliding_window_size{0};
  bool parallel{true};
  bool propagate{true};
};

UfomapMapperParams sanitizeUfomapParams(const UfomapMapperParams& params);

struct UfomapRuntimeStats {
  // 已完成 UFOMap 积分的帧数，以及当前地图节点规模。
  std::uint64_t update_count{0};
  std::size_t node_count{0};
};

struct UfomapTimingStats {
  // 单帧 processInputCloud() 整体耗时，单位为毫秒。
  double process_input_cloud_ms{0.0};
  // 点云提取、地图更新与当前帧分类的分阶段耗时，单位为毫秒。
  double extract_finite_points_ms{0.0};
  double update_ufomap_ms{0.0};
  double classify_current_frame_ms{0.0};
};

struct UfomapVoxelCode {
  // raw_code 保留 UFOMap Code::raw()，可用于调试或后续直接复用节点编码。
  ufo::code_t raw_code{0};
  // key_* / depth 用于后续按 UFOMap key 空间生成 6/18/26 邻域。
  ufo::key_t key_x{0};
  ufo::key_t key_y{0};
  ufo::key_t key_z{0};
  ufo::depth_t depth{0};
};

struct UfomapDynamicClusterPoint {
  // 原始动态点用于聚类后计算 AABB，避免体素中心放大目标框。
  ufo::Point point{};
  UfomapVoxelCode voxel_code;
};

struct UfomapClassificationResult {
  // static_cloud_msg / dynamic_cloud_msg 的语义由调用方定义，
  // 当前在 LDOP 主流程中分别用于“当前帧静态点云 / 当前帧动态点云”。
  sensor_msgs::PointCloud2 static_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  // 当前帧动态点的结构化聚类输入；不改变 dynamic_cloud_msg 的 ROS 输出语义。
  std::vector<UfomapDynamicClusterPoint> dynamic_cluster_points;
  std::size_t static_point_count{0};
  std::size_t dynamic_point_count{0};
  std::size_t input_point_count{0};
};

struct UfomapFrameResult {
  // 当前阶段 static_map_cloud_msg / dynamic_cloud_msg 分别用于发布当前帧静态/动态点云。
  sensor_msgs::PointCloud2 static_map_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  UfomapClassificationResult classification;
  // 按深度分组的静态体素 CUBE_LIST，可直接给 RViz MarkerArray 显示。
  visualization_msgs::MarkerArray static_map_markers;
  UfomapRuntimeStats runtime_stats;
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

  // 暴露 UFOMap 中 seenFree / label 等基础查询能力，供上层构建更高层逻辑。
  std::vector<UfomapCandidateNode> queryDynamicCandidates(
      const UfomapCandidateQuery& query) const;
  UfomapFrameResult processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                      const nav_msgs::Odometry& odom_msg);

 private:
  struct UfomapRuntime;
  using UfomapPointCloud = std::vector<ufo::CloudElement<ufo::Point>>;

  void loadParameters();
  void configureUfomap();
  // 调用方需先持有地图读锁或写锁；当前实现中 runtime_ 在对象存活期内应始终有效。
  UfomapRuntime& runtimeLocked();
  const UfomapRuntime& runtimeLocked() const;
  UfomapClassificationResult classifyPoints(const std_msgs::Header& header,
                                            const UfomapPointCloud& points) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string world_frame_;
  bool verbose_;
  UfomapMapperParams params_;

  std::atomic<std::uint64_t> ufomap_update_count_;
  // 地图读多写少，使用读写锁避免多个查询路径彼此串行化。
  mutable std::shared_mutex map_mutex_;

  std::unique_ptr<UfomapRuntime> runtime_;
};

}  // namespace ldopcore
