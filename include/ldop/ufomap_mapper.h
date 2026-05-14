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

// UFOMap 模块内部使用的运行时配置：构造阶段由 ROS 参数快照转换而来。
struct UfomapMapperConfig {
  double resolution{0.2};
  int depth_levels{16};
  double min_range{0.5};
  double max_range{10.0};
  int insert_hit_depth{0};
  int insert_miss_depth{0};
  int ray_casting_depth{0};
  int num_threads{0};
  bool only_valid{false};
  int inflate_unknown{1};
  bool inflate_unknown_compensation{true};
  bool ray_passthrough_hits{false};
  double inflate_hits_dist{0.4};
  std::string down_sampling_method{"center"};
  bool simple_ray_casting{false};
  double simple_ray_casting_factor{1.0};
  int sliding_window_size{0};
  bool parallel{true};
  bool propagate{true};
  double static_map_visualization_max_z{3.0};
};

struct UfomapMapperParams {
  double resolution{0.2};             // 大于0.0，叶子体素尺寸
  int depth_levels{16};               // [2, 20]，由UFOMAP自身限制范围，八叉树层级规模    
  double min_range{0.5};              // 大于等于0.0，积分和查询的最小范围 
  double max_range{10.0};             // 非负时大于min_range，-1时不限制范围，积分和查询的最大范围   
  int insert_hit_depth{0};            // [0, depth_levels]，命中点积分深度，0 表示叶子节点     
  int insert_miss_depth{0};           // [0, depth_levels]，空闲点积分深度，0 表示叶子节点  
  int ray_casting_depth{0};           // [0, depth_levels]，射线穿透查询深度，0 表示叶子节点
  int num_threads{0};                 // 大于等于0，积分线程数，0 表示交给 UFOMap 使用默认线程策略
  bool only_valid{false};             // 是否仅积分有效点（即满足 min_range <= range <= max_range 的点）
  int inflate_unknown{1};             // 大于等于0，按邻域膨胀d_p未知区域以容忍定位误差，单位为一个体素
  bool inflate_unknown_compensation{true};          // 是否补偿未知区域膨胀导致的过度占用风险
  bool ray_passthrough_hits{false};                 // 射线是否穿过命中点；如果为 true，射线将继续穿过命中点并可能在更远处产生额外的空闲更新
  double inflate_hits_dist{0.4};                    // 大于等于0，建议为体素的倍数，沿射线扩展d_s命中区域以容忍测距噪声，天花板/墙顶这种大平面在仿真雷达里容易被斜向射线、体素离散、测距噪声打出“边缘擦过”的情况。
  std::string down_sampling_method{"center"};     // 下采样方法，支持 "none"（不下采样）、"center"（保留体素中心点）、"centroid"（保留体素质心点）和 "uniform"（在体素内随机均匀采样一个点）
  bool simple_ray_casting{false};                   // 是否使用简单射线穿透方法；false为启用简单投射，射线穿透查询将直接将目标深度的体素视为命中，不考虑实际占用概率，会加速积分但降低分类准确率
  double simple_ray_casting_factor{1.0};            // 大于0，投射节点的间隔的系数，以层级为投射基础？以体素尺寸为投射基础？
  int sliding_window_size{0};                       // 大于等于0，滑动窗口大小，0 表示不使用滑动窗口；启用，UFOMap 将仅保留最近 N 帧的积分结果？还是保留最近 N 帧再/一起积分
  bool parallel{true};                              // 是否启用并行处理；如果为 true，UFOMap 将在积分过程中使用多线程，这可能会加速处理但也可能引入线程安全问题或增加资源竞争
  bool propagate{true};                             // 积分后是否马上更新父节点状态（树的状态）；如果为 false，父节点状态将仅在查询时根据子节点动态计算，这可能会加速积分但增加查询时的计算负担
  double static_map_visualization_max_z{3.0};       // 大于0，静态地图的可视化高度
};

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
               bool verbose);
  ~UfomapMapper();

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

  bool verbose_;
  UfomapMapperParams params_;
  UfomapMapperConfig config_;

  std::atomic<std::uint64_t> ufomap_update_count_;
  // 地图读多写少，使用读写锁避免多个查询路径彼此串行化。
  mutable std::shared_mutex map_mutex_;

  std::unique_ptr<UfomapRuntime> runtime_;
};

}  // namespace ldopcore
