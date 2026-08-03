#include <ldop/ufomap_mapper.h>
#include <ldop/utils.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/types.hpp>
#include <ufo/map/ufomap.hpp>

namespace ldopcore {

struct UfomapMapper::UfomapRuntime {
  using MapType =
      ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;

  explicit UfomapRuntime(const float resolution, const ufo::depth_t depth_levels)
      : map(resolution, depth_levels) {}

  ufo::IntegrationParams integration_params;
  MapType map;
};

namespace {

using DufomapMapType =
    ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;

struct ExtractFinitePointsResult {
  ufo::PointCloud points;
  bool has_invalid_points{false};
};

struct StaticMapVisualizationSnapshot {
  // 只保留 RViz CUBE_LIST 需要的最小数据，缩短 UFOMap 读锁持有时间。
  std::vector<std::vector<geometry_msgs::Point>> points_by_depth;
  std::vector<double> voxel_sizes_by_depth;
};

sensor_msgs::PointCloud2 buildCloudFromPoints(const std_msgs::Header& header,
                                              const ufo::PointCloud& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  cloud.is_dense = true;
  return cloud;
}

StaticMapVisualizationSnapshot captureStaticMapVisualizationSnapshot(
    const DufomapMapType& map,
    const double max_visualization_z) {
  const int max_depth = static_cast<int>(map.depthLevels());
  StaticMapVisualizationSnapshot snapshot;
  snapshot.points_by_depth.resize(static_cast<std::size_t>(max_depth) + 1U);
  snapshot.voxel_sizes_by_depth.resize(static_cast<std::size_t>(max_depth) + 1U);
  for (int depth = 0; depth <= max_depth; ++depth) {
    snapshot.voxel_sizes_by_depth[static_cast<std::size_t>(depth)] =
        map.size(static_cast<ufo::depth_t>(depth));
  }

  // 用 Leaf && HitsMin 先做树级剪枝，再在叶子层手动过滤 seenFree。
  // 原因：!SeenFree 的 InnerCheck 会参与内部节点剪枝；在 seenFree 传播不完全时，
  // 可能提前剪掉仍含有 !seenFree 叶子的分支，导致 RViz 看不到体素。
  for (const auto node : map.query(ufo::pred::Leaf() && ufo::pred::HitsMin(1))) {
    if (map.seenFree(node)) {
      continue;
    }

    const int depth = static_cast<int>(node.depth());
    if (depth < 0 || depth > max_depth) {
      continue;
    }

    geometry_msgs::Point center;
    const auto node_center = map.center(node);
    // 虚拟天花板等高处体素仍保留在 UFOMap 中；
    // 这里只在最终 RViz marker 层隐藏超过上限的方块。
    if (node_center.z > max_visualization_z) {
      continue;
    }
    center.x = node_center.x;
    center.y = node_center.y;
    center.z = node_center.z;
    snapshot.points_by_depth[static_cast<std::size_t>(depth)].push_back(center);
  }

  return snapshot;
}

visualization_msgs::MarkerArray buildStaticMapVisualization(
    const std_msgs::Header& header,
    StaticMapVisualizationSnapshot snapshot) {
  visualization_msgs::MarkerArray static_map_markers;
  std::vector<visualization_msgs::Marker> markers_by_depth(snapshot.points_by_depth.size());
  for (std::size_t depth = 0; depth < markers_by_depth.size(); ++depth) {
    auto& marker = markers_by_depth[depth];
    marker.header = header;
    marker.ns = "static_map_occupied";
    marker.id = static_cast<int>(depth);
    marker.action = visualization_msgs::Marker::DELETE;

    if (snapshot.points_by_depth[depth].empty()) {
      continue;
    }

    // ns/id 按 depth 固定，RViz 会用本帧内容覆盖上一帧同一 marker。
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = snapshot.voxel_sizes_by_depth[depth];
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;
    marker.color.r = 0.10f;
    marker.color.g = 0.85f;
    marker.color.b = 0.25f;
    marker.color.a = 1.0f;
    marker.points = std::move(snapshot.points_by_depth[depth]);
  }

  static_map_markers.markers.reserve(markers_by_depth.size());
  for (auto& marker : markers_by_depth) {
    static_map_markers.markers.push_back(std::move(marker));
  }

  return static_map_markers;
}

ufo::DownSamplingMethod toUfoDownSamplingMethod(const std::string& method) {
  if (method == "none") {
    return ufo::DownSamplingMethod::NONE;
  }
  if (method == "centroid") {
    return ufo::DownSamplingMethod::CENTROID;
  }
  if (method == "uniform") {
    return ufo::DownSamplingMethod::UNIFORM;
  }
  return ufo::DownSamplingMethod::CENTER;
}

ufo::depth_t clampQueryDepth(const DufomapMapType& map, const ufo::depth_t depth) {
  return std::min(depth, static_cast<ufo::depth_t>(map.depthLevels()));
}

UfomapOccupancyState classifyPlannerOccupancyState(const DufomapMapType& map,
                                                   const ufo::Node node) {
  // 当前 DUFOMap 运行时没有使用概率占据层；静态占据来自 reflection hits，
  // planner-facing free 只能由 SEEN_FREE 的射线穿越证据表达，且必须让 hit 优先生效。
  if (map.hits(node) > 0) {
    return UfomapOccupancyState::Occupied;
  }
  if (map.seenFree(node)) {
    return UfomapOccupancyState::Free;
  }
  return UfomapOccupancyState::Unknown;
}

void applyOccupancyStateFlags(const UfomapOccupancyState state,
                              UfomapNodeQueryResult& result) {
  result.state = state;
  result.occupied = state == UfomapOccupancyState::Occupied;
  result.free = state == UfomapOccupancyState::Free;
  result.unknown = state == UfomapOccupancyState::Unknown;
  result.out_of_map = state == UfomapOccupancyState::OutOfMap;
}

UfomapMapperConfig buildUfomapConfig(const UfomapMapperParams& params) {
  const UfomapMapperParams defaults;
  UfomapMapperConfig config;

  config.resolution = params.resolution > 0.0 ? params.resolution : defaults.resolution;
  config.depth_levels = params.depth_levels;
  // UFOMap 的 depth 必须落在 MapType 支持范围内，插入/射线 depth 也不能超过实际树深度。
  const int min_depth_levels = static_cast<int>(DufomapMapType::minDepthLevels());
  const int max_depth_levels = static_cast<int>(DufomapMapType::maxDepthLevels());
  config.depth_levels = std::clamp(config.depth_levels, min_depth_levels, max_depth_levels);
  config.insert_hit_depth = std::clamp(params.insert_hit_depth, 0, config.depth_levels);
  config.insert_miss_depth = std::clamp(params.insert_miss_depth, 0, config.depth_levels);
  config.ray_casting_depth = std::clamp(params.ray_casting_depth, 0, config.depth_levels);
  config.min_range = std::max(0.0, params.min_range);
  if (params.max_range == -1.0) {
    config.max_range = params.max_range;
  } else if (params.max_range >= 0.0 && params.max_range > config.min_range) {
    config.max_range = params.max_range;
  } else {
    // max_range 依赖已收束的 min_range；默认最大距离仍越界时退回 -1 的无上限语义。
    config.max_range = defaults.max_range > config.min_range ? defaults.max_range : -1.0;
  }
  config.input_max_z = params.input_max_z > 0.0 ? params.input_max_z : defaults.input_max_z;
  config.num_threads = std::max(0, params.num_threads);
  config.only_valid = params.only_valid;
  config.inflate_unknown = std::max(0, params.inflate_unknown);
  config.inflate_unknown_compensation = params.inflate_unknown_compensation;
  config.ray_passthrough_hits = params.ray_passthrough_hits;
  config.inflate_hits_dist = std::max(0.0, params.inflate_hits_dist);
  if (params.down_sampling_method == "none" || params.down_sampling_method == "center" ||
      params.down_sampling_method == "centroid" || params.down_sampling_method == "uniform") {
    config.down_sampling_method = params.down_sampling_method;
  } else {
    config.down_sampling_method = defaults.down_sampling_method;
  }
  config.simple_ray_casting = params.simple_ray_casting;
  config.simple_ray_casting_factor = params.simple_ray_casting_factor > 0.0
      ? params.simple_ray_casting_factor : defaults.simple_ray_casting_factor;
  config.sliding_window_size = std::max(0, params.sliding_window_size);
  config.parallel = params.parallel;
  config.propagate = params.propagate;
  config.static_map_visualization_max_z = params.static_map_visualization_max_z > 0.0
      ? params.static_map_visualization_max_z : defaults.static_map_visualization_max_z;
  return config;
}

template <class Predicate>
void appendCandidateNodes(const DufomapMapType& map,
                          Predicate&& predicate,
                          const std::size_t max_results,
                          std::vector<UfomapCandidateNode>& candidates) {
  for (const auto node : map.query(std::forward<Predicate>(predicate))) {
    UfomapCandidateNode candidate;
    candidate.center = map.center(node);
    candidate.depth = node.depth();
    candidate.hits = static_cast<std::uint32_t>(map.hits(node));
    candidate.label = static_cast<std::uint32_t>(map.label(node.index()));
    candidate.seen_free = map.seenFree(node);
    candidates.push_back(candidate);
    if (max_results > 0U && candidates.size() >= max_results) {
      return;
    }
  }
}

template <class Predicate>
void appendLocalOccupiedNodes(const DufomapMapType& map,
                              Predicate&& predicate,
                              const std::size_t max_results,
                              std::vector<UfomapLocalOccupiedNode>& nodes) {
  for (const auto node : map.query(std::forward<Predicate>(predicate))) {
    UfomapLocalOccupiedNode occupied_node;
    occupied_node.center = map.center(node);
    occupied_node.depth = node.depth();
    occupied_node.voxel_size = map.size(node.depth());
    occupied_node.hits = static_cast<std::uint32_t>(map.hits(node));
    nodes.push_back(occupied_node);
    if (max_results > 0U && nodes.size() >= max_results) {
      return;
    }
  }
}

ExtractFinitePointsResult extractFinitePoints(const sensor_msgs::PointCloud2& cloud_msg) {
  ExtractFinitePointsResult result;
  result.points.reserve(static_cast<std::size_t>(cloud_msg.width) *
                        static_cast<std::size_t>(cloud_msg.height));
  if (cloud_msg.data.empty()) {
    return result;
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float point_x = *iter_x;
      const float point_y = *iter_y;
      const float point_z = *iter_z;

      if (!std::isfinite(point_x) || !std::isfinite(point_y) || !std::isfinite(point_z)) {
        result.has_invalid_points = true;
        continue;
      }

      result.points.emplace_back(ufo::Point(point_x, point_y, point_z));
    }
  } catch (const std::runtime_error& error) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "failed to access xyz fields from cloud: " << error.what());
    return ExtractFinitePointsResult{};
  }

  return result;
}
}  // namespace

UfomapMapper::UfomapMapper(ros::NodeHandle& nh,
                           ros::NodeHandle& pnh,
                           const bool verbose)
  : nh_(nh),
      pnh_(pnh),
      verbose_(verbose),
      params_(),
      config_(),
      ufomap_update_count_(0U),
      runtime_(nullptr) {
  loadParameters();
  configureUfomap();

  ROS_INFO_STREAM("UFOMap module is ready. resolution: " << config_.resolution
                  << ", depth levels: " << config_.depth_levels
                  << ", min range: " << config_.min_range
                  << ", max range: " << config_.max_range
                  << ", input max z: " << config_.input_max_z);
}

UfomapMapper::~UfomapMapper() = default;

void UfomapMapper::loadParameters() {
  // 默认值只从参数快照结构体取，避免读参处和结构体初值各维护一份数字。
  const UfomapMapperParams defaults;
  pnh_.param("ufomap_resolution", params_.resolution, defaults.resolution);
  pnh_.param("ufomap_depth_levels", params_.depth_levels, defaults.depth_levels);
  pnh_.param("ufomap_min_range", params_.min_range, defaults.min_range);
  pnh_.param("ufomap_max_range", params_.max_range, defaults.max_range);
  pnh_.param("ufomap_input_max_z", params_.input_max_z, defaults.input_max_z);
  pnh_.param("ufomap_insert_hit_depth", params_.insert_hit_depth, defaults.insert_hit_depth);
  pnh_.param("ufomap_insert_miss_depth", params_.insert_miss_depth, defaults.insert_miss_depth);
  pnh_.param("ufomap_ray_casting_depth", params_.ray_casting_depth, defaults.ray_casting_depth);
  pnh_.param("ufomap_num_threads", params_.num_threads, defaults.num_threads);
  pnh_.param("ufomap_insert_only_valid", params_.only_valid, defaults.only_valid);
  pnh_.param("ufomap_inflate_unknown", params_.inflate_unknown, defaults.inflate_unknown);
  pnh_.param("ufomap_inflate_unknown_compensation", params_.inflate_unknown_compensation, defaults.inflate_unknown_compensation);
  pnh_.param("ufomap_ray_passthrough_hits", params_.ray_passthrough_hits, defaults.ray_passthrough_hits);
  pnh_.param("ufomap_inflate_hits_dist", params_.inflate_hits_dist, defaults.inflate_hits_dist);
  pnh_.param("ufomap_simple_ray_casting", params_.simple_ray_casting, defaults.simple_ray_casting);
  pnh_.param("ufomap_simple_ray_casting_factor", params_.simple_ray_casting_factor, defaults.simple_ray_casting_factor);
  pnh_.param("ufomap_sliding_window_size", params_.sliding_window_size, defaults.sliding_window_size);
  pnh_.param("ufomap_parallel", params_.parallel, defaults.parallel);
  pnh_.param("ufomap_propagate", params_.propagate, defaults.propagate);
  pnh_.param("static_map_visualization_max_z", params_.static_map_visualization_max_z, defaults.static_map_visualization_max_z);
  pnh_.param("ufomap_down_sampling_method", params_.down_sampling_method, defaults.down_sampling_method);
  config_ = buildUfomapConfig(params_);
}

void UfomapMapper::configureUfomap() {
  // 该函数会在 clearMap() 后重建 runtime。
  ufomap_update_count_.store(0U);
  runtime_ = std::make_unique<UfomapRuntime>(static_cast<ufo::node_size_t>(config_.resolution),
                                             static_cast<ufo::depth_t>(config_.depth_levels));

  runtime_->integration_params = {};
  runtime_->integration_params.down_sampling_method =
      toUfoDownSamplingMethod(config_.down_sampling_method);
  runtime_->integration_params.hit_depth = static_cast<ufo::depth_t>(config_.insert_hit_depth);
  runtime_->integration_params.miss_depth = static_cast<ufo::depth_t>(config_.insert_miss_depth);
  runtime_->integration_params.ray_casting_depth =
      static_cast<ufo::depth_t>(config_.ray_casting_depth);
  runtime_->integration_params.min_range = static_cast<float>(config_.min_range);
  runtime_->integration_params.max_range = static_cast<float>(config_.max_range);
  runtime_->integration_params.only_valid = config_.only_valid;
  runtime_->integration_params.inflate_unknown = static_cast<std::size_t>(config_.inflate_unknown);
  runtime_->integration_params.inflate_unknown_compensation =
      config_.inflate_unknown_compensation;
  runtime_->integration_params.ray_passthrough_hits = config_.ray_passthrough_hits;
  runtime_->integration_params.inflate_hits_dist = static_cast<float>(config_.inflate_hits_dist);
  runtime_->integration_params.ray_casting_method =
      config_.simple_ray_casting ? ufo::RayCastingMethod::SIMPLE : ufo::RayCastingMethod::PROPER;
  runtime_->integration_params.simple_ray_casting_factor =
      static_cast<float>(config_.simple_ray_casting_factor);
  runtime_->integration_params.sliding_window_size = config_.sliding_window_size;
  runtime_->integration_params.num_threads =
      static_cast<std::size_t>(config_.num_threads);
  runtime_->integration_params.parallel = config_.parallel;
  runtime_->integration_params.propagate = config_.propagate;
}

UfomapMapper::UfomapRuntime& UfomapMapper::runtimeLocked() {
  assert(runtime_ != nullptr && "runtime_ must exist while UfomapMapper is alive");
  return *runtime_;
}

const UfomapMapper::UfomapRuntime& UfomapMapper::runtimeLocked() const {
  assert(runtime_ != nullptr && "runtime_ must exist while UfomapMapper is alive");
  return *runtime_;
}

UfomapFrameResult UfomapMapper::processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                                  const nav_msgs::Odometry& odom_msg) {
  const auto total_start = std::chrono::steady_clock::now();
  UfomapFrameResult result;
  const auto extract_start = std::chrono::steady_clock::now();
  const ExtractFinitePointsResult extracted = extractFinitePoints(cloud_msg);
  const auto extract_end = std::chrono::steady_clock::now();
  result.timing.extract_finite_points_ms = elapsedMs(extract_start, extract_end);

  const ufo::Point sensor_origin(static_cast<float>(odom_msg.pose.pose.position.x),
                                static_cast<float>(odom_msg.pose.pose.position.y),
                                static_cast<float>(odom_msg.pose.pose.position.z));
  // 先从更新前地图提取轻量可视化快照；允许 RViz 迟一帧，但避免当前帧积分污染分类。
  // 这里把 UFOMap 树遍历放进短读锁里，MarkerArray 组装则留到锁外执行。
  const double static_map_visualization_max_z = config_.static_map_visualization_max_z;
  auto static_map_markers_future =
      std::async(std::launch::async, [this, header = cloud_msg.header, static_map_visualization_max_z] {
        StaticMapVisualizationSnapshot snapshot;
        {
          // 异步线程只在抓取旧地图快照时持有读锁，避免整段 marker 构建阻塞写入。
          std::shared_lock<std::shared_mutex> lock(map_mutex_);
          snapshot = captureStaticMapVisualizationSnapshot(
              runtimeLocked().map, static_map_visualization_max_z);
        }
        return buildStaticMapVisualization(header, std::move(snapshot));
      });

  const auto classify_current_start = std::chrono::steady_clock::now();
  // 当前帧输出只关心配置工作半径内的点，避免对远处点做无意义的 seenFree 查询。
  const ufo::PointCloud* input_points = &extracted.points;
  ufo::PointCloud filtered_input_points;
  if (config_.input_max_z > 0.0) {
    filtered_input_points.reserve(extracted.points.size());
    const float input_max_z = static_cast<float>(config_.input_max_z);
    for (const auto& point : extracted.points) {
      if (point.z <= input_max_z) {
        filtered_input_points.push_back(point);
      }
    }
    input_points = &filtered_input_points;
  }

  const ufo::PointCloud* output_points = input_points;
  ufo::PointCloud filtered_output_points;
  if (config_.max_range > 0.0) {
    const float max_range_sq = static_cast<float>(config_.max_range * config_.max_range);
    filtered_output_points.reserve(input_points->size());
    for (const auto& point : *input_points) {
      if (sensor_origin.squaredDistance(point) <= max_range_sq) {
        filtered_output_points.push_back(point);
      }
    }
    output_points = &filtered_output_points;
  }
  result.classification = classifyPoints(cloud_msg.header, *output_points);
  const auto classify_current_end = std::chrono::steady_clock::now();
  result.timing.classify_current_frame_ms =
      elapsedMs(classify_current_start, classify_current_end);

  if (extracted.has_invalid_points) {
    result.classification.static_cloud_msg.is_dense = false;
    result.classification.dynamic_cloud_msg.is_dense = false;
  }
  // 当前阶段主流程只输出当前帧静态/动态点云，静态体素地图则在本函数内异步构建后随结果返回。
  result.static_map_cloud_msg = result.classification.static_cloud_msg;
  result.dynamic_cloud_msg = result.classification.dynamic_cloud_msg;

  const auto update_start = std::chrono::steady_clock::now();
  if (!input_points->empty()) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    auto& runtime = runtimeLocked();
    // 积分与统计更新在同一把锁内，保证 frame_result 与地图状态一致。
    ufo::insertPointCloud(runtime.map, *input_points, sensor_origin,
                          runtime.integration_params,
                          runtime.integration_params.propagate);

    const std::uint64_t update_count = ufomap_update_count_.fetch_add(1U) + 1U;
    result.runtime_stats.update_count = update_count;
    result.runtime_stats.node_count = runtime.map.numNodes();
  }
  const auto update_end = std::chrono::steady_clock::now();
  result.timing.update_ufomap_ms = elapsedMs(update_start, update_end);
  // updateUfomap 统计完成后，再回收锁外组装完成的旧地图 marker 结果。
  result.static_map_markers = static_map_markers_future.get();

  result.timing.process_input_cloud_ms = elapsedMs(total_start, std::chrono::steady_clock::now());
  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Ufomap timing"
                                      << ": processInputCloud="
                                      << formatFloatMs(result.timing.process_input_cloud_ms)
                                      << "ms, extractFinitePoints="
                                      << formatFloatMs(result.timing.extract_finite_points_ms)
                                      << "ms, updateUfomap="
                                      << formatFloatMs(result.timing.update_ufomap_ms)
                                      << "ms, classifyCurrentFrame="
                                      << formatFloatMs(
                                             result.timing.classify_current_frame_ms)
                                      << "ms");
  }
  return result;
}

void UfomapMapper::reserveMap(const std::size_t node_capacity) {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.reserve(node_capacity);
}

void UfomapMapper::clearMap() {
  std::unique_lock<std::shared_mutex> map_lock(map_mutex_);
  configureUfomap();
}

void UfomapMapper::propagateModified() {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.propagateModified();
}

bool UfomapMapper::seenFree(const ufo::Point& point) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return runtimeLocked().map.seenFree(point);
}

UfomapNodeQueryResult UfomapMapper::queryNode(const ufo::Point& point,
                                              const ufo::depth_t depth) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  const auto& runtime = runtimeLocked();
  const ufo::depth_t query_depth = clampQueryDepth(runtime.map, depth);

  UfomapNodeQueryResult result;
  result.depth = query_depth;
  result.voxel_size = runtime.map.size(query_depth);

  const auto code = runtime.map.toCodeChecked(point, query_depth);
  if (!code.has_value()) {
    result.center = point;
    applyOccupancyStateFlags(UfomapOccupancyState::OutOfMap, result);
    return result;
  }

  result.center = runtime.map.toCoord(*code);
  result.seen_free = runtime.map.seenFree(point, query_depth);
  if (!runtime.map.exists(*code)) {
    if (result.seen_free) {
      // 对规划层而言，射线穿越证据已经足以表达 FREE；
      // 即使 exact code 未物化为普通节点，也不应退化成 UNKNOWN。
      result.exists = true;
      applyOccupancyStateFlags(UfomapOccupancyState::Free, result);
    } else {
      applyOccupancyStateFlags(UfomapOccupancyState::Unknown, result);
    }
    return result;
  }

  const auto node = runtime.map(*code);

  result.exists = true;
  result.seen_free = runtime.map.seenFree(node);
  applyOccupancyStateFlags(classifyPlannerOccupancyState(runtime.map, node), result);
  result.depth = node.depth();
  result.voxel_size = runtime.map.size(node.depth());
  return result;
}

void UfomapMapper::setNodeLabel(const ufo::Point& point,
                                const std::uint32_t label,
                                const bool propagate) {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.setLabel(point, static_cast<ufo::label_t>(label), propagate);
}

std::uint32_t UfomapMapper::nodeLabel(const ufo::Point& point) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return static_cast<std::uint32_t>(runtimeLocked().map.label(point));
}

void UfomapMapper::clearAllLabels() {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  auto& runtime = runtimeLocked();

  for (auto it = runtime.map.begin(); it != runtime.map.end(); ++it) {
    runtime.map.setLabel(*it, 0U);
  }
}

UfomapClassificationResult UfomapMapper::classifyPoints(
    const std_msgs::Header& header,
    const UfomapMapper::UfomapPointCloud& points) const {
  UfomapClassificationResult result;
  ufo::PointCloud static_points;
  ufo::PointCloud dynamic_points;
  static_points.reserve(points.size());
  dynamic_points.reserve(points.size());
  result.dynamic_cluster_points.reserve(points.size());

  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    result.input_point_count = points.size();
    const auto& runtime = runtimeLocked();
    for (const auto& point : points) {
      if (runtime.map.seenFree(point)) {
        dynamic_points.push_back(point);
        const auto code = runtime.map.toCodeChecked(point);
        if (code.has_value()) {
          const ufo::Key key = *code;
          UfomapDynamicClusterPoint cluster_point;
          cluster_point.point = point;
          cluster_point.voxel_code.raw_code = code->raw();
          cluster_point.voxel_code.key_x = key.x();
          cluster_point.voxel_code.key_y = key.y();
          cluster_point.voxel_code.key_z = key.z();
          cluster_point.voxel_code.depth = key.depth();
          result.dynamic_cluster_points.push_back(cluster_point);
        }
      } else {
        static_points.push_back(point);
      }
    }
  }

  result.static_point_count = static_points.size();
  result.dynamic_point_count = dynamic_points.size();
  result.static_cloud_msg = buildCloudFromPoints(header, static_points);
  result.dynamic_cloud_msg = buildCloudFromPoints(header, dynamic_points);
  return result;
}

std::vector<UfomapCandidateNode> UfomapMapper::queryDynamicCandidates(
    const UfomapCandidateQuery& query) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  std::vector<UfomapCandidateNode> candidates;
  const auto& runtime = runtimeLocked();

  const auto leaf = ufo::pred::Leaf(clampQueryDepth(runtime.map, query.depth));
  const auto hits = ufo::pred::HitsMin(static_cast<ufo::count_t>(query.min_hits));
  const auto label = ufo::pred::Label(static_cast<ufo::label_t>(query.label));
  // intersects 为空时走全局查询；非空时附加空间约束查询。
  if (query.intersects.empty()) {
    if (query.require_seen_free) {
      appendCandidateNodes(runtime.map, leaf && ufo::pred::SeenFree() && hits && label,
                           query.max_results, candidates);
    } else {
      appendCandidateNodes(runtime.map, leaf && hits && label, query.max_results, candidates);
    }
    return candidates;
  }

  if (query.require_seen_free) {
    appendCandidateNodes(
        runtime.map,
        leaf && ufo::pred::SeenFree() && hits && label && ufo::pred::Intersects(query.intersects),
        query.max_results, candidates);
  } else {
    appendCandidateNodes(runtime.map,
                         leaf && hits && label && ufo::pred::Intersects(query.intersects),
                         query.max_results, candidates);
  }
  return candidates;
}

std::vector<UfomapLocalOccupiedNode> UfomapMapper::queryLocalOccupied(
    const UfomapLocalOccupiedQuery& query) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  std::vector<UfomapLocalOccupiedNode> occupied_nodes;
  const auto& runtime = runtimeLocked();

  const auto leaf = ufo::pred::Leaf(clampQueryDepth(runtime.map, query.depth));
  const auto hits = ufo::pred::HitsMin(static_cast<ufo::count_t>(query.min_hits));
  // P3 只需要静态占据证据；这里不叠加 label/SeenFree，避免把动态候选语义泄漏给规划侧。
  if (query.intersects.empty()) {
    appendLocalOccupiedNodes(runtime.map, leaf && hits, query.max_results, occupied_nodes);
    return occupied_nodes;
  }

  appendLocalOccupiedNodes(runtime.map,
                           leaf && hits && ufo::pred::Intersects(query.intersects),
                           query.max_results,
                           occupied_nodes);
  return occupied_nodes;
}

std::optional<UfomapLocalOccupiedNode> UfomapMapper::queryNearestOccupied(
    const ufo::Point& center,
    const double radius,
    const ufo::depth_t depth,
    const std::uint32_t min_hits) const {
  if (radius < 0.0) {
    return std::nullopt;
  }

  UfomapLocalOccupiedQuery query;
  query.depth = depth;
  query.min_hits = min_hits;
  query.intersects.emplace_back(center, static_cast<float>(radius));
  const auto occupied_nodes = queryLocalOccupied(query);

  std::optional<UfomapLocalOccupiedNode> nearest;
  double nearest_surface_distance = std::numeric_limits<double>::infinity();
  for (const auto& occupied : occupied_nodes) {
    const double surface_distance =
        static_cast<double>(center.distance(occupied.center)) -
        std::max(0.0, occupied.voxel_size) * 0.5;
    if (surface_distance > radius) {
      continue;
    }
    if (surface_distance < nearest_surface_distance) {
      nearest_surface_distance = surface_distance;
      nearest = occupied;
    }
  }
  return nearest;
}

}  // namespace ldopcore
