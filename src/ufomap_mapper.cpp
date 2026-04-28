#include <ldop/ufomap_mapper.h>
#include <ldop/utils.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <future>
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

sensor_msgs::PointCloud2 buildCloudFromPoints(const std_msgs::Header& header,
                                              const std::string& world_frame,
                                              const ufo::PointCloud& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = world_frame;

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

visualization_msgs::MarkerArray buildStaticMapVisualization(const std_msgs::Header& header,
                                                            const std::string& world_frame,
                                                            const DufomapMapType& map) {
  visualization_msgs::MarkerArray static_map_markers;

  const int max_depth = static_cast<int>(map.depthLevels());
  std::vector<visualization_msgs::Marker> markers_by_depth(
      static_cast<std::size_t>(max_depth) + 1U);
  for (int depth = 0; depth <= max_depth; ++depth) {
    auto& marker = markers_by_depth[static_cast<std::size_t>(depth)];
    marker.header = header;
    marker.header.frame_id = world_frame;
    marker.ns = "static_map_occupied";
    marker.id = depth;
    marker.action = visualization_msgs::Marker::DELETE;
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

    auto& marker = markers_by_depth[static_cast<std::size_t>(depth)];
    if (marker.action != visualization_msgs::Marker::ADD) {
      // ns/id 按 depth 固定，RViz 会用本帧内容覆盖上一帧同一 marker。
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = map.size(static_cast<ufo::depth_t>(depth));
      marker.scale.y = marker.scale.x;
      marker.scale.z = marker.scale.x;
      marker.color.r = 0.10f;
      marker.color.g = 0.85f;
      marker.color.b = 0.25f;
      marker.color.a = 1.0f;
    }

    geometry_msgs::Point center;
    const auto node_center = map.center(node);
    center.x = node_center.x;
    center.y = node_center.y;
    center.z = node_center.z;
    marker.points.push_back(center);
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
                           const std::string& world_frame,
                           const bool verbose)
  : nh_(nh),
      pnh_(pnh),
      world_frame_(world_frame),
      verbose_(verbose),
      params_(),
      ufomap_update_count_(0U),
      runtime_(nullptr) {
  loadParameters();
  configureUfomap();

  ROS_INFO_STREAM("UFOMap module is ready. resolution: " << params_.resolution
                  << ", depth levels: " << params_.depth_levels
                  << ", min range: " << params_.min_range
                  << ", max range: " << params_.max_range
                  << ", world frame: " << world_frame_);
}

UfomapMapper::~UfomapMapper() = default;

void UfomapMapper::loadParameters() {
  pnh_.param("ufomap_resolution", params_.resolution, 0.2);
  pnh_.param("ufomap_depth_levels", params_.depth_levels, 16);
  pnh_.param("ufomap_min_range", params_.min_range, 0.5);
  pnh_.param("ufomap_max_range", params_.max_range, 20.0);
  pnh_.param("ufomap_insert_hit_depth", params_.insert_hit_depth, 0);
  pnh_.param("ufomap_insert_miss_depth", params_.insert_miss_depth, 0);
  pnh_.param("ufomap_ray_casting_depth", params_.ray_casting_depth, 0);
  pnh_.param("ufomap_num_threads", params_.num_threads, 0);
  pnh_.param("ufomap_insert_only_valid", params_.only_valid, false);
  pnh_.param("ufomap_inflate_unknown", params_.inflate_unknown, 0);
  pnh_.param("ufomap_inflate_unknown_compensation", params_.inflate_unknown_compensation, false);
  pnh_.param("ufomap_ray_passthrough_hits", params_.ray_passthrough_hits, true);
  pnh_.param("ufomap_inflate_hits_dist", params_.inflate_hits_dist, 0.0);
  pnh_.param("ufomap_simple_ray_casting", params_.simple_ray_casting, false);
  pnh_.param("ufomap_simple_ray_casting_factor", params_.simple_ray_casting_factor, 1.0);
  pnh_.param("ufomap_sliding_window_size", params_.sliding_window_size, 0);
  pnh_.param("ufomap_parallel", params_.parallel, true);
  pnh_.param("ufomap_propagate", params_.propagate, true);
  pnh_.param<std::string>("ufomap_down_sampling_method", params_.down_sampling_method, "center");
  params_ = sanitizeUfomapParams(params_);
}

UfomapMapperParams sanitizeUfomapParams(const UfomapMapperParams& params) {
  UfomapMapperParams sanitized = params;
  // 统一做参数合法化，避免非法值进入 UFOMap 积分/查询路径。
  sanitized.resolution =
      std::isfinite(sanitized.resolution) && sanitized.resolution > 0.0 ? sanitized.resolution : 0.2;
  const int min_depth_levels = static_cast<int>(DufomapMapType::minDepthLevels());
  const int max_depth_levels = static_cast<int>(DufomapMapType::maxDepthLevels());
  sanitized.depth_levels = std::clamp(sanitized.depth_levels, min_depth_levels, max_depth_levels);
  sanitized.insert_hit_depth = std::clamp(sanitized.insert_hit_depth, 0, sanitized.depth_levels);
  sanitized.insert_miss_depth = std::clamp(sanitized.insert_miss_depth, 0, sanitized.depth_levels);
  sanitized.ray_casting_depth = std::clamp(sanitized.ray_casting_depth, 0, sanitized.depth_levels);
  sanitized.num_threads = std::max(0, sanitized.num_threads);
  sanitized.min_range =
      std::isfinite(sanitized.min_range) ? std::max(0.0, sanitized.min_range) : 0.0;
  // max_range 使用 <=0 表示不限制；非有限值也统一折叠到这个语义，避免把 inf 传进积分配置。
  sanitized.max_range = std::isfinite(sanitized.max_range) && sanitized.max_range > 0.0
                            ? sanitized.max_range
                            : -1.0;
  // 当 min_range 大于有效的 max_range 时，收敛到同一上界，避免出现空积分区间。
  if (sanitized.max_range > 0.0 && sanitized.min_range > sanitized.max_range) {
    sanitized.min_range = sanitized.max_range;
  }
  sanitized.inflate_unknown = std::max(0, sanitized.inflate_unknown);
  sanitized.inflate_hits_dist = std::max(0.0, sanitized.inflate_hits_dist);
  sanitized.simple_ray_casting_factor = std::isfinite(sanitized.simple_ray_casting_factor) && 
          sanitized.simple_ray_casting_factor > 0.0 ? sanitized.simple_ray_casting_factor: 1.0;
  sanitized.sliding_window_size = std::max(0, sanitized.sliding_window_size);
  if (sanitized.down_sampling_method != "none" && sanitized.down_sampling_method != "center" &&
      sanitized.down_sampling_method != "centroid" && sanitized.down_sampling_method != "uniform") {
    sanitized.down_sampling_method = "center";
  }
  return sanitized;
}

void UfomapMapper::configureUfomap() {
  // 该函数会在 clearMap() 后重建 runtime。
  ufomap_update_count_.store(0U);
  runtime_ = std::make_unique<UfomapRuntime>(static_cast<ufo::node_size_t>(params_.resolution),
                                             static_cast<ufo::depth_t>(params_.depth_levels));

  runtime_->integration_params = {};
  runtime_->integration_params.down_sampling_method = toUfoDownSamplingMethod(params_.down_sampling_method);
  runtime_->integration_params.hit_depth = static_cast<ufo::depth_t>(params_.insert_hit_depth);
  runtime_->integration_params.miss_depth = static_cast<ufo::depth_t>(params_.insert_miss_depth);
  runtime_->integration_params.ray_casting_depth = static_cast<ufo::depth_t>(params_.ray_casting_depth);
  runtime_->integration_params.min_range = static_cast<float>(params_.min_range);
  runtime_->integration_params.max_range = static_cast<float>(params_.max_range);
  runtime_->integration_params.only_valid = params_.only_valid;
  runtime_->integration_params.inflate_unknown = static_cast<std::size_t>(params_.inflate_unknown);
  runtime_->integration_params.inflate_unknown_compensation = params_.inflate_unknown_compensation;
  runtime_->integration_params.ray_passthrough_hits = params_.ray_passthrough_hits;
  runtime_->integration_params.inflate_hits_dist = static_cast<float>(params_.inflate_hits_dist);
  runtime_->integration_params.ray_casting_method =
      params_.simple_ray_casting ? ufo::RayCastingMethod::SIMPLE : ufo::RayCastingMethod::PROPER;
  runtime_->integration_params.simple_ray_casting_factor = static_cast<float>(params_.simple_ray_casting_factor);
  runtime_->integration_params.sliding_window_size = params_.sliding_window_size;
  if (params_.num_threads > 0) {
    runtime_->integration_params.num_threads = static_cast<std::size_t>(params_.num_threads);}
  runtime_->integration_params.parallel = params_.parallel;
  runtime_->integration_params.propagate = params_.propagate;
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
  // 先用更新前地图生成可视化快照；允许 RViz 迟一帧，但避免当前帧积分污染分类。
  auto static_map_markers_future = std::async(std::launch::async, [this, header = cloud_msg.header] {
    // 异步构建时显式持有地图读锁，避免再额外绕一层成员转发。
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    return buildStaticMapVisualization(header, world_frame_, runtimeLocked().map);
  });

  const auto classify_current_start = std::chrono::steady_clock::now();
  // 当前帧输出只关心配置工作半径内的点，避免对远处点做无意义的 seenFree 查询。
  const ufo::PointCloud* output_points = &extracted.points;
  ufo::PointCloud filtered_output_points;
  if (params_.max_range > 0.0) {
    const float max_range_sq = static_cast<float>(params_.max_range * params_.max_range);
    filtered_output_points.reserve(extracted.points.size());
    for (const auto& point : extracted.points) {
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
  // 必须在写入当前帧前取回旧地图快照，避免 updateUfomap 计时里夹杂异步读锁等待。
  result.static_map_markers = static_map_markers_future.get();

  const auto update_start = std::chrono::steady_clock::now();
  if (!extracted.points.empty()) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    auto& runtime = runtimeLocked();
    // 积分与统计更新在同一把锁内，保证 frame_result 与地图状态一致。
    ufo::insertPointCloud(runtime.map, extracted.points, sensor_origin,
                          runtime.integration_params,
                          runtime.integration_params.propagate);

    const std::uint64_t update_count = ufomap_update_count_.fetch_add(1U) + 1U;
    result.runtime_stats.update_count = update_count;
    result.runtime_stats.node_count = runtime.map.numNodes();
  }
  const auto update_end = std::chrono::steady_clock::now();
  result.timing.update_ufomap_ms = elapsedMs(update_start, update_end);

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

std::optional<UfomapNodeQueryResult> UfomapMapper::queryNode(const ufo::Point& point) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  const auto& runtime = runtimeLocked();

  const auto code = runtime.map.toCodeChecked(point);
  if (!code.has_value()) {
    return std::nullopt;
  }

  if (!runtime.map.exists(*code)) {
    return std::nullopt;
  }

  const auto node = runtime.map(*code);

  UfomapNodeQueryResult result;
  result.exists = true;
  result.occupied = runtime.map.hits(node) > 0;
  result.center = runtime.map.center(node);
  result.depth = node.depth();
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
  result.static_cloud_msg = buildCloudFromPoints(header, world_frame_, static_points);
  result.dynamic_cloud_msg = buildCloudFromPoints(header, world_frame_, dynamic_points);
  return result;
}

std::vector<UfomapCandidateNode> UfomapMapper::queryDynamicCandidates(
    const UfomapCandidateQuery& query) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  std::vector<UfomapCandidateNode> candidates;
  const auto& runtime = runtimeLocked();

  const auto leaf = ufo::pred::Leaf(query.depth);
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

}  // namespace ldopcore
