#include <ldop/ufomap_mapper.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
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
  std::vector<ufo::Point> points;
  ufo::PointCloud ufomap_cloud;
  bool has_invalid_points{false};
};

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string formatFloatMs(const double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

sensor_msgs::PointCloud2 buildCloudFromPoints(const std_msgs::Header& header,
                                              const std::string& world_frame,
                                              const std::vector<ufo::Point>& points) {
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

UfomapPublishData buildStaticMapVisualization(const std_msgs::Header& header,
                                              const std::string& world_frame,
                                              const DufomapMapType& map,
                                              const UfomapMapperParams& params) {
  UfomapPublishData publish_data;

  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.header.frame_id = world_frame;
  clear_marker.ns = "static_map";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  publish_data.static_map_markers.markers.push_back(clear_marker);

  std::map<int, std::vector<geometry_msgs::Point>> occupied_points_by_depth;
  const std::size_t max_voxels =
      params.static_map_visualization_max_voxels > 0
          ? static_cast<std::size_t>(params.static_map_visualization_max_voxels)
          : std::numeric_limits<std::size_t>::max();
  std::size_t exported_voxel_count = 0U;

  // 用 Leaf && HitsMin 先做树级剪枝，再在叶子层手动过滤 seenFree。
  // 原因：!SeenFree 的 InnerCheck 会参与内部节点剪枝；在 seenFree 传播不完全时，
  // 可能提前剪掉仍含有 !seenFree 叶子的分支，导致 RViz 看不到体素。
  for (const auto node : map.query(ufo::pred::Leaf() && ufo::pred::HitsMin(1))) {
    if (map.seenFree(node)) {
      continue;
    }
    if (exported_voxel_count >= max_voxels) {
      break;
    }

    geometry_msgs::Point center;
    const auto node_center = map.center(node);
    center.x = node_center.x;
    center.y = node_center.y;
    center.z = node_center.z;
    occupied_points_by_depth[static_cast<int>(node.depth())].push_back(center);
    ++exported_voxel_count;
  }

  int marker_id = 1;
  for (const auto& [depth, points] : occupied_points_by_depth) {
    if (points.empty()) {
      continue;
    }

    visualization_msgs::Marker marker;
    marker.header = clear_marker.header;
    marker.ns = "static_map_occupied";
    marker.id = marker_id++;
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
    marker.points = points;
    publish_data.static_map_markers.markers.push_back(std::move(marker));
  }

  return publish_data;
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
  result.ufomap_cloud.reserve(static_cast<std::size_t>(cloud_msg.width) *
                              static_cast<std::size_t>(cloud_msg.height));
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
      result.ufomap_cloud.emplace_back(ufo::Point(point_x, point_y, point_z));
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
  pnh_.param("ufomap_parallel", params_.parallel, true);
  pnh_.param("ufomap_propagate", params_.propagate, true);
  pnh_.param("ufomap_map_reserve_nodes", params_.map_reserve_nodes, 0);
  pnh_.param("ufomap_accumulation_max_points", params_.ufomap_accumulation_max_points, 0);
  pnh_.param("static_map_visualization_max_voxels", params_.static_map_visualization_max_voxels, 50000);
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
  sanitized.min_range = std::max(0.0, sanitized.min_range);
  sanitized.max_range = sanitized.max_range > 0.0 ? static_cast<float>(sanitized.max_range) : -1.0f;
  sanitized.inflate_unknown = std::max(0, sanitized.inflate_unknown);
  sanitized.inflate_hits_dist = std::max(0.0, sanitized.inflate_hits_dist);
  sanitized.ufomap_accumulation_max_points = std::max(0, sanitized.ufomap_accumulation_max_points);
  sanitized.static_map_visualization_max_voxels = std::max(0, sanitized.static_map_visualization_max_voxels);
  sanitized.simple_ray_casting_factor = std::isfinite(sanitized.simple_ray_casting_factor) && 
          sanitized.simple_ray_casting_factor > 0.0 ? sanitized.simple_ray_casting_factor: 1.0;
  sanitized.map_reserve_nodes = std::max(0, sanitized.map_reserve_nodes);
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
  if (params_.num_threads > 0) {
    runtime_->integration_params.num_threads = static_cast<std::size_t>(params_.num_threads);}
  runtime_->integration_params.parallel = params_.parallel;
  runtime_->integration_params.propagate = params_.propagate;
  if (params_.map_reserve_nodes > 0) {
    runtime_->map.reserve(static_cast<std::size_t>(params_.map_reserve_nodes));
  }
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

  const auto update_start = extract_end;
  const ufo::Point sensor_origin(static_cast<float>(odom_msg.pose.pose.position.x),
                                static_cast<float>(odom_msg.pose.pose.position.y),
                                static_cast<float>(odom_msg.pose.pose.position.z));
  if (!extracted.ufomap_cloud.empty()) {
    if (!odom_msg.header.frame_id.empty() && odom_msg.header.frame_id != world_frame_) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "ufomap expects world-frame odom, but got "
                                   << odom_msg.header.frame_id
                                   << ", configured world frame: " << world_frame_);
    }

    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    auto& runtime = runtimeLocked();
    // 积分与统计更新在同一把锁内，保证 frame_result 与地图状态一致。
    ufo::insertPointCloud(runtime.map, extracted.ufomap_cloud, sensor_origin,
                          runtime.integration_params, params_.propagate);

    const std::uint64_t update_count = ufomap_update_count_.fetch_add(1U) + 1U;
    result.runtime_stats.update_count = update_count;
    result.runtime_stats.node_count = runtime.map.numNodes();
  }
  const auto update_end = std::chrono::steady_clock::now();
  result.timing.update_ufomap_ms = elapsedMs(update_start, update_end);

  // 地图更新完成后立即启动静态体素可视化构建，让它与当前帧分类并行执行。
  auto publish_data_future = std::async(std::launch::async, [this, header = cloud_msg.header] {
    // 异步构建时显式持有地图读锁，避免再额外绕一层成员转发。
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    return buildStaticMapVisualization(header, world_frame_, runtimeLocked().map, params_);
  });

  const auto accumulate_cache_start = update_end;
  {
    std::lock_guard<std::mutex> lock(accumulated_mutex_);
    // 历史点缓存与实时地图解耦，避免缓存维护阻塞地图查询路径。
    accumulated_points_.insert(accumulated_points_.end(), extracted.points.begin(),
                               extracted.points.end());
    if (params_.ufomap_accumulation_max_points > 0) {
      const auto max_points =
          static_cast<std::size_t>(params_.ufomap_accumulation_max_points);
      if (accumulated_points_.size() > max_points) {
        accumulated_points_.erase(accumulated_points_.begin(),
                                  accumulated_points_.end() -
                                      static_cast<std::ptrdiff_t>(max_points));
      }
    }
  }
  const auto accumulate_cache_end = std::chrono::steady_clock::now();
  result.timing.accumulate_cache_ms = elapsedMs(accumulate_cache_start, accumulate_cache_end);

  const auto classify_current_start = accumulate_cache_end;
  // 当前帧输出只关心配置工作半径内的点，避免对远处点做无意义的 seenFree 查询。
  std::vector<ufo::Point> output_points;
  if (std::isfinite(params_.max_range) && params_.max_range > 0.0) {
    const float max_range_sq = static_cast<float>(params_.max_range * params_.max_range);
    output_points.reserve(extracted.points.size());
    for (const auto& point : extracted.points) {
      if (sensor_origin.squaredDistance(point) <= max_range_sq) {
        output_points.push_back(point);
      }
    }
  } else {
    output_points = extracted.points;
  }
  result.classification = classifyPoints(cloud_msg.header, output_points);
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
  result.publish_data = publish_data_future.get();
  result.runtime_stats.runtime_ready = true;
  
  result.timing.process_input_cloud_ms = elapsedMs(total_start, std::chrono::steady_clock::now());
  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Ufomap timing"
                                      << ": extractFinitePoints="
                                      << formatFloatMs(result.timing.extract_finite_points_ms)
                                      << "ms, updateUfomap="
                                      << formatFloatMs(result.timing.update_ufomap_ms)
                                      << "ms, accumulateCache="
                                      << formatFloatMs(result.timing.accumulate_cache_ms)
                                      << "ms, classifyCurrentFrame="
                                      << formatFloatMs(
                                             result.timing.classify_current_frame_ms)
                                      << "ms");
  }
  return result;
}

UfomapMapperParams UfomapMapper::effectiveParams() const {
  return params_;
}

UfomapRuntimeStats UfomapMapper::runtimeStats() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  UfomapRuntimeStats stats;
  const auto& runtime = runtimeLocked();
  stats.runtime_ready = true;
  stats.update_count = ufomap_update_count_.load();
  stats.node_count = runtime.map.numNodes();
  return stats;
}

void UfomapMapper::reserveMap(const std::size_t node_capacity) {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.reserve(node_capacity);
}

void UfomapMapper::clearMap() {
  std::unique_lock<std::shared_mutex> map_lock(map_mutex_);
  std::lock_guard<std::mutex> accumulated_lock(accumulated_mutex_);
  accumulated_points_.clear();
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

std::size_t UfomapMapper::accumulatedPointCount() const {
  std::lock_guard<std::mutex> lock(accumulated_mutex_);
  return accumulated_points_.size();
}

void UfomapMapper::clearAccumulatedCloud() {
  std::lock_guard<std::mutex> lock(accumulated_mutex_);
  accumulated_points_.clear();
}

UfomapClassificationResult UfomapMapper::classifyPoints(
    const std_msgs::Header& header,
    const std::vector<ufo::Point>& points) const {
  UfomapClassificationResult result;
  std::vector<ufo::Point> static_points;
  std::vector<ufo::Point> dynamic_points;
  static_points.reserve(points.size());
  dynamic_points.reserve(points.size());

  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    result.input_point_count = points.size();
    const auto& runtime = runtimeLocked();
    for (const auto& point : points) {
      if (runtime.map.seenFree(point)) {
        dynamic_points.push_back(point);
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

UfomapClassificationResult UfomapMapper::classifyAccumulatedCloud(
    const std_msgs::Header& header) const {
  std::vector<ufo::Point> points;
  {
    std::lock_guard<std::mutex> lock(accumulated_mutex_);
    points = accumulated_points_;
  }
  return classifyPoints(header, points);
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
