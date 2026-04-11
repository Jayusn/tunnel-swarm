#include <ldop/ufomap_mapper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.h>
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

UfomapFrameResult UfomapMapper::processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                                  const nav_msgs::Odometry& odom_msg) {
  const auto total_start = std::chrono::steady_clock::now();
  UfomapFrameResult result;
  ufo::PointCloud ufomap_cloud;
  bool has_invalid_points = false;
  const auto extract_start = std::chrono::steady_clock::now();
  const std::vector<ufo::Point> frame_points =
      extractFinitePoints(cloud_msg, &ufomap_cloud, &has_invalid_points);
  const auto extract_end = std::chrono::steady_clock::now();
  result.timing.extract_finite_points_ms = elapsedMs(extract_start, extract_end);

  const auto update_start = extract_end;
  const ufo::Point sensor_origin(static_cast<float>(odom_msg.pose.pose.position.x),
                                static_cast<float>(odom_msg.pose.pose.position.y),
                                static_cast<float>(odom_msg.pose.pose.position.z));
  updateUfomap(&ufomap_cloud, sensor_origin, odom_msg, result);
  const auto update_end = std::chrono::steady_clock::now();
  result.timing.update_ufomap_ms = elapsedMs(update_start, update_end);

  const auto accumulate_cache_start = update_end;
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    // 历史点缓存仅供低频 clean static map 导出使用，不参与主流程输出与主计时。
    accumulated_points_.insert(accumulated_points_.end(), frame_points.begin(), frame_points.end());
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
    output_points.reserve(frame_points.size());
    for (const auto& point : frame_points) {
      if (sensor_origin.squaredDistance(point) <= max_range_sq) {
        output_points.push_back(point);
      }
    }
  } else {
    output_points = frame_points;
  }
  result.classification = classifyPoints(cloud_msg.header, output_points);
  const auto classify_current_end = std::chrono::steady_clock::now();
  result.timing.classify_current_frame_ms =
      elapsedMs(classify_current_start, classify_current_end);

  if (has_invalid_points) {
    result.classification.static_cloud_msg.is_dense = false;
    result.classification.dynamic_cloud_msg.is_dense = false;
  }
  // 当前阶段主流程只输出当前帧静态/动态点云，静态体素地图另由 UFOMap 查询结果可视化。
  result.static_map_cloud_msg = result.classification.static_cloud_msg;
  result.dynamic_cloud_msg = result.classification.dynamic_cloud_msg;
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    result.runtime_stats.runtime_ready = runtime_ != nullptr;
  }
  
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

void UfomapMapper::updateUfomap(void* ufomap_cloud_ptr,
                                const ufo::Point& sensor_origin,
                                const nav_msgs::Odometry& odom_msg,
                                UfomapFrameResult& frame_result) {
  auto* ufomap_cloud = static_cast<ufo::PointCloud*>(ufomap_cloud_ptr);
  if (ufomap_cloud == nullptr || ufomap_cloud->empty()) {
    return;
  }

  if (!odom_msg.header.frame_id.empty() && odom_msg.header.frame_id != world_frame_) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "ufomap expects world-frame odom, but got "
                                 << odom_msg.header.frame_id << ", configured world frame: "
                                 << world_frame_);
  }

  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return;
  }

  // 积分与统计更新在同一把锁内，保证 frame_result 与地图状态一致。
  ufo::insertPointCloud(runtime_->map, *ufomap_cloud, sensor_origin, runtime_->integration_params,
                        params_.propagate);

  const std::uint64_t update_count = ufomap_update_count_.fetch_add(1U) + 1U;
  const std::size_t node_count = runtime_->map.numNodes();
  frame_result.runtime_stats.update_count = update_count;
  frame_result.runtime_stats.node_count = node_count;
}

void UfomapMapper::publish(const UfomapPublishData& publish_data) {
  (void)publish_data;
}

UfomapMapperParams UfomapMapper::effectiveParams() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  return params_;
}

UfomapRuntimeStats UfomapMapper::runtimeStats() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  UfomapRuntimeStats stats;
  stats.runtime_ready = runtime_ != nullptr;
  stats.update_count = ufomap_update_count_.load();
  stats.node_count = runtime_ != nullptr ? runtime_->map.numNodes() : 0U;
  return stats;
}

void UfomapMapper::reserveMap(const std::size_t node_capacity) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr || node_capacity == 0U) {
    return;
  }
  runtime_->map.reserve(node_capacity);
}

void UfomapMapper::clearMap() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  accumulated_points_.clear();
  configureUfomap();
}

void UfomapMapper::propagateModified() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return;
  }
  runtime_->map.propagateModified();
}

bool UfomapMapper::seenFree(const ufo::Point& point) const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  return runtime_ != nullptr && runtime_->map.seenFree(point);
}

std::optional<UfomapNodeQueryResult> UfomapMapper::queryNode(const ufo::Point& point) const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return std::nullopt;
  }

  const auto code = runtime_->map.toCodeChecked(point);
  if (!code.has_value()) {
    return std::nullopt;
  }

  const auto index = runtime_->map.index(*code);
  if (runtime_->map.toCode(index) != *code) {
    return std::nullopt;
  }

  const auto node = runtime_->map(*code);

  UfomapNodeQueryResult result;
  result.exists = true;
  result.occupied = runtime_->map.hits(node) > 0;
  result.center = runtime_->map.center(node);
  result.depth = node.depth();
  return result;
}

void UfomapMapper::setNodeLabel(const ufo::Point& point,
                                const std::uint32_t label,
                                const bool propagate) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return;
  }

  runtime_->map.setLabel(point, static_cast<ufo::label_t>(label), propagate);
}

std::uint32_t UfomapMapper::nodeLabel(const ufo::Point& point) const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return 0U;
  }

  return static_cast<std::uint32_t>(runtime_->map.label(point));
}

void UfomapMapper::clearAllLabels() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr) {
    return;
  }

  for (auto it = runtime_->map.begin(); it != runtime_->map.end(); ++it) {
    runtime_->map.setLabel(*it, 0U);
  }
}

std::size_t UfomapMapper::accumulatedPointCount() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  return accumulated_points_.size();
}

void UfomapMapper::clearAccumulatedCloud() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  accumulated_points_.clear();
}

std::vector<ufo::Point> UfomapMapper::extractFinitePoints(
    const sensor_msgs::PointCloud2& cloud_msg,
    void* ufomap_cloud_ptr,
    bool* has_invalid_points) const {
  std::vector<ufo::Point> points;
  auto* ufomap_cloud = static_cast<ufo::PointCloud*>(ufomap_cloud_ptr);

  if (ufomap_cloud != nullptr) {
    ufomap_cloud->clear();
    ufomap_cloud->reserve(static_cast<std::size_t>(cloud_msg.width) *
                          static_cast<std::size_t>(cloud_msg.height));
  }
  points.reserve(static_cast<std::size_t>(cloud_msg.width) *
                 static_cast<std::size_t>(cloud_msg.height));

  if (has_invalid_points != nullptr) {
    *has_invalid_points = false;
  }
  if (cloud_msg.data.empty()) {
    return points;
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
        if (has_invalid_points != nullptr) {
          *has_invalid_points = true;
        }
        continue;
      }

      points.emplace_back(ufo::Point(point_x, point_y, point_z));
      if (ufomap_cloud != nullptr) {
        ufomap_cloud->emplace_back(ufo::Point(point_x, point_y, point_z));
      }
    }
  } catch (const std::runtime_error& error) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "failed to access xyz fields from cloud: " << error.what());
    return {};
  }

  return points;
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
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    result.input_point_count = points.size();
    if (runtime_ != nullptr) {
      for (const auto& point : points) {
        if (runtime_->map.seenFree(point)) {
          dynamic_points.push_back(point);
        } else {
          static_points.push_back(point);
        }
      }
    } else {
      static_points = points;
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
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    points = accumulated_points_;
  }
  return classifyPoints(header, points);
}

std::vector<UfomapCandidateNode> UfomapMapper::queryDynamicCandidates(
    const UfomapCandidateQuery& query) const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  std::vector<UfomapCandidateNode> candidates;
  if (runtime_ == nullptr) {
    return candidates;
  }

  const auto leaf = ufo::pred::Leaf(query.depth);
  const auto hits = ufo::pred::HitsMin(static_cast<ufo::count_t>(query.min_hits));
  const auto label = ufo::pred::Label(static_cast<ufo::label_t>(query.label));
  // intersects 为空时走全局查询；非空时附加空间约束查询。
  if (query.intersects.empty()) {
    if (query.require_seen_free) {
      appendCandidateNodes(runtime_->map, leaf && ufo::pred::SeenFree() && hits && label,
                           query.max_results, candidates);
    } else {
      appendCandidateNodes(runtime_->map, leaf && hits && label, query.max_results, candidates);
    }
    return candidates;
  }

  if (query.require_seen_free) {
    appendCandidateNodes(
        runtime_->map,
        leaf && ufo::pred::SeenFree() && hits && label && ufo::pred::Intersects(query.intersects),
        query.max_results, candidates);
  } else {
    appendCandidateNodes(runtime_->map,
                         leaf && hits && label && ufo::pred::Intersects(query.intersects),
                         query.max_results, candidates);
  }
  return candidates;
}

}  // namespace ldopcore
