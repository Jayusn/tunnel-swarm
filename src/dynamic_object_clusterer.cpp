#include <ldop/dynamic_object_clusterer.h>
#include <ldop/utils.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

namespace {

struct ClusterKey {
  ufo::key_t x{0};
  ufo::key_t y{0};
  ufo::key_t z{0};
  ufo::depth_t depth{0};

  // 八叉树键相等需同时比较坐标和层级；同坐标不同 depth 代表不同体素。
  bool operator==(const ClusterKey& other) const {
    return x == other.x && y == other.y && z == other.z && depth == other.depth;
  }
};
 
struct ClusterKeyHash {
  // unordered_map 会调用该哈希器把 ClusterKey 映射到哈希桶。
  std::size_t operator()(const ClusterKey& key) const {
    std::size_t seed = static_cast<std::size_t>(key.x);
    seed ^= static_cast<std::size_t>(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(key.depth) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct VoxelBucket {
  ClusterKey key;
  std::vector<std::size_t> point_indices;
};

// 根据 6/18/26 连通规则生成邻域偏移，供后续 key 空间 BFS 使用。
std::vector<std::array<int, 3>> neighborOffsets(const int connectivity) {
  std::vector<std::array<int, 3>> offsets;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const int non_zero = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
        if (non_zero == 0) {
          continue;
        }
        if ((connectivity == 6 && non_zero == 1) ||
            (connectivity == 18 && non_zero <= 2) ||
            (connectivity == 26 && non_zero <= 3)) {
          offsets.push_back({dx, dy, dz});
        }
      }
    }
  }
  return offsets;
}

// 按给定偏移平移 key；负方向平移时做下溢保护。
bool shiftedKey(const ClusterKey& key,
                const std::array<int, 3>& offset,
                const ufo::key_t step,
                ClusterKey& shifted) {
  shifted = key;
  auto shift_axis = [step](const ufo::key_t value, const int delta, ufo::key_t& out) {
    if (delta < 0) {
      // key_t 为无符号，负向平移前先检查是否会下溢。
      if (value < step) {
        return false;
      }
      out = value - step;
      return true;
    }
    if (delta > 0) {
      out = value + step;
      return true;
    }
    out = value;
    return true;
  };

  return shift_axis(key.x, offset[0], shifted.x) &&
         shift_axis(key.y, offset[1], shifted.y) &&
         shift_axis(key.z, offset[2], shifted.z);
}

geometry_msgs::Point makeMarkerPoint(const double x, const double y, const double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

visualization_msgs::Marker makeDeleteAllMarker(const std_msgs::Header& header) {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

void appendBoundingBoxEdges(const BoundingBox3D& bbox, visualization_msgs::Marker& marker) {
  const double half_x = bbox.size.x * 0.5;
  const double half_y = bbox.size.y * 0.5;
  const double half_z = bbox.size.z * 0.5;
  const double min_x = bbox.center.x - half_x;
  const double max_x = bbox.center.x + half_x;
  const double min_y = bbox.center.y - half_y;
  const double max_y = bbox.center.y + half_y;
  const double min_z = bbox.center.z - half_z;
  const double max_z = bbox.center.z + half_z;

    // 8 个角点按“底面 4 点 + 顶面 4 点”的顺序组织，后续通过索引连边。
  const std::array<geometry_msgs::Point, 8> corners{
      makeMarkerPoint(min_x, min_y, min_z), makeMarkerPoint(max_x, min_y, min_z),
      makeMarkerPoint(max_x, max_y, min_z), makeMarkerPoint(min_x, max_y, min_z),
      makeMarkerPoint(min_x, min_y, max_z), makeMarkerPoint(max_x, min_y, max_z),
      makeMarkerPoint(max_x, max_y, max_z), makeMarkerPoint(min_x, max_y, max_z)};
    // 12 条边：底面 4 条、顶面 4 条、上下连接 4 条。
  constexpr std::array<std::array<std::size_t, 2>, 12> kEdges{{
      {{0U, 1U}}, {{1U, 2U}}, {{2U, 3U}}, {{3U, 0U}},
      {{4U, 5U}}, {{5U, 6U}}, {{6U, 7U}}, {{7U, 4U}},
      {{0U, 4U}}, {{1U, 5U}}, {{2U, 6U}}, {{3U, 7U}},
  }};

  marker.points.reserve(kEdges.size() * 2U);
  for (const auto& edge : kEdges) {
    marker.points.push_back(corners[edge[0]]);
    marker.points.push_back(corners[edge[1]]);
  }
}

}  // namespace

DynamicObjectClustererParams sanitizeClustererParams(DynamicObjectClustererParams params) {
  params.min_points = std::max(params.min_points, 1);
  params.max_extent =
      std::isfinite(params.max_extent) && params.max_extent > 0.0 ? params.max_extent : 0.0;
  params.connectivity =
      params.connectivity == 6 || params.connectivity == 18 || params.connectivity == 26
          ? params.connectivity
          : 6;
  return params;
}

DynamicObjectClusterer::DynamicObjectClusterer(ros::NodeHandle& pnh, const bool verbose)
    : pnh_(pnh), params_(), verbose_(verbose) {
  loadParameters();

  ROS_INFO_STREAM("Dynamic object clusterer is ready. connectivity: " << params_.connectivity
                  << ", min points: " << params_.min_points
                  << ", max extent: " << params_.max_extent);
}

void DynamicObjectClusterer::loadParameters() {
  pnh_.param("dynamic_cluster_min_points", params_.min_points, 5);
  pnh_.param("dynamic_cluster_max_extent", params_.max_extent, 3.0);
  pnh_.param("dynamic_cluster_connectivity", params_.connectivity, 6);
  params_ = sanitizeClustererParams(params_);
}

DynamicObjectClustererFrameResult DynamicObjectClusterer::processDynamicObjects(
    const std_msgs::Header& header,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectClustererFrameResult result;

  const auto build_detections_start = std::chrono::steady_clock::now();
  result.detections = buildDetections(header.stamp, dynamic_points);
  result.timing.build_detections_ms =
      elapsedMs(build_detections_start, std::chrono::steady_clock::now());

  const auto build_markers_start = std::chrono::steady_clock::now();
  result.dynamic_object_markers_msg = buildObjectMarkers(header, result.detections);
  result.timing.build_object_markers_ms =
      elapsedMs(build_markers_start, std::chrono::steady_clock::now());

  result.timing.process_dynamic_objects_ms =
      elapsedMs(total_start, std::chrono::steady_clock::now());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Dynamic object clusterer timing"
                                      << ": processDynamicObjects="
                                      << formatFloatMs(result.timing.process_dynamic_objects_ms)
                                      << "ms, buildDetections="
                                      << formatFloatMs(result.timing.build_detections_ms)
                                      << "ms, buildObjectMarkers="
                                      << formatFloatMs(result.timing.build_object_markers_ms)
                                      << "ms, dynamicPoints=" << dynamic_points.size()
                                      << ", detections=" << result.detections.size());
  }

  return result;
}

std::vector<DynamicObjectDetection> DynamicObjectClusterer::buildDetections(
    const ros::Time& stamp,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  std::vector<DynamicObjectDetection> detections;
  if (dynamic_points.empty()) {
    return detections;
  }

  // 1) 先按体素 key 聚合点，后续在 key 空间做连通域，避免对原始点做半径搜索。
  std::vector<VoxelBucket> buckets;
  std::unordered_map<ClusterKey, std::size_t, ClusterKeyHash> bucket_by_key;
  buckets.reserve(dynamic_points.size());
  bucket_by_key.reserve(dynamic_points.size());

  for (std::size_t index = 0; index < dynamic_points.size(); ++index) {
    const auto& code = dynamic_points[index].voxel_code;
    const ClusterKey key{code.key_x, code.key_y, code.key_z, code.depth};
    const auto [it, inserted] = bucket_by_key.emplace(key, buckets.size());
    if (inserted) {
      VoxelBucket bucket;
      bucket.key = key;
      buckets.push_back(std::move(bucket));
    }
    buckets[it->second].point_indices.push_back(index);
  }

  // 2) 固定遍历顺序，降低 unordered_map 无序迭代带来的检测 ID 抖动。
  std::vector<std::size_t> sorted_bucket_indices(buckets.size());
  for (std::size_t i = 0; i < sorted_bucket_indices.size(); ++i) {
    sorted_bucket_indices[i] = i;
  }
  // std::sort 是 C++ 标准库里的“排序算法”，作用是把一段区间按你给的规则排好序。
  // unordered_map 本身遍历顺序不稳定，先把桶索引排好序，后续遍历就稳定了，检测结果顺序和 detection_id 也更不容易抖动。
  std::sort(sorted_bucket_indices.begin(), sorted_bucket_indices.end(),
            [&buckets](const std::size_t lhs, const std::size_t rhs) {
              const auto& a = buckets[lhs].key;
              const auto& b = buckets[rhs].key;
              // 字典序比较：先比 depth，若相等再比 x，再比 y，最后比 z。
              return std::tie(a.depth, a.x, a.y, a.z) < std::tie(b.depth, b.x, b.y, b.z);
            });

  // 3) 在 UFOMap key 空间做 BFS 连通域，每个连通块对应一个候选目标。
  const auto offsets = neighborOffsets(params_.connectivity);  // 根据连通域大小生成邻域偏移，供后续 BFS 使用。
  std::vector<bool> visited(buckets.size(), false);
  std::uint32_t next_id = 0U;

  for (const std::size_t start_bucket : sorted_bucket_indices) {
    if (visited[start_bucket]) {
      continue;
    }

    std::vector<std::size_t> cluster_points;
    std::queue<std::size_t> queue;
    queue.push(start_bucket);
    visited[start_bucket] = true;

    while (!queue.empty()) {
      const std::size_t bucket_index = queue.front(); // 读队首
      queue.pop(); // 弹队首，把这个队首元素从队列中移除。

      const auto& bucket = buckets[bucket_index];
      // 将当前桶里的点索引并入当前连通块。
      cluster_points.insert(cluster_points.end(),
                            bucket.point_indices.begin(),
                            bucket.point_indices.end());

      // 同层 key 邻居步长由 depth 决定。
      const ufo::key_t step = ufo::key_t{1} << bucket.key.depth;
      for (const auto& offset : offsets) {
        ClusterKey neighbor;
        if (!shiftedKey(bucket.key, offset, step, neighbor)) {
          continue;
        }
        const auto found = bucket_by_key.find(neighbor);
        if (found == bucket_by_key.end() || visited[found->second]) {
          continue;
        }
        visited[found->second] = true;
        queue.push(found->second);
      }
    }

    // 点数不足的连通块视为噪声。
    if (cluster_points.size() < static_cast<std::size_t>(params_.min_points)) {
      continue;
    }

    // 4) 同时计算当前连通块的轴对齐包围盒 (AABB) 和点簇质心。
    // AABB 尺寸用于可视化/尺度过滤；质心用于目标位置观测，和 LDOT 的 bbox.x/y 语义对齐。
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (const std::size_t point_index : cluster_points) {
      const auto& point = dynamic_points[point_index].point;
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      max_z = std::max(max_z, static_cast<double>(point.z));
      sum_x += static_cast<double>(point.x);
      sum_y += static_cast<double>(point.y);
      sum_z += static_cast<double>(point.z);
    }

    const double size_x = std::max(max_x - min_x, 0.0);
    const double size_y = std::max(max_y - min_y, 0.0);
    const double size_z = std::max(max_z - min_z, 0.0);
    // 5) 用包围盒对角线做尺寸过滤，剔除异常大目标。
    if (params_.max_extent > 0.0 &&
        std::sqrt(size_x * size_x + size_y * size_y + size_z * size_z) >
            params_.max_extent) {
      continue;
    }

    // 6) 生成检测输出：center 使用点簇质心，size 保留 AABB 尺寸。
    DynamicObjectDetection detection;
    detection.detection_id = next_id++;
    const double point_count = static_cast<double>(cluster_points.size());
    detection.bbox.center.x = sum_x / point_count;
    detection.bbox.center.y = sum_y / point_count;
    detection.bbox.center.z = sum_z / point_count;
    detection.bbox.size.x = size_x;
    detection.bbox.size.y = size_y;
    detection.bbox.size.z = size_z;
    detection.bbox.yaw = 0.0;
    detection.stamp = stamp;
    detections.push_back(detection);
  }

  return detections;
}

visualization_msgs::MarkerArray DynamicObjectClusterer::buildObjectMarkers(
    const std_msgs::Header& header,
    const std::vector<DynamicObjectDetection>& detections) const {
  visualization_msgs::MarkerArray marker_array;
  // 先清空历史 marker，避免旧框残留在 RViz。
  marker_array.markers.reserve(detections.size() + 1U);
  marker_array.markers.push_back(makeDeleteAllMarker(header));

  // 每个检测输出一个 LINE_LIST 包围盒。
  for (const auto& detection : detections) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "dynamic_object_bbox";
    marker.id = static_cast<int>(detection.detection_id);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.95F;
    marker.color.g = 0.35F;
    marker.color.b = 0.10F;
    marker.color.a = 1.0F;

    appendBoundingBoxEdges(detection.bbox, marker);
    marker_array.markers.push_back(std::move(marker));
  }

  return marker_array;
}

}  // namespace ldopcore
