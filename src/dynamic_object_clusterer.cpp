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

  bool operator==(const ClusterKey& other) const {
    return x == other.x && y == other.y && z == other.z && depth == other.depth;
  }
};
 
struct ClusterKeyHash {
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

bool shiftedKey(const ClusterKey& key,
                const std::array<int, 3>& offset,
                const ufo::key_t step,
                ClusterKey& shifted) {
  shifted = key;
  auto shift_axis = [step](const ufo::key_t value, const int delta, ufo::key_t& out) {
    if (delta < 0) {
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

double extentDiagonal(const double size_x, const double size_y, const double size_z) {
  return std::sqrt(size_x * size_x + size_y * size_y + size_z * size_z);
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

void appendBoundingBoxEdges(const ldop::DynamicObject& object,
                            visualization_msgs::Marker& marker) {
  const double half_x = object.size.x * 0.5;
  const double half_y = object.size.y * 0.5;
  const double half_z = object.size.z * 0.5;
  const double min_x = object.center.x - half_x;
  const double max_x = object.center.x + half_x;
  const double min_y = object.center.y - half_y;
  const double max_y = object.center.y + half_y;
  const double min_z = object.center.z - half_z;
  const double max_z = object.center.z + half_z;

  const std::array<geometry_msgs::Point, 8> corners{
      makeMarkerPoint(min_x, min_y, min_z), makeMarkerPoint(max_x, min_y, min_z),
      makeMarkerPoint(max_x, max_y, min_z), makeMarkerPoint(min_x, max_y, min_z),
      makeMarkerPoint(min_x, min_y, max_z), makeMarkerPoint(max_x, min_y, max_z),
      makeMarkerPoint(max_x, max_y, max_z), makeMarkerPoint(min_x, max_y, max_z)};
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

DynamicObjectClusterer::DynamicObjectClusterer(ros::NodeHandle& pnh, const bool verbose)
    : verbose_(verbose) {
  double min_points = static_cast<double>(params_.min_points);
  pnh.param("dynamic_cluster_min_points", min_points, 3.0);
  pnh.param("dynamic_cluster_max_extent", params_.max_extent, 3.0);
  pnh.param("dynamic_cluster_connectivity", params_.connectivity, 6);

  params_.min_points = min_points > 0.0 ? static_cast<std::size_t>(std::llround(min_points)) : 1U;
  params_.min_points = std::max<std::size_t>(params_.min_points, 1U);
  params_.max_extent =
      std::isfinite(params_.max_extent) && params_.max_extent > 0.0 ? params_.max_extent : 0.0;
  params_.connectivity =
      params_.connectivity == 6 || params_.connectivity == 18 || params_.connectivity == 26
          ? params_.connectivity
          : 6;

  ROS_INFO_STREAM("Dynamic object clusterer is ready. connectivity: " << params_.connectivity
                  << ", min points: " << params_.min_points
                  << ", max extent: " << params_.max_extent);
}

const DynamicObjectClustererParams& DynamicObjectClusterer::params() const {
  return params_;
}

DynamicObjectClustererFrameResult DynamicObjectClusterer::processDynamicObjects(
    const std_msgs::Header& header,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectClustererFrameResult result;

  const auto build_objects_start = std::chrono::steady_clock::now();
  result.dynamic_objects_msg = buildDynamicObjectArray(header, dynamic_points);
  result.timing.build_dynamic_object_array_ms =
      elapsedMs(build_objects_start, std::chrono::steady_clock::now());

  const auto build_markers_start = std::chrono::steady_clock::now();
  result.dynamic_object_markers_msg = buildObjectMarkers(result.dynamic_objects_msg);
  result.timing.build_object_markers_ms =
      elapsedMs(build_markers_start, std::chrono::steady_clock::now());

  result.timing.process_dynamic_objects_ms =
      elapsedMs(total_start, std::chrono::steady_clock::now());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Dynamic object clusterer timing"
                                      << ": processDynamicObjects="
                                      << formatFloatMs(result.timing.process_dynamic_objects_ms)
                                      << "ms, buildDynamicObjectArray="
                                      << formatFloatMs(
                                             result.timing.build_dynamic_object_array_ms)
                                      << "ms, buildObjectMarkers="
                                      << formatFloatMs(result.timing.build_object_markers_ms)
                                      << "ms, dynamicPoints=" << dynamic_points.size()
                                      << ", objects="
                                      << result.dynamic_objects_msg.objects.size());
  }

  return result;
}

ldop::DynamicObjectArray DynamicObjectClusterer::buildDynamicObjectArray(
    const std_msgs::Header& header,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  ldop::DynamicObjectArray object_array;
  object_array.header = header;
  if (dynamic_points.empty()) {
    return object_array;
  }

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

  std::vector<std::size_t> sorted_bucket_indices(buckets.size());
  for (std::size_t i = 0; i < sorted_bucket_indices.size(); ++i) {
    sorted_bucket_indices[i] = i;
  }
  std::sort(sorted_bucket_indices.begin(), sorted_bucket_indices.end(),
            [&buckets](const std::size_t lhs, const std::size_t rhs) {
              const auto& a = buckets[lhs].key;
              const auto& b = buckets[rhs].key;
              return std::tie(a.depth, a.x, a.y, a.z) < std::tie(b.depth, b.x, b.y, b.z);
            });

  // 在 UFOMap key 空间做连通域，避免再对原始点云做半径搜索。
  const auto offsets = neighborOffsets(params_.connectivity);
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
      const std::size_t bucket_index = queue.front();
      queue.pop();

      const auto& bucket = buckets[bucket_index];
      cluster_points.insert(cluster_points.end(),
                            bucket.point_indices.begin(),
                            bucket.point_indices.end());

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

    if (cluster_points.size() < params_.min_points) {
      continue;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (const std::size_t point_index : cluster_points) {
      const auto& point = dynamic_points[point_index].point;
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }

    const double size_x = std::max(max_x - min_x, 0.0);
    const double size_y = std::max(max_y - min_y, 0.0);
    const double size_z = std::max(max_z - min_z, 0.0);
    if (params_.max_extent > 0.0 &&
        extentDiagonal(size_x, size_y, size_z) > params_.max_extent) {
      continue;
    }

    ldop::DynamicObject object;
    object.id = next_id++;
    object.center.x = (min_x + max_x) * 0.5;
    object.center.y = (min_y + max_y) * 0.5;
    object.center.z = (min_z + max_z) * 0.5;
    object.size.x = size_x;
    object.size.y = size_y;
    object.size.z = size_z;
    object.velocity.x = 0.0;
    object.velocity.y = 0.0;
    object.velocity.z = 0.0;
    object_array.objects.push_back(object);
  }

  return object_array;
}

visualization_msgs::MarkerArray DynamicObjectClusterer::buildObjectMarkers(
    const ldop::DynamicObjectArray& object_array) const {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.reserve(object_array.objects.size() + 1U);
  marker_array.markers.push_back(makeDeleteAllMarker(object_array.header));

  for (const auto& object : object_array.objects) {
    visualization_msgs::Marker marker;
    marker.header = object_array.header;
    marker.ns = "dynamic_object_bbox";
    marker.id = static_cast<int>(object.id);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.95F;
    marker.color.g = 0.35F;
    marker.color.b = 0.10F;
    marker.color.a = 1.0F;

    appendBoundingBoxEdges(object, marker);
    marker_array.markers.push_back(std::move(marker));
  }

  return marker_array;
}

}  // namespace ldopcore
