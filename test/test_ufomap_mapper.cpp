#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ldop/ufomap_mapper.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ufo/map/ufomap.hpp>

namespace {

void ensureRosInitialized();

template <typename T>
concept HasAccumulationMaxPointsParam = requires(T value) {
  value.ufomap_accumulation_max_points;
};

template <typename T>
concept HasSlidingWindowSizeParam = requires(T value) {
  value.sliding_window_size;
};

template <typename T>
concept HasMapReserveNodesParam = requires(T value) {
  value.map_reserve_nodes;
};

template <typename T>
concept HasStaticMapVisualizationMaxVoxelsParam = requires(T value) {
  value.static_map_visualization_max_voxels;
};

template <typename T>
concept HasAccumulateCacheMs = requires(T value) {
  value.accumulate_cache_ms;
};

template <typename T>
concept HasAccumulatedPointCountMethod = requires(const T& value) {
  value.accumulatedPointCount();
};

template <typename T>
concept HasClearAccumulatedCloudMethod = requires(T value) {
  value.clearAccumulatedCloud();
};

template <typename T>
concept HasClassifyAccumulatedCloudMethod = requires(const T& value,
                                                     const std_msgs::Header& header) {
  value.classifyAccumulatedCloud(header);
};

}  // namespace

TEST(UfomapMapperApiTest, removesAccumulationRelatedPublicApi) {
  EXPECT_FALSE(HasAccumulationMaxPointsParam<ldopcore::UfomapMapperParams>);
  EXPECT_FALSE(HasMapReserveNodesParam<ldopcore::UfomapMapperParams>);
  EXPECT_FALSE(HasStaticMapVisualizationMaxVoxelsParam<ldopcore::UfomapMapperParams>);
  EXPECT_FALSE(HasAccumulateCacheMs<ldopcore::UfomapTimingStats>);
  EXPECT_FALSE(HasAccumulatedPointCountMethod<ldopcore::UfomapMapper>);
  EXPECT_FALSE(HasClearAccumulatedCloudMethod<ldopcore::UfomapMapper>);
  EXPECT_FALSE(HasClassifyAccumulatedCloudMethod<ldopcore::UfomapMapper>);
}

TEST(UfomapMapperApiTest, exposesSlidingWindowSizeParam) {
  EXPECT_TRUE(HasSlidingWindowSizeParam<ldopcore::UfomapMapperParams>);
}

TEST(UfomapMapperParamsTest, effectiveParamsClampInvalidDepthAndRanges) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_clamp_invalid_depth_and_ranges");
  pnh.setParam("ufomap_depth_levels", 0);
  pnh.setParam("ufomap_min_range", 10.0);
  pnh.setParam("ufomap_max_range", 5.0);
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto params = mapper.effectiveParams();
  EXPECT_EQ(params.depth_levels,
            static_cast<int>(ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
                                      ufo::MapType::LABEL>::minDepthLevels()));
  // 非法的 min_range > max_range 会让积分区间为空，这里应收敛到同一个上界。
  EXPECT_DOUBLE_EQ(params.min_range, 5.0);
  EXPECT_DOUBLE_EQ(params.max_range, 5.0);
}

TEST(UfomapMapperParamsTest, effectiveParamsPreserveUnlimitedMaxRangeSemantics) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_preserve_unlimited_max_range");
  pnh.setParam("ufomap_min_range", 10.0);
  pnh.setParam("ufomap_max_range", 0.0);
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto params = mapper.effectiveParams();
  EXPECT_DOUBLE_EQ(params.min_range, 10.0);
  EXPECT_LE(params.max_range, 0.0);
}

TEST(UfomapMapperParamsTest, effectiveParamsSanitizeNonFiniteRangeValues) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_sanitize_non_finite_ranges");
  pnh.setParam("ufomap_min_range", std::numeric_limits<double>::quiet_NaN());
  pnh.setParam("ufomap_max_range", std::numeric_limits<double>::infinity());
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto params = mapper.effectiveParams();
  EXPECT_DOUBLE_EQ(params.min_range, 0.0);
  EXPECT_LE(params.max_range, 0.0);
}

TEST(UfomapMapperParamsTest, effectiveParamsFixInvalidResolutionAndDepths) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_fix_invalid_resolution_and_depths");
  pnh.setParam("ufomap_resolution", 0.0);
  pnh.setParam("ufomap_depth_levels", std::numeric_limits<int>::max());
  pnh.setParam("ufomap_insert_hit_depth", std::numeric_limits<int>::max());
  pnh.setParam("ufomap_insert_miss_depth", std::numeric_limits<int>::max());
  pnh.setParam("ufomap_ray_casting_depth", std::numeric_limits<int>::max());
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto params = mapper.effectiveParams();
  EXPECT_DOUBLE_EQ(params.resolution, 0.2);
  EXPECT_EQ(params.depth_levels,
            static_cast<int>(ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
                                      ufo::MapType::LABEL>::maxDepthLevels()));
  EXPECT_EQ(params.insert_hit_depth, params.depth_levels);
  EXPECT_EQ(params.insert_miss_depth, params.depth_levels);
  EXPECT_EQ(params.ray_casting_depth, params.depth_levels);
}

TEST(UfomapMapperParamsTest, effectiveParamsSanitizeDufomapIntegrationOptions) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_sanitize_dufomap_options");
  pnh.setParam("ufomap_inflate_unknown", -5);
  pnh.setParam("ufomap_inflate_hits_dist", -0.25);
  pnh.setParam("ufomap_simple_ray_casting_factor", 0.0);
  pnh.setParam("ufomap_down_sampling_method", "uniform");
  pnh.setParam("ufomap_parallel", false);
  pnh.setParam("ufomap_propagate", false);
  pnh.setParam("ufomap_ray_passthrough_hits", false);
  pnh.setParam("ufomap_inflate_unknown_compensation", true);
  pnh.setParam("ufomap_simple_ray_casting", true);
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto params = mapper.effectiveParams();
  EXPECT_EQ(params.inflate_unknown, 0);
  EXPECT_DOUBLE_EQ(params.inflate_hits_dist, 0.0);
  EXPECT_DOUBLE_EQ(params.simple_ray_casting_factor, 1.0);
  EXPECT_EQ(params.down_sampling_method, "uniform");
  EXPECT_FALSE(params.parallel);
  EXPECT_FALSE(params.propagate);
  EXPECT_FALSE(params.ray_passthrough_hits);
  EXPECT_TRUE(params.inflate_unknown_compensation);
  EXPECT_TRUE(params.simple_ray_casting);
}

TEST(UfomapMapperParamsTest, effectiveParamsReadSlidingWindowSize) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_read_sliding_window_size");
  pnh.setParam("ufomap_sliding_window_size", 3);
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  EXPECT_EQ(mapper.effectiveParams().sliding_window_size, 3);
}

TEST(UfomapMapperParamsTest, sanitizeClampsNegativeSlidingWindowSize) {
  ldopcore::UfomapMapperParams params;
  params.sliding_window_size = -4;

  EXPECT_EQ(ldopcore::sanitizeUfomapParams(params).sliding_window_size, 0);
}

TEST(UfomapMapperParamsTest, effectiveParamsSanitizeInvalidDownSamplingMethod) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_params_invalid_down_sampling");
  pnh.setParam("ufomap_down_sampling_method", "invalid");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  EXPECT_EQ(mapper.effectiveParams().down_sampling_method, "center");
}

TEST(UfomapMapperResultTest, exposesStructuredRuntimeStatsDefaults) {
  ldopcore::UfomapPublishData publish_data;
  EXPECT_TRUE(publish_data.static_map_markers.markers.empty());

  ldopcore::UfomapRuntimeStats runtime_stats;
  EXPECT_FALSE(runtime_stats.runtime_ready);
  EXPECT_EQ(runtime_stats.update_count, 0U);
  EXPECT_EQ(runtime_stats.node_count, 0U);

  ldopcore::UfomapTimingStats timing_stats;
  EXPECT_DOUBLE_EQ(timing_stats.process_input_cloud_ms, 0.0);
  EXPECT_DOUBLE_EQ(timing_stats.extract_finite_points_ms, 0.0);
  EXPECT_DOUBLE_EQ(timing_stats.update_ufomap_ms, 0.0);
  EXPECT_DOUBLE_EQ(timing_stats.classify_current_frame_ms, 0.0);

  ldopcore::UfomapFrameResult frame_result;
  EXPECT_FALSE(frame_result.runtime_stats.runtime_ready);
  EXPECT_EQ(frame_result.runtime_stats.update_count, 0U);
  EXPECT_EQ(frame_result.runtime_stats.node_count, 0U);
  EXPECT_EQ(frame_result.classification.static_point_count, 0U);
  EXPECT_EQ(frame_result.classification.dynamic_point_count, 0U);
  EXPECT_EQ(frame_result.classification.input_point_count, 0U);
  EXPECT_TRUE(frame_result.publish_data.static_map_markers.markers.empty());
  EXPECT_DOUBLE_EQ(frame_result.timing.process_input_cloud_ms, 0.0);
  EXPECT_DOUBLE_EQ(frame_result.timing.extract_finite_points_ms, 0.0);
  EXPECT_DOUBLE_EQ(frame_result.timing.update_ufomap_ms, 0.0);
  EXPECT_DOUBLE_EQ(frame_result.timing.classify_current_frame_ms, 0.0);
}

namespace {

void ensureRosInitialized() {
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "test_ufomap_mapper", ros::init_options::AnonymousName);
  }
}

sensor_msgs::PointCloud2 makePointCloud(const std::string& frame_id,
                                        const std::vector<std::array<float, 3>>& points);

sensor_msgs::PointCloud2 makeSinglePointCloud(const std::string& frame_id,
                                              const float x,
                                              const float y,
                                              const float z) {
  return makePointCloud(frame_id, {{{x, y, z}}});
}

sensor_msgs::PointCloud2 makePointCloud(
    const std::string& frame_id,
    const std::vector<std::array<float, 3>>& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = ros::Time(1.0);

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  cloud.is_dense = true;
  return cloud;
}

std::size_t countCloudPoints(const sensor_msgs::PointCloud2& cloud) {
  return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}

ufo::Point makePoint(const float x, const float y, const float z) {
  return ufo::Point(x, y, z);
}

nav_msgs::Odometry makeOdom(const std::string& frame_id) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = frame_id;
  odom.header.stamp = ros::Time(1.0);
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

}  // namespace

TEST(UfomapMapperResultTest, processInputCloudProducesRealRuntimeStats) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto cloud = makeSinglePointCloud("map", 1.0F, 0.0F, 0.0F);
  const auto odom = makeOdom("map");

  const auto result = mapper.processInputCloud(cloud, odom);

  EXPECT_TRUE(result.runtime_stats.runtime_ready);
  EXPECT_EQ(result.runtime_stats.update_count, 1U);
  EXPECT_GT(result.runtime_stats.node_count, 0U);
  EXPECT_GE(result.timing.process_input_cloud_ms, 0.0);
  EXPECT_GE(result.timing.extract_finite_points_ms, 0.0);
  EXPECT_GE(result.timing.update_ufomap_ms, 0.0);
  EXPECT_GE(result.timing.classify_current_frame_ms, 0.0);
}

TEST(UfomapMapperResultTest, processInputCloudBuildsStaticMapMarkers) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_static_map_markers");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto cloud = makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F);
  const auto odom = makeOdom("map");
  const auto result = mapper.processInputCloud(cloud, odom);

  ASSERT_FALSE(result.publish_data.static_map_markers.markers.empty());

  std::size_t rendered_voxel_count = 0U;
  for (const auto& marker : result.publish_data.static_map_markers.markers) {
    EXPECT_NE(marker.action, visualization_msgs::Marker::DELETEALL);
    if (marker.action == visualization_msgs::Marker::ADD) {
      EXPECT_EQ(marker.ns, "static_map_occupied");
      EXPECT_GE(marker.id, 0);
      rendered_voxel_count += marker.points.size();
    }
  }
  EXPECT_GT(rendered_voxel_count, 0U);
}

TEST(UfomapMapperClassificationTest, splitsCurrentFrameBySeenFree) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  ASSERT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result = mapper.processInputCloud(mixed_cloud, odom);

  EXPECT_EQ(result.classification.dynamic_point_count, 1U);
  EXPECT_EQ(result.classification.static_point_count, 1U);
  EXPECT_EQ(countCloudPoints(result.dynamic_cloud_msg), 1U);
  EXPECT_EQ(countCloudPoints(result.classification.static_cloud_msg), 1U);
  EXPECT_EQ(result.classification.input_point_count, 2U);
}

TEST(UfomapMapperClassificationTest, frameResultPublishesCurrentFrameStaticCloud) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  // 先插入一帧不会被后续射线重新穿过的静态点，确认结果仍只反映当前帧分类。
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 0.0F, 2.0F, 0.0F), odom);

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result = mapper.processInputCloud(mixed_cloud, odom);

  EXPECT_EQ(result.classification.static_point_count + result.classification.dynamic_point_count,
            2U);
  ASSERT_EQ(result.classification.static_point_count, 2U);
  EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  EXPECT_EQ(countCloudPoints(result.static_map_cloud_msg),
            result.classification.static_point_count);
  EXPECT_EQ(countCloudPoints(result.dynamic_cloud_msg), result.classification.dynamic_point_count);
}

TEST(UfomapMapperClassificationTest, currentFrameClassificationFiltersPointsOutsideMaxRange) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.setParam("ufomap_max_range", 2.0);
  pnh.setParam("ufomap_insert_only_valid", true);
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  ASSERT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result = mapper.processInputCloud(mixed_cloud, odom);

  // 当前帧输出分支只应分类最大判别范围内的点。
  EXPECT_EQ(result.classification.input_point_count, 1U);
  EXPECT_EQ(result.classification.dynamic_point_count, 1U);
  EXPECT_EQ(result.classification.static_point_count, 0U);
  EXPECT_EQ(countCloudPoints(result.dynamic_cloud_msg), 1U);
  EXPECT_EQ(countCloudPoints(result.static_map_cloud_msg), 0U);
}

TEST(UfomapMapperLifecycleTest, clearMapResetsMapState) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  EXPECT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  mapper.clearMap();

  EXPECT_FALSE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));
}

TEST(UfomapMapperQueryTest, supportsSeenFreeAndNodeQuery) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto cloud = makeSinglePointCloud("map", 1.0F, 0.0F, 0.0F);
  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(cloud, odom);

  EXPECT_TRUE(mapper.seenFree(makePoint(0.5F, 0.0F, 0.0F)));

  const auto node = mapper.queryNode(makePoint(1.0F, 0.0F, 0.0F));
  ASSERT_TRUE(node.has_value());
  EXPECT_TRUE(node->exists);
  EXPECT_TRUE(node->occupied);
  EXPECT_NEAR(node->center.x, 1.0F, 0.25F);
  EXPECT_NEAR(node->center.y, 0.0F, 0.25F);
  EXPECT_NEAR(node->center.z, 0.0F, 0.25F);
}

TEST(UfomapMapperLifecycleTest, exposesRuntimeStatsAndCanClearMap) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  mapper.reserveMap(256U);
  const auto cloud = makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F);
  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(cloud, odom);
  EXPECT_GT(mapper.runtimeStats().node_count, 0U);
  EXPECT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  mapper.clearMap();

  const auto stats = mapper.runtimeStats();
  EXPECT_TRUE(stats.runtime_ready);
  EXPECT_EQ(stats.update_count, 0U);
  EXPECT_FALSE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));
}

TEST(UfomapMapperQueryTest, returnsDufomapStyleDynamicCandidates) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 1.0F, 0.0F, 0.0F), odom);
  mapper.propagateModified();

  ldopcore::UfomapCandidateQuery query;
  query.min_hits = 1U;
  query.label = 0U;
  query.require_seen_free = true;
  const auto candidates = mapper.queryDynamicCandidates(query);

  ASSERT_FALSE(candidates.empty());
  const auto candidate =
      std::find_if(candidates.begin(), candidates.end(), [](const auto& node) {
        return std::abs(node.center.x - 1.0F) < 0.35F;
      });
  ASSERT_NE(candidate, candidates.end());
  EXPECT_TRUE(candidate->seen_free);
  EXPECT_GE(candidate->hits, 1U);
  EXPECT_EQ(candidate->label, 0U);

  mapper.setNodeLabel(candidate->center, 7U, false);
  mapper.propagateModified();
  query.label = 7U;
  const auto labeled_candidates = mapper.queryDynamicCandidates(query);
  EXPECT_FALSE(labeled_candidates.empty());
}

TEST(UfomapMapperLabelTest, supportsSettingReadingAndClearingLabels) {
  ensureRosInitialized();

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto point = makePoint(1.0F, 2.0F, 3.0F);

  EXPECT_EQ(mapper.nodeLabel(point), 0U);

  mapper.setNodeLabel(point, 42U);
  EXPECT_EQ(mapper.nodeLabel(point), 42U);

  mapper.clearAllLabels();
  EXPECT_EQ(mapper.nodeLabel(point), 0U);
}
