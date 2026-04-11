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

TEST(UfomapMapperParamsTest, sanitizeInvalidDepthAndRangeValues) {
  ldopcore::UfomapMapperParams params;
  params.depth_levels = 0;
  params.min_range = 10.0;
  params.max_range = 5.0;

  const auto sanitized = ldopcore::sanitizeUfomapParams(params);

  EXPECT_EQ(sanitized.depth_levels,
            static_cast<int>(ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
                                      ufo::MapType::LABEL>::minDepthLevels()));
  EXPECT_DOUBLE_EQ(sanitized.min_range, 5.0);
  EXPECT_DOUBLE_EQ(sanitized.max_range, 5.0);
}

TEST(UfomapMapperParamsTest, preserveUnlimitedMaxRangeSemantics) {
  ldopcore::UfomapMapperParams params;
  params.min_range = 10.0;
  params.max_range = 0.0;

  const auto sanitized = ldopcore::sanitizeUfomapParams(params);

  EXPECT_DOUBLE_EQ(sanitized.min_range, 10.0);
  EXPECT_DOUBLE_EQ(sanitized.max_range, 0.0);
}

TEST(UfomapMapperParamsTest, clampOversizedDepthAndFixInvalidResolution) {
  ldopcore::UfomapMapperParams params;
  params.resolution = 0.0;
  params.depth_levels = std::numeric_limits<int>::max();
  params.insert_hit_depth = std::numeric_limits<int>::max();
  params.insert_miss_depth = std::numeric_limits<int>::max();
  params.ray_casting_depth = std::numeric_limits<int>::max();

  const auto sanitized = ldopcore::sanitizeUfomapParams(params);

  EXPECT_DOUBLE_EQ(sanitized.resolution, 0.2);
  EXPECT_EQ(sanitized.depth_levels,
            static_cast<int>(ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
                                      ufo::MapType::LABEL>::maxDepthLevels()));
  EXPECT_EQ(sanitized.insert_hit_depth, sanitized.depth_levels);
  EXPECT_EQ(sanitized.insert_miss_depth, sanitized.depth_levels);
  EXPECT_EQ(sanitized.ray_casting_depth, sanitized.depth_levels);
}

TEST(UfomapMapperParamsTest, sanitizeDufomapIntegrationOptions) {
  ldopcore::UfomapMapperParams params;
  params.inflate_unknown = -5;
  params.inflate_hits_dist = -0.25;
  params.simple_ray_casting_factor = 0.0;
  params.map_reserve_nodes = -10;
  params.down_sampling_method = "uniform";
  params.parallel = false;
  params.propagate = false;
  params.ray_passthrough_hits = false;
  params.inflate_unknown_compensation = true;
  params.simple_ray_casting = true;

  const auto sanitized = ldopcore::sanitizeUfomapParams(params);

  EXPECT_EQ(sanitized.inflate_unknown, 0);
  EXPECT_DOUBLE_EQ(sanitized.inflate_hits_dist, 0.0);
  EXPECT_DOUBLE_EQ(sanitized.simple_ray_casting_factor, 1.0);
  EXPECT_EQ(sanitized.map_reserve_nodes, 0);
  EXPECT_EQ(sanitized.ufomap_accumulation_max_points, 0);
  EXPECT_EQ(sanitized.ufomap_clustering_min_points, 1);
  EXPECT_DOUBLE_EQ(sanitized.ufomap_clustering_max_distance, 0.2);
  EXPECT_EQ(sanitized.ufomap_clustering_depth, 0);
  EXPECT_EQ(sanitized.down_sampling_method, "uniform");
  EXPECT_FALSE(sanitized.parallel);
  EXPECT_FALSE(sanitized.propagate);
  EXPECT_FALSE(sanitized.ray_passthrough_hits);
  EXPECT_TRUE(sanitized.inflate_unknown_compensation);
  EXPECT_TRUE(sanitized.simple_ray_casting);
}

TEST(UfomapMapperParamsTest, sanitizeInvalidDownSamplingMethod) {
  ldopcore::UfomapMapperParams params;
  params.down_sampling_method = "invalid";

  const auto sanitized = ldopcore::sanitizeUfomapParams(params);

  EXPECT_EQ(sanitized.down_sampling_method, "center");
}

TEST(UfomapMapperResultTest, exposesStructuredRuntimeStatsDefaults) {
  ldopcore::UfomapRuntimeStats runtime_stats;
  EXPECT_FALSE(runtime_stats.runtime_ready);
  EXPECT_EQ(runtime_stats.update_count, 0U);
  EXPECT_EQ(runtime_stats.node_count, 0U);

  ldopcore::UfomapFrameResult frame_result;
  EXPECT_FALSE(frame_result.runtime_stats.runtime_ready);
  EXPECT_EQ(frame_result.runtime_stats.update_count, 0U);
  EXPECT_EQ(frame_result.runtime_stats.node_count, 0U);
  EXPECT_EQ(frame_result.publish_data.update_count, 0U);
  EXPECT_EQ(frame_result.publish_data.node_count, 0U);
  EXPECT_EQ(frame_result.classification.static_point_count, 0U);
  EXPECT_EQ(frame_result.classification.dynamic_point_count, 0U);
  EXPECT_EQ(frame_result.classification.accumulated_point_count, 0U);
  EXPECT_EQ(frame_result.accumulated_classification.static_point_count, 0U);
  EXPECT_EQ(frame_result.accumulated_classification.dynamic_point_count, 0U);
  EXPECT_EQ(frame_result.accumulated_classification.accumulated_point_count, 0U);
  EXPECT_EQ(frame_result.clustering.cluster_count, 0U);
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

TEST(UfomapMapperResultTest, processStaticCloudProducesRealRuntimeStats) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto cloud = makeSinglePointCloud("map", 1.0F, 0.0F, 0.0F);
  const auto odom = makeOdom("map");

  const auto result = mapper.processStaticCloud(cloud, odom);

  EXPECT_TRUE(result.runtime_stats.runtime_ready);
  EXPECT_EQ(result.runtime_stats.update_count, 1U);
  EXPECT_GT(result.runtime_stats.node_count, 0U);
  EXPECT_EQ(result.runtime_stats.update_count, result.publish_data.update_count);
  EXPECT_EQ(result.runtime_stats.node_count, result.publish_data.node_count);
}

TEST(UfomapMapperClassificationTest, splitsCurrentFrameBySeenFreeAndTracksAccumulation) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processStaticCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  ASSERT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result = mapper.processStaticCloud(mixed_cloud, odom);

  EXPECT_EQ(result.classification.dynamic_point_count, 1U);
  EXPECT_EQ(result.classification.static_point_count, 1U);
  EXPECT_EQ(countCloudPoints(result.dynamic_cloud_msg), 1U);
  EXPECT_EQ(countCloudPoints(result.static_cloud_msg), 1U);
  EXPECT_EQ(result.classification.accumulated_point_count, 3U);
  EXPECT_EQ(mapper.accumulatedPointCount(), 3U);

  const auto accumulated = mapper.classifyAccumulatedCloud(mixed_cloud.header);
  EXPECT_EQ(accumulated.accumulated_point_count, 3U);
  EXPECT_GE(accumulated.dynamic_point_count, 1U);
}

TEST(UfomapMapperClassificationTest, frameResultPublishesAccumulatedStaticMapCloud) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processStaticCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result = mapper.processStaticCloud(mixed_cloud, odom);

  EXPECT_EQ(result.classification.static_point_count + result.classification.dynamic_point_count,
            2U);
  EXPECT_EQ(result.accumulated_classification.accumulated_point_count, 3U);
  EXPECT_EQ(result.accumulated_classification.static_point_count +
                result.accumulated_classification.dynamic_point_count,
            3U);
  EXPECT_EQ(countCloudPoints(result.static_cloud_msg),
            result.accumulated_classification.static_point_count);
  EXPECT_EQ(countCloudPoints(result.dynamic_cloud_msg), result.classification.dynamic_point_count);
}

TEST(UfomapMapperLifecycleTest, clearMapAlsoClearsAccumulatedCloud) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processStaticCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  EXPECT_EQ(mapper.accumulatedPointCount(), 1U);

  mapper.clearMap();

  EXPECT_EQ(mapper.accumulatedPointCount(), 0U);
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
  (void)mapper.processStaticCloud(cloud, odom);

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
  (void)mapper.processStaticCloud(cloud, odom);
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
  (void)mapper.processStaticCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  (void)mapper.processStaticCloud(makeSinglePointCloud("map", 1.0F, 0.0F, 0.0F), odom);
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

TEST(UfomapMapperClusteringTest, labelsDynamicCandidatesWithBfsClusters) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.setParam("ufomap_clustering_enabled", true);
  pnh.setParam("ufomap_clustering_min_points", 1);
  pnh.setParam("ufomap_clustering_max_distance", 0.35);
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processStaticCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  const auto result = mapper.processStaticCloud(makeSinglePointCloud("map", 1.0F, 0.0F, 0.0F),
                                                odom);

  EXPECT_GE(result.clustering.cluster_count, 1U);
  EXPECT_GE(result.clustering.labeled_node_count, 1U);
  EXPECT_GT(mapper.nodeLabel(makePoint(1.0F, 0.0F, 0.0F)), 0U);
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
