#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <ldop/dynamic_object_clusterer.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace {

void ensureRosInitialized() {
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "test_dynamic_object_clusterer", ros::init_options::AnonymousName);
  }
}

void skipIfNoRosMaster() {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }
}

ldopcore::UfomapDynamicClusterPoint makeClusterPoint(const float x,
                                                     const float y,
                                                     const float z,
                                                     const ufo::key_t key_x,
                                                     const ufo::key_t key_y,
                                                     const ufo::key_t key_z,
                                                     const ufo::depth_t depth = 0) {
  ldopcore::UfomapDynamicClusterPoint point;
  point.point = ufo::Point(x, y, z);
  point.voxel_code.key_x = key_x;
  point.voxel_code.key_y = key_y;
  point.voxel_code.key_z = key_z;
  point.voxel_code.depth = depth;
  point.voxel_code.raw_code = static_cast<ufo::code_t>(key_x) << 32 |
                              static_cast<ufo::code_t>(key_y) << 16 |
                              static_cast<ufo::code_t>(key_z);
  return point;
}

std_msgs::Header makeHeader() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(1.0);
  return header;
}

ldop::DynamicObjectArray makeObjectArray() {
  ldop::DynamicObjectArray objects;
  objects.header = makeHeader();
  return objects;
}

ldopcore::DynamicObjectClusterer makeClusterer(const int connectivity,
                                               const std::size_t min_points = 1U,
                                               const double max_extent = 0.0) {
  ensureRosInitialized();
  const std::string ns = "dynamic_object_clusterer_test_" + std::to_string(connectivity) + "_" +
                         std::to_string(min_points) + "_" +
                         std::to_string(static_cast<int>(std::llround(max_extent * 1000.0)));
  ros::NodeHandle pnh(ns);
  pnh.setParam("dynamic_cluster_min_points", static_cast<double>(min_points));
  pnh.setParam("dynamic_cluster_max_extent", max_extent);
  pnh.setParam("dynamic_cluster_connectivity", connectivity);
  return ldopcore::DynamicObjectClusterer(pnh);
}

}  // namespace

TEST(DynamicObjectClustererParamsTest, constructorUsesDefaultParameters) {
  skipIfNoRosMaster();

  ros::NodeHandle pnh("dynamic_object_clusterer_default_params");
  pnh.deleteParam("dynamic_cluster_min_points");
  pnh.deleteParam("dynamic_cluster_max_extent");
  pnh.deleteParam("dynamic_cluster_connectivity");

  const ldopcore::DynamicObjectClusterer clusterer(pnh);

  EXPECT_EQ(clusterer.params().min_points, 3U);
  EXPECT_DOUBLE_EQ(clusterer.params().max_extent, 3.0);
  EXPECT_EQ(clusterer.params().connectivity, 6);
}

TEST(DynamicObjectClustererParamsTest, constructorSanitizesInvalidParameters) {
  skipIfNoRosMaster();

  ros::NodeHandle pnh("dynamic_object_clusterer_invalid_params");
  pnh.setParam("dynamic_cluster_min_points", 0.0);
  pnh.setParam("dynamic_cluster_max_extent", std::numeric_limits<double>::quiet_NaN());
  pnh.setParam("dynamic_cluster_connectivity", 12);

  const ldopcore::DynamicObjectClusterer clusterer(pnh);

  EXPECT_EQ(clusterer.params().min_points, 1U);
  EXPECT_DOUBLE_EQ(clusterer.params().max_extent, 0.0);
  EXPECT_EQ(clusterer.params().connectivity, 6);
}

TEST(DynamicObjectClustererParamsTest, constructorReadsParametersFromNodeHandle) {
  skipIfNoRosMaster();

  ros::NodeHandle pnh("dynamic_object_clusterer_params");
  pnh.setParam("dynamic_cluster_min_points", 4.0);
  pnh.setParam("dynamic_cluster_max_extent", 2.5);
  pnh.setParam("dynamic_cluster_connectivity", 18);

  const ldopcore::DynamicObjectClusterer clusterer(pnh);

  EXPECT_EQ(clusterer.params().min_points, 4U);
  EXPECT_DOUBLE_EQ(clusterer.params().max_extent, 2.5);
  EXPECT_EQ(clusterer.params().connectivity, 18);
}

TEST(DynamicObjectClustererTest, emptyInputProducesEmptyObjectArray) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), {});

  EXPECT_EQ(objects.header.frame_id, "map");
  EXPECT_TRUE(objects.objects.empty());
}

TEST(DynamicObjectClustererTest, sixConnectedKeysProduceSingleObject) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10),
      makeClusterPoint(0.2F, 0.0F, 0.0F, 11, 10, 10),
      makeClusterPoint(0.2F, 0.3F, 0.1F, 11, 10, 10)};

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), points);

  ASSERT_EQ(objects.objects.size(), 1U);
  EXPECT_EQ(objects.objects.front().id, 0U);
  EXPECT_NEAR(objects.objects.front().center.x, 0.1, 1e-6);
  EXPECT_NEAR(objects.objects.front().center.y, 0.15, 1e-6);
  EXPECT_NEAR(objects.objects.front().center.z, 0.05, 1e-6);
  EXPECT_NEAR(objects.objects.front().size.x, 0.2, 1e-6);
  EXPECT_NEAR(objects.objects.front().size.y, 0.3, 1e-6);
  EXPECT_NEAR(objects.objects.front().size.z, 0.1, 1e-6);
  EXPECT_TRUE(objects.objects.front().prediction_time_offsets.empty());
  EXPECT_TRUE(objects.objects.front().predicted_positions.empty());
}

TEST(DynamicObjectClustererTest, separatedKeysProduceFrameLocalIdsFromZero) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(5.0F, 0.0F, 0.0F, 50, 10, 10),
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10)};

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), points);

  ASSERT_EQ(objects.objects.size(), 2U);
  EXPECT_EQ(objects.objects[0].id, 0U);
  EXPECT_EQ(objects.objects[1].id, 1U);
  EXPECT_DOUBLE_EQ(objects.objects[0].center.x, 0.0);
  EXPECT_DOUBLE_EQ(objects.objects[1].center.x, 5.0);
}

TEST(DynamicObjectClustererTest, minPointsAndMaxExtentFilterClusters) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6, 2U, 0.5);
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10),
      makeClusterPoint(1.0F, 0.0F, 0.0F, 11, 10, 10),
      makeClusterPoint(5.0F, 0.0F, 0.0F, 50, 10, 10)};

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), points);

  EXPECT_TRUE(objects.objects.empty());
}

TEST(DynamicObjectClustererTest, connectivityControlsDiagonalNeighbors) {
  skipIfNoRosMaster();
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10),
      makeClusterPoint(0.2F, 0.2F, 0.0F, 11, 11, 10),
      makeClusterPoint(0.4F, 0.4F, 0.2F, 12, 12, 11)};

  EXPECT_EQ(makeClusterer(6).buildDynamicObjectArray(makeHeader(), points).objects.size(), 3U);
  EXPECT_EQ(makeClusterer(18).buildDynamicObjectArray(makeHeader(), points).objects.size(), 2U);
  EXPECT_EQ(makeClusterer(26).buildDynamicObjectArray(makeHeader(), points).objects.size(), 1U);
}

TEST(DynamicObjectClustererTimingTest, buildDynamicObjectArrayReportsElapsedMs) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10),
      makeClusterPoint(0.2F, 0.0F, 0.0F, 11, 10, 10)};
  double elapsed_ms = -1.0;

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), points, &elapsed_ms);

  ASSERT_EQ(objects.objects.size(), 1U);
  EXPECT_GE(elapsed_ms, 0.0);
}

TEST(DynamicObjectClustererVisualizationTest, emptyObjectsPublishDeleteAllMarker) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);

  const auto markers = clusterer.buildObjectMarkers(makeObjectArray());

  ASSERT_EQ(markers.markers.size(), 1U);
  EXPECT_EQ(markers.markers.front().header.frame_id, "map");
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::Marker::DELETEALL);
}

TEST(DynamicObjectClustererVisualizationTest, objectsProduceBoundingBoxLineMarkers) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);
  auto objects = makeObjectArray();

  ldop::DynamicObject object;
  object.id = 7U;
  object.center.x = 1.0;
  object.center.y = 2.0;
  object.center.z = 3.0;
  object.size.x = 4.0;
  object.size.y = 2.0;
  object.size.z = 6.0;
  objects.objects.push_back(object);

  const auto markers = clusterer.buildObjectMarkers(objects);

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::Marker::DELETEALL);

  const auto& box_marker = markers.markers.back();
  EXPECT_EQ(box_marker.header.frame_id, "map");
  EXPECT_EQ(box_marker.ns, "dynamic_object_bbox");
  EXPECT_EQ(box_marker.id, 7);
  EXPECT_EQ(box_marker.type, visualization_msgs::Marker::LINE_LIST);
  EXPECT_EQ(box_marker.action, visualization_msgs::Marker::ADD);
  EXPECT_GT(box_marker.scale.x, 0.0);
  ASSERT_EQ(box_marker.points.size(), 24U);

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto& point : box_marker.points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  EXPECT_DOUBLE_EQ(min_x, -1.0);
  EXPECT_DOUBLE_EQ(max_x, 3.0);
  EXPECT_DOUBLE_EQ(min_y, 1.0);
  EXPECT_DOUBLE_EQ(max_y, 3.0);
  EXPECT_DOUBLE_EQ(min_z, 0.0);
  EXPECT_DOUBLE_EQ(max_z, 6.0);
}

TEST(DynamicObjectClustererTimingTest, buildObjectMarkersReportsElapsedMs) {
  skipIfNoRosMaster();
  auto clusterer = makeClusterer(6);
  auto objects = makeObjectArray();
  double elapsed_ms = -1.0;

  ldop::DynamicObject object;
  object.id = 3U;
  object.center.x = 1.0;
  object.center.y = 2.0;
  object.center.z = 3.0;
  object.size.x = 2.0;
  object.size.y = 2.0;
  object.size.z = 2.0;
  objects.objects.push_back(object);

  const auto markers = clusterer.buildObjectMarkers(objects, &elapsed_ms);

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_GE(elapsed_ms, 0.0);
}
