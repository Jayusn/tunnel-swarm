# Dynamic Object Mocap Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the first mocap-assisted dynamic-object fusion layer: LiDAR detections provide existence, bbox, and low-rate center anchors; mocap poses provide high-rate fused pose observations and spline-derived virtual IMU samples.

**Architecture:** Keep ROS I/O in `Ldop`, following the existing package boundary, and add `DynamicObjectPoseFusion` as a stateful processing module. The module owns mocap source state, association, offset anchoring, and B-spline windows; it returns message/marker snapshots so callers can publish after locks are released.

**Tech Stack:** ROS1 Noetic, catkin, C++20, Eigen, gtest, existing `ldop_core` library and generated ROS messages.

---

## Execution Notes

- Before production edits, run `superpowers:using-git-worktrees`. Current checkout was observed as a normal checkout on branch `model_exploration`; creating an isolated worktree requires user consent.
- Do not create intermediate commits during task execution. The repository instruction says commits happen only after the task is complete and verified or when the user explicitly asks.
- Use TDD for production code. For each behavior task below, add or extend `test/test_dynamic_object_pose_fusion.cpp`, run the named test and confirm the expected failure, then implement the smallest code that passes.
- If subagents are used, give each worker a disjoint write set. If the user has not explicitly authorized subagents, execute inline with `superpowers:executing-plans`.

## File Structure

Create:

- `msg/FusedDynamicObject.msg`: one fused pose observation at one mocap knot time.
- `msg/FusedDynamicObjectArray.msg`: event batch of fused pose observations.
- `msg/FusedDynamicObjectVirtualImu.msg`: one virtual IMU / motion-intent sample at one spline evaluation time.
- `msg/FusedDynamicObjectVirtualImuArray.msg`: event batch of virtual IMU samples.
- `include/ldop/dynamic_object_pose_fusion.h`: public fusion module API, params/config structs, output snapshot structs.
- `src/dynamic_object_pose_fusion.cpp`: params loading, SE(3) composition, source state, association, spline generation, marker construction.
- `test/test_dynamic_object_pose_fusion.cpp`: focused unit tests for transform order, association, gating, window reset, virtual IMU timing, quaternion continuity, and marker semantics.

Modify:

- `CMakeLists.txt`: add new messages, add `src/dynamic_object_pose_fusion.cpp` to `ldop_core`, add `test_dynamic_object_pose_fusion`.
- `include/ldop/ldop.h`: include fused messages and fusion module; add mocap subscribers, fusion publishers, fusion fields in `ProcessedFrame`, and `mocapPoseCallback()`.
- `src/ldop.cpp`: construct fusion before ROS interface setup, subscribe mocap topics, advertise fusion topics, route cluster detections and mocap callbacks into fusion, and stop publishing tracker/predictor outputs in the first-version main flow.
- `config/ldop.yaml`: add fusion topics and parameter defaults from the design spec.
- `include/ldop/utils.h`: only add tiny cross-module conversion helpers if needed by both fusion and tests; keep fusion-specific math in `dynamic_object_pose_fusion.cpp`.

## Task 1: Add Fused Message Contracts

**Files:**

- Create: `msg/FusedDynamicObject.msg`
- Create: `msg/FusedDynamicObjectArray.msg`
- Create: `msg/FusedDynamicObjectVirtualImu.msg`
- Create: `msg/FusedDynamicObjectVirtualImuArray.msg`
- Modify: `CMakeLists.txt`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Write the failing message include test**

Add this initial test file:

```cpp
#include <gtest/gtest.h>

#include <ldop/FusedDynamicObjectArray.h>
#include <ldop/FusedDynamicObjectVirtualImuArray.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

TEST(DynamicObjectPoseFusionMessagesTest, FusedObjectCarriesPerObjectStampAndSourceIndex) {
  ldop::FusedDynamicObjectArray array;
  array.header.frame_id = "map";
  array.header.stamp = ros::Time(10.02);
  array.objects.resize(1U);
  array.objects.front().header.frame_id = "map";
  array.objects.front().header.stamp = ros::Time(10.0);
  array.objects.front().mocap_source_index = 2U;
  array.objects.front().lidar_updated_in_current_frame = true;

  EXPECT_EQ(array.objects.front().header.stamp, ros::Time(10.0));
  EXPECT_EQ(array.objects.front().mocap_source_index, 2U);
  EXPECT_TRUE(array.objects.front().lidar_updated_in_current_frame);
}

TEST(DynamicObjectPoseFusionMessagesTest, VirtualImuCarriesEvalAndAvailabilityTime) {
  ldop::FusedDynamicObjectVirtualImuArray array;
  array.header.frame_id = "map";
  array.samples.resize(1U);
  array.samples.front().header.stamp = ros::Time(11.0);
  array.samples.front().available_stamp = ros::Time(11.04);
  array.samples.front().mocap_source_index = 1U;
  array.samples.front().virtual_specific_force.z = 9.80665;

  EXPECT_EQ(array.samples.front().header.stamp, ros::Time(11.0));
  EXPECT_EQ(array.samples.front().available_stamp, ros::Time(11.04));
  EXPECT_EQ(array.samples.front().mocap_source_index, 1U);
  EXPECT_NEAR(array.samples.front().virtual_specific_force.z, 9.80665, 1e-9);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_dynamic_object_pose_fusion");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Run build and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
```

Expected: build fails because generated headers such as `ldop/FusedDynamicObjectArray.h` do not exist.

- [ ] **Step 3: Add message files**

Create `msg/FusedDynamicObject.msg`:

```text
std_msgs/Header header
uint32 mocap_source_index
geometry_msgs/Pose fused_pose
geometry_msgs/Vector3 bbox_size
uint32 point_count
geometry_msgs/Vector3 offset_body
float64 lidar_anchor_age
bool lidar_updated_in_current_frame
```

Create `msg/FusedDynamicObjectArray.msg`:

```text
std_msgs/Header header
FusedDynamicObject[] objects
```

Create `msg/FusedDynamicObjectVirtualImu.msg`:

```text
std_msgs/Header header
time available_stamp
uint32 mocap_source_index
geometry_msgs/Pose fused_pose_at_eval
geometry_msgs/Vector3 virtual_linear_velocity
geometry_msgs/Vector3 virtual_angular_velocity
geometry_msgs/Vector3 virtual_linear_acceleration
geometry_msgs/Vector3 virtual_specific_force
```

Create `msg/FusedDynamicObjectVirtualImuArray.msg`:

```text
std_msgs/Header header
FusedDynamicObjectVirtualImu[] samples
```

- [ ] **Step 4: Register messages and the test target**

In `CMakeLists.txt`, add the four new `.msg` files under `add_message_files(FILES ...)`.

Add this gtest block inside `if(CATKIN_ENABLE_TESTING)`:

```cmake
  catkin_add_gtest(test_dynamic_object_pose_fusion
    test/test_dynamic_object_pose_fusion.cpp
  )
  if(TARGET test_dynamic_object_pose_fusion)
    target_link_libraries(test_dynamic_object_pose_fusion
      ldop_core
      ${catkin_LIBRARIES}
      gtest
    )
  endif()
```

- [ ] **Step 5: Run GREEN verification**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: build succeeds and the two message contract tests pass.

## Task 2: Add Fusion Config and Core API

**Files:**

- Create: `include/ldop/dynamic_object_pose_fusion.h`
- Create: `src/dynamic_object_pose_fusion.cpp`
- Modify: `CMakeLists.txt`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Extend the test with config behavior**

Append:

```cpp
#include <ldop/dynamic_object_pose_fusion.h>

TEST(DynamicObjectPoseFusionConfigTest, BuildsRuntimeConfigFromBoundedParams) {
  DynamicObjectPoseFusionParams params;
  params.mocap_topics = {"/mocap/a", "/mocap/b"};
  params.mocap_to_ldop_translation = {1.0, 2.0, 3.0};
  params.offset_alpha = 1.5;
  params.bspline_min_samples = 3;
  params.bspline_future_samples = 10;

  const DynamicObjectPoseFusionConfig config = buildPoseFusionConfig(params);

  EXPECT_EQ(config.mocap_topics.size(), 2U);
  EXPECT_EQ(config.mocap_topics.front(), "/mocap/a");
  EXPECT_DOUBLE_EQ(config.mocap_to_ldop_translation.x(), 1.0);
  EXPECT_DOUBLE_EQ(config.mocap_to_ldop_translation.y(), 2.0);
  EXPECT_DOUBLE_EQ(config.mocap_to_ldop_translation.z(), 3.0);
  EXPECT_DOUBLE_EQ(config.offset_alpha, 1.0);
  EXPECT_EQ(config.bspline_min_samples, 5U);
  EXPECT_EQ(config.bspline_future_samples, 4U);
}
```

- [ ] **Step 2: Run test and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
```

Expected: build fails because `ldop/dynamic_object_pose_fusion.h` and `buildPoseFusionConfig()` do not exist.

- [ ] **Step 3: Add the public header**

Create `include/ldop/dynamic_object_pose_fusion.h` with these public contracts:

```cpp
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <ldop/FusedDynamicObjectArray.h>
#include <ldop/FusedDynamicObjectVirtualImuArray.h>
#include <ldop/utils.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

struct DynamicObjectPoseFusionParams {
  std::vector<std::string> mocap_topics{"/mocap/target_0/pose"};
  std::vector<double> mocap_to_ldop_translation{0.0, 0.0, 0.0};
  double mocap_sync_tolerance{0.03};
  double association_max_distance{1.0};
  double offset_alpha{0.2};
  double offset_observation_max_norm{2.0};
  double offset_update_max_delta{0.3};
  double lidar_anchor_timeout{0.5};
  int bspline_min_samples{8};
  int bspline_future_samples{2};
  double bspline_reset_translation_delta{0.2};
  double mocap_nominal_rate{50.0};
  double virtual_imu_rate_hz{200.0};
  double mocap_dt_jitter_tolerance{0.005};
  double gravity_z{-9.80665};
  std::string fused_objects_topic{"/ldop/fused_dynamic_objects"};
  std::string virtual_imu_topic{"/ldop/fused_dynamic_object_virtual_imu"};
  std::string marker_topic{"/ldop/fused_dynamic_object_markers"};
};

struct DynamicObjectPoseFusionConfig {
  std::vector<std::string> mocap_topics;
  Eigen::Vector3d mocap_to_ldop_translation{Eigen::Vector3d::Zero()};
  double mocap_sync_tolerance{0.03};
  double association_max_distance{1.0};
  double offset_alpha{0.2};
  double offset_observation_max_norm{2.0};
  double offset_update_max_delta{0.3};
  double lidar_anchor_timeout{0.5};
  std::size_t bspline_min_samples{8U};
  std::size_t bspline_future_samples{2U};
  double bspline_reset_translation_delta{0.2};
  double mocap_nominal_rate{50.0};
  double virtual_imu_rate_hz{200.0};
  double mocap_dt_jitter_tolerance{0.005};
  double gravity_z{-9.80665};
  std::string fused_objects_topic{"/ldop/fused_dynamic_objects"};
  std::string virtual_imu_topic{"/ldop/fused_dynamic_object_virtual_imu"};
  std::string marker_topic{"/ldop/fused_dynamic_object_markers"};
};

struct DynamicObjectPoseFusionSnapshot {
  std::optional<ldop::FusedDynamicObjectArray> fused_objects;
  std::optional<ldop::FusedDynamicObjectVirtualImuArray> virtual_imu;
  std::optional<visualization_msgs::MarkerArray> markers;
};

DynamicObjectPoseFusionConfig buildPoseFusionConfig(
    const DynamicObjectPoseFusionParams& params);

class DynamicObjectPoseFusion {
 public:
  explicit DynamicObjectPoseFusion(ros::NodeHandle& pnh, bool verbose = false);
  explicit DynamicObjectPoseFusion(const DynamicObjectPoseFusionConfig& config,
                                   bool verbose = false);

  const std::vector<std::string>& mocapTopics() const;
  const std::string& fusedObjectsTopic() const;
  const std::string& virtualImuTopic() const;
  const std::string& markerTopic() const;

  DynamicObjectPoseFusionSnapshot processMocapPose(
      std::size_t mocap_source_index,
      const geometry_msgs::PoseStamped& pose_msg);

  DynamicObjectPoseFusionSnapshot updateLidarAnchors(
      const std_msgs::Header& header,
      const std::vector<DynamicObjectDetection>& detections);

 private:
  void loadParameters(ros::NodeHandle& pnh);

  DynamicObjectPoseFusionParams params_;
  DynamicObjectPoseFusionConfig config_;
  bool verbose_{false};
};

}  // namespace ldopcore
```

- [ ] **Step 4: Add minimal implementation and CMake source registration**

Add `src/dynamic_object_pose_fusion.cpp` with `buildPoseFusionConfig()`, constructors, topic accessors, no-output stubs for the two processing methods, and `loadParameters()` using three-argument `pnh.param()` calls from a default `DynamicObjectPoseFusionParams` instance.

Register `src/dynamic_object_pose_fusion.cpp` in the `ldop_core` source list.

- [ ] **Step 5: Run GREEN verification**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
/home/st/ldop_ws/build/ldop/test_dynamic_object_pose_fusion --gtest_filter='DynamicObjectPoseFusionConfigTest.*'
```

Expected: config test passes. If direct gtest binary path differs, run `catkin run_tests ldop --no-status` and inspect `catkin_test_results`.

## Task 3: Implement SE(3) Composition and First Anchor

**Files:**

- Modify: `include/ldop/dynamic_object_pose_fusion.h`
- Modify: `src/dynamic_object_pose_fusion.cpp`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Add transform-order and first-anchor tests**

Append:

```cpp
geometry_msgs::PoseStamped makePoseStamped(const double stamp,
                                           const Eigen::Vector3d& position,
                                           const Eigen::Quaterniond& q) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "mocap";
  msg.header.stamp = ros::Time(stamp);
  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

DynamicObjectDetection makeDetection(const double stamp,
                                     const Eigen::Vector3d& center,
                                     const Eigen::Vector3d& size,
                                     const std::size_t point_count = 12U) {
  DynamicObjectDetection detection;
  detection.stamp = ros::Time(stamp);
  detection.bbox.center.x = center.x();
  detection.bbox.center.y = center.y();
  detection.bbox.center.z = center.z();
  detection.bbox.size.x = size.x();
  detection.bbox.size.y = size.y();
  detection.bbox.size.z = size.z();
  detection.point_count = point_count;
  return detection;
}

TEST(DynamicObjectPoseFusionTest, InitializesOffsetFromLidarCentroidInRigidFrame) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.mocap_to_ldop_translation = Eigen::Vector3d(1.0, 0.0, 0.0);
  DynamicObjectPoseFusion fusion(config);

  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(fusion.processMocapPose(0U, makePoseStamped(10.0, {2.0, 0.0, 0.0}, q))
                   .fused_objects.has_value());

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(10.0);
  const auto snapshot = fusion.updateLidarAnchors(
      header, {makeDetection(10.0, Eigen::Vector3d(3.0, 1.0, 0.0),
                             Eigen::Vector3d(1.0, 1.0, 1.0))});

  ASSERT_TRUE(snapshot.fused_objects.has_value());
  ASSERT_EQ(snapshot.fused_objects->objects.size(), 1U);
  const auto& object = snapshot.fused_objects->objects.front();
  EXPECT_EQ(object.header.stamp, ros::Time(10.0));
  EXPECT_EQ(object.mocap_source_index, 0U);
  EXPECT_NEAR(object.offset_body.x, 1.0, 1e-9);
  EXPECT_NEAR(object.offset_body.y, 0.0, 1e-9);
  EXPECT_NEAR(object.offset_body.z, 0.0, 1e-9);
  EXPECT_TRUE(object.lidar_updated_in_current_frame);
}

TEST(DynamicObjectPoseFusionTest, AppliesCurrentRigidRotationToStoredOffset) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  DynamicObjectPoseFusion fusion(config);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(1.0);
  fusion.processMocapPose(0U, makePoseStamped(1.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.updateLidarAnchors(
      header, {makeDetection(1.0, Eigen::Vector3d(1.0, 0.0, 0.0),
                             Eigen::Vector3d(1.0, 1.0, 1.0))});

  const Eigen::Quaterniond yaw90(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const auto snapshot =
      fusion.processMocapPose(0U, makePoseStamped(1.02, {0.0, 0.0, 0.0}, yaw90));

  ASSERT_TRUE(snapshot.fused_objects.has_value());
  const auto& pose = snapshot.fused_objects->objects.front().fused_pose;
  EXPECT_NEAR(pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(pose.position.z, 0.0, 1e-9);
}
```

- [ ] **Step 2: Run tests and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
/home/st/ldop_ws/build/ldop/test_dynamic_object_pose_fusion --gtest_filter='DynamicObjectPoseFusionTest.InitializesOffsetFromLidarCentroidInRigidFrame:DynamicObjectPoseFusionTest.AppliesCurrentRigidRotationToStoredOffset'
```

Expected: tests fail because the processing methods still return no output.

- [ ] **Step 3: Implement source state and pose composition**

Add private source state in the header:

```cpp
  struct MocapPoseSample {
    ros::Time stamp;
    Eigen::Vector3d position_ldop{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_ldop{Eigen::Quaterniond::Identity()};
  };

  struct SourceState {
    std::deque<MocapPoseSample> mocap_history;
    bool has_anchor{false};
    ros::Time anchor_stamp;
    MocapPoseSample anchor_mocap_pose_ldop;
    geometry_msgs::Pose anchor_fused_pose;
    Eigen::Vector3d offset_body{Eigen::Vector3d::Zero()};
    geometry_msgs::Vector3 latest_bbox_size;
    std::size_t latest_point_count{0U};
  };

  mutable std::mutex mutex_;
  std::vector<SourceState> sources_;
```

Include `<deque>` and `<mutex>`.

In the `.cpp`, implement:

- quaternion normalization and rejection for near-zero norm
- `pose_msg` conversion using `Eigen::Quaterniond(w, x, y, z)`
- `p_rigid_ldop = p_mocap + config_.mocap_to_ldop_translation`
- `p_fused = p_rigid_ldop + q_rigid_ldop * offset_body`
- first anchor offset as `q_rigid_ldop.inverse() * (p_lidar - p_rigid_ldop)`
- output message fields with object header stamp from the mocap sample and array header stamp from the latest object stamp

- [ ] **Step 4: Run GREEN verification**

Run the same gtest filter. Expected: both transform tests pass.

## Task 4: Implement Mocap History, Sync Tolerance, Association, and Offset Gates

**Files:**

- Modify: `src/dynamic_object_pose_fusion.cpp`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Add association and gating tests**

Append:

```cpp
TEST(DynamicObjectPoseFusionTest, AssociatesDetectionsWithGlobalMinimumDistance) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a", "/mocap/b"};
  config.association_max_distance = 2.0;
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(2.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.processMocapPose(1U, makePoseStamped(2.0, {10.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(2.0);
  const auto snapshot = fusion.updateLidarAnchors(
      header,
      {makeDetection(2.0, {10.0, 0.0, -0.5}, {1.0, 1.0, 1.0}),
       makeDetection(2.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  ASSERT_TRUE(snapshot.fused_objects.has_value());
  ASSERT_EQ(snapshot.fused_objects->objects.size(), 2U);
  EXPECT_EQ(snapshot.fused_objects->objects[0].mocap_source_index, 0U);
  EXPECT_EQ(snapshot.fused_objects->objects[1].mocap_source_index, 1U);
}

TEST(DynamicObjectPoseFusionTest, RejectsOffsetObservationJumpAndKeepsPreviousOffset) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.offset_update_max_delta = 0.2;
  config.offset_observation_max_norm = 5.0;
  config.association_max_distance = 10.0;
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(3.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(3.0);
  fusion.updateLidarAnchors(
      header, {makeDetection(3.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  fusion.processMocapPose(0U, makePoseStamped(3.1, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  header.stamp = ros::Time(3.1);
  const auto snapshot = fusion.updateLidarAnchors(
      header, {makeDetection(3.1, {3.0, 0.0, 0.0}, {1.0, 1.0, 1.0})});

  EXPECT_FALSE(snapshot.fused_objects.has_value());
  const auto propagated =
      fusion.processMocapPose(0U, makePoseStamped(3.12, {0.0, 0.0, 0.0},
                                                  Eigen::Quaterniond::Identity()));
  ASSERT_TRUE(propagated.fused_objects.has_value());
  EXPECT_NEAR(propagated.fused_objects->objects.front().offset_body.x, 0.0, 1e-9);
  EXPECT_NEAR(propagated.fused_objects->objects.front().offset_body.z, -0.5, 1e-9);
}
```

- [ ] **Step 2: Run tests and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
/home/st/ldop_ws/build/ldop/test_dynamic_object_pose_fusion --gtest_filter='DynamicObjectPoseFusionTest.AssociatesDetectionsWithGlobalMinimumDistance:DynamicObjectPoseFusionTest.RejectsOffsetObservationJumpAndKeepsPreviousOffset'
```

Expected: association/gating tests fail until matching and gates exist.

- [ ] **Step 3: Implement matching and gates**

Implement in this order:

1. Maintain each source `mocap_history` as a short deque of valid, strictly increasing samples.
2. For a LiDAR header stamp, find the nearest source sample whose absolute time delta is at most `mocap_sync_tolerance`.
3. Build all candidate pairs with `cost = ||p_lidar - (p_rigid + R_rigid * offset)||`.
4. For unanchored sources, use candidate detection prior `[0, 0, -0.5 * bbox_size.z]`.
5. Use bitmask DP or bounded recursive search to minimize total accepted distance for the small source/detection count.
6. Reject matches above `association_max_distance`.
7. Reject offset observations whose norm exceeds `offset_observation_max_norm`.
8. For anchored sources, reject offset jumps above `offset_update_max_delta`.
9. Apply `offset_body = (1 - alpha) * offset_body + alpha * observed` only after all gates pass.

- [ ] **Step 4: Run GREEN verification**

Run the same gtest filter. Expected: both tests pass.

## Task 5: Implement Anchor Timeout, Quaternion Continuity, and Window Reset Signals

**Files:**

- Modify: `include/ldop/dynamic_object_pose_fusion.h`
- Modify: `src/dynamic_object_pose_fusion.cpp`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Add lifecycle tests**

Append:

```cpp
TEST(DynamicObjectPoseFusionTest, StopsPublishingAfterLidarAnchorTimeout) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.lidar_anchor_timeout = 0.1;
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(4.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(4.0);
  fusion.updateLidarAnchors(
      header, {makeDetection(4.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  EXPECT_TRUE(fusion.processMocapPose(0U, makePoseStamped(4.05, {0.0, 0.0, 0.0},
                                                          Eigen::Quaterniond::Identity()))
                  .fused_objects.has_value());
  EXPECT_FALSE(fusion.processMocapPose(0U, makePoseStamped(4.2, {0.0, 0.0, 0.0},
                                                          Eigen::Quaterniond::Identity()))
                   .fused_objects.has_value());
}

TEST(DynamicObjectPoseFusionTest, RejectsRepeatedMocapStamp) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(5.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(5.0);
  fusion.updateLidarAnchors(
      header, {makeDetection(5.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  const auto repeated =
      fusion.processMocapPose(0U, makePoseStamped(5.0, {1.0, 0.0, 0.0},
                                                  Eigen::Quaterniond::Identity()));

  EXPECT_FALSE(repeated.fused_objects.has_value());
}
```

- [ ] **Step 2: Run tests and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
/home/st/ldop_ws/build/ldop/test_dynamic_object_pose_fusion --gtest_filter='DynamicObjectPoseFusionTest.StopsPublishingAfterLidarAnchorTimeout:DynamicObjectPoseFusionTest.RejectsRepeatedMocapStamp'
```

Expected: tests fail if timeout and repeated-stamp rejection are not implemented.

- [ ] **Step 3: Implement lifecycle rules**

Add state fields needed by spline work:

```cpp
    std::deque<MocapPoseSample> fused_pose_knots;
    std::deque<MocapPoseSample> pose_grid_knots;
    std::optional<ros::Time> last_published_virtual_imu_eval_stamp;
    std::optional<Eigen::Quaterniond> last_valid_quaternion;
```

Implement:

- repeated or backward mocap timestamps are rejected before state changes
- invalid quaternion samples are rejected
- quaternion sign continuity uses `if (last.dot(current) < 0) current.coeffs() *= -1`
- anchor timeout suppresses fused output without deleting the anchor immediately
- first anchor and large offset-induced fused-pose jumps clear spline windows
- normal small EMA offset updates preserve spline windows

- [ ] **Step 4: Run GREEN verification**

Run the same gtest filter and then the whole `test_dynamic_object_pose_fusion` binary. Expected: all current fusion tests pass.

## Task 6: Implement Uniform Pose Grid and Virtual IMU Spline Output

**Files:**

- Modify: `src/dynamic_object_pose_fusion.cpp`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Add virtual IMU tests**

Append:

```cpp
TEST(DynamicObjectPoseFusionTest, GeneratesVirtualImuForConstantVelocityTrajectory) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.mocap_nominal_rate = 50.0;
  config.virtual_imu_rate_hz = 200.0;
  config.bspline_min_samples = 8U;
  config.bspline_future_samples = 2U;
  config.gravity_z = -9.80665;
  DynamicObjectPoseFusion fusion(config);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(6.0);
  fusion.processMocapPose(0U, makePoseStamped(6.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.updateLidarAnchors(
      header, {makeDetection(6.0, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0})});

  DynamicObjectPoseFusionSnapshot last_snapshot;
  for (int index = 1; index <= 12; ++index) {
    const double t = 6.0 + 0.02 * static_cast<double>(index);
    last_snapshot = fusion.processMocapPose(
        0U, makePoseStamped(t, {t - 6.0, 0.0, 0.0}, Eigen::Quaterniond::Identity()));
  }

  ASSERT_TRUE(last_snapshot.virtual_imu.has_value());
  ASSERT_FALSE(last_snapshot.virtual_imu->samples.empty());
  for (const auto& sample : last_snapshot.virtual_imu->samples) {
    EXPECT_EQ(sample.mocap_source_index, 0U);
    EXPECT_NEAR(sample.virtual_linear_velocity.x, 1.0, 1e-6);
    EXPECT_NEAR(sample.virtual_linear_velocity.y, 0.0, 1e-6);
    EXPECT_NEAR(sample.virtual_linear_acceleration.norm(), 0.0, 1e-6);
    EXPECT_NEAR(sample.virtual_specific_force.z, 9.80665, 1e-6);
    EXPECT_EQ(sample.available_stamp, ros::Time(6.24));
  }
}

TEST(DynamicObjectPoseFusionTest, DoesNotDuplicateVirtualImuEvalStampsAcrossCallbacks) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.mocap_nominal_rate = 50.0;
  config.virtual_imu_rate_hz = 200.0;
  config.bspline_min_samples = 8U;
  config.bspline_future_samples = 2U;
  DynamicObjectPoseFusion fusion(config);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(7.0);
  fusion.processMocapPose(0U, makePoseStamped(7.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.updateLidarAnchors(
      header, {makeDetection(7.0, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0})});

  std::vector<ros::Time> stamps;
  for (int index = 1; index <= 14; ++index) {
    const double t = 7.0 + 0.02 * static_cast<double>(index);
    const auto snapshot = fusion.processMocapPose(
        0U, makePoseStamped(t, {0.0, t - 7.0, 0.0}, Eigen::Quaterniond::Identity()));
    if (snapshot.virtual_imu.has_value()) {
      for (const auto& sample : snapshot.virtual_imu->samples) {
        stamps.push_back(sample.header.stamp);
      }
    }
  }

  ASSERT_GT(stamps.size(), 4U);
  for (std::size_t i = 1U; i < stamps.size(); ++i) {
    EXPECT_LT(stamps[i - 1U], stamps[i]);
  }
}
```

- [ ] **Step 2: Run tests and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
/home/st/ldop_ws/build/ldop/test_dynamic_object_pose_fusion --gtest_filter='DynamicObjectPoseFusionTest.GeneratesVirtualImuForConstantVelocityTrajectory:DynamicObjectPoseFusionTest.DoesNotDuplicateVirtualImuEvalStampsAcrossCallbacks'
```

Expected: tests fail because no virtual IMU samples are generated.

- [ ] **Step 3: Implement spline pipeline**

Implement the spline chain in `src/dynamic_object_pose_fusion.cpp`:

1. Append valid fused pose knots after each published fused pose.
2. Clear spline windows when mocap dt differs from nominal dt by more than `mocap_dt_jitter_tolerance`.
3. Resample raw fused pose knots to a uniform pose grid at `1 / mocap_nominal_rate` using linear position interpolation and quaternion SLERP.
4. Keep quaternion signs continuous before SLERP and before SO(3) log operations.
5. Build interpolating cubic position control points with endpoint controls fixed and internal equation `(C[i - 1] + 4 C[i] + C[i + 1]) / 6 = p[i]`.
6. Evaluate position, velocity, and acceleration with cubic B-spline basis derivatives and `1/h`, `1/h^2` scaling.
7. Evaluate orientation and body angular velocity with the SO(3) cumulative spline formula from the design spec.
8. Publish only eval stamps in `(last_published_virtual_imu_eval_stamp, latest_confirmed_eval_stamp]` on the `1 / virtual_imu_rate_hz` grid.
9. Set each sample `header.stamp` to `t_eval` and `available_stamp` to the newest raw fused pose knot stamp that triggered the batch.
10. Compute `virtual_specific_force = R_eval.transpose() * (p_ddot - Eigen::Vector3d(0, 0, gravity_z))`.

- [ ] **Step 4: Run GREEN verification**

Run the same gtest filter. Expected: both virtual IMU tests pass.

## Task 7: Implement Fusion Markers

**Files:**

- Modify: `src/dynamic_object_pose_fusion.cpp`
- Test: `test/test_dynamic_object_pose_fusion.cpp`

- [ ] **Step 1: Add marker tests**

Append:

```cpp
TEST(DynamicObjectPoseFusionTest, BuildsBoxMarkerAtFusedPoseAndOffsetLineOnlyWhenVerbose) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  DynamicObjectPoseFusion quiet_fusion(config, false);
  DynamicObjectPoseFusion verbose_fusion(config, true);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(8.0);

  quiet_fusion.processMocapPose(0U, makePoseStamped(8.0, {0.0, 0.0, 0.0},
                                                    Eigen::Quaterniond::Identity()));
  const auto quiet_snapshot = quiet_fusion.updateLidarAnchors(
      header, {makeDetection(8.0, {0.0, 0.0, -0.5}, {1.0, 2.0, 3.0})});
  ASSERT_TRUE(quiet_snapshot.markers.has_value());
  EXPECT_EQ(quiet_snapshot.markers->markers.size(), 2U);

  verbose_fusion.processMocapPose(0U, makePoseStamped(8.0, {0.0, 0.0, 0.0},
                                                      Eigen::Quaterniond::Identity()));
  const auto verbose_snapshot = verbose_fusion.updateLidarAnchors(
      header, {makeDetection(8.0, {0.0, 0.0, -0.5}, {1.0, 2.0, 3.0})});
  ASSERT_TRUE(verbose_snapshot.markers.has_value());
  EXPECT_EQ(verbose_snapshot.markers->markers.size(), 3U);
}
```

- [ ] **Step 2: Run tests and confirm RED**

Run the marker gtest filter. Expected: test fails until marker snapshots exist.

- [ ] **Step 3: Implement marker generation**

Implement:

- marker arrays begin with `makeDeleteAllMarker(header, "fused_dynamic_objects")`
- one CUBE marker per fused object, with pose from `fused_pose` and scale from `bbox_size`
- a stable namespace such as `"fused_dynamic_object_box"` for boxes
- when `verbose_` is true, add one LINE_LIST marker per source from rigid mocap origin to fused pose position
- no offset line when `verbose_` is false

- [ ] **Step 4: Run GREEN verification**

Run marker filter and then all fusion tests. Expected: all fusion tests pass.

## Task 8: Wire Fusion into Ldop and Pause Tracker/Predictor Main Flow

**Files:**

- Modify: `include/ldop/ldop.h`
- Modify: `src/ldop.cpp`
- Modify: `config/ldop.yaml`
- Test: build and existing tests

- [ ] **Step 1: Add compile-facing integration fields**

In `include/ldop/ldop.h`:

- include `geometry_msgs/PoseStamped.h`
- include `ldop/FusedDynamicObjectArray.h`
- include `ldop/FusedDynamicObjectVirtualImuArray.h`
- include `ldop/dynamic_object_pose_fusion.h`
- add `std::vector<ros::Subscriber> mocap_pose_subs_`
- add publishers `fused_objects_pub_`, `fused_virtual_imu_pub_`, `fused_markers_pub_`
- add `std::unique_ptr<DynamicObjectPoseFusion> dynamic_object_pose_fusion_`
- add `void mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg, std::size_t source_index);`
- add optional fusion snapshot fields to `ProcessedFrame`

- [ ] **Step 2: Run build and confirm RED**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
```

Expected: build fails until constructor, setup, callback, and publish code are added.

- [ ] **Step 3: Update `Ldop` constructor and ROS setup**

Change constructor order so fusion exists before `setupRosInterfaces()` needs its topics:

```cpp
loadParameters();
ufomap_mapper_ = std::make_unique<UfomapMapper>(nh_, pnh_, verbose_);
dynamic_object_clusterer_ = std::make_unique<DynamicObjectClusterer>(pnh_, verbose_);
dynamic_object_tracker_ = std::make_unique<DynamicObjectTracker>(pnh_, verbose_);
dynamic_object_predictor_ = std::make_unique<DynamicObjectPredictor>(pnh_, verbose_);
dynamic_object_pose_fusion_ = std::make_unique<DynamicObjectPoseFusion>(pnh_, verbose_);
setupRosInterfaces();
```

This keeps tracker and predictor initialized, but first-version processing will not call them.

In `setupRosInterfaces()`:

- subscribe each `dynamic_object_pose_fusion_->mocapTopics()[i]` with queue size `200`
- bind `mocapPoseCallback` with `source_index`
- advertise fused object, virtual IMU, and fused marker topics from fusion accessors

- [ ] **Step 4: Update point-cloud frame processing**

In `processFrame()`:

1. Keep mapper and clusterer exactly as today.
2. Store cluster timing in `cluster_module_ms`.
3. Call `dynamic_object_pose_fusion_->updateLidarAnchors(processed.dynamic_cloud_msg.header, cluster_result.detections)`.
4. Move returned fused objects, virtual IMU, and markers into `ProcessedFrame`.
5. Do not call `dynamic_object_tracker_->processDynamicTracks()`.
6. Do not call `dynamic_object_predictor_->predict()`.
7. Set `tracking_module_ms` and `prediction_module_ms` to `0.0` for timing summary clarity.

- [ ] **Step 5: Add mocap callback publishing**

Implement:

```cpp
void Ldop::mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                             const std::size_t source_index) {
  if (pose_msg == nullptr || dynamic_object_pose_fusion_ == nullptr) {
    return;
  }
  DynamicObjectPoseFusionSnapshot snapshot =
      dynamic_object_pose_fusion_->processMocapPose(source_index, *pose_msg);
  if (snapshot.fused_objects.has_value()) {
    fused_objects_pub_.publish(*snapshot.fused_objects);
  }
  if (snapshot.virtual_imu.has_value()) {
    fused_virtual_imu_pub_.publish(*snapshot.virtual_imu);
  }
  if (snapshot.markers.has_value()) {
    fused_markers_pub_.publish(*snapshot.markers);
  }
}
```

- [ ] **Step 6: Update `publishFrame()`**

Keep static/dynamic cloud and static map marker publishing. Publish fusion optional outputs when present. Do not publish old `dynamic_objects_pub_`, `dynamic_track_markers_pub_`, `dynamic_predictions_pub_`, or `dynamic_prediction_markers_pub_` in this first version.

- [ ] **Step 7: Add config defaults**

Append to `config/ldop.yaml`:

```yaml
fusion_mocap_topics:
  - /mocap/target_0/pose
fusion_mocap_to_ldop_translation: [0.0, 0.0, 0.0]
fusion_mocap_sync_tolerance: 0.03
fusion_association_max_distance: 1.0
fusion_offset_alpha: 0.2
fusion_offset_observation_max_norm: 2.0
fusion_offset_update_max_delta: 0.3
fusion_lidar_anchor_timeout: 0.5
fusion_bspline_min_samples: 8
fusion_bspline_future_samples: 2
fusion_bspline_reset_translation_delta: 0.2
fusion_mocap_nominal_rate: 50.0
fusion_virtual_imu_rate_hz: 200.0
fusion_mocap_dt_jitter_tolerance: 0.005
fusion_gravity_z: -9.80665
fusion_objects_topic: /ldop/fused_dynamic_objects
fusion_virtual_imu_topic: /ldop/fused_dynamic_object_virtual_imu
fusion_marker_topic: /ldop/fused_dynamic_object_markers
```

- [ ] **Step 8: Run GREEN verification**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: build and tests pass.

## Task 9: Final Verification and Manual Rosbag Checks

**Files:**

- No required source edits unless verification exposes failures.

- [ ] **Step 1: Run full build and tests**

Run:

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all package tests pass.

- [ ] **Step 2: Check generated message availability**

Run:

```bash
cd /home/st/ldop_ws
source devel/setup.bash
rosmsg show ldop/FusedDynamicObject
rosmsg show ldop/FusedDynamicObjectVirtualImu
```

Expected: fields match the four message files exactly.

- [ ] **Step 3: Run optional offline integration check when a rosbag is available**

Run with the dataset-specific config:

```bash
cd /home/st/ldop_ws
source devel/setup.bash
roslaunch ldop run_ldop.launch config_file:=/home/st/ldop_ws/src/LDOP/config/ldop.yaml
```

In another shell, play the bag and inspect:

```bash
rostopic hz /ldop/fused_dynamic_objects
rostopic hz /ldop/fused_dynamic_object_virtual_imu
rostopic echo -n 1 /ldop/fused_dynamic_objects
rostopic echo -n 1 /ldop/fused_dynamic_object_virtual_imu
```

Expected:

- `/ldop/fused_dynamic_objects` follows mocap knot frequency after a LiDAR anchor exists.
- `/ldop/fused_dynamic_object_virtual_imu` publishes only after the spline window has enough samples.
- each fused object uses its own `object.header.stamp` as the physical observation time.
- each virtual IMU sample uses `sample.header.stamp` as `t_eval` and `available_stamp` as the newest raw fused pose knot used to generate it.
- old dynamic object, track marker, and prediction topics are no longer published by `publishFrame()` in this first-version flow.

## Self-Review

Spec coverage:

- Mocap subscriptions are covered by Task 8.
- New fused object and virtual IMU messages are covered by Task 1.
- `T_LM * T_MR * T_RB` ordering and current-rotation offset propagation are covered by Task 3.
- LiDAR association, offset EMA, and gates are covered by Task 4.
- anchor timeout, bad mocap timestamps, quaternion validity, and reset behavior are covered by Task 5.
- uniform pose grid, cubic B-spline derivatives, SO(3) angular velocity, specific force, fixed delay, and duplicate suppression are covered by Task 6.
- fused box and verbose offset line markers are covered by Task 7.
- tracker/predictor pause and fusion publication boundary are covered by Task 8.
- build, test, message, and optional rosbag checks are covered by Task 9.

Placeholder scan:

- No placeholder markers or unspecified implementation slots are intentionally left in this plan.
- The only optional part is the rosbag check, because the plan cannot assume a dataset path.

Type consistency:

- Public API names are consistent: `DynamicObjectPoseFusionParams`, `DynamicObjectPoseFusionConfig`, `DynamicObjectPoseFusionSnapshot`, `buildPoseFusionConfig()`, `processMocapPose()`, and `updateLidarAnchors()`.
- ROS message names match the files registered in `CMakeLists.txt`.
- `Ldop` integration uses optional snapshot fields so empty event batches are not forced onto topics.
