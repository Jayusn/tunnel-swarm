# 动态目标聚类实施计划

> **给执行 agent 的要求：** 必须使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans` 按任务逐步实施。本计划使用复选框 `- [ ]` 跟踪进度。

**目标：** 按 `docs/superpowers/specs/2026-04-24-dynamic-object-clustering-design.md` 中的方案 B 实现当前帧动态点聚类：复用 `UfomapMapper` 分类阶段输出的 UFOMap voxel/key 信息，在 `/ldop/dynamic_objects` 中为每个有效 cluster 输出一个 `ldop/DynamicObject`。

**状态：** 已完成。该计划是第二阶段聚类实现时的执行草案；当前聚类输出已经演进为 tracker 的内部 detection 输入，第二阶段整体已在 2026-05-14 标记完成。

**架构：** `UfomapMapper` 继续负责 UFOMap 积分、`seenFree(point)` 动静态分类和静态地图可视化快照。分类时同步输出当前帧动态点的 `{原始点, UFOMap-derived voxel_code}`，但不在 mapper 内做 BFS 聚类。新增 `DynamicObjectClusterer` 消费这些结构化动态点，在 UFOMap key 空间做 `6/18/26` 邻域连通、按点数和 AABB 尺寸过滤 cluster，并生成 `DynamicObjectArray`。`Ldop` 只负责把 mapper 结果交给 clusterer，再发布结果，不改变 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud` 的当前帧点云语义。

**技术栈：** ROS1 Noetic、catkin、C++20、UFOMap、gtest、`ldop/DynamicObjectArray`。

---

## 范围约束

- 本计划只实现方案 B，不实现从 `/ldop/dynamic_cloud` 反解析点云再建临时 voxel hash 的方案 A。
- `/ldop/static_cloud` 仍表示当前帧静态点云。
- `/ldop/dynamic_cloud` 仍表示当前帧动态点云。
- `UfomapMapper::processInputCloud()` 的阶段顺序保持为：先基于更新前地图完成当前帧动静态分类和静态可视化快照，再积分当前帧。
- `UfomapMapper` 只输出动态点的聚类输入数据，不执行 BFS、不生成 `DynamicObject`。
- 静态地图可视化快照直接使用 `visualization_msgs::MarkerArray`，不再额外包一层薄结构体。
- 按 `AGENTS.md` 提交约定，不为中间计划或过程产物提交；仅在完整实现并验证通过后，或用户明确要求时提交。

## 文件结构

- 修改 `include/ldop/ufomap_mapper.h`：保留/补齐 `UfomapVoxelCode`、`UfomapDynamicClusterPoint`、`UfomapClassificationResult::dynamic_cluster_points`，并让 `processInputCloud()` 可接收聚类 depth。
- 修改 `src/ufomap_mapper.cpp`：动态点分类时用 `runtime.map.toCodeChecked(point, dynamic_cluster_depth)` 填充 `dynamic_cluster_points`。
- 新增 `include/ldop/dynamic_object_clusterer.h`：定义聚类参数、合法化结果和 `DynamicObjectClusterer` API。
- 新增 `src/dynamic_object_clusterer.cpp`：实现参数合法化、邻域生成、key-space BFS、AABB 计算和 `DynamicObjectArray` 构建。
- 修改 `include/ldop/ldop.h`：让 `Ldop` 持有 `DynamicObjectClusterer`，并在 `ProcessedFrame` 中保存 `dynamic_cluster_points`。
- 修改 `src/ldop.cpp`：读取聚类参数，构造 clusterer，把 cluster depth 传给 mapper，并替换旧的单框 `buildDynamicObjectArray()` 逻辑。
- 修改 `config/ldop.yaml`：增加聚类参数。
- 修改 `CMakeLists.txt`：把 `src/dynamic_object_clusterer.cpp` 加入 `ldop_core`，并增加 `test_dynamic_object_clusterer`。
- 修改 `test/test_ufomap_mapper.cpp`：保留动态点 voxel code 回归测试，并补充聚类 depth 测试。
- 新增 `test/test_dynamic_object_clusterer.cpp`：覆盖参数合法化、空输入、连通聚类、多 cluster、过滤、`6/18/26` 连通差异和 frame-local id。

## 任务 1：补齐 mapper 动态点 voxel code 输出

**文件：**
- 修改 `include/ldop/ufomap_mapper.h`
- 修改 `src/ufomap_mapper.cpp`
- 修改 `test/test_ufomap_mapper.cpp`

- [ ] **步骤 1：先写 mapper 元数据测试**

在 `test/test_ufomap_mapper.cpp` 的 `UfomapMapperClassificationTest` 下保留或新增测试：

```cpp
TEST(UfomapMapperClassificationTest, exposesUfomapVoxelCodesForDynamicPoints) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_dynamic_cluster_points");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  ASSERT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result = mapper.processInputCloud(mixed_cloud, odom);

  ASSERT_EQ(result.classification.dynamic_point_count, 1U);
  ASSERT_EQ(result.classification.dynamic_cluster_points.size(), 1U);

  const auto& dynamic_cluster_point = result.classification.dynamic_cluster_points.front();
  EXPECT_FLOAT_EQ(dynamic_cluster_point.point.x, 1.0F);
  EXPECT_FLOAT_EQ(dynamic_cluster_point.point.y, 0.0F);
  EXPECT_FLOAT_EQ(dynamic_cluster_point.point.z, 0.0F);

  using TestMap =
      ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;
  const auto params = mapper.effectiveParams();
  TestMap map(static_cast<ufo::node_size_t>(static_cast<float>(params.resolution)),
              static_cast<ufo::depth_t>(params.depth_levels));
  const auto expected_code = map.toCodeChecked(makePoint(1.0F, 0.0F, 0.0F));
  ASSERT_TRUE(expected_code.has_value());
  const ufo::Key expected_key = *expected_code;

  EXPECT_EQ(dynamic_cluster_point.voxel_code.raw_code, expected_code->raw());
  EXPECT_EQ(dynamic_cluster_point.voxel_code.key_x, expected_key.x());
  EXPECT_EQ(dynamic_cluster_point.voxel_code.key_y, expected_key.y());
  EXPECT_EQ(dynamic_cluster_point.voxel_code.key_z, expected_key.z());
  EXPECT_EQ(dynamic_cluster_point.voxel_code.depth, expected_key.depth());
}
```

再新增一个测试，证明 mapper 能按指定聚类 depth 输出 key：

```cpp
TEST(UfomapMapperClassificationTest, usesRequestedClusterDepthForDynamicVoxelCodes) {
  ensureRosInitialized();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master is not available";
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("ufomap_dynamic_cluster_depth");
  ldopcore::UfomapMapper mapper(nh, pnh, "map", false);

  const auto odom = makeOdom("map");
  (void)mapper.processInputCloud(makeSinglePointCloud("map", 2.0F, 0.0F, 0.0F), odom);
  ASSERT_TRUE(mapper.seenFree(makePoint(1.0F, 0.0F, 0.0F)));

  const auto mixed_cloud =
      makePointCloud("map", {{{1.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}}});
  const auto result =
      mapper.processInputCloud(mixed_cloud, odom, static_cast<ufo::depth_t>(2));

  ASSERT_EQ(result.classification.dynamic_cluster_points.size(), 1U);
  const auto& code = result.classification.dynamic_cluster_points.front().voxel_code;
  EXPECT_EQ(code.depth, static_cast<ufo::depth_t>(2));
  EXPECT_EQ(code.key_x % (ufo::key_t{1} << code.depth), 0U);
  EXPECT_EQ(code.key_y % (ufo::key_t{1} << code.depth), 0U);
  EXPECT_EQ(code.key_z % (ufo::key_t{1} << code.depth), 0U);
}
```

- [ ] **步骤 2：运行 mapper 测试确认 RED**

运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_ufomap_mapper -j4
```

预期：如果接口还没补齐，应出现编译失败，例如 `processInputCloud` 不接受第三个 `ufo::depth_t` 参数，或 `dynamic_cluster_points` 不存在。

- [ ] **步骤 3：补齐 mapper 接口和填充逻辑**

在 `include/ldop/ufomap_mapper.h` 中确认有以下结构体：

```cpp
struct UfomapVoxelCode {
  // raw_code 保留 UFOMap Code::raw()，可用于调试或后续直接复用节点编码。
  ufo::code_t raw_code{0};
  // key_* / depth 用于后续按 UFOMap key 空间生成 6/18/26 邻域。
  ufo::key_t key_x{0};
  ufo::key_t key_y{0};
  ufo::key_t key_z{0};
  ufo::depth_t depth{0};
};

struct UfomapDynamicClusterPoint {
  // 原始动态点用于聚类后计算 AABB，避免体素中心放大目标框。
  ufo::Point point{};
  UfomapVoxelCode voxel_code;
};
```

在 `UfomapClassificationResult` 中确认有：

```cpp
  // 当前帧动态点的结构化聚类输入；不改变 dynamic_cloud_msg 的 ROS 输出语义。
  std::vector<UfomapDynamicClusterPoint> dynamic_cluster_points;
```

把公开接口改为：

```cpp
  UfomapFrameResult processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                      const nav_msgs::Odometry& odom_msg,
                                      ufo::depth_t dynamic_cluster_depth = 0);
```

把私有分类函数改为：

```cpp
  UfomapClassificationResult classifyPoints(const std_msgs::Header& header,
                                            const UfomapPointCloud& points,
                                            ufo::depth_t dynamic_cluster_depth) const;
```

在 `src/ufomap_mapper.cpp` 中包含：

```cpp
#include <ufo/map/key.hpp>
```

在 `processInputCloud()` 中，把当前 `classifyPoints` 调用改为：

```cpp
  result.classification =
      classifyPoints(cloud_msg.header, *output_points, dynamic_cluster_depth);
```

在 `classifyPoints()` 的动态点分支中填充 voxel code：

```cpp
      if (runtime.map.seenFree(point)) {
        dynamic_points.push_back(point);
        const auto code = runtime.map.toCodeChecked(point, dynamic_cluster_depth);
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
```

- [ ] **步骤 4：运行 mapper 测试确认 GREEN**

运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_ufomap_mapper -j4
/home/st/ldop_ws/devel/.private/ldop/lib/ldop/test_ufomap_mapper \
  --gtest_filter=UfomapMapperClassificationTest.exposesUfomapVoxelCodesForDynamicPoints:UfomapMapperClassificationTest.usesRequestedClusterDepthForDynamicVoxelCodes
```

预期：两个测试都通过。

## 任务 2：新增 clusterer 参数与 depth 合法化

**文件：**
- 新建 `include/ldop/dynamic_object_clusterer.h`
- 新建 `src/dynamic_object_clusterer.cpp`
- 新建 `test/test_dynamic_object_clusterer.cpp`
- 修改 `CMakeLists.txt`

- [ ] **步骤 1：先写参数合法化测试**

创建 `test/test_dynamic_object_clusterer.cpp`：

```cpp
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <ldop/dynamic_object_clusterer.h>

TEST(DynamicObjectClustererParamsTest, defaultVoxelSizeUsesUfomapLeafDepth) {
  ldopcore::UfomapMapperParams ufomap_params;
  ufomap_params.resolution = 0.2;
  ufomap_params.depth_levels = 16;

  ldopcore::DynamicObjectClustererParams raw_params;
  raw_params.voxel_size = 0.0;

  const auto config =
      ldopcore::sanitizeDynamicObjectClustererParams(raw_params, ufomap_params);

  EXPECT_EQ(config.cluster_depth, static_cast<ufo::depth_t>(0));
  EXPECT_DOUBLE_EQ(config.effective_voxel_size, 0.2);
  EXPECT_EQ(config.key_step, static_cast<ufo::key_t>(1));
}

TEST(DynamicObjectClustererParamsTest, positiveVoxelSizeChoosesNearestUfomapDepth) {
  ldopcore::UfomapMapperParams ufomap_params;
  ufomap_params.resolution = 0.2;
  ufomap_params.depth_levels = 16;

  ldopcore::DynamicObjectClustererParams raw_params;
  raw_params.voxel_size = 0.79;

  const auto config =
      ldopcore::sanitizeDynamicObjectClustererParams(raw_params, ufomap_params);

  EXPECT_EQ(config.cluster_depth, static_cast<ufo::depth_t>(2));
  EXPECT_DOUBLE_EQ(config.effective_voxel_size, 0.8);
  EXPECT_EQ(config.key_step, static_cast<ufo::key_t>(4));
}

TEST(DynamicObjectClustererParamsTest, invalidValuesFallBackToSafeDefaults) {
  ldopcore::UfomapMapperParams ufomap_params;
  ufomap_params.resolution = 0.2;
  ufomap_params.depth_levels = 16;

  ldopcore::DynamicObjectClustererParams raw_params;
  raw_params.min_points = 0;
  raw_params.max_extent = std::numeric_limits<double>::quiet_NaN();
  raw_params.connectivity = 12;

  const auto config =
      ldopcore::sanitizeDynamicObjectClustererParams(raw_params, ufomap_params);

  EXPECT_EQ(config.min_points, 1U);
  EXPECT_DOUBLE_EQ(config.max_extent, 0.0);
  EXPECT_EQ(config.connectivity, 6);
}
```

在 `CMakeLists.txt` 中把新源文件加入 `ldop_core`：

```cmake
add_library(ldop_core
  src/ldop.cpp
  src/ufomap_mapper.cpp
  src/dynamic_object_clusterer.cpp
)
```

在 `if(CATKIN_ENABLE_TESTING)` 内加入测试目标：

```cmake
  catkin_add_gtest(test_dynamic_object_clusterer
    test/test_dynamic_object_clusterer.cpp
  )
  if(TARGET test_dynamic_object_clusterer)
    target_link_libraries(test_dynamic_object_clusterer
      ldop_core
      ${catkin_LIBRARIES}
      gtest_main
    )
  endif()
```

- [ ] **步骤 2：运行参数测试确认 RED**

运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_dynamic_object_clusterer -j4
```

预期：在实现前应编译失败，原因是 `ldop/dynamic_object_clusterer.h` 或相关类型还不存在。

- [ ] **步骤 3：实现 clusterer 头文件**

创建 `include/ldop/dynamic_object_clusterer.h`：

```cpp
#pragma once

#include <cstddef>
#include <vector>

#include <ldop/DynamicObjectArray.h>
#include <ldop/ufomap_mapper.h>
#include <std_msgs/Header.h>
#include <ufo/map/types.hpp>

namespace ldopcore {

struct DynamicObjectClustererParams {
  double voxel_size{0.0};
  std::size_t min_points{3U};
  double max_extent{3.0};
  int connectivity{6};
};

struct DynamicObjectClustererConfig {
  std::size_t min_points{3U};
  double max_extent{3.0};
  int connectivity{6};
  ufo::depth_t cluster_depth{0};
  double effective_voxel_size{0.2};
  ufo::key_t key_step{1};
};

DynamicObjectClustererConfig sanitizeDynamicObjectClustererParams(
    const DynamicObjectClustererParams& params,
    const UfomapMapperParams& ufomap_params);

class DynamicObjectClusterer {
 public:
  explicit DynamicObjectClusterer(DynamicObjectClustererConfig config);

  ufo::depth_t clusterDepth() const;
  const DynamicObjectClustererConfig& config() const;

  ::ldop::DynamicObjectArray buildDynamicObjectArray(
      const std_msgs::Header& header,
      const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const;

 private:
  DynamicObjectClustererConfig config_;
};

}  // namespace ldopcore
```

- [ ] **步骤 4：实现参数合法化**

创建 `src/dynamic_object_clusterer.cpp`，先放入参数合法化和空 clusterer 壳：

```cpp
#include <ldop/dynamic_object_clusterer.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ldopcore {

namespace {

ufo::depth_t chooseClusterDepth(const double requested_voxel_size,
                                const UfomapMapperParams& ufomap_params) {
  if (!std::isfinite(requested_voxel_size) || requested_voxel_size <= 0.0) {
    return 0;
  }

  const int max_depth = std::max(0, ufomap_params.depth_levels);
  ufo::depth_t best_depth = 0;
  double best_error = std::numeric_limits<double>::infinity();
  for (int depth = 0; depth <= max_depth; ++depth) {
    const double size = ufomap_params.resolution * std::pow(2.0, depth);
    const double error = std::abs(size - requested_voxel_size);
    if (error < best_error) {
      best_error = error;
      best_depth = static_cast<ufo::depth_t>(depth);
    }
  }
  return best_depth;
}

}  // namespace

DynamicObjectClustererConfig sanitizeDynamicObjectClustererParams(
    const DynamicObjectClustererParams& params,
    const UfomapMapperParams& ufomap_params) {
  DynamicObjectClustererConfig config;
  config.min_points = std::max<std::size_t>(params.min_points, 1U);
  config.max_extent =
      std::isfinite(params.max_extent) && params.max_extent > 0.0 ? params.max_extent : 0.0;
  config.connectivity =
      params.connectivity == 6 || params.connectivity == 18 || params.connectivity == 26
          ? params.connectivity
          : 6;

  const double resolution =
      std::isfinite(ufomap_params.resolution) && ufomap_params.resolution > 0.0
          ? ufomap_params.resolution
          : 0.2;
  UfomapMapperParams sanitized_map_params = ufomap_params;
  sanitized_map_params.resolution = resolution;
  sanitized_map_params.depth_levels = std::max(0, sanitized_map_params.depth_levels);

  config.cluster_depth = chooseClusterDepth(params.voxel_size, sanitized_map_params);
  config.effective_voxel_size = resolution * std::pow(2.0, config.cluster_depth);
  config.key_step = ufo::key_t{1} << config.cluster_depth;
  return config;
}

DynamicObjectClusterer::DynamicObjectClusterer(DynamicObjectClustererConfig config)
    : config_(config) {}

ufo::depth_t DynamicObjectClusterer::clusterDepth() const {
  return config_.cluster_depth;
}

const DynamicObjectClustererConfig& DynamicObjectClusterer::config() const {
  return config_;
}

::ldop::DynamicObjectArray DynamicObjectClusterer::buildDynamicObjectArray(
    const std_msgs::Header& header,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  ::ldop::DynamicObjectArray object_array;
  object_array.header = header;
  object_array.objects.clear();
  return object_array;
}

}  // namespace ldopcore
```

- [ ] **步骤 5：运行参数测试确认 GREEN**

运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_dynamic_object_clusterer -j4
/home/st/ldop_ws/devel/.private/ldop/lib/ldop/test_dynamic_object_clusterer \
  --gtest_filter=DynamicObjectClustererParamsTest.*
```

预期：参数合法化测试通过。

## 任务 3：实现 key-space BFS 聚类

**文件：**
- 修改 `src/dynamic_object_clusterer.cpp`
- 修改 `test/test_dynamic_object_clusterer.cpp`

- [ ] **步骤 1：先写聚类行为测试**

在 `test/test_dynamic_object_clusterer.cpp` 追加测试辅助函数和行为测试：

```cpp
namespace {

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

ldopcore::DynamicObjectClusterer makeClusterer(const int connectivity,
                                               const std::size_t min_points = 1U,
                                               const double max_extent = 0.0) {
  ldopcore::DynamicObjectClustererConfig config;
  config.connectivity = connectivity;
  config.min_points = min_points;
  config.max_extent = max_extent;
  config.cluster_depth = 0;
  config.effective_voxel_size = 0.2;
  config.key_step = 1;
  return ldopcore::DynamicObjectClusterer(config);
}

}  // namespace

TEST(DynamicObjectClustererTest, emptyInputProducesEmptyObjectArray) {
  auto clusterer = makeClusterer(6);

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), {});

  EXPECT_EQ(objects.header.frame_id, "map");
  EXPECT_TRUE(objects.objects.empty());
}

TEST(DynamicObjectClustererTest, sixConnectedKeysProduceSingleObject) {
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
}

TEST(DynamicObjectClustererTest, separatedKeysProduceFrameLocalIdsFromZero) {
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
  auto clusterer = makeClusterer(6, 2U, 0.5);
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10),
      makeClusterPoint(1.0F, 0.0F, 0.0F, 11, 10, 10),
      makeClusterPoint(5.0F, 0.0F, 0.0F, 50, 10, 10)};

  const auto objects = clusterer.buildDynamicObjectArray(makeHeader(), points);

  EXPECT_TRUE(objects.objects.empty());
}

TEST(DynamicObjectClustererTest, connectivityControlsDiagonalNeighbors) {
  const std::vector<ldopcore::UfomapDynamicClusterPoint> points{
      makeClusterPoint(0.0F, 0.0F, 0.0F, 10, 10, 10),
      makeClusterPoint(0.2F, 0.2F, 0.0F, 11, 11, 10),
      makeClusterPoint(0.4F, 0.4F, 0.2F, 12, 12, 11)};

  EXPECT_EQ(makeClusterer(6).buildDynamicObjectArray(makeHeader(), points).objects.size(), 3U);
  EXPECT_EQ(makeClusterer(18).buildDynamicObjectArray(makeHeader(), points).objects.size(), 2U);
  EXPECT_EQ(makeClusterer(26).buildDynamicObjectArray(makeHeader(), points).objects.size(), 1U);
}
```

- [ ] **步骤 2：运行聚类测试确认 RED**

运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_dynamic_object_clusterer -j4
/home/st/ldop_ws/devel/.private/ldop/lib/ldop/test_dynamic_object_clusterer \
  --gtest_filter=DynamicObjectClustererTest.*
```

预期：在 BFS 实现前，这些测试应失败，因为 `buildDynamicObjectArray()` 仍返回空数组。

- [ ] **步骤 3：实现 BFS、过滤和 AABB**

在 `src/dynamic_object_clusterer.cpp` 中加入这些头文件：

```cpp
#include <array>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
```

在匿名命名空间中加入内部 key、hash、邻域和工具函数：

```cpp
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
```

把 `DynamicObjectClusterer::buildDynamicObjectArray()` 替换为以下逻辑：

```cpp
::ldop::DynamicObjectArray DynamicObjectClusterer::buildDynamicObjectArray(
    const std_msgs::Header& header,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  ::ldop::DynamicObjectArray object_array;
  object_array.header = header;
  object_array.objects.clear();
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

  const auto offsets = neighborOffsets(config_.connectivity);
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

    if (cluster_points.size() < config_.min_points) {
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
    if (config_.max_extent > 0.0 &&
        extentDiagonal(size_x, size_y, size_z) > config_.max_extent) {
      continue;
    }

    ::ldop::DynamicObject object;
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
```

- [ ] **步骤 4：运行聚类测试确认 GREEN**

运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_dynamic_object_clusterer -j4
/home/st/ldop_ws/devel/.private/ldop/lib/ldop/test_dynamic_object_clusterer
```

预期：`test_dynamic_object_clusterer` 全部通过。

## 任务 4：接入 `Ldop` 主流程

**文件：**
- 修改 `include/ldop/ldop.h`
- 修改 `src/ldop.cpp`
- 修改 `config/ldop.yaml`
- 修改 `CMakeLists.txt`

- [ ] **步骤 1：增加配置参数**

在 `config/ldop.yaml` 的 UFOMap 参数后加入：

```yaml

# 当前帧动态点聚类参数；voxel_size<=0 时使用 ufomap_resolution 对应的 UFOMap leaf depth。
dynamic_cluster_voxel_size: 0.0
dynamic_cluster_min_points: 3
dynamic_cluster_max_extent: 3.0
dynamic_cluster_connectivity: 6
```

- [ ] **步骤 2：确认 clusterer 源文件已进入核心库**

`CMakeLists.txt` 中的 `ldop_core` 应包含：

```cmake
add_library(ldop_core
  src/ldop.cpp
  src/ufomap_mapper.cpp
  src/dynamic_object_clusterer.cpp
)
```

- [ ] **步骤 3：在 `Ldop` 中增加 clusterer 成员**

在 `include/ldop/ldop.h` 中加入：

```cpp
#include <ldop/dynamic_object_clusterer.h>
```

在 `ProcessedFrame` 中加入：

```cpp
    std::vector<UfomapDynamicClusterPoint> dynamic_cluster_points;
```

在 mapper 成员附近加入：

```cpp
  // 当前帧动态点聚类模块，消费 UfomapMapper 输出的结构化动态点。
  std::unique_ptr<DynamicObjectClusterer> dynamic_object_clusterer_;
  DynamicObjectClustererParams dynamic_cluster_params_;
```

- [ ] **步骤 4：读取聚类参数**

在 `Ldop::loadParameters()` 中加入：

```cpp
  double dynamic_cluster_min_points = static_cast<double>(dynamic_cluster_params_.min_points);
  pnh_.param("dynamic_cluster_voxel_size", dynamic_cluster_params_.voxel_size, 0.0);
  pnh_.param("dynamic_cluster_min_points", dynamic_cluster_min_points, 3.0);
  pnh_.param("dynamic_cluster_max_extent", dynamic_cluster_params_.max_extent, 3.0);
  pnh_.param("dynamic_cluster_connectivity", dynamic_cluster_params_.connectivity, 6);
  dynamic_cluster_params_.min_points =
      dynamic_cluster_min_points > 0.0
          ? static_cast<std::size_t>(std::llround(dynamic_cluster_min_points))
          : 0U;
```

- [ ] **步骤 5：构造 clusterer**

在 `Ldop` 构造函数中，`ufomap_mapper_` 创建之后加入：

```cpp
  const DynamicObjectClustererConfig cluster_config =
      sanitizeDynamicObjectClustererParams(dynamic_cluster_params_,
                                          ufomap_mapper_->effectiveParams());
  dynamic_object_clusterer_ = std::make_unique<DynamicObjectClusterer>(cluster_config);
  ROS_INFO_STREAM("  dynamic cluster voxel size: " << cluster_config.effective_voxel_size
                  << ", depth: " << static_cast<int>(cluster_config.cluster_depth)
                  << ", connectivity: " << cluster_config.connectivity
                  << ", min points: " << cluster_config.min_points
                  << ", max extent: " << cluster_config.max_extent);
```

- [ ] **步骤 6：替换旧的单框行为**

在 `Ldop::processFrame()` 中，把 mapper 调用改成传入 cluster depth，并保存结构化动态点：

```cpp
  UfomapFrameResult ufomap_result = ufomap_mapper_->processInputCloud(
      frame.cloud_msg, frame.odom_msg, dynamic_object_clusterer_->clusterDepth());
  processed.static_map_cloud_msg = std::move(ufomap_result.static_map_cloud_msg);
  processed.static_map_markers_msg = std::move(ufomap_result.static_map_markers);
  processed.ufomap_runtime_stats = ufomap_result.runtime_stats;
  processed.ufomap_timing = ufomap_result.timing;
  processed.dynamic_cloud_msg = std::move(ufomap_result.dynamic_cloud_msg);
  processed.dynamic_cluster_points =
      std::move(ufomap_result.classification.dynamic_cluster_points);

  processed.dynamic_objects_msg = buildDynamicObjectArray(processed);
```

把 `Ldop::buildDynamicObjectArray()` 改成调用 clusterer，不再遍历 `dynamic_cloud_msg` 构造整帧单框：

```cpp
::ldop::DynamicObjectArray Ldop::buildDynamicObjectArray(const ProcessedFrame& frame) const {
  std_msgs::Header header = frame.dynamic_cloud_msg.header;
  if (header.frame_id.empty()) {
    header.frame_id = world_frame_;
  }
  return dynamic_object_clusterer_->buildDynamicObjectArray(header, frame.dynamic_cluster_points);
}
```

- [ ] **步骤 7：构建验证**

运行：

```bash
catkin build ldop --no-status
```

预期：构建通过，无编译错误。

## 任务 5：完整验证

**文件：**
- 验证任务 1-4 涉及的所有文件。

- [ ] **步骤 1：运行定向测试**

确保有 ROS master 后运行：

```bash
cmake --build /home/st/ldop_ws/build/ldop --target test_ufomap_mapper -j4
cmake --build /home/st/ldop_ws/build/ldop --target test_dynamic_object_clusterer -j4
/home/st/ldop_ws/devel/.private/ldop/lib/ldop/test_ufomap_mapper
/home/st/ldop_ws/devel/.private/ldop/lib/ldop/test_dynamic_object_clusterer
```

预期：

```text
test_ufomap_mapper 全部通过
test_dynamic_object_clusterer 全部通过
```

- [ ] **步骤 2：运行包级测试**

运行：

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

预期：

```text
Summary: all tests pass with 0 errors and 0 failures
```

- [ ] **步骤 3：检查 diff 范围**

运行：

```bash
git diff --check
git diff --stat
```

预期：

- `git diff --check` 没有空白错误。
- 变更文件只包含本计划列出的实现、测试、配置和设计文档文件。
- 静态地图 marker 输出不再通过 `UfomapPublishData` 薄包装转发。

## 自检结果

- spec 覆盖：方案 B 的 mapper 结构化输出由任务 1 覆盖；参数、depth、`6/18/26` 连通、点数过滤、尺寸过滤、AABB、帧内 ID 和空输出由任务 2-3 覆盖；`Ldop` 接线和配置由任务 4 覆盖；构建/测试验收由任务 5 覆盖。
- 范围检查：静态地图 marker 仍只是可视化快照，直接作为 `visualization_msgs::MarkerArray` 随帧结果返回。
- 类型一致性：计划中统一使用 `UfomapVoxelCode`、`UfomapDynamicClusterPoint`、`DynamicObjectClustererParams`、`DynamicObjectClustererConfig` 和 `DynamicObjectClusterer::buildDynamicObjectArray()`。
