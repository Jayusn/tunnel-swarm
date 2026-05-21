# Dynamic Object Interaction-Aware GMM P3 Implementation Plan

**Status:** Completed. The checklist below is retained as the historical execution plan; it no longer indicates open P3 work. Runtime P3 now includes `PredictionInteractionContext`, UFOMap read-only adapter wiring, probability reweighting, smooth-control rerollout, branch-level map/corridor query budgets, and internal `branch_risk_level` summaries.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build LDOP third-stage P3 interaction-aware GMM trajectory prediction by adding target-target interaction, static UFOMap constraints, free-corridor priors, probability reweighting, and bounded mean-trajectory corrections.

**Architecture:** Add a focused `PredictionInteractionContext` module between P2 branch generation and predictor output application. Keep UFOMap access behind a read-only adapter supplied by `Ldop`, so the context remains testable and reusable by the later P4 class-ROGMAP/risk-map builder without becoming the planner map itself.

**Tech Stack:** ROS1 Noetic, catkin, C++20, Eigen, UFOMap, GoogleTest, LDOP prediction messages.

---

## Scope Check

This plan implements one subsystem: P3 interaction-aware prediction. That subsystem is now complete in the current codebase. It does not implement the P4 planner-facing class-ROGMAP/risk-map builder, but it intentionally exposes structured interaction evidence that P4 can consume later.

Repository rule override: the general planning skill suggests frequent commits, but this repository explicitly says not to create intermediate commits during the task. Execute all steps without committing; after verification, ask the user before creating any commit.

## Design Anchors

- Keep `/ldop/dynamic_object_predictions` message structure unchanged.
- Keep RViz prediction markers as true branch center lines only: `dynamic_prediction_line`.
- Do not restore sample markers, uncertainty ellipsoids, or default diagnostic ROS topics.
- Use tracker/predictor state for dynamic targets; do not infer future dynamic risk from UFOMap dynamic candidates.
- Use UFOMap only for static occupied/near-obstacle/free-space evidence.
- Do not cache a full UFOMap copy. Runtime implementation uses branch-level query budgets inside `PredictionInteractionContext`: one local occupied query per branch and 8 horizontal `seenFree()` corridor probes per branch.
- Do not shrink `model_covariance` because of interaction confidence. Modify branch probability and bounded mean trajectory only.

## File Structure

- Create `include/ldop/prediction_interaction_context.h`
  - Owns P3 params/config structs, read-only map-query abstraction, per-branch interaction results, diagnostics, and `PredictionInteractionContext`.
  - Exposes `PredictionBranchRiskLevel::{Low, Medium, High, Critical}` as an internal P4-facing branch risk summary; it is not added to ROS prediction messages.
  - Depends on Eigen, ROS header/time types, prediction messages, and tracker input structs.
  - Does not include UFOMap headers.

- Create `src/prediction_interaction_context.cpp`
  - Implements target-target costs, map/near-obstacle costs, free-corridor prior, correction hints, branch-level query budgets, and branch risk level classification.
  - Does not read ROS params and does not access `UfomapMapper`.

- Create `test/test_prediction_interaction_context.cpp`
  - Pure focused GoogleTest scenarios using a fake `PredictionMapQuery`.
  - Covers crossing, head-on, following, occupied, near obstacle, free corridor, and unavailable map cases.

- Modify `include/ldop/ufomap_mapper.h`
  - Add neutral static local occupied query structs and method.
  - Add voxel size to point query results so the adapter can report obstacle footprint evidence.

- Modify `src/ufomap_mapper.cpp`
  - Implement `queryLocalOccupied()` using UFOMap region predicates: `Leaf(depth)`, `HitsMin(min_hits)`, and `Intersects(spheres)`.
  - Keep dynamic-candidate query semantics separate.

- Modify `include/ldop/dynamic_object_predictor.h`
  - Add interaction params/config to predictor params/config.
  - Add optional `PredictionMapQuery*` argument to `predict()`.
  - Expose interaction diagnostics in the frame result.

- Modify `src/dynamic_object_predictor.cpp`
  - Load new params.
  - Evaluate `PredictionInteractionContext` after P2 branch generation.
  - Apply probability scales, bounded correction hints, and per-target probability normalization.
  - Keep marker generation unchanged except that it sees the final branches.

- Modify `include/ldop/ldop.h`
  - No persistent member is required for the adapter; keep ownership in `Ldop::processFrame()`.

- Modify `src/ldop.cpp`
  - Add a local adapter class that maps `PredictionMapQuery` calls to `UfomapMapper` read-only queries.
  - Pass the adapter into `DynamicObjectPredictor::predict()`.

- Modify `config/ldop.yaml`
  - Add P3 interaction parameters beside existing prediction parameters.

- Modify `CMakeLists.txt`
  - Add `src/prediction_interaction_context.cpp` to `ldop_core`.
  - Add `test_prediction_interaction_context`.

- Modify `test/test_dynamic_object_predictor.cpp`
  - Add integration regression tests for probability normalization, covariance preservation, correction bounds, and marker semantics.

## Task 1: Add Neutral UFOMap Static Local Occupied Query

**Files:**
- Modify: `include/ldop/ufomap_mapper.h`
- Modify: `src/ufomap_mapper.cpp`

- [ ] **Step 1: Add query result fields and neutral query types**

Add these declarations near the existing query structs in `include/ldop/ufomap_mapper.h`:

```cpp
struct UfomapNodeQueryResult {
  bool exists{false};
  bool occupied{false};
  ufo::Point center{};
  ufo::depth_t depth{0};
  double voxel_size{0.0};
};

struct UfomapLocalOccupiedQuery {
  // depth=0 保持 UFOMap 叶子查询语义；P3 只读取静态占据证据，不混入动态 label 语义。
  ufo::depth_t depth{0};
  std::uint32_t min_hits{1};
  std::size_t max_results{0};
  std::vector<ufo::Sphere> intersects;
};

struct UfomapLocalOccupiedNode {
  ufo::Point center{};
  ufo::depth_t depth{0};
  double voxel_size{0.0};
  std::uint32_t hits{0};
};
```

Add this method next to `queryDynamicCandidates()`:

```cpp
  // 中性静态占据邻域查询：P3 用它查询预测点膨胀半径内的 occupied/near obstacle。
  std::vector<UfomapLocalOccupiedNode> queryLocalOccupied(
      const UfomapLocalOccupiedQuery& query) const;
```

- [ ] **Step 2: Implement `voxel_size` in `queryNode()`**

In `src/ufomap_mapper.cpp`, extend the existing result assignment:

```cpp
  result.center = runtime.map.center(node);
  result.depth = node.depth();
  result.voxel_size = runtime.map.size(node.depth());
  return result;
```

- [ ] **Step 3: Add local occupied append helper**

Add this helper near `appendCandidateNodes()`:

```cpp
template <class Predicate>
void appendLocalOccupiedNodes(const DufomapMapType& map,
                              Predicate&& predicate,
                              const std::size_t max_results,
                              std::vector<UfomapLocalOccupiedNode>& nodes) {
  for (const auto node : map.query(std::forward<Predicate>(predicate))) {
    UfomapLocalOccupiedNode occupied;
    occupied.center = map.center(node);
    occupied.depth = node.depth();
    occupied.voxel_size = map.size(node.depth());
    occupied.hits = static_cast<std::uint32_t>(map.hits(node));
    nodes.push_back(occupied);
    if (max_results > 0U && nodes.size() >= max_results) {
      return;
    }
  }
}
```

- [ ] **Step 4: Implement `queryLocalOccupied()`**

Add this method after `queryDynamicCandidates()`:

```cpp
std::vector<UfomapLocalOccupiedNode> UfomapMapper::queryLocalOccupied(
    const UfomapLocalOccupiedQuery& query) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  std::vector<UfomapLocalOccupiedNode> occupied_nodes;
  const auto& runtime = runtimeLocked();

  const auto leaf = ufo::pred::Leaf(query.depth);
  const auto hits = ufo::pred::HitsMin(static_cast<ufo::count_t>(query.min_hits));
  if (query.intersects.empty()) {
    appendLocalOccupiedNodes(runtime.map, leaf && hits, query.max_results, occupied_nodes);
    return occupied_nodes;
  }

  appendLocalOccupiedNodes(runtime.map,
                           leaf && hits && ufo::pred::Intersects(query.intersects),
                           query.max_results,
                           occupied_nodes);
  return occupied_nodes;
}
```

- [ ] **Step 5: Build-check this interface**

Run from `/home/st/ldop_ws`:

```bash
catkin build ldop --no-status
```

Expected: the build reaches compile or link success. If it fails before P3 code exists because later tasks have not been applied, keep the exact compiler error and continue only when the error is the missing future context type. After Task 2, rerun this command and require success.

## Task 2: Add `PredictionInteractionContext` Public API and Fake-Map Test Scaffold

**Files:**
- Create: `include/ldop/prediction_interaction_context.h`
- Create: `src/prediction_interaction_context.cpp`
- Create: `test/test_prediction_interaction_context.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the public header**

Create `include/ldop/prediction_interaction_context.h` with this API:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/dynamic_object_tracker.h>
#include <std_msgs/Header.h>

namespace ldopcore {

struct PredictionInteractionParams {
  bool enabled{true};                         // P3 主开关；测试可关闭以保持 P2 行为基线。
  double pair_interaction_weight{1.0};        // 目标间距离/TTC/交叉冲突能量权重。
  double map_interaction_weight{1.0};         // 静态 occupied/near obstacle 能量权重。
  double corridor_interaction_weight{0.4};    // free corridor 方向先验权重。
  double interaction_safe_distance{0.4};      // 目标尺寸膨胀后的额外安全距离，单位 m。
  double near_obstacle_distance{0.6};         // 静态障碍近邻软代价半径，单位 m。
  double ttc_safe_time{1.5};                  // 低于该 TTC 的接近关系被认为需要降权。
  double free_corridor_query_radius{1.2};     // 估计可通行方向的局部采样半径，单位 m。
  double max_correction_distance{0.2};        // 单个预测点均值位置最大平移修正，单位 m。
  double max_deceleration_ratio{0.35};        // 单个预测点速度最大衰减比例。
  double interaction_energy_tau{1.0};         // 概率重权重温度，越小越强惩罚高能分支。
};

struct PredictionInteractionConfig {
  bool enabled{true};
  double pair_interaction_weight{1.0};
  double map_interaction_weight{1.0};
  double corridor_interaction_weight{0.4};
  double interaction_safe_distance{0.4};
  double near_obstacle_distance{0.6};
  double ttc_safe_time{1.5};
  double free_corridor_query_radius{1.2};
  double max_correction_distance{0.2};
  double max_deceleration_ratio{0.35};
  double interaction_energy_tau{1.0};
};

struct PredictionMapNodeQuery {
  bool exists{false};
  bool occupied{false};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double voxel_size{0.0};
};

struct PredictionLocalOccupiedNode {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double voxel_size{0.0};
  std::uint32_t hits{0};
};

class PredictionMapQuery {
 public:
  virtual ~PredictionMapQuery() = default;
  virtual bool available() const = 0;
  virtual std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const = 0;
  virtual bool seenFree(const Eigen::Vector3d& point) const = 0;
  virtual std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      double radius,
      std::size_t max_results) const = 0;
};

struct TrajectoryCorrectionHint {
  bool has_position_offset{false};
  bool has_deceleration{false};
  bool has_direction_bias{false};
  Eigen::Vector3d position_offset{Eigen::Vector3d::Zero()};
  double deceleration_ratio{0.0};
  Eigen::Vector3d direction_bias{Eigen::Vector3d::Zero()};
};

struct PredictionBranchInteraction {
  std::uint32_t object_id{0};
  std::size_t branch_index{0};
  double pair_cost{0.0};
  double map_cost{0.0};
  double corridor_cost{0.0};
  double total_energy{0.0};
  double probability_scale{1.0};
  double nearest_obstacle_distance{std::numeric_limits<double>::infinity()};
  bool occupied_overlap{false};
  bool near_obstacle{false};
  Eigen::Vector3d free_corridor_direction{Eigen::Vector3d::Zero()};
  double free_corridor_confidence{0.0};
  double nearest_pair_distance{std::numeric_limits<double>::infinity()};
  double minimum_ttc{std::numeric_limits<double>::infinity()};
  TrajectoryCorrectionHint correction_hint;
};

struct PredictionInteractionDiagnostics {
  double max_pair_cost{0.0};
  double max_map_cost{0.0};
  double max_corridor_cost{0.0};
  std::size_t colliding_branch_count{0};
  std::size_t near_obstacle_branch_count{0};
  std::size_t near_pair_branch_count{0};
  std::size_t free_corridor_aligned_branch_count{0};
  bool map_unavailable{false};
};

struct PredictionInteractionResult {
  std::vector<std::vector<PredictionBranchInteraction>> interactions_by_prediction;
  PredictionInteractionDiagnostics diagnostics;
};

PredictionInteractionConfig buildPredictionInteractionConfig(
    const PredictionInteractionParams& params);

class PredictionInteractionContext {
 public:
  explicit PredictionInteractionContext(PredictionInteractionConfig config);

  PredictionInteractionResult evaluate(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      const ldop::DynamicObjectPredictionArray& predictions,
      const PredictionMapQuery* map_query) const;

 private:
  struct QueryCacheKey {
    int x{0};
    int y{0};
    int z{0};
    int radius_bucket{0};
  };

  PredictionInteractionConfig config_;
};

}  // namespace ldopcore
```

If `std::numeric_limits` is used in the header, include `<limits>`.

- [ ] **Step 2: Create a compiling implementation shell**

Create `src/prediction_interaction_context.cpp`:

```cpp
#include <ldop/prediction_interaction_context.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ldopcore {

PredictionInteractionConfig buildPredictionInteractionConfig(
    const PredictionInteractionParams& params) {
  const PredictionInteractionParams defaults;
  PredictionInteractionConfig config;
  config.enabled = params.enabled;
  config.pair_interaction_weight =
      params.pair_interaction_weight >= 0.0 ? params.pair_interaction_weight
                                           : defaults.pair_interaction_weight;
  config.map_interaction_weight =
      params.map_interaction_weight >= 0.0 ? params.map_interaction_weight
                                          : defaults.map_interaction_weight;
  config.corridor_interaction_weight =
      params.corridor_interaction_weight >= 0.0 ? params.corridor_interaction_weight
                                               : defaults.corridor_interaction_weight;
  config.interaction_safe_distance =
      params.interaction_safe_distance >= 0.0 ? params.interaction_safe_distance
                                             : defaults.interaction_safe_distance;
  config.near_obstacle_distance =
      params.near_obstacle_distance >= 0.0 ? params.near_obstacle_distance
                                          : defaults.near_obstacle_distance;
  config.ttc_safe_time = params.ttc_safe_time > 0.0 ? params.ttc_safe_time
                                                   : defaults.ttc_safe_time;
  config.free_corridor_query_radius =
      params.free_corridor_query_radius > 0.0 ? params.free_corridor_query_radius
                                             : defaults.free_corridor_query_radius;
  config.max_correction_distance =
      params.max_correction_distance >= 0.0 ? params.max_correction_distance
                                           : defaults.max_correction_distance;
  config.max_deceleration_ratio =
      std::clamp(params.max_deceleration_ratio, 0.0, 1.0);
  config.interaction_energy_tau =
      params.interaction_energy_tau > 0.0 ? params.interaction_energy_tau
                                         : defaults.interaction_energy_tau;
  return config;
}

PredictionInteractionContext::PredictionInteractionContext(
    PredictionInteractionConfig config)
    : config_(std::move(config)) {}

PredictionInteractionResult PredictionInteractionContext::evaluate(
    const std_msgs::Header&,
    const std::vector<TrackPredictionInput>&,
    const ldop::DynamicObjectPredictionArray& predictions,
    const PredictionMapQuery* map_query) const {
  PredictionInteractionResult result;
  result.interactions_by_prediction.resize(predictions.predictions.size());
  result.diagnostics.map_unavailable =
      map_query == nullptr || !map_query->available();
  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];
    auto& interactions = result.interactions_by_prediction[prediction_index];
    interactions.reserve(prediction.branches.size());
    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      PredictionBranchInteraction interaction;
      interaction.object_id = prediction.id;
      interaction.branch_index = branch_index;
      interactions.push_back(interaction);
    }
  }
  return result;
}

}  // namespace ldopcore
```

- [ ] **Step 3: Register source and test target**

Modify `CMakeLists.txt`:

```cmake
add_library(ldop_core
  src/ldop.cpp
  src/dynamic_object_clusterer.cpp
  src/dynamic_object_tracker.cpp
  src/dynamic_object_predictor.cpp
  src/prediction_interaction_context.cpp
  src/motion_model.cpp
  src/multi_model_kalman_filter.cpp
  src/ufomap_mapper.cpp
)
```

Inside `if(CATKIN_ENABLE_TESTING)`, add:

```cmake
  catkin_add_gtest(test_prediction_interaction_context
    test/test_prediction_interaction_context.cpp
  )
  if(TARGET test_prediction_interaction_context)
    target_link_libraries(test_prediction_interaction_context
      ldop_core
      ${catkin_LIBRARIES}
      gtest
    )
  endif()
```

- [ ] **Step 4: Create fake-map test scaffold**

Create `test/test_prediction_interaction_context.cpp` with:

```cpp
#include <gtest/gtest.h>

#include <ldop/prediction_interaction_context.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

class FakePredictionMapQuery final : public PredictionMapQuery {
 public:
  bool available() const override { return available_; }

  std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const override {
    if (!available_) {
      return std::nullopt;
    }
    for (const auto& occupied : occupied_nodes_) {
      if ((occupied.center - point).norm() <= occupied.voxel_size * 0.5) {
        PredictionMapNodeQuery node;
        node.exists = true;
        node.occupied = true;
        node.center = occupied.center;
        node.voxel_size = occupied.voxel_size;
        return node;
      }
    }
    PredictionMapNodeQuery node;
    node.exists = true;
    node.occupied = false;
    node.center = point;
    node.voxel_size = 0.2;
    return node;
  }

  bool seenFree(const Eigen::Vector3d& point) const override {
    if (!available_) {
      return false;
    }
    return point.x() > free_x_threshold_;
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t max_results) const override {
    std::vector<PredictionLocalOccupiedNode> nodes;
    if (!available_) {
      return nodes;
    }
    for (const auto& occupied : occupied_nodes_) {
      if ((occupied.center - center).norm() <= radius + occupied.voxel_size * 0.5) {
        nodes.push_back(occupied);
        if (max_results > 0U && nodes.size() >= max_results) {
          return nodes;
        }
      }
    }
    return nodes;
  }

  bool available_{true};
  double free_x_threshold_{0.0};
  std::vector<PredictionLocalOccupiedNode> occupied_nodes_;
};

TrackPredictionInput makeInput(const std::uint32_t id,
                               const Eigen::Vector3d& position,
                               const Eigen::Vector3d& velocity) {
  TrackPredictionInput input;
  input.id = id;
  input.object_class = ObjectClass::Human;
  input.motion_model_type = MotionModelType::CV3D;
  input.bbox.size.x = 0.6;
  input.bbox.size.y = 0.6;
  input.bbox.size.z = 1.5;
  input.model_state = Eigen::VectorXd::Zero(6);
  input.model_state.segment<3>(0) = position;
  input.model_state.segment<3>(3) = velocity;
  input.model_covariance = 0.1 * Eigen::MatrixXd::Identity(6, 6);
  input.matched_in_current_frame = true;
  input.hits = 8U;
  input.age = 8U;
  return input;
}

ldop::DynamicObjectPredictionBranch makeBranch(
    const std::vector<Eigen::Vector3d>& positions) {
  ldop::DynamicObjectPredictionBranch branch;
  branch.behavior_type = ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL;
  branch.behavior_value = 0.0;
  branch.probability = 1.0;
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    ldop::DynamicObjectPredictionPoint point;
    point.time_from_start = ros::Duration(0.1 * static_cast<double>(index + 1U));
    point.model_state = {positions[index].x(), positions[index].y(), positions[index].z(),
                         1.0, 0.0, 0.0};
    point.model_covariance = std::vector<double>(36U, 0.0);
    point.influence_weight = 1.0;
    branch.points.push_back(point);
  }
  return branch;
}

ldop::DynamicObjectPredictionArray makePredictions(
    const std::vector<std::pair<std::uint32_t, ldop::DynamicObjectPredictionBranch>>& branches) {
  ldop::DynamicObjectPredictionArray array;
  array.header.frame_id = "map";
  for (const auto& [id, branch] : branches) {
    ldop::DynamicObjectPrediction prediction;
    prediction.id = id;
    prediction.size.x = 0.6;
    prediction.size.y = 0.6;
    prediction.size.z = 1.5;
    prediction.motion_model_type = ldop::DynamicObjectPrediction::MOTION_MODEL_CV3D;
    prediction.object_class = ldop::DynamicObjectPrediction::CLASS_HUMAN;
    prediction.matched_in_current_frame = true;
    prediction.branches.push_back(branch);
    array.predictions.push_back(prediction);
  }
  return array;
}

TEST(PredictionInteractionContextTest, DisabledContextReturnsIdentityScales) {
  PredictionInteractionParams params;
  params.enabled = false;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.1, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);

  ASSERT_EQ(result.interactions_by_prediction.size(), 1U);
  ASSERT_EQ(result.interactions_by_prediction.front().size(), 1U);
  EXPECT_DOUBLE_EQ(result.interactions_by_prediction.front().front().probability_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.interactions_by_prediction.front().front().total_energy, 0.0);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_prediction_interaction_context");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 5: Run the new scaffold test**

Run from `/home/st/ldop_ws`:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status --no-deps --this
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: `test_prediction_interaction_context` compiles and the scaffold test passes.

## Task 3: Implement Target-Target Interaction Costs

**Files:**
- Modify: `src/prediction_interaction_context.cpp`
- Modify: `test/test_prediction_interaction_context.cpp`

- [ ] **Step 1: Add failing pair-interaction tests**

Add these tests before the namespace closing in `test/test_prediction_interaction_context.cpp`:

```cpp
TEST(PredictionInteractionContextTest, CrossingBranchesGetHigherPairCostThanSeparatedBranches) {
  PredictionInteractionParams params;
  params.enabled = true;
  params.map_interaction_weight = 0.0;
  params.corridor_interaction_weight = 0.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));

  const std::vector<TrackPredictionInput> inputs = {
      makeInput(1U, Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)),
      makeInput(2U, Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)),
  };

  const auto crossing = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, -0.5, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
  });
  const auto separated = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, -0.5, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3d(4.0, 0.0, 0.0)})},
  });

  const auto crossing_result = context.evaluate(crossing.header, inputs, crossing, nullptr);
  const auto separated_result = context.evaluate(separated.header, inputs, separated, nullptr);

  EXPECT_GT(crossing_result.interactions_by_prediction[0][0].pair_cost,
            separated_result.interactions_by_prediction[0][0].pair_cost);
  EXPECT_LT(crossing_result.interactions_by_prediction[0][0].probability_scale, 1.0);
}

TEST(PredictionInteractionContextTest, HeadOnBranchGetsTtcCostAndCorrectionHint) {
  PredictionInteractionParams params;
  params.enabled = true;
  params.map_interaction_weight = 0.0;
  params.corridor_interaction_weight = 0.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));

  const std::vector<TrackPredictionInput> inputs = {
      makeInput(1U, Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)),
      makeInput(2U, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-1.0, 0.0, 0.0)),
  };
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, inputs, predictions, nullptr);

  const auto& interaction = result.interactions_by_prediction[0][0];
  EXPECT_GT(interaction.pair_cost, 0.0);
  EXPECT_LT(interaction.minimum_ttc, params.ttc_safe_time);
  EXPECT_TRUE(interaction.correction_hint.has_deceleration ||
              interaction.correction_hint.has_position_offset);
}

TEST(PredictionInteractionContextTest, FollowingCostsLessThanHeadOn) {
  PredictionInteractionParams params;
  params.enabled = true;
  params.map_interaction_weight = 0.0;
  params.corridor_interaction_weight = 0.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));

  const std::vector<TrackPredictionInput> head_on_inputs = {
      makeInput(1U, Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)),
      makeInput(2U, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-1.0, 0.0, 0.0)),
  };
  const std::vector<TrackPredictionInput> following_inputs = {
      makeInput(1U, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)),
      makeInput(2U, Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)),
  };
  const auto head_on = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)})},
  });
  const auto following = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(0.2, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.6, 0.0, 0.0), Eigen::Vector3d(0.7, 0.0, 0.0)})},
  });

  const auto head_on_result =
      context.evaluate(head_on.header, head_on_inputs, head_on, nullptr);
  const auto following_result =
      context.evaluate(following.header, following_inputs, following, nullptr);

  EXPECT_LT(following_result.interactions_by_prediction[0][0].pair_cost,
            head_on_result.interactions_by_prediction[0][0].pair_cost);
}
```

Run:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status --no-deps --this
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: the new tests fail because pair costs are still zero.

- [ ] **Step 2: Add geometry helpers**

In `src/prediction_interaction_context.cpp`, add these helpers inside an anonymous namespace:

```cpp
namespace {

constexpr double kEpsilon = 1e-9;

Eigen::Vector3d pointPosition(const ldop::DynamicObjectPredictionPoint& point) {
  if (point.model_state.size() < 3U) {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(point.model_state[0], point.model_state[1], point.model_state[2]);
}

double horizontalObjectRadius(const geometry_msgs::Vector3& size) {
  return 0.5 * std::hypot(std::max(0.0, size.x), std::max(0.0, size.y));
}

Eigen::Vector3d branchVelocityAt(const ldop::DynamicObjectPredictionBranch& branch,
                                 const std::size_t index) {
  if (branch.points.size() < 2U) {
    return Eigen::Vector3d::Zero();
  }
  const std::size_t prev_index = index == 0U ? 0U : index - 1U;
  const std::size_t next_index = std::min(index + 1U, branch.points.size() - 1U);
  if (prev_index == next_index) {
    return Eigen::Vector3d::Zero();
  }
  const double dt = std::max((branch.points[next_index].time_from_start -
                              branch.points[prev_index].time_from_start).toSec(),
                             kEpsilon);
  return (pointPosition(branch.points[next_index]) -
          pointPosition(branch.points[prev_index])) / dt;
}

double branchTimeToCollision(const Eigen::Vector3d& relative_position,
                             const Eigen::Vector3d& relative_velocity) {
  const double speed_sq = relative_velocity.head<2>().squaredNorm();
  if (speed_sq <= kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  const double ttc =
      -relative_position.head<2>().dot(relative_velocity.head<2>()) / speed_sq;
  return ttc > 0.0 ? ttc : std::numeric_limits<double>::infinity();
}

double approachClassMultiplier(const Eigen::Vector3d& velocity_a,
                               const Eigen::Vector3d& velocity_b) {
  const double norm_product = velocity_a.head<2>().norm() * velocity_b.head<2>().norm();
  if (norm_product <= kEpsilon) {
    return 1.0;
  }
  const double cos_angle =
      std::clamp(velocity_a.head<2>().dot(velocity_b.head<2>()) / norm_product, -1.0, 1.0);
  if (cos_angle < -0.5) {
    return 1.8;
  }
  if (cos_angle < 0.5) {
    return 1.35;
  }
  return 0.75;
}

void updateDiagnostics(PredictionInteractionDiagnostics& diagnostics,
                       const PredictionBranchInteraction& interaction) {
  diagnostics.max_pair_cost = std::max(diagnostics.max_pair_cost, interaction.pair_cost);
  diagnostics.max_map_cost = std::max(diagnostics.max_map_cost, interaction.map_cost);
  diagnostics.max_corridor_cost =
      std::max(diagnostics.max_corridor_cost, interaction.corridor_cost);
  if (interaction.nearest_pair_distance <
      std::numeric_limits<double>::infinity()) {
    ++diagnostics.near_pair_branch_count;
  }
  if (interaction.occupied_overlap) {
    ++diagnostics.colliding_branch_count;
  }
  if (interaction.near_obstacle) {
    ++diagnostics.near_obstacle_branch_count;
  }
  if (interaction.free_corridor_confidence > 0.5) {
    ++diagnostics.free_corridor_aligned_branch_count;
  }
}

}  // namespace
```

- [ ] **Step 3: Implement pair-cost evaluation**

Add a private local function in `src/prediction_interaction_context.cpp`:

```cpp
void evaluatePairCosts(const std::vector<TrackPredictionInput>& inputs,
                       const ldop::DynamicObjectPredictionArray& predictions,
                       const PredictionInteractionConfig& config,
                       PredictionInteractionResult& result) {
  for (std::size_t a = 0U; a < predictions.predictions.size(); ++a) {
    const auto& prediction_a = predictions.predictions[a];
    const double radius_a = horizontalObjectRadius(prediction_a.size);
    for (std::size_t branch_a_index = 0U;
         branch_a_index < prediction_a.branches.size();
         ++branch_a_index) {
      auto& interaction =
          result.interactions_by_prediction[a][branch_a_index];
      const auto& branch_a = prediction_a.branches[branch_a_index];

      for (std::size_t b = 0U; b < predictions.predictions.size(); ++b) {
        if (a == b) {
          continue;
        }
        const auto& prediction_b = predictions.predictions[b];
        const double radius_b = horizontalObjectRadius(prediction_b.size);
        for (const auto& branch_b : prediction_b.branches) {
          const std::size_t sample_count =
              std::min(branch_a.points.size(), branch_b.points.size());
          for (std::size_t sample = 0U; sample < sample_count; ++sample) {
            const Eigen::Vector3d position_a = pointPosition(branch_a.points[sample]);
            const Eigen::Vector3d position_b = pointPosition(branch_b.points[sample]);
            const Eigen::Vector3d delta = position_b - position_a;
            const double distance = delta.head<2>().norm();
            interaction.nearest_pair_distance =
                std::min(interaction.nearest_pair_distance, distance);

            const Eigen::Vector3d velocity_a = branchVelocityAt(branch_a, sample);
            const Eigen::Vector3d velocity_b = branchVelocityAt(branch_b, sample);
            const double ttc = branchTimeToCollision(delta, velocity_b - velocity_a);
            interaction.minimum_ttc = std::min(interaction.minimum_ttc, ttc);

            const double safe_distance =
                radius_a + radius_b + config.interaction_safe_distance;
            const double distance_margin = safe_distance - distance;
            if (distance_margin > 0.0) {
              const double close_penalty =
                  distance_margin / std::max(config.interaction_safe_distance, kEpsilon);
              interaction.pair_cost += close_penalty *
                  approachClassMultiplier(velocity_a, velocity_b);
            }

            if (ttc < config.ttc_safe_time && distance < safe_distance * 1.5) {
              const double ttc_penalty =
                  (config.ttc_safe_time - ttc) / std::max(config.ttc_safe_time, kEpsilon);
              interaction.pair_cost += ttc_penalty *
                  approachClassMultiplier(velocity_a, velocity_b);
              interaction.correction_hint.has_deceleration = true;
              interaction.correction_hint.deceleration_ratio =
                  std::max(interaction.correction_hint.deceleration_ratio,
                           config.max_deceleration_ratio);
            }
          }
        }
      }
    }
  }

  (void)inputs;
}
```

The `(void)inputs;` line is intentional for the first implementation because dynamic interaction uses the generated future branches and prediction sizes. Keep `TrackPredictionInput[]` in the signature because later P4 reuse may need model-native current state without changing the interface.

- [ ] **Step 4: Apply total energy and scales**

In `PredictionInteractionContext::evaluate()`, after initializing the result but before returning:

```cpp
  if (!config_.enabled) {
    return result;
  }

  evaluatePairCosts(inputs, predictions, config_, result);

  for (auto& interactions : result.interactions_by_prediction) {
    for (auto& interaction : interactions) {
      interaction.total_energy =
          config_.pair_interaction_weight * interaction.pair_cost +
          config_.map_interaction_weight * interaction.map_cost +
          config_.corridor_interaction_weight * interaction.corridor_cost;
      interaction.probability_scale =
          std::exp(-interaction.total_energy / config_.interaction_energy_tau);
      updateDiagnostics(result.diagnostics, interaction);
    }
  }
```

- [ ] **Step 5: Run pair tests**

Run:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status --no-deps --this
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all current `PredictionInteractionContextTest` tests pass.

## Task 4: Implement Static Map, Near-Obstacle, Free-Corridor Costs, and Local Query Cache

**Files:**
- Modify: `src/prediction_interaction_context.cpp`
- Modify: `test/test_prediction_interaction_context.cpp`

- [ ] **Step 1: Add failing map and corridor tests**

Append these tests:

```cpp
TEST(PredictionInteractionContextTest, OccupiedBranchGetsMapCostAndCollisionDiagnostic) {
  PredictionInteractionParams params;
  params.enabled = true;
  params.pair_interaction_weight = 0.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.occupied_nodes_.push_back(
      PredictionLocalOccupiedNode{Eigen::Vector3d(0.2, 0.0, 0.0), 0.2, 3U});

  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.2, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);

  const auto& interaction = result.interactions_by_prediction[0][0];
  EXPECT_GT(interaction.map_cost, 0.0);
  EXPECT_TRUE(interaction.occupied_overlap);
  EXPECT_GT(result.diagnostics.colliding_branch_count, 0U);
  EXPECT_TRUE(interaction.correction_hint.has_position_offset);
}

TEST(PredictionInteractionContextTest, NearObstacleCostsLessThanOccupied) {
  PredictionInteractionParams params;
  params.enabled = true;
  params.pair_interaction_weight = 0.0;
  params.near_obstacle_distance = 0.8;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.occupied_nodes_.push_back(
      PredictionLocalOccupiedNode{Eigen::Vector3d(0.0, 0.0, 0.0), 0.2, 3U});

  const auto occupied = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0)})},
  });
  const auto near = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.6, 0.0, 0.0)})},
  });
  const auto free = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(2.0, 0.0, 0.0)})},
  });

  const auto occupied_result = context.evaluate(occupied.header, {}, occupied, &map);
  const auto near_result = context.evaluate(near.header, {}, near, &map);
  const auto free_result = context.evaluate(free.header, {}, free, &map);

  EXPECT_GT(occupied_result.interactions_by_prediction[0][0].map_cost,
            near_result.interactions_by_prediction[0][0].map_cost);
  EXPECT_GT(near_result.interactions_by_prediction[0][0].map_cost,
            free_result.interactions_by_prediction[0][0].map_cost);
}

TEST(PredictionInteractionContextTest, FreeCorridorAlignedBranchGetsLowerCorridorCost) {
  PredictionInteractionParams params;
  params.enabled = true;
  params.pair_interaction_weight = 0.0;
  params.map_interaction_weight = 0.0;
  params.corridor_interaction_weight = 1.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.free_x_threshold_ = 0.0;

  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.2, 0.0, 0.0), Eigen::Vector3d(0.4, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(-0.2, 0.0, 0.0), Eigen::Vector3d(-0.4, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);

  EXPECT_LT(result.interactions_by_prediction[0][0].corridor_cost,
            result.interactions_by_prediction[1][0].corridor_cost);
  EXPECT_GT(result.interactions_by_prediction[0][0].free_corridor_confidence, 0.0);
}

TEST(PredictionInteractionContextTest, UnavailableMapDisablesMapCostsOnly) {
  PredictionInteractionParams params;
  params.enabled = true;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.available_ = false;
  map.occupied_nodes_.push_back(
      PredictionLocalOccupiedNode{Eigen::Vector3d(0.0, 0.0, 0.0), 0.2, 3U});
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);

  EXPECT_TRUE(result.diagnostics.map_unavailable);
  EXPECT_DOUBLE_EQ(result.interactions_by_prediction[0][0].map_cost, 0.0);
  EXPECT_DOUBLE_EQ(result.interactions_by_prediction[0][0].corridor_cost, 0.0);
}
```

Run the test and confirm these fail while pair tests still pass.

- [ ] **Step 2: Add map helper functions**

Add to the anonymous namespace:

```cpp
double objectQueryRadius(const geometry_msgs::Vector3& size,
                         const PredictionInteractionConfig& config) {
  return horizontalObjectRadius(size) + config.interaction_safe_distance;
}

Eigen::Vector3d normalizedOrZero(const Eigen::Vector3d& vector) {
  const double norm = vector.norm();
  return norm > kEpsilon ? vector / norm : Eigen::Vector3d::Zero();
}

double distanceToOccupiedSurface(const Eigen::Vector3d& position,
                                 const PredictionLocalOccupiedNode& node) {
  const double node_radius = std::max(0.0, node.voxel_size) * 0.5;
  return std::max(0.0, (position - node.center).norm() - node_radius);
}

Eigen::Vector3d branchDirection(const ldop::DynamicObjectPredictionBranch& branch) {
  if (branch.points.size() < 2U) {
    return Eigen::Vector3d::Zero();
  }
  return normalizedOrZero(pointPosition(branch.points.back()) -
                          pointPosition(branch.points.front()));
}
```

- [ ] **Step 3: Implement local map evidence evaluation**

Add this function:

```cpp
void evaluateMapCosts(const ldop::DynamicObjectPredictionArray& predictions,
                      const PredictionMapQuery* map_query,
                      const PredictionInteractionConfig& config,
                      PredictionInteractionResult& result) {
  if (map_query == nullptr || !map_query->available()) {
    result.diagnostics.map_unavailable = true;
    return;
  }

  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];
    const double query_radius = objectQueryRadius(prediction.size, config);
    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      auto& interaction =
          result.interactions_by_prediction[prediction_index][branch_index];
      const auto& branch = prediction.branches[branch_index];
      Eigen::Vector3d push_sum = Eigen::Vector3d::Zero();

      for (const auto& point : branch.points) {
        const Eigen::Vector3d position = pointPosition(point);
        const auto node = map_query->queryNode(position);
        if (node.has_value() && node->occupied) {
          interaction.occupied_overlap = true;
          interaction.map_cost += 2.0;
          push_sum += normalizedOrZero(position - node->center);
        }

        const auto nearby =
            map_query->queryLocalOccupied(position,
                                          config.near_obstacle_distance + query_radius,
                                          16U);
        for (const auto& occupied : nearby) {
          const double distance = distanceToOccupiedSurface(position, occupied);
          interaction.nearest_obstacle_distance =
              std::min(interaction.nearest_obstacle_distance, distance);
          if (distance <= query_radius) {
            interaction.occupied_overlap = true;
            interaction.map_cost += 2.0;
            push_sum += normalizedOrZero(position - occupied.center);
          } else if (distance <= config.near_obstacle_distance + query_radius) {
            interaction.near_obstacle = true;
            const double near_ratio =
                1.0 - (distance - query_radius) /
                    std::max(config.near_obstacle_distance, kEpsilon);
            interaction.map_cost += std::clamp(near_ratio, 0.0, 1.0);
            push_sum += normalizedOrZero(position - occupied.center) *
                        std::clamp(near_ratio, 0.0, 1.0);
          }
        }
      }

      if ((interaction.occupied_overlap || interaction.near_obstacle) &&
          push_sum.norm() > kEpsilon) {
        interaction.correction_hint.has_position_offset = true;
        interaction.correction_hint.position_offset =
            normalizedOrZero(push_sum) * config.max_correction_distance;
      }
    }
  }
}
```

- [ ] **Step 4: Implement free-corridor direction prior**

Add this function:

```cpp
void evaluateCorridorCosts(const ldop::DynamicObjectPredictionArray& predictions,
                           const PredictionMapQuery* map_query,
                           const PredictionInteractionConfig& config,
                           PredictionInteractionResult& result) {
  if (map_query == nullptr || !map_query->available()) {
    return;
  }

  const std::array<Eigen::Vector3d, 8U> directions = {
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      normalizedOrZero(Eigen::Vector3d(1.0, 1.0, 0.0)),
      normalizedOrZero(Eigen::Vector3d(1.0, -1.0, 0.0)),
      normalizedOrZero(Eigen::Vector3d(-1.0, 1.0, 0.0)),
      normalizedOrZero(Eigen::Vector3d(-1.0, -1.0, 0.0)),
  };

  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];
    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      const auto& branch = prediction.branches[branch_index];
      if (branch.points.empty()) {
        continue;
      }
      auto& interaction =
          result.interactions_by_prediction[prediction_index][branch_index];
      const Eigen::Vector3d anchor = pointPosition(branch.points.front());
      Eigen::Vector3d best_direction = Eigen::Vector3d::Zero();
      int best_score = -1;
      int second_score = -1;

      for (const auto& direction : directions) {
        int score = 0;
        for (int step = 1; step <= 3; ++step) {
          const Eigen::Vector3d sample =
              anchor + direction *
                  (config.free_corridor_query_radius *
                   static_cast<double>(step) / 3.0);
          if (map_query->seenFree(sample)) {
            ++score;
          }
          if (!map_query->queryLocalOccupied(sample, config.interaction_safe_distance, 1U)
                   .empty()) {
            --score;
          }
        }
        if (score > best_score) {
          second_score = best_score;
          best_score = score;
          best_direction = direction;
        } else if (score > second_score) {
          second_score = score;
        }
      }

      if (best_score <= 0 || best_score == second_score) {
        continue;
      }
      const double confidence =
          std::clamp(static_cast<double>(best_score - second_score) / 3.0, 0.0, 1.0);
      const Eigen::Vector3d direction = branchDirection(branch);
      if (direction.norm() <= kEpsilon) {
        continue;
      }
      const double alignment =
          std::clamp(direction.head<2>().dot(best_direction.head<2>()), -1.0, 1.0);
      interaction.free_corridor_direction = best_direction;
      interaction.free_corridor_confidence = confidence;
      interaction.corridor_cost = confidence * std::max(0.0, 1.0 - alignment);
      if (confidence > 0.0 && alignment < 0.9) {
        interaction.correction_hint.has_direction_bias = true;
        interaction.correction_hint.direction_bias = best_direction;
      }
    }
  }
}
```

Add `#include <array>` at the top.

- [ ] **Step 5: Wire map and corridor evaluation**

In `PredictionInteractionContext::evaluate()`, call these after pair costs and before total energy:

```cpp
  evaluatePairCosts(inputs, predictions, config_, result);
  evaluateMapCosts(predictions, map_query, config_, result);
  evaluateCorridorCosts(predictions, map_query, config_, result);
```

- [ ] **Step 6: Enforce branch-level query budget after correctness passes**

Once tests pass, reduce repeated local queries by bounding query count directly instead of relying on cache hit rate:

- In `evaluateMapCosts()`, build one local sphere per branch that covers the short rollout, object circumscribed radius, and near-obstacle radius.
- Call `PredictionMapQuery::queryLocalOccupied()` once per branch, then compute each sample's signed distance, occupied overlap, near-obstacle cost, and strongest local correction hint in memory.
- In `evaluateCorridorCosts()`, use the branch representative point and probe exactly 8 horizontal `seenFree()` directions per branch.
- Do not call `queryNode()` in the optimized P3 main path.

The earlier `LocalOccupiedCacheKey` / `std::unordered_map` idea is superseded by this branch-level budget because the runtime issue was query-count explosion (`points * directions`), not merely repeated keys. If P4 later needs 26-neighborhood or 3D UAV corridor probes, add it as a configurable mode and rerun timing verification.

- [ ] **Step 7: Run map and corridor tests**

Run:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status --no-deps --this
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all `PredictionInteractionContextTest` tests pass.

## Task 5: Integrate Interaction Context into `DynamicObjectPredictor`

**Files:**
- Modify: `include/ldop/dynamic_object_predictor.h`
- Modify: `src/dynamic_object_predictor.cpp`
- Modify: `test/test_dynamic_object_predictor.cpp`

- [ ] **Step 1: Add predictor-facing interaction fields**

In `include/ldop/dynamic_object_predictor.h`, include:

```cpp
#include <ldop/prediction_interaction_context.h>
```

Extend params and config:

```cpp
struct DynamicObjectPredictorParams {
  double prediction_horizon{1.0};
  int max_branches{6};
  double min_branch_probability{0.03};
  int history_window{10};
  double heading_stable_angle{0.35};
  double stop_speed{0.2};
  double turn_yaw_rate{0.5};
  double lateral_speed{0.4};
  double vertical_speed{0.3};
  double history_quality_noise_scale{1.5};
  double hypothesis_noise_scale{1.3};
  PredictionInteractionParams interaction;
  MultiModelKalmanFilterParams filter;
};

struct DynamicObjectPredictorConfig {
  double prediction_horizon{1.0};
  std::size_t max_branches{6U};
  double min_branch_probability{0.03};
  std::size_t history_window{10U};
  double heading_stable_angle{0.35};
  double stop_speed{0.2};
  double turn_yaw_rate{0.5};
  double lateral_speed{0.4};
  double vertical_speed{0.3};
  double history_quality_noise_scale{1.5};
  double hypothesis_noise_scale{1.3};
  PredictionInteractionConfig interaction_config;
  MultiModelKalmanFilterConfig filter_config;
};
```

Extend timing and result:

```cpp
struct DynamicObjectPredictorTimingStats {
  double predict_dynamic_objects_ms{0.0};
  double apply_interaction_context_ms{0.0};
  double build_prediction_markers_ms{0.0};
  double predict_total_ms{0.0};
};

struct DynamicObjectPredictorFrameResult {
  ldop::DynamicObjectPredictionArray predictions_msg;
  visualization_msgs::MarkerArray prediction_markers_msg;
  PredictionInteractionDiagnostics interaction_diagnostics;
  DynamicObjectPredictorTimingStats timing;
};
```

Change the predict signature:

```cpp
  DynamicObjectPredictorFrameResult predict(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      const PredictionMapQuery* map_query = nullptr) const;
```

- [ ] **Step 2: Load and build interaction params**

In `buildPredictorConfig()`:

```cpp
  config.interaction_config = buildPredictionInteractionConfig(params.interaction);
```

In `DynamicObjectPredictor::loadParameters()`:

```cpp
  pnh_.param("prediction_enable_interaction_context",
             params_.interaction.enabled,
             defaults.interaction.enabled);
  pnh_.param("prediction_pair_interaction_weight",
             params_.interaction.pair_interaction_weight,
             defaults.interaction.pair_interaction_weight);
  pnh_.param("prediction_map_interaction_weight",
             params_.interaction.map_interaction_weight,
             defaults.interaction.map_interaction_weight);
  pnh_.param("prediction_corridor_interaction_weight",
             params_.interaction.corridor_interaction_weight,
             defaults.interaction.corridor_interaction_weight);
  pnh_.param("prediction_interaction_safe_distance",
             params_.interaction.interaction_safe_distance,
             defaults.interaction.interaction_safe_distance);
  pnh_.param("prediction_near_obstacle_distance",
             params_.interaction.near_obstacle_distance,
             defaults.interaction.near_obstacle_distance);
  pnh_.param("prediction_ttc_safe_time",
             params_.interaction.ttc_safe_time,
             defaults.interaction.ttc_safe_time);
  pnh_.param("prediction_free_corridor_query_radius",
             params_.interaction.free_corridor_query_radius,
             defaults.interaction.free_corridor_query_radius);
  pnh_.param("prediction_max_correction_distance",
             params_.interaction.max_correction_distance,
             defaults.interaction.max_correction_distance);
  pnh_.param("prediction_max_deceleration_ratio",
             params_.interaction.max_deceleration_ratio,
             defaults.interaction.max_deceleration_ratio);
  pnh_.param("prediction_interaction_energy_tau",
             params_.interaction.interaction_energy_tau,
             defaults.interaction.interaction_energy_tau);
```

- [ ] **Step 3: Add branch correction helpers**

In `src/dynamic_object_predictor.cpp`, add helpers in the existing anonymous namespace:

```cpp
MotionModelType motionModelTypeFromPrediction(const std::uint8_t value) {
  switch (value) {
    case ldop::DynamicObjectPrediction::MOTION_MODEL_CA2D:
      return MotionModelType::CA2D;
    case ldop::DynamicObjectPrediction::MOTION_MODEL_CA3D:
      return MotionModelType::CA3D;
    case ldop::DynamicObjectPrediction::MOTION_MODEL_CV3D:
      return MotionModelType::CV3D;
    case ldop::DynamicObjectPrediction::MOTION_MODEL_CTRA:
      return MotionModelType::CTRA;
    default:
      return MotionModelType::CV3D;
  }
}

void applyPositionOffsetToPoint(const TrajectoryCorrectionHint& hint,
                                ldop::DynamicObjectPredictionPoint& point) {
  if (!hint.has_position_offset || point.model_state.size() < 3U) {
    return;
  }
  point.model_state[0] += hint.position_offset.x();
  point.model_state[1] += hint.position_offset.y();
  point.model_state[2] += hint.position_offset.z();
}

void applyVelocityCorrectionToPoint(const MotionModelType model_type,
                                    const TrajectoryCorrectionHint& hint,
                                    ldop::DynamicObjectPredictionPoint& point) {
  if ((!hint.has_deceleration && !hint.has_direction_bias) ||
      point.model_state.size() < 6U) {
    return;
  }

  Eigen::Vector2d velocity(point.model_state[3], point.model_state[4]);
  if (model_type == MotionModelType::CTRA) {
    const double speed = point.model_state[3];
    const double yaw = point.model_state[5];
    velocity = Eigen::Vector2d(speed * std::cos(yaw), speed * std::sin(yaw));
  }

  if (hint.has_deceleration) {
    velocity *= std::max(0.0, 1.0 - hint.deceleration_ratio);
  }
  if (hint.has_direction_bias && velocity.norm() > kTimeEpsilon) {
    const Eigen::Vector2d bias(hint.direction_bias.x(), hint.direction_bias.y());
    if (bias.norm() > kTimeEpsilon) {
      velocity = 0.8 * velocity + 0.2 * velocity.norm() * bias.normalized();
    }
  }

  setHorizontalVelocity(model_type, point.model_state, velocity);
}

void applyInteractionResult(ldop::DynamicObjectPredictionArray& predictions,
                            const PredictionInteractionResult& interaction_result,
                            const DynamicObjectPredictorConfig& config) {
  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    auto& prediction = predictions.predictions[prediction_index];
    if (prediction_index >= interaction_result.interactions_by_prediction.size()) {
      continue;
    }
    const auto& interactions =
        interaction_result.interactions_by_prediction[prediction_index];

    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size() && branch_index < interactions.size();
         ++branch_index) {
      auto& branch = prediction.branches[branch_index];
      const auto& interaction = interactions[branch_index];
      branch.probability *= interaction.probability_scale;

      const MotionModelType model_type =
          motionModelTypeFromPrediction(prediction.motion_model_type);
      for (auto& point : branch.points) {
        applyPositionOffsetToPoint(interaction.correction_hint, point);
        applyVelocityCorrectionToPoint(model_type, interaction.correction_hint, point);
      }
    }

    const double probability_sum =
        std::accumulate(prediction.branches.begin(),
                        prediction.branches.end(),
                        0.0,
                        [](const double sum, const auto& branch) {
                          return sum + branch.probability;
                        });
    if (probability_sum <= 1e-12 && !prediction.branches.empty()) {
      auto best = std::max_element(prediction.branches.begin(),
                                   prediction.branches.end(),
                                   [](const auto& lhs, const auto& rhs) {
                                     return lhs.probability < rhs.probability;
                                   });
      for (auto& branch : prediction.branches) {
        branch.probability = (&branch == &(*best)) ? 1.0 : 0.0;
      }
      continue;
    }
    if (probability_sum > 0.0) {
      for (auto& branch : prediction.branches) {
        branch.probability /= probability_sum;
      }
    }
  }

  (void)config;
}
```

If `setHorizontalVelocity()` currently takes `Eigen::VectorXd&`, this helper cannot pass `std::vector<double>&`. Add a separate overload:

```cpp
void setHorizontalVelocity(const MotionModelType model_type,
                           std::vector<double>& state,
                           const Eigen::Vector2d& velocity) {
  switch (model_type) {
    case MotionModelType::CA2D:
    case MotionModelType::CA3D:
    case MotionModelType::CV3D:
      if (state.size() >= 5U) {
        state[3] = velocity.x();
        state[4] = velocity.y();
      }
      break;
    case MotionModelType::CTRA:
      if (state.size() >= 6U) {
        state[3] = velocity.norm();
        if (velocity.norm() > 1e-9) {
          state[5] = std::atan2(velocity.y(), velocity.x());
        }
      }
      break;
  }
}
```

- [ ] **Step 4: Call the context in `predict()`**

Change the signature implementation and add the context stage after all predictions are generated:

```cpp
DynamicObjectPredictorFrameResult DynamicObjectPredictor::predict(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs,
    const PredictionMapQuery* map_query) const {
```

After the prediction generation loop and before `predict_dynamic_objects_ms` is finalized:

```cpp
  if (config_.interaction_config.enabled && !result.predictions_msg.predictions.empty()) {
    const auto interaction_start = std::chrono::steady_clock::now();
    PredictionInteractionContext interaction_context(config_.interaction_config);
    const PredictionInteractionResult interaction_result =
        interaction_context.evaluate(header, inputs, result.predictions_msg, map_query);
    applyInteractionResult(result.predictions_msg, interaction_result, config_);
    result.interaction_diagnostics = interaction_result.diagnostics;
    result.timing.apply_interaction_context_ms =
        elapsedMs(interaction_start, std::chrono::steady_clock::now());
  }
```

Update verbose logging:

```cpp
                                       << "ms, applyInteractionContext="
                                       << formatFloatMs(result.timing.apply_interaction_context_ms)
```

- [ ] **Step 5: Add predictor integration tests**

Append tests to `test/test_dynamic_object_predictor.cpp`. Use a local fake map compatible with `PredictionMapQuery`:

```cpp
class PredictorFakeMapQuery final : public PredictionMapQuery {
 public:
  bool available() const override { return true; }
  std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d&) const override {
    return std::nullopt;
  }
  bool seenFree(const Eigen::Vector3d& point) const override {
    return point.x() > 0.0;
  }
  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t) const override {
    if ((center - occupied_center_).norm() <= radius) {
      return {PredictionLocalOccupiedNode{occupied_center_, 0.2, 3U}};
    }
    return {};
  }
  Eigen::Vector3d occupied_center_{0.2, 0.0, 0.0};
};
```

Add:

```cpp
TEST(DynamicObjectPredictorTest, InteractionContextKeepsBranchProbabilitiesNormalized) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.max_branches = 8U;
  config.interaction_config.enabled = true;
  config.interaction_config.map_interaction_weight = 0.0;
  DynamicObjectPredictor predictor(config);

  TrackPredictionInput first = makeCvInput();
  first.id = 1U;
  first.object_class = ObjectClass::Human;
  first.hits = 8U;
  first.age = 8U;
  appendCvLinearHistory(first, 140.0, 8, 1.0, 0.0);
  TrackPredictionInput second = makeCvInput();
  second.id = 2U;
  second.object_class = ObjectClass::Human;
  second.model_state(0) = 0.2;
  second.model_state(3) = -1.0;
  second.hits = 8U;
  second.age = 8U;
  appendCvLinearHistory(second, 140.0, 8, -1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(141.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {first, second});

  ASSERT_EQ(result.predictions_msg.predictions.size(), 2U);
  for (const auto& prediction : result.predictions_msg.predictions) {
    EXPECT_NEAR(probabilitySum(prediction), 1.0, 1e-9);
    EXPECT_FALSE(prediction.branches.empty());
  }
}

TEST(DynamicObjectPredictorTest, InteractionContextDoesNotShrinkModelCovariance) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.2;
  config.filter_config.default_dt = 0.1;
  config.max_branches = 1U;
  config.interaction_config.enabled = true;
  config.interaction_config.map_interaction_weight = 1.0;
  DynamicObjectPredictor predictor(config);
  PredictorFakeMapQuery map;

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 150.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(151.0);
  header.frame_id = "map";
  DynamicObjectPredictorConfig baseline_config = config;
  baseline_config.interaction_config.enabled = false;
  DynamicObjectPredictor baseline_predictor(baseline_config);
  const auto baseline = baseline_predictor.predict(header, {input});
  const auto interacted = predictor.predict(header, {input}, &map);

  const auto& baseline_point =
      baseline.predictions_msg.predictions.front().branches.front().points.back();
  const auto& interacted_point =
      interacted.predictions_msg.predictions.front().branches.front().points.back();
  EXPECT_GE(positionVarianceTrace(interacted_point),
            positionVarianceTrace(baseline_point));
}

TEST(DynamicObjectPredictorTest, InteractionContextStillBuildsOnlyPredictionLineMarkers) {
  DynamicObjectPredictorConfig config;
  config.prediction_horizon = 0.3;
  config.filter_config.default_dt = 0.1;
  config.min_branch_probability = 0.0;
  config.interaction_config.enabled = true;
  DynamicObjectPredictor predictor(config);
  PredictorFakeMapQuery map;

  TrackPredictionInput input = makeCvInput();
  input.object_class = ObjectClass::Human;
  input.hits = 8U;
  input.age = 8U;
  appendCvLinearHistory(input, 160.0, 8, 1.0, 0.0);

  std_msgs::Header header;
  header.stamp = ros::Time(161.0);
  header.frame_id = "map";
  const auto result = predictor.predict(header, {input}, &map);

  EXPECT_GT(countMarkersByNamespace(result.prediction_markers_msg,
                                    "dynamic_prediction_line"),
            0U);
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg,
                                    "dynamic_prediction_samples"),
            0U);
  EXPECT_EQ(countMarkersByNamespace(result.prediction_markers_msg,
                                    "dynamic_prediction_uncertainty"),
            0U);
}
```

- [ ] **Step 6: Run predictor tests**

Run:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status --no-deps --this
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: existing predictor tests and new interaction tests pass.

## Task 6: Add `Ldop` UFOMap Adapter and Runtime Wiring

**Files:**
- Modify: `src/ldop.cpp`
- Modify: `include/ldop/ldop.h` only if the compiler needs a declaration for a moved helper.

- [ ] **Step 1: Add adapter class in `src/ldop.cpp`**

After includes and before `Ldop::Ldop`, add an anonymous namespace:

```cpp
namespace {

Eigen::Vector3d fromUfoPoint(const ufo::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

ufo::Point toUfoPoint(const Eigen::Vector3d& point) {
  return ufo::Point(static_cast<float>(point.x()),
                    static_cast<float>(point.y()),
                    static_cast<float>(point.z()));
}

class UfomapPredictionMapQuery final : public PredictionMapQuery {
 public:
  explicit UfomapPredictionMapQuery(const UfomapMapper& mapper)
      : mapper_(mapper) {}

  bool available() const override { return true; }

  std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const override {
    const auto result = mapper_.queryNode(toUfoPoint(point));
    if (!result.has_value()) {
      return std::nullopt;
    }
    PredictionMapNodeQuery node;
    node.exists = result->exists;
    node.occupied = result->occupied;
    node.center = fromUfoPoint(result->center);
    node.voxel_size = result->voxel_size;
    return node;
  }

  bool seenFree(const Eigen::Vector3d& point) const override {
    return mapper_.seenFree(toUfoPoint(point));
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t max_results) const override {
    UfomapLocalOccupiedQuery query;
    query.min_hits = 1U;
    query.max_results = max_results;
    query.intersects.emplace_back(toUfoPoint(center), static_cast<float>(radius));
    const auto nodes = mapper_.queryLocalOccupied(query);
    std::vector<PredictionLocalOccupiedNode> converted;
    converted.reserve(nodes.size());
    for (const auto& node : nodes) {
      PredictionLocalOccupiedNode converted_node;
      converted_node.center = fromUfoPoint(node.center);
      converted_node.voxel_size = node.voxel_size;
      converted_node.hits = node.hits;
      converted.push_back(converted_node);
    }
    return converted;
  }

 private:
  const UfomapMapper& mapper_;
};

}  // namespace
```

If `ufo::Sphere` constructor takes a different signature in this installed UFOMap version, inspect the existing headers under `external/install/ufomap/include/ufo/geometry/sphere.hpp` and adapt only this construction line.

- [ ] **Step 2: Pass adapter into predictor**

In `Ldop::processFrame()`, replace the predictor call with:

```cpp
  UfomapPredictionMapQuery prediction_map_query(*ufomap_mapper_);
  DynamicObjectPredictorFrameResult prediction_result =
      dynamic_object_predictor_->predict(processed.dynamic_cloud_msg.header,
                                         tracking_result.prediction_inputs,
                                         &prediction_map_query);
```

This adapter is stack-local because it does not own map state. The underlying `UfomapMapper` retains its own lock discipline for every read-only query.

- [ ] **Step 3: Build-check runtime wiring**

Run:

```bash
catkin build ldop --no-status
```

Expected: `ldop_core` and `ldop_node` compile and link successfully.

## Task 7: Add YAML Defaults and Final CMake/Test Integration

**Files:**
- Modify: `config/ldop.yaml`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add P3 interaction parameters to YAML**

Insert after the existing P2 prediction parameters:

```yaml
prediction_enable_interaction_context: true  # 启用P3交互感知预测：目标间交互、静态地图约束和free corridor方向先验
prediction_pair_interaction_weight: 1.0  # 目标间距离/TTC/交叉冲突代价权重
prediction_map_interaction_weight: 1.0  # 静态occupied/near obstacle代价权重
prediction_corridor_interaction_weight: 0.4  # free corridor方向先验权重
prediction_interaction_safe_distance: 0.4  # 目标尺寸膨胀后的额外安全距离，单位m
prediction_near_obstacle_distance: 0.6  # 静态障碍近邻软惩罚半径，单位m
prediction_ttc_safe_time: 1.5  # TTC低于该值时提高迎面/交叉冲突惩罚，单位s
prediction_free_corridor_query_radius: 1.2  # 局部free corridor方向估计采样半径，单位m
prediction_max_correction_distance: 0.2  # 单个预测点均值位置最大修正距离，单位m
prediction_max_deceleration_ratio: 0.35  # 单个预测点速度最大衰减比例
prediction_interaction_energy_tau: 1.0  # 交互能量转概率缩放的温度参数
```

- [ ] **Step 2: Confirm CMake contains both new entries**

Make sure these two entries are present:

```cmake
  src/prediction_interaction_context.cpp
```

```cmake
  catkin_add_gtest(test_prediction_interaction_context
    test/test_prediction_interaction_context.cpp
  )
```

- [ ] **Step 3: Run all package tests**

Run:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: build succeeds and `catkin_test_results` reports no failures for `ldop`.

## Task 8: Documentation Sync and Final Verification

**Files:**
- Modify: `task_plan.md`
- Modify: `findings.md`
- Modify: `progress.md`
- Optional modify: `docs/superpowers/specs/2026-05-19-dynamic-object-interaction-aware-gmm-p3-design.md` only if implementation exposes a necessary naming correction.

- [ ] **Step 1: Update planning records**

Update `task_plan.md`:

```markdown
- P3 实现计划：`docs/superpowers/plans/2026-05-19-dynamic-object-interaction-aware-gmm-p3-implementation-plan.md` 已生成。
- P3 执行边界：实现 `PredictionInteractionContext`、UFOMap 静态只读邻域查询、predictor 重权重/轻量修正、Ldop adapter 和回归测试；不实现 P4 planner-facing 类 ROG-Map builder。
```

Update `findings.md` with implementation facts after code exists:

```markdown
- P3 静态地图约束通过 `PredictionMapQuery` 抽象读取 UFOMap；动态目标交互继续来自 tracker/predictor 的稳定目标状态、尺寸和预测分支，不使用 UFOMap dynamic candidate 查询反推未来动态风险。
- `DynamicObjectPredictor` 应用交互结果时调整 branch probability，并把局部 correction hint 平滑成速度/减速/方向控制 profile 后重新 rollout 分支；不逐点硬改轨迹，也不缩小 `model_covariance`。
- `PredictionBranchInteraction` 保留连续证据和 `PredictionBranchRiskLevel` 离散摘要；P4 可用 `Low/Medium/High/Critical` 快速分层，再结合连续代价、TTC、最近障碍距离等计算 planner-facing 风险。
```

Update `progress.md` with verification results:

```markdown
## 2026-05-19 P3 交互感知预测实现

- 已实现 `PredictionInteractionContext`、UFOMap 静态只读邻域查询、Ldop 查询 adapter、predictor 概率重权重、平滑控制重新 rollout、分支级查询预算和 `branch_risk_level` 内部风险摘要。
- 已验证：`catkin build ldop --no-status`、`catkin run_tests ldop --no-status`、`catkin_test_results /home/st/ldop_ws/build/ldop`。
```

- [ ] **Step 2: Static diff hygiene**

Run from `/home/st/ldop_ws/src/LDOP`:

```bash
git diff --check
```

Expected: no whitespace errors.

- [ ] **Step 3: Inspect marker namespace safety**

Run:

```bash
rg -n "dynamic_prediction_samples|dynamic_prediction_uncertainty|diagnostic" src include test config docs/superpowers/specs/2026-05-19-dynamic-object-interaction-aware-gmm-p3-design.md
```

Expected:
- Any `dynamic_prediction_samples` or `dynamic_prediction_uncertainty` hits are only in regression tests or docs saying they remain absent.
- No new default diagnostic publisher or marker namespace is introduced for P3.

- [ ] **Step 4: Final package verification**

Run from `/home/st/ldop_ws`:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

Expected: all commands succeed and test results show no failures.

## Self-Review Record

- Spec coverage: this plan maps the P3 design to concrete files and tasks for target-target interaction, static UFOMap occupied/near-obstacle evidence, free-corridor prior, probability reweighting, bounded correction hints, branch-level query budgets, predictor integration, `Ldop` adapter wiring, YAML params, and tests.
- P4 boundary: the plan keeps `PredictionInteractionContext` as a shared scene-evidence layer and leaves planner-facing class-ROGMAP/risk-map building outside this implementation.
- Type consistency: `PredictionMapQuery`, `PredictionInteractionContext`, `PredictionInteractionResult`, and `PredictionInteractionDiagnostics` are defined before predictor and `Ldop` tasks reference them.
- Repository consistency: ROS params remain in `DynamicObjectPredictorParams`, loaded through three-argument `pnh_.param()` calls, and converted through `buildPredictorConfig()`.
- Verification coverage: the plan requires focused context tests, predictor regression tests, full package build/test, `catkin_test_results`, and `git diff --check`.
