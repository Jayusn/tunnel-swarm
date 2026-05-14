# Dynamic Object Classification Multimodel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在现有 LDOP 动态目标 tracker 中补齐对象类别分类、类别证据积分、短时遮挡保护，以及 `Human/Vehicle/UAV/Other` 到 `CA2D/CTRA/CA3D/CV3D` 的 Kalman 模型切换。

**Status:** 已完成。用户已在 2026-05-14 确认第二阶段完成；后续工作切入第三阶段短时轨迹预测。

**Architecture:** `DynamicObjectClusterer` 只把现有 `Fragment::point_count` 传给内部 detection；`DynamicObjectTracker` 继续作为分类、轨迹生命周期、模型切换和输出合同的唯一拥有者。KF/EKF 底座保持现有 `MotionModel` / `KalmanFilterBase` 边界，只增加“用迁移后的完整状态初始化新 filter”的入口。

**Tech Stack:** ROS1 Noetic、catkin、C++20、Eigen、现有 `ldop_core`、自定义 ROS message。

---

## 文件结构

- 修改 `msg/DynamicObject.msg`：新增类别常量和 `uint8 object_class`。
- 修改 `include/ldop/utils.h`：给 `DynamicObjectDetection` 增加 `point_count`。
- 修改 `src/dynamic_object_clusterer.cpp`：`fragmentToDetection()` 复制 `Fragment::point_count`。
- 修改 `include/ldop/multi_model_kalman_filter.h` 和 `src/multi_model_kalman_filter.cpp`：增加按完整状态初始化 filter 的接口。
- 修改 `include/ldop/dynamic_object_tracker.h` 和 `src/dynamic_object_tracker.cpp`：新增类别枚举、分类参数、证据积分、临时合并/分离保护、模型切换、输出类别。
- 修改 `config/ldop.yaml`：新增分类参数和 `kalman_filter/...` 分组参数，同时保留已有 `tracking_*` 生命周期/关联参数。
- 修改 `docs/note/dynamic_object_tracker_数学逻辑说明.md`、`docs/note/dynamic_object_tracker_执行流程说明.md`、`docs/note/LDOP_design.md`：同步目标分类和模型切换语义。
- 更新 `task_plan.md`、`findings.md`、`progress.md`：记录计划和实施结果。

---

### Task 1: 消息和内部 detection 合同

**Files:**
- Modify: `msg/DynamicObject.msg`
- Modify: `include/ldop/utils.h`
- Modify: `src/dynamic_object_clusterer.cpp`

- [x] **Step 1: 扩展 ROS 输出消息**

在 `msg/DynamicObject.msg` 顶部加入：

```text
uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4
```

在 `velocity` 后加入：

```text
uint8 object_class
```

- [x] **Step 2: 扩展内部 detection**

在 `DynamicObjectDetection` 中加入：

```cpp
// 当前 detection 中的点数只服务于 tracker 的合并/分离保护；
// 它不进入对外 ROS 消息，避免把聚类实现细节固化到公共接口。
std::size_t point_count{0U};
```

- [x] **Step 3: 从 clusterer 传递点数**

在 `fragmentToDetection()` 中写入：

```cpp
detection.point_count = fragment.point_count;
```

- [x] **Step 4: 验证消息生成入口**

Run: `catkin build ldop --no-status`

Expected: 消息重新生成，`ldop_core` 和 `ldop_node` 编译成功。

---

### Task 2: filter 支持完整状态迁移

**Files:**
- Modify: `include/ldop/multi_model_kalman_filter.h`
- Modify: `src/multi_model_kalman_filter.cpp`

- [x] **Step 1: 声明完整状态初始化接口**

在 `KalmanFilterBase` 的 public 区域加入：

```cpp
void initializeState(const Eigen::VectorXd& state);
```

- [x] **Step 2: 实现完整状态初始化**

在 `KalmanFilterBase` 中实现：

```cpp
void KalmanFilterBase::initializeState(const Eigen::VectorXd& state) {
  if (state.size() != state_dim_ || !isFiniteVector(state)) {
    throw std::invalid_argument("initial state must be finite and match model dimension");
  }

  state_ = state;
  covariance_ = model_->getInitCovP();
  process_noise_ = model_->getProcessNoiseQ();
  measurement_noise_ = model_->getMeasNoiseR();
  innovation_buffer_.clear();
  initialized_ = true;
  model_->normalizeYaw(state_);
  clampCovariance();
}
```

- [x] **Step 3: 验证**

Run: `catkin build ldop --no-status`

Expected: 编译通过，现有 `initialize()` 行为不变。

---

### Task 3: tracker 参数和分类状态

**Files:**
- Modify: `include/ldop/dynamic_object_tracker.h`
- Modify: `src/dynamic_object_tracker.cpp`
- Modify: `config/ldop.yaml`

- [x] **Step 1: 增加内部类别枚举**

在 tracker 头文件中加入：

```cpp
enum class ObjectClass : std::uint8_t {
  Unknown = 0U,
  Human = 1U,
  Vehicle = 2U,
  Uav = 3U,
  Other = 4U,
};
```

- [x] **Step 2: 增加分类配置**

在 `DynamicObjectTrackerConfig` 中加入：

```cpp
std::size_t classification_start_frame{10U};
double classification_score_decay{0.85};
double classification_score_increment{1.0};
double classification_confirm_score{3.0};
double classification_switch_margin{1.5};
double classification_size_change_ratio{0.5};
double classification_point_count_change_ratio{0.5};
std::size_t classification_size_change_confirm_frames{5U};
std::vector<double> classify_human_threshold{1.1, 0.8};
std::vector<double> classify_vehicle_threshold{1.5, 0.7};
std::vector<double> classify_uav_threshold{0.6, 1.5};
```

- [x] **Step 3: 增加轨迹分类状态**

在 `TrackState` 中加入：

```cpp
ObjectClass object_class{ObjectClass::Unknown};
std::array<double, 4U> classification_scores{0.0, 0.0, 0.0, 0.0};
geometry_msgs::Vector3 max_observed_size;
std::size_t last_point_count{0U};
std::size_t large_size_counter{0U};
std::size_t small_size_counter{0U};
```

- [x] **Step 4: 参数读取**

`loadParameters()` 读取 `classification_*`、`classify_*` 和 `kalman_filter/...`；未使用的旧顶层 CA 参数不再保留兼容入口。

- [x] **Step 5: 配置文件**

`config/ldop.yaml` 增加分类参数和 `kalman_filter/...` 分组；不再添加 `classification_interval_sec`、`classification_stable_confirmations` 或 `classify_xy_distance_threshold`。

---

### Task 4: 分类证据、遮挡冻结和模型切换

**Files:**
- Modify: `src/dynamic_object_tracker.cpp`
- Modify: `include/ldop/dynamic_object_tracker.h`

- [x] **Step 1: 新轨迹默认 UNKNOWN + CV3D**

`createTrack()` 使用：

```cpp
TrackState track(next_track_id_++, MotionModelType::CV3D, config_.filter_config);
track.object_class = ObjectClass::Unknown;
track.max_observed_size = detection.bbox.size;
track.last_point_count = detection.point_count;
```

- [x] **Step 2: 分类发生在 Kalman update 前**

`updateMatchedTrack()` 的顺序调整为：

```text
prepare size/point protection
if matched hits reached classification_start_frame:
  classify reliable observation
  update classification scores
  switch filter when confirmed class changes
filter->update()
smooth bbox size
update lifecycle counters
```

- [x] **Step 3: 临时合并/分离保护**

实现尺寸比例和点数比例检测。临时状态返回稳定尺寸 `max_observed_size`，并冻结类别证据；持续超过确认帧数后接受新尺寸。

- [x] **Step 4: 证据积分**

可靠匹配帧对四类分数做 decay + increment。最高分达到 `classification_confirm_score` 且领先第二名至少 `classification_switch_margin` 后确认类别。

- [x] **Step 5: 模型切换**

类别到模型映射：

```text
Unknown -> keep current model
Human   -> CA2D
Vehicle -> CTRA
UAV     -> CA3D
Other   -> CV3D
```

切换时从旧 filter 读位置/速度，构造新模型状态，调用 `initializeState()`，并保留 track id、bbox、history、hits、missed_frames 和分类分数。

- [x] **Step 6: coasting 冻结**

`coastTrack()` 不修改 `object_class` 和 `classification_scores`。短时漏检只更新生命周期和历史样本。

---

### Task 5: 输出、文档和验证

**Files:**
- Modify: `src/dynamic_object_tracker.cpp`
- Modify: `docs/note/dynamic_object_tracker_数学逻辑说明.md`
- Modify: `docs/note/dynamic_object_tracker_执行流程说明.md`
- Modify: `docs/note/LDOP_design.md`
- Modify: `task_plan.md`
- Modify: `findings.md`
- Modify: `progress.md`

- [x] **Step 1: 输出类别**

`buildOutput()` 写入：

```cpp
object.object_class = toRosObjectClass(track->object_class);
```

- [x] **Step 2: 文档同步**

文档说明：

```text
LDOP 不移植 LDOT 的动静态投票分类；UFOMap 已负责动静态分割。
本轮目标分类只用于 human/vehicle/uav/other 和运动模型选择。
类别稳定使用证据积分；coasting 和临时合并/分离帧冻结类别证据。
```

- [x] **Step 3: 验证**

Run:

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
git diff --check
```

Expected: 构建通过；若当前包没有测试目标，`catkin run_tests` 仍应完成且 `catkin_test_results` 不报告失败。
