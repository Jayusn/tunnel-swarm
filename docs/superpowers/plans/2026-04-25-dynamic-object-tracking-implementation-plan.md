# LDOP Dynamic Object Tracking Implementation Plan

## 输入

- 设计文档：`docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md`
- 当前代码基线：`DynamicObjectClusterer` 已输出帧内 `ldop/DynamicObjectArray` 和 `/ldop/dynamic_object_markers`，`DynamicObjectTracker` 与多模型 Kalman 组件尚未落地

## 目标

把第二阶段从“当前帧动态聚类输出”推进到“稳定跟踪输出”：

- `DynamicObjectClusterer` 改为输出内部 `DynamicObjectDetection[]` 和当前帧检测框 marker
- `DynamicObjectTracker` 新增并输出稳定 `ldop/DynamicObjectArray`
- 引入 `MultiModelKalmanFilter`，第一版固定 `CA3D`
- 从 `DynamicObject.msg` 中移除预测数组
- 增加 `/ldop/dynamic_track_markers`

## 实施顺序

### 阶段 1：接口与消息契约

目标：先把对外/跨模块契约改成设计文档要求的形态，为后续 tracker 接入腾出接口。

涉及文件：

- `msg/DynamicObject.msg`
- `include/ldop/utils.h`
- `include/ldop/dynamic_object_clusterer.h`
- `src/dynamic_object_clusterer.cpp`

任务：

1. 从 `DynamicObject.msg` 移除 `prediction_time_offsets` 和 `predicted_positions`
2. 在 `utils.h` 增加 `BoundingBox3D`、`DynamicObjectDetection`、`TrackHistorySample`
3. 调整 `DynamicObjectClustererFrameResult`，使其携带 detection 列表和检测框 marker，而不是把帧内 `DynamicObjectArray` 当作正式输出契约
4. 保持 clusterer 当前 `/ldop/dynamic_object_markers` 样式不变：`dynamic_object_bbox`、`LINE_LIST`、橙色、每帧 `DELETEALL`
5. 保留 clusterer 内部从动态点生成 AABB 的现有逻辑，只把输出载体改成 detection

检查点：

- `DynamicObject.msg` 改完后能重新生成消息代码
- clusterer 单独仍能输出检测框 marker
- clusterer 测试按新接口编译通过

### 阶段 2：多模型 Kalman 组件

目标：先把 track 内部滤波器能力独立出来，避免 tracker 主流程里堆矩阵细节。

涉及文件：

- `include/ldop/multi_model_kalman_filter.h`
- `src/multi_model_kalman_filter.cpp`
- `test/test_multi_model_kalman_filter.cpp`

任务：

1. 在 `multi_model_kalman_filter.{h,cpp}` 内实现 `MotionModelType`、`MotionModel` 接口、`CA3D` 模型、filter factory
2. 实现线性 KF 的 `predict()` / `update()`
3. 实现 LDOT 语义下的 adaptive Q/R：
   `innovation_buffer`、`C_gamma`、`EMA`、`min_noise_ratio`
4. 实现 cov limit 和 `prediction_cov_multiplier`
5. 暴露 tracker 需要的最小接口：
   `setDt()`、`predict()`、`update()`、`state()`、`covariance()`、`position()`、`velocity()`

检查点：

- `CA3D` 状态维度/观测维度正确
- `predict(dt)`、`update(position)` 数值行为符合测试预期
- adaptive Q/R 和 cov limit 在窗口边界条件下不崩

### 阶段 3：Tracker 主流程

目标：实现稳定 track 管理、Hungarian 关联、coasting、history、稳定输出。

涉及文件：

- `include/ldop/dynamic_object_tracker.h`
- `src/dynamic_object_tracker.cpp`
- `test/test_dynamic_object_tracker.cpp`

任务：

1. 定义 `TrackState`，内部持有 `MultiModelKalmanFilter`
2. 实现 detection 到 track 的主流程：
   `predict -> build cost matrix -> gate -> simplified Hungarian -> update/create/coast/delete`
3. 使用 LDOT 当前语义下的简化 Hungarian：
   行列归约后，从零候选中按原始代价选不冲突匹配
4. history 固定写入 `TrackHistorySample(stamp, position, velocity, matched)`
5. 第一版不实现主动重复轨迹清理
6. `DynamicObjectArray` 由 tracker 生成，`id` 为稳定 track id，`velocity` 来自滤波状态

检查点：

- 单目标连续运动保持稳定 `id`
- 两目标可分离场景下不串轨
- gate 外检测会新建轨迹
- coasting 轨迹在阈值内保留、超阈值删除

### 阶段 4：轨迹可视化与节点集成

目标：让 `Ldop` 正式接入 tracker，并补齐检测/跟踪两套 marker 发布链路。

涉及文件：

- `include/ldop/ldop.h`
- `src/ldop.cpp`
- `config/ldop.yaml`

任务：

1. 在 `Ldop` 中持有 `DynamicObjectTracker`
2. 调整 `processFrame()`：
   `UfomapMapper -> DynamicObjectClusterer -> DynamicObjectTracker -> publish`
3. `ProcessedFrame` 同时携带：
   `dynamic_object_markers_msg`、`dynamic_track_markers_msg`、`dynamic_objects_msg`
4. 新增 `dynamic_track_markers_topic`
5. 实现 `/ldop/dynamic_track_markers`：
   `LINE_STRIP` 历史线 + `SPHERE` 头部，coasting 时 alpha 降低

检查点：

- `/ldop/dynamic_object_markers` 仍是当前帧检测框
- `/ldop/dynamic_objects` 已切换为 tracker 输出
- `/ldop/dynamic_track_markers` 与检测框可同时显示

### 阶段 5：参数接入与测试收尾

目标：把配置、构建、测试收口到可验证状态。

涉及文件：

- `config/ldop.yaml`
- `CMakeLists.txt`
- `test/test_dynamic_object_clusterer.cpp`
- `test/test_dynamic_object_tracker.cpp`
- `test/test_multi_model_kalman_filter.cpp`

任务：

1. 在 `ldop.yaml` 中增加全部 `tracking_*` 参数和 `dynamic_track_markers_topic`
2. 在 tracker / filter 构造阶段完成参数读取与合法化
3. 更新 `CMakeLists.txt`，加入新源文件、测试目标、消息改动后的依赖
4. 修正现有 clusterer 测试与新接口不一致的问题
5. 新增 tracker / filter 单测，并补齐集成层测试

检查点：

- `catkin build ldop --no-status`
- `catkin run_tests ldop --no-status`
- `catkin_test_results /home/st/ldop_ws/build/ldop`

## 推荐实施粒度

建议按下面的提交/检查节奏推进，而不是一次性大改：

1. 消息与 clusterer 接口
2. Kalman 组件
3. Tracker 主流程
4. Ldop 集成与 marker
5. 参数与测试收尾

这样每一段都能独立编译和回归，定位问题更直接。

## 主要风险

- `DynamicObject.msg` 改动会触发消息重新生成，相关代码和测试会一起受影响
- clusterer 现有输出语义要从“正式对象输出”降为“内部 detection + 检测框”，接口改动面不小
- Hungarian、adaptive Q/R、cov limit 同时接入时，数值问题和单测边界很容易互相干扰
- `test/test_dynamic_object_clusterer.cpp` 已知存在接口滞后，需要在实施中顺手收敛

## 延后事项

这些不进入本次实施计划：

- 独立预测消息与预测话题实现
- 目标类别字段
- CV/CTRA/分类切换
- 主动重复轨迹清理
- 规划风险地图、ESDF、动态膨胀
