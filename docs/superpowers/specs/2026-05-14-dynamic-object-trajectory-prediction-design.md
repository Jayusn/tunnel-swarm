# LDOP 动态目标轨迹预测设计

## 状态

本文档设计 LDOP 第三阶段中本轮要实现的基础预测能力：基于 tracker 稳定目标输出，按当前运动模型做基准轨迹外推，发布独立预测消息和 RViz 可视化。

本文档不替代第二阶段 tracker 设计。tracker 仍负责动态目标检测后的稳定 ID、类别、运动模型、模型原生状态和协方差维护；predictor 只消费 tracker 已经构造好的内部预测输入，不重新处理点云，不修改 tracker 当前消息语义，也不把未来预测回填到 `DynamicObject.msg`。

## 目标

- 新增独立 `DynamicObjectPredictor` 模块。
- 消费 `DynamicObjectTrackerFrameResult::prediction_inputs`。
- 对 tracker 稳定输出的全部动态目标做 1.0 秒短时预测。
- 每个目标输出一个预测分支，表示不额外假设意图、仅按当前运动模型做基准轨迹外推。
- 预测采样点保留模型原生状态和模型原生协方差，语义与 `DynamicObject.msg` 的 `model_state/model_covariance` 一致，只是表示未来采样时刻。
- 发布独立预测话题和独立预测 marker。
- 为后续航向/意图多分支预测、交互预测和规划风险层预留稳定接口。

## 非目标

- 不实现航向/意图多分支预测。
- 不实现目标间交互或目标-地图交互修正。
- 不实现闭环误差评估或概率反馈。
- 不新增目标数量限制。
- 不为低速或近似静止目标单独设置退化分支；预测输入已经是 tracker 确认的动态目标，速度大小只是当前状态的一部分。
- 不读取 tracker filter 内部自适应 `Q/R` 状态。

## 命名

本轮能力称为“轨迹外推”，不称为“线性外推”。原因是 predictor 按 `motion_model_type` 传播模型原生状态：`CV3D/CA2D/CA3D` 可能表现为直线、近似线性或加速度轨迹，`CTRA` 会保留速度、加速度、航向和角速度，因此可以产生转弯曲线。统一称为轨迹外推更准确。

## 架构

主流程增加 predictor，顺序为：

```text
UFOMap 动静态分割
  -> DynamicObjectClusterer 当前帧检测
  -> DynamicObjectTracker 稳定目标跟踪
  -> DynamicObjectPredictor 轨迹外推预测
  -> Ldop::publishFrame() 统一发布
```

`DynamicObjectPredictor` 构造时读取预测相关 ROS 参数，提供单一主流程入口：

```cpp
DynamicObjectPredictorFrameResult predict(
    const std_msgs::Header& header,
    const std::vector<TrackPredictionInput>& inputs);
```

返回结构与现有 clusterer/tracker 风格一致：

```cpp
struct DynamicObjectPredictorTimingStats {
  double predict_dynamic_objects_ms{0.0};
  double build_prediction_markers_ms{0.0};
};

struct DynamicObjectPredictorFrameResult {
  ldop::DynamicObjectPredictionArray predictions_msg;
  visualization_msgs::MarkerArray prediction_markers_msg;
  DynamicObjectPredictorTimingStats timing;
};
```

`Ldop::processFrame()` 在 tracker 后调用 predictor，把预测消息和 marker 放进 `ProcessedFrame`；`Ldop::publishFrame()` 统一发布。

## 消息

新增 4 个消息：

```text
DynamicObjectPredictionArray.msg
DynamicObjectPrediction.msg
DynamicObjectPredictionBranch.msg
DynamicObjectPredictionPoint.msg
```

数组消息：

```text
std_msgs/Header header
DynamicObjectPrediction[] predictions
```

单目标预测消息：

```text
uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4

uint8 MOTION_MODEL_CA2D=0
uint8 MOTION_MODEL_CA3D=1
uint8 MOTION_MODEL_CV3D=2
uint8 MOTION_MODEL_CTRA=3

uint32 id
geometry_msgs/Vector3 size
uint8 object_class
uint8 motion_model_type
bool matched_in_current_frame
DynamicObjectPredictionBranch[] branches
```

`motion_model_type` 是解释所有采样点 `model_state/model_covariance` 布局的键。第一版同一个目标的所有预测分支都使用同一个运动模型类型。

类别和运动模型常量与 `DynamicObject.msg` 保持一致，避免预测消息消费者维护第二套数字映射。

预测分支消息：

```text
uint8 BEHAVIOR_BASELINE=0
uint8 BEHAVIOR_LONGITUDINAL=1
uint8 BEHAVIOR_VERTICAL=2
uint8 BEHAVIOR_TURNING=3
uint8 BEHAVIOR_LATERAL=4

uint8 behavior_type
float64 behavior_value
float64 probability
DynamicObjectPredictionPoint[] points
```

`BEHAVIOR_BASELINE` 只用于本轮基准轨迹外推，表示不额外假设意图。后续多意图或交互预测不再使用该行为类型，而是使用纵向、垂向、转向或横向等明确行为族。

`behavior_value` 是 `[-1.0, 1.0]` 的归一化有符号行为强度，不承载真实物理单位。第一版固定为 `0.0`。

预测采样点消息：

```text
duration time_from_start
float64[] model_state
float64[] model_covariance
float64 influence_weight
```

`model_state` 和 `model_covariance` 的语义与 `DynamicObject.msg` 完全一致，只是表示未来采样时刻的预测状态。`model_covariance` 仍按 row-major 展开，长度为 `len(model_state)^2`。

## 参数

新增参数：

```yaml
prediction_horizon: 1.0
dynamic_predictions_topic: /ldop/dynamic_object_predictions
dynamic_prediction_markers_topic: /ldop/dynamic_prediction_markers
```

复用参数：

```yaml
tracking_default_dt: 0.1
```

predictor 自己读取 `tracking_default_dt` 作为预测采样间隔。这里是参数名复用，不通过 `Ldop` 把 tracker 的内部参数值注入 predictor，从而保持模块边界稳定。

不新增这些参数：

```text
prediction_dt
prediction_max_objects
prediction_min_speed
prediction_influence_full_time
prediction_min_influence_weight
```

尾部最小影响权重固定为 `0.3`。

## 轨迹外推

predictor 对每个 `TrackPredictionInput` 生成一个预测分支：

```text
behavior_type = BEHAVIOR_BASELINE
behavior_value = 0.0
probability = 1.0
```

按 `motion_model_type` 创建对应 `MotionModel`：

- `CA2D`：水平面 constant acceleration，z 保持。
- `CA3D`：三维 constant acceleration。
- `CV3D`：三维 constant velocity。
- `CTRA`：带速度、加速度、航向和角速度的转弯模型。

从 tracker 输入开始：

```text
state0 = TrackPredictionInput.model_state
P0 = TrackPredictionInput.model_covariance
```

每一步：

```text
F = motion_model.getTransitionF(state_before)
state_next = motion_model.stateTransition(state_before)
Q = motion_model.getProcessNoiseQ()
P_next = F * P * F^T + Q
```

`P0` 来自 tracker 当前输出的 `model_covariance`，已经包含 tracker 当前滤波和自适应噪声估计后的结果。未来传播新增过程噪声使用运动模型基础 `Q`，即 `MotionModel::getProcessNoiseQ()`；第一版不读取 tracker filter 内部自适应 `process_noise_`，也不扩大 `TrackPredictionInput` 去导出 filter 内部噪声状态。

CTRA 的协方差传播使用预测前状态处的雅可比 `F`，与当前 EKF predict 语义一致。

## 影响权重

`model_covariance` 保持模型真实传播，不为了规划影响做人为缩小。规划和 RViz 影响用 `influence_weight` 表达：

```text
if time_from_start <= prediction_horizon * 0.5:
  influence_weight = 1.0
else:
  influence_weight 从 1.0 线性衰减到 0.3
```

`0.3` 写死，不新增参数。这样远端预测仍保留风险提示，但不会因为远端协方差增大而把规划可通行区域过度压缩。

## 可视化

预测 marker 话题为 `/ldop/dynamic_prediction_markers`，可通过 `dynamic_prediction_markers_topic` 配置。

MarkerArray 首个 marker 使用 `DELETEALL`，延续当前动态目标 marker 和轨迹 marker 风格。

每个目标输出：

- 一条预测轨迹线：`LINE_STRIP`，连接每个采样点的中心位置 `model_state[0..2]`。
- 多个不确定性椭球：每个采样点一个 `SPHERE` marker。

椭球来自 `model_covariance` 的前三维位置协方差子块。对 3x3 位置协方差做特征分解，使用主轴方向作为 marker 姿态，使用 1σ 作为椭球尺度：

```text
scale_i = 2 * sqrt(lambda_i)
```

RViz `SPHERE.scale` 表示直径，因此乘以 2。若数值误差产生极小负特征值，裁剪到 0。

marker alpha 乘以采样点 `influence_weight`。漏检延续但尚未删除的稳定轨迹继续预测，但 marker 整体透明度降低，以表达当前帧未匹配。

## 内部契约

`TrackPredictionInput` 来自 tracker 内部稳定输出，不作为外部不可信输入。predictor 不对状态维度、协方差方阵性和有限性做外部输入式重复兜底；实现中用 assert 或清晰的状态维度契约表达不变量。

predictor 不额外裁剪目标数量。是否稳定发布、是否进入预测输入，由 tracker 的 `min_hits_to_publish` 等规则控制。

## 验证

因为本轮新增消息、C++ 模块、参数和发布链路，实现后至少执行：

```bash
catkin build ldop --no-status
```

如果实现中增加测试，再执行：

```bash
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

## 文件范围

预计修改：

- `msg/DynamicObjectPredictionArray.msg`
- `msg/DynamicObjectPrediction.msg`
- `msg/DynamicObjectPredictionBranch.msg`
- `msg/DynamicObjectPredictionPoint.msg`
- `include/ldop/dynamic_object_predictor.h`
- `src/dynamic_object_predictor.cpp`
- `include/ldop/ldop.h`
- `src/ldop.cpp`
- `config/ldop.yaml`
- `CMakeLists.txt`

可选同步：

- `docs/note/LDOP_design.md`
- predictor 执行流程说明和数学逻辑说明。
