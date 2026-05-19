# LDOP 动态目标航向/意图 GMM 多轨迹预测设计

## 状态

本文档细化 LDOP 第三阶段 P2：在已实现的基础轨迹外推之上，加入航向/意图多分支预测，输出 branch-level trajectory-level GMM，并在 RViz 中显示每条分支的中心状态轨迹。

当前代码已经按本文档的 P2 主线落地：`DynamicObjectPredictor` 直接生成多条航向/意图预测分支，旧 `BEHAVIOR_BASELINE` 枚举已移除，RViz 只显示真实保留的 GMM 分支中心状态线，不再显示协方差椭球或额外采样轨迹。

本设计基于以下既有边界：

- P1 已实现 `DynamicObjectPredictor`，并在 tracker 后接入 LDOP 主流程。
- tracker 已提供 `TrackPredictionInput`，包含稳定 ID、类别、运动模型、模型原生状态、模型原生协方差、bbox、历史轨迹和生命周期信息。
- 预测消息已经支持 `DynamicObjectPredictionBranch[]`，每条 branch 有 `behavior_type`、`behavior_value`、`probability` 和未来 `points[]`。
- P2 不重新处理点云，不修改 tracker 当前输出语义，不把未来预测写回 `DynamicObject.msg`。

## 目标

- 在 `DynamicObjectPredictor` 内部生成多条航向/意图候选分支。
- 每条分支独立 rollout 未来模型原生状态和模型原生协方差。
- 每个目标输出概率归一化的 branch-level trajectory-level GMM。
- 基于历史运动特征、类别、运动模型、历史长度和漏检状态分配 P2 初始概率。
- P2 是当前 predictor 的唯一轨迹预测路径；不再保留旧单分支回退开关。
- 在 RViz 中显示多分支中心状态轨迹。
- 不引入额外 sampled trajectories，保持 `/ldop/dynamic_object_predictions` 和可视化表达一致。

## 非目标

- 不实现目标间交互能量。
- 不实现目标与地图交互能量。
- 不实现 NIS 或预测残差闭环概率反馈。
- 不新增采样轨迹 ROS 消息字段、独立 samples 话题或 sample marker。
- 不导出 tracker 内部自适应 `process_noise_`。
- 不修改 `DynamicObject.msg`。

## 架构

P2 仍只增强 `DynamicObjectPredictor`，外部调用关系不变：

```text
DynamicObjectTracker
  -> TrackPredictionInput[]
  -> DynamicObjectPredictor::predict()
  -> /ldop/dynamic_object_predictions
  -> /ldop/dynamic_prediction_markers
```

predictor 内部数据流为：

```text
TrackPredictionInput
  -> MotionHistoryFeatures
  -> PredictionMode[]
  -> DynamicObjectPredictionBranch[]
```

`predict()` 仍是唯一主入口。P2 复杂度通过私有 helper 组织，避免向 tracker、LDOP 主流程或 ROS 消息层泄漏实现细节。

## 内部结构

`MotionHistoryFeatures` 是 predictor 内部的历史运动摘要，来自 `input.history` 和当前 `model_state`。它表达：

- 稳定航向。
- 当前速度和历史速度趋势。
- 加速度趋势。
- yaw 或曲率趋势。
- z 方向速度或高度变化趋势。
- 历史长度质量。
- 漏检延续惩罚。
- 航向或速度不稳定程度。

`PredictionMode` 是 predictor 内部的候选意图描述，转换为 ROS branch 前使用：

```text
behavior_type
behavior_value
probability
history_quality_scale
mode_uncertainty_scale
rollout_modifier
```

`rollout_modifier` 只在 predictor 内部使用，用于决定该 mode 如何改写未来候选走法。它不作为公共接口发布。

一个 `PredictionMode` 最终对应一个 GMM component，也就是一个 `DynamicObjectPredictionBranch`。当前 P2 中每个分支内部只有一条 rollout 得到的中心状态序列 `points[]`，以及每个时间点对应的 `model_covariance`；分支内部没有多条原始预测轨迹样本。`prediction_max_branches` 和 `prediction_min_branch_probability` 都作用在这些 GMM component 上，而不是作用在分支内部的时间采样点或额外轨迹样本上。

## Mode 候选集合

P2 使用完整但受控的候选集合：

```text
keep / trend-follow
longitudinal-decelerate
longitudinal-stop
turn-left
turn-right
lateral-left
lateral-right
vertical-climb
vertical-descend
vertical-hold
```

映射到现有消息时不新增枚举：

```text
keep / trend-follow          -> BEHAVIOR_LONGITUDINAL, value = 0.0 或正小值
decelerate / stop            -> BEHAVIOR_LONGITUDINAL, value < 0
turn-left / turn-right       -> BEHAVIOR_TURNING, value = +/- strength
lateral-left / lateral-right -> BEHAVIOR_LATERAL, value = +/- strength
climb / descend / hold-z     -> BEHAVIOR_VERTICAL, value = + / - / 0
```

旧单分支行为枚举已从预测分支消息中删除。P2 正常输出不提供单分支回退路径，也不提供 `prediction_enable_intent_gmm=false` 开关。

## 类别策略

候选 mode 按类别和运动模型选择：

- human：生成 `keep`、`longitudinal-decelerate`、`longitudinal-stop`、`lateral-left/right`、`turn-left/right`。人类目标允许更灵活的横向偏转和停止假设。
- vehicle：生成 `keep`、`longitudinal-decelerate`、`turn-left/right`。车辆默认不生成 lateral 分支，除非后续设计有明确道路或交互证据。
- UAV：生成 `keep`、`longitudinal-decelerate`、`turn-left/right`、`vertical-climb/descend/hold`。UAV 可以表达 2.5D 意图。
- other / unknown：以 `keep` 为主，只在历史航向不稳定、速度趋势明显或 z 方向证据明显时生成低权重候选。

分支数量受 `prediction_max_branches` 控制。候选 mode 先通过 softmax 得到分支概率，再删除低于 `prediction_min_branch_probability` 的 GMM component；如果剩余分支数仍超过 `prediction_max_branches`，按概率从高到低保留前 N 个，最后对保留分支重新归一化。

## 概率策略

P2 概率先使用启发式 logit 归一化，不做 P4 闭环递推：

```text
score_z = class_prior_z
        + history_heading_score_z
        + speed_trend_score_z
        + yaw_or_curvature_score_z
        + vertical_trend_score_z
        - short_history_penalty_z
        - missed_frame_penalty_z

probability_z = softmax(score_z)
```

含义如下：

- 历史航向越稳定，`keep/trend-follow` 概率越高。
- 速度持续下降时，`longitudinal-decelerate` 和 `longitudinal-stop` 增权。
- yaw、曲率或水平航向变化明显时，`turn-left/right` 增权。
- z 速度或高度趋势明显时，UAV 的 `vertical-climb/descend` 增权。
- 短历史和漏检延续降低尖锐判断，使分布更平缓。
- 低于 `prediction_min_branch_probability` 的 GMM 分支可丢弃；若剩余数量超过 `prediction_max_branches`，只保留概率最高的前 N 个。最终发布前重新归一化概率和为 1。

概率总和必须为 1。若所有候选被阈值过滤，保留概率最高分支并设置为 1。

## Rollout 机制

每个 mode 从同一个当前输入拷贝开始：

```text
state0 = input.model_state
P0 = input.model_covariance
```

mode 只改变未来候选走法，不修改 tracker 当前状态，也不影响其他分支。

状态布局按当前运动模型解释：

```text
CV3D: [x,y,z,vx,vy,vz]
CA2D: [x,y,z,vx,vy,ax,ay]
CA3D: [x,y,z,vx,vy,vz,ax,ay,az]
CTRA: [x,y,z,v,a,yaw,yaw_rate]
```

mode 对状态的影响保持局部和可解释：

- `keep/trend-follow`：保持现有运动模型传播。
- `longitudinal-decelerate`：沿稳定航向降低速度或加速度。
- `longitudinal-stop`：让速度在预测 horizon 内逐步趋近停止，不因当前速度小而切换算法。
- `turn-left/right`：对 CTRA 调整 `yaw_rate`；对 CV/CA 用旋转后的水平速度或加速度近似。
- `lateral-left/right`：在稳定航向法向加入小横向速度分量，主要用于 human。
- `vertical-climb/descend/hold`：调整 z 方向速度或加速度，仅用于 UAV 或 z 方向证据明显目标。

每一步仍使用现有运动模型传播：

```text
F = motion_model.getTransitionF(state_before)
state_next = modified_state_transition(state_before, mode)
P_next = F * P * F^T + Q_future
```

`modified_state_transition()` 表示在当前 mode 下对候选状态的有限改写。实现时应尽量靠近现有 `MotionModel` 传播流程，偏离基础模型的地方用中文注释说明原因。

## 未来过程噪声

P2 不导出 tracker 内部自适应 `process_noise_`。当前滤波和自适应噪声的历史影响通过 `input.model_covariance` 作为 `P0` 进入预测。

未来 rollout 使用：

```text
Q_future = Q_base * history_quality_scale * mode_uncertainty_scale
P_next = F * P * F^T + Q_future
```

`Q_base` 来自 `MotionModel::getProcessNoiseQ()`。`history_quality_scale` 表达轨迹输入质量，短历史、漏检延续、航向或速度不稳定时增大。`mode_uncertainty_scale` 表达候选假设强度，`keep` 接近 1，`decelerate/stop` 稍大，`turn/lateral/vertical` 更大但有上限。

这一区分保证：

- `P0` 表示 tracker 当前状态估计不确定性。
- `Q_future` 表示未来候选意图假设的不确定性。
- P2 不与 tracker filter 内部实现强耦合。
- P4 仍可在以后用 NIS 或残差闭环校准概率和过程噪声尺度。

## 参数

新增 predictor 参数集中写入 `DynamicObjectPredictorParams` 默认值，并由 `loadParameters()` 使用三参数 `pnh_.param(name, target, default_value)` 读取。

当前 YAML 运行参数：

```yaml
prediction_max_branches: 3
prediction_min_branch_probability: 0.05

prediction_history_window: 20
prediction_heading_stable_angle: 0.35
prediction_stop_speed: 0.2

prediction_turn_yaw_rate: 0.5
prediction_lateral_speed: 0.4
prediction_vertical_speed: 0.3

prediction_history_quality_noise_scale: 1.5
prediction_hypothesis_noise_scale: 1.3
```

参数语义：

- `prediction_max_branches` 控制每个目标最多输出多少 GMM component，不控制单个分支内部的时间采样点数量。
- `prediction_min_branch_probability` 控制低概率 GMM component 过滤，不作用于分支内的 `points[]`。
- `prediction_history_window` 控制用于航向和趋势估计的历史样本数量。
- `prediction_heading_stable_angle` 控制航向稳定性判断。
- `prediction_stop_speed` 用于判断停止倾向和 stop rollout 目标速度。
- `prediction_turn_yaw_rate`、`prediction_lateral_speed`、`prediction_vertical_speed` 控制候选分支的物理强度。
- `prediction_history_quality_noise_scale` 和 `prediction_hypothesis_noise_scale` 控制 P2 有限噪声放大，不做无界膨胀。

`DynamicObjectPredictorParams` 仍保留模块内兜底默认值；当前 `config/ldop.yaml` 显式给出运行默认值。`buildPredictorConfig()` 只做参数到运行时配置的语义转换和简单范围兜底。

## RViz 可视化

P2 marker 只输出一类：

- branch center-state line：每条分支一条中心状态轨迹线，连接该分支各未来时间点 `model_state[0..2]`，透明度按 branch probability 调整。

采样轨迹不进入 `/ldop/dynamic_object_predictions`，也不在 RViz 中额外绘制。此前仅用于可视化的 sample markers 已删除，因为它不服务实际规划或评估契约；规划器和评估器应优先直接消费 GMM branches，只有算法内部明确需要 Monte Carlo samples 时才在消费端临时派生。

## 边界情况

- 短历史：仍输出多分支，但降低尖锐概率判断，使概率更平缓。
- 漏检延续：继续预测，整体可视化透明度降低，并提高 `history_quality_scale`。
- 低速目标：不切换到独立算法；是否停止由历史速度趋势和 stop mode 概率表达。
- 类别 unknown：以保守 `keep` 为主，低权重补充分支只在运动证据明显时生成。
- 概率裁剪后空集合：保留裁剪前最高分支，概率设为 1。
- 不生成 sample markers，避免 RViz 出现不服务实际输出契约的额外轨迹线。

## 测试与验证

单元测试覆盖：

- CV3D 和 CTRA 输入能生成多分支，概率和为 1；CA2D/CA3D 走同一 `MotionModel` 构建和状态传播路径，当前通过包级构建与共享路径覆盖。
- human、vehicle、UAV、unknown 的分支集合符合类别策略。
- 短历史和漏检延续使概率更平缓，并提高未来过程噪声尺度。
- `prediction_max_branches` 裁剪后概率重新归一化。
- RViz 只生成分支中心状态线，不产生 sample marker；回归测试应防止仅可视化采样轨迹被重新引入。

构建和测试验证：

```text
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

若本轮只写文档，则至少运行：

```text
git diff --check
```

## 验收标准

- 每个稳定目标可输出多条航向/意图分支。
- 每条分支有独立未来状态、协方差、行为族、行为强度和概率。
- 概率总和归一化。
- MAP 分支通常与历史趋势一致。
- RViz 能检查多分支中心状态轨迹。
- P2 不保留旧单分支回退；消融实验通过分支数量和概率阈值控制。
- 不改变 `DynamicObject.msg`，不新增 samples 消息字段，不引入 P3/P4 责任。
