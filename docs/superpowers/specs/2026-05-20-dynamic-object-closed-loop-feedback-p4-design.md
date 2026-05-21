# LDOP 动态目标闭环概率反馈 P4 设计

## 状态

本文档细化 LDOP 第三阶段里的轨迹预测内部 P4：在已经完成的 P1 轨迹外推、P2 航向/意图 GMM 多分支预测和 P3 交互感知重权重/轻量修正之上，引入预测残差、NIS 和 likelihood 闭环反馈，使同一目标的下一帧预测概率和未来过程噪声尺度能根据上一轮预测是否解释当前观测而调整。

2026-05-21 状态补充：本文档对应的 P4 predictor 内部在线反馈已经实现并通过本地验证；因此 LDOP 第三阶段动态目标短时轨迹预测内容已经完成。离线多 horizon evaluator 和 LDOP 第四阶段 planner-facing 风险地图仍作为后续工作独立推进。

当前已确认边界：

- P4 默认启用在线反馈，并提供参数关闭。
- 反馈状态按 `object_id` 维护，只影响同一目标的短期预测，不做类别级或全局泛化。
- 反馈同时调整分支概率和过程噪声尺度。
- 分支概率可以增大或降低；过程噪声尺度允许在 `[min, max]` 内变化，默认下界为 0.3，避免预测 Q 被压到 0。
- P4 不修改 tracker 当前状态，不改变 `DynamicObject.msg`，不新增默认 ROS 诊断消息。
- 诊断首版保留在 C++ 内部统计、测试可读结构和 `verbose` 日志中。

## 目标

- 在 `DynamicObjectPredictor` 内部维护上一轮预测缓存和短期反馈状态。
- 使用当前 tracker 输出按目标 ID 对齐上一轮预测，计算每个预测分支的 residual、NIS 和 Gaussian likelihood。
- 将上一轮分支解释当前观测的能力反馈到下一轮同语义 mode 的 score bias。
- 当某个 mode 长期 NIS 偏大时，放大该 mode 的未来过程噪声尺度，避免分支过度自信。
- 当 NIS 偏低或长期稳定时，过程噪声尺度可缓慢降低到固定下界 0.3，用于校准 predictor 自定义的未来 Q。
- 保持 P2 softmax/剪枝和 P3 interaction reweighting 的既有流程，P4 只提供短期先验修正。

## 非目标

- 不实现类别/行为族级全局校准器。
- 不新增 `/ldop/dynamic_object_predictions` 字段。
- 不新增独立诊断话题或诊断 ROS msg。
- 不把 feedback 写回 tracker，不改变当前帧 `model_state` 或 `model_covariance`。
- 不用反馈缩小 tracker 传入的当前 `model_covariance`；P4 只校准 predictor 未来 rollout 使用的 `Q_future` 尺度，并保留非零下界。
- 不引入学习式概率校准或离线训练。

## 架构边界

P4 直接收敛在 `DynamicObjectPredictor` 内部。predictor 从无状态外推器变为带短期反馈记忆的预测器：

```text
DynamicObjectTracker
  -> TrackPredictionInput[]
  -> DynamicObjectPredictor::predict()
       1. consume previous feedback cache
       2. build per-object/per-mode feedback hints
       3. generate P2 modes with feedback score bias
       4. rollout with feedback process-noise scale
       5. apply P3 interaction context
       6. store current prediction snapshot for next frame
  -> /ldop/dynamic_object_predictions
  -> /ldop/dynamic_prediction_markers
```

`Ldop` 仍只负责串接 tracker、predictor 和 UFOMap adapter；它不理解 NIS、likelihood 或 feedback state。tracker 仍只发布当前目标状态和历史快照，不接收预测反馈。

因为 predictor 需要读取并更新内部缓存，`DynamicObjectPredictor::predict()` 应改为非 `const`。当前 LDOP 只有一个处理线程调用 predictor，首版不需要给 predictor 内部 feedback cache 额外加锁；如果后续出现多线程调用，再把状态保护提升为显式互斥边界。

## 内部状态

新增 predictor 内部短期状态，不单独抽出新模块：

```text
PredictionFeedbackParams
PredictionFeedbackConfig
PredictionFeedbackBranchKey
PredictionFeedbackBranchState
PredictionFeedbackObjectState
PredictionFeedbackFrameDiagnostics
```

`PredictionFeedbackBranchKey` 用行为族和行为强度近似桶匹配 mode：

```text
behavior_type
behavior_value_bucket
```

匹配不依赖 branch index。这样 P2/P3 剪枝后分支顺序变化时，上一轮 feedback 仍能落回语义相近的 mode。

`PredictionFeedbackObjectState` 按 `object_id` 保存：

```text
last_stamp
last_prediction_snapshot
branch_states keyed by PredictionFeedbackBranchKey
missed_or_unmatched_frame_count
```

上一轮预测快照只保存 P4 必要字段：

```text
behavior_type
behavior_value
probability
time_from_start[]
position_mean[]
position_covariance_3x3[]
```

不保存完整 ROS msg 的深层对象，不复制 interaction diagnostics，也不保存 UFOMap 查询结果。P4 只需要用上一轮分支对齐当前观测。

`PredictionFeedbackBranchState` 保存平滑反馈：

```text
smoothed_log_likelihood
smoothed_nis
probability_score_bias
process_noise_scale
last_update_stamp
```

`process_noise_scale` 的默认下界是 `0.3`。它表达 predictor 对未来 rollout 过程噪声 `Q_future` 的闭环校准，不是新的 tracker 状态噪声，也不改变 tracker 传入的当前 `model_covariance`。

## 反馈计算

当当前帧输入目标 `i` 在上一轮缓存中存在同一 `object_id`，且当前目标 `matched_in_current_frame == true` 时，按当前观测时间对齐上一轮预测：

```text
dt = current_stamp - previous_prediction_stamp
sample = branch point whose time_from_start is closest to dt
```

对每条上一轮分支 `z` 计算：

```text
nu_z = observed_position - predicted_position_z
S_z = predicted_position_covariance_z + observed_position_covariance
NIS_z = nu_z^T S_z^-1 nu_z
L_z = N(nu_z; 0, S_z)
```

`observed_position` 来自当前 `TrackPredictionInput::model_state[0..2]`。`observed_position_covariance` 来自当前 `model_covariance` 的前三维位置子块。这样 P4 使用 tracker 已有不确定性，不新增一套测量噪声参数。

三维位置观测的期望 NIS 自由度为 3。首版以该自由度作为校准中心：

```text
nis_ratio = NIS_z / 3.0
```

若 `S_z` 维度不足、不可求逆或不是可用的正定位置协方差，该分支本轮反馈跳过。跳过表示“不更新反馈”，不做隐式数值合法化。

## 概率反馈

P4 不直接改最终发布分支概率，而是给下一轮 mode score 提供 bias：

```text
score_z =
  p2_score_z
+ feedback_probability_gain * probability_score_bias_z
```

`probability_score_bias_z` 来自上一轮 likelihood 的平滑值。观测更接近的分支得到更高 bias；长期解释不了观测的分支得到更低 bias。之后仍走现有 P2 softmax、`prediction_min_branch_probability` 过滤、`prediction_max_branches` 剪枝和归一化。

这样概率反馈是 mode prior 的短期调整，而不是绕过 P2/P3 的最终概率覆盖。P3 interaction reweighting 仍在 P2/P4 之后执行：

```text
P2 score + P4 feedback bias
  -> softmax / branch pruning
  -> rollout with feedback noise scale
  -> P3 interaction probability_scale
  -> final branch probabilities
```

## 过程噪声反馈

P4 只调整 predictor 未来 rollout 的过程噪声尺度，不改变当前 tracker 协方差：

```text
P_future = F * P_current * F^T + feedback_scale * Q_future
0.3 <= feedback_scale <= prediction_feedback_max_noise_scale
```

其中 `Q_future` 仍由 P2 的历史质量和 mode 不确定性得到：

```text
Q_future = Q_base * history_quality_scale * mode_uncertainty_scale
```

P4 只乘上额外尺度：

```text
Q_feedback =
  Q_base
* history_quality_scale
* mode_uncertainty_scale
* process_noise_scale
```

更新语义：

```text
if NIS_z > expected_nis:
  process_noise_scale moves upward
else:
  process_noise_scale can move downward

process_noise_scale is clamped to
  [0.3, prediction_feedback_max_noise_scale]
```

NIS 正常或偏小时允许 `process_noise_scale` 低于 1.0，因为 predictor 的 `Q_base` 是预测模块自己的未来过程噪声参数，不是 tracker 传入的当前协方差。下界默认 0.3，用来避免闭环把未来过程噪声压到 0，导致预测分支过度自信或数值退化。

## 参数策略

新增参数集中写在 `DynamicObjectPredictorParams` 中，并由 `loadParameters()` 使用三参数 `pnh_.param(name, target, default_value)` 导入。

建议参数：

```text
prediction_enable_feedback: true
prediction_feedback_max_age: 1.5
prediction_feedback_probability_gain: 1.0
prediction_feedback_noise_gain: 0.25
prediction_feedback_max_noise_scale: 3.0
prediction_feedback_smoothing_alpha: 0.35
prediction_feedback_behavior_value_bucket: 0.25
prediction_feedback_max_sample_time_error: 0.15
```

含义：

- `prediction_enable_feedback`：P4 主开关，关闭后行为应等价于当前 P3。
- `prediction_feedback_max_age`：上一轮预测过旧时不再用于残差更新。
- `prediction_feedback_probability_gain`：likelihood 对下一轮 mode score 的影响强度。
- `prediction_feedback_noise_gain`：NIS 偏大时过程噪声尺度上升速度。
- `prediction_feedback_max_noise_scale`：未来过程噪声尺度上限。
- `prediction_feedback_smoothing_alpha`：NIS 和 likelihood 的指数平滑系数。
- `prediction_feedback_behavior_value_bucket`：行为强度近似匹配粒度。
- `prediction_feedback_max_sample_time_error`：上一轮预测采样点与当前 `dt` 的最大允许时间偏差。

`process_noise_scale` 下界固定为 0.3，不暴露为 YAML 参数。该下界只作用于 predictor 未来 rollout 的 `Q_future`，不作用于 tracker 当前 `P_current`；固定非零下界可以避免反馈把过程噪声清零。

首版反馈缓存固定为上一轮预测快照，不暴露 `history_size` 参数；多帧 ADE/稳定性统计需要独立设计缓存语义后再增加参数。

## 异常处理

反馈更新遵循显式跳过策略：

- `prediction_enable_feedback == false`：不读取旧缓存，不写反馈 bias；仍可覆盖缓存以便重新启用后使用。
- `dt <= 0`：跳过本目标反馈。
- `dt > prediction_feedback_max_age`：旧状态过期并清理。
- 当前目标 `matched_in_current_frame == false`：不使用该目标更新残差，但可保留旧状态等待短时重关联。
- 找不到时间接近的上一轮预测点：跳过该分支反馈。
- 预测点没有前三维位置、协方差维度不足、位置子块不可用：跳过该分支反馈。
- 当前 tracker 输入没有前三维位置或位置协方差不可用：跳过该目标反馈。
- 某个分支无法得到反馈时，该分支下一轮使用默认 `score_bias = 0` 和 `process_noise_scale = 1.0`，表示回到 predictor 参数定义的基础未来 Q。

状态清理按当前输入 ID 集合和时间戳执行。连续消失或超过 `prediction_feedback_max_age` 的目标状态应删除，避免缓存无限增长。

## 诊断

首版不新增 ROS 诊断话题。`DynamicObjectPredictorFrameResult` 可扩展内部诊断结构，用于测试和 verbose 日志：

```text
feedback_evaluated_object_count
feedback_updated_branch_count
feedback_skipped_object_count
feedback_skipped_branch_count
max_nis
mean_nis
min_process_noise_scale
max_process_noise_scale
max_abs_probability_bias
expired_state_count
```

`verbose` 日志节流输出 P4 摘要，并继续保留现有 predictor timing。若后续需要在线可视化，再单独设计诊断 msg 或 marker，不在 P4 首版混入 planner-facing prediction message。

## 测试策略

新增或扩展 `test/test_dynamic_object_predictor.cpp`，优先覆盖 predictor 内部闭环行为：

- 同一目标连续两帧，当前观测靠近某个上一轮分支时，该语义 mode 下一帧概率上升。
- 当前观测远离某个上一轮分支且 NIS 偏大时，该 mode 下一帧过程噪声尺度增大，但不超过 `prediction_feedback_max_noise_scale`。
- NIS 偏低或长期稳定时，`process_noise_scale` 可逐步降低，但不低于固定下界 0.3。
- branch index 变化但 `behavior_type` 和 `behavior_value` 近似相同，feedback 仍匹配到相应 mode。
- 当前目标漏检或 `matched_in_current_frame == false` 时，不更新残差。
- 旧状态超过 `prediction_feedback_max_age` 后被清理。
- `prediction_enable_feedback == false` 时，输出与当前 P3 等价。
- P3 interaction 开启时，P4 feedback bias 先影响 P2 prior，最终概率仍经过 interaction reweighting。

验证范围：

- 参数、头文件和 C++ 实现改动后运行 `catkin build ldop --no-status`。
- 行为逻辑和测试改动后运行 `catkin run_tests ldop --no-status`，并用 `catkin_test_results /home/st/ldop_ws/build/ldop` 查看结果。

## 预期文件范围

预计修改：

- `include/ldop/dynamic_object_predictor.h`
- `src/dynamic_object_predictor.cpp`
- `test/test_dynamic_object_predictor.cpp`
- `config/ldop.yaml`

必要时同步：

- `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`
- `task_plan.md`
- `progress.md`
- `findings.md`

P4 首版不需要新增 ROS msg，不需要修改 `DynamicObjectPrediction*.msg`。
