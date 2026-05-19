# 动态目标短时轨迹预测实施安排

## 目标

LDOP 第三阶段的目标是在第二阶段稳定跟踪输出之上，构建独立的动态目标短时轨迹预测能力。预测模块不重新处理原始动态点云，而是消费 tracker 已经稳定维护的目标 ID、类别、运动模型、模型原生状态及协方差和历史快照，输出未来 1-3 秒内物体中心的预测轨迹、轨迹概率分布和 RViz 可视化。

本文档已按 `docs/note/2026-05-18_intent_prediction_trajectory_innovation.md` 和当前 P2 代码事实更新。后续路线不再停留在 RAL-25 式 `MDP intent + sampled trajectory mean/std`，而是把基础轨迹外推扩展为 **闭环不确定性交互意图混合预测器**：每个目标维护多条 intent/motion mode 预测分支，每条分支保存未来采样状态、协方差和概率，整体构成 trajectory-level GMM。当前已经完成航向/意图多分支 GMM，后续重点转向交互能量和闭环概率反馈。

预测结果后续会服务 LDOP 第四阶段规划地图或动态风险层，因此 LDOP 第三阶段需要同时关注三件事：

1. 预测结果本身是否合理。
2. 预测不确定性能否随时间表达风险扩散，并且不会因为简单膨胀而压垮规划空间。
3. 预测误差能否闭环反馈到不同轨迹模式的概率和不确定性。

## 术语边界

本文使用三套不同层级的阶段名，后文不混用：

- `LDOP 第一/二/三/四阶段`：指系统总体路线，分别是静态层、动态目标跟踪、短时轨迹预测、规划风险层。
- `P1-P4`：指 LDOP 第三阶段内部的 predictor 实施里程碑。
- `D1-D4`：指轨迹预测分布表达的复杂度层级，从单预测分支到闭环校正概率。预测分支表示同一目标未来的一种候选走法，每条分支都有自己的轨迹、概率和不确定性。

术语约定：

- `trajectory-level GMM`：同一目标未来轨迹分布由多条预测分支构成，每条分支是一个 GMM component，包含一条中心状态序列和对应的时序协方差，分支概率归一化。
- `MAP trajectory`：概率最高预测分支的中心状态轨迹，适合作为单条中心预测轨迹默认显示。
- `mixture mean trajectory`：所有预测分支按概率加权后的跨分支均值轨迹，只作为统计视图，不替代多峰分布本体，也不是当前 RViz 默认显示内容。
- `NIS`：Normalized Innovation Squared，用于判断上一轮预测分支是否能解释当前观测，并据此校正分支概率或过程噪声尺度。

## 输入

预测模块的输入来自 LDOP 第二阶段 tracker，而不是原始点云：

- 稳定目标 ID。
- 当前时间戳和 frame。
- 目标类别 `object_class`。
- 当前运动模型 `motion_model_type`。
- 当前模型原生状态 `model_state`。
- 当前模型原生协方差 `model_covariance`。
- bbox 尺寸。
- 过往轨迹历史，保存各历史时刻的模型原生状态及协方差快照。
- 生命周期信息，如 `age`、`hits`、`missed_frames` 和当前帧是否匹配。

本文在可视化和风险解释层可以使用“中心轨迹”“位置协方差”等术语，但预测消息层仍保持模型原生语义：每个未来采样点保存对应运动模型自己的 `model_state` / `model_covariance`。中心位置固定由 `model_state[0..2]` 派生，位置协方差由 `model_covariance` 的前三维位置子块派生。bbox 仅提供目标尺寸，用于 marker、风险膨胀或碰撞近似，不作为运动中心来源。

当前代码已经具备 P2-P4 的重要基础：`TrackPredictionInput` 和 `DynamicObjectTrackerFrameResult::prediction_inputs` 已在 tracker 内部存在，`DynamicObjectPredictor` 已在 tracker 后接入，预测消息已经按多预测分支结构设计。后续工作应继续在这个边界之后增强 predictor，而不是回头修改 clusterer 或把预测数组塞进 `DynamicObject.msg`。

## 系统过程

轨迹预测按能力递进组织：

```text
动态目标历史状态 + 运动学特征
  -> 已实现：基于当前运动模型的单分支轨迹外推
  -> 已实现：航向/意图多分支 GMM
  -> 下一步：目标间交互与目标-地图交互能量
  -> 完整输出：trajectory-level GMM + MAP trajectory + 校准后的分支概率
  -> 闭环：预测残差/NIS 更新分支概率和不确定性
```

P1 已经建立最小可运行预测链路，P2 已经把预测输出扩展为分支级 trajectory-level GMM。P3-P4 应保持可解释、可验证，首选运动学、贝叶斯递归更新和规则型交互能量。学习方法只作为后续增强或强基线，不进入当前主线的最小实现路径。

## 输出

LDOP 第三阶段使用独立输出，不改变 LDOP 第二阶段当前跟踪消息语义：

- `/ldop/dynamic_object_predictions`：动态目标预测结果，通过参数 `dynamic_predictions_topic` 配置。
- `/ldop/dynamic_prediction_markers`：预测轨迹 RViz 可视化，通过参数 `dynamic_prediction_markers_topic` 配置。
- 每个目标输出一组预测分支：
  - 预测分支概率。
  - 未来时间采样。
  - 每个采样点的模型原生状态，中心位置由状态前三维派生。
  - 每个采样点的模型原生协方差，位置协方差或风险半径只作为下游派生量。
  - 预测分支行为族和归一化有符号行为强度。

输出解释规则：

- 当前预测分支消息只保留纵向、垂向、转向和横向等行为族；旧 `BEHAVIOR_BASELINE` 枚举已移除。
- 需要做 P1 基线消融时，用 `prediction_max_branches`、概率阈值和纵向 keep 分支控制输出，而不是恢复旧 baseline 枚举。
- `prediction_max_branches` 和 `prediction_min_branch_probability` 作用在 GMM 分支/component 层面：先过滤低概率分支，再按概率最多保留前 N 个分支，最后重新归一化。它们不控制单个分支内部的 `points[]`，也不表示一个分支里有多条原始轨迹。
- 单条中心轨迹默认使用 `MAP trajectory`。`mixture mean trajectory` 可以作为诊断输出，但不能把多峰预测强行平均成一条物理上不可实现的轨迹。
- 当前预测话题和 RViz 不发布额外采样轨迹；此前仅用于可视化的 samples 已删除，因为它不服务实际规划或评估契约。后续若某个算法内部确实需要 Monte Carlo samples，应由该消费者从 GMM 临时派生，并保留 `component_id` 或等价分支来源；这不进入 predictor 默认消息或 RViz marker。

轨迹预测分布表达建议按 `D1-D4` 演进：

- D1：单高斯或单预测分支，概率为 1。
- D2：简化高斯混合，少量航向/意图预测分支，概率归一化。
- D3：交互修正后的高斯混合，预测分支概率受目标间、目标-地图约束影响；必要时对轨迹均值做轻量修正。
- D4：概率和过程噪声尺度由后续观测误差闭环校正。

## LDOP 第三阶段内部实施里程碑

### P1：轨迹外推，得到单条轨迹（已实现）

目标是建立最小可运行预测链路。

当前实现内容：

- 已新增 `DynamicObjectPredictor` 模块。
- 已新增预测参数结构；当前预测时长由 `prediction_horizon` 控制，采样间隔复用 `tracking_default_dt`。
- 已新增预测消息，表达单目标多预测分支的未来轨迹；P1 只填一个预测分支。
- 已从 `TrackPredictionInput` 按 `motion_model_type` 解释预测所需运动学量；接口层没有新增第二套公共状态投影。
- 已对每个未来采样时间做轨迹外推：
  - 中心均值按解释后的模型状态推进。
  - 模型原生协方差按当前运动模型传播，位置不确定性可从前三维位置协方差派生。
- 已保持低速或近似静止目标同一预测链路；预测输入已经是 tracker 确认的动态目标，不额外设置退化分支。
- 已对短历史和漏检延续目标保留同一预测链路；漏检延续目标在 RViz 中用更低透明度表达当前观测不确定性。
- 已发布预测消息和预测 marker。

当前输出：

- 每个稳定目标一条未来中心轨迹。
- 单预测分支概率为 1。
- 历史 P1 版本曾在 RViz 中显示预测线和 1σ 不确定性椭球；当前 P2 可视化已经收口为真实保留预测分支的中心状态轨迹线。

后续维护重点：

- P1 应作为所有后续方法的最弱基线，不被删除。
- 后续 GMM 分支要能和 P1 基线做 ablation，对比 `CV/CA/CTRA` 纯外推是否已经足够。

### P2：轨迹外推 + 航向/意图，得到简化 GMM 多轨迹（已实现）

目标是把单一确定性轨迹扩展为少量候选意图分支，并复用现有多分支消息结构。当前实现已在 `DynamicObjectPredictor` 内部完成历史特征提取、类别感知候选分支、启发式概率归一化、每分支 rollout 和分支中心状态线可视化。

当前实现内容：

- 在 `DynamicObjectPredictor` 内部增加 `PredictionMode` 或等价内部结构，描述每个 mode 的行为族、行为强度、概率、过程噪声尺度和 rollout 结果。
- 复用 `DynamicObjectPredictionBranch`；P2 清理了旧 `BEHAVIOR_BASELINE` 枚举，但不新增消息文件或采样轨迹字段：
  - `BEHAVIOR_LONGITUDINAL`：沿历史主运动方向加速、减速、停止或继续。
  - `BEHAVIOR_TURNING`：左转、右转或保持曲率。
  - `BEHAVIOR_VERTICAL`：上升、下降或保持高度。
  - `BEHAVIOR_LATERAL`：相对当前航向左/右避让。
- 基于历史轨迹估计稳定航向，而不是只依赖当前状态快照中的瞬时运动量。
- 根据类别和运动模型选择候选预测分支：
  - human 可允许更灵活的横向偏转、减速和停止。
  - vehicle 更偏向保持航向、小曲率转向或纵向加减速。
  - UAV 可允许三维速度方向延续和垂向行为。
  - other 默认保守单峰或低权重多峰。
- 根据历史航向稳定性、速度/加速度趋势、类别可信度、历史长度和漏检状态分配初始概率。
- 每个预测分支输出一条独立模型原生中心状态轨迹和对应的模型原生协方差序列；分支内部不包含多条原始轨迹样本。
- RViz 只显示每条保留 GMM 分支的一条中心状态轨迹线；不显示 P1 椭球 marker，也不显示额外 sample marker。

P2 推荐的最小候选集合：

```text
baseline/keep
longitudinal-decelerate-or-stop
turn-left
turn-right
vertical-hold-or-climb-descend（仅 UAV 或 z 方向证据明显时启用）
```

当前输出：

- 每个目标多条候选 GMM 分支，每条分支是一条带时序协方差的中心状态轨迹分布。
- 轨迹概率构成简化 trajectory-level GMM。
- 概率总和归一化，主预测分支应与历史运动趋势一致。

已覆盖的验收重点：

- 多预测分支能覆盖转向、停止或航向不确定目标。
- 不因速度数值较小额外切换算法；是否偏向停止应由历史速度趋势和概率更新表达。
- 预测分支数量由 `prediction_max_branches` 和 `prediction_min_branch_probability` 控制；每分支时间采样点数量由预测 horizon 和采样间隔决定。
- 保留 P1 基线作为对照的能力通过分支数量和概率阈值控制；离线 `minADE/minFDE` 或多模态 coverage 对比放入 P4/evaluator 阶段。

### P3：轨迹外推 + 航向/意图 + 交互，得到交互感知 GMM 多轨迹

目标是在多意图轨迹基础上加入交互能量，使预测不只做碰撞约束，而是能表达动态目标的主观能动性近似。

主要内容：

- 目标间交互：
  - 根据未来采样点的目标间距离、相对速度和 TTC 识别潜在过近风险。
  - 引入 collision cone overlap，识别交叉、迎面和追随关系。
  - 对明显冲突的预测分支降低概率；必要时做轻量避让修正，但首版优先只改概率，减少对运动模型语义的破坏。
- 目标与地图交互：
  - 使用 LDOP 第一阶段静态地图或查询接口判断预测点是否进入静态障碍物、近障区域或明显不可通行区域。
  - 对穿越障碍或贴近障碍的预测分支降低概率。
  - 使用自由通道方向作为航向先验，使候选分支更偏向可通行空间。
  - 将地图约束写成 predictor 的可选输入，避免 predictor 反向污染静态地图模块。
- 交互能量形式：

```text
E_pair(i,j,z_i,z_j) =
  w_ttc · softplus(TTC_safe - TTC_ij)
+ w_cone · collision_cone_overlap
+ w_yield · priority_inconsistency

E_map =
  w_occ · occupied_or_near_obstacle_cost
+ w_esdf · exp(-distance_to_obstacle / sigma)
+ w_free · heading_to_free_corridor_penalty
```

- 分支概率更新：

```text
π_z = normalize(π_z_prior · exp(-(E_pair + E_map + E_heading) / τ))
```

预期输出：

- 多目标场景下更合理的预测分布。
- 穿越静态障碍或明显冲突的预测分支概率降低。
- 交互计算耗时可控，适合在线运行。

验收重点：

- 目标接近、交叉、迎面或狭窄通道会车时，预测概率变化符合直觉。
- 地图约束不会破坏无交互场景中的基础轨迹外推表现。
- 交互逻辑保持可解释，便于后续替换为学习方法或学习权重。

### P4：闭环评估与概率反馈

目标是让预测不只是一次性输出，而是能根据后续真实跟踪结果评估和校正。

主要内容：

- 建立预测缓存，把每一帧发布的预测和后续实际 tracker 输出按目标 ID 对齐。
- predictor 当前是无状态外推器；闭环阶段需要让 predictor 或独立 evaluator 维护上一轮预测缓存。若缓存只用于评估和概率先验，优先放入 evaluator 或 predictor 内部状态，避免污染 tracker。
- 计算位置误差和位移误差：
  - `ADE`：平均位移误差。
  - `FDE`：终点位移误差。
  - `minADE/minFDE`：多模态预测必须按 minimum-over-modes 统计。
  - `MAP ADE/FDE`：检查最高概率分支是否可信。
  - 分 horizon 统计误差，如 0.5s、1.0s、2.0s、3.0s。
- 评估不确定性一致性：
  - 实际位置是否落在由模型原生协方差前三维位置子块派生的协方差椭球内。
  - 使用 `NLL`、`coverage error`、`ECE/Brier` 和 `NIS consistency` 判断分布是否校准。
- 统计耗时：
  - predictor 总耗时。
  - 预测分支生成耗时。
  - 交互修正耗时。
  - marker 构建耗时。
  - runtime mean/p95，尤其关注目标数量和 mixture 数增长时的延迟。
- 根据误差反馈不同预测分支的概率：
  - 命中真实轨迹附近的预测分支增权。
  - 长期偏离的预测分支降权。
  - NIS 偏大时增加该 mode 的过程噪声尺度或降低其先验置信度。
  - 对类别、速度区间和交互场景分别维护轻量统计。

闭环更新建议：

```text
ν_z = y_t - H μ_z^-(t)
NIS_z = ν_z^T S_z^{-1} ν_z
L_z = N(y_t ; H μ_z^-(t), S_z)

π_z(t) = normalize(π_z(t|t-1) · L_z · exp(-E_z / τ))
Q_z(t) = α_z(t) Q_z_base
```

预期输出：

- 预测评估日志或诊断话题。
- 各 horizon 的误差统计。
- 轨迹概率随闭环误差逐步校正。

验收重点：

- 评估不影响在线预测稳定性。
- 反馈只改变概率或轻量参数，不反向破坏 tracker 当前状态。
- 闭环指标能解释预测效果变好或变差的原因。
- 如果 proposed 方法无法改善 `NLL` 或 coverage，只能将交互模块保留为 ablation，不应强行作为主贡献。

## LDOP 总体实现进展

已经完成：

- LDOP 第一阶段：动静态点云分割与静态地图构建。
- LDOP 第二阶段：动态点云聚类、检测、跟踪、目标类别、多运动模型 KF/EKF。
- tracker 当前输出稳定 `DynamicObjectArray`，包含目标 ID、尺寸、模型原生状态、模型原生协方差、对象类别和运动模型。
- tracker 内部已构建 `TrackPredictionInput`，为 predictor 提供完整输入。
- LDOP 第三阶段 P1 已实现：
  - `DynamicObjectPredictor`。
  - `DynamicObjectPredictionPoint.msg`、`DynamicObjectPredictionBranch.msg`、`DynamicObjectPrediction.msg`、`DynamicObjectPredictionArray.msg`。
  - 独立预测话题 `/ldop/dynamic_object_predictions`。
  - 独立预测 marker 话题 `/ldop/dynamic_prediction_markers`。
  - 按当前 `motion_model_type` 外推状态和协方差。
  - 历史 P1 预测线和位置协方差椭球 RViz 可视化。
  - `test_dynamic_object_predictor` 覆盖 CV3D 基线预测点和 `influence_weight`。
- LDOP 第三阶段 P2 已实现：
  - 根据历史轨迹估计航向、速度趋势、垂向趋势、历史质量和漏检惩罚。
  - 按 human、vehicle、UAV、unknown/other 生成纵向、转向、横向和垂向候选分支。
  - 使用启发式 score + softmax 得到归一化分支概率，并按 `prediction_max_branches` 和 `prediction_min_branch_probability` 裁剪。
  - 每条分支独立传播模型原生状态和协方差；假设性分支和低质量历史会放大未来过程噪声。
  - RViz 只显示真实保留 GMM 分支中心状态线，不生成协方差椭球或额外采样轨迹 marker。
  - `BEHAVIOR_BASELINE` 已从 `DynamicObjectPredictionBranch.msg` 移除，保留的分支类型均为行为族。

LDOP 第三阶段尚未完成：

- D3 级别的目标间、目标-地图交互能量。
- D4 级别的闭环预测误差统计、NIS 校准和概率反馈。
- 多模态预测所需的 `minADE/minFDE`、`NLL`、coverage、ECE/Brier 等评估工具。

因此，LDOP 第三阶段下一步应从“目标间/目标-地图交互能量”和“闭环评估缓存”开始，而不是重新实现 P1/P2，也不是直接上学习式交互预测。

## 建议文件范围

P2 当前已改动并应继续维护这些文件：

- `include/ldop/dynamic_object_predictor.h`
- `src/dynamic_object_predictor.cpp`
- `test/test_dynamic_object_predictor.cpp`
- `config/ldop.yaml`
- `msg/DynamicObjectPredictionBranch.msg`
- `launch/rviz/ldop_rviz.rviz`
- `CMakeLists.txt`

P3 交互预测可能新增或修改：

- `include/ldop/dynamic_object_predictor.h`
- `src/dynamic_object_predictor.cpp`
- `include/ldop/ufomap_mapper.h` 或新增只读地图查询接口
- `src/ufomap_mapper.cpp`
- `test/test_dynamic_object_predictor.cpp`

P4 闭环评估可能新增：

- `include/ldop/dynamic_object_prediction_evaluator.h`
- `src/dynamic_object_prediction_evaluator.cpp`
- 诊断或评估消息（是否需要新增 `msg` 需单独收口）
- `src/ldop.cpp`
- `include/ldop/ldop.h`
- `test/test_dynamic_object_prediction_evaluator.cpp`

P2 不新增消息文件或采样轨迹字段；当前只清理了 `DynamicObjectPredictionBranch.msg` 中的旧 `BEHAVIOR_BASELINE` 枚举。仅用于可视化的额外采样轨迹不再作为后续方向保留；只有当未来明确的评估或规划接口必须跨模块传递 Monte Carlo samples，且无法直接消费 branch GMM 时，才单独讨论新增诊断消息。

## 验证策略

验证范围按 `AGENTS.md` 的风险匹配原则选择，不把测试作为每次改动的固定动作：

- 仅修改文档或规划：执行 `git diff --check`，必要时复扫关键术语。
- 修改 C++ 实现、参数读取或配置默认值：至少执行 `catkin build ldop --no-status`。
- 修改 `msg/*.msg`：必须执行 `catkin build ldop --no-status`，确保消息重新生成。
- 修改预测行为、接口契约、并发链路或新增测试：再执行 `catkin run_tests ldop --no-status`，并用 `catkin_test_results /home/st/ldop_ws/build/ldop` 查看结果。

多模态预测专项验证：

- P2 至少比较 P1 基线与多分支 GMM 的 `MAP ADE/FDE`、`minADE/minFDE` 和概率归一化。
- P3 至少增加交叉、迎面、跟随、靠近静态障碍物等压力场景。
- P4 必须报告 `NLL`、coverage error、ECE/Brier 或 NIS consistency 中至少一类校准指标。

本安排文档只记录规划和当前代码事实；仅修改文档时运行 `git diff --check` 即可。

## 决策记录

| 时间 | 决策 | 原因 |
|------|------|------|
| 2026-05-14 | 第三阶段预测走独立预测消息和话题 | 当前 `DynamicObject.msg` 表达当前跟踪状态，未来预测应保持独立，避免混淆当前状态和未来分布 |
| 2026-05-14 | 第一版先实现轨迹外推单轨迹 | 这是最小可验证闭环，可直接复用 tracker 的模型原生状态及协方差；CTRA 等转弯模型并不是线性外推 |
| 2026-05-14 | 预测消息一开始就按多预测分支结构设计 | 轨迹外推阶段只填一条基准分支、概率为 1.0；后续加入航向/意图和交互预测时不破坏预测话题接口 |
| 2026-05-14 | 默认预测未来 1.0 秒，采样间隔复用 `tracking_default_dt` | 默认短时预测先覆盖更近未来；采样时间粒度与 tracker 默认时间步长保持一致，避免新增重复参数 |
| 2026-05-14 | 漏检延续但尚未删除的稳定轨迹继续预测 | 与 tracker 当前仍发布 coasting 轨迹的策略保持一致；消息保留，RViz marker 用更低透明度表达不确定性 |
| 2026-05-14 | 不为低速或近似静止目标单独设置退化策略 | 预测模块只消费 tracker 已确认的动态目标；低速仍属于动态目标状态范围，按当前运动模型状态正常外推 |
| 2026-05-14 | 预测消息保存模型原生状态和模型原生协方差 | 与当前 `DynamicObject.msg` 的 `model_state/model_covariance` 语义保持一致；RViz 椭球只从预测状态前三维位置协方差派生 |
| 2026-05-14 | 未来采样点单独建消息类型 | 用 `DynamicObjectPredictionPoint` 同时承载 `time_from_start`、`model_state` 和 `model_covariance`，避免并行数组错位 |
| 2026-05-14 | 预测分支类型改为通用行为族方向 | 不把分支类型绑定到实现阶段或具体方向；上升/下降、前进/后退、转弯这类成对行为应由同一行为族加有符号参数表达 |
| 2026-05-14 | `BEHAVIOR_BASELINE` 只用于第一阶段基准预测 | 第一阶段用它表示不额外假设意图、仅按当前运动模型外推；后续多意图/交互预测不再使用该行为类型 |
| 2026-05-14 | `behavior_value` 使用归一化有符号行为强度 | 范围为 `[-1.0, 1.0]`；真实轨迹仍由采样点模型原生状态表达，行为强度只辅助解释预测分支偏向 |
| 2026-05-14 | 模型不确定性和规划影响权重分开表达 | `model_covariance` 按运动模型真实传播；预测尾部对规划/RViz 的影响通过 `influence_weight` 衰减，不篡改协方差 |
| 2026-05-14 | 影响权重前半段保持 1.0，后半段衰减到 0.3 | 分段时间直接取预测总时长的一半；最小尾部影响权重写死为 0.3，不额外暴露参数 |
| 2026-05-14 | 不设置预测目标数量限制 | predictor 对 tracker 输出的全部稳定预测输入做预测，稳定性和数量边界由 tracker 控制 |
| 2026-05-14 | RViz 预测可视化包含轨迹线和不确定性椭球 | 轨迹线连接采样点中心；每个采样点用椭球表达前三维位置协方差，透明度叠加 `influence_weight` 和漏检状态 |
| 2026-05-14 | 不确定性椭球由 3x3 位置协方差特征分解生成 | 使用协方差主轴方向和长度显示椭球，不只取 xyz 对角线，保留位置相关性 |
| 2026-05-14 | 不确定性椭球默认显示 1σ | 椭球轴长按 `2 * sqrt(lambda)` 设置为 RViz `SPHERE` 直径；负特征值仅作为数值误差裁剪到 0 |
| 2026-05-14 | 预测目标消息保留 `size` 和 `matched_in_current_frame` | `size` 服务后续风险膨胀和可视化；`matched_in_current_frame` 表达预测是否来自当前帧匹配目标 |
| 2026-05-14 | 预测目标消息复用当前目标类别和运动模型常量 | `DynamicObjectPrediction.msg` 中保留与 `DynamicObject.msg` 一致的 `CLASS_*` 与 `MOTION_MODEL_*` 常量，避免下游维护第二套数字映射 |
| 2026-05-14 | predictor 自己读取 `tracking_default_dt` 作为采样间隔 | 采样间隔参数名与 tracker 复用，但模块各自读取，避免为了传参改动 tracker/Ldop 现有边界 |
| 2026-05-14 | 预测结果和预测 marker 使用独立可配置话题 | 新增 `dynamic_predictions_topic` 和 `dynamic_prediction_markers_topic`，默认分别为 `/ldop/dynamic_object_predictions` 与 `/ldop/dynamic_prediction_markers` |
| 2026-05-14 | predictor 返回帧级结果并由 `Ldop` 统一发布 | `DynamicObjectPredictorFrameResult` 包含预测消息、预测 marker 和 timing；`Ldop::processFrame()` 在 tracker 后调用，`publishFrame()` 统一发布 |
| 2026-05-14 | predictor 输入按内部契约处理 | `TrackPredictionInput` 来自 tracker 内部稳定输出，不按外部不可信输入做重复兜底；状态维度等使用 assert 或明确契约表达 |
| 2026-05-14 | 修改消息和 C++ 后至少执行 `catkin build ldop --no-status` | 新增 `msg` 会触发消息生成，构建验证是第一阶段实现后的最低验证范围 |
| 2026-05-14 | 第一阶段采用独立 predictor 并复用 `MotionModel` | predictor 不复用 `KalmanFilterBase`，避免重新初始化协方差或混入 tracker 自适应/限幅语义；直接从输入状态和协方差做只读轨迹外推 |
| 2026-05-14 | 未来传播使用运动模型基础 `Q` | 协方差初值来自 tracker 当前自适应后的 `model_covariance`，但未来传播新增过程噪声使用运动模型基础 `Q`，不读取 tracker filter 内部自适应噪声状态 |
| 2026-05-14 | 多轨迹概率分布分两步实现 | 先用航向/意图形成简化 GMM，再引入交互约束，降低一次性设计复杂度 |
| 2026-05-14 | 闭环反馈放到第四阶段 | 需要先有预测输出和后续真实跟踪结果，才能可靠评估和更新概率 |
| 2026-05-18 | 后续方法以 trajectory-level GMM 为核心输出 | 避免 RAL-25 式多采样轨迹被均值化后丢失多峰性 |
| 2026-05-18 | 单条中心轨迹默认使用 MAP 分支 | mixture mean 可能落在不可实现的中间区域，只能作为诊断统计视图 |
| 2026-05-18 | P2 首选不改消息，直接复用现有多 branch 结构 | 当前 `DynamicObjectPredictionBranch` 已包含行为族、行为强度、概率和采样点，足以表达第一版 GMM |
| 2026-05-18 | 交互首版优先通过能量项调整分支概率 | 先保持轨迹生成和运动模型语义稳定，降低一次性实现风险 |
| 2026-05-18 | 闭环不确定性通过预测残差/NIS 更新概率和过程噪声尺度 | 把上一轮预测是否解释当前观测的信息反馈到下一轮意图概率与协方差，而不反向破坏 tracker 当前状态 |
| 2026-05-18 | 学习式交互预测暂列后续增强或强基线 | 当前主线优先可解释、无训练、在线可验证的方法，适配 LDOP 现有代码和数据条件 |
| 2026-05-19 | P2 航向/意图 GMM 已作为当前 predictor 主路径 | 当前代码已在 `DynamicObjectPredictor` 内部完成多分支模式生成、概率归一化、分支 rollout 和分支线可视化 |
| 2026-05-19 | 移除 `BEHAVIOR_BASELINE` | 当前预测分支统一使用行为族枚举；P1 基线消融通过分支数量和概率阈值控制，不再依赖旧枚举 |
| 2026-05-19 | P2 RViz 只显示真实保留分支中心状态线 | 仅用于可视化的采样轨迹没有实际使用价值，且容易被误认为真实预测输出；当前可视化与 `/ldop/dynamic_object_predictions` 的 branch 输出保持一致 |
| 2026-05-19 | P2 不新增 sample 消息字段 | 仅用于可视化的额外采样轨迹已删除；当前主线让 planner/evaluator 直接消费 branch GMM |

## 待确认问题

| 优先级 | 问题 | 默认建议 |
|--------|------|----------|
| 已收口 | P2 是否需要新增参数控制最大分支数 | 已新增 `prediction_max_branches`，当前 YAML 运行为 3 |
| 已收口 | P2 分支概率是否先使用启发式还是贝叶斯递推 | 当前 P2 使用历史特征启发式归一化；P4 再引入上一轮 likelihood/NIS 递推 |
| 已收口 | P2 是否输出采样轨迹 | 当前只输出 branch GMM；仅用于可视化的额外采样轨迹已删除，不作为默认后续接口恢复 |
| 中 | P3 地图查询接口如何接入 predictor | 默认只读查询，不让 predictor 修改 UFOMap；若接口尚不稳定，先实现目标间交互 |
| 中 | P4 evaluator 是在线模块还是离线工具 | 默认先做在线轻量缓存和日志统计，再按需要扩展离线 rosbag 评估工具 |
| 后续 | 是否引入学习式 interaction encoder | 默认不进入当前主线；只有当规则型能量项在压力场景中不足时再作为对照或增强 |

## 沟通约束

- 面向用户说明当前工作时，不依赖过程文档的阶段简称；直接描述“基础轨迹外推已实现”“下一步多分支 GMM”“后续交互能量”“闭环 NIS/概率反馈”等具体能力。
- 区分“预测分布”和“中心轨迹”：多模态分布是主要输出，MAP 中心轨迹只是便于可视化和下游快速消费的派生结果。

## 错误记录

| 时间 | 错误 | 处理 |
|------|------|------|
| 2026-05-14 | 默认沙箱缺少 `bwrap`，首次读取技能文件失败 | 已按审批流程以 escalated 方式读取本地文件并继续执行 |
| 2026-05-14 | `apply_patch` 直接删除重建 `task_plan.md` 失败，且一次只改了文件标题 | 改用受控脚本整体重写三份规划记录和新安排文档；重写后继续做格式检查 |
| 2026-05-14 | 记录低速目标决策时补丁上下文不匹配 | 重新读取相关段落后使用精确上下文更新规划文件 |
