# 第三阶段动态目标轨迹预测发现记录

## 已知约束

- 面向用户和文档的自然语言默认使用简体中文；代码标识符、消息字段、参数名和 ROS 话题名保留英文。
- 第一阶段动静态点云分割与静态地图已经完成。
- 第二阶段动态点云聚类、检测与跟踪已经完成，当前主线进入第三阶段动态目标短时轨迹预测。
- 第三阶段预测应只消费第二阶段稳定跟踪目标，不直接对未聚类动态点云做规划级预测。
- 预测结果应通过独立消息和独立话题发布，不回填到 `DynamicObject.msg`。

## 当前代码事实

- `msg/DynamicObject.msg` 当前只表达当前跟踪目标状态：
  - `id`
  - `size`
  - `model_state`
  - `model_covariance`
  - `object_class`
  - `motion_model_type`
- `DynamicObject.msg` 中没有预测数组；未来预测需要新增独立消息。
- `include/ldop/dynamic_object_tracker.h` 中已经存在 `TrackPredictionInput`，它是 tracker 预留给未来 predictor 的内部快照。
- `TrackPredictionInput` 已包含预测所需的关键输入：
  - `stamp`
  - `id`
  - `object_class`
  - `motion_model_type`
  - `bbox`
  - `model_state`
  - `model_covariance`
  - `history`
  - `age`
  - `hits`
  - `missed_frames`
  - `matched_in_current_frame`
- `DynamicObjectTrackerFrameResult` 已包含 `prediction_inputs`，当前仍只在 C++ 内部流转，没有新增预测话题。
- `docs/note/LDOP_design.md` 已明确第三阶段目标是对第二阶段稳定跟踪目标做 1-3 秒短时预测，并输出独立预测话题和可视化。
- 最近提交 `d0dfaec 轨迹预测的准备` 已把当前主线推进到预测准备状态；本轮开始时工作区只有 `task_plan.md` 和 `progress.md` 由本轮流程更新。
- `src/ldop.cpp` 当前已经发布动态对象 bbox marker、稳定轨迹 marker 和静态地图 marker；预测 marker 应按相同 `MarkerArray` 模式接入 `Ldop::publishFrame()`。
- 预测第一阶段的代码范围大概率集中在新增 predictor、预测 msg、`Ldop` 持有/调用/发布、`config/ldop.yaml` 参数和 `CMakeLists.txt` 消息构建。
- `Ldop::processFrame()` 当前顺序是 UFOMap -> clusterer -> tracker -> 装入 `ProcessedFrame`；轨迹外推 predictor 最贴合的位置是在 `dynamic_object_tracker_->processDynamicTracks(...)` 之后消费 `tracking_result.prediction_inputs`。
- `Ldop::publishFrame()` 是所有输出的统一发布点；轨迹外推预测需要在 `ProcessedFrame` 中新增预测消息和预测 `MarkerArray`，并增加对应 publisher/topic 参数。
- `buildPredictionInputs()` 与 `buildOutput()` 使用同一 `min_hits_to_publish` 过滤策略，因此 predictor 默认只处理已经对外发布的稳定轨迹；coasting 轨迹仍可能进入预测输入。
- 所有运动模型的中心位置都固定在 `model_state[0..2]`：
  - `CA2D`: `[x,y,z,vx,vy,ax,ay]`，z 位置保持，水平面 constant acceleration。
  - `CA3D`: `[x,y,z,vx,vy,vz,ax,ay,az]`。
  - `CV3D`: `[x,y,z,vx,vy,vz]`。
  - `CTRA`: `[x,y,z,v,a,yaw,yaw_rate]`，z 保持，水平面按非线性 CTRA 转移。
- `motion_model.cpp` 已经实现 `stateTransition()` / `getTransitionF()`，`multi_model_kalman_filter.cpp` 也明确 EKF 协方差传播要用预测前状态处的 CTRA 雅可比；轨迹外推单轨迹预测可以复用同一运动模型语义做多步预测和 `P' = FPF^T + Q`。
- 当前 `CMakeLists.txt` 已有 `message_generation`、`geometry_msgs`、`std_msgs`、`visualization_msgs`，新增预测 `msg` 主要需要加入 `add_message_files()`；如果预测消息使用现有依赖，应无需新增 package 依赖。
- 当前 `config/ldop.yaml` 参数按模块前缀组织；预测模块应新增 `prediction_*` 参数，保持默认值在 `DynamicObjectPredictorParams` 结构中集中声明，再由 `loadParameters()` 三参数形式读取。
- P2 当前代码事实：
  - `DynamicObjectPredictor` 已直接生成多条航向/意图预测分支，不再走旧单分支 fallback。
  - `DynamicObjectPredictionBranch.msg` 已移除 `BEHAVIOR_BASELINE`，分支类型统一为纵向、垂向、转向和横向行为族。
  - `config/ldop.yaml` 当前运行值为 `prediction_max_branches: 3`、`prediction_min_branch_probability: 0.05`；这两个参数都作用在 GMM 分支/component 层面，不作用在分支内的时间采样点或额外轨迹样本上。
  - RViz 预测 marker 只保留 `dynamic_prediction_line`，一条 marker 对应一条真实保留的 GMM 分支；不生成协方差椭球或额外采样轨迹 marker。此前仅用于可视化的 sample marker 已删除，因为它没有实际规划或评估消费者。
  - 当前 P2 测试重点覆盖 CV3D、CTRA、类别策略、概率归一化、分支发散、噪声放大和 marker 语义；CA2D/CA3D 通过共享 `MotionModel` 构建和包级构建覆盖。
- `DynamicObjectPredictor::predict()` 当前接收 `header`、`TrackPredictionInput[]` 和可选 `PredictionMapQuery*`；地图约束通过只读 adapter 注入，predictor 不拥有 UFOMap runtime。
- `UfomapMapper` 当前提供 `queryNode()`、`seenFree()`、`queryDynamicCandidates()` 和中性静态邻域查询 `queryLocalOccupied()`；P3 静态近障碍查询使用 `queryLocalOccupied()`，不把 dynamic candidate / label 语义混入预测地图代价。
- `Ldop::processFrame()` 当前顺序为 UFOMap、clusterer、tracker、predictor；P3 在 tracker 后构造栈上 `UfomapPredictionMapQuery` adapter，供 predictor 查询静态占据、近障碍和 free corridor 证据。
- `DynamicObjectPredictorTimingStats` 目前区分预测对象、应用交互上下文和 marker 构建耗时；P4 若加入闭环评估，可继续细分 `evaluation` 或 `feedback` 耗时。
- 预测消息当前没有 interaction cost、map cost、NIS 或 evaluator 指标字段；如果 P3/P4 只改变分支概率和协方差，现有消息可继续承载 planner-facing 输出，诊断信息应优先放日志、统计结构或独立诊断话题，而不是污染默认预测消息。
- 当前 P2 的私有数据流是 `MotionHistoryFeatures -> PredictionMode[] -> DynamicObjectPredictionBranch[]`；P3 可以在 `PredictionMode` 概率归一化之后、rollout 之后或 branch 输出前加入交互能量。若需要基于未来轨迹相互影响，rollout 后重权重更直接。
- P3 当前代码事实：
  - `PredictionInteractionContext` 已作为独立模块落地，使用 `PredictionMapQuery` 抽象读取地图证据；当前实现不复制完整 UFOMap，也不再做逐点地图查询，而是用分支级 occupied 查询预算和每分支 8 方向 corridor 探针控制耗时。
  - P3 静态地图约束通过 `PredictionMapQuery` 抽象读取 UFOMap；动态目标交互继续来自 tracker/predictor 的稳定目标状态、尺寸和预测分支，不使用 UFOMap dynamic candidate 查询反推未来动态风险。
  - `DynamicObjectPredictor` 应用交互结果时先调整 branch probability，再把逐点修正 hint 平滑成速度/减速/方向控制 profile，并用原 `MotionModel` 重新 rollout 分支；不再直接改写离散采样点位置，也不缩小 `model_covariance`。
  - `PredictionBranchInteraction` 已新增内部 `PredictionBranchRiskLevel::{Low, Medium, High, Critical}`，由 occupied overlap、TTC、pair/map/corridor cost 和 total energy 派生；该字段不进入 ROS prediction message，作为 P4 风险地图 builder 的快速分层摘要。
  - `Ldop` 的 UFOMap adapter 是 `processFrame()` 内的栈上对象，所有读查询仍走 `UfomapMapper` 自身锁语义。
  - `config/ldop.yaml` 已加入 P3 中等粒度参数：交互开关、目标间/地图/corridor 权重、安全距离、近障碍距离、TTC 阈值、free corridor 半径、修正上限和能量温度。

## 用户给出的第三阶段思路

- 输入：动态目标的过往轨迹；当前接口层由模型原生状态及协方差承载预测所需运动学量和不确定性。
- 基础方法：利用运动学特征做轨迹外推；对 `CV3D/CA2D/CA3D` 表现为线性或近似线性的状态传播，对 `CTRA` 等转弯模型表现为非线性轨迹传播。
- 意图增强：利用过往轨迹和运动学特征确定航向/意图。
- 交互增强：考虑目标间交互、目标与地图交互；当前热门方法多为学习方法，但本仓库首版应优先用可解释规则型方法起步。
- 输出：物体中心的轨迹预测，以及采样得到的多条轨迹概率分布，分布可先用高斯或高斯混合表示。
- 评估：闭环统计位置误差、位移误差和耗时，并将不确定性与误差反馈给不同轨迹概率。

## 阶段性实现结论

- 第一阶段应先实现轨迹外推，得到单条中心轨迹；这是最小可运行预测链路。
- 第二阶段在单轨迹基础上加入航向/意图，得到简化版高斯混合分布的多条轨迹。
- 第三阶段加入交互，得到完整高斯混合分布的多条轨迹。
- 第四阶段完成闭环评估和概率反馈，使不同轨迹模式的概率能随观测误差调整。

## 待核实内容

- 已确认预测消息内部直接预留多预测分支结构；轨迹外推单轨迹阶段只填一条 `trajectory_extrapolation` 分支，概率为 1.0。这里的预测分支表示同一目标未来的一种候选走法，例如保持当前方向、轻微左偏或轻微右偏。
- RViz 预测 marker 默认话题名已确定为 `/ldop/dynamic_prediction_markers`；当前 P2 样式为每条真实 GMM 分支一条中心状态轨迹线，漏检延续目标降低透明度。
- P3 交互预测的首版范围已确认：目标间交互与目标-地图占据/近障碍约束同时进入设计。
- P3 交互作用范围已确认：以分支概率重权重为主，允许对明显穿障、过近或迎面冲突的分支做轻量中心轨迹修正。
- P3 地图约束策略已确认：同时使用 UFOMap 占据/近障碍软代价和局部 free corridor 方向先验。
- P3 交互上下文接入方式已确认：新建独立 `PredictionInteractionContext` 模块聚合目标间关系、UFOMap 静态约束、free corridor 和分支交互代价，再交给 predictor 消费。
- `PredictionInteractionContext` 不等同于第四阶段 planner-facing 类 ROG-Map 地图。它是 P3/P4 可复用的场景证据层；第四阶段应单独实现风险/局部地图 builder，消费预测结果、分支概率、协方差、目标尺寸、`influence_weight` 和 context 输出的局部风险摘要。
- P3 交互上下文输出边界已确认：输出给 predictor 的交互/地图代价、总能量和轻量修正建议，同时保留内部诊断统计；首版不新增默认 ROS 诊断话题。
- P3 参数暴露策略已确认：采用中等粒度参数，包括总开关、目标间权重、地图权重、安全距离、近障碍距离、TTC 安全阈值、free corridor 查询半径、轻量修正最大位移/减速比例、能量温度 `tau`。
- P3 验证范围已确认：单元测试覆盖目标交叉、迎面、穿障、近障碍和 free corridor 场景；再执行包级构建和测试，包括 `catkin build ldop --no-status`、`catkin run_tests ldop --no-status`、`catkin_test_results /home/st/ldop_ws/build/ldop`。
- P3 验证还需要回归预测 marker 语义：RViz marker 仍只显示真实保留 GMM 分支中心状态线，不恢复 sample marker 或默认诊断话题。
- P3 总体落地方案已确认：采用独立 `PredictionInteractionContext` 方案，而不是直接扩展 predictor 或提前做第四阶段规划风险上下文雏形。
- P3 架构边界已确认：`PredictionInteractionContext` 聚合当前帧交互证据，predictor 负责应用概率重权重和轻量轨迹修正；第四阶段风险/局部地图 builder 是后续独立模块。
- P3 组件与数据结构已确认：新增 `include/ldop/prediction_interaction_context.h` 和 `src/prediction_interaction_context.cpp`；内部结构包括参数/配置、每分支交互代价、`TrajectoryCorrectionHint` 和 `PredictionInteractionDiagnostics`。
- P3 交互能量与轻量修正规则已确认：`total_energy = w_pair * pair_cost + w_map * map_cost + w_corridor * corridor_cost`；概率按 `exp(-total_energy / tau)` 重权重后归一化；轨迹只做受限的小幅推离、减速或朝 free corridor 偏置；协方差不因交互代价被人为缩小。
- P3 数据流和边界处理已确认：P2 candidate branches 先生成，`PredictionInteractionContext` 评估交互/地图证据，predictor 再应用概率缩放、平滑控制 rerollout 和归一化；地图查询通过只读 adapter 隔离 UFOMap 类型，地图不可用时 `map_cost = 0`。
- P3 地图查询与预算策略已确认：运行时 adapter 使用 `UfomapMapper` 只读查询接口，不复制完整 UFOMap；当前 `PredictionInteractionContext` 按分支构造局部 occupied 查询球，corridor 使用分支代表点做 8 方向 `seenFree()` 探针，避免查询次数随 `points * directions` 放大。
- P3 地图 adapter 构造边界已澄清：`PredictionInteractionContext` 定义抽象查询语义，不持有 `UfomapMapper`；由 `Ldop` 或上层将现有 `UfomapMapper` 包装为 adapter 注入。当前性能优化采用分支级查询预算，而不是 `LocalOccupiedCacheKey` / `unordered_map` 缓存。
- P3 UFOMap 查询函数场景已澄清：`queryLocalOccupied()` 是当前静态 occupied/near-obstacle 主路径，内部使用 UFOMap `Intersects` 等区域查询能力；`seenFree()` 用于已知 free 与 free corridor 采样；`queryNode()` 保留在抽象接口中供后续点查询使用，但当前性能优化后的 P3 主路径不再逐点调用它。
- P3 静态/动态来源分离已澄清：静态障碍和 free corridor 来自 UFOMap 静态/占据查询；动态目标风险来自 tracker 传入的 `TrackPredictionInput[]`、目标尺寸和 GMM 预测分支，不从 UFOMap 动态体素或动态候选查询反推未来动态风险。
- P3 为第四阶段类 ROG-Map/risk map builder 保留的 context 摘要包括：分支风险等级、最近障碍距离、occupied overlap、near-obstacle 标记、free corridor 方向和置信度、最近目标距离、最小 TTC。
- P3 测试与验证设计已确认：单元测试覆盖交互/地图关键场景，predictor 集成测试覆盖概率、修正和 marker 语义，最终执行 `catkin build`、`catkin run_tests` 和 `catkin_test_results`。
- P3 设计规格已写入 `docs/superpowers/specs/2026-05-19-dynamic-object-interaction-aware-gmm-p3-design.md`。
- P3 实现计划已写入 `docs/superpowers/plans/2026-05-19-dynamic-object-interaction-aware-gmm-p3-implementation-plan.md`，计划中把 `PredictionInteractionContext`、UFOMap 静态只读区域查询、predictor 应用逻辑、`Ldop` adapter 和回归测试拆成独立任务。
- P3 实现计划明确：新增中性 `UfomapMapper::queryLocalOccupied()` 只服务静态 occupied/near-obstacle 查询；`queryDynamicCandidates()` 不作为 P3 静态近障碍主接口。
- P3 实现已完成并通过验证：`catkin build ldop --no-status`、`catkin run_tests ldop --no-status` 和 `catkin_test_results /home/st/ldop_ws/build/ldop`；最终汇总为 72 tests, 0 errors, 0 failures。
- 第四阶段闭环评估所需的真实轨迹缓存窗口和目标匹配策略尚未设计。

## 本轮用户选择

- 2026-05-19：P3 首版范围选择“目标间交互 + 静态地图约束”同时进入设计，而不是只做目标间交互或只做地图约束。
- 该选择意味着 P3 需要同时设计两类代价：
  - 目标间代价：基于分支未来采样点之间的距离、相对速度、TTC 或碰撞趋势。
  - 地图代价：基于 UFOMap 占据、近障碍或可通行空间查询。
- 2026-05-19：P3 交互作用范围选择“概率为主，允许轻量修正”。即先按交互能量重权重分支概率，再对明显穿障、过近或迎面冲突的分支做小幅避让、减速或贴合可通行空间修正。
- 2026-05-19：P3 静态地图约束选择“占据/近障碍惩罚 + 可通行方向先验”。设计上需要同时处理穿障/贴障分支降权，以及局部 free corridor 对分支概率和轻量修正方向的引导。
- 2026-05-19：P3 上下文接入选择独立 `PredictionInteractionContext`。设计中需明确该上下文是可复用的场景约束/交互证据层，不是第四阶段类 ROG-Map 规划地图；第四阶段再由独立风险/规划地图构建器消费预测输出。
- 2026-05-19：P3 上下文输出选择“predictor 内容 + 内部诊断统计”。诊断包括每帧最大交互代价、穿障分支数、近碰撞分支数、free corridor 命中率等，用于日志、测试和后续评估衔接，但不默认新增 ROS 话题。
- 2026-05-19：P3 参数暴露选择中等粒度。关键权重、阈值、查询半径和轻量修正上限可配，公式内部细节保持在实现中，避免配置文件膨胀。
- 2026-05-19：P3 验证范围选择“单元测试 + 包级构建/运行测试”。手工 RViz 场景可作为后续观察手段，但不作为本阶段完成硬门槛。
- 2026-05-19：P3 总体方案确认采用独立 `PredictionInteractionContext`。该方案在预测阶段收敛交互/地图证据，同时避免 predictor 承担地图查询和场景理解职责。

## 技术决策

| 决策 | 理由 |
|------|------|
| 预测消息从一开始表达多预测分支 | 轨迹外推阶段仍只发布单条分支，但后续多候选轨迹概率分布可以复用同一话题接口，避免改消息导致下游迁移 |
| 默认预测未来 1.0 秒，采样间隔复用 `tracking_default_dt` | 当前先服务短时动态风险表达；采样间隔沿用 tracker 默认时间步长，减少重复参数并让预测采样粒度与跟踪时间粒度一致 |
| 漏检延续但尚未删除的稳定轨迹继续预测 | tracker 当前会继续发布满足命中阈值且未删除的 coasting 轨迹；预测侧保持同一目标集合，消息保留，marker 透明度降低 |
| 不为低速或近似静止目标单独退化 | 当前系统只跟踪动态目标；速度大小是动态目标状态的一部分，预测模块不额外引入低速/静止分支判断 |
| 预测消息保存模型原生状态和模型原生协方差 | 预测输出与当前 tracker 消息的状态语义一致，不在消息层只保留三维位置不确定性；RViz 椭球从前三维位置协方差派生 |
| 未来采样点单独建消息类型 | 每个采样点把相对时间、模型原生状态和模型原生协方差打包在一起，比多个并行数组更不容易产生索引错位 |
| 预测分支类型改为通用行为族方向 | 分支类型不绑定实现阶段，也不枚举具体方向；上升/下降、前进/后退、转弯等应作为运动行为族，方向和强度由有符号参数表达 |
| 当前基础能力命名为轨迹外推而非线性外推 | 保持直行时线性外推表述成立，但目标航向变化或 CTRA 转弯时轨迹传播不是线性的；轨迹外推能覆盖所有当前运动模型 |
| `BEHAVIOR_BASELINE` 只用于第一阶段基准预测 | 第一阶段需要一个不额外假设意图的基准分支来跑通链路；后续多意图/交互阶段只使用明确行为族分支 |
| `behavior_value` 使用归一化有符号行为强度 | 避免同一字段在不同分支类型下混用不同物理单位；真实轨迹由 `points[].model_state` 表达，行为强度只说明分支偏向 |
| 模型不确定性和规划影响权重分开表达 | 协方差继续表示模型传播出来的不确定性；规划影响由采样点 `influence_weight` 控制，避免远端大协方差直接压缩可通行区域 |
| 影响权重按预测总时长一半分段 | 前半段权重保持 1.0，后半段线性衰减到 0.3；分段点由 `prediction_horizon * 0.5` 得到，最小尾部权重写死为 0.3，不新增影响权重参数 |
| 不设置预测目标数量限制 | predictor 不额外裁剪目标集合；tracker 已经用稳定发布策略控制预测输入，后续若有性能压力再考虑限额参数 |
| RViz 预测可视化包含轨迹线和不确定性椭球 | 预测线使用采样点中心；椭球大小来自模型原生协方差前三维位置子块，透明度由采样点 `influence_weight` 和目标是否漏检延续共同决定 |
| 不确定性椭球由 3x3 位置协方差特征分解生成 | RViz `SPHERE` marker 可通过姿态和 scale 表达旋转椭球；特征分解能保留位置协方差主轴方向和相关性 |
| 不确定性椭球默认显示 1σ | 对位置协方差特征值取平方根得到标准差；RViz `SPHERE` 的 scale 表示直径，因此每个轴使用 `2 * sqrt(lambda)` |
| 预测目标消息保留 `size` 和 `matched_in_current_frame` | `size` 让规划风险层能把实体尺寸和位置不确定性一起考虑；`matched_in_current_frame` 让下游区分当前帧匹配目标和漏检延续目标 |
| 预测目标消息复用当前目标类别和运动模型常量 | `DynamicObjectPrediction.msg` 中保留与 `DynamicObject.msg` 一致的 `CLASS_*` 与 `MOTION_MODEL_*` 常量，方便测试和下游消费预测消息 |
| 顶层 `motion_model_type` 用于解释采样点状态布局 | 预测目标顶层不保存单独协方差；每个预测分支采样点保存随时间传播的 `model_state/model_covariance`，顶层运动模型类型是解释这些数组布局的键 |
| predictor 自己读取 `tracking_default_dt` 作为采样间隔 | 参数名复用保证预测采样粒度与 tracker 默认步长一致；模块各自读取，保持 `Ldop` 和 tracker 构造边界稳定 |
| 预测结果和预测 marker 使用独立可配置话题 | `dynamic_predictions_topic` 默认 `/ldop/dynamic_object_predictions`；`dynamic_prediction_markers_topic` 默认 `/ldop/dynamic_prediction_markers`，命名风格与现有动态目标话题一致 |
| predictor 返回帧级结果并由 `Ldop` 统一发布 | 接口风格与 clusterer/tracker 一致；predictor frame result 承载预测消息、预测 marker 和 timing，主流程在 tracker 后调用 predictor |
| predictor 输入按内部契约处理 | `TrackPredictionInput` 是 tracker 内部构造结果，不作为外部不可信输入；实现中用 assert 或清晰状态维度契约表达不变量，不添加重复兜底 |
| 实现后至少构建验证 | 新增消息、C++ 模块和发布链路后必须执行 `catkin build ldop --no-status`；若实现测试再运行 `catkin run_tests ldop --no-status` 并查看结果 |
| 第一阶段采用独立 predictor 并复用 `MotionModel` | 方案比复用 `KalmanFilterBase` 更适合纯预测输出：不重置协方差，不混入 tracker 自适应/限幅语义，也避免手写重复模型传播逻辑 |
| 未来传播使用运动模型基础 `Q` | 协方差初值来自 tracker 当前自适应后的 `model_covariance`，未来传播新增过程噪声来自 `MotionModel::getProcessNoiseQ()`；当前不把 tracker filter 内部自适应 `process_noise_` 导出到 predictor |
| P2 使用航向/意图 GMM 作为当前 predictor 主路径 | 当前分支由历史特征、类别策略和启发式 score 生成，softmax 后按概率阈值和最大分支数裁剪 |
| P2 不再保留 `BEHAVIOR_BASELINE` | 所有预测分支都必须属于明确行为族；P1 基线消融通过分支数量和概率阈值控制 |
| P2 不发布采样轨迹或椭球 marker | 当前可视化只表达真实保留分支中心状态线；仅用于可视化的 sample marker 没有实际使用价值，避免被误认为 planner-facing prediction |

## 错误与风险

- 当前没有预测测试资产；实现第一阶段时至少需要补轨迹外推和消息构建的单元/构建验证。
- 各运动模型的状态布局不同；预测模块读取 `model_state` 和 `model_covariance` 时必须按 `motion_model_type` 明确解释，不应假设所有模型都是同一状态维度。
- 预测消息中的每个未来采样点也应保留该运动模型自己的完整 `model_state` 和 row-major `model_covariance`；三维位置协方差只是可视化和风险层可派生的子块。
- `model_covariance` 不为了规划保守性做人为先放大再缩小；预测尾部对规划和 RViz 的影响用 `influence_weight` 衰减表达，权重不降为 0。
- 第一阶段默认预测未来 1.0 秒时，`0.1s` 到 `0.5s` 的 `influence_weight` 为 1.0，`0.6s` 到 `1.0s` 线性衰减到 0.3。
- 轨迹外推阶段不单独为低速或近似静止目标设置退化分支；预测只消费 tracker 已确认的动态目标，低速仍属于动态目标状态范围，按当前运动模型状态正常外推。
- 复用 tracker 的 `createKalmanFilter()` 做预测会重新初始化协方差并带入 adaptive/clamp 等 tracker 语义，不适合作为轨迹外推单轨迹的直接方案；更清晰的做法是 predictor 直接构建对应 `MotionModel`，从输入状态/协方差开始做只读外推。
- 当前 `TrackPredictionInput` 不携带 tracker filter 的自适应 `Q/R` 内部状态。为了保持第一阶段接口简单，预测传播使用运动模型基础 `Q`；tracker 自适应估计对当前不确定性的影响已经体现在输入 `model_covariance` 初值里。
- 预测分支类型如果直接枚举左/右/上/下/前/后，会把方向和行为族混在一起；更合适的是枚举行为族，并用有符号连续参数表达方向和强度，例如前进减速仍属于纵向行为族。
- `BEHAVIOR_BASELINE` 已从当前预测分支消息中移除；后续分支应明确属于纵向、垂向、转向或横向等行为族。
- `behavior_value` 的范围固定为 `[-1.0, 1.0]`。纵向正值表示沿主要运动方向加速倾向，负值表示减速或反向趋势；垂向正负表示上升/下降；转向正负表示左右转向；横向正负表示相对当前航向的侧移方向。
- 第一阶段轨迹外推实现计划文档已经生成，路径为 `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-implementation-plan.md`；当前事实不是计划缺失，而是根目录进度记录此前未同步到“实现计划已生成”的状态。

## 沟通约束

- 面向用户交流和后续规格文档中，避免使用依赖过程文档的阶段简称；直接写明当前目标和能力边界，例如“轨迹外推单轨迹预测”“航向/意图多轨迹预测”“目标间和地图交互预测”。
