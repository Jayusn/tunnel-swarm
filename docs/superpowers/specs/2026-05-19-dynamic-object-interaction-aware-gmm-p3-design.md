# LDOP 动态目标交互感知 GMM 多轨迹预测设计

## 状态

本文档细化 LDOP 第三阶段里的轨迹预测内部 P3：在已经完成的轨迹外推与航向/意图 GMM 多分支预测之上，引入目标间交互和目标-地图交互，使预测分支概率和必要的中心轨迹均值能反映动态目标之间的冲突、静态障碍约束和局部可通行方向。

当前已确认边界：

- P1 轨迹外推已完成。
- P2 航向/意图 GMM 已完成，predictor 当前输出 branch-level trajectory-level GMM。
- P3 交互感知预测已完成；后续 P4 predictor 内部在线闭环反馈也已另文实现，因此 LDOP 第三阶段轨迹预测内容已经完成。
- RViz 预测 marker 当前只显示真实保留 GMM 分支中心状态线，不恢复额外 sample marker。
- P3 不修改 `DynamicObject.msg`，不把未来预测回填到 tracker 当前状态消息。
- P3 首版不新增默认 ROS 诊断话题，内部诊断统计用于日志、测试和后续评估衔接。

## 目标

- 同时引入目标间交互和静态地图约束。
- 使用 UFOMap 占据/近障碍软代价和局部 free corridor 方向先验。
- 以分支概率重权重为主，对明显穿障、过近或迎面冲突的分支允许轻量中心轨迹修正。
- 新建独立 `PredictionInteractionContext` 模块聚合交互/地图证据，避免 `DynamicObjectPredictor` 直接承担地图查询和场景理解职责。
- 保持 `/ldop/dynamic_object_predictions` 的 planner-facing GMM 输出结构不变。
- 为第四阶段类 ROG-Map 风险/局部地图构建保留清晰边界。

## 非目标

- 不在本文档的 P3 范围内实现闭环预测误差缓存、NIS 校准或概率反馈；这些已由后续 P4 predictor 内部在线反馈另文实现。
- 不实现第四阶段 planner-facing 类 ROG-Map 或风险地图构建器。
- 不新增预测 sample 消息字段、sample marker 或默认诊断 marker。
- 不引入学习式 interaction encoder。
- 不把每个能量项公式细节全部暴露为 YAML 参数。

## 架构边界

P3 增加独立 `PredictionInteractionContext`，位置在 P2 candidate branches 与 predictor 分支重权重之间：

```text
UFOMap / static map query
TrackPredictionInput[]
P2 candidate branches
  -> PredictionInteractionContext
       - pair interaction cost
       - map / near-obstacle cost
       - free corridor prior
       - lightweight correction suggestion
       - internal diagnostics
  -> DynamicObjectPredictor applies:
       - probability reweighting
       - smooth control profile and model rollout
  -> /ldop/dynamic_object_predictions
  -> /ldop/dynamic_prediction_markers
```

`PredictionInteractionContext` 是预测阶段的交互/地图约束上下文，不是第四阶段 planner-facing 类 ROG-Map 地图。第四阶段应单独实现风险/局部地图 builder，但该 builder 可以复用 `PredictionInteractionContext` 输出的局部交互证据、地图约束证据和风险摘要，避免重复实现目标间关系、近障碍查询和 free corridor 估计。

边界应保持为：

```text
PredictionInteractionContext:
  负责从 tracker/prediction/UFOMap 查询中提取局部约束证据和分支风险摘要

DynamicObjectPredictor:
  负责把约束证据应用到 GMM 分支概率，并把局部修正 hint 转成时间平滑控制后重新 rollout 中心轨迹

P4 risk / local map builder:
  负责把预测分支、概率、协方差、目标尺寸和局部约束证据转成 planner-facing 类 ROG-Map / 动态风险地图
```

因此 P4 可以使用这个新增模块，但不应把它当成最终规划地图。它是 P3 与 P4 共享的场景证据层，输出应保持稳定、结构化，避免只作为 predictor 私有 helper 存在。

## 组件与数据结构

新增模块：

```text
include/ldop/prediction_interaction_context.h
src/prediction_interaction_context.cpp
```

核心类型使用内部 C++ 结构，不新增 ROS msg：

```text
PredictionInteractionParams
PredictionInteractionConfig
PredictionInteractionContext
PredictionBranchInteraction
PredictionInteractionDiagnostics
TrajectoryCorrectionHint
```

`PredictionInteractionContext` 主入口接收：

```text
std_msgs::Header
TrackPredictionInput[]
ldop::DynamicObjectPredictionArray
只读地图查询 adapter
```

输出每个目标、每个 branch 的交互结果：

```text
pair_cost
map_cost
corridor_cost
free_corridor_score
total_energy
probability_scale = exp(-total_energy / tau)
correction_hint
```

为了服务第四阶段类 ROG-Map / 动态风险地图构建，context result 还应保留可复用的局部风险摘要：

```text
branch_risk_level
nearest_obstacle_distance
occupied_overlap
near_obstacle
free_corridor_direction
free_corridor_confidence
nearest_pair_distance
minimum_ttc
```

这些字段不是 ROS prediction message 的默认输出；它们是 C++ 内部结构，供 predictor、测试和后续 LDOP 第四阶段 risk/local map builder 复用。当前 `branch_risk_level` 已落地为内部枚举 `PredictionBranchRiskLevel::{Low, Medium, High, Critical}`，由 occupied overlap、TTC、pair/map/corridor cost 和 total energy 派生；第四阶段 builder 后续可以把该离散摘要与预测分支的 `points[]`、概率、协方差、`size`、`influence_weight` 和连续风险证据结合，生成时间索引的动态风险或类 ROG-Map 局部地图。

`TrajectoryCorrectionHint` 只表达轻量建议，例如：

- 从 occupied 或 near obstacle 区域推离的水平修正方向。
- 沿速度方向的减速比例。
- 朝 free corridor 的小幅方向偏置。

真正更新 branch probability、重新归一化概率，以及把局部 hint 转换为时间平滑的速度/减速/方向控制 profile 后重新 rollout `points[].model_state`，仍由 `DynamicObjectPredictor` 执行。这样上下文模块负责“场景证据”，predictor 负责“预测输出契约”。

内部诊断统计保存在 context result 中：

```text
max_pair_cost
max_map_cost
max_corridor_cost
colliding_branch_count
near_obstacle_branch_count
near_pair_branch_count
free_corridor_aligned_branch_count
map_unavailable
```

## 参数策略

P3 参数采用中等粒度暴露。必要可调项写入对应 `*Params` 默认值，并由 `loadParameters()` 使用三参数 `pnh_.param(name, target, default_value)` 导入。

建议参数：

```text
prediction_enable_interaction_context
prediction_pair_interaction_weight
prediction_map_interaction_weight
prediction_corridor_interaction_weight
prediction_interaction_safe_distance
prediction_near_obstacle_distance
prediction_ttc_safe_time
prediction_free_corridor_query_radius
prediction_max_correction_distance
prediction_max_deceleration_ratio
prediction_interaction_energy_tau
```

公式内部常数保持实现内聚，不全部暴露到 YAML，避免配置文件膨胀。

## 交互能量

P3 总能量由三类代价组成：

```text
total_energy =
  w_pair * pair_cost
+ w_map * map_cost
+ w_corridor * corridor_cost
```

### 目标间代价

目标间交互首版使用可解释几何项：

```text
pair_cost =
  close_distance_penalty
+ ttc_penalty
+ opposing_or_crossing_penalty
```

- `close_distance_penalty`：未来采样点距离低于目标尺寸膨胀后的 `prediction_interaction_safe_distance` 时增大。
- `ttc_penalty`：相对速度指向接近，且预计在 `prediction_ttc_safe_time` 内过近时增大。
- `opposing_or_crossing_penalty`：用相对速度夹角区分迎面、交叉和追随；迎面和交叉高于同向追随。

首版不做复杂博弈优先级，只对明显冲突分支降权，并给出小幅减速或侧向避让建议。

### 地图代价

地图约束包含占据/近障碍软代价和 free corridor 方向先验：

```text
map_cost =
  occupied_penalty
+ near_obstacle_penalty

corridor_cost =
  heading_to_free_corridor_penalty
```

- `occupied_penalty`：预测点或目标尺寸膨胀半径进入 UFOMap occupied 区域时增大。
- `near_obstacle_penalty`：预测点接近 occupied 区域但未穿障时给出软惩罚。
- `heading_to_free_corridor_penalty`：局部可通行方向明确时，分支航向越背离该方向，代价越高。

free corridor 方向由局部采样估计；如果方向不明确，不生成方向修正，只保留 occupied/near-obstacle 软代价。

## 概率重权重与轻量修正

每个目标内对每个 branch 应用：

```text
branch.probability *= exp(-total_energy / tau)
```

随后对该目标的保留分支重新归一化。若所有分支都被强烈惩罚，仍保留概率最高分支，避免输出空预测。

轻量轨迹修正只作用于明显高风险分支，并受参数上限约束：

- 穿障或近障碍：生成沿推离障碍方向的受限位置偏置 hint。
- 迎面或过近：生成受限减速或侧向避让 hint。
- free corridor 明确：生成朝可通行方向的小幅方向偏置 hint。

这些 hint 不直接逐点改写预测位置，而是在 predictor 内被平滑成随时间连续的控制 profile，再交给运动模型重新 rollout。协方差不因为交互代价被人为缩小。若后续认为修正代表更强假设，可以小幅放大对应分支未来过程不确定性；P3 首版先修正概率，并通过平滑控制重新生成均值轨迹。

## 数据流

P3 保持两段式流程，避免交互逻辑侵入 P2 mode 生成：

```text
1. DynamicObjectPredictor 按 P2 生成 candidate branches
2. PredictionInteractionContext 对这些 branches 做交互/地图评估
3. DynamicObjectPredictor 应用 context result：
   - branch.probability *= probability_scale
   - convert point correction hints to smooth control profile
   - rerollout branch points with the original motion model
   - renormalize branches per object
4. buildPredictionMarkers() 仍只画保留后的真实 GMM 分支线
```

地图查询通过只读 adapter 隔离 UFOMap 类型。上下文模块只依赖“给定点和膨胀半径，返回 occupied / near obstacle / free direction”的抽象查询能力。测试中使用假地图 adapter，运行时由 `Ldop` 或上层构造 UFOMap adapter。

## 地图查询与预算策略

### 职责边界

P3 使用现有 `UfomapMapper` 的只读查询能力，但 predictor 和 `PredictionInteractionContext` 都不直接访问 UFOMap runtime。`PredictionInteractionContext` 定义抽象查询语义，例如“点是否占据”“邻域是否靠近障碍”“局部可通行方向是什么”；运行时由持有 `UfomapMapper` 实例的 `Ldop` 或上层对象把这些语义适配到具体 UFOMap 查询函数。

不让 `PredictionInteractionContext` 自己构造 concrete UFOMap adapter，原因是：

- `UfomapMapper` 的生命周期和线程锁由 `Ldop` 当前主流程管理，context 不应反向拥有或复制地图模块。
- context 需要在测试中使用假地图 adapter，不能硬依赖 UFOMap runtime 和 ROS 节点环境。
- 第四阶段 risk/local map builder 也要复用同一套场景证据接口；把 adapter 注入而不是内建在 context 里，更容易让 P3/P4 共用。
- 地图不可用时，上层可以传入空 adapter 或 unavailable 状态，context 明确退化为 `map_cost = 0`。

### 查询接口选择

具体 UFOMap 查询应按场景分层：

| 场景 | 建议查询 | 用途 |
|------|----------|------|
| 预测采样点或目标膨胀中心是否落在 occupied 体素 | `UfomapMapper::queryNode(point)` | 保留为抽象点查询能力；当前性能优化后的 P3 主路径不逐点调用它 |
| 预测点是否位于已知 free 区域 | `UfomapMapper::seenFree(point)` | 区分 free 与 unknown/occupied，用于 free corridor 采样和未知空间保守处理 |
| 预测点膨胀半径内是否有静态 occupied/near obstacle 节点 | 新增中性只读区域查询封装，例如 `UfomapMapper::queryLocalOccupied(query)`，内部使用 UFOMap 空间谓词 `Intersects(Sphere(center, radius))`、`Leaf(depth)`、`HitsMin(1)` | 做静态近障碍软代价、最近障碍距离近似和 occupied overlap 估计 |
| 局部 free corridor 方向估计 | 每个分支代表点周围 8 个水平 `seenFree()` 探针 | 比较候选方向上的 free 证据，得到 `free_corridor_direction` 和置信度；不使用 `points * directions` 查询 |

P3 不使用 `setNodeLabel()`、`clearAllLabels()` 等写接口，也不把 `queryDynamicCandidates()` 作为静态近障碍查询主接口。`queryDynamicCandidates()` 当前带有 dynamic candidate / label / seenFree 语义，更适合当前动静态分割或动态候选点流程；P3 的地图代价需要查询静态占据或近障碍区域，应新增命名中性的只读区域查询封装。该封装可以直接在 `UfomapMapper` 内部使用 UFOMap 的区域查询能力，例如 `map.query(Leaf && HitsMin && Intersects(...))`，必要时后续再补最近邻查询或批量查询接口。

### 静态/动态来源分离

静态地图约束和动态目标交互的来源必须分离：

- 静态障碍、近障碍和 free corridor 来自 `UfomapMapper` 管理的 UFOMap 静态/占据查询。
- 动态目标之间的距离、TTC、occupied overlap 和风险来自 tracker 传入的 `TrackPredictionInput[]`、目标 `size` 以及 predictor 已生成的未来 GMM 分支。
- 不从 UFOMap 的动态体素或动态候选查询中反推未来动态目标风险；这类信息会比 tracker 的稳定 ID、模型状态、尺寸和预测分支更粗糙，也容易把当前帧动态点云语义和未来预测语义混在一起。

### 分支级查询预算

P3 不缓存一整份 UFOMap 地图：

- UFOMap 已由 `UfomapMapper` 维护，并用读写锁保护。
- 完整复制地图会增加内存和同步成本，并可能让预测使用过期地图。
- P3 只需要未来分支附近的局部占据、近障碍和可通行方向信息。

当前实现采用分支级查询预算，而不是 `LocalOccupiedCacheKey` / `unordered_map` 形式的单帧局部缓存：

- map cost 每个预测分支只做一次 `queryLocalOccupied()`：构造覆盖短时 rollout、目标外接圆和近障碍半径的局部查询球，再在内存中对各采样点计算 signed distance、occupied overlap 和 near-obstacle 代价。
- free corridor 每个预测分支只做 8 个水平 `seenFree()` 探针：使用分支时间中点作为代表点，避免查询次数随 `points * directions` 放大。
- 这个预算比缓存更稳定：它直接限制地图查询上限，不依赖不同分支是否刚好命中相同离散 key。若后续 LDOP 第四阶段风险地图或 3D UAV 场景需要 26 邻域，应作为可配置增强，并重新评估 `applyInteractionContext` 耗时。

### 性能扩展

若后续性能瓶颈来自反复获取 UFOMap 读锁，可以在 `UfomapMapper` 侧补充批量只读查询接口，让 adapter 在一次读锁内完成一组局部查询；这仍不是复制地图，而是优化查询路径。

## 边界情况

- 没有地图或地图查询失败：`map_cost = 0`，只使用目标间交互；诊断记录 `map_unavailable`。
- 单目标：`pair_cost = 0`，只使用地图约束。
- 所有分支都被强烈惩罚：保留概率最高分支并重新归一化。
- free corridor 不明确：不做方向修正，只保留 occupied/near-obstacle 软惩罚。
- 漏检延续目标：不特殊删除，交互/地图代价继续作用，RViz 透明度沿用现有 coasting 逻辑。
- 地图修正和目标间修正方向冲突：优先避免穿障和近障碍；目标间侧向避让作为次级建议。

`TrackPredictionInput` 和 prediction branches 仍按内部契约处理，用 assert 表达状态维度等不变量。地图查询属于跨模块边界，必须显式处理可用性，不能让查询失败中断 predictor 输出。

## 测试与验证

P3 验证分三层。

### `PredictionInteractionContext` 单元测试

使用假地图 adapter，构造不依赖 ROS master 的关键场景：

- 两目标交叉：冲突分支 `pair_cost` 高于无冲突分支。
- 迎面接近：TTC 低的分支被降权，并给出减速或侧向修正建议。
- 追随同向：近距离有代价，但低于迎面/交叉。
- 穿障分支：`map_cost` 高，`colliding_branch_count` 增加。
- 近障碍分支：软代价高于自由空间但低于穿障。
- free corridor：朝可通行方向的分支 `corridor_cost` 更低，不明确时不产生方向修正。
- 地图不可用：地图代价为 0，目标间代价仍正常。

### predictor 集成测试

复用现有 `test_dynamic_object_predictor.cpp` 或新增聚焦测试文件：

- 分支概率经过交互能量后仍归一化。
- 极端惩罚时至少保留一个分支。
- 轻量修正不超过配置上限。
- marker 仍只包含 `dynamic_prediction_line`，不出现 sample marker 或默认诊断 marker。
- `model_covariance` 不因交互代价被缩小。

### 包级验证

执行：

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

完成标准：

- 核心交互/地图场景测试覆盖关键行为。
- 包级构建和测试通过。
- 预测话题结构不变。
- RViz marker 语义仍只表达真实 GMM 分支中心状态线。
- 不恢复 sample marker，不新增默认诊断话题。
