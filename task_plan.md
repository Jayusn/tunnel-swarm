# 第三阶段动态目标轨迹预测规划

## 目标

基于第二阶段已经稳定输出的动态目标 ID、目标类别、运动模型、模型原生状态及协方差和历史轨迹，逐步实现动态目标短时轨迹预测。预测阶段应独立输出未来物体中心轨迹、轨迹概率分布和 RViz 可视化，不把未来轨迹数组回填到当前跟踪对象消息中。

## 当前状态

- 当前阶段：规划记录已重置为第三阶段轨迹预测专用记录。
- 本轮目标：基于当前 P2 已完成事实，使用 brainstorming 细化 P3/P4 后续需求，并用规划文件持续记录第三阶段后续推进。
- 已完成基础：第一阶段动静态点云分割与静态地图、第二阶段动态点云聚类检测跟踪均已完成。
- 已有预测输入：`DynamicObjectTrackerFrameResult::prediction_inputs` 和 `TrackPredictionInput` 已在 tracker 内部预留，包含稳定目标 ID、类别、运动模型、模型原生状态及协方差、历史样本和生命周期信息。
- 已有文档：P1 设计规格 `docs/superpowers/specs/2026-05-14-dynamic-object-trajectory-prediction-design.md`，P1 实现计划 `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-implementation-plan.md`，P2 设计规格 `docs/superpowers/specs/2026-05-18-dynamic-object-intent-gmm-p2-design.md`，P2 实现计划 `docs/superpowers/plans/2026-05-18-dynamic-object-intent-gmm-p2-implementation-plan.md`。
- P3 设计规格：`docs/superpowers/specs/2026-05-19-dynamic-object-interaction-aware-gmm-p3-design.md` 已写入并按地图查询/分支级预算边界完成整理。
- P3 实现计划：`docs/superpowers/plans/2026-05-19-dynamic-object-interaction-aware-gmm-p3-implementation-plan.md` 已写入并完成代码落地。
- 已实现：独立 predictor 模块、预测消息、预测话题、预测 marker、轨迹外推单分支预测、航向/意图 GMM 多分支预测，以及目标间/目标-静态地图交互感知重权重、分支级地图查询预算与平滑控制 rerollout。
- 尚未实现：闭环预测误差统计、NIS 校准和概率反馈。
- 安排文档：`docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`。

## 阶段

| 阶段 | 状态 | 内容 |
|------|------|------|
| 1 | complete | 恢复旧规划上下文，确认第二阶段已完成，重置三份规划记录为第三阶段专用记录 |
| 2 | complete | 梳理第三阶段预测输入、系统过程、输出和闭环评估目标 |
| 3 | complete | 新建轨迹预测安排文档，明确四个实现阶段和当前实现进展 |
| 4 | complete | 按已生成实现计划落地轨迹外推，输出单条中心预测轨迹 |
| 5 | complete | 引入航向/意图，输出简化高斯混合多轨迹 |
| 6 | complete | 引入目标间和目标-地图交互，输出完整高斯混合多轨迹 |
| 7 | pending | 建立闭环评估与概率反馈，按误差和不确定性更新各轨迹概率 |
| 8 | complete | 预测模块完善：耗时打印 + 四元数归一化 |
| 9 | complete | 根据 2026-05-18 创新方案和当前代码事实，同步更新第三阶段轨迹预测安排文档 |
| 10 | complete | 根据当前 P2 实现收口，更新设计规格、实现计划、创新笔记、总安排和根规划记录 |
| 11 | complete | 将 P3 交互轨迹修正从逐点位置改写改为平滑控制 profile 后重新 rollout |
| 12 | complete | 将 P3 交互上下文地图/corridor 查询从逐点查询优化为分支级查询预算 |

## 实现阶段安排

| 实现阶段 | 核心输入 | 核心方法 | 输出形态 | 验证重点 |
|----------|----------|----------|----------|----------|
| 第一阶段：轨迹外推 | 当前模型原生状态及协方差、短历史 | 按 `motion_model_type` 解释预测所需运动学量，沿时间采样做轨迹外推，并传播模型原生协方差 | 单目标单条中心预测轨迹 | 位置误差、位移误差、耗时、预测方向是否与当前运动趋势一致 |
| 第二阶段：轨迹外推 + 航向/意图 | 第一阶段输入 + 历史轨迹 + 类别/模型特征 | 根据历史速度方向、类别和运动模型生成少量候选意图，形成简化 GMM | 单目标多条候选轨迹，每条带概率和协方差 | 多模态覆盖率、概率归一化、主轨迹误差、耗时 |
| 第三阶段：轨迹外推 + 航向/意图 + 交互 | 第二阶段输入 + 目标间相对关系 + 静态地图约束 | 引入目标间避让、目标-地图碰撞/可通行约束，修正轨迹均值和概率 | 完整 GMM 多轨迹分布 | 交互场景合理性、地图约束有效性、概率分布稳定性 |
| 第四阶段：闭环概率反馈 | 预测结果 + 后续真实跟踪输出 + 运行耗时 | 用位置误差、位移误差和不确定性一致性反馈各模式概率 | 评估报告、在线概率校正状态 | ADE/FDE、NLL 或等价不确定性指标、单帧耗时 |

## 决策记录

| 时间 | 决策 | 原因 |
|------|------|------|
| 2026-05-14 | 第三阶段预测走独立预测消息和话题 | 当前 `DynamicObject.msg` 表达当前跟踪状态，未来预测应保持独立，避免混淆当前状态和未来分布 |
| 2026-05-14 | 第一版先实现轨迹外推单轨迹 | 这是最小可验证闭环，可直接复用 tracker 的模型原生状态及协方差；CTRA 等转弯模型并不是线性外推 |
| 2026-05-14 | 预测消息一开始就按多预测分支结构设计 | 轨迹外推阶段只填一条 `trajectory_extrapolation` 分支、概率为 1.0；后续加入航向/意图和交互预测时不破坏预测话题接口 |
| 2026-05-14 | 默认预测未来 1.0 秒，采样间隔复用 `tracking_default_dt` | 默认短时预测先覆盖更近未来；采样时间粒度与 tracker 默认时间步长保持一致，避免新增重复参数 |
| 2026-05-14 | 漏检延续但尚未删除的稳定轨迹继续预测 | 与 tracker 当前仍发布 coasting 轨迹的策略保持一致；消息保留，RViz marker 用更低透明度表达不确定性 |
| 2026-05-14 | 不为低速或近似静止目标单独设置退化策略 | 预测模块只消费 tracker 已确认的动态目标；低速仍属于动态目标状态范围，按当前运动模型状态正常外推 |
| 2026-05-14 | 预测消息保存模型原生状态和模型原生协方差 | 与当前 `DynamicObject.msg` 的 `model_state/model_covariance` 语义保持一致；RViz 椭球只从预测状态前三维位置协方差派生 |
| 2026-05-14 | 未来采样点单独建消息类型 | 用 `DynamicObjectPredictionPoint` 同时承载 `time_from_start`、`model_state` 和 `model_covariance`，避免并行数组错位 |
| 2026-05-14 | 预测分支类型改为通用行为族方向 | 不把分支类型绑定到实现阶段或具体方向；上升/下降、前进/后退、转弯这类成对行为应由同一行为族加有符号参数表达 |
| 2026-05-14 | 历史 P1 版本曾用 `BEHAVIOR_BASELINE` 表达基准预测 | 第一阶段用它表示不额外假设意图、仅按当前运动模型外推；P2 已移除该枚举 |
| 2026-05-14 | `behavior_value` 使用归一化有符号行为强度 | 范围为 `[-1.0, 1.0]`；真实轨迹仍由采样点模型原生状态表达，行为强度只辅助解释预测分支偏向 |
| 2026-05-14 | 模型不确定性和规划影响权重分开表达 | `model_covariance` 按运动模型真实传播；预测尾部对规划/RViz 的影响通过 `influence_weight` 衰减，不篡改协方差 |
| 2026-05-14 | 影响权重前半段保持 1.0，后半段衰减到 0.3 | 分段时间直接取预测总时长的一半；最小尾部影响权重写死为 0.3，不额外暴露参数 |
| 2026-05-14 | 不设置预测目标数量限制 | predictor 对 tracker 输出的全部稳定预测输入做预测，稳定性和数量边界由 tracker 控制 |
| 2026-05-14 | 历史 P1 版本 RViz 预测可视化包含轨迹线和不确定性椭球 | P2 当前已收口为真实分支中心状态线；椭球语义仍可作为后续风险/评估派生视图参考 |
| 2026-05-14 | 历史 P1 椭球由 3x3 位置协方差特征分解生成 | 若后续重新引入不确定性视图，应继续保留主轴方向和相关性 |
| 2026-05-14 | 历史 P1 椭球默认显示 1σ | 若后续重新引入不确定性视图，轴长仍按 RViz `SPHERE` 直径语义处理 |
| 2026-05-14 | 预测目标消息保留 `size` 和 `matched_in_current_frame` | `size` 服务后续风险膨胀和可视化；`matched_in_current_frame` 表达预测是否来自当前帧匹配目标 |
| 2026-05-14 | 预测目标消息复用当前目标类别和运动模型常量 | `DynamicObjectPrediction.msg` 中保留与 `DynamicObject.msg` 一致的 `CLASS_*` 与 `MOTION_MODEL_*` 常量，避免下游维护第二套数字映射 |
| 2026-05-14 | predictor 自己读取 `tracking_default_dt` 作为采样间隔 | 采样间隔参数名与 tracker 复用，但模块各自读取，避免为了传参改动 tracker/Ldop 现有边界 |
| 2026-05-14 | 预测结果和预测 marker 使用独立可配置话题 | 新增 `dynamic_predictions_topic` 和 `dynamic_prediction_markers_topic`，默认分别为 `/ldop/dynamic_object_predictions` 与 `/ldop/dynamic_prediction_markers` |
| 2026-05-14 | predictor 返回帧级结果并由 `Ldop` 统一发布 | `DynamicObjectPredictorFrameResult` 包含预测消息、预测 marker 和 timing；`Ldop::processFrame()` 在 tracker 后调用，`publishFrame()` 统一发布 |
| 2026-05-14 | predictor 输入按内部契约处理 | `TrackPredictionInput` 来自 tracker 内部稳定输出，不按外部不可信输入做重复兜底；状态维度等使用 assert 或明确契约表达 |
| 2026-05-14 | 修改消息和 C++ 后至少执行 `catkin build ldop --no-status` | 新增 `msg` 会触发消息生成，构建验证是第一阶段实现后的最低验证范围 |
| 2026-05-14 | 第一阶段采用独立 predictor 并复用 `MotionModel` | predictor 不复用 `KalmanFilterBase`，避免重新初始化协方差或混入 tracker 自适应/限幅语义；直接从输入状态和协方差做只读轨迹外推 |
| 2026-05-14 | 未来传播使用运动模型基础 `Q` | 协方差初值来自 tracker 当前自适应后的 `model_covariance`，但未来传播新增过程噪声使用运动模型基础 `Q`，不读取 tracker filter 内部自适应噪声状态 |
| 2026-05-14 | 设计规格文档已写入但不提交 | 仓库指令要求任务过程中不创建中间提交，除非任务完成且验证通过或用户明确要求 |
| 2026-05-14 | 第一阶段轨迹外推实现计划已写入 | `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-implementation-plan.md` 已覆盖消息、共享运动模型构建、predictor、marker、测试、Ldop 集成和验证步骤 |
| 2026-05-14 | 多轨迹概率分布分两步实现 | 先用航向/意图形成简化 GMM，再引入交互约束，降低一次性设计复杂度 |
| 2026-05-14 | 闭环反馈放到第四阶段 | 需要先有预测输出和后续真实跟踪结果，才能可靠评估和更新概率 |
| 2026-05-18 | 第三阶段后续路线以 trajectory-level GMM 为核心 | 当前 P1 基础轨迹外推已完成，下一步应复用现有多分支消息结构实现航向/意图多轨迹，而不是重做 P1 |
| 2026-05-18 | 闭环反馈使用预测残差/NIS 更新概率和不确定性 | 该机制能把上一轮预测是否解释当前观测的信息反馈到下一轮分支概率和过程噪声尺度，且不反向破坏 tracker 状态 |
| 2026-05-19 | P2 航向/意图 GMM 已完成 | 当前 predictor 主路径已生成多条行为族分支，概率归一化后输出 branch-level trajectory-level GMM |
| 2026-05-19 | 当前 P2 不再保留 `BEHAVIOR_BASELINE` | 所有预测分支都使用纵向、垂向、转向或横向行为族；P1 基线消融用分支数量和概率阈值控制 |
| 2026-05-19 | P2 RViz 只显示真实分支中心状态线 | 不再显示 P1 椭球或仅用于可视化的采样轨迹；sample marker 没有实际使用价值，且会让可视化输出和预测消息语义分离 |
| 2026-05-19 | P2 分支裁剪参数作用在 GMM component 上 | `prediction_max_branches` 和 `prediction_min_branch_probability` 控制每目标保留多少预测分支；每个分支内部只有一条中心状态轨迹和对应协方差序列 |
| 2026-05-19 | 继续第三阶段前先细化 P3/P4 需求 | P2 已完成并收口，后续涉及交互输入边界、地图查询、概率更新和评估缓存；这些会影响接口和验证范围，设计批准前不修改实现代码 |
| 2026-05-19 | P3 首版同时纳入目标间交互和静态地图约束 | 用户选择目标间交互与 UFOMap 静态地图约束一起进入交互预测设计，后续需明确概率重权重和轨迹修正边界 |
| 2026-05-19 | P3 交互作用范围采用“概率为主 + 轻量轨迹修正” | 保留 P2 分支运动模型语义，同时允许对明显穿障、过近或迎面冲突的分支做小幅避让、减速或贴合可通行空间修正 |
| 2026-05-19 | P3 地图约束采用占据/近障碍软代价 + free corridor 方向先验 | 地图约束既要降低穿障和贴近障碍分支概率，也要用局部可通行方向引导轻量修正，而不是只做硬删除 |
| 2026-05-19 | P3 交互上下文采用独立 `PredictionInteractionContext` 模块 | 该模块聚合目标间关系、UFOMap 静态约束、free corridor、分支交互代价和局部风险摘要，服务预测分支重权重和轻量修正，也为第四阶段类 ROG-Map/risk map builder 提供可复用证据；它本身不是最终 planner-facing 地图 |
| 2026-05-19 | `PredictionInteractionContext` 输出 predictor 内容和内部诊断统计 | 输出分支交互/地图代价、总能量和轻量修正建议，同时保留每帧最大代价、穿障分支数、近碰撞分支数、free corridor 命中率等内部统计；首版不新增默认 ROS 诊断话题 |
| 2026-05-19 | P3 参数采用中等粒度暴露 | 暴露总开关、目标间/地图权重、安全距离、近障碍距离、TTC 安全阈值、free corridor 查询半径、轻量修正最大位移/减速比例和能量温度 `tau`；不把每个公式细节都变成 YAML 参数 |
| 2026-05-19 | P3 验证采用单元测试 + 包级构建/运行测试 | 构造交叉、迎面、穿障、近障碍和 free corridor 场景验证代价、概率重权重和轻量修正方向；执行 `catkin build`、`catkin run_tests` 和 `catkin_test_results`，并回归预测 marker 仍只表达真实分支线 |
| 2026-05-19 | P3 采用独立上下文方案进入正式设计 | 在“直接扩展 predictor”“独立交互上下文”“提前做规划风险上下文雏形”三种方案中，确认采用独立 `PredictionInteractionContext`，避免 predictor 膨胀，并为第四阶段独立风险/规划地图构建保留清晰边界 |
| 2026-05-19 | `PredictionInteractionContext` 需要服务第四阶段类 ROG-Map 构建 | 该模块作为 P3/P4 共享的场景证据层，输出分支风险等级、最近障碍距离、occupied overlap、free corridor 方向/置信度、最近目标距离和最小 TTC 等摘要；第四阶段 builder 再负责转换为 planner-facing 时间索引风险/局部地图 |
| 2026-05-19 | P3 实现计划已写入 | 实现计划覆盖 `PredictionInteractionContext`、UFOMap 静态区域查询封装、predictor 重权重/轻量修正、`Ldop` adapter、YAML 参数和测试验证；按仓库规则不包含中间提交步骤 |
| 2026-05-19 | P3 交互感知预测实现已落地并验证 | 已实现 `PredictionInteractionContext`、UFOMap 静态只读邻域查询、predictor 重权重/轻量修正、`Ldop` adapter、YAML 参数和回归测试；`catkin build`、`catkin run_tests` 和 `catkin_test_results` 均通过 |
| 2026-05-20 | P3 交互均值轨迹修正改为平滑控制后重新 rollout | 保留 `PredictionInteractionContext` 的逐点风险证据，但 predictor 不再直接改写离散采样点位置，而是先平滑成速度/减速/方向控制 profile，再交给 motion model 重新积分，减少 planner-facing 轨迹折线尖角 |
| 2026-05-20 | P3 地图/corridor 查询预算收敛到分支级 | map cost 每分支只做一次局部 occupied 查询，再在内存中逐点计算距离；corridor 每分支只做 8 个 seenFree 探针，避免 `points * directions` 查询爆炸 |
| 2026-05-20 | P3 单帧 query cache 方案被分支级查询预算取代 | 前期 `LocalOccupiedCacheKey` / `unordered_map` 缓存设想依赖命中率；现场瓶颈来自查询数量爆炸，因此当前实现直接约束每分支查询上限，更利于把 `applyInteractionContext` 压到实时预算内 |
| 2026-05-20 | P3 新增内部 `branch_risk_level` 摘要 | `PredictionBranchInteraction` 已加入 `PredictionBranchRiskLevel::{Low, Medium, High, Critical}`；该字段不进入 ROS msg，供 P4 快速分层，连续风险强度仍由 cost、TTC、最近障碍距离等指标计算 |

## 待确认问题

| 优先级 | 问题 | 默认建议 |
|--------|------|----------|
| 已收口 | 预测消息字段命名和结构 | 当前预测数组、目标预测、预测分支和预测采样点消息已存在；P2 仅清理旧 `BEHAVIOR_BASELINE` 枚举，不新增 sample 字段 |
| 高 | 预测 horizon 与采样间隔 | 默认 1.0 秒 horizon；采样间隔默认复用 `tracking_default_dt`，当前配置中为 0.1 秒 |
| 已收口 | 轨迹外推单轨迹的不确定性传播方式 | 消息保存完整模型协方差；P2 当前不显示椭球，后续风险/评估可从前三维位置协方差派生不确定性视图 |
| 已收口 | 代码实现顺序 | 下一步按 P3 实现计划落地目标间/地图交互预测；P4 预测缓存、评估指标和 NIS/概率反馈仍保持后续阶段 |
| 已收口 | 航向/意图候选表达 | 当前用通用行为族和有符号参数表达候选预测分支，例如纵向减速/停止、垂向爬升/下降、左右转向和横向偏移；不继续使用 `BEHAVIOR_BASELINE` |
| 已收口 | 交互模型复杂度 | P3 首版同时使用目标间规则型几何交互和静态地图约束；学习方法只作为后续扩展，不进入第三阶段首版实现 |
| 已收口 | P3 交互作用范围 | 以分支概率重权重为主，对明显冲突分支允许小幅修正中心轨迹；修正先转为时间平滑控制再重新 rollout，不让交互逻辑逐点替代运动模型 |
| 已收口 | P3 地图约束策略 | 同时使用 UFOMap 占据/近障碍软惩罚和局部 free corridor 方向先验；不采用只做硬碰撞过滤的首版 |
| 已收口 | P3 交互上下文接入方式 | 新建独立 `PredictionInteractionContext` 聚合交互/地图证据；predictor 消费上下文，第四阶段规划地图模块再消费预测结果，不依赖 predictor 内部 helper |
| 已收口 | P3 交互上下文输出边界 | 给 predictor 的代价/修正建议和内部诊断统计都保留；诊断用于日志、测试和后续评估，不新增默认 ROS 话题 |
| 已收口 | P3 参数暴露策略 | 采用中等粒度参数；必要可调项进入 `*Params` 默认值和 `loadParameters()`，公式内部常数保持实现内聚 |
| 已收口 | P3 验证范围 | 单元测试覆盖交互/地图关键场景，加 `catkin build`、`catkin run_tests`、`catkin_test_results`；RViz 手工场景不作为完成硬门槛 |
| 已收口 | P3 总体落地方案 | 采用独立 `PredictionInteractionContext`，predictor 消费上下文输出；不直接把地图查询和交互逻辑塞进 predictor |

## 沟通约束

- 面向用户说明当前工作时，不用依赖过程文档的阶段简称；直接描述“轨迹外推单轨迹预测”“航向/意图多轨迹”“目标/地图交互预测”等具体内容。

## 错误记录

| 时间 | 错误 | 处理 |
|------|------|------|
| 2026-05-14 | 默认沙箱缺少 `bwrap`，首次读取技能文件失败 | 已按审批流程以 escalated 方式读取本地文件并继续执行 |
| 2026-05-14 | `apply_patch` 直接删除重建 `task_plan.md` 失败，且一次只改了文件标题 | 改用受控脚本整体重写三份规划记录和新安排文档；重写后继续做格式检查 |
| 2026-05-14 | 记录低速目标决策时补丁上下文不匹配 | 重新读取相关段落后使用精确上下文更新规划文件 |
