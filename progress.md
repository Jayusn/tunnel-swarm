# 第三阶段动态目标轨迹预测进度记录

## 2026-05-14

- 用户给出第三阶段轨迹预测总体思路：由动态目标历史状态和运动学特征出发，先做基础外推，再加入航向/意图，再加入目标间和目标-地图交互，最终输出轨迹概率分布，并用闭环误差反馈概率。
- 已使用 `planning-with-files-zh` 恢复旧规划记录，确认旧记录主要是第二阶段动态对象跟踪、分类和多模型 Kalman 的历史。
- 已按用户要求重置 `task_plan.md`、`findings.md` 和 `progress.md`，使三份文件独立记录第三阶段轨迹预测规划进度。
- 已检查当前接口事实：
  - `DynamicObject.msg` 当前表达 tracker 当前状态，不包含预测数组。
  - `TrackPredictionInput` 已在 tracker 内部预留预测输入，包含 ID、类别、模型原生状态及协方差、历史和生命周期信息。
  - 当前没有独立预测消息、预测话题和预测 marker。
- 曾尝试用 `apply_patch` 删除并重建规划文件，但当前工具无法删除 `task_plan.md`；随后改用受控脚本整体重写三份规划记录。
- 已新建安排文档：`docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`。
- 已执行 `git diff --check -- task_plan.md findings.md progress.md docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`，结果通过。
- 本轮只做规划和文档，不修改 C++ 实现，不运行构建测试。

## 2026-05-14 第一阶段实现设计启动

- 用户明确要求开始轨迹预测第一阶段实现，并指定使用 `superpowers:brainstorming` 与 `planning-with-files-zh`。
- 已恢复现有三份规划文件，并按技能约束确认：设计批准前不修改 C++、ROS msg、launch 或构建配置实现。
- 本轮当前目标是收敛“第一阶段轨迹外推单轨迹预测”的设计规格，再进入实现计划。
- 工具环境事实：默认沙箱缺少 `bwrap`，读取本地文件需要走 escalated 执行；后续工具调用继续记录重要错误和替代方案。
- 已阅读第三阶段安排文档、关键符号检索结果、文件结构、最近提交和工作区状态；确认预测准备提交存在，预测第一阶段仍未实现，当前未发现用户未说明的代码改动。
- 已阅读 `Ldop` 主流程、tracker 预测输入、运动模型、Kalman 预测逻辑、消息和构建配置；关键发现已同步到 `findings.md`。
- 用户提醒不要用过程文档阶段简称描述当前工作，因为规划文件和安排文档里的阶段标签都可能变化；已将当前规划记录中的相关简称改为明确能力描述。
- 用户询问英文轨迹分布术语的含义；已统一文档中文术语为“预测分支”，表示同一目标未来的一种候选走法，例如保持当前方向、轻微左偏或轻微右偏。
- 用户确认预测消息采用多预测分支结构；轨迹外推单轨迹阶段只填一条 `trajectory_extrapolation` 分支，概率为 1.0。
- 用户确认默认预测未来 1.0 秒，每 0.1 秒一个采样点；采样间隔尽量复用 `tracking_default_dt`，避免新增重复参数。
- 用户同意漏检延续但尚未删除的稳定轨迹继续预测；预测消息保留，RViz marker 降低透明度。
- 用户明确低速或近似静止目标仍属于动态目标范围，不需要单独考虑或退化处理；轨迹外推按当前运动模型状态正常预测。
- 记录低速目标决策时第一次补丁上下文不匹配，已重新读取相关段落后修正。
- 复扫发现安排文档仍有低速退化旧表述，已同步改为“不按低速或近似静止单独退化”。
- 用户确认不确定性可视化使用三轴椭球表达，但预测消息中的状态和不确定性必须与运动模型原生状态/协方差保持一致；消息层不只保存位置协方差。
- 复扫发现安排文档仍有“位置协方差或风险半径”的旧输出表述，已改为消息保存模型原生状态/协方差，位置协方差仅作为派生视图。
- 用户确认未来预测采样点单独建消息类型，用一个采样点消息同时承载 `time_from_start`、`model_state` 和 `model_covariance`。
- 用户提出预测分支类型可以先补完整；已确认消息常量可预留完整候选轨迹语义，但轨迹外推阶段只生成轨迹外推分支。
- 用户指出“线性外推”命名不准确：只有保持直行时才适用，目标航向变化或转弯时应称为“轨迹外推”；已开始同步文档术语。
- 用户进一步指出预测分支类型不应按具体方向硬编码：上升/下降是一组行为，前进/后退是一组行为，转弯是一组行为；前进/后退和上升/下降都可用可变加速度表达，例如减速前进仍是纵向行为。
- 用户明确除第一阶段外，不希望继续使用 `BEHAVIOR_BASELINE`；已记录为第一阶段基准预测专用行为类型，后续多意图/交互预测不再使用。
- 用户确认 `behavior_value` 使用 `[-1.0, 1.0]` 归一化有符号行为强度；真实轨迹仍由预测采样点的模型原生状态表达。
- 用户提出预测尾部不确定性若持续放大会压缩无人机规划可通行区域；已确认采用“模型协方差真实传播 + `influence_weight` 衰减规划/RViz 影响”的设计。
- 用户确认影响权重分段时间不单独配置，直接取整个预测时长的一半；后半段衰减到 0.3，且最小影响权重写死，不新增参数。
- 用户确认不设置预测目标数量限制；predictor 预测 tracker 给出的全部稳定目标。
- 用户确认 RViz 预测可视化包含预测轨迹线和每个采样点的不确定性椭球；漏检延续目标整体透明度降低。
- 用户确认不确定性椭球使用 3x3 位置协方差特征分解生成，保留主轴方向和相关性。
- 用户确认不确定性椭球默认显示 1σ，RViz `SPHERE` 轴长用 `2 * sqrt(lambda)`。
- 用户确认预测目标消息保留 `size` 和 `matched_in_current_frame`。
- 用户确认模型不确定性放在预测分支采样点中；顶层 `motion_model_type` 用于解释所有采样点的模型原生状态和协方差布局。
- 用户确认 predictor 自己读取 `tracking_default_dt` 作为采样间隔；参数名复用，但不通过 `Ldop` 注入。
- 用户确认新增预测结果话题参数 `dynamic_predictions_topic` 和预测 marker 话题参数 `dynamic_prediction_markers_topic`，默认分别为 `/ldop/dynamic_object_predictions` 与 `/ldop/dynamic_prediction_markers`。
- 用户确认 predictor 返回 `DynamicObjectPredictorFrameResult`，包含预测消息、预测 marker 和 timing；`Ldop::processFrame()` 在 tracker 后调用 predictor，并由 `publishFrame()` 统一发布。
- 用户确认实现后至少执行 `catkin build ldop --no-status`；若加测试再运行 `catkin run_tests ldop --no-status` 并查看结果。
- 用户认为 predictor 输入来自 tracker 内部，tracker 正常时维度/有限性问题不应出现；设计改为按内部契约处理，不做外部输入式重复兜底。
- 已给出三种实现方案取舍；用户同意采用“独立 `DynamicObjectPredictor` + 复用 `MotionModel` 做只读轨迹外推”的方案。
- 用户要求一次性收口剩余设计，并明确写入：协方差初值来自 tracker 当前自适应后的 `model_covariance`，未来传播新增过程噪声使用运动模型基础 `Q`，不读取 tracker filter 内部自适应噪声状态。
- 已写入正式设计规格：`docs/superpowers/specs/2026-05-14-dynamic-object-trajectory-prediction-design.md`。
- 已自审规格文档：无占位符；“线性外推”只保留在命名说明中用于解释为什么采用“轨迹外推”；未发现内部矛盾或范围外实现内容。
- 已执行 `git diff --check -- docs/superpowers/specs/2026-05-14-dynamic-object-trajectory-prediction-design.md task_plan.md findings.md progress.md docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`，结果通过。
- 按仓库指令不创建中间提交；等待用户审阅设计规格后再进入实现计划。

## 2026-05-14 第一阶段实现计划恢复检查

- 用户反馈远端 compact 任务中断，怀疑轨迹外推实现计划文档生成失败。
- 已检查规划文件，确认 `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-implementation-plan.md` 已存在，内容完整，共 1045 行。
- 实现计划覆盖范围：新增预测消息、`CMakeLists.txt` 消息注册、共享 Kalman/motion model 参数构建、`DynamicObjectPredictor` 接口与实现、协方差传播、`influence_weight`、RViz 预测线和椭球 marker、聚焦测试、`Ldop` 接入、配置默认值和最终验证。
- 已运行 `session-catchup.py` 检查恢复状态，未输出未同步上下文。
- 已运行 `git diff --check -- task_plan.md findings.md progress.md docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-implementation-plan.md`，结果通过。
- 已同步根目录三份规划记录，使其反映“设计规格和实现计划均已生成，代码实现尚未开始”的当前事实。
- 按用户补充的恢复记录复核后发现，上一轮被自动审核拒绝的“预测目标消息常量收口”确实没有落盘；已把 `CLASS_*` 与 `MOTION_MODEL_*` 常量补入设计规格和实现计划，并删除测试段落里“如果常量不存在则用数字值”的不确定分支。
- 本轮仍只处理规划文档和状态同步，不修改 C++、ROS msg、launch 或构建配置，不运行 catkin 构建。

## 2026-05-15 第一阶段轨迹外推实现完成

- 按实现计划完成第一阶段轨迹外推单轨迹预测的全部代码实现。
- 新增预测消息：`DynamicObjectPredictionPoint.msg`、`DynamicObjectPredictionBranch.msg`、`DynamicObjectPrediction.msg`、`DynamicObjectPredictionArray.msg`。
- 新增共享参数结构 `MultiModelKalmanFilterParams` 和工厂函数 `buildMultiModelKalmanFilterConfig()`、`createMotionModel()`，tracker 和 predictor 各自读取参数再通过共享构建函数转换为运行时配置。
- 新增 `DynamicObjectPredictor` 模块：从 ROS 参数读取预测时长和卡尔曼参数，按采样间隔逐步外推状态和协方差，构建预测消息和 RViz marker。
- 集成到 `Ldop` 主流程：在 tracker 后调用 predictor，发布预测消息和预测 marker。
- 新增测试 `test_dynamic_object_predictor`，验证 CV3D 模型的基线预测点生成和影响权重。
- 构建验证通过：`catkin build ldop --no-status` 成功，测试二进制已编译（运行需要 ROS master）。
- 新增配置：`prediction_horizon`、`dynamic_predictions_topic`、`dynamic_prediction_markers_topic`。

## 2026-05-18 轨迹预测安排文档同步

- 用户要求根据 `docs/note/2026-05-18_intent_prediction_trajectory_innovation.md` 和当前 LDOP 实现状态，修改 `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md`。
- 已将安排文档从“P1 尚未实现”的旧状态更新为“P1 基础轨迹外推已实现”，并把下一步主线明确为“多分支 GMM + 历史航向/意图概率”。
- 已把创新方案中的 trajectory-level GMM、MAP trajectory、交互能量、NIS 闭环概率/过程噪声反馈和多模态评估指标同步到安排文档。
- 本轮只修改规划/进度文档，不修改 C++、ROS msg、launch 或构建配置。

## 2026-05-19 P2 文档收口

- 用户指出此前检查到的“不完整/需要同步”项都是后续已确认的设计改动，要求把相关文档全部更新。
- 已按当前 P2 代码事实同步文档：P2 航向/意图 GMM 已实现，`BEHAVIOR_BASELINE` 已移除，RViz 只显示真实保留分支中心状态线，不再显示椭球或额外采样轨迹。
- 已进一步补充分支语义：`prediction_max_branches` 和 `prediction_min_branch_probability` 作用在 GMM 分支/component 上；每个分支内部只有一条中心状态轨迹和对应协方差序列。
- 已更新 `docs/superpowers/specs/2026-05-18-dynamic-object-intent-gmm-p2-design.md`、`docs/superpowers/plans/2026-05-18-dynamic-object-intent-gmm-p2-implementation-plan.md`、`docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-arrangement.md` 和 `docs/note/2026-05-18_intent_prediction_trajectory_innovation.md`。
- 已同步根目录规划记录：`task_plan.md` 标记航向/意图 GMM 阶段完成，`findings.md` 记录当前 P2 代码事实。
- 进一步明确：删除的是仅用于可视化的额外采样轨迹，因为它没有实际规划或评估消费者；后续主线不默认恢复 sample marker 或 sample 消息字段。

## 2026-05-19 P3/P4 需求细化启动

- 用户要求继续完成 LDOP 第三阶段，并明确先使用 `superpowers:brainstorming` 细化需求，再用 `planning-with-files-zh` 记录规划进展。
- 已读取两个技能说明，确认 brainstorming 的硬门槛：设计获得用户批准前不修改 C++、ROS msg、配置或构建脚本等实现内容。
- 已恢复 `task_plan.md`、`findings.md`、`progress.md`，并运行 `session-catchup.py`；当前没有未同步恢复输出。
- 已阅读第三阶段安排文档、2026-05-18 创新方案、P2 设计规格、P2 实现计划、predictor 头文件/实现、`ldop.yaml`、最近提交和工作区状态。
- 当前判断：P1 轨迹外推和 P2 航向/意图 GMM 已完成；本轮应细化 P3 目标间/目标-地图交互预测和 P4 闭环评估/概率反馈的边界。
- 已把 `task_plan.md` 的阶段 6 标记为 `in_progress`，并在 `findings.md` 记录 P3/P4 相关代码事实。
- 已补读 `DynamicObjectPredictor` 主流程、预测消息、`Ldop::processFrame()` 和 `UfomapMapper` 查询接口；确认 P3 的接口关键是如何把只读交互/地图上下文接入 predictor，而不是修改 tracker 输出或预测消息主结构。
- 已记录：现有预测消息可继续承载重权重后的 branch-level GMM；interaction cost、NIS、评估指标等诊断信息不应默认塞进 planner-facing prediction message。
- 用户选择 P3 首版范围为“目标间交互 + 静态地图约束”同时进入设计。
- 已同步 `task_plan.md` 决策记录和 `findings.md` 本轮用户选择；下一步需要确认交互首版只调概率，还是也允许轻量修正分支中心轨迹。
- 用户选择 P3 交互作用范围为“概率为主，允许轻量修正”：先做分支概率重权重，对明显穿障、过近或迎面冲突分支允许小幅避让、减速或贴合可通行空间修正。
- 已更新 `task_plan.md` 和 `findings.md`，把相关“待确认”措辞改为已确认决策。
- 用户选择 P3 静态地图约束策略为“占据/近障碍惩罚 + 局部 free corridor 方向先验”。
- 已同步 `task_plan.md` 和 `findings.md`，记录地图约束不是只做硬碰撞过滤，而是软代价与可通行方向引导并用。
- 用户确认 P3 交互上下文采用独立 `PredictionInteractionContext` 模块，并要求说明它与第四阶段类 ROG-Map 规划地图构建的关系。
- 已同步 `task_plan.md` 和 `findings.md`：`PredictionInteractionContext` 聚合目标间/地图/free corridor 约束证据并服务 predictor；它不是第四阶段 planner-facing 地图，后续类 ROG-Map 风险/局部地图 builder 应独立消费预测结果和风险摘要。
- 用户选择 `PredictionInteractionContext` 输出边界为“predictor 内容 + 内部诊断统计”，不默认新增 ROS 诊断话题。
- 已同步 `task_plan.md` 和 `findings.md`：上下文输出分支代价、总能量和轻量修正建议，同时保留最大交互代价、穿障分支数、近碰撞分支数、free corridor 命中率等内部统计。
- 用户选择 P3 参数暴露策略为中等粒度：额外暴露 TTC 安全阈值、free corridor 查询半径、轻量修正最大位移/减速比例、能量温度 `tau` 等关键调参项。
- 已同步 `task_plan.md` 和 `findings.md`：参数集中写入对应 `*Params` 默认值并经 `loadParameters()` 导入，公式内部细节不全部暴露到 YAML。
- 用户选择 P3 验证范围为“单元测试 + 包级构建/运行测试”：覆盖交叉、迎面、穿障、近障碍、free corridor 场景，并执行 `catkin build`、`catkin run_tests` 和 `catkin_test_results`。
- 已同步 `task_plan.md` 和 `findings.md`：P3 需要回归预测 marker 仍只是真实 GMM 分支中心状态线，不恢复 sample marker，也不把 RViz 手工场景作为完成硬门槛。
- 用户确认 P3 在三种落地方式中采用独立 `PredictionInteractionContext` 方案。
- 已同步 `task_plan.md` 和 `findings.md`：正式设计从独立上下文方案展开，避免直接扩展 predictor 或过早引入第四阶段规划风险地图雏形。
- 用户确认 P3 架构边界：`PredictionInteractionContext` 位于 tracker/P2 candidate branches 与 predictor 重权重之间，作为预测阶段交互/地图约束上下文；第四阶段类 ROG-Map/risk map builder 独立消费预测结果，不依赖 predictor 内部 helper。
- 用户确认 P3 组件与数据结构：新增 `prediction_interaction_context` 小模块，使用内部 C++ 结构表达参数、每分支交互结果、轻量修正建议和内部诊断统计，不新增 ROS msg 或默认诊断话题。
- 用户确认 P3 交互能量与轻量修正规则：总能量由目标间代价、地图占据/近障碍代价和 free corridor 方向代价组成；概率按 `exp(-energy/tau)` 重权重，高风险分支允许受上限约束的小幅推离、减速或朝可通行方向偏置，协方差不人为缩小。
- 用户确认 P3 数据流和边界处理：predictor 先生成 P2 candidate branches，`PredictionInteractionContext` 做交互/地图评估，再由 predictor 应用概率缩放、轻量修正和目标内归一化；地图查询通过只读 adapter 隔离 UFOMap 类型，缺图或查询失败时退化为只用目标间交互。
- 用户询问 P3 地图查询是否使用 UFOMap 接口、是否需要缓存地图；当时设计明确为通过 `UfomapMapper` 只读查询 adapter 访问 UFOMap、不缓存完整地图。后续性能优化已将“单帧局部缓存”设想替换为分支级查询预算。
- 用户进一步指出地图查询策略应说明为什么由 `Ldop` 或上层构造 adapter，而不是 `PredictionInteractionContext` 自己构建，并明确不同场景使用哪些 UFOMap 查询函数。
- 已更新 P3 设计规格：`PredictionInteractionContext` 定义抽象查询语义；`Ldop`/上层注入 concrete adapter；`queryNode()` 保留为点查询能力，`seenFree()` 用于 free/corridor 采样，近障碍邻域查询应新增中性只读区域查询封装，写接口不用于 P3。后续实现主路径已改为分支级 `queryLocalOccupied()` 和 8 方向 corridor 探针。
- 用户进一步说明新增 `PredictionInteractionContext` 也要给 LDOP 第四阶段类 ROG-Map 地图构建使用；已补充设计边界：该模块作为 P3 与后续 LDOP 第四阶段可复用的场景证据层，输出局部风险摘要，第四阶段独立 builder 再转换为 planner-facing 时间索引风险/局部地图。
- 用户指出 UFOMap 有区域查询接口，且动态体素查询不适合判断预测点膨胀半径内的动态障碍；已澄清设计：静态近障碍使用 UFOMap 区域查询封装，动态目标风险使用 tracker/P2 预测分支和目标尺寸，不从 UFOMap 动态体素反推未来动态风险。
- 用户询问“地图查询与缓存策略”是否需要整理；已将该节整理为职责边界、查询接口选择、静态/动态来源分离、查询预算和性能扩展几个部分，并在后续文档同步中移除对 `LocalOccupiedCacheKey` 缓存实现的默认要求。
- 用户确认 P3 测试与验证设计：`PredictionInteractionContext` 纯单元测试覆盖交叉、迎面、追随、穿障、近障碍、free corridor 和地图不可用；predictor 集成测试覆盖概率归一化、极端惩罚、修正上限、marker 语义和协方差不缩小；最终执行 catkin 构建和测试。
- 已写入 P3 设计规格：`docs/superpowers/specs/2026-05-19-dynamic-object-interaction-aware-gmm-p3-design.md`。
- 已自审 P3 设计规格：精确占位词扫描无 `TBD/TODO/待定/待确认/placeholder/FIXME`；`git diff --check` 通过；未发现规格内部矛盾。按仓库规则未创建中间提交。

## 2026-05-19 P3 实现计划生成

- 用户询问是否还有补充，以及下一步是否使用 `superpowers:writing-plans` 写计划文档。
- 已使用 `superpowers:writing-plans`，基于 P3 设计规格和当前代码接口生成实现计划：`docs/superpowers/plans/2026-05-19-dynamic-object-interaction-aware-gmm-p3-implementation-plan.md`。
- 计划覆盖 `PredictionInteractionContext`、UFOMap 静态只读区域查询封装、predictor 交互重权重/轻量修正、`Ldop` 地图查询 adapter、YAML 参数、CMake 注册和单元/集成测试。
- 已同步 `task_plan.md` 和 `findings.md`，记录 P3 设计与实现计划均已生成，下一步进入执行方式选择。
- 已对实现计划执行精确占位词扫描，未发现问题；已执行 `git diff --check -- docs/superpowers/plans/2026-05-19-dynamic-object-interaction-aware-gmm-p3-implementation-plan.md`，结果通过。

## 2026-05-19 P3 交互感知预测实现

- 用户要求使用 `superpowers:subagent-driven-development` 执行 P3 实现计划。
- 已按计划实现 `PredictionInteractionContext`、UFOMap 中性静态只读邻域查询、predictor 概率重权重/受限均值轨迹修正、`Ldop` UFOMap 查询 adapter、YAML 参数和回归测试。
- `PredictionInteractionContext` 作为 P3 与后续 LDOP 第四阶段可复用的场景证据层，输出每分支 pair/map/corridor cost、`total_energy`、`probability_scale`、轻量修正建议、最近障碍距离、occupied overlap、near obstacle、free corridor 方向/置信度、最近目标距离和最小 TTC。
- 地图查询边界已落地：`PredictionInteractionContext` 只依赖 `PredictionMapQuery`；`Ldop` 在 `processFrame()` 中构造栈上 `UfomapPredictionMapQuery` adapter；静态近障碍使用新增 `UfomapMapper::queryLocalOccupied()`，动态目标风险继续来自 tracker/predictor 分支，不使用 UFOMap dynamic candidate 查询反推未来风险。
- predictor 应用交互结果时只调整 branch probability 和受限均值轨迹；`model_covariance` 仍保持运动模型 rollout 的不确定性语义，已用回归测试逐元素比较协方差。
- RViz 预测 marker 仍只生成 `dynamic_prediction_line`；`dynamic_prediction_samples` 和 `dynamic_prediction_uncertainty` 只在回归测试中作为“不应出现”的命名空间检查。
- 子代理执行记录：Task 1-4 已经过实现、规格审查和质量审查；Task 5 最后一轮质量复审和 Task 6 worker 因子代理 usage limit 阻塞，后续由主线按同一任务边界完成静态审查、实现和验证。
- 沙箱内首次 `catkin run_tests ldop --no-status` 因 ROS/XMLRPC socket 被禁用而卡住，日志反复出现 `XmlRpcClient::doConnect: Could not create socket (Operation not permitted)`；随后已终止卡住进程，并在沙箱外用相同命令完成测试。
- 已验证：`catkin build ldop --no-status` 成功；`catkin run_tests ldop --no-status` 成功；`catkin_test_results /home/st/ldop_ws/build/ldop` 输出 `Summary: 72 tests, 0 errors, 0 failures, 0 skipped`。
- 已验证：`git diff --check` 通过；marker 命名扫描确认未新增 sample/uncertainty marker 或默认诊断发布器。

## 2026-05-20 P3 交互轨迹平滑 rerollout

- 用户指出交互后的预测轨迹仍显折线，希望改交互预测本身，而不是只做 RViz 插值。
- 已按 TDD 新增回归测试 `InteractionCorrectionRerolloutAvoidsLocalPositionKink`：构造局部地图冲突，要求交互修正后的 planner-facing 中心轨迹不能出现 x 方向前进后退折返。
- 红灯验证：旧逐点位置修正实现下该测试失败，出现 `step_x = -0.1`。
- 已将 predictor 应用交互结果的方式改为：保留 `PredictionInteractionContext` 的逐点风险证据，先平滑成位置/减速/方向控制 profile，再在对应 `PredictionMode` 上重新调用 motion model rollout。
- `model_covariance` 仍由重新 rollout 的运动模型传播，不因交互代价被人为缩小；RViz marker 仍只输出真实分支的 `dynamic_prediction_line`。
- 已运行 `catkin build ldop --no-status --make-args tests`，构建测试目标通过。
- 已运行 `/home/st/ldop_ws/devel/lib/ldop/test_dynamic_object_predictor --gtest_filter=DynamicObjectPredictorTest.InteractionCorrectionRerolloutAvoidsLocalPositionKink`，新增聚焦测试通过。
- 已运行 `/home/st/ldop_ws/devel/lib/ldop/test_dynamic_object_predictor`，24 个 predictor 回归测试全部通过。
- 已运行 `catkin build ldop --no-status`，构建通过。
- 已在沙箱外运行 `catkin run_tests ldop --no-status`，结果通过；`catkin_test_results /home/st/ldop_ws/build/ldop` 输出 `Summary: 76 tests, 0 errors, 0 failures, 0 skipped`。
- 已运行 `git diff --check`，结果通过。

## 2026-05-20 P3 交互上下文查询预算优化

- 用户报告现场日志中 `applyInteractionContext=10.937ms`，目标是压到 2ms 内。
- 已用查询计数回归测试复现热点：7 个目标、每目标 3 分支、每分支 10 个点时，旧实现触发 `localOccupied=1760`、`seenFree=1680`、`queryNode=210` 次地图访问。
- 已将 map cost 改为分支级查询：每条分支构造覆盖短时 rollout、目标外接圆和近障碍半径的局部查询球，只调用一次 `queryLocalOccupied()`，再在内存中对各采样点计算 signed distance、occupied overlap 和 near-obstacle 代价。
- 已将 corridor cost 改为分支级方向探针：每条分支使用时间中点作为代表点，只做 8 个 `seenFree()` 探针；多方向同为 free 时用分支运动方向做 tie-break，全方向 free 时不注入任意方向偏置。
- 交互修正仍保持平滑控制 rerollout：map cost 保留逐点计分，但控制 hint 只取该分支最强局部证据，避免分支级 occupied 候选把整条轨迹推成满幅修正。
- 新增 `UsesBranchLevelMapQueryBudget` 回归测试，预算约束为 local occupied 查询不超过分支数、`seenFree()` 不超过分支数乘 8、`queryNode()` 为 0。
- 该实现取代早期设计中的 `LocalOccupiedCacheKey` / `unordered_map` 单帧局部缓存方案：当前瓶颈是查询数量爆炸，直接限制每分支查询上限比依赖缓存命中率更稳定。
- 已运行 `catkin build ldop --no-status --make-args tests`，构建测试目标通过。
- 已运行 `/home/st/ldop_ws/devel/lib/ldop/test_prediction_interaction_context`，15 个交互上下文测试全部通过。
- 已运行 `/home/st/ldop_ws/devel/lib/ldop/test_dynamic_object_predictor`，24 个 predictor 回归测试全部通过。
- 已运行 `catkin build ldop --no-status`，完整构建通过。
- 已运行 `catkin run_tests ldop --no-status`，包级测试通过；`catkin_test_results /home/st/ldop_ws/build/ldop` 输出 `Summary: 78 tests, 0 errors, 0 failures, 0 skipped`。
- 已运行 `git diff --check`，结果通过。

## 2026-05-20 P3 branch risk level 摘要

- 已实现内部 `PredictionBranchRiskLevel::{Low, Medium, High, Critical}`，并加入 `PredictionBranchInteraction::risk_level`。
- 分级规则保留连续证据不变：occupied overlap 为 `Critical`；短 TTC、高 pair/map cost 或高 total energy 为 `High`；near obstacle、轻 pair risk 或 corridor bias 为 `Medium`；其余为 `Low`。
- 该字段不新增 ROS msg，也不替代 `pair_cost`、`map_cost`、`corridor_cost`、`nearest_obstacle_distance`、`minimum_ttc` 等连续指标；后续 LDOP 第四阶段风险地图 builder 可用它做快速分层，再用连续证据计算 planner-facing 风险强度。
- 已补充 `test_prediction_interaction_context.cpp` 对 `Low/High/Critical` 和近障碍非低风险的断言。
- 已运行 `catkin build ldop --no-status`，构建通过。
- 已运行 `catkin run_tests ldop --no-status`，包级测试通过；`catkin_test_results /home/st/ldop_ws/build/ldop` 输出 `Summary: 78 tests, 0 errors, 0 failures, 0 skipped`。

## 2026-05-20 P3 完成状态文档同步

- 已按当前代码事实把轨迹预测内部 P3 标记为完成：目标间/地图交互、分支概率重权重、平滑控制 rerollout、分支级查询预算和内部 `branch_risk_level` 摘要均已落地；`branch_risk_level` 服务后续 LDOP 第四阶段风险地图快速分层，不属于 predictor 内部 P4 闭环反馈。
- 已同步总安排文档、根规划记录、发现记录和 P3 实现计划状态说明。
- 当时后续推进边界收敛为 P4 闭环预测误差统计、NIS 校准和概率反馈；planner-facing 风险地图消费仍属于 LDOP 第四阶段。

## 2026-05-20 P4 predictor 内部闭环反馈实现

- 已完成 P4 在线闭环反馈：`DynamicObjectPredictor` 内部按 `object_id` 缓存上一轮预测快照，用当前 tracker 观测对齐上一轮分支并计算 residual、NIS 和 log-likelihood。
- feedback 只作用于下一帧同语义分支的 mode score bias 和 predictor 未来 `Q_future` 尺度；分支通过 `behavior_type + behavior_value` 近似桶匹配，不依赖 branch index。
- P4 不新增 ROS 预测消息字段或默认诊断 topic，不修改 tracker 当前 `model_state/model_covariance`；诊断通过 `PredictionFeedbackDiagnostics` 和 `verbose` 日志暴露。
- `process_noise_scale = 1.0` 表示基础 predictor Q；闭环可降到固定下界 0.3，也可升到 `prediction_feedback_max_noise_scale`。0.3 不暴露为 YAML 参数，且未恢复 `prediction_feedback_history_size` 或新增 `prediction_feedback_min_noise_scale`。
- P3 interaction reweighting 仍在 P2/P4 之后执行；当前 P4 不把 LDOP 第四阶段 planner-facing 风险地图视为已实现。
- 多 horizon `ADE/FDE/minADE/minFDE/NLL/ECE/Brier/coverage` 等离线 rosbag evaluator 仍是后续扩展。
- 本轮复核已验证通过：`git diff --cached --check`、`catkin build ldop --no-status`、`catkin run_tests ldop --no-status` 和 `catkin_test_results /home/st/ldop_ws/build/ldop`，汇总为 106 tests, 0 errors, 0 failures, 0 skipped。

## 2026-05-21 LDOP 第三阶段轨迹预测完成状态同步

- 用户确认：轨迹预测内部 P4 已实现，因此 LDOP 第三阶段动态目标短时轨迹预测内容视为完成。
- 已将总安排文档、根规划记录和发现记录同步为“第三阶段轨迹预测已完成”；后续离线多 horizon evaluator 与 LDOP 第四阶段 planner-facing 风险地图/动态风险层不再作为第三阶段完成条件。
- 当前第三阶段完成边界：P1 轨迹外推、P2 航向/意图 GMM、P3 目标间/地图交互感知预测、P4 predictor 内部在线闭环反馈均已落地；预测输出仍保持独立 prediction msg 和 `dynamic_prediction_line` RViz marker。

## 2026-05-21 面向阅读者文档状态补充同步

- 用户指出 `docs/note/LDOP_design.md` 和 `README.md` 等入口仍未同步第三阶段完成状态。
- 已补充同步 `README.md`、`docs/note/LDOP_design.md`、`docs/note/2026-05-18_intent_prediction_trajectory_innovation.md`，以及 P2/P3/P4 设计规格顶部状态说明。
- 已把“P4 尚未完成”“第三阶段进行中”等旧口径改为“P1-P4 已完成，LDOP 第三阶段短时轨迹预测已完成”；离线 evaluator 和 planner-facing 风险地图继续作为后续工具或 LDOP 第四阶段工作。
