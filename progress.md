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
