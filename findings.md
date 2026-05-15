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
- RViz 预测 marker 默认话题名已确定为 `/ldop/dynamic_prediction_markers`；样式为预测线加每个采样点的 1σ 位置协方差椭球，漏检延续目标降低透明度。
- 第四阶段闭环评估所需的真实轨迹缓存窗口和目标匹配策略尚未设计。

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
- `BEHAVIOR_BASELINE` 不应成为后续长期混用的默认分支；后续分支应明确属于纵向、垂向、转向或横向等行为族。
- `behavior_value` 的范围固定为 `[-1.0, 1.0]`。纵向正值表示沿主要运动方向加速倾向，负值表示减速或反向趋势；垂向正负表示上升/下降；转向正负表示左右转向；横向正负表示相对当前航向的侧移方向。
- 第一阶段轨迹外推实现计划文档已经生成，路径为 `docs/superpowers/plans/2026-05-14-dynamic-object-trajectory-prediction-implementation-plan.md`；当前事实不是计划缺失，而是根目录进度记录此前未同步到“实现计划已生成”的状态。

## 沟通约束

- 面向用户交流和后续规格文档中，避免使用依赖过程文档的阶段简称；直接写明当前目标和能力边界，例如“轨迹外推单轨迹预测”“航向/意图多轨迹预测”“目标间和地图交互预测”。
