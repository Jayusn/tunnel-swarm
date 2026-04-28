# 动态对象跟踪设计发现记录

## 已知约束

- 用户要求使用 `planning-with-files-zh` 记录全过程，并使用 `superpowers:brainstorming` 继续明确和优化设计方案。
- `superpowers:brainstorming` 要求先探索项目上下文、澄清需求、提出方案并获得确认；在设计确认前不进入实现代码。
- 仓库约定：`Ldop` 负责 ROS I/O 与线程调度；`UfomapMapper` 负责 UFOMap 插入、`seenFree` 判别、当前帧动静态点云分割、累计静态地图云输出及基础查询接口。
- 仓库约定：`UfomapMapper` 已移除 clustering/BFS 聚类职责；聚类、检测、跟踪和预测不应塞回 mapper。
- 当前 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud` 语义均为当前帧输出，累计地图由 `/ldop/ufomap` 表达。
- 当前阶段优先级仍是“动静态分割 + 静态地图闭环可视化”；复杂跟踪/预测/规划融合不应抢占该优先级。

## 待核实内容

- 现有设计文档对动态对象跟踪的范围、接口、数据流、状态语义是否已经和当前代码一致。
- `DynamicObjectClusterer`、`DynamicObjectTracker`、`Ldop` 当前代码实际承担的职责边界。
- 测试中已经锁定的行为和仍未覆盖的设计风险。

## 当前事实

- `docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md` 已存在并处于 git 新增状态。
- 当前工作区还存在 `launch/rviz/ldop_rviz.rviz` 修改，暂不触碰。
- 最近提交显示第二阶段已有“聚类及可视化”等工作，设计文档应建立在当前聚类输出之上。
- `include/ldop/dynamic_object_clusterer.h` 当前的 `DynamicObjectClustererFrameResult` 只包含 `dynamic_objects_msg`、`dynamic_object_markers_msg` 和 timing；还没有单独的 `DynamicObjectDetection` 输出。
- `include/ldop/dynamic_object_tracker.h` 当前不存在，说明 tracker 仍是设计/待实现内容。
- `include/ldop/utils.h` 当前只包含 timing helper，没有 spec 中提到的 bbox/detection 共享结构。
- 现有 spec 推荐独立 `DynamicObjectTracker`，并明确不把跟踪职责塞回 `UfomapMapper` 或 `DynamicObjectClusterer`。
- 原始 spec 第一版跟踪方案曾写为单一 3D CA Kalman、3D 马氏距离关联、贪婪分配、coasting 保留、稳定 `track_id`、预测字段保持空；当前已按用户确认改为 Hungarian 分配，且 `DynamicObject.msg` 后续实现应移除预测字段。
- `src/ldop.cpp` 当前在 `processFrame()` 中直接发布 clusterer 的当前帧 `DynamicObjectArray`；tracker 接入点应位于 clusterer 后、publish 前。
- `DynamicObjectClusterer::buildDynamicObjectArray()` 当前使用连通域顺序生成帧内 `id`，并将 `velocity` 固定为零；这符合现阶段聚类输出，但不是稳定跟踪输出。
- `msg/DynamicObject.msg` 当前代码仍有 `id`、`center`、`size`、`velocity`、`prediction_time_offsets`、`predicted_positions`；最新设计要求后续实现移除两个预测数组，`DynamicObject` 只承载当前跟踪状态，预测改走独立消息和独立话题。
- `config/ldop.yaml` 当前只有聚类参数，没有 tracking 参数；spec 的参数列表若确认保留，后续实现需要新增配置。
- `docs/note/LDOP_design.md` 把第二阶段定义为“动态点云聚类、检测与跟踪”，并把第三阶段预测单独列出；因此 tracking spec 不应把预测实现提前塞入第二阶段。
- 实施计划文档已新增为 `docs/superpowers/plans/2026-04-25-dynamic-object-tracking-implementation-plan.md`，按“接口与消息契约 -> Kalman 组件 -> Tracker 主流程 -> Ldop 集成与 marker -> 参数与测试收尾”拆分实现顺序。
- `Superpowers` 的 `writing-plans` skill 明确要求计划生成后提供两个执行选项：`Subagent-Driven` 和 `Inline Execution`；不能直接默认进入某一种执行方式。
- `test/test_dynamic_object_clusterer.cpp` 中仍出现 `buildDynamicObjectArray(..., &elapsed_ms)` 和 `buildObjectMarkers(..., &elapsed_ms)` 调用，但当前头文件没有这些重载；这可能是测试滞后于 timing 重构，需要在后续实现计划里单独处理。
- `CMakeLists.txt` 当前只编译 `src/dynamic_object_clusterer.cpp` 和 `src/ufomap_mapper.cpp`，没有 tracker 源文件或 tracker 测试目标。
- LDOT 参考文件 `multiModelKalmanFilter.h` 把滤波器分为 `KalmanFilterBase`、`LinearKalmanFilter`、`ExtendedKalmanFilter`，并把 adaptive Q/R、innovation buffer 和 cov limit 放在滤波器层。
- LDOT 参考文件 `motionModel.h/.cpp` 把运动模型抽象成 `MotionModel`，由具体模型提供初始状态、初始协方差、过程噪声、测量噪声、状态转移和测量映射。
- LDOT 模型类别映射大致为：human -> CA 2D，uav -> CA 3D，vehicle -> CTRA EKF，else -> CV 3D。
- LDOT 的分类流程和旧 `box3D` 携带了大量类别标志、尺寸规则和历史逻辑，不适合直接塞进 LDOP 第一版；LDOP 应借鉴“运动模型/滤波器独立于 tracker”的模块边界。
- 用户明确要求：即便当前只使用单一 CA Kalman，完整实现最终要有多运动模型和目标类别内容，因此第一版设计应提前把运动模型卡尔曼滤波器模块独立出来，避免 `dynamic_object_tracker` 臃肿。
- 进一步查看 LDOT 后发现：`ldot_detector` 是一个库目标，核心实现文件放在 `include/ldot_detector/*.cpp`；`motionModel.cpp` 单独编译进库，`multiModelKalmanFilter.h` 是头文件实现的滤波器工具。
- LDOT 的主类 `dynamicDetector` 非常大，集中持有 ROS I/O、预处理、聚类、跟踪、分类、预测、服务、双缓冲和可视化；LDOP 不应复制这个主类形态。
- LDOT 的 `dynamicDetector` 持有 `std::vector<std::shared_ptr<KalmanFilterBase>> filters_`，说明滤波器是每条 track 的内部状态，而不是与 tracker 平级的主流程模块。
- LDOT 的 `box3D` 同时承载 bbox、id、速度、加速度、类别标志、动态标志、估计标志和 yaw；LDOP 可借鉴字段，但不应把所有语义塞进一个共享 bbox。
- LDOT 的跟踪调用链是 `runTracking()` -> `boxAssociation()` -> `kalmanFilterAndUpdateHist()` -> `removeDuplicateTracks()`；分类和 `switchKalmanModel()` 插在匹配成功轨迹的 Kalman update 之前。
- LDOT 的 `boxAssociation()` 会先对所有历史轨迹调用 filter `predict()`，用 filter 的预测状态和协方差计算关联代价；`kalmanFilterAndUpdateHist()` 对匹配轨迹调用 `update()`、对新目标创建 filter、对未匹配目标做 coasting。
- LDOT 的 `boxAssociation()` 构建当前检测到历史轨迹的 `costMatrix`，gate 外候选填入大代价，然后调用 `hungarianAlgorithm(costMatrix, bestMatch)`；`bestMatch[i] = j` 表示第 `i` 个当前检测匹配到第 `j` 个历史轨迹。
- LDOT 的新目标当前强制以 `is_else -> 3D CV` 初始化，后续分类后通过 `switchKalmanModel()` 切到 Human CA、Vehicle CTRA、UAV CA 或 Other CV。
- LDOT 参数组织把 tracking 参数、classification 参数、`kalman_filter` 参数分开；`kalman_filter` 内部再按 `ca_model/human`、`ca_model/uav`、`cv_model`、`ctra_model` 分组。
- 用户确认第一版默认模型策略：所有 track 默认使用 3D CA，符合当前 spec“单一 CA”；先不启用类别切换，但代码结构保留 `MotionModelType` 和 factory。
- 已将确认后的 LDOT 风格模块边界和默认 3D CA 策略写入 `docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md`。
- 用户提出减少文件数量；设计已调整为第一版不单独创建 `motion_model.{h,cpp}`，而是在 `multi_model_kalman_filter.{h,cpp}` 内包含 `MotionModelType`、`MotionModel` 接口、CA3D 模型实现和 filter factory。
- 用户要求明确 `ldot_detector` 的具体引用路径，因为本地系统中同名目录不止一个；spec 已注明 `ldot_detector` 指 `/home/st/catkin/LDOT/ldot_detector`。
- 用户确认当前推荐策略：`DynamicObjectArray` 的正式输出移到 tracker 阶段，clusterer 输出内部 detection 并保留 `/ldop/dynamic_object_markers` 检测框可视化；不采用把跟踪塞进 `DynamicObjectClusterer`，也不新增 `TrackedObjectArray`。
- 用户要求 `DynamicObject.msg` 去掉预测内容；未来预测另建独立预测消息/话题，避免当前跟踪状态和未来预测轨迹混在同一消息。
- 跟踪阶段需要新增轨迹可视化，与 `/ldop/dynamic_object_markers` 共同形成检测跟踪可视化闭环。
- 已对 spec 做轻量自检：旧的“预测数组保持空”表述已经改为“`DynamicObject.msg` 不包含预测数组”；`/ldop/dynamic_object_markers` 明确归 clusterer 检测框，`/ldop/dynamic_track_markers` 明确归 tracker 轨迹。
- 用户要求第一版关联求解器直接使用 Hungarian，以保持与 LDOT 一致；spec 已从贪婪最小代价分配改为 Hungarian 分配。

## 已收口结论与后续阶段项

- 已确认：动态对象跟踪第一版按当前 spec 完整版推进，包含 adaptive Q/R、cov limit、history、coasting gate 放松等参数。
- 已确认：clusterer 到 tracker 的内部接口采用 `DynamicObjectDetection[]`；`DynamicObjectArray` 只作为 tracker 对外输出。
- 已确认：`DynamicObject.msg` 后续实现移除预测数组；预测未来走独立消息和独立话题。
- 已确认：数据关联求解器第一版使用 Hungarian；代价仍只使用 3D 马氏距离，gate 外候选用无效大代价处理。
- 已确认：独立运动模型/滤波器组件第一版默认使用 3D CA；CV/CTRA 类在 spec 中作为后续扩展，不进入第一版主流程。
- 已确认：`DynamicObjectTracker` 是第二阶段 tracking 主模块；`MultiModelKalmanFilter` 是 track 内部算法组件；`MotionModel` 是 filter 内部依赖，不是三个平级 ROS 模块。
- 已收口：第一版不实现主动重复轨迹清理，不引入 `removeDuplicateTracks()` 对应的 IoU/速度方向去重启发式。
- 已收口：Hungarian 实现细节直接对齐 LDOT 当前简化语义，即行/列归约后从零候选中按原始代价选择不冲突匹配。
- 已收口：参数组织继续使用扁平 `tracking_*`，不在第一版引入 `tracking/...`、`kalman_filter/...` 树状命名。
- 已收口：adaptive Q/R 公式语义直接对齐 LDOT 当前 `innovation_buffer + C_gamma + EMA + min_noise_ratio` 实现。
- 已收口：track history 固定为 `TrackHistorySample(stamp, position, velocity, matched)`，不保存协方差历史。
- 已收口：`/ldop/dynamic_track_markers` 固定为 `LINE_STRIP` 历史线和 `SPHERE` 头部，coasting 轨迹降低 alpha，不显示文本 id，不画速度箭头。
- 已收口：`/ldop/dynamic_object_markers` 沿用现有 clusterer 线框样式：`dynamic_object_bbox` + `LINE_LIST` + 橙色 + `DELETEALL`。
- 已产出：实施计划文档已经生成，下一步不再是继续讨论设计，而是决定是否按计划进入实现。
- 已确认：用户已选择 `Subagent-Driven`，并指定项目内 `.worktrees/` 作为隔离工作区位置。
- 当前实现进度：实施计划 5 个阶段已经在 `/home/st/ldop_ws/src/LDOP/.worktrees/feature-dynamic-object-tracking` 落地。`DynamicObject.msg` 已移除预测数组，clusterer 正式输出 detection 与检测框 marker，tracker 输出稳定 `DynamicObjectArray`，`Ldop` 已接入 tracker 和 `/ldop/dynamic_track_markers`。
- 当前验证事实：worktree 临时 CMake 构建已通过 `ldop_node`、`test_dynamic_object_tracker`、`test_multi_model_kalman_filter`、`test_dynamic_object_clusterer`；运行结果分别覆盖 tracker `5/5`、Kalman `8/8`、clusterer `12/12`。由于 worktree 是嵌套在主仓内，常规 `/home/st/ldop_ws` catkin source space 仍指向主工作区，不能直接代表 worktree 构建结果。
- 当前实现限制：第一版仍只支持 `CA3D`；`tracking_ca_initial_position/velocity/acceleration_cov` 和 `tracking_max_pos/vel/acc_cov` 在当前 filter 接口中折算为单一初始协方差/协方差上限，后续若要严格分轴/分状态上限，需要扩展 `MultiModelKalmanFilterConfig`。
- 后续阶段：未来预测消息字段、CV/CTRA、目标类别字段和模型切换策略放到后续阶段；第一版只保留扩展边界。
