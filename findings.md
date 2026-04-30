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

## 2026-04-30 新任务事实

- 用户要求继续参考 LDOT，完整实现目标分类和多运动模型 KF/EKF；这已经超出上一轮“第一版默认 CA3D，类别切换暂不启用”的范围。
- 本轮必须先使用 `superpowers:brainstorming` 做设计收敛：设计获批前不改实现代码。
- 本轮继续使用 `planning-with-files-zh`，关键发现写入 `findings.md`，阶段进度写入 `task_plan.md` 和 `progress.md`。
- 仓库规则要求任务过程中不创建中间提交；因此即使设计文档落地，也不在用户明确要求前提交。

## 2026-04-30 待核实内容

- 当前主工作区是否已经包含上一轮 worktree 的 tracker/Kalman 实现，还是仍停留在更早的主线状态。
- 当前 LDOP 的 `DynamicObject` / detection / tracker 内部结构是否已有类别字段、yaw、acceleration、模型类型等扩展点。
- LDOT 当前目标分类入口、类别字段、模型选择和 `switchKalmanModel()` 的真实调用顺序。
- LDOT 的 CV/CA/CTRA 模型状态维度、测量维度、噪声参数、线性 KF 与 EKF 的分工。
- LDOP 本轮是否需要完全复制 LDOT 类别集合，还是保留 LDOP 更小的阶段二输出合同。

## 2026-04-30 已核实现状

- 当前主工作区已经包含 `include/ldop/dynamic_object_tracker.h`、`include/ldop/multi_model_kalman_filter.h`、`include/ldop/motion_model.h` 以及对应 `.cpp`，说明上一轮滤波/跟踪实现已经进入主线。
- 当前 `DynamicObjectTrackerConfig` 仍有单个 `model_type{MotionModelType::CA3D}`；`TrackState` 虽保存 `model_type`，但 tracker 入口尚未按类别选择或切换模型。
- 当前 `DynamicObject.msg` 只有 `id/center/size/velocity`，没有类别、yaw、加速度、模型类型或分类置信度字段。
- 当前 `MultiModelKalmanFilterConfig` 已包含 `ca_human`、`ca_uav`、`cv`、`ctra` 四组参数，并已有 `LinearKalmanFilter` / `ExtendedKalmanFilter` 接口；这说明本轮更像是补齐分类入口、模型选择/切换、参数和消息合同，而不是从零写滤波器。
- 当前 `MotionModelType` 已覆盖 `CA2D/CA3D/CV3D/CTRA`；`createKalmanFilter(bool is_human, bool is_vehicle, bool is_uav, bool is_other, ...)` 已按 LDOT 类别映射到 CA2D、CTRA、CA3D、CV3D。
- 当前 `CAMotionModel`、`CVMotionModel` 是线性模型，`CTRAMotionModel` 覆写 `stateTransition()` 和雅可比，已通过 `ExtendedKalmanFilter` 接入。
- 当前 `config/ldop.yaml` 只有 `tracking_ca_*` 参数，尚未暴露 human/uav/vehicle/other 分组参数，也没有分类参数。
- 当前 clusterer 到 tracker 的输入仍只是 `std::vector<DynamicObjectDetection>`，需要继续核实 `DynamicObjectDetection` 是否有类别字段；从已读接口看，分类目前还没有进入主流程。
- 当前 `DynamicObjectDetection` 只有 `detection_id`、`bbox`、`stamp`；`BoundingBox3D` 虽有 `yaw`，但 detection 和输出消息都没有目标类别。
- 当前 `DynamicObjectTracker::createTrack()` 始终用 `config_.model_type` 创建 `TrackState`，且该配置默认 `CA3D`；`buildOutput()` 只发布 `id/center/size/velocity`。
- 当前 `CMakeLists.txt` 已把 `src/motion_model.cpp` 和 `src/multi_model_kalman_filter.cpp` 编进 `ldop_core`，但没有找到 tracker/Kalman 测试文件；上一轮进度中的测试结果不能直接代表当前主线测试资产仍存在。

## 2026-04-30 LDOT 初步对照

- LDOT 参考路径为 `/home/st/catkin/LDOT/ldot_detector`；源码主要放在 `include/ldot_detector/*.cpp/.h`，配置目录是 `cfg/`。
- LDOT `box3D` 同时保存 bbox、稳定/临时 id、速度、加速度、`is_human/is_che/is_uav/is_else`、动态标志、估计标志、`fix_size` 和 `yaw`。
- LDOT 的运动模型/KF 结构与当前 LDOP 底座基本同源：human -> 2D CA，uav -> 3D CA，vehicle -> CTRA EKF，else -> 3D CV。
- LDOT `KF_Params` 还包含 `prediction_cov_multiplier`；当前 LDOP `MultiModelKalmanFilterConfig` 没有该字段，因为当前阶段还不做未来预测。
- LDOT 跟踪顺序是 `runTracking()` -> `boxAssociation()` -> 对匹配旧轨迹做分类/必要时 `switchKalmanModel()` -> `kalmanFilterAndUpdateHist()` -> `removeDuplicateTracks()`。
- LDOT 新轨迹默认初始化为 `is_else` + 3D CV；达到 `classification_start_frame` 后才对匹配旧轨迹按尺寸/质心规则分类。
- LDOT 分类逻辑使用历史最大尺寸判断 human/vehicle/uav/else，并用 `classification_interval_sec` 控制重分类频率；连续相同明确分类达到 `fix_size_classification_threshold` 后会固定分类和尺寸。
- LDOT `switchKalmanModel()` 在 Kalman update 前执行；它从旧模型预测后的状态抽取位置/速度，再转换到新模型状态维度：human CA2D、vehicle CTRA、uav CA3D、else CV3D。
- LDOT 参数组织采用 `kalman_filter/...` 树状分组，分别给 human CA、uav CA、CV、CTRA 配置噪声与协方差；当前 LDOP 仍只有扁平 CA 参数。
- LDOT 的动态/静态分类文档还包含点云运动投票等动态性判别，但 LDOP 当前阶段的“目标分类”更接近对象类别 human/vehicle/uav/else，而不是重新做动静态点云分类。
- 用户已确认：LDOP 的动静态分类从一开始已经由 UFOMap 分割链路实现，本轮“目标分类”只做对象类别分类和模型切换，不搬 LDOT 的动态/静态投票分类模块。
- 用户已确认：新轨迹先默认 `else + CV3D`；当轨迹命中数达到 `classification_start_frame` 后才执行对象类别分类；如果分类结果变为 `human/vehicle/uav/else`，则在本帧 Kalman update 前切换到 `CA2D/CTRA/CA3D/CV3D`。
- 用户认可对外类别枚举分成 `UNKNOWN=0, HUMAN=1, VEHICLE=2, UAV=3, OTHER=4`；其中 `UNKNOWN` 只表示尚未可靠分类，`OTHER` 表示已经执行分类但不满足前三类规则。
- 设计含义：新轨迹内部可使用 `CV3D` 作为默认滤波模型，但 `DynamicObject.msg.object_class` 在分类前应发布 `UNKNOWN`，不能因为内部模型是 CV3D 就对外发布 `OTHER`。
- 用户已确认：对象类别分类按 LDOT 的核心尺寸/质心规则做 LDOP 适配，使用轨迹历史最大 bbox 尺寸和当前 detection 质心高度；`classification_start_frame` 继续作为新轨迹开始分类前的最小命中数。
- 本轮不引入点云 PCA/std 作为分类输入，也不引入 LDOT 的点云运动投票动态性判别；这保持 tracker 输入仍为 detection/bbox/centroid 级别。
- 用户已确认参数组织：`tracking_*` 生命周期/关联参数和 `classification_*` 对象类别参数保持扁平；多模型 Kalman 噪声和协方差参数按 LDOT 采用 `kalman_filter/ca_model/human/...`、`kalman_filter/ca_model/uav/...`、`kalman_filter/cv_model/...`、`kalman_filter/ctra_model/...` 分组。
- 用户已确认：`DynamicObject.msg` 本轮只增加 `uint8 object_class`；不增加 acceleration、yaw、motion_model 或分类置信度字段。
- 用户追问 LDOT 合并/分离检测机制后重新核对：LDOT 在分类前先比较当前 bbox 尺寸与历史最大尺寸的比例，以及当前点数与上一帧点数的比例；短期突增视为临时合并，短期突减视为临时分离。
- LDOT 对临时合并/分离使用 `largeSizeCounter_` / `smallSizeCounter_` 和 `classification_size_change_confirm_frames` 做确认；临时状态下使用历史最大尺寸替代当前尺寸，避免分类被瞬时粘连/分裂污染。
- 当前 LDOP `Fragment` 已经统计 `point_count`，但 `DynamicObjectDetection` 尚未暴露该字段；如果要贴近 LDOT，可把 `point_count` 加到内部 detection，用于 tracker 的合并/分离保护，不需要把原始点云历史带入 tracker。
- 用户已确认：本轮纳入 LDOT 的临时合并/分离检测机制。实现边界是 tracker 内部的分类尺寸保护和 bbox 尺寸稳定，不做 `removeDuplicateTracks()`，也不改变 clusterer 的连通域/竖向碎片合并流程。

## 2026-04-30 错误与缺口

- 读取 `test/test_multi_model_kalman_filter.cpp` 和 `test/test_dynamic_object_tracker.cpp` 失败：当前主工作区不存在这两个文件。后续设计/实施需要把测试是否恢复或新增列为验证项。
- 第一次搜索 LDOT 时误把配置目录写成 `/home/st/catkin/LDOT/ldot_detector/config`，实际应使用 `cfg/`；后续已按正确目录继续。

## 2026-04-30 设计产出

- 新增设计文档：`docs/superpowers/specs/2026-04-30-dynamic-object-classification-multimodel-design.md`。
- 设计文档采用方案 A：在现有 `DynamicObjectTracker` 上增量补齐对象分类、多模型切换、内部 `point_count` 和临时合并/分离保护，不新建独立 classifier 模块，不照搬 LDOT 大主类。
- 已完成 spec 自检：未发现 `TODO/TBD/待定/占位/FIXME`；`git diff --check -- docs/superpowers/specs/2026-04-30-dynamic-object-classification-multimodel-design.md task_plan.md findings.md progress.md` 通过。
- 按仓库规则没有创建 git 提交。
- 用户指出新增 spec 使用英文，不符合 `AGENTS.md` 的语言约定。已读取 `AGENTS.md`，其中明确要求面向用户的自然语言或可视化内容默认用简体中文，代码标识符、命令、配置键、协议名保留英文。
- 已将 `docs/superpowers/specs/2026-04-30-dynamic-object-classification-multimodel-design.md` 改为中文；保留模型名、参数名、消息字段和命令为英文。
- 用户质疑 LDOT 的 `classification_size_change_confirm_frames`、`fix_size_classification_threshold` 和 `classify_xy_distance_threshold` 的稳定策略是否重复或可优化。重新核对后结论：LDOT 中 `classification_size_change_confirm_frames` 用于确认持续的尺寸/点数变化是否真实，`fix_size_classification_threshold` 用于连续相同明确类别后设置 `fix_size`；但 `fix_size` 同时表示分类锁定和尺寸固定，语义耦合较强，LDOP 设计应考虑拆成更清楚的状态。
- 用户决定删除 `classify_xy_distance_threshold`，后续如果需要平台近场保护再单独加入。
- 设计修正：放弃 `classification_interval_sec` + `classification_stable_confirmations` 的固定间隔/连续次数方案，改为 `classification_scores` 证据积分。可靠匹配帧按当前 bbox 分类结果更新分数，最高分达到 `classification_confirm_score` 且领先第二名至少 `classification_switch_margin` 后才确认或切换类别。
- 短时遮挡处理已收口：coasting 帧冻结类别分数和当前模型，不把缺观测当成反证；遮挡后如果 bbox/点数突降，先按临时分离或疑似遮挡残片冻结类别证据，等观测稳定后再继续积分。

## 2026-04-30 实施结果

- 已按设计实现目标类别输出：`DynamicObject.msg` 增加 `CLASS_UNKNOWN/HUMAN/VEHICLE/UAV/OTHER` 常量和 `object_class` 字段。
- 内部 detection 已增加 `point_count`，clusterer 在 `fragmentToDetection()` 中把 `Fragment::point_count` 传给 tracker。
- 新轨迹现在以 `Unknown + CV3D` 初始化；达到 `classification_start_frame` 后，可靠匹配帧更新 `classification_scores`。
- 分类证据策略已按用户确认的优化方案落地：可靠匹配帧 decay + increment，coasting、临时合并/分离和疑似遮挡残片帧冻结证据；不实现 `classification_interval_sec`、`classification_stable_confirmations` 或 `classify_xy_distance_threshold`。
- 模型切换在 matched-track Kalman update 前执行；`initializeState()` 用于把旧模型的位置/速度迁移到新模型状态，CTRA yaw 优先由速度方向估计，低速时保留旧 yaw 或置 0。
- `config/ldop.yaml` 已加入分类参数和 `kalman_filter/...` 分组；旧 `tracking_ca_*` 参数保留为兼容默认值，不再作为主语义入口。
- 文档已同步：tracker 执行流程、数学逻辑和 LDOP 第二阶段路线图都记录了对象类别、证据积分和模型切换语义。

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
- 已更新：`/ldop/dynamic_track_markers` 固定为 `LINE_STRIP` 历史线、`SPHERE` 头部和 `TEXT_VIEW_FACING` 标签；标签显示轨迹 ID、类别和速度模长，仍不画速度箭头。
- 已收口：`/ldop/dynamic_object_markers` 沿用现有 clusterer 线框样式：`dynamic_object_bbox` + `LINE_LIST` + 橙色 + `DELETEALL`。
- 已产出：实施计划文档已经生成，下一步不再是继续讨论设计，而是决定是否按计划进入实现。
- 已确认：用户已选择 `Subagent-Driven`，并指定项目内 `.worktrees/` 作为隔离工作区位置。
- 当前实现进度：实施计划 5 个阶段已经在 `/home/st/ldop_ws/src/LDOP/.worktrees/feature-dynamic-object-tracking` 落地。`DynamicObject.msg` 已移除预测数组，clusterer 正式输出 detection 与检测框 marker，tracker 输出稳定 `DynamicObjectArray`，`Ldop` 已接入 tracker 和 `/ldop/dynamic_track_markers`。
- 当前验证事实：worktree 临时 CMake 构建已通过 `ldop_node`、`test_dynamic_object_tracker`、`test_multi_model_kalman_filter`、`test_dynamic_object_clusterer`；运行结果分别覆盖 tracker `5/5`、Kalman `8/8`、clusterer `12/12`。由于 worktree 是嵌套在主仓内，常规 `/home/st/ldop_ws` catkin source space 仍指向主工作区，不能直接代表 worktree 构建结果。
- 当前实现限制：第一版仍只支持 `CA3D`；`tracking_ca_initial_position/velocity/acceleration_cov` 和 `tracking_max_pos/vel/acc_cov` 在当前 filter 接口中折算为单一初始协方差/协方差上限，后续若要严格分轴/分状态上限，需要扩展 `MultiModelKalmanFilterConfig`。
- 后续阶段：未来预测消息字段、CV/CTRA、目标类别字段和模型切换策略放到后续阶段；第一版只保留扩展边界。
