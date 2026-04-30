# LDOP Dynamic Object Tracking Design

## 背景

`LDOP` 第一阶段已经完成当前帧动静态点云分割与静态地图可视化。第二阶段的当前实现已经把动态点云聚类为当前帧动态目标集合：`DynamicObjectClusterer` 消费 `UfomapMapper` 输出的动态点和 UFOMap voxel key，在当前帧内生成 `ldop/DynamicObjectArray` 与 `/ldop/dynamic_object_markers`。

当前仍缺少第二阶段后半段能力：跨帧数据关联、稳定目标 ID、速度估计和目标状态维护。本设计只覆盖第二阶段的跟踪部分，不进入第三阶段短时轨迹预测。

旧 `ldot_detector` 指 `/home/st/catkin/LDOT/ldot_detector`，可作为算法参考，但本次不是照搬。`LDOP` 已经有动静态判别和动态聚类，因此跟踪层只需要消费当前帧 cluster 级目标，不再引入旧流程里的动态候选判别、预测服务或双缓冲可视化框架。

LDOT 中值得保留的结构是：跟踪主流程维护轨迹列表，每条轨迹内部持有一个 Kalman filter；filter 内部再组合 motion model。`LDOP` 也采用这个边界，避免把 Kalman 公式、adaptive Q/R、协方差限制和未来多模型扩展全部塞进 `DynamicObjectTracker`。

## 目标

- 将当前帧 cluster 输出升级为跨帧稳定动态目标输出。
- 对同一动态目标尽量保持稳定 `id`。
- 第一版所有目标默认使用 3D constant-acceleration Kalman filter 估计目标中心、速度和加速度。
- 将运动模型和 Kalman filter 作为 `DynamicObjectTracker` 的内部算法组件独立出来，预留 `MotionModelType` 和 factory，后续可接入 CV/CTRA 与目标类别切换。
- 数据关联第一版只使用 3D 马氏距离，不使用 IoU、类别或外观特征。
- 包围盒尺寸只作为目标外观状态维护和输出，不参与关联代价。
- 继续复用 `ldop/DynamicObjectArray` 作为 `/ldop/dynamic_objects` 输出消息。
- `ldop/DynamicObjectArray` 的正式语义从“当前帧聚类结果”移到“tracker 输出的稳定动态目标集合”。
- 从 `ldop/DynamicObject.msg` 中移除预测数组；未来预测能力使用独立预测消息和独立预测话题输出。
- 保留聚类阶段的 `/ldop/dynamic_object_markers` 检测框可视化，并新增跟踪阶段轨迹可视化话题，形成“检测框 + 跟踪轨迹”的可视化闭环。
- 保持 `/ldop/static_cloud`、`/ldop/dynamic_cloud` 和静态地图输出语义不变。

## 非目标

- 不做短时轨迹预测。
- 不在 `/ldop/dynamic_objects` 中承载预测内容。
- 不在第一版实现预测消息或预测话题；本设计只固定预测输出应与 tracking 输出解耦。
- 不新增目标类别字段，不做人、车、无人机等分类。
- 不在第一版启用类别驱动的多模型切换；CV/CTRA 与类别映射只作为内部结构预留，不参与主流程。
- 不把跟踪职责塞回 `UfomapMapper`。
- 不把跨帧状态放进 `DynamicObjectClusterer`。
- 不使用 IoU 进行数据关联。
- 不新增独立 `TrackedObjectArray` ROS 消息。
- 不实现规划风险地图、动态障碍物膨胀或 ESDF 融合。

## 方案选择

推荐方案：新增独立 `DynamicObjectTracker`，位于 `DynamicObjectClusterer` 后面。

```text
UfomapMapper
-> DynamicObjectClusterer
-> DynamicObjectTracker
-> /ldop/dynamic_objects
```

`DynamicObjectClusterer` 继续只负责当前帧动态点连通、过滤和 AABB 估计。`DynamicObjectTracker` 负责跨帧状态：数据关联、驱动轨迹内部 filter 执行 predict/update、track id 分配、漏检保留和超时删除。这样聚类和跟踪的职责边界清楚，后续第三阶段预测也可以只消费 tracker 的稳定状态。

备选方案一：把跟踪直接加进 `DynamicObjectClusterer`。这会少一个类，但 clusterer 会同时承担当前帧几何检测和跨帧状态管理，后续测试、参数和生命周期都会混在一起。

备选方案二：新增 `TrackedObjectArray` 消息。接口更严格，但现在 `/ldop/dynamic_objects` 在 `LDOP_design.md` 中已经是第二阶段必须输出，当前消息也已经包含 `id`、`center`、`size` 和 `velocity`，第一版没有必要扩大 ROS 接口变更。

## 架构

新增和修改模块：

- `include/ldop/utils.h`
- `include/ldop/dynamic_object_tracker.h`
- `src/dynamic_object_tracker.cpp`
- `include/ldop/multi_model_kalman_filter.h`
- `src/multi_model_kalman_filter.cpp`
- `msg/DynamicObject.msg`
- `test/test_dynamic_object_tracker.cpp`
- `test/test_multi_model_kalman_filter.cpp`

`utils.h` 放置多个模块可能复用的小工具、接口函数和轻量结构体。第一版在这里增加共享 bbox 与当前帧检测结构；Kalman 状态、分类标志和预测字段不放进 `utils.h`。

`DynamicObjectTracker` 是第二阶段 tracking 的唯一主模块。它维护轨迹列表、数据关联、生命周期和输出组装。每个 `TrackState` 内部持有一个 `MultiModelKalmanFilter`；tracker 只调用 filter 的 `setDt()`、`predict()`、`update()` 和状态读取接口，不直接维护 Kalman 矩阵。

`MultiModelKalmanFilter` 是 tracker 内部算法组件，不持有 track 列表，不接触 ROS I/O。为避免第一版文件过多，`MotionModelType`、`MotionModel` 接口、`CA3D` 模型实现和 filter factory 都放在这一组文件中；后续 CV/CTRA 等模型实现膨胀后，再把 motion model 拆成单独文件。

`MultiModelKalmanFilter` 负责：

- 管理当前 filter 的 `MotionModelType`，第一版默认为 `CA3D`。
- 执行 Linear Kalman filter 的 predict/update。
- 维护 innovation buffer。
- 执行 adaptive Q/R 更新。
- 执行协方差上限钳制。
- 暴露预测/更新后的状态、协方差、位置和速度读取接口。

`MotionModel` 是 `MultiModelKalmanFilter` 文件组内的内部依赖。它负责状态维度、初始状态、初始协方差、过程噪声、测量噪声、状态转移和状态到观测的映射。第一版主流程只创建 `CA3D`，但保留 `MotionModelType` 和 factory 边界，后续可以接入 `CV3D`、`CTRA`、`HumanCA2D`、`UavCA3D` 等模型，而不重拆 tracker。

`DynamicObjectTracker` 负责：

- 读取和合法化跟踪参数。
- 消费 `DynamicObjectClusterer` 输出的当前帧 `DynamicObjectDetection` 列表。
- 基于上一帧轨迹和当前帧时间间隔驱动每条轨迹的 filter 执行预测。
- 用 3D 马氏距离构建关联代价，并根据门限接受或拒绝匹配。
- 对匹配轨迹调用 filter update。
- 对未匹配检测新建轨迹。
- 对未匹配历史轨迹执行短暂 coasting，超过阈值后删除。
- 输出跟踪后的 `ldop/DynamicObjectArray`。
- 生成跟踪轨迹 marker，用于显示稳定 track 的历史轨迹。

`Ldop` 持有 `std::unique_ptr<DynamicObjectTracker>`。`processFrame()` 先调用 `DynamicObjectClusterer::processDynamicObjects()` 获得当前帧 detection 和当前帧检测框 marker，再调用 tracker 获得稳定目标集合和轨迹 marker。发布仍通过 `publishFrame()` 完成。

`UfomapMapper` 和 `DynamicObjectClusterer` 不维护 track id，也不读取 tracker 的状态。

## 内部数据结构

第一版建议新增一个聚类和跟踪复用的轻量 bbox 结构，但不把旧 `ldot_detector::box3D` 原样搬入 LDOP。bbox 只表达目标几何框；`track_id`、速度、加速度、Kalman 协方差和漏检计数属于跟踪状态，不塞进 bbox 本身。

```text
BoundingBox3D
- center: Eigen::Vector3d
- size: Eigen::Vector3d
- yaw: double，第一版固定为 0.0，仅保留后续 OBB 扩展位置

DynamicObjectDetection
- detection_id: uint32，当前帧临时编号
- bbox: BoundingBox3D
- stamp: ros::Time

TrackHistorySample
- stamp: ros::Time
- position: Eigen::Vector3d，使用滤波后的轨迹中心
- velocity: Eigen::Vector3d，使用当前滤波状态速度
- matched: bool，本帧是否由真实 detection update 而来；`false` 表示 coasting 样本

TrackState
- id: uint32，跨帧稳定轨迹 ID
- model_type: MotionModelType，第一版固定为 CA3D
- filter: MultiModelKalmanFilter，轨迹内部滤波器状态
- bbox: BoundingBox3D，中心和尺寸由跟踪状态维护
- history: std::deque<TrackHistorySample>，容量受 `tracking_history_size` 限制
- age: int，轨迹存在帧数
- hits: int，成功匹配次数
- missed_frames: int，连续漏检帧数
- last_stamp: ros::Time
```

`TrackHistorySample` 只保留轨迹可视化和调试确实需要的字段，不回溯保存完整协方差、bbox 尺寸历史或完整 measurement。第一版 marker 和调试主要依赖 `stamp/position/velocity/matched`；若后续分类或预测确实需要更多历史，再单独扩展。

`DynamicObjectClusterer` 输出当前帧 `DynamicObjectDetection` 列表，并由同一组 detection 构建 `/ldop/dynamic_object_markers`。`DynamicObjectTracker` 消费 detection 列表，输出跟踪后的 `DynamicObjectArray`。`BoundingBox3D` 可以被两个模块复用，但仍不直接暴露为 ROS 消息；`DynamicObject.msg` 是 tracking 对外接口，不再作为 clusterer 的帧内对象输出。

## 多模型 Kalman 组件

第一版主流程只使用 `MotionModelType::CA3D`。这保持当前 spec 的“单一 CA”语义，同时把 Kalman 细节从 `DynamicObjectTracker` 中拆出去。

```text
DynamicObjectTracker
-> TrackState
   -> MultiModelKalmanFilter
      -> MotionModel(CA3D，定义在同一文件组内)
```

后续目标类别接入时，只需要在 track 分类稳定后替换或重建 `MultiModelKalmanFilter` 内部的 motion model；tracker 的关联、生命周期和输出接口不需要重写。第一版不执行这个切换。

## 默认 CA3D 模型

统一使用 3D constant-acceleration 状态：

```text
x = [px, py, pz, vx, vy, vz, ax, ay, az]^T
z = [px, py, pz]^T
```

预测模型按真实时间间隔 `dt` 更新：

```text
p' = p + v * dt + 0.5 * a * dt^2
v' = v + a * dt
a' = a
```

观测只测量目标中心位置。速度和加速度由滤波器状态估计，不直接从 cluster 输入读取。

`dt` 优先使用当前 `DynamicObjectArray.header.stamp` 与轨迹 `last_stamp` 的差值。`tracking_default_dt` 是点云周期的手动兜底值，例如 10 Hz 点云可设为 `0.1` 秒；正常时间戳递增时不使用它。只有 `header.stamp` 为零、非递增、缺失测试时间戳，或其他时间戳异常场景，才回退到该值，并输出 throttle warning。若真实帧间隔过大，则用 `tracking_max_dt` 钳制，也必须输出 throttle warning，方便定位时间戳、bag 播放、仿真暂停或处理积压问题。

保留 LDOT 中 adaptive Q/R 和 cov limit 的语义，但第一版只作用于默认 3D CA 模型：

- adaptive Q：根据最近 innovation 统计调整过程噪声，降低固定 Q 对不同运动强度的敏感性。
- adaptive R：根据最近 innovation 统计调整测量噪声，缓解 cluster 中心抖动或测量质量变化。
- cov limit：固定开启，限制位置、速度和加速度协方差上限，避免长时间 coasting 或异常 `dt` 后不确定性膨胀到不可用。

第一版在实现语义上直接沿用 LDOT 当前 `multiModelKalmanFilter.h` 的思路：

- 每次 `update()` 时先把 innovation 推入固定窗口 `innovation_buffer`。
- 当窗口样本数达到 `tracking_adaptive_window_size` 后，先按 `C_gamma - H * P_pred * H^T` 估计 `R`。
- 再按 `K * (C_gamma - H * (P_pred - Q_old) * H^T - R) * K^T` 估计 `Q`。
- `Q` 和 `R` 都只更新对角线，并使用 `tracking_adaptive_alpha` / `tracking_adaptive_r_alpha` 做 EMA 平滑。
- `tracking_adaptive_min_noise_ratio` 用于给 `Q/R` 施加相对初始噪声的下限，防止估计值塌缩到过小。

这些机制不启用目标类别，也不在第一版切换 motion model；核心目标是保持参数语义与 LDOT 接近，而不是重新发明一套新公式。

## 数据关联

第一版只使用 3D 马氏距离。每条轨迹先 predict，再取预测位置和预测位置协方差：

```text
innovation = detection.center - predicted.position
cost = innovation^T * P_pos^-1 * innovation
```

`P_pos` 是预测协方差左上角 `3x3` 位置块。若 `P_pos` 奇异或数值异常，先加小正则项；仍异常时拒绝该候选匹配。

关联门限采用 3D 卡方门限：

```text
gate = chi_square_quantile(association_gate_confidence, dof = 3)
```

默认 `association_gate_confidence = 0.99`，对应门限约 `11.34`。coasting 轨迹可用 `tracking_coasting_gate_relax_factor` 放宽门限，默认 `1.0` 表示不放松，避免第一版过度吸附错误目标。

匹配求解第一版直接与 `/home/st/catkin/LDOT/ldot_detector` 对齐，使用 LDOT 当前语义下的简化 Hungarian 分配，而不是自行引入另一套完整实现：

1. 构建 `num_detections x num_tracks` 代价矩阵，行表示当前帧 detection，列表示历史 track。
2. gate 内候选填入 3D 马氏距离代价；gate 外候选填入无效大代价，例如 `1e9`。
3. 对代价矩阵执行行归约和列归约。
4. 收集归约后接近零且原始代价有效的候选项。
5. 按原始代价从小到大选择不冲突匹配，得到 `assignment[detection_index] = track_index`。
6. 对 solver 返回结果再次检查原始代价是否仍为有效代价；无效结果视为未匹配。

这样做的目的不是追求教科书版 Hungarian 的形式完整，而是保证第一版实现与 LDOT 现有行为保持一致，减少后续跨仓库比对和调参歧义。第一版先只使用 3D 马氏距离作为代价，不引入 IoU、类别或外观项；Hungarian 只负责在已有代价矩阵上求全局匹配。

## 生命周期

轨迹状态按帧维护：

- 匹配成功：执行 Kalman update，`missed_frames = 0`，`hits += 1`，`age += 1`。
- 未匹配检测：新建轨迹，分配新的单调递增 `track_id`。
- 未匹配轨迹：只执行 predict，`missed_frames += 1`，在删除前继续输出。
- 超过 `tracking_max_missed_frames`：删除轨迹。

第一版始终输出 coasting 轨迹。漏检短时间内 `/ldop/dynamic_objects` 仍保留该目标，速度来自预测状态；这有助于保持 ID 连续，也便于观察跟踪器是否正常工作。

第一版不实现 LDOT 风格的 `removeDuplicateTracks()`。主动重复轨迹清理通常要额外引入 IoU、中心距离、速度方向等启发式，还会改变 ID 生命周期；这已经超出第一版“3D 马氏距离关联 + 稳定 ID 输出”的最小闭环范围。若后续实测表明重复轨迹常见，再在独立设计里引入去重流程和对应参数。

## 尺寸维护

尺寸不参与数据关联。匹配成功时，轨迹尺寸采用指数平滑：

```text
size = alpha * detection.size + (1 - alpha) * previous_size
```

`tracking_box_size_smoothing_alpha` 默认 `0.3`。新轨迹直接使用首帧 detection 尺寸。coasting 时尺寸保持上一状态。

为了避免空尺寸或极小尺寸导致 marker 退化，输出前可对尺寸做最小下限钳制；这个下限只影响输出和可视化，不回写到关联代价。

## 参数

第一版参数组织保持 `LDOP` 当前 `ldop.yaml` 的扁平风格，不照搬 `LDOT` 的 `tracking/...`、`kalman_filter/...` 树形层级。当前仓库的 topic、UFOMap 和 cluster 参数都使用顶层平铺键，tracking 第一版继续使用 `tracking_*` 更符合现有读参方式，也能减少 `Ldop`、tracker 和文档同时改参数层级的噪声。等未来类别和多模型真正进入主流程，再考虑统一重构为嵌套层级。

除数值参数外，第一版还新增一个话题配置键：

```yaml
dynamic_track_markers_topic: /ldop/dynamic_track_markers
```

在 `config/ldop.yaml` 增加：

```yaml
tracking_history_size: 100
tracking_max_missed_frames: 5
tracking_association_gate_confidence: 0.99
tracking_coasting_gate_relax_factor: 1.0
tracking_box_size_smoothing_alpha: 0.3
tracking_default_dt: 0.1
tracking_max_dt: 0.5

tracking_adaptive_window_size: 10
tracking_adaptive_alpha: 0.7
tracking_adaptive_r_alpha: 0.7
tracking_adaptive_min_noise_ratio: 0.2

tracking_max_pos_cov: 1.5
tracking_max_vel_cov: 9.0
tracking_max_acc_cov: 25.0

tracking_ca_jerk_sigma: 3.0
tracking_ca_position_noise: 0.1
tracking_ca_initial_position_cov: 0.1
tracking_ca_initial_velocity_cov: 1.0
tracking_ca_initial_acceleration_cov: 10.0
```

参数语义：

- `tracking_history_size`：每条轨迹保留的 `TrackHistorySample` 容量，默认 100 帧；每个样本只存 `stamp/position/velocity/matched`，主要用于轨迹 marker 和调试，不影响 Kalman 状态维度，也不决定轨迹删除时机。
- `tracking_max_missed_frames`：连续漏检多少帧后删除轨迹。
- `tracking_association_gate_confidence`：3D 卡方门限置信度，合法范围建议限制在 `(0.0, 1.0)`。
- `tracking_coasting_gate_relax_factor`：漏检轨迹重新参与关联时的门限放松倍数，最小为 `1.0`；`1.0` 表示不放松，值越大越容易把漏检轨迹重新匹配到当前检测，但也更容易误关联。
- `tracking_box_size_smoothing_alpha`：bbox 尺寸指数平滑系数，合法范围 `[0.0, 1.0]`；`1.0` 表示完全使用当前帧 cluster 尺寸，`0.0` 表示保持上一帧尺寸，默认 `0.3` 表示尺寸变化偏平滑。
- `tracking_default_dt`：时间戳不可用、为零或非递增时的回退时间步长，单位为秒；手动设置为点云周期，只影响 Kalman predict 的时间间隔；触发时必须输出 throttle warning。
- `tracking_max_dt`：允许用于 predict 的最大时间间隔，单位为秒；真实帧间隔超过该值时进行钳制，避免长时间卡顿后一次预测把状态和协方差放大过头；触发时必须输出 throttle warning。
- `tracking_adaptive_window_size`：adaptive Q/R 使用的 innovation 历史窗口大小；`<=1` 可关闭自适应更新。
- `tracking_adaptive_alpha`：adaptive Q 平滑系数，值越大越快响应新 innovation。
- `tracking_adaptive_r_alpha`：adaptive R 平滑系数，值越大越快响应测量噪声变化。
- `tracking_adaptive_min_noise_ratio`：adaptive Q/R 相对初始噪声的最小比例，防止噪声被压得过小。
- `tracking_max_pos_cov`：位置协方差上限，单位 `m^2`。
- `tracking_max_vel_cov`：速度协方差上限，单位 `(m/s)^2`。
- `tracking_max_acc_cov`：加速度协方差上限，单位 `(m/s^2)^2`。
- `tracking_ca_jerk_sigma`：CA 模型 jerk 噪声标准差，等价于 LDOT 3D CA 的 `jerk_sigma`。
- `tracking_ca_position_noise`：中心观测噪声方差。
- `tracking_ca_initial_*_cov`：新轨迹初始化协方差。

第一版不增加 IoU 权重、类别权重、模型切换阈值、预测时域或预测步长参数。默认 motion model 固定为 `CA3D`，不额外暴露 `tracking_default_motion_model` 参数；`MotionModelType` 和 factory 是内部扩展点。

## 输出语义

`/ldop/dynamic_objects` 表示已跟踪动态目标集合。

每个 `DynamicObject`：

- `id`：稳定轨迹 ID，单调递增，不再是帧内 cluster id。
- `center`：Kalman update 或 predict 后的轨迹中心。
- `size`：轨迹维护的平滑 AABB 尺寸。
- `velocity`：CA 状态中的 `vx/vy/vz`。

`DynamicObject.msg` 不再包含 `prediction_time_offsets` 或 `predicted_positions`。第三阶段预测能力另建预测消息和预测话题，例如 `/ldop/dynamic_object_predictions` 与 `/ldop/dynamic_prediction_markers`，避免把当前跟踪状态和未来预测轨迹混在同一消息里。

`/ldop/dynamic_object_markers` 保留为聚类阶段检测框可视化。它展示当前帧 `DynamicObjectDetection` 的 AABB，marker id 可以是帧内 detection id，每帧先 `DELETEALL` 清理旧检测框。

为了避免 clusterer 当前实现与 spec 分叉，第一版明确沿用现有检测框 marker 约定：

- topic：`/ldop/dynamic_object_markers`
- `MarkerArray` 首元素固定为 `DELETEALL`
- namespace：`dynamic_object_bbox`
- type：`LINE_LIST`
- `scale.x = 0.05`
- color：`rgba = (0.95, 0.35, 0.10, 1.0)`
- 不显示文字标签，不单独显示 detection id

跟踪阶段新增 `/ldop/dynamic_track_markers`。当前固定输出三类 marker，不再保留“线/点/箭头三选一”的模糊描述：

- `dynamic_track_history`：`LINE_STRIP`，使用 `TrackHistorySample.position` 绘制历史轨迹线，颜色 `rgba = (0.10, 0.80, 0.95, 1.0)`，`scale.x = 0.04`
- `dynamic_track_head`：`SPHERE`，显示当前稳定轨迹头部，颜色与历史线一致，`scale.x/y/z = 0.18`
- `dynamic_track_label`：`TEXT_VIEW_FACING`，显示轨迹 ID、类别和速度模长，位置在当前目标上方

`dynamic_track_markers` 也在每帧开头先发一个 `DELETEALL`。如果轨迹当前处于 coasting 状态，则这些轨迹 marker 的 alpha 降低，用来区分“真实检测更新”和“仅预测保活”。当前仍不额外绘制速度箭头，避免 marker 数量和 RViz 负担过大。`/ldop/dynamic_object_markers` 和 `/ldop/dynamic_track_markers` 共同构成第二阶段的检测跟踪可视化：前者看当前帧检测框，后者看跨帧轨迹连续性和 track 状态。

## 数据流

```text
InputFrame
-> UfomapMapper::processInputCloud()
-> 当前帧 static_cloud / dynamic_cloud + dynamic_cluster_points
-> DynamicObjectClusterer::processDynamicObjects()
-> 当前帧 cluster detections + /ldop/dynamic_object_markers
-> DynamicObjectTracker::processDetections()
-> /ldop/dynamic_objects + /ldop/dynamic_track_markers
-> publishFrame()
```

`DynamicObjectClustererFrameResult` 携带当前帧 detection 列表和检测框 marker。`Ldop::ProcessedFrame` 中的 `dynamic_objects_msg` 保存 tracker 输出；`dynamic_object_markers_msg` 保存 clusterer 当前帧检测框；`dynamic_track_markers_msg` 保存 tracker 轨迹可视化。

## 错误处理

- 输入 detection 为空时：所有现有轨迹进入 missed/coasting 流程，超过阈值后删除；删除前继续输出 coasting 轨迹。
- 首帧没有轨迹时：所有 detection 新建轨迹。
- `header.stamp` 为零或非递增时：使用 `tracking_default_dt`，并输出 throttle warning。
- `dt > tracking_max_dt` 时：使用 `tracking_max_dt` 钳制，并输出 throttle warning，避免一次 predict 把协方差放大过头。
- 协方差矩阵非有限或不可逆时：该轨迹本帧不参与关联，保留 predict 状态并计入 missed。
- 非有限 detection 中心或尺寸：在 tracker 输入边界丢弃该 detection，clusterer 已保证的内部数据不重复检查。

## 测试

新增 `test/test_dynamic_object_tracker.cpp`，覆盖：

- 空输入输出空对象数组，且不崩溃。
- 首帧 detection 新建稳定 track id。
- 同一目标连续平移时保持同一 `id`。
- 速度估计方向与位移方向一致。
- 两个目标在可分离距离内各自保持 ID。
- 超过 gate 的 detection 新建新轨迹。
- Hungarian 分配按 LDOT 当前简化实现语义工作：行/列归约后，从零候选中按原始代价选择不冲突匹配。
- gate 外候选即使被填入代价矩阵，也不会被当作有效匹配。
- 漏检不超过 `tracking_max_missed_frames` 时保留 coasting 轨迹。
- 漏检超过阈值后删除轨迹。
- 第一版不实现主动重复轨迹清理；重复轨迹若出现，先通过测试和日志暴露，不在主流程静默合并。
- `DynamicObject` 不包含预测数组字段。
- 尺寸平滑不影响数据关联。
- tracker 输出稳定 `DynamicObjectArray`，clusterer 不再输出帧内 `DynamicObjectArray`。
- clusterer 在没有 tracker 的情况下仍能生成 `/ldop/dynamic_object_markers` 检测框。
- tracker 能根据 `TrackHistorySample` 生成 `/ldop/dynamic_track_markers`，并在 coasting 时降低 marker alpha。
- 时间戳异常时使用默认 `dt`，并触发 warning 日志。
- 帧间隔超过 `tracking_max_dt` 时钳制 `dt`，并触发 warning 日志。
- adaptive Q/R 开启时不会改变目标 ID 分配语义。
- cov limit 固定开启时协方差不会超过配置上限。

新增 `test/test_multi_model_kalman_filter.cpp`，覆盖：

- `MotionModelType::CA3D` factory 能创建有效 filter。
- CA3D 初始状态维度为 9，观测维度为 3。
- `predict(dt)` 按 CA 状态转移更新位置和速度。
- `update(position)` 只使用三维中心观测修正状态。
- adaptive Q/R 的 innovation buffer 达到窗口前不产生异常状态。
- cov limit 会钳制位置、速度、加速度协方差对角线。
- 预留的非 CA3D 类型在第一版不进入 tracker 主流程。

集成层面补充：

- `Ldop::processFrame()` 固定发布 tracker 输出。
- `DynamicObjectClustererFrameResult` 保留当前帧 detection 和检测框 marker，便于测试 tracker 输入和聚类可视化。
- `ProcessedFrame` 同时携带 `/ldop/dynamic_object_markers` 与 `/ldop/dynamic_track_markers`。

## 验收

- 构建通过：`catkin build ldop --no-status`。
- 单包测试通过：`catkin run_tests ldop --no-status`。
- 测试结果汇总无失败：`catkin_test_results /home/st/ldop_ws/build/ldop`。
- `/ldop/dynamic_objects` 中同一动态目标连续帧尽量保持稳定 ID。
- `/ldop/dynamic_objects.velocity` 由 CA 状态估计，不再固定为零。
- `DynamicObject.msg` 不包含预测数组；预测输出未来走独立预测话题。
- `/ldop/dynamic_object_markers` 继续展示当前帧聚类检测框。
- `/ldop/dynamic_track_markers` 展示稳定 track 的历史轨迹和当前头部位置，coasting 轨迹会明显变淡。
- `/ldop/static_cloud`、`/ldop/dynamic_cloud` 和 `/ldop/static_map_markers` 输出语义不变。
