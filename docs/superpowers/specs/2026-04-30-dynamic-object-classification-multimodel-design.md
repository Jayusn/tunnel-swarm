# LDOP 动态目标分类与多运动模型 KF/EKF 设计

## 状态

本文档是第二阶段动态目标跟踪设计的增量补充，目标是参考 `/home/st/catkin/LDOT/ldot_detector`，在当前 LDOP 跟踪链路上补齐对象类别分类和类别驱动的多运动模型切换。

本文档不替代 `docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md`。当前 LDOP 主线已经有 `CA2D / CA3D / CV3D / CTRA` 运动模型和 `LinearKalmanFilter / ExtendedKalmanFilter` 底座。实现收口后，tracker 主流程已经按类别切换模型，并对外发布目标类别和当前运动模型类型。

实施状态（2026-05-14）：本文档对应的对象类别、证据积分、临时合并/分离保护和类别驱动模型切换已经完成，第二阶段整体已标记完成。后续预测能力属于第三阶段，不回写到当前跟踪消息。

## 目标

- 为已跟踪的动态目标增加对象类别分类。
- 保持 LDOP 现有 UFOMap 动静态分割职责不变；本轮不重新实现动静态分类。
- 建立类别到运动模型的映射：
  - `HUMAN` -> `CA2D` + linear KF
  - `VEHICLE` -> `CTRA` + EKF
  - `UAV` -> `CA3D` + linear KF
  - `OTHER` -> `CV3D` + linear KF
- 新轨迹对外先保持未分类状态，内部用 `CV3D` 作为初始滤波模型。
- 轨迹拥有足够历史后才分类；如果类别变化，在当前帧 Kalman update 前切换滤波模型。
- 引入 LDOT 的临时合并/分离保护机制，但只在 bbox 尺寸和点数层面适配。
- `ldop/DynamicObject.msg` 发布 `uint8 object_class`，并保留当前模型原生状态、协方差和 `motion_model_type`，供后续预测模块按模型解释。
- 分类状态、模型切换和合并/分离保护都归 `DynamicObjectTracker` 管理，不塞回 `DynamicObjectClusterer` 或 `UfomapMapper`。

## 非目标

- 不移植 LDOT 的点云运动投票式动静态分类。
- 不在本轮引入 PCA/std 类别判别。
- 不实现短时轨迹预测。
- 不向 `DynamicObject.msg` 增加 prediction 字段或 classification confidence；模型内部的 acceleration、yaw 等状态只通过 `model_state` 按 `motion_model_type` 间接表达。
- 不在本轮实现 LDOT 的 `removeDuplicateTracks()`。
- 不改变 clusterer 的连通域聚类或竖向碎片合并行为；只把已有 `point_count` 通过内部 detection 合同传给 tracker。

## LDOT 参考流程

LDOT 中和本轮相关的跟踪顺序是：

```text
runTracking()
  -> boxAssociation()
       - predict existing filters
       - build detection-track cost matrix
       - solve assignment
  -> classify matched old tracks when classification timing allows it
  -> switchKalmanModel() before the matched track update
  -> kalmanFilterAndUpdateHist()
```

LDOT 新轨迹默认是 `is_else`，并使用 3D CV filter。只有当轨迹达到 `classification_start_frame` 后才开始分类；之后按 `classification_interval_sec` 间隔重新分类，直到连续稳定分类把类别锁定。

LDOP 保留“先关联、后分类、再在 Kalman update 前切模型”的时序，但不照搬 LDOT 的固定间隔和连续次数锁定。LDOT 里混在一起的两个概念在 LDOP 中拆开：

- 新轨迹内部默认滤波模型：`CV3D`
- 新轨迹对外类别：`UNKNOWN`

这样新目标刚出现时不会被下游误认为已经确认是 `OTHER`。类别稳定性改用轨迹内部的证据积分来判断，短时误判和短时遮挡不会直接打断历史判断。

## 数据合同

### DynamicObject.msg

当前对外 ROS 消息包含类别枚举、运动模型枚举、目标尺寸、模型原生状态和模型原生协方差。类别相关字段为：

```text
uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4

uint8 object_class
```

语义如下：

- `CLASS_UNKNOWN`：轨迹还没有达到可靠分类条件。
- `CLASS_OTHER`：已经执行过分类，但不满足 human、vehicle 或 UAV 规则。

当前接口不再单独发布公共 `center/velocity/covariance` 字段。目标中心固定在 `model_state[0..2]`；速度和协方差由下游结合 `motion_model_type` 解释 `model_state/model_covariance` 得到。

### 内部 Detection

给 `DynamicObjectDetection` 增加点数：

```text
point_count: std::size_t
```

`DynamicObjectClusterer` 现在已经在 `Fragment::point_count` 中统计点数；`fragmentToDetection()` 只需要把它复制到 detection。这个字段只属于 clusterer 到 tracker 的内部合同，不改变 ROS 消息。

`point_count` 只用于临时合并/分离保护。tracker 不消费原始点云，也不维护点云历史。

### 内部分类状态

新增一个轻量内部枚举：

```text
ObjectClass
- Unknown
- Human
- Vehicle
- Uav
- Other
```

每个 `TrackState` 增加这些状态：

- `object_class`：当前对外类别，初始为 `Unknown`
- `classification_scores`：`Human / Vehicle / Uav / Other` 四类证据分数，不给 `Unknown` 积分
- `max_history_size`：该轨迹已经接受的历史最大 bbox 尺寸
- `last_point_count`：上一帧匹配 detection 的点数
- `large_size_counter`：连续临时合并计数
- `small_size_counter`：连续临时分离计数

这些状态属于 `TrackState`，因为它们依赖轨迹历史和匹配结果，不是当前帧 detection 的局部属性。

## 单帧流程

`DynamicObjectTracker::processDynamicTracks()` 的单帧流程调整为：

```text
processDynamicTracks(header, detections)
  -> predictTracks(header.stamp)
  -> buildCostMatrix()
  -> selectAssociations()
  -> for each match:
       prepareMatchedTrackForUpdate(track, detection, stamp)
         -> update merge/separation protection
         -> classify current observation when hits are sufficient
         -> update classification evidence if observation is reliable
         -> switch model if confirmed class changed
       updateMatchedTrack(track, detection, stamp)
  -> createTrack() for unsuppressed unmatched detections
  -> coastTrack() for unmatched old tracks
  -> deleteExpiredTracks()
  -> buildOutput()
  -> buildTrackMarkers()
```

关键顺序是：先完成关联，再对匹配到的旧轨迹做分类和模型切换，最后才执行当前帧 Kalman update。

## 新轨迹语义

创建新轨迹时：

- `object_class = Unknown`
- `model_type = CV3D`
- filter 使用 `MotionModelType::CV3D`
- `max_history_size = detection.bbox.size`
- `last_point_count = detection.point_count`
- 分类证据分数和尺寸突变计数器清零
- 对外输出在真正分类前保持 `CLASS_UNKNOWN`

这样既贴近 LDOT 的稳定默认模型，又不会把未分类目标对外说成已确认 `OTHER`。

## 类别证据更新

分类只对匹配成功的旧轨迹执行。

使用这些参数：

- `classification_start_frame`
- `classification_score_decay`
- `classification_score_increment`
- `classification_confirm_score`
- `classification_switch_margin`

满足以下条件时才允许更新类别证据：

- `track.hits >= classification_start_frame`
- 当前帧成功匹配 detection
- 当前 detection 没有处在临时合并、临时分离或疑似遮挡观测状态

满足条件后，每个匹配帧都执行一次轻量 bbox 分类，不再等待固定时间间隔。这样稳定目标可以更快积累证据，偶发误判也不会像连续次数方案那样直接清零。

证据更新规则：

- 每次可靠观测先让四个类别分数乘以 `classification_score_decay`。
- 当前帧分类结果对应的类别增加 `classification_score_increment`。
- 取最高分和第二高分；如果最高分大于等于 `classification_confirm_score`，且领先第二名至少 `classification_switch_margin`，则把最高分类别作为确认类别。
- 如果确认类别和 `object_class` 不同，就在本帧 Kalman update 前切换运动模型。

`Unknown` 不参与积分。轨迹在任意类别达到确认条件前，对外继续发布 `CLASS_UNKNOWN`。

这个策略不再需要 `classification_locked`。类别不是永久锁死，而是通过分数和 margin 形成滞回：已经确认的类别会保留，只有另一个类别积累出明显更强的证据时才切换。`Other` 可以被确认并发布，但它也不会变成永久锁定状态；后续如果 human、vehicle 或 UAV 证据足够强，仍然可以切走。

### 短时遮挡

短时遮挡或短时未匹配时，轨迹进入已有 coasting 流程。coasting 帧不更新 `classification_scores`，也不降低已有分数；缺少观测只代表没有新证据，不代表类别判断被推翻。

遮挡后重新匹配到目标，如果 bbox 尺寸明显变小或点数突降，先按临时分离处理。这些疑似遮挡残片帧冻结类别证据，等尺寸和点数恢复稳定后再继续积分。

模型切换只允许发生在有可靠匹配 detection 的帧内；coasting 阶段保持原类别和原运动模型。

## 临时合并/分离保护

分类前先按 LDOT 逻辑更新尺寸稳定状态。LDOP 的输入来自 detection，不引入原始点云历史。

输入：

- 当前 detection 的 bbox 尺寸
- 当前 detection 的点数
- 轨迹的 `max_history_size`
- 轨迹的 `last_point_count`

参数：

- `classification_size_change_ratio`
- `classification_point_count_change_ratio`
- `classification_size_change_confirm_frames`

派生阈值：

```text
merge threshold = 1.0 + ratio
separation threshold = 1.0 - ratio
```

计算量：

- `size_ratio_x/y/z = current_size / max(max_history_size, epsilon)`
- `max_size_ratio = max(size_ratio_x, size_ratio_y, size_ratio_z)`
- `point_ratio = current_point_count / max(last_point_count, 1)`

规则：

- 如果 `max_size_ratio > 1 + classification_size_change_ratio`，或 `point_ratio > 1 + classification_point_count_change_ratio`，视为临时合并。
- 如果 `max_size_ratio < 1 - classification_size_change_ratio`，或 `point_ratio < 1 - classification_point_count_change_ratio`，视为临时分离。
- 临时合并时递增 `large_size_counter`，并重置 `small_size_counter`。
- 临时分离时递增 `small_size_counter`，并重置 `large_size_counter`。
- 如果 `large_size_counter >= classification_size_change_confirm_frames`，接受更大的目标尺寸，并允许 `max_history_size` 继续增长。
- 如果 `small_size_counter > classification_size_change_confirm_frames`，接受更小的目标尺寸，并把 `max_history_size` 重置为当前尺寸。
- 在合并/分离仍是临时状态时，用 `max_history_size` 代替当前尺寸参与分类和 bbox 尺寸稳定。
- 在合并/分离仍是临时状态时，冻结类别证据分数，不让粘连或残片帧影响类别判断。
- 如果没有临时合并/分离，则按分量用当前尺寸更新 `max_history_size`。

这个机制只保护分类和 bbox 尺寸稳定，防止瞬时粘连或分裂污染类别判断。它不删除轨迹、不合并轨迹，也不改变 clusterer 输出。

## 分类规则

分类使用 LDOT 的核心 bbox/质心规则，并按 LDOP 当前能提供的数据做适配。

输入：

- 经过临时合并/分离保护后的稳定尺寸
- 当前 detection 中心高度

参数：

- `classify_human_threshold: [z_width_ratio, centroid_z_ratio]`
- `classify_vehicle_threshold: [xy_width_ratio, centroid_z_ratio]`
- `classify_uav_threshold: [max_size, centroid_z_ratio]`

分类规则：

- `Human`：`z_width >= max(x_width, y_width) * human_z_width_ratio`，且 `centroid_z < z_width * human_centroid_z_ratio`
- `Vehicle`：`max(x_width, y_width) >= z_width * vehicle_xy_width_ratio`，且 `centroid_z < z_width * vehicle_centroid_z_ratio`
- `UAV`：三个尺寸都小于 `uav_max_size`，且 `centroid_z > z_width * uav_centroid_z_ratio`
- 其他情况为 `Other`

分类结果只是当前帧观测，不直接等同于轨迹类别。轨迹类别由上一节的证据积分确认。

LDOT 的 `fix_size` 同时承担“分类锁定”和“尺寸固定”两层含义，LDOP 本轮不照搬这种耦合。类别稳定由 `classification_scores`、`classification_confirm_score` 和 `classification_switch_margin` 判断；bbox 输出尺寸仍按 tracker 的尺寸平滑和临时合并/分离保护更新。换句话说，`classification_size_change_confirm_frames` 只判断尺寸/点数突变是否真实，类别证据积分只判断类别是否足够可靠。

## 模型映射与切换

模型映射：

```text
Unknown -> 保持当前模型
Human   -> CA2D
Vehicle -> CTRA
UAV     -> CA3D
Other   -> CV3D
```

如果一次分类导致目标模型变化，必须在当前帧 matched-track update 前切换 filter。

状态迁移要尽量保留位置和世界坐标系速度：

- 从 `CV3D` 读出 `[x, y, z, vx, vy, vz]`
- 从 `CA2D` 读出 `[x, y, z, vx, vy, ax, ay]`，并令 `vz = 0`
- 从 `CA3D` 读出 `[x, y, z, vx, vy, vz, ax, ay, az]`
- 从 `CTRA` 读出 `[x, y, z, v, a, yaw, yaw_rate]`，并转换为 `vx = v*cos(yaw)`、`vy = v*sin(yaw)`、`vz = 0`

新模型初始化状态：

- `Human/CA2D`：`[x, y, z, vx, vy, 0, 0]`
- `Vehicle/CTRA`：`[x, y, z, speed, 0, yaw, 0]`
- `UAV/CA3D`：`[x, y, z, vx, vy, vz, 0, 0, 0]`
- `Other/CV3D`：`[x, y, z, vx, vy, vz]`

车辆 yaw 的处理：

- 如果水平速度足够大，用 `atan2(vy, vx)`。
- 如果速度接近 0，且旧模型已经是 CTRA，则保留旧 yaw。
- 否则使用 `0`。

切换后，用迁移后的状态初始化新 filter，并保留 track id、bbox、history、hits 和分类状态。

## 参数组织

采用混合参数组织：

- 已有生命周期和关联参数继续使用扁平 `tracking_*`。
- 对象类别分类参数使用扁平 `classification_*` 和 `classify_*`。
- 多运动模型 Kalman 参数使用 LDOT 风格的 `kalman_filter/...` 分组。

分类参数：

```yaml
classification_start_frame: 10
classification_score_decay: 0.85
classification_score_increment: 1.0
classification_confirm_score: 3.0
classification_switch_margin: 1.5
classification_size_change_ratio: 0.5
classification_point_count_change_ratio: 0.5
classification_size_change_confirm_frames: 5
classify_human_threshold: [1.1, 0.8]
classify_vehicle_threshold: [1.5, 0.7]
classify_uav_threshold: [0.6, 1.5]
```

Kalman 参数分组：

```yaml
kalman_filter:
  adaptive_window_size: 10
  adaptive_alpha: 0.7
  adaptive_r_alpha: 0.7
  adaptive_min_noise_ratio: 0.2
  enable_cov_limit: true
  max_pos_cov: 1.5
  max_vel_cov: 9.0
  max_acc_cov: 25.0
  ca_model:
    human:
      jerk_sigma: 1.0
      init_cov: [0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 10.0]
      meas_noise: [0.1, 0.1, 0.1]
      z_process_noise: 0.01
    uav:
      jerk_sigma: 1.0
      init_cov: [0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0]
      meas_noise: [0.1, 0.1, 0.1]
  cv_model:
    acc_sigma: 3.0
    init_cov: [0.5, 0.5, 0.5, 2.0, 2.0, 2.0]
    meas_noise: [0.3, 0.3, 0.3]
  ctra_model:
    init_cov: [0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0]
    process_noise: [0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0]
    meas_noise: [0.1, 0.1, 0.1]
```

`prediction_cov_multiplier` 仍然不进入本轮，因为 LDOP 的预测能力属于后续阶段。

## 输出与可视化

`buildOutput()` 对外发布：

- 稳定 `id`
- 平滑后的 `size`
- 当前模型原生 `model_state`
- 当前模型原生 `model_covariance`
- `object_class`
- `motion_model_type`

`DynamicObject.msg` 不发布预测数组；第三阶段预测仍走独立话题。

RViz 话题保持检测和跟踪分离：

- `/ldop/dynamic_object_markers`：clusterer 当前帧 detection box
- `/ldop/dynamic_track_markers`：tracker 历史线、轨迹头部球，以及目标上方 ID/类别/速度文字

可以选择按类别给轨迹 marker 上色，但这只能是 `DynamicObjectTracker` 内部可视化行为，不应要求新增对外消息。若实现类别上色，coasting 状态的 alpha 降低仍然优先生效。

## 测试

本轮至少覆盖：

- `DynamicObject.msg` 中 `object_class`、`motion_model_type`、`model_state` 和 `model_covariance` 的语义与当前实现一致。
- `DynamicObjectDetection::point_count` 能从 clusterer fragment 传出。
- 新轨迹对外发布 `CLASS_UNKNOWN`，内部使用 `CV3D`。
- 匹配轨迹达到 `classification_start_frame` 后能分别分类为 `Human`、`Vehicle`、`UAV`、`Other`，并映射到预期模型。
- 可靠匹配帧会更新 `classification_scores`，并在分数和 margin 达标后确认类别。
- 偶发一帧误判不会立刻切换已确认类别。
- coasting 帧冻结类别证据，不会降低类别分数或切回 `Unknown`。
- 临时合并、临时分离和疑似遮挡残片帧冻结类别证据。
- 模型切换能在 `CV3D -> CA2D`、`CV3D -> CTRA`、`CV3D -> CA3D`、`CA3D -> CV3D` 时保持位置和速度。
- 模型切换发生在 matched-track update 前。
- 临时合并使用 `max_history_size`，不会立刻改变类别。
- 临时分离使用 `max_history_size`，不会立刻缩小分类尺寸。
- 持续合并超过确认阈值后接受更大尺寸。
- 持续分离超过确认阈值后重置 `max_history_size`。
- `Other` 可以被确认并发布，但不会永久阻止后续类别切换。
- 类别确认后，bbox 输出尺寸仍可继续按尺寸平滑和合并/分离保护更新。

验证命令：

```bash
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
git diff --check
```

如果后续再次修改 `msg/*.msg`，实现后必须重新执行 `catkin build ldop --no-status`，触发消息生成；本次接口收口只同步当前 `DynamicObject.msg` 的文档描述，不要求修改消息定义。

## 文档同步

实现完成后需要同步更新：

- `docs/note/dynamic_object_tracker_数学逻辑说明.md`
- `docs/note/dynamic_object_tracker_执行流程说明.md`
- `docs/note/LDOP_design.md`，如果第二阶段能力摘要涉及 tracking 输出

文档中必须明确：LDOP 本轮不移植 LDOT 的动静态运动投票分类，因为 UFOMap 链路已经负责动静态分割。

## 实现注意事项

- 本轮删除 `classification_interval_sec`、`classification_stable_confirmations` 和 `classify_xy_distance_threshold`。平台近场保护需要清晰的平台位姿输入和实际误分类证据，后续如果要做，再作为独立保护机制加入。
- 当前主工作区没有 tracker/Kalman 测试文件，尽管之前 worktree 进度记录里出现过这些测试。实现时应恢复或新增测试，不能假设临时工作区测试仍存在。
- 分类状态保留在 `TrackState` 内部。除非后续出现明显重复逻辑或稳定复用边界，否则不要为了“看起来更模块化”抽一个独立 classifier 类。
