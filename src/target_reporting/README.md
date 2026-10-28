# 目标坐标与证据图远程上报

该包统一接收颜色标签、二维码和热源检测结果，通过 ROS TF 转换到 FAST-LIO 的 `camera_init` 世界坐标系，并将其作为比赛语义上的 `channel` 坐标。三个检测器都会先发布原始候选，随后由各自的时间稳定/真实性校验产生 `stable`、`confirmable` 状态；候选立即生成机载端实时观测并显示在 RViz，但只有通过确认门控后才登记、保存证据图并发送远程端。颜色标签和二维码沿用检测器自身门控；热源还需在世界坐标附近稳定满 1 秒且至少获得 3 次有效观测，用于过滤移动人员。当前阶段候选和确认结果都不接入规划器。

保存证据时，通用信息写入原图下方新增的区域，不覆盖检测画面。热成像图和
D435 映射图都保留证据信息栏；D435 映射图本身已有的状态、映射框、深度和坐标
也会继续保留。

热源正式确认还要求缓存中已有同时间的 D435 映射帧。D435 自动曝光预热期间，
热源仍作为本机候选显示，但不开始累计确认；映射画面可用后再连续确认约 1 秒，
从而保证正式热源记录同时保存并上传热成像图和 D435 映射图。

## 数据流

UAV1 通过 `config/uav1_detection_activation.yaml` 统一控制二维码、颜色、热源与上报：
`wait_for_entry_goal: true` 时，程序和相机提前启动并保持预览，首次收到
`entry_goal_topic: /UAV1/planning/goal/entry` 的入口规划目标后才开始识别和保存。
这不是前机的 `workspace_lock` 门位置话题，也不需要等待到达入口；开启后持续运行。
入口目标到来前的事件不会补存，未定位图片也遵守同一条件。设为 `false` 可进行地面检测与保存测试。
该配置由 UAV1 launch 同时加载到三个检测节点及上报节点；独立启动检测包默认立即检测。
原先颜色检测的固定 5 秒等待已移除。

```text
三个检测包的原始候选/相机坐标/检测图
  -> target_reporter_node.py
  -> camera optical frame -> body -> camera_init
  -> 首帧候选机载观测、检测器稳定门控、空间去重、确认JSONL和JPEG
  -> /UAV0/target_reporting/markers（RViz显示）
  -> TCP 5000(确认观测、最终JSON/ACK) + TCP 5001(JPEG/图片ACK)
  -> target_report_server.py
```

FAST-LIO 在整场任务中不得重启或重置。UAV0 统一启动配置使用
`UAV0/camera_init` 作为 ROS 内部定位帧，后机使用 `UAV1/camera_init`。
本机 observation 的 `position.frame_id` 使用该定位帧；最终记录和远程上报
使用 `channel`，通过下面的旋转和平移接口计算。

### 后机雷达世界坐标到通道坐标的预留接口

后机 `uav1_detection_stack.launch` 自动加载
`config/uav1_channel_transform.yaml`，也可通过 launch 参数
`channel_transform_config:=/绝对路径/标定.yaml` 指定文件。
节点私有参数 `world_to_channel/rotation` 是按行展开的 3×3 旋转矩阵，
`world_to_channel/translation` 是三维平移向量（米）：

```text
p_channel = R_channel_world * p_world + t_channel_world
```

其中 `p_world` 属于 launch 的 `world_frame`（默认 `UAV1/camera_init`）；
`t_channel_world` 表示雷达世界原点在通道系中的位置。
例如绕 Z 轴正向旋转 90° 再平移 `[10,20,30]`：

```yaml
world_to_channel:
  rotation: [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  translation: [10.0, 20.0, 30.0]
```

这会把世界点 `[1,2,3]` 转为通道点 `[8,21,33]`。默认文件为单位旋转、零平移，
仅作接口占位，不表示已完成通道标定；填入实际标定值后重启检测入口生效。
转换统一作用于颜色标签、二维码和热源的确认 JSONL、证据图坐标及远程 JSON。
本机 observation、RViz、空间去重和热源过滤继续使用雷达世界坐标。
转换不会发布或修改 TF，不改变飞行控制；历史记录不重算。

## 输入话题

### 后机出口平面过滤

后机入口额外加载 `config/uav1_reporting_filters.yaml`：

```yaml
exit_plane_filter_enabled: true
final_exit_pose_topic: /UAV0/mission/final_exit
```

订阅前机已确认、锁存的 `PoseStamped` 出口消息。门中心定义平面位置，
消息的 yaw 定义朝通道外的水平法向；形成垂直出口平面，不按图像边界裁剪。
目标先按检测时间通过 TF 转到出口消息坐标系，再计算
`d = (target.x - exit.x) * cos(yaw) + (target.y - exit.y) * sin(yaw)`。
`d > 0` 表示出口外：二维码、颜色标签和热源均在累计确认之前排除，
不新增 observation、确认 JSON、照片或远端上报，也不计入三类目标累计完成条件。
平面上和通道内的目标保留。过滤在最终通道坐标转换之前执行。

原始检测、热成像融合和检测调试图继续运行，画面仍可能出现出口外的框。
尚未收到出口、关闭开关或比较所需 TF 不可用时，不应用此过滤；TF 失败会记录日志。
已有照片和 JSON 不删除；重启后机会接收前机仍在发布端锁存的出口消息。
过滤依赖检测三维定位与出口坐标精度，深度把外部人员错误定位到平面内时仍可能保存。
若前机通过人工触发把当前位置作为最终出口锚点，则以该消息位置为边界。
本功能不改变跟随、航向或飞行控制；修改 YAML 后重启后机目标上报节点生效。

### 检测输入

默认使用三个检测包的“原始候选”话题，先用于实时上报和 RViz 显示：

```text
/UAV0/color_tag_detector/candidate_point_camera  geometry_msgs/PointStamped
/UAV0/color_tag_detector/candidate_text           std_msgs/String(JSON)
/UAV0/vision/qr_candidate_pose_camera            geometry_msgs/PoseStamped
/UAV0/vision/qr_candidate_detected               std_msgs/String(JSON)
/UAV0/thermal/target_candidate_camera_point      geometry_msgs/PointStamped
/UAV0/thermal/target_candidate_status            std_msgs/String(JSON)
```

候选状态 JSON 统一包含 `candidate`、`stable`、`confirmable`、
`stable_count`、`stable_window`、`stamp` 和 `reason`。`stamp` 必须与候选点的
ROS 时间戳一致，`target_reporting` 会用它配对点和调试图。`candidate=true, stable=false`
只用于 RViz 观察；`confirmable=true` 才允许 target_reporting 登记并远程发送。
颜色标签的稳定条件为 12 帧窗口内至少 8 次匹配，二维码为内部真实性/深度校验连续
5 帧。热源先通过 3 帧窗口内至少 2 次且像素跳变不超过配置阈值；位于热成像
视野边缘的热点也允许确认。随后还需在世界坐标 0.30 m 范围内稳定满 1 秒且
至少获得 3 次有效观测，相邻观测中断超过 0.50 秒就重新累计，以过滤移动人员。

旧版检测节点可将 `use_detector_candidates` 设为 `false`，回退到下面的稳定结果话题：

```text
/UAV0/color_tag_detector/target_point_camera   geometry_msgs/PointStamped
/UAV0/color_tag_detector/result_text           std_msgs/String(JSON)
/UAV0/color_tag_detector/debug_image           sensor_msgs/Image

/UAV0/vision/qr_pose_camera                    geometry_msgs/PoseStamped
/UAV0/vision/qr_detected                       std_msgs/String(JSON)
/UAV0/vision/qr_debug_image                    sensor_msgs/Image

/UAV0/thermal/target_camera_point              geometry_msgs/PointStamped
/UAV0/thermal/fusion_valid                     std_msgs/Bool
/UAV0/thermal/debug_image                      sensor_msgs/Image
/UAV0/thermal/d435_debug_image                 sensor_msgs/Image
```

所有空间消息必须使用真实相机光学坐标 `frame_id`、源图像时间戳和米制坐标。TF树至少需要连通：

```text
UAV0/camera_init -> UAV0/body -> camera_link -> camera_color_optical_frame
UAV0/camera_init -> UAV0/body -> camera_link -> camera_depth_optical_frame
```

其中 `camera_link` 到彩色/深度光学坐标由 RealSense 发布，`UAV0/body` 到
`camera_link` 使用手眼标定外参。D435 原始点云 `/camera/depth/color/points` 使用
`camera_depth_optical_frame`；显示适配器再输出只包含检测目标附近点的
`/UAV0/target_reporting/detected_object_cloud`，默认保留目标中心半径 0.25 m 内的点。
显示适配器还根据这些目标附近点发布三维轴对齐线框包围盒；包围盒只用于 RViz，
不参与检测判定、避障或规划决策。

检查命令：

```bash
rosrun tf tf_echo UAV0/camera_init camera_color_optical_frame
rostopic echo /UAV0/target_reporting/observation
```

RViz 标记输出：

```text
/UAV0/target_reporting/markers  visualization_msgs/MarkerArray
/UAV0/target_reporting/detected_object_cloud  sensor_msgs/PointCloud2
/UAV0/target_reporting/detected_object_boxes  visualization_msgs/MarkerArray
```

标记使用本机定位坐标：UAV0 比赛配置为数值一致的 `world`，UAV1 为 `UAV1/camera_init`，不使用最终上报的通道坐标。候选球和文字使用半透明颜色，已确认目标使用不透明颜色；三个检测器尚未稳定时文字统一显示“候选(待检测器确认)”，约 0.8 秒没有新观测后自动消失。
包围盒使用 D435 点云坐标系发布并由 RViz TF 转到固定坐标系：颜色标签为橙色、二维码为绿色、热源为品红色。
包围盒是目标附近点云的三维轴对齐包围盒；点云稀疏时使用以检测位置为中心的最小尺寸盒，
因此它不是二维图像像素级分割轮廓。

## 局域网配置

两台电脑连接同一个 Wi-Fi。建议固定地址：

```text
机载端：192.168.10.20
远程端：192.168.10.100
TCP JSON端口：5000
TCP 图片端口：5001
```

在远程端防火墙中允许Python或ROS访问TCP 5000、5001。机载端修改 `config/target_reporting.yaml` 中的 `remote_host`。

## 启动

远程端先启动：

```bash
rosrun target_reporting target_report_server.py \
  --host 0.0.0.0 --port 5000 --image-port 5001 \
  --output ~/received_target_reports
```

Windows PowerShell 运行源码脚本时，先设置模块路径，并让 `--mission-id` 与机载端当天编号一致：

```powershell
$env:PYTHONPATH = (Resolve-Path ".\target_reporting\src").Path
$missionId = "onboard_test_{0:yyyyMMdd}" -f (Get-Date)
python .\target_reporting\scripts\target_report_server.py `
  --host 0.0.0.0 --port 5000 --image-port 5001 `
  --output .\received_target_reports --mission-id $missionId
```

机载端在三个检测节点和 FAST-LIO 正常后启动：

```bash
roslaunch target_reporting target_reporting.launch
```

另开终端启动检测结果 RViz：

```bash
roslaunch target_reporting target_rviz.launch
```

该启动文件只启动显示适配节点和 RViz，不启动规划器，也不会改变飞行逻辑。若只想启动适配节点而手动打开 RViz，可传入 `open_rviz:=false`。

也可在远程端使用：

```bash
roslaunch target_reporting remote_server.launch
```

## 记录目录

UAV1 的前机热源过滤按检测帧时间匹配历史 UAV0 里程计（缓存最近 400 条），
选择时间最近的一条，误差不超过独立的 `leader_odom_match_tolerance_s`（当前 0.06 秒）。
检测点、状态和图像的原有 `source_match_tolerance_s: 0.03` 不受影响。
前机位置的 TF 变换也使用匹配里程计的时间戳，不再使用最新位置和最新 TF。
使用 0.50 米水平过滤半径及 0.50 秒接收新鲜度检查；
无匹配里程计或 TF 不可用时仍跳过前机过滤，不阻止目标后续确认和保存。

`leader_thermal_projection_filter_enabled: true` 开启前机图像方向辅助过滤，
`leader_thermal_projection_radius_m: 0.35` 为前机深度处、垂直相机光轴平面上的区域半径（米）。
参数均位于 `config/target_reporting.yaml`，launch 不覆盖这些调节参数。
原有水平距离过滤未命中时，将匹配的前机位置转换到检测时刻的 D435 光学坐标系，
再把热源方向 `(X/Z, Y/Z)` 投影到前机深度平面。两者平面距离小于等于配置半径时排除。
这相当于图像中随距离缩放的前机区域，不依赖热源所取深度的大小，也不需要重复订阅内参。
因此深度取错但方向仍在前机区域内的热源也可过滤；前机之后或之前、与其画面重叠的真实热源也可能被排除。
设为 `false` 可关闭辅助判断，保留水平距离过滤。缺少里程计、TF，或前机在相机后方时不会据此过滤。
过滤发生在正式确认/保存之前，原始检测和实时调试图继续运行。
日志分别记录水平距离命中、投影区域命中和里程计时间差过大，方便区分原因。


`~enable_remote_upload` 控制远端发送，默认 `true`。UAV1 的
`uav1_detection_stack.launch` 当前设为 `false`：仍检测、确认、发布本机观测，
并保存 JSONL 和证据图，但不创建网络客户端、不重试连接、不向发送队列入队。
UAV1 保存到 `~/match_ws/target_reports/uav1/<任务编号>/`。
修改此开关后需重启上报节点；重新开启时，仍会按原逻辑补发当前任务目录内未收到 ACK 的记录和图片。

机载端默认写入：

```text
~/target_reports/mission_YYYYMMDD_HHMMSS/
  reports.jsonl
  images/annotated/qr_code/*.jpg
  images/annotated/color_tag/*.jpg
  images/annotated/thermal_source/*.jpg
```

机载图片按目标类型分目录：`qr_code` 为二维码，`color_tag` 为颜色标签，
`thermal_source` 为热源（热成像图和对应 D435/D455 映射图都在这里）。
启动上报节点时创建三个文件夹；未检出的类型文件夹为空。每次飞行仍共用一份 `reports.jsonl`，
图片 ID 保持原文件名，可根据记录的 `target_type` 在对应文件夹中找到图片。
旧的平铺图片不自动搬迁，重启补发同时支持新分类目录和旧目录。

当前 `target_reporting.yaml` 设置 `save_unlocalized_evidence: false`：三类目标必须
通过原有深度定位与确认流程才保存图片和 JSON，不再新增未定位记录；已有历史图片保留。
入口目标触发与正常定位记录的保存流程继续生效。

以下为可选未定位留图功能的说明，仅将 `save_unlocalized_evidence` 改为 `true` 时启用。
`save_unlocalized_evidence: true` 将本地图片留存与
三维定位分开。二维码已解码验证、颜色标签通过现有颜色/轮廓/形状检查、热源通过
检测器的稳定检查后，即使深度缺失或定位质量不合格，也可保存同帧图片。
颜色标签仍使用彩色/深度同步回调：这里支持深度图中无效值或质量不足，
不改变整个深度话题停止发布时的同步机制。没有通过外观识别的原始候选不留存。

未定位图片保存在对应类型文件夹，文件名为 `uav1_unlocalized_<六位编号>_<类型>.jpg`，
标记 `UNLOCALIZED`；独立的 `unlocalized.jsonl` 记录 `localization_label: "未定位"`、
`position: null`、图像几何和文件信息。同类型首次匹配成功立即保存；之后相对上次
保存，时间间隔达到 `unlocalized_save_interval_s`（默认 5 秒），或原图中心位移
达到 `unlocalized_center_shift_px`（默认 80 像素），即可再次保存。比较使用检测帧
时间戳；旧帧或同一时间戳不重复保存。热源按热图像素、颜色/二维码按彩色图像素计算。
这只是留图去重，不代表确认发现了新的物理目标。使用相同任务目录重启时继续编号和
去重，兼容旧的首张记录及文件名，已有图片不覆盖。
随后定位成功仍走原确认流程，保存带坐标的 `reports.jsonl` 和图片，旧证据保留。
未定位记录不发布为确认观测，不参与三类累计停扫，也不进入远端上传或重启补发。

没有有效三维坐标时无法判断出口平面、地面高度和前机空间距离，记录中会写明
`spatial_filters: "not_evaluable_without_valid_position"`。热源有相机方向时仍尝试
现有前机投影过滤；`leader_projection_status` 为 `clear` 表示完成投影比较且未落入
前机区域，`incomplete` 表示缺少方向/里程计/TF 等信息而未完成检查，`disabled`
表示过滤开关未启用；未完成或关闭时图片底部也标注 `not verified`。这不意味着
自动排除前机，且不新增拦截条件。未定位热源不要求配套 D435 深度映射图。
已定位目标继续使用原来的前机、地面和出口过滤，所有确认阈值保持原配置。

远程端仍写入指定的 `--output` 下的任务目录，接收图片沿用 `images/annotated/*.jpg` 平铺结构。
候选观测和确认观测都会发布到机载端的
`/UAV0/target_reporting/observation`，因此规划器 RViz 仍能看到候选；默认只有确认观测
通过 TCP 5000 发往远程端，候选不会传到 Windows，也不会生成远程记录或图片。只有确认目标
生成带坐标的正式 JSON 和证据图片；颜色标签、二维码保留检测器框选。
热源通过 `thermal_evidence_image_topic` 订阅无框原图（UAV1 为 `/UAV1/thermal/image_raw`），
保存时只绘制本条记录的 `bbox`，不使用可能框着另一个候选的通用调试图。
热图严格匹配融合消息的 `source_timing.thermal_stamp`；融合输出的 D435 证据图
则严格匹配当前候选时间戳（同帧多候选用微秒差异区分），不会用旁边候选的图替代。
这里匹配的是融合节点重新标记后的证据时间戳，不要求两台相机硬件同步。
原有 RViz 调试图继续发布。确认热源时会同时上传
热成像标注图和 D435 映射标注图；确认目标不会再产生飞行目标。
候选观测只发布在机载 ROS 话题供 RViz 使用，远程端始终只接收确认结果。

确认事件的证据图必须与检测点使用同一源时间戳（默认允许误差 0.03 秒）。上报节点会等待
`image_wait_timeout_s`（默认 0.50 秒）让同帧调试图进入缓存；超时只发送确认 JSON，绝不使用
前后时刻的未稳定候选图替代。

后机每次飞行都会重新启动检测程序，因此按启动时间区分保存目录。
`start_uav1_six_terminator.sh`、`start_uav1_seven_terminator.sh` 和后机 `mission_id: auto`
统一使用 `YYYYMMDD/flight_HHMMSS`，先按日期，再在日期下面分每次飞行。
日期和时间取本机启动时的当地时间，精确到秒。
例如两次飞行分别保存为：

```text
~/match_ws/target_reports/uav1/20261027/flight_143000/
  reports.jsonl
  images/annotated/qr_code/*.jpg
  images/annotated/color_tag/*.jpg
  images/annotated/thermal_source/*.jpg
~/match_ws/target_reports/uav1/20261027/flight_150500/
  reports.jsonl
  images/annotated/qr_code/*.jpg
  images/annotated/color_tag/*.jpg
  images/annotated/thermal_source/*.jpg
```

各目录独立编号，不再把同一天后续飞行追加进前一次目录；旧的按日期目录保留。
目录按启动而非解锁/起飞切换：同一次启动中连续起降仍写同一目录；中途重启且使用自动编号会创建新目录。
前机 `auto` 仍生成 `onboard_test_YYYYMMDD`。显式指定 `mission_id` 则保持原有目录复用行为。
若开启远端上传，Windows 端要使用与机载端完全相同的任务编号；
同一场任务中机载端和远程端必须保持该值不变。显式指定旧编号重启，会根据
`event_acks.jsonl` 和 `image_acks.jsonl` 只补发尚未确认的最终 JSON 和图片。若需要手动指定编号，
可在机载端启动时传入 `mission_id:=final_20260811_01`，Windows 端的 `--mission-id` 使用完全相同的值。

同一目标类型可以存在多个空间候选。相距超过 `dedup_distance_m` 的后续真目标会获得新的 `target_id`，不会被先前稳定误检阻挡。对于解码内容非空的二维码，内容优先作为目标身份：相同内容即使因定位漂移超过空间阈值也只上报一次，不同内容仍分别登记。ACK重传保持相同 `seq`，远程端只保存一次。

## 真机测试清单

1. 使用 `rostopic echo` 确认三类相机坐标都带正确时间戳、光学帧名和米制数值。
2. 使用 `tf_echo` 确认三个相机帧均可转换到 `camera_init`。
3. 在已知位置放置目标，核对 RViz 标记、实时观察话题和远程端世界坐标一致。
4. 确认目标后检查机载与远程端均生成 JSONL 和可打开的标注JPEG。
5. 断开 Wi-Fi，确认检测和飞行不停止；恢复后确认待发结果自动补发。
6. 在最终消息尚未收到ACK时重启机载上报节点，确认同一 `mission_id` 下未确认JSON和图片会从磁盘重新加载并补发。
7. 重复发送相同 `seq`，确认远程 JSONL 只增加一条。
8. 先制造一个稳定误检，再在距离大于0.3米处放置真目标，确认远程端出现第二个 `target_id`。
9. 比赛任务期间不要重启FAST-LIO；若发生重启，应重新开始本场记录，不能混合两个世界原点。

## 本机关键测试

```bash
python target_reporting/test/test_core.py -v
python target_reporting/test/test_transport.py -v
python target_reporting/test/test_end_to_end.py -v
python -m compileall -q target_reporting
```

## Checkerboard hand-eye calibration

The calibration tool uses this aircraft's checkerboard: 11x8 inner corners
and 0.040 m square size. It estimates the fixed `body -> camera_link` transform
from FAST-LIO `/Odometry` and the RealSense color image without changing any
detector or flight-control node.

Collect a bag while the board is fixed and the complete rigid camera, IMU, and
LiDAR assembly is moved slowly through 10-20 varied poses:

```bash
mkdir -p ~/handeye_calibration
rosbag record -O ~/handeye_calibration/checkerboard.bag \
  /Odometry /camera/color/image_raw /camera/color/camera_info /tf_static
```

Run the solver:

```bash
source /opt/ros/noetic/setup.bash
source ~/db_ws/devel/setup.bash
rosrun target_reporting checkerboard_handeye_calibrate.py \
  --bag ~/handeye_calibration/checkerboard.bag \
  --output ~/handeye_calibration/body_camera.yaml \
  --pattern-cols 11 --pattern-rows 8 --square-size 0.040
```

The bag must contain the RealSense `/tf_static` chain from `camera_link` to
`camera_color_optical_frame`. Inspect `samples_used`, reprojection error, and
target-pose spread before using the result.

After a valid YAML is produced, publish both transforms:

```bash
roslaunch target_reporting camera_body_tf.launch \
  calibration_file:=/home/asus/handeye_calibration/body_camera.yaml \
  enable_odom_tf:=false
rosrun tf tf_echo camera_init body
rosrun tf tf_echo camera_init camera_color_optical_frame
```

Do not start `camera_body_tf.launch` before a real calibration YAML exists;
there is deliberately no identity-transform fallback.
Set `enable_odom_tf:=false` when the localization node already broadcasts the
world-to-body TF. Enable it only for an odometry source that does not publish TF.

OpenCV图像标注和ROS TF需要在安装了ROS Noetic、`python3-opencv` 与相机驱动的Ubuntu机载环境中实测。
