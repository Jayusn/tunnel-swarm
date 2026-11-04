# UAV1 单机桥梁巡检

只运行 UAV1；本机 ROS master 默认 `127.0.0.1:11311`。小车由外部操作，不启动 UAV0、多机跟随器或小车控制器。

流程：小车在入口停稳 → 操作者触发 NMPC 起飞 → 按配置依次给 Diff 发航点 → 出洞 → 到出口停车点上方 → 18 cm ArUco 精降。静态障碍物由 MID360/FAST-LIO 点云进入 Diff 占据地图进行避障；裂缝、热源和二维码检测独立运行，不论检测是否齐全都按航点继续。

## 现场坐标

所有 launch 坐标均为 `UAV1/camera_init`，不是 GPS，也不直接等于离地高度。FAST-LIO 在车上初始化后，地面的 Z 通常不是 0。需要测量/换算：

- `ground_z`：桥洞下地面在 FAST-LIO 坐标系中的 Z。若用车高换算，还要计入 FAST-LIO 机体原点到车面的高度，不能只减 ArUco 平面高度。
- `landing_x / landing_y`：小车在出口的固定停车位置。执行前填写实测值，车辆应在精降前停好。

入口偏移参数 `entrance_x / entrance_y` 已移除。可选 `bridge_yaw` 默认 0，单位弧度，仅填写发出目标的姿态，不再旋转航点 XYZ。

地面和降落点参数在 `launch/bridge.launch` 开头设置，默认 `ground_z=0.0`、`landing_x=11.3`、`landing_y=0.0` 仅供直接启动查看界面，飞行前需填写实测值；默认降落 XY 与示例末点一致，尚未填写出洞右侧 ArUco 的实际位置。桥洞底板 Z=`ground_z+1.5`，顶板 Z=`ground_z+2.7`。默认起飞高度是底板以上 0.5 m（离地 2.0 m），这是待现场调整的初始值，并非已验证的飞行高度。

Diff 的虚拟边界默认向洞内各收缩 0.15 m：下边界为 `ground_z+1.65`，上边界为 `ground_z+2.55`，可规划高度带宽为 0.9 m。可追加 `floor_margin:=0.15 ceiling_margin:=0.15` 分别调整，单位 m。这只改变 Diff 规划边界，不改变实际底板坐标、航点、起飞目标或精降接地高度；洞外的 Diff 航段也使用同一边界。余量应结合定位参考点到机体上下边缘的尺寸及定位/跟踪误差确定，15 cm 是初始配置，尚未实飞验证。调整余量或航点时，应确保目标位于收缩后的高度范围内。

`config/route.yaml` 中每行只填 `[x, y, z]`，直接使用 FAST-LIO 的 `UAV1/camera_init` 坐标（米），程序原样发给 Diff，不做入口偏移、方向旋转或底板高度叠加。当前文件中的数值是待按现场修改的示例，X 不再表示距入口距离；示例 Z=2.0 只在地面 Z=0 时等于离地 2 m。若实测 `ground_z=-0.2`，希望离地 2 m，则各点应填 Z=1.8。填好从入口外、洞内到出口外的点后，程序仍自动追加 `landing_x/y`、`landing_approach_z` 指定的停车点上方航点。

不再填写第四项停留秒数。距离目标不超过 0.15 m 且速度不超过 0.15 m/s 后，立即切换下一点，没有额外停留计时；仍需减速满足到点条件。检测结果不参与切点。旧的四项航点文件需要改成三项，并把坐标换算为 FAST-LIO XYZ，不能只删掉第四项而保留相对底板高度。

起飞高度通过 `cruise_height_above_floor` 配置；它不替代 route.yaml 每个点的高度。最后车上方高度通过 `landing_approach_z` 配置（FAST-LIO Z，默认等于起飞目标）。无人机在起飞位置垂直上升，需要起飞位置有对应上升空间。

## 启动与操作

直接运行即可打开六分屏，使用桥梁配置中的默认参数；不再需要在命令行填写坐标才能查看窗口：

```bash
bash shfiles/start_uav1_bridge_inspection.sh
```

本机 ROS master 就绪后，巡检任务自动打开左侧四路检测、右侧嵌入 RViz 的窗口；不需要先解锁或切 OFFBOARD，没有传感器消息时相应画面显示等待。分屏程序退出后保留终端，便于查看报错。仅使用 `--help` 时才打印帮助并退出。

本机 `/home/oem/.bashrc` 已将新终端的默认 ROS 环境设为 `ROS_MASTER_URI=http://127.0.0.1:11311`、`ROS_IP=127.0.0.1`，并取消 `ROS_HOSTNAME`。桥梁脚本运行后，新开终端即可直接执行 `rostopic list` 或 `rostopic echo -n 1 /UAV1/fast_lio/Odom_high_freq/pose/pose/position`；已经打开的终端需要关闭后重开。原多机启动脚本仍自行设置多机网络，新开查询终端默认连接本机。这项终端设置位于仓库外，复制仓库到其他电脑不会自动修改该电脑的 `.bashrc`。

飞行前在 `launch/bridge.launch` 填好地面 Z、降落点 XY，在 `config/route.yaml` 填好航点。先在车辆静止时完成 FAST-LIO 初始化，再让车辆行驶；整个任务保持同一定位坐标系。也可在启动时临时覆盖配置：

```bash
bash shfiles/start_uav1_bridge_inspection.sh \
  ground_z:=实际地面Z landing_x:=停车X landing_y:=停车Y
```

这些中文占位必须替换成数字。入口打开六个终端：本机 Master/MAVROS、MID360、FAST-LIO、位姿回传、下视相机、巡检任务。请使用这一入口替代同一架机上正在运行的多机或原地降落入口，避免重复控制器。

1. 小车停稳，定位和画面正常后手动解锁。沿用多机 NMPC 操作：CH8 低位，遥控器切入 OFFBOARD 起飞。
2. 到配置的起飞高度后将 CH8 切高位，开始依次执行巡检航点。
3. 小车自行移动到配置的出口位置并停好，无人机端不等待车辆到位话题。
4. 最后一个航点到达后自动启动精降。沿用 18 cm、DICT_4X4_50、下视外参、中心补偿、固定 XY 下降和接地减推力上锁逻辑。CH10 仍交给现有控制器的人工降落流程。

接地基准是起飞时 FAST-LIO 的机体 Z，适用于同一辆车在同一地面标高接回无人机。若出口地面标高不同，设置 `landing_platform_delta_z` 为相对起点的标高差。ArUco 用于末端对准；本任务未增加移动平台追踪、出洞搜车或找不到车时的新处置逻辑，沿用现有精降丢码行为。

## 显示和记录

左侧依次显示裂缝、热源融合、二维码、下视 ArUco；右侧为 FAST-LIO 点云、Diff 轨迹和占据地图、配置航点及确认的热源/二维码标记。D435 只启动一次，裂缝、热源和二维码共用 RGB/对齐深度，不启动独立实验的另一套相机/窗口。

地图中的无人机模型沿用多机的 `odom_visualization`，位置来自 `/UAV1/fast_lio/Odom_high_freq`。橙色 `drone_path` 是 `/UAV1/fast_lio/path` 发布的实际飞行轨迹；`optimal_traj`、`global_path` 和 `goal_point` 分别显示 Diff 发布的规划轨迹、全局路径和目标点，有相应数据后显示。模型与多机使用相同的大小和航向显示配置。

拖动画面之间的横向分隔条可上下调整各检测画面的高度，拖动检测区与地图之间的竖向分隔条可调整左右宽度；图像保持原始宽高比。降落画面沿用独立降落测试的底栏，显示 `/UAV1/fast_lio/Odom_high_freq` 的当前 XYZ（米，`UAV1/camera_init`），保留原有 ArUco 框、ID 和状态文字。没有定位、定位无效或超时时，底栏沿用测试程序的等待/无效/过期提示。

所有检测存图均位于 `~/match_ws/target_reports/bridge`：

- 裂缝：`crack/YYYYMMDD/flight_HHMMSS_微秒/crack_图像时间戳_序号.png`。按同一时间戳匹配 YOLO 检测框与带框图，每个成功配对且含裂缝的检测帧保存一张 PNG，保留原检测框和置信度，底部附相机 XYZ 和地图 XYZ；同名 `.json` 保存每个框的像素范围、置信度、坐标系、米制坐标及定位状态。位置由框中心射线、现有 YOLO 框中心区域的最近有效深度（mm 转 m）及图像时刻 TF 估算，代表检测框的位置，不是整条裂缝的精确几何。地图系为 `UAV1/camera_init`，不是以桥洞入口为原点；未使用 Channel 占位变换。深度或 TF 不可用时仍存图，缺失坐标写 `null`、图上标注 `unavailable`，不使用最新 TF 代替该帧位姿。无裂缝帧不保存，未做裂缝空间去重或三维标记。
- 热源：`YYYYMMDD/flight_HHMMSS/images/annotated/thermal_source/`，沿用已有确认、深度定位、去重和证据图记录。正常图像配对成功后，每个确认记录保存两张图：热成像带框图和 D435 映射带框图（文件名带 `_d435`）；缺失对应图像时沿用原逻辑，不拿其他帧代替。
- 二维码：`YYYYMMDD/flight_HHMMSS/images/annotated/qr_code/`，沿用已有解码、深度验证、确认和去重流程保存带框证据图；同一任务目录中的 `reports.jsonl` 记录确认结果。二维码启动预热为 5 秒，解码或深度验证未通过时不会按已确认目标保存。

关闭显示窗口或用 `enable_view:=false` 不影响检测存图；关闭整个任务才停止记录。

检测保留现有算法参数，但本入口去掉多机的入口等待、前机过滤和旧场地高度/出口过滤。没有远程上报。检测结果不用于延长任务、重复巡检或阻止降落。

可追加 `enable_view:=false`、`enable_thermal:=false` 或 `enable_detection:=false` 进行组件调试；这些开关不改变航点流程。
