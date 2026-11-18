# RRT＋MINCO 独立适配

本包新增静态地图点到点规划入口，不修改 FUEL、Diff、控制器及现有任务入口。
算法来自 `rrt_super_bundle_20261116`，基础提交
`6003f70ac1520447137bf01854813c9c7a895818`，原包包含未提交源码；迁入的
`rrt_core` 保留原算法主体，并适配独立的边界净空选项。原包未为这些 Python 文件明确指定许可，
本包不作额外开源授权。SUPER 未纳入此次接入。

## 不飞行：雷达三维体素栅格地图显示

保留ROS master、MID360和FAST-LIO。停止旧的 `cloud_to_scene_node.py` 后，执行：

```bash
source ~/match_ws/devel/setup.bash
source ~/match_ws/shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav0
roslaunch rrt_minco_adapter grid_map_view.launch
```

该入口只启动独立FUEL建图节点、占据快照转换节点和RViz，不启动探索FSM、控制器或飞行任务。
`rrt_fuel_map` 直接链接原 `fuel_plan_env` 库，加载原
`exploration_manager/config/fuel_map.yaml`，不修改FUEL核心代码。
输入为 `/UAV0/fast_lio/cloud_registered` 和同坐标系的
`/UAV0/fast_lio/Odometry`（点云与里程计按时间同步）。

链路：FAST-LIO → FUEL概率占据/射线融合 → `/UAV0/rrt/fuel/occupancy_all`
→ 当前占据快照 → 三维显示及RRT高度切片。每次快照替换旧内容，保留FUEL清除结果，
不再永久累计单次点云命中。FUEL的膨胀层噪点过滤与原始概率占据层不同；此入口使用后者，
RRT仍自行计算净空，不能宣称所有噪点都被滤除。

建图参数沿用原实飞：分辨率0.05m、命中0.65、未命中0.35、占据阈值0.80、
最大射线距离8m，关闭静态永久保留。默认地图尺寸20×20×2.5m，
实际边界X/Y为-10～10m、Z为-1～1.5m。完整显示此FUEL地图，
不按RRT的0～5m规划区域裁剪显示。FUEL使用有限大小的稠密地图，并非无限范围地图。
扩大显示/建图范围可显式传参（会增加内存和运算量），例如：

```bash
roslaunch rrt_minco_adapter grid_map_view.launch map_size_z:=6.0
```

此例Z范围为-1～5m；`map_size_x/map_size_y/map_ground/raycast_max_dist`也可设置。
规划区域仍为X/Y=0～5m，Z=0.3～0.7m仅用于规划障碍切片，路径高度0.5m。
RViz默认以Orbit立体视角显示 `/UAV0/rrt/voxel_map` 的0.1m立方体，按高度青→蓝→紫着色，
红色路径来自 `/UAV0/rrt/dry_run_path`。`resolution`参数控制转换格大小，不修改FUEL分辨率。

扫描后冻结供规划和显示使用的占据快照：

```bash
rosservice call /UAV0/rrt/freeze_map "{}"
rosrun rrt_minco_adapter ground_plan.py
```

冻结后FUEL上游继续融合，RRT快照和默认RViz体素保持冻结状态。
RRT只接收占据障碍，仍不会使用FUEL自由/未知分类来约束路径；必须确认规划区域已扫描。
重启入口会清空地图，需要重新扫描；雷达与FAST-LIO保持运行，不重复启动。
已有新入口运行时，仅打开RViz可用 `start_mapper:=false`，后台建图可用 `rviz:=false`。
手动配置RViz：Fixed Frame=`UAV0/camera_init`，Marker Topic=`/UAV0/rrt/voxel_map`。

构建与隔离验证：

```bash
catkin_make -C ~/match_ws --pkg rrt_minco_adapter -j2
source ~/match_ws/devel/setup.bash
python3 ~/match_ws/src/rrt_minco_adapter/tests/fuel_map_smoke.py
```

本次仅将不飞行入口 `grid_map_view.launch` 改为FUEL建图；
`real_cloud.launch` 和自动飞行入口仍保留原先点云累计方式。

### 第六步：直接调用地面规划脚本

地面脚本取消边界净空，允许起点位于边界，整条路径仍限制在闭区间 `[0,5]×[0,5]`。
真实障碍不删除，仍使用0.2m障碍净空＋0.1m跟踪余量。自动飞行入口的起终点和边界策略不变。

冻结地图后，在已经加载工作区和相同ROS网络环境的终端执行：

```bash
rosrun rrt_minco_adapter ground_plan.py
```

脚本等待冻结地图最多30秒，以虚拟起点 `(0,0,0.5)` 和目标
`(3,3,0.5)` 进行一次规划，仅发布锁存的 `/UAV0/rrt/dry_run_path`。
不订阅实际无人机位置，不启动控制器、不发布PositionCommand、不调用飞控服务。
保持终端运行即可在栅格地图上看到红色路径，Ctrl+C退出。
地图与二维多项式结果分别保存为 `~/.ros/rrt_ground_check/scene.json` 和
`trajectory_xy.json`，重复成功运行会覆盖文件；规划失败请查看当前终端错误，
旧的轨迹文件不能作为本次规划成功的依据。

可选参数示例（保持相同高度切片）：

```bash
rosrun rrt_minco_adapter ground_plan.py _start:="[0,0]" _goal:="[3,3]" _height:=0.5
```

## 起飞后自动完成点到点任务

`point_to_point_mission.launch` 包含点云地图、RRT 和独立任务脚本，替代下面的
`real_cloud.launch`；不要同时启动两者。完整流程为：

1. 在 FAST-LIO 坐标 `(0.5,0.5)` 处准备起飞，先扫描并冻结静态地图。
2. 遥控器解锁、切入 OFFBOARD；现有简单控制器保持起始 XY 并爬升到 Z=0.5m。
3. 实际位置在 `(0.5,0.5,0.5)` 附近连续稳定2秒后，任务脚本自动发送目标
   `(4.5,4.5,0.5)`。无需 RViz 点击。
4. RRT＋MINCO 按地图规划并发布轨迹，现有控制器执行。
5. 轨迹执行时间结束，且实际位置在目标附近连续稳定3秒后，脚本仅请求一次
   `/UAV0/mavros/set_mode` 的 `AUTO.LAND`，进行普通就地降落。
6. 收到实际 `AUTO.LAND` 模式后进入 LANDING；同时收到落地和解除武装状态后才报告 DONE。

稳定判定：XY距离≤0.1m、Z误差≤0.02m、三维速度≤0.1m/s；里程计需新鲜且坐标系匹配。
偏离条件会重新累计稳定时间。里程计有效期1秒，飞控状态有效期3秒；
任务同时检查 MAVROS 已连接、已解锁且为 OFFBOARD。
任务脚本不解锁、不自动进入 OFFBOARD，也不触发 ArUco 搜索或精确降落流程。

保留已有 FAST-LIO、MAVROS 和控制输出仲裁链路，停用 FUEL/Diff 及其规划指令发布者后：

```bash
source ~/match_ws/devel/setup.bash
# 终端1：已有简单控制器需按这些参数启动，不重复启动同用途控制器。
roslaunch exploration_control simple_controller.launch \
  offboard_takeoff_height:=0.5 planner_enable_height:=0.45
# 终端2：任务入口，自带实机点云和规划节点。
roslaunch rrt_minco_adapter point_to_point_mission.launch
# 完整扫描后，终端3冻结地图，再通过遥控器解锁并切入OFFBOARD。
rosservice call /UAV0/rrt/freeze_map "{}"
rostopic echo /UAV0/rrt/mission_status
```

自动任务使用 `/UAV0/rrt/mission_goal`，不接收 RViz 目标，避免任务期间目标被点选覆盖。
其余点云、里程计、路径和控制指令话题与 `real_cloud.launch` 相同。
任务状态为 WAIT_START → PLANNING → FLYING → LAND_REQUESTED → LANDING → DONE。
规划失败或降落服务拒绝会报告 FAILED，不自动重试；规划/飞行阶段退出 OFFBOARD、
解除武装或飞控状态过期时报告 CANCELLED，不再自动请求降落。
LAND_REQUESTED 只表示已发起请求，不能视为实际进入降落；服务接受后仍等待实际模式反馈。
RRT节点在轨迹结束后仍发布终点保持指令，由PX4 AUTO.LAND模式执行降落。
任务节点不负责撤回已有轨迹或切换到其他控制源；人工接管后若要重新测试，
应重启整套测试入口和控制器并重新建图冻结，不要直接切回OFFBOARD复用旧轨迹。

默认稳定和悬停时间可用 `start_stable_seconds`、`goal_hover_seconds` 调整。
这里所有XYZ仍是FAST-LIO地图坐标；地图初始原点若就在机身位置，起点通常为 `(0,0)`，
需确认实际坐标满足 `(0.5,0.5)`，不能通过伪改里程计实现。

完整任务隔离验证（真实RRT规划，模拟里程计跟踪和MAVROS服务，无实机控制）：

```bash
python3 ~/match_ws/src/rrt_minco_adapter/tests/mission_smoke.py
```

## 实机点云静态地图入口

新增 `real_cloud.launch`，默认配置：

| 项目 | 配置 |
|---|---|
| 障碍点高度筛选 | Z=0.3～0.7m，包含边界 |
| 固定飞行平面 | Z=0.5m；覆盖目标消息中的 Z，适配 RViz 二维点选 |
| 已扫描规划区域 | X=0～5m、Y=0～5m |
| 栅格分辨率 | 0.1m；占据格合并为矩形，不重复做障碍膨胀 |
| 点云输入 | `/UAV0/fast_lio/cloud_registered`，PointCloud2 |
| 里程计输入 | `/UAV0/fast_lio/Odom_high_freq`，Odometry |
| 目标输入 | `/UAV0/move_base_simple/goal`，PoseStamped |
| 静态地图中间话题 | `/UAV0/rrt/scene`，锁存的 String JSON |
| 控制器指令输出 | `/UAV0/planning/pos_cmd`，PositionCommand |
| 规划结果 / RViz | `/UAV0/rrt/polynomial`、`/UAV0/rrt/path` |
| 坐标系 | `UAV0/camera_init`；不做 TF 转换 |
| 参考速度 | 0.2m/s，与现有简单控制器默认水平限速对应 |

所有 XYZ 数值是 FAST-LIO 地图坐标，不是测距仪离地高度。需确认初始化原点后使用。
该入口只是静态已扫描区域转换，不建立自由/未知体素地图：bounds 内未收到障碍点的位置
会按自由空间处理。必须先完整扫描并确认该区域，不能把遮挡未扫描区域当成已知自由空间。
只保留指定高度内的障碍点，边界外障碍、地面和高度区间外障碍不参与本次二维规划。

启动前停用 FUEL/Diff 轨迹发布节点、规划指令仲裁器及其他向
`/UAV0/planning/pos_cmd` 发布的节点；本入口不启动或停止现有控制器、定位和 MAVROS。
不要与 `run_swarm_indoor1_fuel_exploration.launch` 并行作为控制源。

```bash
source ~/match_ws/devel/setup.bash
roslaunch rrt_minco_adapter real_cloud.launch
```

在无控制输出的观察阶段，可加 `command_topic:=/UAV0/rrt/pos_cmd`。
点云累计到稀疏二维栅格中，未冻结前目标请求会返回 `order=0`。
完整扫描所选静态区域后，冻结地图：

```bash
rosservice call /UAV0/rrt/freeze_map "{}"
```

冻结成功后发布静态场景，后续点云不会修改该地图；重建需重启此测试入口。
地图只保存在节点内存和锁存话题中，不自动写入 Git 工作区。
规划器和地图节点需一起重启，避免规划器保留旧地图。

现有简单控制器默认悬停0.6m、接管门限0.5m。独立测试采用0.5m飞行时，
启动该控制器应传入以下已有参数（不要再启动第二个同用途控制器）：

```bash
roslaunch exploration_control simple_controller.launch \
  offboard_takeoff_height:=0.5 planner_enable_height:=0.45
```

该控制器仍通过原来的 `/UAV0/control/position_setpoint` 和降落仲裁链路输出，
以上命令不代替已有 MAVROS、定位、降落指令仲裁等启动流程，也不解锁或切换飞行模式。
开始规划前应在Z=0.5m附近稳定悬停；继承原有高度误差0.02m、速度0.15m/s限制。

目标示例（实际需选择已确认无障碍的位置）：

```bash
rostopic pub -1 /UAV0/move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {stamp: now, frame_id: 'UAV0/camera_init'}, pose: {position: {x: 2.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"
rostopic info /UAV0/planning/pos_cmd
rostopic echo -n 1 /UAV0/rrt/polynomial
```

默认障碍净空0.2m加跟踪余量0.1m，合计0.3m，因此起终点需要严格位于 `(0.3,4.7)×(0.3,4.7)` 内，
同时避开障碍膨胀区。如果 FAST-LIO 起点为 `(0,0)`，它不满足当前区域要求，
需要选择区域内起点，或根据实际已扫描范围显式调整 `xmin/xmax/ymin/ymax`，不能伪改里程计。

点云链路的隔离验证命令：

```bash
python3 ~/match_ws/src/rrt_minco_adapter/tests/ros_smoke.py --cloud
```

## 运行

依赖 ROS1、当前工作区的 `quadrotor_msgs`、numpy、scipy、threadpoolctl。
不要 source SUPER 的消息包。缺少 threadpoolctl 时可执行：

```bash
python3 -m pip install --user 'threadpoolctl==3.5.0'
source ~/match_ws/devel/setup.bash
roslaunch rrt_minco_adapter standalone.launch scene_file:=/绝对路径/实际场景.json
```

`scene_file` 必填；`config/example_scene.json` 是原包测试地图，不是实际场地。
JSON 格式为 `{"bounds":[xmin,xmax,ymin,ymax],"boxes":[[cx,cy,sx,sy],...]}`，
单位米，必须与里程计、目标点在同一坐标系。矩形障碍视为覆盖飞行高度的障碍。
此 `standalone.launch` 文件地图模式不订阅雷达点云，不处理未知空间、动态障碍或连续在线重规划。

默认输入输出：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/UAV0/fast_lio/Odom_high_freq` | `nav_msgs/Odometry` | 起点和初始航向 |
| `/UAV0/rrt/goal` | `geometry_msgs/PoseStamped` | 独立目标点，不消费比赛开始/探索触发话题 |
| `/UAV0/rrt/polynomial` | `std_msgs/String` | 多项式 JSON，失败时 `order=0` 及 error |
| `/UAV0/rrt/path` | `nav_msgs/Path` | 锁存的 RViz 路径 |
| `/UAV0/rrt/pos_cmd` | 工作区版 `quadrotor_msgs/PositionCommand` | 50Hz 采样位置、速度、加速度、jerk |

通过 launch 参数 `odom_topic`、`goal_topic`、`polynomial_topic`、`path_topic`、
`command_topic` 调整话题。默认 `frame_id=UAV0/camera_init`；脚本校验输入 frame，
不做 TF 变换，不能只改 frame 标签来代替坐标转换。

继承原适配的限制：速度不超过 0.15m/s，目标与起点高度差不超过 0.02m；
里程计时间戳需在最近 1 秒内。规划中收到新请求返回 busy，不排队。
规划完成时开始采样，新成功轨迹替换上一条；拒绝/失败请求不替换上一条轨迹。
高度固定为目标 Z，航向固定为请求时的机体航向，目标姿态不参与规划。
轨迹结束后继续输出终点零速度指令，停止节点后停止输出。
JSON 的 `start_ns` 是请求标识；`execution_start_ns` 是采样开始的 ROS 时间，
各段 `coef_x/y/z` 为真实局部秒变量的降幂系数。

默认速度 0.6m/s、加速度 0.8m/s²、净空 0.2m、额外余量 0.1m，可用
`speed`、`acceleration`、`clearance`、`tracking_margin` 参数调整。

## 与现有系统的边界

目前独立输出，不接管 `/UAV0/planning/pos_cmd`，不改动
`run_swarm_indoor1_fuel_exploration.launch`。原任务入口中的 FUEL/Diff 仲裁、
探索目标反馈、动态绕障和升降任务不由本包实现。
不要将输出直接 remap 到已有发布者的控制话题而造成多源命令混发。
真正切换控制源需要在独立启动组合中明确唯一指令来源，并处理现有仲裁的任务所有权；
本次未修改该流程。独立输出的时间轴会自行推进，不会等待控制器接受轨迹。

## 验证

```bash
python3 -m unittest discover -s ~/match_ws/src/rrt_minco_adapter/tests -v
source ~/match_ws/devel/setup.bash
python3 ~/match_ws/src/rrt_minco_adapter/tests/ros_smoke.py
```

ROS 测试自动建立独立 master，使用合成地图/里程计，不连接默认 master，
验证输入拒绝、多项式与指令的一致性、终点保持和输出话题隔离，退出时停止测试进程。
