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
不按RRT的-1～4m规划区域裁剪显示。FUEL使用有限大小的稠密地图，并非无限范围地图。
扩大显示/建图范围可显式传参（会增加内存和运算量），例如：

```bash
roslaunch rrt_minco_adapter grid_map_view.launch map_size_z:=6.0
```

此例Z范围为-1～5m；`map_size_x/map_size_y/map_ground/raycast_max_dist`也可设置。
规划区域仍为X/Y=-1～4m，Z=0.3～0.7m仅用于规划障碍切片，路径高度0.5m。
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

地面入口、`real_cloud.launch` 和自动飞行入口均使用FUEL占据建图；
雷达点云经过FUEL处理后，由占据地图生成规划障碍。

### 第六步：直接调用地面规划脚本

地面脚本取消边界净空，允许起点位于边界，整条路径仍限制在闭区间 `[-1,4]×[-1,4]`。
真实障碍不删除，仍使用0.2m障碍净空＋0.1m跟踪余量。实飞入口同样取消虚拟边界净空，但保留真实障碍净空。

冻结地图后，在已经加载工作区和相同ROS网络环境的终端执行：

```bash
rosrun rrt_minco_adapter ground_plan.py
```

脚本等待冻结地图最多30秒，以虚拟起点 `(0.01,0.01,0.5)` 和目标
`(3,3,0.5)` 进行一次规划，仅发布锁存的 `/UAV0/rrt/dry_run_path`。
不订阅实际无人机位置，不启动控制器、不发布PositionCommand、不调用飞控服务。
保持终端运行即可在栅格地图上看到红色路径，Ctrl+C退出。
地图与二维多项式结果分别保存为 `~/.ros/rrt_ground_check/scene.json` 和
`trajectory_xy.json`，重复成功运行会覆盖文件；规划失败请查看当前终端错误，
旧的轨迹文件不能作为本次规划成功的依据。

可选参数示例（保持相同高度切片）：

```bash
rosrun rrt_minco_adapter ground_plan.py _start:="[0.01,0.01]" _goal:="[3,3]" _height:=0.5
```

## 起飞后自动完成点到点任务

`point_to_point_mission.launch` 包含点云地图、RRT 和独立任务脚本，替代下面的
`real_cloud.launch`；不要同时启动两者。完整流程为：

1. 在 FAST-LIO 地面坐标 `(0.01,0.01,0)` 处准备起飞，每次启动新建图，重新扫描并冻结静态地图。
2. 遥控器解锁、切入 OFFBOARD；AutoTrans MPC在CH8低位时保持实际起飞XY并爬升到Z=0.5m；升至0.5m后CH8拨高进入轨迹模式。
3. CH8高位且实际XY位置距 `(0.01,0.01)` ≤0.5m后立即发送目标
   `(3,3,0.5)`。无需 RViz 点击。
4. RRT＋MINCO以50Hz发布PositionCommand位置/速度/加速度/jerk参考，AutoTrans MPC执行。
5. 轨迹执行时间结束，满足终点位置和速度条件后，脚本立即请求一次
   `/mpc_controller_node/request_land`，调用与CH10一致的MPC原地降落。
6. 收到MPC降落锁存后进入 LANDING；同时收到落地和解除武装状态后才报告 DONE。

终点稳定判定：XY距离≤0.1m、Z误差≤0.05m、三维速度≤0.1m/s；里程计需新鲜且坐标系匹配。
偏离条件会重新累计稳定时间。里程计有效期1秒，飞控状态有效期3秒；
任务同时检查 MAVROS 已连接、已解锁且为 OFFBOARD。
任务脚本不解锁、不自动进入 OFFBOARD，也不触发 ArUco 搜索或精确降落流程。

保留已有FAST-LIO和MAVROS；退出简单控制器、旧MPC、输出仲裁器及其他规划指令发布者后：

```bash
source ~/match_ws/devel/setup.bash
# 终端1：MPC入口直接输出MAVROS姿态/推力，不再启动简单控制器和输出仲裁器。
roslaunch rrt_minco_adapter autotrans_controller.launch
# 先Ctrl+C退出上一轮实飞任务及grid_map_view/real_cloud建图入口。
# 终端2：每次启动新FUEL地图和任务，保持运行并重新扫描。
rosrun rrt_minco_adapter start_flight.sh
# 终端3：本轮扫描完成后冻结地图，确认success为True再解锁起飞。
rosservice call /UAV0/rrt/freeze_map "{}"
# 观察状态；通过遥控器解锁并切入OFFBOARD。
rostopic echo /UAV0/rrt/mission_status
```

实飞脚本与 `point_to_point_mission.launch` 每次启动新的FUEL地图和未冻结的场景转换节点，
不再提供复用旧地图的 `start_mapper` 参数。必须先退出上一轮任务和建图入口，
避免同名节点互相顶掉；每次重新扫描、冻结后再起飞。雷达和FAST-LIO可以保持运行。
任务脚本不启动雷达、FAST-LIO、MAVROS或控制器；autotrans_controller.launch已覆盖起飞高度为0.5m。
脚本可用 `--help` 查看说明。

自动任务使用 `/UAV0/rrt/mission_goal`，不接收 RViz 目标，避免任务期间目标被点选覆盖。
其余点云、里程计、路径和控制指令话题与 `real_cloud.launch` 相同。
任务状态为 WAIT_START → PLANNING → FLYING → LAND_REQUESTED → LANDING → DONE。
规划失败或降落服务拒绝会报告 FAILED，不自动重试；规划/飞行阶段退出 OFFBOARD、
解除武装或飞控状态过期时报告 CANCELLED，不再自动请求降落。
LAND_REQUESTED 只表示已发起请求，不能视为实际进入降落；服务接受后仍等待MPC降落锁存反馈。
RRT节点在收到降落锁存时停止终点保持指令。MPC以0.2m/s降低高度参考，再按现有近地/超时条件交给PX4 AUTO.LAND收尾。
CH10降落由AutoTrans原有逻辑识别：先低于1300，再高于1800（数组下标9）。
MPC发布 `/UAV0/mission/rc_land_active=true` 后，主动请求降落的RRT任务进入LANDING；人工CH10中断则进入CANCELLED，
规划节点清除活动轨迹并停止PositionCommand，规划线程迟到的结果也不会重新发布。
降落锁存不自动恢复，落地后重启整套任务与MPC，并重新扫描冻结地图。
仅退出OFFBOARD的人工接管仍须落地后重启，不能直接切回复用旧轨迹。

本入口CH8配置为数组下标7：低位起飞，高位CMD_CTRL。任务默认等待新鲜CH8高位，
避免控制器尚未跟踪时轨迹时钟已经开始。MPC部分日志仍写“CH8”，以本入口参数为准。
CH6是否切OFFBOARD由飞控实际映射决定，必须现场确认。CH10受RC数据有效性影响，
必须在不解锁状态确认通道值，并进行受控接管测试，不能仅凭代码接入认定实机有效。
一键入口：`bash ~/match_ws/shfiles/start_rrt_flight_all.sh`。
MPC自动logger关闭，避免额外重复录包；使用现有record_rrt_flight.sh独立录包，
已加入姿态指令、CH10锁存、IMU、RPM和MPC参考轨迹。

默认稳定和悬停时间可用 `start_stable_seconds`、`goal_hover_seconds` 调整。
这里所有XYZ仍是FAST-LIO地图坐标；地图初始原点若就在机身位置，起点通常为 `(0,0)`，
需确认实际坐标满足 `(0.01,0.01)`，不能通过伪改里程计实现。

完整任务隔离验证（真实RRT规划，模拟里程计跟踪和MAVROS服务，无实机控制）：

```bash
python3 ~/match_ws/src/rrt_minco_adapter/tests/mission_smoke.py
# 真实MPC节点＋模拟传感器，验证RRT参考接收和CH10中断
python3 ~/match_ws/src/rrt_minco_adapter/tests/mission_smoke.py --mpc-ch10
```

## 实机点云静态地图入口

新增 `real_cloud.launch`，默认配置：

| 项目 | 配置 |
|---|---|
| 障碍点高度筛选 | Z=0.3～0.7m，包含边界 |
| 固定飞行平面 | Z=0.5m；覆盖目标消息中的 Z，适配 RViz 二维点选 |
| 已扫描规划区域 | X=-1～4m、Y=-1～4m |
| 栅格分辨率 | 0.1m；占据格合并为矩形，不重复做障碍膨胀 |
| 雷达点云输入（经FUEL处理） | `/UAV0/fast_lio/cloud_registered`，PointCloud2 |
| FUEL占据地图输入 | `/UAV0/rrt/fuel/occupancy_all`，PointCloud2 |
| 建图里程计输入 | `/UAV0/fast_lio/Odometry`，Odometry |
| 规划里程计输入 | `/UAV0/fast_lio/Odom_high_freq`，Odometry |
| 目标输入 | `/UAV0/move_base_simple/goal`，PoseStamped |
| 静态地图中间话题 | `/UAV0/rrt/scene`，锁存的 String JSON |
| 控制器指令输出 | `/UAV0/planning/pos_cmd`，PositionCommand |
| 规划结果 / RViz | `/UAV0/rrt/polynomial`、`/UAV0/rrt/path` |
| 坐标系 | `UAV0/camera_init`；不做 TF 转换 |
| 参考速度 | 0.2m/s，RRT参考速度 |

所有 XYZ 数值是 FAST-LIO 地图坐标，不是测距仪离地高度。需确认初始化原点后使用。
FUEL维护占据地图，但RRT适配器只提取静态占据障碍，不使用自由/未知状态：bounds 内未收到障碍点的位置
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
FUEL占据快照投影到指定高度切片，未冻结前目标请求会返回 `order=0`。
完整扫描所选静态区域后，冻结地图：

```bash
rosservice call /UAV0/rrt/freeze_map "{}"
```

冻结成功后发布静态场景，后续点云不会修改该地图；重建需重启此测试入口。
地图只保存在节点内存和锁存话题中，不自动写入 Git 工作区。
规划器和地图节点需一起重启，避免规划器保留旧地图。

本入口使用AutoTrans MPC，起飞高度覆盖为0.5m。手动启动时执行
（使用一键脚本时不要重复启动）：

```bash
roslaunch rrt_minco_adapter autotrans_controller.launch
```

AutoTrans通过 `/UAV0/mavros/setpoint_raw/attitude` 输出姿态与推力，
原有恢复逻辑可能使用 `/UAV0/mavros/setpoint_raw/local`。不要同时启动其他控制输出源。
开始规划前需稳定悬停在0.5m，CH8高位进入轨迹模式。

目标示例（实际需选择已确认无障碍的位置）：

```bash
rostopic pub -1 /UAV0/move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {stamp: now, frame_id: 'UAV0/camera_init'}, pose: {position: {x: 2.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"
rostopic info /UAV0/planning/pos_cmd
rostopic echo -n 1 /UAV0/rrt/polynomial
```

默认障碍净空0.2m加跟踪余量0.1m，合计0.3m；虚拟边界净空为0，
规划路径仍限制在 `[-1,4]×[-1,4]`，同时避开真实障碍膨胀区。
实飞从真实里程计位置规划，不能伪改里程计。起点邻近边界时，实际坐标若越界仍会规划失败。

点云链路的隔离验证命令：

```bash
python3 ~/match_ws/src/rrt_minco_adapter/tests/fuel_map_smoke.py
python3 ~/match_ws/src/rrt_minco_adapter/tests/mission_smoke.py
# 真实MPC节点＋模拟传感器，验证RRT参考接收和CH10中断
python3 ~/match_ws/src/rrt_minco_adapter/tests/mission_smoke.py --mpc-ch10
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

规划入口不限制当前速度和起点高度差，仍校验数值有效性；
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

起点规划触发更新：CH8高位且距起点(0.01,0.01)的XY距离≤0.5m即可，无起点高度/速度门槛，无起点等待。规划器入口也不再拒绝高度差或运动状态。轨迹仍固定Z=0.5m；终点保留位置和速度判断，无额外悬停等待。

到点立即调用MPC降落服务，下降参考0.2m/s；不再由启动脚本写入MPC_LAND_SPEED。此前已写入飞控的参数不会自动恢复。解除武装或退出OFFBOARD后，降落锁存阻止MPC重新输出，需落地重启下一轮。

## SUPER 算法对比

已增加 `planner:=super` 切换入口，默认仍为 RRT。地面运行 `rosrun rrt_minco_adapter compare_planners.py` 可对同一冻结地图进行比较。构建、命令、比较口径和限制见 [SUPER对比说明](SUPER对比说明.md)。

## 固定起点占据过滤（2026-11-24）

经操作者确认，`grid_map_view.launch` 默认在地图转换时过滤固定圆柱内的输入占据点：中心XY=(0.01,0.01)，半径0.4m，Z=[0.3,0.7]m，坐标系UAV0/camera_init。RRT/SUPER共用此设置；范围不随无人机或目标点移动。该区域内的真实障碍也会被屏蔽，属于操作者声明可通行的区域，不是人体识别或自由空间观测。

过滤同时作用于转换后的`/UAV0/rrt/voxel_map`、`grid_map`和冻结`scene`，上游FAST-LIO/FUEL点云保持原样。逐点过滤在体素化之前执行：范围外的点即使与被过滤点处于同一栅格，仍保留该栅格，避免扩大清除范围。冻结后不再更新；旧节点、旧冻结地图和旧路径不会因修改源码自动改变。

一键入口下次启动自动启用。仅重启转换节点时，须显式添加`_start_clear_radius:=0.4`，沿用原话题重映射；裸节点默认0（关闭）。独立地图入口可使用`roslaunch rrt_minco_adapter grid_map_view.launch start_clear_radius:=0`关闭。不要在已运行的整套入口旁重复启动另一个建图入口。

冻结scene新增`operator_clear_region`元数据，记录被人为过滤的范围，便于录包/算法对比追溯。它不改变0.3m规划净空，也不保证任意地图的起点都通过检查；范围外障碍仍可能使规划失败。启用前应确认该固定范围内没有真实障碍。

单次地图编辑还可通过转换节点私有参数 `removed_voxels` 指定需要删除的体素索引。
索引按本次 bounds/resolution 计算，仅删除明确选中的方块；显示、规划转换以及保存的定位点云均排除这些体素。
保存场景中的 `operator_removed_voxels` 记录编辑位置。上游 FUEL 原始占据数据不被改写。
新建地图入口将此列表重置为空，避免将上次人工删除位置带入新的扫描；已保存的地图保留编辑结果。

## 断电后复用地图（2026-11-24）

一键扫描建图（仅扫描，不启动飞行任务）：

```bash
bash ~/match_ws/shfiles/start_rrt_mapping.sh
```

脚本自动加载 ROS 网络环境，按顺序启动 MID360、FAST-LIO、FUEL 和 RViz；每一步收到输入数据后才继续。
已有雷达/FAST-LIO 会复用；已有建图、旧地图加载或控制任务时提示先退出旧入口。
启动时在原起飞点附近静置、保持未解锁，看到“扫描建图已启动”后再缓慢移动补扫。
此入口无需 MAVROS、遥控通道或 OFFBOARD，不启动控制器和录包，不会自动冻结或保存。
扫描完整后按下面命令冻结、保存；保存成功后 Ctrl+C 结束本脚本启动的进程，复用节点保持运行。
仍沿用现有起点过滤与 FUEL 建图配置，不自动识别或过滤人体。

支持将本轮 FUEL 占据快照、三维体素显示、规划场景及起点清除区写入磁盘。
关程序、无人机断电不会删除保存文件。新入口不重建 FUEL；使用旧场景规划。
规划仍为固定高度 0.5m，XY 范围沿用保存时的设置（当前默认 [-1,4]×[-1,4]）。
这不是完整 FUEL 概率/自由空间状态的恢复，也不是 FAST-LIO 的全局地图加载。

首次保存：运行原来的建图入口，从原起飞点附近启动 FAST-LIO 后扫描各个遮挡方向。
人员离开后检查三维地图，确认真实障碍完整、没有人体残影；起点清除区仍会过滤真实障碍。
在已有 ROS 环境的终端执行：

```bash
source ~/match_ws/devel/setup.bash
source ~/match_ws/shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav0
rosservice call /UAV0/rrt/freeze_map "{}"
rosparam set /rrt_cloud_scene/save_path "$HOME/rrt_maps/site.json"
rosservice call /UAV0/rrt/save_map "{}"
```

仅 `save_map` 返回 `success: True` 才表示文件已保存。文件存在时拒绝覆盖，另选文件名即可。
保存操作需要新版 `cloud_to_scene_node.py`，此前已运行的旧节点需先在地面退出、重启建图。
旧的 `ground_plan.py` 输出 `scene.json` 只有规划矩形，不能当成本入口所需的完整地图文件。
保存不会修复漏扫、人体残影；保存不完整就会在以后重复加载这些缺陷。

重启后先验证：将无人机放回原起飞点附近、地面水平放稳，朝向近似一致；启动雷达、FAST-LIO、
MAVROS 后，退出旧建图/任务/控制入口，保持未解锁：

```bash
roslaunch rrt_minco_adapter saved_map_view.launch map_file:=$HOME/rrt_maps/site.json
# 另一个已加载 ROS 环境的终端，等当前点云正常后：
rosservice call /UAV0/rrt/freeze_map "{}"
rosrun rrt_minco_adapter ground_plan.py _planner:=super _frame_id:=UAV0/saved_map _output_dir:=$HOME/.ros/super_saved_map_check
```

`freeze_map` 在旧地图模式下做一次局部匹配，不是新建地图或重新冻结。
使用约最近 2 秒扫描、10cm 降采样、XYZ 平移+偏航角 ICP，假设两次 FAST-LIO 世界坐标都重力对齐。
匹配要求至少100点，20cm范围内对应点比例≥60%，这些对应点 RMSE≤10cm；不足时返回失败。
这些数值只是匹配质量初筛，不能证明定位唯一或飞行安全。长走廊、重复墙面可能出现错误匹配。
不支持任意初始位置/朝向，不做飞行中回环和持续全局校正；长距离漂移仍来自 FAST-LIO。

RViz Fixed Frame 为 `UAV0/saved_map`；添加 PointCloud2 `/UAV0/rrt/aligned_cloud`，
将当前扫描与旧三维占据图叠加核对。匹配后移动少量距离观察是否仍重合。
`/UAV0/rrt/map_odom` 是旧地图坐标下位置，`/UAV0/rrt/map_alignment` 给出本次平移、偏航和匹配分数。
不要把原始 `cloud_registered` 直接当成旧地图坐标来显示。地图里的起点和目标始终是旧地图坐标。

地面验证完成后，退出上述加载入口和地面规划节点，再启动实飞入口：

```bash
bash ~/match_ws/shfiles/start_rrt_flight_all.sh super "$HOME/rrt_maps/site.json"
# 另一个已加载 ROS 环境的终端，未解锁时重新匹配本次启动：
rosservice call /UAV0/rrt/freeze_map "{}"
```

RRT 对比时将 `super` 换为 `rrt`。不提供第二个参数仍走原来的每轮重新建图流程。
飞行前仍需现场核对匹配结果、路径、遥控接管和控制状态；本次改动未进行实飞验证。
遥控起飞/跟踪/降落流程不变，脚本不自动解锁。

坐标链路：原始 FAST-LIO → `map_odom` → 旧地图规划与任务 → `map_pos_cmd`
→ 逆变换 → `/UAV0/planning/pos_cmd` → 原 AutoTrans MPC。
MPC、MAVROS 定位桥保持原始 FAST-LIO 接口；变换包含位置、速度、加速度、jerk 和 yaw。
匹配结果在本次节点运行期间固定，不能在飞行中反复重定位；重新匹配需落地后重启。
保存文件是静态环境快照，不会自动反映后来搬动的障碍，也不会自动避让新进入的人员。

人工确认补墙面小孔时，`wall_holes.find_wall_holes` 可从一份原始体素快照中找候选：
仅补竖直平面3×3邻域中八邻格齐全的单体素孔洞（默认10cm），不迭代扩张、不补地面，排除起点半径0.4m及人工删除体素。
转换节点的 `filled_voxels` 参数接收该次确认的索引；推断体素进入显示和规划占据层，保存场景记录 `inferred_wall_voxels`，但不加入定位匹配点云。
补洞为几何推断，可能封住真实小孔，不能作为扫描完整性的证明。大缺口仍应换角度补扫。
新建扫描时清空本次补洞参数；保存地图则包含补洞结果。改变该参数后需在地面重启转换节点。


## 当前扩大区域任务（2026-11-25）

新建扫描与实飞入口默认规划边界为 X=[-2,6]、Y=[-2,6]，地面规划、对比和任务默认起点为(0,0)、终点为(4,5)，固定高度仍为0.5m。
加载保存地图时边界以文件里的 scene.bounds 为准；修改 launch 不会自动扩大旧文件。
本次将 ~/rrt_maps/site.json 用已保存的完整 FUEL 定位点云重建扩大区域的障碍，原规划区域占据格保持一致，三维显示点云和定位点云保持原数据。
人工删除与补洞索引随新地图原点重编号，原文件备份及扩大版本放在 ~/target_reports/map_resize_20261125/；这些地图生成物不纳入 Git。
点云覆盖到某坐标不代表整片区域都已观测；新增区域仍需现场检查遮挡和漏扫。
本次离线 RRT+MINCO 从(0,0)到(4,5)试算失败：MINCO corridor has no obstacle separation。
终点最近占据矩形约为 X=[3.7,4.0]、Y=[5.3,5.4]，距终点约0.30m，贴当前0.2m障碍净空+0.1m跟踪余量边界。
终点按用户要求保留，未删除该障碍或缩小净空；应核对其真实性与目标位置后再做地面规划检查。

```bash
roslaunch rrt_minco_adapter saved_map_view.launch map_file:=$HOME/rrt_maps/site.json
# 已启动雷达、FAST-LIO、MAVROS并保持未解锁，另一个ROS终端做本次定位匹配：
rosservice call /UAV0/rrt/freeze_map "{}"
rosrun rrt_minco_adapter ground_plan.py _planner:=super _frame_id:=UAV0/saved_map _start:='[0,0]' _goal:='[4,5]'
```

### 保存地图的 RViz 规划对比

```bash
source ~/match_ws/devel/setup.bash
source ~/match_ws/shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav0
roslaunch rrt_minco_adapter compare_saved_map.launch map_file:=$HOME/rrt_maps/site.json
```

直接加载已保存地图，启动 RViz 并计算 RRT+MINCO、SUPER；无需重新扫描、冻结或做本次定位匹配，所有坐标属于保存地图。
红线为 RRT+MINCO，绿线为 SUPER，绿色球为起点(0,0,0.5)，橙色球为终点(4,5,0.5)。规划失败时显示 FAILED，路线为空，不用折线冒充有效轨迹。
`compare_planners.py` 新增可选 `_visualize:=true`；`_scene_file` 兼容裸场景 JSON 和完整保存地图 JSON。
结果默认放在 ~/target_reports/rrt_super_current_map，保持启动终端运行以维持地图和路径话题；Ctrl+C退出。
该入口只启动规划对比和RViz，不发布飞行控制指令。


2026-11-25终点再次更新：当前默认终点为(3.5,5,0.5)，起点仍为(0,0,0.5)，规划范围仍为[-2,6]×[-2,6]。上述(4,5)试算失败记录属于旧终点。重新运行 compare_saved_map.launch 使用新终点。
