# FUEL 手填 XYZ 多点飞行

入口：`roslaunch diff_planner run_exp_single_fuel_waypoints.launch`。
与 `run_swarm_indoor1_fuel_exploration.launch` 二选一，不能同时启动。
原探索 launch 不变；本入口由人工列表选点，FUEL A* 和 B 样条负责航点间避障。

## 五个示例点

同目录 `fuel_waypoints.yaml` 支持如下格式。下面仅展示五点格式，实际飞行点以当前 YAML 为准；
坐标应按实际场地填写，示例不代表场地核验结果：

```yaml
mission:
  manual_path:
    arrive_distance: 0.25
    waypoints:
      - [1.0, 0.0, 0.8]
      - [1.0, 2.0, 0.8]
      - [3.0, 2.0, 0.8]
      - [3.0, 4.0, 0.8]
      - [5.0, 4.0, 0.8]
```

数量没有固定上限，可以继续按相同格式追加。坐标是
`/UAV0/fast_lio/Odom_high_freq` 所在世界系（通常为 `UAV0/camera_init`）的绝对 XYZ，
单位米，z 不是相对当前高度。实际里程计进入 `arrive_distance` 三维半径后才推进列表；
预测起点到点或规划失败都不算完成。修改配置或重新飞行，需要重启规划器。

## 入口、出口和动态避障开关

在 `run_exp_single_fuel_waypoints.launch` 顶部直接改 `true` / `false`，也可以通过
`roslaunch` 参数覆盖。七分屏航点脚本默认尊重 launch 文件顶部的设置；显式设置 FAPP
环境变量，或使用 `--dynamic-mode`、`--detection-only`、`--dynamic-avoidance`、
`--planner-priority`、`--strict-dynamic` 时，才覆盖对应的 FAPP 参数。自动探索 launch 的启动参数
和默认行为不变。

| 开关 | true | false |
| --- | --- | --- |
| `enable_entry_recognition` | RViz点击后沿用现有找门、接近入口流程，到门平面附近才触发航点 | 不启动入口识别，RViz点击直接开始航点 |
| `enable_exit_recognition` | 识别并锁存出口，完成最近门内点后转向出口，再接搜板降落 | 不识别出口、不自动进入降落；飞完全部点后结束路线 |
| `enable_fapp_detection` | 启动FAPP运动目标检测节点，发布动态目标状态 | 不运行FAPP检测节点 |
| `enable_fapp_dynamic_avoidance` | FUEL订阅FAPP状态，并将移动目标用于动态避障 | FUEL忽略FAPP动态目标；检测节点仍可单独运行供查看 |
| `fapp_planner_priority` | 保留动态障碍软优化代价，同时优先生成可行轨迹 | 对动态碰撞启用硬拒绝/检查，可能因此暂缓或拒绝规划 |

这三个 FAPP 开关默认均为 `true`，但控制的是链路里的不同环节。检测流程是：点云和里程计进入
FAPP检测节点，检测节点把确认的动态目标发布到 `/UAV0/fapp/states`；开启动态避障后，FUEL
读取目标的位置、速度和尺寸，在航点之间的局部路径优化和轨迹检查中考虑这些目标。静态地图、
A* 和原有静态碰撞检查仍由 FUEL 原流程负责。关闭检测时，即使动态避障开关为 `true`，也没有
这条检测状态输入，因而不会得到有效的 FAPP 动态障碍物避让。

`fapp_planner_priority` 不是检测开关，也不是动态避障总开关：`true` 会继续把动态障碍代价交给
优化器，但常规候选轨迹和执行期轨迹检查会跳过 FAPP 动态硬拒绝；接力点/停留空间的动态安全
检查仍保留。如果软代价未能绕开预测区域，轨迹仍可能与动态障碍重叠。静态地图和机体碰撞检查
仍然生效。`false` 会按动态碰撞硬检查处理，安全
约束更严格，但障碍堵住通道或检测不稳定时可能出现规划失败。检测开启、动态避障关闭适合只观察检测输出；要完全关闭 FAPP，在 launch 顶部
同时把 `enable_fapp_detection` 和 `enable_fapp_dynamic_avoidance` 改为 `false`。默认组合是三项均为
`true`，即检测、FUEL动态避障和规划优先都开启。

### 摆球检测断续时的处理

航点入口默认开启 `fapp_memory_enabled=true`。规划器按空间邻近关系更新障碍区域，
不依赖目标 ID；只需避开该区域，不区分重新检测到的是不是同一个球。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `fapp_memory_enabled` | `true` | 按空间位置保留历史摆动包络，用于连续下穿或侧绕 |
| `fapp_observation_retention` | `0.8` 秒 | 新鲜状态用于短时运动预测的有效期，超过此时间不会自动刹停 |
| `fapp_history_duration` | `10.0` 秒 | 选路前收集历史观测的时间窗；选中区域的包络保留到实际通过 |
| `fapp_association_distance` | `0.6` 米 | 新观测到旧位置或短时预测位置的最大关联距离 |

历史包络包含球各次实际观测的尺寸、位置，以及机体尺寸和动态净空。
新观测到更低的位置会降低包络底部，球再次升高或检测为空不会抬高已记录的最低点。
这里保留的是**已观测范围**，并非从短时间观测证明了摆球未来的完整摆幅。

动态下穿使用“向侧下方平滑前进 → 左下/右下通过 → 平滑回升并回到航线”的三维引导路径，
优先检查已有绕行侧（没有偏好时先左后右），每侧从 0.20 米起按 0.10 米
（或更大的地图分辨率）递增偏移，直到现有 `wide_side_bypass/lateral_range` 上限。
一侧不可通时仍检查另一侧；选定下穿曲线后保持该侧和偏移。
横移从接近段开始平滑展开，下降仍在障碍膨胀包络前约 0.50 米开始，
每个候选保留整机已知自由空间、静态路径和动态包络检查。
复用正常的 B 样条规划与重规划。斜下穿路径受阻或接近距离不足时，
先检查固定 XY 的垂直下降，再从低位续接，不必等到距包络不足 0.10 米。
下降整根机体空间必须已知自由，
且不能穿入动态包络。发布前按实测 XY 重新生成并检查，不能从预测位置斜切下来。
高度随前向进度采用五次平滑插值，进入包络前完成下降，离开后再恢复原高度；
动态穿越（侧下方弧线及已选动态侧绕）豁免 1.20 米局部长度上限，保留本次穿越路径，
不因长度检查或短前缀重试截断在障碍区域内；重规划仍保留起点速度与加速度。
人工航点位于历史包络内时，允许先低位到该航点，再按原列表续接下一段；
只有实际位置通过整个包络才释放历史，不因为到达中间航点提前回升。
穿越结束后，普通航点立即恢复原有长度上限。
生成样条时检查历史包络，防止优化后切入球的范围；执行期沿用现有碰撞检查。

持续空帧或状态话题停发时，保留历史约束继续选路，不再仅因超过 0.8 秒而制动等待。
下穿需要满足现有最低飞行高度和已知自由空间要求；无法下穿时沿用已有侧绕选择。
新观测扩大历史包络时，后续重规划使用更新后的范围。
只有真实里程计确认通过才释放记录，预测起点不能提前释放；连续下穿还保留到低位穿越段末端。

动态低位穿越依赖空间记忆，自动探索共用下穿及近距离垂直下降逻辑。
`fapp_planner_priority` 的默认值不变；已确认的动态区域也约束航点普通接近段，
下穿失败后不能回退成穿过该区域的普通前进轨迹。
执行中的剩余样条被动态区域挡住时，立即刹停并从实测位置重规划，不等整段接近轨迹结束；
可通的下穿或侧绕继续执行，不因机头前方直线上有障碍就反复取消。
已选动态空隙穿越仍沿用新鲜观测和到达时间检查。
检测器的背景过滤和运动确认阈值未改，不能把规划继续运行等同于即时检测恢复。

航点普通规划失败时，也从实测位置尝试已有的膨胀层局部脱障：检查前后及侧方已知空地，
允许先走净空改善的小步，执行后再规划原航点。沿途原始机体接触不得增加，
离开膨胀层后不能再次进入；脱障不消耗或改写航点，未触发任务时不启动。

七分屏入口默认保留轻量规划录包，关闭双机建图点云、地图/规划可视化和独立 ArUco 录包，
相机识别、实时建图和控制照常运行。需要录制点云以离线重建地图时，使用
`UAV0_PLANNER_ROSBAG_CLOUDS=true bash ~/match_ws/shfiles/start_uav0_try.sh`；
需要独立 ArUco 判定数据时设置 `UAV0_ARUCO_ROSBAG_ENABLED=true`。
其他话题可以通过 `UAV0_PLANNER_ROSBAG_EXTRA_TOPICS` 补录。
默认轻量包可排查轨迹、里程计和动态状态，但不能单独重建完整地图。

开启原有录包开关后，保留：

- `/UAV0/fapp/states`：检测器原始输出。
- `/exploration_node/fapp/memory`：近期障碍，以及穿越中的历史区域；每个状态保留原观测时间。
- `/exploration_node/fapp/memory_status`：观测年龄、位置、`retained` / `historical` 状态和历史包络上下界。
- `/rosout_agg` 中的 `[manual_fapp]`、`[side_recovery]`、`[vertical_detour]`：
  历史区域选取、连续下穿、通过和绕行阶段切换原因。

`memory` 是诊断话题，包含已经超过新鲜预测有效期的历史区域，不能当作检测器的新鲜状态重新输入规划器。
RViz 原检测框仍表示检测器的即时输出，可能闪烁；可用上述话题区分“检测没输出”与“规划器已遗忘”。

入口沿用原方案C及交接阈值 `door_cross_trigger_dist=-0.08`：入口参考完成、满足横向条件，
实际位置到门前约8cm范围即可交给FUEL。入口阶段不会提前消耗航点。
入口关闭、出口开启时，默认从通道内开始；以任务启动位置和机头朝向建立出口检测参考。

出口启用后的具体顺序：

1. 保留现有地图出口检测和确认条件，不用任意单帧结果接管。
2. 确认时锁存出口中心及向外法向；用法向划分门内/门外，在**完整配置列表的门内点**中，
   按到出口中心的三维距离选择最近点。距离相同取原列表中较早的一项。
3. 保留原顺序飞到这个最近点，同时跳过尚未执行的门外点；最近点之后的配置点取消。
   最近点已经飞完，或列表没有门内点时，直接转向出口，不回头补点。
4. 必须由真实里程计确认最近点完成，才让FUEL规划到锁存出口。
   到出口前若地图不允许通行，继续重试该出口目标，不恢复前沿探索。
5. 到出口截面附近并满足原巡航高度条件后，切入现有 `SEARCH_OUTSIDE_LANDING`。
   此后由现有Diff搜板降落流程接管：出口外接近、前视/下视识别、精降。
   **不再执行 YAML 中的门外定点；也不是检测出口后直接原地降落。**

开出口但全部航点已完成、仍未确认出口时，等待出口检测，不自动追加探索点或盲目降落。
识别只影响确认时尚未执行的航点；已飞过的路段不能撤销。

## 启动

完整双机网络和七分屏入口：

```bash
bash ~/match_ws/shfiles/start_uav0_twaypoints.sh
```

入口、出口和 FAPP 开关直接修改 `run_exp_single_fuel_waypoints.launch` 顶部的 `true` / `false`，
七分屏脚本默认使用这里的设置。例如仅关闭入口：

```xml
<arg name="enable_entry_recognition" default="false"/>
<arg name="enable_exit_recognition" default="true"/>
```

新脚本沿用原网络、检测栈、控制器和录包布局，第六屏加载本规划器，第七屏启用现有
FUEL/DIFF AutoTrans阶段仲裁，保证出口后降落轨迹能够接管。关闭出口时，控制权始终属于FUEL。
原七分屏脚本不设置 `UAV0_AUTOTRANS_PURE_FUEL_MODE` 时仍保持原来的纯FUEL默认模式。

单独启动规划器（两个开关都关闭，即纯多点飞行）：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch diff_planner run_exp_single_fuel_waypoints.launch \
  enable_entry_recognition:=false enable_exit_recognition:=false
```

两者都开启时无需额外参数。单独使用AutoTrans控制器并开启出口时，应启动混合桥接：

```bash
roslaunch autotrans_reference_bridge uav0_autotrans_controller.launch \
  pure_fuel_mode:=false precision_landing_mission_mode:=true
```

单独启动规划器不会启动相机、识别栈、精降控制器或自动起飞；搜板降落依赖七分屏原有
检测/精降链路。不要重复启动控制器或同时运行两套规划器。

启动后沿用现有起飞流程，等待里程计、地图和控制器就绪，再在 RViz 点击 `2D Nav Goal`。
点击位置不会覆盖 YAML 航点；入口开启时只触发找门，入口关闭时触发执行列表。
不使用RViz时，同样可向 `/UAV0/move_base_simple/goal` 发布一次 `geometry_msgs/PoseStamped`。
仅在入口关闭时，`/UAV0/fuel_waypoints/start` 也可直接开始列表；入口开启时它不连接航点执行器。
`/UAV0/fuel_waypoints/entry_complete` 为入口管理器内部交接信号，不作为人工开始入口。

自定义文件及速度：

```bash
roslaunch diff_planner run_exp_single_fuel_waypoints.launch \
  waypoints_file:=/绝对路径/我的航点.yaml fuel_max_vel:=0.4 fuel_max_acc:=0.5
```

## 在 RViz 查看 FAPP 检测

航点入口加载同目录的 `fuel_waypoints.rviz`，默认展开并勾选 `FAPP_Dynamic`：

| 显示项 | 话题 | 画面含义 |
| --- | --- | --- |
| `dynamic_points` | `/UAV0/fapp/dynamic_cloud` | 红色动态目标点云 |
| `dynamic_bboxes` | `/UAV0/fapp/dynamic_object_markers` | 橙色目标包围框 |
| `dynamic_velocity` | `/UAV0/fapp/dynamic_track_markers` | 绿色速度方向箭头，长度固定，不代表预测轨迹或速度大小 |

Fixed Frame 保持 `world`，launch 提供到 `UAV0/camera_init` 的变换。
`enable_fapp_detection=true` 即可观察检测；显示不依赖 `enable_fapp_dynamic_avoidance`。
本配置不添加预测轨迹显示，因为当前 FAPP 没有发布相应的 Marker 话题。

已经打开的 RViz 可用 **File → Open Config** 加载此文件，无需重启规划器。
不要为切换配置直接关闭 RViz 窗口：当前 launch 将它设为 `required=true`，退出会连带停止该 launch。

确认目标前，输出可能是空点云、空状态列表和清除标记；话题有频率不等于检测到目标。
摆球暂时接近静止时，速度箭头也可能不可见。需要排查时在已加载工作区环境的终端运行：

```bash
rosnode info /UAV0/fapp/mapping_node
rostopic hz /UAV0/fapp/states
rostopic echo -n 1 /UAV0/fapp/states
```

节点存在说明已启动，状态话题持续发布说明检测链路有输出；`states` 非空才表示当帧发布了动态目标。
若三项显示均正常订阅但 `states: []`，应继续检查输入点云、里程计和目标确认，而不是只调整 RViz。

## 规划链路

- 航点段和出口接近段仍使用FUEL `algorithm.xml`、地图、A*、B样条和既有碰撞检查。
- 此 launch 默认启动刚同步的 UAV0 FAPP 运动证据检测，并将动态目标接入 FUEL 优化器及运行期
  轨迹检查；默认启用动态避障和规划优先，设置与 `start_uav0_twaypoints.sh` 一致。
  如果只想看检测输出而不让FUEL避障，可启动时设置 `enable_fapp_dynamic_avoidance:=false`；
  关闭检测节点用 `enable_fapp_detection:=false`。运动证据由
  `fapp_object_motion_evidence_enabled` 控制，双向匹配、候选拆分及候选关联隔离默认关闭。
- 航点只决定远处目的地，普通航点每次沿已知路径规划最多 **1.2米** 的局部段；还检查优化后B样条的
  控制多边形长度上界，避免短路径被平滑成超长轨迹。到局部端点后按新地图续接，
  只有真实里程计到达用户航点，才推进 YAML 列表。长度在 launch 的
  `mission/manual_path/max_segment_length` 设置。
- 默认开启 `mission/manual_path/exploration_style`：复用探索的地图转弯和绕障。
  直角弯先制动、原地对准新通道，再继续平移；真实航向连续0.15秒进入8度范围才算完成。
  弧形通道使用原有沿墙方向渐变，普通绕柱侧移不会把机头一起扭过去。出口开关关闭也保留转弯。
- 难飞的绕障段按 **侧移 → 同侧通过 → 回到航点方向** 执行，复用原有宽侧选择和侧向续接。
  每一步独立检查、规划，不把三步平滑成一条跨障长曲线；复杂障碍可能需要多个短步完成某一阶段。
- 悬挂物和 FAPP 摆球采用 **下穿优先，下方不可通再选左右侧**。静态悬挂物使用占据地图确认，
  摆球使用 FAPP 的位置、速度和尺寸；下穿高度扣除已有机体净空及高度跟踪余量，并遵守任务最低高度。
  静态悬挂物仍在下降柱和低位通过路径已知可走时，执行 **原地下降 → 低位每次约0.20米通过 → 原地回升**；
  越过障碍后还需确认上方可通，回到下穿前高度再继续原航点。下方未知或被挡时比较左右可通过路线，
  复用三段侧绕；下方与两侧都不可通时保留航点重规划。静态竖直段仍受1.2米长度上限约束。
  开启空间记忆的动态摆球使用前述左下/右下连续弧线，动态穿越路径豁免该长度上限。
  `enable_fapp_dynamic_avoidance=false` 时不使用摆球状态，静态悬挂物处理仍保留。
  `fapp_planner_priority=true` 仍保留此下穿/侧绕选路顺序，只沿用原来的常规轨迹软代价优先策略；
  FAPP 仍是现有短时匀速预测，不是摆动周期或整个摆幅的预测。
- A*需要已知自由空间。远点暂不连通时，可以先沿通向该点的已知自由短段推进；
  如果连短段也不可行，就保留航点等待重规划，不穿墙、不跳过航点，也不切回自主前沿探索。
  拐角遮挡严重时，仍建议像当前 YAML 一样在转角内侧填写中间点。
- 开出口时，FUEL位置命令先输出 `/UAV0/fuel/planning/pos_cmd`，由原任务阶段仲裁器发布
  `/UAV0/planning/pos_cmd`；关出口时直接输出原最终话题。
- `fuel_waypoint_portals.xml` 单独保存复用的入口/出口和降落节点参数；不包含原自动前沿目标选择。
- 原 `uav0_manual_waypoints.launch` 及自动探索入口不启用本航点出口延期逻辑，保持原行为。
