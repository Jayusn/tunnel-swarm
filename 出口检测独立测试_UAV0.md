# UAV0 出口检测独立测试

只测试“通道向双侧开阔区域过渡”的识别，不需要起飞、入口识别、完整探索或飞控。
建图直接使用正式 FUEL 的 `SDFMap/MapROS` 实现，并与正式 `algorithm.xml` 共用
`exploration_manager/config/fuel_map.yaml`，不再维护另一套测试建图参数。
地图尺寸、搜索边界、射线距离和静态占据开关的默认值与
`run_swarm_indoor1_fuel_exploration.launch` 一致；出口几何判定和连续地图确认也复用正式逻辑。
测试入口启动 `exit_detection_node` 和 RViz，不会生成探索航点、穿越轨迹或降落请求。
所有检测与任务接口放在 `/exit_detection_test` 下；正式比赛 launch 的出口开关保持原设置。

数据链路为：FAST-LIO 注册点云与里程计 → 正式 FUEL 建图实现 → 出口检测。
测试进程里的建图与检测共用同一个 `SDFMap` 对象；正式流程内部也已采用该接法。
独立测试不会连接另一个正在运行的探索进程中的地图，因此累计地图从本次测试启动开始建立。
无需额外启动完整探索 launch。以后调整共同建图参数，修改上述 YAML 即同时影响两种入口。

## 手持设备测试

每个终端先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/match_ws/devel/setup.bash
```

终端 1 启动 MID360（若已启动则不用重复运行）：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch vehicle_ns:=UAV0 \
  msg_frame_id:=UAV0/livox_frame publish_freq:=30.0
```

终端 2 启动 FAST-LIO：

```bash
roslaunch fast_lio mapping_mid360.launch vehicle_ns:=UAV0 rviz:=false
```

全程检查误检时，将设备放在通道入口内侧（已经经过入口）、机头朝向第一段通道，
等待 FAST-LIO 定位稳定，再在终端 3 启动检测。当前入口不会判断“从门外进入门内”，
不要在入口外启动检测后才进入；若已经这样启动，进入第一段通道后重启检测 launch。
只检查出口时，也可以从出口前的最后一段通道内开始：

```bash
roslaunch exploration_manager exit_detection_test.launch
```

第一帧里程计的 yaw 定义初始通道前向；单纯转动机头不会重定义通道方向。
独立入口复用正式流程的地图转弯判定，地图确认转弯后切换到新通道方向。
手持模式会等实测机头朝向与地图分支一致后再采纳转弯，避免提前锁住一条未走入的支路。
正式飞行模式仍由地图先确认转弯，再由控制器完成转向；这个手持处理只用于独立入口。
本入口从通道内直接开始，不等待正式入口流程的前 2 m 直行阶段。
转弯过程中手持设备跟着新通道转向，机头对准新方向后解除转向等待；沿新方向走出
既有约 0.6 m 过渡区后恢复出口检测。没有地图转弯证据时，不会只因机头旋转而更新方向。

可以从入口内侧慢慢走完整条通道，保持设备约 0.6 m 高，机头跟随当前通道方向。
在每个拐角、单侧开口和障碍旁稍作停留，扫描周围后继续前进；关注真实出口之前
是否出现 `exit=1` 或 `confirmed.data=true`，出现即记录为疑似误检。
`open_space_hits>0` 只表示候选证据，不等于已经确认出口；同样可以记录用于排查。
到达真实出口内侧时保持设备高度、停留几秒，扫描门外左右区域，确认距离候选出口不超过 1 m。
先观察候选与确认结果，再按需走到出口外侧；无需穿出出口。
再次完整走一遍或改换起始方向时，重启检测 launch 清空地图与检测状态。

终端 4 查看状态：

```bash
rostopic echo /exit_detection_test/status
rostopic echo /exit_detection_test/confirmed
rostopic echo /exit_detection_test/detected_exit
```

`open_space_hits` 为连续有效地图证据数；`exit=1` 和 `confirmed.data=true` 表示已确认。
`detected_exit` 在当前确认有效时发布出口中心和朝外方向，不永久锁存。
状态阶段始终为 `SEARCH_CORRIDOR`，`landing=0`；它不会执行接近、穿越或降落。
候选失效时 `confirmed` 回到 false，旧 `detected_exit` 消息不能代表当前仍已确认。

完整走通道时建议在另一终端录下输入和检测结果，便于回看误检位置：

```bash
rosbag record -O ~/exit_detection_walk.bag \
  /UAV0/fast_lio/Odom_high_freq /UAV0/fast_lio/cloud_registered \
  /exit_detection_test/status /exit_detection_test/confirmed \
  /exit_detection_test/detected_exit /exit_detection_test/markers /rosout
```

若已经绕过拐角，但日志没有 `confirmed LEFT/RIGHT turn` 且出口检测迟迟无结果，
先检查地图是否充分观察到了前墙和新通道；这属于转弯未确认，不能视为出口检测已通过。

## RViz

检测 launch 默认自动打开专用 RViz，已经配置累计占据地图和出口标记，无需手动添加显示项。
出口位置用矩形、中心圆盘、方向箭头和文字一起显示：

| RViz 显示 | 含义 |
| --- | --- |
| 橙色矩形、中心圆盘，文字 `OPEN-SPACE CANDIDATE / INSIDE N/3` | 出口候选，显示证据次数，尚未确认出口 |
| 绿色矩形、中心圆盘，文字 `OPEN-SPACE READY / STILL INSIDE` | 已确认出口；对应状态 `exit=1`、`confirmed.data=true` |
| 箭头 | 推断的出通道方向 |

`STILL INSIDE` 表示独立模式只检测、不执行穿越，不表示出口未确认。
小绿色点是手持设备经过的位置，不能当作出口；确认应看绿色矩形及上方文字。
矩形标记表示开阔区域过渡截面，不代表识别到了实体门框。
候选或确认失效时标记会删除。通道拐角出现绿色出口矩形，就记录为疑似误检。

已在运行检测但没有 RViz 时，可以另开终端加载同一份显示配置：

```bash
rosrun rviz rviz \
  -d $(rospack find exploration_manager)/config/exit_detection_test.rviz \
  -f UAV0/camera_init
```

如果使用其他输入世界坐标系，最后的 `-f` 参数改为检测 launch 的 `world_frame`。
自动打开时会跟随 launch 的 `world_frame` 设置。若看不到数据，检查左侧两个显示项
是否启用、是否有坐标系错误，以及 FAST-LIO 里程计和点云是否正常更新。

无图形界面或只录包时可关闭 RViz：

```bash
roslaunch exploration_manager exit_detection_test.launch rviz:=false
```

## 回放已注册点云与里程计

录包需包含同一坐标系的 FAST-LIO 注册点云和里程计；无需重新运行 FAST-LIO。
先启动检测，再播放，确保第一帧机头朝向录包起点所在的第一段通道：

```bash
roslaunch exploration_manager exit_detection_test.launch use_sim_time:=true
```

另一个终端：

```bash
rosbag play --clock your_recording.bag --topics \
  /UAV0/fast_lio/Odom_high_freq /UAV0/fast_lio/cloud_registered
```

若录包只有 `/UAV0/fast_lio/Odometry`，启动时加
`odom_topic:=/UAV0/fast_lio/Odometry`，并相应修改回放话题。
原始 `/livox/lidar` 不能直接作为输入，需要先通过 FAST-LIO 生成注册点云与里程计。
每次重新回放应重启检测节点，清空累计地图和首帧方向。

## 输入与未确认原因

无命名空间的话题可以这样启动：

```bash
roslaunch exploration_manager exit_detection_test.launch \
  odom_topic:=/Odometry cloud_topic:=/cloud_registered world_frame:=camera_init
```

`world_frame` 必须与输入注册点云的世界坐标系一致。本入口不转换机体系点云。
地图默认世界 XY 范围为 ±10 m，Z 为 [-1, 2) m；可通过 `map_size_x/y/z` 放大范围。
正式搜索边界也已对齐：X 为 [-1, 10] m，Y 为 [-3, 10] m，Z 为 [0.05, 2] m。
这组边界使用 FAST-LIO 世界坐标；若正式流程使用了其他覆盖值，测试时也传入相同值：

```bash
roslaunch exploration_manager exit_detection_test.launch \
  map_size_x:=20.0 map_size_y:=20.0 map_size_z:=3.0 \
  box_min_x:=-1.0 box_min_y:=-3.0 box_min_z:=0.05 \
  box_max_x:=10.0 box_max_y:=10.0 box_max_z:=2.0 \
  raycast_max_dist:=8.0 retain_global_static_cloud:=false
```

`retain_global_static_cloud` 与正式入口同义，默认关闭；若正式测试开启，也在这里设为 `true`。
检测高度取实时里程计 Z，可按正式测试高度约 0.6 m 手持观察。

检测要求来路有连续双侧墙体，门外两侧相对旧墙位置都有已观测的扩展空间。
沿已确认的通道前进方向，搜索前方最多 2 m 内的候选出口截面；
在候选出口外侧连续检查 1.2 m；探测点优先使用通道中心延长线，也保留偏心出口的横向适配，要求这些前方探测点的
**左侧已观测净空 + 右侧已观测净空严格大于 2 m**，净宽扇区通过率至少 80%。
这是无人机前方出口外侧的宽度，当前位置的左右仍小于 1.5 m 也可以提前发现出口。
左右无需各大于 1 m，例如左 0.6 m、右 1.5 m 可以满足净宽条件；合计恰好 2 m 不通过。
参数为 `mission/task_search/exit/min_open_width: 2.0`，独立测试与两套正式 FUEL 入口一致。
出口位置仍由门内通道中心确定，允许左右开阔空间不对称；偏移探测点只能使用与通道中心连续相通的已观测净空。
当前位置到出口及门外通过段的完整机体范围仍须已观测为空闲。
出口方向允许在已确认通道方向附近校正最多 15°，不会只按实时机头方向重定义出口。
确认仍须 3 次新地图证据稳定，证据间隔至少 0.5 s；边界上的 15°方向校正不会丢弃已有效的证据。
未知区域不算空闲，因此只看到前方一条空线、单侧开口或未扫到门外时不会确认。
识别到候选时，终端的 `ahead_width` 是前方门外已观测净宽的保守下界，
`width_ratio` 是左右净空之和超过阈值的扇区通过率；最终确认仍看 RViz 绿色出口矩形。

检测终端会打印未通过的具体阶段：

| 日志内容 | 含义 |
| --- | --- |
| `inside_walls_missing` | 当前检测截面的来路缺少足够双侧墙体证据；走到出口外较远后也会出现 |
| `bilateral_widening_unobserved_or_blocked` | 门外两侧还没有证明相对来路墙体都向外扩展 |
| `body_path_unobserved_or_blocked` | 当前位置至出口及门外段的机体通过空间有占据或未知 |
| `outside_clearance_unobserved_or_blocked` | 前方出口外侧的左右已观测净空之和未超过 2 m，或扇区有占据/未知而未通过 |
| `turn_transition` / `yaw_alignment` | 还在过弯或等待朝向对准，暂不累积出口证据 |

录包加上 `/rosout` 可保留这些原因；最终是否确认仍看 `confirmed` 和出口矩形。

没有输出时先检查输入频率：

```bash
rostopic hz /UAV0/fast_lio/Odom_high_freq
rostopic hz /UAV0/fast_lio/cloud_registered
```

按 `Ctrl+C` 关闭检测即可；本流程不需要 MAVROS、位姿回传或飞行控制节点。
