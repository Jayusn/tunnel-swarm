# UAV0 出口检测独立测试

只测试“通道向双侧开阔区域过渡”的识别，不需要起飞、入口识别、完整探索或飞控。
复用正式 FUEL 的累计 SDFMap、出口几何判定和连续地图确认逻辑。
测试入口启动 `exit_detection_node` 和 RViz，不会生成探索航点、穿越轨迹或降落请求。
所有检测与任务接口放在 `/exit_detection_test` 下；正式比赛 launch 的出口开关保持原设置。

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

全程检查误检时，将设备放在通道入口内侧、机头朝向第一段通道，再在终端 3 启动检测。
只检查出口时，也可以从出口前的最后一段通道内开始：

```bash
roslaunch exploration_manager exit_detection_test.launch
```

第一帧里程计的 yaw 定义初始通道前向；单纯转动机头不会重定义通道方向。
独立入口复用正式流程的地图转弯判定，地图确认转弯后切换到新通道方向。
本入口从通道内直接开始，不等待正式入口流程的前 2 m 直行阶段。
转弯过程中手持设备跟着新通道转向，机头对准新方向后解除转向等待；沿新方向走出
既有约 0.6 m 过渡区后恢复出口检测。没有地图转弯证据时，不会只因机头旋转而更新方向。

可以从入口内侧慢慢走完整条通道，保持设备约 0.6 m 高，机头跟随当前通道方向。
在每个拐角、单侧开口和障碍旁稍作停留，扫描周围后继续前进；关注真实出口之前
是否出现 `exit=1` 或 `confirmed.data=true`，出现即记录为疑似误检。
`open_space_hits>0` 只表示候选证据，不等于已经确认出口；同样可以记录用于排查。
到达真实出口时扫描门外左右区域，确认距离候选出口不超过 1 m。无需穿出出口。
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
  /exit_detection_test/detected_exit /exit_detection_test/markers
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
检测高度取实时里程计 Z，可按正式测试高度约 0.6 m 手持观察。

检测要求来路有连续双侧墙体，出口外左右净空各超过 2.1 m，扇区通过率至少 80%，
完整机体范围已观测为空闲，并在 3 次新地图证据中稳定，证据间隔至少 0.5 s。
未知区域不算空闲，因此只看到前方一条空线、单侧开口或未扫到门外时不会确认。

没有输出时先检查输入频率：

```bash
rostopic hz /UAV0/fast_lio/Odom_high_freq
rostopic hz /UAV0/fast_lio/cloud_registered
```

按 `Ctrl+C` 关闭检测即可；本流程不需要 MAVROS、位姿回传或飞行控制节点。
