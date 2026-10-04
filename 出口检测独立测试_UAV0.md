# UAV0 出口检测独立测试

只测试“通道向双侧开阔区域过渡”的识别，不需要起飞、入口识别、完整探索或飞控。
复用正式 FUEL 的累计 SDFMap、出口几何判定和连续地图确认逻辑。
测试入口只有 `exit_detection_node`，不会生成探索航点、穿越轨迹或降落请求。
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

将设备放在出口内侧，机头朝向出通道方向，再在终端 3 启动检测：

```bash
roslaunch exploration_manager exit_detection_test.launch
```

第一帧里程计的 yaw 固定为本轮检测的通道前向；转动机头不会重新定义该方向。
慢慢手持走近出口，保留门内双侧墙体并扫描门外左右区域。确认时设备距离候选出口
不超过 1 m。无需走完整条通道，也无需穿出出口。重测另一个方向时重启检测 launch。

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

## RViz

```bash
rviz -f UAV0/camera_init
```

添加 `PointCloud2`：`/exit_detection_test/map/occupancy_all`。
添加 `Marker`：`/exit_detection_test/markers`。
橙色表示候选，绿色及 `OPEN-SPACE READY / STILL INSIDE` 表示稳定确认；箭头朝出口外。
矩形标记表示开阔区域过渡截面，不代表识别到了实体门框。

## 回放已注册点云与里程计

录包需包含同一坐标系的 FAST-LIO 注册点云和里程计；无需重新运行 FAST-LIO。
先启动检测，再播放，确保第一帧机头朝向待测出口：

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
