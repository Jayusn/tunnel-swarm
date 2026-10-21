# FUEL 手填 XYZ 多点飞行

入口：`roslaunch diff_planner run_exp_single_fuel_waypoints.launch`。
与 `run_swarm_indoor1_fuel_exploration.launch` 二选一，不能同时启动。
原探索 launch 内容不变；本入口不启动通道搜索、前沿选点、出口任务或 Diff 规划器。

## 填写航点

修改同目录的 `fuel_waypoints.yaml`，将 `waypoints: []` 改成自己的 XYZ 列表。
全部坐标均是 `/UAV0/fast_lio/Odom_high_freq` 的世界系绝对坐标，通常为
`UAV0/camera_init`，单位米；高度也是该坐标系中的 z，不是相对当前位置的高度。
查看当前位置可用 `rostopic echo -n 1 /UAV0/fast_lio/Odom_high_freq/pose/pose/position`。
示例仅用于说明格式，须按实际场地填写：

```yaml
mission:
  manual_path:
    arrive_distance: 0.25
    waypoints:
      - [1.0, 0.0, 0.8]
      - [1.0, 2.0, 0.8]
      - [3.0, 2.0, 0.8]
```

不限制五个点；一个点就是点到点，多项按列表顺序执行。实际里程计进入目标
`arrive_distance` 三维半径后才推进列表，预测轨迹到点或规划失败不消耗航点。
全部到点后结束，不再追加探索目标，也不自动降落；空列表没有飞行目标。

## 启动及开始

需要沿用双机网络和完整七分屏时，运行新入口：

```bash
bash ~/match_ws/shfiles/start_uav0_twaypoints.sh
```

它沿用 `start_uav0_dual_network_six_terminator.sh` 的网络、传感器、检测、控制器、
录包和终端布局，只通过 `UAV0_FUEL_PLANNER_LAUNCH` 把第六屏换成本规划器。
原入口不设置该变量时仍启动原探索 launch；两套入口不要同时运行。
七分屏中的 FAPP 参数仍传给新规划器；直接运行下面的 launch 默认不启用 FAPP。

沿用现有雷达、FAST-LIO、MAVROS 和控制器启动流程，只替换规划器终端：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch diff_planner run_exp_single_fuel_waypoints.launch
```

启动后先等待里程计、点云和控制器就绪，并按现有流程起飞。再在 RViz 点击
`2D Nav Goal` 开始执行列表；这次点击仅表示“开始”，点击位置不替换 YAML 航点。
本 launch 不负责解锁、起飞或启动控制器。

不使用 RViz 时可以关闭窗口，并在控制器已就绪后发送一次开始信号：

```bash
roslaunch diff_planner run_exp_single_fuel_waypoints.launch enable_rviz:=false
rostopic pub -1 /UAV0/fuel_waypoints/start geometry_msgs/PoseStamped \
  '{header: {frame_id: "UAV0/camera_init"}, pose: {orientation: {w: 1.0}}}'
```

修改或重新执行路线需要重启本规划器。也可以指定其他配置文件：

```bash
roslaunch diff_planner run_exp_single_fuel_waypoints.launch \
  waypoints_file:=/绝对路径/我的航点.yaml fuel_max_vel:=0.4 fuel_max_acc:=0.5
```

## 规划链路和边界

- 输入仍为 `/UAV0/fast_lio/Odom_high_freq` 和 `/UAV0/fast_lio/cloud_registered`。
- 复用 FUEL `algorithm.xml`、`fuel_map.yaml`、A*、B 样条优化和碰撞重规划。
- 轨迹为 `/UAV0/planning/bspline`，位置命令直接输出 `/UAV0/planning/pos_cmd`。
- 默认速度 0.40 m/s、加速度 0.50 m/s²，地图尺寸和搜索边界沿用原入口默认值。
- 目标之间允许绕障，但不会跳过人工航点。A* 需要当前地图中的已知自由连通路径；
  目标在墙内、地图外、尚未知区域或无连通路径时保持当前目标并重试，不自行探索开路。
  应在转弯、遮挡处布置中间点，且航点高度和位置须在配置地图范围内。
- 原 `uav0_manual_waypoints.launch` 的直线段模式保持原行为；仅本入口开启
  `mission/manual_path/use_astar`。

原简单控制器可沿用现有命令。若使用 AutoTrans MPC，单独启动
`roslaunch autotrans_reference_bridge uav0_autotrans_controller.launch pure_fuel_mode:=true`。
两种控制器只选一种。
