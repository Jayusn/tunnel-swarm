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

## 两个独立开关

在 `run_exp_single_fuel_waypoints.launch` 顶部设置，也可以通过启动参数覆盖。
两个开关默认均为 `true`，支持全部四种组合。

| 开关 | true | false |
| --- | --- | --- |
| `enable_entry_recognition` | RViz点击后沿用现有找门、接近入口流程，到门平面附近才触发航点 | 不启动入口识别，RViz点击直接开始航点 |
| `enable_exit_recognition` | 识别并锁存出口，完成最近门内点后转向出口，再接搜板降落 | 不识别出口、不自动进入降落；飞完全部点后结束路线 |

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

通过环境变量分别开关入口、出口，例如仅关闭入口：

```bash
UAV0_WAYPOINT_ENTRY_ENABLED=false UAV0_WAYPOINT_EXIT_ENABLED=true \
  bash ~/match_ws/shfiles/start_uav0_twaypoints.sh
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

## 规划链路

- 航点段和出口接近段仍使用FUEL `algorithm.xml`、地图、A*、B样条和既有碰撞检查。
- A*需要当前地图中的已知自由连通路径。目标在墙内、地图外、未知区或无连通路径时保留目标并重试；
  不自行探索开路。转弯、遮挡处应填写中间点。
- 开出口时，FUEL位置命令先输出 `/UAV0/fuel/planning/pos_cmd`，由原任务阶段仲裁器发布
  `/UAV0/planning/pos_cmd`；关出口时直接输出原最终话题。
- `fuel_waypoint_portals.xml` 单独保存复用的入口/出口和降落节点参数；不包含原自动前沿目标选择。
- 原 `uav0_manual_waypoints.launch` 及自动探索入口不启用本航点出口延期逻辑，保持原行为。
