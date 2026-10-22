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

入口、出口开关直接修改 `run_exp_single_fuel_waypoints.launch` 顶部的 `true` / `false`，
七分屏脚本使用这里的设置。例如仅关闭入口：

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

## 规划链路

- 航点段和出口接近段仍使用FUEL `algorithm.xml`、地图、A*、B样条和既有碰撞检查。
- 此 launch 默认启动刚同步的 UAV0 FAPP 运动证据检测，并将动态目标接入 FUEL 优化器及运行期
  轨迹检查；默认启用动态避障和规划优先，设置与 `start_uav0_twaypoints.sh` 一致。
  如果只想看检测输出而不让FUEL避障，可启动时设置 `enable_fapp_dynamic_avoidance:=false`；
  关闭检测节点用 `enable_fapp_detection:=false`。运动证据由
  `fapp_object_motion_evidence_enabled` 控制，双向匹配、候选拆分及候选关联隔离默认关闭。
- 航点只决定远处目的地，每次沿已知路径规划最多 **1.2米** 的局部段；还检查优化后B样条的
  控制多边形长度上界，避免短路径被平滑成超长轨迹。到局部端点后按新地图续接，
  只有真实里程计到达用户航点，才推进 YAML 列表。长度在 launch 的
  `mission/manual_path/max_segment_length` 设置。
- 默认开启 `mission/manual_path/exploration_style`：复用探索的地图转弯和绕障。
  直角弯先制动、原地对准新通道，再继续平移；真实航向连续0.15秒进入8度范围才算完成。
  弧形通道使用原有沿墙方向渐变，普通绕柱侧移不会把机头一起扭过去。出口开关关闭也保留转弯。
- 难飞的绕障段按 **侧移 → 同侧通过 → 回到航点方向** 执行，复用原有宽侧选择和侧向续接。
  每一步独立检查、规划，不把三步平滑成一条跨障长曲线；复杂障碍可能需要多个短步完成某一阶段。
- A*需要已知自由空间。远点暂不连通时，可以先沿通向该点的已知自由短段推进；
  如果连短段也不可行，就保留航点等待重规划，不穿墙、不跳过航点，也不切回自主前沿探索。
  拐角遮挡严重时，仍建议像当前 YAML 一样在转角内侧填写中间点。
- 开出口时，FUEL位置命令先输出 `/UAV0/fuel/planning/pos_cmd`，由原任务阶段仲裁器发布
  `/UAV0/planning/pos_cmd`；关出口时直接输出原最终话题。
- `fuel_waypoint_portals.xml` 单独保存复用的入口/出口和降落节点参数；不包含原自动前沿目标选择。
- 原 `uav0_manual_waypoints.launch` 及自动探索入口不启用本航点出口延期逻辑，保持原行为。
