# 前机 UAV0 LDOP 动态检测接入方案

## 1. 文档目的

本文用于指导在前机工作空间 `/home/asus/match_ws` 中，将后机当前使用的 LDOP
动态检测能力接入以下 FUEL 探索入口：

`src/Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_swarm_indoor1_fuel_exploration.launch`

本文只描述实施方案，不代表相关源码和 launch 已经修改完成。

## 2. 当前前机状态

2026-08-28 只读检查结果：

- SSH：`asus@10.54.87.212`
- 工作空间：`/home/asus/match_ws`
- 根仓库分支：`vehicle/cav0`
- 根仓库 HEAD：`2f4cf5d`
- 前机实际里程计：`/UAV0/fast_lio/Odom_high_freq`
- 前机注册点云：`/UAV0/fast_lio/cloud_registered`
- `ldop_node` 和 `planner_command_arbiter` 已有编译产物

实施前必须重新检查根仓库、Diff-Planner 子仓库和 LDOP 子仓库的分支、HEAD 与工作区，
不得直接假定仍是上述版本。

```bash
cd /home/asus/match_ws
git branch --show-current
git rev-parse --short HEAD
git status --short
git -C src/Diff-Planner branch --show-current
git -C src/Diff-Planner rev-parse --short HEAD
git -C src/Diff-Planner status --short
git -C src/LDOP branch --show-current
git -C src/LDOP rev-parse --short HEAD
git -C src/LDOP status --short
```

## 3. 当前接口与缺口

前机 launch 已预留以下接口：

- LDOP 输出：`/UAV0/ldop/dynamic_objects`
- `planner_command_arbiter` 订阅该话题
- 环境变量 `UAV0_ENABLE_DYNAMIC_OBSTACLE_DETECTION` 控制检测和仲裁
- `run_ldop_uav_pair.launch` 可启动 UAV0/UAV1 两套 LDOP

但当前实现存在以下问题：

1. 前机 LDOP 版本只有通用 UFOMap 时序动态检测，没有后机当前使用的通道候选检测逻辑。
2. `run_ldop_uav_pair.launch` 中 UAV0 里程计仍写为
   `/UAV0/fast_lio/Odometry`，与前机实际的 `Odom_high_freq` 不一致。
3. LDOP 直接订阅满频注册点云，没有后机使用的独立 5 Hz 限频，容易增加 CPU 占用并积压旧帧。
4. 检测节点和规划拦截共用一个开关，不能只观察检测结果而不影响飞行。
5. 现有仲裁器不是动态绕行规划器，只检查当前位置到下一条 PositionCommand 的短线段。
6. 命中动态目标后，仲裁器发布当前位置 hold 和 emergency brake；不会把动态目标送回
   FUEL 优化器生成绕行轨迹。

因此不能直接把 `UAV0_ENABLE_DYNAMIC_OBSTACLE_DETECTION=true` 当作最终接入方案。

## 4. 总体实施原则

接入分为两个独立阶段：

1. 第一阶段只启动和验证 LDOP 检测，不能干预 FUEL 控制输出。
2. 第二阶段在检测稳定后，让 FUEL 的完整未来轨迹参与动态碰撞检查和重规划。

第一阶段没有通过前，不实施第二阶段。

## 5. 第一阶段：只接动态检测

### 5.1 对齐 LDOP 版本

将后机当前已实飞调试的通道动态检测逻辑整理为独立提交，再同步到前机 LDOP 子仓库。
需要包含：

- 通道 ROI 候选提取
- 墙面过滤
- 候选聚类和候选数量限制
- 跨帧确认与 ID 跟踪
- UAV0/UAV1 可配置输入输出话题

同步时不能覆盖前机 LDOP 子仓库已有修改，应先比较提交历史和工作区差异。

### 5.2 拆分开关

在前机 FUEL launch 中使用两个含义明确的开关：

- `enable_uav0_ldop_detection`：只控制 LDOP 节点是否运行
- `enable_uav0_dynamic_avoidance`：只控制检测结果是否影响规划和控制

第一阶段默认值：

```text
enable_uav0_ldop_detection=true
enable_uav0_dynamic_avoidance=false
```

不要继续使用一个环境变量同时打开检测节点和强制悬停仲裁。

### 5.3 独立限制 LDOP 输入频率

FUEL 地图继续订阅原始注册点云，不改变其建图输入。新增一个仅供 LDOP 使用的限频话题：

```text
输入：/UAV0/fast_lio/cloud_registered
频率：5 Hz
输出：/UAV0/ldop/cloud_registered_throttled
```

LDOP 订阅队列应保持较小，目标是处理最新帧，不是排队处理历史帧。FUEL 不应改为订阅
LDOP 的 `static_cloud`，否则动态分类错误或检测延迟可能造成静态障碍从规划地图消失。

### 5.4 UAV0 输入输出

LDOP UAV0 实例应使用：

```text
input_cloud_topic=/UAV0/ldop/cloud_registered_throttled
odom_topic=/UAV0/fast_lio/Odom_high_freq
static_cloud_topic=/UAV0/ldop/static_cloud
dynamic_cloud_topic=/UAV0/ldop/dynamic_cloud
dynamic_objects_topic=/UAV0/ldop/dynamic_objects
dynamic_object_markers_topic=/UAV0/ldop/dynamic_object_markers
dynamic_track_markers_topic=/UAV0/ldop/dynamic_track_markers
dynamic_predictions_topic=/UAV0/ldop/dynamic_object_predictions
dynamic_prediction_markers_topic=/UAV0/ldop/dynamic_prediction_markers
static_map_markers_topic=/UAV0/ldop/static_map_markers
```

所有 UAV0 输出必须放在 `/UAV0/ldop/` 下，不能与 UAV1 共用全局话题。

### 5.5 第一轮参数基线

先复用后机当前参数作为前机测试基线，不代表最终参数：

```text
ldop_input_rate=5.0
ufomap_resolution=0.05
ufomap_inflate_hits_dist=0.10
corridor_min_lateral_span=0.15
corridor_min_confirm_hits=3
corridor_min_cluster_points=4
corridor_max_cluster_extent=0.8
corridor_oversized_split_enabled=false
corridor_max_candidates=3
corridor_reject_candidate_count=5
corridor_wall_clearance=0.10
dynamic_cluster_detector_min_points=4
```

前机的通道宽度、雷达安装方向和飞行速度与后机不完全相同时，应基于前机录包单独调整，
不能只因为后机可用就直接用于正式飞行。

### 5.6 第一阶段录包

至少记录：

```text
/UAV0/fast_lio/Odom_high_freq
/UAV0/ldop/cloud_registered_throttled
/UAV0/ldop/dynamic_cloud
/UAV0/ldop/dynamic_objects
/UAV0/ldop/dynamic_object_predictions
/UAV0/ldop/dynamic_object_markers
/UAV0/ldop/dynamic_track_markers
/tf
/tf_static
/rosout
```

不建议在同一个轻量诊断包中重复录制满频 `/UAV0/fast_lio/cloud_registered`。

### 5.7 第一阶段验收条件

第一阶段至少满足：

- 静态墙面和固定杆不会持续发布为动态目标。
- 动态目标进入通道后 5 秒内稳定发布。
- 同一目标短时遮挡后能保持或快速恢复同一 ID。
- `dynamic_objects` 的 header 坐标系与 UAV0 里程计坐标系一致。
- RViz 显示没有持续积压，动态目标位置不明显落后实物。
- 开关 LDOP 不改变 `/UAV0/planning/pos_cmd` 和 FUEL 飞行行为。

## 6. 第二阶段：接入 FUEL 动态避障

### 6.1 不沿用当前末端短线段检查作为最终方案

当前 `planner_command_arbiter` 的动态检查只比较：

```text
无人机当前位置 -> 当前一条 PositionCommand 位置
```

它不能代表完整 B 样条轨迹，也不能生成绕行路径。该逻辑可以保留为最终输出端的紧急兜底，
但不能承担正常动态避障。

### 6.2 推荐接入位置

将 LDOP 动态目标接入 FUEL 已有安全检查链：

```text
src/zyc_fuel_ws/src/FUEL/fuel_planner/exploration_manager/
  include/exploration_manager/fast_exploration_fsm.h
  src/fast_exploration_fsm.cpp

src/zyc_fuel_ws/src/FUEL/fuel_planner/plan_manage/
  include/plan_manage/planner_manager.h
  src/planner_manager.cpp
```

FUEL 当前每 0.05 秒执行一次 `safetyCallback()`，并通过 `checkTrajCollision()` 检查静态地图。
动态检查应与此入口并列，而不是放在 PositionCommand 发布之后才处理。

### 6.3 动态轨迹检查逻辑

最小实现范围：

1. 订阅 `/UAV0/ldop/dynamic_objects`。
2. 保存消息时间、目标 ID、当前位置、速度和包围盒尺寸。
3. 过期消息不参与检查。
4. 对当前正在执行的 B 样条未来 `1.0~1.5s` 按固定时间步采样。
5. 同时按 LDOP 速度预测目标位置。
6. 检查无人机轨迹采样点与目标膨胀包围体是否相交。
7. 预测碰撞时进入 FUEL 原有 `PLAN_TRAJ` 流程。

第一版不要同时引入复杂多分支概率决策。先使用 `dynamic_objects` 的当前状态和短时恒速预测，
确认闭环正确后，再评估 `dynamic_object_predictions`。

### 6.4 重规划与停车边界

- 预测碰撞距离充足：触发 FUEL 重规划，继续沿用 FUEL 的静态地图避障。
- 重规划成功且新轨迹不与动态目标相交：发布新轨迹。
- 重规划失败或碰撞时间过短：进入现有 safety hold。
- LDOP 短时漏检：在消息超时范围内继续使用最后状态预测。
- LDOP 消息超时后：停止使用旧动态目标，不能无限锁住无人机。

当前仲裁器的 emergency brake 可作为临近碰撞兜底，但应由独立
`enable_uav0_dynamic_emergency_guard` 开关控制。

### 6.5 第二阶段建议初始参数

```text
dynamic_object_timeout=0.60
dynamic_safety_radius=0.45
dynamic_vertical_margin=0.25
dynamic_prediction_horizon=1.0
dynamic_prediction_step=0.10
```

`dynamic_safety_radius` 是机体附加安全半径，目标自身半径还需要由 bbox 尺寸计算，二者不能混为
一个固定值。最终数值必须用前机真实跟踪误差和动态目标尺寸验证。

## 7. 不建议的接法

以下方式暂不采用：

- 不把 `/UAV0/ldop/dynamic_cloud` 直接长期写入 FUEL 静态占据地图。
- 不把 FUEL 建图输入替换为 `/UAV0/ldop/static_cloud`。
- 不直接打开现有统一环境开关后进行正式飞行。
- 不只检查单条 PositionCommand 就宣称完成动态避障。
- 不在未确认坐标系一致时消费 `dynamic_objects`。
- 不同时修改 LDOP 检测、FUEL 规划器和控制器悬停逻辑后一次性实飞。

## 8. 实施与验证顺序

1. 核对三个仓库状态并保存当前提交号。
2. 同步后机 LDOP 通道检测代码到前机。
3. 修正 UAV0 里程计话题并增加 5 Hz 独立点云输入。
4. 拆分“检测”和“规划消费”开关。
5. 编译 LDOP、Diff-Planner `plan_manage` 和受影响的 FUEL 包。
6. 执行 roslaunch XML/参数解析检查。
7. 静态运行 LDOP，检查输出话题和坐标系。
8. 手持动态目标录包，规划消费保持关闭。
9. 完成第一阶段验收后，再实现 FUEL 完整轨迹动态碰撞检查。
10. 先低速系留或保护条件测试，再进入完整通道实飞。

## 9. 编译和静态检查

源码修改后必须更新前机实际启动使用的 `/home/asus/match_ws/devel`。根据最终修改范围编译全部
受影响包，不能只在隔离工作空间编译。

至少检查：

```bash
cd /home/asus/match_ws
source /opt/ros/noetic/setup.bash
catkin build ldop diff_planner exploration_manager plan_manage
source devel/setup.bash --extend

roslaunch --files \
  /home/asus/match_ws/src/Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_swarm_indoor1_fuel_exploration.launch

rospack find ldop
rospack find diff_planner
ls -l devel/lib/ldop/ldop_node
ls -l devel/lib/diff_planner/planner_command_arbiter
```

实际包名应以 `catkin list` 为准。如果工作空间不是由 `catkin build` 管理，应沿用前机当前构建方式，
但必须确认主工作空间 `devel` 中的产物时间晚于本次源码修改。

## 10. 运行时核对

```bash
rostopic hz /UAV0/ldop/cloud_registered_throttled
rostopic hz /UAV0/ldop/dynamic_objects
rostopic echo -n 1 /UAV0/ldop/dynamic_objects
rostopic info /UAV0/ldop/dynamic_objects
rosnode info /UAV0_ldop_node
```

重点确认：

- LDOP 点云输入约为 5 Hz。
- `dynamic_objects` 不积压旧帧。
- 目标坐标与 `/UAV0/fast_lio/Odom_high_freq` 一致。
- 第一阶段 `/UAV0/planning/pos_cmd` 不受 LDOP 启停影响。
- 第二阶段只有预测碰撞时才触发重规划或急停。

## 11. 预期最终结构

```text
/UAV0/fast_lio/cloud_registered
  |---> FUEL SDF map（保持原链路）
  `---> topic_tools throttle 5 Hz
         `---> /UAV0/ldop/cloud_registered_throttled
                `---> LDOP
                       |---> /UAV0/ldop/dynamic_objects
                       |---> /UAV0/ldop/dynamic_object_predictions
                       `---> RViz/录包诊断输出

FUEL current B-spline + LDOP dynamic objects
  `---> FUEL safetyCallback 动态碰撞检查
         |---> 有时间：PLAN_TRAJ 重规划
         `---> 来不及或重规划失败：safety hold / emergency brake
```

这样检测故障不会改变 FUEL 的静态建图来源，动态目标也不会只在控制命令发布末端被粗暴拦截。
