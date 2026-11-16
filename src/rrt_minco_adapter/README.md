# RRT＋MINCO 独立适配

本包新增静态地图点到点规划入口，不修改 FUEL、Diff、控制器及现有任务入口。
算法来自 `rrt_super_bundle_20261116`，基础提交
`6003f70ac1520447137bf01854813c9c7a895818`，原包包含未提交源码；迁入的
`rrt_core` 保持原算法实现。原包未为这些 Python 文件明确指定许可，
本包不作额外开源授权。SUPER 未纳入此次接入。

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
不订阅雷达点云，不处理未知空间、动态障碍或连续在线重规划。

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

默认速度 0.6m/s、加速度 0.8m/s²、净空 0.3m、额外余量 0.1m，可用
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
