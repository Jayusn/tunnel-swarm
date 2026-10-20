# 后机普通路线与停车点

普通 FIFO 跟随启用 `enable_ordinary_route`。入口、出口、门外等待、
降落点和占据脱障继续使用原来的单点接口。

## 接口

- `/UAV1/planning/goal/ordinary_route`：`nav_msgs/Path`，后机里程计局部坐标。
  `header.stamp` 是请求编号；`poses` 是从当前队首起、到下一个任务关键点之前
  的连续普通航点。相同编号的空路径取消请求；旧编号不能取消新任务。
- `/UAV1/planning/goal/leader_position`：`geometry_msgs/PointStamped`，
  复用双机对齐变换，将前机位置转换到后机坐标。时间戳保留前机原始里程计时间，
  不使用转发时间刷新有效期。
- `/drone_1_planning/status`：沿用 `std_msgs/String`。
  `ORDINARY_ROUTE_WAIT goal_stamp_ns=N` 表示尚无可执行路线；
  `ORDINARY_ROUTE_FAILED goal_stamp_ns=N` 表示轨迹提交检查失败；
  `ORDINARY_ROUTE_PUBLISHED goal_stamp_ns=N count x y z duration`
  表示覆盖请求前 `count` 个航点的轨迹已发布，末尾三坐标为停车终点，
  `duration` 为轨迹时长（秒）。

跟随器只接受当前请求的反馈；普通经过点实际到达后顺序出队，不截断本段轨迹。
停车终点还须等轨迹结束、实际速度降低并满足原有到达驻留时间，才锁点等待下一段。
紧急间距锁点、里程计失效及脱障接管会取消普通路线。

## 规划规则

- 跟随、普通点放行、间距锁点及恢复配置统一为 1.20 m。
  出口及门外等待通过 `task_separation=1.50 m` 保留原间距；入口仍按原门内进度规则。
- 普通路线全程检查前机当前位置的 XY 距离，要求
  `ordinary_separation + ordinary_tracking_margin = 1.20 + 0.10 m`。
  前机数据有效期为 0.50 s。
- 按 FIFO 顺序选择第一个可停车、可规划的普通终点。停车检查使用现有
  `fapp_vehicle_radius`、`fapp_vehicle_half_height` 和独立的
  `ordinary_stop_margin=0.10 m`，检查整个圆柱包络。
- 沿原路线插入支撑点，生成连续五次多项式；经过点不强制零速，
  停车终点速度和加速度为零。曲线逐段检查原路线偏差、反向运动、
  膨胀地图占据和前机间距，速度/加速度使用普通段 0.40 m/s、0.50 m/s²。
- 不允许通过把终点直接交给单目标重规划而绕过经过点。
  普通段关闭直接到终点的周期替换；地图突变仍由现有碰撞检查、
  紧急停车及占据脱障处理。

这是一种保守的提前等待策略：没有足够缓存点、没有可停车位置、曲线无法沿原路线
通过或间距不足时，继续等待，不自动绕远或放宽约束。前机位置按当前快照检查，
不预测前机突然倒退；1.20 m 不是对实际瞬时距离的无误差保证。

## 验证

`diff_planner` 的 `planner_odometry.test` 覆盖连续经过点、弯道、整段间距、
机体停车包络、过期里程计和取消消息归属；`exploration_control` 的
`fifo_entry_spacing.test` 覆盖出队不中断轨迹、旧回执、实际间距急停、
任务关键点排除及已有入口/出口/脱障行为。测试必须使用本机隔离 ROS master。

2026-10-20 本地验证：主工作空间两个包编译通过；规划器 80 项、跟随器 55 项
ROS 测试通过；启动参数解析通过；相关 Python 静态检查 37 项通过。
另外 5 项静态检查在修改前 HEAD 上也失败，涉及七屏网络默认值、RViz 环境、
相机配置、旧源码字符串匹配和旧应答超时值，本次没有修改这些既有问题。
尚未实飞验证；未对实机发布测试控制指令。
