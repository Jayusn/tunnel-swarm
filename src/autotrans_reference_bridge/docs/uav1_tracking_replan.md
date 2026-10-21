# UAV1 跟踪偏离重规划与控制器同步

## 运行行为

正式入口 `roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch` 默认启用：

- `tracking_replan_enabled:=true`
- `tracking_replan_error:=0.10`（m）
- `tracking_stop_error:=0.15`（m）

在控制器原有 100 Hz 控制调用中检查正在执行的轨迹。距路线超过 10 cm 持续 0.1 s 时，从实际里程计位置、速度直接重规划到当前任务目标；不要求回接原路线、不设置“回到 3 cm”恢复门槛。超过 15 cm 时本次控制调用即清空旧轨迹队列及旧终点缓存，切换当前 XYZ 与航向锁点，等待匹配的新轨迹。

“距路线”是三维几何距离：在当前执行时刻前后各 1 s 的局部轨迹上，以不超过 20 ms 的间隔取样，再投影到相邻点的线段。它包含横向和高度偏离，但不直接把沿路线落后当成横向偏离；有限搜索区间避免在弯道/交叉处匹配远处轨迹分支。超出此区间的严重落后仍可能表现为局部距离增加。

仍在路线附近、但距当前时刻参考点超过 15 cm 持续 0.1 s，也发起普通重规划，重新安排时间；仅此时间误差不触发 15 cm 几何偏差停止。普通请求等待期间保留当前轨迹，清除预先排队的未来旧轨迹；几何偏差扩大到 15 cm 可随时升级为停止请求。若等待时旧轨迹已结束，锁当前位置而不是继续追旧终点。

停止沿用现有 NMPC 的当前位置锁点控制，不新增未经验证的制动多项式。清空旧参考不代表机体瞬间停止；本次未给出碰撞检查过的连续制动轨迹或实测制动距离保证。规划器仍使用现有地图与轨迹碰撞检查，实际占据/外部停止等现有处理保持优先。目标仍占据时不反复自动恢复同一目标。

## 两端同步

新增 `traj_utils/TrackingReplan` 和 `TrackingReplanResult`，不修改原 PolyTraj / PolynomialTraj 消息格式：

1. 控制器通过 `/drone_1_planning/tracking_replan` 发送请求，带独立 `request_id`、请求时间、原轨迹编号、是否停止及几何误差。
2. 规划器刷新里程计，用现有重规划函数生成轨迹；跟踪纠偏不套用“相对旧轨迹长度/jerk 必须改善”的周期质量门控。失败后保持请求，按现有重规划间隔重试，不返回旧轨迹作为成功结果。
3. 规划器通过 `/drone_1_planning/tracking_result` 返回新轨迹并回传 `request_id` 与请求时间。普通轨迹话题继续发布，用于已有日志/可视化和其他链路。
4. 控制器等待期间拒收普通轨迹替换，只接收 `request_id` 与时间均匹配当前请求的结果。15 cm 升级产生新请求，之前 10 cm 请求的晚到结果不再有效。转换后仍调用原生产轨迹校验/入队函数；无效结果保持等待。
5. 外部 HOLD、中止、降落/控制恢复阻止轨迹接收时，取消当前请求的接收资格，晚到结果不能解除这些状态。

两个节点的回调均沿用原串行调度，没有新增线程或 FSM 状态。100 Hz 为标称调度频率，处理耗时/调度抖动不构成硬实时保证。控制器停止在本地执行，不等待规划器完成求解。

新请求/结果话题已加入正式录包清单。窗口估计、Z 补偿关闭、用户既有 Q_velocity 与水平限速配置均保持原状。通用 controller launch 默认关闭这项功能；UAV1 正式入口明确开启。原 `fsm/max_tracking_error` 仍保留旧重规划起点兼容逻辑，本机首先由新的 10/15 cm 检查介入。

## 构建与验证

主工作空间为 `/home/oem/match_ws`，继续使用缓存中的 GCC 10，输出至 `/home/oem/match_ws/devel`。本次涉及 `traj_utils` 新消息、`diff_planner`、`payload_mpc_controller` 与 `autotrans_reference_bridge` launch；桥接包也参与构建。LDOP 公开头文件在现有环境需要提供包含目录，未修改 LDOP 源码：

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export CPLUS_INCLUDE_PATH=/home/oem/ldop_ws/external/install/ufomap/include:/home/oem/.local/usr/include
catkin_make -DCATKIN_WHITELIST_PACKAGES='autotrans_reference_bridge;cmake_utils;diff_planner;fast_lio;ldop;livox_ros_driver2;obj_state_msgs;path_searching;payload_mpc_controller;plan_env;quadrotor_msgs;traj_opt;traj_utils;uav_utils' -j2
catkin_make test_locked_exit_yaw test_planner_odometry -j2
rostest payload_mpc_controller locked_exit_yaw.test
rostest diff_planner planner_odometry.test
roslaunch --dump-params autotrans_reference_bridge uav1_diff_autotrans.launch enable_planner:=true enable_rviz:=false
```

最终运行结果：控制器 29 项、规划器 77 项全部通过；相关静态检查 38 项通过，5 项已在改动前 HEAD 复现的历史失败另行记录。正式启动参数解析及两端话题 remap 核对通过；主 devel 三个相关可执行文件的生成时间均晚于本次对应源码/launch 修改。

测试涵盖局部几何偏差/沿轨迹落后的区分、10 cm 持续时间、15 cm 本地退出旧轨迹、请求升级、延迟/乱序、无效结果、HOLD/中止优先及占据目标处理。测试使用独立 ROS master，不连接飞控。

日志与启动参数快照位于仓库外 `/home/oem/.ros/analysis/tracking_sync_20261021/`。旧静态契约测试中，heartbeat_timeout、escape_speed、run_swarm 恢复配置、旧速度默认值和旧航向实现字符串共 5 项断言在改动前 HEAD 上也失败；本轮未为适配这些历史断言修改运行参数。

本次是代码、消息时序、求解器回归和启动解析验证，没有完成闭环飞行仿真或实飞。因此不能据此宣称跟踪精度已经改善。生效需在地面重启 UAV1 规划器和控制器；不要在飞行中热切换二进制。窗口估计与 Z 补偿配置不随本功能切换。

2026-10-21 后续修正：ROS 发布会覆盖 `Header.seq`，因此它不能充当请求编号。请求/结果消息现在独立携带 `request_id`，新增实际 ROS 发布订阅测试，验证消息头序号改写后仍正确接替。消息 MD5 已变化，需在地面一起重启规划器与控制器。上述 29/77 项为前次提交的验证记录，本次结果见 [连续分段规划](uav1_rolling_segments.md)。
