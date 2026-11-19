# RRT / SUPER 固定高度对比接入

本接入与桥梁平台无关。默认仍运行 RRT；选择 SUPER 后，共用 FUEL 冻结地图、任务流程、PositionCommand、AutoTrans MPC、遥控器和降落接口。不要同时启动两套实飞入口。

## 依赖与构建

SUPER 依赖本机 `~/rrt_super_bundle` 中已适配固定高度的原生代码：
- `upstream_ws/src/super_planner`、`rog_map`；
- `adapter_ws/src/super_gazebo_adapter` 及其 generated 源码。

源码继续保留在该目录，不复制到 match_ws；换机需要同时迁移整个源码包并重新构建。该包有同名但不兼容的 quadrotor_msgs，不能放到 match_ws/src，也不要在飞行终端 source 它的 setup.bash。适配器在子进程内隔离其运行环境，通过标准 ROS 消息通信。

首次构建：
```bash
env -u CMAKE_PREFIX_PATH -u ROS_PACKAGE_PATH -u PYTHONPATH bash ~/rrt_super_bundle/scripts/build_super.sh
source ~/match_ws/devel/setup.bash
catkin_make -C ~/match_ws --pkg rrt_minco_adapter -j2
```

## 先做地面对比（不发送飞行指令）

沿用现有雷达、FAST-LIO、FUEL 建图流程，扫描后冻结本次地图。每次实飞仍须重新建图，不复用旧冻结地图。

```bash
source ~/match_ws/devel/setup.bash
source ~/match_ws/shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav0
rosservice call /UAV0/rrt/freeze_map "{}"
```

只显示 SUPER 路径：
```bash
rosrun rrt_minco_adapter ground_plan.py _planner:=super _output_dir:=$HOME/.ros/super_ground_check
```

只显示 RRT 路径：
```bash
rosrun rrt_minco_adapter ground_plan.py _planner:=rrt
```

这两个命令都使用 `/UAV0/rrt/dry_run_path`，一次运行一个。运行中保持终端，Ctrl+C退出。

同一份冻结地图同时比较两个算法，不需要先运行 ground_plan：
```bash
rosrun rrt_minco_adapter compare_planners.py _trials:=3
```

结果在 `~/.ros/planner_comparison/时间戳/`，包括 scene.json、两算法成功轨迹以及 summary.json。失败也记录，不剔除失败样本。离线回放已保存地图（只对比，不用于新一轮实飞）：
```bash
rosrun rrt_minco_adapter compare_planners.py _scene_file:=$HOME/.ros/rrt_ground_check/scene.json
```

默认起点(0.01,0.01)、终点(3,3)、高度0.5m、速度0.2m/s、加速度0.8m/s²、净空0.2+0.1=0.3m。边界取冻结地图，实飞入口仍为[-1,4]×[-1,4]。历史地图可能使用旧边界，脚本不会偷偷替换其边界。可通过 `_start:='[x,y]'`、`_goal:='[x,y]'`、`_speed:=...` 修改对比条件，两算法使用同一参数。

## 实飞入口选择

SUPER：
```bash
bash ~/match_ws/shfiles/start_rrt_flight_all.sh super
```
RRT（原默认行为）：
```bash
bash ~/match_ws/shfiles/start_rrt_flight_all.sh rrt
```
之后仍按原实飞文档：重新扫描、冻结地图，遥控解锁/OFFBOARD，CH8低位起飞、升高后高位执行任务。切换算法不会改变解锁、起飞、任务门槛、终点降落或CH10接管逻辑。

直接任务入口也支持：`rosrun rrt_minco_adapter start_flight.sh planner:=super`。
现有录包脚本继续适用；`/UAV0/rrt/polynomial` 的 planner 字段标明 rrt/super，order 分别为5/7。话题中的 rrt 是兼容命名，不代表始终运行RRT。

## 比较口径及限制

- RRT：双向RRT*前端 + 五次MINCO；SUPER：本机适配版平面A*、凸走廊 + 七次MINCO。不是拿RRT前端单独对比完整SUPER。
- 同一冻结场景矩形、同一起终点、同一固定高度、同一净空验收。SUPER输入先将矩形按0.3m扩张，再离散成0.1m占据体素；其内部0.05m半径和一格膨胀用于体素规划。因此SUPER可能更保守，窄通道结果会受离散分辨率影响，不能称两者内部地图完全相同。
- SUPER原生软约束轨迹仍须通过现有连续净空/边界检查；不合格返回失败，不回退成RRT冒充成功。七次多项式保留原阶数，检查速度/加速度上界，必要时统一延长时间，不改变路径形状；time_stretch记录该变化。
- 子进程内部临时把相同数值坐标标记为world，输入和输出仍解释为原冻结地图坐标系，不做真实坐标变换。规划从静止状态开始，与当前RRT一致，不适用于运动中在线重规划。
- total_wall_s为返回可用轨迹前的耗时；SUPER含进程启动、地图构造和约1秒输入同步。native_wall_s仅为发送目标至收到原生结果的墙钟耗时（含ROS传输），不是纯优化器CPU时间。不可拿SUPER原生耗时直接对比RRT端到端耗时。
- summary提供成功/失败、路径长度、轨迹时长、jerk平方积分和耗时。长度与jerk采用20ms采样近似。重复试验使用固定RRT种子，主要观察耗时稳定性，不代表随机种子成功率。正式比较应覆盖多种相同地图、起终点和障碍密度。
- RRT的几何表达更直接、不依赖体素离散；SUPER的走廊优化及更高阶轨迹提供不同的平滑性/效率取舍。但更高阶不必然更好，需依据相同条件下的统计及实测跟踪误差判断。
- 这是静态、固定高度版本比较，不代表原版SUPER完整三维在线重规划与备份轨迹能力。

SUPER每次请求的原生日志、配置、场景和原生轨迹保存于 `~/.ros/super_matchws/`。若失败，检查该目录下最近一次 native.log；这些运行数据不提交Git。

本次验证使用独立ROS master和模拟传感器，包含真实MPC代码链路；不等同于已经完成SUPER实飞验证。

## 本机接入验证记录（2026-11-19）

合成场景：边界[-1,4]×[-1,4]，矩形障碍中心(1.5,1.5)、尺寸0.3×1.0m，其余为上述默认参数；单次结果：

| 指标 | RRT | SUPER |
|---|---:|---:|
| 规划及连续净空检查 | 通过 | 通过 |
| 路径长度 | 4.626m | 4.751m |
| 轨迹时间 | 28.604s | 46.267s |
| 返回轨迹耗时 | 0.548s | 2.463s（含启动及退出清理） |
| jerk平方积分 | 0.00836 | 0.000212 |

SUPER原生目标请求耗时约0.01051s，最终时间拉伸系数1.188。此例RRT路线更短、完成时间更短；SUPER的jerk积分更低，但它飞得更慢，不能据此单独证明算法更平滑或跟踪更好。一次合成测试不足以得出总体优劣。

历史 `~/.ros/rrt_ground_check/scene.json` 仍是[0,5]×[0,5]：RRT通过，SUPER第0段未通过连续边界认证，作为失败样本保留。这是保守认证结果，不等同于已证实轨迹发生碰撞。SUPER内部没有添加虚拟边界围墙，可能先规划出界，再被共同边界检查拒绝；不会自动改地图或改起点来通过测试。

详细运行数据在 `~/.ros/planner_comparison/synthetic_check/` 和 `integration_check/`，不纳入Git。

验证通过：54项单元测试；RRT和SUPER的ROS轨迹/指令一致性测试；SUPER配合真实MPC代码的模拟传感器任务测试（正常到点降落、落地停发、CH10中断）。SUPER原生代码及match_ws适配包均完成编译。
