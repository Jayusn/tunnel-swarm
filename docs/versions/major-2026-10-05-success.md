# 成功大版本：2026-10-05

- 工作空间：`match_ws`
- 主分支：`vehicle/cav0`
- 大版本标签：`major-version-2026-10-05-success`
- 版本记录前的源码基线：`98d4fc0fa3083dc4e9da605c1a45cfeea3b8d07a`
- 对应启动脚本：`shfiles/start_uav0_dual_network_six_terminator.sh`
- 工作空间远端：`https://github.com/szc-cnd/FUEL-AutoTrans-Multi-UAV-Workspace.git`

按用户要求，将当前版本保存为成功大版本。本次只记录版本和建立快照引用，不改变运行逻辑。

## 本版本功能

- 出口检测接入正式FUEL建图，确认后锁存出口位置和向外朝向；自动检测与CH9均可触发穿越及搜索降落。
- 到达出口后，由Diff向出口外前飞1 m；接近、穿越和前视搜索目标高度为0.60 m。
- FUEL接近出口保留锁存朝向，横移和局部路径不再改变机头方向；Diff穿越、前视扫描中心、回正和下视搜索沿用同一出口朝向。
- CH9无确认出口时，继续使用人工入口，以触发时机头朝向为基准。
- 前视扫描速度为15°/s，初始等待不计入扫描积分，修复等待结束时的航向跳变。
- 前视未找到目标后升至下视搜索高度；侧向边界停滞后记录边界，换行反扫，正常高度波动不再反复重置水平停滞计时。
- CH10低位后拨高位，可优先打断穿越和搜索，触发原地降落。

## 验证记录

源码基线提交记录了受影响目标编译通过、11项ROS测试通过、实录横移轨迹航向规划验证通过，以及出口几何与真实地图集成测试通过。
上述为上一轮航向修复的验证结果；最新航向修复尚未实飞验证。本次版本归档未修改代码，不重复运行这些测试。
提交前已检查主仓库及13个嵌套仓库的完整状态和差异，无遗漏的源码、配置、启动文件或文档修改。
构建目录、缓存、测试日志、录包、点云及压缩包均不纳入版本。

## 嵌套仓库快照

以下本地快照分支固定各嵌套仓库的当前提交，不切换其工作分支。
主仓库现有嵌套目录并非完整配置的Git子模块；仅保存主仓库标签不足以独立恢复整个工作空间，应同时保留以下仓库。

| 工作空间内路径 | 提交号 | 本地快照分支／计划远端分支 |
| --- | --- | --- |
| `src/Diff-Planner` | `b55f8ff1952ab617ab619011a686154004163154` | `snapshots/major-2026-10-05-success/Diff-Planner` |
| `src/Diff-Planner/src/realflight_modules/faster-lio` | `89f6802383167feee8776d9aa5e406d860355176` | `snapshots/major-2026-10-05-success/Diff-Planner--src--realflight_modules--faster-lio` |
| `src/Diff-Planner/src/realflight_modules/px4ctrl` | `c7d67e1447cda6d75441f501b507f80890cb092f` | `snapshots/major-2026-10-05-success/Diff-Planner--src--realflight_modules--px4ctrl` |
| `src/FAST_LIO` | `c9cde0cf55f66ddbfedbe7cf19b1f72a393c60ce` | `snapshots/major-2026-10-05-success/FAST_LIO` |
| `src/FAST_LIO/include/ikd-Tree/ikd-Tree` | `5f1ec7df5e201bcd0669c5249da46ae4fdb008c4` | `snapshots/major-2026-10-05-success/FAST_LIO--include--ikd-Tree--ikd-Tree` |
| `src/LDOP` | `a4c94578cd04dd961663f190e04c5c6ed31cf957` | `snapshots/major-2026-10-05-success/LDOP` |
| `src/Livox-SDK2` | `b27f2fa800cf945e1d1ccaa184d7b18d4c51912b` | `snapshots/major-2026-10-05-success/Livox-SDK2` |
| `src/cxr_ego_ctrl` | `cf8f0bbc5cedf07e19d78effa5afbeffe537721d` | `snapshots/major-2026-10-05-success/cxr_ego_ctrl` |
| `src/livox_ros_driver2` | `02c88582e59051c4641d2ed28a6f82253d0354e1` | `snapshots/major-2026-10-05-success/livox_ros_driver2` |
| `src/zyc_fuel_ws/src/drone_land` | `c1687bb958c38c679ff84c5d57a9ac27ad084534` | `snapshots/major-2026-10-05-success/zyc_fuel_ws--src--drone_land` |
| `src/zyc_fuel_ws/src/offb_takeoff` | `da23866710e431777251dcf593294b43e031e8ef` | `snapshots/major-2026-10-05-success/zyc_fuel_ws--src--offb_takeoff` |
| `src/zyc_fuel_ws/src/pose_to_odom_converter` | `1cc5c028be3ab5030b434d9786dbb7bfc219b55f` | `snapshots/major-2026-10-05-success/zyc_fuel_ws--src--pose_to_odom_converter` |
| `src/zyc_fuel_ws/src/vins_to_mavros` | `01efef76ec27bb5328d2dec33a6ca67aec64db49` | `snapshots/major-2026-10-05-success/zyc_fuel_ws--src--vins_to_mavros` |

## 同步与查阅

13个嵌套仓库的快照分支推送均因HTTPS缺少GitHub登录凭据失败，快照已保留在本机。
主仓库版本提交和标签也需成功推送后，才能从远端完整恢复该大版本。

本机查阅版本：

```bash
cd ~/match_ws
git show major-version-2026-10-05-success --stat
git show major-version-2026-10-05-success:docs/versions/major-2026-10-05-success.md
```

恢复到新机器时，需按上表恢复每个嵌套仓库至指定提交，并安装依赖、重新编译；实验数据另行保存。
