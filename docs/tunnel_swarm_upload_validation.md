# 2026-11-09 工程归档验证

## 范围

保存 UAV1 当前 `vehicle/cav1` 工程，包括多机协同、桥梁巡检、运行模型和当前未提交的源码/参数；新增根 README 与 5 份主目录标定归档。保留原有逻辑，仅经用户确认修正角色测试中后机网络默认值的过时断言。

没有连接或修改 UAV0 实机，没有启动飞控、相机、雷达或真实飞行任务。

## 构建

主工作空间 `/home/oem/match_ws` 使用现有完整 catkin 白名单执行构建，输出到实际启动入口使用的 `devel`。

首次构建在 LDOP 节点处因缺少 UFOMap 头文件搜索路径失败。头文件实际已安装，添加以下环境变量后同一主工作空间构建成功，未修改 LDOP 源码：

```bash
source /opt/ros/noetic/setup.bash
export CPLUS_INCLUDE_PATH="/home/oem/ldop_ws/external/install/ufomap/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
catkin_make -j2 -l2
```

涉及的包包括 `exploration_control`、`traj_opt` 及其规划依赖者 `diff_planner`，以及配置涉及的 `payload_mpc_controller`、`uav1_bridge_inspection`、`yolo_detector`；本次全白名单构建覆盖了这些包与前机目标。

已确认 `devel/lib/exploration_control/leader_safe_path_follower`、`devel/lib/libtraj_opt.so`、`devel/lib/diff_planner/diff_planner_node` 存在，时间不早于对应改动源码/头文件。构建有已有警告，不代表完成实飞验收。

## 检查

- 角色配置测试：4 项通过。首次失败来自旧 IP 断言，经用户明确同意更新断言后通过。
- 桥梁启动布局和真实 launch 解析测试：3 项通过；测试替换 Terminator，不启动硬件。
- `roslaunch --nodes uav1_bridge_inspection bridge.launch`：通过。
- 5 份标定文件：与主目录原文件逐字节相同，SHA-256 一致，YAML 可解析。
- 当前变动的 YAML 可解析；两套桥梁检测配置引用的模型文件存在。
- 单机七分屏脚本 `bash -n`：通过。

- 接力 FIFO/入口扫描回归：98 项通过，0 失败/错误。
- 轨迹优化回归：动态障碍 24 项、轨迹采样 8 项、悬空障碍下穿 11 项、垂直障碍侧向锁定 23 项，共 66 项通过。
- 合计上述自动化用例 171 项通过；不包含实际飞行验证。

回归命令（source 主工作空间后，使用独立本地测试 master）：

```bash
export ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11471
catkin_make -j2 -l2 run_tests_exploration_control_rostest_test_fifo_entry_spacing.test run_tests_traj_opt
```

## 网络

`git ls-remote myrepo` 返回 `gnutls_handshake() failed: The TLS connection was non-properly terminated.`。未绕过 TLS 校验，未改写远程历史。本记录中的本地验证不能视为已上传成功。
