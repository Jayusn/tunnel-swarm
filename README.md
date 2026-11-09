# Tunnel Swarm：多机协同与桥梁巡检工作空间

本仓库保存本机 UAV1 的 ROS Noetic 工程，包含多机探索接力、Diff-Planner 规划、AutoTrans NMPC 控制、桥梁巡检、裂缝/热源/二维码检测、目标上报和视觉精降。当前快照日期为 2026-11-09，工作分支为 `vehicle/cav1`。

本机同时保留前机角色源码和启动入口，详见 [前后机角色切换](docs/前后机角色切换.md) 和 [前机源码来源记录](docs/uav0_role_source.json)。这是本机工作空间快照，不代表对 UAV0 实机当前目录的重新同步。

## 工程范围

包含源码、launch、配置、运行模型、启动脚本、已有技术文档，以及从本机主目录归档的标定 YAML。已有现场调参和未提交代码一并保存。录屏、采集图片、训练数据集、ROS bag、运行日志、build/devel/install、临时隔离工作空间和重复 ZIP 备份不纳入本次上传。

| 功能 | 主要目录 |
| --- | --- |
| 多机接力、跟随和坐标对齐 | `src/control`（ROS 包名 `exploration_control`） |
| Diff 规划与轨迹优化 | `src/Diff-Planner/src/diff_planner` |
| FUEL 前机探索 | `src/zyc_fuel_ws/src/FUEL` |
| NMPC 与参考桥接 | `src/AutoTrans-quadrotor-wind-mpc`、`src/autotrans_reference_bridge` |
| MID360、FAST-LIO | `src/livox_ros_driver2`、`src/FAST_LIO` |
| 动态障碍感知 | `src/LDOP`、`src/mot_mapping` |
| 桥梁任务 | `src/uav1_bridge_inspection` |
| 裂缝检测与模型 | `src/yolo_fastest_ros/ncnn`（ROS 包名 `yolo_detector`） |
| 检测、融合与上报 | `src/color_tag_detector`、`src/qr_detector`、`src/uvc_ubuntu`、`src/target_reporting` |
| 精降与前后机检测入口 | `src/precision_landing`、`src/uav0_competition_bringup` |
| 主目录标定归档 | `config/calibration/uav1` |

## 环境和构建

当前环境使用 Ubuntu 20.04 / ROS Noetic、GCC 10、CMake、catkin_make。运行依赖包含 MAVROS、RealSense ROS、OpenCV、PCL、Eigen、Terminator，以及各包 `package.xml` 声明的 ROS/Python 依赖。硬件串口、相机设备和网络地址需与目标机器一致。

本机主工作空间采用 GCC 10、C++20、`-fcommon` 和 Release 构建，部分包单独指定 C++14。依赖中的 UFOMap 2.0、liblzf 和 NCNN 在本机独立安装，其编译产物不随工程上传。当前 CMake 使用的位置如下；迁移时先安装依赖，并把路径替换为实际安装位置：

- UFOMap：`/home/oem/ldop_ws/external/install/ufomap/lib/cmake/ufomap`
- liblzf：`/home/oem/.local/usr/lib/x86_64-linux-gnu/cmake/liblzf`
- NCNN：`src/yolo_fastest_ros/ncnn/install/lib/cmake/ncnn`
- 编译器：`/home/oem/.local/gcc-10/usr/bin/gcc-10`、`g++-10`

当前 LDOP 节点目标未继承 UFOMap 头文件目录，需要在构建环境设置 `CPLUS_INCLUDE_PATH`（如下）；此处仅补充构建环境，不修改节点代码。

已配置的本机工作空间重新编译：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
export CPLUS_INCLUDE_PATH="/home/oem/ldop_ws/external/install/ufomap/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
catkin_make -j2 -l2
source devel/setup.bash
```

首次克隆后，先生成 Livox 的 ROS1 包描述：

```bash
cp src/livox_ros_driver2/package_ROS1.xml src/livox_ros_driver2/package.xml
```

Livox-SDK2 需按 `src/Livox-SDK2/README.md` 编译安装。原有 `.gitignore` 排除的 FUEL 备用仿真目录、旧起降/位姿转换工程未启用，也未纳入本次归档。

首次构建应保留当前使用的包白名单，避免把历史备用工程一起编译。在依赖准备完成后，可按下面的实际构建配置设置（路径按部署机器调整）：

```bash
source /opt/ros/noetic/setup.bash
export CPLUS_INCLUDE_PATH="/home/oem/ldop_ws/external/install/ufomap/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
catkin_make -j2 -l2 \
  -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 \
  -DCMAKE_C_COMPILER=/home/oem/.local/gcc-10/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER=/home/oem/.local/gcc-10/usr/bin/g++-10 \
  -DCMAKE_CXX_STANDARD=20 -DCMAKE_C_FLAGS=-fcommon \
  -Dufomap_DIR=/home/oem/ldop_ws/external/install/ufomap/lib/cmake/ufomap \
  -Dliblzf_DIR=/home/oem/.local/usr/lib/x86_64-linux-gnu/cmake/liblzf \
  -Dncnn_DIR="$PWD/src/yolo_fastest_ros/ncnn/install/lib/cmake/ncnn" \
  -DCATKIN_WHITELIST_PACKAGES="active_perception;autotrans_reference_bridge;bspline;bspline_opt;cmake_utils;color_tag_detector;cxr_ego_ctrl;diff_planner;exploration_control;exploration_manager;fast_lio;fuel_path_searching;fuel_plan_env;fuel_traj_utils;ldop;livox_ros_driver2;lkh_tsp_solver;mot_mapping;obj_state_msgs;odom_visualization;path_searching;payload_mpc_controller;plan_env;plan_manage;poly_traj;pose_utils;precision_landing;qr_detector;quadrotor_msgs;target_reporting;traj_opt;traj_utils;uav0_competition_bringup;uav1_inplace_landing_test;uav_utils;uvc_ubuntu;waypoint_generator;target_ros_msgs;yolo_detector;uav1_bridge_inspection"
```

`shfiles/run_uav1_ldop.sh` 还使用 `.ldop_only_ws_gcc10` 和 `~/ldop_ws/ldop_env.sh`；`shfiles/start_uav1_ldop_camera.sh` 使用 `~/rh_ws`。这些是可选 LDOP 入口的外部环境，不是克隆仓库后就存在的编译目录。启用这些入口需按其脚本准备对应环境；普通工程源码仍位于 `src/LDOP`。

## 标定恢复

参见 [标定归档说明](config/calibration/uav1/README.md)。主目录文件原样复制并记录 SHA-256，未修改当前运行文件或 launch 的读取路径。

在新的 UAV1 部署目录恢复相机内参及当前检测入口需要的外参：

```bash
mkdir -p "$HOME/.ros/camera_info" "$HOME/handeye_calibration"
cp -i config/calibration/uav1/camera_info/down_camera.yaml "$HOME/.ros/camera_info/"
cp -i config/calibration/uav1/handeye_calibration/*.yaml "$HOME/handeye_calibration/"
```

这些是 UAV1 本机标定，不应直接当作另一台飞机的标定。仓库中的热成像/D435融合、雷达外参、下视外参和降落中心补偿也应随对应机体安装状态使用。

## 启动入口

以下入口在 Ubuntu 图形桌面终端中运行，执行前先完成构建与标定恢复。不同任务任选对应入口，避免同时启动重复节点。

```bash
cd ~/match_ws
# 多机后机：连接前机 ROS master，执行探索接力跟随。
bash shfiles/start_uav1_six_terminator.sh
# 本机切换前机角色：提供本机 master，运行前机探索流程。
bash shfiles/start_uav0_dual_network_six_terminator.sh
# UAV1 单机 Diff 航点与 AutoTrans。
bash shfiles/start_uav1_seven_terminator.sh
# UAV1 桥梁巡检、检测和精降。
bash shfiles/start_uav1_bridge_inspection.sh
```

默认网络配置并不相同：多机后机是 `10.59.40.233`，连接 `10.59.40.213:11311`；本机前机角色默认使用 `192.168.43.21` 提供 master。当前单机七分屏默认 `10.59.40.233`，桥梁入口默认 `192.168.230.232`。多机环境可通过 `DUAL_UAV_ROS_MASTER_IP`、`UAV0_ROS_IP`、`UAV1_ROS_IP` 覆盖；单机和桥梁可通过 `UAV1_SINGLE_ROS_IP`、`UAV1_SINGLE_ROS_MASTER_IP` 覆盖；在终端查询话题时也应设置一致的 `ROS_IP`、`ROS_MASTER_URI`。仓库不自动恢复或修改用户 `.bashrc`。

桥梁详细操作见 [桥梁巡检说明](src/uav1_bridge_inspection/README.md)。航点使用 FAST-LIO 的 `UAV1/camera_init` 世界系绝对坐标；地面 Z、降落 XY 位于 `src/uav1_bridge_inspection/launch/bridge.launch`，航点位于 `config/route.yaml`。当前检测使用 `yolo_bridge_neg_20261107.yaml` 和对应 `.bin/.param`，置信度阈值为 0.90。检测证据输出到 `~/match_ws/target_reports/bridge`。

## 本次保存的已有运行改动

相对于上一个提交，本快照保留入口门点停稳扫描、出口到达后解除通道规划限高的代码及测试，以及当前 MPC/质量参数、桥梁航点/降落点、裂缝阈值和单机网络设置。本次归档本身只增加文档和标定备份，并按用户确认更新角色测试的旧网络断言，不进一步修改这些运行逻辑。

当前参数包括：机体质量 1.936 kg、MPC 速度权重 55、起飞/固定悬停 Z 为 2.0、外力窗口 30、f_Q 低通截止频率配置为 0；后机通道规划中心 Z 上限 1.35 m，按既有到达规则抵达出口后解除；桥梁降落点 XY 为 (16.10, 0.60)。这些是当前现场配置，迁移或换场地时以对应 YAML/launch 和实际测量为准。

## 验证

不启动飞控或传感器的配置检查：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 shfiles/tests/test_role_configuration.py
python3 src/uav1_bridge_inspection/test/test_startup.py
roslaunch --nodes uav1_bridge_inspection bridge.launch
```

编译和离线/合成测试结果见 [本次归档验证记录](docs/tunnel_swarm_upload_validation.md)。这些验证不等同于实际飞行验收。
