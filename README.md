# Tunnel Swarm：前机 UAV0 / CAV0 多机协同工程

本分支 `vehicle/cav0` 保存前机 `asus@192.168.230.212` 的 `/home/asus/match_ws`，归档日期 2026-11-09。后机独立保存在同仓库的 `vehicle/cav1`。本次保存前机当前源码、现场调参、启动脚本、运行模型和标定文件，不改变现有飞行逻辑。

## 工程组成

| 功能 | 目录 |
| --- | --- |
| 前机 FUEL 探索、出口检测和接力点 | `src/zyc_fuel_ws/src/FUEL` |
| 双机协同、坐标对齐与跟随 | `src/control`（包名 `exploration_control`） |
| Diff 规划与搜索降落 | `src/Diff-Planner` |
| AutoTrans NMPC、模型与参考桥接 | `src/AutoTrans-quadrotor-wind-mpc`、`src/autotrans_reference_bridge` |
| MID360、FAST-LIO 与动态障碍感知 | `src/livox_ros_driver2`、`src/Livox-SDK2`、`src/FAST_LIO`、`src/LDOP`、`src/mot_mapping` |
| 相机检测、精降与目标上报 | `src/uav0_competition_bringup`、`src/precision_landing`、`src/color_tag_detector`、`src/qr_detector`、`src/uvc_ubuntu`、`src/target_reporting` |
| 前机启动入口 | `shfiles/` |
| 主目录标定归档 | `config/calibration/uav0/` |

NMPC 运行参数位于 `controller/payload_mpc_controller/config/{model,mpc}.yaml`，ACADO 模型及运行所需的 C/C++ 求解器源文件位于同包 `model/`，均随源码保存。前机目录未发现额外的 `.pt`、`.pth`、`.onnx` 或 TensorRT 模型；后机的裂缝检测模型不据此复制到前机。

已有现场修改原样保存：双机入口默认 ROS 地址 `10.227.10.212`；机体质量 `1.851 kg`；位置权重 `250`；固定悬停与起飞目标高度 `1.0 m`；`kf=1.926e-8`；`cutoff_freq_fq=0.0`。这些是归档时的实际参数，不是本次新增调参。

## 克隆与依赖源码

嵌套仓库仍以 Git 子模块保存，各依赖提交发布到本仓库的 `components/cav0/*` 分支；原始提交记录见 `docs/cav0-source-origins.json`。所有依赖分支先上传，再上传前机主分支。后机分支不参与本次推送。

```bash
git clone --branch vehicle/cav0 --recurse-submodules https://github.com/Jayusn/tunnel-swarm.git match_ws
cd match_ws
# 若之前没有递归克隆：
git submodule update --init --recursive
```

原来被忽略、但参与本机全工作空间编译的旧起降/位姿转换仓库及 FUEL 仿真源码一并保存；重复包的现有 `CATKIN_IGNORE` 原样保留。录屏、数据集、ROS bag、点云、运行日志、重复 ZIP、缓存和 build/devel/install 不纳入新增文件。上游仓库已有的文档资源和第三方依赖源码仍由各子模块历史保留。

## 环境与编译

验证环境：Ubuntu 20.04、ROS Noetic、GCC 10、CMake、catkin_make。依赖包括 MAVROS、RealSense ROS、PCL、OpenCV、Eigen、yaml-cpp、Livox-SDK2、UFOMap、liblzf、NLopt、TBB，以及各包 `package.xml` 声明的依赖。GUI 启动使用 Terminator。

本机已有的外部工具和依赖安装位置：

- GCC 10：`/home/asus/rh_ws/external/tools/gcc-10/usr/bin/`
- UFOMap：`/home/asus/rh_ws/external/install/ufomap/lib/cmake/ufomap`
- liblzf：`/home/asus/rh_ws/external/install/lzf/usr/lib/x86_64-linux-gnu/cmake/liblzf`
- Livox SDK 头文件：`/home/asus/.local/livox_sdk2/prefix/include`；本机静态库：`/home/asus/.local/livox_sdk2/prefix/lib/liblivox_lidar_sdk_static.a`。
- NLopt：`/home/asus/.local/rosdeps/nlopt/prefix/usr/lib/x86_64-linux-gnu/cmake/nlopt_cxx`。

这些外部安装目录不作为编译产物上传。迁移机器时先安装相应依赖，并替换下面路径。首次克隆后为 Livox 驱动生成 ROS1 包描述：

```bash
cp src/livox_ros_driver2/package_ROS1.xml src/livox_ros_driver2/package.xml
```

前机现有配置的全工作空间编译命令：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES= -j2 -l2
```

首次配置时还需传入当前构建缓存使用的参数（依赖路径按机器调整）：

```bash
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES= \
  -DROS_EDITION=ROS1 \
  -DCMAKE_C_COMPILER=/home/asus/rh_ws/external/tools/gcc-10/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER=/home/asus/rh_ws/external/tools/gcc-10/usr/bin/g++-10 \
  -DCMAKE_C_FLAGS=-fcommon \
  -DCMAKE_CXX_FLAGS=-I/home/asus/.local/livox_sdk2/prefix/include \
  -DLIVOX_LIDAR_SDK_LIBRARY=/home/asus/.local/livox_sdk2/prefix/lib/liblivox_lidar_sdk_static.a \
  -DNLopt_DIR=/home/asus/.local/rosdeps/nlopt/prefix/usr/lib/x86_64-linux-gnu/cmake/nlopt_cxx \
  -Dufomap_DIR=/home/asus/rh_ws/external/install/ufomap/lib/cmake/ufomap \
  -Dliblzf_DIR=/home/asus/rh_ws/external/install/lzf/usr/lib/x86_64-linux-gnu/cmake/liblzf
source devel/setup.bash
```

本机 `.bashrc` 同时把 `/home/asus/.local/rosdeps/nlopt/prefix/usr/lib/x86_64-linux-gnu` 加入 `LD_LIBRARY_PATH`。迁移时若使用相同的本地 NLopt 安装方式，也需配置运行时库路径。

## 标定与启动

先按 [前机标定说明](config/calibration/uav0/README.md) 恢复本机主目录文件。该归档保留原读取路径，不自动覆盖当前标定。

正式双机前机入口（在桌面终端运行）：

```bash
cd ~/match_ws
bash shfiles/start_uav0_dual_network_six_terminator.sh
```

当前启动脚本内默认 ROS 网络是 `10.227.10.212`，SSH 管理地址 `192.168.230.212` 不会自动改写 ROS 地址。若双机实际 ROS 通信使用 `192.168.230.*` 网段，可仅在启动时覆盖环境变量：

```bash
DUAL_UAV_ROS_MASTER_IP=192.168.230.212 UAV0_ROS_IP=192.168.230.212 \
  bash shfiles/start_uav0_dual_network_six_terminator.sh
```

后机应使用相同 ROS master 及其自身可达 ROS IP。入口提供 MAVROS、MID360、FAST-LIO、高频视觉位姿、检测/精降、FUEL/RViz、AutoTrans/logger 七屏；不自动解锁或切换 OFFBOARD。当前双机入口关闭颜色标签、二维码、热成像和目标上报，保留前/下视降落搜索链路，代码本身仍随工程保存。详细实飞操作见 `双机协同实飞测试流程.md`，启动与飞控操作沿用现有流程。

## 本次验证

2026-11-09，在前机现有依赖环境完成全工作空间 Release 编译（`catkin_make ... -j2 -l2`，返回码 0），覆盖 FUEL、Diff、NMPC、协同控制、感知与精降等模块。24 项已注册的非 `_ctest_` 离线测试全部通过。已从待提交内容导出独立源码副本，确认 60 个 ROS 包的名称和路径与本机一致；启动脚本通过 Bash 语法检查，四份标定 YAML 的 SHA-256 与原文件一致。完整构建及测试日志仅保存在本机 `/tmp/tunnel-swarm-cav0-build.log`、`/tmp/tunnel-swarm-cav0-tests.log`，不上传。

该验证是编译与离线检查，不代表执行过新的实飞验证；本次没有启动飞行节点。
