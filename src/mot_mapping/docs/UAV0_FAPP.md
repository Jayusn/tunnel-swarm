# UAV0 FAPP 检测与 FUEL 动态避障

## 2026-10-22 当前同步版本

从后机 `vehicle/cav1` 的已提交版本 `2b3f65fa` 同步 FAPP 检测代码、参数、单元测试和近期实验报告；运动证据入口行为对应后机 `0f625c98`。本次仅更新 `mot_mapping`，不修改 FUEL、Diff、AutoTrans、航点、网络和控制器配置。

前机 `uav0_frame_event_verified.launch` 默认开启 `object_motion_evidence_enabled`，原有六/七分屏及 FUEL 航点入口通过现有 include 使用新检测。该路径根据连续三个有效观测中的两次移动匹配、静态匹配优势和深度证据确认运动，无需等待目标换向；保持当前后机阈值 `0.45 / 0.30 / 0.40`，并将额外连续性确认时长设为 `0.0`。双向局部匹配、候选拆分及候选关联隔离实验均保持关闭。

检测订阅和发布仍使用 UAV0：`/UAV0/fast_lio/cloud_registered`、`/UAV0/fast_lio/Odom_high_freq`、`/UAV0/fapp/states` 及原有点云/标记输出；坐标系为 `UAV0/camera_init`，雷达外参读取 `/UAV0/mapping/extrinsic_T`。共享 YAML 沿用 `uav1_frame_event_verified.yaml` 文件名，实际话题和坐标系由前机 launch 覆盖。

**前机原有动态避障设置保持原值**：启动脚本仍默认开启 FUEL 动态避障及规划优先；与后机六分屏默认只检测不同。原有 `--detection-only`、`--dynamic-avoidance`、`--strict-dynamic` 等开关继续生效。使用新检测会改变送入已开启 FUEL 动态代价的目标集合，未新增规划流程。

需要复现原摆动检测时可直接运行：

```bash
roslaunch mot_mapping uav0_frame_event_verified.launch object_motion_evidence_enabled:=false dynamic_mode:=swing-only
```

本次同步不代表实飞验收通过。已同步的历史报告保留了真实误报、慢摆漏检和处理延迟问题：历史球运动证据检出为 101/113；最新既有录包仅 204/819，尚未达到既有检出率及延迟目标。墙面场景的非空输出不能一概视为误报，完整审查见 `motion_truth_audit_20261016.md`。这些是后机历史录包结果，不是前机实测结果。

本次验证和备份目录：`/home/asus/match_ws_transfer_backups/fapp_20261022_115635`。构建更新前机主工作空间 `devel/lib/mot_mapping/mapping_node`；未启动 MAVROS、控制器或飞行流程。

### 本次验证结果

- 前机主工作空间使用 `catkin_make --pkg mot_mapping -j2 -l2` 构建成功，依赖消息目标同步检查；检测器不导出被其他包链接的实现，本次不需修改或重编 FUEL/Diff/控制器源码。
- `devel/lib/mot_mapping/mapping_node` 的生成时间晚于本次同步及相关源码时间，确认实际启动入口使用的新产物。
- 7 项 `mot_mapping` CTest 全部通过；6 组 launch 解析覆盖当前默认、general、原检测回退、FUEL 探索和手填航点入口，核对前机话题、坐标系与实验开关。
- 两组独立 localhost ROS master 合成输出测试通过：运动证据覆盖直线/双目标、换向、停止、重获、静止抖动和四路输出一致性；原摆动回归覆盖换向门槛、停止、重获及稀疏形状拒绝。
- 运动证据合成直线首次确认约 0.252 秒，2 秒周期摆动命中 34/40 帧；6 秒周期慢摆 0/70，稀疏输入首次确认约 0.653 秒。慢摆和稀疏输入限制仍保留，没有为通过同步验证调参。

## 2026-10-15 原同步记录（历史版本）

2026-10-15 从后机已提交的 `b8da5304` 同步少误报摆动检测，保留前机 FUEL 探索、AutoTrans 控制和降落流程。检测节点的输出命名空间、默认坐标系及雷达外参参数路径改为 UAV0；检测算法不变。

在前机使用原入口启动，默认开启 FAPP 检测和动态避障，模式为 `swing-only`：

```bash
bash /home/asus/match_ws/shfiles/start_uav0_dual_network_six_terminator.sh
# 通用动态检测 + 避障
bash /home/asus/match_ws/shfiles/start_uav0_dual_network_six_terminator.sh --dynamic-mode general
# 只检测，不让 FAPP 参与 FUEL 动态避障
bash /home/asus/match_ws/shfiles/start_uav0_dual_network_six_terminator.sh --detection-only
# 明确开启摆动专用检测 + 避障
bash /home/asus/match_ws/shfiles/start_uav0_dual_network_six_terminator.sh --dynamic-mode swing-only --dynamic-avoidance
```

`start_uav0_six_terminator.sh` 和 `start_uav0_first_seven_terminator.sh` 同样接受上述开关；开关在启动时生效，切换需要重启相应流程。也可用环境变量 `UAV0_FAPP_DYNAMIC_MODE` 和 `UAV0_FAPP_DYNAMIC_AVOIDANCE` 设置默认值。

FAPP 使用 `/UAV0/fast_lio/cloud_registered`、`/UAV0/fast_lio/Odom_high_freq`，在 `UAV0/camera_init` 坐标系发布 `/UAV0/fapp/states`、`dynamic_cloud`、`dynamic_object_markers`、`dynamic_track_markers`。FUEL 订阅本机状态，使用后机的实测尺寸、速度预测、0.8 秒观测保留期和 1 秒预测上限。检测原始点云仍进入 FUEL 静态地图。

动态目标参与 FUEL B 样条优化代价、发布前校验和执行期碰撞检查；直线路径遇到目标时进入原有轨迹优化。下穿、左右侧绕三个几何候选使用目标尺寸和前机已有静态路径校验，最终轨迹仍走 FUEL 原有发布流程。候选均不可行时返回原有规划失败路径，没有新增状态机或恢复策略。此处是 FUEL 适配，不能将后机 Diff 的轨迹表现直接等同于前机结果。

`--detection-only` 保留 FAPP 输出，关闭其对 FUEL 的订阅、动态代价和碰撞判定。FAPP 开启时主 launch 默认不重复启动前机 LDOP。底层 launch 的 `enable_fapp_detection` 和 `enable_fapp_dynamic_avoidance` 默认仍为 false，原脚本显式传入 true；主流程的探索目标、网络地址、控制器配置保持原值。

少误报版本采用 0.4 秒拟合窗口、2 厘米残差和稀疏形状检查，未包含后续静态平面实验。后机原始录包验证为三个负样本零误报、走廊有效标注 13/113；存在明显漏检，不能把同步完成理解为实飞验收完成。通用模式来自已提交的配置，未夹带后机尚未提交的 0.5 秒连续性实验。

## 验证

前机主 `/home/asus/match_ws/devel` 编译 `mot_mapping`、`obj_state_msgs`、`bspline_opt`、`plan_manage`、`exploration_manager`、`diff_planner` 及依赖。6 项 CTest 通过，包含复用的动态几何用例、B 样条空间/时间梯度校验和检测单元测试。

独立 localhost ROS master 下验证 UAV0 检测四路输出、多目标隔离、停摆/重获、稀疏形状；FUEL 验证订阅、发布前/执行中碰撞检查、下穿、侧绕、完全阻塞、关闭避障基线一致、空帧和过期观测。四种 launch 参数组合解析通过。未启动 MAVROS、控制器或实际飞行流程。

复测检测：`python3 src/mot_mapping/test/uav0_swing_output_smoke.py`。
FUEL 联调用例为 `plan_manage` 包的 `fapp_fuel_smoke`，应在独立 localhost ROS master 中运行。测试用例会在自身私有命名空间设置合成地图与规划参数。

原文件备份和验证日志：`/home/asus/match_ws_transfer_backups/fapp_20261015/`。

## 规划优先开关

原启动脚本默认开启规划优先，无需追加参数；追加 `--strict-dynamic` 恢复动态硬检查，追加 `--planner-priority` 可明确开启。环境变量 `UAV0_FAPP_PLANNER_PRIORITY` 可覆盖脚本默认值。直接调用 launch 或节点时，`fapp_planner_priority`、`fapp/planner_priority` 的底层默认值仍为 `false`。

开启后，FAPP 仍参与 B 样条优化；直线路径遇到动态目标仍进入优化，但动态预测不再强制要求下穿/侧绕引导候选，也不再单独否决发布和执行。固定 XY 的垂直轨迹保持原有几何约束。原有静态地图、动力学和轨迹交接检查继续生效。`--detection-only` 仍关闭 FAPP 动态代价。

该模式允许轨迹与预测目标重叠，真实动态障碍也可能被放行；规划自身失败时不会强制发布。开关作用于本机 FUEL，不改变出口后独立 Diff 的配置。

扩展的 `fapp_fuel_smoke` 在独立 localhost ROS master 下验证开关关闭时原有行为、开启后的动态放行、动态优化代价仍有效，以及静态占据仍阻止发布和执行。未进行实飞验证。
