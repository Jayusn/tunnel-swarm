# UAV0 FAPP 检测与 FUEL 动态避障

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
