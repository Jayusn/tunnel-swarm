# FAPP perception integration

Source: https://github.com/arclab-hku/FAPP
Revision: dbd8173bd1518419b5e33faa7eecb75ed9d9cdcd (MIT).

Only perception and original EKF tracking are integrated. Input clouds must
already be registered in the odometry frame. No planner consumes these outputs.
ROS topics, frame IDs and timestamps are adapted for UAV1 FAST-LIO. Tracking
uses cloud timestamp intervals instead of the upstream fixed 0.02 seconds.

Launch: `roslaunch mot_mapping uav1_detection.launch`.

The six/seven Terminator entrypoints default to FAPP; `--detector ldop` selects
the existing LDOP detector. RViz groups are configured in the existing UAV1
profiles. Static point output and upstream planning are not enabled here.

Smoke test (after sourcing the workspace):
`ROS_MASTER_URI=http://127.0.0.1:11431 ROS_IP=127.0.0.1 python3 src/mot_mapping/test/perception_smoke.py`
The test starts and stops its own local ROS master and perception launch.

## 小障碍误检对照试验（2026-10-05）

目标最小物理直径为 20 cm；不按稀疏点簇的包围尺寸设置 20 cm 删除门槛。
新增两个启动时读取、默认关闭的独立开关：

- `geometry_enabled:=true`：设置 `association/geometry_enabled`，按上一实测距离、
  EKF 预测距离与密集点簇尺寸差排序配对，权重分别为 0.5、0.5、0.25。
- `depth_evidence_enabled:=true`：设置 `detection/depth_evidence_enabled`，以历史
  雷达视角的深度支持/前景遮挡修正新点比例，未知射线保留原判据。
  雷达原点使用 `/UAV1/mapping/extrinsic_T`；该参数缺省时采用当前 MID360
  配置的 `[-0.011, -0.02329, 0.04412]` 米。更换外参时需提供实际参数。

静止包累计框观测：基线 1135、仅关联 868、仅深度 1316、组合 925。
仅关联和组合在下午摆球包新增 39 个原有命中的损失；仅深度保留原有命中，
但增加静止杂框。三种试验均未满足“减少杂框且不新增动态漏检”，不推荐启用。
两项关闭时保持现有检测判据，主工作空间编译和默认感知回归通过。

20 cm 六点目标的径向运动、贴墙、换向和无人机移动样例独立核验四种组合。
完整回归可用 `FAPP_SMOKE_GEOMETRY`、`FAPP_SMOKE_DEPTH` 选择试验开关，
`FAPP_SMOKE_PORT` 指定独立 master 端口（同时设置对应 `ROS_MASTER_URI`）。
深度试验在完整回归的静态采样变化样例中出现误报，作为负向试验结果记录。
完整录包指标与原因分析保存在本地 `.fapp_ball_debug/association_depth/`。

## 频率、确认帧数与位移对照（2026-10-05）

运动确认帧数现在由启动参数 `detection/motion_confirmation_frames` 控制，
支持 3、4、5；同步控制连续观测历史的长度及确认所需次数。
位移仍为窗口内实测中心的最大两两距离，速度仍取实测相邻段速度的均值，
新点证据仍比较前两帧原始观测；改变确认帧数不会改变新点核验的历史长度。
默认独立 launch 保持 5 Hz、3 帧、8 cm，两项旧试验开关保持关闭。

复现其他组合（试验用途）：

```bash
roslaunch mot_mapping uav1_detection.launch input_rate:=10 motion_confirmation_frames:=5 motion_displacement:=0.08
```

本轮比较 5/10 Hz、3/4/5 帧、6/8/10 cm 的 18 种组合。
5 Hz 下 4 帧、5 帧减少静止框，但丢失原有快摆命中；3 帧、6 cm
保留已有命中并增加慢摆检出，却增加静止框。10 Hz 也出现原有命中损失。
没有组合同时通过冻结命中与静止杂框改善验收，保持默认值。

频率参数是限频设置，不是实际频率保证。原包有采样/录包空缺；
本轮 10 Hz 输入取真实原始云，并补回原始话题漏录但限频话题保留的真实帧，
不插值、不重复生成帧。独立矩阵使用冻结输入，实际 launch 的限频效果另测。
完整逐时刻结果、实际频率、缺输出及原因保存在 `.fapp_ball_debug/motion_sweep/`。

新增确认回归可按组合执行（自建 localhost ROS master，真实 5/10 Hz 发布）：

```bash
python3 src/mot_mapping/test/motion_confirmation_smoke.py --rate 10 --frames 5 --displacement 0.08 --port 11620
```

该回归检查首次确认、停止后撤销、20 cm 六点稀疏目标、径向运动、贴墙换向、
无人机移动和四路输出一致性。默认完整感知回归仍保留。
这些感知验证不等同于 FAPP 规划/控制避障验证；当前静态地图输出和规划接入
尚未启用，状态消息的不确定性字段尚未填充。

## 多帧背景与点簇形状核验（2026-10-06）

新增默认关闭的 `background_consistency_enabled`、`shape_consistency_enabled`。
前者核对实际历史宽平面，后者比较连续密集点簇的相对形状，稀疏六点仍采用
原运动判据。`background_consistency_post_association` 默认 true；有效组合
使用 false，在关联前剔除已证实的稳定背景。原默认 5 Hz/3 帧/8 cm 保持。

实际主工作空间回放：杂框优先组合静止框 1135→882（减少 22.29%）；
慢摆优先组合为 982（减少 13.48%），慢摆 52/59→59/59，恢复上次 3 处损失。
两组均覆盖静止全部 467 时刻，逐时刻保留全部 135 个原有动态命中。
20 cm 六点、平面小目标、慢速径向、贴墙换向和无人机运动回归通过。

新增 `tracked_motion_novel_fraction` 默认等于原新点比例，不改变默认行为；
试验中可仅为近期实际发布的连续轨迹放宽新点证据。更强平面剔除、5 点簇
以及 2.5 cm 背景容差出现原有命中损失，未采用。没有目标包围尺寸过滤。

完整启动命令、结果、失败原因和验证局限见
[对照报告](docs/background_comparison_20261006.md)。默认关闭新开关，旧 A/B
试验也保持关闭。有限录包的验收通过不等同于全场景零漏检或飞行避障验证。

## 继续减少杂框：历史局部双平面（2026-10-06）

本轮纯雷达对照以 982 框、142 个当前动态命中为基线，允许最多损失 4 个命中。
新增默认关闭的 `background_local_planes_enabled`：当历史邻域混合多个面、
整体 PCA 不满足平面条件时，最多分离两个宽平面，只提供背景证据，不裁掉
当前目标点。`background_local_post_association` 默认 true，仅靠新增局部面
判为背景时保留关联和运动记录，抑制当帧发布。

推荐原整体面关联前、新局部面关联后，`background_consistency_fraction:=0.55`，
其余沿用上轮慢摆优先参数：静止 **982→730 框（再减少 25.66%）**，全部
467 时刻齐全，快摆/慢摆/另一段仍为 **45/53、59/59、38/47**，142 个当前
命中逐时刻全部保留。50% 虽为 721 框，却丢失 8 个快摆命中，未推荐。

比例同时作用于原整体面和新局部面；默认仍为 80%，新开关仍关闭，频率仍为
5 Hz、3 帧确认。20 cm 六点目标继续保留，新增密集目标启动/遮挡重现、
两面固定墙与小目标同时存在及墙角采样继承轨迹回归。完整命令、全部 12 组
新对照、失败时刻和验证记录见[报告本轮部分](docs/background_comparison_20261006.md#本轮继续改善历史局部双平面核验)。

## 5 Hz 静态背景差分与动态重检（2026-10-06）

本轮将真实目标最小尺寸按 20 cm 回归，保持 5 Hz、3 帧确认。启动时可选采集 5 秒静态注册点云并冻结 KD 树：`background_reference_enabled` 负责静态背景抑制，`reference_motion_enabled` 负责背景差分运动启动证据和单帧断轨重检，两个开关默认关闭。

10 组静止/动态对照中，A-only 4 cm/50% 为静止代理指标最优：730→53 框、467/467 时刻、原有动态命中无损失；快摆仍为 45/53，另一段为 38/47，未达到动态检测率 90% 门槛。B 会增加静止框，A+B 4 cm 为 68 框并各恢复 1 个动态标注，但仍未通过双 90 验收，因此没有启用为默认配置。完整矩阵、诊断、最终单独回放和限制见[背景差分报告](docs/background_reference_comparison_20261006.md)。

## 预采集抑制与新场景行为（2026-10-06）

启用 `background_reference_enabled` 或 `reference_motion_enabled` 时，节点先采集 5 秒静态注册云。采集期间四路动态输出发送带时间戳的空消息，动态避障从背景冻结后的下一帧开始；冻结时清空预采阶段的轨迹和观测缓存。普通目标仍保持三次实际观测确认。

新增默认关闭的 `reference_strong_two_frame_enabled` 仅用于对照试验，且必须同时打开 `reference_motion_enabled`。它只对参考背景前景中满足连续两帧、间隔不超过 0.4 秒、位移至少 0.30 m、实测速率至少 0.50 m/s 和新点比例至少 50% 的候选生效，普通目标不会因此改成两帧确认。

参考树只冻结启动时看到的通道段。无人机移动到拐角或新场景后，新墙面进入正常点云和三帧确认流程，不会被旧背景树直接删除；新场景静态碎片仍可能在确认窗口产生短暂候选，需用拐角和转动录包单独验收。详见[背景差分对照报告](docs/background_reference_comparison_20261006.md)。
