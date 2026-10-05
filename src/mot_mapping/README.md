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
