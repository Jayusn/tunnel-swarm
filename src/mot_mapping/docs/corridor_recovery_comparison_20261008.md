# 通道摆球漏检后续对照（2026-10-08）

本轮保持 5 Hz、普通目标三次真实观测、20 cm 最小物理直径。沿用 A04 的 5 秒静态预采、4 cm/50% 冻结背景、6 cm 位移与慢摆参数，对关联竞争、遮挡启动及形状比例 0.45 分别回放。两个新开关默认关闭；只有通过无球各阶段框数不增加、旧命中逐时刻保留的形状参数写入 `run_swarm.launch` 的 A04 入口。

## 数据与判据

- 动球输入：`.fapp_ball_debug/corridor_20261008_ball/evaluation.bag`，源为 11:13:37 录包。
- 无球输入：`.fapp_ball_debug/corridor_20261008_a04/evaluation.bag`，源为 10:41:58 录包。
- 旧静止输入：`.fapp_ball_debug/latest_20261005_115116/review.bag`。
- 旧下午输入：`.fapp_ball_debug/input.bag`；另一段：`.fapp_ball_debug/outside_suppression/latest_input.bag`。

每组使用独立 localhost ROS master、相同点云及里程计、1 倍速完整回放。无球包各阶段累计框观测均不得增加；框数是误检代理指标，没有据此声称真实误检率。新球包按冻结的状态中心距离小于 0.25 m、最大框边小于 0.65 m 判据比较；旧 159 个验收时刻沿用原框中心距离小于 0.25 m 且目标表面点保留不少于 max(3,50%) 的判据。缺输出计漏检，新增命中不抵消原有命中丢失。

新球包标注为局部空间候选筛查，尚不是人工逐帧真值：113 个时刻有至少六个目标表面点，其中 27 个时刻及前两次输入都有足够点。这个子集并非连续 27 帧，也不能替代全程检测率。原有 49/113 和 19/27 命中冻结不变。旧动态 159 个验收时刻含 142 个已有命中（45+59+38），与“159 个命中”含义不同。

## 两项代码试验

`competition_enabled` 对应 `association/competition_enabled`，默认 false。只有连续观测且最近 0.6 秒实际发布过的轨迹同时有静态/运动候选竞争时，才修改排序。弱候选为已证实静态背景，或至少两帧历史背景支持且新点比例不足启动门槛的候选；弱候选增加 0.30 m 排序代价，密集点数相对上一实测的差异增加至多 0.20 m 代价。保留所有原配对、0.82 m 范围、连续优先和一对一匹配，不删除候选、不新增点数或方向门槛。

`occlusion_startup_enabled` 对应 `detection/occlusion_startup_enabled`，默认 false。复用两帧原始云与同期雷达位姿，在各历史雷达视角下以已有 0.25° 网格、3 cm 横向邻域、3 cm 深度容差核验前景遮挡。取两帧前景比例的较小值提供启动证据。仍需三次实测、配置的位移/速度/新点比例，且不能由已知静态背景解释；缺历史或只一帧支持不启动。它不会自动启用旧的深度新点判据或严格两帧路径。

形状比例试验只把既有 `shape_consistency_fraction` 从 0.50 改成 0.45，不修改点簇、轨迹确认或稀疏六点旁路。恢复的是 93.442 s 处形状匹配约 45.45% 的候选。

## 完整对照结果

| 配置 | 球包可观测时刻 | 球包三帧子集 | 无球累计框 | 无球验收 |
|---|---:|---:|---:|---|
| A04 基线 | 49/113 | 19/27 | 333 | 比较基准 |
| 关联竞争最终版 | 54/113 | 23/27 | 341 | 未通过 |
| 仅遮挡启动 | 55/113 | 20/27 | 340 | 未通过 |
| 仅形状 0.45 | 50/113 | 20/27 | 333 | 通过 |
| 关联竞争＋形状 0.45 | 55/113 | 24/27 | 341 | 未通过 |
| 三项组合 | 59/113 | 24/27 | 349 | 未通过 |

上述球包组的原有 49 和 19 个命中均保留，113 个可观测标注没有缺输出。三项组合也仅达到子集 88.89%，且增加静态输出；未通过双 90% 目标。早期关联版本曾达到 57/113，但丢失旧慢摆 3 个命中及另一段 1 个命中，因此没有采用；表格使用最终关联版本，避免混用版本数字。

| 无球配置 | 预采 | 初始静止 | 拿入并走廊移动 | 返回 | 输出时刻 |
|---|---:|---:|---:|---:|---:|
| 基线/形状 0.45 | 0 | 37 | 247 | 49 | 292 |
| 关联竞争/竞争＋形状 | 0 | 34 | 252 | 55 | 292 |
| 遮挡启动 | 0 | 39 | 251 | 50 | 292 |
| 三项组合 | 0 | 36 | 256 | 57 | 292 |

只有形状参数同时保留每阶段结果；“总数更低但某阶段更高”的早期竞争版本也没有通过。仅形状 0.45 的旧包结果为静止 **6/467**、快摆 **45/53**、慢摆 **59/59**、另一段 **38/47**，142 个原有命中损失为零。另一段 276.726 s 的原有缺输出仍存在；下午包回放少三个非验收尾部输出，112 个动态验收时刻都有输出。

全部 113 个标注的首个命中均为 87.041 s，最长连续漏检仍为八个标注、跨度 3.600 s。三帧子集首检基线/形状为 88.940 s，竞争为 87.541 s。子集最长遗漏一个时刻不表示全程没有断框。

补跑的同二进制基线 `ball_basefinal` 仍为 49/113、19/27，形状方案新增的是 93.442 s 的命中；这处时刻及之前的输出均对齐。完整球包输出分别为 661/662，基线缺少一处非标注回调，返回阶段框数为 57/63，不能声称该包所有静止阶段都未增加，也不能把尾段差异全部归因于形状参数。无球 292 个输出时刻完全对齐的独立验收仍为 333/333；本轮只改善一处动态命中，没有降低新的无球包误检代理指标。

## 验证与产物

主工作空间 `catkin_make --pkg mot_mapping -j4 -l4` 和 `catkin_make --pkg diff_planner -j4 -l4` 通过。实际 `devel/lib/mot_mapping/mapping_node` 时间晚于修改的 C++/头文件；最终推荐方案、旧包验证与最终组合使用当前 devel 或同 SHA 的快照（SHA-256 为 `db2ea4a6f669f0b24e42d2a6246625e3a6df2af9bfd66ee1b0c66c08b48c9211`）。初筛基线和独立遮挡结果使用早期试验快照，由各 `*_job.json` 记录版本。默认独立 launch 和 A04 入口的 `roslaunch --dump-params` 已核对：两个新开关 false，A04 形状为 0.45、三帧确认。

默认 5 Hz/3 帧/8 cm、A04 5 Hz/3 帧/6 cm/形状 0.45、关联竞争合成回归通过，覆盖六点/20 cm 球面与平面、径向/慢摆/贴墙/换向、遮挡重现、无人机平移转动和四路输出。独立遮挡启动回归五组通过：双历史启动、关闭开关不启动、缺历史不启动、单历史不启动、零新点阈值仍拒绝未知深度。该独立测试使用单位姿态与零外参，真实运动位姿由既有 ego 回归和真实录包补充验证。

旧三包形状方案的阶段耗时之和 p95 分别为 78.921/76.481/79.803 ms，低于 200 ms。最终方案单独原速回放 `ball_shfinal`/`noball_shfinal` 的 p95 为 82.905/78.263 ms，峰值进程 RSS 为 130052/129164 KiB；最大阶段耗时之和为 106.915/101.114 ms。耗时是算法阶段日志之和，不含端到端 ROS 排队。四路状态、框、轨迹和动态云由逐时刻加载校验核对。

完整产物位于 `.fapp_ball_debug/corridor_recovery_20261008/`：`*_job.json` 保存完整节点/回放命令、二进制 SHA 和端口，`*_runtime_params.yaml` 保存实际参数，`*_metrics.json` 保存逐时刻命中、丢失/新增列表、各阶段框数、耗时和内存。中断的 `latest_shape45` 和 `ball_all_v3` 缺少完成标记，不用作验收；重新完成的 `latest_sh45r2`、`ball_allfinal`、`ball_basefinal`、`ball_shfinal`、`noball_shfinal` 用于最终统计。

## 当前运行配置

`run_swarm.launch` 已在 A04 include 中传入形状 0.45；两个新试验开关、旧参考启动和严格两帧均关闭。其他规划/控制配置保持用户当前内容。独立 `uav1_detection.launch` 默认不变。独立复现采用：

```bash
source /home/oem/match_ws/devel/setup.bash
roslaunch mot_mapping uav1_detection.launch \
  input_rate:=5 motion_confirmation_frames:=3 motion_displacement:=0.06 \
  min_motion_speed:=0.14 min_motion_novel_fraction:=0.26 tracked_motion_novel_fraction:=0.20 \
  background_consistency_enabled:=true background_consistency_post_association:=false \
  background_local_planes_enabled:=true background_local_post_association:=true \
  background_consistency_fraction:=0.55 \
  shape_consistency_enabled:=true shape_consistency_distance:=0.05 shape_consistency_fraction:=0.45 \
  background_reference_enabled:=true background_reference_capture_duration:=5.0 \
  background_reference_distance:=0.04 background_reference_fraction:=0.5 \
  reference_motion_enabled:=false reference_strong_two_frame_enabled:=false \
  competition_enabled:=false occlusion_startup_enabled:=false
```

启动预采五秒期间保持通道静止。冻结参考只覆盖启动可见通道段；拐角、新墙面与转动时仍进入原检测逻辑，不能由此次结果推断全程零误检或飞行避障可靠性。剩余漏检包括球/吊绳竞争、旧地图启动证据不足、稀疏观测与重建历史、状态 EKF 偏离实测框；无球移动段仍有 247 框，说明新场景问题未解决。下一步应针对同一时间戳的静态采样竞争和运动体匹配进一步诊断，而不是直接把两个未通过开关打开。
