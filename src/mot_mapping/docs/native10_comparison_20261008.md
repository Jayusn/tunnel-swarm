# 原生 10 Hz 小障碍检测与点簇分离对照（2026-10-08）

本轮按用户后续要求改用 FAST-LIO 原生 10 Hz 实测点云，并允许从真实点簇中分离球面与吊绳。保持类别无关检测、20 cm 最小物理目标、六点稀疏目标及当前观测发布要求。冻结标注不变，在线算法没有加入球的位置范围或 65 cm 尺寸截断。

**新球冻结样本命中已经超过 95%，联合验收仍未通过。** 最终候选 `final06` 达到 **111/113（98.23%）、27/27（100%）**，当前 50 个命中全部保留；同一候选静止包为 7 框/467 时刻，较 6 框基线增加 1 框。无球原生同频分段框数为 0/112/661/155，对比原生基线 0/109/607/96 全部增加，原 5 Hz 锚点为 0/41/253/61，对比基线 0/37/247/49 也增加。因此球样本已达 95%，但联合误检验收仍未通过，实验开关保持关闭。

> `final06` 的遮挡启动、无球、静止、旧下午段和另一段录包均已完成可复核回放；联合验收失败的配置不启用默认。下表只列已经取得 `PASS completed`、完整原速回放并生成 `*_metrics.json` 的结果。中断的 `ball_reacquire08` 不纳入验收。

## 冻结数据与指标

有球输入为 `.fapp_ball_debug/corridor_20261008_ball/evaluation.bag` 中的 `/UAV1/fast_lio/cloud_registered` 与 `/UAV1/fast_lio/Odom_high_freq`。原生云共有 1727 帧；原来限频云共有 663 帧，其实际平均频率约 3.83 Hz，间隔中位数约 298.8 ms。原生云间隔中位数约 100.0 ms、p95 约 101.6 ms。10 Hz 对照直接使用原始时间戳，没有复制点云补帧。

无球输入从 `/home/oem/bags/corridor_debug_2026-10-08-10-41-58.bag.active` 的完整块只读提取，保存为本轮 `noball_raw_input.bag`，含 763 帧原生云及 15262 条里程计。原始文件大小、修改时间不变，提取记录在 `noball_raw_input_meta.json`。先前无球 `evaluation.bag` 未保存实际原生云，不能将其中限频云当作原生 10 Hz 输入。

状态命中沿用冻结判据：同期状态位置距标注可见球面中心小于 25 cm，输出最大边长小于 65 cm。实测框命中使用 Marker 中心单独统计。65 cm 是离线判据，不是在线筛除或裁框门槛；20 cm 指真实物体尺寸，也不是点簇包围尺寸的最小值。

| 验收对象 | 固定分母与当前基线 | 本轮要求 |
| --- | --- | --- |
| 新有球录包，至少六个可分离球面点 | 113 时刻；当前 50 个状态命中 | 至少 108/113（95.58%），原有 50 命中逐个保留 |
| 上述数据中连续三帧可观测子集 | 27 时刻；当前 20 个状态命中 | 至少 26/27（96.30%），原有 20 命中逐个保留 |
| 旧动态冻结数据 | 快摆 45/53、慢摆 59/59、另一段 38/47 | 原有 142 命中无损失；缺输出计漏检 |
| 旧静止冻结数据 | 467 输出时刻、6 次累计框观测 | 覆盖全部 467 时刻，累计框不增加 |
| 无球移动录包的原 5 Hz 锚点 | 292 时刻、333 框；四阶段 0/37/247/49 | 各阶段框数不增加，所有锚点有输出 |
| 无球移动录包的原生 10 Hz 全帧 | 另跑原生基线，全段 812 框；四阶段 0/109/607/96 | 各阶段不增加，并单独核对原生基线输出覆盖 |
| 原生 10 Hz 处理耗时 | 实际整回调计时 | 最终候选单独运行 p95 小于 100 ms |

113 与 27 两个集合冻结于原来的标注，改用 10 Hz 后没有重新挑选分母。全部 113 标注时间戳都存在于原始云。三帧子集仅表示点云标注可观测，不保证算法已经把这些返回关联成同一轨迹。新增命中不能抵消旧命中损失。

**累计框观测是杂框代理指标，不是真实误检率。** 同一目标在多帧重复计数；无球录包中操作者、手臂等也可能实际移动。原生输入增加了观测次数，因此不能将原生全段 812 框直接与旧 333 框相减解释为误检变化。报告同时比较原生同频全帧和原 292 个锚点；少输出也不能作为减少杂框的合格收益。冻结空间标注没有覆盖整场景真值，95% 结论只能指这些冻结样本。

## 已完成的完整回放

每组在独立 localhost ROS master 下按 `rosbag play --clock -r 1` 回放。表中名称对应 `native10_20261008/<case>_<name>_metrics.json`。下列已完成有球数据的 113 个验收时刻均有输出，四路输出时间戳及数量核对通过。

| 配置名称 | 实现版本及实验参数 | 状态命中 /113 | 三帧子集 /27 | 旧 50 命中损失 | 无球原生累计框 | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 当前 5 Hz 基线 `shfinal` | A04，形状匹配比例 0.45 | 50（44.25%） | 20（74.07%） | 0 | 不适用；旧锚点 333 | 原有基线 |
| `nativebase` | 原算法，原生云，体素 5 cm，位置方差 0.09 | 94（83.19%） | 27（100%） | 2 | 812 | 频率收益明显；未通过 |
| `time6` | v1；仅增加 0.6 s 时间证据 | 94（83.19%） | 27（100%） | 2 | 896 | 无新增球命中且杂框增加；未通过 |
| `density05` | v1；聚类前段拆父簇，8 cm/70% 密度核，体素 5 cm，位置方差 0.01 | 100（88.50%） | 25（92.59%） | 4 | 1645 | 拆簇改变背景证据及关联竞争；未通过 |
| `allv1` | v1；聚类前段拆簇，体素 3 cm，核 10 cm/70%，时间 0.6 s，重检 1.6 s，全局/几何关联，平移启动比例 0.8，位置方差 0.01 | 101（89.38%） | 22（81.48%） | 4 | 2344 | 杂框、旧命中及耗时均恶化；未通过 |
| `publish08` | v2；父簇先通过原核验，仅在发布阶段分离，核 8 cm/70%，体素 5 cm，位置方差 0.01 | 104（92.04%） | 27（100%） | 1 | 829 | 比前段拆簇稳定，但全部验收仍未通过 |
| `priority08` | v3；沿用 v2 分离，增加已发布轨迹 1.0 s 重检和同层关联竞争，其他新旗关闭 | 111（98.23%） | 27（100%） | 0 | 907 | 球样本 95% 通过；无球框数增加，联合验收未通过 |
| `gapshape08` | v4；同 v3，既有形状核验覆盖短断轨重检，8 点/15°/0.45 不变 | 111（98.23%） | 27（100%） | 0 | 888 | 无球仍增加，且旧动态回归失败 |
| `depthnovel4` | v4；同 `gapshape08`，旧深度 B 开启，新目标新点比例 0.40，已发布轨迹仍 0.20 | 111（98.23%） | 27（100%） | 0 | 875 | 比 v4 少 13 框，仍未通过无球分段门槛 |
| `bg3` | v4；同 `gapshape08`，历史局部背景支持由 4 帧改为 3 帧 | 111（98.23%） | 27（100%） | 0 | 895 | 部分阶段减少，联合验收未通过 |
| `geometry_v5` | v5；动态云恢复完整确认父簇，同 v4 分离框；几何关联开启 | 111（98.23%） | 27（100%） | 0 | 未运行 | 旧快/慢摆为 44/53、19/59，43 个旧命中损失，淘汰 |
| `final06` | v6；发布阶段点簇分离、完整父簇动态云、遮挡启动、0.6 s 重检，几何/时间/深度/全局关联关闭 | 111（98.23%） | 27（100%） | 0 | 928 | 球样本通过；无球分段、静止包和旧快摆联合不通过 |

`nativebase` 与 `time6` 的实测框命中为 96/113，状态命中为 94/113；`publish08` 两者均为 104/113，`priority08` 两者均为 111/113。不能用框命中替代状态命中。`priority08` 相对当前基线新增 61 个命中且无旧命中损失，全部 113 冻结时刻都有输出。首个有效样本命中仍在录包约 87.041 s，最长漏检为一个有效样本；三帧子集没有漏检。这里的“单个样本”不意味着整段仅有 0.1 s 漏检，也不覆盖点数不足的时刻；首次命中也不等于物体进入视野后的完整首检延迟。

旧静止包 `static_priority08` 以其原有真实限频输入完整原速回放，没有插帧模拟 10 Hz，基线为 **6 框/467 冻结输出时刻**。最终候选 `static_final06` 为 **7 框/467**，四路一致，冻结时刻均有输出，整回调 p95 为 85.31 ms；因此静止包也未通过“不增加框数”。新增框来自开启 `occlusion_startup` 的 6 点候选（录包相对约 105 s，最终中心约 `(2.741, 2.175, -0.111)` m，尺寸约 `(0.436, 0.138, 0.026)` m）；这是后续优先回归该开关的具体失败案例。

### 无球分段与输出覆盖

阶段依次为预采、初始静止、手持走通道、返回。原始锚点分母依次为 19、74、115、84，共 292。

| 配置 | 原生全帧分段框数 | 原 292 锚点分段框数 | 缺原锚点数 | 同频分段及旧锚点分段验收 |
| --- | --- | --- | ---: | --- |
| 旧 5 Hz | 不适用 | 0/37/247/49 | 0 | 基线 |
| `nativebase` | 0/109/607/96 | 0/41/231/43 | 0 | 初始静止锚点 41>37，未通过 |
| `time6` | 0/111/685/100 | 0/42/263/44 | 0 | 未通过 |
| `density05` | 0/224/1144/277 | 0/85/434/117 | 6 | 未通过；少输出未计作改善 |
| `allv1` | 0/227/1835/282 | 0/94/708/109 | 3 | 未通过；少输出未计作改善 |
| `publish08` | 0/109/611/109 | 0/41/231/48 | 0 | 原生通道、返回增加；旧静止锚点增加，未通过 |
| `priority08` | 0/113/672/122 | 0/42/255/52 | 0 | 两种时间口径三个非预采阶段均增加，未通过 |
| `gapshape08` | 0/113/670/105 | 0/42/254/43 | 0 | 原生三个阶段增加，旧静止/通道增加，未通过 |
| `depthnovel4` | 0/114/660/101 | 0/44/253/38 | 0 | 原生三个阶段增加，旧静止/通道增加，未通过 |
| `bg3` | 0/113/660/122 | 0/42/246/52 | 0 | 原生三个阶段增加，旧静止/返回增加，未通过 |

原生基线输出 759/763，`publish08` 输出 760/763，`priority08` 无球输出 761/763；`final06` 无球输出 760/763。`final06` 相对原生基线的无球分段框数增加 3/54/59，原 5 Hz 锚点增加 4/6/12；缺输出为 1 个回调没有发布、2 个源帧未进入回调，四路输出一致。球 `final06` 输出 1714/1727，113 个冻结验收时刻全部有输出；整段有 1 个回调无输出、12 个源帧未进入回调，其中缺口不落在 113 个验收时刻。缺输出详情在各组 `audit.input_without_output`、`source_without_callback_stamps`、`callback_without_output_stamps` 中；初始化及个别输入不出结果需与漏检统计分开报告，不能把空缺当零框。

### 实际耗时与内存

| 配置 | 有球整回调 p95 /ms | 无球整回调 p95 /ms | 有球订阅端总延迟 p95 /ms | 无球订阅端总延迟 p95 /ms | 有球/无球峰值 RSS /MiB |
| --- | ---: | ---: | ---: | ---: | ---: |
| `nativebase` | 未测量 | 未测量 | 144.54 | 139.07 | 158.91/148.62 |
| `time6` | 88.75 | 88.04 | 134.27 | 132.17 | 167.59/159.77 |
| `density05` | 130.74 | 118.36 | 811.68 | 468.31 | 163.16/152.12 |
| `allv1` | 142.05 | 113.90 | 883.10 | 517.09 | 165.86/161.22 |
| `publish08` | 101.11 | 100.97 | 152.49 | 145.61 | 158.24/151.20 |
| `priority08` | 77.59 | 71.59 | 121.03 | 113.29 | 158.13/150.56 |
| `gapshape08` | 77.10 | 72.55 | 119.51 | 116.41 | 159.73/149.38 |
| `depthnovel4` | 85.82 | 81.02 | 129.68 | 124.93 | 157.89/148.59 |
| `bg3` | 77.62 | 71.32 | 122.29 | 112.21 | 159.79/149.73 |
| `geometry_v5` | 80.13 | 未运行 | 124.73 | 未运行 | 158.95/未运行 |

新增 `FAPP_CALLBACK_TIME <输入时间戳纳秒> <毫秒>` 以 steady clock 测量整个点云回调，包含反序列化、初始化/提前返回及结束清理。旧二进制没有这个日志，其 Filter/Cluster/ikd-Tree/Tracking 的阶段耗时和仅供参考（有球/无球 p95 为 95.56/97.49 ms），不能宣称整回调已经通过。

订阅端总延迟从审核订阅器收到原始点云，到同一订阅器收到状态，使用单调墙钟；包括同步等待、ROS 调度和传输，不等于算法回调耗时。当前筛查有并行回放，CPU 负载及频率也会影响耗时，不将不同版本差值直接归因为某项算法加速。`final06` 单独回放的有球/无球整回调 p95 为 87.84/82.97 ms，低于 10 Hz 的 100 ms 周期；这只是录包回放测量，不等于实机闭环的延迟上限。

## 实现语义与当前失败原因

通用 `uav1_detection.launch` 默认使用 10 Hz、三帧、8 cm，实验开关仍默认关闭。`native_input_enabled=true` 时以 `topic_tools relay` 转发原始点云，保持原有 `/UAV1/fapp/cloud_registered_throttled` 入口名称和消息接口；此时 `input_rate` 不执行限频，实际频率由上游决定。本数据 FAST-LIO 上游约 10 Hz。`run_swarm.launch` 的 FAPP include 使用 `input_rate=10.0` 和原生 relay，LDOP 独立参数仍为 5.0 Hz。launch 解析已验证，实验旗在通用入口和 A04 include 中均保持关闭；`final06` 只作为离线对照，不改变新增实验开关行为。

本轮实验参数含义：

- `clustering/voxel_size`：当前聚类云体素，默认 5 cm；3 cm 仅为对照，不能由离线球区域点数增长直接推断整场景收益。
- `clustering/density_split_enabled`：v2/v3/v4 在父簇已通过运动、背景、形状核验后，分离真实点集。核点要求局部邻居数量达到 `max(6, ceil(alpha×父簇最大邻居数))`，局部协方差中间/最大特征值至少 0.1，核点按 `core_radius` 连通。边界点只附着最近核点，不再次扩张；余点按现有 DBSCAN 参数分组。v2–v4 的框和动态云仅发布含原密集中心点的分量；没有可用核或不能真正分开时保持父簇。所有点索引互斥且完整保留于分量或余点，原注册云、背景历史不因分离而裁掉。**v5 保留该真实分量的最小/最大坐标框，但 `dynamic_cloud` 和用于地图动态点删除的 `deleted_indices` 恢复完整已确认父簇。** 因此动态云可包含真实吊绳和分量外稀疏表面，其点坐标可超出小框；该框描述分离出的实测主体，不是所有父簇点的包围盒。状态仍使用 EKF 位置、尺寸使用实际分量。此行为变化用于保留旧动态目标表面支持，没有把余点伪装成位于小框内。
- `detection/time_evidence_enabled`：将新点证据历史选择改为距当前约 0.3/0.6 s 的两幅不同原始云，历史雷达位姿与云配对；误差容许 0.15 s。轨迹观察历史用 `motion_window` 秒并至少保留配置的三次真实观测。保留实测速率和位移门槛，不取消三次观测确认。
- `detection/confirmed_reacquisition_enabled`：仅对近期实际发布且实际观测间隔在 `reacquisition_max_gap` 内的轨迹保留短断轨历史。重现时仍需当前候选、实测运动及新点比例。v3 同时让这类近期已发布轨迹按距离与连续轨迹竞争，避免未发布墙片仅凭连续性占用球候选；断轨不满足条件时清空旧运动确认。v3 `priority08` 已完成有球、无球及旧静止对照：球样本改善，但无球杂框增加，仍为实验开关。
- `association/global_assignment_enabled`：两阶段一对一 Hungarian 分配，在现有 0.82 m 可配对图内先最大化配对数再减小代价；可与现有 `geometry_enabled` 独立评估。默认继续原贪心分配。
- `tracking/position_measurement_variance`：EKF 位置测量方差，默认 0.09 m²；0.01 m² 增加当前实测位置权重，也可能增大输出抖动，必须回归。
- `detection/translated_startup_enabled`：依赖时间证据和真实轨迹中心，在两幅历史云中比较“按实测平移的密集相对点”与“世界坐标不动的点”。两帧移动支持均达到配置比例，且比静态支持高至少 0.25，才补充启动证据；仍要求三次实测、位移、速率、新点和非背景候选。没有固定位置或球类别限制。

已有 `dbscan/tolerance=0.02` 在其 nanoflann 路线表示**平方距离 m²**，实际半径约 14.14 cm；新增 `core_radius=0.08` 是线性 8 cm。两者单位不同，不能按同一种半径解释。

早期 v1 在原聚类后、背景与关联前拆分，球命中改善却使原静态墙簇分裂成更多可竞争候选，无球全帧框数增至 1645/2344。v2 将分离后移到发布，保持父簇原有核验，降回 829；因此不继续推荐 v1 前段拆簇。

原生基线剩余 19 个漏检中，日志诊断分为：9 个球附近父簇最大边长约 0.75–1.12 m，4 个短断轨后只有两次观测，2 个实测框命中但 EKF 状态偏差超过判据，2 个当前实测中心已经偏离球面，2 个启动或候选关联竞争问题。这是按日志归纳的主因，部分时刻还存在次要条件失败。

`publish08` 九个状态漏检时刻为录包 +100.641、103.740、121.944、125.242、129.241、134.741、134.941、137.641、144.341 s。+129.241 s 是唯一丢失的原有 50 命中：球候选距旧球轨迹上次实测中心约 7.5 cm，但球轨迹中断约 0.5 s，连续墙片轨迹以约 81.4 cm 的距离先关联，造成球候选历史重启。v3 短断轨关联调整针对这个具体原因，完整 `priority08` 已恢复该时刻及另外六处 v2 漏检。剩余仅 +121.944、125.242 s，均为六点可见球面样本；其附近父簇密集实测中心已经偏离标注可见表面，不是简单的三次观测不足。同期日志中轨迹已经确认并发布分离点集，不能靠继续减少确认帧数解决。

无球 `priority08` 的新增输出来自短断轨保留/重新关联，而不是新增分离候选；它与 v2 仍共用“父簇核验后才分离”的路径。v4 `gapshape08` 已完整比较把既有密集形状核验同样应用于这些短断轨重检的效果，原有双方至少八点、观察方向变化不超过 15°及 0.45 比例条件保持不变。球 111 命中全部保留，无球从 v3 的 907 降至 888，但仍未满足两个时间口径的分段门槛。

固定启动背景只覆盖初始视野；走入拐角后新墙面不一定能由它解释。现有历史局部平面仍继续核验，但这些结果没有证明无人机进入任意新场景时都能保持相同检测质量。换到新通道或拐角后，若要重新建立固定参考，需要重新启动一次预采；当前实现没有把运动目标追加进冻结背景，也没有宣称未知新场景同样达到本轮命中率。

## 旧动态回归及 v5 调整

旧下午快/慢摆采用原有真实限频输入，完整原速回放；未插帧模拟 10 Hz，仍以旧 45/53、59/59 共 104 个命中逐个保留验收。v4/v5 结果均覆盖全部 112 冻结标注输出，四路一致。

| 完整对照 | 快摆 /53 | 慢摆 /59 | 原 104 命中损失 | 最大连续未命中 | 整回调 p95 /ms | 结论 |
| --- | ---: | ---: | ---: | --- | ---: | --- |
| 当前旧基线 | 45 | 59 | 0 | 见旧报告 | 见旧报告 | 冻结基线 |
| `afternoon_gapshape08`（v4） | 39 | 55 | 11 | 快 5 帧/1.301 s；慢 2 帧/0.301 s | 71.43 | 旧动态失败 |
| `afternoon_geometry_v5`（v5，几何关联） | 44 | 19 | 43 | 快 5 帧/1.301 s；慢 32 帧/11.047 s | 71.61 | 严重慢摆损失，淘汰 |

`afternoon_gapshape08` 的 11 处损失中，8 处父簇原本确认并输出，但分离后 `dynamic_cloud` 仅保留密集主体，使冻结球面支持低于 50%。相关时刻为 +134.930、139.917、143.123、147.915、183.532、188.144、193.311、193.612 s。另 3 处 +145.672、145.918、146.124 s 是关联/启动证据不足；不能通过扩大输出云解决。这促使 v5 只恢复完整已确认父簇动态云及地图删除索引，保留真实分量小框，避免更改标注或仅靠裁框提高分数。

v5 几何关联在新球录包仍为 111/113、27/27，但旧慢摆降至 19/59，失去 40 个慢摆旧命中，加上 3 个快摆旧损失共 43。快摆有新增命中，不能抵消这些损失。该组不继续消耗无球回放，缺无球数字明确标为“未运行”；不得根据新球的 98.23% 宣称关联方案合格。

`final06` 已完成几何关联关闭、开启 `detection/occlusion_startup_enabled` 的完整对照：新球 111/113、27/27，当前 50 命中零损失；无球原生分段 0/112/661/155，静止 7/467，旧下午快/慢摆 44/53、59/59（旧 104 个命中丢失 +145.672 s 一处），另一段限频录包 39/47 且保留原有 38 个命中。完整参数、缺输出和四路一致性见 `ball_final06_metrics.json`、`noball_final06_metrics.json`、`static_final06_metrics.json`、`afternoon_final06_metrics.json` 与 `latest_final06_metrics.json`。由于联合条件未通过，所有新增实验开关保持关闭。

## 复现命令与逐时刻证据

工作目录 `/home/oem/match_ws`，先加载主工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
source /home/oem/match_ws/devel/setup.bash
python3 .fapp_ball_debug/native10_20261008/evaluate.py --check-frozen
```

完整启动参数及实际 node/play 命令以每组 `*_job.json` 为准，运行后的 ROS 参数保存在 `*_runtime_params.yaml`。以下使用保留的不可变二进制复现已经完成的版本；名称带 `repro_`，避免覆盖已有产物。端口示例只用于彼此独立的本地主节点。

```bash
python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_nativebase --port 12841 --node .fapp_ball_debug/corridor_recovery_20261008/competition_v3_node
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_nativebase --port 12842 --node .fapp_ball_debug/corridor_recovery_20261008/competition_v3_node

python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_time6 --port 12843 --node .fapp_ball_debug/native10_20261008/v1_mapping_node --time-evidence --window 0.6
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_time6 --port 12844 --node .fapp_ball_debug/native10_20261008/v1_mapping_node --time-evidence --window 0.6

python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_density05 --port 12848 --node .fapp_ball_debug/native10_20261008/v1_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_density05 --port 12847 --node .fapp_ball_debug/native10_20261008/v1_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --param tracking/position_measurement_variance=0.01

python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_allv1 --port 12846 --node .fapp_ball_debug/native10_20261008/v1_mapping_node --time-evidence --window 0.6 --voxel 0.03 --density-split --core-radius 0.10 --density-fraction 0.7 --reacquisition --reacquisition-gap 1.6 --param association/global_assignment_enabled=true --param association/geometry_enabled=true --param tracking/position_measurement_variance=0.01 --param detection/translated_startup_enabled=true --param detection/translated_startup_fraction=0.8
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_allv1 --port 12845 --node .fapp_ball_debug/native10_20261008/v1_mapping_node --time-evidence --window 0.6 --voxel 0.03 --density-split --core-radius 0.10 --density-fraction 0.7 --reacquisition --reacquisition-gap 1.6 --param association/global_assignment_enabled=true --param association/geometry_enabled=true --param tracking/position_measurement_variance=0.01 --param detection/translated_startup_enabled=true --param detection/translated_startup_fraction=0.8

python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_publish08 --port 12850 --node .fapp_ball_debug/native10_20261008/v2_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_publish08 --port 12851 --node .fapp_ball_debug/native10_20261008/v2_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --param tracking/position_measurement_variance=0.01

python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_priority08 --port 12854 --node .fapp_ball_debug/native10_20261008/v3_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_priority08 --port 12855 --node .fapp_ball_debug/native10_20261008/v3_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py static static_repro_priority08 --port 12856 --node .fapp_ball_debug/native10_20261008/v3_mapping_node --voxel 0.05 --density-split --core-radius 0.08 --density-fraction 0.7 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01

python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_gapshape08 --port 12858 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_gapshape08 --port 12859 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_depthnovel4 --port 12860 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01 --param detection/depth_evidence_enabled=true --param detection/min_motion_novel_fraction=0.4
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_depthnovel4 --port 12861 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01 --param detection/depth_evidence_enabled=true --param detection/min_motion_novel_fraction=0.4
python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_bg3 --port 12864 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01 --param background_consistency/min_frames=3
python3 .fapp_ball_debug/native10_20261008/run_case.py noball noball_repro_bg3 --port 12865 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01 --param background_consistency/min_frames=3
python3 .fapp_ball_debug/native10_20261008/run_case.py afternoon afternoon_repro_gapshape08 --port 12862 --node .fapp_ball_debug/native10_20261008/v4_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01
python3 .fapp_ball_debug/native10_20261008/run_case.py ball ball_repro_geometry_v5 --port 12866 --node .fapp_ball_debug/native10_20261008/v5_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01 --param association/geometry_enabled=true
python3 .fapp_ball_debug/native10_20261008/run_case.py afternoon afternoon_repro_geometry_v5 --port 12867 --node .fapp_ball_debug/native10_20261008/v5_mapping_node --density-split --core-radius 0.08 --reacquisition --reacquisition-gap 1.0 --param tracking/position_measurement_variance=0.01 --param association/geometry_enabled=true

python3 .fapp_ball_debug/native10_20261008/evaluate.py ball_repro_nativebase
python3 .fapp_ball_debug/native10_20261008/evaluate.py noball_repro_nativebase
python3 .fapp_ball_debug/native10_20261008/evaluate.py ball_repro_time6 ball_repro_density05 ball_repro_allv1 ball_repro_publish08 ball_repro_priority08 ball_repro_gapshape08 ball_repro_depthnovel4 ball_repro_bg3 ball_repro_geometry_v5 --baseline ball_repro_nativebase
python3 .fapp_ball_debug/native10_20261008/evaluate.py noball_repro_time6 noball_repro_density05 noball_repro_allv1 noball_repro_publish08 noball_repro_priority08 noball_repro_gapshape08 noball_repro_depthnovel4 noball_repro_bg3 --baseline noball_repro_nativebase
python3 .fapp_ball_debug/native10_20261008/evaluate.py static_repro_priority08 afternoon_repro_gapshape08 afternoon_repro_geometry_v5
```

原生基线二进制 SHA256 为 `db2ea4a6f669f0b24e42d2a6246625e3a6df2af9bfd66ee1b0c66c08b48c9211`，v1 为 `2d41ab78ee56a03dd168a3033f040f81713be586bfcb0285db7c8308d2de74ec`，v2 为 `883ad9699a5f44c721ff6f9e60a6bcfd1198355e32e521df3a904232ebe62aef`，v3 为 `87333e6fcc6d55023de11ac7d38ae3c5eb7454bae43ecee3d35733f92c0f7d65`，v4 为 `3e53ee7473b89a30af5f51ffa0a7c1ca703c8015c01c3ef0aae5bd748f36ef0f`，v5 为 `2a50fffa0cc58ff5693d0fdf6be9bd9e6123e4aed0a2654f86fd3f097cdb83f9`。修改后的当前 `devel` 不能代替旧快照复现已列版本。

证据目录：[native10_20261008](../../../.fapp_ball_debug/native10_20261008/)。每组提供：

- `*_metrics.json`：球逐时刻 `observations`（含输出是否存在、状态/框命中、原命中）、失去/新增命中、分段框数、全部缺失时间戳、回调与总延迟。
- `*_job.json`、`*_runtime_params.yaml`：输入/二进制 SHA256、源文件元数据、精确参数、完整节点与回放命令。
- `*_input_stamps.json`、`*_audit.json`：源输入时间戳及四路消息实际接收时间；`*_memory.json`：RSS/高水位采样。
- `*.bag`、`*_node.log`：同一次完整输出与算法诊断；`*_driver.log`、`*_terminal.json`：是否完整完成、源输入是否保持不变。

## 最终验证与运行配置

`final06` 的二进制 SHA256 为 `5131e45a9d4b9e0280865c05a88515f78985e41d15f448a3b0e1a2b434dc3040`。有球原生输入 1727 帧，实际回调 1715、四路输出 1714，整回调 p95 87.84 ms；113 个冻结时刻全部有输出，状态命中 111/113，三帧子集 27/27，当前 50 个命中全部保留。无球原生输入 763 帧，实际回调 761、输出 760，整回调 p95 82.97 ms，四路一致；相对原生基线无球分段框数 0/112/661/155 对 0/109/607/96，原 5 Hz 锚点 0/41/253/61 对 0/37/247/49，未通过误检代理门槛。静止 `static_final06` 为 7/467，对 6/467，未通过静止门槛。旧下午段为快摆 44/53、慢摆 59/59，丢旧命中 +145.672 s 一处；另一段为 39/47，原有 38 个命中均保留。各组回放均核对四路输出一致；缺输出按漏检统计，球冻结时刻无缺输出，无球有 1 个无输出回调和 2 个源帧未进入回调。

主工作空间已执行 `catkin_make --pkg mot_mapping diff_planner -j2 -l2`，结果为 exit 0（日志：`/tmp/native10_build_final.log`）；实际 `devel/lib/mot_mapping/mapping_node` 产物时间为 2026-10-08 23:38:04，测试产物也已更新。`CTest` 的 `mot_mapping_density_partition`、默认与 A04 10 Hz smoke、20 cm 六点球面/平面和 launch XML 解析均通过。默认启动保持 `input_rate=10.0`、三帧、8 cm，新增实验旗全部为 false；A04 的 FAPP include 使用原生约 10 Hz relay，LDOP 仍为 5 Hz。

因此 `final06` 仅作为球样本达到 95% 的离线候选，不启用为默认配置。当前可运行配置继续使用 A04 的已有参数和默认关闭的新开关。A04 入口命令为：

```bash
source /opt/ros/noetic/setup.bash
source /home/oem/match_ws/devel/setup.bash
roslaunch diff_planner run_swarm.launch detector_backend:=fapp enable_rviz:=false record_dynamic_debug:=false
```

通用 FAPP 10 Hz 入口为 `roslaunch mot_mapping uav1_detection.launch`；下一轮若要同时压低无球框数并保留 95% 动态命中，应先增加覆盖转角新背景、手持移动和遮挡重现的标注，再针对无球新增的 3/54/59 分段框做候选来源诊断。
