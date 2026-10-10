# 独立点级原型性能对照（2026-10-10）

**当前联合门槛未通过，不推荐启用。** 本轮只优化独立点级原型的计算成本，保留运动、聚类和关联判据。用户选定的FAPP入口继续为10 Hz、A04＋点源竞争，真实目标最小直径20 cm、普通目标三帧确认保持；主体位置、双向射线启动、双向静态抑制三个新开关仍关闭。`run_swarm`须选择`detector_backend:=fapp`才使用这组配置，其默认backend仍为legacy。正式 launch 不调用此原型，`devel/lib/mot_mapping/mapping_node` 未被原型替换。

已按新球、无球、静止、旧快慢摆、旧通道顺序单独完成五段完整回放，各终态成功。“新球”的113/27冻结样本来自2026-10-08摆球录包。逐时刻交付见[结果 JSON](point_event_performance_results_20261010.json)。

## 修改范围与等价性

仅独立 `test/point_event_prototype/event_core.py` 改变计算方式：

- 射线邻域按连续分段归约，替代重复索引 `ufunc.at`；保留无返回为未知、真实近返回遮挡、事件/非事件支持及最近深度并列时最后原索引。
- 上一实测点祖先按索引去重，查询后恢复原逐点标签。
- 聚类先查询邻点数量，仅为非core点展开邻域；保留成员、border归属和原组顺序。不同候选的历史最近点查询合批，仍逐组计算原新点比例。
- 上一实测中心KD树以0.83 m粗筛，仍按原标量距离`<=0.82 m`、连续优先、距离/候选/轨迹索引排序和一对一规则配对。
- 世界最近点树直接使用原候选ROI，完整原始云仍保留给射线判别。

没有修改三次实测、6 cm位移、0.14 m/s、事件26%、新点26%/20%、3 cm射线邻域/容差、6点最小簇、800点最大簇、5秒预采或5 m候选窗口。没有新增20 cm尺寸筛除、两帧特例、方向要求、预测发布、降采样、补帧或同步策略。

计数查询和排序批量邻域使用SciPy 1.10.1的`return_length`、`return_sorted`参数：[官方API说明](https://docs.scipy.org/doc/scipy-1.10.1/reference/generated/scipy.spatial.cKDTree.query_ball_point.html)。操作等价性由本轮实际边界回归和完整输入对照证明，API支持本身不能证明算法等价。

源文件身份：

| 文件 | SHA256 |
|---|---|
| 原核心（本地提交`d878f5de`） | `aca0a0240eb4dc15eda1b4441a13a2f94cdc1609262917665367da931beb1047` |
| 优化核心 | `7cb3876ba26ad4276122be05b96963d53feb2d1a38b4cd3b7a3cbd4754d47954` |
| 未改ROS适配器 | `f82c39a9ca1e964a3ff15f31a450ed9d66e4414edcb92c72b85ba249f271a14c` |
| 正式C++可执行文件 | `0cb78170a05500e06a9f12d933c406e21ea4852e2e37f3ea91ebe98aa2aa51c2` |

[全1727帧等价记录](../../../.fapp_ball_debug/point_event_perf_20261010/equivalence_full_ball.json)对每个原生输入云按header时间选择最近实测HF位姿，旧/新核心使用相同输入、交替先运行；逐点事件/静态标签、候选点索引、位置尺寸、轨迹、发布及原因记录全部精确相等，**差异0、跳过输入0**。累计候选104785、发布框观测1752、事件点286060、静态点9775820也相等。这种配对与ROS实际50 ms近似同步不同；此结果证明同输入决策等价，不能证明检测正确或ROS实时覆盖。

## 控制输入下的核心成本

| 全段`EventDetector.step` | 原核心 | 优化核心 |
|---|---:|---:|
| p50 | 118.019 ms | 69.812 ms |
| p95 | 150.232 ms | 100.943 ms |
| 最大 | 194.440 ms | 116.016 ms |
| 平均 | 117.982 ms | 75.348 ms |

核心p95减少32.81%，但仍超过100 ms。此计时不包含解码、ROS同步、消息构建、发布和日志；不是整回调验收值。前120帧的小段结果也不能替代全段。证据：[摘要](../../../.fapp_ball_debug/point_event_perf_20261010/equivalence_summary.json)、[独立边界审计](../../../.fapp_ball_debug/point_event_perf_20261010/audit_equivalence.json)。

## 原速ROS回放

旧`*_event_v1`属于并发筛查，曾与主C++矩阵重叠；新`*_event_v2_perf`一次只运行一段回放，保留普通桌面负载。两组的实际接收回调和匹配位姿不同，**不能把两组整回调p95的差直接当作优化的因果降幅**。能支持成本下降的是上面的相同全输入交替核心对照。

| 新独立回放 | 冻结结果 | 输出／原输入 | 整回调p50／p95／最大 ms | 回放期节点CPU均值 | 峰值RSS |
|---|---|---:|---:|---:|---:|
| 新球`ball_event_v2_perf` | 17/113；三帧5/27 | 1506/1727 | 101.78／136.24／201.74 | 84.06% | 149.63 MiB |
| 无球`noball_event_v2_perf` | 阶段框3／189／255／193 | 698/763 | 103.11／133.06／176.13 | 88.84% | 117.72 MiB |
| 静止`static_event_v2_perf` | 653框／467冻结时刻 | 468/468 | 118.41／152.53／189.55 | 46.24% | 84.49 MiB |
| 旧快慢摆`afternoon_event_v2_perf` | 快摆10/53、慢摆2/59；损失旧92命中 | 507/515 | 104.27／134.70／182.47 | 27.87% | 119.22 MiB |
| 旧通道`latest_event_v2_perf` | 1/47；损失旧37命中 | 1043/1075 | 107.45／143.45／223.40 | 25.33% | 116.93 MiB |

CPU为节点进程用户＋系统时间，100%表示占满一个逻辑核，不是整机占用；排除rosbag、master、审计订阅、FAST-LIO和规划器。RSS由`/proc`的KiB除1024转MiB。整回调计时来自每次实际`FAPP_CALLBACK_TIME`，包括本节点处理、发布及诊断日志；审计接收时延是另外的端点，不能混算。

新球仍有221个原输入没有输出，冻结113中3个无回调，全部计漏检。无球缺65个输入输出，不能把较少框数直接看作误检改善。静止覆盖全部467冻结时刻，另有一个空输出时刻；653框确实来自冻结时刻，仍超过6框门槛。

原有142个命中丢失129个：快摆35、慢摆57、旧通道37，仅保留13个，旧命中门槛失败。旧快慢摆缺8个输入输出，旧通道缺32个。五段实际四路输出的一致性检查均通过，但这一检查不证明每个原输入都有输出，也不证明发布目标正确。

无球固定同频参考保持**0／109／607／96**，不改成缺输出的新基线。新原型初始静止189>109、返回193>96，固定`capture`评分分段3>0；通道255<607不能抵消其他阶段损失。无球`capture`冻结分段截至录包+5.64 s，新球为+5.84 s，不能把它们与节点从首个实测云开始的5秒预采直接等同。累计框只是误检代理，尚无逐框真实物理误检标注。

回调积压审计同时保存播放结束与其后2秒检查点。新球实际输入到状态审计接收时延p95为641.41 ms，无球617.92 ms；这些包含同步/传输。新球播放结束与稳定后均1506个输出，无球696→698、最后补出2个。不能因队列不无限增长就宣称10 Hz实时达标：两段整回调p95超100 ms且仍缺输入输出。精确缺帧和时间窗口见各`*_audit.json`、`*_play_end_audit.json`、`*_settled_audit.json`和`*_metrics.json`。

## 新球逐时刻缺口

按冻结中心距离重排当前实测候选，并沿用中心小于25 cm、最大尺寸小于65 cm判据：

| 113个冻结时刻的首要诊断 | 时刻数 |
|---|---:|
| 无回调 | 3 |
| 没有合格几何候选 | 73 |
| 有合格候选，但不足三次实测 | 18（11次只有一帧、7次只有两帧） |
| 有合格三帧候选，新点比例首拒 | 1（99.443 s） |
| 有合格三帧候选，事件比例首拒 | 1（105.541 s） |
| 合格发布 | 17 |

17个合格发布与实际bag命中逐时刻一致，回调与输出有无也一致。73个无合格几何候选中，66个没有40 cm内候选中心，4个中心在25～40 cm，3个25 cm内候选全超过65 cm。**37/113是当前实测候选路径在此回放的几何覆盖上限，不是不可突破的算法上限**；新增候选或改变分离方式仍需独立试验。

18个观测不足候选中14个事件比例已足够、17个新点比例已足够，当前主要缺口是候选表示和连续历史，而非只调事件/新点阈值。40 cm邻近仅用于定位诊断，不能据此认定候选就是物理球。证据：[候选几何核对](../../../.fapp_ball_debug/point_event_perf_20261010/ball_event_v2_perf_frozen_geometry_audit.json)。

首次冻结命中为录包+92.142 s；最长连续冻结漏检样本18个，首尾跨度7.500 s。样本时刻并非均匀覆盖，因此这个跨度**不是对真实连续无检测空档的测量**。三帧子集最长3个漏检样本，跨度0.300 s；旧快摆、慢摆、旧通道首个冻结命中分别为各自录包+132.542、182.415、281.826 s，最长漏检样本分别8/32/11个、跨度2.288/11.047/3.301 s。这些录包相对时刻不是从目标进入视野起算的检测延迟；完整记录在结果JSON。

`*_event_diagnostics.json`逐框记录实际事件来源（横穿、靠近、远离）和轨迹发布持续片段，缺发布回调或时间间隔会分段。这些是算法点来源，不能冒充真实物体/墙面标签。`*_additional_box_diagnostics.json`另保存与固定参考的几何新增框匹配；新增观测数量也不是净新增框或真实误检率。

追加按实际发布中心、尺寸、输出序号核对来源后，新球1303个参考未匹配几何观测、无球463个，全部对应真实`POINT_EVENT_CANDIDATE`发布；物理原因仍未逐框人工标注。新球同轨迹在连续原输入时刻的最长片段0.3002 s、各片段首尾跨度之和12.5146 s；无球对应0.2999 s、2.9932 s。时间片段会在任何原输入缺发布或阶段切换处分断，这些跨度既不是物理占用时长，也不是所有新增框持续时间的并集。完整来源及片段由结果JSON保存。

旧原型40个“缺附近候选”时刻的只读射线核对，即使采用最大静态删除上界，冻结中心15 cm内仍至少保留6个真实点/体素。它只能证明附近点没被全部剔除，不能证明这些点全部属于球或必能成簇；历史逐点事件掩码未记录，不能声称精确知道所有被删球点。证据：[上界与限制](../../../.fapp_ball_debug/point_event_perf_20261010/ball_event_v1_static_support_upper_bound.json)。

## 复现与验证

算法不读取冻结标注；标签只传给评分与诊断程序。原型实际配置以节点`POINT_EVENT_CONFIG`为准，驱动中未使用的A04参数不能被算作原型已有功能。以下用新前缀依次原速回放，禁止覆盖已存在证据；ball/noball使用原生10 Hz输入，三份历史5 Hz链路输入保持原时间戳与实际间隔，不补成10 Hz。

```bash
cd /home/oem/match_ws
set -e
source devel/setup.bash
export ROS_HOME=/tmp/point_event_v2_repeat_01
performance_tag=event_v2_perf_repeat_01
performance_port=15600
for performance_case in ball noball static afternoon latest; do
  performance_name="${performance_case}_${performance_tag}"
  python3 .fapp_ball_debug/bidirectional_20261009/run_case.py \
    "$performance_case" "$performance_name" --port "$performance_port" \
    --method baseline --core-radius .08 \
    --node /home/oem/match_ws/src/mot_mapping/test/point_event_prototype/event_node.py
  case "$performance_case" in
    ball|noball) python3 .fapp_ball_debug/bidirectional_20261009/evaluate.py \
      "$performance_name" --baseline "../native10_20261008/${performance_case}_nativebase" ;;
    *) python3 .fapp_ball_debug/bidirectional_20261009/evaluate.py "$performance_name" ;;
  esac
  if [ "$performance_case" = ball ]; then
    python3 src/mot_mapping/test/point_event_prototype/diagnostics.py \
      ".fapp_ball_debug/bidirectional_20261009/$performance_name" \
      --labels .fapp_ball_debug/corridor_20261008_ball/ball_labels.json
  else
    python3 src/mot_mapping/test/point_event_prototype/diagnostics.py \
      ".fapp_ball_debug/bidirectional_20261009/$performance_name"
  fi
  performance_port=$((performance_port + 1))
done
python3 -m unittest discover -s src/mot_mapping/test/point_event_prototype -p 'test_*.py' -v
```

27项单元/等价回归通过；8帧ROS样例输出`[0,0,0,1,1,0,0,0]`、四路一致；主工作空间`catkin_make --pkg mot_mapping -j2`成功，3项CTest通过，默认感知与正式FAPP launch解析通过。证据目录：[构建、测试和参数](../../../.fapp_ball_debug/point_event_perf_20261010/)。这些检查证明实现等价及接口可运行，不能替代完整动态回归或整机闭环避障验证。

本轮性能优化可以保留为独立研究工具；正式启用仍不成立。需要恢复108/113、26/27、全部旧142命中、静止不超过6框、无球每阶段不增框、整回调p95小于100 ms及完整场景回归，才能改变推荐配置。本报告不把任何单项性能收益当作联合验收完成。
