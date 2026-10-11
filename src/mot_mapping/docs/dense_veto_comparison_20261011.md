# 稠密 changed 证据 V2：原判定与稠密判定取 AND

本轮**未达标**。启用 V2 保留控制组全部 111 个球命中，三帧条件均为 27/27；无球框数由 907 降至 882，但走廊及返回阶段仍超过冻结上限，球数据整段 callback p95 为 101.4458 ms，未达到严格小于 100 ms。四次原始 driver PASS 表示完整回放与正常关闭，不代表质量验收通过。旧 142 个命中、静态 467 帧及完整候选组合回归未运行，结果保持 `null`。

完整原 metrics、113 个有效冻结标签的逐行控制/启用结果、原 GT 行、门控日志样例、缺失时间戳、资源记录及文件 SHA256 见 [结果 JSON](dense_veto_results_20261011.json)。大体积逐行门控核验保存在 `.fapp_ball_debug/dense_veto_20261011/review_details.json`，其路径、大小及 SHA256 也记录在结果 JSON。

## 本轮代码和参数语义

本轮 OFF/ON 均继承 F06 + density_split + confirmed_reacquisition + measured_body_position + bidirectional_static 实验组合，并开启点源竞争；111/113 是该候选组合的结果，不是正式 A04 + 点源竞争的默认命中率，正式新增增强项仍默认关闭。

V2 沿用 V1 的参数名 `detection/dense_change_evidence_enabled`，但开启语义已从“用稠密统计代替原 changed 统计”改为：

```text
changed = enabled ? (parent_changed && dense_changed) : parent_changed
```

相对 V1 的源码快照，本轮 `mapping_manger.cpp` 仅新增 5 行：保存原 `parent_changed` 的 2 行，以及说明和 AND 限制的 3 行。源码及 `uav1_detection.launch` 默认均为 `false`。显式实验启用示例：

```bash
roslaunch mot_mapping uav1_detection.launch dense_change_evidence_enabled:=true
```

稠密判定复用原候选的真实最近地图查询，并取原稠密中心对应邻域的均值与归一化方差。本轮运行阈值为 `thresh_dist=0.06`、`thresh_var=0.28`，判据是严格的 `0.06 < mean < 5` 且 `variance < 0.28`；不能把源码距离默认值 0.08 当成本轮参数。“parent”指当前候选 `indices.indices` 的原统计，开启候选分割时不一定是完整 DBSCAN 父簇。

V2 不允许原 `parent_changed=false` 的候选仅因稠密判据变成 `changed=true`，没有增加直接启动分支。已有三次观察、0.82 m 关联、背景/形状/主体及遮挡启动逻辑保持原配置。后续历史、关联和其他既有启动证据仍可改变输出，不能据此声称启用组输出必为控制组的子集。

## 四次真实运行与开关证据

使用原 `run_case.py` 顺序运行相同输入的完整 1× 回放，控制/启用除本开关外参数及播放命令相同；未并行运行本矩阵。参数在 dry-run 设计、job、metrics 和 runtime YAML 中一致，错误的 `synchronization/dense_change_odom_enabled` 参数不存在。DEBUG 配置为 `ros.mot_mapping=DEBUG`，原日志如下：

| 原 case 名 | dry-run / job / runtime | FAPP_DENSE_CHANGE 行数 | 原 driver 结论 |
|---|---|---:|---|
| noball_dense_veto_control_20261011 | false / false / false | 0 | PASS completed 原 case |
| noball_dense_veto_enabled_20261011 | true / true / true | 20607 | PASS completed 原 case |
| ball_dense_veto_control_20261011 | false / false / false | 0 | PASS completed 原 case |
| ball_dense_veto_enabled_20261011 | true / true / true | 47520 | PASS completed 原 case |

四次原 terminal 均 `success=true`、`input_unchanged=true`、`output_finalized=true`；master、record、node、audit、play 返回码均为 0，输出 `.bag` 已关闭且没有同名 `.bag.active`。原 `matrix_terminal.json` 为 `success=true/cases=4/all_actual_flags_verified=true`，ROOT 从原运行 handle 23700 得到 `exit_code=0`、`PASS actualveto4runs` 后才执行后处理。打包未重启算法、修改原 PASS、重新评分或修改 GT。

## 固定质量门槛

| 指标 | 冻结要求 | 控制 OFF | V2 ON | ON 结论 |
|---|---:|---:|---:|---|
| 球有效标签命中 | ≥108/113 | 111/113 | 111/113 | 通过 |
| 连续三帧标签命中 | ≥26/27 | 27/27 | 27/27 | 通过 |
| 无球 capture 框数 | ≤0 | 0 | 0 | 通过 |
| 无球 initial_stationary 框数 | ≤109 | 105 | 104 | 通过 |
| 无球 carried_corridor 框数 | ≤607 | 652 | 635 | 未通过，超 28 |
| 无球 returned 框数 | ≤96 | 150 | 143 | 未通过，超 47 |
| 无球 whole callback p95 | <100 ms | 98.004385 ms | 97.4811 ms | 通过 |
| 球 whole callback p95 | <100 ms | 101.23265 ms | 101.4458 ms | 未通过 |
| 旧 142 个命中全部保留 | 全保留 | null | null | 未测 |
| 静态冻结 467 帧框数 | ≤6 | null | null | 未测 |
| 完整候选场景回归与合成组合 | 全部通过 | null | null | 未测 |

ON 无球分阶段相对 OFF 为 `0/-1/-17/-7`，总框数减少 25。113 个有效冻结标签按精确原 header 逐行核对，原标签点数、时间、原命中和三帧资格均未改变；111 个控制命中全部保留，lost=0、gained=0。两组均未命中的有效标签为 `1791429339268123627`（121.943645 s）与 `1791429342566560745`（125.242082 s），均有输出，均不属于三帧条件。冻结标签本身仍为 113/49，三帧原标签仍为 27/19。

## 真实日志中的 AND 核验

用既有 `compare_logs.py` 处理可明确重拼的 PCL stderr 插入，原日志未改。将启用组 `FAPP_DENSE_CHANGE` 的父/稠密判据与同一精确 stamp、同一精确打印中心的唯一 `FAPP_TRACK.changed` 对应。六位打印阈值边界保持 unknown；没有匹配或多重匹配时不判成 false。本轮实际没有未匹配或歧义行。

| 父判据 / 稠密判据 | 无球行数 | 球行数 | 实际 TRACK.changed |
|---|---:|---:|---|
| false / false | 19453 | 45606 | false |
| false / true | 274 | 457 | false，未发生直接提升 |
| true / false | 152 | 217 | false，执行 veto |
| true / true | 727 | 1239 | true |
| 打印阈值边界未知 | 1 | 1 | 不分类 |
| 总核验行数 | 20607 | 47520 | 非边界行零不一致 |

`FAPP_TRACK.changed` 直接输出未再修改的 candidate 标志；其 confirmed 仍在背景、静态、形状等后续拒绝之前，因此这些行不能替代实际输出或 GT 命中判定。

## 输出覆盖、同期差异与资源

| case / 模式 | 源输入 | callback 日志 | 输出帧 | 缺 callback | 缺输出 | callback p50 / p95 / max (ms) | 回放进程 CPU 均值 | 峰值 RSS (KiB) |
|---|---:|---:|---:|---:|---:|---|---:|---:|
| noball OFF | 763 | 760 | 759 | 3 | 4 | 77.8897 / 98.004385 / 176.005 | 82.07% | 155676 |
| noball ON | 763 | 761 | 760 | 2 | 3 | 79.2743 / 97.4811 / 165.341 | 83.13% | 156480 |
| ball OFF | 1727 | 1714 | 1713 | 13 | 14 | 79.5407 / 101.23265 / 169.849 | 78.41% | 164404 |
| ball ON | 1727 | 1714 | 1713 | 13 | 14 | 79.6281 / 101.4458 / 168.111 | 78.89% | 164060 |

每组四个输出 topic 的精确 stamp 集合、状态/框/轨迹数量及观察器输出集合均通过原检查，但不等于每个源输入都有输出。原 `actual_completed_callbacks` 字段实际是输出帧数，以上 callback 数使用 `actual_callbacks`。完整缺失 stamp 和早退 callback 原样保留在结果 JSON。球两组 113 个有效标签的输出都存在；无球原 292 个 5 Hz anchor 输出都存在。球原 5 Hz 全 anchor 中 OFF 缺 2 个、ON 缺 3 个，不能隐藏为零框。

控制与启用的 callback 集合不同：无球 ON 多 1 个；球两组各有 1 个独有 callback。共享 callback 中，无球 `SYNC_POSE`/候选几何/plane 差异分别为 7/2/1 个 stamp，球为 30/9/8 个 stamp。输入、播放命令和二进制相同仍不足以证明全部内部配对与几何相同，故上述框数差异是本轮实测，不能全部归因于 veto。

CPU 来自 mapping_node 进程 user/system ticks，每约 0.5 秒采样；100% 为一个逻辑 CPU，未包含 rosbag、master、观察器、FAST-LIO、规划器及全机负载。RSS 和 HWM 原记录保留。whole callback 包含进入 callback 后的早退，不包括同步等待；观察器端到端延迟还包含同步和 ROS 传输，两者不能混用。

回放结束与约 2 秒 settled 时的未输出输入分别为无球 OFF 4/4、ON 3/3、球 OFF 14/14、ON 14/14，两个 checkpoint 间新增 states 均为 0；末 1 秒输入 pending 均为空。无球延迟趋势 OFF/ON 为 0.068894/0.128451 ms/墙钟秒，首末 10 秒均值差为 +13.2906/+22.6921 ms；球为 -0.074102/-0.076884、+3.1658/-3.5833 ms。四段延迟非持续单调上升，本轮未显示末端队列持续增长；这些 pending 也可能是既有早退或未进入同步 callback，不能仅由正趋势或 pending 总数证明持续积压。报告不新增数值判据，`sustained_backlog_pass` 保持 `null` 并保留所有窗口、趋势及 checkpoint 原证据。已测无球质量和球 p95 的失败已经足以判定本轮未达标。

## 源码、编译产物与原输入一致性

主工作空间 `devel/lib/mot_mapping/mapping_node` 在本轮源码修改后生成，冻结及四次 job/metrics 的二进制 SHA256 全部一致。源码与 source snapshot 的 SHA256 在运行前及四次关闭后一致。以下记录本轮关闭及打包时的快照，不声明后续 live 源码或产物仍保持相同版本：

| 文件 | SHA256 |
|---|---|
| mapping_manger.cpp | `382a64fdb2900bee371115af629cf83e6814aafb6000a297712084a6db9d3b5d` |
| mapping_manager.h | `2dfecbf8521fb592654ccf73a433205fcfe6081f9ed545e90bb78b63cd2a6f14` |
| uav1_detection.launch | `7f4bd4f6caf6e298f2163c8ef5c6f3606aa0d4d6eb6ce5b1ad8975955edeb605` |
| convergence_smoke.py | `a487527acead52da0332d8bc14e34619e1c62bfcb12b0498f75fd3be9fd07f41` |
| devel/lib/mot_mapping/mapping_node | `c722098b8293725017341ae6b3cbd398b2a9a76ea258ecaa85717d8d22f49689` |
| noball_raw_input.bag | `12af567b7595d3361c815a13fbc4b87410a505fee3e38cd5860c1365015087f2` |
| ball evaluation.bag | `8e95b2765978c0d773d9f4a59dc4800e30b2d16d30c355d4489ff19b0b205310` |

运行前已完成主 `devel` 编译、默认 7 个及 V2+competition 8 个真实 ROS 回归、3 个 CTest、3 个 launch 解析；冻结 manifest 全部通过，原 build/回归/CTest 日志及 launch 参数快照随证据 manifest 保存。这些合成和解析验证不能替代本轮未运行的旧142、静态467或完整场景矩阵。

## 与 V1 证据的边界

V1 最初四个含 dense_change 名字的 case 使用错误参数 `synchronization/dense_change_odom_enabled`，实际 dense 功能均 OFF，不能作为启用效果。原产物及错误参数审计保留。随后两次正确键名的 V1 真启用为独立旧证据：球 111/113、三帧 27/27，无球 `0/105/722/148`、975 框，球 p95 101.36965 ms；其二进制 SHA 为 `11eb20751bab402e421afe8c56ce97daec2cf8e389b916368f256d12dc7ca45d`，不等于本轮 V2。旧 V1、无效参数回放及本轮 V2 均分别记录，不合并成同一控制实验，不用旧结果补齐未测验收项。
