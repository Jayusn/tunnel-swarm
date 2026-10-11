# 普通启动历史改用已发布来源：真实四次对照

本轮**尚未达到联合验收要求**。启用已发布历史来源后，无球返回阶段由 138 框降至 76 框，达到不超过 96 的要求；走廊阶段由 630 降至 627，仍比冻结上限 607 多 20。球两组均为 111/113、连续三帧 27/27，控制组全部 111 个命中保留。四组 whole callback p95 均小于 100 ms。旧142、静态467和完整候选组合回归未运行，保持 `null`，不作为通过项。

本轮 OFF/ON 均继承 F06 + density_split + confirmed_reacquisition + measured_body_position + bidirectional_static 实验组合及点源竞争，且 dense V2 两组恒为 true。111/113 是这个候选组合的结果，不是正式 A04 + 点源竞争的默认命中率；正式新增增强项仍默认关闭。原结果及证据见 [结果 JSON](published_startup_results_20261011.json)。

## 两项源码修改及开关范围

两组使用相同的最近点查询复用版本。对同一个真实当前点、同一个不可变历史树、同一个 k=1 查询，independent 与未知深度时的 novel 统计共享结果；两项仍使用原空查询判据、首个平方距离和平方阈值，不使用 nearest index。

| 既有分支 | 原查询次数 | 本轮查询次数 |
|---|---:|---:|
| independent 开启，novel 深度未知 | 2 | 1 |
| independent 开启，深度已知 | 1 | 1 |
| independent 关闭，novel 深度未知 | 1 | 1 |
| independent 关闭，深度已知 | 0 | 0 |

新参数 `detection/published_startup_source_enabled` 在源码及 `uav1_detection.launch` 中默认 `false`，只选择普通 original startup 的历史项：

```text
historical_startup = enabled ? (last_published_stamp != 0) : (last_dynamic_stamp != 0)
```

启用示例：

```bash
roslaunch mot_mapping uav1_detection.launch published_startup_source_enabled:=true
```

关闭时保留原 `last_dynamic_stamp` 非零条件；开启时用 `last_published_stamp` 非零，不新增历史年龄限制。recently_changed、reference/occlusion/translated、strict 分支、bidirectional 补充、0.6 s 运动确认、关联/观测/阈值和形状使用的 last_dynamic 及其写入均保持原用途。`FAPP_PUBLISHED_STARTUP` 只在开启时记录本帧更新 motion/dynamic/published stamp 前的历史值与 bidirectional 补充前的 original startup。

最近点复用分支依据为 `.fapp_ball_debug/next_evidence_20261011/live_source_readonly_review.json` 的 `knn_reuse.old_and_new_counts/semantic_evidence/performance_boundary`。该审阅支持局部计数语义一致，不能直接证明 callback p95 或调度一致。

## 原四次运行与实际开关

ROOT 从原 session 27066 得到 `exit_code=0`、`DONE VERIFIED ball_published_source_enabled_20261011`、`PASS actual published-history four runs`，明确四次完整回放和 evaluator 均结束后才执行打包。使用原 `run_case.py` 顺序完整 1× 回放，除新开关外两组参数及播放命令相同；没有重跑算法、修改原 PASS、修改 GT 或重新评分。

| 原 case 名 | dry-run / job / runtime | 新 PUBLISHED_STARTUP 行数 | DENSE_CHANGE 行数 |
|---|---|---:|---:|
| noball_published_source_control_20261011 | false / false / false | 0 | 20562 |
| noball_published_source_enabled_20261011 | true / true / true | 20565 | 20565 |
| ball_published_source_control_20261011 | false / false / false | 0 | 47553 |
| ball_published_source_enabled_20261011 | true / true / true | 47540 | 47540 |

metrics 参数与各自 dry-run、job 完全一致，dense 开关在四份 runtime YAML 中均 true。四份原 driver 均为唯一一行 `PASS completed <原case名>`；原 terminal 均 success/input_unchanged/output_finalized=true，master/record/node/audit/play 返回码均 0，输出 bag 均已关闭且没有同名 `.bag.active`。原 matrix_terminal 为 success=true、cases=4、all_actual_flags_verified=true。这些 PASS 是完成证据，不是质量通过结论。

## 固定门槛与完整113行

| 指标 | 冻结要求 | OFF | ON | ON 结论 |
|---|---:|---:|---:|---|
| 球有效标签命中 | ≥108/113 | 111/113 | 111/113 | 通过 |
| 连续三帧标签命中 | ≥26/27 | 27/27 | 27/27 | 通过 |
| 无球 capture | ≤0 | 0 | 0 | 通过 |
| 无球 initial_stationary | ≤109 | 104 | 104 | 通过 |
| 无球 carried_corridor | ≤607 | 630 | 627 | 未通过，超20 |
| 无球 returned | ≤96 | 138 | 76 | 通过 |
| 无球 whole callback p95 | <100 ms | 85.703685 ms | 86.62256 ms | 通过 |
| 球 whole callback p95 | <100 ms | 92.41595 ms | 92.25673 ms | 通过 |
| 旧142个命中全部保留 | 全保留 | null | null | 未测 |
| 静态冻结467帧框数 | ≤6 | null | null | 未测 |
| 完整候选场景回归/合成组合 | 全部通过 | null | null | 未测 |

ON 无球分阶段相对 OFF 为 `0/0/-3/-62`，总框数由 872 降至 807。球总框数 2097→2085；这个总数不能替代固定命中判据。完整113个有效标签按原精确 stamp 对齐，原 GT 行、时间、点数、原命中和三帧资格均逐项核对，控制命中 kept=111、lost=0、gained=0。两组均未命中的有效标签仍为 `1791429339268123627` 与 `1791429342566560745`，均有输出且不属于三帧条件。冻结 GT 保持 113/49、三帧27/19。

## 新历史来源及 dense AND 的真实日志核验

使用既有 `compare_logs.py` 处理可明确重拼的 PCL stderr 插入，原日志未改。启用组新 tag 的整数 ns 严格验证 `historical_startup == (prior_last_published_stamp != 0)`；按唯一的 stamp+track ID 对应 BIDIRECTIONAL_DECISION.original_startup，并按 TRACK 的精确打印中心对应 OCCLUSION。在本轮 strict/reference/translated 关闭的参数下，验证 `base_startup == recently_changed || published_nonzero || occlusion_startup`。

| ON 实际核验 | 无球 | 球 |
|---|---:|---:|
| 新tag / TRACK行数 | 20565 / 20565 | 47540 / 47540 |
| 整数ns来源匹配 | 20565 | 47540 |
| original startup 匹配 | 20565 | 47540 |
| 布尔 OR 匹配 | 20565 | 47540 |
| prior published非零 | 2137 | 6176 |
| prior published零 | 18428 | 41364 |
| prior dynamic非零而published零 | 1316 | 4226 |
| 缺失或歧义对应 | 0 | 0 |

这些行证明当前真实运行使用了已发布历史项，不能将 dynamic非零/published零的行数直接解释成“删除了同样数量的框”。关闭组没有新 tag，其来源式由冻结源码、实际参数和原二进制身份核对；关闭组没有 tag 属于预期，不记为丢失日志。完整逐行来源证据为 `.fapp_ball_debug/published_startup_20261011/published_startup_rows.json`，报告 JSON 保留其 SHA256、统计及原始样例。报告 JSON 对关闭组预期无 tag 的长列表仅保留总数及前三例，完整列表留在未修改的 scratch summary，并记录其 SHA256。

两组 dense V2 同样按唯一 stamp+精确打印中心核验 `TRACK.changed == parent_changed && dense_changed`：无球 OFF/ON 共 20562/20565 行，球47553/47540行，全部匹配，本轮没有打印阈值边界、未匹配或歧义行。父false/稠密true 的候选在无球两组各264行、球两组各458行，actual changed 均false；父true/稠密false 在无球各161行、球218/217行，actual changed 均false。缺失、歧义或打印边界原规则仍为 unknown，不猜配或判成 false。TRACK.confirmed 在后续拒绝之前，不能替代实际输出判定。

## 配对、输出覆盖与资源

| case / 模式 | 源输入 | callback | 输出帧 | 缺callback | 缺输出 | callback p50 / p95 / max (ms) | 回放进程CPU均值 | 峰值RSS (KiB) |
|---|---:|---:|---:|---:|---:|---|---:|---:|
| noball OFF | 763 | 760 | 759 | 3 | 4 | 73.9618 / 85.703685 / 158.022 | 74.26% | 155384 |
| noball ON | 763 | 760 | 759 | 3 | 4 | 74.22305 / 86.62256 / 157.898 | 74.46% | 155560 |
| ball OFF | 1727 | 1715 | 1714 | 12 | 13 | 75.1903 / 92.41595 / 160.362 | 70.68% | 163656 |
| ball ON | 1727 | 1715 | 1714 | 12 | 13 | 75.2612 / 92.25673 / 165.298 | 70.85% | 163996 |

原四输出 loader 检查和观察器 stamp 集合检查均一致，但不表示所有源输入都有输出。每组均有1个已进入 callback 的早退未输出；完整缺 callback/output 的 stamp 原样保留。113个有效球标签的输出均存在，无球原292个5 Hz anchor全部存在；球原5 Hz全anchor两组均缺2个，不能隐藏成零框。CPU只测 mapping_node 的user/system ticks，100%为一个逻辑CPU；不含播放器、master、观察器、FAST-LIO、规划器及全机负载，采样边界有插值。callback时间不含同步等待，观察器输入到states延迟含同步和ROS传输。

同包 OFF/ON 的 callback 序列相同，但共享 callback 内无球 SYNC_POSE/候选几何/plane 差异为5/1/1个stamp，球为12/4/4。因此回放输入、配置和二进制相同仍不足以证明内部配对及几何全相同，不把全部框差异绝对归于历史项改变。

play_end 与約2秒 settled 的 pending 无球两组均4/4、球均13/13，两个checkpoint间新增states均0，末1秒pending均为空。无球延迟趋势OFF/ON为0.109822/0.043506 ms/墙钟秒，首末10秒均值差+18.5233/+15.6710 ms；球趋势-0.091189/-0.089012，均值差-9.3973/-7.4231 ms。窗口未显示持续单调增长或末端队列持续积累；pending可能是未同步callback或既有早退，不仅凭正趋势或pending数判持续积压。保留全部窗口/checkpoint原数据，不新增数值判据，`sustained_backlog_pass` 保持 `null`。

## 查询复用的跨二进制实测参考

本轮 OFF 与上一轮 `dense_veto_enabled` 配置在去掉唯一新增且为false的 published 开关后一致，同输入SHA；两者二进制分别为本轮d997…与旧c722…。

| 实测参考 | 旧V2 enabled p95 | 本轮source OFF p95 | 差值 |
|---|---:|---:|---:|
| 无球 | 97.4811 ms | 85.703685 ms | -11.777415 ms |
| 球 | 101.4458 ms | 92.41595 ms | -9.02985 ms |

这支持本轮版本实测 p95 更低，不能将差值绝对归因于查询复用。无球旧/新callback集合有旧独有1个，球新独有1个；共享pose/候选几何/plane差异分别7/2/2与22/8/7个stamp，跨时间的桌面负载也未隔离。历史原metrics和日志哈希及具体差异保留在JSON，查询分支审阅不替代实测结果。

## 来源诊断、验证与版本快照

实现依据 `.fapp_ball_debug/next_evidence_20261011/dynamic_source_risk_audit_v2.json` 是上一轮固定候选/关联历史上仅改来源项的诊断：四组原confirmed重建不一致均0，首次发布fixed-history-dependent为9/7/7/4；球两组原111/113及27/27中落入 latch-first/fixed-history-dependent/prefix-latch 的命中均0。它不是本轮真实回放，历史固定的诊断不能预测实际 nonpublication 对后续confirmed/shape/association/novelty的影响；事后GT对应也不证明真实点来源或保证删框。唯一远离阈值的 novel 日志 fallback 原样保留，不掩盖缺EVIDENCE行。

本轮运行前主 `devel` 已重新编译，产物时间晚于本轮cpp/header修改；原 main_build.log、默认7个和published+dense+competition 8个真实ROS用例、3个CTest及3份launch解析均通过。默认launch新flag=false/dense=false，实验解析新flag=true/dense=true，正式FAPP解析新flag=false/dense=false/competition=true。这些验证不补齐未测旧142、静态467或完整场景矩阵。

源码及source snapshot在冻结和四次关闭后SHA一致，四次job/metrics二进制SHA均一致。以下仅是本轮冻结/关闭/打包时的身份，不声明后续live源码或产物永久不变：

| 文件 | SHA256 |
|---|---|
| mapping_manger.cpp | `850f8d02d707c9ada618c6c724153fcf2127188a95c42324c6b180d0f3017a4a` |
| mapping_manager.h | `2d785e34b98f70a5ead3fa6c823f4308015febc3ceebf6b23ae057cd9474aba5` |
| uav1_detection.launch | `0a654462a5aae440f87cfac93507873910c4b8e740b148242809c5ec1cf1376e` |
| convergence_smoke.py | `8bef4747a641e3c9e641d9c1accd08d2728964453da5f329ebe296a545e2bbfd` |
| mapping_node | `d997d35799635ce97a6a21270f5c64f6d7c66b46d439baa9af0c6380e6ba7851` |
| noball_raw_input.bag | `12af567b7595d3361c815a13fbc4b87410a505fee3e38cd5860c1365015087f2` |
| ball evaluation.bag | `8e95b2765978c0d773d9f4a59dc4800e30b2d16d30c355d4489ff19b0b205310` |

联合验收仍有已测通道失败，旧项未测为null；报告保留结论和缺口，不自动开启正式新增强项。
