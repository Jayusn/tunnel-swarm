# 同步配对与原形状覆盖诊断（2026-10-11）

本次无球全包回放输出 **903 框（0/105/650/148）**，冻结 native 同频基线为 **812 框（0/109/607/96）**，通道和返回均增加，未通过无球不增要求。它是已完成单次算法回放的诊断，**没有原驱动 PASS，不是正式合格回放或联合验收成功**。[完整逐项结果与证据 SHA](sync_source_results_20261011.json)保存全部 761 条实际配对、1166 条有效原形状测试、903 条当前发布、166 条相对冻结新增几何、57 条相对原同参数新增几何，及完整命令、参数、统计、终态和证据清单。

## 完成凭证与原异常

五个 ROS 子进程 master/record/node/audit/play 均退出 0；原输入全程 1x 回放、输出包已关闭，terminal 记录成功且输入 SHA 不变。但 scratch launcher 起初将 stdout 写入原驱动的最终完成标记路径，原驱动在完成算法、保存 terminal 后以独占创建方式写标记时发生 `FileExistsError`，外层退出 1。原异常日志已逐字恢复，SHA256 为 `20d2bc48d96b2d80bf32d6ae934692f785e360b7f885dc9dc27c7b79d950a9e6`；它包含 traceback 中写 PASS 的源码行，但没有实际 `PASS completed` 完成标记。

初次后处理曾短暂使用重建 PASS 标记，随后移除该标记并恢复原异常；初次结果保存在 `superseded_temporary_marker*`，`first_final_*`保存补充 gap 汇总前的同一关闭包结果。最终统计在恢复原异常后重新生成，只针对当前案例将原 evaluator 的完成门槛替换为成功 terminal、全部子进程退出 0、完整已关闭输出包及输入 SHA 验证，评分、冻结判据及基线的原完成门槛保持原样。`verified_completion.json`明确记录这一比原驱动直接 PASS 更弱的来源。没有重跑检测器、重新回放或修改输入。

## 对照与覆盖

| 阶段 | 冻结 native | 原同参数完成回放 | 当前诊断 | 当前－冻结 | 当前－原同参数 |
|---|---:|---:|---:|---:|---:|
| capture | 0 | 0 | 0 | 0 | 0 |
| initial_stationary | 109 | 105 | 105 | -4 | 0 |
| carried_corridor | 607 | 650 | 650 | +43 | 0 |
| returned | 96 | 95 | 148 | +52 | +53 |
| 总计 | 812 | 850 | 903 | +91 | +53 |

原同参数组 `noball_final06_body_static_current_20261010` 的参数与 `play_command`逐项完全相同；两次 761 个实际 callback 时间戳集合及四路各 760 个输出时间戳集合完全相同。当前相对原同参数组新增几何观察 57 个、旧观察消失 4 个，净增加 53 框均在 returned。相同时间戳集合不证明配对位姿或历史状态相同；历史组没有实际配对日志，不能把增加的 53 框归因于同步配对变化或两个 DEBUG 输出。

观察器收到全部 763 cloud／15262 HF odom；761 callback 包含 1 个无四路输出的初始化回调。两个真实输入没有 callback，纳秒时间戳为 `1791427375403943062`、`1791427385510099649`，未当作源包缺帧或补齐。四路输出均 760 帧，时间戳集合一致。回放结束与等待 2 秒后的集合相同，没有收尾后新增状态；最新一秒没有待输出输入，三条永久缺输出仍保留。

无球累计框是误检代理，操作者及手臂可能移动，未做全场景物理标签。新增几何采用原工具同时间戳内 0.25 m 中心的一对一最大匹配后最小距离，不是净框数或人工场景类别；不同回放轨迹 ID 不能直接互相比。

## 实际同步配对

| 实测量 | p50 | p95 | 最大／范围 |
|---|---:|---:|---:|
| paired odom－cloud 有符号时差 ms | -0.234365 | 1.547337 | -95.554828～53.989172 |
| paired odom－cloud 绝对时差 ms | 0.441551 | 1.746416 | 95.554828 |
| 日志 position 对同实际 header HF position 距离 µm | 0.046602 | 5.432134 | 7.703836 |
| 实际 position 对最近 header HF position 距离 mm | 0.0000469 | 0.005681 | 5.258885 |

761 次实际配对中，750 次 odom header 等于离 cloud 最近的真实 HF header，11 次不等，其中 capture 1、carried_corridor 2、returned 8；initial_stationary 均相等。所有实际 odom header 均存在于输入 HF 消息。DEBUG 位姿约六位有效数字，同 header position 最大 7.70 µm 差异符合日志舍入。同步离群值集中在 returned，与框数增量并存仅是相关观察。

输入没有 TF，检测器没有接收 GT；最近 HF 对照使用真实消息，不插值，上述距离不是定位真值误差。JSON保留每条 pair/header/pose/最近 HF 对照及分阶段统计；未改变 `ApproximateTime`配对、队列或 `ros::spin()`。

## 原形状判据实际覆盖

原测试实际有效时共 1166 行，以原 forward/current dense 与 backward/previous dense 整数计数重算方向比例，原 MAX 舍入值及 pass 位一致。固定原门槛 0.45 下，MAX 通过 1120 行，离线 MIN 通过 1042 行，78 行仅 MAX 通过。56 行有效原测试跨 frame gap，其中 9 行仅 MAX 通过。没有实施 MIN 或用它回放。

| 几何观察集合 | 总观察 | 原 shape 有效 | shape 未知 | 仅 MAX 通过 | 已确认 gap 重获 | gap 重获且 shape 未知 |
|---|---:|---:|---:|---:|---:|---:|
| 当前全部已发布 | 903 | 432 | 471 | 17 | 101 | 81 |
| 相对冻结 native 新增 | 166 | 31 | 135 | 3 | 54 | 45 |
| 相对原同参数新增 | 57 | 8 | 49 | 1 | 14 | 13 |

相对冻结新增几何分阶段为：静止 8 个（有效 shape 3）；通道 86 个（有效 19、仅 MAX 通过 1）；返回 72 个（有效 9、仅 MAX 通过 2）。当前全部输出的 17 个仅 MAX 通过观察中，2 个属于有效 shape 跨 gap。没有 shape 日志表示原测试未记录，必须保留未知／null，不能解释成 0、形状不一致或通过；大量重获输出没有有效 shape，不能用其余局部比例代替。

这些数值仅覆盖已发生的测试和输出，不能从事后 MIN 筛查推算修改算法后的轨迹、最终框数或 ball recall；本次没有新判据、保护条件、插值或额外状态机。

## 两次回放的第一实质分歧与受限组件复现

只读对比原 850 框与当前 903 框日志，761 个 callback header 序列完全相等。最早非耗时差异是 `1791427329609696865` 的 ikd-Tree Size 9964→9955。源码 `size()`返回 `TreeSize`或 rebuild 临时值，包含待删除节点；`validnum()`另行扣除 invalid 节点。树节点数差异不能证明有效点集、最近距离或检测证据已变化。第一实质分歧之前，共同 FAPP 几何、证据、轨迹和全部 plane 系数日志相同。

第一实质分歧 header 为 **`1791427375709764242`**，相对第一 callback 是 **56.499882221 s**；相对冻结评分器 `source_meta.start=1791427318.9690928`是 **56.740671396 s**。两个时间原点不同，该帧在两者下均属 returned；原审计的 first-callback 秒数及 phase 不能混写成冻结评分器的阶段计时。

该帧第一条 plane 从旧 1130 点、系数 `[0.0354203,-0.999163,0.0204718,1.90442]`变为新 405 点、系数 `[0.266728,0.108259,0.957672,-0.049359]`，均标 background。DBSCAN 簇数 75→73、后续候选 54→45，先于关联／shape／发布；当帧框数仍均为 2，空间差异先于计数差异。Input Size 同为 6670，只说明数量相同。

原 plane、candidate 几何／尺寸／点数及全局地图 `avg_dist/var_dist`、background/reference/point-source 日志各仅这 1 帧不同；另外 **710 个有候选帧**的对应多重集合在日志精度下相等。下一帧原 evidence 有一次 history1 ray_supported 22→23，之后恢复相等。TRACK 与双向 decision 却在 **197 个 returned 帧**持续不同，支持单帧 plane／候选扰动后关联和历史状态传播的观察链条；没有证据把它写成持续全局地图距离变化，日志相等也不证明未记录的内部地图完全相同。

当前该帧实际配对 HF header 为 `1791427375675761700`，比 cloud 早 34.002542 ms；最近 HF header 为 `1791427375709510088`。使用同一输入真实 HF 精度，无插值，position 相差 **2.031707 mm**。按原代码严格水平 5 m crop，两者各保留 **9012 点**，但 actual 仅保留 source index5816，nearest 仅保留7151，各替换一个边界点。立即前一 callback 的实际与最近 HF header 相同，crop9319 点且索引／坐标 hash 一致。

受限 PCL 组件仅复用该帧及上一帧真实 registered XYZ，原 5 m crop、当前0.05 m voxel、历史0.1 m voxel和原 plane removal/PCL 参数，不运行 ROS、完整检测器、tracker 或评分。两次组件均退出0：实际配对 crop的 **全部6条 plane**在六位有效数字日志精度下逐条复现新日志；最近 header 反事实 crop的 **全部6条 plane**逐条复现旧日志。两者 voxel6670／history3718 点相同，去平面后分别3183／2816 点。这比单纯 crop边界计数更直接地支持“配对敏感的crop输入能触发plane分歧”这个有限机制；**旧回放实际 paired header/pose仍未记录，不能把nearest当成旧actual，不能宣称复现旧完整运行或证明+53框因果**。本地 PCL 此路径采用默认固定seed12345，没有wall-clock随机seed依据；这也不保证所有未记录内部状态确定。

`first_divergence_audit`嵌入原 bounded summary、crop、组件结果、log summary、源码／运行provenance和验证。原始完整帧、每类差异、stdout被stderr中断后离线重接的11对片段/运行、源快照及组件源码/二进制全部在 `.fapp_ball_debug/first_divergence_20261011/`保留，并逐文件记录SHA；大体积原始逐行差异保留本地且由SHA索引，不重复嵌入结果。审计按精确header和原行号进行，不改变原日志或冻结评分。

提交前在字节一致恢复头文件后重新构建主工作空间，`main_build_precommit.log`、`ctest_precommit.log`及`precommit_validation.json`记录实际devel更新和3项CTest通过。当前二进制SHA仍为 `ab1d273824077379f74f052f66ac699ecff7dbf1e79b40c9bb64e44660214eb8`，内容与诊断回放相同；提交前mtime `1791685094551787741`与原回放产物mtime `1791683912777656674`分开保留，未改写原运行来源。

## 时间、资源与验证边界

整回调实测 p50/p95/max 为 **75.5728/91.8731/165.999 ms**，p95 数值低于原 100 ms 间隔。观察器输入接收到 state 接收 p95 为 **135.542579 ms**，包含同步、队列和传输，不能混写为 callback 时间。回放阶段 mapping_node 平均 CPU **77.5398%**，采样 CPU p95/max **99.9207%/111.9188%**；100% 表示一个逻辑核。峰值 RSS/HWM **155472 KiB（151.83 MiB）**，不含 FAST-LIO、规划或全机负荷。

默认 7 案例和 competition 8 案例 ROS 回归与本回放曾短暂并行；耗时及资源是并发筛查测量，不能作为最终单独运行性能验收或证明 DEBUG／配对使算法提速。evaluator 的 `final_10hz_processing_p95_pass=true`仅保留为原评分器的数值判断，不能据此宣称单独性能或联合通过。球冻结召回、旧 142 动态命中、467 静止时刻、完整场景回归及单独最终性能在本次配置诊断均未测，结果为 null。

实际主工作空间构建更新 `devel/lib/mot_mapping/mapping_node`，生成时间晚于本次源码。`main_build.log`记录 mapping_node 重新编译链接及三个 CTest 目标生成；默认 ROS 7 案例、competition ROS 8 案例通过，3 项 CTest 通过，默认和正式 FAPP launch 参数解析通过。这些代码验证与未通过的无球代理验收分别保留。

## 源码、参数与证据来源

本次 detector 源码仅在 `mapping_manger.cpp`加入两个 `ROS_DEBUG_STREAM`：`FAPP_SYNC_POSE`记录实际 cloud／配对 odom header 和位姿；`FAPP_SHAPE_SOURCE`记录原形状测试轨迹、帧 gap、dense 点数、方向整数计数和原 MAX 比例。DEBUG 开启时增加日志开销；检测、同步、判据、发布、正式 detector 默认值、配置和启动参数未变。实验沿用已完成主体静态组全部原参数，独立 localhost master 端口 15531、原输入全程 1x，没有截短或调参。

| 对象 | SHA256 |
|---|---|
| 原输入 `noball_raw_input.bag` | `12af567b7595d3361c815a13fbc4b87410a505fee3e38cd5860c1365015087f2` |
| 当前 `mapping_manger.cpp` | `1d7badef7dba025cf70eaad7d54ece4ae9ad37f05b1de73d04bf34df8a9238db` |
| 当前主工作空间 `mapping_node` | `ab1d273824077379f74f052f66ac699ecff7dbf1e79b40c9bb64e44660214eb8` |
| 原同参数 `mapping_node` | `08385dad4a1350f0d2a3cfb1efc080e20eaa6d42a57c7ea64149c958d42afc20` |
| 冻结 native `mapping_node` | `db2ea4a6f669f0b24e42d2a6246625e3a6df2af9bfd66ee1b0c66c08b48c9211` |
| 原异常 driver log | `20d2bc48d96b2d80bf32d6ae934692f785e360b7f885dc9dc27c7b79d950a9e6` |

JSON 的 `evidence_manifest`包含第一分歧全部现存归档及提交前验证，并逐项保留输入／关闭包、job、terminal、原异常、audit、输入时间戳、资源、节点日志、原评分输出、完整诊断与归档初次结果、源码／辅助脚本、主构建、15 个 ROS 回归及三项 CTest／两种 launch 解析的现存证据文件 SHA。历史二进制 SHA 来自各原 job，不表示当前文件仍是历史版本。`run_design`保存实际命令和全部参数；原始目录为 `.fapp_ball_debug/sync_source_20261011/`及 `.fapp_ball_debug/bidirectional_20261009/noball_body_static_sync_20261011*`。

交付只新增本页和结果 JSON；原 scratch 证据、冻结标注／评分器与既有正式报告均保留。最终仍是无球代理不通过，完成来源比原驱动 PASS 弱，同步因果、MIN 新算法及未测验收项没有成功结论。
