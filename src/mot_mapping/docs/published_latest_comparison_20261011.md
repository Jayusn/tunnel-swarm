# 已发布来源 V2：最近动态确认必须确实发布

本轮**仍未达标**：启用后无球框数为 `0/104/610/56`，总数770；通道610仍超过冻结607上限3框，不能用返回阶段低于96抵消。球由控制111/113降至110/113，三帧均27/27，仍满足108/113及26/27阈值，但实际丢失1个控制命中。四组真实回放 whole callback p95 均小于100 ms。旧142、静态467及完整真实场景组合未测，保持 `null`。

两组均为 F06 + density_split + confirmed_reacquisition + measured_body_position + bidirectional_static + 点源竞争实验组合，dense V2和既有NN查询复用共同保留。110/113或111/113不是正式 A04 + 点源竞争的默认率；正式新增增强项仍默认关闭。完整原metrics、113行及原GT、控制kept/lost、日志样例、闭合与SHA证据见 [结果JSON](published_latest_results_20261011.json)。

## 公式与运行范围

沿用 `detection/published_startup_source_enabled`，源码及launch仍默认false。开启语义从V1的pub非零收紧为：

```text
historical_startup = enabled
    ? (last_published_stamp != 0 && last_dynamic_stamp == last_published_stamp)
    : (last_dynamic_stamp != 0)
```

这只替换普通original startup历史项。关闭保留legacy动态确认历史；开启要求最近动态确认时间等于实际发布时间。没有新增状态或时间阈值；recently_changed、reference/occlusion/translated、strict、bidirectional补充、原motion 0.6 s、形状及其他last_dynamic用途不变。显式实验参数为 `published_startup_source_enabled:=true`。六字段 `FAPP_PUBLISHED_STARTUP` 布局不变，记录本帧更新前的track、dynamic ns、published ns、recent、historical和base startup。

ROOT从原session66273得到exit0、`DONE VERIFIED ball_published_latest_enabled_20261011`、`PASS actual latest-published-history four runs`并确认四次full回放及evaluator全部结束后才打包。仍使用原run_case、完整1×相同输入和播放命令，除新flag ON/OFF外参数相同，四次二进制相同。未重跑算法、修改原driver PASS、评分或GT。

| 原case | dry-run/job/runtime | PUBLISHED_STARTUP行 | DENSE_CHANGE行 |
|---|---|---:|---:|
| noball_published_latest_control_20261011 | false/false/false | 0 | 20565 |
| noball_published_latest_enabled_20261011 | true/true/true | 20607 | 20607 |
| ball_published_latest_control_20261011 | false/false/false | 0 | 47499 |
| ball_published_latest_enabled_20261011 | true/true/true | 47512 | 47512 |

四次原driver均为唯一的 `PASS completed <原case>`；terminal success/input_unchanged/output_finalized均true，master/record/node/audit/play返回码均0，bag已关闭且无同名bag.active。原matrix_terminal success=true、cases=4、all_actual_flags_verified=true。metrics参数与各自dry-run/job一致，dense runtime四组均true。原PASS证明完成，不代替质量门槛。

## 固定门槛及真实失误

| 指标 | 冻结要求 | OFF | ON | ON结论 |
|---|---:|---:|---:|---|
| 球有效标签 | ≥108/113 | 111/113 | 110/113 | 通过门槛，损失1个控制命中 |
| 连续三帧 | ≥26/27 | 27/27 | 27/27 | 通过 |
| 无球capture | ≤0 | 0 | 0 | 通过 |
| 无球initial_stationary | ≤109 | 104 | 104 | 通过 |
| 无球carried_corridor | ≤607 | 630 | 610 | 未通过，超3 |
| 无球returned | ≤96 | 139 | 56 | 通过 |
| 无球whole callback p95 | <100 ms | 86.58222 ms | 87.1011 ms | 通过 |
| 球whole callback p95 | <100 ms | 92.646915 ms | 93.168715 ms | 通过 |
| 旧142全部保留 | 全保留 | null | null | 未测 |
| 静态467帧框数 | ≤6 | null | null | 未测 |
| 完整真实场景验收 | 全通过 | null | null | 本报告未测 |
| 完整F06六项synthetic组合 | 原fixture断言通过 | — | 6/6 PASS | 已测，另列验证证据 |

无球分阶段差值为 `0/0/-20/-83`，总873→770；球总2098→1943，总框数不能替代GT命中。113个有效冻结标签逐行核对了原GT、点数、时间、baseline hit和三帧资格。控制命中kept=110、lost=1、gained=0，lost为以下第三行：

| ON有效漏检stamp | 时间(s) | 点数 | OFF→ON | 三帧资格 | 两侧输出 |
|---|---:|---:|---|---|---|
| 1791429339268123627 | 121.9436450005 | 6 | miss→miss | false | 都存在 |
| 1791429342566560745 | 125.2420821190 | 6 | miss→miss | false | 都存在 |
| 1791429352065476179 | 134.7409975529 | 10 | hit→miss | false | 都存在 |

134.7409975529秒新增漏检与 `.fapp_ball_debug/published_source_followup_20261011/history_limit_frozen_risk.json` 的method B风险stamp完全对应。实际hit和box_hit均true→false，是有输出但目标未命中，未改变原评分条件。原metrics的 `lost_current_hits=[]/current_recall_preserved=true` 指旧CURRENT约50命中基线，不是本轮111命中控制组；不能据此说保留全部控制命中。

## 整数时间戳及dense AND核验

使用既有parse处理可明确重拼的PCL stderr插入，原日志未改。ON逐行验证 `historical == (pub_ns != 0 && dyn_ns == pub_ns)`，不是近似时间比较，也不缩写成dyn大于pub。按唯一stamp+track连接BIDIRECTIONAL_DECISION.original_startup，再通过TRACK精确打印中心连接OCCLUSION。在实际strict/reference/translated关闭配置下验证 `base == recent || historical || occlusion`。

| ON核验 | 无球 | 球 |
|---|---:|---:|
| source/TRACK行 | 20607/20607 | 47512/47512 |
| 历史整数式、原startup、OR匹配 | 各20607 | 各47512 |
| pub非零且dyn相等，historical=true | 940 | 3284 |
| pub非零且dyn不等，historical=false | 1382 | 2922 |
| pub为零，historical=false | 18285 | 41306 |
| 缺失/歧义连接 | 0 | 0 |

OFF不输出新tag属于预期，来源由冻结源码、参数和二进制核对，EXPECTED-OFF长列表仅在scratch保留并给出计数。整数匹配不等于这些行对应同量被删除的框。完整来源行保存在 `.fapp_ball_debug/published_latest_20261011/published_startup_rows.json`，结果JSON记录大小和SHA。

dense V2按同stamp、唯一精确中心验证TRACK.changed与parent AND dense。无球OFF/ON共20565/20607行，球47499/47512行，非边界行零不一致、零未匹配/歧义。无球ON与球OFF各有1行打印阈值边界保持unknown，另外两组没有边界unknown；不能将unknown判false。TRACK的confirmed在后续拒绝前，不能替代输出或GT评分。

## 覆盖、配对和资源

| case/模式 | 输入 | callback | 输出 | 缺callback/输出 | callback p50/p95/max(ms) | 回放进程CPU均值 | 峰值RSS(KiB) |
|---|---:|---:|---:|---|---|---:|---:|
| noball OFF | 763 | 760 | 759 | 3/4 | 74.3751/86.58222/158.946 | 74.10% | 154948 |
| noball ON | 763 | 761 | 760 | 2/3 | 75.0038/87.1011/159.182 | 74.83% | 155308 |
| ball OFF | 1727 | 1714 | 1713 | 13/14 | 75.647/92.646915/158.469 | 70.82% | 164200 |
| ball ON | 1727 | 1714 | 1713 | 13/14 | 75.3205/93.168715/164.408 | 71.17% | 164152 |

原四输出loader及观察器集合检查均一致，不代表每个输入都有输出。每组都有1个callback早退未输出；完整缺失stamp与原audit保留。113个有效球标签均有输出，无球原292个5 Hz anchor全部存在；球全5 Hz anchor两组均缺2个，不隐藏成零框。CPU只测mapping_node进程ticks，100%为一个逻辑CPU，不含播放器/master/观察器/FAST-LIO/规划器或全机负载。whole callback不含同步等待，audit延迟包含同步和ROS传输。

两组callback集合不同：无球ON多1个，球双方各有1个独有callback；共享SYNC_POSE/候选几何/plane差异无球3/1/1、球11/5/4个stamp。相同输入、参数及二进制不能证明所有内部配对或几何相同，不能把全部框差异绝对归于新历史条件。V1 OFF→V2 OFF只是跨二进制重复参考，两者NNcache共同存在，不作为独立缓存收益实验。

play_end/约2秒settled pending为无球OFF4/4、ON3/3，球两组14/14；checkpoint间新增states均0，末1秒pending均空。无球延迟趋势OFF/ON为0.070404/0.068215 ms/墙钟秒，首末10秒均值差+15.5324/+16.4886 ms；球趋势-0.099709/-0.110764，均值差-9.7541/-9.6245 ms。窗口非持续单调增长，未显示尾部积累；pending可能是未同步或既有早退，正趋势/pending总数不能单独证明持续积压。原窗口和checkpoint均保留，不发明新数值判据，sustained_backlog_pass仍null。

## 冻结预测、验证与版本身份

history_limit_frozen_risk的method B是在V1固定候选/关联历史上的诊断：预计通道减少21个、球111→110/113且three27，134.74秒是非three风险。真实本轮ON通道610仍超3，不能把旧预测当实测删除量或验收通过。诊断没有模拟形状、关联、novelty放宽和失去输出后保留反馈；源文件、方法和限制随结果JSON的artifact manifest保存。

主devel已编译，产物时间晚于本轮cpp/header；预输入的默认7个、published+dense+competition8个真实ROS回归、3CTest和3launch解析原证据均通过。ROOT随后从原F06执行handle28646得到exit0，原execution_launcher_stdout.log含 `PASS full frozen F06 candidate FULL_CASES all6 original fixture assertions`；terminal记录六项returncode均0、all_original_assertions_pass=true，六份driver原PASS及SHA均核对。

完整F06配置的六项为six_radial_toward、six_radial_away、six_flat、six_wall_reversal_ego、six_occlusion、new_corner，方法分组为body_bidir_static，实际71项检测参数来自冻结detector_params.json（dense/latest published均true，reference distance=0.04、fraction=0.5）。与继承的完成job相比仅lidar/odom两个topic改为原fixture输入，fixture原外参[0,0,0]保持。prepared_manifest证明原输入、时间、GT及断言AST不变；原/注入脚本各66个静态assert节点逐项相同，去掉唯一params JSON注入赋值后整个AST相同。66包含未选择case分支，不能写成此次运行执行66次断言。

证据位于 `.fapp_ball_debug/published_latest_20261011/full_candidate_regression/`：prepared_manifest.json、detector_params.json、参数注入diff、六个原driver、执行前manifest及runs/latest_source_B/terminal.json。执行前后node均87c91f…、源码均94232c…，与本轮真实四回放的冻结身份相同且未改变。六driver完整SHA、method/参数及terminal原内容加入JSON的 `full_candidate_synthetic_six`。该synthetic验证不证明真实回放p95，不替代旧142、静态467或完整真实场景，相关null和本轮未达标结论保持。

四次job/metrics二进制SHA与冻结manifest一致，source snapshot及live源码在本轮关闭打包时匹配；身份仅限该时点，不承诺未来live永久相同：

| 文件 | SHA256 |
|---|---|
| mapping_manger.cpp | `94232c939ba602c977d1d3aae79a0d37544af80d2522a885922fa7b09f854228` |
| mapping_node | `87c91fdaf05a5eef7cf2f1e35a4f0aca47a6fb229792409edaec95fd2710774e` |
| mapping_manager.h | `2d785e34b98f70a5ead3fa6c823f4308015febc3ceebf6b23ae057cd9474aba5` |
| uav1_detection.launch | `0a654462a5aae440f87cfac93507873910c4b8e740b148242809c5ec1cf1376e` |
| convergence_smoke.py | `8bef4747a641e3c9e641d9c1accd08d2728964453da5f329ebe296a545e2bbfd` |

两组原无球输入SHA为12af567b…5087f2、球输入为8e95b276…b205310，完整SHA及stat/原terminal未变检查见JSON。尚有通道实测失败、1个控制命中损失及未测旧项，新增开关继续默认关闭。
