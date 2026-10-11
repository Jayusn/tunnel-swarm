# 背景参考半径 .04 → .05：主场景通过，旧命中保留失败

本轮联合验收 **不通过**：.05 的近期球110/113、连续三帧27/27、无球四阶段0/103/579/56和static0/467通过，但旧下午fast冻结45个命中只保留44，旧142个命中共保留141。latest新增1个命中不能抵消该损失。正式启动仍为A04＋点源竞争，新增强开关保持关闭；本报告不推荐启用这个候选组合。

本轮沿用F06＋density_split＋confirmed_reacquisition＋measured_body_position＋bidirectional_static实验组合及点源竞争，dense V2与published strict B均为true。110/113属于这个实验组合，不是正式A04＋点源竞争默认率。仅私有参数`background_reference/distance`从.04变成.05，`background_reference/fraction=.5`不变；未改源码、launch、默认参数或标签。`detection/published_startup_source_enabled`默认false，启用公式仍为`last_published_stamp != 0 && last_dynamic_stamp == last_published_stamp`；dense V2仍为`parent_changed && dense_changed`。

## 真实执行与主场景

两项.04控制直接复用已闭合的`noball_published_latest_enabled_20261011`、`ball_published_latest_enabled_20261011`，仅新执行`noball_reference05_enabled_20261011`、`ball_reference05_enabled_20261011`两次完整1×回放及原evaluator。它们共享同输入SHA、同节点和源码，参数字典唯一差异为distance；这是两旧控制＋两新场景，不是新跑四场。dry-run、job、metrics和`/mapping_node/background_reference`运行参数均确认实际.04/.05与fraction=.5，pub/dense两项启用也有真实DEBUG记录。

| 指标 | .04控制 | .05候选 | 固定验收 |
|---|---:|---:|---:|
| 近期球 | 110/113 | 110/113 | ≥108/113 |
| 连续三帧子集 | 27/27 | 27/27 | ≥26/27 |
| 无球capture | 0 | 0 | ≤0 |
| 无球initial stationary | 104 | 103 | ≤109 |
| 无球carried corridor | 610 | 579 | ≤607 |
| 无球returned | 56 | 56 | ≤96 |
| 无球whole callback p95 ms | 87.101100 | 85.585000 | <100 |
| 球whole callback p95 ms | 93.168715 | 92.000370 | <100 |

新无球evaluator的`before=0/104/610/56`是真实.04控制，不能当成验收上限0/109/607/96。四阶段分别比较，不跨阶段抵消。113个冻结时刻逐行保留在结果JSON：控制110个命中全部保留，新增0、丢失0；三个继承未命中为121.943645、125.242082、134.740998秒（最后一项标签戳1791429352065476179），并非本次半径新增损失。

原driver均PASS，terminal确认全部进程返回0、输入前后不变及bag收尾。四输出在实际输出时刻的戳一致，但不表示输入覆盖完整。无球.04/.05分别callback761/760、输出760/759，输入763；球分别callback1714/1715、输出1713/1714，输入1727。控制与新场景callback戳序列不同，共享时刻pose/candidate/plane也存在少量差异，完整差异戳保存在JSON；这里报告同包实测差异，不把性能变化绝对归于单项算法或桌面调度。

## 旧资格与原谓词

`static_reference05_enabled_20261011`、`afternoon_reference05_enabled_20261011`、`latest_reference05_enabled_20261011`均为原输入完整1×回放。与主场景参数仅输入topic改为`/UAV1/fapp/cloud_registered_throttled`；原输入SHA与已闭合v4参考一致。原localStage55命令没有历史输入SHA清单，不能补证更早字节身份。

| 项目 | 冻结要求 | .05实际 | 结论 |
|---|---:|---:|---|
| static原时刻 | 467全覆盖，累计框≤6 | 467全覆盖，0框，额外0框 | 通过 |
| afternoon fast | 保留原45 | 44/53，原45丢1 | 失败 |
| afternoon slow | 保留原59 | 59/59 | 通过 |
| latest | 保留原38 | 原38全保留，实际39/47 | 通过；新增1单列 |
| 原142逐戳 | 142全保留 | 141保留、1丢失 | 失败 |

唯一失误标签`1791101224825826816`、t=145.67184138298035秒。实际input及states戳为`1791101224825826883`，比标签晚67ns；原评分要求最近输出与标签差值严格<1000ns，该条件满足。输出存在，但保留真值点0、hit=false，不能归因于输出缺失；该输出receipt latency为41.391794ms。旧评分还要求框中心距离<.25m、距离<.025m的保留真值点数量≥max(3,0.5×真值点数)，保留原谓词，没有套用近期113的尺寸条件。afternoon112标签均受历史浮点时间戳转换影响，与输出不字面相等，但全部差值<1µs、最大127ns；标签戳与实际输出戳在JSON分列。

latest唯一缺标签输出`1791116353425527811`、t=276.72558331489563秒，仍计miss；它原本为阴性。新增命中`1791116317825914860`独立列出。最新完整v4参考比原`latest_localStage55 --duration 310`多306个尾部输出，本轮对完整v4没有缺/增输出；不能把尾部时长差直接当误报。

## 性能、覆盖和资源

主场景使用近期原生约10Hz输入；旧数据实际频率如下，没有重采样为10Hz。旧metrics的`processing_interval_ms=200`不作为通过100ms的依据，表中统一使用真实whole callback p95。

| 场景 | 输入/ callback/输出 | 中位输入间隔 ms | Header平均 Hz | whole callback p95 ms | receipt latency p95 ms | node回放CPU平均 % | peak RSS KiB |
|---|---:|---:|---:|---:|---:|---:|---:|
| .04无球 | 763/761/760 | 约100 | 约10 | 87.101100 | 127.958525 | 74.828 | 155308 |
| .05无球 | 763/760/759 | 约100 | 约10 | 85.585000 | 128.869152 | 73.506 | 154400 |
| .04球 | 1727/1714/1713 | 约100 | 约10 | 93.168715 | 136.543016 | 71.172 | 164152 |
| .05球 | 1727/1715/1714 | 约100 | 约10 | 92.000370 | 134.109531 | 70.278 | 164676 |
| .05 static | 468/468/467 | 299.401998 | 3.856313 | 83.270555 | 119.302163 | 34.991 | 126292 |
| .05 afternoon | 515/511/510 | 299.628496 | 2.514681 | 83.560450 | 117.642336 | 18.391 | 129116 |
| .05 latest | 1075/1073/1072 | 300.004125 | 2.203082 | 88.784600 | 132.404821 | 19.550 | 134072 |

CPU为mapping_node进程user＋system，100%表示一个逻辑核，排除rosbag、FAST-LIO、规划器和整机负载。Header平均频率=(输入帧数−1)/Header跨度。receipt latency是audit subscriber收到实际输入topic至收到states的时间，含同步与传输，不能替代whole callback计时；旧场景最大receipt latency分别137.362200、575.134758、799.441847ms。

五个.05真实场景末一秒pending均为空，play_end与settled后无新增迟到states；完整pending戳与10秒窗口曲线保存在JSON。无球/球/static/afternoon/latest最终输入无输出分别4/13/1/5/3，不能把漏callback当作误报降低，也不据此新造持续积压判定，`sustained_backlog_pass=null`。缺标签输出计miss；closed-loop未测为null。

已有额外框诊断按一对一中心距离<.25m匹配且不加尺寸过滤；它不是物理误报真值。相对各自实际控制，.05无球额外4个框观察（均carried，轨迹span合计.199635983秒，采样时长.399949552秒），球额外14（initial1/carried out6/returned2，span.699267149、采样1.399835108秒）。旧static额外0、afternoon12（span.501456021、采样3.595556256秒）、latest32（span2.297840594、采样9.60012816秒）。这些新增与消失观察同时发生，不能由净数量反推物理成因；已记录的candidate source字段与track spans在JSON，未知保留未知，未新增标签。

## 合成回归、公式和可追溯性

.05完整候选在six radial toward/away、flat、wall reversal ego、occlusion、new corner六个原FULL_CASES均RC0及原driver PASS。仅注入已完成.05 job参数，fixture输入topic与既有extrinsic0保留；原输入、GT、时间及assert AST不变，静态66个assert节点一致。执行manifest、六driver SHA和terminal在JSON，合成回归通过不能证明真实场景p95或闭环性能。

主场景既有解析核对真实整数ns的published strict B公式，以及同stamp同中心FAPP_TRACK.changed的dense AND。阈值打印边界、无匹配和多匹配保持unknown（.04无球有1个打印边界），不自行判false。FAPP_REFERENCE的整数supported/dense验证fraction=.5；半径本身由实际私有运行参数证明，不由该tag反推。

本轮节点SHA256 `87c91fdaf05a5eef7cf2f1e35a4f0aca47a6fb229792409edaec95fd2710774e`，CPP `94232c939ba602c977d1d3aae79a0d37544af80d2522a885922fa7b09f854228`。main devel产物mtime不早于CPP/header，source_before、快照、run_manifest、各真实job和闭合记录一致。.05仅参数试验，继承同源码main build＋default7 ROS＋published/dense competition8 ROS（共15）＋3CTest＋3launch验证，不称其为改变默认参数后的新增默认验证。哈希描述本轮闭合时版本，后续live变化不影响冻结证据。

首轮主包`.fapp_ball_debug/reference_radius_20261011/summary.json`及attempt1 stdout保留原样，后续旧资格和六场分别追加`qualification_summary.json`、`qualification_attempt1_stdout.log`；两次打包均成功，schema通过。原启动node/play commands、完整参数、全部113及旧159标签行/142命中掩码、static467戳、四输出、缺callback、CPU/RSS、终端及原产物SHA见[结果JSON](reference_radius_results_20261011.json)。

## 后续单参筛查的已知边界

在相同.05组合上仅`detection/bidirectional_ray_evidence_enabled=true`的pilot实际完成一个无球full场景：0/103/635/80，whole callback p95=85.5522ms，carried比固定607上限多28。原driver和execution成功，筛查质量失败；`all_five_completed=false`，afternoon/ball/static/latest四场未跑，不能声称恢复fast或联合通过，ray-on合成六场也未测。随后仅`detection/reference_motion_enabled=true`的pilot也只完成一个无球full场景：0/114/826/191，whole callback p95=85.4642ms。initial stationary/carried/returned分别超过固定109/607/96上限5/219/95；原driver及execution成功，质量失败后停止，afternoon/ball/static/latest四场未跑，不能声称恢复fast或联合通过，reference-on合成六场仍未测。两个pilot保留独立的关闭manifest、原driver与实际运行参数，未更改.05主候选旧141/142的结论。
