# 实际帧 NN 与两次 Ray_E：七场质量和完整候选六用例通过

七场真实完整回放的已测门槛全部通过：近期球110/113、连续三帧27/27，旧fast45、slow59、latest38共142个原命中逐戳全部保留，static原467戳累计0框；无球ON四阶段0/103/585/56均不超过固定0/109/607/96上限。实际完整F06候选FULL6也全部原assert PASS、73项运行参数核验通过。独立callback-entry审计支持本机、上述有限冻结输入原速回放中未观察到持续回调积压，**有限冻结感知联合资格通过**。内部ROS队列未直接测量、未进入回调的原因及闭环效果仍为null。

正式启动仍为A04＋点源竞争，`detection/translated_startup_enabled`默认false。本轮使用F06＋density_split＋confirmed_reacquisition＋measured_body_position＋bidirectional_static及点源竞争、dense V2、published strict B的实验组合，background reference distance=.05、fraction=.5；110/113不代表正式A04默认率。当前报告不把实验配置写入正式默认启动。

## 新启动支持的实际语义

本轮沿用现有translated开关和fraction=.8。在`time_evidence=false`的实际帧分支，新的startup支持同时需要：已有三次实测运动等前置条件、当前候选与实际前两个历史registered raw cloud的NN对应、两次已有双向ray事件支持。普通recently_changed、published历史、reference及occlusion启动途径保留；`time_evidence=true`保留旧约0.3/0.6秒历史选择和旧translated行为，不附加本轮Ray AND。

实际帧history0为较旧一帧，history1为最近一帧。每个历史cloud的戳必须与同track实测中心的整数ns精确相等，不插值、预测或假定固定0.1秒间隔。对当前dense点的相对位置分别加当前中心和该历史实测中心，在同一历史cloud查询≤.05m的NN支持。每个历史项要求`moving_count/N >= .8`且`moving_count/N - static_count/N >= .25`，两个历史都通过才形成NN支持；第一项失败会短路。两次Ray_E各要求`E >= .26`，E是已有forward novel ray与vacated prior dense ray比例的最大值。没有新增N≥8/20、最短duration、时间阈值或状态机。

冻结日志审计按原顺序配对`FAPP_TRANSLATED_SOURCE`整数count/历史戳和`FAPP_TRANSLATED_STARTUP`，核对同戳同track历史，逐项重算NN分数；`FAPP_TRANSLATED_RAY`核对before AND ray等于after，并关联两E。阈值打印边界、无唯一E匹配保持unknown；双历史NN成立只是startup支持，不能直接当最终发布。ON各场NN两历史均通过的候选观察次数为无球127、afternoon17、ball363、static0、latest93，完整计数和少量原始例证在结果JSON，原大行留在scratch。

## 同源码控制和真实质量

原matrix session74654 exit0；七场均原driver PASS、原evaluator结束、terminal各进程RC0及bag收尾。无球和球均重新跑同一新二进制OFF/ON控制，唯一算法参数差是translated bool；time=false、confirmation_frames=3、fraction=.8恒定，job/dry/metrics和`/mapping_node`全部73项私有运行参数一致。三旧场景仅输入topic使用原throttled数据。

| 真实组 | 已测结果 | 固定要求 |
|---|---|---|
| 无球OFF | 0/103/579/56 | 四阶段分别≤0/109/607/96 |
| 无球ON | 0/103/585/56 | 同上，分别比较，不跨阶段抵消 |
| 球OFF | 110/113，27/27 | ≥108/113，≥26/27 |
| 球ON | 110/113，27/27 | 同上 |
| afternoon ON | fast45/53＋slow59/59，原104全保留 | 原45＋59具体时刻全保留 |
| static ON | 原467戳全覆盖、0框、额外0框 | 全467覆盖，累计框≤6 |
| latest ON | 39/47，原38全保留，新增1单列 | 原38具体时刻全保留 |

完整113个近期冻结标签行、159个旧标签行及467个static戳保留在[结果JSON](actual_frame_event_results_20261012.json)。球ON相对新OFF保留110、丢失0、新增0；旧142无损失，latest新增命中`1791116317825914860`不能抵消任何原命中损失。缺标签输出按原规则计miss；latest原阴性标签`1791116353425527811`仍缺输出，不排除分母。旧scorer保留最近输出与标签差值严格<1000ns、框中心<.25m、点距离<.025m且保留真值点≥max(3,.5×真值点数)，不套用近期113的尺寸规则。

球三项未命中在新OFF/ON均相同，输出存在且均不属于三帧子集：

| 标签整数戳 | t秒 | 真值点数 | 最近已发布中心距离m（OFF=ON） |
|---|---:|---:|---:|
| 1791429339268123627 | 121.9436450005 | 6 | .350079 |
| 1791429342566560745 | 125.2420821190 | 6 | .299217 |
| 1791429352065476179 | 134.7409975529 | 10 | 2.099128 |

三者最近已发布框的最大边分别.264958/.184380/.344721m，但中心均不满足<.25m命中要求。该几何诊断来自已有closed输出诊断JSON；不能由此自动判断未发布候选的算法拒绝原因或物理遮挡成因。134.74秒是继承的strict B未命中，本轮没有恢复，也没有新增该损失。

## 145.67秒原fast命中恢复与无球新增观察

原fast标签`1791101224825826816`，t=145.67184138298035秒；真实input/states戳`1791101224825826883`，相差+67ns，满足旧<1000ns谓词。此前radius05组合此帧output=true、retained0、hit=false；当前retained17、hit=true。相同dense中心附近，两版本的changed均为false，普通published历史和occlusion支持为false。当前track258日志两E为1/.928571；history戳为`1791101224024805784`和`1791101224624788523`，整数NN支持为17/17和14/17，static支持均0，两项通过。published startup原OR由0变1，最终confirmed=1并有MEASURED_BODY发布。该片段解释本帧新支持路径；此前radius05使用旧binary87，未新跑同源码afternoon OFF，故不把跨版本比较当唯一因果实验。

无球ON carried累计框观察585比新OFF579**净增6**；returned仍56。已有一对一中心<.25m诊断列出11个新增和5个消失观察：carried10−4=6、returned1−1=0。新增不是6个已标注物理目标，也不把11项全归因于translated启动。11项均有当前candidate来源匹配，保留dense/EVIDENCE/REFERENCE/POINT_SOURCE/TRACK字段；10项没有当前TRANSLATED_STARTUP tag，returned一项的NN第0支持22/35=.628571失败，但TRACK.changed=1仍可走原启动途径。来源与历史因果保持其证据范围。

| 新增track/观察段 | t范围秒 | 观察数 | span秒 | 采样时长秒 |
|---|---|---:|---:|---:|
| 241 | 27.340565 | 1 | 0 | .099987507 |
| 486 | 46.840598–47.140636 | 4 | .300037623 | .399950028 |
| 486 | 47.340653 | 1 | 0 | .099987507 |
| 518 | 50.143035 | 1 | 0 | .099987507 |
| 646 | 53.240613 | 1 | 0 | .099987507 |
| 663 | 54.338256 | 1 | 0 | .099987507 |
| 581 | 54.738599 | 1 | 0 | .099987507 |
| 598（returned） | 56.740671 | 1 | 0 | .099987507 |

span合计.300037623秒，采样时长合计1.099862577秒，采样时长是观察数乘中位输出间隔，非物理持续占据时间。track486中间缺观察会分段；具体整数戳、源字段和消失观察均在JSON。

## 性能、实际输入频率与覆盖

whole callback使用真实完整回调计时，统一p95严格<100ms；各组最大值可超过100ms，门槛不是max。旧metrics的processing_interval=200ms不替代100ms要求。CPU只统计mapping_node user＋system，100%表示一个逻辑核，不含rosbag、FAST-LIO、规划器或整机。

| 组 | whole callback p50 / p95 / max ms | node回放平均CPU% | peak RSS KiB | 输入 / callback / 输出 |
|---|---|---:|---:|---|
| 无球OFF | 75.142700 / 87.395105 / 160.909 | 75.273 | 155860 | 763 / 760 / 759 |
| 无球ON | 75.000100 / 88.321600 / 170.243 | 75.524 | 155080 | 763 / 761 / 760 |
| afternoon ON | 46.557650 / 86.318360 / 97.9972 | 18.897 | 129380 | 515 / 512 / 511 |
| 球OFF | 75.828200 / 93.403270 / 163.613 | 71.502 | 164324 | 1727 / 1714 / 1713 |
| 球ON | 76.375200 / 93.327710 / 162.634 | 71.907 | 164316 | 1727 / 1715 / 1714 |
| static ON | 76.710700 / 85.644305 / 101.996 | 35.878 | 126608 | 468 / 468 / 467 |
| latest ON | 76.937600 / 91.503360 / 137.104 | 19.976 | 134508 | 1075 / 1073 / 1072 |

近期NB中位输入间隔99.986196ms、全Header跨度平均9.999700Hz；球99.988103ms、全跨度平均9.269616Hz（保留原间断）。旧afternoon/static/latest中位间隔299.628496/299.401998/300.004125ms，全跨度平均2.514681/3.856313/2.203082Hz，是原数据完整1×回放，未补成10Hz。两native OFF/ON使用相同原输入SHA与play命令，callback戳集合仍有差异，完整差异和缺输出保留，不把漏callback计作误报降低。

七组四输出在实际输出戳一致；输入无callback数依表顺序3/2/3/13/12/0/2，每组还有1个callback无输出。receipt latency p95依序129.682327/128.530123/118.218899/137.821371/139.081828/122.690199/133.265481ms，包含输入subscriber接收、同步与传输，不能当whole callback耗时。

## 持续积压与回归范围

已完成原closed记录的receipt重构：[backlog summary](../../../.fapp_ball_debug/actual_frame_event_20261012/backlog_diagnostics/results/summary.json)，各完整结果链接在JSON。其q(t)仅计唯一成功同戳配对的`input_receipt <= t < state_receipt`，同ns事件批次按原子处理；缺输出、负顺序和重复配对不补为永久队列项。无球ON已知配对峰值3、最长连续非零1.085000547秒、回零494次；球ON峰值4、最长3.919231398秒、回零795次。play_end/settled没有已知后续完成项，末一秒配对没有尾部pending。

receipt配对最终q=0本身由完成定义得到。独立[callback-entry定性审计](../../../.fapp_ball_debug/actual_frame_event_20261012/backlog_diagnostics/callback_entry_audit/results/evidence_and_conclusion.json)另将原日志的真实callback入口、whole callback耗时、observer单调时钟与play_end校准配对：无球ON四段入口延迟均值20.79/25.80/21.07/23.84ms，球ON27.09/33.33/38.40/36.83ms；各10秒窗receipt配对q均实际回零。入口与完成间隙反复出现，在实测时钟校准下，末次完成相对play_end的估计裕量分别249.36/204.52ms，无play_end之后迟到状态；中段也反复追上最新已接收输入。结合七场完整轨迹，这些证据支持本次有限录包未观察到持续回调积压。

结果JSON据[独立资格terminal](../../../.fapp_ball_debug/actual_frame_event_20261012/qualification_terminal.json)映射记录`sustained_backlog_pass=true`、`joint_acceptance_pass=true`，范围限定本机、本二进制、native近期输入及原throttled旧输入的原速冻结感知回放；未添加新数值积压门槛。原打包等待状态null完整保留在scratch及JSON的`original_packaging_pending_quality`，追加资格不改原metrics。

入口时钟偏移为实测校准，日志入口延迟和双时钟采样差未获得严格上界；邻近入口的subscriber负顺序单列。未进入回调的输入保持原因unknown，既不补成零延迟，也不计成无限pending。内部ROS队列始终为空没有直接证据，长期在线与闭环动态避障保持null。

main devel实际编译产物为节点SHA `668f4a3b63dfbdf6437348c97485f5c243192a19e83c4f9f9af9089009503366`，CPP `44c3090a6a139a6e67be9b0b1e92d93b4167e231a24621c2efdc6c05a6dfce92`，header `6a63e3a3068390d6270f464cc9df137892d3ba791019363c27263bdef61d25d5`。节点mtime不早于CPP/header；原build日志、运行前冻结快照、各job和闭合manifest一致。身份描述本轮闭合版本，后续live变更另计；未重新读取多GB输入hash，沿原runner前SHA、大小/mtime guard和terminal input_unchanged证据。

原functional验证default7、translated14、旧time2、CTest3、launch3均PASS，translated14包含ray未知不得补启和普通启动保留。另原FULL6 session19982 exit0，绑定实际ball ON闭合job与runtime73项，仅fixture的两个输入topic覆盖及既有extrinsic0，六driver原assert与现场runtime73全验证。原FULL6输入/时间/GT/assert AST保留；79个静态assert节点含原66个保留证明，不声称运行了79次assert。功能回归、真实场景性能和闭环资格分开记录。

前序NN-only source e834/nodee992新OFF/ON无球0/103/579/56与0/103/627/62，ON carried超607上限20；仅2/7场闭合，五场未跑。原driver成功和质量失败均保留。首包因7个fixture PASS之外还有1个aggregate PASS而统计失败，失败stdout保留；修正统计后attempt2成功。最终event七场不复用该前序控制或率。

## 独立验证预设启动

本轮有限冻结感知验收通过后，推荐显式使用独立[launch](../launch/uav1_frame_event_verified.launch)及其[73项参数YAML](../config/uav1_frame_event_verified.yaml)：

```bash
roslaunch mot_mapping uav1_frame_event_verified.launch
```

该预设逐项等于原ball ON运行的73项私有参数，直接使用registered raw cloud和实际odom，不增加throttle；只允许`cloud_topic`、`odom_topic`、`frame_id`三个输入覆盖。ROOT的原[delivery验证](../../../.fapp_ball_debug/actual_frame_event_20261012/delivery_validation/terminal.json)已核验默认与覆盖参数解析、73项一致、通用与正式入口参数保留及CTest3通过，主devel重新编译后源码和实际节点身份未变。

启动前须停止原检测节点，禁止与该预设同时运行。保持平台与场景静止完成已有5秒静态参考采集，期间不发布动态目标，采集后开始检测；依赖FAST-LIO实际`/UAV1/mapping/extrinsic_T`三分量平移参数。通用translated默认仍false，正式Diff-Planner用户入口尚未切换。该推荐仅由本次有限感知回放资格支持，闭环动态避障仍未测。

## 复现与证据

[结果JSON](actual_frame_event_results_20261012.json)保留七组完整73参数、原node/play/driver/evaluator命令、输入SHA、terminal、逐时刻命中表、NN前序、FULL6实际绑定、formula摘要与artifact/source清单；[schema](actual_frame_event_results_schema_20261012.json)检查七组、73项参数及113/159/467基数。原大formula行、callback戳、NN包、failed attempt、backlog原结果均保留scratch，不修改标签或原日志。

下面命令只打印JSON内的完整原回放/evaluator命令；无需手工省略73项参数。端口为17001–17007，method=competition，完整1×未加duration；新native OFF/ON参数仅translated bool不同。重新执行应另起输出名并保留本轮原产物，本报告未运行复现：

```bash
cd /home/oem/match_ws
python3 - <<'PY'
import json, shlex
from pathlib import Path
d = json.loads(Path('src/mot_mapping/docs/actual_frame_event_results_20261012.json').read_text())
for r in d['reproduction_commands']:
    print(r['original_run_case_shell'])
    print(shlex.join(r['original_evaluate_command']))
PY
```
