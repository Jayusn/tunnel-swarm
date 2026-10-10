# 独立点级遮挡事件原型（未接正式感知入口）

这是用户计划约定的后备实验，仅借鉴 M-detector 的遮挡运动原理，**不是完整 M-detector 移植，也没有微秒级检测结论**。算法不读取球体标注、摄像头标签、冻结命中列表或特定场景时间段；这些数据只由录包评分脚本读取。

官方来源：

- [作者仓库](https://github.com/hku-mars/M-detector)
- [作者论文](https://www.nature.com/articles/s41467-023-44554-8)
- [论文公开全文](https://pmc.ncbi.nlm.nih.gov/articles/PMC10771495/)

本目录代码是独立实现的简化验证原型，没有复制作者源码，没有改动主工作空间任何感知、规划或传感器接口。正式 launch 不引用它。

## 点级算法

保留最近两帧注册世界点、同期真实雷达位姿和点事件标签。对当前每个真实点查询历史实际返回，形成当前、上一帧、更早一帧的三帧实际观测链：

1. 当前点遮挡两帧历史真实返回，形成横穿/前景事件。
2. 当前点位于历史点之前，上一帧历史点又位于更早历史点之前，形成径向靠近递归链。
3. 当前点位于历史点之后，上一帧历史点又位于更早历史点之后，形成径向远离递归链。

每次深度查询的横向邻域、深度容差固定为3 cm；多个真实返回分别保留，不能把近远两层中间空区间填成表面。一个旧近返回本身不足以判定远离运动；必须存在另一个真实祖先，且深度顺序一致。无返回、视角没有对应邻域、时间断链或非递归近遮挡均不给运动事件。当前位置被历史非事件真实返回解释时，撤销事件。

用单位方向 KD 树检索真实邻域，**没有实现论文完整的0.25°深度图库**。点级标签产生后，再对当前非静态支持点做 DBSCAN 及区域汇集。沿用当前 NanoDBSCAN 的实际邻域：其 `.02` 参数传入 nanoflann 平方距离，因此原型真实邻域为 `sqrt(.02) = 0.141421... m`、core最少3点、候选至少6点、至多800点。未按20 cm包围尺寸过滤，不依赖球、人或其他类别。

聚类候选和最近点新点比例使用正式 `pointsBodyToWorld` 相同的实际输入窗口：相对当前无人机位置的世界XY距离小于5 m，不按`lidar/range_z`裁切当前点。只在形成候选后过滤整个候选底部高于无人机1.5 m的高物（`detection/max_above_uav`）。完整原始注册云仍用于射线深度。`lidar/range_x/y/z`是正式地图范围，不是当前候选输入窗口，因此此独立原型也不拿它们裁候选。

未知候选也保留实际观测记录，然后按上一实际中心、0.82 m范围、一对一、连续轨迹优先关联。只有三次实际候选、6 cm最大位移、三帧相邻段速率均值至少0.14 m/s、当前事件比例至少26%、两帧历史新点比例的较小值达到26%（已经发布过的轨迹20%）才发布。每次发布都含当前点；发布速度为最近两次实测差分，没有纯预测输出。两帧特例完全不存在。

5秒预采期间四路输出带时间戳的空结果。预采结束清空旧观测历史，再开始确认。此原型**没有冻结背景KD树、EKF或正式局部平面抑制**；静态一致性来自最近两帧的真实非事件返回，不能把实验结果视为只改变A04中的一个开关。

## 输入时序限制

使用 registered 世界点，不重复施加世界变换。雷达原点为同期实际里程计位置加旋转后的 FAST-LIO `extrinsic_T`；旋转采用同期里程计四元数。

已核对 Oct8球包：1727帧注册云的 `curvature` 字段每100帧抽检均为0。本地 `FAST_LIO/src/laserMapping.cpp` 的 `RGBpointBodyToWorld` 只赋xyz和intensity，没有保留源点采样时间。所以原型只能按帧时间与同期位姿逐点判别，不能恢复论文的逐点时间戳，也不能宣称逐点流延迟。

独立ROS适配器使用两个真实消息50 ms近似同步。这与正式C++ ApproximateTime同步策略不同，须核对完整回放的实际输入/输出覆盖及 `POINT_EVENT_FRAME` 中的 `odom_dt_ms`，缺输出仍记漏检。不会修复源位姿异常、补输入帧或插值输出。

## 已执行的验证

在工作空间执行 `python3 -m unittest discover -s src/mot_mapping/test/point_event_prototype -p test_event_core.py -v`：14项通过，覆盖前景横穿、径向两种递归链、未知证据、近远两层空区间及近物遮挡远层真实返回、静态支持、雷达位姿与方位环绕、20 cm六点球面/平面及可见小表面、预采结束与时间断链、两幅新点比例较小值、实测换向路径速率、候选窗口与跨高度线目标保留。

性能优化另有13项旧操作对照回归，检查深度邻域归约、并列最近返回、重复祖先、聚类顺序、逐簇新点比例、候选窗口和关联边界。运行两组共27项：

```bash
python3 -m unittest discover -s src/mot_mapping/test/point_event_prototype -p 'test_*.py' -v
```

`ros_smoke.py`：8帧合成检查，输出数为 `[0,0,0,1,1,0,0,0]`，四路时间戳完整一致，状态/框/箭头中心一致。它只证明短合成场景的通信及三帧行为，**不证明录包验收通过**。

单元测试直接检查点级原则及少量候选，不能用它替代完整贴墙、换向、遮挡、拐角和无人机运动回归。慢速点位移小于3 cm、目标回到曾被标成非事件的位置、稀疏视线缺乏三次返回、同一射线上多个物体以及注册位姿异常都可能造成漏检或误事件。

## 独立原速回放和评分

主任务已完成同五份输入的原速回放（点级原型与主矩阵末尾曾重叠，耗时为筛查值）；本目录开发阶段的短测试不替代该结果。使用隔离ROS master，直接把独立脚本传给现有回放驱动 `--node`，不会替换 `devel` 正式可执行文件。

```bash
cd /home/oem/match_ws
source devel/setup.bash
export ROS_HOME=/tmp/point_event_replay
python3 .fapp_ball_debug/bidirectional_20261009/run_case.py ball ball_event_v1 \
  --port 15400 --method baseline --core-radius .08 \
  --node /home/oem/match_ws/src/mot_mapping/test/point_event_prototype/event_node.py
python3 .fapp_ball_debug/bidirectional_20261009/evaluate.py ball_event_v1 \
  --baseline ../native10_20261008/ball_nativebase
python3 src/mot_mapping/test/point_event_prototype/diagnostics.py \
  .fapp_ball_debug/bidirectional_20261009/ball_event_v1 \
  --labels .fapp_ball_debug/corridor_20261008_ball/ball_labels.json
```

对应无球、静止、旧摆球：

```bash
python3 .fapp_ball_debug/bidirectional_20261009/run_case.py noball noball_event_v1 --port 15410 --method baseline --node /home/oem/match_ws/src/mot_mapping/test/point_event_prototype/event_node.py
python3 .fapp_ball_debug/bidirectional_20261009/run_case.py static static_event_v1 --port 15420 --method baseline --node /home/oem/match_ws/src/mot_mapping/test/point_event_prototype/event_node.py
python3 .fapp_ball_debug/bidirectional_20261009/run_case.py afternoon afternoon_event_v1 --port 15430 --method baseline --node /home/oem/match_ws/src/mot_mapping/test/point_event_prototype/event_node.py
python3 .fapp_ball_debug/bidirectional_20261009/run_case.py latest latest_event_v1 --port 15440 --method baseline --node /home/oem/match_ws/src/mot_mapping/test/point_event_prototype/event_node.py
```

每次运行使用新名称，驱动禁止覆盖旧结果。所有参数命令、输入与脚本SHA、实际时间戳、CPU/内存、原速尾部积压及终态由原驱动保存。`FAPP_CALLBACK_TIME` 沿用耗时评分语法。未使用的A04参数仍出现在驱动参数清单，**实际算法配置以节点的 `POINT_EVENT_CONFIG` 和本说明为准**。

评分复用冻结113/27样本、静止467时刻、无球阶段和旧142命中判据；正式 `evaluate.py` 只解析FAPP原因，原型额外诊断以 `POINT_EVENT_CANDIDATE` JSON（当前实际候选中心/尺寸、观测数、位移/速度、事件比例、新点比例、拒绝原因）与 `POINT_EVENT_FRAME` 为准。每个新增框仍须按实际候选和持续时间核对来源。累计框只是误检代理，不能当真实误检率。完整联合门槛未通过就不能推荐或启用正式入口。

## 本轮完整回放结论（2026-10-10）

**未通过，不推荐启用。** 新球10/113、三帧1/27；旧快摆10/53、慢摆2/59、另一段1/47，损失旧129个命中；静止653框/467冻结时刻。新球/无球整回调p95分别206.41/197.09 ms，输出覆盖991/1727、473/763。无球469框包含初始静止与返回增加，并缺290个输入输出，不能称误检降低。详细矩阵、缺口与失败原因见[主报告](../../docs/bidirectional_comparison_20261010.md)。

以上是旧核心 `aca0a024…` 的历史结果。本轮性能优化核心 `7cb3876b…` 保持算法判据与关联规则，在同一整段1727帧及相同实际位姿输入下逐点标签、候选和发布决策完全一致。独立新回放使用 `*_event_v2_perf` 名称，不能复用或覆盖旧 `*_event_v1` 结果。完整回调、输入覆盖与冻结命中结论见[性能对照报告](../../docs/point_event_performance_20261010.md)及[逐时刻结果](../../docs/point_event_performance_results_20261010.json)。正式launch仍不使用此原型。
