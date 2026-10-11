# 最近已接收里程计单因子对照（2026-10-11）

**未通过联合验收，不写入正式推荐配置。** 四次完整原速ROS回放使用同一新二进制，只有 `synchronization/nearest_available_odom_enabled` false／true 的区别。新球两组均 **111/113（98.23%）、三帧子集27/27**，当前控制已有命中无损失；无球两组均 **0/105/652/150框**，通道及返回超过固定 **0/109/607/96**。这些是final06＋点源竞争＋主体位置＋静态抑制实验结果，不能当成正式10 Hz A04＋点源竞争默认检测率。

[完整结果与113/27逐时刻](nearest_available_odom_results_20261011.json)保存实际命令、全部参数、四路输出、源缺回调集合、资源与计时、每类第一日志差异、已冻结产物SHA。全部实际配对逐行留在本地 `selection_pose_rows.json`，其身份SHA保留在交付JSON。

## 改动与运行影响

新增默认关闭开关 `synchronization/nearest_available_odom_enabled`，基础launch参数 `nearest_available_odom_enabled` 默认false。开启时，原高频里程计订阅先进入已有message_filters的100条Cache，再送往原ApproximateTime同步器；原云订阅50、里程计订阅25、同步队列100、单线程spin和回调触发条件不变。

原同步回调触发后，在原配对消息及仍在缓存的真实消息中选择与当前云header整数纳秒差最小的一条，等距取较早header。只替换当次实际位姿来源，不插值、外推或等待未来消息；缓存尚未收到／已淘汰的近时位姿不参与。关闭时不创建Cache，直接走原同步路径。没有新状态机、重试、拦截或最小点数规则。实际选中的位置和姿态一起供原裁剪及历史深度使用；新增DEBUG `FAPP_SYNC_SELECTION`记录原配对、选中header及缓存数量，既有 `FAPP_SYNC_POSE`记录所选真实位姿。

三次真实观测、6 cm位移、0.14 m/s、26%／20%新点比例、0.82 m连续优先一对一关联和各项背景、形状、射线、发布条件全部保留；两帧特例未开启。20 cm仍为真实物体尺寸，不按包围框尺寸删目标。正式FAPP入口解析确认新开关false；原主题和消息接口保持。

## 完整结果与单独运行成本

| 数据／开关 | 命中／三帧 | 累计框 | 实际回调／四路输出／原云 | 回调p50／p95／最大ms | 节点平均CPU | 峰值RSS MiB |
|---|---|---:|---|---|---:|---:|
| 无球／关闭 | — | 907 | 760／759／763 | 76.46／92.05／162.65 | 79.05% | 151.44 |
| 无球／开启 | — | 907 | 760／759／763 | 75.96／91.65／161.40 | 78.74% | 150.60 |
| 新球／关闭 | 111/113；27/27 | 2140 | 1715／1714／1727 | 77.38／96.08／168.50 | 75.18% | 160.21 |
| 新球／开启 | 111/113；27/27 | 2141 | 1715／1714／1727 | 78.06／95.93／165.72 | 75.20% | 161.08 |

各回调计数包含1个初始化无四路输出回调。四次采用独立localhost master端口15541–15544、全包1x，严格逐段运行；构建及回归已在播放前停止，实测播放／收尾区间不重叠。普通桌面负荷仍存在，不能将约0.15–0.40ms的跨回放p95差称为因果加速或测量缓存独立成本。CPU为mapping_node进程，100%表示一个逻辑核，不含FAST-LIO、rosbag、master、审计、规划或整机负荷。

回调p95都小于100ms，但最大值仍超过100ms。输入接收到状态接收的端点包括同步／传输，JSON另列，不能写成算法回调耗时。四次均有原驱动 `PASS completed`、完整关闭包、五子进程退出0、输入未变终态和原评分；本轮没有完成门槛替代或重建PASS。外层stdout使用独立 `_launcher_stdout.log`，不会占用原驱动完成标记。

## 无球固定门槛

| 阶段 | 固定同频门槛 | 关闭 | 开启 |
|---|---:|---:|---:|
| 5秒预采统计分段 | 0 | 0 | 0 |
| 初始静止 | 109 | 105 | 105 |
| 携带通道 | 607 | 652 | 652 |
| 返回 | 96 | 150 | 150 |

不能把全段907框解释为真实误检率。累计框仅为误检代理；操作者等物体可能实际移动，未逐框人工确认物理类别。通道+45、返回+54的超限不能由静止−4抵消。新增几何来源与连续段仍由原evaluator保存在各 `*_additional_box_diagnostics.json`。

## 实测配对与失败原因

| 开启组 | 实际回调 | 配对改成更近消息 | 与离线全输入最近HF一致 | 所选绝对时差p95／最大ms |
|---|---:|---:|---:|---|
| 无球 | 760 | 3 | 754/760 | 1.72／95.55 |
| 新球 | 1715 | 69 | 1691/1715 | 2.18／327.55 |

全部所选header存在于实际输入HF，其position／quaternion与真实消息在日志六位有效数字精度内一致；选择后的绝对时间差从未比原配对更大。缓存实际内容和各HF接收时刻没有逐条日志，不能仅凭日志独立证明对每个缓存header都已取全局最小值；实现及固定ROS到达fixture验证这一限定行为。离线最近header对照只用于诊断，不是假定当前必然已收到它。

之前定位的无球分叉header `1791427375709764242`，开启后仍选择 `1791427375675761700`，早34.002542ms；离线最近HF为 `1791427375709510088`，仅早0.254154ms。本次开关未改变该选择，六条plane仍精确等于此前的“实际配对”分支。由实现的遍历及日志可以限定：近时消息当时未参与缓存选择；不能仅凭原包record时间证明它为何未收到、被队列丢弃还是其他未记录原因。因此该开关没有修复这一裁剪／拟合分叉。

两组实际回调及四路输出时间戳集合相同；无球各3个源时刻缺回调、球各12个源时刻缺回调，另各有1个初始化无输出。观察器都收到原云；缺回调不能当成原录包缺帧或零框改善。新球113个冻结时刻全部有输出，开启相对本轮关闭的新增／丢失命中为0／0，两个冻结漏检保留。配对变化可以造成run-local轨迹ID及后续历史变化，日志行不同不能自动当成物理目标不同。

播放结束与等待2秒的checkpoint均无新增状态，末一秒无待输出；未观察到持续积压，永久缺输出仍保留。旧142个动态命中、467个静止时刻及实验组合完整场景回归未在四组重跑，仍为null。无球已经失败，不继承其他配置的旧命中／静止“通过”。

## 构建、必要回归与复现

受影响ROS包仅mot_mapping，主工作空间实际 `devel/lib/mot_mapping/mapping_node` 已重新生成，时间晚于修改的CPP和header；四组实际SHA均为 `97e3c6d6c1fb51eabbe209ae40bab53ffe74425163502e8c30f7c8417b16e614`。

- 默认7项、开启nearest的competition8项真实ROS回归通过，覆盖20 cm六点球面／平面、贴墙换向、自运动、径向靠近／远离、遮挡与拐角；这是对应合成配置验证，不能替代四组final06组合的完整场景验收。
- 两项专项真实ROS fixture通过：关闭保留原同步配对；开启验证等距选较早header、复用已被同步器消费但仍在缓存的消息、真实position／quaternion以及五个实际callback。固定到达序列另有一次严格更近选择。
- 3项CTest通过；基础默认、nearest开启及实际正式FAPP launch解析通过，新开关默认false。最初使用错误的package内launch路径的解析尝试失败，其空YAML保留，之后用实际完整launch路径验证，未修改配置来绕过失败。

复现须用新案例名／端口，不覆盖现有结果：

```bash
source /opt/ros/noetic/setup.bash
source /home/oem/match_ws/devel/setup.bash
python3 .fapp_ball_debug/bidirectional_20261009/run_case.py noball noball_nearest_repro_01 --port 15591 --method competition --density-split --core-radius .08 --density-fraction .7 --reacquisition --reacquisition-gap .6 --param tracking/position_measurement_variance=0.01 --param detection/occlusion_startup_enabled=true --param detection/publish_measured_body_position_enabled=true --param detection/bidirectional_static_suppression_enabled=true --param detection/bidirectional_ray_evidence_enabled=false --param synchronization/nearest_available_odom_enabled=true
python3 .fapp_ball_debug/bidirectional_20261009/evaluate.py noball_nearest_repro_01 --baseline ../native10_20261008/noball_nativebase
```

已有四组完整命令、参数和端口见JSON及 `*_job.json`。基线命令只将新参数改为false。正式launch不开启此方案，该方案仍不能被称为联合动态避障达标。

归档核对发现summary把自己的重定向stdout在最终print前记录为0字节SHA；唯一不匹配项已在交付JSON改成关闭后的真实SHA／大小，原scratch summary和readback审计均保留。算法输出、评分、PASS及其他207项原证据未变，不重新回放或评分。
