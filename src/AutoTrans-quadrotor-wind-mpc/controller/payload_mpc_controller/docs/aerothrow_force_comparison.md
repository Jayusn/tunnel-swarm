# AeroThrow 三轴 NDOB：UAV1 接入与离线比较

默认仍使用 `window_opt`，控制频率保持 100 Hz。只修改本工作空间的 UAV1 正式 `payload_mpc_controller` 及其 launch 入口，没有修改独立精降测试 MPC，也没有部署或同步 UAV0。已有质量、推力标定、输入预滤波、同步、补偿模式门控和 NMPC 参数继续共用。

2026-10-19 后续修改：按用户要求取消补偿端额外 Z 轴 0.7 Hz 低通及 XY/Z 变化率限制；正式控制器与当前回放均直接应用分轴增益、合力限幅后的估计力。估计器自身的输出低通仍保留。下文数值表和既有产物目录为取消前版本 `7479a75b` 的历史结果，不代表当前补偿链路；精确复跑该历史版本需使用该提交的工具和构建。

## 公式来源和本机约定

- [AeroThrow，IV-B-3，式 (12)–(13)](https://arxiv.org/html/2507.13903v1#S4.SS2.SSS2)：外力 NDOB、欧拉离散及输出二阶 Butterworth 滤波。本文实现是按公式和本机输入适配，非作者源码逐行移植；作者 [AeroThrow 仓库](https://github.com/zedliang67/AeroThrow) 查阅时没有可直接复用的估计器源码。
- [2025 年前作，IV-A](https://arxiv.org/html/2501.06122v2#S4.SS1)：解释 NDOB 低频估计与机械臂高频补偿的分工；其 50 Hz 输出截止频率不适合直接用于本机 100 Hz 采样。本文不接入机械臂补偿。
- [2024 年风估计前作，IV](https://arxiv.org/html/2409.01549v1#S4)：扰动观测误差的一阶收敛关系可帮助解释增益；不移植风速映射、旋翼阻力模型及其慢速风估计滤波。前作记号/离散式与 AeroThrow 并不完全相同，以此次明确选择的 AeroThrow 方程和本机动力学为准。

本机动力学取 `m a_world = T R e3 + m g_world + f_world`，`R` 将 UAV1 机体系转至当前 FAST-LIO 世界系（`UAV1/camera_init`）。四路 RPM 是机械转速，`kf` 单位 N/rpm²；IMU 数据已经旋转到机体并从 g 换算成 m/s²，输入为比力，不能再减一次重力。

```
b = m R a_IMU_body - kf sum(RPM_j²) R e3
state[k+1] = state[k] + rate / control_frequency * (b[k] - state[k])
rate = c / m
```

`AeroThrowNDOB` 分别保存递推状态、最近残差和输出滤波状态。沿用估计力模长 5 N 的滤波前/后限幅，滤波和限幅的输出不会覆盖递推状态。选择 NDOB 后只用自己的输出滤波，不叠加窗口方法的 3 Hz 低通。无饱和时，未滤波状态满足 `state[k] = b*(1-(1-rate/100)^k)`。

新增启动参数：

| ROS 参数 | 默认值 | 约束/含义 |
| --- | --- | --- |
| `force_estimator/method` | `window_opt` | 另一合法值 `aerothrow_ndob` |
| `force_estimator/ndob_rate` | 20.0 s⁻¹ | 有限、正值，`0 < rate/control_frequency < 2` |
| `force_estimator/ndob_cutoff_freq` | 20.0 Hz | 有限，`0 < cutoff < control_frequency/2` |

20 s⁻¹ 和 20 Hz 是初始实验值，并非论文给出的本机最优参数。当前质量 1.926 kg 下，20 s⁻¹ 对应 `c=38.52 kg/s`。这属于“论文公式的本机适配”；100 Hz 下不使用 50 Hz 截止频率。非法配置按现有参数加载方式报错，不自动调参或切回。

## 调用和状态语义

- 观察器定时器继续收集新的有效同步样本，只有新样本更新残差及有效样本计数。窗口方法仍按原先方式入窗、优化、低通。
- `setForceEstimation()` 每次正常调用执行一次 NDOB 欧拉更新，固定 `dt=0.01 s`。两次同步样本之间使用最近有效残差零阶保持；保持不增加有效样本数。
- 原有起飞、输入失效、离地切换和其他清零位置通过同一个 `reset()` 清除 NDOB 递推、滤波与样本计数。恢复中原本跳过的估计周期不执行，也不追补。
- 20 个有效样本用于“补偿接入等待”。NDOB 第一个有效样本后即可递推，但准备完成之前不能写入补偿。相同的 0.8 s 空中等待、状态/模式/RPM 门控继续适用。
- 估计之后直接执行分轴增益和 5 N 合力上限，精降 XY 关闭逻辑保持现状。取消额外 Z 轴低通及变化率限制后，补偿接入、恢复和估计快速变化时会更直接地反映到 NMPC 输入。原三个低通/限速参数已从正式控制器配置与加载中移除。
- 新增 `~mpc/force_timing` (`Vector3Stamped`)：`x` 为两次正常估计调用的实际 ROS 时间间隔（秒，首帧 0），`y` 为估计计算耗时（微秒），`z` 为有效样本数（达到准备数后封顶）。这是辅助日志，原 `force`/`force_applied` 消息格式不变。
- 新增锁存 `~mpc/force_config` (`String`)：方法、NDOB 参数、控制频率、解析后的输入话题、构建版本/源码指纹/编译器及控制器参数 XML。日志程序自动录入两个话题，并保存 `force_estimator_snapshot.txt` 与 `controller_params.json`。构建时 Git 描述可能含 `dirty`，源码 SHA256 用于辨别实际编译内容；参数另由启动快照确定。

## 离线工具与可重复命令

原始 bag 和所有生成数据均位于仓库外。本次产物目录：

```
/home/oem/.ros/analysis/aerothrow_comparison_20261019/
```

其中 `report.md` 是中文数值报告；`parameter_grid.csv` 为完整 16 组扫描；`synthetic_metrics.csv` 为三轴合成实验指标；`*_forces.png`、`*_applied.png`、`*_spectra.png`、`parameter_tradeoff.png` 为图表。各包的完整 CSV 保留同一输入下所有估计器结果，`*_baseline_validation.csv` 为原方法回放校验，`*_events.tsv` 为 UAV1 模式/门控日志，`*_segments.csv` 为飞行表现分段，`*_relative.csv` 为两机相对位置，`*_timing.json` 为接收间隔统计。快照及 SHA256 在 `snapshot/`。

```bash
cd /home/oem/match_ws
source devel/setup.bash
P=src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller

# 完整复跑：当前配置会复制到输出目录；已有历史结果应使用新的目录名。
python3 "$P/scripts/compare_force_estimators.py" \
  --replay "$PWD/devel/lib/payload_mpc_controller/force_estimator_replay" \
  --config "$P/config/mpc.yaml" --model "$P/config/model.yaml" \
  --bag-root /home/oem/.ros/autotrans_mpc_logs \
  --output /home/oem/.ros/analysis/aerothrow_comparison_repeat

# 复现本次冻结参数，而不是读取今后可能改动的工作区配置。
python3 "$P/scripts/compare_force_estimators.py" \
  --replay "$PWD/devel/lib/payload_mpc_controller/force_estimator_replay" \
  --config /home/oem/.ros/analysis/aerothrow_comparison_20261019/snapshot/mpc.yaml \
  --model /home/oem/.ros/analysis/aerothrow_comparison_20261019/snapshot/model.yaml \
  --output /home/oem/.ros/analysis/aerothrow_frozen_repeat

# 单包 C++ 回放；程序不发布消息，不需要 ROS master。
rosrun payload_mpc_controller force_estimator_replay \
  --config "$P/config/mpc.yaml" --model "$P/config/model.yaml" \
  --bag /home/oem/.ros/autotrans_mpc_logs/20261017_152939__unlabeled__comp_on/20261017_152939__unlabeled__comp_on.bag \
  --output /home/oem/.ros/analysis/aerothrow_single
```

Python 依赖 numpy、pandas、scipy、matplotlib 和 ROS rosbag；C++ 依赖 rosbag、yaml-cpp 及原控制器依赖。`--reuse` 仅在明确要用既有 CSV 重画图时使用，不校验旧 CSV 与当前源码是否匹配。`--observer-phase` 可显式指定观察器相对于控制器的相位（秒）；本次全包固定为 0，不调整相位拟合曲线。`--imu`、`--attitude` 可显式覆盖输入话题，默认使用本机 Livox IMU 和 FAST-LIO 高频里程计。

回放按 bag 接收顺序处理输入，源时间戳用于生产同步器，名义观察器事件先于同一时刻控制事件；首个控制事件锚定第一条已记录 force 的接收时间。同步器、输入低通和两种 C++ 估计器复用生产实现。当前下游只执行分轴增益、合力限幅及门控，不再复现旧录包中的补偿低通/限速。因此用当前工具重放旧包时，实际补偿与记录的差异包含这项明确的行为变更。

历史数据缺少完整参数、精确回调顺序及部分模式/恢复事件，当前参数快照并非历史参数证明。恢复/清零仅依据实际传感器状态和可识别的 UAV1 日志重建，无法识别的部分保留不确定性。ROS 日志节点必须匹配 `/UAV1/mpc_controller_node`，同包中 `/mpc_controller_node` 的另一架机日志不能混用。录包接收时间不等于控制器订阅回调时间；本次新有效同步样本明显少于 100 Hz，不应把 20 样本一概称作 0.2 s 窗口。

## 取消补偿平滑前的历史结果与两个候选

以下均为当前窗口参数的比较，单位、统计口径及全部数值详见生成报告。

| 方法 | 1 N 阶跃 10%—90% | 阶跃起始到 90% | 孤立 3 N 异常产生的峰值 | 0.2 s 脉冲误差 RMS（2–3 s） |
| --- | --- | --- | --- | --- |
| 窗口 20 样本 + Huber + 3 Hz | 0.18 s | 0.26 s | 0.054 N | 0.487 N |
| NDOB 20 s⁻¹ / 5 Hz | 0.11 s | 0.14 s | 0.277 N | 0.346 N |
| NDOB 20 s⁻¹ / 20 Hz | 0.10 s | 0.11 s | 0.522 N | 0.233 N |

合成响应在 100 Hz 有效入样、准备完成后测得，不包含补偿接入等待或输入预滤波。窗口在实际有效入样变慢时，其时间跨度也会变长。NDOB 的计算耗时明显更低，但当前窗口离线计算 P95 也只有约数十微秒，远小于 10 ms 周期；不能把计算时间节省直接等同于飞行改善。本机计算耗时仅为离线测量，非机载最坏执行时间。

| 录包 | 窗口估计波动 RMS | NDOB 20/5 | NDOB 20/20 | 窗口补偿变化率 RMS | NDOB 20/5 | NDOB 20/20 |
| --- | --- | --- | --- | --- | --- | --- |
| 10-17 双机 | 0.672 N | 0.871 N | 0.908 N | 1.165 N/s | 2.055 N/s | 2.064 N/s |
| 10-19 14:41 通道 | 0.666 N | 0.871 N | 0.904 N | 1.231 N/s | 2.170 N/s | 2.177 N/s |
| 10-19 16:27 悬停/异常 | 0.704 N | 0.809 N | 0.836 N | 1.141 N/s | 2.077 N/s | 2.094 N/s |

这里的波动是三轴去均值后的向量 RMS，包含机动、环境变化和传感器因素，不能全称作噪声。变化率只统计相邻且补偿有效的计算周期，不跨停用段求导。完整报告另有按轨迹执行/悬停等分段统计及频谱。

原窗口回放与录包在重建补偿有效片段的估计力向量误差 RMS 约为 **0.370、0.354、0.248 N**，实际补偿误差约 **0.248、0.247、0.195 N**。因此本轮实测比较属于“相同重建输入下的算法比较”，并非精确复现历史控制。没有为减小这些偏差而拟合质量、kf 或滤波参数。

两种方法优缺点：

| 方法 | 优点 | 弱点 |
| --- | --- | --- |
| 当前窗口 Huber | 本轮录包下波动及补偿变化率较低，孤立异常抑制更好 | 窗口与 3 Hz 滤波使瞬变滞后；有效入样变慢会延长窗口；优化计算更重 |
| AeroThrow NDOB | 递推简洁、计算轻；足够增益下阶跃/脉冲跟随更快；无新样本仍按控制周期演化 | 对孤立异常及偏置缺乏 Huber 鲁棒性；增益/截止频率提高会增加输出波动；滞后的输入被保持并不意味着增加了真实信息 |

两者都把质量、推力系数、安装姿态、RPM 和 IMU 偏差混入估计外力，不能仅凭估计值识别“真实尾流”。NDOB rate=5 s⁻¹ 的部分组合比当前窗口更慢；论文方法并非在所有参数下都更快。

依据 10 月 17 日扫描，保留 **20 s⁻¹ / 5 Hz** 与 **20 s⁻¹ / 20 Hz** 两个代表性候选：前者减小异常峰值且已有较快响应，后者用于检验更快脉冲跟随是否值得更大的波动。10 月 19 日两组复核均保留了这种取舍。它们是实验候选，不是最优参数结论；目前没有证据支持直接替换默认方法。

## UAV1 实飞启动命令

以下为后续实飞入口，本次验证只解析 launch，没有启动飞控控制。传感器/MAVROS 仍使用现有流程。

```bash
cd /home/oem/match_ws
# 原窗口基线（也是不带新增参数时的默认行为）
bash src/Diff-Planner/sh_files/run_uav1_autotrans.sh force_estimator_method:=window_opt

# NDOB 候选 A
bash src/Diff-Planner/sh_files/run_uav1_autotrans.sh \
  force_estimator_method:=aerothrow_ndob ndob_rate:=20.0 ndob_cutoff_freq:=5.0

# NDOB 候选 B
bash src/Diff-Planner/sh_files/run_uav1_autotrans.sh \
  force_estimator_method:=aerothrow_ndob ndob_rate:=20.0 ndob_cutoff_freq:=20.0
```

方法和参数重启控制器生效。其余原有实飞 launch 参数应保持各轮一致。

后续固定路径、速度、NMPC 参数和前机运动安排，基线与候选交错各至少三次；记录电池电压、相对位置及所有参数快照。比较位置误差 RMS/P95、去均值摆动、实际 IMU 角速度、指令变化和实际补偿。两机距离只在可用 TF 链、源时间对齐通过检查时作为环境标签，不能当尾流真值。离线位置误差仅描述原飞行，不代表 NDOB 反事实轨迹。

当前起飞阶段不启用补偿，本次实验不用于证明改善起飞抖动。最终是否适用于通道双机飞行，仍待配对实飞判断。

## 验证与构建

主工作空间使用既有 GCC 10 和 `catkin_make`；实际产物位于 `/home/oem/match_ws/devel`。新增离线目标单独指定 C++14，以兼容 ROS Noetic/Boost 的 rosbag allocator API，不更改在线控制器的编译标准。

```bash
source /opt/ros/noetic/setup.bash
cd /home/oem/match_ws
catkin_make --only-pkg-with-deps payload_mpc_controller autotrans_reference_bridge --make-args -j2
catkin_make run_tests_payload_mpc_controller --make-args -j2
catkin_test_results build/test_results/payload_mpc_controller
source devel/setup.bash
roslaunch --dump-params autotrans_reference_bridge uav1_diff_autotrans.launch
roslaunch --dump-params autotrans_reference_bridge uav1_diff_autotrans.launch \
  force_estimator_method:=aerothrow_ndob ndob_rate:=20 ndob_cutoff_freq:=5
```

`--only-pkg-with-deps` 会修改本地 CMake 包白名单；若原工作空间已有其他目标，之后按原白名单恢复。本轮不提交构建缓存。

测试覆盖：默认窗口分支与原实现的同事件序列逐值一致；NDOB 闭式欧拉响应、独立 Butterworth 参考、零阶保持、准备计数、重置、限幅不反馈、三轴零外力/倾斜与坐标变换、非法参数；真实 `MPCFSM::setForceEstimation()` 与离线估计器在同一有效样本/计算/重置序列下结果一致，并检查两种方法共用的准备、模式门控、分轴增益、合力限幅、直接应用以及精降 XY 关闭。集成测试使用隔离的 rostest master，不连接飞控。

本轮验证结果：catkin 汇总 101 项、0 错误、0 失败；另有 5 项 Python 静态回归、日志快照回调检查及默认/NDOB 两套 launch 参数解析通过。构建日志、测试 XML、解析后的参数及可执行文件 SHA256 已保存到上述产物目录的 `verification/`。主工作空间原有包白名单已恢复，并加入本次编译的 bridge 包。
