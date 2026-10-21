# UAV1 垂直速度与 IMU 异常诊断记录

2026-10-21 增加记录，不改变 FAST-LIO 状态传播、激光更新公式、收敛阈值、融合噪声参数或飞行控制逻辑。控制器继续使用窗口外力估计，Z 补偿关闭，既有用户参数保留。

## 正式启动与文件位置

现有六/七窗口的 FAST-LIO 分屏调用 `shfiles/run_uav1_sensor_stack.sh fastlio`。现在该入口会先启动独立 rosbag，收到其 latched `begin_write` 通知后再启动原 FAST-LIO launch。8 秒内未确认录包就绪只报告警告，仍按原参数启动 FAST-LIO；该情况下不能保证初始化阶段的数据完整。

每次创建独立目录：

```
~/.ros/fast_lio_diagnostics/YYYYMMDD_HHMMSS_XXXXXX/
  raw_*.bag                 # 原始 CustomMsg、IMU、滤波诊断等
  recorder.log              # rosbag 输出，包括缓存溢出/写盘异常提示
  recording_started.txt     # 录包开始确认
  source_revision.txt       # 源码 HEAD，不单独作为二进制版本证明
  source_diff.patch         # FAST-LIO 和启动脚本未提交差异
  binary.sha256             # 实际 devel/fastlio_mapping 的摘要
  mid360.yaml
  mapping_mid360.launch
  record_lio_diagnostics.launch
  params_before.yaml        # 初始化前已有 UAV1 参数，包括驱动参数（若存在）
  params_end.yaml           # 正常退出时的 UAV1 参数；master 已退出时可能缺失
```

记录内容：原始 `/UAV1/livox/lidar`（保持驱动 CustomMsg 及逐点时间）、原始/机体系/SI 单位 Livox IMU、FAST-LIO 常规/高频里程计与状态诊断、飞控 `/UAV1/mavros/imu/data_raw`、状态、ESC、电池、TF、rosout。控制器原有日志另外增加飞控原始 IMU；外力、补偿力、控制指令和目标轨迹仍在原 AutoTrans 目录。分析时按时间戳关联两组包。这里“原始点云”指驱动输出；驱动已有的安装角处理仍保留。

原始数据录包使用 LZ4、**64 MiB 消息缓存、1024 MiB 分包**。分包不会删除旧文件，也不限制总磁盘使用量；64 MiB 是消息缓存额度，不是进程总内存上限。没有开启订阅全部话题，也不重复录配准后的大点云。直接运行 `mapping_mid360.launch` 不会启动这个原始录包，需使用正式传感器入口或单独启动 `record_lio_diagnostics.launch bag_prefix:=绝对路径`（目录需已存在）。

起飞前和落地停桨后各保留约 20 秒，然后正常退出 FAST-LIO 分屏或使用已有 `stop_all_ros.sh`。脚本没有自动解锁、延迟降落或延迟停桨逻辑，也没有自动等待 20 秒。停止脚本复用现有日志收尾等待，在全量清理前向独立点云 recorder 发 SIGINT，让它完成索引。异常断电或强杀仍可能留下 `.bag.active`。

## 新诊断字段及精确语义

`/UAV1/fast_lio/state_diagnostics` 保留原来的校正前后位置、速度、加速度零偏和重力，新增：

- `attitude_before/after`：机体系到世界系四元数，顺序 **x/y/z/w**。
- `gyro_bias_before/after`：机体系陀螺仪零偏，rad/s。
- `covariance_before/after`：完整 23×23 误差状态协方差，**行优先**，元素索引 `row*23+column`。状态顺序为位置 0～2、姿态切空间 3～5、外参旋转 6～8、外参平移 9～11、速度 12～14、陀螺仪零偏 15～17、加速度零偏 18～20、重力切空间 21～22。不能把最后两维直接当作世界系重力 XYZ。
- `downsampled_points/matched_points`：下采样点数和最后一次测量线性化的有效匹配点数。
- `residual_mean_abs_m/residual_rms_m`：**最后一次线性化**的点面距离绝对值均值及 RMS，非更新完成后重新匹配的残差；没有有效最终测量时为 NaN，避免沿用上一帧结果。
- `iteration_count/valid_iteration_count`：包含 IKFoM 原有 `i=-1` 初始迭代，配置 max_iteration=3 时最多 4 次。
- `final_measurement_valid`：最后一次测量模型调用是否有效。
- `last_increment_converged`：最后一次有效状态增量是否满足原有收敛阈值；不把算法强制重匹配标志误当作数值收敛。若末次测量无效，该字段仍只描述最近一次有效增量，须结合有效性字段读取。
- `covariance_updated`：本次是否执行到原有协方差最终更新分支，不等同于数值收敛。
- `update_wall_ms`：现有更新计时区间耗时，包含更新前诊断快照与更新后状态提取，不包括后续高频预测交接、诊断发布、地图维护和写盘，不能当作整个扫描周期耗时。

每次正常进入激光 EKF 更新时发布一条，当前约 10 Hz；初始化、缺数据或前置跳过的扫描不产生此消息。仅有订阅者时提取额外状态和统计；不把这些数据送回滤波器。消息头仍为扫描结束时间，before/after 描述同一个时间点。

消息定义新增字段后 MD5 会变化，需重启 FAST-LIO 与记录进程，不能期待已运行的旧进程自动使用新定义。旧录包自带旧消息定义，仍可按旧字段读取。此次未改变 PX4 端 MAVLink 发送频率，也未开启或修改 PX4 ULog 配置；实际飞控 IMU 频率需从新包检查。

## 构建及验证

沿用主工作空间 GCC 10：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps fast_lio payload_mpc_controller autotrans_reference_bridge --make-args -j2
source devel/setup.bash
python3 src/FAST_LIO/test/test_update_diagnostics.py
python3 src/FAST_LIO/test/test_imu_repropagation.py
python3 src/FAST_LIO/test/check_diagnostics_recording.py --sensor-wrapper \
  --output /home/oem/.ros/analysis/lio_diagnostics_smoke_new
```

最后一个命令自行启动隔离 master，使用合成点云和 IMU，不连接飞控。不要在同一个输出目录重复跑，以免旧 bag 混入统计。正式启动脚本产生的测试目录带 `SYNTHETIC_TEST.txt`，不应当作飞行日志。

本次验证：

- 主工作空间 fast_lio、payload_mpc_controller、autotrans_reference_bridge 及依赖编译成功，生成扩展消息及正式 devel 可执行文件。
- 生产 IKFoM 诊断开关的状态/协方差一致性、全无效测量、末次无效测量、强制重匹配与真实收敛区分共四种断言场景通过；已有高频 IMU 重传播测试通过。
- 正式启动脚本的隔离合成短录：100 帧原始点云、2000 条原始 Livox IMU、2000 条模拟飞控 IMU 全部录入，96 条有效 EKF 诊断；CustomMsg 点数和逐点时间保留，协方差尺寸/有限性、四元数单位范数等检查通过。
- 正常停止生成可读 `.bag` 和参数快照，无 `.bag.active`，recorder 日志无缓存溢出提示。`stop_all_ros.sh --dry-run` 确认 recorder 被纳入先行正常收尾名单。
- 该合成短录每帧 2700 点、10 Hz、约 10 秒，压缩 bag 3,766,361 字节；录包进程采样最大 RSS 25,044 KiB（约 24.5 MiB）。激光更新区间中位数 5.76 ms、最大 10.20 ms。这是单次合成测试总耗时，不是新增记录的增量开销，也不代表实机点云吞吐量或最坏耗时。

验证结果在仓库外 `/home/oem/.ros/analysis/lio_diagnostics_wrapper_20261021/`，实际合成 bag 目录见其中 `raw_record_dir.txt`。后续真实录制仍需核对消息数/时间间隔、`recorder.log` 溢出提示、录包进程内存和更新耗时；没有警告也不能独立证明全链路绝无丢帧。
