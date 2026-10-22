# UAV1 七分屏输入新鲜度与队列诊断

通过 `bash /home/oem/match_ws/shfiles/start_uav1_seven_terminator.sh` 正常启动即可，无需另外启动录包。

- 第 3 屏沿用 `run_uav1_sensor_stack.sh fastlio`：启动前先开启原始诊断录包，新增 `/UAV1/fast_lio/queue_diagnostics`。
- 第 7 屏沿用 `uav1_diff_autotrans.launch` → `quad_wind_mpc_controller.launch`：主录包新增上述 FAST-LIO 话题及 `/UAV1/mpc_controller_node/mpc/input_diagnostics`。
- 诊断直接由原节点发布，没有增加控制线程、定时器或第二套输入缓存；不改变队列上限、同步/拒收规则、数据消费顺序或控制参数。
- 更新后需在地面结束旧的相关节点，再通过七分屏脚本重新启动。编译不会使已运行的旧进程自动获得新代码。

## 控制器 input_diagnostics（100 Hz 控制调用）

消息类型 `payload_mpc_controller/InputDiagnostics`。`header.stamp` 是处理本次输入队列后、进入 `process()` 前的快照时间；`publish_stamp` 是 process 返回后诊断发布时间。

| 字段 | 含义 |
|---|---|
| lio_latest_source_stamp / receive_stamp | 最新进入 LIO 回调的消息源时间/本机回调时间，即使它被拒收也更新 |
| lio_used_source_stamp / receive_stamp | 最后被接受的 LIO 位置对应源时间/回调时间，拒收时保持 |
| px4_used_source_stamp / receive_stamp | 本周期最新 PX4 姿态/速度输入时间；速度沿用已有 PX4 姿态旋转到 PX4 local 世界系逻辑 |
| lio_latest_age_s / lio_used_age_s / px4_used_age_s | 相对快照时间的源数据年龄（秒），未收到数据时为 NaN，源时间为 0；不把负年龄截零 |
| since_lio_accept_s | 距最后一次接受 LIO 位置回调的时间，包含正常采样间隔 |
| rejected_hold_s | 当前连续拒收序列从第一条拒收到快照时刻的时长，无连续拒收时为 0 |
| lio_rejected_total / consecutive | 含瞬态、持续异常及恢复等待在内的拒收总数/连续数；接受时仅连续数清零 |
| position / velocity | 输入位置和速度；state_prepared=true 时对应组装后的 est_state。位置属 LIO 世界系，速度属 PX4 local 世界系，不表示完成两套世界系对齐 |
| state_prepared | 本周期是否执行状态组装；为 false 时不能将 position/velocity 解释为已用于求解 |
| mpc_solve_count / success | 本周期 execMPC 调用次数及最后一次成功状态；次数 0 时 success=false，耗时为 0，不重复报告上一周期求解 |
| input_dispatch_ms | 上次控制调用以来累计输入队列 callAvailable 的稳态时钟耗时，包含控制前的输入处理，不是某一条消息延迟 |
| process_wall_ms | 本次完整 process() 的稳态时钟耗时，包含求解/参考处理等，排除诊断发布 |
| mpc_exec_wall_ms | 本周期 execMPC 累计墙钟耗时，包含等待上次 preparation 线程及反馈求解；不包含返回后异步执行的下一轮 preparation |

是否真正进入求解以 mpc_solve_count 为准；state_prepared 也可能发生在手动模式或随后进入既有恢复处理的周期。输入缺失/无效时沿用原控制逻辑，不因诊断改变处理结果。

## FAST-LIO queue_diagnostics（每次完成的激光建图帧，约 10 Hz）

消息类型 `fast_lio/QueueDiagnostics`。初始化/跳过帧不发布这条消息；阻塞期间也不会有一个独立诊断线程继续发布。

- `queue_sample_stamp`：sync_packages 即将消费本帧之前，在已有 mtx_buffer 锁内读取的快照。
- `lidar_queue_size`、`lidar_time_queue_size`、`imu_queue_size`：该时刻内部缓存长度，不是 ROS 订阅队列长度。
- `oldest_lidar_stamp`、`oldest_imu_stamp` 与对应 `*_age_s`：队首源时间及相对快照的年龄。雷达时间是扫描起点，年龄包含扫描采集时间，不能全部当作排队延迟。
- `lidar_callback_*`、`imu_callback_*`：快照前最新处理回调的入口时间和源时间；雷达入口时间在预处理/等待 buffer 锁之前采集，IMU 源时间使用原代码校正后的时间。可辅助区分订阅回调积压和内部 buffer 积压。
- `imu_replay_count`、`imu_replay_ms`：本帧激光校正交接后，实际前向传播的 IMU 条数与重放循环耗时；不是被清除的条数，重放不消费缓存。
- `handoff_lock_wait_ms`：校正交接等待既有 buffer 锁的时长。
- `scan_end_stamp`、`scan_end_to_handoff_ms`：扫描结束时间及至状态交接/重放完成的延迟。
- `frame_wall_ms`：本次主循环的回调处理、同步、IMU 传播、匹配、交接、建图、常规发布和调试写入耗时；不包含诊断发布和循环末尾休眠。它不包含该帧在此前循环/上游等待的全部时间，应与源时间龄一起分析。

诊断仅复制标量，不复制 IMU 队列或点云，不新增锁；队列快照与回放计时在有诊断订阅者时采集，发布在已有锁释放后进行。控制器也是有订阅者时才发布。未给出实时性或飞行性能保证。

## 录包位置及检查

主包：`~/.ros/autotrans_mpc_logs/<运行名称>/*.bag`，含两条新诊断。
原始包：`~/.ros/fast_lio_diagnostics/<启动时间>_<后缀>/raw_*.bag`，含 FAST-LIO 队列诊断。

在七分屏启动后、实际 ROS 环境中可检查：

```bash
rostopic hz /UAV1/mpc_controller_node/mpc/input_diagnostics
rostopic hz /UAV1/fast_lio/queue_diagnostics
```

两条诊断都是少量标量消息。仍保留原有状态、IMU、控制指令与参考话题，使下次能对齐“前段正常—开始摆动”的时刻；不需要改变现有飞行参数来采集这些诊断。

## 验证

主工作空间 `/home/oem/match_ws/devel`，沿用缓存编译器构建 fast_lio、payload_mpc_controller 及现有工作空间依赖。启动参数解析验证七分屏实际调用的 launch 已包括两条话题。

控制器回归覆盖实际拒收/恢复时的源时间、保持时长、计数，缺失输入的 NaN，以及无求解周期不沿用旧耗时，并通过真实 ROS 订阅验证新消息。
FAST-LIO 使用历史原始雷达/IMU，在回环地址的独立 ROS master 中离线回放；验证队列计数、年龄计算、回放计时、扫描时间和 state_diagnostics 的对应，以及生成的诊断 bag 可读。测试未连接 MAVROS/飞控。

本次构建、测试、启动快照及回放结果位于仓库外 `~/.ros/analysis/input_queue_diagnostics_20261022/`。这些验证证明诊断接入有效，不代表上下摆动原因已经被修复。

验证结果：主 devel 编译成功，控制器 42 项测试全部通过；隔离回放的 115 条 QueueDiagnostics 与 115 条 StateDiagnostics 对应通过，诊断 bag 写入/读取通过；七分屏脚本语法、主录包与原始录包的 launch 解析检查通过。无运行中的飞行节点被重启，无远端修改或同步。
