# 激光校正前后状态录制

正式 UAV1 传感器入口启动后提供 `/UAV1/fast_lio/state_diagnostics`，消息类型为
`fast_lio/StateDiagnostics`。MPC logger 已将此话题加入现有 rosbag 录制列表。
更新后需要重启 FAST-LIO 和 MPC logger；仅重启控制器不会更新 FAST-LIO 进程。

每次激光 EKF 测量更新记录一条，通常约 10 Hz，初始化或跳过点云时不生成。
`header.stamp` 是扫描结束的源时间，前后状态属于同一时刻；并非消息发布时间。
`before` 是 IMU 传播完成、激光测量更新之前；`after` 是更新后的滤波状态，
不是外推到最新 IMU 时刻的高频里程计。

|字段|单位与坐标|
|---|---|
|`position_before/after`|m，`header.frame_id` 世界系|
|`velocity_before/after`|m/s，世界系|
|`accel_bias_before/after`|m/s²，IMU/机体系|
|`gravity_before/after`|m/s²，世界系|

所有数组均为固定三个元素，顺序 x/y/z。共 24 个 double（192 字节）和 Header，
10 Hz 消息数据量约数 KB/s，不含 rosbag/连接开销。有订阅者时才复制状态和发布；
发布队列为 10，不打印逐帧文本、不在主循环写文件，且在高频预测状态交接之后、
IMU 缓冲锁之外发布。只添加观测，不改变估计公式或控制参数。

查看与单独录制：

```bash
source /home/oem/match_ws/devel/setup.bash
rostopic echo -n 1 /UAV1/fast_lio/state_diagnostics
rostopic hz /UAV1/fast_lio/state_diagnostics
rosbag record -O /home/oem/bags/fastlio_state_diagnostics.bag /UAV1/fast_lio/state_diagnostics
```

诊断时将 `velocity_after - velocity_before` 与位置修正、零偏变化一起比较。
性能影响应使用同一输入下的校正耗时和高频输出间隔评估；消息体小不代表实机零开销。
