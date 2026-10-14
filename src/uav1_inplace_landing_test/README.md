# 后机原地起飞精降独立测试

同步前机 2026-10-14 已验证的 NMPC 精降源码，原始文件哈希见 SOURCE_MANIFEST.txt。
专用包和可执行文件不覆盖后机总流程的 MPC 或精降。

在后机桌面关闭其他飞行流程后执行：

```bash
bash /home/oem/match_ws/shfiles/start_uav1_inplace_landing_test.sh
```

待定位和下视画面正常后，在另一终端执行（此命令会起飞）：

```bash
bash /home/oem/match_ws/shfiles/start_uav1_inplace_landing_test.sh start
```

使用后机本机 ROS master 10.224.119.232，飞控接口为 UAV1。
目标起飞高度 1.7m，MPC 起飞目标用 5 秒爬升；50cm ArUco，任意 ID。
相对码高度不高于 1.2m 且水平误差不超过 12cm 连续 0.5 秒后锁 XY；
估计距码面 0.3m 时切 AUTO.LAND。CH10 可优先打断。

相机内参使用后机 ~/.ros/camera_info/down_camera.yaml，外参使用后机
precision_landing/config/down_camera_extrinsic_uav1.yaml。
不使用前机的机体中心补偿；后机 bias_xy_m 初始为零，需要后机重新测试标定。
MPC 模型、推力和调参由后机原 payload_mpc_controller/config 覆盖默认值，
保留后机质量 1.836kg 和已有调参。State 超时为 1.5 秒以兼容飞控 1Hz 状态。

当前独立测试用于落点左偏对照：X/Y 外力补偿增益为 0，Z 增益为 1。
水平外力仍估计和记录，但不应用于控制；重启测试流程后生效。

源码副本用于复现本次确认版本；后续前机改动不会自动更新此包。

兼容调整：后机 PolynomialTraj 消息无 has_yaw/yaw_trajectory 扩展，测试副本移除该可选规划轨迹解析；精降航向仍由 PositionTarget.yaw 给出。

复制源码仅清理行尾空白，SOURCE_MANIFEST 记录清理前的前机原始文件哈希。
