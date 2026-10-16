# FAST-LIO 到 PX4 的局部位姿回传

实机命令保持为 `python3 laser_mid360.py iris 0 fastlio off`。
`off` 表示关闭 Gazebo 真值替代，使用真实 FAST-LIO 数据。

实机 FAST-LIO 模式经 `/UAV0/mavlink/to`（UAV1 对应自身命名空间）发送
MAVLink 2 `ODOMETRY`，世界坐标系标记为 `MAV_FRAME_LOCAL_FRD`。
该标记表示局部世界轴不以磁北为基准，使 PX4 能按外部局部航向初始化。
位置与姿态仍使用原 MAVROS vision_pose 的 ENU→NED、FLU→FRD 数值转换，
保证 PX4 经 MAVROS 回传的坐标与原规划/控制坐标约定一致。

保留测量时间戳，最多发送 50 Hz，同一输入样本不重复发送；不提供速度、角速度，
位姿协方差保持原 PoseStamped 接口的零值，不修改任何飞控融合参数。
PX4 1.16 的外部视觉入口不接受 LIDAR 类型，因此使用 VISION 类型传输 LIO 位姿。
需要 Python 3 的 `pymavlink`，依赖通过 `python3-pymavlink-pip` 声明。

每次回传进程开始发送、源序号或测量时间回退、源坐标系名称变化时，递增
`reset_counter`；正常序号溢出不触发重置。计数保存在
`${ROS_HOME:-~/.ros}/UAV0_fastlio_odom_reset_counter`，模 256 回绕。
此计数通知 PX4 外部估计基准变化；不是对罗盘或 IMU 进行传感器校准。
没有显式重置信号的源若静默更改坐标系但保持序号、时间与 frame_id 连续，
无法由该接口可靠识别，应随源一起重启回传进程。

实机 FAST-LIO 模式不再发布 `/UAV0/mavros/vision_pose/pose`，避免两路外部
位姿交替融合；旧的该话题录包配置也不会记录新 ODOMETRY 报文。
仿真真值模式和其他 SLAM 模式继续使用原 PoseStamped 接口。

启动顺序仍是：启动定位和回传，确认飞控航向融合及预检正常，手动解锁，
再拨通道进入 OFFBOARD。更新后应在地面重新启动回传验证。
基准重置可能使 PX4 本地航向、位置发生跳变，不能在飞行中重启来执行“校准”。
代码测试验证报文字段和转换，不代替地面飞控接收、预检和后续实飞验证。
