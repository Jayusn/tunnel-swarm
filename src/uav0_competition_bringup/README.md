# UAV0 比赛统一启动包

`uav0_detection_landing_stack.launch` 把 `match_ws/src` 中的 RealSense D435、
颜色标签、二维码、热成像融合、相机外参 TF、目标远程上报和精确降落包集中到一个入口。
默认启动 D435、三类检测和 `target_reporting`，不默认打开下视相机、精确降落
或独立 RViz。D435 原始点云默认同步发布，但 RViz 只显示检测目标附近的过滤点云。
RViz 同时显示由过滤点云计算出的检测目标三维线框包围盒。

## 仿真/检测显示

### 后机独立移动检测测试

```bash
bash /home/oem/match_ws/shfiles/test_uav1_detection.sh
```

启动 MID360、FAST-LIO、D435、热成像、三类检测/上报与多机后机同一套 RViz。
直接复用 `exploration_control/launch/uav1_relay_visualization.launch` 和 `uav1_lite.rviz`，
显示雷达配准点云、手持移动轨迹、机体模型、三类检测画面与目标位置标记。
配准点云是当前扫描；本测试不启动规划器，因此不会生成规划器的膨胀栅格地图或规划轨迹。
脚本使用独立本机 master `http://127.0.0.1:11321`，不需要前机或 MAVROS，
不启动规划/飞行控制。启动前关闭占用同一硬件的其他程序；开始时保持机体静止，
待 FAST-LIO 初始化、相机自动曝光稳定后，再移动后机或展示目标。Ctrl+C 结束所有本次启动节点。
查看话题时需使用 `ROS_MASTER_URI=http://127.0.0.1:11321`。

测试 launch 直接包含正式 `uav1_detection_stack.launch`，复用所有检测源码和算法 YAML。
仅使用 `target_reporting/config/uav1_detection_test_activation.yaml` 关闭入口等待，
并分开保存路径；正常多机启动仍等待入口目标。深度容差仍是 500 ms、未定位不保存，
二维码仍按三维距离 0.30 m 去重。检测到不等于每帧保存，仍须通过正式确认条件。

默认保存到 `match_ws/target_reports/uav1_detection_test/YYYYMMDD/test_HHMMSS_纳秒/`：
`reports.jsonl` 记录已确认目标，图片位于 `images/annotated/qr_code`、`color_tag`、
`thermal_source`（含热图和 D435 配图）。坐标沿用 FAST-LIO 世界坐标到 channel 的现有配置，
没有使用固定或伪造定位。程序实际运行前不会预创建测试目录。

前机与出口过滤参数也沿用多机配置，但独立 master 没有前机里程计/出口输入，
因此不能用本测试验证这两项过滤；热源地面高度过滤继续生效。
RViz 中的前机、动态障碍、规划和降落显示项没有相应输入时会空白，保持多机原有显示配置。
可加 `open_rviz:=false` 关闭窗口，或加 `enable_thermal:=false` 只测颜色和二维码。
`--dump-params` 只解析配置，不启动设备。

测试默认录包到当次目录的 `bags/detection_*.bag`，LZ4 压缩、每 1 GiB 分包，
可加 `record_bag:=false` 关闭。使用 Ctrl+C 正常结束并等待 roslaunch 退出，便于录包完成索引。
录制 D435 压缩彩色图、无损压缩对齐深度、相机内参、原始热图、热源调试图与
逐候选 D435 配图，以及三类候选/确认状态、上报观测、FAST-LIO 里程计、TF 和 ROS 日志。
回放检测算法时，需要将 `compressed` / `compressedDepth` 图像解压为原始图像话题；
这些消息保留拍摄时间戳，可用于检查二维码漏检、深度匹配及热源映射。
直接调用 launch 并希望图片和录包按批次一起归档时，应同时传入非空 `mission_id`；
上面的测试脚本已自动生成日期和批次。

热源上报与多机共用：先等待对应帧的 D435 配图，再处理确认；配图迟到时由现有定时器
重试尚未处理的结果，不把图片到达顺序当成检测失败。仍保留原有深度、同帧证据、
检测稳定性、世界坐标确认与过滤条件。

根据热成像相机是否接入，选择下面一种启动方式。方案 A 适用于当前未接热成像
相机的电脑；方案 B 适用于 D435 和热成像相机都已接入的电脑。

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 方案 A：启动 D435，关闭热成像
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=false \
  realsense_enable_pointcloud:=true \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  target_reporting_mission_id:=onboard_test_$(date +%Y%m%d)
```

```bash
# 方案 B：D435 和热成像都启动
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=true \
  realsense_enable_pointcloud:=true \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  target_reporting_mission_id:=onboard_test_$(date +%Y%m%d)
```

入口还会自动启动 `camera_body_tf.launch` 中的相机静态外参发布节点，读取：

```text
$HOME/handeye_calibration/body_camera_03.yaml
```

当前入口默认发布 `UAV0/body -> camera_link` 外参，与 FAST-LIO 的 `UAV0/body`
坐标系对齐。若实际 FAST-LIO 仍发布无前缀的 `body`，可追加
`camera_body_tf_parent_frame:=body`。

当前 FAST-LIO 已自行广播 `UAV0/camera_init -> UAV0/body`，因此统一入口默认设置
`enable_camera_body_odom_tf:=false`，不会再启动 `fastlio_odometry_tf` 重复发布同一
变换。只有更换为“不发布 TF、仅发布 Odometry”的定位源时，才设置
`enable_camera_body_odom_tf:=true`，并用 `camera_body_tf_odom_topic` 指定其里程计话题。

脚本不修改 `ROS_LOG_DIR`，因此和规划器一样使用 ROS 默认日志目录：

```text
~/.ros/log/<本次运行ID>/
```

最近一次运行也可以通过以下路径查看：

```text
~/.ros/log/latest/
```

颜色标签和二维码共用这一套 D435，不能重复启动第二个 RealSense 节点。
`/camera/depth/color/points` 是 D435 原始点云输入，
`/UAV0/target_reporting/detected_object_cloud` 是只保留检测目标附近点的 RViz 输出。
`/UAV0/target_reporting/detected_object_boxes` 是按目标类型着色的三维线框包围盒输出。
若需降低 D435 负载，
可传入 `realsense_enable_pointcloud:=false` 关闭点云输出。
过滤半径默认是目标中心周围 0.25 m。
如果 D435 已经由其他终端启动，才使用：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=false \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true
```

规划器已经在自己的 RViz 中启动 `target_rviz_marker`，因此通常不要把
`enable_target_rviz` 设为 `true`。独立查看检测结果时才使用：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  enable_target_rviz:=true
```

## UAV0 精确降落

比赛六分屏脚本的第 5 屏会自动查找 Generic USB 下视相机，并同时启动 UAV0
精确降落节点。手动使用统一入口时追加以下参数：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  enable_down_camera:=true \
  enable_precision_landing:=true \
  landing_vehicle_ns:=UAV0
```

该入口不会自动解锁、切换 `OFFBOARD` 或触发降落；仍需按现场安全流程发布
上升沿 `/UAV0/need_to_land`。下视相机、状态、调试图和 MAVROS 接口也分别使用
`/UAV0/down_camera/...`、`/UAV0/landing/...` 与 `/UAV0/mavros/...`。

## 话题关系

三个检测包发布 `/UAV0/...` 候选和调试话题；`target_reporting` 完成坐标转换、
候选/确认跟踪、TCP 远程上报，并发布 `/UAV0/target_reporting/observation`。
规划器 RViz 的标记适配器再将观测转成 `/UAV0/target_reporting/markers`。
候选结果目前只用于机载端 RViz 显示，不通过 TCP 发送到 Windows；Windows 端只接收确认目标
及其最终证据图片。确认图片由 `target_reporting` 根据检测几何统一绘制二维码四角、颜色标签
或热源矩形框；候选和确认结果都不参与规划决策。
任务编号默认按当天生成，也可以通过 `target_reporting_mission_id` 显式指定。

D435 原始点云 `/camera/depth/color/points` 只作为显示过滤器输入；RViz 实际显示
`/UAV0/target_reporting/detected_object_cloud`，只包含当前候选/确认目标附近的点，
不参与检测判定、避障或规划决策。若 D435 已在其他终端启动，需确保原始点云话题已发布。
包围盒话题为 `/UAV0/target_reporting/detected_object_boxes`，同样只用于 RViz 显示。
