# 热成像 + D435 深度融合说明

本模块不重写热成像检测和 UVC 取流，只订阅已有热源像素结果，再结合 D435 深度和 TF 输出空间坐标。

## 当前热成像取流基准

当前最终取流参数为：

```text
streamType = 10
UVC 请求尺寸 = 384 x 288
offset = 0
len = 221184
格式 = YUYV / YUV422
Python reshape = (288, 384, 2)
gray = yuv[:, :, 0]
```

这套参数变更后，热成像像素坐标可能和旧标定结果有轻微差异。

重要提醒：

```text
切换为 384 x 288 offset=0 后，需要重新标定 thermal_to_d435_homography。
```

## 融合链路

```text
热成像像素
  -> D435 彩色图像像素
  -> D435 对齐深度
  -> D435 相机坐标系三维点
  -> odom / channel 坐标系
```

第一版默认把 `odom` 当作通道坐标系。

## 运动场景时间匹配

热成像在读到完整一帧后立即记录软件接收时间，原图、检测结果和候选点沿用该时间。
这不是硬件曝光时间，仍可能包含 UVC/管道传输延迟。

`config/thermal_d435_fusion.yaml` 中统一配置时间匹配参数，launch 不覆盖这些值：

- `fusion_delay_s: 0.40`：等热成像帧时间过去 0.40 秒，再选取缓存中的最近彩色/深度帧。设为 `0` 恢复立即融合。
- `d435_image_cache_size: 20`：彩色和深度各自缓存的帧数。
- `d435_image_buff_size: 16777216`：每个图像订阅的接收缓冲大小（16 MiB），配合 `queue_size=1` 处理大图像。

热成像检测持续运行。候选元数据放入短队列，定时器取最新已到等待时间的事件进行融合，不阻塞图像回调、不积压补发旧事件。
融合显示、定位和上报相应延迟约 0.40 秒（另加定时器调度/处理耗时），仍使用原始热成像事件时间查 TF。
同一轮融合的显示、候选点和证据图都使用该事件，避免把延迟事件与最新检测点混用。
不修改传感器时间戳，也不新增时间差丢弃条件；若等待结束后图像仍未到齐，仍按已有最近帧逻辑处理，时间差记录真实残差。
画框、深度定位和证据图使用选中的输入；空间点沿用热成像事件时间用于 TF。
缓存图像约占 92 MB（1280×720、BGR8 彩色和 16UC1 深度各 20 帧），比原来的 10 帧增加约 46 MB，不含接收及处理缓冲。

`thermal/report_candidate` JSON 和本地报告的 `result.source_timing` 保留：

- `thermal_stamp`：热成像软件接收时间，秒。
- `color_stamp` / `depth_stamp`：选中彩色/深度帧的原始时间，秒。
- `color_dt_s` / `depth_dt_s`：对应 D435 时间减热成像时间；正值表示 D435 帧较晚，缺帧为 null。
- `fusion_stamp` / `fusion_age_s`：融合时刻及其距离热成像事件的时间，用来区分等待延迟与配对误差。
- `color_depth_dt_s`：彩色帧时间减深度帧时间，缺帧时为 null。

证据图时间仍使用候选事件时间，供上报节点配对；原始彩色帧 header 不会被改写。
同一热帧多个候选的事件时间保留原有微秒编号偏移，`source_timing` 记录共同的真实输入时间。
节点日志每秒最多输出一次 `Thermal frame match`，显示彩色/深度的时间差和融合延迟（毫秒）。
2026-10-27 检查的四个正式 corridor_debug 录包包含候选状态、融合结果和 CameraInfo，但不含热成像原图及实际 D435 aux 图像；可估算配对改善，不能据此验证真实曝光同步或像素误差。
优化不改变映射矩阵、热源检测阈值及前机排除半径，也不代表硬件同步。

## 文件

```text
thermal_d435_fusion_node.py
thermal_d435_calib_node.py
config/thermal_d435_fusion.yaml
launch/thermal_d435_fusion.launch
README_thermal_d435_fusion.md
```

## 订阅话题

```text
/UAV0/thermal/target_detected
/UAV0/thermal/target_pixel
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
```

## 发布话题

```text
/UAV0/thermal/target_d435_pixel
/UAV0/thermal/target_camera_point
/UAV0/thermal/target_channel_position
/UAV0/thermal/fusion_valid
```

含义：

```text
/UAV0/thermal/target_d435_pixel
热源映射到 D435 彩色图像后的像素点，z 为该点深度，单位 m。

/UAV0/thermal/target_camera_point
热源在 camera_color_optical_frame 下的三维点。

/UAV0/thermal/target_channel_position
热源在 target_frame 下的位置，默认 target_frame=odom。

/UAV0/thermal/fusion_valid
True 表示热源像素、深度、内参、TF 全部有效。

/UAV0/thermal/d435_debug_image
热源框映射到 D435 后的调试画面。首幅彩色图到达后默认等待 5 秒，避开
D435 自动曝光尚未稳定的偏暗画面；预热只延迟调试图发布，不暂停深度融合。
```

## 启动流程

启动 roscore：

```bash
roscore
```

启动 D435：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_color:=true enable_depth:=true
```

启动热成像 ROS 节点：

```bash
cd ~/uvc_ubuntu
sudo -E bash -c 'source /opt/ros/noetic/setup.bash && cd /home/cat/uvc_ubuntu && python3 -u thermal_ros_node.py'
```

启动融合节点：

```bash
cd ~/uvc_ubuntu
python3 thermal_d435_fusion_node.py
```

查看结果：

```bash
rostopic echo /UAV0/thermal/fusion_valid
rostopic echo /UAV0/thermal/target_d435_pixel
rostopic echo /UAV0/thermal/target_camera_point
rostopic echo /UAV0/thermal/target_channel_position
```

## 单应矩阵标定

标定前先启动 D435 和热成像 ROS 节点，确保这些话题有数据：

```text
/UAV0/thermal/debug_image
/camera/color/image_raw
```

运行标定工具：

```bash
cd ~/uvc_ubuntu
python3 thermal_d435_calib_node.py
```

操作：

```text
先点击热成像图中的点
再点击 D435 彩色图中的同一个物理点
至少 4 对点，建议 6 到 10 对点
```

按键：

```text
e：计算并打印重投影误差
s：保存单应矩阵 H
r：清空点
q / ESC：退出
```

误差判断：

```text
mean < 5 px：很好
mean 5 到 10 px：可用
mean > 10 px 或 max > 20 px：建议重新标定
```

保存后会写入：

```text
config/thermal_d435_fusion.yaml
```

字段：

```yaml
thermal_to_d435_homography:
  - [h11, h12, h13]
  - [h21, h22, h23]
  - [h31, h32, h33]
```

## 配置文件

关键配置：

```yaml
thermal_width: 384
thermal_height: 288
uvc_offset_fix: 0
shift_x: 0
shift_y: 0
roi_margin_x: 30
roi_margin_y: 20
d435_exposure_warmup_seconds: 5.0

target_frame: "odom"
d435_optical_frame: "camera_color_optical_frame"
use_tf: true
```

如果后续建立真正的通道坐标系，可以把：

```yaml
target_frame: "channel"
```

## 常见检查命令

检查热成像：

```bash
rostopic echo /UAV0/thermal/target_detected
rostopic echo /UAV0/thermal/target_pixel
rqt_image_view /UAV0/thermal/debug_image
```

检查 D435：

```bash
rostopic echo /camera/color/camera_info
rostopic echo /camera/aligned_depth_to_color/image_raw
```

检查 TF：

```bash
rosrun tf tf_echo odom camera_color_optical_frame
```

## ROS 功能包启动

当前工程已经支持标准 catkin 功能包方式。完整迁移、编译和启动步骤见 `README_ROS_PACKAGE.md`。

只启动热成像检测：

```bash
roslaunch uvc_ubuntu thermal_detector.launch
```

同时启动热成像检测和 D435 融合：

```bash
roslaunch uvc_ubuntu thermal_d435_system.launch
```
