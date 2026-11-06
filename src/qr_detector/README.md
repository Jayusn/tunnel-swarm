# qr_detector

这是一个用于 ROS Noetic 的普通二维码检测基线包，面向 Intel
RealSense D435 的彩色图像与对齐深度图。

本包用于无人机比赛通道内作业目标点中的普通 QR Code 检测，不是降落用
ArUco 码检测器。

## 功能

- 订阅 RealSense 彩色图像、对齐深度图和相机内参。
- 使用 OpenCV `cv2.QRCodeDetector` 检测普通 QR Code 的四角，并在系统版
  OpenCV 未链接 QUIRC、只能检测而不能解码时自动调用 `libzbar` 解码。
- 角点检测结果先作为本机候选显示；只有通过内部二维码真实性校验和深度
  一致性检查后，才允许进入稳定确认和远程上报。二维码内容默认不输出。
- 根据四个角点计算二维码中心像素坐标。
- 在中心点附近取深度窗口中位数，避免只依赖单个深度像素。
- 同时检查中心和四个角点的深度有效率、标准差和深度范围，排除跨越网格、
  地面和背景的伪四边形。
- 使用相机内参反投影，输出二维码中心在相机坐标系下的位置。
- 发布 JSON 检测状态、`PoseStamped` 位姿和调试图像。
- 支持连续多帧确认、短时间丢失保持和 EMA 位置滤波；未验证候选不会被
  `target_reporting` 累计为确认目标。
- 针对 D435 启动时自动曝光尚未稳定的问题，首帧后默认预热 5 秒；预热阶段只
  保留本机候选观察，不会确认或远程上报暗启动帧。
- 支持保存检测失败帧，便于后续采集数据训练 YOLO 兜底模型。
- 有效候选会发布到机载候选话题供 target_reporting/RViz 观察，但默认不发送到远程端；只有连续确认后的结果才进入稳定上报链路。

## 编译

运行时需要系统动态库 `libzbar0`。比赛机当前已安装；新设备可执行：

```bash
sudo apt install libzbar0
```

将 `qr_detector` 放到 `catkin_ws/src` 下，然后编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/qr_detector/scripts/qr_detector_node.py
```

## 启动

```bash
roslaunch qr_detector qr_detector.launch
```

## 订阅话题

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`

## 发布话题

- `/UAV0/vision/qr_pose_camera`
  - 类型：`geometry_msgs/PoseStamped`
  - 含义：二维码中心点相对于相机坐标系的位置。
- `/UAV0/vision/qr_detected`
  - 类型：`std_msgs/String`
  - 内容：JSON 字符串，包括 `detected`、`data`、`points`、
    `center_u`、`center_v`、`x`、`y`、`z`、`method` 等字段。
- `/UAV0/vision/qr_debug_image`
  - 类型：`sensor_msgs/Image`
  - 内容：带二维码边框、中心点、识别内容和相机坐标的调试图像。

## 参数

参数文件位于 `config/qr_detector.yaml`。
UAV1 检测入口直接加载该文件中的检测参数，不在 launch 中覆盖检测阈值、
预处理或局部搜索设置；修改后重启二维码检测节点生效。

- `image_topic`：彩色图像话题。
- `depth_topic`：对齐到彩色图的深度图话题。
- `camera_info_topic`：彩色相机内参话题。
- `depth_window_size`：中心点附近取深度中位数的窗口大小，默认 `11`。
- `min_area`：二维码四边形的最小像素面积，当前为 `150`。
- `min_side_length`：二维码最短边像素阈值，当前为 `16`。
- `max_side_ratio`：最长边与最短边比例上限，当前为 `4`。
- `max_angle_cos`：角点直角约束阈值，当前为 `0.80`。
- `max_quad_area_ratio`、`max_quad_width_ratio`、`max_quad_height_ratio`：
  限制候选四边形不能占据整幅背景图。
- `qr_eps_x`、`qr_eps_y`：OpenCV QR 角点扫描容差，当前均为 `0.22`。
- `require_decode_for_confirmation`：是否将内部解码成功作为确认门槛，当前为
  `true`。同一空间连续目标只需成功解码一帧，随后可在短时缓存内依靠角点、
  深度和连续帧完成确认。
- `decode_qr_data`：是否在调试图、检测结果和上报记录中携带解码内容，当前为
  `true`。
- `decode_verification_hold_seconds`：单帧成功解码对同一目标的验证有效期，当前
  为 `2.0` 秒。
- `decode_verification_max_center_shift_px`：复用单帧解码验证时允许的图像中心
  位移，当前为 `120` 像素，避免将验证结果误用于远处的其他四边形。
  超过该位移时立即尝试重新解码，无需等待 `2.0` 秒有效期结束；局部搜索和
  放大图像均换算到原始整图坐标后判断位移。
- `preprocess_mode`：预处理方式，当前为 `gray`。
- `upscale_factor`：检测前图像放大倍数，当前为 `1.5`。
- `enable_preprocess_fallbacks`：局部搜索中尝试其他预处理方式，当前为 `true`。
- `enable_zbar_full_frame_fallback`：OpenCV 局部和整图搜索均未返回有效候选时，
  使用 ZBar 在原始尺寸灰度整图上补充搜索，当前为 `true`。成功结果标记为
  `method=zbar_full_frame`，仍经过原有几何、深度、曝光预热和确认检查。
- `zbar_full_frame_interval`：补充搜索最小间隔，当前 `0.5` 秒（最多约每秒两次）；
  失败也计入间隔，避免持续漏检时每帧重复搜索。设开关为 `false` 可关闭补充搜索。
- `enable_local_search`：优先搜索最近二维码附近区域，当前为 `true`；
  `local_search_hold_seconds` 为 `2.0` 秒，`local_search_margin_ratio` 为 `2.0`。
- `confirm_frames`：连续通过真实性和深度校验多少帧后确认 `detected=true`，当前为 `1`；
  首帧满足条件即可确认，仍需通过解码验证和深度校验。
- `startup_warmup_seconds`：D435 首帧后的曝光预热时间，默认 `5.0` 秒；预热期间
  候选仍可在机载 RViz 观察，但不会进入稳定确认和远程上报。
- `draw_raw_candidates`：是否在二维码调试图像中绘制单帧原始候选，当前为 `false`；RViz 候选标记仍由 target_reporting 单独显示。
- `draw_current_decoded_only`：默认 `false`，桥梁配置启用。只显示已确认目标当前帧的有效四边形；允许成功解码后的有效连续跟踪，不显示未确认候选或丢失后的旧框。仅影响画面，不改变检测、坐标和上报逻辑。
- `lost_hold_time`：短时间丢失后保留上一帧结果的时间，默认 `0.3` 秒。
- `ema_alpha`：相机坐标 `x,y,z` 的 EMA 滤波系数。
- `save_failed_frame`：是否保存检测失败帧，默认 `false`。
- `failed_frame_dir`：检测失败帧保存目录，默认 `~/qr_failed_frames`。
- `failed_frame_interval`：检测失败帧保存间隔，默认 `1.0` 秒。

## 输出坐标

当前节点只输出二维码中心点在相机坐标系下的位置：

```text
Z = depth
X = (center_u - cx) * Z / fx
Y = (center_v - cy) * Z / fy
```

后续如果需要输出通道坐标系下的位置，可以在此结果基础上接入无人机定位和
相机外参变换。

## 后续扩展

当前版本使用 OpenCV `QRCodeDetector` 定位、`libzbar` 解码回退。如果比赛现场光照、运动模糊或角度
导致 OpenCV 检测不稳定，可以在节点中的 QR 检测入口后面加入 YOLO 兜底分支，
复用现有的深度取值、相机反投影、滤波和发布逻辑。
