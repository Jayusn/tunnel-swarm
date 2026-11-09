# UAV1 标定归档

2026-11-09 从本机 `/home/oem` 复制，原文件保持不变。来源、归档相对路径与 SHA-256 见 `manifest.json`。

| 文件 | 来源 / 用途 |
| --- | --- |
| `camera_info/down_camera.yaml` | `~/.ros/camera_info/down_camera.yaml`；下视相机内参，传感器与精降入口实际读取 |
| `handeye_calibration/body_camera_03.yaml` | 前视相机手眼外参；前后机检测综合入口显式传入 |
| `handeye_calibration/body_down_camera_uav1_fixed_z_04.yaml` | UAV1 下视外参原始结果；仓库精降配置记录此文件为来源 |
| `handeye_calibration/body_down_camera_uav1_park.yaml`、`body_down_camera_uav1_park_03.yaml` | 同目录旧标定结果，留作追溯，不自动启用 |

恢复命令见根目录 README。单独使用 `target_reporting/launch/camera_body_tf.launch` 时，其默认 `~/handeye_calibration/body_camera.yaml` 在本机不存在，应显式传入实际的 `calibration_file`；综合检测入口已使用 `body_camera_03.yaml`。

运行中还使用仓库内这些标定或补偿配置：

- `src/precision_landing/config/down_camera_extrinsic_uav1.yaml`：精降使用的 UAV1 下视外参。
- `src/control/config/center_offset_uav1.yaml`：降落中心补偿。
- `src/uvc_ubuntu/config/thermal_d435_fusion.yaml`：热成像与 D435 配准。
- `src/FAST_LIO/config/mid360.yaml`：雷达/IMU 配置及外参。
- `src/target_reporting/config/uav1_channel_transform.yaml`：文件注明当前为单位变换占位，不能当作已完成的通道坐标标定。

本次未采集 UAV0 实机主目录文件。不能把此处 UAV1 标定直接替代 UAV0 标定。原始标定 bag 不在上传范围内。
