# 前机 UAV0 标定归档

来源为前机 `asus@192.168.230.212`，归档日期 2026-11-09。文件原样复制，来源路径与 SHA-256 见 `manifest.json`。采集 bag、图片和日志不归档。

| 文件 | 当前用途 |
| --- | --- |
| `camera_info/down_camera.yaml` | 下视相机内参；`down_camera.launch`、前机检测/精降入口读取 `$HOME/.ros/camera_info/down_camera.yaml` |
| `handeye_calibration/body_camera_03.yaml` | 前机正式检测入口使用的 `body → camera_link` 外参 |
| `handeye_calibration/body_camera.yaml` | 独立 `target_reporting/camera_body_tf.launch` 默认读取的外参 |
| `handeye_calibration/body_down_camera_park_03_stable.yaml` | 下视相机手眼标定原始结果；其外参已写入 `precision_landing/config/precision_landing.yaml` 和 `landing_search.yaml` |

在同一机体的新部署环境恢复文件（`-i` 在覆盖已有文件前询问）：

```bash
cd ~/match_ws
mkdir -p "$HOME/.ros/camera_info" "$HOME/handeye_calibration"
cp -i config/calibration/uav0/camera_info/down_camera.yaml "$HOME/.ros/camera_info/"
cp -i config/calibration/uav0/handeye_calibration/*.yaml "$HOME/handeye_calibration/"
```

本次归档未覆盖主目录原文件，也未改变 launch 读取路径。下视标定文件内的 `metadata.bag` 是采集来源记录，不是运行依赖。D435 内参由相机驱动发布；雷达外参及热成像融合参数保留在各包原配置中。前机标定对应前机安装状态，不应直接覆盖后机标定。
