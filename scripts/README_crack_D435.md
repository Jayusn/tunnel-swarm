# D435 裂缝检测
启动：`~/match_ws/scripts/start_experiment.sh`
停止：`~/match_ws/scripts/stop_experiment.sh`
只查看已保存照片：`~/match_ws/scripts/start_crack_viewer.sh`
相机配置：src/yolo_fastest_ros/ncnn/config/realsense_camera.yaml
检测配置：src/yolo_fastest_ros/ncnn/config/yolo_ros.yaml
照片：~/match_ws/captures/crack_live
实际相机 D435IF，RGB 1280×720/30 FPS，对齐深度 848×480/30 FPS。
裂缝单类352模型，hybrid整图+512切片，重叠25%，置信度70%。
启动脚本连接本机ROS master，不启动飞控或规划节点。相机需空闲。
