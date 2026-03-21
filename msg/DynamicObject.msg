uint32 id
geometry_msgs/Point center
geometry_msgs/Vector3 size
geometry_msgs/Vector3 velocity
# 预测时刻，单位为秒；与 predicted_positions 按下标一一对应。
float32[] prediction_time_offsets
# 未来预测位置；长度为 0 表示当前没有可用预测。
geometry_msgs/Point[] predicted_positions
