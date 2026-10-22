# One message per completed mapping frame. header.stamp is publication time.
std_msgs/Header header
time scan_end_stamp
# Snapshot under the existing buffer mutex immediately before sync consumes data.
time queue_sample_stamp
uint32 lidar_queue_size
uint32 lidar_time_queue_size
uint32 imu_queue_size
time oldest_lidar_stamp
time oldest_imu_stamp
# Ages at queue_sample_stamp, seconds; NaN when the corresponding queue is empty.
# LiDAR stamp is scan START, so its age includes acquisition of the scan itself.
float64 oldest_lidar_age_s
float64 oldest_imu_age_s
# Last callback ENTRY times and source stamps observed at the snapshot.
time lidar_callback_receive_stamp
time lidar_callback_source_stamp
time imu_callback_receive_stamp
time imu_callback_source_stamp
uint32 imu_replay_count
float64 handoff_lock_wait_ms
float64 imu_replay_ms
float64 scan_end_to_handoff_ms
# Includes callback dispatch, synchronization, matching, map update and publication;
# excludes this diagnostic publication and the final loop sleep.
float64 frame_wall_ms
