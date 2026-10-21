# One sample per LiDAR EKF measurement update (normally about 10 Hz).
# stamp: scan end time shared by the before/after states, NOT publication time.
# frame_id: world frame (e.g. UAV1/camera_init).
std_msgs/Header header
# Position [m] and velocity [m/s] in header.frame_id, ordered x/y/z.
float64[3] position_before
float64[3] position_after
float64[3] velocity_before
float64[3] velocity_after
# Accelerometer bias [m/s^2] in the IMU/body frame, ordered x/y/z.
float64[3] accel_bias_before
float64[3] accel_bias_after
# Gravity [m/s^2] in header.frame_id, ordered x/y/z.
float64[3] gravity_before
float64[3] gravity_after
