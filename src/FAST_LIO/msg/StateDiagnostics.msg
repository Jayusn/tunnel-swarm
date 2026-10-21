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

# Body-to-world attitude, quaternion x/y/z/w, at the same scan-end time.
float64[4] attitude_before
float64[4] attitude_after
# Gyroscope bias in the IMU/body frame [rad/s].
float64[3] gyro_bias_before
float64[3] gyro_bias_after
# Full error-state covariance, row-major (index = row*23+column).
# pos[0:3], rot[3:6], extrinsic_rot[6:9], extrinsic_pos[9:12],
# vel[12:15], gyro_bias[15:18], accel_bias[18:21], gravity tangent[21:23].
float64[529] covariance_before
float64[529] covariance_after
# Final measurement linearization, NOT residuals recomputed at the final state.
# No valid final measurement: residual statistics are NaN, never stale values.
uint32 downsampled_points
uint32 matched_points
float64 residual_mean_abs_m
float64 residual_rms_m
# Counts include the initial i=-1 linearization in IKFoM.
uint32 iteration_count
uint32 valid_iteration_count
bool final_measurement_valid
# True only if the last valid increment passed the original convergence test;
# does not include the forced rematching flag in IKFoM.
bool last_increment_converged
bool covariance_updated
float64 update_wall_ms
