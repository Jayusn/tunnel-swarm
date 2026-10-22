# header.stamp: control-cycle input snapshot, before process(); published after it.
# Ages are seconds in ROS time; missing input => zero stamp and NaN age.
std_msgs/Header header
time publish_stamp
time lio_latest_source_stamp
time lio_latest_receive_stamp
time lio_used_source_stamp
time lio_used_receive_stamp
# Legacy PX4 fields: zero stamps / NaN age when control feedback uses FAST-LIO.
time px4_used_source_stamp
time px4_used_receive_stamp
float64 lio_latest_age_s
float64 lio_used_age_s
float64 px4_used_age_s
float64 since_lio_accept_s
float64 rejected_hold_s
uint64 lio_rejected_total
uint32 lio_rejected_consecutive
bool state_prepared
uint32 mpc_solve_count
bool mpc_solve_success
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Steady-clock durations. Input dispatch accumulates since previous control cycle.
float64 input_dispatch_ms
float64 process_wall_ms
# execMPC duration includes previous preparation-thread join and feedback;
# excludes asynchronous preparation work after execMPC returns.
float64 mpc_exec_wall_ms
