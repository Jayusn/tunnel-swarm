# Controller-issued request. Echo the full header in the result.
std_msgs/Header header
# Application identifier; ROS may overwrite header.seq during publication.
uint32 request_id
int32 trajectory_id
bool stop_current
float64 path_error
