# Exact request header, independent of the new trajectory start time.
std_msgs/Header header
# Application identifier; ROS may overwrite header.seq during publication.
uint32 request_id
traj_utils/PolyTraj trajectory
