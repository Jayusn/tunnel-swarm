std_msgs/Header header

uint64 session_id
uint64 goal_id

uint8 TARGET_FRONTIER=0
uint8 TARGET_FORWARD_FALLBACK=1
uint8 TARGET_VERTICAL_PROBE=2
uint8 TARGET_TURN_ALIGNMENT=3
uint8 target_type

uint8 MOTION_MOVE=0
uint8 MOTION_YAW_ONLY=1
uint8 motion_type

geometry_msgs/Pose target_pose
bool enforce_yaw
quadrotor_msgs/ExplorationMotionConstraint motion_constraint
