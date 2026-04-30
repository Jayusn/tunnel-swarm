uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4

uint32 id
geometry_msgs/Point center
geometry_msgs/Vector3 size
geometry_msgs/Vector3 velocity
uint8 object_class
