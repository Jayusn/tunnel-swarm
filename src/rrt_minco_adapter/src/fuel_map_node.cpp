#include <ros/ros.h>
#include <plan_env/sdf_map.h>

// Reuse FUEL mapping without starting its exploration FSM or command publishers.
int main(int argc, char** argv) {
  ros::init(argc, argv, "rrt_fuel_map");
  ros::NodeHandle nh("~");
  fast_planner::SDFMap map;
  map.initMap(nh);
  ros::spin();
  return 0;
}
