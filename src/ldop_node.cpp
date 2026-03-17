#include <ros/ros.h>
#include <ldop/ldop.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ldop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ldop::LDOP ldop_node(nh, pnh);

  ros::AsyncSpinner spinner(ldop_node.spinnerThreadCount());
  spinner.start();

  ros::waitForShutdown();
  return 0;
}
