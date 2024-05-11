#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry vins_odom;
void vins_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
   vins_odom = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vins_to_mavros");
  ros::NodeHandle nh;
  ros::Publisher pos_pub=nh.advertise<geometry_msgs::PoseStamped> ("mavros/vision_pose/pose",10);
  ros::Subscriber vins_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_fusion/imu_propagate", 2, vins_cb);
  geometry_msgs::PoseStamped pose;
  setlocale(LC_ALL,"");
  ros::Rate rate(30.0);
  ROS_INFO("vins开始融合");
  while (ros::ok())
  {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = vins_odom.pose.pose.position.x;
    pose.pose.position.y = vins_odom.pose.pose.position.y;
    pose.pose.position.z = vins_odom.pose.pose.position.z;
    pose.pose.orientation.x = vins_odom.pose.pose.orientation.x;
    pose.pose.orientation.y = vins_odom.pose.pose.orientation.y;
    pose.pose.orientation.z = vins_odom.pose.pose.orientation.z;
    pose.pose.orientation.w = vins_odom.pose.pose.orientation.w;
    pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}