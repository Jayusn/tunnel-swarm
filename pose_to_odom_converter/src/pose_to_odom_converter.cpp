#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;

void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.pose.pose = msg->pose;
    // 设置其他必要的nav_msgs::Odometry字段
    odom_pub.publish(odom_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_to_odom_converter");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/mavros/vision_pose/pose", 10, poseStampedCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/converted_odom", 10);

    ros::spin();
    return 0;
}
