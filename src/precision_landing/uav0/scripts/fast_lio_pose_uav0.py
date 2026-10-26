#!/usr/bin/env python3
"""仅转换消息类型，原样保留 FAST-LIO 位姿、坐标系及测量时间。"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("landing_fast_lio_pose")
    publisher = rospy.Publisher("pose", PoseStamped, queue_size=1)
    def callback(message):
        pose = PoseStamped(header=message.header, pose=message.pose.pose)
        publisher.publish(pose)
    subscriber = rospy.Subscriber("odometry", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
