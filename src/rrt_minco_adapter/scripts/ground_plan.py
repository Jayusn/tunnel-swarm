#!/usr/bin/env python3
"""Plan once on the frozen map and publish an RViz path; no flight commands."""
import json
from pathlib import Path
import sys

import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String

PACKAGE = Path(rospkg.RosPack().get_path('rrt_minco_adapter'))
sys.path.insert(0, str(PACKAGE / 'scripts'))
from plan_rrt import solve


def main():
    rospy.init_node('rrt_ground_planning_check', anonymous=True)
    frame = rospy.get_param('~frame_id', 'UAV0/camera_init')
    scene_topic = rospy.get_param('~scene_topic', '/UAV0/rrt/scene')
    path_topic = rospy.get_param('~path_topic', '/UAV0/rrt/dry_run_path')
    start = rospy.get_param('~start', [0.01, 0.01])
    goal = rospy.get_param('~goal', [3., 3.])
    height = float(rospy.get_param('~height', .5))
    if len(start) != 2 or len(goal) != 2 or not np.isfinite(list(start) + list(goal) + [height]).all():
        raise ValueError('start/goal 必须为两个有限XY坐标，height必须为有限数值')
    output = Path(rospy.get_param('~output_dir', str(Path.home() / '.ros/rrt_ground_check'))).expanduser()
    rospy.loginfo('等待冻结地图 %s；请先调用 /UAV0/rrt/freeze_map', scene_topic)
    message = rospy.wait_for_message(scene_topic, String, timeout=30)
    scene = json.loads(message.data)
    if scene.get('frame_id') != frame:
        raise ValueError('地图坐标系不匹配，期望 ' + frame)
    scene['boundary_clearance'] = 0.
    output.mkdir(parents=True, exist_ok=True)
    (output / 'scene.json').write_text(json.dumps(scene, indent=2, allow_nan=False))
    rospy.loginfo('障碍矩形数量：%d；虚拟起点%s，目标%s，高度%.2fm',
                  len(scene['boxes']), start, goal, height)
    trajectory, result = solve(
        scene, start=start, goal=goal,
        speed=float(rospy.get_param('~speed', .2)),
        acceleration=float(rospy.get_param('~acceleration', .8)),
        clearance=float(rospy.get_param('~clearance', .2)),
        reserve=float(rospy.get_param('~tracking_margin', .1)))
    result.update(frame_id=frame, visualization_height=height)
    (output / 'trajectory_xy.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    path = RosPath()
    path.header.frame_id, path.header.stamp = frame, rospy.Time.now()
    for t in np.linspace(0, trajectory.total_time,
                         max(2, int(trajectory.total_time / .1) + 1)):
        xy, _ = trajectory.sample(float(t))
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y = map(float, xy)
        pose.pose.position.z = height
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    publisher = rospy.Publisher(path_topic, RosPath, queue_size=1, latch=True)
    publisher.publish(path)
    rospy.loginfo('规划成功，参考轨迹时长%.2f秒；文件保存到%s', trajectory.total_time, output)
    rospy.loginfo('仅发布显示路径 %s，不发送飞行指令。保持终端运行，Ctrl+C退出。', path_topic)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (rospy.ROSException, ValueError, RuntimeError, KeyError, TypeError, OSError) as error:
        rospy.logerr('地面规划失败：%s', error)
        sys.exit(1)
