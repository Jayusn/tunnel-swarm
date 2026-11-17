#!/usr/bin/env python3
"""Accumulate registered obstacle points; freeze an operator-surveyed static scene."""
import json
from pathlib import Path
import sys
import threading

import rospkg
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, str(Path(rospkg.RosPack().get_path('rrt_minco_adapter')) / 'scripts'))
from cloud_scene import CloudScene


class CloudToScene:
    def __init__(self):
        self.frame = rospy.get_param('~frame_id', 'UAV0/camera_init')
        self.map = CloudScene(rospy.get_param('~bounds'), rospy.get_param('~resolution', .1),
                              rospy.get_param('~z_min', .3), rospy.get_param('~z_max', .7))
        self.lock = threading.Lock()
        self.frozen = False
        self.cloud_count = 0
        self.pub = rospy.Publisher('~scene', String, queue_size=1, latch=True)
        self.grid_pub = rospy.Publisher('~grid_map', OccupancyGrid, queue_size=1, latch=True)
        self.loaded_at = rospy.Time.now()
        self.sub = rospy.Subscriber('~cloud', PointCloud2, self.cloud, queue_size=1)
        self.service = rospy.Service('~freeze', Trigger, self.freeze)
        self.grid_timer = rospy.Timer(rospy.Duration(1.), self.publish_grid)
        rospy.loginfo('Accumulating static map z=[%.2f, %.2f]; call ~freeze after surveying bounds',
                      self.map.z_min, self.map.z_max)

    def publish_grid(self, _event):
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame
        grid.header.stamp = rospy.Time.now()
        grid.info.map_load_time = self.loaded_at
        grid.info.resolution = self.map.resolution
        grid.info.origin.position.x = self.map.bounds[0]
        grid.info.origin.position.y = self.map.bounds[2]
        grid.info.origin.orientation.w = 1.
        with self.lock:
            grid.info.width, grid.info.height, grid.data = self.map.occupancy_grid()
        self.grid_pub.publish(grid)

    def cloud(self, msg):
        if msg.header.frame_id != self.frame:
            rospy.logwarn_throttle(5., 'Point cloud frame mismatch; expected ' + self.frame)
            return
        if not {'x', 'y', 'z'}.issubset({field.name for field in msg.fields}):
            rospy.logwarn_throttle(5., 'Point cloud requires XYZ fields')
            return
        with self.lock:
            if self.frozen:
                return
            self.map.add(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
            self.cloud_count += 1

    def freeze(self, _request):
        with self.lock:
            if not self.cloud_count:
                return TriggerResponse(False, 'No valid registered point cloud received')
            scene = self.map.scene(self.frame)
            self.pub.publish(String(data=json.dumps(scene, allow_nan=False)))
            self.frozen = True
            return TriggerResponse(True, 'Static scene frozen: %d cells, %d rectangles; restart node to rebuild'
                                   % (len(self.map.cells), len(scene['boxes'])))


if __name__ == '__main__':
    rospy.init_node('rrt_cloud_scene')
    node = CloudToScene()
    rospy.spin()
