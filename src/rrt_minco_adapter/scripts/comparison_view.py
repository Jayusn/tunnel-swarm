"""RViz-only publishers for static-map comparison. No control topics."""
import colorsys
import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ComparisonView:
    def __init__(self, payload, scene, start, goal, height):
        self.frame = scene.get('frame_id', 'UAV0/camera_init')
        self.start, self.goal, self.height = start, goal, height
        self.map_pub = rospy.Publisher('/UAV0/rrt/comparison_map', Marker, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/UAV0/rrt/comparison_status', MarkerArray, queue_size=1, latch=True)
        self.paths = {name: rospy.Publisher('/UAV0/rrt/'+name+'_comparison_path', Path,
                                          queue_size=1, latch=True) for name in ('rrt', 'super')}
        marker = self.marker(0, Marker.CUBE_LIST)
        marker.scale.x = marker.scale.y = marker.scale.z = scene.get('resolution', .1)
        points = payload.get('voxels')
        if points is None:
            # Bare scenes contain obstacle rectangles only; draw their planning-height cells.
            r = marker.scale.x
            points = [(x, y, height) for cx, cy, w, h in scene['boxes']
                      for x in np.arange(cx-w/2+r/2, cx+w/2, r)
                      for y in np.arange(cy-h/2+r/2, cy+h/2, r)]
        low = min((p[2] for p in points), default=0.)
        high = max((p[2] for p in points), default=1.)
        for x, y, z in points:
            marker.points.append(Point(x=x, y=y, z=z))
            r, g, b = colorsys.hsv_to_rgb(.5+(z-low)/max(high-low, 1e-6)/3., 1., 1.)
            marker.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.))
        self.map_pub.publish(marker)
        for publisher in self.paths.values():
            path = Path(); path.header.frame_id = self.frame
            publisher.publish(path)  # clear any previous run's latched route
        self.status({})

    def marker(self, index, kind):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = self.frame, rospy.Time.now()
        marker.ns, marker.id, marker.type, marker.action = 'comparison', index, kind, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.color.a = 1.
        return marker

    def status(self, states):
        array = MarkerArray()
        for index, (xy, color) in enumerate(((self.start, (0., 1., 0.)),
                                            (self.goal, (1., .5, 0.)))):
            sphere = self.marker(index, Marker.SPHERE)
            sphere.pose.position.x, sphere.pose.position.y = xy
            sphere.pose.position.z = self.height
            sphere.scale.x = sphere.scale.y = sphere.scale.z = .15
            sphere.color.r, sphere.color.g, sphere.color.b = color
            array.markers.append(sphere)
            text = self.marker(index+2, Marker.TEXT_VIEW_FACING)
            text.pose.position = Point(x=xy[0], y=xy[1], z=self.height+.3)
            text.scale.z = .16
            text.color.r, text.color.g, text.color.b = color
            text.text = ('Start' if index == 0 else 'Goal') + ' (%g,%g)' % tuple(xy)
            array.markers.append(text)
        for index, name in enumerate(('rrt', 'super')):
            text = self.marker(index+4, Marker.TEXT_VIEW_FACING)
            text.pose.position = Point(x=self.start[0]+1., y=self.start[1]-.6, z=1.+index*.3)
            text.scale.z = .14
            state = states.get(name)
            text.text = name.upper() + ': ' + ('waiting' if state is None else
                        ('SUCCESS' if state['success'] else 'FAILED: '+state['error']))
            text.color.r, text.color.g, text.color.b = ((0., .7, 0.) if state and state['success'] else (1., .15, .1))
            array.markers.append(text)
        self.status_pub.publish(array)

    def path(self, name, trajectory=None):
        path = Path()
        path.header.frame_id, path.header.stamp = self.frame, rospy.Time.now()
        times = [] if trajectory is None else np.linspace(0., trajectory.total_time, max(2, int(trajectory.total_time/.1)+1))
        for t in times:
            xy, _ = trajectory.sample(t)
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position = Point(x=float(xy[0]), y=float(xy[1]), z=self.height)
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.paths[name].publish(path)
