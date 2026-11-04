#!/usr/bin/env python3
"""隔离 ROS master：暂停对中后模拟继续下沉，验证高度目标不随之下降。"""
import subprocess
import time
import unittest

import cv2
import numpy as np
import roslib.packages
import rospy
from test_landing_memory_ros import LandingMemoryTest


class DescentHoldTest(LandingMemoryTest):
    def setUp(self):
        self.use_fastlio = True
        super().setUp()
        self.stop(self.node)
        params = {
            'marker/size_m': .18,
            'center_alignment/bias_xy_m': [0., 0.],
            'camera_to_body/rotation': [[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]],
            'camera_to_body/translation_m': [0., 0., 0.],
            'control/freeze_descent_hold_height': True,
            'state_machine/align_error_m': .08,
            'stages/high/min_height_m': .6,
            'stages/high/error_m': .1,
            'stages/high/taper_start_height_m': 1.3,
            'stages/high/slowdown_start_height_m': 1.,
            'stages/high/slowdown_end_height_m': .8,
            'stages/high/slow_descent_mps': .1,
            'stages/auto_land/height_m': .6,
            'stages/auto_land/error_m': .05,
            'safety/total_timeout_sec': 30.,
        }
        for key, value in params.items():
            rospy.set_param(self.ns+'/node/'+key, value)
        self.draw_marker(0)
        path = roslib.packages.find_node('uav1_inplace_landing_test',
                                       'uav1_test_precision_landing_node')[0]
        self.node = subprocess.Popen([path, '__name:=node', '__ns:='+self.ns],
                                     stdout=self.log, stderr=self.log, start_new_session=True)
        self.commands.clear()
        self.wait(lambda: self.pubs['trigger'].get_num_connections(), 'restarted node')
        time.sleep(.4)

    def draw_marker(self, offset):
        image = np.full((480, 640, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        marker = cv2.aruco.drawMarker(dictionary, 1, 120)
        image[180:300, 260+offset:380+offset] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        self.image = image

    def latest_mask(self, mask):
        return bool(self.commands and self.commands[-1].type_mask == mask)

    def test_pause_loss_and_resume_keep_distinct_hold_heights(self):
        self.trigger = True
        self.wait(lambda: self.latest_mask(2503), 'aligned visual descent')
        # The synthetic marker is about 0.9 m away, so the real node must slow down.
        self.assertAlmostEqual(self.commands[-1].velocity.z, -.18, delta=.015)
        self.z = 2.9
        time.sleep(.15)
        self.draw_marker(80)
        self.wait(lambda: self.latest_mask(2531), 'XY error pauses descent')
        locked = self.commands[-1].position.z
        self.assertAlmostEqual(locked, 2.9, delta=.001)
        # Reproduce the flight: odometry sinks after the pause command.
        self.z = 2.82
        time.sleep(.35)
        self.assertTrue(self.latest_mask(2531))
        self.assertAlmostEqual(self.commands[-1].position.z, locked, delta=.001)
        self.have_marker = False
        self.wait(lambda: self.latest_mask(2552), 'loss uses remembered XY')
        self.z = 2.78
        time.sleep(.65)
        self.assertNotEqual(self.stage(), 'FIXED_XY_DESCENT')
        self.assertAlmostEqual(self.commands[-1].position.z, locked, delta=.001)
        # A new descent may capture a new hold height, instead of retaining the old one.
        self.draw_marker(0)
        self.have_marker = True
        self.wait(lambda: self.latest_mask(2503), 'reacquired and aligned descent')
        self.z = 2.65
        time.sleep(.15)
        self.draw_marker(80)
        self.wait(lambda: self.latest_mask(2531), 'second pause')
        self.assertAlmostEqual(self.commands[-1].position.z, 2.65, delta=.001)
        self.z = 2.58
        time.sleep(.35)
        self.assertAlmostEqual(self.commands[-1].position.z, 2.65, delta=.001)
        self.assertFalse(self.requests)


if __name__ == '__main__':
    suite = unittest.TestSuite([DescentHoldTest('test_pause_loss_and_resume_keep_distinct_hold_heights')])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
