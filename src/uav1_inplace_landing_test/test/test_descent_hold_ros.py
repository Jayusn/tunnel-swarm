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
        if self._testMethodName == 'test_high_descent_keeps_xy_control_then_aligns_low':
            params['stages/high/align_while_descending_above_m'] = 1.0
            params['stages/high/slow_descent_mps'] = .15
        if self._testMethodName == 'test_fastlio_z_controls_speed_and_final_alignment':
            params.update({
                'control/use_fastlio_z_for_height': True,
                'stages/high/align_while_descending_above_m': 1.0,
                'stages/high/slow_descent_mps': .15,
                'state_machine/align_error_m': .15,
                'stages/high/error_m': .15,
                'stages/high/taper_start_height_m': 1.0,
                'stages/high/min_height_m': .65,
                'stages/auto_land/height_m': .65,
            })
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

    def draw_marker(self, offset, size=120):
        image = np.full((480, 640, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        marker = cv2.aruco.drawMarker(dictionary, 1, size)
        x, y = 320-size//2+offset, 240-size//2
        image[y:y+size, x:x+size] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        self.image = image

    def latest_mask(self, mask):
        return bool(self.commands and self.commands[-1].type_mask == mask)

    def test_high_descent_keeps_xy_control_then_aligns_low(self):
        self.draw_marker(80, 80)  # Visual height about 1.37 m, XY error about 18 cm.
        time.sleep(.2)
        self.trigger = True
        self.wait(lambda: self.latest_mask(2503), 'high simultaneous alignment/descent')
        cmd = self.commands[-1]
        self.assertAlmostEqual(cmd.velocity.z, -.25, delta=.001)
        self.assertGreater(abs(cmd.velocity.x)+abs(cmd.velocity.y), .02)
        self.assertGreater(np.hypot(self.errors[-1].point.x, self.errors[-1].point.y), .1)
        self.draw_marker(80, 120)  # Below 1 m, the same lateral offset must pause descent.
        self.wait(lambda: self.latest_mask(2531), 'low altitude alignment pause')
        self.draw_marker(0, 120)
        self.wait(lambda: self.latest_mask(2503), 'low aligned descent resumes')
        self.assertAlmostEqual(self.commands[-1].velocity.z, -.20, delta=.015)

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

    def test_fastlio_z_controls_speed_and_final_alignment(self):
        # Vision remains about 0.9 m; only FAST-LIO Z changes. MAVROS Z is 30 m.
        self.z = 1.2
        self.draw_marker(40)
        time.sleep(.2)
        self.trigger = True
        self.wait(lambda: self.latest_mask(2503), 'FAST-LIO high descent')
        self.assertAlmostEqual(self.commands[-1].velocity.z, -.25, delta=.001)
        self.z = .9
        self.wait(lambda: self.latest_mask(2503) and
                  abs(self.commands[-1].velocity.z+.20) < .005, 'FAST-LIO speed taper')
        self.z = .75
        self.draw_marker(60)
        self.wait(lambda: self.latest_mask(2531), 'FAST-LIO low-height error gate')
        self.z = .65
        self.wait(lambda: self.stage() == 'ALIGN', 'FAST-LIO final alignment height')
        self.assertNotEqual(self.stage(), 'FIXED_XY_DESCENT')
        self.draw_marker(0)
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'aligned lock at FAST-LIO Z 0.65')
        self.assertGreater(self.targets[-1].pose.position.z, .85)
        self.assertFalse(self.requests)


if __name__ == '__main__':
    suite = unittest.TestSuite([
        DescentHoldTest('test_pause_loss_and_resume_keep_distinct_hold_heights'),
        DescentHoldTest('test_high_descent_keeps_xy_control_then_aligns_low'),
        DescentHoldTest('test_fastlio_z_controls_speed_and_final_alignment'),
    ])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
