#!/usr/bin/env python3
"""隔离master验证：交接保持Z、XY/速度连续达标后才开始下降。"""
import subprocess
import time
import unittest

import cv2
import numpy as np
import roslib.packages
import rospy
from test_landing_memory_ros import LandingMemoryTest


class FixedXYSettleTest(LandingMemoryTest):
    def setUp(self):
        self.use_fastlio = True
        super().setUp()
        self.stop(self.node)
        params = {
            'control/fixed_xy_settle_enabled': True,
            'control/fixed_xy_handoff_sec': 1.5,
            'control/fixed_xy_settle_error_m': .05,
            'control/fixed_xy_settle_speed_mps': .05,
            'control/fixed_xy_settle_sec': .5,
            'control/use_fastlio_z_for_height': True,
            'center_alignment/bias_xy_m': [0., 0.],
            'stages/high/min_height_m': .65,
            'stages/auto_land/height_m': .65,
            'stages/auto_land/error_m': .05,
            'safety/total_timeout_sec': 30.,
        }
        for key, value in params.items():
            rospy.set_param(self.ns+'/node/'+key, value)
        self.z = .43
        image = np.full((480, 640, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        marker = cv2.aruco.drawMarker(dictionary, 1, 120)
        image[180:300, 260:380] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        self.image = image
        path = roslib.packages.find_node('uav1_inplace_landing_test',
                                       'uav1_test_precision_landing_node')[0]
        self.node = subprocess.Popen([path, '__name:=node', '__ns:='+self.ns],
                                     stdout=self.log, stderr=self.log, start_new_session=True)
        self.commands.clear()
        self.wait(lambda: self.pubs['trigger'].get_num_connections(), 'restarted node')
        time.sleep(.4)

    def test_handoff_and_continuous_xy_settle_before_descent(self):
        self.trigger = True
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'fixed XY')
        time.sleep(.08)
        target = self.commands[-1]
        z = target.position.z
        locked_yaw = target.yaw
        self.assertAlmostEqual(z, .43, delta=.001)
        # 即使刚锁点已对准，1.5秒交接也必须保持高度。
        time.sleep(.7)
        self.assertAlmostEqual(self.commands[-1].position.z, z, delta=.001)
        # 实际下沉、横向偏离都不能把保持Z继续向下移动。
        self.z = .35
        self.x = target.position.x + .12
        self.have_marker = False
        time.sleep(1.1)
        self.assertAlmostEqual(self.commands[-1].position.z, z, delta=.001)
        # 位置到位但仍有横向速度，继续保持Z。
        self.x, self.y = target.position.x, target.position.y
        self.vx = .1
        time.sleep(.7)
        self.assertAlmostEqual(self.commands[-1].position.z, z, delta=.001)
        self.vx = 0.
        time.sleep(.25)
        self.assertAlmostEqual(self.commands[-1].position.z, z, delta=.001)
        self.vx = .1  # 速度超限打断连续稳定计时。
        time.sleep(.2)
        self.vx = 0.
        time.sleep(.3)
        self.assertAlmostEqual(self.commands[-1].position.z, z, delta=.001)
        self.wait(lambda: self.commands[-1].position.z < z-.025, 'settled descent')
        self.assertAlmostEqual(self.commands[-1].yaw, locked_yaw, places=5)
        # 交接只等待一次；已开始的下降不因之后的速度样本重新停住。
        before = self.commands[-1].position.z
        self.vx = .1
        time.sleep(.3)
        self.assertLess(self.commands[-1].position.z, before-.02)


if __name__ == '__main__':
    suite = unittest.TestSuite([FixedXYSettleTest(
        'test_handoff_and_continuous_xy_settle_before_descent')])
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
