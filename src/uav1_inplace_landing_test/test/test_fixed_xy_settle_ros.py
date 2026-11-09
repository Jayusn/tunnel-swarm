#!/usr/bin/env python3
"""隔离master验证：限时交接、可选二次对中及总超时优先下降。"""
import subprocess
import time
import unittest
import math
from pathlib import Path

import cv2
import numpy as np
import roslib.packages
import rospy
import yaml
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
        if self._testMethodName == 'test_locked_target_descends_after_handoff_without_rechecking_error':
            config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                     'config/standalone_18cm_landing.yaml').read_text())
            for key in ('fixed_xy_settle_enabled', 'fixed_xy_handoff_sec'):
                params['control/'+key] = config['control'][key]
            self.assertFalse(params['control/fixed_xy_settle_enabled'])
            params['control/fixed_yaw_deg'] = -90.
        if self._testMethodName == 'test_timeout_overrides_handoff_and_xy_settle_wait':
            params['control/fixed_xy_handoff_sec'] = 30.
            params['safety/total_timeout_sec'] = 2.
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

    def test_locked_target_descends_after_handoff_without_rechecking_error(self):
        self.trigger = True
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'fixed XY')
        time.sleep(.08)
        target = self.commands[-1]
        # 实际航向为0；桥梁指定航向必须替代当前航向捕获值。
        self.assertAlmostEqual(target.yaw, -math.pi/2., places=5)
        z = target.position.z
        # 锁点后模拟12厘米偏差、横向速度和丢码；交接仍只按时间结束。
        self.x = target.position.x + .12
        self.vx = .1
        self.have_marker = False
        time.sleep(.7)
        self.assertAlmostEqual(self.commands[-1].position.z, z, delta=.001)
        self.wait(lambda: self.commands[-1].position.z < z-.025, 'timed handoff descent')
        self.assertAlmostEqual(self.commands[-1].position.x, target.position.x, places=5)
        self.assertAlmostEqual(self.commands[-1].position.y, target.position.y, places=5)
        self.assertAlmostEqual(self.commands[-1].yaw, target.yaw, places=5)
        before = self.commands[-1].position.z
        time.sleep(.3)
        self.assertLess(self.commands[-1].position.z, before-.02)

    def test_timeout_overrides_handoff_and_xy_settle_wait(self):
        started = time.monotonic()
        self.trigger = True
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'fixed XY')
        time.sleep(.08)
        target = self.commands[-1]
        self.x = target.position.x + .20
        self.vx = .1
        self.have_marker = False
        time.sleep(.3)
        self.assertAlmostEqual(self.commands[-1].position.z, target.position.z, delta=.001)
        self.wait(lambda: self.commands[-1].position.z < target.position.z-.025,
                  'timeout overrides handoff and error')
        self.assertLess(time.monotonic()-started, 3.5)
        self.assertAlmostEqual(self.commands[-1].position.x, target.position.x, places=5)
        before = self.commands[-1].position.z
        time.sleep(.3)
        self.assertLess(self.commands[-1].position.z, before-.02)


if __name__ == '__main__':
    suite = unittest.TestSuite(FixedXYSettleTest(name) for name in (
        'test_handoff_and_continuous_xy_settle_before_descent',
        'test_locked_target_descends_after_handoff_without_rechecking_error',
        'test_timeout_overrides_handoff_and_xy_settle_wait'))
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
