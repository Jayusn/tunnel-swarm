#!/usr/bin/env python3
"""用桥梁实际launch验证FAST-LIO末端交接；使用隔离master，不连接硬件。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'uav1_inplace_landing_test/test'))
import roslaunch
import rospy
import yaml
from test_fastlio_landing_ros import FastlioTakeoffReferenceTest
from test_landing_memory_ros import LandingMemoryTest


class BridgeLandingSyncTest(FastlioTakeoffReferenceTest):
    def setUp(self):
        base = Path(__file__).resolve().parents[1]
        config = roslaunch.config.load_config_default(
            [(str(base/'launch/control.launch'), ['takeoff_z:=1.5'])], None)
        root = '/UAV1/precision_landing_node/'
        standalone = yaml.safe_load((base.parent/'uav1_inplace_landing_test/config/standalone_18cm_landing.yaml').read_text())
        bridge = yaml.safe_load((base/'config/landing_18cm.yaml').read_text())
        self.assertEqual(bridge, standalone)
        self.assertTrue(config.params[root+'auto_land_at_cutoff'].value)
        self.assertFalse(config.params['/bridge_landing_router/offboard_touchdown'].value)
        self.assertEqual(config.params[root+'topics/odometry'].value, '/UAV1/fast_lio/Odom_high_freq')
        self.assertEqual(config.params[root+'topics/takeoff_reference'].value, '/UAV1/bridge/platform_reference')
        self.assertTrue(config.params['/UAV1/mpc_controller_node/precision_landing/use_waypoint_tracking_weights'].value)
        for key in ('stages/fixed_xy_descent/cutoff_height_m', 'control/use_fastlio_z_for_height'):
            rospy.set_param('/'+self._testMethodName+'/node/'+key, config.params[root+key].value)
        LandingMemoryTest.setUp(self)


if __name__ == '__main__':
    suite = unittest.TestSuite(BridgeLandingSyncTest(name) for name in (
        'test_takeoff_reference_handoff_with_marker_offset',
        'test_takeoff_reference_handoff_without_marker'))
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
