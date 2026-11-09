#!/usr/bin/env python3
"""Parse production launches without starting sensors, controllers or a ROS master."""
import os
from pathlib import Path
import subprocess
import unittest

import roslaunch
import roslib.packages
import rospkg
import yaml

ROOT = Path(__file__).resolve().parents[2]
MPC = ROOT / 'src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller'


def load(relative, arguments=()):
    config = roslaunch.ROSLaunchConfig()
    roslaunch.XmlLoader().load(str(ROOT / relative), config, argv=list(arguments), verbose=False)
    return config


def flatten(value, prefix=''):
    for key, item in value.items():
        name = prefix + '/' + key
        if isinstance(item, dict):
            yield from flatten(item, name)
        else:
            yield name, item


class RoleConfigurationTest(unittest.TestCase):
    def test_all_role_launches_resolve_nodes_and_sensor_frames(self):
        packages = rospkg.RosPack()
        cases = [
            ('uav0_competition_bringup', 'launch/uav1_detection_stack.launch', []),
            ('exploration_control', 'launch/uav1_relay_visualization.launch', []),
            ('exploration_control', 'launch/leader_safe_path_follower.launch', [
                'enable_diff_rviz:=false', 'fapp_verified_preset:=true',
                'object_motion_evidence_enabled:=true', 'use_nmpc_landing:=true',
                'fapp_planner_priority:=true']),
            ('autotrans_reference_bridge', 'launch/uav1_diff_autotrans.launch', [
                'setpoint_topic:=/UAV1/control/attitude_setpoint',
                'position_setpoint_topic:=/UAV1/control/position_setpoint',
                'precision_landing_enabled:=true']),
            ('autotrans_reference_bridge', 'uav0/launch/uav0_autotrans_controller.launch', [
                'pure_fuel_mode:=true', 'precision_landing_mission_mode:=true']),
            ('diff_planner', 'uav0/launch/exp/run_swarm_indoor1_fuel_exploration.launch', [
                'enable_fapp_detection:=true', 'dynamic_mode:=swing-only',
                'enable_fapp_dynamic_avoidance:=true', 'fapp_planner_priority:=true']),
            ('uav0_competition_bringup', 'uav0/launch/uav0_detection_landing_stack.launch', [
                'enable_color_tag:=false', 'enable_qr:=false', 'enable_thermal:=false',
                'enable_thermal_d435_fusion:=false', 'enable_target_reporting:=false',
                'enable_down_camera:=true', 'enable_precision_landing:=true',
                'landing_via_mpc:=true']),
        ]
        for role in ('UAV0', 'UAV1'):
            cases.extend([
                ('cxr_ego_ctrl', 'launch/mavros_uav.launch', ['vehicle_ns:=' + role]),
                ('livox_ros_driver2', 'launch_ROS1/msg_MID360.launch', [
                    'vehicle_ns:=' + role, 'msg_frame_id:=' + role + '/livox_frame']),
                ('fast_lio', 'launch/mapping_mid360.launch', ['vehicle_ns:=' + role]),
            ])
        for package, launch, args in cases:
            with self.subTest(package=package, launch=launch, args=args):
                config = load(Path(packages.get_path(package)) / launch, args)
                for node in config.nodes:
                    self.assertTrue(roslib.packages.find_node(node.package, node.type),
                                    (node.package, node.type))
                if package == 'fast_lio':
                    role = args[0].split(':=')[1]
                    prefix = '/' + role
                    self.assertEqual(config.params[prefix + '/publish/world_frame'].value,
                                     role + '/camera_init')
                    self.assertEqual(config.params[prefix + '/publish/body_frame'].value,
                                     role + '/body')
                    self.assertEqual(config.params[prefix + '/common/lid_topic'].value,
                                     prefix + '/livox/lidar')

    def test_front_and_rear_load_separate_mpc_parameters(self):
        rear_before = load('src/autotrans_reference_bridge/launch/uav1_diff_autotrans.launch')
        front = load('src/autotrans_reference_bridge/uav0/launch/uav0_autotrans_controller.launch',
                     ['pure_fuel_mode:=true', 'precision_landing_mission_mode:=true'])
        rear = load('src/autotrans_reference_bridge/launch/uav1_diff_autotrans.launch')
        self.assertEqual({k: p.value for k, p in rear_before.params.items()},
                         {k: p.value for k, p in rear.params.items()})
        for config, prefix, file in [
                (front, '/mpc_controller_node', MPC / 'uav0/config/mpc.yaml'),
                (rear, '/UAV1/mpc_controller_node', MPC / 'config/mpc.yaml')]:
            expected = yaml.safe_load(file.read_text())
            # These are loaded directly from each role's YAML, with no tuning overlay.
            for name in ('Q_velocity', 'R_pitchroll', 'max_bodyrate_z'):
                self.assertEqual(config.params[prefix + '/' + name].value, expected[name])
            for name in ('force_estimator/kf', 'thrust_model/hover_percentage', 'force_estimator/max_queue'):
                section, key = name.split('/')
                self.assertEqual(config.params[prefix + '/' + name].value, expected[section][key])
            model = yaml.safe_load((MPC / 'config/model.yaml').read_text())
            self.assertEqual(config.params[prefix + '/mass_q'].value, model['mass_q'])
        self.assertIn('mpc_controller_node_uav0', [n.type for n in front.nodes])
        self.assertIn('fuel_autotrans_bridge_node_uav0', [n.type for n in front.nodes])
        self.assertIn('mpc_controller_node', [n.type for n in rear.nodes])
        self.assertFalse(any(n.type.endswith('_uav0') for n in rear.nodes))
        controller = next(n for n in front.nodes if n.type == 'mpc_controller_node_uav0')
        remaps = dict(controller.remap_args)
        self.assertEqual(remaps['~traj'], '/UAV0/planning/autotrans_trajectory')
        self.assertEqual(remaps['/mavros/setpoint_raw/attitude'],
                         '/UAV0/control/attitude_setpoint')
        self.assertEqual(remaps['~recovery_position_setpoint'], '/UAV0/control/position_setpoint')

    def test_front_task_and_landing_use_front_targets_and_local_calibration(self):
        planner = load('src/Diff-Planner/src/diff_planner/plan_manage/uav0/launch/exp/run_swarm_indoor1_fuel_exploration.launch')
        types = {n.type for n in planner.nodes}
        self.assertTrue({'exploration_node_uav0', 'diff_planner_node_uav0',
                         'landing_diff_search_manager_uav0', 'mapping_node_uav0'} <= types)
        detection = load('src/uav0_competition_bringup/uav0/launch/uav0_detection_landing_stack.launch', [
            'enable_color_tag:=false', 'enable_qr:=false', 'enable_thermal:=false',
            'enable_target_reporting:=false', 'enable_down_camera:=true',
            'enable_precision_landing:=true', 'landing_via_mpc:=true'])
        self.assertIn('precision_landing_node_uav0', [n.type for n in detection.nodes])
        self.assertIn('dual_uav_landing_coordinator_node_uav0', [n.type for n in detection.nodes])
        arbiter = '/UAV0/landing_setpoint_arbiter/'
        for key, value in {
                'controller_attitude_input': '/UAV0/control/attitude_setpoint',
                'controller_position_input': '/UAV0/control/position_setpoint',
                'mavros_attitude_output': '/UAV0/mavros/setpoint_raw/attitude',
                'mavros_position_output': '/UAV0/mavros/setpoint_raw/local',
                'landing_mpc_output': '/UAV0/landing/nmpc_target'}.items():
            self.assertEqual(detection.params[arbiter + key].value, value)
        calibration = yaml.safe_load((ROOT / 'src/precision_landing/config/down_camera_extrinsic_uav1.yaml').read_text())
        for node in ('landing_search_node', 'precision_landing_node'):
            for name, value in flatten(calibration):
                self.assertEqual(detection.params['/UAV0/' + node + name].value, value)
        offset = yaml.safe_load((ROOT / 'src/control/config/center_offset_uav1.yaml').read_text())
        for name, value in flatten(offset):
            self.assertEqual(detection.params['/UAV0/precision_landing_node' + name].value, value)

    def test_front_help_uses_local_network_without_changing_rear_defaults(self):
        env = dict(os.environ)
        for key in ('DUAL_UAV_ROS_MASTER_IP', 'UAV0_ROS_IP', 'UAV1_ROS_IP',
                    'UAV0_LOCAL_MASTER_IP', 'UAV0_LOCAL_ROS_IP'):
            env.pop(key, None)
        result = subprocess.run(['bash', str(ROOT / 'shfiles/start_uav0_dual_network_six_terminator.sh'), '--help'],
                                env=env, text=True, capture_output=True, check=True)
        self.assertIn('master=http://192.168.43.21:11311 local_ip=192.168.43.21', result.stdout)
        result = subprocess.run(['bash', '-c', 'source "$1"; configure_dual_uav_ros_network uav1',
                                 'test', str(ROOT / 'shfiles/configure_dual_uav_ros_network.sh')],
                                env=env, text=True, capture_output=True, check=True)
        self.assertIn('master=http://10.59.40.213:11311 local_ip=10.59.40.233', result.stdout)


if __name__ == '__main__':
    unittest.main()
