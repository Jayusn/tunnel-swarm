#!/usr/bin/env python3

import subprocess
import unittest

import rospkg
import yaml


class GlobalRetentionLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        package_path = rospkg.RosPack().get_path("diff_planner")
        cls.launch_file = (
            package_path + "/launch/exp/run_swarm_indoor1_fuel_exploration.launch"
        )

    def dump_params(self, *launch_args):
        output = subprocess.check_output(
            ["roslaunch", "--dump-params", self.launch_file, *launch_args],
            text=True,
        )
        return yaml.safe_load(output)

    def test_global_static_cloud_retention_is_disabled_by_default(self):
        params = self.dump_params()
        self.assertIs(
            params["/exploration_node/sdf_map/static_retention_enabled"], False
        )

    def test_global_static_cloud_retention_can_be_enabled_explicitly(self):
        params = self.dump_params("retain_global_static_cloud:=true")
        self.assertIs(
            params["/exploration_node/sdf_map/static_retention_enabled"], True
        )

    def test_global_static_cloud_retention_can_be_disabled_explicitly(self):
        params = self.dump_params("retain_global_static_cloud:=false")
        self.assertIs(
            params["/exploration_node/sdf_map/static_retention_enabled"], False
        )

    def test_vertical_detour_uses_short_ten_centimeter_low_probe(self):
        params = self.dump_params()
        self.assertAlmostEqual(
            params["/exploration_node/mission/task_search/min_height"], 0.10
        )
        self.assertAlmostEqual(
            params["/exploration_node/astar/min_search_height"], 0.10
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/vertical_detour/low_height"], 0.10
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/vertical_detour/forward_check_distance"], 0.20
        )
        self.assertEqual(
            params["/exploration_node/mission/vertical_detour/required_confirmations"], 1
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/vertical_detour/footprint_radius"], 0.17
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/vertical_detour/upper_forward_max_distance"],
            0.60,
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/vertical_detour/upper_max_side_angle_deg"],
            45.0,
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/vertical_detour/down_first_height"],
            0.75,
        )

    def test_core_footprint_ignores_single_occupied_voxel(self):
        params = self.dump_params()
        self.assertAlmostEqual(
            params["/exploration_node/manager/footprint_check_radius"], 0.18
        )
        self.assertEqual(
            params["/exploration_node/manager/footprint_min_occupied_support"], 2
        )
        self.assertIs(
            params[
                "/exploration_node/manager/supported_occupancy_hard_reject_enabled"
            ],
            True,
        )

    def test_frontier_clearance_reward_uses_known_thirty_to_sixty_centimeters(self):
        params = self.dump_params()
        self.assertIs(
            params["/exploration_node/mission/task_search/clearance_reward_enabled"],
            True,
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/task_search/clearance_reward_start"],
            0.30,
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/task_search/clearance_reward_full"],
            0.60,
        )
        self.assertAlmostEqual(
            params["/exploration_node/mission/task_search/clearance_reward_max"], 5.0
        )


if __name__ == "__main__":
    unittest.main()
