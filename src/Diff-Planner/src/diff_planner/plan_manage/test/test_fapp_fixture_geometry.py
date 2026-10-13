#!/usr/bin/env python3
"""Check independent smoke-oracle geometry and fixture passage feasibility."""
import math
import unittest

from fapp_suspended_obstacle_v1_smoke import (MARGIN, box_bounds,
                                              cylinder_box_gap, fixtures,
                                              object_gap, physical_box_gap,
                                              scenario_boxes, scenario_corridor, static_gap)


class FixtureGeometryTest(unittest.TestCase):
    def test_cylinder_box_separation_and_corner(self):
        box = ((-.1, .1), (-.1, .1), (-.1, .1))
        self.assertAlmostEqual(cylinder_box_gap((.5, 0., 0.), box), .138)
        self.assertAlmostEqual(cylinder_box_gap((0., 0., .5), box), .275)
        corner = cylinder_box_gap((.5, .5, .5), box)
        self.assertAlmostEqual(corner, math.hypot(math.hypot(.4, .4) - .262, .275))
        self.assertLess(cylinder_box_gap((0., 0., 0.), box), 0.)

    def test_box_corner_contact_and_cylinder_encloses_body(self):
        box = ((-.1, .1), (-.1, .1), (-.1, .1))
        self.assertAlmostEqual(physical_box_gap((.285, 0., 0.), box), 0.)
        for position in ((.285, 0., 0.), (.3, .3, 0.), (.1, .1, .3)):
            self.assertLessEqual(cylinder_box_gap(position, box), physical_box_gap(position, box))

    def test_underpass_fixture_also_has_real_left_passage(self):
        center, _, size, _, _ = fixtures('underpass', 0., 0.)[0]
        box = box_bounds(center, size)
        for point in ((2., 0., .33), (2., -.30, .85)):
            self.assertGreaterEqual(cylinder_box_gap(point, box), MARGIN)
            self.assertGreaterEqual(static_gap(point, ()), MARGIN)

    def test_unique_side_passages_and_blocked_lower_gap(self):
        for scenario, side in (('left', .30), ('right', -.30)):
            center, _, size, _, _ = fixtures(scenario, 0., 0.)[0]
            box = box_bounds(center, size)
            point = (2., side, .85)
            self.assertGreaterEqual(cylinder_box_gap(point, box), MARGIN)
            self.assertGreaterEqual(static_gap(point, scenario_boxes(scenario)), MARGIN)
            self.assertLess(static_gap((2., 0., .33), scenario_boxes(scenario)), 0.)

    def test_comparison_fixture_has_two_real_passages(self):
        center, _, size, _, _ = fixtures('both_sides_cost', 0., 0.)[0]
        box = box_bounds(center, size)
        corridor = scenario_corridor('both_sides_cost')
        for point in ((2., -.45, .85), (2., .62, .85)):
            self.assertGreaterEqual(cylinder_box_gap(point, box), MARGIN)
            self.assertGreaterEqual(static_gap(point, scenario_boxes('both_sides_cost'), corridor=corridor), MARGIN)
        self.assertLess(static_gap((2., 0., .33), scenario_boxes('both_sides_cost'), corridor=corridor), 0.)

    def test_center_free_can_hide_full_body_wall_and_lower_contact(self):
        # Point center lies inside the corridor but the cylinder clips a wall.
        self.assertGreater(.75 - .55, 0.)
        self.assertLess(static_gap((2., .55, .85), ()), 0.)
        platform = (((1.5, 2.5), (-.75, .75), (0., .4)),)
        self.assertGreater(.5 - .4, 0.)
        self.assertLess(static_gap((2., 0., .5), platform), 0.)

    def test_invalid_size_uses_original_ball_and_thin_box_remains_valid(self):
        point, center = (.4, 0., 0.), (0., 0., 0.)
        self.assertAlmostEqual(object_gap(point, center, (-.1, .2, .2)), .038)
        self.assertAlmostEqual(object_gap(point, center, (0., .2, .2)), .138)


if __name__ == '__main__':
    unittest.main()
