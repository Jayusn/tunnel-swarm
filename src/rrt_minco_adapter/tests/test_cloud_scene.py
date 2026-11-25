import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from cloud_scene import CloudScene


class CloudSceneTests(unittest.TestCase):
    def test_slice_bounds_and_nonfinite_points(self):
        scene = CloudScene([-1, 1, -1, 1], .1)
        scene.add([(-.85, -.85, .3), (-.75, -.85, .7), (-.65, -.85, .299),
                   (-.55, -.85, .701), (2., 0., .5), (float('nan'), 0., .5)])
        self.assertEqual(scene.cells, {(1, 1), (2, 1)})
        result = scene.scene('UAV0/camera_init')
        self.assertEqual(len(result['boxes']), 1)
        self.assertAlmostEqual(result['boxes'][0][2], .2)

    def test_merge_preserves_holes_and_accumulates(self):
        scene = CloudScene([0, 10, 0, 10], 1.)
        points = [(x+.5, y+.5, .5) for x, y in [(0, 0), (1, 0), (0, 1), (1, 1),
                                               (3, 1), (0, 3), (2, 3)]]
        scene.add(points[:3])
        scene.add(points[3:] + points)
        result = scene.scene('map')
        covered = set()
        for cx, cy, sx, sy in result['boxes']:
            for x in range(10):
                for y in range(10):
                    if cx-sx/2 < x+.5 < cx+sx/2 and cy-sy/2 < y+.5 < cy+sy/2:
                        covered.add((x, y))
        self.assertEqual(covered, scene.cells)
        self.assertEqual(len(result['boxes']), 4)

    def test_voxels_retain_height_deduplicate_and_preserve_slice(self):
        scene = CloudScene([0, 5, 0, 5], .1)
        points = [(.55, .55, z) for z in [-.15, .45, 2.15]]
        scene.add(points + points + [(float('inf'), 0., 1.)])
        self.assertEqual(len(scene.voxels), 3)
        self.assertEqual(scene.cells, {(5, 5)})
        centers = scene.voxel_centers()
        for point, expected in zip(centers, points):
            for value, target in zip(point, expected):
                self.assertAlmostEqual(value, target)

    def test_full_voxel_map_keeps_points_outside_planning_region(self):
        scene = CloudScene([0, 5, 0, 5], .1)
        points = [(-2.05, .55, .45), (6.05, .55, .45),
                  (.55, -3.05, .45), (.55, 7.05, .45), (.55, .55, .45)]
        scene.add(points)
        self.assertEqual(len(scene.voxels), 5)
        self.assertEqual(scene.cells, {(5, 5)})
        for actual, expected in zip(scene.voxel_centers(), sorted(points)):
            for value, target in zip(actual, expected):
                self.assertAlmostEqual(value, target)
        self.assertEqual(scene.scene('map')['bounds'], [0, 5, 0, 5])
        width, height, data = scene.occupancy_grid()
        self.assertEqual((width, height), (50, 50))
        self.assertEqual(data.count(100), 1)

    def test_fixed_start_filter_and_height_limits(self):
        scene = CloudScene([-1, 4, -1, 4], start_clear_radius=.4)
        scene.add([(.01, .01, .5), (-.2, -.05, .4), (.01, .01, .3),
                   (.01, .01, .7), (.01, .01, .2), (.01, .01, .8),
                   (.5, .01, .5), (3., 3., .5)])
        self.assertEqual(len(scene.voxels), 4)
        self.assertEqual(len(scene.cells), 2)
        self.assertEqual(scene.scene('map')['operator_clear_region']['center_xy'], [.01, .01])

    def test_filter_does_not_erase_outside_hit_in_shared_voxel(self):
        scene = CloudScene([-1, 4, -1, 4], start_clear_radius=.4)
        scene.add([(.405, .01, .5), (.42, .01, .5)])
        self.assertEqual(len(scene.cells), 1)
        self.assertEqual(len(scene.voxels), 1)

    def test_start_filter_disabled_by_default(self):
        scene = CloudScene([-1, 4, -1, 4])
        scene.add([(.01, .01, .5)])
        self.assertEqual(len(scene.cells), 1)
        self.assertNotIn('operator_clear_region', scene.scene('map'))
        with self.assertRaises(ValueError):
            CloudScene([-1, 4, -1, 4], start_clear_radius=float('nan'))

    def test_invalid_extent(self):
        for bounds in [[0, 0, 0, 1], [0, 1, 0, float('nan')]]:
            with self.assertRaises(ValueError):
                CloudScene(bounds)

    def test_explicit_voxel_removal_preserves_neighbors_and_records_edit(self):
        scene = CloudScene([-1, 4, -1, 4], removed_voxels=[[10, 10, 1]])
        points = [(.025, .025, .125), (.075, .075, .175),
                  (.125, .025, .125), (.025, .025, -.025), (.025, .025, .225)]
        scene.add(points)
        self.assertEqual(len(scene.voxels), 3)
        self.assertTrue(scene.is_removed(points[0]))
        self.assertFalse(scene.is_removed(points[-1]))
        self.assertEqual(scene.scene('map')['operator_removed_voxels'], [(10, 10, 1)])
        with self.assertRaises(ValueError):
            CloudScene([-1, 4, -1, 4], removed_voxels=[[10.5, 10, 1]])

    def test_grid_origin_row_order_and_unknown_cells(self):
        scene = CloudScene([-1, 1, 2, 3], .5)
        scene.add([(-.75, 2.25, .5), (.75, 2.75, .5)])
        width, height, data = scene.occupancy_grid()
        self.assertEqual((width, height), (4, 2))
        self.assertEqual(data, [100, -1, -1, -1, -1, -1, -1, 100])

    def test_grid_partial_edge_cell(self):
        scene = CloudScene([0, 1.05, 0, .25], .1)
        scene.add([(1.04, .24, .5)])
        width, height, data = scene.occupancy_grid()
        self.assertEqual((width, height), (11, 3))
        self.assertEqual(data[-1], 100)
        self.assertEqual(data.count(100), 1)


if __name__ == '__main__':
    unittest.main()
