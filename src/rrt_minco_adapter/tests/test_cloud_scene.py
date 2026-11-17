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
        scene.add(points + points + [(float('inf'), 0., 1.), (5., .5, 1.)])
        self.assertEqual(len(scene.voxels), 3)
        self.assertEqual(scene.cells, {(5, 5)})
        centers = scene.voxel_centers()
        for point, expected in zip(centers, points):
            for value, target in zip(point, expected):
                self.assertAlmostEqual(value, target)

    def test_invalid_extent(self):
        for bounds in [[0, 0, 0, 1], [0, 1, 0, float('nan')]]:
            with self.assertRaises(ValueError):
                CloudScene(bounds)

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
