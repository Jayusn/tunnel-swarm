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

    def test_invalid_extent(self):
        for bounds in [[0, 0, 0, 1], [0, 1, 0, float('nan')]]:
            with self.assertRaises(ValueError):
                CloudScene(bounds)


if __name__ == '__main__':
    unittest.main()
