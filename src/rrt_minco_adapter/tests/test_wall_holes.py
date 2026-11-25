import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from wall_holes import find_wall_holes
from cloud_scene import CloudScene


class WallHolesTest(unittest.TestCase):
    def test_enclosed_hole_and_open_gap(self):
        ring = {(20+a, 20, 10+b) for a in (-1, 0, 1) for b in (-1, 0, 1) if a or b}
        self.assertEqual(find_wall_holes(ring, [-1, 4, -1, 4], .1), [(20, 20, 10)])
        self.assertEqual(find_wall_holes(ring-{(19, 20, 10)}, [-1, 4, -1, 4], .1), [])
        self.assertEqual(find_wall_holes(ring, [-1, 4, -1, 4], .1, [(20, 20, 10)]), [])

    def test_floor_and_start_are_excluded(self):
        ring = {(10+a, 10, 10+b) for a in (-1, 0, 1) for b in (-1, 0, 1) if a or b}
        self.assertEqual(find_wall_holes(ring, [-1, 4, -1, 4], .1), [])
        floor = {(20+a, 20+b, 0) for a in (-1, 0, 1) for b in (-1, 0, 1) if a or b}
        self.assertEqual(find_wall_holes(floor, [-1, 4, -1, 4], .1), [])

    def test_repairs_enter_scene_but_respect_manual_deletion(self):
        scene = CloudScene([-1, 4, -1, 4], removed_voxels=[[20, 20, 4]])
        scene.fill_voxels([[20, 20, 4], [21, 20, 4], [10, 10, 4]])
        self.assertEqual(scene.voxels, {(21, 20, 4)})
        self.assertEqual(scene.scene('map')['inferred_wall_voxels'], [(21, 20, 4)])
        self.assertEqual(len(scene.cells), 1)


if __name__ == '__main__':
    unittest.main()
