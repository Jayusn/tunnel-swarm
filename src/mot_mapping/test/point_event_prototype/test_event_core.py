import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from event_core import DepthFrame, EventDetector, point_events


def frame(x, ys=(0.,), **kwargs):
    return DepthFrame(np.array([[x, y, 0.] for y in ys]), **kwargs)


class OcclusionEvents(unittest.TestCase):
    def test_crossing_two_real_background_returns(self):
        kind, _ = point_events([[1., 0., 0.]], frame(2.), frame(2.))
        self.assertEqual(int(kind[0]), 1)

    def test_radial_toward_has_recursive_real_ancestors(self):
        kind, _ = point_events([[1.6, 0., 0.]], frame(1.8), frame(2.))
        self.assertEqual(int(kind[0]), 3)

    def test_radial_away_requires_recursive_real_ancestors(self):
        kind, _ = point_events([[1.6, 0., 0.]], frame(1.4), frame(1.2))
        self.assertEqual(int(kind[0]), 4)
        kind, _ = point_events([[1.6, 0., 0.]], frame(1.4), frame(1.4))
        self.assertEqual(int(kind[0]), 0)

    def test_no_return_or_view_is_unknown(self):
        for newest, older in [(frame(1., (.2,)), frame(2.)), (frame(2.), DepthFrame([])), (DepthFrame([]), frame(2.))]:
            kind, diagnostic = point_events([[1., 0., 0.]], newest, older)
            self.assertEqual(int(kind[0]), 0)
            self.assertTrue(diagnostic['unknown'][0])

    def test_near_object_occlusion_is_unknown_without_motion_chain(self):
        kind, diagnostic = point_events([[3., 0., 0.]], frame(2.), frame(2.))
        self.assertEqual(int(kind[0]), 0)
        self.assertTrue(diagnostic['unknown'][0])

    def test_depth_layers_do_not_fill_an_empty_interval(self):
        layers = DepthFrame([[2., 0., 0.], [4., 0., 0.]])
        q = layers.query([[3., 0., 0.]])
        self.assertFalse(q.static[0])
        self.assertFalse(q.front[0])
        q = layers.query([[4., 0., 0.]])
        self.assertFalse(q.static[0])
        self.assertTrue(q.unknown[0])
        kind, diagnostic = point_events([[3., 0., 0.]], layers, layers)
        self.assertEqual(int(kind[0]), 0)
        self.assertTrue(diagnostic['unknown'][0])

    def test_static_true_return_blocks_event(self):
        layers = DepthFrame([[1., 0., 0.], [2., 0., 0.]])
        kind, diagnostic = point_events([[1., 0., 0.]], layers, frame(2.))
        self.assertEqual(int(kind[0]), 0)
        self.assertTrue(diagnostic['static'][0])

    def test_sensor_rotation_translation_and_wrap(self):
        rotation = Rotation.from_euler('z', 175, degrees=True).as_matrix()
        origin = np.array([.5, .3, -.1])
        old = DepthFrame([[2., 0., 0.]], rotation, origin)
        older = DepthFrame([[2., 0., 0.]], np.eye(3), [0., 0., 0.])
        kind, _ = point_events([[1., 0., 0.]], old, older)
        # Translation changes the ray, so this differs from a sensor fixed at
        # the world origin: a missing matching historical ray stays unknown.
        self.assertEqual(int(kind[0]), 0)
        old = DepthFrame([[-2., -.0001, 0.]], rotation, [0., 0., 0.])
        older = DepthFrame([[-2., .0001, 0.]])
        kind, _ = point_events([[-1., 0., 0.]], old, older)
        self.assertEqual(int(kind[0]), 1)

    def test_six_points_have_no_physical_size_filter(self):
        # Six points spanning a 20 cm flat target; only 2 cm separation in
        # the sparse visible patch. No 20 cm bbox threshold is introduced.
        patch = np.array([[1., y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        wall = np.array([[2., 2*y, 0.] for y in patch[:, 1]])
        detector = EventDetector(capture=0)
        detector.step(0., wall, np.eye(3), np.zeros(3))
        a = detector.step(.1, patch, np.eye(3), np.zeros(3))
        b = detector.step(.2, patch + [.1, 0., 0.], np.eye(3), np.zeros(3))
        c = detector.step(.3, patch + [.2, 0., 0.], np.eye(3), np.zeros(3))
        self.assertTrue(a['candidates'])
        self.assertTrue(b['candidates'])
        self.assertTrue(c['published'])
        self.assertEqual(len(c['published'][0]['indices']), 6)

    def test_gaps_and_capture_reset_actual_observations(self):
        detector = EventDetector(capture=.5)
        cloud = np.array([[1., y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        for t in [0., .1, .2, .3, .4]:
            row = detector.step(t, cloud + [t, 0., 0.], np.eye(3), np.zeros(3))
            self.assertFalse(row['published'])
        row = detector.step(.5, cloud + [.5, 0., 0.], np.eye(3), np.zeros(3))
        self.assertFalse(row['point_chain_valid'])
        self.assertTrue(all(d['observations'] == 1 for d in row['decisions']))
        row = detector.step(1.5, cloud + [.6, 0., 0.], np.eye(3), np.zeros(3))
        self.assertFalse(row['point_chain_valid'])
        self.assertFalse(row['published'])

    def test_20cm_six_point_full_plane_and_sphere_geometry(self):
        plane = np.array([[.1*np.cos(a), -.6, 1+.1*np.sin(a)] for a in np.arange(6)*np.pi/3])
        sphere = np.array([[x, y, 1+z] for x, y, z in
                           [(.1, 0, 0), (.06, .08, 0), (.06, -.08, 0),
                            (.06, 0, .08), (.06, 0, -.08), (0, .0707106781, .0707106781)]])
        # Dense, actual rear returns establish geometric foreground support;
        # the algorithm sees no nominal diameter or target class.
        wall = np.array([[3., y, z] for y in np.arange(-2., 2.01, .02) for z in np.arange(.2, 2.01, .02)])
        from event_core import candidates
        for shape in [plane, sphere]:
            cloud = shape + [1., 0., 0.]
            kind, _ = point_events(cloud, DepthFrame(wall, origin=[0, 0, 1]), DepthFrame(wall, origin=[0, 0, 1]))
            candidate = candidates(cloud, kind, np.zeros(6, dtype=bool), [])
            self.assertEqual(len(candidate), 1)
            self.assertEqual(len(candidate[0]['indices']), 6)
            self.assertGreater(candidate[0]['event_fraction'], .26)

    def test_novelty_requires_both_histories(self):
        from event_core import candidates
        patch = np.array([[1., y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        candidate = candidates(patch, np.ones(6, dtype=np.uint8), np.zeros(6, dtype=bool),
                               [DepthFrame(patch), DepthFrame(patch + [1., 0., 0.])])
        self.assertEqual(candidate[0]['novelty'], 0.)

    def test_measured_rate_preserves_reversal_path_length(self):
        detector = EventDetector(capture=0)
        patch = np.array([[1., y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        detector.step(0., patch, np.eye(3), np.zeros(3))
        detector.step(.1, patch + [.12, 0., 0.], np.eye(3), np.zeros(3))
        # The oldest real points were already moving; a return to their old
        # location cannot provide stationary-map evidence. This only checks
        # real path-rate semantics; no point event or publication is forced.
        detector.history[0][1].event[:] = True
        row = detector.step(.2, patch, np.eye(3), np.zeros(3))
        self.assertEqual(row['decisions'][0]['observations'], 3)
        self.assertAlmostEqual(row['decisions'][0]['speed'], 1.2)
        self.assertFalse(row['published'])

    def test_candidate_roi_preserves_full_ray_cloud_and_crossing_height(self):
        patch = np.array([[5., y, 1.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        detector = EventDetector(capture=0)
        row = detector.step(0., patch, np.eye(3), np.zeros(3), np.zeros(6, dtype=bool), 1.5)
        self.assertFalse(row['candidates'])
        self.assertEqual(len(detector.history[-1][1].points), 6)
        self.assertIsNone(detector.history[-1][1].world_tree)
        vertical = np.array([[1., 0., z] for z in [1.4, 1.45, 1.5, 1.55, 1.6, 1.65]])
        row = EventDetector(capture=0).step(0., vertical, np.eye(3), np.zeros(3), np.ones(6, dtype=bool), 1.5)
        self.assertEqual(len(row['candidates']), 1)
        row = EventDetector(capture=0).step(0., vertical + [0, 0, .2], np.eye(3), np.zeros(3), np.ones(6, dtype=bool), 1.5)
        self.assertFalse(row['candidates'])


if __name__ == '__main__':
    unittest.main()
