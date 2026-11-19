import copy
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from plan_super import Polynomial, validate_and_scale


class SuperPolynomialTests(unittest.TestCase):
    def setUp(self):
        self.result = dict(order=7, duration=[1.],
            coef_x=[-20., 70., -84., 35., 0., 0., 0., .01],
            coef_y=[0.]*8, coef_z=[0.]*7+[.5])
        self.scene = dict(bounds=[-1., 4., -1., 4.], boxes=[], boundary_clearance=0.)

    def validate(self, result=None):
        return validate_and_scale(self.result if result is None else result, self.scene,
                                  [.01, 0.], [1.01, 0.], .5, .2, .8, .3)

    def test_time_scaling_preserves_geometry_and_limits(self):
        native = Polynomial(copy.deepcopy(self.result))
        scaled = self.validate()
        self.assertGreater(scaled.total_time, 1.)
        for u in np.linspace(0, 1, 101):
            np.testing.assert_allclose(native.sample(u)[0], scaled.sample(u*scaled.total_time)[0], atol=1e-10)
            self.assertLessEqual(np.linalg.norm(scaled.derivative(u*scaled.total_time, 1)), .200001)
            self.assertLessEqual(np.linalg.norm(scaled.derivative(u*scaled.total_time, 2)), .800001)

    def test_collision_rejected(self):
        self.scene['boxes'] = [[.5, 0., .1, .1]]
        with self.assertRaisesRegex(ValueError, 'clearance'):
            self.validate()

    def test_height_rejected(self):
        self.result['coef_z'][-2] = .1
        with self.assertRaisesRegex(ValueError, 'height'):
            self.validate()

    def test_bounds_rejected(self):
        self.scene['bounds'][1] = .9
        with self.assertRaisesRegex(ValueError, 'bounds'):
            self.validate()

    def test_invalid_duration(self):
        self.result['duration'] = [0.]
        with self.assertRaises(ValueError):
            self.validate()


if __name__ == '__main__': unittest.main()
