"""Exercise the forwarding loop without starting ROS or flight publishers."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


class VisionForwardingTest(unittest.TestCase):
    def run_loop(self, samples, truth=False):
        source = Path(__file__).parents[1] / 'src' / 'laser_mid360.py'
        tree = ast.parse(source.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == 'fastlio')
        published = []
        state = {'index': 0}
        def stopped():
            i = state['index']
            if i >= len(samples):
                return True
            scope['latest_fastlio_odom'] = samples[i]
            return False
        def sleep():
            state['index'] += 1
        last_sample = [None]
        def publish(sample):
            if sample.header.stamp != last_sample[0]:
                published.append(sample)
                last_sample[0] = sample.header.stamp
        scope = dict(rospy=NS(Rate=lambda hz: NS(sleep=sleep), is_shutdown=stopped),
                     latest_fastlio_odom=None, enable_truth_odom=truth,
                     fastlio_sender=NS(publish=publish))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
        scope['fastlio']()
        return published

    def test_original_timestamp_pose_and_no_duplicate(self):
        first = NS(header=NS(stamp=10.0), pose=NS(pose=NS(x=1.0)))
        second = NS(header=NS(stamp=10.02), pose=NS(pose=NS(x=2.0)))
        output = self.run_loop([None, first, first, second])
        self.assertEqual([p.header.stamp for p in output], [10.0, 10.02])
        self.assertIs(output[0], first)
        self.assertIs(output[1], second)

    def test_truth_mode_has_no_second_pose_publisher(self):
        sample = NS(header=NS(stamp=10.0), pose=NS(pose=NS(x=1.0)))
        self.assertEqual(self.run_loop([sample, sample], truth=True), [])


if __name__ == '__main__':
    unittest.main()
