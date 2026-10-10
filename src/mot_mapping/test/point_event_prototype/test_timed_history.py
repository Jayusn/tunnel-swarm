import unittest

import numpy as np

from event_core import DepthFrame
from timed_history import select_timed_history


class ActualTimedHistory(unittest.TestCase):
    def test_actual_frame_pose_event_and_tuple_identity(self):
        frames = []
        for stamp, event in ((.6, True), (.8, False)):
            origin = np.array([stamp, .02, .03])
            rotation = np.eye(3)
            mask = np.array([event], dtype=bool)
            frame = DepthFrame([[2., stamp, 0.]], rotation, origin, mask)
            frames.append((stamp, frame))
        selected = select_timed_history(1., frames)
        self.assertEqual(len(selected), 2)
        for actual, original in zip(selected, frames):
            self.assertIs(actual, original)
            self.assertIs(actual[1], original[1])
            self.assertIs(actual[1].origin, original[1].origin)
            self.assertIs(actual[1].rotation, original[1].rotation)
            self.assertIs(actual[1].event, original[1].event)
        self.assertTrue(selected[0][1].event[0])
        self.assertFalse(selected[1][1].event[0])

    def test_positive_and_negative_age_tolerance_boundaries(self):
        # The frozen selector compares floating-point age errors directly.
        # Exercise the last representable accepted age and its immediately
        # adjacent rejected age on both sides, without adding another epsilon.
        for target in (.4, .2):
            for sign in (-1, 1):
                with self.subTest(target=target, sign=sign):
                    inside = target+sign*.050001
                    while abs(inside-target) > .050001:
                        inside = np.nextafter(inside, target)
                    outside = np.nextafter(inside, -np.inf if sign < 0 else np.inf)
                    self.assertLessEqual(abs(inside-target), .050001)
                    self.assertGreater(abs(outside-target), .050001)
                    actual = (-inside, object())
                    selected = select_timed_history(0., [actual])
                    self.assertEqual(len(selected), 1)
                    self.assertIs(selected[0], actual)
                    self.assertEqual(select_timed_history(0., [(-outside, object())]), [])

    def test_missing_current_and_future_frames_remain_unavailable(self):
        older, newer = (.6, object()), (.8, object())
        self.assertEqual(select_timed_history(1., []), [])
        self.assertEqual(select_timed_history(1., [older]), [older])
        self.assertEqual(select_timed_history(1., [newer]), [newer])
        self.assertEqual(select_timed_history(1., [(1., object()), (1.1, object()),
                                                  (.95, object()), (.1, object())]), [])

    def test_equal_age_error_selects_newer_actual_frame(self):
        for target in (.4, .2):
            older = (-(target+.03125), object())
            newer = (-(target-.03125), object())
            self.assertEqual(abs(-older[0]-target), abs(-newer[0]-target))
            for history in ([older, newer], [newer, older]):
                selected = select_timed_history(0., history)
                self.assertEqual(len(selected), 1)
                self.assertIs(selected[0], newer)

    def test_unsorted_input_selects_target_age_order(self):
        older, newer = (.6, object()), (.8, object())
        history = [newer, (.99, object()), older, (.5, object())]
        before = list(history)
        selected = select_timed_history(1., history)
        self.assertEqual(selected, [older, newer])
        self.assertEqual(history, before)

    def test_native_epoch_timestamps_select_known_actual_frames(self):
        # Registered-cloud headers from the first eight native input frames.
        # Keep this explicit fixture independent of bags and scratch artifacts.
        headers = [1791429217565721035, 1791429217665592194,
                   1791429217765424967, 1791429217865443468,
                   1791429217965681553, 1791429218065871716,
                   1791429218165584564, 1791429218265455961]
        frames = [(stamp*1e-9, object()) for stamp in headers[:-1]]
        history = [frames[index] for index in (6, 0, 4, 3, 2, 1, 5)]
        selected = select_timed_history(headers[-1]*1e-9, history)
        self.assertEqual(len(selected), 2)
        self.assertIs(selected[0], frames[3])
        self.assertIs(selected[1], frames[5])


if __name__ == '__main__':
    unittest.main()
