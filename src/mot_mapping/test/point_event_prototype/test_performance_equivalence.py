"""Semantic checks against the pre-optimization point-event operations.

The small references preserve the original reduction, clustering and pairing
rules. No recording, annotation, ROS node or Git command enters these tests.
"""
from collections import deque
import unittest
from unittest.mock import patch

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import event_core
from event_core import DEPTH, LATERAL, REGION_RADIUS, DepthFrame, EventDetector


def reference_query(frame, points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    size = len(points)
    front = np.zeros(size, dtype=bool)
    behind, static = front.copy(), front.copy()
    unknown = np.ones(size, dtype=bool)
    nearest = np.full(size, -1, dtype=int)
    if frame.tree is None or not size:
        return event_core.Query(front, behind, static, unknown, nearest)
    rays = (points - frame.origin) @ frame.rotation
    ranges = np.linalg.norm(rays, axis=1)
    valid = np.isfinite(rays).all(axis=1) & (ranges > 0)
    rows = np.flatnonzero(valid)
    if not len(rows):
        return event_core.Query(front, behind, static, unknown, nearest)
    directions = rays[valid] / ranges[valid, None]
    angles = np.arcsin(np.minimum(1., LATERAL / ranges[valid]))
    neighborhoods = frame.tree.query_ball_point(directions, 2*np.sin(angles/2), workers=1)
    counts = np.fromiter((len(x) for x in neighborhoods), dtype=int)
    if not counts.sum():
        return event_core.Query(front, behind, static, unknown, nearest)
    query_index = np.repeat(rows, counts)
    sample_index = np.concatenate(neighborhoods).astype(int)
    difference = frame.ranges[sample_index] - ranges[query_index]
    exact = np.abs(difference) <= DEPTH
    exact_any = np.zeros(size, dtype=bool)
    np.logical_or.at(exact_any, query_index, exact)
    np.logical_or.at(static, query_index, exact & ~frame.event[frame.indices[sample_index]])
    minimum = np.full(size, np.inf)
    np.minimum.at(minimum, query_index, frame.ranges[sample_index])
    static &= ~(minimum < ranges - DEPTH)
    first = frame.ranges[sample_index] == minimum[query_index]
    nearest[query_index[first]] = frame.indices[sample_index[first]]
    observed = np.isfinite(minimum)
    front = observed & ~exact_any & (minimum > ranges + DEPTH)
    behind = observed & ~exact_any & (minimum < ranges - DEPTH)
    unknown = ~(static | front | behind)
    return event_core.Query(front, behind, static, unknown, nearest)


def reference_events(points, newest, older):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    q1, q2 = reference_query(newest, points), reference_query(older, points)
    rows = np.flatnonzero(q1.nearest >= 0)
    chain_front = np.zeros(len(points), dtype=bool)
    chain_behind, chain_static = chain_front.copy(), chain_front.copy()
    if len(rows):
        ancestors = newest.points[q1.nearest[rows]]
        query = reference_query(older, ancestors)
        chain_front[rows], chain_behind[rows], chain_static[rows] = query.front, query.behind, query.static
    stationary = q1.static | q2.static
    crossing = q1.front & q2.front & ~stationary
    toward = crossing & chain_front & ~chain_static
    away = q1.behind & q2.behind & chain_behind & ~chain_static & ~stationary
    kinds = crossing.astype(np.uint8) | (toward.astype(np.uint8) << 1) | (away.astype(np.uint8) << 2)
    return kinds, dict(front=q1.front, behind=q1.behind, static=stationary,
                       unknown=~(stationary | crossing | toward | away))


def reference_candidates(points, kinds, static, history, minimum=6, maximum=800, eligible=None):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    indices = np.flatnonzero(~static if eligible is None else (~static & eligible))
    if not len(indices):
        return []
    local = points[indices]
    tree = cKDTree(local)
    neighbors = tree.query_ball_point(local, REGION_RADIUS, workers=1)
    core = np.array([len(n) >= 3 for n in neighbors])
    core_indices = np.flatnonzero(core)
    if not len(core_indices):
        return []
    lookup = np.full(len(local), -1, dtype=int)
    lookup[core_indices] = np.arange(len(core_indices))
    pairs = tree.query_pairs(REGION_RADIUS, output_type='ndarray')
    pairs = pairs[core[pairs[:, 0]] & core[pairs[:, 1]]] if len(pairs) else pairs
    graph = coo_matrix((np.ones(len(pairs)), (lookup[pairs[:, 0]], lookup[pairs[:, 1]])),
                       shape=(len(core_indices), len(core_indices))).tocsr()
    _, core_groups = connected_components(graph, directed=False)
    groups = np.full(len(local), -1, dtype=int)
    groups[core_indices] = core_groups
    for index in np.flatnonzero(~core):
        adjacent = [n for n in neighbors[index] if core[n]]
        if adjacent:
            groups[index] = groups[adjacent[0]]
    output = []
    for group in range(int(groups.max()) + 1):
        member = indices[groups == group]
        if not minimum <= len(member) <= maximum:
            continue
        observed = points[member]
        low, high = observed.min(axis=0), observed.max(axis=0)
        novelty = []
        for frame in history:
            if frame.world_tree is not None:
                distances, _ = frame.world_tree.query(observed, k=1, workers=1)
                novelty.append(float(np.mean(distances > .06)))
        output.append(dict(indices=member, center=(low+high)/2, size=high-low,
                           event_fraction=float(np.mean(kinds[member] != 0)),
                           event_counts=[int(np.sum((kinds[member] & bit) != 0)) for bit in (1, 2, 4)],
                           novelty=min(novelty) if len(novelty) == 2 else 0.))
    return output


def reference_pairing(candidate, tracks, sequence):
    edges = []
    for ci, value in enumerate(candidate):
        for ti, track in enumerate(tracks):
            distance = np.linalg.norm(value['center'] - track['observations'][-1][1])
            if distance <= .82:
                edges.append((track['sequence'] != sequence - 1, distance, ci, ti))
    assigned_candidate, assigned_track, matches = set(), set(), {}
    for _, _, ci, ti in sorted(edges):
        if ci not in assigned_candidate and ti not in assigned_track:
            assigned_candidate.add(ci)
            assigned_track.add(ti)
            matches[ci] = tracks[ti]['id']
    return matches


class PerformanceEquivalence(unittest.TestCase):
    def assert_query_equal(self, frame, points):
        expected, actual = reference_query(frame, points), frame.query(points)
        for name in ['front', 'behind', 'static', 'unknown', 'nearest']:
            np.testing.assert_array_equal(getattr(actual, name), getattr(expected, name), err_msg=name)

    def test_empty_neighborhoods_and_nonfinite_queries(self):
        with np.errstate(invalid='ignore'):
            for frame in [DepthFrame([]), DepthFrame([[2., 0., 0.]])]:
                for points in [[], [[0., 0., 0.], [np.nan, 1., 0.], [np.inf, 0., 0.]],
                               [[1., 0., 0.], [1., 1., 0.], [3., 0., 0.]]]:
                    self.assert_query_equal(frame, points)

    def test_nearest_equal_depth_tie_retains_last_original_index(self):
        frame = DepthFrame([[2., 0., 0.], [2., 0., 0.], [3., 0., 0.], [2., 0., 0.]])
        points = [[1., 0., 0.], [2., 0., 0.], [4., 0., 0.]]
        self.assert_query_equal(frame, points)
        np.testing.assert_array_equal(frame.query(points).nearest, [3, 3, 3])

    def test_non_event_and_event_depth_layers(self):
        points = [[.9, 0., 0.], [1., 0., 0.], [1.01, 0., 0.], [1.5, 0., 0.], [2., 0., 0.]]
        for events in [[False, False], [True, False], [False, True], [True, True]]:
            self.assert_query_equal(DepthFrame([[1., 0., 0.], [2., 0., 0.]], event=events), points)

    def test_depth_reductions_with_rotations_and_random_mixed_returns(self):
        rng = np.random.default_rng(391)
        angles = rng.normal(0., .002, (400, 2))
        ranges = rng.uniform(.6, 3., 400)
        points = np.column_stack([ranges, ranges[:, None]*angles])
        rotation = Rotation.from_euler('xyz', [.03, -.2, 2.9]).as_matrix()
        for origin in [np.zeros(3), np.array([.1, -.3, .7])]:
            frame = DepthFrame(points, rotation, origin, rng.random(len(points)) > .4)
            queries = np.vstack([points[::4], points[::3] + [.05, .002, 0.], [[1., 5., 0.]]])
            self.assert_query_equal(frame, queries)

    def test_ancestor_deduplication_preserves_event_bits(self):
        points = np.array([[1.2, 0., 0.], [1.3, 0., 0.], [1.4, 0., 0.], [1.2, .3, 0.]])
        newest = DepthFrame([[1.6, 0., 0.], [1.6, 0., 0.]])
        older = DepthFrame([[1.8, 0., 0.]])
        expected = reference_events(points, newest, older)
        with patch.object(older, 'query', wraps=older.query) as query:
            actual = event_core.point_events(points, newest, older)
        np.testing.assert_array_equal(actual[0], expected[0])
        for name in expected[1]:
            np.testing.assert_array_equal(actual[1][name], expected[1][name], err_msg=name)
        # The first call checks all points. The ancestry call can use one
        # unique real ancestor although three points share that ancestor.
        self.assertEqual(len(query.call_args_list), 2)
        self.assertEqual(len(query.call_args_list[1].args[0]), 1)

    def test_ancestor_deduplication_at_depth_and_lateral_boundaries(self):
        rotation = Rotation.from_euler('xyz', [.17, -.28, 2.81]).as_matrix()
        for separation in [np.nextafter(.03, 0.), .03, np.nextafter(.03, np.inf)]:
            newest = DepthFrame([[1.6, 0., 0.], [1.6, 0., 0.], [1.6, .03, 0.]], rotation)
            older = DepthFrame([[1.6 + separation, 0., 0.], [1.6, .03 + separation, 0.]], rotation)
            points = np.array([[x, y, 0.] for x in [1.2, 1.4, 1.6] for y in [0., .003, .03]])
            expected = reference_events(points, newest, older)
            actual = event_core.point_events(points, newest, older)
            np.testing.assert_array_equal(actual[0], expected[0])
            for name in expected[1]:
                np.testing.assert_array_equal(actual[1][name], expected[1][name], err_msg=name)

    def assert_candidates_equal(self, points, kinds, static, history, minimum=6, maximum=800, eligible=None):
        expected = reference_candidates(points, kinds, static, history, minimum, maximum, eligible)
        actual = event_core.candidates(points, kinds, static, history, minimum, maximum, eligible)
        self.assertEqual(len(actual), len(expected))
        for left, right in zip(actual, expected):
            self.assertEqual(set(left), set(right))
            for key in left:
                np.testing.assert_array_equal(left[key], right[key], err_msg=key)
        return actual

    def test_core_count_border_membership_and_original_group_order(self):
        # The first group in original index order is at x=3, not x=0.
        # Each group's final point is a border with only one core neighbor.
        a = np.array([[3., 0., 0.], [3.05, 0., 0.], [3.10, 0., 0.], [3.24, 0., 0.]])
        b = a - [3., 0., 0.]
        points = np.vstack([a, b, [[8., 0., 0.]]])
        kinds = np.array([1, 0, 2, 4, 4, 2, 0, 1, 0], dtype=np.uint8)
        actual = self.assert_candidates_equal(points, kinds, np.zeros(len(points), dtype=bool), [], minimum=3)
        np.testing.assert_array_equal(actual[0]['indices'], [0, 1, 2, 3])
        np.testing.assert_array_equal(actual[1]['indices'], [4, 5, 6, 7])
        # With core >=3 including self, a non-core point cannot neighbor two
        # cores from different groups: those two plus self already make core.

    def test_candidate_empty_core_roi_static_and_limits(self):
        rng = np.random.default_rng(63)
        patch = rng.normal(0., .025, (12, 3)) + [1., 0., 0.]
        points = np.vstack([patch, patch + [2., 0., 0.], [[7., 0., 0.], [8., 0., 0.]]])
        history = [DepthFrame(points + [.07, 0., 0.]), DepthFrame(points + [.10, 0., 0.])]
        kinds = np.arange(len(points), dtype=np.uint8) % 8
        for static, eligible in [(np.zeros(len(points), dtype=bool), None),
                                 (np.zeros(len(points), dtype=bool), np.arange(len(points)) < 12),
                                 (np.arange(len(points)) < 12, None),
                                 (np.ones(len(points), dtype=bool), None)]:
            for maximum in [8, 800]:
                self.assert_candidates_equal(points, kinds, static, history, maximum=maximum, eligible=eligible)
        self.assert_candidates_equal([[1., 0., 0.], [2., 0., 0.]], np.zeros(2, dtype=np.uint8),
                                     np.zeros(2, dtype=bool), [])

    def test_batch_novelty_keeps_group_order_thresholds_and_missing_history(self):
        rng = np.random.default_rng(915)
        patch = rng.normal(0., .012, (10, 3))
        points = np.vstack([patch + [x, 0., 0.] for x in [4., 1., 3., 2.]])
        kinds = rng.integers(0, 8, len(points), dtype=np.uint8)
        static = np.zeros(len(points), dtype=bool)
        # Exact threshold distances and both sides are retained, rather than
        # changing novelty to an aggregate statistic across candidate groups.
        offsets = np.array([.06, np.nextafter(.06, 0.), np.nextafter(.06, np.inf)])
        history = DepthFrame(points + offsets[np.arange(len(points)) % 3, None]*[1., 0., 0.])
        for frames in [[], [history], [history, DepthFrame([])],
                       [history, DepthFrame(points + [.15, 0., 0.])]]:
            actual = self.assert_candidates_equal(points, kinds, static, frames)
            self.assertEqual(len(actual), 4)
            if len([f for f in frames if f.world_tree is not None]) != 2:
                self.assertTrue(all(value['novelty'] == 0. for value in actual))

    def test_roi_tree_construction_keeps_complete_ray_observations(self):
        points = np.array([[1., 0., 0.], [2., 0., 0.], [3., 0., 0.]])
        for eligible in [np.array([True, False, True]), np.zeros(3, dtype=bool)]:
            frame = DepthFrame(points, eligible=eligible)
            np.testing.assert_array_equal(frame.points, points)
            self.assert_query_equal(frame, [[2., 0., 0.]])
            if np.any(eligible):
                expected = cKDTree(points[eligible]).query([[2., 0., 0.]], k=1, workers=1)
                actual = frame.world_tree.query([[2., 0., 0.]], k=1, workers=1)
                np.testing.assert_array_equal(actual[0], expected[0])
                np.testing.assert_array_equal(actual[1], expected[1])
            else:
                self.assertIsNone(frame.world_tree)

    def assert_pairing_equal(self, centers, track_centers, continuous):
        detector = EventDetector(capture=0)
        detector.first_stamp, detector.was_capturing, detector.sequence = 0., False, 4
        detector.history.extend([(0.2, DepthFrame([])), (0.3, DepthFrame([]))])
        detector.next_id = len(track_centers)
        detector.tracks = [dict(id=i, sequence=4 if continuous[i] else 3,
                                observations=deque([(.3, np.asarray(center))], maxlen=3), published=False)
                           for i, center in enumerate(track_centers)]
        candidates = [dict(center=np.asarray(center), size=np.array([.1, .1, .1]),
                           indices=np.array([], dtype=int), event_fraction=0., event_counts=[0, 0, 0], novelty=0.)
                      for center in centers]
        expected = reference_pairing(candidates, detector.tracks, 5)
        with patch.object(event_core, 'candidates', return_value=candidates):
            result = detector.step(.4, [], np.eye(3), np.zeros(3))
        new_id = len(track_centers)
        identifiers = []
        for ci in range(len(candidates)):
            if ci in expected:
                identifiers.append(expected[ci])
            else:
                identifiers.append(new_id)
                new_id += 1
        self.assertEqual([row['track'] for row in result['decisions']], identifiers)
        return identifiers

    def test_pairing_radius_boundary_uses_exact_original_distance(self):
        for distance in [.82, np.nextafter(.82, np.inf), .825, .83, .84]:
            identifiers = self.assert_pairing_equal([[distance, 0., 0.]], [[0., 0., 0.]], [True])
            self.assertEqual(identifiers, [0] if distance <= .82 else [1])

    def test_pairing_continuous_priority_and_one_to_one_sorting(self):
        self.assertEqual(self.assert_pairing_equal([[.82, 0., 0.]],
                                                  [[0., 0., 0.], [.8, 0., 0.]], [True, False]), [0])
        self.assert_pairing_equal([[.49, 0., 0.], [.51, 0., 0.], [.5, .01, 0.]],
                                  [[0., 0., 0.], [.5, 0., 0.]], [True, True])
        self.assert_pairing_equal([[.5, 0., 0.]], [[0., 0., 0.], [1., 0., 0.]], [True, True])

    def test_pairing_prefilter_keeps_all_edges_in_mixed_scene(self):
        rng = np.random.default_rng(731)
        tracks = rng.normal(0., 1., (100, 3))
        centers = np.vstack([tracks[:20] + rng.normal(0., .1, (20, 3)),
                             rng.normal(0., 2., (20, 3))])
        self.assert_pairing_equal(centers, tracks, rng.random(len(tracks)) > .4)


if __name__ == '__main__':
    unittest.main()
