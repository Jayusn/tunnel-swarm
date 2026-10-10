"""Experimental point occlusion chains; frame timestamps, not M-detector.

No ROS or annotation imports. All source/ancestor points are real observations.
Missing returns and gaps are unknown. Geometry constants are frozen a priori.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

LATERAL = .03
DEPTH = .03
# The production NanoDBSCAN passes .02 to nanoflann's squared-distance
# radiusSearch. Its actual radius is sqrt(.02), not .02 metres.
REGION_RADIUS = float(np.sqrt(.02))


@dataclass
class Query:
    front: np.ndarray
    behind: np.ndarray
    static: np.ndarray
    unknown: np.ndarray
    nearest: np.ndarray


class DepthFrame:
    """Retains every actual return, including independent near/far layers."""
    def __init__(self, points, rotation=None, origin=None, event=None):
        self.points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        self.rotation = np.eye(3) if rotation is None else np.asarray(rotation)
        self.origin = np.zeros(3) if origin is None else np.asarray(origin)
        rays = (self.points - self.origin) @ self.rotation
        ranges = np.linalg.norm(rays, axis=1)
        valid = np.isfinite(rays).all(axis=1) & (ranges > 0)
        self.indices = np.flatnonzero(valid)
        self.ranges = ranges[valid]
        self.directions = rays[valid] / self.ranges[:, None]
        self.tree = cKDTree(self.directions) if len(self.indices) else None
        self.world_tree = cKDTree(self.points) if len(self.points) else None
        self.event = np.zeros(len(self.points), dtype=bool) if event is None else np.asarray(event, dtype=bool)

    def query(self, points):
        """No depth-envelope interpolation. Nearest occluder dominates a ray."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        size = len(points)
        front = np.zeros(size, dtype=bool)
        behind = front.copy()
        static = front.copy()
        unknown = np.ones(size, dtype=bool)
        nearest = np.full(size, -1, dtype=int)
        if self.tree is None or size == 0:
            return Query(front, behind, static, unknown, nearest)
        rays = (points - self.origin) @ self.rotation
        ranges = np.linalg.norm(rays, axis=1)
        valid = np.isfinite(rays).all(axis=1) & (ranges > 0)
        rows = np.flatnonzero(valid)
        if not len(rows):
            return Query(front, behind, static, unknown, nearest)
        directions = rays[valid] / ranges[valid, None]
        angles = np.arcsin(np.minimum(1., LATERAL / ranges[valid]))
        neighborhoods = self.tree.query_ball_point(directions, 2 * np.sin(angles / 2), workers=1)
        counts = np.fromiter((len(x) for x in neighborhoods), dtype=int)
        if not counts.sum():
            return Query(front, behind, static, unknown, nearest)
        query_index = np.repeat(rows, counts)
        sample_index = np.concatenate(neighborhoods).astype(int)
        depth_difference = self.ranges[sample_index] - ranges[query_index]
        exact = np.abs(depth_difference) <= DEPTH
        exact_any = np.zeros(size, dtype=bool)
        np.logical_or.at(exact_any, query_index, exact)
        np.logical_or.at(static, query_index, exact & ~self.event[self.indices[sample_index]])
        # Real near/far layers can coexist. Use the first return, never the empty
        # interval between the layers, as the ray's visible depth.
        minimum = np.full(size, np.inf)
        np.minimum.at(minimum, query_index, self.ranges[sample_index])
        # An actual farther-layer return cannot establish visible static
        # support when a nearer return occludes the same historical ray.
        static &= ~(minimum < ranges - DEPTH)
        first = self.ranges[sample_index] == minimum[query_index]
        nearest[query_index[first]] = self.indices[sample_index[first]]
        observed = np.isfinite(minimum)
        front = observed & ~exact_any & (minimum > ranges + DEPTH)
        behind = observed & ~exact_any & (minimum < ranges - DEPTH)
        unknown = ~(static | front | behind)
        return Query(front, behind, static, unknown, nearest)


def point_events(points, newest, older):
    """Three actual frames; bits 1=crossing, 2=toward, 4=away.

    Behind a single historical return remains unknown until a real older
    ancestor establishes a second consistent depth order. Static-map support
    checks only historical non-event returns, following the paper's idea.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    q1, q2 = newest.query(points), older.query(points)
    ancestor_rows = np.flatnonzero(q1.nearest >= 0)
    chain_front = np.zeros(len(points), dtype=bool)
    chain_behind = chain_front.copy()
    chain_static = chain_front.copy()
    if len(ancestor_rows):
        ancestors = newest.points[q1.nearest[ancestor_rows]]
        q_ancestor = older.query(ancestors)
        chain_front[ancestor_rows] = q_ancestor.front
        chain_behind[ancestor_rows] = q_ancestor.behind
        chain_static[ancestor_rows] = q_ancestor.static
    stationary = q1.static | q2.static
    crossing = q1.front & q2.front & ~stationary
    toward = crossing & chain_front & ~chain_static
    away = q1.behind & q2.behind & chain_behind & ~chain_static & ~stationary
    types = crossing.astype(np.uint8) | (toward.astype(np.uint8) << 1) | (away.astype(np.uint8) << 2)
    diagnostics = dict(front=q1.front, behind=q1.behind, static=stationary,
                       unknown=~(stationary | crossing | toward | away))
    return types, diagnostics


def candidates(points, event_types, static, history, min_cluster=6, max_cluster=800, eligible=None):
    """Grow point events in current real observations; retain unknown surfaces.

    Unknown clusters are associated before they acquire events so confirmation
    still concerns three real target observations, not three predicted points.
    No physical-size filtering or shape/class prior is used.
    """
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
    core_lookup = np.full(len(local), -1, dtype=int)
    core_lookup[core_indices] = np.arange(len(core_indices))
    pairs = tree.query_pairs(REGION_RADIUS, output_type='ndarray')
    pairs = pairs[core[pairs[:, 0]] & core[pairs[:, 1]]] if len(pairs) else pairs
    graph = coo_matrix((np.ones(len(pairs)), (core_lookup[pairs[:, 0]], core_lookup[pairs[:, 1]])), shape=(len(core_indices), len(core_indices))).tocsr()
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
        if not min_cluster <= len(member) <= max_cluster:
            continue
        observed = points[member]
        low, high = observed.min(axis=0), observed.max(axis=0)
        novel = []
        for frame in history:
            if frame.world_tree is None:
                continue
            distances, _ = frame.world_tree.query(observed, k=1, workers=1)
            novel.append(float(np.mean(distances > .06)))
        output.append(dict(indices=member, center=(low + high) / 2, size=high - low,
                           event_fraction=float(np.mean(event_types[member] != 0)),
                           event_counts=[int(np.sum((event_types[member] & bit) != 0)) for bit in (1, 2, 4)],
                           novelty=min(novel) if len(novel) == 2 else 0.))
    return output


class EventDetector:
    """A small independent experiment, without changes to production tracking."""
    def __init__(self, capture=5., min_cluster=6, max_cluster=800,
                 confirmation_frames=3, displacement=.06, speed=.14,
                 novel_fraction=.26, tracked_novel_fraction=.20):
        assert confirmation_frames == 3, 'This experiment has no two-frame mode'
        self.capture = capture
        self.min_cluster, self.max_cluster = min_cluster, max_cluster
        self.displacement, self.speed = displacement, speed
        self.novel_fraction, self.tracked_novel_fraction = novel_fraction, tracked_novel_fraction
        self.history = deque(maxlen=2)
        self.tracks = []
        self.first_stamp = None
        self.next_id = 0
        self.sequence = 0
        self.was_capturing = True

    def step(self, stamp, points, rotation, origin, eligible=None, candidate_height_limit=None):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if eligible is not None:
            eligible = np.asarray(eligible, dtype=bool)[finite]
        self.sequence += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
        capturing = stamp - self.first_stamp < self.capture
        if self.was_capturing and not capturing:
            self.history.clear()
            self.tracks.clear()
        self.was_capturing = capturing
        current = DepthFrame(points, rotation, origin)
        if eligible is not None:
            # Keep the complete raw cloud for actual ray depth, while the
            # nearest-point novelty buffer shares the candidate input ROI.
            current.world_tree = cKDTree(points[eligible]) if np.any(eligible) else None
        types = np.zeros(len(points), dtype=np.uint8)
        static = np.zeros(len(points), dtype=bool)
        # Frames without a chronological, bounded pair cannot create a chain.
        consecutive = len(self.history) == 2 and 0 < stamp - self.history[-1][0] <= .4 and 0 < self.history[-1][0] - self.history[0][0] <= .4
        if consecutive:
            types, diagnostic = point_events(points, self.history[-1][1], self.history[0][1])
            static = diagnostic['static']
        elif len(self.history):
            static = self.history[-1][1].query(points).static
        types[static] = 0
        current.event = types != 0
        candidate = candidates(points, types, static, [f for _, f in self.history], self.min_cluster, self.max_cluster, eligible)
        if candidate_height_limit is not None:
            candidate = [v for v in candidate if v['center'][2] - .5*v['size'][2] <= candidate_height_limit]
        edges = []
        for ci, value in enumerate(candidate):
            for ti, track in enumerate(self.tracks):
                distance = np.linalg.norm(value['center'] - track['observations'][-1][1])
                if distance <= .82:
                    edges.append((track['sequence'] != self.sequence - 1, distance, ci, ti))
        assigned_candidate, assigned_track, matched = set(), set(), {}
        for _, _, ci, ti in sorted(edges):
            if ci not in assigned_candidate and ti not in assigned_track:
                assigned_candidate.add(ci); assigned_track.add(ti); matched[ci] = ti
        published, decisions = [], []
        for ci, value in enumerate(candidate):
            if ci in matched:
                track = self.tracks[matched[ci]]
                if track['sequence'] != self.sequence - 1 or stamp - track['observations'][-1][0] > .4:
                    track['observations'].clear()
                    track['published'] = False
                track['observations'].append((stamp, value['center']))
            else:
                track = dict(id=self.next_id, sequence=self.sequence, observations=deque([(stamp, value['center'])], maxlen=3), published=False)
                self.next_id += 1
                self.tracks.append(track)
            track['sequence'] = self.sequence
            observations = list(track['observations'])
            displacement = max((np.linalg.norm(a[1] - b[1]) for a in observations for b in observations), default=0.)
            speeds = [np.linalg.norm(b[1] - a[1]) / (b[0] - a[0])
                      for a, b in zip(observations[:-1], observations[1:]) if b[0] > a[0]]
            speed = float(np.mean(speeds)) if speeds else 0.
            threshold = self.tracked_novel_fraction if track['published'] else self.novel_fraction
            confirmed = len(observations) == 3 and displacement >= self.displacement and speed >= self.speed
            accepted = not capturing and consecutive and confirmed and value['event_fraction'] >= self.novel_fraction and value['novelty'] >= threshold
            reason = 'publish' if accepted else 'capture' if capturing else 'point_chain_missing' if not consecutive else 'observation_count' if len(observations) < 3 else 'motion_displacement' if displacement < self.displacement else 'motion_speed' if speed < self.speed else 'point_event' if value['event_fraction'] < self.novel_fraction else 'novelty'
            decisions.append(dict(center=value['center'].tolist(), size=value['size'].tolist(), track=track['id'], observations=len(observations), displacement=float(displacement), speed=float(speed), event_fraction=value['event_fraction'], novelty=value['novelty'], event_counts=value['event_counts'], reason=reason))
            if accepted:
                velocity = (observations[-1][1] - observations[-2][1]) / (observations[-1][0] - observations[-2][0])
                value = dict(value, velocity=velocity, track=track['id'])
                published.append(value)
                track['published'] = True
        self.tracks = [t for t in self.tracks if self.sequence - t['sequence'] <= 3]
        self.history.append((stamp, current))
        return dict(points=points, event_types=types, static=static, candidates=candidate, published=published, decisions=decisions, capturing=capturing, point_chain_valid=consecutive)
