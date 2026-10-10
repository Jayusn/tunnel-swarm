"""Frame-level evidence from actual frozen startup points; experiment only."""
import numpy as np


def actual_view_reference(points, rotation=None, origin=None, angle_degrees=.25):
    """Project historical samples; retain nearest actual sample per cell.

    The projected depth is not a current actual lidar observation.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    rotation = np.eye(3) if rotation is None else np.asarray(rotation)
    origin = np.zeros(3) if origin is None else np.asarray(origin)
    rays = (points - origin) @ rotation
    ranges = np.linalg.norm(rays, axis=1)
    indices = np.flatnonzero(np.isfinite(rays).all(axis=1) & (ranges > 0))
    rays, ranges = rays[indices], ranges[indices]
    step = np.deg2rad(angle_degrees)
    width = int(np.ceil(2 * np.pi / step))
    azimuth = np.floor((np.arctan2(rays[:, 1], rays[:, 0]) + np.pi) / step).astype(int) % width
    elevation = np.floor((np.arctan2(rays[:, 2], np.linalg.norm(rays[:, :2], axis=1)) + np.pi/2) / step).astype(int)
    cells = elevation * width + azimuth
    order = np.lexsort((ranges, cells))
    _, first = np.unique(cells[order], return_index=True)
    return points[np.sort(indices[order[first]])]


def reference_candidate_features(points, candidates, view=None, world_tree=None):
    """Current real-member foreground and distance from captured background.

    Foreground needs a farther historical sample projected into this view;
    absent or nearer samples supply no foreground support. A nonempty world
    tree can still mark an uncovered point novel when its ray is unknown.
    Neither feature proves motion. Ordinary observation/motion gates live in
    the independent detector, not in this helper.
    """
    if not candidates:
        return []
    counts = np.array([len(value['indices']) for value in candidates])
    starts = np.r_[0, np.cumsum(counts)[:-1]]
    actual = np.asarray(points)[np.concatenate([value['indices'] for value in candidates])]
    foreground = np.zeros(len(candidates))
    novelty = foreground.copy()
    if view is not None:
        foreground = np.add.reduceat(view.query(actual).front, starts, dtype=np.int64) / counts
    if world_tree is not None:
        novelty = np.add.reduceat(world_tree.query(actual, workers=1)[0] > .06, starts, dtype=np.int64) / counts
    return [dict(value, reference_front_fraction=float(foreground[i]),
                 reference_novelty=float(novelty[i])) for i, value in enumerate(candidates)]
