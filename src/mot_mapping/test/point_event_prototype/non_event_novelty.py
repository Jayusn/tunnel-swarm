"""Isolated novelty-reference experiment; production ROS does not import it."""
import numpy as np
from scipy.spatial import cKDTree


def non_event_novelty_tree(points, event, eligible=None):
    """Compare novelty with real ROI returns without historical motion labels.

    Unknown non-event returns remain references. Empty support stays unknown
    (None), never a fabricated infinite-distance observation. This does not
    change the raw points, angular ray tree, candidate membership or labels.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    keep = ~np.asarray(event, dtype=bool)
    if eligible is not None:
        keep = keep & np.asarray(eligible, dtype=bool)
    reference = points[keep]
    return cKDTree(reference) if len(reference) else None
