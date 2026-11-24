"""Gravity-aligned local registration. T maps this boot's FAST-LIO into saved map."""
import numpy as np
from scipy.spatial import cKDTree


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def downsample(points, resolution=.1):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    _, indices = np.unique(np.floor(points / resolution), axis=0, return_index=True)
    return points[indices]


def register(source, target, initial=(0., 0., 0., 0.), max_distance=.5):
    """Trimmed point-to-point ICP: XYZ translation + yaw, initial=[x,y,z,yaw]."""
    source, target = downsample(source), downsample(target)
    if min(len(source), len(target)) < 100:
        raise ValueError('匹配点不足100个，请增加静态环境扫描')
    tree = cKDTree(target)
    yaw = float(initial[3])
    shift = np.array(initial[:3], dtype=float)
    for _ in range(60):
        transformed = source @ rotation(yaw).T + shift
        distance, index = tree.query(transformed)
        mask = distance < max_distance
        if mask.sum() < 100:
            raise ValueError('匹配重叠不足，请回到原起飞点并保持原朝向')
        threshold = np.quantile(distance[mask], .85)
        mask &= distance <= threshold
        a, b = source[mask], target[index[mask]]
        ac, bc = a - a.mean(axis=0), b - b.mean(axis=0)
        new_yaw = np.arctan2(np.sum(ac[:, 0]*bc[:, 1]-ac[:, 1]*bc[:, 0]),
                             np.sum(ac[:, 0]*bc[:, 0]+ac[:, 1]*bc[:, 1]))
        new_shift = b.mean(axis=0) - rotation(new_yaw) @ a.mean(axis=0)
        delta = np.linalg.norm(new_shift-shift) + abs(np.arctan2(np.sin(new_yaw-yaw), np.cos(new_yaw-yaw)))
        yaw, shift = new_yaw, new_shift
        if delta < 1.e-5:
            break
    distance, _ = tree.query(source @ rotation(yaw).T + shift)
    mask = distance < .2
    ratio = float(mask.mean())
    rmse = float(np.sqrt(np.mean(distance[mask]**2))) if mask.any() else float('inf')
    return float(yaw), shift, {'inlier_ratio': ratio, 'rmse_m': rmse, 'points': len(source)}
