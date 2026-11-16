"""ROS-independent map conversion and stopped-at-corners quintic segments."""
import csv
import math


def load_path(filename, resolution, origin_x, origin_y, yaw, flip_y):
    if resolution <= 0 or not all(math.isfinite(v) for v in (resolution, origin_x, origin_y, yaw)):
        raise ValueError('Invalid map transform')
    points = []
    c, s = math.cos(yaw), math.sin(yaw)
    with open(filename, newline='') as stream:
        for row in csv.reader(stream):
            if not row:
                continue
            if len(row) != 2:
                raise ValueError('CSV must contain exactly two numeric columns, no header')
            u, v = map(float, row)
            if not math.isfinite(u) or not math.isfinite(v):
                raise ValueError('Non-finite path coordinate')
            # MATLAB pixel (1,1) maps to origin; image y usually points downward.
            x, y = (u-1)*resolution, (v-1)*resolution*(-1 if flip_y else 1)
            point = (origin_x+c*x-s*y, origin_y+s*x+c*y)
            if not points or math.dist(point, points[-1]) > 1e-6:
                points.append(point)
    if len(points) < 2:
        raise ValueError('Path needs at least two distinct points')
    return points


def duration(a, b, speed, acceleration):
    distance = math.dist(a, b)
    return max(0.1, 1.875*distance/speed, math.sqrt(5.774*distance/acceleration))


def sample(a, b, elapsed, seconds):
    u = max(0.0, min(1.0, elapsed/seconds))
    h = 10*u**3-15*u**4+6*u**5
    dh = (30*u**2-60*u**3+30*u**4)/seconds
    ddh = (60*u-180*u**2+120*u**3)/seconds**2
    d = [b[i]-a[i] for i in range(2)]
    return ([a[i]+h*d[i] for i in range(2)],
            [dh*d[i] for i in range(2)], [ddh*d[i] for i in range(2)])
