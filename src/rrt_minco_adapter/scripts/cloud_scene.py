"""Project a surveyed static region into conservative XY obstacle rectangles."""
import math


class CloudScene:
    def __init__(self, bounds, resolution=.1, z_min=.3, z_max=.7, start_clear_radius=0., removed_voxels=None):
        self.bounds = list(map(float, bounds))
        self.resolution, self.z_min, self.z_max = map(float, (resolution, z_min, z_max))
        if (len(self.bounds) != 4 or not all(map(math.isfinite,
                self.bounds + [self.resolution, self.z_min, self.z_max]))
                or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]
                or resolution <= 0 or z_min >= z_max):
            raise ValueError('Invalid bounds, resolution or height interval')
        self.start_clear_radius = float(start_clear_radius)
        if not math.isfinite(self.start_clear_radius) or self.start_clear_radius < 0:
            raise ValueError('Invalid start_clear_radius')
        self.cells = set()
        self.voxels = set()
        self.removed_voxels = set()
        for voxel in removed_voxels or []:
            if len(voxel) != 3 or any(type(v) is not int for v in voxel):
                raise ValueError('removed_voxels must contain integer XYZ indices')
            self.removed_voxels.add(tuple(voxel))

    def is_removed(self, point):
        if not self.removed_voxels:
            return False
        x, y, z = point
        if not all(map(math.isfinite, point)):
            return False
        return (math.floor((x-self.bounds[0])/self.resolution),
                math.floor((y-self.bounds[2])/self.resolution),
                math.floor(z/self.resolution)) in self.removed_voxels

    def add(self, points):
        xmin, xmax, ymin, ymax = self.bounds
        for x, y, z in points:
            if all(map(math.isfinite, (x, y, z))):
                if self.is_removed((x, y, z)):
                    continue
                # Operator-confirmed fixed start cylinder; never follows odometry.
                # Filter before voxelization so an outside hit in the same voxel survives.
                if (self.start_clear_radius > 0 and .3 <= z <= .7
                        and math.hypot(x-.01, y-.01) <= self.start_clear_radius):
                    continue
                ix = math.floor((x-xmin)/self.resolution)
                iy = math.floor((y-ymin)/self.resolution)
                self.voxels.add((ix, iy, math.floor(z/self.resolution)))
                if (xmin <= x < xmax and ymin <= y < ymax
                        and self.z_min <= z <= self.z_max):
                    self.cells.add((ix, iy))

    def voxel_centers(self):
        r = self.resolution
        return [(self.bounds[0] + (x+.5)*r, self.bounds[2] + (y+.5)*r, (z+.5)*r)
                for x, y, z in sorted(self.voxels)]

    def scene(self, frame):
        # Merge horizontal runs and identical runs in consecutive rows; never bridge holes.
        rows = {}
        for x, y in self.cells:
            rows.setdefault(y, []).append(x)
        rectangles, active = [], {}
        previous = None
        for y in sorted(rows):
            if previous is not None and y != previous + 1:
                rectangles.extend(active.values())
                active = {}
            runs = []
            for x in sorted(rows[y]):
                if runs and x == runs[-1][1] + 1:
                    runs[-1][1] = x
                else:
                    runs.append([x, x])
            keys = set(map(tuple, runs))
            for key in list(active):
                if key not in keys:
                    rectangles.append(active.pop(key))
            for key in sorted(keys):
                if key in active:
                    active[key][3] = y + 1
                else:
                    active[key] = [key[0], key[1]+1, y, y+1]
            previous = y
        rectangles.extend(active.values())
        xmin, _, ymin, _ = self.bounds
        r = self.resolution
        boxes = [[xmin+(a+b)*r/2, ymin+(c+d)*r/2, (b-a)*r, (d-c)*r]
                 for a, b, c, d in rectangles]
        result = dict(bounds=self.bounds, boxes=boxes, frame_id=frame,
                    z_min=self.z_min, z_max=self.z_max, resolution=r,
                    map_semantics='operator_surveyed_static_region')
        if self.start_clear_radius > 0:
            result['operator_clear_region'] = dict(center_xy=[.01, .01],
                radius=self.start_clear_radius, z_min=.3, z_max=.7)
        if self.removed_voxels:
            result['operator_removed_voxels'] = sorted(self.removed_voxels)
        return result

    def occupancy_grid(self):
        xmin, xmax, ymin, ymax = self.bounds
        width = math.ceil((xmax-xmin)/self.resolution)
        height = math.ceil((ymax-ymin)/self.resolution)
        # Point hits establish obstacles, but absence of hits does not establish free space.
        data = [-1] * (width * height)
        for x, y in self.cells:
            data[y * width + x] = 100
        return width, height, data
