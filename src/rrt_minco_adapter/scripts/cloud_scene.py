"""Project a surveyed static region into conservative XY obstacle rectangles."""
import math


class CloudScene:
    def __init__(self, bounds, resolution=.1, z_min=.3, z_max=.7):
        self.bounds = list(map(float, bounds))
        self.resolution, self.z_min, self.z_max = map(float, (resolution, z_min, z_max))
        if (len(self.bounds) != 4 or not all(map(math.isfinite,
                self.bounds + [self.resolution, self.z_min, self.z_max]))
                or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]
                or resolution <= 0 or z_min >= z_max):
            raise ValueError('Invalid bounds, resolution or height interval')
        self.cells = set()

    def add(self, points):
        xmin, xmax, ymin, ymax = self.bounds
        for x, y, z in points:
            if (all(map(math.isfinite, (x, y, z))) and self.z_min <= z <= self.z_max
                    and xmin <= x < xmax and ymin <= y < ymax):
                self.cells.add((math.floor((x-xmin)/self.resolution),
                                math.floor((y-ymin)/self.resolution)))

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
        return dict(bounds=self.bounds, boxes=boxes, frame_id=frame,
                    z_min=self.z_min, z_max=self.z_max, resolution=r,
                    map_semantics='operator_surveyed_static_region')

    def occupancy_grid(self):
        xmin, xmax, ymin, ymax = self.bounds
        width = math.ceil((xmax-xmin)/self.resolution)
        height = math.ceil((ymax-ymin)/self.resolution)
        # Point hits establish obstacles, but absence of hits does not establish free space.
        data = [-1] * (width * height)
        for x, y in self.cells:
            data[y * width + x] = 100
        return width, height, data
