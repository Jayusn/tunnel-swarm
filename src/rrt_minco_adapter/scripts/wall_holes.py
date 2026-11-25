"""One-pass single-voxel wall repairs from observed occupancy, without growth."""


def find_wall_holes(voxels, bounds, resolution, removed_voxels=()):
    occupied = set(map(tuple, voxels))
    removed = set(map(tuple, removed_voxels))
    candidates = {(x, y, z+1) for x, y, z in occupied}
    result = []
    for x, y, z in sorted(candidates - occupied - removed):
        px = bounds[0] + (x+.5)*resolution
        py = bounds[2] + (y+.5)*resolution
        pz = (z+.5)*resolution
        # Exclude floor and the entire manually cleared start cylinder.
        if pz < .3 or (px-.01)**2 + (py-.01)**2 <= .4**2:
            continue
        # All eight neighbors in a vertical 3x3 plane must be observed.
        # Only the original set is consulted; filled voxels never seed another fill.
        if any(all((x+dx*a, y+dy*a, z+b) in occupied
                   for a in (-1, 0, 1) for b in (-1, 0, 1) if a or b)
               for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1))):
            result.append((x, y, z))
    return result
