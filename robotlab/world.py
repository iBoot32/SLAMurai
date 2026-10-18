"""The static map as data, so a script can pick goals without looking at pictures. The map file comes from the profile (`map:`)."""
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


class World(object):
    def __init__(self, yaml_path):
        y = yaml.safe_load(open(yaml_path))
        im = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), y['image'])).convert('L'), dtype=float)
        p = (255.0 - im) / 255.0 if not y.get('negate') else im / 255.0
        self.res, (self.ox, self.oy) = float(y['resolution']), y['origin'][:2]
        self.free = np.flipud(p < float(y.get('free_thresh', 0.25)))       # [row from the bottom, col]: row = (y - oy) / res
        self.dist = ndimage.distance_transform_edt(self.free) * self.res    # metres to the nearest non-free cell

    def cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def clearance(self, x, y):
        """Metres from (x, y) to the nearest wall or unknown cell (0 outside the map or in a blocked cell)."""
        r, c = self.cell(x, y)
        return float(self.dist[r, c]) if 0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1] else 0.0

    def goals(self, pose, clearance, rmin=0.5, rmax=1.5, n=8):
        """Up to n goal points (x, y) between rmin and rmax from pose that are connected to the pose through cells with at least
        `clearance` of room, spread around the robot. Sorted by the room they have."""
        ok = self.dist >= clearance
        r0, c0 = self.cell(pose[0], pose[1])
        lab, _ = ndimage.label(ok)
        if not ok[r0, c0]:                                # the robot itself is tighter than the clearance: use the nearest ok cell
            rr, cc = np.nonzero(ok)
            k = int(np.argmin((rr - r0) ** 2 + (cc - c0) ** 2))
            r0, c0 = rr[k], cc[k]
        rr, cc = np.nonzero(lab == lab[r0, c0])
        x, y = self.ox + (cc + 0.5) * self.res, self.oy + (rr + 0.5) * self.res
        d = np.hypot(x - pose[0], y - pose[1])
        m = (d >= rmin) & (d <= rmax)
        pts = sorted(zip(x[m], y[m], self.dist[rr[m], cc[m]]), key=lambda t: -t[2])
        out = []
        for px, py, room in pts:                          # keep points at least rmin/2 apart
            if all(math.hypot(px - a, py - b) > rmin / 2 for a, b, _ in out):
                out.append((float(px), float(py), float(room)))
            if len(out) >= n:
                break
        return out
