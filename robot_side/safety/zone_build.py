"""Allowed-area helpers shared by the zone editor and the gate. numpy + scipy (distance transform)."""
import numpy as np
from scipy import ndimage

from safety_core import Zone


def compose(free, paint, res, clearance_m=0.10):
    """Allowed cells. free: bool map (row 0 = lowest y). paint: 0 none, 1 allow, 2 keep-out (same shape).
    Free cells at least clearance_m from a wall or unknown cell; if anything is painted allow, only that; never keep-out."""
    ok = free & (ndimage.distance_transform_edt(free) * res - 0.5 * res >= clearance_m)
    if (paint == 1).any():
        ok &= paint == 1
    return ok & (paint != 2)


def zone_from_allowed(allowed, res, ox, oy):
    sdf = (ndimage.distance_transform_edt(allowed) - ndimage.distance_transform_edt(~allowed)) * res
    return Zone(sdf.astype(np.float32), res, ox, oy)
