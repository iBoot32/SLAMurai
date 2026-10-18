"""Build the tuning zone ('safe area') from a Nav2 map.  Offline tool (numpy, scipy, PIL, yaml).

zone.yaml (hand-drawn by the human, map frame, metres):
    map: ../../autonav/nav/maps/firstfloor_360.yaml
    seed: [0.5, 0.5]                    # a point known to be safe floor ('auto' = most open point of the largest area)
    include_polygons: []                # optional: restrict to these polygons ([[x, y], ...]); empty = no restriction
    keepout_polygons:                   # stairs, drops, rugs, cables, anything the lidar cannot see
      - [[1, 1], [2, 1], [2, 2], [1, 2]]
    clearance_m: 0.30                   # zone centre points stay at least this far from every wall / unknown cell

Output zone.npz: sdf (m, + inside), allowed (bool), resolution, origin_x, origin_y.  The safety gate loads `sdf`; the
scenario generator uses `allowed`.  Cells are stored with row index = y (north up), like sim/world.py.
"""
import os
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), m['image'])).convert('L'))
    free = np.flipud(img >= 250)                       # unknown (205) counts as not free: never plan into it
    return free, float(m['resolution']), float(m['origin'][0]), float(m['origin'][1])


def _raster(poly, shape, res, ox, oy):
    h, w = shape
    im = Image.new('L', (w, h), 0)
    pts = [((x - ox) / res, (h - 1) - (y - oy) / res) for x, y in poly]      # PIL rows go down; our rows go up
    ImageDraw.Draw(im).polygon(pts, fill=1)
    return np.array(im).astype(bool)[::-1]


def build_zone(zone_yaml):
    z = yaml.safe_load(open(zone_yaml))
    base = os.path.dirname(os.path.abspath(zone_yaml))
    free, res, ox, oy = load_map(os.path.join(base, z['map']))
    edt = ndimage.distance_transform_edt(free) * res
    ok = free & (edt - 0.5 * res >= float(z.get('clearance_m', 0.30)))
    if z.get('include_polygons'):
        inc = np.zeros_like(ok)
        for p in z['include_polygons']:
            inc |= _raster(p, ok.shape, res, ox, oy)
        ok &= inc
    for p in z.get('keepout_polygons') or []:
        ok &= ~_raster(p, ok.shape, res, ox, oy)
    lab, n = ndimage.label(ok)
    if z['seed'] == 'auto':                    # largest connected piece of floor, started at its most open point
        sizes = ndimage.sum(ok, lab, range(1, n + 1))
        big = lab == (1 + int(np.argmax(sizes)))
        si, sj = np.unravel_index(np.argmax(np.where(big, edt, 0)), edt.shape)
        z['seed'] = [ox + (sj + 0.5) * res, oy + (si + 0.5) * res]
    sj, si = int((z['seed'][0] - ox) / res), int((z['seed'][1] - oy) / res)
    if not (0 <= si < ok.shape[0] and 0 <= sj < ok.shape[1]) or lab[si, sj] == 0:
        raise ValueError('seed %s is not inside the allowed area (wall clearance / keep-out / polygon)' % (z['seed'],))
    allowed = lab == lab[si, sj]
    sdf = (ndimage.distance_transform_edt(allowed) - ndimage.distance_transform_edt(~allowed)) * res
    return {'seed': np.array(z['seed']), 'sdf': sdf.astype(np.float32), 'allowed': allowed, 'resolution': res, 'origin_x': ox, 'origin_y': oy,
            'map_free': free, 'wall_dist': edt.astype(np.float32)}


def save_zone(zone, path):
    np.savez_compressed(path, **zone)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('zone_yaml')
    ap.add_argument('out_npz')
    a = ap.parse_args()
    zn = build_zone(a.zone_yaml)
    save_zone(zn, a.out_npz)
    print('zone area %.1f m2 of %.1f m2 free' % (zn['allowed'].sum() * zn['resolution'] ** 2,
                                                 zn['map_free'].sum() * zn['resolution'] ** 2))
