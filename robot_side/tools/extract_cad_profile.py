#!/usr/bin/env python3
"""Read the SLAMurai CAD (default: robot_cad/v17) and write the geometry numbers the safety layer and Nav2 need.

No SolidWorks needed: it reads the design generator's numeric constants (design10.py, evaluated with `ast`, nothing
imported or executed) and the per-component bounding boxes in boxes.json.  Output: config/cad_profile.json.
SolidWorks axes: +X front, +Y up, +Z right (ROS: x forward, y left, z up).  All lengths in the output are metres.
"""
import argparse, ast, json, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))


def eval_constants(path):
    """Evaluate only module-level `NAME = <numeric expression of earlier names / math>` assignments."""
    tree = ast.parse(open(path, encoding='utf-8').read())
    ns = {'math': math}
    allowed = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Constant, ast.Name, ast.Load, ast.Add, ast.Sub, ast.Mult,
               ast.Div, ast.Pow, ast.USub, ast.UAdd, ast.Tuple, ast.Attribute, ast.Call, ast.FloorDiv)
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        tgt = node.targets[0]
        if isinstance(tgt, ast.Name):
            names, pairs = [tgt.id], [(tgt.id, node.value)]
        elif (isinstance(tgt, ast.Tuple) and isinstance(node.value, ast.Tuple)
              and all(isinstance(e, ast.Name) for e in tgt.elts) and len(tgt.elts) == len(node.value.elts)):
            pairs = [(t.id, v) for t, v in zip(tgt.elts, node.value.elts)]
        else:
            continue
        for name, val in pairs:
            expr = ast.Expression(val)
            ok = all(isinstance(n, allowed) for n in ast.walk(expr))
            for n in ast.walk(expr):       # only math.* calls are allowed
                if isinstance(n, ast.Call) and not (isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name)
                                                   and n.func.value.id == 'math'):
                    ok = False
            if not ok:
                continue
            try:
                ns[name] = eval(compile(expr, path, 'eval'), {'__builtins__': {}}, ns)
            except Exception:
                pass
    return ns


def box_center(b):
    return [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cad', default=os.path.join(ROOT, 'robot_cad', 'v17'))
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'config', 'cad_profile.json'))
    a = ap.parse_args()
    gen = os.path.join(a.cad, 'gen', 'design10.py')
    boxes = json.load(open(os.path.join(a.cad, 'boxes.json')))
    c = eval_constants(gen)
    need = ['PLATE_R', 'WALL_RO', 'SCAN_Y', 'LIDAR_Y', 'AXIS_Y', 'CAP_BLADE_DEG', 'RIM_RO', 'LIDAR_T', 'CAM_T']
    miss = [k for k in need if k not in c]
    if miss:
        sys.exit('design constants missing: %s' % miss)

    names = lambda pre: [k for k in boxes if k.startswith(pre)]
    wheels = [box_center(boxes[k]) for k in names('nex_omni60')]
    wheel_r = [math.hypot(w[0], w[2]) for w in wheels]
    wheel_ang = sorted(round(math.degrees(math.atan2(-w[2], w[0]))) for w in wheels)   # ROS y = -SW z
    wheel_dia = [boxes[k][4] - boxes[k][1] for k in names('nex_omni60')]
    cam = boxes['D455_Solid-1']
    lidar = boxes['RPLIDAR A1-1']
    # real radial extent of the dome wall (outer) and of the camera pod corner
    cam_corner = max(math.hypot(x, z) for x in (cam[0], cam[3]) for z in (cam[2], cam[5]))
    # conservative upper bound: farthest bbox corner of any component (axis-aligned boxes of rounded parts overshoot)
    bbox_max = max(max(math.hypot(x, z) for x in (b[0], b[3]) for z in (b[2], b[5])) for b in boxes.values())
    footprint_r = max(c['WALL_RO'], cam_corner, c['PLATE_R'])
    lidar_tz = c['LIDAR_T']
    # scan axis offset from the robot axis (design10.BEAMS): (LIDAR_T[0]-94.2, LIDAR_T[2]+132.9) in SW (x, z)
    lidar_off_x = (lidar_tz[0] - 94.2) / 1000.0
    lidar_off_y = -(lidar_tz[2] + 132.9) / 1000.0
    # blade azimuth in ROS: same construction as design10._blade_az, then y = -z
    az = []
    for d in c['CAP_BLADE_DEG']:
        dx, dz = math.cos(math.radians(d)), -math.sin(math.radians(d))       # roty(d) applied to (1,0,0)
        az.append(round(math.degrees(math.atan2(-dz, dx)) % 360.0, 1))
    blade_t_mm, blade_r0_mm = 2.0, 68.0                                        # lidar_cap(): rot_rect_top(68, 80, 1.0)
    shadow_half = math.degrees(math.atan2(blade_t_mm / 2.0, blade_r0_mm))
    prof = {
        'source': {'cad_dir': os.path.relpath(a.cad, ROOT), 'generator': 'gen/design10.py', 'boxes': 'boxes.json'},
        'footprint_circumradius_m': round(footprint_r / 1000, 4),
        'footprint_radius_parts_m': {'dome_wall_outer': c['WALL_RO'] / 1000, 'camera_pod_corner': round(cam_corner / 1000, 4),
                                     'bottom_plate': c['PLATE_R'] / 1000},
        'footprint_bbox_upper_bound_m': round(bbox_max / 1000, 4),
        'wheel_centre_radius_m': round(sum(wheel_r) / len(wheel_r) / 1000, 4),
        'wheel_centre_radius_spread_mm': round(max(wheel_r) - min(wheel_r), 3),
        'wheel_azimuths_deg': wheel_ang,
        'wheel_diameter_cad_m': round(sum(wheel_dia) / len(wheel_dia) / 1000, 4),
        'scan_plane_height_m': round(c['SCAN_Y'] / 1000, 4),
        'lidar_axis_offset_xy_m': [round(lidar_off_x, 4), round(lidar_off_y, 4)],
        'lidar_cap_blade_azimuths_deg': az,
        'lidar_blade_shadow_halfwidth_deg': round(shadow_half, 2),
        'camera_front_x_m': round(cam[3] / 1000, 4),
        'camera_half_width_m': round((cam[5] - cam[2]) / 2000, 4),
        'camera_centre_height_m': round((cam[1] + cam[4]) / 2000, 4),
        'lidar_body_box_mm_sw': lidar,
        'ground_clearance_m': round(c['FLOOR_Y0'] / 1000, 4) if 'FLOOR_Y0' in c else None,
        'motor_axis_height_m': round(c['AXIS_Y'] / 1000, 4),
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(prof, open(a.out, 'w'), indent=1)
    print(json.dumps(prof, indent=1))


if __name__ == '__main__':
    main()
