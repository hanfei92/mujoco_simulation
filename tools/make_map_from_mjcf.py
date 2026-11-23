#!/usr/bin/env python3
"""Rasterise the MuJoCo scene into a 2D occupancy map by raycasting.

Run inside the wrapper container (it has mujoco and the model):

    docker exec mujoco-wrapper uv run python /app/tools/make_map_from_mjcf.py \
        --out /tmp/pf_map.json

The map only covers static geometry: the car and the G1 are excluded, so it
matches what a SLAM map of the empty room would look like.
"""

import argparse
import json
import math

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene',
                        default='/app/aidea_mujoco/mjcf/scene.xml')
    parser.add_argument('--out', default='/tmp/pf_map.json')
    parser.add_argument('--resolution', type=float, default=0.1)
    parser.add_argument('--size', type=float, default=20.0)
    parser.add_argument('--scan-height', type=float, default=2.0)
    parser.add_argument('--exclude-bodies',
                        default='base_link,g1_pelvis,g1_torso')
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    excluded = set()
    for name in filter(None, (part.strip()
                              for part in args.exclude_bodies.split(','))):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0:
            excluded.add(body_id)
    root_excluded = [body_id for body_id in excluded
                     if model.body_parentid[body_id] == 0]

    res = args.resolution
    count = int(round(args.size / res))
    origin = -args.size / 2.0
    grid = np.zeros((count, count), dtype=np.int8)
    geomid = np.zeros(1, dtype=np.int32)

    for iy in range(count):
        y = origin + (iy + 0.5) * res
        for ix in range(count):
            x = origin + (ix + 0.5) * res
            distance = mujoco.mj_ray(
                model, data,
                np.array([x, y, args.scan_height], dtype=np.float64),
                np.array([0.0, 0.0, -1.0], dtype=np.float64),
                None, 1, -1, geomid)
            if 0.0 < distance < args.scan_height - 0.05:
                if root_excluded and int(geomid[0]) in excluded:
                    continue
                grid[iy, ix] = 100

    free = int((grid == 0).sum())
    payload = {
        'frame': 'world',
        'resolution': res,
        'width': count,
        'height': count,
        'origin_x': origin,
        'origin_y': origin,
        'data': grid.reshape(-1).tolist(),
    }
    with open(args.out, 'w') as handle:
        json.dump(payload, handle)
    print('wrote {} ({}x{} at {:.2f} m, free {:.1f} m^2)'.format(
        args.out, count, count, res, free * res * res))


if __name__ == '__main__':
    main()
