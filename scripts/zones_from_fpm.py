#!/usr/bin/env python3
"""Write a world's zones (its doorways, with their motion constraints) in the map frame from FPM.

Usage: zones_from_fpm.py <fpm json-ld dir> worlds/secoro.xml maps/secoro_zones.yaml
Needs scenery_builder's fpm package (the workspace .venv).
"""

import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from fpm.graph import build_graph_from_directory, get_3d_structure

# eddie_navigation/msg/MotionConstraints, by name
DOOR_CONSTRAINTS = {'heading_mode': 'forward', 'max_speed': 0.1, 'align_tolerance_xy': 0.01,
                    'align_tolerance_yaw': 0.01, 'stop_distance': 0.1}


def main():
    fpm_dir, world, out = sys.argv[1:4]
    # the map frame is the mesh frame moved by the world body's pos, as maps/<world>.yaml
    offset = np.array([float(v) for v in ET.parse(world).find('.//body').get('pos').split()[:2]])
    g = build_graph_from_directory((fpm_dir,))
    walls = {w['name']: np.array(w['vertices'])[:, :2] for w in get_3d_structure(g, 'Wall')}
    zones = []
    for e in get_3d_structure(g, 'Entryway'):
        v = np.array(e['vertices'])[:, :2]
        w = walls[e['voids'][0]]
        # the opening's solid runs deeper than the wall: the voided wall's thin axis is across
        across = int(np.argmin(w.max(0) - w.min(0)))
        along = 1 - across
        centre = np.empty(2)
        centre[across] = (w.min(0)[across] + w.max(0)[across]) / 2
        centre[along] = (v.min(0)[along] + v.max(0)[along]) / 2
        normal = [0.0, 0.0]
        normal[across] = 1.0
        zones.append({'name': e['name'],
                      'type': 'door',
                      'centre': [round(float(c), 3) for c in centre + offset],
                      'normal': normal,
                      'width': round(float(v.max(0)[along] - v.min(0)[along]), 3),
                      'constraints': dict(DOOR_CONSTRAINTS)})
    zones.sort(key=lambda z: z['name'])
    with open(out, 'w') as f:
        yaml.safe_dump({'zones': zones}, f, default_flow_style=None, sort_keys=False)
    print(f'{len(zones)} zones -> {out}')


if __name__ == '__main__':
    main()
