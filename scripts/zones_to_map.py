#!/usr/bin/env python3
"""Move a map's zones onto another map of the same building by registering their walls.

Usage: zones_to_map.py maps/secoro.yaml maps/secoro_zones.yaml maps/secoro_slam.yaml \
           maps/secoro_slam_zones.yaml
Coarse search over heading (FFT cross-correlation of the wall rasters per 1 deg), then ICP.
"""

import math
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree

FIT_TOLERANCE = 0.10  # [m] a source wall point within this of a target wall counts as fitted


def walls(map_yaml):
    """Occupied cells of a map as world points [m], and the map's resolution."""
    with open(map_yaml) as f:
        info = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), info['image'])),
                   dtype=float)
    occ = (255.0 - img) / 255.0
    if info.get('negate', 0):
        occ = 1.0 - occ
    rows, cols = np.nonzero(occ > info['occupied_thresh'])
    res = info['resolution']
    ox, oy = info['origin'][:2]
    # image row 0 is the top of the map
    pts = np.column_stack([ox + (cols + 0.5) * res, oy + (img.shape[0] - rows - 0.5) * res])
    return pts, res


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def coarse(src, dst, res):
    """Best (yaw, translation) over 1 deg steps: overlap of the rasterised wall points."""
    lo = dst.min(0) - 5.0
    shape = np.ceil((dst.max(0) + 5.0 - lo) / res).astype(int) + 1

    def raster(pts, origin, size):
        g = np.zeros(size[::-1])
        ij = np.floor((pts - origin) / res).astype(int)
        ok = (ij >= 0).all(1) & (ij < size).all(1)
        g[ij[ok, 1], ij[ok, 0]] = 1.0
        return g

    target = raster(dst, lo, shape)
    best = (-1.0, 0.0, np.zeros(2))
    for deg in range(360):
        r = rotation(math.radians(deg))
        pts = src @ r.T
        slo = pts.min(0)
        size = np.ceil((pts.max(0) - slo) / res).astype(int) + 1
        score = fftconvolve(target, raster(pts, slo, size)[::-1, ::-1], mode='full')
        j, i = np.unravel_index(np.argmax(score), score.shape)
        if score[j, i] > best[0]:
            # full-mode index -> offset of the source raster's origin within the target raster
            shift = np.array([i - (size[0] - 1), j - (size[1] - 1)]) * res
            best = (score[j, i], math.radians(deg), lo + shift - slo)
    return best[1], best[2]


def icp(src, dst, yaw, t, iterations=50):
    tree = cKDTree(dst)
    r = rotation(yaw)
    for _ in range(iterations):
        moved = src @ r.T + t
        d, idx = tree.query(moved)
        keep = d < max(0.3, 3 * np.median(d))
        a, b = src[keep], dst[idx[keep]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        r = (u @ vt).T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = (u @ vt).T
        t = cb - r @ ca
    d, _ = tree.query(src @ r.T + t)
    return r, t, float(np.mean(d < FIT_TOLERANCE))


def main():
    src_map, src_zones, dst_map, out = sys.argv[1:5]
    src, res = walls(src_map)
    dst, _ = walls(dst_map)
    yaw, t = coarse(src, dst, res)
    r, t, fit = icp(src, dst, yaw, t)
    yaw = math.atan2(r[1, 0], r[0, 0])
    print(f'{os.path.basename(src_map)} -> {os.path.basename(dst_map)}: yaw {math.degrees(yaw):.2f}'
          f' deg, t ({t[0]:.3f}, {t[1]:.3f}) m; {100 * fit:.0f}% of the walls within '
          f'{FIT_TOLERANCE} m')

    with open(src_zones) as f:
        zones = yaml.safe_load(f)
    for z in zones['zones']:
        z['centre'] = [round(float(v), 3) for v in r @ np.array(z['centre']) + t]
        z['normal'] = [round(float(v), 4) for v in r @ np.array(z['normal'])]
    with open(out, 'w') as f:
        f.write(f'# {os.path.basename(src_zones)} registered onto {os.path.basename(dst_map)}: '
                f'yaw {math.degrees(yaw):.2f} deg, t ({t[0]:.3f}, {t[1]:.3f}) m, '
                f'fit {100 * fit:.0f}%\n')
        yaml.safe_dump(zones, f, default_flow_style=None, sort_keys=False)
    print(f'{len(zones["zones"])} zones -> {out}')


if __name__ == '__main__':
    main()
