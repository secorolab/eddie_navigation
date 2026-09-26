#!/usr/bin/env python3
"""Rasterise a MuJoCo world (primitives, meshes) into a map_server map at the lidars' height.

Usage: world_to_map.py worlds/nav_test.xml maps/nav_test  (needs the mujoco Python package)
"""

import argparse
import pathlib

import mujoco
import numpy as np

# URDF: eddie_base_link 0.2164 above the floor, the laser attachments 0.0682 above it
LASER_Z = 0.2164 + 0.0682
MARGIN = 0.5


def geom_extent(model, data, g):
    kind = model.geom_type[g]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners = data.geom_xpos[g] + (signs * model.geom_size[g]) @ data.geom_xmat[g].reshape(
            3, 3).T
        return corners[:, :2].min(0), corners[:, :2].max(0)
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        m = model.geom_dataid[g]
        verts = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
        world = data.geom_xpos[g] + verts @ data.geom_xmat[g].reshape(3, 3).T
        return world[:, :2].min(0), world[:, :2].max(0)
    r = model.geom_rbound[g]
    return data.geom_xpos[g][:2] - r, data.geom_xpos[g][:2] + r


def inside(model, data, g, points):
    local = (points - data.geom_xpos[g]) @ data.geom_xmat[g].reshape(3, 3)
    size = model.geom_size[g]
    kind = model.geom_type[g]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return np.all(np.abs(local) <= size, axis=-1)
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return (local[..., 0] ** 2 + local[..., 1] ** 2 <= size[0] ** 2) & (
            np.abs(local[..., 2]) <= size[1])
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return np.sum(local ** 2, axis=-1) <= size[0] ** 2
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
    raise SystemExit(f'geom {name or g}: type {kind} is not supported')


def mesh_cut(model, data, g, height):
    """The segments where a mesh geom's triangles cross the plane z = height, as (n, 2, 2)."""
    m = model.geom_dataid[g]
    verts = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
    faces = model.mesh_face[model.mesh_faceadr[m]:model.mesh_faceadr[m] + model.mesh_facenum[m]]
    world = data.geom_xpos[g] + verts @ data.geom_xmat[g].reshape(3, 3).T
    segments = []
    for tri in world[faces]:
        above = tri[:, 2] > height
        if above.all() or not above.any():
            continue
        cut = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            if above[a] != above[b]:
                t = (height - tri[a, 2]) / (tri[b, 2] - tri[a, 2])
                cut.append(tri[a, :2] + t * (tri[b, :2] - tri[a, :2]))
        segments.append(cut[:2])
    return np.array(segments).reshape(-1, 2, 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('world', type=pathlib.Path)
    parser.add_argument('out', type=pathlib.Path, help='output path without extension')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--height', type=float, default=LASER_Z)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.world))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geoms = [g for g in range(model.ngeom) if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
    extents = [geom_extent(model, data, g) for g in geoms]
    res = args.resolution
    lo = np.floor((np.min([e[0] for e in extents], 0) - MARGIN) / res) * res
    hi = np.ceil((np.max([e[1] for e in extents], 0) + MARGIN) / res) * res
    nx, ny = np.round((hi - lo) / res).astype(int)

    x = lo[0] + (np.arange(nx) + 0.5) * res
    y = lo[1] + (np.arange(ny) + 0.5) * res
    xx, yy = np.meshgrid(x, y)
    points = np.stack([xx, yy, np.full_like(xx, args.height)], axis=-1)
    occupied = np.zeros(xx.shape, dtype=bool)
    for g in geoms:
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            occupied |= inside(model, data, g, points)
            continue
        for a, b in mesh_cut(model, data, g, args.height):
            steps = int(np.ceil(np.linalg.norm(b - a) / (0.25 * res))) + 1
            along = a + np.linspace(0.0, 1.0, steps)[:, None] * (b - a)
            ix = np.clip(((along[:, 0] - lo[0]) / res).astype(int), 0, nx - 1)
            iy = np.clip(((along[:, 1] - lo[1]) / res).astype(int), 0, ny - 1)
            occupied[iy, ix] = True

    # PGM rows run from the top (max y); map_server's origin is the bottom-left cell
    image = np.where(occupied, 0, 254).astype(np.uint8)[::-1]
    pgm = args.out.with_suffix('.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{nx} {ny}\n255\n'.encode())
        f.write(image.tobytes())
    args.out.with_suffix('.yaml').write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        f'resolution: {res}\n'
        f'origin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n')
    print(f'{pgm}: {nx} x {ny} cells, origin ({lo[0]:.3f}, {lo[1]:.3f}), '
          f'{occupied.sum()} occupied, sliced at z = {args.height:.4f} m')


if __name__ == '__main__':
    main()
