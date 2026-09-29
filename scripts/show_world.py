#!/usr/bin/env python3
"""Show a floor-plan mesh (.stl) or an MJCF world (.xml) in an mj_kdl_wrapper scene.

Usage: show_world.py <mesh.stl | world.xml>  (needs the mj_kdl_wrapper Python package)
"""

import argparse
import pathlib
import tempfile

import mj_kdl_wrapper as mjk

# visual only: MuJoCo would collide the mesh's convex hull
WORLD = """<mujoco>
  <asset><mesh name="floorplan" file="{mesh}"/></asset>
  <worldbody>
    <body name="floorplan">
      <geom type="mesh" mesh="floorplan" rgba="0.85 0.85 0.8 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""


def build(path, tmp):
    if path.suffix.lower() == '.xml':
        world = path.resolve()
    else:
        world = pathlib.Path(tmp) / 'floorplan.xml'
        world.write_text(WORLD.format(mesh=path.resolve()))
    obj = mjk.SceneObject()
    obj.name = 'floorplan'
    obj.mjcf_path = str(world)
    obj.fixed = True
    spec = mjk.SceneSpec()
    spec.timestep = 0.002
    spec.add_floor = True
    spec.add_skybox = True
    spec.objects = [obj]
    return mjk.Env.build(spec)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('path', type=pathlib.Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp, build(args.path, tmp) as env:
        env.open_viewer(args.path.name)
        while env.step():
            env.pace()


if __name__ == '__main__':
    main()
