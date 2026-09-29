# eddie_navigation

Mapping, localisation and navigation (nav2) for the Eddie robot, on the real robot and in a
MuJoCo simulation. The base is driven by
[eddie_driver_ros](https://github.com/secorolab/eddie_driver_ros) in both. Doorways are passed
by a behaviour tree plugin and an action server of this package (see [Doors](#doors-zones)).

## Dependencies

ROS 2 Jazzy (Ubuntu 24.04) and, from apt:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-dual-laser-merger ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

From source, in the same workspace:

| Package | For |
|---|---|
| [eddie_driver](https://github.com/secorolab/eddie_driver), [eddie_driver_ros](https://github.com/secorolab/eddie_driver_ros) | the base: robot and MuJoCo |
| [eddie_description](https://github.com/secorolab/eddie_description) | URDF (laser frames) and the MuJoCo model |
| [urg_node2](https://github.com/Hokuyo-aut/urg_node2) (clone with `--recursive`) | the two Hokuyo lidars |
| [hddc2b](https://github.com/secorolab/hddc2b/tree/pst-vel-dist) (`pst-vel-dist`), [robif2b](https://github.com/secorolab/robif2b) | eddie_driver's solvers and EtherCAT; plain CMake, on `CMAKE_PREFIX_PATH` |
| [mj_kdl_wrapper](https://github.com/vamsikalagaturu/mj_kdl_wrapper) (`v0.4.0`) | the MuJoCo simulation only |

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build                       # robot only
EDDIE_SIM=1 colcon build           # + the MuJoCo simulation (eddie_driver, eddie_driver_ros)
source install/setup.bash
```

## Real robot

`eddie_bringup_base.launch.py` starts the base driver (`force_mode`, `impedance_mode` pass
through), the base-only URDF (`eddie_base.urdf.xacro`: the full robot's arm descriptions do not
match the apt kortex/robotiq packages), the two Hokuyos, the laser merger, the joystick and
nav2 with `nav2_params_real.yaml`. The EtherCAT interface is `ethernet_interface`
in eddie_driver's `eddie.yaml`, the lidar addresses are in `config/params_ether*.yaml`, and the
driver's one-time sudoers setup is in its README.

```bash
ros2 launch eddie_navigation eddie_bringup_base.launch.py map_name:=secoro_slam.yaml enable_rviz:=true
```

In RViz, set the robot's pose with *2D Pose Estimate*, then send goals with *2D Goal Pose*.
Keep cables out of the lidar plane: the lidars see them as obstacles.

### Mapping

```bash
ros2 launch eddie_navigation eddie_bringup_base.launch.py enable_slam:=true enable_nav2:=false enable_rviz:=true
ros2 run nav2_map_server map_saver_cli -f src/eddie_navigation/maps/<name> --occ 0.65 --free 0.15 --ros-args -p save_map_timeout:=20.0
```

Drive with the joystick (hold RB) or `ros2 run eddie_driver_ros key_teleop.py --ros-args -r
platform_vel:=/cmd_vel -p ramp:=0.0`, then rebuild so the launch finds the new map. Leave nav2
off while mapping: its map_server would publish `/map` too.

### nav2 setup

- Footprint 0.66 x 0.70 m, no padding; 2.5 cm local costmap.
- Controller: MPPI (Omni motion model, no preferred direction) inside a RotationShimController
  that turns the robot to the goal heading on the spot, in one smooth rotation.
- Planner: NavFn. It plans for a circle (the inscribed radius), which is why doors are passed by
  the zone BT below rather than by nav2.

## MuJoCo simulation

`eddie_sim_nav.launch.py` runs the same stack against the MuJoCo sim: eddie_driver_node with
`io:=mujoco` in `worlds/nav_test.xml` (an 8 x 6 m room, robot at the origin), its simulated
`scan_1st`/`scan_2nd`, the laser merger and nav2 with `nav2_params_real.yaml` on
`maps/nav_test.yaml`. It needs the `EDDIE_SIM` build. Sim and robot share the ROS network: set a
different `ROS_DOMAIN_ID` for the sim when the robot stack is running.

```bash
ros2 launch eddie_navigation eddie_sim_nav.launch.py                 # headless
ros2 launch eddie_navigation eddie_sim_nav.launch.py viewer:=true enable_rviz:=true
```

`world:=secoro` runs in the SeCoRo lab from bim-experiments (`worlds/secoro.xml`: its mesh, seen
by the viewer and the lidars; the robot does not collide with it). `sim_record:=<file.mp4>` records a top view
there; `sim_record_cameras:="top <camera>"` adds cameras of the model, one mp4 each. `nav_waypoints.py` drives the world's preset
waypoints (or `--goal X Y YAW`, repeated) and shows them on `/nav_goals`:

```bash
ros2 run eddie_navigation nav_waypoints.py --world secoro
```

The secoro mesh and map are bim-experiments' `gen/3d-mesh/uni-bremen_secoro.stl` and
`gen/maps/uni-bremen_secoro.pgm`, copied; the map's origin is shifted by the body pos in
`worlds/secoro.xml`. The nav_test map is the world sliced at the lidars' height; regenerate it
after editing the world (needs the `mujoco` Python package):

```bash
python3 scripts/world_to_map.py worlds/nav_test.xml maps/nav_test
```

To look at a floor-plan mesh or a world on its own, `show_world.py` builds an mj_kdl_wrapper scene
around it (floor, skybox; a mesh is visual only) and opens the viewer; it needs the
`mj_kdl_wrapper` Python package:

```bash
python3 scripts/show_world.py worlds/meshes/uni-bremen_secoro.stl
python3 scripts/show_world.py worlds/secoro.xml
```

The mesh is static: doors in it cannot open.

## Doors (zones)

Doors are zones in `maps/<map>_zones.yaml`, one file per map; the robot and sim launches pick up
the file that matches `map_name` / `world`. Each zone has a `name`, `centre`, `normal`, `width`
and `constraints` (`eddie_navigation/msg/MotionConstraints`: `heading_mode` `along` / `forward`
/ `any`, `max_speed`, `align_tolerance_xy` / `_yaw`, `stop_time`).

nav2's BT (`behavior_trees/navigate_to_pose_zones.xml`) finds the first zone on the path
(`ZoneOnPath`), drives in front of it (skipped when the robot is already there) and hands it
to the `constrained_pass` server (`ConstrainedPass.action`). The server measures the gap the
lidars see, turns on the spot to face through or back through (`along`), slides into line and
drives straight through; it stops on anything inside the opening within `stop_time`, checked
against the real base outline (`check_footprint`), the jambs being the door. Then nav2 replans
to the goal.

Three ways to make a zone file:

```bash
# from the BIM (bim-experiments FPM model; needs scenery_builder's `fpm` Python package)
python3 scripts/zones_from_fpm.py \
  ../bim-experiments/environments/secorolab/gen/json-ld/fpm worlds/secoro.xml maps/secoro_zones.yaml
# by hand on a map: click both jambs of each door, Enter saves (doors squared to the walls)
python3 scripts/draw_zones.py maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
# onto another map of the same building, by registering the two maps' walls
python3 scripts/zones_to_map.py maps/secoro.yaml maps/secoro_zones.yaml maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
```

Rebuild eddie_navigation after changing a zone file.
