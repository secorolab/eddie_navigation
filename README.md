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
| `bim_experiments` | owns scenario worlds, the MuJoCo host, infrastructure dynamics and the simulation plugin interface |
| [mj_kdl_wrapper](https://github.com/vamsikalagaturu/mj_kdl_wrapper) | MuJoCo scene construction and physics, required by the simulator and simulation backends |

## Build

Run these commands from the workspace root after installing/building the required
dependencies. Hardware and simulation are selected at build time for the Eddie
packages and `bim_experiments` using the same `EDDIE_SIM` option.

### Hardware build

```bash
source /opt/ros/jazzy/setup.bash
# If the workspace already has installed dependencies:
source install/setup.bash

EDDIE_SIM=0 colcon build \
  --packages-select eddie_driver eddie_driver_ros bim_experiments eddie_navigation \
  --cmake-clean-cache --cmake-args -DEDDIE_SIM=OFF

source install/setup.bash
```

This disables the driver's MuJoCo backend, the shared-world adapter, the Eddie
simulation plugin and the scenario simulator in `bim_experiments`. The real-robot launch uses the `robif2b` hardware backend.

### Simulation build

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

EDDIE_SIM=1 colcon build \
  --packages-select eddie_driver eddie_driver_ros bim_experiments eddie_navigation \
  --cmake-clean-cache --cmake-args -DEDDIE_SIM=ON

source install/setup.bash
```

This also builds the MuJoCo backend in `eddie_driver`, its adapter in
`eddie_driver_ros`, and `eddie_navigation/EddieSimulationPlugin`. Hardware support
remains compiled; choose the hardware or simulation launch at runtime.

The environment variable controls conditional package dependencies; the CMake
argument explicitly sets the cached build option. Omitting the variable on a later
build does not turn off an existing cached `EDDIE_SIM=ON`. Use the explicit OFF
command when switching to hardware-only, or maintain separate workspaces for the
robot and simulation to keep their installed artifacts separate.

A machine without a display can still run simulation with `viewer:=false` and
`enable_rviz:=false`, but it needs the simulation libraries. With `EDDIE_SIM=0` and `-DEDDIE_SIM=OFF`,
`bim_experiments` also excludes its simulator, plugin interface and dynamics;
its `mj_kdl_wrapper` and `pluginlib` dependencies are conditional. The hardware
build above therefore includes `bim_experiments` to configure it consistently.
A whole-workspace build can still select simulation-only packages present in
`src/`; use the package selection above to build these packages only.

## Real robot

`eddie_bringup_base.launch.py` starts the base driver (`force_mode`, `impedance_mode` pass
through), the base-only URDF (`eddie_base.urdf.xacro`: the full robot's arm descriptions do not
match the apt kortex/robotiq packages), the two Hokuyos, the laser merger, the joystick and
nav2 with `nav2_params_real.yaml`. The EtherCAT interface is `ethernet_interface`
in eddie_driver's `eddie.yaml`, the lidar addresses are in `config/params_ether*.yaml`, and the
driver's one-time sudoers setup is in its README.

```bash
ros2 launch eddie_navigation eddie_bringup_base.launch.py map_name:=secoro_slam.yaml enable_rviz:=false
```

`enable_rviz:=false` is suitable for a robot without a display. Use
`enable_rviz:=true` on a machine with a graphical session.

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

`eddie_sim_nav.launch.py` includes the scenario launch from `bim_experiments` and
enables the optional Eddie plugin. Use the simulation build above first. Nav2 uses
wall-clock ROS timestamps as before; the infrastructure dynamics use MuJoCo time
from the `/sim` state messages. Set a different `ROS_DOMAIN_ID` in all simulation
terminals if the real robot stack is running on the same ROS network.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# Default test room, no graphical interfaces:
ros2 launch eddie_navigation eddie_sim_nav.launch.py \
  world:=nav_test viewer:=false enable_rviz:=false

# Laboratory with articulated doors, buttons and the test elevator:
ros2 launch eddie_navigation eddie_sim_nav.launch.py \
  world:=secoro_doors viewer:=true enable_rviz:=true
```

`nav_test` is an 8 x 6 m room with Eddie at the origin. Both launches provide the
simulated `scan_1st`/`scan_2nd`, laser merging, odometry and Nav2 on the matching
map. `viewer` controls the MuJoCo window; `enable_rviz` controls RViz independently.

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
python3 scripts/world_to_map.py ../bim-experiments/worlds/nav_test.xml maps/nav_test
```

To look at a floor-plan mesh or a world on its own, `show_world.py` builds an mj_kdl_wrapper scene
around it (floor, skybox; a mesh is visual only) and opens the viewer; it needs the
`mj_kdl_wrapper` Python package:

```bash
python3 scripts/show_world.py ../bim-experiments/worlds/meshes/uni-bremen_secoro.stl
python3 scripts/show_world.py ../bim-experiments/worlds/secoro.xml
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
  ../bim-experiments/environments/secorolab/gen/json-ld/fpm ../bim-experiments/worlds/secoro.xml maps/secoro_zones.yaml
# by hand on a map: click both jambs of each door, Enter saves (doors squared to the walls)
python3 scripts/draw_zones.py maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
# onto another map of the same building, by registering the two maps' walls
python3 scripts/zones_to_map.py maps/secoro.yaml maps/secoro_zones.yaml maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
```

Rebuild eddie_navigation after changing a zone file.


### Automatic doors in secoro_doors

`ros2 launch eddie_navigation eddie_sim_nav.launch.py world:=secoro_doors viewer:=true`
starts `bim_experiments/simulation_dynamics` automatically, loading
`bim_experiments/config/secoro_doors_dynamics.yaml`. That file associates
each slide-button joint with a door hinge/position actuator and its cycle timings.
Add one entry per automatic door; no driver source change is needed. Other worlds
leave automatic-door behavior disabled. Rebuild `bim_experiments` after editing
the configuration unless using a symlink install. The XML defines the bodies and
actuators; the YAML defines their associations and behavior.

The same YAML declares `manual_doors` with type `manual_door`. Their MJCF latches
start locked and are released by `simulation_dynamics` only when the closed leaf's
handle is fully lowered. Open leaves remain movable without holding the handle;
closing with a released handle locks them again. The automatic 1627 is excluded.

The scenario also contains a fixed elevator test entrance south of the building,
at `(3, -4)`, facing the laboratory. Its two telescopic leaves and external call
button are controlled by `bim_experiments/simulation_dynamics` from the `elevators`
section of `secoro_doors_dynamics.yaml`. The map extends to Y=-6.6 for the test area;
static walls are mapped, moving leaves remain dynamic obstacles from the lidars.


## Scenario ownership and optional Eddie plugin

There is one compiled MuJoCo world, shared by the scenario and the robot:

```text
scenario_simulator (bim_experiments; one process and one MuJoCo Env)
├── Building, doors, buttons and elevator
├── Physics loop and /sim state/command interface
└── EddieSimulationPlugin (loaded from eddie_navigation)
    └── Existing driver node: wheel control, odometry and simulated lasers

simulation_dynamics (bim_experiments; separate ROS process)
└── Infrastructure behaviors using the /sim interface

Nav2 and robot-state publishers (started by eddie_sim_nav.launch.py)
```

`bim_experiments` owns world XMLs and meshes under `worlds/`, the host executable
`scenario_simulator`, and `scenario.launch.py`. Nav2 maps, door zones and navigation
configuration remain in `eddie_navigation`. Per-instance infrastructure behavior
is configured in `bim_experiments/config/secoro_doors_dynamics.yaml`.

The plugin is a C++ library discovered with `pluginlib`, not another simulator or
process. Its declaration is in `simulation_plugins.xml`; its implementation and
header are `src/simulation/eddie_simulation_plugin.cpp` and
`include/simulation/eddie_simulation_plugin.hpp`. It implements the common
`bim_experiments` interface `include/simulation/simulation_plugin.hpp`:

1. `configure` adds Eddie's model and actuators before the world is compiled.
2. `initialize` connects the existing driver to the host's borrowed `mj_kdl::Env`.
3. `afterStep` schedules driver updates at the configured control period.
4. `reset` restores the robot and controller when the world resets.
5. `nodes` adds the driver ROS node to the host's single-threaded executor.

The driver code is reused through `eddie_driver_ros`'s simulation adapter; wheel
conventions, control, odometry, simulated lasers, plan display and robot recording
stay in that implementation. With the adapter, the driver creates no physics timer
or second world. The host alone advances physics at 1 ms by default. Measurements
and robot commands are exchanged directly in memory, not over ROS at each physics
step. The `/cmd_vel`, `/odom`, TF and scan interfaces remain available to navigation.

`eddie_sim_nav.launch.py` includes `bim_experiments/scenario.launch.py` and passes
`plugins:=eddie_navigation/EddieSimulationPlugin`, driver parameters, control mode,
start pose and recording options. The scenario launch also starts infrastructure
dynamics for `world:=secoro_doors`. No robot plugin is loaded unless requested.

To run only the building and its dynamics, without Eddie or Nav2:

```bash
ros2 launch bim_experiments scenario.launch.py world:=secoro_doors viewer:=true
```

That standalone launch requires `bim_experiments` built with `EDDIE_SIM=ON`,
but does not require the Eddie plugin or robot-specific packages.
The real-robot bringup continues to use the hardware driver directly and does not
start the scenario simulator or load a simulation plugin.
