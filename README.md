# Navigation of Eddie robot

> [!NOTE]
> This repository provides configured ROS2 mapping, localisation, and navigation packages for the
Eddie robot in Gazebo simulator.

## ROS and Gazebo

- **ROS Distribution**: Jazzy (on Ubuntu 24.04) -
  [Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- **Gazebo Version**: Harmonic

### Dependencies

#### Nav2

  ```bash
  sudo apt-get install ros-jazzy-navigation2 \
                        ros-jazzy-nav2-bringup \
                        ros-jazzy-slam-toolbox
  ```

#### Eddie Gazebo

- Clone the [main](https://github.com/secorolab/eddie_gazebo.git) branch into the workspace

  > Also, install the dependencies of the package

- Clone the `eddie_navigation` package into the workspace

  ```bash
  cd ~/eddie_ws/src

  git clone https://github.com/secorolab/eddie_navigation.git
  ```

## Build and source the workpace

```bash
cd ~/eddie_ws

colcon build

source install/setup.bash
```

## Run Navigation

1. Launch the `eddie` in simulation

    ```bash
    ros2 launch eddie_gazebo run_sim.launch.py use_kelo_tulip:=true
    ```

2. If map of the arena is avaialble, skip to next step, otherwise follow the steps to create a map

    a. start mapping using `slam_toolbox`

    ```bash
    ros2 launch eddie_navigation online_async_slam.launch.py
    ```

    b. Use teleop to move the robot around

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    c. After mapping, save the map using following command from `eddie_navigation/maps/` path

    ```bash
    ros2 run nav2_map_server map_saver_cli -f map_name --occ 0.65 --free 0.15 --ros-args -p save_map_timeout:=20.0
    ```

3. Run the navigation launch file

    ```bash
    ros2 launch eddie_navigation eddie_nav_bringup.launch.py map_name:=<map_name>.yaml
    ```

    - The maps are available in [maps](maps) directory

4. The topic `/goal_pose` of `geometry_msgs/msg/PoseStamped` type is available to get goal pose

## Real robot

`eddie_bringup_base.launch.py` starts the base driver from
[eddie_driver_ros](https://github.com/secorolab/eddie_driver_ros) (`force_mode`,
`impedance_mode` pass through), the base-only URDF (`eddie_base.urdf.xacro`: the full robot's
arm descriptions do not match the apt kortex/robotiq packages), the two Hokuyos
([urg_node2](https://github.com/Hokuyo-aut/urg_node2), from source), the laser merger, the
joystick and nav2 with `nav2_params_real.yaml`. The EtherCAT interface is `ethernet_interface`
in eddie_driver's `eddie.yaml`, the lidar addresses are in `config/params_ether*.yaml`, and the
driver's one-time sudoers setup is in its README.

```bash
ros2 launch eddie_navigation eddie_bringup_base.launch.py map_name:=secoro_slam.yaml enable_rviz:=true
```

Depends on, besides nav2 and slam_toolbox: eddie_driver_ros and
[eddie_driver](https://github.com/secorolab/eddie_driver),
[eddie_description](https://github.com/secorolab/eddie_description), urg_node2 (source), and from
apt `ros-jazzy-dual-laser-merger`, `ros-jazzy-joy`, `ros-jazzy-teleop-twist-joy`. The MuJoCo
simulation below also needs eddie_driver(_ros) built with `EDDIE_SIM` and
[mj_kdl_wrapper](https://github.com/vamsikalagaturu/mj_kdl_wrapper) (`feat/v0.4.0`).

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
`maps/nav_test.yaml`. Build eddie_driver and eddie_driver_ros with `EDDIE_SIM` (see their
READMEs).

```bash
ros2 launch eddie_navigation eddie_sim_nav.launch.py                 # headless
ros2 launch eddie_navigation eddie_sim_nav.launch.py viewer:=true enable_rviz:=true
```

`world:=secoro` runs in the SeCoRo lab from bim-experiments (`worlds/secoro.xml`: its mesh, seen
by the viewer and the lidars; the robot does not collide with it). `sim_record:=<file.mp4>` records the torso
camera there and a top view to `<file>_top.mp4`. `nav_waypoints.py` drives the world's preset
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
# from the BIM (bim-experiments FPM model; the workspace venv has scenery_builder)
../../.venv/bin/python scripts/zones_from_fpm.py \
  ../bim-experiments/environments/secorolab/gen/json-ld/fpm worlds/secoro.xml maps/secoro_zones.yaml
# by hand on a map: click both jambs of each door, Enter saves (doors squared to the walls)
python3 scripts/draw_zones.py maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
# onto another map of the same building, by registering the two maps' walls
python3 scripts/zones_to_map.py maps/secoro.yaml maps/secoro_zones.yaml maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
```

Rebuild eddie_navigation after changing a zone file.
