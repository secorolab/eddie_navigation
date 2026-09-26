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
[eddie_driver_ros](../eddie_driver_ros) (`force_mode`, `impedance_mode` pass through), the two
Hokuyos (urg_node2), the laser merger, the joystick and nav2 with `nav2_params_real.yaml`:

```bash
ros2 launch eddie_navigation eddie_bringup_base.launch.py map_name:=<map_name>.yaml
```

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

`world:=secoro` runs in the SeCoRo lab from bim-experiments (`worlds/secoro.xml`: its IFC mesh
for show, `worlds/secoro_walls.xml` for collision). `sim_record:=<file.mp4>` records the torso
camera there and a top view to `<file>_top.mp4`. `nav_waypoints.py` drives the world's preset
waypoints (or `--goal X Y YAW`, repeated) and shows them on `/nav_goals`:

```bash
ros2 run eddie_navigation nav_waypoints.py --world secoro
```

The map is the world sliced at the lidars' height; regenerate it after editing the world, and
the collision boxes after changing the mesh (needs the `mujoco` Python package):

```bash
python3 scripts/world_to_map.py worlds/nav_test.xml maps/nav_test
python3 scripts/mesh_to_walls.py worlds/meshes/uni-bremen_secoro.stl worlds/secoro_walls.xml
python3 scripts/world_to_map.py worlds/secoro.xml maps/secoro
```
