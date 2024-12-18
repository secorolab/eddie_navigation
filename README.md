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
