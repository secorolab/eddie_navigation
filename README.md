# Navigation of Eddie robot in Gazebo

This repository provides configured ROS2 mapping, localisation, and navigation packages for the Eddie robot in Gazebo
simulator.

## ROS and Gazebo

- **ROS Distribution**: Jazzy (on Ubuntu 24.04) -
  [Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- **Gazebo Version**: Harmonic

### Linked Libraries and Repositories

Install following libraries

  ```bash
  sudo apt-get install ros-jazzy-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers libgsl-dev
  ```

The following repositories were tested with specific versions or commits:

- [eddie_navigation](https://github.com/secorolab/eddie_navigation.git): `main` branch
- [eddie_description](https://github.com/secorolab/eddie_description.git): `main` branch
- [eddie_gazebo](https://github.com/secorolab/eddie_gazebo.git): `main` branch
- [kelo_interfaces](https://github.com/secorolab/kelo_interfaces): `main` branch
- [kelo_tulip](https://github.com/secorolab/kelo_tulip): `ros2` branch

## Instructions to Start Navigation

1. Run eddie in simulation

    ```bash
    ros2 launch eddie_gazebo run_sim.launch.py use_kelo_tulip:=true
    ```

2. If map of the arena is avaialble, skip to next step, or

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

3. Set the environment variable to `map_name` in the terminal where the navigation node will be run

    ```bash
    export ROBOT_ENV=map_name
    ```

4. Run the navigation launch file

    ```bash
    ros2 launch eddie_navigation eddie_nav_bringup.launch.py
    ```

5. The topic `/goal_pose` of `geometry_msgs/msg/PoseStamped` type is available to get goal pose
