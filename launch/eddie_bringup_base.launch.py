#!/usr/bin/env python3

import os
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar', default_value='true'
    )
    enable_joypad_arg = DeclareLaunchArgument(
        'enable_joypad', default_value='true'
    )
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam', default_value='false'
    )
    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2', default_value='true'
    )
    map_arg = DeclareLaunchArgument(
        'map_name', default_value='map_test.yaml'
    )
    force_mode_arg = DeclareLaunchArgument(
        'force_mode', default_value='false'
    )
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz', default_value='false'
    )

    joy_config = os.path.join(
        get_package_share_directory('eddie_navigation'), 'config', 'joy_teleop.yaml'
    )

    joy = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[joy_config, {"use_sim_time": False}],
        condition=conditions.IfCondition(LaunchConfiguration("enable_joypad"))
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[joy_config, {"use_sim_time": False}],
        condition=conditions.IfCondition(LaunchConfiguration("enable_joypad"))
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": False}]
    )
    
    static_transform = Node(package="tf2_ros",
                        executable="static_transform_publisher",
                        output="screen",
                        arguments=["0", "0", "-0.2164", "0", "0",
                                    "0", "eddie_base_link", "eddie_base_footprint"],
                        parameters=[{'use_sim_time': False}]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('eddie_navigation'), 'config', 'rviz', 'eddie_rviz.rviz')],
        parameters=[{"use_sim_time": False}],
        condition=conditions.IfCondition(LaunchConfiguration("enable_rviz"))
    )

    print('[INFO] [launch] loading eddie_base driver')
    eddie_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('eddie_base'), 'launch'),
            '/eddie_base.launch.py']),
        launch_arguments={
            "force_mode": LaunchConfiguration("force_mode")
        }.items()
    )

    print('[INFO] [launch] loading Hokuyo laser scanners')
    laser_scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('eddie_navigation'), 'launch'),
            '/urg_node2_2lidar.launch.py']),
        launch_arguments={"use_sim_time": "false"}.items(),
        condition=conditions.IfCondition(LaunchConfiguration("enable_lidar"))
    )
    
    laser_scanner_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('eddie_navigation'), 'launch'),
            '/laser_merger.launch.py']),
        launch_arguments={"use_sim_time": "false"}.items(),
        condition=conditions.IfCondition(LaunchConfiguration("enable_lidar"))
    )
    
    eddie_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('eddie_description'), 'launch'),
            '/load_eddie.launch.py']),
        launch_arguments={"use_sim_time": "false"}.items()
    )

    slam_map_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("eddie_navigation"),
            "launch",
            "slam.launch.py",
        )),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": os.path.join(
                get_package_share_directory("eddie_navigation"),
                "config",
                "mapper_params_online_async.yaml",
            ),
        }.items(),
        condition=conditions.IfCondition(LaunchConfiguration("enable_slam"))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("eddie_navigation"),
            "launch",
            "eddie_nav_bringup.launch.py",
        )),
        launch_arguments={
            "use_sim_time": "false",
            "simulation": "false",
            "map_name": LaunchConfiguration("map_name")
        }.items(),
        condition=conditions.IfCondition(LaunchConfiguration("enable_nav2"))
    )
    
    ld = LaunchDescription()
    ld.add_action(enable_lidar_arg)
    ld.add_action(enable_joypad_arg)
    ld.add_action(enable_slam_arg)
    ld.add_action(enable_nav2_arg)
    ld.add_action(map_arg)
    ld.add_action(force_mode_arg)
    ld.add_action(enable_rviz_arg)
    ld.add_action(eddie_driver)
    ld.add_action(joy)
    ld.add_action(teleop_twist_joy)
    ld.add_action(eddie_description)
    ld.add_action(static_transform)
    ld.add_action(joint_state_publisher)
    ld.add_action(laser_scanner)
    ld.add_action(laser_scanner_merger)
    ld.add_action(slam_map_generator)
    ld.add_action(navigation)
    ld.add_action(rviz)
    
    return ld