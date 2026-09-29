#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_dir = get_package_share_directory('eddie_navigation')

    def include(package, launch_file, arguments, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', launch_file)),
            launch_arguments=arguments.items(), **kwargs)

    # the base spawns at the map origin with this heading, and amcl starts from the same pose
    start_yaw = PythonExpression(
        ["{'nav_test': '0.0', 'secoro': '1.5708'}['", LaunchConfiguration('world'), "']"])

    # The driver runs on a wall timer in real time, so nothing here uses sim time.
    eddie_driver = include('eddie_driver_ros', 'eddie_driver.launch.py', {
        'io': 'mujoco',
        'viewer': LaunchConfiguration('viewer'),
        'force_mode': LaunchConfiguration('force_mode'),
        'impedance_mode': LaunchConfiguration('impedance_mode'),
        'sim_world': PathJoinSubstitution(
            [nav_dir, 'worlds', [LaunchConfiguration('world'), '.xml']]),
        'sim_record': LaunchConfiguration('sim_record'),
        'sim_record_cameras': LaunchConfiguration('sim_record_cameras'),
        'sim_start_pose': ['0 0 ', start_yaw],
    })

    # base only, as the sim; eddie_robot.urdf.xacro fails against the apt kortex_description
    urdf = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(get_package_share_directory('eddie_description'), 'urdf',
                     'eddie_base.urdf.xacro'),
        ' robot_prefix:=eddie_']), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': False}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.2164', '0', '0', '0',
                   'eddie_base_link', 'eddie_base_footprint'],
        parameters=[{'use_sim_time': False}],
    )

    laser_merger = include('eddie_navigation', 'laser_merger.launch.py',
                           {'use_sim_time': 'false'})

    navigation = include('eddie_navigation', 'eddie_nav_bringup.launch.py', {
        'use_sim_time': 'false',
        'simulation': 'false',
        'map_name': [LaunchConfiguration('world'), '.yaml'],
        'initial_yaw': start_yaw,
        'zones_file': PythonExpression(
            ["{'nav_test': '', 'secoro': '", os.path.join(nav_dir, 'maps', 'secoro_zones.yaml'),
             "'}['", LaunchConfiguration('world'), "']"]),
    })

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav_dir, 'config', 'rviz', 'eddie_nav.rviz')],
        parameters=[{'use_sim_time': False}],
        # Ogre's GLX window fails under Wayland: run rviz on XWayland
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=conditions.IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='nav_test', choices=['nav_test', 'secoro'],
                              description='worlds/<world>.xml, localized on maps/<world>.yaml'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='Open the MuJoCo viewer'),
        DeclareLaunchArgument('force_mode', default_value='false',
                              description='Force mode (a wrench on platform_force)'),
        DeclareLaunchArgument('impedance_mode', default_value='false',
                              description='Impedance mode (a twist on /cmd_vel, torques out)'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('sim_record', default_value='',
                              description='Record sim_record_cameras to this mp4'),
        DeclareLaunchArgument('sim_record_cameras', default_value='top',
                              description='Cameras to record, space-separated (see '
                                          'eddie_driver_ros)'),
        eddie_driver,
        robot_state_publisher,
        joint_state_publisher,
        static_transform,
        laser_merger,
        navigation,
        rviz,
    ])
