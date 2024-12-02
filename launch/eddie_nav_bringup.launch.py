import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    eddie_nav_dir = get_package_share_directory("eddie_navigation")
    map_name = os.environ["ROBOT_ENV"]
    map_file = os.path.join(eddie_nav_dir, "maps", map_name + ".yaml")
    use_sim_time = True

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_file},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [eddie_nav_dir, "/launch/localization.launch.py"]
        ),
        launch_arguments={"use_sim_time": str(use_sim_time)}.items(),
    )

    tf2_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # navigation_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([eddie_nav_dir, "/launch/navigation.launch.py"]),
    #     launch_arguments={"use_sim_time": str(use_sim_time)}.items(),
    # )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("eddie_navigation"),
                "config/rviz",
                "eddie_nav.rviz",
            )
        ],
    )

    print("map_file: ", map_file)

    return LaunchDescription(
        [
            rviz_launch_cmd,
            map_server,
            lifecycle_manager,
            tf2_ros,
            localization_node,
            # navigation_node,
        ]
    )
