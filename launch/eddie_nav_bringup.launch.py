import os

from simple_launch import SimpleLauncher
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    eddie_nav_dir = get_package_share_directory("eddie_navigation")

    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg("use_sim_time", "true")
    sl.declare_arg("simulation", "true")
    sl.declare_arg("map_name", "map_trial_1.yaml")

    sl.add_action(LogInfo(msg=["using map: ", sl.arg("map_name")]))

    sl.node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": sl.arg("use_sim_time")},
            {"yaml_filename": sl.find("eddie_navigation", sl.arg("map_name"), "maps")},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    sl.node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": sl.arg("use_sim_time")},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    sl.node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        parameters=[{"use_sim_time": sl.arg("use_sim_time")}],
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    

    with sl.group(if_arg="simulation"):

        sl.include(
            "eddie_navigation",
            "localization.launch.py",
            launch_arguments={"use_sim_time": sl.arg("use_sim_time")},
        )

        sl.include(
            "eddie_navigation",
            "navigation.launch.py",
            launch_arguments={"use_sim_time": sl.arg("use_sim_time")},
        )
        
        sl.node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": sl.arg("use_sim_time")}],
            arguments=[
                "-d",
                sl.find("eddie_navigation", "eddie_nav.rviz", "config/rviz"),
            ],
        )

    with sl.group(unless_arg="simulation"):
        sl.include(
            "eddie_navigation",
            "localization.launch.py",
            launch_arguments={
                "use_sim_time": sl.arg("use_sim_time"),
                "params_file": "nav2_params_real.yaml"},
        )

        sl.include(
            "eddie_navigation",
            "navigation.launch.py",
            launch_arguments={
                "use_sim_time": sl.arg("use_sim_time"),
                "params_file": os.path.join(eddie_nav_dir, "config", "nav2_params_real.yaml")},
        )

    return sl.launch_description()
