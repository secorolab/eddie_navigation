from simple_launch import SimpleLauncher
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    # sl.include(
    #     "eddie_navigation",
    #     "navigation.launch.py",
    #     launch_arguments={
    #         "use_sim_time": "true",
    #         "autostart": "true",
    #         "use_respawn": "true",
    #         "params_file": sl.find("eddie_navigation", "nav2_params.yaml", "config"),
    #     },
    # )

    return sl.launch_description()