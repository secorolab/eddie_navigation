from simple_launch import SimpleLauncher
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    # launch eddie simulation
    sl.include('eddie_gazebo', 'run_sim.launch.py')

    # rviz
    sl.rviz(sl.find("eddie_navigation", "eddie_gz.rviz", "config/rviz"))

    # localization
    sl.node('robot_localization', 'ekf_node', parameters=[sl.find("eddie_navigation", "ekf.yaml", "config")])


    return sl.launch_description()
