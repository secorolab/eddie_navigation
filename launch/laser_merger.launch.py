# Copyright 2024 pradyum
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    dual_laser_merger_node = ComposableNodeContainer(
        name='laser_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'laser_1_topic': '/scan_1st'},
                    {'laser_2_topic': '/scan_2nd'},
                    {'merged_scan_topic': '/scan'},
                    {'target_frame': 'eddie_base_link'},
                    {'laser_1_x_offset': 0.0},
                    {'laser_1_y_offset': 0.0},
                    {'laser_1_yaw_offset': 0.0},
                    {'laser_2_x_offset': 0.0},
                    {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': 0.0},
                    # time tolerance for merging laser scans
                    {'tolerance': 0.01},
                    {'queue_size': 25},
                    {'angle_increment': 0.00436},
                    {'scan_time': 0.025},
                    {'range_min': 0.06},
                    {'range_max': 10.0},
                    {'min_height': -0.3},
                    {'max_height': 0.3},
                    {'angle_min': -3.141592654},
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': True},
                    # Formula: actual_radius = (allowed_radius/range_max) * distance_from_robot.
                    # parameter of shadowfilter, which is used to remove outliers. It increases proportionately to the distance of the laser point from the robot
                    {'allowed_radius': 0.4},
                    {'enable_shadow_filter': True},
                    {'enable_average_filter': True},
                    ],
            )
        ],
        output='screen',
    )

    ld.add_action(dual_laser_merger_node)

    return ld
