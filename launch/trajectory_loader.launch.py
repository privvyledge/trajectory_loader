# Copyright 2023 Amadeusz Szymko
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    container = ComposableNodeContainer(
        name='trajectory_loader_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='trajectory_loader',
                    plugin='trajectory_loader::TrajectoryLoaderNode',
                    namespace='planning',
                    name='racing_planner',
                    parameters=[
                        LaunchConfiguration('param_file'),
                        {
                            'csv_path': LaunchConfiguration('csv_path')
                        }
                    ],
                    remappings=[
                        ('~/output/trajectory', 'racing_planner/trajectory')
                    ]
                ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    return [
        container
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    pkg_prefix = FindPackageShare('trajectory_loader')
    add_launch_arg('param_file', PathJoinSubstitution([pkg_prefix, 'config/defaults.param.yaml']))
    add_launch_arg('csv_path', PathJoinSubstitution([pkg_prefix, 'data/imola.csv']))
    
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
