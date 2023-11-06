# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path', description="filename of the path to follow"),
        LifecycleNode(
            package='romea_path_matching',
            executable='path_matching_node',
            name='path_matching',
            exec_name='path_matching',
            namespace='alpo',
            parameters=[{
                'autostart': True,
                'path': LaunchConfiguration('path'),
            }],
            remappings=[
                ('odom', 'localisation/filtered_odom'),
            ],
            # prefix='terminator -x gdbserver --no-startup-with-shell localhost:1337',
        )
    ])
