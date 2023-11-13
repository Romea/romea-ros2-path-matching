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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

import yaml


def load_parameters(filename):
    with open(filename) as file:
        return yaml.safe_load(file.read())


def launch_setup(context, *args, **kargs):
    path_file = LaunchConfiguration('path_file').perform(context)
    path_directory = LaunchConfiguration('path_directory').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    config_file = LaunchConfiguration('configuration_file').perform(context)

    parameters = load_parameters(config_file)
    if len(path_file):
        parameters['path'] = path_file
    parameters['path'] = f"{path_directory}/{parameters['path']}"

    return [
        LifecycleNode(
            package='romea_path_matching',
            executable='path_matching_node',
            name='path_matching',
            exec_name='path_matching',
            namespace=robot_namespace,
            parameters=[parameters],
            remappings=[
                ('odom', 'localisation/filtered_odom'),
            ],
        )
    ]


def generate_launch_description():
    path_description = 'filename of the path to follow. ' \
        'If this parameter is not specified, the path name will be loaded from the config file'
    path_dir_description = 'directory containing the available path files'

    return LaunchDescription([
        DeclareLaunchArgument('path_file', default_value='', description=path_description),
        DeclareLaunchArgument('path_directory', description=path_dir_description),
        DeclareLaunchArgument('robot_namespace', description='namespace used for these nodes'),
        DeclareLaunchArgument('configuration_file', description='YAML file containing ROS params'),
        OpaqueFunction(function=launch_setup),
    ])
