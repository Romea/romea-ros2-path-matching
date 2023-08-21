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
