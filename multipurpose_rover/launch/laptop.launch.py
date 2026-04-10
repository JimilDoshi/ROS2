from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('multipurpose_rover')
    slam_params = os.path.join(pkg, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    return LaunchDescription([

        # slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time':      'false',
                'slam_params_file':  slam_params,
            }.items()
        ),

        # rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

        # teleop direction publisher
        Node(
            package='rover_teleop',
            executable='rover_cmd_publisher',
            name='rover_cmd_publisher',
            output='screen',
        ),

        # enable toggle client
        Node(
            package='rover_teleop',
            executable='rover_enable_client',
            name='rover_enable_client',
            output='screen',
        ),

    ])
