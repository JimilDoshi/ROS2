from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('multipurpose_rover')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')

    return LaunchDescription([

        Node(
            package='multipurpose_rover',
            executable='rover_enable_server',
            name='rover_enable_server',
            output='screen'
        ),

        Node(
            package='multipurpose_rover',
            executable='rover_can_driver',
            name='rover_can_driver',
            output='screen'
        ),

        Node(
            package='multipurpose_rover',
            executable='rover_can_receiver',
            name='rover_can_receiver',
            output='screen'
        ),

        Node(
            package='multipurpose_rover',
            executable='rover_odom',
            name='rover_odom',
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':    '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id':       'laser',
                'use_sim_time':   False,
                'angle_compensate': True,
                'scan_mode':      'Standard',
            }],
        ),

        # Raspberry Pi Camera v2 — uses libcamera via camera_ros
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'width':     640,
                'height':    480,
                'framerate': 15.0,
            }]
        ),

    ])
