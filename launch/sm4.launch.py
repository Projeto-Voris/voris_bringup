import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join( 
                get_package_share_directory('ros2_fringe_projection'), 'launch'),
                '/fringe_projection.launch.py'
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join( 
                get_package_share_directory('spinnaker_camera_driver'), 'launch'),
                '/sm4_camera.launch.py'
            ])
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['foxglove-studio'],
            output='screen',
        ),
    ])