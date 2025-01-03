import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        DeclareLaunchArgument('pattern', default_value='True', description='Open Pattern Projection process?'),
        DeclareLaunchArgument('inverse_triangulation', default_value='False', description='Open Inverse Triangulation process?'),
        DeclareLaunchArgument('description', default_value='False', description='Open description process?'),
        
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution([
        #         FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm3_camera.launch.py'])
        #         ])
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch',
                'noise_display.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments = {'namespace':'SM2',
                                'service_topic': 'pattern_change',
                                'monitor_name': 'Monitor_1'}.items(),

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch',
                'stereo_acquisition.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('pattern')),
            launch_arguments = {'namespace':'SM3',
                                'left_topic': '/SM2/left/image_raw',
                                'right_topic': '/SM2/right/image_raw',
                                'service_topic': 'pattern_change',
                                'n_images': '10'}.items(),
        ),
    ])