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
        DeclareLaunchArgument('disparity', default_value='True', description='Open disparity process?'),
        DeclareLaunchArgument('slam', default_value='True', description='Open SLAM process?'),
        DeclareLaunchArgument('description', default_value='False', description='Open description process?'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm2_camera.launch.py'])
                ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'), 'launch',
                'stereo.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments = {'pangolin':"False"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('passive_stereo'), 'launch',
                'triangulation.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('disparity'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch',
                'voris_visualize.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('description'))
        ),

    ])