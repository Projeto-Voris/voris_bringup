import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument('gscam2', default_value='true', description='Launch GSCam2 Camera'),
        DeclareLaunchArgument('mavros', default_value='true', description='Launch mavros_control'),
        DeclareLaunchArgument('sonar3d', default_value='true', description='Launch Sonar3D Node'),
        DeclareLaunchArgument('namespace', default_value='bluerov2', description='Namespace of the MAVROS system'),



        # --- Sonar3D ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('sonar3d'), 'launch', 'sonar3d.launch.py'])
            ),
            launch_arguments={'namespace': PathJoinSubstitution([LaunchConfiguration('namespace'),'sonar3d']),
                              'ip': '192.168.2.30',
                              'speed_of_sound': '1491'}.items(),
            condition=IfCondition(LaunchConfiguration('sonar3d'))
        ),

        # --- Câmera GSCam2 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('gscam2'), 'launch', 'node_param_launch.py'])
                ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'image_topic': 'image_raw/compressed',
                'camera_info': 'camera_info'}.items(),
            condition=IfCondition(LaunchConfiguration('gscam2'))
        ),

        # --- MAVROS Control ---
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mavros_control'), 'launch', 'mavros.launch'])
                ),
            launch_arguments={
                'fcu_url': 'tcp://0.0.0.0:5777@',
                'namespace': 'mavros'
            }.items(),
            condition=IfCondition(LaunchConfiguration('mavros'))
        ),
    ])
