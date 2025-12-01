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
        DeclareLaunchArgument('ping360', default_value='true', description='Launch Ping360 Sonar'),
        DeclareLaunchArgument('gscam2', default_value='true', description='Launch GSCam2 Camera'),
        DeclareLaunchArgument('mavros', default_value='true', description='Launch mavros_control'),
        DeclareLaunchArgument('namespace', default_value='bluerov2', description='Namespace of the MAVROS system'),


        # --- Sonar Ping360 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ping360_sonar'), 'launch', 'ping360_bringup.launch.py'])
                ),
                launch_arguments={'namespace': LaunchConfiguration('namespace'),
                                  'connection_type': 'udp',
                                  'udp_address': '0.0.0.0',
                                  'frame': 'ping360_link',
                                  'publish_echo': 'false',
                                  'publish_image': 'false',
                                  'baudnrate': '115200',
                                  }.items(),
                condition=IfCondition(LaunchConfiguration('ping360'))
        ),

        # --- Câmera GSCam2 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('gscam2'), 'launch', 'node_param_launch.py'])
                ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'image_topic': 'image_raw',
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
