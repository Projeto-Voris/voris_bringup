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
        DeclareLaunchArgument('cameras', default_value='True', description='Open Pattern Projection process?'),
        DeclareLaunchArgument('slam', default_value='True', description='Open ORBSLAM?'),
        DeclareLaunchArgument('inertial', default_value='False', description='Open IMU data?'),
        DeclareLaunchArgument('disparity', default_value='False', description='Open Disparity process?'),
        DeclareLaunchArgument('description', default_value='True', description='Open Description process?'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm2_color_camera.launch.py'])]),
                launch_arguments = {'namespace': 'Passive',
                                    'cam_0_frame_id': 'Passive/left_camera_link',
                                    'cam_1_frame_id': 'Passive/right_camera_link',
                                    'camera_type': 'color',
                                    'cam0_serial': '22548025',
                                    'cam1_serial': '22548033'}.items(),
            condition=IfCondition(LaunchConfiguration('cameras'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'), 'launch', 'stereo_color.launch.py'])]),
            launch_arguments={
                'namespace':'Passive',
                'pangolin': "False",
                'rescale': 'True',
                'child_frame_id': 'Passive/left_camera_link',
                'parent_frame_id': 'base_link',
                'frame_id': 'orbslam3'}.items(),
            condition=IfCondition(LaunchConfiguration('slam'))
        ),        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'), 'launch', 'stereo_inertial.launch.py'])]),
            launch_arguments={
                'namespace':'Passive',
                'pangolin': "False",
                'rescale': 'True',
                'child_frame_id': 'Passive/left_camera_link',
                'parent_frame_id': 'base_link',
                'frame_id': 'orbslam3'}.items(),
            condition=IfCondition(LaunchConfiguration('inertial'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('driver_stim300'), 'launch', 'stim300_driver.launch.py'])]),
            launch_arguments={'namespace': 'Passive',}.items(),
            condition=IfCondition(LaunchConfiguration('inertial'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('passive_stereo'), 'launch',
                'triangulation.launch.py'])
            ]),
            launch_arguments={'namespace': 'Passive',
                              'yaml_file_disp': 'sm2_20EM4-C.yaml'}.items(),
            condition=IfCondition(LaunchConfiguration('disparity'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'mobile_bench.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            namespace='Passive',
            name='foxglove_bridge',
            output='log'
        )
    ])