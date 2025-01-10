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
        DeclareLaunchArgument('cameras_on', default_value='False', description='Open Pattern Projection process?'),
        DeclareLaunchArgument('inverse_triangulation', default_value='False', description='Open Inverse Triangulation process?'),
        DeclareLaunchArgument('description', default_value='False', description='Open description process?'),
        DeclareLaunchArgument('motor_topic', default_value='motor/angle', description='Topic of stepper motor angle'),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution([
        #         FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm4_camera.launch.py'])
        #         ]),
        #     condition=IfCondition(LaunchConfiguration('cameras_on'))
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch','gpio_control.launch.py'])
            ]),
            launch_arguments = {'namespace':'SM3',
                                'stepping_mode': 'full',
                                'step_delay': '4',
                                'steps_per_rev': '2048',
                                'motor_angle_topic': LaunchConfiguration('motor_topic')}.items(),

        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch',
                'inverse_triangulation.launch.py'])
            ]),
            launch_arguments = {'namespace':'SM3',
                            'left_image': '/SM4/left/image_raw',
                            'right_image': '/SM4/right/image_raw',
                            'left_camera_info': '/SM4/left/camera_info',
                            'right_camera_info': '/SM4/right/camera_info',
                            'motor_topic': LaunchConfiguration('motor_topic'),
                            'n_images': '10'}.items(),
        ),
    ])