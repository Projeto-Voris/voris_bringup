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
        DeclareLaunchArgument('cameras_on', default_value='True', description='Open Pattern Projection process?'),
        DeclareLaunchArgument('inv_triang', default_value='True', description='Open Inverse Triangulation process?'),
        DeclareLaunchArgument('description', default_value='True', description='Open description process?'),
        DeclareLaunchArgument('motor_topic', default_value='motor/angle', description='Topic of stepper motor angle'),
        DeclareLaunchArgument('num_images', default_value='10', description='Number of images to acquire'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm3_camera.launch.py'])
                ]),
            condition=IfCondition(LaunchConfiguration('cameras_on'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch','gpio_control.launch.py'])
            ]),
            launch_arguments = {'namespace':'SM3',
                                'stepping_mode': 'full',
                                'step_delay': '20',
                                'steps_per_rev': '2048',
                                'motor_angle_topic': LaunchConfiguration('motor_topic')}.items(),

        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch',
                'inverse_triangulation.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('inv_triang')),
            launch_arguments = {'namespace':'SM3',
                            'left_image': 'left/image_raw',
                            'right_image': 'right/image_raw',
                            'motor_topic': LaunchConfiguration('motor_topic'),
                            'n_images': LaunchConfiguration('num_images')}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch',
                'lab_visualize.launch.py'])
            ]),
            condition=IfCondition(LaunchConfiguration('description'))
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        )
    ])