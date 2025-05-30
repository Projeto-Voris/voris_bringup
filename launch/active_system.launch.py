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
        DeclareLaunchArgument('sm3', default_value='True', description='Open Inverse Triangulation process?'),
        DeclareLaunchArgument('sm4', default_value='True', description='Open description process?'),
        DeclareLaunchArgument('namespace', default_value='Active', description='Namespace of system'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm4_camera.launch.py'])
                ]),
                launch_arguments = {'namespace': LaunchConfiguration('namespace'),
                                    'cam_0_frame_id': 'Active/left_camera_link',
                                    'cam_1_frame_id': 'Active/right_camera_link'}.items(),
            condition=IfCondition(LaunchConfiguration('cameras'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch','gpio_control.launch.py'])
            ]),
            launch_arguments = {'namespace':LaunchConfiguration('namespace'),
                                'stepping_mode': 'full',
                                'step_delay': '10',
                                'steps_per_rev': '1024',
                                'motor_angle_topic': 'motor/angle'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch',
                'inverse_triangulation.launch.py'])
            ]),
            launch_arguments = {'namespace':LaunchConfiguration('namespace'),
                            'left_image': 'left/image_raw',
                            'right_image': 'right/image_raw',
                            'motor_topic': 'motor/angle',
                            'camera_frame_id': 'Active/left_camera_link',
                            'point_cloud': 'SM3/pointcloud',
                            'yaml_path': '/home/jetson/ros2_ws/src/ros2_fringe_projection/params/SM4.yaml',
                            
                            'n_images': '15'}.items(),
            condition=IfCondition(LaunchConfiguration('sm3'))
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('ros2_fringe_projection'),'launch', 'phase.launch.py')
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'frequency': '10',
                'px_f': '64',
                'steps': '8',
                'index': '0',
            }.items(),
            condition=IfCondition(LaunchConfiguration('sm4'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('ros2_fringe_projection'), 'launch', 'triangulation.launch.py'])
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'xy_step': '5',
                'z_step': '0.1',
                'num_splits': '3',
                'camera_frame_id': 'Active/left_camera_link',
                'point_cloud': 'SM4/pointcloud'
                }.items(),
            condition=IfCondition(LaunchConfiguration('sm4'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch',
                'mobile_bench.launch.py'])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'jetson_power_stats.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            namespace=LaunchConfiguration('namespace'),
            name='foxglove_bridge',
            output='screen'
        )
    ])