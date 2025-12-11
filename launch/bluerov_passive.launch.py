import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
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
        DeclareLaunchArgument('imu', default_value='True', description='Open IMU process?'),
        DeclareLaunchArgument('disparity', default_value='True', description='Open Disparity process?'),
        DeclareLaunchArgument('description', default_value='True', description='Open Description process?'),
        DeclareLaunchArgument('foxglove', default_value='False', description='Open Foxglove Bridge?'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='Namespace of the system'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm2_color_camera.launch.py'])]),
                launch_arguments = {'namespace': LaunchConfiguration('namespace'),
                                    'cam_0_frame_id': 'Passive/left_camera_link',
                                    'cam_1_frame_id': 'Passive/right_camera_link',
                                    'camera_type': 'grayscale',
                                    'cam_0_serial': '16378753',
                                    'cam_1_serial': '16378754'}.items(),
            condition=IfCondition(LaunchConfiguration('cameras'))
        ),
        TimerAction(
            period=5.0,  # seconds
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'), 'launch', 'stereo_bluerov.launch.py'])]),
                launch_arguments={
                    'namespace':LaunchConfiguration('namespace'),
                    'pangolin': "False",
                    'rescale': 'True',
                    'child_frame_id': 'Passive/left_camera_enu_link',
                    'parent_frame_id': 'base_link',
                    'frame_id': 'map',
                    'ENU_publish':'True',
                    'tracked_points': 'True',
                    'pose': '/mavros/vision_pose/pose'}.items(),
                condition=IfCondition(LaunchConfiguration('slam'))
            )
            ]
        ),
                              
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'), 'launch', 'stereo_inertial.launch.py'])]),
            launch_arguments={
                'namespace':LaunchConfiguration('namespace'),
                'pangolin': "False",
                'rescale': 'True',
                'child_frame_id': 'Passive/left_camera_link',
                'parent_frame_id': 'base_link',
                'frame_id': 'orbslam3',
                'tracked_points': 'True'}.items(),
            condition=IfCondition(LaunchConfiguration('inertial'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('driver_stim300'), 'launch', 'stim300_driver.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'frame_id': 'stim3600_link'}.items(),
            condition=IfCondition(LaunchConfiguration('imu'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('passive_stereo'), 'launch', 'passive_stereo.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                            'frame_id': 'Passive/left_camera_link',
                            'sampling_factor': "6",
                            'publish_rectified': "True",
                            'debug_image': "False",
                            'crop_factor': "0.8"}.items(),
        condition=IfCondition(LaunchConfiguration('disparity')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'voris_visualize.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'nano_jetson_power.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),

        Node(
            package='jetson_power_monitor',
            executable='leak_sensor_publisher.py',
            namespace=LaunchConfiguration('namespace'),
            name='leak_sensor_publisher',
            output='screen'
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            namespace=LaunchConfiguration('namespace'),
            name='foxglove_bridge',
            output='log',
            condition=IfCondition(LaunchConfiguration('foxglove'))
        )
    ])