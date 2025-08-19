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
        DeclareLaunchArgument('disparity', default_value='True', description='Open Disparity process?'),
        DeclareLaunchArgument('description', default_value='True', description='Open Description process?'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='Namespace of the system'),
        DeclareLaunchArgument('camera_type', default_value='grayscale', description='Type of the cameras (color or grayscale)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'), 'launch', 'sm2_color_camera.launch.py'])]),
                # launch_arguments = {'namespace': LaunchConfiguration('namespace'),
                #                     'cam_0_frame_id': 'Passive/left_camera_link',
                #                     'cam_1_frame_id': 'Passive/right_camera_link',
                #                     'camera_type': LaunchConfiguration('camera_type'),
                #                     'cam_0_serial': '16378753',
                #                     'cam_1_serial': '16378754'}.items(), 
                launch_arguments = {'namespace': LaunchConfiguration('namespace'), # Colored Cameras
                                    'cam_0_frame_id': 'Passive/left_camera_link',
                                    'cam_1_frame_id': 'Passive/right_camera_link',
                                    'camera_type': LaunchConfiguration('camera_type'),
                                    'cam_0_serial': '22548025',
                                    'cam_1_serial': '22548033'}.items(),
            condition=IfCondition(LaunchConfiguration('cameras'))
        ),
        TimerAction(
            period=5.0,  # seconds
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'), 'launch', 'stereo_color.launch.py'])]),
                launch_arguments={
                    'namespace':LaunchConfiguration('namespace'),
                    'pangolin': "False",
                    'rescale': 'True',
                    'child_frame_id': 'Passive/left_camera_link',
                    'parent_frame_id': 'base_link',
                    'frame_id': 'orbslam3'}.items(),
                condition=IfCondition(LaunchConfiguration('slam'))
            )
            ]
        ),                
        # TimerAction(
        #     period=5.0,  # seconds
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([PathJoinSubstitution([
        #                 FindPackageShare('orbslam3_ros2'), 'launch', 'stereo.launch.py'])]),
        #             launch_arguments={
        #                 'namespace':LaunchConfiguration('namespace'),
        #                 'pangolin': "False",
        #                 'rescale': 'True',
        #                 'child_frame_id': 'Passive/left_camera_link',
        #                 'parent_frame_id': 'base_link',
        #                 'frame_id': 'orbslam3'}.items(),
        #             condition=IfCondition(LaunchConfiguration('slam'))
        #         )
        #     ]
        # ),        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'), 'launch', 'stereo_inertial.launch.py'])]),
            launch_arguments={
                'namespace':LaunchConfiguration('namespace'),
                'pangolin': "False",
                'rescale': 'True',
                'child_frame_id': 'Passive/left_camera_link',
                'parent_frame_id': 'base_link_ned',
                'frame_id': 'orbslam3'}.items(),
            condition=IfCondition(LaunchConfiguration('inertial'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('driver_stim300'), 'launch', 'stim300_driver.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),}.items(),
            condition=IfCondition(LaunchConfiguration('inertial'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('passive_stereo'), 'launch', 'triangulation.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        condition=IfCondition(LaunchConfiguration('disparity')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'mobile_bench.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'agx_jetson_power.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),



        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            namespace=LaunchConfiguration('namespace'),
            name='foxglove_bridge',
            output='log'
        )
    ])