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
        DeclareLaunchArgument('bag_file', default_value='g:\Drives compartilhados\BLOCO DE LETRAS LOOP CLOSRURE 08\08',  description='Path to the rosbag file'),
         # Launch the ros2 bag play command as a Node
        # Node(
        #     package='ros2_unreal_bridge',  # The package that handles rosbag2 operations
        #     executable='pose_unreal_bridge',  # The command to play the bag file
        #     name='ue_pose_bridge',
        #     namespace=LaunchConfiguration('namespace'),
        #     output='screen',
        #     remappings=[
        #         ('pose': 'pose'),
        #     ]
        # ),
        
        # Node(
        #     package='rosbag2',  # The package that handles rosbag2 operations
        #     executable='rosbag2_play',  # The command to play the bag file
        #     name='rosbag2_play_node',
        #     arguments=[
        #         'play',
        #         LaunchConfiguration('bag_file')  # Pass the bag file path
        #     ],
        #     output='screen'
        # ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    FindPackageShare('voris_log'), 'launch', 'cameraInfo_publisher.launch.py'])]),
                launch_arguments={
                    'namespace':LaunchConfiguration('namespace'),
                    'left_frameID': "/Passive/left_camera_link",
                    'right_frameID': '/Passive/right_camera_link',
                    'yaml_right': '/home/voris/ros2_ws/src/flir_camera_driver/spinnaker_camera_driver/config/16378754.yaml',
                    'yaml_left': '/home/voris/ros2_ws/src/flir_camera_driver/spinnaker_camera_driver/config/16378753.yaml'}.items(),
            ),          
        TimerAction(
            period=5.0,  # seconds
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'), 'launch', 'stereo.launch.py'])]),
                launch_arguments={
                    'namespace':LaunchConfiguration('namespace'),
                    'pangolin': "False",
                    'rescale': 'True',
                    'child_frame_id': 'Passive/left_camera_link',
                    'parent_frame_id': 'base_link',
                    'frame_id': 'orbslam3',
                    'left_image': '/SM2/left/image_raw',
                    'right_image': '/SM2/right/image_raw'}.items(),
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
                'parent_frame_id': 'base_link_ned',
                'frame_id': 'orbslam3'}.items(),
            condition=IfCondition(LaunchConfiguration('inertial'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('passive_stereo'), 'launch', 'passive_stereo.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                            'left_image': '/SM2/left/image_raw',
                            'right_image': '/SM2/right/image_raw'}.items(),
        condition=IfCondition(LaunchConfiguration('disparity')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'voris_visualize.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),



        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            namespace=LaunchConfiguration('namespace'),
            name='foxglove_bridge',
            output='log'
        )
    ])