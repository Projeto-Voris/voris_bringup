import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    fringe_projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('ros2_fringe_projection'),'launch', 'fringe_projection.launch.py')
        ])
    )

    sm4_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('spinnaker_camera_driver'), 'launch', 'sm4_camera.launch.py')
        ])
    )

    voris_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('voris_description'), 'launch',
                'lab_visualize.launch.py')
            ]),
    )

    foxglove = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
    )

    trigger_control = TimerAction(
            period=5.0,
            actions=[
            Node(
                package='ros2_fringe_projection',
                executable='param_controller.py',
                name='param_controller',
                namespace='SM4',
                parameters=[{'namespace': 'SM4'}],
                output='screen'
        )]
    )

    return LaunchDescription([
        foxglove,
        sm4_camera,
        trigger_control,
        voris_description,        
        fringe_projection
    ])