import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    orca4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('orca_bringup'), 'launch'),
            '/heavy_sim_no_tf.launch.py'
        ])
    )
    foxglove = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
    )
    orbslam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('orbslam3_ros2'), 'launch'),
            '/stereo_sim.launch.py',
        ]),
        launch_arguments = {'pangolin':"False"}.items()
    )
    passive_stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('disparity'), 'launch'),
            '/triangulation_rgb.launch.py'
        ])
    )
    mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pointcloud_concatenator'), 'launch'),
            '/pointcloud_concatenator.launch.py'
        ])
    )
    return LaunchDescription([
        orca4,
        foxglove,
        orbslam3,
        passive_stereo,
        mapper,
        ExecuteProcess(
            cmd=['foxglove-studio'],
            output='screen',
        )
    ])