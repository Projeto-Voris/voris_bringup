import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    # --- Sonar Ping360 ---
    ping360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ping360_sonar'),
                'launch',
                'ping360_bringup.launch.py'
            )
        )
    )

    # --- Câmera GSCam2 ---
    gscam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gscam2'),
                'launch',
                'node_param_launch.py'
            )
        )
    )

    # --- Controle (MAVROS + Controller) ---
    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros_control'),
        'launch',
        'mavros.launch'
    )
    
    launch_mavros = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(mavros_launch_file),
        launch_arguments={
            'fcu_url': 'tcp://0.0.0.0:5777@',
            'namespace': 'mavros'
        }.items()
    )

    return LaunchDescription([
        ping360,
        gscam2,
        launch_mavros,
    ])
