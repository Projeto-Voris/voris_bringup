import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource


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
                'composition_launch.py'   
            )
        )
    )

    # --- Controle (MAVROS + Controller) ---
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros_control'),
                'launch',
                'demo.launch.py'
            )
        ),
        launch_arguments={
            "fcu_url": "udp://0.0.0.0:14550@"   
        }.items()
    )


    return LaunchDescription([
        ping360,
        gscam2,
        control,
    ])
