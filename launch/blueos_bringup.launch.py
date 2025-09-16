import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    # --- Arguments ---
    declare_launch_ping360_arg = DeclareLaunchArgument(
        'launch_ping360',
        default_value='true',
        description='Launch Ping360 Sonar'
    )

    declare_launch_gscam2_arg = DeclareLaunchArgument(
        'launch_gscam2',
        default_value='true',
        description='Launch GSCam2 Camera'
    )

    declare_launch_mavros_arg = DeclareLaunchArgument(
        'launch_mavros',
        default_value='true',
        description='Launch mavros_control'
    )
    # --- --------- ---   


    # --- Sonar Ping360 ---
    ping360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ping360_sonar'),
                'launch',
                'ping360_bringup.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('launch_ping360'))
    )

    # --- Câmera GSCam2 ---
    gscam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gscam2'),
                'launch',
                'node_param_launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('launch_gscam2'))
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
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_mavros'))
    )

    return LaunchDescription([

        declare_launch_ping360_arg,
        declare_launch_gscam2_arg,
        declare_launch_mavros_arg,

        ping360,
        gscam2,
        launch_mavros,
    ])
