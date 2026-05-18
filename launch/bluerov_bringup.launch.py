import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    composable_node = [ComposableNode(
                package='voris_log',
                plugin='dvl_log::DVLNode',
                name='dvl_node',
                namespace=namespace,
                remappings=[
                    ('/model/bluerov2/dvl/velocity', 'dvl/velocity'),
                    ('dvl_twist', 'dvl/twist'),
                ],
                extra_arguments={'use_intra_process_comms': True}

            ),
            ComposableNode(
                package='waterlinked_dvl_driver',
                plugin='waterlinked::ros::WaterLinkedDvlDriver',
                name='dvl_driver',
                namespace=namespace,
                parameters=[PathJoinSubstitution([FindPackageShare("waterlinked_dvl_driver"), "config", "dvl.yaml"])],
                remappings=[
                    ('~/velocity_report', 'dvl/velocity'),
                    ('~/odom', 'dvl/odom'),   
                    ('~/dead_reackoning_report', 'dvl/dead_reackoning_report'),
                ],
                extra_arguments={'use_intra_process_comms': True}
            )
    ]

    container = ComposableNodeContainer(
        name='bluerov2_dvl_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_node,
        output='screen',
    )
    return [container]

def generate_launch_description():

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument('gscam2', default_value='false', description='Launch GSCam2 Camera'),
        DeclareLaunchArgument('mavros', default_value='true', description='Launch mavros_control'),
        DeclareLaunchArgument('sonar3d', default_value='false', description='Launch Sonar3D Node'),
        DeclareLaunchArgument('namespace', default_value='bluerov2', description='Namespace of the MAVROS system'),



        # --- Sonar3D ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('sonar3d'), 'launch', 'sonar3d.launch.py'])
            ),
            launch_arguments={'namespace': PathJoinSubstitution([LaunchConfiguration('namespace'),'sonar3d']),
                              'ip': '192.168.2.30',
                              'speed_of_sound': '1491'}.items(),
            condition=IfCondition(LaunchConfiguration('sonar3d'))
        ),

        # --- Câmera GSCam2 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('gscam2'), 'launch', 'node_param_launch.py'])
                ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'image_topic': 'image_raw/compressed',
                'camera_info': 'camera_info'}.items(),
            condition=IfCondition(LaunchConfiguration('gscam2'))
        ),

        # --- MAVROS Control ---
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mavros_control'), 'launch', 'mavros.launch'])
                ),
            launch_arguments={
                'fcu_url': 'tcp://0.0.0.0:5777@',
                'namespace': 'mavros'
            }.items(),
            condition=IfCondition(LaunchConfiguration('mavros'))
        ),
        OpaqueFunction(function=launch_setup),
    ])
