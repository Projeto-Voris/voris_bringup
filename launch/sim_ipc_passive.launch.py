from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer,LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from datetime import datetime
import os

def make_resizer_node(name, input_topic, output_topic):
    return ComposableNode(
        package='voris_log',
        plugin='voris_log::ImageProcessorNode',
        name=name,
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'resize_width': 480,
            'resize_height': 360,
            'out_topic/compressed/jpeg_quality': 50,
            'use_sim_time': True, 
        }],
        remappings=[
            ('input/image', input_topic),
            ('output/compressed_image', output_topic)
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def make_rectify_node(name):
    return ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name=f"{name}_rectify",
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'use_intra_process_comms': True,
            'use_sim_time': True,
        }],
        remappings=[
            ('image', f"{name}/image_raw"),
            ('camera_info', f"{name}/camera_info"),
            ('image_rect', f"{name}/image_rect")
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def make_gazebo_bridge_node(gz_bridge_file):
    return ComposableNode(
                        package='ros_gz_bridge',
                        plugin='ros_gz_bridge::RosGzBridge',
                        parameters=[{
                            'config_file': gz_bridge_file,
                            'use_sim_time': True, # Vital para sincronia com Gazebo 
                            'use_intra_process_comms': True # Habilita Zero-Copy [cite: 355, 410]
                        }]
            )
                


def launch_setup(context, *args, **kwargs):
    # 1. Configurações das Câmeras
    name_0 = LaunchConfiguration('cam_0_name').perform(context)
    frame_0 = LaunchConfiguration('cam_0_frame_id').perform(context)
    frame_1 = LaunchConfiguration('cam_1_frame_id').perform(context)
    name_1 = LaunchConfiguration('cam_1_name').perform(context)
    gazebo_bridge_file = LaunchConfiguration('gazebo_bridge_file').perform(context)
    save_path = LaunchConfiguration('save_directory').perform(context)

    # Lista de componentes (inicia com as câmeras)
    composable_nodes = [
        make_gazebo_bridge_node(gazebo_bridge_file)
        # make_resizer_node(f"{name_0}_debug", f"{name_0}/image_raw",f"{name_0}/debug/image_raw"),
        # make_resizer_node(f"{name_1}_debug", f"{name_1}/image_raw",f"{name_1}/debug/image_raw")
    ]
    composable_nodes_2 = [make_rectify_node(name_0), make_rectify_node(name_1)] # Container separado para os nós de retificação e disparidade
    
    if LaunchConfiguration('slam').perform(context) == 'true':
        if LaunchConfiguration('inertial').perform(context) == 'true':
            slam_node = ComposableNode(
                package='orbslam3_ros2',
                plugin='orbslam3_ros2::StereoInertialSlamNode',
                name='slam_stereo_inertial_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[{
                    'voc_file': LaunchConfiguration('voc_file'),
                    'settings_file': LaunchConfiguration('settings_file'),
                    'resize_factor': 0.5,
                    'ENU_publish': True,
                    'clahe': False,
                    'tracked_points': False,
                    'frame_id': 'map',
                    'parent_frame_id': 'base_link',
                    'child_frame_id': frame_0,
                    'tf_publish': False,
                    'use_sim_time': True,
                }],
                remappings=[
                    ('camera/left', f"{name_0}/image_raw"),
                    ('camera/right', f"{name_1}/image_raw"),
                    ('imu', '/imu/data') # Supondo que o IMU esteja publicando neste tópico
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        else:
            slam_node = ComposableNode(
                    package='orbslam3_ros2',
                    plugin='orbslam3_ros2::StereoSlamNode',
                    name='slam_stereo_node',
                    namespace=f'{LaunchConfiguration("namespace").perform(context)}/slam',
                    parameters=[{
                        'voc_file': LaunchConfiguration('voc_file'),
                        'settings_file': LaunchConfiguration('settings_file'),
                        'resize_factor': 0.5,
                        'ENU_publish': True,
                        'clahe': False, 
                        'tracked_points': False,
                        'frame_id': 'map',
                        'parent_frame_id': 'base_link',
                        'child_frame_id': frame_0,
                        'tf_publish': False,
                        'use_sim_time': True,
                    }],
                    remappings=[
                        ('camera/left', f"/{LaunchConfiguration("namespace").perform(context)}/{name_0}/image_raw"),
                        ('camera/right', f"/{LaunchConfiguration("namespace").perform(context)}/{name_1}/image_raw"),
                        ('pose_cov', 'pose_cov'),
                        ('slam_status', 'status')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            composable_nodes.append(slam_node)
    
    if  LaunchConfiguration('save_sonar_stereo').perform(context) == 'true':
        save_stereo = ComposableNode(
                                    package='voris_log', # Nome do seu pacote
                                    plugin='voris_log::DataSaverNode',
                                    name='stereo_sonar_image_saver',
                                    namespace=LaunchConfiguration('namespace'),
                                    parameters=[{
                                        'save_directory': f'{save_path}_stereo_sonar',
                                    }],
                                    remappings=[
                                        ('camera/left', f"/{LaunchConfiguration("namespace").perform(context)}/{name_0}/image_raw"),
                                        ('camera/right', f"/{LaunchConfiguration("namespace").perform(context)}{name_1}/image_raw"),
                                        ('sonar_point_cloud', '/model/bluerov2/sonar3d/pointcloud'),
                                        ('odometry', '/mavros/local_position/odom')
                                    ],
                                    extra_arguments=[{'use_intra_process_comms': True}]
                                )
        composable_nodes.append(save_stereo)

    if LaunchConfiguration('save_stereo').perform(context) == 'true':
        save_node = ComposableNode(
                    package='voris_log',
                    plugin='voris_log::ImageSaver',
                    name='stereo_image_saver',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'saving_path': f'{save_path}_stereo',
                    }],
                    remappings=[
                        ('/camera_1/image_raw', f"{name_0}/image_raw"),
                        ('/camera_2/image_raw', f"{name_1}/image_raw"),
                        ('/odometry', '/mavros/local_position/odom')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]

                )
        composable_nodes.append(save_node)

    if LaunchConfiguration('disparity').perform(context) == 'true':
        retinify_node = ComposableNode(
            package='passive_stereo',
            plugin='passive_stereo::RetinifyStereoNode',
            name='retinify_stereo_node',
            namespace=f'{LaunchConfiguration("namespace").perform(context)}/disparity',
            parameters=[{
                'depth_mode': 'accuracy',
                'publish_disparity': False,
                'publish_pointcloud': True,
                'publish_depth': False,
                'publish_rectified': False,
                'debug_image': False,
                'sampling_factor': 0.5,
                'crop_factor': 1.0,
                'min_disp': 1.0,
                'max_dist': 10.0,
                'publish_confidence_field': False,
                'confidence_radius': 0,
                'confidence_alpha': 0.2,
                'min_confidence': 0.6,
                'frame_id': 'Passive/left_camera_link',
                'parent_frame': 'base_link',
                'calibration_file': '',
                'use_sim_time': True, 
            }],
            remappings=[
                ('left/image_rect', f"/{LaunchConfiguration("namespace").perform(context)}/{name_0}/image_rect"),
                ('left/camera_info', f"/{LaunchConfiguration("namespace").perform(context)}/{name_0}/camera_info"),
                ('right/image_rect', f"/{LaunchConfiguration("namespace").perform(context)}/{name_1}/image_rect"),
                ('right/camera_info', f"/{LaunchConfiguration("namespace").perform(context)}/{name_1}/camera_info"),
                ('disparity/image', 'image'),
                ('disparity/pointcloud', 'pointcloud'),
                ('depth/image', 'depth/image'),
                ('disparity/debug/image', 'debug/image'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes_2.append(retinify_node)
    

    container = ComposableNodeContainer(
        name='stereo_slam_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
    container_2 = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=composable_nodes_2
    )

    # Envolver container_2 em um TimerAction para aguardar o container inicializar
    container_disp_load_action = TimerAction(
            period=2.0,
            actions=[container_2]
        )

    if LaunchConfiguration('disparity').perform(context) == 'true':
        return [container, container_disp_load_action]
    else:
        return [container]

def generate_launch_description():
    return LaunchDescription([
        # Argumentos das Câmeras
        DeclareLaunchArgument('cam_0_name', default_value='left', description='Camera 0 name'),
        DeclareLaunchArgument('cam_1_name', default_value='right', description='Camera 1 name'),
        DeclareLaunchArgument('cam_0_frame_id', default_value='Passive/left_camera_link', description='Frame ID for camera 0'),
        DeclareLaunchArgument('cam_1_frame_id', default_value='Passive/right_camera_link', description='Frame ID for camera 1'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='ROS namespace'),
        
        # Inicalização
        DeclareLaunchArgument('slam', default_value='true', description='Usar SLAM?'),
        DeclareLaunchArgument('inertial', default_value='false', description='Usar SLAM Stereo inertial?'),
        DeclareLaunchArgument('disparity', default_value='true', description='Ativar nó de disparidade?'),

        DeclareLaunchArgument('voc_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orca5/orca_bringup/cfg/sim.yaml', description='Caminho para o settings .yaml'),
        DeclareLaunchArgument('gazebo_bridge_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orca5/orca_bringup/cfg/gzbridge_config.yaml', description='Caminho para o arquivo de configuração do Gazebo Bridge'), 
        # Argumentos do Saver
        DeclareLaunchArgument('save_sonar_stereo', default_value='false', description='Ativar gravação de imagens e sonar'),
        DeclareLaunchArgument('save_stereo', default_value='false', description='Ativar salvamento imagens'),
        DeclareLaunchArgument('save_directory', default_value=f'/home/{os.getenv("USER")}/Documents/{datetime.now().strftime("%Y%m%d%H%M")}', description='Pasta para salvar imagens'),
        
        OpaqueFunction(function=launch_setup),
    ])