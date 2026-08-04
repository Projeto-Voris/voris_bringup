from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode 
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import datetime

# --- Configurações da Câmera ---
camera_params = {
        'debug': False,
        'quiet': True,
        'compute_brightness': True,
        'dump_node_map': False,
        'adjust_timestamp': True,
        'pixel_format': 'BayerRG8',
        'gain_auto': 'Off',
        'balance_white_auto': 'On',
        'gain': 0.0,
        'exposure_auto': 'Off',
        'exposure_time': 33333.84,
        'frame_rate': 30.00,
        'frame_rate_enable': True,
        'auto_exposure_lower_limit': 30,
        'auto_exposure_upper_limit': 33333.84,
        'buffer_queue_size': 3,
        'line2_selector': 'Line2',
        'line2_v33enable': False,
        'line3_selector': 'Line3',
        'line3_linemode': 'Input',
        'trigger_selector': 'FrameStart',
        'trigger_mode': 'Off',
        'trigger_source': 'Line3',
        'trigger_delay': 29,
        'trigger_overlap': 'ReadOut',
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True,
        'binning_x': 1,
        'binning_y': 1,
}

def make_resizer_node(name, input_topic, output_topic):
    return ComposableNode(
        package='voris_log',
        plugin='voris_log::ImageProcessorNode',
        name=name,
        namespace=LaunchConfig('namespace'),
        parameters=[{
            'resize_width': 612,
            'resize_height': 512,
            'out_topic/compressed/jpeg_quality': 50, 
            'apply_clahe': LaunchConfig('use_clahe'),
            'clahe_climp': 2,
            'clahe_tile': 2 
        }],
        remappings=[
            ('input/image', input_topic),
            ('output/compressed_image', output_topic)
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def make_debayer_node(name):
    return ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name=f"{name}_debayer",
            namespace=LaunchConfig('namespace'),
            parameters=[{
                'debayer': 0
            }],
            remappings=[
                ('image_raw', f"{name}/image_raw"),
                ('image_color', f"{name}/image_color")
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

def make_rectify_node(name):
    return ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name=f"{name}_rectify",
        namespace=LaunchConfig('namespace'),
        parameters=[{
            'queue_size': 2,
            'interpolation': 1
        }],
        remappings=[
            ('image', f"{name}/image_color"),
            ('camera_info', f"{name}/camera_info"),
            ('image_rect', f"{name}/image_rect_color")
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def make_camera_node(name, cam_type, serial, camera_info_url, frame_id):
    parameter_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config', cam_type + '.yaml']
    )
    return ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraDriver',
        name=name,
        namespace=LaunchConfig('namespace'),
        parameters=[
            camera_params,
            {   'parameter_file': parameter_file,
                'serial_number': serial,
                'camerainfo_url': camera_info_url,
                'frame_id': frame_id,
                'use_intra_process_comms': True # Força no parametro tambem por segurança
            }
        ],
        remappings=[('~/control', '/exposure_control/control')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

def launch_setup(context, *args, **kwargs):
    # 1. Configurações das Câmeras
    cam_type_0 = LaunchConfig('cam_0_type').perform(context)
    cam_type_1 = LaunchConfig('cam_1_type').perform(context)
    serial_0 = LaunchConfig('cam_0_serial').perform(context)
    name_0 = LaunchConfig('cam_0_name').perform(context)
    frame_0 = LaunchConfig('cam_0_frame_id').perform(context)
    serial_1 = LaunchConfig('cam_1_serial').perform(context)
    frame_1 = LaunchConfig('cam_1_frame_id').perform(context)
    name_1 = LaunchConfig('cam_1_name').perform(context)
    save_path = LaunchConfig('save_directory').perform(context)

    cam_0_camera_info_url = 'file://' + str(PathJoinSubstitution([
        FindPackageShare('spinnaker_camera_driver'), 'config', serial_0 + '.yaml'
    ]).perform(context))
    cam_1_camera_info_url = 'file://' + str(PathJoinSubstitution([
        FindPackageShare('spinnaker_camera_driver'), 'config', serial_1 + '.yaml'
    ]).perform(context))

    # Lista de componentes (inicia com as câmeras)
    composable_nodes = [
        make_camera_node(name_0, cam_type_0, serial_0, cam_0_camera_info_url, frame_0),
        make_camera_node(name_1, cam_type_1, serial_1, cam_1_camera_info_url, frame_1),

        make_resizer_node(f"{name_0}_debug", f"{name_0}/image_raw",f"{name_0}/debug/image_raw"),
        make_resizer_node(f"{name_1}_debug", f"{name_1}/image_raw",f"{name_1}/debug/image_raw")

    ]
    composable_nodes_2 = [] # Container separado para os nós de retificação e disparidade

    


    # 2. Configuração do SLAM Node   
    if LaunchConfig('slam').perform(context) == 'true':
        if LaunchConfig('use_stereo_intertial').perform(context) == 'true':
            slam_node = ComposableNode(
                package='orbslam3_ros2',
                plugin='orbslam3_ros2::StereoInertialSlamNode',
                name='slam_stereo_inertial_node',
                namespace=LaunchConfig('namespace'),
                parameters=[{
                    'voc_file': LaunchConfig('voc_file'),
                    'settings_file': LaunchConfig('settings_file'),
                    'do_rectify': True,
                    'ENU_publish': True,
                    'frame_id': 'map',
                    'parent_frame_id': 'base_link',
                    'child_frame_id': frame_0
                }],
                remappings=[
                    ('camera/left', f"{name_0}/image_raw"),
                    ('camera/right', f"{name_1}/image_raw"),
                    ('imu', '/imu/data') # Supondo que o IMU esteja publicando neste tópico
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
            composable_nodes.append(slam_node)
        else:
            slam_node = ComposableNode(
                    package='orbslam3_ros2',
                    plugin='orbslam3_ros2::StereoSlamNode',
                    name='slam_stereo_node',
                    namespace=LaunchConfig('namespace'),
                    parameters=[{
                        'voc_file': LaunchConfig('voc_file'),
                        'settings_file': LaunchConfig('settings_file'),
                        'do_rectify': False,
                        'ENU_publish': True,
                        'tf_publish': False,
                        'resize_factor': 0.25,
                        'frame_id': 'map',
                        'parent_frame_id': 'base_link',
                        'child_frame_id': frame_0,
                        'tracked_points': False,
                        'clahe': LaunchConfig('use_clahe'),
                    }],
                    remappings=[
                        ('camera/left', f"{name_0}/image_rect_color"),
                        ('camera/right', f"{name_1}/image_rect_color"),
                        ('pose_cov', 'slam/pose_cov')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            composable_nodes.append(slam_node)
    if LaunchConfig('save_stereo').perform(context) == 'true':
            composable_nodes.append(ComposableNode(
                    package='orbslam3_ros2',
                    plugin='slam::ImageSaver',
                    name='stereo_image_saver',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'saving_path': f'{save_path}_stereo_images',
                        'apply_clahe': LaunchConfig('use_clahe'),
                        'clahe_tiles': 5.0,
                        'clahe_clip': 5.0,
                    }],
                    remappings=[
                        ('camera/left', f"{name_0}/image_raw"),
                        ('camera/right', f"{name_1}/image_raw"),
                        ('odometry', '/mavros/local_position/odom')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ))


    if LaunchConfig('save_sonar_stereo').perform(context) == 'true':
        composable_nodes.append(ComposableNode(
                                    package='voris_log', # Nome do seu pacote
                                    plugin='voris_log::DataSaverNode',
                                    name='stereo_sonar_image_saver',
                                    namespace=LaunchConfig('namespace'),
                                    parameters=[{
                                        'save_directory': f'{save_path}_stereo_sonar',
                                    }],
                                    remappings=[
                                        ('camera/left', f"{name_0}/image_raw"),
                                        ('camera/right', f"{name_1}/image_raw"),
                                        ('sonar_point_cloud', 'sonar3d/pointcloud'),
                                        ('odometry', '/mavros/local_position/odom')
                                    ],
                                    extra_arguments=[{'use_intra_process_comms': True}]
                                ))

    if LaunchConfig('dvl').perform(context) == 'true':
        composable_nodes.append(ComposableNode(
                                                package='voris_log', # Nome do seu pacote
                                                plugin='voris_log::DVL2MavrosNode',
                                                name='dvl_convert',
                                                namespace=LaunchConfig('namespace'),
                                                parameters=[{
                                                    'base_frame': 'base_link',
                                                }],
                                                remappings=[
                                                    ('dvl/pose', "/waterlinked_dvl_driver/dead_reckoning_report"),
                                                    ('dvl/velocity', "/waterlinked_dvl_driver/velocity_report"),
                                                    ('dvl/twist_cov', '/mavros/vision_speed/speed_twist_cov'),
                                                    ('dvl/pose_cov', 'dvl/pose_cov'),
                                                ],
                                                extra_arguments=[{'use_intra_process_comms': True}]
                                            )
        )

    composable_nodes.append(ComposableNode(
                            package='navigation_filter',
                            plugin='nav_filter::PoseFusionComponent',
                            name='pose_fusion_node',
                            namespace=LaunchConfig('namespace'),
                            parameters=[{
                                'home_lat': -27.5951,
                                'home_long': -48.5637,
                                'home_alt': 0.0,
                                'debug': True,
                                'align_heading': False,
                            }],
                            remappings=[
                                ('slam/pose_cov', 'slam/pose_cov'),
                                ('slam/status', 'slam_status'),
                                ('dvl/pose_cov', 'dvl/pose_cov'),
                                ('fused/pose_cov', '/mavros/vision_pose/pose_cov'),
                            ]))



    if LaunchConfig('disparity').perform(context) == 'true':
        composable_nodes_2.append(ComposableNode(package='passive_stereo',
                                    plugin='RetinifyDisparityNode',
                                    name='retinify_disparity_node',
                                    namespace=LaunchConfig('namespace'),
                                    parameters=[{
                                        'debug_image': True,
                                        'publish_disp': True,
                                        'clahe': LaunchConfig('use_clahe'),

                                    }],
                                    remappings=[
                                        ('left/image_rect', f"{name_0}/image_rect_color"),
                                        ('left/camera_info', f"{name_0}/camera_info"),
                                        ('right/image_rect', f"{name_1}/image_rect_color"),
                                        ('right/camera_info', f"{name_1}/camera_info")
                                    ],
                                    extra_arguments=[{'use_intra_process_comms': True}]
                                ))
        composable_nodes_2.append(ComposableNode(package='passive_stereo',
                                    plugin='TriangulationNode',
                                    name='triangulation_node',
                                    namespace=LaunchConfig('namespace'),
                                    parameters=[{
                                        'frame_id': frame_0,
                                        'sampling_factor': 0.2,
                                        'crop_factor': 0.5,
                                        'parent_frame': 'base_link'
                                    }],
                                    remappings=[
                                        ('left/image_rect', f"{name_0}/image_rect_color"),
                                        ('right/camera_info', f"{name_1}/camera_info"),
                                        ('disparity/image', 'disparity/image'),
                                        ('pointcloud', 'disparity/pointcloud')
                                    ],
                                    extra_arguments=[{'use_intra_process_comms': True}]
                                ))

    composable_nodes_2.append(make_debayer_node(name_0))
    composable_nodes_2.append(make_rectify_node(name_0))
    composable_nodes_2.append(make_debayer_node(name_1))
    composable_nodes_2.append(make_rectify_node(name_1))
    # 4. Container Principal
    container = ComposableNodeContainer(
        name='passive_stereo_container',
        namespace=LaunchConfig('namespace'),
        package='rclcpp_components',
        executable='component_container_mt', # IMPORTANTE: Multi-Threaded para performance
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

    return [container, container_disp_load_action]


def generate_launch_description():
    return LaunchDescription([
        # Argumentos das Câmeras
        DeclareLaunchArgument('cam_0_name', default_value='left', description='Camera 0 name'),
        DeclareLaunchArgument('cam_1_name', default_value='right', description='Camera 1 name'),
        DeclareLaunchArgument('cam_0_type', default_value='blackfly_s', description='Camera 0 type'),
        DeclareLaunchArgument('cam_1_type', default_value='blackfly_s', description='Camera 1 type'),
        DeclareLaunchArgument('cam_0_serial', default_value='22548033', description='Camera 0 serial number'),
        DeclareLaunchArgument('cam_1_serial', default_value='22548025', description='Camera 1 serial number'),
        DeclareLaunchArgument('cam_0_frame_id', default_value='Passive/left_camera_link', description='Frame ID for camera 0'),
        DeclareLaunchArgument('cam_1_frame_id', default_value='Passive/right_camera_link', description='Frame ID for camera 1'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='ROS namespace'),
        
        # Argumentos do SLAM
        DeclareLaunchArgument('use_stereo_intertial', default_value='false', description='Usar SLAM Stereo inertial?'),
        DeclareLaunchArgument('voc_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value=f'/home/{os.getenv("USER")}/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/stereo_bluerov.yaml', 
                  description='Caminho para o settings .yaml'),
        
        # Argumentos do Saver
            DeclareLaunchArgument('save_sonar_stereo', default_value='false', description='Ativar gravação de imagens e sonar'),
        DeclareLaunchArgument('save_stereo', default_value='false', description='Ativar salvamento imagens'),
        DeclareLaunchArgument('save_directory', default_value=f'/home/{os.getenv("USER")}/Documents/{datetime.datetime.now().strftime("%Y%m%d_%H%M")}', description='Pasta para salvar imagens'),

        # Argumentos nodos extras
        DeclareLaunchArgument('slam', default_value='true', description='Ativar SLAM?'),
        DeclareLaunchArgument('disparity', default_value='false', description='Ativar nó de disparidade?'),
        DeclareLaunchArgument('description', default_value='true', description='Ativar visualização da descrição?'),
        DeclareLaunchArgument('sonar3d', default_value='true', description='Ativar Sonar 3d?'),
        DeclareLaunchArgument('dvl', default_value='true', description='Ativar DVL converter?'),
        DeclareLaunchArgument('mavros', default_value='false', description='Ativar MAVROS?'),
        DeclareLaunchArgument('use_clahe', default_value='true', description='Use CLAHE on images?'),
        # Nó de robot_description (visualização)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'voris_visualize.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),

        # Nó de monitoramento de energia (para o Jetson Nano)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'nano_jetson_power.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),

        # Nó de recepção do Sonar 3D-15
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('sonar3d'), 'launch', 'sonar3d.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'ip': '192.168.2.30', #Static Sonar IP
                              'speed_of_sound': '1500.0', #Freshwater
                              'max_dist': '6'}.items(),
            condition=IfCondition(LaunchConfiguration('sonar3d')),
        ),

        Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='mavros',
        output='screen',
        parameters=[
            {
                'fcu_url': 'tcp://150.162.167.10:5777@',
                'gcs_url': 'udp://@150.162.167.10:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'pluginlist_yaml': PathJoinSubstitution([FindPackageShare('voris_bringup'), 'config', 'apm_pluginlists.yaml']),     
                'config_yaml': PathJoinSubstitution([FindPackageShare('voris_bringup'), 'config', 'apm_config.yaml']),
            }
        ],
        condition=IfCondition(LaunchConfiguration('mavros')),
    ),


        OpaqueFunction(function=launch_setup),
    ])