from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

# --- Configurações da Câmera ---
camera_params = {
        'debug': False,
        'compute_brightness': False,
        'dump_node_map': False,
        'adjust_timestamp': True,
        'pixel_format': 'BGR8',
        'gain_auto': 'On',
        'gain': 0,
        'exposure_auto': 'Continuous',
        'exposure_time': 16667,
        'frame_rate': 59.99,
        'frame_rate_enable': True,
        'auto_exposure_lower_limit': 30,
        'auto_exposure_upper_limit': 33333.84,
        'buffer_queue_size': 10,
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
        'binning_x': 2,
        'binning_y': 2,
}

def make_resizer_node(name, input_topic, output_topic):
    return ComposableNode(
        package='voris_log',
        plugin='voris_log::ImageProcessorNode',
        name=name,
        namespace=LaunchConfig('namespace'),
        parameters=[{
            'resize_width': 480,
            'resize_height': 360,
            'out_topic/compressed/jpeg_quality': 50 
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
        namespace=LaunchConfig('namespace'),
        parameters=[{
            'use_intra_process_comms': True
        }],
        remappings=[
            ('image', f"{name}/image_raw"),
            ('camera_info', f"{name}/camera_info"),
            ('image_rect', f"{name}/image_rect")
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
    
    if LaunchConfig('rectify_images').perform(context) == 'true':
        composable_nodes.append(make_rectify_node(name_0))
        composable_nodes.append(make_rectify_node(name_1))

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
                    'rescale': True,
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
                        'do_rectify': True,
                        'rescale': True,
                        'ENU_publish': True,
                        'frame_id': 'map',
                        'parent_frame_id': 'base_link',
                        'child_frame_id': frame_0,
                        'publish_tf': True
                    }],
                    remappings=[
                        ('camera/left', f"{name_0}/image_raw"),
                        ('camera/right', f"{name_1}/image_raw"),
                        ('pose', '/mavros/vision_pose/pose')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            composable_nodes.append(slam_node)

    # 3. Configuração do Saver Node (Opcional)
    if LaunchConfig('enable_saver').perform(context) == 'true':
        saver_node = ComposableNode(
            package='orbslam3_ros2', # Nome do seu pacote
            plugin='orbslam3_ros2::StereoImageSaverNode',
            name='stereo_saver_node',
            namespace=LaunchConfig('namespace'),
            parameters=[{
                'save_directory': LaunchConfig('save_directory')
            }],
            remappings=[
                ('camera/left', topic_left),
                ('camera/right', topic_right)
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes.append(saver_node)

    if LaunchConfig('disparity').perform(context) == 'true':
        retinify_node = ComposableNode(
            package='passive_stereo',
            plugin='RetinifyDisparityNode',
            name='retinify_disparity_node',
            namespace=LaunchConfig('namespace'),
            parameters=[{
                'debug_image': True,
                'publish_disp': True
            }],
            remappings=[
                ('left/image_rect', f"{name_0}/image_rect"),
                ('left/camera_info', f"{name_0}/camera_info"),
                ('right/image_rect', f"{name_1}/image_rect"),
                ('right/camera_info', f"{name_1}/camera_info")
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        triangulation_node = ComposableNode(
            package='passive_stereo',
            plugin='TriangulationNode',
            name='triangulation_node',
            namespace=LaunchConfig('namespace'),
            parameters=[{
                'frame_id': frame_0,
                'sampling_factor': 0.2,
                'crop_factor': 0.6,
            }],
            remappings=[
                ('left/image_rect', f"{name_0}/image_rect"),
                ('right/camera_info', f"{name_1}/camera_info"),
                ('disparity/image', 'disparity/image'),
                ('pointcloud', 'disparity/pointcloud')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes.append(retinify_node)
        composable_nodes.append(triangulation_node)

    # 4. Container Principal
    container = ComposableNodeContainer(
        name='passive_stereo_container',
        namespace=LaunchConfig('namespace'),
        package='rclcpp_components',
        executable='component_container_mt', # IMPORTANTE: Multi-Threaded para performance
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return [container]

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
        DeclareLaunchArgument('slam', default_value='true', description='Ativar SLAM?'),
        DeclareLaunchArgument('use_stereo_intertial', default_value='false', description='Usar SLAM Stereo inertial?'),
        DeclareLaunchArgument('voc_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/stereo_bluerov.yaml', 
                  description='Caminho para o settings .yaml'),
        
        # Argumentos do Saver
        DeclareLaunchArgument('enable_saver', default_value='false', description='Ativar gravação de imagens?'),
        DeclareLaunchArgument('save_directory', default_value='/home/jetson/Documents/stereo_images', description='Pasta para salvar imagens'),

        # Argumento de disparidade
        DeclareLaunchArgument('rectify_images', default_value='true', description='Ativar retificação das imagens?'),
        DeclareLaunchArgument('disparity', default_value='true', description='Ativar nó de disparidade?'),

        # Argumentos nodos extras
        DeclareLaunchArgument('description', default_value='true', description='Ativar visualização da descrição?'),
        DeclareLaunchArgument('sonar3d', default_value='false', description='Ativar Sonar 3d?'),

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
                              'ip': '192.168.2.30'}.items(),
            condition=IfCondition(LaunchConfiguration('sonar3d')),
        ),

        OpaqueFunction(function=launch_setup),
    ])