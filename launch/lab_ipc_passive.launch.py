from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer,LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

# --- Configurações da Câmera ---
camera_params = {
    'debug': False,
    'compute_brightness': True,
    'dump_node_map': False,
    'adjust_timestamp': False,
    'gain_auto': 'Off',
    'gain': 0,
    'frame_rate_enable': True,
    'frame_rate': 30,
    'exposure_auto': 'Continuous',		 
    # 'exposure_time':16000,
    'auto_exposure_lower_limit':30 ,
    'auto_exposure_upper_limit':33333.33,
    'line2_selector': 'Line2',
    'line2_v33enable': False,
    'line3_selector': 'Line3',
    'line3_linemode': 'Input',
    'trigger_selector': 'FrameStart',
    'trigger_mode': 'Off',
    'trigger_source': 'Line3',
    'trigger_delay': 9,
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
}

def make_resizer_node(name, input_topic, output_topic):
    return ComposableNode(
        package='voris_log',
        plugin='voris_log::ImageProcessorNode',
        name=name,
        namespace=LaunchConfiguration('namespace'),
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
        namespace=LaunchConfiguration('namespace'),
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
        namespace=LaunchConfiguration('namespace'),
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
    cam_type_0 = LaunchConfiguration('cam_0_type').perform(context)
    cam_type_1 = LaunchConfiguration('cam_1_type').perform(context)
    serial_0 = LaunchConfiguration('cam_0_serial').perform(context)
    name_0 = LaunchConfiguration('cam_0_name').perform(context)
    frame_0 = LaunchConfiguration('cam_0_frame_id').perform(context)
    serial_1 = LaunchConfiguration('cam_1_serial').perform(context)
    frame_1 = LaunchConfiguration('cam_1_frame_id').perform(context)
    name_1 = LaunchConfiguration('cam_1_name').perform(context)

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
                    'do_rectify': True,
                    'rescale': True,
                    'ENU_publish': True,
                    'tracked_points': True,
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
        else:
            slam_node = ComposableNode(
                    package='orbslam3_ros2',
                    plugin='orbslam3_ros2::StereoSlamNode',
                    name='slam_stereo_node',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'voc_file': LaunchConfiguration('voc_file'),
                        'settings_file': LaunchConfiguration('settings_file'),
                        'do_rectify': True,
                        'rescale': True,
                        'ENU_publish': True,
                        'tracked_points': False,
                        'frame_id': 'map',
                        'parent_frame_id': 'base_link',
                        'child_frame_id': frame_0
                    }],
                    remappings=[
                        ('camera/left', f"{name_0}/image_raw"),
                        ('camera/right', f"{name_1}/image_raw")
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            composable_nodes.append(slam_node)
    
    # 3. Configuração do Saver Node (Opcional)
    if LaunchConfiguration('enable_saver').perform(context) == 'true':
        saver_node = ComposableNode(
            package='orbslam3_ros2', # Nome do seu pacote
            plugin='orbslam3_ros2::StereoImageSaverNode',
            name='stereo_saver_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'save_directory': LaunchConfiguration('save_directory')
            }],
            remappings=[
                ('camera/left', topic_left),
                ('camera/right', topic_right)
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes.append(saver_node)

    if LaunchConfiguration('disparity').perform(context) == 'true':
        composable_nodes_2.append(make_rectify_node(name_0))
        composable_nodes_2.append(make_rectify_node(name_1))

        retinify_node = ComposableNode(
            package='passive_stereo',
            plugin='RetinifyDisparityNode',
            name='retinify_disparity_node',
            namespace=LaunchConfiguration('namespace'),
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
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'frame_id': frame_0,
                'sampling_factor': 0.2,
                'crop_factor': 0.7,
            }],
            remappings=[
                ('left/image_rect', f"{name_0}/image_rect"),
                ('right/camera_info', f"{name_1}/camera_info"),
                ('disparity/image', 'disparity/image'),
                ('pointcloud', 'disparity/pointcloud')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes_2.append(retinify_node)
        composable_nodes_2.append(triangulation_node)


    

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
        DeclareLaunchArgument('cam_0_type', default_value='blackfly_s', description='Camera 0 type'),
        DeclareLaunchArgument('cam_1_type', default_value='blackfly_s', description='Camera 1 type'),
        DeclareLaunchArgument('cam_0_serial', default_value='16378753', description='Camera 0 serial number'),
        DeclareLaunchArgument('cam_1_serial', default_value='16378754', description='Camera 1 serial number'),
        DeclareLaunchArgument('cam_0_frame_id', default_value='Passive/left_camera_link', description='Frame ID for camera 0'),
        DeclareLaunchArgument('cam_1_frame_id', default_value='Passive/right_camera_link', description='Frame ID for camera 1'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='ROS namespace'),
        
        # Argumentos do SLAM
        DeclareLaunchArgument('slam', default_value='true', description='Usar SLAM?'),
        DeclareLaunchArgument('inertial', default_value='false', description='Usar SLAM Stereo inertial?'),
        DeclareLaunchArgument('voc_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/lab_bw.yaml', 
                  description='Caminho para o settings .yaml'),
        
        # Argumentos do Saver
        DeclareLaunchArgument('enable_saver', default_value='false', description='Ativar gravação de imagens?'),
        DeclareLaunchArgument('save_directory', default_value='/home/jetson/Documents/stereo_images', description='Pasta para salvar imagens'),

        # Argumento de disparidade
        DeclareLaunchArgument('disparity', default_value='true', description='Ativar nó de disparidade?'),

        # Argumentos nodos extras
        DeclareLaunchArgument('description', default_value='true', description='Ativar visualização da descrição?'),

        # Nó de robot_description (visualização)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'mobile_bench.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),

        # Nó de monitoramento de energia (para o Jetson AGX)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'agx_jetson_power.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),

        OpaqueFunction(function=launch_setup),
    ])