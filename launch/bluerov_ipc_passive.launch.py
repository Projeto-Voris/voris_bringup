from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# --- Configurações da Câmera ---
camera_params = {
        'debug': False,
        'compute_brightness': True,
        'dump_node_map': False,
        'adjust_timestamp': False,
        'pixel_format': 'BGR8',
        'gain_auto': 'On',
        'gain': 0,
        'exposure_auto': 'On',
        'exposure_time': 16667,
        'frame_rate': 59.99,
        'frame_rate_enable': True,
        'auto_exposure_lower_limit': 30,
        'auto_exposure_upper_limit': 16797.84,
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
            ('output/image', output_topic)
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
        make_resizer_node('resized_left', f"{name_0}/image_raw",f"{name_0}/debug/image"),
        make_resizer_node('resized_right', f"{name_1}/image_raw", f"{name_1}/debug/image"),
    ]
    

    # 2. Configuração do SLAM Node   
    topic_left = f"{name_0}/image_raw"
    topic_right = f"{name_1}/image_raw"

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
                ('camera/left', topic_left),
                ('camera/right', topic_right),
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
                    'child_frame_id': frame_0
                }],
                remappings=[
                    ('camera/left', topic_left),
                    ('camera/right', topic_right)
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



    # 4. Container Principal
    container = ComposableNodeContainer(
        name='stereo_slam_container',
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
        DeclareLaunchArgument('use_stereo_intertial', default_value='false', description='Usar SLAM Stereo inertial?'),
        DeclareLaunchArgument('voc_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/jetson/ros2_ws/src/orbslam3_ros2/config/stereo_bluerov.yaml', 
                  description='Caminho para o settings .yaml'),
        
        # Argumentos do Saver
        DeclareLaunchArgument('enable_saver', default_value='true', description='Ativar gravação de imagens?'),
        DeclareLaunchArgument('save_directory', default_value='/home/jetson/Documents/stereo_images', description='Pasta para salvar imagens'),

        OpaqueFunction(function=launch_setup),
    ])