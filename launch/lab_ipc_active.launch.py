from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


camera_list = {
    'left': {
        'serial': '22348161',
        'cam_type': 'blackfly_s',
        'frame_id': 'Passive/left_camera_link'
    },
    'right': {
        'serial': '22348163',
        'cam_type': 'blackfly_s',
        'frame_id': 'Passive/right_camera_link'
    },
}

exposure_controller_parameters = {
    'brightness_target': 120,  # from 0..255
    'brightness_tolerance': 20,  # when to update exposure/gain
    # watch that max_exposure_time is short enough
    # to support the trigger frame rate!
    'max_exposure_time': 15000,  # usec
    'min_exposure_time': 5000,  # usec
    'max_gain': 0.0,
    'gain_priority': False,
}

cam_parameters = {
    'debug': False,
    'quiet': True,
    'image_queue_size' : 15,
    'buffer_queue_size' : 15,
    'compute_brightness': True,
    'exposure_auto': 'Off',
    'exposure_time': 10000,  # not used under auto exposure
    'trigger_mode': 'On',
    'gain_auto': 'Off',
    'trigger_source': 'Line2',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
    'trigger_activation': 'RisingEdge',
    'balance_white_auto': 'Off',
    # You must enable chunk mode and chunks: frame_id, exposure_time, and gain
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    # The Timestamp is not used at the moment
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}

def make_parameters(context):
    """Launch synchronized camera driver node."""
    pd = PathJoinSubstitution([ FindPackageShare('spinnaker_camera_driver'), 'config' ])
    calib_url = 'file://' + str(PathJoinSubstitution([    FindPackageShare('spinnaker_camera_driver'), 'config' ]).perform(context))

    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',  # only for ffmpeg image transport
    }

    cam_parameters['parameter_file'] = PathJoinSubstitution([pd, 'blackfly_s.yaml'])

    # generate camera parameters
    for cam, info in camera_list.items():
        cam_params = {cam + '.' + k: v for k, v in cam_parameters.items()}
        cam_params[cam + '.serial_number'] = info['serial']
        cam_params[cam + '.camerainfo_url'] = calib_url + '/' + info['serial'] + '.yaml'
        cam_params[cam + '.frame_id'] = info['frame_id']
        driver_parameters.update(cam_params)  # insert into main parameter list
        
        # Desabilitado o controlador de exposicao para evitar o crash
        # driver_parameters.update({cam + '.exposure_controller_name': cam + '.exposure_controller'})
        
    return driver_parameters

def make_resizer_node(name, input_topic, output_topic):
    return ComposableNode(
        package='voris_log',
        plugin='voris_log::ImageProcessorNode',
        name=name,
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'resize_width': 612,
            'resize_height': 512,
            'out_topic/compressed/jpeg_quality': 50 
        }],
        remappings=[
            ('input/image', input_topic),
            ('output/compressed_image', output_topic)
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def make_camera_node(context):
    return ComposableNode(
                package='spinnaker_synchronized_camera_driver',
                plugin='spinnaker_synchronized_camera_driver::SynchronizedCameraDriver',
                name='sync',
                namespace=LaunchConfiguration('namespace'),
                parameters=[make_parameters(context)],

                extra_arguments=[{'use_intra_process_comms': True}],
            )

def launch_setup(context, *args, **kwargs):
    # Lista de componentes (inicia com as câmeras)
    composable_nodes = [
        make_camera_node(context),
        make_resizer_node("left_debug", "sync/left/image_raw","sync/left/debug/image_raw"),
        make_resizer_node("right_debug", "sync/right/image_raw","sync/right/debug/image_raw")
    ]

    if LaunchConfiguration('StructuredLight').perform(context) == 'true':
        composable_nodes.append(
            ComposableNode(
                package='ros2_active_stereo',
                plugin='ros2_active_stereo::StereoFringeProcess',
                name='stereo_fringe_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[{
                    'monitor_name': 'Monitor_0',
                    'pixel_per_fringe': 64,
                    'fringe_steps': 12,
                    'image_color': 'blue',
                    'camera_hz': 20,
                    'skip_trigger': 3,
                    'save_image': False,
                    'save_path': '/tmp/structured-light',
                    'debug': True,
                }],
                remappings=[
                    ('left/image_raw', 'sync/left/image_raw'),
                    ('right/image_raw', 'sync/right/image_raw'),
                    ('camera_info', 'sync/left/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ))

    if LaunchConfiguration('RRP').perform(context) == 'true':
        composable_nodes.append(ComposableNode(
                        package='ros2_active_stereo',
                        plugin='ros2_active_stereo::StereoCorrelProcess',        
                        name='stereo_correl_node',
                        namespace=LaunchConfiguration('namespace'),
                        parameters=[{
                            'num_images': 5,
                            'steps': 10
                        }],
                        remappings=[
                            ('left/image', 'sync/left/image_raw'),
                            ('right/image', 'sync/right/image_raw')
                        ],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ))

    container = ComposableNodeContainer(
        name='cam_sync_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )


    return [container]

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='Active', description='ROS namespace'),
        
        # SM3
        DeclareLaunchArgument('RRP', default_value='true', description='Ativar SM3'),

        # Argumentos do Saver
        DeclareLaunchArgument('enable_saver', default_value='false', description='Ativar gravação de imagens?'),
        DeclareLaunchArgument('save_directory', default_value='/home/jetson/Documents/stereo_images', description='Pasta para salvar imagens'),

        # Argumentos nodos extras
        DeclareLaunchArgument('description', default_value='true', description='Ativar visualização da descrição?'),

        DeclareLaunchArgument('StructuredLight', default_value='true', description='Ativar Structured Light?'),
        # # Nó de robot_description (visualização)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'mobile_bench.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('ros2_active_stereo'), 'launch','gpio_control.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),

        # # Nó de monitoramento de energia (para o Jetson AGX)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'agx_jetson_power.launch.py'])]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),

        # Nó de triangulação do Structured Light (SM4)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('ros2_active_stereo'), 'launch', 'fringe_triangulation.launch.py'])
            ]),
            launch_arguments = {'namespace':LaunchConfiguration('namespace'),
                                'mod_tresh': '30',
                                'rad_tresh': '0.06',
                                'neighbours': '15',
                                'radius': '5',
                                'camera_frame_id': 'Active/left_camera_link',
                                'yaml_path': '/home/jetson/ros2_ws/src/ros2_active_stereo/ros2_active_stereo/config/lab_active.yaml',
                                'triangulated_pointcloud': 'SM4/pointcloud',
                                'disparity_pointcloud': '/Passive/disparity/pointcloud'}.items(),
            
            condition=IfCondition(LaunchConfiguration('StructuredLight'))
        ),
        # # Nó que coordena o SM3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('ros2_active_stereo'), 'launch',
                'correl_triangulation.launch.py'])
            ]),
            launch_arguments = {'namespace':LaunchConfiguration('namespace'),
                            'n_images': '5',
                            'window_size': '3',
                            'camera_frame_id': 'Active/left_camera_link',
                            'point_cloud': 'SM3/pointcloud',
                            'disparity_pointcloud': '/Passive/disparity/pointcloud',
                            'yaml_path': '/home/jetson/ros2_ws/src/ros2_active_stereo/ros2_active_stereo/config/lab_active.yaml'}.items(),
            condition=IfCondition(LaunchConfiguration('RRP'))
        ),

        OpaqueFunction(function=launch_setup),
    ])