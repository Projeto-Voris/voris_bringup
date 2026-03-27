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
        'serial': '22348163',
        'cam_type': 'blackfly_s',
        'frame_id': 'Passive/left_camera_link'
    },
    'right': {
        'serial': '22348161',
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
    'image_queue_size' : 10,
    'buffer_queue_size' : 10,
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

    exp_ctrl_names = [cam + '.exposure_controller' for cam in camera_list.keys()]
    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'exposure_controllers': exp_ctrl_names,
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',  # only for ffmpeg image transport
    }
    # generate identical exposure controller parameters for all cameras
    for exp in exp_ctrl_names:
        driver_parameters.update(
            {exp + '.' + k: v for k, v in exposure_controller_parameters.items()}
        )
    # now set cam0 to be master, cam1 to be follower
    driver_parameters[exp_ctrl_names[0] + '.type'] = 'master'
    driver_parameters[exp_ctrl_names[1] + '.type'] = 'follower'
    # tell camera 1 that the master is (camera 0)
    driver_parameters[exp_ctrl_names[1] + '.master'] = exp_ctrl_names[0]
    cam_parameters['parameter_file'] = PathJoinSubstitution([pd, 'blackfly_s.yaml'])

    # generate camera parameters
    for cam, info in camera_list.items():
        cam_params = {cam + '.' + k: v for k, v in cam_parameters.items()}
        cam_params[cam + '.serial_number'] = info['serial']
        cam_params[cam + '.camerainfo_url'] = calib_url + '/' + info['serial'] + '.yaml'
        cam_params[cam + '.frame_id'] = info['frame_id']
        driver_parameters.update(cam_params)  # insert into main parameter list
        # link the camera to its exposure controller. Each camera has its own controller
        driver_parameters.update({cam + '.exposure_controller_name': cam + '.exposure_controller'})
    return driver_parameters

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

    container = ComposableNodeContainer(
        name='cam_sync_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )


    return [container]

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='Active', description='ROS namespace'),
        
        
        # Argumentos do Saver
        DeclareLaunchArgument('enable_saver', default_value='false', description='Ativar gravação de imagens?'),
        DeclareLaunchArgument('save_directory', default_value='/home/jetson/Documents/stereo_images', description='Pasta para salvar imagens'),

        # Argumentos nodos extras
        DeclareLaunchArgument('description', default_value='true', description='Ativar visualização da descrição?'),

        # # Nó de robot_description (visualização)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'mobile_bench.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('stereo_active'), 'launch','gpio_control.launch.py'])
            ]),
            # launch_arguments = {'namespace':LaunchConfiguration('namespace'),
            #                     'stepping_mode': 'full',
            #                     'step_delay': '5',
            #                     'steps_per_rev': '2048',
            #                     'motor_angle_topic': 'motor/angle'}.items(),
        ),

        # # Nó de monitoramento de energia (para o Jetson AGX)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('jetson_power_monitor'), 'launch', 'nano_jetson_power.launch.py'])
            ]),
            launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        ),

        OpaqueFunction(function=launch_setup),
    ])