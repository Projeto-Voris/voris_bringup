from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


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
            'use_sim_time': True, 
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )


def launch_setup(context, *args, **kwargs):
    # 1. Configurações Iniciais das Câmeras e Parâmetros
    name_0 = LaunchConfiguration('cam_0_name').perform(context)
    name_1 = LaunchConfiguration('cam_1_name').perform(context)
    frame_0 = LaunchConfiguration('cam_0_frame_id').perform(context)
    frame_1 = LaunchConfiguration('cam_1_frame_id').perform(context)
    gazebo_bridge_file = LaunchConfiguration('gazebo_bridge_file').perform(context)

    # Mapeamento local de variáveis para evitar erros de escopo (NameError)
    topic_left = f"{name_0}/image_raw"
    topic_right = f"{name_1}/image_raw"
    dvl_twist_topic = '/dvl_twist'
    dvl_velocity_topic = '/model/bluerov2/dvl/velocity'

    # Listas de nós para os containers
    composable_nodes = [
        make_gazebo_bridge_node(gazebo_bridge_file)
    ]
    composable_nodes_2 = [] 

    # 2. Configuração do SLAM
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
                    'child_frame_id': frame_0,
                    'use_sim_time': True,
                }],
                remappings=[
                    ('camera/left', topic_left),
                    ('camera/right', topic_right),
                    ('imu', '/imu/data') 
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
                    'child_frame_id': frame_0,
                    'use_sim_time': True,
                    'tf_publish': False,
                }],
                remappings=[
                    ('camera/left', topic_left),
                    ('camera/right', topic_right),
                    ('pose', '/mavros/vision_pose/pose')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        # Correção: Adiciona o slam_node independente da ramificação condicional interna
        composable_nodes.append(slam_node)
    
    # 3. Configuração do Saver Node
    if LaunchConfiguration('enable_saver').perform(context) == 'true':
        saver_node = ComposableNode(
            package='orbslam3_ros2', 
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

    # 4. Configuração do DVL Node
    if LaunchConfiguration('dvl_composable').perform(context) == 'true':
        dvl_node = ComposableNode(
            package='voris_log', 
            plugin='voris_log::DVLNode',
            name='dvl_msg_converter',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('/dvl_twist', dvl_twist_topic),
                ('/model/bluerov2/dvl/velocity', dvl_velocity_topic)
            ],  
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes.append(dvl_node)

    # 5. Configuração de Disparidade e Retificação
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
                'publish_disp': True,
                'use_sim_time': True,
            }],
            remappings=[
                ('left/image_rect', f"{name_0}/image_rect"),
                ('left/camera_info', f"{name_0}/camera_info"),
                ('right/image_rect', f"{name_1}/image_rect"),
                ('right/camera_info', f"{name_1}/camera_info")
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes_2.append(retinify_node)

        triangulation_node = ComposableNode(
            package='passive_stereo',
            plugin='TriangulationNode',
            name='triangulation_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'frame_id': frame_0,
                'sampling_factor': 1
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        composable_nodes_2.append(triangulation_node)

    # 6. Inicialização dos Containers de nós
    container_1 = ComposableNodeContainer(
        name='voris_main_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    container_2 = ComposableNodeContainer(
        name='voris_stereo_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes_2,
        output='screen',
    )

    return [container_1, container_2]


def generate_launch_description():
    # Caminho dinâmico padrão para o arquivo da bridge do Gazebo
    default_bridge_file = PathJoinSubstitution([
        FindPackageShare('voris_log'), 'config', 'gazebo_bridge.yaml'
    ])

    return LaunchDescription([
        # Definição e argumentos aceitos via linha de comando
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument('cam_0_name', default_value='left_cam'),
        DeclareLaunchArgument('cam_1_name', default_value='right_cam'),
        DeclareLaunchArgument('cam_0_frame_id', default_value='left_camera_frame'),
        DeclareLaunchArgument('cam_1_frame_id', default_value='right_camera_frame'),
        DeclareLaunchArgument('gazebo_bridge_file', default_value=default_bridge_file),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('inertial', default_value='false'),
        DeclareLaunchArgument('enable_saver', default_value='false'),
        DeclareLaunchArgument('dvl_composable', default_value='true'),
        DeclareLaunchArgument('disparity', default_value='false'),
        DeclareLaunchArgument('voc_file', default_value=''),
        DeclareLaunchArgument('settings_file', default_value=''),
        DeclareLaunchArgument('save_directory', default_value='/tmp'),
        
        OpaqueFunction(function=launch_setup)
    ])
