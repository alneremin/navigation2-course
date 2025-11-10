import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command


def image_transport_republisher(transport, camera_topics):
    base_topic = camera_topics.split('/')[-1]
    return Node(
        package='image_transport',
        executable='republish',
        name=f'image_transport_republish_{transport}_{base_topic}',
        arguments=['raw', transport],
        remappings=[
            ('in', f'/camera/{camera_topics}'),
            (f'out/{transport}', f'/camera/{camera_topics}/{transport}'),
        ],
    )


def generate_launch_description():
    # --- Определение пакетов по новой структуре ---
    edubot_gazebo_dir = get_package_share_directory('edubot_gazebo')
    edubot_description_dir = get_package_share_directory('edubot_description')
    edubot_control_dir = get_package_share_directory('edubot_control')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_gz_sim = LaunchConfiguration('use_ros2_control_gz_sim')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock'
    )
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='If true, use ros2_control'
    )
    declare_use_ros2_control_gz_sim = DeclareLaunchArgument(
        'use_ros2_control_gz_sim',
        default_value='true',
        description='If true, use ros2_control in gz_sim'
    )

    # --- Обновлённые пути к файлам ---
    robot_description_xacro_file = os.path.join(
        edubot_description_dir,
        'description',
        'robot.urdf.xacro'
    )

    world_file_path = os.path.join(
        edubot_gazebo_dir,
        'worlds',
        'empty.sdf'
    )

    rviz_config_file = os.path.join(
        edubot_gazebo_dir,
        'config',
        'sim_config.rviz'
    )

    robot_controllers_file = os.path.join(
        edubot_control_dir,
        'config',
        'controller_gz_sim.yaml'
    )

    gazebo_params_file = os.path.join(
        edubot_gazebo_dir,
        'config',
        'gz_params.yaml'
    )

    twist_mux_params_file = os.path.join(
        edubot_gazebo_dir,  # предполагаем, что twist_mux.yaml остаётся в edubot_gazebo/config
        'config',
        'twist_mux.yaml'
    )

    # robot_state_publisher setup
    robot_description_config = Command([
        'xacro ',
        robot_description_xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' use_ros2_control_gz_sim:=', use_ros2_control_gz_sim,
    ])

    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),
        'use_sim_time': use_sim_time,
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # General
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Gazebo_Control
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Lidar
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # RGBD Camera
            '/camera/depth/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Image Transport Republishers
    depth_camera = 'depth/image_raw'
    image_transports = ['compressed', 'compressedDepth', 'theora', 'zstd']
    node_image_republishers = [
        image_transport_republisher(transport, depth_camera)
        for transport in image_transports
    ]

    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_file_path}',
            'extra_gazebo_args': f'--ros-args --params-file {gazebo_params_file}'
        }.items()
    )

    # Spawn robot entity
    node_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'edubot_gazebo',
            '-allow_renaming', 'true',
            '-z', '0.1'
        ],
    )

    # RViz2
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='both'
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", robot_controllers_file],
    )

    # Twist mux & stamper
    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 'diff_drive_controller/cmd_vel'),
        ],
    )

    node_twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel_smoothed'),
            ('cmd_vel_out', 'nav_vel'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Event handlers for controller spawning
    register_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_gz_spawn_entity,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    register_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[diff_drive_controller_spawner],
        )
    )

    group_spawn_gz = GroupAction([
        gazebo,
        node_gz_spawn_entity,
    ])

    # Launch description
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(declare_use_ros2_control_gz_sim)

    ld.add_action(register_joint_state_broadcaster_spawner)
    ld.add_action(register_diff_drive_controller_spawner)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_gz_bridge)
    ld.add_action(node_twist_mux)
    ld.add_action(node_twist_stamper)
    for republisher in node_image_republishers:
        ld.add_action(republisher)
    ld.add_action(group_spawn_gz)
    ld.add_action(node_rviz2)

    return ld