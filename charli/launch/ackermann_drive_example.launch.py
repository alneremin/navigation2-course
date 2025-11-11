from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('charli'),
                    performed_description_format,
                    f'supercharli.xacro.{performed_description_format}'
                ]),
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('charli'),
            'config',
            'ackermann_drive_controller.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'charli', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   '--controller-ros-args',
                   '-r /ackermann_steering_controller/tf_odometry:=/tf',
                   ],
    )
    
    # World configuration
    world_path = LaunchConfiguration('world_path')
    declare_world_path_cmd = DeclareLaunchArgument(
        name="world_path",
        default_value=os.path.join(get_package_share_directory("charli"), "worlds", "empty_world.world"),
        description="Gazebo world"
    )

    # Bridge
    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(get_package_share_directory('charli'), 'config', 'bridge.yaml')}]
    )

    # Gazebo launch with world file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [' -r -v 1 ', world_path])]
    )

    ld = LaunchDescription([
        # Launch Arguments
        declare_world_path_cmd,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
        
        # Gazebo simulation
        gazebo_launch,
        
        # Bridge
        gazebo_ros_bridge_cmd,
        
        # Spawn entity
        gz_spawn_entity,
        
        # Event handlers for controller spawning
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
    ])
    
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld