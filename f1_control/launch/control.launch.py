from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='f1',
        description='Top-level namespace'
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["--controller-manager-timeout", "100", 
    #          "-c", [namespace, '/controller_manager'], 
    #          'joint_state_broadcaster',
    #          ],
    #     output='screen',
    # )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('f1_control'),
            'config',
            'ackermann_drive_controller.yaml',
        ]
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "100", 
             "-c", [namespace, '/controller_manager'], 
             'joint_state_broadcaster',
             ],
        output='screen',
    )

    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "100", 
             "-c", [namespace, '/controller_manager'], 
             'ackermann_steering_controller',
             ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(ackermann_steering_controller_spawner)

    return ld