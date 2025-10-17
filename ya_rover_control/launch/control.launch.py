from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('ya_rover_control')
    controller_config = os.path.join(pkg_share, 'config', 'rover_controllers.yaml')
    namespace = LaunchConfiguration('namespace')

    # Declare the launch arguments
    # declare_namespace_cmd = DeclareLaunchArgument(
    #     name='namespace',
    #     default_value='ya_rover',
    #     description='Top-level namespace'
    # )

    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[controller_config],
    #     output='screen'
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "100", 
             "-c",  '/controller_manager', 
             'joint_state_broadcaster',
             ],
        output='screen',
    )

    bicycle_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "100", 
             "-c",'/controller_manager', 
             'diff_drive_controller',
             ],
        # remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
        output='screen',
    )

    ld = LaunchDescription()
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(bicycle_steering_controller_spawner)

    return ld