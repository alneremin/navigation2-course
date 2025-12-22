import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', "f1")}
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='f1',
        description='Top-level namespace'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=os.path.join(
            get_package_share_directory('f1_navigation'),
            'config',
            'rviz.rviz'
        ),
        description='RViz config file'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(
            get_package_share_directory('f1_navigation'),
            'config',
            'navigation.yaml'
        ),
        description='Nav2 navigation parameters'
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )

    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )

    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [
            ('cmd_vel', 'ackermann_steering_controller/reference')
        ],
    )

    behavior_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [
            ('cmd_vel', 'ackermann_steering_controller/reference')
        ],
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
    )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     parameters=[{'use_sim_time': True}],
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_cmd)

    ld.add_action(planner_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(behavior_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(lifecycle_manager_cmd)
    # ld.add_action(rviz)


    return ld
