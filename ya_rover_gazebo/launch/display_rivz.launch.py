from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'ya_rover_description',
        'share',
        'ya_rover_description'
    )
    xacro_file = os.path.join(pkg_share, 'urdf', 'robots', 'ya_rover.urdf.xacro')
    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value=xacro_file,
            description='Absolute path to robot xacro file'
        )    ]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    model_arg = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command(['xacro ', model_arg])
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])
    
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}])
    
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'ya_rover_rviz.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    )

    return LaunchDescription(declared_arguments + [declare_use_sim_time_cmd, 
                                                   robot_state_publisher_node, 
                                                   start_joint_state_publisher_cmd, 
                                                   start_joint_state_publisher_gui_cmd, 
                                                   rviz_node])
