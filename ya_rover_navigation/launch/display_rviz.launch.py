from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'ya_rover_navigation',
        'share',
        'ya_rover_navigation'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    rviz_config_path = os.path.join(pkg_share, 'config', 'nav_config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    )

    return LaunchDescription([declare_use_sim_time_cmd, 
                                                   rviz_node])
