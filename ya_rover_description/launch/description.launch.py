import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
	robot_xacro_path = os.path.join(get_package_share_directory('ya_rover_description'), 'urdf/robots', 'ya_rover.urdf.xacro')
	# namespace = LaunchConfiguration('namespace')
	use_sim_time = LaunchConfiguration('use_sim_time')
	
	# declare_namespace_cmd = DeclareLaunchArgument(
    #     name='namespace',
    #     default_value='ya_rover',
    #     description='Top-level namespace'
    # )

	declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

	robot_description = ParameterValue(
        Command(['xacro ', robot_xacro_path]),
        value_type=str
    )

	joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
        # namespace=namespace

	robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )
	        # namespace=namespace


	ld = LaunchDescription()
	# ld.add_action(declare_namespace_cmd)
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(joint_state_publisher_node)
	ld.add_action(robot_state_publisher_node)

	return ld