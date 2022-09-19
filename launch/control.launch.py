from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Run xacro to convert bot.urdf to a string containing the robot description
    xacro_path = PathJoinSubstitution(
        [FindPackageShare('gz_control_hw_demo'), 'urdf', 'bot.urdf'])

    # Controller Manager node
    controller_config = PathJoinSubstitution(
        [FindPackageShare('gz_control_hw_demo'), 'control', 'controllers.yaml'])
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(Command(['xacro ', xacro_path]))},
            controller_config],

    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Spawn joint_group_position_controller
    spawn_joint_group_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller'],
    )

    return LaunchDescription([
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_joint_group_position_controller,
    ])
