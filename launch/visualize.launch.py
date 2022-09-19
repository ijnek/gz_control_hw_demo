from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Run xacro to convert quasor.xacro to a string containing the robot description
    xacro_path = PathJoinSubstitution(
        [FindPackageShare('gz_control_hw_demo'), 'urdf', 'bot.urdf'])

    # Set up robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_path]))}]
    )

    # Set up joint state publisher gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Set up rviz2, with configuration file path
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('gz_control_hw_demo'), 'rviz', 'visualize.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
