from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        name='world',
        description='Path to the world file to open.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('gz_control_hw_demo'), 'worlds', 'empty.world']))

    # Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'])),
        launch_arguments={
            'ign_args': LaunchConfiguration('world'),
        }.items())

    # Create model
    # Run xacro to convert bot.urdf to a string containing the robot description
    xacro_path = PathJoinSubstitution(
        [FindPackageShare('gz_control_hw_demo'), 'urdf', 'bot.urdf'])
    create_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-string', Command(['xacro ', xacro_path])])

    # Joint State Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                ],
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        create_node,
        # bridge,
    ])
