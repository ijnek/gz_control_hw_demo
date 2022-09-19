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
    create_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description'])

    # Joint State Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                'joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                ],
    )

    # Robot State Publisher
    # Run xacro to convert bot.urdf to a string containing the robot description
    xacro_path = PathJoinSubstitution(
        [FindPackageShare('gz_control_hw_demo'), 'urdf', 'bot.urdf'])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_path]))}]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        create_node,
        bridge,
        robot_state_publisher_node,
    ])
