# Copyright 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        output='screen'
    )

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

    # # Spawn joint_trajectory_controller
    # spawn_joint_trajectory_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_trajectory_controller'],
    # )

    # # Spawn joint_group_velocity_controller
    # spawn_joint_group_velocity_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_group_velocity_controller'],
    # )

    return LaunchDescription([
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_joint_group_position_controller,
        # spawn_joint_group_velocity_controller,
        # spawn_joint_trajectory_controller,
    ])
