"""Stage 4 — BARQ on real hardware (or the Teensy emulator PTY).

Brings up the hardware control stack ONLY (no sim):
  - ros2_control_node with mode:=real (barq_hw/BarqSystem on `device`)
  - robot_state_publisher
  - joint_state_broadcaster + joint_group_position_controller
  - gait:=true adds ik_node + gait_planner -> walk with /cmd_vel

On the robot:        ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0
Against the emulator: ros2 run barq_hw teensy_emulator   # prints PTY /dev/pts/N
                      ros2 launch barq_bringup real.launch.py device:=/dev/pts/N
The launch line is IDENTICAL either way — that is the drop-in property (Stage 4 contract).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('barq_description')
    bringup_share = get_package_share_directory('barq_bringup')
    xacro_path = os.path.join(desc_share, 'urdf', 'barq.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' mode:=real',
                 ' device:=', LaunchConfiguration('device')]), value_type=str)
    controllers_yaml = os.path.join(bringup_share, 'config', 'ros2_controllers.yaml')
    use_gait = LaunchConfiguration('gait')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0',
                              description='Serial device: Teensy USB CDC or emulator PTY'),
        DeclareLaunchArgument('gait', default_value='false',
                              description='Also launch IK + gait planner (walk via /cmd_vel)'),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers_yaml],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_position_controller',
                       '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
        ),
        Node(
            package='barq_control',
            executable='ik_node',
            output='screen',
            condition=IfCondition(use_gait),
        ),
        Node(
            package='barq_control',
            executable='gait_planner',
            output='screen',
            condition=IfCondition(use_gait),
        ),
    ])
