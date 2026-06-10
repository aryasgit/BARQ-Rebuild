"""Stage 2B — full ros2_control loop against mock hardware (no robot needed).

Brings up:
  - robot_state_publisher   (TF from the xacro, mode:=mock)
  - controller_manager      (ros2_control_node, mock_components/GenericSystem)
  - joint_state_broadcaster (publishes /joint_states from the hardware state interfaces)
  - joint_group_position_controller (accepts 12 joint targets on
        /joint_group_position_controller/commands as a Float64MultiArray)
  - rviz2                   (optional: rviz:=false to skip)

Try it:
  ros2 launch barq_bringup control.launch.py
  ros2 control list_controllers
  ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray \\
    "{data: [0,0.3,-0.6, 0,0.3,-0.6, 0,0.3,-0.6, 0,0.3,-0.6]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('barq_description')
    bringup_share = get_package_share_directory('barq_bringup')

    xacro_path = os.path.join(desc_share, 'urdf', 'barq.urdf.xacro')
    controllers = os.path.join(bringup_share, 'config', 'ros2_controllers.yaml')
    rviz_path = os.path.join(bringup_share, 'rviz', 'barq.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' mode:=mock']), value_type=str)

    use_rviz = LaunchConfiguration('rviz')
    use_ik = LaunchConfiguration('ik')
    use_gait = LaunchConfiguration('gait')
    # gait implies ik (gait feeds /foot_targets -> ik -> controller)
    ik_or_gait = PythonExpression(["'", use_ik, "' == 'true' or '", use_gait, "' == 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('ik', default_value='false',
                              description='Also launch the IK node (Stage 2C)'),
        DeclareLaunchArgument('gait', default_value='false',
                              description='Also launch the gait planner (Stage 2D); implies ik'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_position_controller',
                       '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            condition=IfCondition(use_rviz),
        ),

        Node(
            package='barq_control',
            executable='ik_node',
            output='screen',
            condition=IfCondition(ik_or_gait),
        ),

        Node(
            package='barq_control',
            executable='gait_planner',
            output='screen',
            condition=IfCondition(use_gait),
        ),
    ])
