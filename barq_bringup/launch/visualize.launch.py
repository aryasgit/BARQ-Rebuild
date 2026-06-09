"""Visualise BARQ in RViz with joint sliders -- no hardware required (Stage 2A).

Launches:
  - robot_state_publisher    publishes TF from the URDF + /robot_description
  - joint_state_publisher_gui  sliders to drive all 12 joints (toggle: gui:=false)
  - rviz2                    preloaded with the BARQ config

Usage:
  ros2 launch barq_bringup visualize.launch.py
  ros2 launch barq_bringup visualize.launch.py gui:=false
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

    urdf_path = os.path.join(desc_share, 'urdf', 'barq.urdf.xacro')
    rviz_path = os.path.join(bringup_share, 'rviz', 'barq.rviz')

    # xacro passes a plain URDF through unchanged; keeps us future-proof if we
    # later parameterise the description.
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)

    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch joint_state_publisher_gui sliders'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path],
            output='screen',
        ),
    ])
