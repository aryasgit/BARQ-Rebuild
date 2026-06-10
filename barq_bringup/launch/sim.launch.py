"""Stage 2E — BARQ in Gazebo Fortress with physics + ground contact.

Brings up:
  - gz sim (server only by default; gui:=true adds the Gazebo GUI window)
  - robot_state_publisher        (xacro mode:=gazebo -> ign_ros2_control hardware)
  - ros_gz_sim create            (spawns BARQ from /robot_description, z=0.25)
  - /clock bridge                (sim time into ROS)
  - joint_state_broadcaster + joint_group_position_controller
    (controller_manager runs INSIDE Gazebo via the ign_ros2_control plugin)
  - gait:=true adds ik_node + gait_planner (use_sim_time) -> walk with /cmd_vel

Try it:
  ros2 launch barq_bringup sim.launch.py gait:=true
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('barq_description')
    sim_share = get_package_share_directory('barq_sim')
    rosgz_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(sim_share, 'worlds', 'barq_world.sdf')
    xacro_path = os.path.join(desc_share, 'urdf', 'barq.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' mode:=gazebo']), value_type=str)

    gui = LaunchConfiguration('gui')
    use_gait = LaunchConfiguration('gait')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Run the Gazebo GUI (default headless server)'),
        DeclareLaunchArgument('gait', default_value='false',
                              description='Also launch IK + gait planner (walk via /cmd_vel)'),

        # Gazebo server (and optional GUI). -r = run immediately.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosgz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r -s -v 3 {world}'}.items(),
            condition=UnlessCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosgz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r -v 3 {world}'}.items(),
            condition=IfCondition(gui),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, sim_time],
        ),

        # Spawn just above the standing height (stance body z ~= 0.142): a ~3 cm drop onto
        # legs already at the stance pose (initial_value), no collapse/snap transient.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'barq', '-z', '0.17'],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_position_controller',
                       '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120'],
        ),

        Node(
            package='barq_control',
            executable='ik_node',
            output='screen',
            parameters=[sim_time],
            condition=IfCondition(use_gait),
        ),

        Node(
            package='barq_control',
            executable='gait_planner',
            output='screen',
            parameters=[sim_time],
            condition=IfCondition(use_gait),
        ),
    ])
