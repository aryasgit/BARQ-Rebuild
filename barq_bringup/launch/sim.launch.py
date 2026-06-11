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
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('barq_description')
    sim_share = get_package_share_directory('barq_sim')
    bringup_share = get_package_share_directory('barq_bringup')
    rosgz_share = get_package_share_directory('ros_gz_sim')

    world_path = PathJoinSubstitution(
        [sim_share, 'worlds', LaunchConfiguration('world_file')])
    xacro_path = os.path.join(desc_share, 'urdf', 'barq.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' mode:=gazebo']), value_type=str)

    gui = LaunchConfiguration('gui')
    use_gait = LaunchConfiguration('gait')
    use_slam = LaunchConfiguration('slam')
    use_nav = LaunchConfiguration('nav')
    odom_source = LaunchConfiguration('odom_source')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # Let Gazebo resolve package://barq_description/... mesh URIs (GUI rendering).
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                               os.path.dirname(desc_share)),

        DeclareLaunchArgument('world_file', default_value='barq_world.sdf',
                              description='World in barq_sim/worlds (barq_course.sdf = obstacle course)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Run the Gazebo GUI (default headless server)'),
        DeclareLaunchArgument('gait', default_value='false',
                              description='Also launch IK + gait planner (walk via /cmd_vel)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Run slam_toolbox (2D mapping from the sim lidar)'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Run nav2 (autonomous navigation; implies you also want slam:=true)'),
        DeclareLaunchArgument('odom_source', default_value='ground_truth',
                              description='odom->base_link TF source: ground_truth | estimated (legged odometry)'),

        # nav2 stack (plans on the SLAM map, streams /cmd_vel into the gait)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'),
                             'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(bringup_share, 'config', 'barq_nav2.yaml'),
            }.items(),
            condition=IfCondition(use_nav),
        ),

        # Gazebo server (and optional GUI). -r = run immediately. --headless-rendering: EGL
        # context for the gpu_lidar Sensors system without an X display (ogre2-only path).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosgz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s -v 3 --headless-rendering ', world_path]}.items(),
            condition=UnlessCondition(gui),
        ),
        # GUI renderer forced to classic ogre: ogre2 renders black on the Jetson's GL (Tegra).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosgz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -v 3 --render-engine-gui ogre ', world_path]}.items(),
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
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/model/barq/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
            remappings=[('/model/barq/odometry', '/odom_gt'),
                        ('/imu', '/imu/data')],
            output='screen',
        ),

        # Ground-truth TF (odom->base_link, forced frame names). Default odometry source;
        # disabled when odom_source:=estimated hands TF to the state estimator.
        Node(
            package='barq_control',
            executable='odom_tf',
            output='screen',
            parameters=[sim_time],
            remappings=[('/odom', '/odom_gt')],
            condition=IfCondition(PythonExpression(
                ["'", odom_source, "' == 'ground_truth'"])),
        ),

        # Legged-odometry state estimator: always running (publishes /odom_est for A/B
        # against /odom_gt); owns the odom->base_link TF when odom_source:=estimated.
        Node(
            package='barq_control',
            executable='state_estimator',
            output='screen',
            parameters=[sim_time,
                        {'publish_tf': PythonExpression(
                            ["'", odom_source, "' == 'estimated'"])}],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=[os.path.join(bringup_share, 'config', 'barq_slam.yaml'),
                        sim_time],
            condition=IfCondition(use_slam),
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
