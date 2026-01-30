import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_esp32_bridge = LaunchConfiguration('start_esp32_bridge')
    start_cmdvel_odom = LaunchConfiguration('start_cmdvel_odom')
    sensor_delay_sec = LaunchConfiguration('sensor_delay_sec')

    omni_bot_share = get_package_share_directory('omni_bot')

    # robot_state_publisher
    xacro_file = os.path.join(omni_bot_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # joint_state_publisher (for RViz joints)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # LiDAR
    ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    ld19_launch = os.path.join(ldlidar_share, 'launch', 'ld19.launch.py')
    lidar_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        condition=IfCondition(start_lidar),
    )
    lidar_delayed = TimerAction(period=sensor_delay_sec, actions=[lidar_action])

    # cmd_vel -> ESP32
    esp32_bridge_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/cmdvel_to_esp32.py'
    esp32_bridge_action = ExecuteProcess(
        cmd=[
            'python3', esp32_bridge_script,
            '--ros-args',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'port:=/dev/ttyUSB0',
            '-p', 'baud:=115200',
            '-p', 'wheel_separation_m:=0.30',
            '-p', 'max_linear_mps:=0.6',
            '-p', 'timeout_sec:=0.5',
            # Breakaway behavior:
            '-p', 'deadband:=0.05',
            '-p', 'min_output:=0.50',
            '-p', 'start_boost:=0.65',
            '-p', 'start_boost_ms:=200',
        ],
        output='screen',
        condition=IfCondition(start_esp32_bridge),
    )

    # cmd_vel -> odom TF
    cmdvel_odom_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/cmdvel_to_odom.py'
    cmdvel_odom_action = ExecuteProcess(
        cmd=[
            'python3', cmdvel_odom_script,
            '--ros-args',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'odom_topic:=/odom',
            '-p', 'base_frame_id:=base_link',
            '-p', 'odom_frame_id:=odom',
            '-p', 'publish_tf:=true',
            '-p', 'timeout_sec:=0.5',
            '-p', 'rate_hz:=30.0',
        ],
        output='screen',
        condition=IfCondition(start_cmdvel_odom),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_esp32_bridge', default_value='true'),
        DeclareLaunchArgument('start_cmdvel_odom', default_value='true'),
        DeclareLaunchArgument('sensor_delay_sec', default_value='2.0'),

        rsp_node,
        jsp_node,
        cmdvel_odom_action,
        esp32_bridge_action,
        lidar_delayed,
    ])
