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
    # --------------------
    # Launch Arguments
    # --------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_lidar = LaunchConfiguration('start_lidar')
    start_camera = LaunchConfiguration('start_camera')
    start_bridge = LaunchConfiguration('start_bridge')

    sensor_delay_sec = LaunchConfiguration('sensor_delay_sec')

    esp32_port = LaunchConfiguration('esp32_port')
    esp32_baud = LaunchConfiguration('esp32_baud')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')
    max_linear_mps = LaunchConfiguration('max_linear_mps')
    timeout_sec = LaunchConfiguration('timeout_sec')

    # --------------------
    # Robot Description (URDF/Xacro)
    # --------------------
    pkg_path = get_package_share_directory('omni_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_params],
    )

    # joint_state_publisher (no real encoders, so this is fine for now)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_params],
    )

    # --------------------
    # LiDAR (LD19)
    # --------------------
    ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    ld19_launch = os.path.join(ldlidar_share, 'launch', 'ld19.launch.py')

    lidar_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        condition=IfCondition(start_lidar),
    )

    # --------------------
    # Arducam ToF (script)
    # --------------------
    tof_script_dir = os.path.expanduser(
        '~/omni_bot_ws/src/Arducam_tof_camera/ros2_publisher/src/arducam/arducam_rclpy_tof_pointcloud/arducam_rclpy_tof_pointcloud'
    )
    tof_script_path = os.path.join(tof_script_dir, 'tof_pointcloud.py')

    camera_action = ExecuteProcess(
        cmd=['python3', tof_script_path],
        cwd=tof_script_dir,
        output='screen',
        condition=IfCondition(start_camera),
    )

    # --------------------
    # cmd_vel -> ESP32 bridge (script)
    # IMPORTANT: Use ExecuteProcess (NOT Node) since it's not installed as a ROS exec
    # --------------------
    bridge_script_path = os.path.expanduser('~/omni_bot_ws/src/omni_bot/scripts/cmdvel_to_esp32.py')
    bridge_action = ExecuteProcess(
        cmd=[
            'python3', bridge_script_path,
            '--ros-args',
            '-p', ['port:=', esp32_port],
            '-p', ['baud:=', esp32_baud],
            '-p', ['wheel_separation_m:=', wheel_separation_m],
            '-p', ['max_linear_mps:=', max_linear_mps],
            '-p', ['timeout_sec:=', timeout_sec],
        ],
        output='screen',
        condition=IfCondition(start_bridge),
    )

    # --------------------
    # Delay Sensors / Bridge (optional)
    # --------------------
    lidar_delayed = TimerAction(period=sensor_delay_sec, actions=[lidar_action])
    camera_delayed = TimerAction(period=sensor_delay_sec, actions=[camera_action])
    bridge_delayed = TimerAction(period=sensor_delay_sec, actions=[bridge_action])

    return LaunchDescription([
        # Common
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),

        # Start toggles
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start LD19 LiDAR if true'),
        DeclareLaunchArgument('start_camera', default_value='true', description='Start Arducam ToF publisher if true'),
        DeclareLaunchArgument('start_bridge', default_value='true', description='Start cmd_vel -> ESP32 bridge if true'),

        # Delay
        DeclareLaunchArgument('sensor_delay_sec', default_value='2.0', description='Delay (seconds) before starting sensors/bridge'),

        # ESP32 bridge params
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0', description='ESP32 serial port'),
        DeclareLaunchArgument('esp32_baud', default_value='115200', description='ESP32 baud rate'),
        DeclareLaunchArgument('wheel_separation_m', default_value='0.30', description='Wheel separation in meters'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.6', description='1.0 command corresponds to this m/s'),
        DeclareLaunchArgument('timeout_sec', default_value='0.5', description='Stop if cmd_vel not received for this long'),

        rsp,
        jsp,
        lidar_delayed,
        camera_delayed,
        bridge_delayed,
    ])
