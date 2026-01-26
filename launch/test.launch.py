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

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_camera = LaunchConfiguration('start_camera')
    sensor_delay_sec = LaunchConfiguration('sensor_delay_sec')

    start_esp32_bridge = LaunchConfiguration('start_esp32_bridge')
    esp32_port = LaunchConfiguration('esp32_port')
    esp32_baud = LaunchConfiguration('esp32_baud')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')
    max_linear_mps = LaunchConfiguration('max_linear_mps')
    esp32_timeout_sec = LaunchConfiguration('esp32_timeout_sec')

    # Process URDF File
    pkg_path = get_package_share_directory('omni_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # robot_state_publisher (TF from URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_params],
    )

    # joint_state_publisher (optional — mainly for moving joints in RViz if you have them)
    # For a real robot without encoders/joint states, this doesn't actually animate wheels.
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # LIDAR
    ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    ld19_launch = os.path.join(ldlidar_share, 'launch', 'ld19.launch.py')
    lidar_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        condition=IfCondition(start_lidar),
    )

    # CAMERA
    tof_script_dir = os.path.expanduser(
        '~/omni_bot_ws/src/Arducam_tof_camera/ros2_publisher/src/arducam/'
        'arducam_rclpy_tof_pointcloud/arducam_rclpy_tof_pointcloud'
    )
    tof_script_path = os.path.join(tof_script_dir, 'tof_pointcloud.py')
    camera_action = ExecuteProcess(
        cmd=['python3', tof_script_path],
        cwd=tof_script_dir,
        output='screen',
        condition=IfCondition(start_camera),
    )

    # ESP32 cmd_vel bridge (Nav2/SLAM publishes /cmd_vel -> ESP32 in AUTO mode)
    esp32_bridge = Node(
        package='omni_bot',
        executable='cmdvel_to_esp32.py',
        name='cmdvel_to_esp32',
        output='screen',
        condition=IfCondition(start_esp32_bridge),
        parameters=[{
            'port': esp32_port,
            'baud': esp32_baud,
            'wheel_separation_m': wheel_separation_m,
            'max_linear_mps': max_linear_mps,
            'timeout_sec': esp32_timeout_sec,
        }]
    )

    # Delay Sensors
    lidar_delayed = TimerAction(
        period=sensor_delay_sec,
        actions=[lidar_action]
    )

    camera_delayed = TimerAction(
        period=sensor_delay_sec,
        actions=[camera_action],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),

        DeclareLaunchArgument('start_lidar', default_value='true', description='Start LD19 LiDAR if true'),
        DeclareLaunchArgument('start_camera', default_value='true', description='Start Arducam ToF publisher if true'),
        DeclareLaunchArgument('sensor_delay_sec', default_value='2.0', description='Delay (seconds) before starting sensors'),

        # ESP32 bridge args
        DeclareLaunchArgument('start_esp32_bridge', default_value='true', description='Start cmd_vel -> ESP32 bridge'),
        DeclareLaunchArgument(
            'esp32_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='ESP32 serial port (use /dev/serial/by-id/*)'
        ),
        DeclareLaunchArgument('esp32_baud', default_value='115200', description='ESP32 serial baud rate'),
        DeclareLaunchArgument('wheel_separation_m', default_value='0.30', description='Wheel separation (m) for skid kinematics'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.60', description='Max linear speed that maps to command 1.0'),
        DeclareLaunchArgument('esp32_timeout_sec', default_value='0.5', description='Stop if no cmd_vel seen'),

        rsp,
        jsp,            # optional; keep for now
        lidar_delayed,
        camera_delayed,
        esp32_bridge,
    ])
