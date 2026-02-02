import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_camera = LaunchConfiguration('start_camera')
    start_teleop = LaunchConfiguration('start_teleop')
    start_slam = LaunchConfiguration('start_slam')

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    esp_port = LaunchConfiguration('esp_port')
    lidar_port = LaunchConfiguration('lidar_port')

    sensor_delay_sec = LaunchConfiguration('sensor_delay_sec')

    pkg_path = get_package_share_directory('omni_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml(),
              'use_sim_time': use_sim_time}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # LiDAR driver
    ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    ld19_launch = os.path.join(ldlidar_share, 'launch', 'ld19.launch.py')
    lidar_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        launch_arguments={'port_name': lidar_port}.items(),
        condition=IfCondition(start_lidar),
    )

    # Optional ToF publisher (your existing script)
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

    lidar_delayed = TimerAction(period=sensor_delay_sec, actions=[lidar_action])
    camera_delayed = TimerAction(period=sensor_delay_sec, actions=[camera_action])

    # Teleop (Xbox)
    joy = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        condition=IfCondition(start_teleop),
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        condition=IfCondition(start_teleop),
        parameters=[{
            # Default mappings usually work; tune later
            'axis_linear.x': 1,
            'scale_linear.x': 0.25,
            'axis_angular.yaw': 3,
            'scale_angular.yaw': 1.5,
            'enable_button': 4,   # LB
        }],
    )

    # Dead reckoning odom from cmd_vel
    cmdvel_to_odom = Node(
        package='omni_bot',
        executable='cmdvel_to_odom.py',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'publish_hz': 50.0,
            'cmd_timeout_ms': 300,
        }],
    )

    # SLAM
    slam_params = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(start_slam),
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    # CmdVel -> ESP32 serial
    cmdvel_to_serial = Node(
        package='omni_bot',
        executable='cmdvel_to_serial.py',
        output='screen',
        parameters=[{
            'port': esp_port,
            'baud': serial_baud,
            'wheel_base': 0.22,
            'max_lin': 0.35,
            'max_ang': 2.5,
            'send_rate_hz': 20.0,
            'stop_on_timeout_ms': 400,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_teleop', default_value='true'),
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('sensor_delay_sec', default_value='1.0'),

        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('esp_port', default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0', description='ESP32 serial device path'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0', description='LD19 LiDAR serial device path'),

        rsp,
        lidar_delayed,
        camera_delayed,

        joy,
        teleop,
        cmdvel_to_odom,
        slam,
        cmdvel_to_serial,
    ])
