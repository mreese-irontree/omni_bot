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

    # Process URDF File
    pkg_path = get_package_share_directory('omni_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }


    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )


    # joint_state_publisher
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=['params'],
    )


    # LIDAR
    #ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    #ld19_launch = os.path.join(ldlidar_share, 'launch', 'ld19.launch.py')

    LIDAR_PORT = '/dev/serial/by-path/platform-fd500000.pcie-pci-000:01:00.0-usb-0:1.1:1.0-port0'
    LIDAR_BAUD = '230400'

    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros_node',
        name='ldlidar',
        output='screen',
        parameters=[{'port_name': LIDAR_PORT, 'port_baudrate': LIDAR_BAUD, 'use_sim_time': use_sim_time,}],
        condition=IfCondition(start_lidar),
    )

    #lidar_action = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(ld19_launch),
    #    launch_arguments={'serial_port': LIDAR_PORT, 'serial_baudrate': LIDAR_BAUD,}.items(),
    #    condition=IfCondition(start_lidar),
    #)


    # CAMERA
    tof_script_dir = os.path.expanduser('~/omni_bot_ws/src/Arducam_tof_camera/ros2_publisher/src/arducam/arducam_rclpy_tof_pointcloud/arducam_rclpy_tof_pointcloud')
    tof_script_path = os.path.join(tof_script_dir, 'tof_pointcloud.py')

    camera_action = ExecuteProcess(
        cmd=['python3', tof_script_path],
        cwd=tof_script_dir,
        output='screen',
        condition=IfCondition(start_camera),
    )


    # Delay Sensors
    lidar_delayed = TimerAction(
        period=sensor_delay_sec,
        actions=[lidar_node]
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

        rsp,
        jsp,
        lidar_delayed,
        camera_delayed,
    ])