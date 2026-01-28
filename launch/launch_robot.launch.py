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
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_camera = LaunchConfiguration('start_camera')
    start_esp32_bridge = LaunchConfiguration('start_esp32_bridge')
    start_laser_odom = LaunchConfiguration('start_laser_odom')
    sensor_delay_sec = LaunchConfiguration('sensor_delay_sec')

    # Package share + FULL PATHS
    omni_bot_share = get_package_share_directory('omni_bot')

    # URDF
    xacro_file = os.path.join(omni_bot_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # LiDAR include
    ldlidar_share = get_package_share_directory('ldlidar_stl_ros2')
    ld19_launch = os.path.join(ldlidar_share, 'launch', 'ld19.launch.py')
    lidar_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        condition=IfCondition(start_lidar),
    )

    # Depth camera publisher (visualization only)
    tof_script_dir = '/home/matt/omni_bot_ws/src/Arducam_tof_camera/ros2_publisher/src/arducam/arducam_rclpy_tof_pointcloud/arducam_rclpy_tof_pointcloud'
    tof_script_path = tof_script_dir + '/tof_pointcloud.py'
    camera_action = ExecuteProcess(
        cmd=['python3', tof_script_path],
        cwd=tof_script_dir,
        output='screen',
        condition=IfCondition(start_camera),
    )

    # cmd_vel -> ESP32 bridge
    esp32_bridge_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/cmdvel_to_esp32.py'
    esp32_bridge_action = ExecuteProcess(
        cmd=[
            'python3', esp32_bridge_script,
            '--ros-args',
            '-p', 'port:=/dev/ttyUSB0',
            '-p', 'baud:=115200',
            '-p', 'wheel_separation_m:=0.30',
            '-p', 'max_linear_mps:=0.6',
            '-p', 'timeout_sec:=0.5',
        ],
        output='screen',
        condition=IfCondition(start_esp32_bridge),
    )

    # LiDAR scan matching odom (recommended: provides odom->base_link TF + /odom)
    # NOTE: This requires: sudo apt install ros-jazzy-scan-tools
    laser_scan_matcher_node = Node(
        package='laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        condition=IfCondition(start_laser_odom),
        parameters=[{
            'use_sim_time': False,
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'laser_frame': 'base_laser',   # matches your /scan frame_id
            'odom_frame': 'odom',
            'fixed_frame': 'odom',
            'publish_tf': True,
            'publish_odom': True,
        }],
    )

    # Delays
    lidar_delayed = TimerAction(period=sensor_delay_sec, actions=[lidar_action])
    camera_delayed = TimerAction(period=sensor_delay_sec, actions=[camera_action])

    # Start bridges + laser odom together after delay
    control_delayed = TimerAction(
        period=sensor_delay_sec,
        actions=[esp32_bridge_action, laser_scan_matcher_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_esp32_bridge', default_value='true'),
        DeclareLaunchArgument('start_laser_odom', default_value='true'),
        DeclareLaunchArgument('sensor_delay_sec', default_value='2.0'),

        rsp_node,
        jsp_node,
        lidar_delayed,
        camera_delayed,
        control_delayed,
    ])
