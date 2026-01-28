from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/matt/omni_bot_ws/src/omni_bot/launch/launch_robot.launch.py'),
    )

    slam_params = '/home/matt/omni_bot_ws/src/omni_bot/config/slam_toolbox.yaml'

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    return LaunchDescription([
        launch_robot,
        slam_toolbox_node,
        lifecycle_manager,
    ])
