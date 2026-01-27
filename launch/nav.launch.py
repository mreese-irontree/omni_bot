from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')

    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/matt/omni_bot_ws/src/omni_bot/launch/launch_robot.launch.py'),
    )

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_params = '/home/matt/omni_bot_ws/src/omni_bot/config/nav2_params_lidar.yaml'

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_bringup_share) + '/launch/bringup_launch.py'),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml, e.g. /home/matt/maps/room1.yaml'
        ),
        launch_robot,
        nav2,
    ])
