from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # User will enter: ros2 launch nav.launch.py map:=example_map.yaml
    map_filename = LaunchConfiguration('map')
    map_directory = '/home/matt/omni_bot_maps'
    map_full_path = PathJoinSubstitution([map_directory, map_filename])

    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/matt/omni_bot_ws/src/omni_bot/launch/launch_robot.launch.py'
        )
    )

    # Nav2 bringup
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_params = '/home/matt/omni_bot_ws/src/omni_bot/config/nav2_params_lidar.yaml'

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_bringup_share) + '/launch/bringup_launch.py'
        ),
        launch_arguments={
            'map': map_full_path,
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Map filename inside /home/matt/omni_bot_maps (example: garage.yaml)'
        ),
        launch_robot,
        nav2,
    ])
