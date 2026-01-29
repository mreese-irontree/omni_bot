from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Args
    use_nav2 = LaunchConfiguration('use_nav2')   # false => wander mode
    map_filename = LaunchConfiguration('map')    # used only when use_nav2:=true

    map_directory = '/home/matt/omni_bot_maps'
    map_full_path = PathJoinSubstitution([map_directory, map_filename])

    # Bring up robot stack (URDF, lidar, cmdvel bridges, etc.)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/matt/omni_bot_ws/src/omni_bot/launch/launch_robot.launch.py'
        )
    )

    # -------- Wander (LiDAR-only) --------
    wander_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/wander_lidar.py'
    wander = ExecuteProcess(
        cmd=[
            'python3', wander_script,
            '--ros-args',
            '-p', 'scan_topic:=/scan',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'forward_speed:=0.12',
            '-p', 'turn_speed:=0.65',
            '-p', 'front_stop_m:=0.55',
            '-p', 'side_clear_m:=0.40',
            '-p', 'rate_hz:=15.0',
        ],
        output='screen',
        condition=IfCondition(use_nav2.__invert__()),  # run wander when use_nav2 is false
    )

    # -------- Optional Nav2 bringup (requires a MAP) --------
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
        condition=IfCondition(use_nav2),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='false = LiDAR wander/avoid mode, true = Nav2 bringup (requires map)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='garage.yaml',
            description='Map filename inside /home/matt/omni_bot_maps (only used when use_nav2:=true)'
        ),
        launch_robot,
        wander,
        nav2,
    ])
