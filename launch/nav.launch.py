from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Args
    use_nav2 = LaunchConfiguration('use_nav2')   # "true" or "false"
    map_filename = LaunchConfiguration('map')    # used only when use_nav2:=true

    map_directory = '/home/matt/omni_bot_maps'
    map_full_path = PathJoinSubstitution([map_directory, map_filename])

    # Bring up robot stack (URDF, lidar, cmdvel bridges, etc.)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/matt/omni_bot_ws/src/omni_bot/launch/launch_robot.launch.py'
        )
    )

    # -------- Wander (LiDAR wall-follow + explore) --------
    wander_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/wander_wall_follow.py'
    wander = ExecuteProcess(
        cmd=[
            'python3', wander_script,
            '--ros-args',
            '-p', 'scan_topic:=/scan',
            '-p', 'cmd_vel_topic:=/cmd_vel_raw',
            '-p', 'forward_speed:=0.14',
            '-p', 'turn_speed:=0.80',
            '-p', 'front_stop_m:=0.55',
            '-p', 'front_slow_m:=0.80',
            '-p', 'wall_target_m:=0.55',
            '-p', 'wall_kp:=1.2',
            '-p', 'rate_hz:=15.0',
        ],
        output='screen',
        condition=UnlessCondition(use_nav2),
    )

    # -------- Depth safety gate (low obstacles) --------
    depth_filter_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/depth_safety_filter.py'
    depth_filter = ExecuteProcess(
        cmd=[
            'python3', depth_filter_script,
            '--ros-args',
            '-p', 'cmd_vel_in:=/cmd_vel_raw',
            '-p', 'cmd_vel_out:=/cmd_vel',
            '-p', 'cloud_topic:=/point_cloud',
            '-p', 'stop_dist_m:=0.35',
            '-p', 'slow_dist_m:=0.55',
            '-p', 'min_z:=0.05',
            '-p', 'max_z:=0.70',
        ],
        output='screen',
        condition=UnlessCondition(use_nav2),
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
        condition=IfCondition(use_nav2),  # run when use_nav2 is true
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
        depth_filter,
        nav2,
    ])
