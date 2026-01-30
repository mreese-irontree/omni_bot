from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Bring up robot stack (URDF, lidar, cmdvel bridges, etc.)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/matt/omni_bot_ws/src/omni_bot/launch/launch_robot.launch.py'
        )
    )

    # LiDAR wall-follow wander (publishes directly to /cmd_vel)
    wander_script = '/home/matt/omni_bot_ws/src/omni_bot/scripts/wander_lidar.py'
    wander = ExecuteProcess(
        cmd=[
            'python3', wander_script,
            '--ros-args',
            '-p', 'scan_topic:=/scan',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'forward_speed:=0.14',
            '-p', 'turn_speed:=0.80',
            '-p', 'front_stop_m:=0.55',
            '-p', 'front_slow_m:=0.90',
            '-p', 'wall_target_m:=0.55',
            '-p', 'wall_kp:=1.2',
            '-p', 'bias_change_sec:=8.0',
            '-p', 'bias_max:=0.30',
            '-p', 'rate_hz:=15.0',
            '-p', 'scan_timeout_sec:=0.35',
        ],
        output='screen',
    )

    return LaunchDescription([
        launch_robot,
        wander,
    ])
