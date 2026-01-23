import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('omni_bot')

    teleop_yaml = os.path.join(pkg, 'config', 'teleop_twist_joy.yaml')
    mux_yaml = os.path.join(pkg, 'config', 'twist_mux.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}] 
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_yaml],
        remappings=[('/cmd_vel', '/cmd_vel_manual'),],
    )

    mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_yaml],
        remappings=[('/cmd_vel_out', '/cmd_vel'),],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        mux_node,
    ])