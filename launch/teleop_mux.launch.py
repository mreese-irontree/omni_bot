import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('omni_bot')

    teleop_yaml = os.path.expanduser('~/omni_bot_ws/src/omni_bot/config/teleop_twist_joy.yaml')
    mux_yaml = os.path.expanduser('~/omni_bot_ws/src/omni_bot/config/twist_mux.yaml')

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

    esp32_bridge = Node(
    package='omni_bot',
    executable='cmdvel_to_esp32.py',
    name='cmdvel_to_esp32',
    output='screen',
    parameters=[{
        'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        'baud': 115200,
        'wheel_separation_m': 0.30,
        'max_linear_mps': 0.6,
        'timeout_sec': 0.5,
    }]
)

    return LaunchDescription([
        joy_node,
        teleop_node,
        mux_node,
    ])