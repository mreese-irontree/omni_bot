from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    esp_port = LaunchConfiguration("esp_port")
    esp_baud = LaunchConfiguration("esp_baud")
    lidar_port = LaunchConfiguration("lidar_port")
    follow_side = LaunchConfiguration("follow_side")

    omni_repo = "/home/matt/omni_bot_ws/src/omni_bot"
    scripts_dir = omni_repo + "/scripts"

    wall_follower = scripts_dir + "/wall_follower.py"
    cmd_to_esp = scripts_dir + "/cmdvel_to_esp32.py"

    return LaunchDescription([
        # --- Ports ---
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0",
            description="LD19 serial port (use /dev/serial/by-path to keep stable)"
        ),
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0",
            description="ESP32 serial port (use /dev/serial/by-path to keep stable)"
        ),
        DeclareLaunchArgument("esp_baud", default_value="115200"),
        DeclareLaunchArgument("follow_side", default_value="left"),

        # --- LD19 LiDAR node (port controlled here) ---
        Node(
            package="ldlidar_stl_ros2",
            executable="ldlidar_stl_ros2_node",
            name="LD19",
            output="screen",
            parameters=[{
                "product_name": "LDLiDAR_LD19",
                "topic_name": "scan",
                "frame_id": "base_laser",
                "port_name": lidar_port,
                "port_baudrate": 230400,
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
            }],
        ),

        # --- Wall follower (publishes /cmd_vel) ---
        ExecuteProcess(
            cmd=[
                "python3", wall_follower,
                "--ros-args",
                "-p", "scan_topic:=/scan",
                "-p", "cmd_topic:=/cmd_vel",

                # set this once using your debug output
                # if it says nearest wall is around -90deg when wall is in FRONT, keep -90
                "-p", "scan_yaw_offset_deg:=-90.0",

                "-p", ["follow_side:=", follow_side],

                "-p", "desired_dist:=0.35",
                "-p", "v_nom:=0.18",
                "-p", "v_min:=0.06",
                "-p", "w_max:=1.2",

                # corner / clutter behavior
                "-p", "front_stop:=0.32",
                "-p", "front_slow:=0.85",
                "-p", "corner_turn_gain:=1.2",

                # smoothing
                "-p", "cmd_smooth_tau:=0.20",
                "-p", "publish_rate_hz:=20.0",

                # debug
                "-p", "debug_print_nearest:=true",
                "-p", "debug_print_every_s:=1.0",
            ],
            output="screen",
        ),

        # --- Serial → ESP32 (subscribes /cmd_vel) ---
        ExecuteProcess(
            cmd=[
                "python3", cmd_to_esp,
                "--ros-args",
                "-p", "cmd_topic:=/cmd_vel",
                "-p", ["port:=", esp_port],
                "-p", ["baud:=", esp_baud],
                "-p", "rate_hz:=20.0",
                "-p", "cmd_timeout_s:=0.6",

                # scale to your robot speed
                "-p", "v_to_cmd:=1.0",
                "-p", "w_to_cmd:=0.7",

                # protect against tiny jitter causing spin
                "-p", "deadband:=0.03",
            ],
            output="screen",
        ),
    ])
