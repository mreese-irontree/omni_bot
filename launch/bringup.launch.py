from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os


def generate_launch_description():
    esp_port = LaunchConfiguration("esp_port")
    esp_baud = LaunchConfiguration("esp_baud")
    lidar_port = LaunchConfiguration("lidar_port")
    follow_side = LaunchConfiguration("follow_side")
    scan_yaw_offset_deg = LaunchConfiguration("scan_yaw_offset_deg")
    invert_turn = LaunchConfiguration("invert_turn")

    omni_repo = "/home/matt/omni_bot_ws/src/omni_bot"
    scripts_dir = os.path.join(omni_repo, "scripts")
    description_dir = os.path.join(omni_repo, "description")

    wall_follower = os.path.join(scripts_dir, "wall_follower.py")
    cmd_mux = os.path.join(scripts_dir, "cmd_mux.py")
    cmd_to_esp = os.path.join(scripts_dir, "cmdvel_to_esp32.py")

    xacro_file = os.path.join(description_dir, "robot.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0",
        ),
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0",
        ),
        DeclareLaunchArgument("esp_baud", default_value="115200"),
        DeclareLaunchArgument("follow_side", default_value="left"),

        # If your /scan 0deg is not robot forward, change this.
        # You’ve been using -90; keep that as default.
        DeclareLaunchArgument("scan_yaw_offset_deg", default_value="-90.0"),

        # Start with False. Your earlier logs suggest your turn direction was flipped.
        DeclareLaunchArgument("invert_turn", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),

        # LD19 LiDAR
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

        # Wall follower (LIDAR only)
        ExecuteProcess(
            cmd=[
                "python3", wall_follower,
                "--ros-args",
                "-p", "scan_topic:=/scan",
                "-p", "cmd_topic:=/cmd_vel_raw",
                "-p", ["follow_side:=", follow_side],
                "-p", ["scan_yaw_offset_deg:=", scan_yaw_offset_deg],
                "-p", ["invert_turn:=", invert_turn],

                "-p", "desired_dist:=0.35",
                "-p", "lookahead:=0.25",
                "-p", "k_dist:=1.4",
                "-p", "k_ang:=0.6",
                "-p", "w_max:=0.55",

                "-p", "side_deg:=90.0",
                "-p", "fwd_side_deg:=75.0",
                "-p", "window_half_deg:=9.0",

                "-p", "alpha_max_deg:=50.0",
                "-p", "alpha_smooth_tau:=0.35",

                "-p", "v_nom:=0.16",
                "-p", "v_min:=0.07",
                "-p", "turn_slow_strength:=0.65",

                "-p", "front_half_deg:=25.0",
                "-p", "front_stop:=0.30",
                "-p", "front_slow:=0.90",

                "-p", "cmd_smooth_tau:=0.30",
                "-p", "w_deadband:=0.04",

                "-p", "publish_rate_hz:=20.0",
                "-p", "debug_every_s:=0.5",
            ],
            output="screen",
        ),


        # Cmd mux (stop disabled)
        ExecuteProcess(
            cmd=[
                "python3", cmd_mux,
                "--ros-args",
                "-p", "in_cmd:=/cmd_vel_raw",
                "-p", "in_stop:=none",
                "-p", "out_cmd:=/cmd_vel_safe",
                "-p", "publish_rate_hz:=20.0",
            ],
            output="screen",
        ),

        # Serial to ESP32
        ExecuteProcess(
            cmd=[
                "python3", cmd_to_esp,
                "--ros-args",
                "-p", "cmd_topic:=/cmd_vel_safe",
                "-p", ["port:=", esp_port],
                "-p", ["baud:=", esp_baud],
                "-p", "rate_hz:=20.0",
                "-p", "cmd_timeout_s:=0.5",
                "-p", "v_to_cmd:=1.0",
                "-p", "w_to_cmd:=0.8",
            ],
            output="screen",
        ),
    ])
