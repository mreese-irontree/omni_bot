from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os


def generate_launch_description():
    # ---------------------------
    # Launch arguments
    # ---------------------------
    esp_port = LaunchConfiguration("esp_port")
    esp_baud = LaunchConfiguration("esp_baud")
    lidar_port = LaunchConfiguration("lidar_port")
    follow_side = LaunchConfiguration("follow_side")

    # ---------------------------
    # Paths
    # ---------------------------
    omni_repo = "/home/matt/omni_bot_ws/src/omni_bot"
    scripts_dir = os.path.join(omni_repo, "scripts")
    description_dir = os.path.join(omni_repo, "description")

    wall_follower = os.path.join(scripts_dir, "wall_follower.py")
    cmd_mux = os.path.join(scripts_dir, "cmd_mux.py")
    cmd_to_esp = os.path.join(scripts_dir, "cmdvel_to_esp32.py")

    # Robot description (TF only; doesn't control anything)
    xacro_file = os.path.join(description_dir, "robot.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file])

    return LaunchDescription([

        # ===== Arguments =====
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0",
            description="LD19 LiDAR serial port"
        ),
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0",
            description="ESP32 motor controller serial port"
        ),
        DeclareLaunchArgument(
            "esp_baud",
            default_value="115200",
            description="ESP32 serial baud rate"
        ),
        DeclareLaunchArgument(
            "follow_side",
            default_value="left",
            description="Wall follow side: left or right"
        ),

        # ===== Robot TF =====
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

        # ===== LD19 LiDAR (run directly so we set port safely) =====
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

        # ===== Wall follower (lidar-only) =====
        ExecuteProcess(
            cmd=[
                "python3", wall_follower,
                "--ros-args",
                "-p", "scan_topic:=/scan",
                "-p", "cmd_topic:=/cmd_vel_raw",
                "-p", ["follow_side:=", follow_side],

                # IMPORTANT: scan_yaw_offset_deg
                # You previously saw "Nearest is at -92 deg". That means 0° is likely pointing right.
                # Keep -90 for your current setup; you can tweak later.
                "-p", "scan_yaw_offset_deg:=-90.0",

                # Target behavior
                "-p", "desired_dist:=0.35",
                "-p", "v_nominal:=0.14",
                "-p", "v_min:=0.06",
                "-p", "v_max:=0.18",
                "-p", "w_max:=1.0",

                # Front safety
                "-p", "front_stop_dist:=0.35",
                "-p", "front_slow_dist:=0.80",
                "-p", "front_sector_half_deg:=18.0",

                # Wall sector for fitting
                "-p", "side_sector_center_deg:=90.0",
                "-p", "side_sector_half_deg:=35.0",

                # RANSAC / stability
                "-p", "ransac_iters:=60",
                "-p", "ransac_thresh:=0.03",
                "-p", "min_inliers:=35",

                # Controller tuning (stable defaults)
                "-p", "k_dist:=2.2",
                "-p", "k_head:=1.6",

                # Smoothing
                "-p", "w_slew_rate:=2.5",
                "-p", "w_lowpass_alpha:=0.35",
                "-p", "publish_rate_hz:=20.0",

                # Debug print
                "-p", "debug:=true",
                "-p", "debug_every_s:=0.6",
            ],
            output="screen",
        ),

        # ===== Cmd mux (optional safety stop topic; default disabled here) =====
        ExecuteProcess(
            cmd=[
                "python3", cmd_mux,
                "--ros-args",
                "-p", "in_cmd:=/cmd_vel_raw",
                "-p", "out_cmd:=/cmd_vel_safe",
                # leave stop_topic empty to disable stop gating (lidar-only focus)
                "-p", "stop_topic:=",
                "-p", "publish_rate_hz:=20.0",
            ],
            output="screen",
        ),

        # ===== Serial -> ESP32 =====
        ExecuteProcess(
            cmd=[
                "python3", cmd_to_esp,
                "--ros-args",
                "-p", "cmd_topic:=/cmd_vel_safe",
                "-p", ["port:=", esp_port],
                "-p", ["baud:=", esp_baud],
                "-p", "rate_hz:=20.0",
                "-p", "cmd_timeout_s:=0.5",

                # These are scaling from Twist to [-1..1]
                # Start conservative.
                "-p", "v_to_cmd:=1.0",
                "-p", "w_to_cmd:=0.75",
            ],
            output="screen",
        ),
    ])
