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

    # IMPORTANT: if it turns away from wall, flip this
    invert_turn = LaunchConfiguration("invert_turn")

    # ---------------------------
    # Paths (run scripts from source tree)
    # ---------------------------
    omni_repo = "/home/matt/omni_bot_ws/src/omni_bot"
    scripts_dir = os.path.join(omni_repo, "scripts")
    description_dir = os.path.join(omni_repo, "description")

    wall_follower = os.path.join(scripts_dir, "wall_follower.py")
    cmd_mux = os.path.join(scripts_dir, "cmd_mux.py")
    cmd_to_esp = os.path.join(scripts_dir, "cmdvel_to_esp32.py")

    # Robot description (TF only)
    xacro_file = os.path.join(description_dir, "robot.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file])

    return LaunchDescription([
        # ===== Arguments =====
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0",
            description="LD19 LiDAR serial port (stable by-path)"
        ),
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0",
            description="ESP32 motor controller serial port (stable by-path)"
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
        DeclareLaunchArgument(
            "invert_turn",
            default_value="true",
            description="Flip angular direction if robot turns wrong way"
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

        # ===== LD19 LiDAR (direct Node so we control port) =====
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

        # ===== Wall Follower =====
        ExecuteProcess(
            cmd=[
                "python3", wall_follower,
                "--ros-args",
                "-p", "scan_topic:=/scan",
                "-p", "cmd_topic:=/cmd_vel_raw",
                "-p", ["follow_side:=", follow_side],

                # Your scan is 0..2pi, and your physical lidar appears rotated.
                # Keep this as you had unless you re-mount.
                "-p", "scan_yaw_offset_deg:=-90.0",

                # If it turns away from the wall, invert_turn should be true.
                "-p", ["invert_turn:=", invert_turn],

                # Core behavior
                "-p", "desired_dist:=0.35",
                "-p", "v_nom:=0.14",
                "-p", "v_min:=0.05",
                "-p", "w_max:=1.1",

                # Angles for wall estimate
                "-p", "side_deg:=90.0",
                "-p", "fwd_side_deg:=50.0",
                "-p", "window_half_deg:=6.0",

                # Gains (stable first)
                "-p", "k_dist:=1.6",
                "-p", "k_ang:=1.0",

                # Anti-peel: if it drifts away, slow down and turn back gently
                "-p", "far_dist:=0.65",
                "-p", "far_speed:=0.08",
                "-p", "far_turn:=0.60",

                # Front safety
                "-p", "front_half_deg:=28.0",
                "-p", "front_stop:=0.32",
                "-p", "front_slow:=0.85",

                # Filtering
                "-p", "wall_hold_s:=0.40",
                "-p", "wall_filter_tau:=0.25",
                "-p", "cmd_smooth_tau:=0.25",
                "-p", "w_deadband:=0.06",

                "-p", "publish_rate_hz:=20.0",
                "-p", "debug_every_s:=0.5",
            ],
            output="screen",
        ),

        # ===== Cmd Mux (stop topic removed here since we're lidar-only) =====
        # This mux just passes /cmd_vel_raw -> /cmd_vel_safe for now.
        ExecuteProcess(
            cmd=[
                "python3", cmd_mux,
                "--ros-args",
                "-p", "in_cmd:=/cmd_vel_raw",
                "-p", "in_stop:=",
                "-p", "out_cmd:=/cmd_vel_safe",
                "-p", "publish_rate_hz:=20.0",
            ],
            output="screen",
        ),

        # ===== Serial → ESP32 =====
        ExecuteProcess(
            cmd=[
                "python3", cmd_to_esp,
                "--ros-args",
                "-p", "cmd_topic:=/cmd_vel_safe",
                "-p", ["port:=", esp_port],
                "-p", ["baud:=", esp_baud],
                "-p", "rate_hz:=20.0",
                "-p", "cmd_timeout_s:=0.5",

                # Start conservative
                "-p", "v_to_cmd:=1.0",
                "-p", "w_to_cmd:=0.8",
            ],
            output="screen",
        ),
    ])
