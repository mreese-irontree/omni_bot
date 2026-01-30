from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    obstacle_guard = os.path.join(scripts_dir, "ground_obstacle_guard.py")
    cmd_mux = os.path.join(scripts_dir, "cmd_mux.py")
    cmd_to_esp = os.path.join(scripts_dir, "cmdvel_to_esp32.py")

    # Arducam TOF publisher (absolute path, as requested)
    tof_script = (
        "/home/matt/omni_bot_ws/src/Arducam_tof_camera/"
        "ros2_publisher/src/arducam/"
        "arducam_rclpy_tof_pointcloud/"
        "arducam_rclpy_tof_pointcloud/tof_pointcloud.py"
    )

    # Robot description
    xacro_file = os.path.join(description_dir, "robot.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file])

    # LD19 node (run directly so we control the port)
    ldlidar_pkg = FindPackageShare("ldlidar_stl_ros2")

    # ---------------------------
    # Launch description
    # ---------------------------
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

        # ===== LD19 LiDAR =====
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

        # ===== Arducam TOF PointCloud =====
        ExecuteProcess(
            cmd=["python3", tof_script],
            output="screen",
        ),

        # ===== Wall Follower =====
        ExecuteProcess(
            cmd=[
                "python3", wall_follower,
                "--ros-args",
                "-p", "scan_topic:=/scan",
                "-p", "cmd_topic:=/cmd_vel_raw",
                "-p", ["follow_side:=", follow_side],

                # IMPORTANT: LiDAR alignment
                "-p", "scan_yaw_offset_deg:=-90.0",

                "-p", "desired_dist:=0.35",
                "-p", "linear_speed:=0.20",
                "-p", "publish_rate_hz:=20.0",

                # Front safety
                "-p", "front_sector_half_deg:=30.0",
                "-p", "front_stop_dist:=0.45",
                "-p", "front_slow_dist:=1.00",
            ],
            output="screen",
        ),

        # ===== Ground Obstacle Guard =====
        ExecuteProcess(
            cmd=[
                "python3", obstacle_guard,
                "--ros-args",
                "-p", "points_topic:=/point_cloud",
                "-p", "stop_topic:=/safety/stop",

                "-p", "x_min:=0.10",
                "-p", "x_max:=0.80",
                "-p", "y_half:=0.25",
                "-p", "z_min:=-0.20",
                "-p", "z_max:=0.10",

                "-p", "stop_dist:=0.25",
                "-p", "hold_stop_ms:=400",
                "-p", "publish_rate_hz:=20.0",
                "-p", "decimation:=8",
            ],
            output="screen",
        ),

        # ===== Cmd Mux =====
        ExecuteProcess(
            cmd=[
                "python3", cmd_mux,
                "--ros-args",
                "-p", "in_cmd:=/cmd_vel_raw",
                "-p", "in_stop:=/safety/stop",
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
                "-p", "v_to_cmd:=1.0",
                "-p", "w_to_cmd:=0.6",
            ],
            output="screen",
        ),
    ])
