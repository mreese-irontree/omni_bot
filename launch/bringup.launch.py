from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Launch args
    esp_port = LaunchConfiguration("esp_port")
    esp_baud = LaunchConfiguration("esp_baud")
    follow_side = LaunchConfiguration("follow_side")

    # --- Your Arducam script path (absolute) ---
    tof_script = "/home/matt/omni_bot_ws/src/Arducam_tof_camera/ros2_publisher/src/arducam/arducam_rclpy_tof_pointcloud/arducam_rclpy_tof_pointcloud/tof_pointcloud.py"

    # --- LD19 launch file ---
    ldlidar_share = FindPackageShare("ldlidar_stl_ros2")
    ld19_launch = PythonLaunchDescriptionSource([ldlidar_share, "/launch/ld19.launch.py"])

    # --- Run scripts from SOURCE TREE ---
    omni_repo = "/home/matt/omni_bot_ws/src/omni_bot"
    scripts_dir = os.path.join(omni_repo, "scripts")

    wall_follower = os.path.join(scripts_dir, "wall_follower.py")
    obstacle_guard = os.path.join(scripts_dir, "ground_obstacle_guard.py")
    cmd_mux = os.path.join(scripts_dir, "cmd_mux.py")
    to_esp = os.path.join(scripts_dir, "cmdvel_to_esp32.py")

    # --- URDF/Xacro for TF publishing ---
    xacro_file = os.path.join(omni_repo, "description", "robot.urdf.xacro")
    robot_description = Command(["xacro", " ", xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0"
        ),
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0"
        ),
        DeclareLaunchArgument("esp_baud", default_value="115200"),
        DeclareLaunchArgument("follow_side", default_value="left"),

        # --- ROBOT TF (this creates /tf and /tf_static) ---
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        # Publishes dummy joint states so robot_state_publisher can publish wheel joint TFs
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),

        # --- LIDAR ---
        IncludeLaunchDescription(ld19_launch),

        # --- ARDUCAM POINTCLOUD ---
        ExecuteProcess(
            cmd=["python3", tof_script],
            output="screen"
        ),

        # --- WALL FOLLOWER ---
        ExecuteProcess(
            cmd=[
                "python3", wall_follower,
                "--ros-args",
                "-p", "scan_topic:=/scan",
                "-p", "cmd_topic:=/cmd_vel_raw",
                "-p", ["follow_side:=", follow_side],
                "-p", "scan_yaw_offset_deg:=-90.0",
                "-p", "desired_dist:=0.35",
                "-p", "linear_speed:=0.20",
                "-p", "publish_rate_hz:=20.0",

                # safer front behavior (optional but recommended)
                "-p", "front_stop_dist:=0.45",
                "-p", "front_slow_dist:=1.00",
                "-p", "front_sector_half_deg:=30.0",
            ],
            output="screen"
        ),

        # --- DEPTH SAFETY ---
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
            output="screen"
        ),

        # --- MUX ---
        ExecuteProcess(
            cmd=[
                "python3", cmd_mux,
                "--ros-args",
                "-p", "in_cmd:=/cmd_vel_raw",
                "-p", "in_stop:=/safety/stop",
                "-p", "out_cmd:=/cmd_vel_safe",
                "-p", "publish_rate_hz:=20.0",
            ],
            output="screen"
        ),

        # --- SERIAL TO ESP32 ---
        ExecuteProcess(
            cmd=[
                "python3", to_esp,
                "--ros-args",
                "-p", "cmd_topic:=/cmd_vel_safe",
                "-p", ["port:=", esp_port],
                "-p", ["baud:=", esp_baud],
                "-p", "rate_hz:=20.0",
                "-p", "cmd_timeout_s:=0.5",
                "-p", "v_to_cmd:=1.0",
                "-p", "w_to_cmd:=0.6",
            ],
            output="screen"
        ),
    ])
