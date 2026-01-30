#!/usr/bin/env python3
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def valid_range(scan: LaserScan, r: float) -> bool:
    return math.isfinite(r) and (scan.range_min <= r <= scan.range_max)


class WallFollower(Node):
    """
    Subscribes: /scan (LaserScan)
    Publishes:  /cmd_vel_raw (Twist)

    Behavior:
      - If a wall is detected within acquire_wall_dist on the follow side -> follow with PD + heading term
      - If no wall / too far (open space) -> "acquire mode": drive forward and gently bias toward follow side
      - Front safety using lidar: slow/stop if something is close ahead
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Side to follow
        self.declare_parameter("follow_side", "left")   # "left" or "right"

        # Follow distance + limits
        self.declare_parameter("desired_dist", 0.35)    # meters
        self.declare_parameter("linear_speed", 0.20)    # m/s
        self.declare_parameter("max_ang", 1.2)          # rad/s

        # Gains (main)
        self.declare_parameter("kp", 2.5)
        self.declare_parameter("kd", 0.8)
        self.declare_parameter("heading_k", 1.2)

        # Angles/windows used for measurements
        self.declare_parameter("side_deg", 90.0)         # left uses +90, right uses -90 automatically
        self.declare_parameter("front_side_deg", 45.0)   # left uses +45, right uses -45 automatically
        self.declare_parameter("window_half_deg", 10.0)  # +/- around each angle (deg)
        self.declare_parameter("front_window_half_deg", 12.0)

        # Acquire mode (prevents “spin in open space”)
        self.declare_parameter("acquire_wall_dist", 1.8)        # if side distance > this, treat as open
        self.declare_parameter("acquire_forward_speed", 0.18)   # m/s
        self.declare_parameter("acquire_turn", 0.10)            # rad/s bias toward follow side (0 = straight)

        # Front safety (lidar)
        self.declare_parameter("front_stop_dist", 0.28)
        self.declare_parameter("front_slow_dist", 0.55)

        # Publish rate
        self.declare_parameter("publish_rate_hz", 20.0)

        # --- read params ---
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.follow_side = str(self.get_parameter("follow_side").value).lower().strip()
        self.desired_dist = float(self.get_parameter("desired_dist").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.max_ang = float(self.get_parameter("max_ang").value)

        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.heading_k = float(self.get_parameter("heading_k").value)

        self.side_deg_param = float(self.get_parameter("side_deg").value)
        self.front_side_deg_param = float(self.get_parameter("front_side_deg").value)
        self.window_half_deg = float(self.get_parameter("window_half_deg").value)
        self.front_window_half_deg = float(self.get_parameter("front_window_half_deg").value)

        self.acquire_wall_dist = float(self.get_parameter("acquire_wall_dist").value)
        self.acquire_forward_speed = float(self.get_parameter("acquire_forward_speed").value)
        self.acquire_turn = float(self.get_parameter("acquire_turn").value)

        self.front_stop_dist = float(self.get_parameter("front_stop_dist").value)
        self.front_slow_dist = float(self.get_parameter("front_slow_dist").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # --- ROS ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # --- state ---
        self.have_scan = False
        self.latest_cmd = Twist()
        self.last_error = 0.0

        if self.follow_side not in ("left", "right"):
            self.get_logger().warn(f"follow_side='{self.follow_side}' invalid; defaulting to 'left'")
            self.follow_side = "left"

        self.get_logger().info(
            f"WallFollower: scan={self.scan_topic} cmd={self.cmd_topic} side={self.follow_side} "
            f"desired={self.desired_dist:.2f}m v={self.linear_speed:.2f}m/s "
            f"acquire_dist={self.acquire_wall_dist:.2f}m"
        )

    def on_timer(self):
        if not self.have_scan:
            return
        self.cmd_pub.publish(self.latest_cmd)

    def get_range_deg(self, scan: LaserScan, deg: float) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None
        ang = math.radians(deg)
        idx = int(round((ang - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        r = scan.ranges[idx]
        if not valid_range(scan, r):
            return None
        return float(r)

    def robust_min_window(self, scan: LaserScan, deg_center: float, half_window_deg: float) -> Optional[float]:
        vals: List[float] = []
        d = deg_center - half_window_deg
        while d <= deg_center + half_window_deg:
            r = self.get_range_deg(scan, d)
            if r is not None:
                vals.append(r)
            d += 1.0

        if not vals:
            return None
        vals.sort()

        # Use ~20th percentile to reduce outliers
        k = max(0, int(0.2 * (len(vals) - 1)))
        return vals[k]

    def on_scan(self, scan: LaserScan):
        self.have_scan = True

        # front obstacle check
        front = self.robust_min_window(scan, 0.0, self.front_window_half_deg)

        # Determine follow geometry
        if self.follow_side == "left":
            side_sign = +1.0
            side_deg = +abs(self.side_deg_param)
            front_side_deg = +abs(self.front_side_deg_param)
        else:
            side_sign = -1.0
            side_deg = -abs(self.side_deg_param)
            front_side_deg = -abs(self.front_side_deg_param)

        # Measure side + front-side (for heading)
        side = self.robust_min_window(scan, side_deg, self.window_half_deg)
        front_side = self.robust_min_window(scan, front_side_deg, self.window_half_deg)

        cmd = Twist()

        # --- Acquire mode (fixes “spin forever in open room”) ---
        # If side is missing or extremely far, drive forward and gently bias toward follow side.
        if side is None or side > self.acquire_wall_dist:
            v = self.acquire_forward_speed
            w = self.acquire_turn * side_sign

            # still respect front slow/stop even in acquire
            if front is not None:
                if front < self.front_stop_dist:
                    v = 0.0
                    w = -0.8 * side_sign
                elif front < self.front_slow_dist:
                    t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                    v = clamp(v * t, 0.05, self.acquire_forward_speed)

            cmd.linear.x = float(v)
            cmd.angular.z = float(clamp(w, -self.max_ang, self.max_ang))
            self.latest_cmd = cmd
            return

        # --- Follow mode (PD on distance + heading term) ---
        error = float(side - self.desired_dist)
        derr = (error - self.last_error) * self.publish_rate_hz
        self.last_error = error

        heading_term = 0.0
        if front_side is not None:
            # If front_side < side, you're angling toward wall
            heading_term = (front_side - side)

        ang = side_sign * (self.kp * error + self.kd * derr + self.heading_k * heading_term)
        v = self.linear_speed

        # Front safety overrides
        if front is not None:
            if front < self.front_stop_dist:
                v = 0.0
                ang = -0.8 * side_sign
            elif front < self.front_slow_dist:
                t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                v = clamp(v * t, 0.05, self.linear_speed)

        cmd.linear.x = float(v)
        cmd.angular.z = float(clamp(ang, -self.max_ang, self.max_ang))
        self.latest_cmd = cmd


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
