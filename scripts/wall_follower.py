#!/usr/bin/env python3
import math
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def is_valid(scan: LaserScan, r: float) -> bool:
    return math.isfinite(r) and (scan.range_min <= r <= scan.range_max)


def wrap_deg_0_360(deg: float) -> float:
    d = deg % 360.0
    if d < 0.0:
        d += 360.0
    return d


class WallFollower(Node):
    """
    Robust wall follower for scans that are either:
      - angle_min ~ -pi, angle_max ~ +pi  (classic)
      - angle_min ~ 0,   angle_max ~ 2pi  (your LD19 driver output)

    Key param:
      scan_yaw_offset_deg:
        "How many degrees do I rotate the scan angles so that 0° corresponds to ROBOT FORWARD?"
        If your scan's 0° is actually pointing to robot right, set scan_yaw_offset_deg=+90 (or -90).
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Follow side
        self.declare_parameter("follow_side", "left")  # left or right

        # Scan alignment
        self.declare_parameter("scan_yaw_offset_deg", 0.0)

        # Follow setpoints
        self.declare_parameter("desired_dist", 0.35)
        self.declare_parameter("linear_speed", 0.18)
        self.declare_parameter("max_ang", 1.2)

        # Gains (start conservative)
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kd", 0.6)
        self.declare_parameter("heading_k", 0.8)

        # Geometry / windows
        self.declare_parameter("side_deg", 90.0)           # measure at ±90 from forward
        self.declare_parameter("front_side_deg", 45.0)     # measure at ±45 for heading
        self.declare_parameter("window_half_deg", 10.0)

        # Acquire mode
        self.declare_parameter("acquire_wall_dist", 1.8)
        self.declare_parameter("acquire_forward_speed", 0.16)
        self.declare_parameter("acquire_turn", 0.08)       # gentle bias toward follow side

        # Strong front safety
        self.declare_parameter("front_sector_half_deg", 25.0)  # wide!
        self.declare_parameter("front_stop_dist", 0.35)
        self.declare_parameter("front_slow_dist", 0.80)

        # Publish rate
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug: print where nearest obstacle is (helps set scan_yaw_offset_deg)
        self.declare_parameter("debug_print_nearest", True)
        self.declare_parameter("debug_print_every_s", 1.0)

        # --- Read params ---
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.follow_side = str(self.get_parameter("follow_side").value).lower().strip()
        if self.follow_side not in ("left", "right"):
            self.follow_side = "left"

        self.scan_yaw_offset_deg = float(self.get_parameter("scan_yaw_offset_deg").value)

        self.desired_dist = float(self.get_parameter("desired_dist").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.max_ang = float(self.get_parameter("max_ang").value)

        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.heading_k = float(self.get_parameter("heading_k").value)

        self.side_deg_param = float(self.get_parameter("side_deg").value)
        self.front_side_deg_param = float(self.get_parameter("front_side_deg").value)
        self.window_half_deg = float(self.get_parameter("window_half_deg").value)

        self.acquire_wall_dist = float(self.get_parameter("acquire_wall_dist").value)
        self.acquire_forward_speed = float(self.get_parameter("acquire_forward_speed").value)
        self.acquire_turn = float(self.get_parameter("acquire_turn").value)

        self.front_sector_half_deg = float(self.get_parameter("front_sector_half_deg").value)
        self.front_stop_dist = float(self.get_parameter("front_stop_dist").value)
        self.front_slow_dist = float(self.get_parameter("front_slow_dist").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.debug_print_nearest = bool(self.get_parameter("debug_print_nearest").value)
        self.debug_print_every_s = float(self.get_parameter("debug_print_every_s").value)
        self._last_debug_t = 0.0

        # ROS
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # State
        self.have_scan = False
        self.latest_cmd = Twist()
        self.last_error = 0.0

        self.get_logger().info(
            f"WallFollower: scan={self.scan_topic} cmd={self.cmd_topic} side={self.follow_side} "
            f"desired={self.desired_dist:.2f} v={self.linear_speed:.2f} "
            f"scan_yaw_offset_deg={self.scan_yaw_offset_deg:.1f}"
        )

    def on_timer(self):
        if self.have_scan:
            self.cmd_pub.publish(self.latest_cmd)

    # --- Angle/index helpers for scans with angle_min possibly 0 or -pi ---
    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot_frame: float) -> float:
        """
        deg_robot_frame: degrees where 0 means ROBOT FORWARD, +CCW (left).
        We apply scan_yaw_offset so that we look up the correct ray in the scan.
        """
        deg_scan = wrap_deg_0_360(deg_robot_frame + self.scan_yaw_offset_deg)

        # Convert to radians consistent with scan's angle_min
        # If scan is 0..2pi, this is fine.
        # If scan is -pi..pi, we can convert 0..360 to -180..180 representation.
        if scan.angle_min < 0.0:
            # map 0..360 -> -180..180
            if deg_scan > 180.0:
                deg_scan -= 360.0

        return math.radians(deg_scan)

    def get_range_deg(self, scan: LaserScan, deg_robot_frame: float) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None
        ang = self.deg_to_rad_in_scan(scan, deg_robot_frame)
        idx = int(round((ang - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        r = float(scan.ranges[idx])
        if not is_valid(scan, r):
            return None
        return r

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
        k = max(0, int(0.2 * (len(vals) - 1)))  # 20th percentile
        return vals[k]

    def nearest_obstacle(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float]]:
        # returns (min_range_m, min_angle_deg_in_robot_frame)
        best_r = None
        best_i = None
        for i, r0 in enumerate(scan.ranges):
            try:
                r = float(r0)
            except Exception:
                continue
            if not is_valid(scan, r):
                continue
            if best_r is None or r < best_r:
                best_r = r
                best_i = i

        if best_r is None or best_i is None:
            return None, None

        ang = scan.angle_min + best_i * scan.angle_increment  # radians in scan convention
        deg_scan = math.degrees(ang)

        # Convert scan angle -> robot-frame angle (undo offset)
        if scan.angle_min < 0.0:
            # deg_scan already in -180..180
            deg_robot = deg_scan - self.scan_yaw_offset_deg
        else:
            # deg_scan likely 0..360
            deg_robot = deg_scan - self.scan_yaw_offset_deg

        # normalize to -180..180 for readable debug
        deg_robot = ((deg_robot + 180.0) % 360.0) - 180.0

        return best_r, deg_robot

    def on_scan(self, scan: LaserScan):
        self.have_scan = True

        # Debug: where is the nearest thing relative to robot forward?
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.debug_print_nearest and (now - self._last_debug_t) > self.debug_print_every_s:
            self._last_debug_t = now
            mr, ma = self.nearest_obstacle(scan)
            if mr is not None and ma is not None:
                self.get_logger().info(f"Nearest: {mr:.2f} m at {ma:+.1f} deg (robot frame, 0=forward)")
            else:
                self.get_logger().info("Nearest: none (all invalid/NaN)")

        # Determine follow geometry in robot frame
        if self.follow_side == "left":
            side_sign = +1.0
            side_deg = +abs(self.side_deg_param)
            front_side_deg = +abs(self.front_side_deg_param)
        else:
            side_sign = -1.0
            side_deg = -abs(self.side_deg_param)
            front_side_deg = -abs(self.front_side_deg_param)

        # Strong front safety: wide sector around 0 deg
        front = self.robust_min_window(scan, 0.0, self.front_sector_half_deg)

        # Side measurements
        side = self.robust_min_window(scan, side_deg, self.window_half_deg)
        front_side = self.robust_min_window(scan, front_side_deg, self.window_half_deg)

        cmd = Twist()

        # Acquire mode (open space): don't spin, roll forward with gentle bias
        if side is None or side > self.acquire_wall_dist:
            v = self.acquire_forward_speed
            w = self.acquire_turn * side_sign

            if front is not None:
                if front < self.front_stop_dist:
                    v = 0.0
                    w = -0.9 * side_sign
                elif front < self.front_slow_dist:
                    t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                    v = clamp(v * t, 0.05, self.acquire_forward_speed)

            cmd.linear.x = float(v)
            cmd.angular.z = float(clamp(w, -self.max_ang, self.max_ang))
            self.latest_cmd = cmd
            return

        # Follow mode: PD on distance + heading term
        error = float(side - self.desired_dist)
        derr = (error - self.last_error) * self.publish_rate_hz
        self.last_error = error

        heading_term = 0.0
        if front_side is not None:
            heading_term = (front_side - side)

        w = side_sign * (self.kp * error + self.kd * derr + self.heading_k * heading_term)
        v = self.linear_speed

        # Front override (strong)
        if front is not None:
            if front < self.front_stop_dist:
                v = 0.0
                w = -0.9 * side_sign
            elif front < self.front_slow_dist:
                t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                v = clamp(v * t, 0.05, self.linear_speed)

        cmd.linear.x = float(v)
        cmd.angular.z = float(clamp(w, -self.max_ang, self.max_ang))
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
