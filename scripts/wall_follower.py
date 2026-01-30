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
    Wall follower that works well with your LD19 scan (0..2pi), and supports a scan yaw offset.

    Key ideas to reduce "bounce":
      - robust distance measurement using percentile of a window (reject spikes)
      - low-pass filter on side distance (smooth uneven wall edges)
      - derivative term computed on filtered distance
      - angular rate limit (can't whip left/right instantly)
      - corner slowdown (slow down when front is tight)
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

        # Controller gains (distance PD + heading assist)
        self.declare_parameter("kp", 1.8)
        self.declare_parameter("kd", 0.35)
        self.declare_parameter("heading_k", 0.65)

        # Measurement geometry
        self.declare_parameter("side_deg", 90.0)           # measure at ±90 from forward
        self.declare_parameter("front_side_deg", 45.0)     # measure at ±45 for heading
        self.declare_parameter("window_half_deg", 12.0)    # widen this to calm noise

        # Filtering
        self.declare_parameter("side_lpf_alpha", 0.18)     # lower = smoother
        self.declare_parameter("derr_lpf_alpha", 0.25)

        # Acquire mode (if wall not seen)
        self.declare_parameter("acquire_wall_dist", 1.8)
        self.declare_parameter("acquire_forward_speed", 0.14)
        self.declare_parameter("acquire_turn", 0.08)       # gentle bias toward follow side

        # Front safety / corner behavior
        self.declare_parameter("front_sector_half_deg", 30.0)
        self.declare_parameter("front_stop_dist", 0.50)
        self.declare_parameter("front_slow_dist", 1.10)
        self.declare_parameter("min_forward_speed", 0.06)

        # Turn rate limiting
        self.declare_parameter("w_rate_limit", 2.0)        # rad/s^2
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug
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

        self.side_lpf_alpha = float(self.get_parameter("side_lpf_alpha").value)
        self.derr_lpf_alpha = float(self.get_parameter("derr_lpf_alpha").value)

        self.acquire_wall_dist = float(self.get_parameter("acquire_wall_dist").value)
        self.acquire_forward_speed = float(self.get_parameter("acquire_forward_speed").value)
        self.acquire_turn = float(self.get_parameter("acquire_turn").value)

        self.front_sector_half_deg = float(self.get_parameter("front_sector_half_deg").value)
        self.front_stop_dist = float(self.get_parameter("front_stop_dist").value)
        self.front_slow_dist = float(self.get_parameter("front_slow_dist").value)
        self.min_forward_speed = float(self.get_parameter("min_forward_speed").value)

        self.w_rate_limit = float(self.get_parameter("w_rate_limit").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.debug_print_nearest = bool(self.get_parameter("debug_print_nearest").value)
        self.debug_print_every_s = float(self.get_parameter("debug_print_every_s").value)
        self._last_debug_t = 0.0

        # Side sign
        if self.follow_side == "left":
            self.side_sign = +1.0
            self.side_deg = +abs(self.side_deg_param)
            self.front_side_deg = +abs(self.front_side_deg_param)
        else:
            self.side_sign = -1.0
            self.side_deg = -abs(self.side_deg_param)
            self.front_side_deg = -abs(self.front_side_deg_param)

        # ROS
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # State
        self.have_scan = False
        self.latest_cmd = Twist()

        self.filt_side: Optional[float] = None
        self.prev_filt_side: Optional[float] = None
        self.filt_derr: float = 0.0

        self.prev_w: float = 0.0

        self.get_logger().info(
            f"WallFollower(smooth): scan={self.scan_topic} cmd={self.cmd_topic} side={self.follow_side} "
            f"desired={self.desired_dist:.2f} v={self.linear_speed:.2f} scan_yaw_offset_deg={self.scan_yaw_offset_deg:.1f}"
        )

    def on_timer(self):
        if self.have_scan:
            self.cmd_pub.publish(self.latest_cmd)

    # --- Angle/index helpers ---
    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot_frame: float) -> float:
        deg_scan = wrap_deg_0_360(deg_robot_frame + self.scan_yaw_offset_deg)
        if scan.angle_min < 0.0:
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

    def robust_percentile_window(self, scan: LaserScan, deg_center: float, half_window_deg: float, pct: float) -> Optional[float]:
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
        k = int(clamp(pct, 0.0, 1.0) * (len(vals) - 1))
        return vals[k]

    def nearest_obstacle(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float]]:
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

        ang = scan.angle_min + best_i * scan.angle_increment
        deg_scan = math.degrees(ang)

        # scan angle -> robot frame (undo offset)
        deg_robot = deg_scan - self.scan_yaw_offset_deg
        deg_robot = ((deg_robot + 180.0) % 360.0) - 180.0
        return best_r, deg_robot

    def rate_limit_w(self, w_cmd: float) -> float:
        dt = 1.0 / max(1e-6, self.publish_rate_hz)
        max_dw = self.w_rate_limit * dt
        w = clamp(w_cmd, self.prev_w - max_dw, self.prev_w + max_dw)
        self.prev_w = w
        return w

    def lpf(self, prev: Optional[float], new: float, alpha: float) -> float:
        if prev is None:
            return new
        return (1.0 - alpha) * prev + alpha * new

    def on_scan(self, scan: LaserScan):
        self.have_scan = True

        # Debug: nearest obstacle in robot frame
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.debug_print_nearest and (now - self._last_debug_t) > self.debug_print_every_s:
            self._last_debug_t = now
            mr, ma = self.nearest_obstacle(scan)
            if mr is not None and ma is not None:
                self.get_logger().info(f"Nearest: {mr:.2f} m at {ma:+.1f} deg (robot frame, 0=forward)")

        # Strong front safety: use a low percentile (ignore a couple NaNs/spikes)
        front = self.robust_percentile_window(scan, 0.0, self.front_sector_half_deg, pct=0.15)

        # Side distance & heading assist
        side_raw = self.robust_percentile_window(scan, self.side_deg, self.window_half_deg, pct=0.25)
        front_side_raw = self.robust_percentile_window(scan, self.front_side_deg, self.window_half_deg, pct=0.25)

        cmd = Twist()

        # Acquire mode if we don't see a wall
        if side_raw is None or side_raw > self.acquire_wall_dist:
            v = self.acquire_forward_speed
            w = self.acquire_turn * self.side_sign

            if front is not None:
                if front < self.front_stop_dist:
                    v = 0.0
                    w = -0.9 * self.side_sign
                elif front < self.front_slow_dist:
                    t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                    v = clamp(v * t, self.min_forward_speed, self.acquire_forward_speed)

            cmd.linear.x = float(v)
            cmd.angular.z = float(self.rate_limit_w(clamp(w, -self.max_ang, self.max_ang)))
            self.latest_cmd = cmd
            return

        # Filter side distance
        self.filt_side = self.lpf(self.filt_side, float(side_raw), self.side_lpf_alpha)

        # Derivative on filtered signal
        derr = 0.0
        if self.prev_filt_side is not None and self.filt_side is not None:
            derr = (self.filt_side - self.prev_filt_side) * self.publish_rate_hz
        self.prev_filt_side = self.filt_side
        self.filt_derr = self.lpf(self.filt_derr, derr, self.derr_lpf_alpha)

        # Distance error (positive if too far from wall)
        error = float(self.filt_side - self.desired_dist)

        # Heading term: if wall is angled, front_side differs from side
        heading_term = 0.0
        if front_side_raw is not None and self.filt_side is not None:
            heading_term = float(front_side_raw - self.filt_side)

        # Controller:
        # For LEFT wall: too far => turn left (positive w). For RIGHT wall: too far => turn right (negative w).
        w_cmd = self.side_sign * (self.kp * error + self.kd * self.filt_derr + self.heading_k * heading_term)
        v_cmd = self.linear_speed

        # Front slowdown (corners/clutter)
        if front is not None:
            if front < self.front_stop_dist:
                v_cmd = 0.0
                w_cmd = -0.9 * self.side_sign
            elif front < self.front_slow_dist:
                t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                v_cmd = clamp(v_cmd * t, self.min_forward_speed, self.linear_speed)

        w_cmd = clamp(w_cmd, -self.max_ang, self.max_ang)
        w_cmd = self.rate_limit_w(w_cmd)

        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(w_cmd)
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
