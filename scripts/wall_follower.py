#!/usr/bin/env python3
import math
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_deg_0_360(deg: float) -> float:
    d = deg % 360.0
    if d < 0.0:
        d += 360.0
    return d


def is_valid(scan: LaserScan, r: float) -> bool:
    return math.isfinite(r) and (scan.range_min <= r <= scan.range_max)


class WallFollower(Node):
    """
    Lidar-only wall follower tuned to avoid "peel off + wobble".

    Key improvements vs basic PD:
      - Hold last valid wall estimate briefly when measurements drop out
      - Filter wall distance/angle
      - Anti-peel behavior (slow down when far from wall)
      - Angular deadband + command smoothing
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        # Side
        self.declare_parameter("follow_side", "left")  # left/right

        # Scan alignment
        self.declare_parameter("scan_yaw_offset_deg", 0.0)

        # Setpoint + speed
        self.declare_parameter("desired_dist", 0.35)
        self.declare_parameter("v_nom", 0.16)
        self.declare_parameter("v_min", 0.05)
        self.declare_parameter("w_max", 1.1)

        # Wall geometry rays
        self.declare_parameter("side_deg", 90.0)
        self.declare_parameter("fwd_side_deg", 50.0)
        self.declare_parameter("window_half_deg", 6.0)

        # Front safety
        self.declare_parameter("front_half_deg", 28.0)
        self.declare_parameter("front_stop", 0.32)
        self.declare_parameter("front_slow", 0.85)

        # Controller gains (lower = less wobble)
        self.declare_parameter("k_dist", 1.6)
        self.declare_parameter("k_ang", 1.0)

        # Anti-peel tuning
        self.declare_parameter("far_dist", 0.65)      # if dist > this, treat as "peeled off"
        self.declare_parameter("far_turn", 0.55)      # gentle turn back toward wall (rad/s cap)
        self.declare_parameter("far_speed", 0.08)     # slow forward when far so you don't run away

        # Hold-last when wall disappears briefly
        self.declare_parameter("wall_hold_s", 0.40)

        # Filtering / smoothing
        self.declare_parameter("wall_filter_tau", 0.25)   # seconds
        self.declare_parameter("cmd_smooth_tau", 0.22)    # seconds
        self.declare_parameter("w_deadband", 0.05)        # rad/s deadband
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug
        self.declare_parameter("debug_print", True)
        self.declare_parameter("debug_every_s", 1.0)

        # --- read params ---
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.follow_side = str(self.get_parameter("follow_side").value).lower().strip()
        if self.follow_side not in ("left", "right"):
            self.follow_side = "left"

        self.scan_yaw_offset_deg = float(self.get_parameter("scan_yaw_offset_deg").value)

        self.desired_dist = float(self.get_parameter("desired_dist").value)
        self.v_nom = float(self.get_parameter("v_nom").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.side_deg = float(self.get_parameter("side_deg").value)
        self.fwd_side_deg = float(self.get_parameter("fwd_side_deg").value)
        self.window_half_deg = float(self.get_parameter("window_half_deg").value)

        self.front_half_deg = float(self.get_parameter("front_half_deg").value)
        self.front_stop = float(self.get_parameter("front_stop").value)
        self.front_slow = float(self.get_parameter("front_slow").value)

        self.k_dist = float(self.get_parameter("k_dist").value)
        self.k_ang = float(self.get_parameter("k_ang").value)

        self.far_dist = float(self.get_parameter("far_dist").value)
        self.far_turn = float(self.get_parameter("far_turn").value)
        self.far_speed = float(self.get_parameter("far_speed").value)

        self.wall_hold_s = float(self.get_parameter("wall_hold_s").value)

        self.wall_filter_tau = float(self.get_parameter("wall_filter_tau").value)
        self.cmd_smooth_tau = float(self.get_parameter("cmd_smooth_tau").value)
        self.w_deadband = float(self.get_parameter("w_deadband").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.debug_print = bool(self.get_parameter("debug_print").value)
        self.debug_every_s = float(self.get_parameter("debug_every_s").value)
        self._last_debug_t = 0.0

        # follow side sign
        self.side_sign = +1.0 if self.follow_side == "left" else -1.0

        # ROS
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # state
        self.have_scan = False
        self.latest_cmd = Twist()
        self.cmd_filt = Twist()

        self.last_wall_time = 0.0
        self.dist_filt: Optional[float] = None
        self.alpha_filt: Optional[float] = None
        self.front_filt: Optional[float] = None

        self.get_logger().info(
            f"WallFollower: side={self.follow_side} desired={self.desired_dist:.2f} "
            f"scan_yaw_offset_deg={self.scan_yaw_offset_deg:.1f} v_nom={self.v_nom:.2f}"
        )

    # --- scan helpers ---
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

    def robust_window(self, scan: LaserScan, deg_center: float, half_window_deg: float) -> Optional[float]:
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
        # 25th percentile: rejects occasional big outliers
        k = max(0, int(0.25 * (len(vals) - 1)))
        return vals[k]

    def compute_wall(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float]]:
        a_deg = self.side_sign * abs(self.side_deg)
        b_deg = self.side_sign * abs(self.fwd_side_deg)

        a = self.robust_window(scan, a_deg, self.window_half_deg)
        b = self.robust_window(scan, b_deg, self.window_half_deg)
        if a is None or b is None:
            return None, None

        theta = math.radians(abs(self.side_deg - self.fwd_side_deg))
        if theta < 1e-3:
            return None, None

        # alpha in radians
        num = (a * math.cos(theta)) - b
        den = max(1e-6, a * math.sin(theta))
        alpha = math.atan2(num, den)

        dist = b * math.cos(alpha)  # perpendicular distance
        return dist, alpha

    def front_distance(self, scan: LaserScan) -> Optional[float]:
        return self.robust_window(scan, 0.0, self.front_half_deg)

    # --- filtering ---
    def lowpass(self, prev: Optional[float], x: Optional[float], dt: float, tau: float) -> Optional[float]:
        if x is None:
            return prev
        if prev is None:
            return x
        a = clamp(dt / max(1e-3, tau), 0.0, 1.0)
        return (1.0 - a) * prev + a * x

    # --- main ---
    def on_scan(self, scan: LaserScan):
        self.have_scan = True
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / max(1e-3, self.publish_rate_hz)

        dist, alpha = self.compute_wall(scan)
        front = self.front_distance(scan)

        # update filters if we got new values
        if dist is not None and alpha is not None:
            self.last_wall_time = now
        self.dist_filt = self.lowpass(self.dist_filt, dist, dt, self.wall_filter_tau)
        self.alpha_filt = self.lowpass(self.alpha_filt, alpha, dt, self.wall_filter_tau)
        self.front_filt = self.lowpass(self.front_filt, front, dt, 0.20)

        # wall usable?
        wall_recent = (now - self.last_wall_time) <= self.wall_hold_s
        use_dist = self.dist_filt if wall_recent else None
        use_alpha = self.alpha_filt if wall_recent else None
        use_front = self.front_filt

        # debug prints
        if self.debug_print and (now - self._last_debug_t) > self.debug_every_s:
            self._last_debug_t = now
            self.get_logger().info(
                f"dist={use_dist if use_dist is not None else None} "
                f"alpha={use_alpha if use_alpha is not None else None} "
                f"front={use_front if use_front is not None else None} "
                f"wall_recent={wall_recent}"
            )

        cmd = Twist()

        # if no wall for too long -> gentle search toward wall (slow)
        if use_dist is None or use_alpha is None:
            v = self.v_min
            w = 0.35 * self.side_sign

            # but don't hit front stuff
            if use_front is not None:
                if use_front < self.front_stop:
                    v = 0.0
                    w = -0.9 * self.side_sign
                elif use_front < self.front_slow:
                    t = (use_front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
                    v = clamp(self.v_min * t, 0.0, self.v_min)

            cmd.linear.x = float(v)
            cmd.angular.z = float(clamp(w, -self.w_max, self.w_max))
            self.latest_cmd = cmd
            return

        # --- anti-peel logic ---
        # If we drift far from wall, slow down and turn back gently toward it.
        if use_dist > self.far_dist:
            cmd.linear.x = float(self.far_speed)
            cmd.angular.z = float(clamp(self.far_turn * self.side_sign, -self.w_max, self.w_max))
            self.latest_cmd = cmd
            return

        # --- normal follow ---
        e_dist = use_dist - self.desired_dist
        e_ang = use_alpha

        # turn: negative when too close on left (e_dist<0) -> turn right, which is correct
        w = self.side_sign * (self.k_dist * e_dist + self.k_ang * e_ang)

        # front safety speed schedule
        v = self.v_nom
        if use_front is not None:
            if use_front < self.front_stop:
                v = 0.0
                w = -0.9 * self.side_sign
            elif use_front < self.front_slow:
                t = (use_front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
                v = clamp(self.v_nom * t, self.v_min, self.v_nom)

        # angular deadband to stop tiny wobble
        if abs(w) < self.w_deadband:
            w = 0.0

        cmd.linear.x = float(v)
        cmd.angular.z = float(clamp(w, -self.w_max, self.w_max))
        self.latest_cmd = cmd

    def on_timer(self):
        if not self.have_scan:
            return

        dt = 1.0 / max(1e-3, self.publish_rate_hz)
        tau = max(1e-3, self.cmd_smooth_tau)
        a = clamp(dt / tau, 0.0, 1.0)

        self.cmd_filt.linear.x = (1 - a) * self.cmd_filt.linear.x + a * self.latest_cmd.linear.x
        self.cmd_filt.angular.z = (1 - a) * self.cmd_filt.angular.z + a * self.latest_cmd.angular.z

        self.pub.publish(self.cmd_filt)


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
