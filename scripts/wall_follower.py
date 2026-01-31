#!/usr/bin/env python3
import math
import time
from typing import Optional, List

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
    Lidar-only wall follower, robust for corners/clutter.

    Uses two rays (side + forward-side) to estimate wall angle alpha,
    BUT rejects unstable alpha and falls back to distance-only control.

    Robot frame:
      0 deg = forward
      +deg  = left (CCW)

    Params:
      follow_side: left/right
      scan_yaw_offset_deg: rotate scan so 0deg matches robot forward
      invert_turn: flip turn sign if drivetrain yaw sign is reversed
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Alignment
        self.declare_parameter("follow_side", "left")
        self.declare_parameter("scan_yaw_offset_deg", 0.0)
        self.declare_parameter("invert_turn", False)

        # Ray geometry (make forward-side close to side for stability)
        self.declare_parameter("side_deg", 90.0)
        self.declare_parameter("fwd_side_deg", 75.0)   # <-- was 50; 75 is MUCH more stable
        self.declare_parameter("window_half_deg", 9.0) # <-- widen a bit

        # Control
        self.declare_parameter("desired_dist", 0.35)
        self.declare_parameter("lookahead", 0.25)      # <-- shorter to reduce overshoot
        self.declare_parameter("k_dist", 1.4)          # <-- lower gains to avoid slam
        self.declare_parameter("k_ang", 0.6)
        self.declare_parameter("w_max", 0.55)          # <-- stop the 90-degree whips

        # Alpha rejection / fallback
        self.declare_parameter("alpha_max_deg", 50.0)  # reject alpha beyond this
        self.declare_parameter("alpha_smooth_tau", 0.35)

        # Speed
        self.declare_parameter("v_nom", 0.16)
        self.declare_parameter("v_min", 0.07)
        self.declare_parameter("turn_slow_strength", 0.65)  # 0..1, higher slows more during turns

        # Front safety (lidar only)
        self.declare_parameter("front_half_deg", 25.0)
        self.declare_parameter("front_stop", 0.30)
        self.declare_parameter("front_slow", 0.90)

        # Output smoothing
        self.declare_parameter("cmd_smooth_tau", 0.30)
        self.declare_parameter("w_deadband", 0.04)

        # Loop / debug
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("debug_every_s", 0.5)

        # --- Read params ---
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.follow_side = str(self.get_parameter("follow_side").value).strip().lower()
        if self.follow_side not in ("left", "right"):
            self.follow_side = "left"

        self.scan_yaw_offset_deg = float(self.get_parameter("scan_yaw_offset_deg").value)
        self.invert_turn = bool(self.get_parameter("invert_turn").value)

        self.side_deg = float(self.get_parameter("side_deg").value)
        self.fwd_side_deg = float(self.get_parameter("fwd_side_deg").value)
        self.window_half_deg = float(self.get_parameter("window_half_deg").value)

        self.desired_dist = float(self.get_parameter("desired_dist").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.k_dist = float(self.get_parameter("k_dist").value)
        self.k_ang = float(self.get_parameter("k_ang").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.alpha_max = math.radians(float(self.get_parameter("alpha_max_deg").value))
        self.alpha_smooth_tau = float(self.get_parameter("alpha_smooth_tau").value)

        self.v_nom = float(self.get_parameter("v_nom").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.turn_slow_strength = float(self.get_parameter("turn_slow_strength").value)

        self.front_half_deg = float(self.get_parameter("front_half_deg").value)
        self.front_stop = float(self.get_parameter("front_stop").value)
        self.front_slow = float(self.get_parameter("front_slow").value)

        self.cmd_smooth_tau = float(self.get_parameter("cmd_smooth_tau").value)
        self.w_deadband = float(self.get_parameter("w_deadband").value)

        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.debug_every_s = float(self.get_parameter("debug_every_s").value)

        # side sign: left wall uses +1, right wall uses -1
        self.side_sign = +1.0 if self.follow_side == "left" else -1.0
        if self.invert_turn:
            self.side_sign *= -1.0

        # ROS
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.rate), self.on_timer)

        # State
        self.have_scan = False
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self._last_cmd_t = time.time()

        self.alpha_filt = 0.0
        self.last_dist = None
        self.last_front = None

        self._last_dbg = 0.0

        self.get_logger().info(
            f"WallFollower: side={self.follow_side} desired={self.desired_dist:.2f} "
            f"yaw_offset={self.scan_yaw_offset_deg:.1f} invert_turn={self.invert_turn}"
        )

    def on_timer(self):
        if not self.have_scan:
            return
        msg = Twist()
        msg.linear.x = float(self.cmd_v)
        msg.angular.z = float(self.cmd_w)
        self.pub.publish(msg)

    # --- Helpers ---
    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot: float) -> float:
        deg_scan = wrap_deg_0_360(deg_robot + self.scan_yaw_offset_deg)
        if scan.angle_min < 0.0 and deg_scan > 180.0:
            deg_scan -= 360.0
        return math.radians(deg_scan)

    def get_range_deg(self, scan: LaserScan, deg_robot: float) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None
        ang = self.deg_to_rad_in_scan(scan, deg_robot)
        idx = int(round((ang - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        r = float(scan.ranges[idx])
        if not is_valid(scan, r):
            return None
        return r

    def robust_percentile_window(self, scan: LaserScan, deg_center: float, half_deg: float, pct: float = 0.30) -> Optional[float]:
        vals: List[float] = []
        d = deg_center - half_deg
        while d <= deg_center + half_deg:
            r = self.get_range_deg(scan, d)
            if r is not None:
                vals.append(r)
            d += 1.0
        if not vals:
            return None
        vals.sort()
        k = int(clamp(pct, 0.0, 1.0) * (len(vals) - 1))
        return vals[k]

    def apply_front_safety(self, v: float, w: float, front: Optional[float]):
        if front is None:
            return v, w
        if front < self.front_stop:
            return 0.0, clamp(-0.7 * self.side_sign, -self.w_max, self.w_max)
        if front < self.front_slow:
            t = (front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
            v = clamp(v * t, 0.0, self.v_nom)
        return v, w

    def set_cmd(self, v: float, w: float):
        t = time.time()
        dt = max(1e-3, t - self._last_cmd_t)
        self._last_cmd_t = t

        tau = max(1e-3, self.cmd_smooth_tau)
        a = dt / (tau + dt)

        self.cmd_v = (1.0 - a) * self.cmd_v + a * v
        self.cmd_w = (1.0 - a) * self.cmd_w + a * w

    def on_scan(self, scan: LaserScan):
        self.have_scan = True
        now = time.time()

        # Rays based on side
        side_ray = +abs(self.side_deg) if self.follow_side == "left" else -abs(self.side_deg)
        fwd_ray  = +abs(self.fwd_side_deg) if self.follow_side == "left" else -abs(self.fwd_side_deg)

        d_side = self.robust_percentile_window(scan, side_ray, self.window_half_deg, pct=0.30)
        d_fwd  = self.robust_percentile_window(scan, fwd_ray,  self.window_half_deg, pct=0.30)
        front  = self.robust_percentile_window(scan, 0.0,      self.front_half_deg, pct=0.25)

        # If no side distance, creep forward slowly to “find” a wall
        if d_side is None:
            v = self.v_min
            w = 0.18 * self.side_sign
            v, w = self.apply_front_safety(v, w, front)
            if abs(w) < self.w_deadband:
                w = 0.0
            self.set_cmd(v, w)
            self.debug(now, None, None, front, v, w, note="no_wall")
            return

        # Estimate alpha if possible
        alpha_ok = False
        alpha = 0.0
        if d_fwd is not None:
            theta = math.radians(abs(side_ray - fwd_ray))
            alpha_raw = math.atan2(d_fwd * math.cos(theta) - d_side, d_fwd * math.sin(theta))

            if abs(alpha_raw) <= self.alpha_max:
                # smooth alpha to prevent sign-flip thrash
                dt = 1.0 / max(1.0, self.rate)
                a = dt / (self.alpha_smooth_tau + dt)
                self.alpha_filt = (1.0 - a) * self.alpha_filt + a * alpha_raw
                alpha = self.alpha_filt
                alpha_ok = True

        # Perp distance to wall
        dist = d_side * math.cos(alpha) if alpha_ok else d_side

        # Predicted distance ahead
        dist_pred = dist + (self.lookahead * math.sin(alpha) if alpha_ok else 0.0)

        # Control
        error = float(dist_pred - self.desired_dist)

        # Distance-only fallback if alpha not ok (very stable)
        if not alpha_ok:
            w = self.side_sign * (self.k_dist * error)
            mode = "dist_only"
        else:
            w = self.side_sign * (self.k_dist * error + self.k_ang * alpha)
            mode = "follow"

        w = clamp(w, -self.w_max, self.w_max)

        # Speed shaping: less aggressive slow-down than before
        slow = 1.0 - self.turn_slow_strength * min(1.0, abs(w) / max(1e-6, self.w_max))
        v = self.v_min + (self.v_nom - self.v_min) * slow
        v = clamp(v, 0.0, self.v_nom)

        # Front safety
        v, w = self.apply_front_safety(v, w, front)

        if abs(w) < self.w_deadband:
            w = 0.0

        self.set_cmd(v, w)
        self.debug(now, dist, (alpha if alpha_ok else None), front, v, w, note=mode)

    def debug(self, now: float, dist, alpha, front, v, w, note=""):
        if (now - self._last_dbg) < self.debug_every_s:
            return
        self._last_dbg = now
        if dist is None:
            self.get_logger().info(f"{note} front={front} cmd v={v:.2f} w={w:+.2f}")
        elif alpha is None:
            self.get_logger().info(f"{note} dist={dist:.3f} alpha=REJECT front={front} cmd v={v:.2f} w={w:+.2f}")
        else:
            self.get_logger().info(f"{note} dist={dist:.3f} alpha={alpha:+.3f} front={front} cmd v={v:.2f} w={w:+.2f}")


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
