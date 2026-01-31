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
    Wall follower that uses two scan rays to estimate wall angle and distance.

    Works with scans that are either:
      - angle_min ~ -pi..pi
      - angle_min ~ 0..2pi  (your LD19 driver output)

    Robot frame convention:
      0 deg = forward
      +deg  = left (CCW)

    Params:
      scan_yaw_offset_deg: rotate scan so that 0deg corresponds to robot forward
      follow_side: left or right
      invert_turn: flip turn direction if your drivetrain sign is reversed
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Alignment / side
        self.declare_parameter("follow_side", "left")      # left/right
        self.declare_parameter("scan_yaw_offset_deg", 0.0) # degrees
        self.declare_parameter("invert_turn", False)       # flip w sign if needed

        # Geometry
        self.declare_parameter("side_deg", 90.0)      # distance ray (left=+90, right=-90 applied internally)
        self.declare_parameter("fwd_side_deg", 50.0)  # forward-side ray (left=+50, right=-50 applied internally)
        self.declare_parameter("window_half_deg", 6.0)

        # Control
        self.declare_parameter("desired_dist", 0.35)   # m
        self.declare_parameter("lookahead", 0.45)      # m (predict distance ahead)
        self.declare_parameter("k_dist", 2.2)          # distance gain
        self.declare_parameter("k_ang", 1.2)           # wall angle gain
        self.declare_parameter("w_max", 1.0)           # rad/s limit

        # Speed shaping
        self.declare_parameter("v_nom", 0.14)
        self.declare_parameter("v_min", 0.05)

        # Front safety (pure lidar)
        self.declare_parameter("front_half_deg", 25.0)
        self.declare_parameter("front_stop", 0.30)
        self.declare_parameter("front_slow", 0.85)

        # Smoothing / stability
        self.declare_parameter("cmd_smooth_tau", 0.25)   # seconds
        self.declare_parameter("w_deadband", 0.06)       # rad/s
        self.declare_parameter("wall_hold_s", 0.40)      # keep last wall estimate briefly

        # Loop
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug
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

        self.v_nom = float(self.get_parameter("v_nom").value)
        self.v_min = float(self.get_parameter("v_min").value)

        self.front_half_deg = float(self.get_parameter("front_half_deg").value)
        self.front_stop = float(self.get_parameter("front_stop").value)
        self.front_slow = float(self.get_parameter("front_slow").value)

        self.cmd_smooth_tau = float(self.get_parameter("cmd_smooth_tau").value)
        self.w_deadband = float(self.get_parameter("w_deadband").value)
        self.wall_hold_s = float(self.get_parameter("wall_hold_s").value)

        self.rate = float(self.get_parameter("publish_rate_hz").value)

        self.debug_every_s = float(self.get_parameter("debug_every_s").value)
        self._last_dbg = 0.0

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
        self.last_wall_time = 0.0
        self.last_dist = None
        self.last_alpha = None
        self.last_front = None

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self._last_cmd_t = time.time()

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

    # --- Scan angle/index helpers ---
    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot: float) -> float:
        # Apply yaw offset: scan_deg = robot_deg + offset
        deg_scan = wrap_deg_0_360(deg_robot + self.scan_yaw_offset_deg)

        # If scan is -pi..pi, convert 0..360 into -180..180
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

    def robust_percentile_window(self, scan: LaserScan, deg_center: float, half_deg: float, pct: float = 0.2) -> Optional[float]:
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

    def on_scan(self, scan: LaserScan):
        self.have_scan = True
        now = time.time()

        # Choose rays based on side
        side_ray = +abs(self.side_deg) if self.follow_side == "left" else -abs(self.side_deg)
        fwd_ray  = +abs(self.fwd_side_deg) if self.follow_side == "left" else -abs(self.fwd_side_deg)

        d_side = self.robust_percentile_window(scan, side_ray, self.window_half_deg, pct=0.2)
        d_fwd  = self.robust_percentile_window(scan, fwd_ray,  self.window_half_deg, pct=0.2)
        front  = self.robust_percentile_window(scan, 0.0,      self.front_half_deg, pct=0.2)

        # Hold last wall estimate briefly if one ray is missing
        if d_side is None or d_fwd is None:
            if (self.last_dist is not None) and ((now - self.last_wall_time) < self.wall_hold_s):
                dist = self.last_dist
                alpha = self.last_alpha if self.last_alpha is not None else 0.0
            else:
                # no wall -> just creep forward slowly and very gently bias toward side (find a wall)
                v = self.v_min
                w = 0.25 * self.side_sign
                v, w = self.apply_front_safety(v, w, front)
                self.set_cmd(v, w)
                self.last_front = front
                self.debug(now, dist=None, alpha=None, front=front, v=v, w=w, note="no_wall")
                return
        else:
            # Wall angle estimation (classic two-ray geometry)
            theta = math.radians(abs(side_ray - fwd_ray))  # e.g. 90-50=40deg

            # alpha: wall angle relative to robot
            # alpha > 0 means wall is rotating "away" (depends on side), we correct with k_ang * alpha
            alpha = math.atan2(d_fwd * math.cos(theta) - d_side, d_fwd * math.sin(theta))

            # perpendicular distance to wall
            dist = d_side * math.cos(alpha)

            # predict distance ahead
            dist_pred = dist + self.lookahead * math.sin(alpha)

            self.last_wall_time = now
            self.last_dist = dist
            self.last_alpha = alpha

        # Control law:
        # error = dist_pred - desired
        # w_cmd = side_sign * (k_dist * error + k_ang * alpha)
        error = float(dist_pred - self.desired_dist)
        w = self.side_sign * (self.k_dist * error + self.k_ang * alpha)
        w = clamp(w, -self.w_max, self.w_max)

        # Speed shaping: slow down if turning hard
        turn_factor = 1.0 - min(0.85, abs(w) / max(1e-6, self.w_max))
        v = self.v_min + (self.v_nom - self.v_min) * turn_factor
        v = clamp(v, 0.0, self.v_nom)

        # Front safety
        v, w = self.apply_front_safety(v, w, front)

        # Small deadband on angular to prevent “micro oscillation”
        if abs(w) < self.w_deadband:
            w = 0.0

        self.set_cmd(v, w)
        self.last_front = front
        self.debug(now, dist=dist, alpha=alpha, front=front, v=v, w=w, note="follow")

    def apply_front_safety(self, v: float, w: float, front: Optional[float]):
        if front is None:
            return v, w
        if front < self.front_stop:
            return 0.0, clamp(-0.9 * self.side_sign, -self.w_max, self.w_max)
        if front < self.front_slow:
            t = (front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
            v = clamp(v * t, 0.0, self.v_nom)
        return v, w

    def set_cmd(self, v: float, w: float):
        # 1st-order smoothing
        t = time.time()
        dt = max(1e-3, t - self._last_cmd_t)
        self._last_cmd_t = t

        tau = max(1e-3, self.cmd_smooth_tau)
        a = dt / (tau + dt)

        self.cmd_v = (1.0 - a) * self.cmd_v + a * v
        self.cmd_w = (1.0 - a) * self.cmd_w + a * w

    def debug(self, now: float, dist, alpha, front, v, w, note=""):
        if (now - self._last_dbg) < self.debug_every_s:
            return
        self._last_dbg = now

        if dist is None or alpha is None:
            self.get_logger().info(f"{note} front={front} cmd v={v:.2f} w={w:+.2f}")
        else:
            self.get_logger().info(
                f"{note} dist={dist:.3f} alpha={alpha:+.3f} front={front} cmd v={v:.2f} w={w:+.2f}"
            )


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
