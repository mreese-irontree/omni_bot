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


def angdiff(a: float, b: float) -> float:
    """smallest signed difference a-b in radians"""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


class WallFollower(Node):
    """
    Wall follower tuned for real indoor walls using LaserScan with either:
      - angle_min ~ -pi .. +pi
      - angle_min ~ 0   .. 2pi  (your LD19 output)

    Improvements over simple "single-ray" follower:
      - line fit on a side sector to estimate wall distance + wall angle
      - low-pass filtering (reduces jitter on rough/stacked edges)
      - corner/clutter slowdown
      - turn-rate limiting (prevents rapid left-right bouncing)
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

        # Desired behavior
        self.declare_parameter("desired_dist", 0.35)
        self.declare_parameter("linear_speed", 0.18)
        self.declare_parameter("max_ang", 1.2)

        # Control gains (for wall-distance + wall-angle)
        self.declare_parameter("k_dist", 2.2)     # distance error -> turn
        self.declare_parameter("k_ang", 1.4)      # wall heading angle -> turn

        # Sector / fit settings
        self.declare_parameter("side_center_deg", 90.0)     # side look direction (±90)
        self.declare_parameter("fit_half_deg", 18.0)        # fit sector size around side
        self.declare_parameter("max_fit_range", 2.5)        # ignore far points for fitting
        self.declare_parameter("min_fit_points", 18)        # require enough points

        # Filtering
        self.declare_parameter("dist_alpha", 0.20)          # 0..1 (higher = faster response)
        self.declare_parameter("ang_alpha", 0.18)

        # Corner handling
        self.declare_parameter("corner_check_deg", 35.0)    # check around front-left/right
        self.declare_parameter("corner_slow_dist", 0.90)
        self.declare_parameter("corner_stop_dist", 0.45)
        self.declare_parameter("corner_speed_min", 0.07)

        # Acquire mode
        self.declare_parameter("acquire_wall_dist", 1.8)
        self.declare_parameter("acquire_forward_speed", 0.14)
        self.declare_parameter("acquire_turn", 0.10)        # gentle bias toward follow side

        # Turn limiting (reduces bounce)
        self.declare_parameter("w_rate_limit", 2.5)         # rad/s^2
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug
        self.declare_parameter("debug_print", False)
        self.declare_parameter("debug_every_s", 1.0)

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

        self.k_dist = float(self.get_parameter("k_dist").value)
        self.k_ang = float(self.get_parameter("k_ang").value)

        self.side_center_deg = float(self.get_parameter("side_center_deg").value)
        self.fit_half_deg = float(self.get_parameter("fit_half_deg").value)
        self.max_fit_range = float(self.get_parameter("max_fit_range").value)
        self.min_fit_points = int(self.get_parameter("min_fit_points").value)

        self.dist_alpha = float(self.get_parameter("dist_alpha").value)
        self.ang_alpha = float(self.get_parameter("ang_alpha").value)

        self.corner_check_deg = float(self.get_parameter("corner_check_deg").value)
        self.corner_slow_dist = float(self.get_parameter("corner_slow_dist").value)
        self.corner_stop_dist = float(self.get_parameter("corner_stop_dist").value)
        self.corner_speed_min = float(self.get_parameter("corner_speed_min").value)

        self.acquire_wall_dist = float(self.get_parameter("acquire_wall_dist").value)
        self.acquire_forward_speed = float(self.get_parameter("acquire_forward_speed").value)
        self.acquire_turn = float(self.get_parameter("acquire_turn").value)

        self.w_rate_limit = float(self.get_parameter("w_rate_limit").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.debug_print = bool(self.get_parameter("debug_print").value)
        self.debug_every_s = float(self.get_parameter("debug_every_s").value)
        self._last_dbg_t = 0.0

        # Follow side sign/angles
        if self.follow_side == "left":
            self.side_sign = +1.0
            self.side_deg = +abs(self.side_center_deg)
            self.corner_deg = +abs(self.corner_check_deg)  # front-left
        else:
            self.side_sign = -1.0
            self.side_deg = -abs(self.side_center_deg)
            self.corner_deg = -abs(self.corner_check_deg)  # front-right

        # ROS
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # State
        self.have_scan = False
        self.latest_cmd = Twist()

        self.filt_dist: Optional[float] = None
        self.filt_wall_ang: Optional[float] = None  # radians (0 means wall parallel to forward)
        self.prev_w = 0.0

        self.get_logger().info(
            f"WallFollower(linefit): scan={self.scan_topic} cmd={self.cmd_topic} side={self.follow_side} "
            f"desired={self.desired_dist:.2f} v={self.linear_speed:.2f} offset_deg={self.scan_yaw_offset_deg:.1f}"
        )

    def on_timer(self):
        if self.have_scan:
            self.cmd_pub.publish(self.latest_cmd)

    # ---- Angle/index helpers ----
    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot_frame: float) -> float:
        # robot frame deg: 0 forward, +left
        deg_scan = wrap_deg_0_360(deg_robot_frame + self.scan_yaw_offset_deg)
        if scan.angle_min < 0.0:
            if deg_scan > 180.0:
                deg_scan -= 360.0
        return math.radians(deg_scan)

    def range_at_deg(self, scan: LaserScan, deg_robot_frame: float) -> Optional[float]:
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
            r = self.range_at_deg(scan, d)
            if r is not None:
                vals.append(r)
            d += 1.0
        if not vals:
            return None
        vals.sort()
        k = max(0, int(0.2 * (len(vals) - 1)))
        return vals[k]

    # ---- Line fit on side sector ----
    def fit_wall_in_sector(self, scan: LaserScan, center_deg: float, half_deg: float) -> Optional[Tuple[float, float]]:
        """
        Returns (dist_perp, wall_angle_rad)
          dist_perp: perpendicular distance from robot origin to wall line (meters), positive
          wall_angle_rad: wall direction relative to robot forward (0 means wall is parallel to forward),
                          sign indicates rotation (CCW positive).
        """
        pts: List[Tuple[float, float]] = []  # (x, y) in robot frame
        d = center_deg - half_deg
        while d <= center_deg + half_deg:
            r = self.range_at_deg(scan, d)
            if r is not None and r <= self.max_fit_range:
                a = math.radians(d)  # robot frame
                x = r * math.cos(a)
                y = r * math.sin(a)
                pts.append((x, y))
            d += 1.0

        if len(pts) < self.min_fit_points:
            return None

        # Fit line in ax + by + c = 0 form using least squares:
        # We can fit y = m x + b, but vertical-ish lines are possible.
        # Use PCA on points for robustness.

        # centroid
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)

        # covariance
        sxx = 0.0
        sxy = 0.0
        syy = 0.0
        for x, y in pts:
            dx = x - mx
            dy = y - my
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy

        # principal direction: eigenvector of largest eigenvalue
        # angle of direction:
        theta = 0.5 * math.atan2(2.0 * sxy, (sxx - syy))  # direction of maximum variance
        vx = math.cos(theta)
        vy = math.sin(theta)

        # normal vector to the line
        nx = -vy
        ny = vx

        # line passes through centroid: nx*(x-mx) + ny*(y-my)=0 -> nx*x + ny*y + c = 0
        c = -(nx * mx + ny * my)

        # perpendicular distance from origin to line:
        dist = abs(c) / max(1e-6, math.hypot(nx, ny))

        # wall direction angle relative to forward is theta (direction vector)
        # Normalize to [-pi, pi]
        wall_ang = (theta + math.pi) % (2.0 * math.pi) - math.pi

        return dist, wall_ang

    def rate_limit(self, w_cmd: float) -> float:
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
        cmd = Twist()

        # Corner/clutter look (front-left or front-right sector)
        front_corner = self.robust_min_window(scan, self.corner_deg, 18.0)
        front_center = self.robust_min_window(scan, 0.0, 20.0)

        # Side wall line fit
        fit = self.fit_wall_in_sector(scan, self.side_deg, self.fit_half_deg)

        # Acquire mode (no wall in range)
        if fit is None:
            v = self.acquire_forward_speed
            w = self.acquire_turn * self.side_sign

            # If something is in front, turn away strongly
            if front_center is not None and front_center < self.corner_stop_dist:
                v = 0.0
                w = -0.9 * self.side_sign
            elif front_center is not None and front_center < self.corner_slow_dist:
                t = (front_center - self.corner_stop_dist) / max(1e-3, (self.corner_slow_dist - self.corner_stop_dist))
                v = clamp(v * t, self.corner_speed_min, self.acquire_forward_speed)

            cmd.linear.x = float(v)
            cmd.angular.z = float(self.rate_limit(clamp(w, -self.max_ang, self.max_ang)))
            self.latest_cmd = cmd
            return

        dist, wall_ang = fit

        # Filter distance and wall angle (angle filtering with wrap)
        self.filt_dist = self.lpf(self.filt_dist, dist, self.dist_alpha)

        if self.filt_wall_ang is None:
            self.filt_wall_ang = wall_ang
        else:
            # wrap-aware LPF for angles
            err = angdiff(wall_ang, self.filt_wall_ang)
            self.filt_wall_ang = self.filt_wall_ang + self.ang_alpha * err

        dist_f = self.filt_dist
        ang_f = self.filt_wall_ang

        # If the "wall" is far, treat as acquire-ish
        if dist_f is None or dist_f > self.acquire_wall_dist:
            v = self.acquire_forward_speed
            w = self.acquire_turn * self.side_sign
            cmd.linear.x = float(v)
            cmd.angular.z = float(self.rate_limit(clamp(w, -self.max_ang, self.max_ang)))
            self.latest_cmd = cmd
            return

        # Core controller:
        # - distance error: if too far from left wall -> turn left (positive w) when following left
        e_dist = float(dist_f - self.desired_dist)

        # - wall angle: want wall direction parallel to forward (angle 0)
        #   If wall is angled such that it points "toward" us, correct.
        e_ang = float(ang_f)

        # Turn command (sign so it works for left/right)
        w_cmd = self.side_sign * (self.k_dist * e_dist) + (-self.k_ang * e_ang)

        # Base speed
        v_cmd = self.linear_speed

        # Corner slowdown:
        # If front-left/front-right is close, slow down, and bias turn away from corner
        if front_corner is not None:
            if front_corner < self.corner_stop_dist:
                v_cmd = 0.0
                w_cmd = -0.9 * self.side_sign  # turn away from the followed wall corner
            elif front_corner < self.corner_slow_dist:
                t = (front_corner - self.corner_stop_dist) / max(1e-3, (self.corner_slow_dist - self.corner_stop_dist))
                v_cmd = clamp(v_cmd * t, self.corner_speed_min, self.linear_speed)
                # small additional turn away from corner
                w_cmd += (-0.35 * self.side_sign) * (1.0 - clamp(t, 0.0, 1.0))

        # Front center "never crash"
        if front_center is not None:
            if front_center < self.corner_stop_dist:
                v_cmd = 0.0
                w_cmd = -0.9 * self.side_sign
            elif front_center < self.corner_slow_dist:
                t = (front_center - self.corner_stop_dist) / max(1e-3, (self.corner_slow_dist - self.corner_stop_dist))
                v_cmd = clamp(v_cmd * t, self.corner_speed_min, v_cmd)

        w_cmd = clamp(w_cmd, -self.max_ang, self.max_ang)
        w_cmd = self.rate_limit(w_cmd)

        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.latest_cmd = cmd

        if self.debug_print:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self._last_dbg_t) > self.debug_every_s:
                self._last_dbg_t = now
                self.get_logger().info(
                    f"dist={dist_f:.2f}m ang={math.degrees(ang_f):+.1f}deg "
                    f"front={front_center if front_center is not None else float('nan'):.2f} "
                    f"corner={front_corner if front_corner is not None else float('nan'):.2f} "
                    f"cmd v={v_cmd:.2f} w={w_cmd:+.2f}"
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
