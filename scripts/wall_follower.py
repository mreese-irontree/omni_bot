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
    Lidar-only wall follower tuned for "office wall hugging".
    Works with scans that are 0..2pi OR -pi..pi.

    Uses two angles on the follow side:
      a = side ray (near 90deg)
      b = forward-side ray (near 45deg)
    From these, compute wall angle alpha and perpendicular distance.

    Also uses a wide front sector for corner behavior.
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        # Side: left/right
        self.declare_parameter("follow_side", "left")

        # Scan alignment: rotate scan so 0deg means robot forward.
        self.declare_parameter("scan_yaw_offset_deg", 0.0)

        # Follow distance
        self.declare_parameter("desired_dist", 0.35)

        # Speeds
        self.declare_parameter("v_nom", 0.18)
        self.declare_parameter("v_min", 0.06)
        self.declare_parameter("w_max", 1.2)

        # Geometry angles
        self.declare_parameter("side_deg", 90.0)          # side ray
        self.declare_parameter("fwd_side_deg", 50.0)      # forward-side ray (lookahead)
        self.declare_parameter("window_half_deg", 6.0)    # robust min window

        # Front safety
        self.declare_parameter("front_half_deg", 28.0)
        self.declare_parameter("front_stop", 0.32)
        self.declare_parameter("front_slow", 0.85)

        # Control gains
        self.declare_parameter("k_dist", 1.5)     # distance error -> turn
        self.declare_parameter("k_ang", 1.4)      # wall angle -> turn
        self.declare_parameter("corner_turn_gain", 1.0)  # extra turning when cornering

        # Command smoothing
        self.declare_parameter("cmd_smooth_tau", 0.30)  # seconds
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug
        self.declare_parameter("debug_print_nearest", True)
        self.declare_parameter("debug_print_every_s", 1.0)

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
        self.corner_turn_gain = float(self.get_parameter("corner_turn_gain").value)

        self.cmd_smooth_tau = float(self.get_parameter("cmd_smooth_tau").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.debug_print_nearest = bool(self.get_parameter("debug_print_nearest").value)
        self.debug_print_every_s = float(self.get_parameter("debug_print_every_s").value)
        self._last_debug_t = 0.0

        # ROS I/O
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # state
        self.have_scan = False
        self.latest_cmd = Twist()
        self.cmd_filt = Twist()

        self.get_logger().info(
            f"WallFollower: scan={self.scan_topic} cmd={self.cmd_topic} side={self.follow_side} "
            f"desired={self.desired_dist:.2f} scan_yaw_offset_deg={self.scan_yaw_offset_deg:.1f}"
        )

    # --- scan helpers ---
    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot_frame: float) -> float:
        # 0deg robot = forward, +CCW (left)
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
        # 25th percentile avoids single outliers
        k = max(0, int(0.25 * (len(vals) - 1)))
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
        deg_robot = deg_scan - self.scan_yaw_offset_deg
        deg_robot = ((deg_robot + 180.0) % 360.0) - 180.0
        return best_r, deg_robot

    def compute_wall(self, scan: LaserScan, side_sign: float) -> Tuple[Optional[float], Optional[float]]:
        """
        Returns (dist_perp, alpha)
          dist_perp: perpendicular distance to wall (meters)
          alpha: wall angle relative to robot forward (radians). 0 means wall parallel to robot heading.
        """
        # choose angles on follow side
        # left: +90 and +50 ; right: -90 and -50
        a_deg = side_sign * abs(self.side_deg)
        b_deg = side_sign * abs(self.fwd_side_deg)

        a = self.robust_min_window(scan, a_deg, self.window_half_deg)
        b = self.robust_min_window(scan, b_deg, self.window_half_deg)
        if a is None or b is None:
            return None, None

        # convert geometry
        # theta between rays:
        theta = math.radians(abs(self.side_deg - self.fwd_side_deg))
        if theta < 1e-3:
            return None, None

        # "alpha" from common wall follower derivation
        # alpha = atan((a*cos(theta) - b) / (a*sin(theta)))
        num = (a * math.cos(theta)) - b
        den = max(1e-6, a * math.sin(theta))
        alpha = math.atan2(num, den)

        # perpendicular distance to wall
        dist = b * math.cos(alpha)
        return dist, alpha

    def front_distance(self, scan: LaserScan) -> Optional[float]:
        return self.robust_min_window(scan, 0.0, self.front_half_deg)

    # --- control ---
    def on_scan(self, scan: LaserScan):
        self.have_scan = True

        # debug
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.debug_print_nearest and (now - self._last_debug_t) > self.debug_print_every_s:
            self._last_debug_t = now
            mr, ma = self.nearest_obstacle(scan)
            if mr is not None and ma is not None:
                self.get_logger().info(f"Nearest: {mr:.2f} m at {ma:+.1f} deg (robot frame, 0=forward)")
            else:
                self.get_logger().info("Nearest: none")

        side_sign = +1.0 if self.follow_side == "left" else -1.0

        front = self.front_distance(scan)

        dist, alpha = self.compute_wall(scan, side_sign)

        cmd = Twist()

        # If we can't see the wall on our side, do a gentle "search" turn toward that side
        if dist is None or alpha is None:
            v = self.v_min
            w = 0.35 * side_sign

            # but don't plow into something
            if front is not None:
                if front < self.front_stop:
                    v = 0.0
                    w = -0.9 * side_sign
                elif front < self.front_slow:
                    t = (front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
                    v = clamp(self.v_min * t, 0.0, self.v_min)

            cmd.linear.x = float(v)
            cmd.angular.z = float(clamp(w, -self.w_max, self.w_max))
            self.latest_cmd = cmd
            return

        # primary error terms
        e_dist = dist - self.desired_dist            # + means too far from wall
        e_ang = alpha                                # + means wall "tilted" in front-side ray sense

        # base turn command
        w = side_sign * (self.k_dist * e_dist + self.k_ang * e_ang)

        # Corner behavior:
        # If the front is getting close, increase turn *away* from front by turning more toward follow side
        if front is not None and front < self.front_slow:
            # 0..1 (0 near slow edge, 1 near stop)
            t = clamp((self.front_slow - front) / max(1e-3, (self.front_slow - self.front_stop)), 0.0, 1.0)
            w += side_sign * self.corner_turn_gain * t

        # speed schedule based on front
        v = self.v_nom
        if front is not None:
            if front < self.front_stop:
                v = 0.0
                w = -0.9 * side_sign  # bail out from frontal obstacle
            elif front < self.front_slow:
                t = (front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
                v = clamp(self.v_nom * t, self.v_min, self.v_nom)

        cmd.linear.x = float(v)
        cmd.angular.z = float(clamp(w, -self.w_max, self.w_max))
        self.latest_cmd = cmd

    def on_timer(self):
        if not self.have_scan:
            return

        # simple first-order smoothing on commands
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
