#!/usr/bin/env python3
import math
from typing import Optional, List

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
    Stable wall follower (no encoders required).

    Key idea:
      - Use TWO rays on follow side: theta=90deg and theta=60deg (or symmetric on right)
      - Compute wall angle alpha and predicted distance d_pred at lookahead L.
      - Control uses d_pred error and alpha heading error.
      - Includes filtering, hysteresis, and safe seek (no spin-in-place).

    Works with scan published as either:
      - angle_min ~ 0, angle_max ~ 2*pi (LD19 typical)
      - angle_min ~ -pi, angle_max ~ +pi

    scan_yaw_offset_deg:
      Rotate the scan so robot-forward is 0 deg.
      You’ve been using -90.0 which is plausible for your mounting.
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Behavior
        self.declare_parameter("follow_side", "left")     # left or right
        self.declare_parameter("scan_yaw_offset_deg", -90.0)

        # Desired wall distance & lookahead
        self.declare_parameter("desired_dist", 0.35)      # meters
        self.declare_parameter("lookahead", 0.45)         # meters ahead projection

        # Speeds
        self.declare_parameter("v_nom", 0.12)             # nominal forward speed (m/s)
        self.declare_parameter("v_min", 0.05)
        self.declare_parameter("v_max", 0.18)

        self.declare_parameter("w_max", 0.9)              # rad/s limit (keep under 1.0)
        self.declare_parameter("w_deadband", 0.06)        # ignore tiny steering commands

        # Control gains (start conservative)
        self.declare_parameter("k_dist", 1.8)             # distance error -> turn
        self.declare_parameter("k_alpha", 1.1)            # wall angle -> turn

        # Ray angles (deg) on the follow side
        self.declare_parameter("theta_far_deg", 60.0)     # "ahead-side" ray
        self.declare_parameter("theta_near_deg", 90.0)    # true side ray
        self.declare_parameter("ray_window_deg", 6.0)     # median window half-width

        # Front safety (optional but very useful for corners)
        self.declare_parameter("front_half_deg", 20.0)
        self.declare_parameter("front_stop", 0.28)
        self.declare_parameter("front_slow", 0.70)

        # Wall reacquire behavior (NO spinning)
        self.declare_parameter("seek_turn", 0.35)         # gentle turn toward wall side
        self.declare_parameter("seek_v", 0.08)            # still moves forward while seeking
        self.declare_parameter("wall_lost_dist", 2.0)     # if side reading above this => wall lost
        self.declare_parameter("wall_hold_s", 1.0)        # how long to trust last wall before seeking

        # Output smoothing (huge difference!)
        self.declare_parameter("filter_alpha", 0.35)      # 0..1, higher = smoother/slower
        self.declare_parameter("filter_dist", 0.35)

        self.declare_parameter("slew_w", 2.5)             # rad/s^2, max change per second
        self.declare_parameter("slew_v", 0.6)             # m/s^2

        # Loop rate / debug
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_every_s", 0.5)

        # ---- Read params ----
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.follow_side = str(self.get_parameter("follow_side").value).strip().lower()
        if self.follow_side not in ("left", "right"):
            self.follow_side = "left"

        self.scan_yaw_offset_deg = float(self.get_parameter("scan_yaw_offset_deg").value)

        self.desired = float(self.get_parameter("desired_dist").value)
        self.L = float(self.get_parameter("lookahead").value)

        self.v_nom = float(self.get_parameter("v_nom").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.v_max = float(self.get_parameter("v_max").value)

        self.w_max = float(self.get_parameter("w_max").value)
        self.w_deadband = float(self.get_parameter("w_deadband").value)

        self.k_dist = float(self.get_parameter("k_dist").value)
        self.k_alpha = float(self.get_parameter("k_alpha").value)

        self.theta_far = float(self.get_parameter("theta_far_deg").value)
        self.theta_near = float(self.get_parameter("theta_near_deg").value)
        self.win = float(self.get_parameter("ray_window_deg").value)

        self.front_half = float(self.get_parameter("front_half_deg").value)
        self.front_stop = float(self.get_parameter("front_stop").value)
        self.front_slow = float(self.get_parameter("front_slow").value)

        self.seek_turn = float(self.get_parameter("seek_turn").value)
        self.seek_v = float(self.get_parameter("seek_v").value)
        self.wall_lost_dist = float(self.get_parameter("wall_lost_dist").value)
        self.wall_hold_s = float(self.get_parameter("wall_hold_s").value)

        self.f_alpha = float(self.get_parameter("filter_alpha").value)
        self.f_dist = float(self.get_parameter("filter_dist").value)

        self.slew_w = float(self.get_parameter("slew_w").value)
        self.slew_v = float(self.get_parameter("slew_v").value)

        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.debug = bool(self.get_parameter("debug").value)
        self.debug_every_s = float(self.get_parameter("debug_every_s").value)

        # ---- ROS I/O ----
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        # ---- State ----
        self.have_scan = False
        self.last_scan: Optional[LaserScan] = None

        self.alpha_f = 0.0          # filtered wall angle (rad)
        self.dpred_f = self.desired # filtered predicted dist (m)
        self.last_wall_t = 0.0

        self.cmd_v = 0.0            # smoothed output
        self.cmd_w = 0.0
        self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

        self._last_dbg = 0.0

        self.get_logger().info(
            f"WallFollower ready: side={self.follow_side} desired={self.desired:.2f} "
            f"yaw_offset={self.scan_yaw_offset_deg:+.1f} cmd={self.cmd_topic}"
        )

    def on_scan(self, scan: LaserScan):
        self.have_scan = True
        self.last_scan = scan

    def on_timer(self):
        if not self.have_scan or self.last_scan is None:
            return

        scan = self.last_scan

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = max(1e-3, now - self._last_cmd_time)
        self._last_cmd_time = now

        # Determine sign conventions
        # Robot frame: 0=forward, +CCW left.
        # For left-follow, positive steering usually means "turn left".
        # But depending on your motor / esp32 mixing, this might invert.
        # We'll keep this as "normal ROS": +w = left turn.
        side_sign = +1.0 if self.follow_side == "left" else -1.0

        # Rays in robot frame (degrees)
        # Left follow uses + angles, right follow uses - angles
        theta_near = side_sign * abs(self.theta_near)
        theta_far = side_sign * abs(self.theta_far)

        # Get two-ray distances
        r_near = self.median_window_range(scan, theta_near, self.win)
        r_far  = self.median_window_range(scan, theta_far, self.win)

        # Front range for corner behavior / collision avoidance
        r_front = self.median_window_range(scan, 0.0, self.front_half)

        wall_ok = (r_near is not None) and (r_far is not None) and (r_near < self.wall_lost_dist)

        if wall_ok:
            # Compute wall angle alpha and distance to wall from geometry
            # Using typical wall-follow derivation:
            #   alpha = atan2(r_far*cos(theta) - r_near, r_far*sin(theta))
            # where theta is the separation between the rays (near - far).
            #
            # Here theta_near is 90, theta_far is 60, so separation is 30deg.
            #
            theta = math.radians(abs(theta_near - theta_far))
            # Use the ray at theta_far as "a", near as "b" in the classic formula
            a = float(r_far)
            b = float(r_near)

            # Protect against degenerate cases
            denom = max(1e-6, a * math.sin(theta))
            alpha = math.atan2((a * math.cos(theta) - b), denom)  # rad

            # Current perpendicular distance to wall:
            d = b * math.cos(alpha)

            # Predict distance at lookahead L:
            d_pred = d + self.L * math.sin(alpha)

            # Filter them
            self.alpha_f = (1.0 - self.f_alpha) * self.alpha_f + self.f_alpha * alpha
            self.dpred_f = (1.0 - self.f_dist) * self.dpred_f + self.f_dist * d_pred

            self.last_wall_t = now

            # Control: keep dpred near desired, and keep alpha near 0
            e = (self.desired - self.dpred_f)   # + => need to get closer to wall
            w_cmd = side_sign * (self.k_dist * e + self.k_alpha * (-self.alpha_f))

            # Speed scheduling: slow down if turning hard or if front is close
            v_cmd = self.v_nom

            turn_mag = min(1.0, abs(w_cmd) / max(1e-6, self.w_max))
            v_cmd *= (1.0 - 0.55 * turn_mag)

            # Front slow/stop
            if r_front is not None:
                if r_front < self.front_stop:
                    v_cmd = 0.0
                    # turn away from obstacle (away from front => turn toward open space)
                    w_cmd = -side_sign * 0.7
                elif r_front < self.front_slow:
                    t = (r_front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
                    v_cmd = clamp(v_cmd * t, 0.0, self.v_nom)

            v_cmd = clamp(v_cmd, self.v_min if v_cmd > 0.0 else 0.0, self.v_max)
            w_cmd = clamp(w_cmd, -self.w_max, self.w_max)

            # Deadband to stop dithering
            if abs(w_cmd) < self.w_deadband:
                w_cmd = 0.0

            # Slew-rate limit (prevents snap left-right)
            self.cmd_w = self.slew_limit(self.cmd_w, w_cmd, self.slew_w, dt)
            self.cmd_v = self.slew_limit(self.cmd_v, v_cmd, self.slew_v, dt)

            self.publish(self.cmd_v, self.cmd_w)

            if self.debug and (now - self._last_dbg) > self.debug_every_s:
                self._last_dbg = now
                self.get_logger().info(
                    f"follow d_pred={self.dpred_f:.3f} e={e:+.3f} alpha={self.alpha_f:+.3f} "
                    f"front={(r_front if r_front is not None else -1.0):.3f} "
                    f"cmd v={self.cmd_v:.2f} w={self.cmd_w:+.2f}"
                )
            return

        # ---- SEEK mode (wall missing): DO NOT SPIN IN PLACE ----
        # If we recently saw a wall, gently steer back toward it while moving forward.
        wall_recent = (now - self.last_wall_t) < self.wall_hold_s

        v_cmd = self.seek_v
        w_cmd = side_sign * self.seek_turn

        # If front is close, stop and turn away to avoid punching into things
        if r_front is not None and r_front < self.front_stop:
            v_cmd = 0.0
            w_cmd = -side_sign * 0.7
        elif r_front is not None and r_front < self.front_slow:
            t = (r_front - self.front_stop) / max(1e-3, (self.front_slow - self.front_stop))
            v_cmd = clamp(v_cmd * t, 0.0, self.seek_v)

        # If not recent, reduce steering even more (so it doesn't “windmill”)
        if not wall_recent:
            w_cmd *= 0.5

        v_cmd = clamp(v_cmd, 0.0, self.v_max)
        w_cmd = clamp(w_cmd, -self.w_max, self.w_max)
        if abs(w_cmd) < self.w_deadband:
            w_cmd = 0.0

        self.cmd_w = self.slew_limit(self.cmd_w, w_cmd, self.slew_w, dt)
        self.cmd_v = self.slew_limit(self.cmd_v, v_cmd, self.slew_v, dt)

        self.publish(self.cmd_v, self.cmd_w)

        if self.debug and (now - self._last_dbg) > self.debug_every_s:
            self._last_dbg = now
            self.get_logger().info(
                f"seek wall_recent={wall_recent} front={(r_front if r_front is not None else -1.0):.3f} "
                f"cmd v={self.cmd_v:.2f} w={self.cmd_w:+.2f}"
            )

    def publish(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def slew_limit(self, current: float, target: float, max_rate: float, dt: float) -> float:
        # max change = max_rate * dt
        step = max_rate * dt
        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target

    def deg_to_rad_in_scan(self, scan: LaserScan, deg_robot: float) -> float:
        # Apply yaw offset so deg_robot=0 means robot forward
        deg_scan = wrap_deg_0_360(deg_robot + self.scan_yaw_offset_deg)

        # If scan uses -pi..pi, map to -180..180
        if scan.angle_min < 0.0:
            if deg_scan > 180.0:
                deg_scan -= 360.0

        return math.radians(deg_scan)

    def range_at_deg(self, scan: LaserScan, deg_robot: float) -> Optional[float]:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return None

        ang = self.deg_to_rad_in_scan(scan, deg_robot)
        idx = int(round((ang - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None

        r = float(scan.ranges[idx])
        if not is_valid(scan, r):
            return None
        return r

    def median_window_range(self, scan: LaserScan, deg_center: float, half_window_deg: float) -> Optional[float]:
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
        return vals[len(vals) // 2]


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
