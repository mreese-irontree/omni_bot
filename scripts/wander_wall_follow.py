#!/usr/bin/env python3
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def median(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WanderWallFollow(Node):
    """
    Simple LiDAR-based wander + right-wall-follow:
    - Drives forward by default
    - If obstacle close in front -> stop + turn away
    - Else: wall-follow on the right side if right ranges are available
    - Adds a small random bias occasionally to avoid looping
    """

    def __init__(self):
        super().__init__('wander_wall_follow')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')

        # Speeds
        self.declare_parameter('forward_speed', 0.14)
        self.declare_parameter('turn_speed', 0.80)

        # Front safety
        self.declare_parameter('front_stop_m', 0.55)   # stop+turn if closer than this
        self.declare_parameter('front_slow_m', 0.80)   # linearly slow down in this zone

        # Wall follow target (right side)
        self.declare_parameter('wall_target_m', 0.55)
        self.declare_parameter('wall_kp', 1.2)         # proportional gain

        # Limits
        self.declare_parameter('max_wall_correction', 0.70)  # clamp for wall correction (rad/s)
        self.declare_parameter('max_total_angular', 1.00)    # clamp for total angular cmd

        # Loop rate
        self.declare_parameter('rate_hz', 15.0)

        # Exploration bias (small random angular add)
        self.declare_parameter('bias_change_sec', 8.0)
        self.declare_parameter('bias_max', 0.35)

        # Debug
        self.declare_parameter('debug_hz', 1.0)  # 0 disables

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.fwd = float(self.get_parameter('forward_speed').value)
        self.turn = float(self.get_parameter('turn_speed').value)

        self.front_stop = float(self.get_parameter('front_stop_m').value)
        self.front_slow = float(self.get_parameter('front_slow_m').value)

        self.wall_target = float(self.get_parameter('wall_target_m').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)

        self.max_wall_correction = float(self.get_parameter('max_wall_correction').value)
        self.max_total_angular = float(self.get_parameter('max_total_angular').value)

        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.bias_change = float(self.get_parameter('bias_change_sec').value)
        self.bias_max = float(self.get_parameter('bias_max').value)

        self.debug_hz = float(self.get_parameter('debug_hz').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.last_scan: LaserScan | None = None

        self.bias = 0.0
        self.bias_until = time.time()

        self._dbg_last = 0.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f"WanderWallFollow running. scan={self.scan_topic} cmd_vel={self.cmd_vel_topic} "
            f"fwd={self.fwd} turn={self.turn} front_stop={self.front_stop} wall_target={self.wall_target}"
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _sector_median(self, scan: LaserScan, ang_min_deg: float, ang_max_deg: float):
        """
        Compute median range in a sector.
        Angles are degrees relative to forward (0 deg).
        NOTE: LD19 publishes 0..2pi, and your code already assumes:
          right  ~ -90 deg, left ~ +90 deg
        This function uses scan.angle_min/angle_increment so it works either way,
        as long as the sector angles match the scan convention you're using.
        """
        a0 = math.radians(ang_min_deg)
        a1 = math.radians(ang_max_deg)

        rmin = max(0.02, float(scan.range_min))
        rmax = float(scan.range_max)

        vals = []
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if a0 <= a <= a1:
                if not math.isfinite(r):
                    continue
                if rmin < r < rmax:
                    vals.append(r)

        return median(vals)

    def _maybe_debug(self, front, right, left, cmd: Twist):
        if self.debug_hz <= 0.0:
            return
        now = time.time()
        if now - self._dbg_last >= (1.0 / self.debug_hz):
            self._dbg_last = now
            self.get_logger().info(
                f"front={front} right={right} left={left} "
                f"bias={self.bias:+.2f} cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})"
            )

    def tick(self):
        if self.last_scan is None:
            return

        now = time.time()

        # Update exploration bias occasionally
        if now >= self.bias_until:
            self.bias = random.uniform(-self.bias_max, self.bias_max)
            self.bias_until = now + self.bias_change

        scan = self.last_scan

        # Sectors (deg)
        front = self._sector_median(scan, -15, 15)
        right = self._sector_median(scan, -110, -70)
        left  = self._sector_median(scan, 70, 110)

        cmd = Twist()
        cmd.linear.x = self.fwd
        cmd.angular.z = 0.0

        # ---- FRONT STOP + TURN ----
        if front is not None and front < self.front_stop:
            cmd.linear.x = 0.0

            # Turn away from the closer side when possible
            if right is not None and left is not None:
                cmd.angular.z = self.turn if right < left else -self.turn
            elif right is not None:
                cmd.angular.z = self.turn  # obstacle on right -> turn left
            elif left is not None:
                cmd.angular.z = -self.turn # obstacle on left -> turn right
            else:
                cmd.angular.z = self.turn  # fallback

            cmd.angular.z = clamp(cmd.angular.z, -self.max_total_angular, self.max_total_angular)
            self._maybe_debug(front, right, left, cmd)
            self.pub.publish(cmd)
            return

        # ---- FRONT SLOWDOWN ----
        if front is not None and front < self.front_slow:
            t = (front - self.front_stop) / max(1e-6, (self.front_slow - self.front_stop))
            t = clamp(t, 0.0, 1.0)
            cmd.linear.x *= t

        # ---- RIGHT WALL FOLLOW ----
        # IMPORTANT CHANGE: if right is None, do NOT force a right turn.
        if right is not None:
            # err positive => too far from wall => turn right (negative yaw if right is negative angles)
            # Your previous convention used: +err means too far -> turn right
            # That matched: cmd.angular.z += clamp(kp*err, -0.7, 0.7)
            err = (self.wall_target - right)
            wall_correction = clamp(self.wall_kp * err, -self.max_wall_correction, self.max_wall_correction)
            cmd.angular.z += wall_correction

        # ---- EXPLORATION BIAS ----
        cmd.angular.z += self.bias

        # Clamp final angular speed
        cmd.angular.z = clamp(cmd.angular.z, -self.max_total_angular, self.max_total_angular)

        self._maybe_debug(front, right, left, cmd)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = WanderWallFollow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
