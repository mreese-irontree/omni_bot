#!/usr/bin/env python3
import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def finite_range(r, rmin, rmax):
    return (r is not None) and math.isfinite(r) and (r > rmin) and (r < rmax)


def median(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


class WanderLidarWallFollow(Node):
    """
    LiDAR-only wall-follow + exploration, designed to KEEP MOVING.

    Key features:
      - Uses median-of-sector (robust to NaNs).
      - Correct handling for 0..2pi scans (wrap-around sectors).
      - Always publishes cmd_vel at rate_hz (prevents ESP32 timeout stops).
      - If right wall not seen, gently steer right to reacquire wall.
      - If front blocked, turn away from closer side until clear.
      - Adds slow-changing bias so it doesn't loop forever.
      - Anti-oscillation: limits angular rate and doesn't flip rapidly.
    """

    def __init__(self):
        super().__init__("wander_lidar")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Speeds
        self.declare_parameter("forward_speed", 0.14)      # m/s
        self.declare_parameter("turn_speed", 0.80)         # rad/s max

        # Front behavior
        self.declare_parameter("front_stop_m", 0.55)       # stop+turn if closer than this
        self.declare_parameter("front_slow_m", 0.90)       # slow when approaching

        # Wall follow (right side)
        self.declare_parameter("wall_target_m", 0.55)
        self.declare_parameter("wall_kp", 1.2)
        self.declare_parameter("wall_search_turn", -0.25)  # rad/s when right wall missing (negative=turn right)

        # Exploration bias
        self.declare_parameter("bias_change_sec", 8.0)
        self.declare_parameter("bias_max", 0.30)           # rad/s

        # Control loop
        self.declare_parameter("rate_hz", 15.0)

        # Scan robustness
        self.declare_parameter("min_valid_front", 8)       # require at least N valid ranges in front sector
        self.declare_parameter("scan_timeout_sec", 0.35)   # if scans stop, keep a gentle forward creep

        # Sector widths (degrees)
        self.declare_parameter("front_deg", 20.0)
        self.declare_parameter("right_center_deg", -90.0)
        self.declare_parameter("right_width_deg", 30.0)
        self.declare_parameter("left_center_deg", 90.0)
        self.declare_parameter("left_width_deg", 30.0)

        # Read params
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value

        self.fwd = float(self.get_parameter("forward_speed").value)
        self.turn_max = float(self.get_parameter("turn_speed").value)

        self.front_stop = float(self.get_parameter("front_stop_m").value)
        self.front_slow = float(self.get_parameter("front_slow_m").value)

        self.wall_target = float(self.get_parameter("wall_target_m").value)
        self.wall_kp = float(self.get_parameter("wall_kp").value)
        self.wall_search_turn = float(self.get_parameter("wall_search_turn").value)

        self.bias_change = float(self.get_parameter("bias_change_sec").value)
        self.bias_max = float(self.get_parameter("bias_max").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.min_valid_front = int(self.get_parameter("min_valid_front").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)

        self.front_deg = float(self.get_parameter("front_deg").value)
        self.right_center = float(self.get_parameter("right_center_deg").value)
        self.right_width = float(self.get_parameter("right_width_deg").value)
        self.left_center = float(self.get_parameter("left_center_deg").value)
        self.left_width = float(self.get_parameter("left_width_deg").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)

        self.last_scan = None
        self.last_scan_time = 0.0

        # State
        self.bias = 0.0
        self.bias_until = time.time()
        self.turn_dir = 1.0   # +left, -right
        self.turn_hold_until = 0.0  # prevent rapid flip-flop

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f"wander_lidar: scan={self.scan_topic} cmd_vel={self.cmd_topic} "
            f"fwd={self.fwd} turn_max={self.turn_max} wall_target={self.wall_target}"
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = time.time()

    def _angles_to_indices(self, scan: LaserScan, ang0: float, ang1: float):
        """
        Return list of indices for the angular interval [ang0, ang1] in radians,
        handling wrap-around for scans that go 0..2pi.
        """
        # Normalize angles into [0, 2pi)
        def norm(a):
            a = math.fmod(a, 2.0 * math.pi)
            if a < 0:
                a += 2.0 * math.pi
            return a

        a0 = norm(ang0)
        a1 = norm(ang1)

        # scan angles might not start at 0; build indices by checking each sample angle
        idxs = []
        n = len(scan.ranges)
        for i in range(n):
            a = scan.angle_min + i * scan.angle_increment
            a = norm(a)
            if a0 <= a1:
                if a0 <= a <= a1:
                    idxs.append(i)
            else:
                # interval wraps across 0
                if a >= a0 or a <= a1:
                    idxs.append(i)
        return idxs

    def _sector_stats(self, scan: LaserScan, center_deg: float, half_width_deg: float):
        """
        Returns (median_distance, valid_count).
        Uses robust filtering against NaNs/inf/out-of-range.
        """
        if scan is None or not scan.ranges:
            return (None, 0)

        center = math.radians(center_deg)
        half = math.radians(half_width_deg)

        # interval [center-half, center+half]
        idxs = self._angles_to_indices(scan, center - half, center + half)

        vals = []
        rmin = float(scan.range_min)
        rmax = float(scan.range_max)
        for i in idxs:
            r = scan.ranges[i]
            if finite_range(r, rmin, rmax):
                vals.append(float(r))

        if not vals:
            return (None, 0)

        return (median(vals), len(vals))

    def _pick_turn_dir(self, left_d, right_d):
        # turn away from closer side
        if left_d is None and right_d is None:
            return 1.0 if random.random() > 0.5 else -1.0
        if left_d is None:
            return 1.0   # left unknown -> assume open -> turn left
        if right_d is None:
            return -1.0  # right unknown -> assume open -> turn right
        return 1.0 if right_d < left_d else -1.0

    def _publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)

    def tick(self):
        now = time.time()

        # Keep bias fresh (slow changes)
        if now >= self.bias_until:
            self.bias = random.uniform(-self.bias_max, self.bias_max)
            self.bias_until = now + self.bias_change

        # If scans pause, DO NOT stop (prevents stalling on tabletop)
        if self.last_scan is None or (now - self.last_scan_time) > self.scan_timeout:
            # gentle creep forward + small bias
            v = 0.06
            w = clamp(self.bias, -0.25, 0.25)
            self._publish(v, w)
            return

        scan = self.last_scan

        # Sectors
        front_d, front_valid = self._sector_stats(scan, 0.0, self.front_deg)
        right_d, _ = self._sector_stats(scan, self.right_center, self.right_width)
        left_d, _ = self._sector_stats(scan, self.left_center, self.left_width)

        # Default command
        v = self.fwd
        w = 0.0

        # If front readings are too sparse, avoid making stop/turn decisions from junk
        front_ok = (front_d is not None) and (front_valid >= self.min_valid_front)

        # --- Obstacle in front: stop & turn until clear ---
        if front_ok and front_d < self.front_stop:
            # hold a chosen turn direction for a short time to avoid flip-flop
            if now >= self.turn_hold_until:
                self.turn_dir = self._pick_turn_dir(left_d, right_d)
                self.turn_hold_until = now + 0.6

            v = 0.0

            # turn harder if REALLY close
            k = 1.0
            if front_d < self.front_stop * 0.75:
                k = 1.35

            w = self.turn_dir * self.turn_max * k
            self._publish(v, w)
            self.get_logger().info(
                f"front={front_d:.3f} right={right_d} left={left_d} bias={self.bias:+.2f} cmd=({v:.2f},{w:+.2f})"
            )
            return

        # --- Slow down when approaching ---
        if front_ok and front_d < self.front_slow:
            t = (front_d - self.front_stop) / max(1e-6, (self.front_slow - self.front_stop))
            t = clamp(t, 0.15, 1.0)  # never go to 0 while "clear"
            v *= t

        # --- Wall follow on the right ---
        if right_d is None:
            # If right wall not visible: steer right gently to reacquire
            w += self.wall_search_turn
        else:
            # Error: positive means too far from wall -> turn right (negative yaw)
            err = (self.wall_target - right_d)
            w += clamp(self.wall_kp * err, -0.7, 0.7)

        # --- Exploration bias (small) ---
        w += self.bias
        w = clamp(w, -self.turn_max, self.turn_max)

        self._publish(v, w)

        # Optional debug once per ~1s
        if int(now) % 1 == 0:
            self.get_logger().info(
                f"front={front_d} right={right_d} left={left_d} bias={self.bias:+.2f} cmd=({v:.2f},{w:+.2f})"
            )


def main():
    rclpy.init()
    node = WanderLidarWallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop robot
        try:
            node.pub.publish(Twist())
            time.sleep(0.05)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
