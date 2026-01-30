#!/usr/bin/env python3
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def valid_range(scan: LaserScan, r: float) -> bool:
    return math.isfinite(r) and (scan.range_min <= r <= scan.range_max)


class WallFollower(Node):
    """
    Subscribes: /scan (LaserScan)
    Publishes:  /cmd_vel_raw (Twist)
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Behavior
        self.declare_parameter("follow_side", "left")   # left or right
        self.declare_parameter("desired_dist", 0.35)    # meters
        self.declare_parameter("linear_speed", 0.20)    # m/s
        self.declare_parameter("max_ang", 1.2)          # rad/s

        # Gains
        self.declare_parameter("kp", 2.5)
        self.declare_parameter("kd", 0.8)
        self.declare_parameter("heading_k", 1.2)

        # Front safety (lidar-based)
        self.declare_parameter("front_stop_dist", 0.28)
        self.declare_parameter("front_slow_dist", 0.55)

        self.declare_parameter("publish_rate_hz", 20.0)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.follow_side = self.get_parameter("follow_side").value
        self.desired_dist = float(self.get_parameter("desired_dist").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.max_ang = float(self.get_parameter("max_ang").value)

        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.heading_k = float(self.get_parameter("heading_k").value)

        self.front_stop_dist = float(self.get_parameter("front_stop_dist").value)
        self.front_slow_dist = float(self.get_parameter("front_slow_dist").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.last_error = 0.0
        self.have_scan = False
        self.latest_cmd = Twist()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(
            f"WallFollower: scan={self.scan_topic} cmd={self.cmd_topic} side={self.follow_side} "
            f"desired={self.desired_dist}m v={self.linear_speed}m/s"
        )

    def on_timer(self):
        if not self.have_scan:
            return
        self.cmd_pub.publish(self.latest_cmd)

    def get_range_deg(self, scan: LaserScan, deg: float) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None
        ang = math.radians(deg)
        idx = int(round((ang - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        r = scan.ranges[idx]
        if not valid_range(scan, r):
            return None
        return float(r)

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
        # use 20th percentile to avoid single outliers
        k = max(0, int(0.2 * (len(vals) - 1)))
        return vals[k]

    def on_scan(self, scan: LaserScan):
        self.have_scan = True

        front = self.robust_min_window(scan, 0.0, 12.0)

        if self.follow_side.lower() == "left":
            side_deg = 90.0
            front_side_deg = 45.0
            side_sign = +1.0
        else:
            side_deg = -90.0
            front_side_deg = -45.0
            side_sign = -1.0

        side = self.robust_min_window(scan, side_deg, 10.0)
        front_side = self.robust_min_window(scan, front_side_deg, 10.0)

        cmd = Twist()

        # Lost wall -> slow creep and turn to search
        if side is None:
            cmd.linear.x = 0.10
            cmd.angular.z = 0.50 * side_sign
            self.latest_cmd = cmd
            return

        # error = measured - desired
        # left wall: too close => error negative => ang negative (turn right) OK
        error = float(side - self.desired_dist)
        derr = (error - self.last_error) * self.publish_rate_hz
        self.last_error = error

        heading_term = 0.0
        if front_side is not None:
            heading_term = (front_side - side)  # negative means you're pointing into wall

        ang = side_sign * (self.kp * error + self.kd * derr + self.heading_k * heading_term)

        v = self.linear_speed
        if front is not None:
            if front < self.front_stop_dist:
                v = 0.0
                ang = -0.8 * side_sign
            elif front < self.front_slow_dist:
                t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                v = clamp(v * t, 0.05, self.linear_speed)

        cmd.linear.x = float(v)
        cmd.angular.z = float(clamp(ang, -self.max_ang, self.max_ang))

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
