#!/usr/bin/env python3
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def median(vals):
    vals = [v for v in vals if v is not None and not math.isinf(v) and not math.isnan(v)]
    if not vals:
        return None
    vals.sort()
    n = len(vals)
    return vals[n // 2]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WanderWallFollow(Node):
    def __init__(self):
        super().__init__('wander_wall_follow')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')

        self.declare_parameter('forward_speed', 0.14)
        self.declare_parameter('turn_speed', 0.80)

        self.declare_parameter('front_stop_m', 0.55)
        self.declare_parameter('front_slow_m', 0.80)

        # Wall follow: try to keep right side at this distance
        self.declare_parameter('wall_target_m', 0.55)
        self.declare_parameter('wall_kp', 1.2)

        self.declare_parameter('rate_hz', 15.0)

        # exploration: occasional random bias so it doesn’t just loop
        self.declare_parameter('bias_change_sec', 8.0)
        self.declare_parameter('bias_max', 0.35)  # rad/s added

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.fwd = float(self.get_parameter('forward_speed').value)
        self.turn = float(self.get_parameter('turn_speed').value)
        self.front_stop = float(self.get_parameter('front_stop_m').value)
        self.front_slow = float(self.get_parameter('front_slow_m').value)

        self.wall_target = float(self.get_parameter('wall_target_m').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)

        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.bias_change = float(self.get_parameter('bias_change_sec').value)
        self.bias_max = float(self.get_parameter('bias_max').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.last_scan = None
        self.bias = 0.0
        self.bias_until = time.time()

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(f"WanderWallFollow running. scan={self.scan_topic} cmd_vel={self.cmd_vel_topic}")

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _sector_median(self, scan: LaserScan, ang_min_deg: float, ang_max_deg: float):
        # angles in degrees, relative to forward (0 deg)
        a0 = math.radians(ang_min_deg)
        a1 = math.radians(ang_max_deg)

        vals = []
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if a0 <= a <= a1:
                if r > 0.02 and r < scan.range_max:
                    vals.append(r)
        return median(vals)

    def tick(self):
        if self.last_scan is None:
            return

        now = time.time()
        if now >= self.bias_until:
            self.bias = random.uniform(-self.bias_max, self.bias_max)
            self.bias_until = now + self.bias_change

        scan = self.last_scan

        # Front sector: +/- 15 degrees
        front = self._sector_median(scan, -15, 15)
        # Right wall: around -90 deg (right), take a wide slice
        right = self._sector_median(scan, -110, -70)
        left = self._sector_median(scan, 70, 110)

        cmd = Twist()

        # Default move
        cmd.linear.x = self.fwd
        cmd.angular.z = 0.0

        # If something is close in front, turn away from the closer side
        if front is not None and front < self.front_stop:
            cmd.linear.x = 0.0
            # If right is closer than left, turn left; else turn right
            if right is not None and left is not None:
                cmd.angular.z = self.turn if right < left else -self.turn
            elif right is not None:
                cmd.angular.z = self.turn
            else:
                cmd.angular.z = -self.turn
            self.pub.publish(cmd)
            return

        # Slow down when approaching something
        if front is not None and front < self.front_slow:
            t = (front - self.front_stop) / max(1e-6, (self.front_slow - self.front_stop))
            t = clamp(t, 0.0, 1.0)
            cmd.linear.x *= t

        # Wall follow (right side). If right is far/None, gently turn right to find wall
        if right is None:
            cmd.angular.z += -0.25
        else:
            err = (self.wall_target - right)  # +err means too far (need turn right)
            cmd.angular.z += clamp(self.wall_kp * err, -0.7, 0.7)

        # Add exploration bias
        cmd.angular.z += self.bias

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
