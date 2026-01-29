#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class WanderLidar(Node):
    """
    Very simple LiDAR-only obstacle avoidance:
      - drive forward
      - if obstacle in front within front_stop_m -> rotate in place toward the clearer side
      - if sides are tight -> rotate harder / keep turning until clear

    This does NOT do mapping/localization; it just tries not to hit things.
    """

    def __init__(self):
        super().__init__('wander_lidar')

        # Params
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value

        self.forward_speed = float(self.declare_parameter('forward_speed', 0.12).value)
        self.turn_speed = float(self.declare_parameter('turn_speed', 0.65).value)

        self.front_stop_m = float(self.declare_parameter('front_stop_m', 0.55).value)
        self.side_clear_m = float(self.declare_parameter('side_clear_m', 0.40).value)

        self.rate_hz = float(self.declare_parameter('rate_hz', 15.0).value)

        # Internals
        self.last_scan = None
        self.turn_dir = 1.0  # +1 left, -1 right (chosen when obstacle appears)
        self.state = 'FORWARD'  # FORWARD or TURN

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"Wander LiDAR running. scan={self.scan_topic} cmd_vel={self.cmd_vel_topic} "
            f"front_stop_m={self.front_stop_m} side_clear_m={self.side_clear_m}"
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _sector_min(self, scan: LaserScan, ang_min: float, ang_max: float) -> float:
        """
        Returns minimum range in [ang_min, ang_max] (radians, in scan frame).
        Ignores inf/0/NaN.
        """
        if scan is None or not scan.ranges:
            return float('inf')

        # normalize requested angles into indices
        a0 = scan.angle_min
        inc = scan.angle_increment
        n = len(scan.ranges)

        i0 = int((ang_min - a0) / inc)
        i1 = int((ang_max - a0) / inc)
        i0 = clamp(i0, 0, n - 1)
        i1 = clamp(i1, 0, n - 1)
        if i1 < i0:
            i0, i1 = i1, i0

        m = float('inf')
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if r is None:
                continue
            if isinstance(r, float):
                if math.isinf(r) or math.isnan(r) or r <= 0.01:
                    continue
            if r < m:
                m = r
        return m

    def _compute_obstacles(self, scan: LaserScan):
        # Front: +-20 deg
        front = self._sector_min(scan, math.radians(-20), math.radians(20))
        # Left: 20..90 deg
        left = self._sector_min(scan, math.radians(20), math.radians(90))
        # Right: -90..-20 deg
        right = self._sector_min(scan, math.radians(-90), math.radians(-20))
        return front, left, right

    def stop(self):
        t = Twist()
        self.pub.publish(t)

    def on_timer(self):
        if self.last_scan is None:
            # no scan yet -> stop
            self.stop()
            return

        front, left, right = self._compute_obstacles(self.last_scan)

        # Decide state transitions
        if self.state == 'FORWARD':
            if front < self.front_stop_m:
                # choose turn direction toward the clearer side
                if left > right:
                    self.turn_dir = 1.0
                elif right > left:
                    self.turn_dir = -1.0
                else:
                    self.turn_dir = 1.0 if random.random() > 0.5 else -1.0
                self.state = 'TURN'
        else:  # TURN
            # return to forward when front is clear enough
            if front > (self.front_stop_m * 1.35):
                self.state = 'FORWARD'

        # Publish cmd_vel
        cmd = Twist()

        if self.state == 'FORWARD':
            # If a side is too close, bias turning away slightly
            bias = 0.0
            if left < self.side_clear_m and right >= self.side_clear_m:
                bias = -0.35
            elif right < self.side_clear_m and left >= self.side_clear_m:
                bias = 0.35
            elif left < self.side_clear_m and right < self.side_clear_m:
                # both tight -> slow and turn a bit
                bias = 0.55 if left > right else -0.55

            cmd.linear.x = self.forward_speed
            cmd.angular.z = bias * self.turn_speed
        else:
            # rotate in place; rotate harder if very close
            k = 1.0
            if front < (self.front_stop_m * 0.7):
                k = 1.35
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_dir * self.turn_speed * k

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = WanderLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
