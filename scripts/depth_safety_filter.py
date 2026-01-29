#!/usr/bin/env python3
import math
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DepthSafetyFilter(Node):
    """
    For your Arducam point cloud:
      - forward axis = z
      - lateral axes = x, y
    We look for any points in a forward corridor and, if found, veto forward motion.
    """

    def __init__(self):
        super().__init__('depth_safety_filter')

        # Topics
        self.declare_parameter('point_cloud_topic', '/point_cloud')
        self.declare_parameter('cmd_vel_in', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')

        # Enable/disable quickly
        self.declare_parameter('enabled', True)

        # Corridor + stop behavior
        self.declare_parameter('stop_dist_m', 0.45)            # stop if anything closer than this
        self.declare_parameter('min_dist_m', 0.08)             # ignore ultra-close noise
        self.declare_parameter('corridor_half_width_m', 0.20)  # abs(x) < this
        self.declare_parameter('corridor_half_height_m', 0.15) # abs(y) < this

        # Sampling / performance
        self.declare_parameter('max_points_to_check', 2500)    # downsample cap
        self.declare_parameter('cloud_timeout_sec', 0.5)       # if cloud stale, don't veto

        # Behavior when blocked
        self.declare_parameter('blocked_turn_radps', -0.8)     # turn right by default
        self.declare_parameter('hold_blocked_sec', 0.25)       # keep “blocked” state briefly

        self.pc_topic = self.get_parameter('point_cloud_topic').value
        self.cmd_in = self.get_parameter('cmd_vel_in').value
        self.cmd_out = self.get_parameter('cmd_vel_out').value

        self.enabled = bool(self.get_parameter('enabled').value)

        self.stop_dist = float(self.get_parameter('stop_dist_m').value)
        self.min_dist = float(self.get_parameter('min_dist_m').value)
        self.half_w = float(self.get_parameter('corridor_half_width_m').value)
        self.half_h = float(self.get_parameter('corridor_half_height_m').value)

        self.max_points = int(self.get_parameter('max_points_to_check').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout_sec').value)

        self.blocked_turn = float(self.get_parameter('blocked_turn_radps').value)
        self.hold_blocked = float(self.get_parameter('hold_blocked_sec').value)

        self.last_cloud_time = 0.0
        self.last_blocked_until = 0.0
        self.latest_cloud = None

        self.pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_in, self.on_cmd, 10)
        self.sub_pc = self.create_subscription(PointCloud2, self.pc_topic, self.on_cloud, 10)

        self.get_logger().info(
            f"DepthSafetyFilter running. cloud={self.pc_topic} cmd_in={self.cmd_in} cmd_out={self.cmd_out} "
            f"forward_axis=z corridor abs(x)<{self.half_w} abs(y)<{self.half_h} stop_dist={self.stop_dist}"
        )

    def on_cloud(self, msg: PointCloud2):
        self.latest_cloud = msg
        self.last_cloud_time = time.time()

    def _cloud_is_fresh(self) -> bool:
        return (time.time() - self.last_cloud_time) <= self.cloud_timeout

    def _get_xyz_offsets(self, msg: PointCloud2):
        offs = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                offs[f.name] = f.offset
        if any(k not in offs for k in ('x', 'y', 'z')):
            return None
        return offs['x'], offs['y'], offs['z']

    def _blocked_by_depth(self) -> bool:
        if not self.enabled:
            return False
        if self.latest_cloud is None:
            return False
        if not self._cloud_is_fresh():
            return False

        msg = self.latest_cloud
        offsets = self._get_xyz_offsets(msg)
        if offsets is None:
            self.get_logger().warn("PointCloud2 missing x/y/z fields; depth safety disabled for this cloud.")
            return False

        ox, oy, oz = offsets
        step = msg.point_step
        data = msg.data
        n = msg.width * msg.height
        if n <= 0:
            return False

        # Downsample by stride so we check at most max_points
        stride = max(1, n // max(1, self.max_points))

        # For YOUR camera:
        #   forward distance = z
        #   corridor width = x
        #   corridor height = y
        for i in range(0, n, stride):
            base = i * step
            x = struct.unpack_from('<f', data, base + ox)[0]
            y = struct.unpack_from('<f', data, base + oy)[0]
            z = struct.unpack_from('<f', data, base + oz)[0]

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                continue

            # forward corridor test
            if z < self.min_dist or z > self.stop_dist:
                continue
            if abs(x) > self.half_w:
                continue
            if abs(y) > self.half_h:
                continue

            return True

        return False

    def on_cmd(self, cmd: Twist):
        out = Twist()
        out.linear.x = cmd.linear.x
        out.angular.z = cmd.angular.z

        blocked = self._blocked_by_depth()

        # Sticky “blocked” window to avoid flicker
        now = time.time()
        if blocked:
            self.last_blocked_until = now + self.hold_blocked

        if now < self.last_blocked_until:
            # veto forward only; keep turning (or force turn if none)
            out.linear.x = 0.0
            if abs(out.angular.z) < 1e-3:
                out.angular.z = self.blocked_turn

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
