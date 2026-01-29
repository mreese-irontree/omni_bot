#!/usr/bin/env python3
import math
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    # Rotate vector v by quaternion q (x,y,z,w)
    # Using q * v * q_conj
    # Convert v to quaternion (vx,vy,vz,0)
    # Optimized form:
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)
    return vpx, vpy, vpz


class DepthSafetyFilter(Node):
    """
    TF-aware depth safety:
      - reads PointCloud2 in camera frame
      - transforms sampled points into base_link
      - safety corridor uses base_link axes:
          forward = +X, left/right = +/-Y, height = +Z
    """

    def __init__(self):
        super().__init__('depth_safety_filter')

        # Topics
        self.declare_parameter('point_cloud_topic', '/point_cloud')
        self.declare_parameter('cmd_vel_in', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')

        # Frames
        self.declare_parameter('base_frame', 'base_link')

        # Enable/disable quickly
        self.declare_parameter('enabled', True)

        # Corridor + stop behavior (in base_link frame)
        self.declare_parameter('stop_dist_m', 0.55)           # stop if obstacle closer than this (forward X)
        self.declare_parameter('min_dist_m', 0.12)            # ignore super-close noise
        self.declare_parameter('corridor_half_width_m', 0.22) # |Y| < this
        self.declare_parameter('min_height_m', 0.03)          # ignore floor / very low noise (Z < this)
        self.declare_parameter('max_height_m', 0.35)          # only consider “low obstacles” up to this height

        # Sampling / performance
        self.declare_parameter('max_points_to_check', 2500)
        self.declare_parameter('cloud_timeout_sec', 0.5)

        # Behavior when blocked
        self.declare_parameter('blocked_turn_radps', -0.8)    # turn right if no turn commanded
        self.declare_parameter('hold_blocked_sec', 0.25)

        self.pc_topic = self.get_parameter('point_cloud_topic').value
        self.cmd_in = self.get_parameter('cmd_vel_in').value
        self.cmd_out = self.get_parameter('cmd_vel_out').value

        self.base_frame = self.get_parameter('base_frame').value

        self.enabled = bool(self.get_parameter('enabled').value)

        self.stop_dist = float(self.get_parameter('stop_dist_m').value)
        self.min_dist = float(self.get_parameter('min_dist_m').value)
        self.half_w = float(self.get_parameter('corridor_half_width_m').value)
        self.min_h = float(self.get_parameter('min_height_m').value)
        self.max_h = float(self.get_parameter('max_height_m').value)

        self.max_points = int(self.get_parameter('max_points_to_check').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout_sec').value)

        self.blocked_turn = float(self.get_parameter('blocked_turn_radps').value)
        self.hold_blocked = float(self.get_parameter('hold_blocked_sec').value)

        self.last_cloud_time = 0.0
        self.last_blocked_until = 0.0
        self.latest_cloud = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_in, self.on_cmd, 10)
        self.sub_pc = self.create_subscription(PointCloud2, self.pc_topic, self.on_cloud, 10)

        self.get_logger().info(
            "DepthSafetyFilter(TF) running. "
            f"cloud={self.pc_topic} cmd_in={self.cmd_in} cmd_out={self.cmd_out} base_frame={self.base_frame} "
            f"corridor: X[{self.min_dist},{self.stop_dist}] |Y|<{self.half_w} Z[{self.min_h},{self.max_h}]"
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

    def _lookup_tf(self, target_frame: str, source_frame: str):
        try:
            # Use "latest" transform (Time=0)
            return self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _blocked_by_depth(self) -> bool:
        if not self.enabled:
            return False
        if self.latest_cloud is None:
            return False
        if not self._cloud_is_fresh():
            return False

        cloud = self.latest_cloud
        source_frame = cloud.header.frame_id.strip()
        if not source_frame:
            return False

        tf = self._lookup_tf(self.base_frame, source_frame)
        if tf is None:
            # If TF is missing, don't hard-stop (avoid freezing forever)
            self.get_logger().warn_throttle(2.0, f"No TF {self.base_frame} <- {source_frame}; depth safety skipping.")
            return False

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        offsets = self._get_xyz_offsets(cloud)
        if offsets is None:
            self.get_logger().warn_throttle(2.0, "PointCloud2 missing x/y/z fields; skipping.")
            return False

        ox, oy, oz = offsets
        step = cloud.point_step
        data = cloud.data
        n = cloud.width * cloud.height
        if n <= 0:
            return False

        stride = max(1, n // max(1, self.max_points))

        # Corridor test in base_link
        for i in range(0, n, stride):
            base = i * step
            cx = struct.unpack_from('<f', data, base + ox)[0]
            cy = struct.unpack_from('<f', data, base + oy)[0]
            cz = struct.unpack_from('<f', data, base + oz)[0]

            if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(cz)):
                continue
            if abs(cx) < 1e-6 and abs(cy) < 1e-6 and abs(cz) < 1e-6:
                continue

            # Transform camera point -> base_link
            rx, ry, rz = quat_rotate(qx, qy, qz, qw, cx, cy, cz)
            bx = rx + tx
            by = ry + ty
            bz = rz + tz

            # Only consider low-ish obstacles in front corridor
            if bx < self.min_dist or bx > self.stop_dist:
                continue
            if abs(by) > self.half_w:
                continue
            if bz < self.min_h or bz > self.max_h:
                continue

            return True

        return False

    def on_cmd(self, cmd: Twist):
        out = Twist()
        out.linear.x = cmd.linear.x
        out.angular.z = cmd.angular.z

        blocked = self._blocked_by_depth()

        now = time.time()
        if blocked:
            self.last_blocked_until = now + self.hold_blocked

        if now < self.last_blocked_until:
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
