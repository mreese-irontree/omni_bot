#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DepthSafetyFilter(Node):
    """
    cmd_vel_in  -> cmd_vel_out
    Uses PointCloud2 from depth camera to stop/slow for low obstacles.
    """

    def __init__(self):
        super().__init__('depth_safety_filter')

        self.declare_parameter('cmd_vel_in',  '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')
        self.declare_parameter('cloud_topic', '/point_cloud')

        # ROI in *cloud frame* (assumes x forward, y left, z up)
        self.declare_parameter('roi_x_min', 0.08)
        self.declare_parameter('roi_x_max', 1.20)
        self.declare_parameter('roi_y_min', -0.25)
        self.declare_parameter('roi_y_max',  0.25)

        # Ignore floor: only consider "low obstacles" above ground
        self.declare_parameter('min_z', 0.15)   # raise if floor triggers
        self.declare_parameter('max_z', 0.70)

        # Behavior thresholds
        self.declare_parameter('stop_dist_m', 0.35)
        self.declare_parameter('slow_dist_m', 0.55)
        self.declare_parameter('turn_rate',   0.8)

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('cloud_timeout_sec', 0.5)

        self.cmd_vel_in  = self.get_parameter('cmd_vel_in').value
        self.cmd_vel_out = self.get_parameter('cmd_vel_out').value
        self.cloud_topic = self.get_parameter('cloud_topic').value

        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)

        self.stop_dist = float(self.get_parameter('stop_dist_m').value)
        self.slow_dist = float(self.get_parameter('slow_dist_m').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout_sec').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_out, 10)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_in, self.on_cmd, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)

        self.last_cmd = Twist()
        self.last_cloud_time = 0.0
        self.min_front_dist = None

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f"DepthSafetyFilter: in={self.cmd_vel_in} out={self.cmd_vel_out} cloud={self.cloud_topic}"
        )

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def on_cloud(self, msg: PointCloud2):
        min_d = None
        try:
            for (x, y, z) in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if x < self.roi_x_min or x > self.roi_x_max:
                    continue
                if y < self.roi_y_min or y > self.roi_y_max:
                    continue
                if z < self.min_z or z > self.max_z:
                    continue
                if min_d is None or x < min_d:
                    min_d = x
        except Exception as e:
            self.get_logger().warn(f"PointCloud read failed: {e}")
            return

        self.min_front_dist = min_d
        self.last_cloud_time = time.time()

    def tick(self):
        # stale depth -> passthrough
        if (time.time() - self.last_cloud_time) > self.cloud_timeout:
            self.pub.publish(self.last_cmd)
            return

        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z

        d = self.min_front_dist

        if d is None:
            self.pub.publish(cmd)
            return

        if d <= self.stop_dist:
            cmd.linear.x = 0.0
            # Turn away to unstick
            cmd.angular.z = self.turn_rate if cmd.angular.z >= 0.0 else -self.turn_rate
            self.pub.publish(cmd)
            return

        if d <= self.slow_dist:
            t = (d - self.stop_dist) / max(1e-6, (self.slow_dist - self.stop_dist))
            t = clamp(t, 0.0, 1.0)
            cmd.linear.x *= t
            self.pub.publish(cmd)
            return

        self.pub.publish(cmd)


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
