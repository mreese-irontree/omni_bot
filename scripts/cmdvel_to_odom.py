#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CmdVelToOdom(Node):
    """
    Open-loop odom from /cmd_vel integration (no encoders).
    Publishes /odom and TF odom->base_link.
    SLAM / AMCL can still work, but drift exists.
    """

    def __init__(self):
        super().__init__("cmdvel_to_odom")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("rate_hz", 50.0)

        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.base_frame = self.get_parameter("base_frame_id").value
        self.odom_frame = self.get_parameter("odom_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0

        self.linear_mps = 0.0
        self.angular_rps = 0.0
        self.last_cmd_time = time.time()

        self.last_time = time.time()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

    def on_cmd_vel(self, msg: Twist):
        self.linear_mps = float(msg.linear.x)
        self.angular_rps = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def on_timer(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now

        cmd_fresh = (now - self.last_cmd_time) <= self.timeout_sec
        v = self.linear_mps if cmd_fresh else 0.0
        w = self.angular_rps if cmd_fresh else 0.0

        self.yaw_rad += w * dt
        self.yaw_rad = math.atan2(math.sin(self.yaw_rad), math.cos(self.yaw_rad))

        self.x_m += v * math.cos(self.yaw_rad) * dt
        self.y_m += v * math.sin(self.yaw_rad) * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw_rad)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x_m
        odom_msg.pose.pose.position.y = self.y_m
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        # Loose covariances (open-loop)
        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.10

        odom_msg.twist.covariance[0] = 0.10
        odom_msg.twist.covariance[35] = 0.20

        self.publisher.publish(odom_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom_msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x_m
            tf_msg.transform.translation.y = self.y_m
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = CmdVelToOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
