#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmdvel_to_odom')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('cmd_timeout_ms', 300)

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.cmd_timeout_ms = int(self.get_parameter('cmd_timeout_ms').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

    def on_cmd(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        age_ms = (now - self.last_cmd_time).nanoseconds / 1e6
        if age_ms > self.cmd_timeout_ms:
            v = 0.0
            w = 0.0
        else:
            v = self.v
            w = self.w

        # integrate pose
        self.yaw += w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # publish TF odom->base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # yaw to quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(t)

        # publish nav_msgs/Odometry
        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.position.z = 0.0
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w

        self.odom_pub.publish(o)


def main():
    rclpy.init()
    node = CmdVelToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
