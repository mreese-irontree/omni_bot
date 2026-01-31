#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdMux(Node):
    def __init__(self):
        super().__init__("cmd_mux")

        self.declare_parameter("in_cmd", "/cmd_vel_raw")
        self.declare_parameter("out_cmd", "/cmd_vel_safe")
        self.declare_parameter("stop_topic", "")  # empty = disabled
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("stop_hold_ms", 250)

        self.in_cmd = str(self.get_parameter("in_cmd").value)
        self.out_cmd = str(self.get_parameter("out_cmd").value)
        self.stop_topic = str(self.get_parameter("stop_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.stop_hold_ms = int(self.get_parameter("stop_hold_ms").value)

        self.cmd_sub = self.create_subscription(Twist, self.in_cmd, self.on_cmd, 10)
        self.cmd_pub = self.create_publisher(Twist, self.out_cmd, 10)

        self.stop_sub = None
        if self.stop_topic.strip() != "":
            self.stop_sub = self.create_subscription(Bool, self.stop_topic, self.on_stop, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.latest_cmd = Twist()
        self.last_cmd_time = time.time()

        self.stop_active_until = 0.0

        if self.stop_sub is None:
            self.get_logger().info(f"CmdMux: {self.in_cmd} -> {self.out_cmd} (stop disabled)")
        else:
            self.get_logger().info(f"CmdMux: {self.in_cmd} + {self.stop_topic} -> {self.out_cmd}")

    def on_cmd(self, msg: Twist):
        self.latest_cmd = msg
        self.last_cmd_time = time.time()

    def on_stop(self, msg: Bool):
        if bool(msg.data):
            self.stop_active_until = time.time() + (self.stop_hold_ms / 1000.0)

    def on_timer(self):
        now = time.time()
        if now < self.stop_active_until:
            z = Twist()
            self.cmd_pub.publish(z)
            return

        # pass-through
        self.cmd_pub.publish(self.latest_cmd)


def main():
    rclpy.init()
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
