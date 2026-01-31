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
        self.declare_parameter("in_stop", "")  # if empty, ignore
        self.declare_parameter("out_cmd", "/cmd_vel_safe")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.in_cmd = self.get_parameter("in_cmd").value
        self.in_stop = str(self.get_parameter("in_stop").value)
        self.out_cmd = self.get_parameter("out_cmd").value
        self.rate = float(self.get_parameter("publish_rate_hz").value)

        self.pub = self.create_publisher(Twist, self.out_cmd, 10)
        self.sub_cmd = self.create_subscription(Twist, self.in_cmd, self.on_cmd, 10)

        self.stop_active = False
        self.last_stop_time = 0.0
        self.stop_timeout_s = 0.5

        if self.in_stop.strip():
            self.sub_stop = self.create_subscription(Bool, self.in_stop, self.on_stop, 10)
            self.get_logger().info(f"CmdMux: {self.in_cmd} + {self.in_stop} -> {self.out_cmd}")
        else:
            self.sub_stop = None
            self.get_logger().info(f"CmdMux: {self.in_cmd} -> {self.out_cmd}")

        self.latest_cmd = Twist()
        self.timer = self.create_timer(1.0 / max(1.0, self.rate), self.on_timer)

    def on_cmd(self, msg: Twist):
        self.latest_cmd = msg

    def on_stop(self, msg: Bool):
        self.stop_active = bool(msg.data)
        self.last_stop_time = time.time()

    def on_timer(self):
        out = Twist()

        if self.sub_stop is not None:
            # if stop is active or stale -> stop
            if self.stop_active:
                self.pub.publish(out)
                return
            if (time.time() - self.last_stop_time) > self.stop_timeout_s:
                # stop topic stale; be safe
                self.pub.publish(out)
                return

        out = self.latest_cmd
        self.pub.publish(out)


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
