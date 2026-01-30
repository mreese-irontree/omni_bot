#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdMux(Node):
    """
    Subscribes:
      /cmd_vel_raw (Twist)
      /safety/stop (Bool)
    Publishes:
      /cmd_vel_safe (Twist)
    """

    def __init__(self):
        super().__init__("cmd_mux")

        self.declare_parameter("in_cmd", "/cmd_vel_raw")
        self.declare_parameter("in_stop", "/safety/stop")
        self.declare_parameter("out_cmd", "/cmd_vel_safe")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.in_cmd = self.get_parameter("in_cmd").value
        self.in_stop = self.get_parameter("in_stop").value
        self.out_cmd = self.get_parameter("out_cmd").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.pub = self.create_publisher(Twist, self.out_cmd, 10)
        self.sub_cmd = self.create_subscription(Twist, self.in_cmd, self.on_cmd, 10)
        self.sub_stop = self.create_subscription(Bool, self.in_stop, self.on_stop, 10)

        self.latest_cmd = Twist()
        self.stop = False

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(f"CmdMux: {self.in_cmd} + {self.in_stop} -> {self.out_cmd}")

    def on_cmd(self, msg: Twist):
        self.latest_cmd = msg

    def on_stop(self, msg: Bool):
        self.stop = bool(msg.data)

    def on_timer(self):
        out = Twist()
        if self.stop:
            out.linear.x = 0.0
            out.angular.z = 0.0
        else:
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
