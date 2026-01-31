#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from std_msgs.msg import Bool
except Exception:
    Bool = None


class CmdMux(Node):
    """
    Pass-through mux with optional stop gate.

    Params:
      in_cmd: Twist input (default /cmd_vel_raw)
      out_cmd: Twist output (default /cmd_vel_safe)
      stop_topic: Bool topic. If empty or missing, stop gating disabled.
      publish_rate_hz: output publish rate
      cmd_timeout_s: if no cmd recently, output zeros
    """

    def __init__(self):
        super().__init__("cmd_mux")

        self.declare_parameter("in_cmd", "/cmd_vel_raw")
        self.declare_parameter("out_cmd", "/cmd_vel_safe")
        self.declare_parameter("stop_topic", "")          # optional; empty disables gating
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.6)

        self.in_cmd = str(self.get_parameter("in_cmd").value)
        self.out_cmd = str(self.get_parameter("out_cmd").value)
        self.stop_topic = str(self.get_parameter("stop_topic").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)

        self.pub = self.create_publisher(Twist, self.out_cmd, 10)
        self.sub_cmd = self.create_subscription(Twist, self.in_cmd, self.on_cmd, 10)

        self.stop_active = False
        self.sub_stop = None
        if self.stop_topic and Bool is not None:
            self.sub_stop = self.create_subscription(Bool, self.stop_topic, self.on_stop, 10)
            self.get_logger().info(f"CmdMux: gating enabled stop_topic={self.stop_topic}")
        else:
            self.get_logger().info("CmdMux: gating disabled (stop_topic empty)")

        self.last_cmd = Twist()
        self.last_cmd_time = time.time()

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(f"CmdMux: {self.in_cmd} -> {self.out_cmd} @ {self.rate_hz:.1f}Hz")

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def on_stop(self, msg):
        try:
            self.stop_active = bool(msg.data)
        except Exception:
            self.stop_active = False

    def on_timer(self):
        out = Twist()

        timed_out = (time.time() - self.last_cmd_time) > self.cmd_timeout_s

        if timed_out or self.stop_active:
            # publish zeros
            self.pub.publish(out)
            return

        # forward
        out.linear.x = float(self.last_cmd.linear.x)
        out.angular.z = float(self.last_cmd.angular.z)
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
