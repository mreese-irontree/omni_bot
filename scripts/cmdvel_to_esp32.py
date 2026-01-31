#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import serial  # pyserial
except Exception:
    serial = None


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def apply_deadband(x: float, db: float) -> float:
    return 0.0 if abs(x) < db else x


class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__("cmdvel_to_esp32")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.6)

        self.declare_parameter("v_to_cmd", 1.0)
        self.declare_parameter("w_to_cmd", 0.7)
        self.declare_parameter("deadband", 0.03)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)

        self.v_to_cmd = float(self.get_parameter("v_to_cmd").value)
        self.w_to_cmd = float(self.get_parameter("w_to_cmd").value)
        self.deadband = float(self.get_parameter("deadband").value)

        self.sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.last_cmd = Twist()
        self.last_cmd_time = time.time()

        self.ser = None
        self.last_open_attempt = 0.0

        self.open_serial()
        self.get_logger().info(f"CmdVelToESP32: cmd={self.cmd_topic} port={self.port} baud={self.baud}")

    def open_serial(self):
        if serial is None:
            self.get_logger().error("pyserial not available. Install: sudo apt install python3-serial")
            self.ser = None
            return

        now = time.time()
        if (now - self.last_open_attempt) < 1.0:
            return
        self.last_open_attempt = now

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def send_stop(self):
        if self.ser is None:
            return
        try:
            self.ser.write(b"S\n")
        except Exception:
            pass

    def on_timer(self):
        if self.ser is None:
            self.open_serial()
            return

        if (time.time() - self.last_cmd_time) > self.cmd_timeout_s:
            self.send_stop()
            return

        v = float(self.last_cmd.linear.x)
        w = float(self.last_cmd.angular.z)

        # scale and clamp
        v_cmd = clamp(v * self.v_to_cmd, -1.0, 1.0)
        w_cmd = clamp(w * self.w_to_cmd, -1.0, 1.0)

        # deadband to stop constant micro-turning
        v_cmd = apply_deadband(v_cmd, self.deadband)
        w_cmd = apply_deadband(w_cmd, self.deadband)

        left = clamp(v_cmd - w_cmd, -1.0, 1.0)
        right = clamp(v_cmd + w_cmd, -1.0, 1.0)

        line = f"D {left:.3f} {right:.3f}\n"
        try:
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial write failed ({self.port}): {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self):
        try:
            self.send_stop()
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelToESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
