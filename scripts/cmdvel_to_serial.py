#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.22)      # meters (left-right distance)
        self.declare_parameter('max_lin', 0.35)         # m/s (your best guess)
        self.declare_parameter('max_ang', 2.5)          # rad/s (your best guess)
        self.declare_parameter('send_rate_hz', 20.0)    # keep-alive to avoid ESP timeout
        self.declare_parameter('stop_on_timeout_ms', 400)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.stop_on_timeout_ms = int(self.get_parameter('stop_on_timeout_ms').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.timer = self.create_timer(1.0 / self.send_rate_hz, self.on_timer)

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        age_ms = (now - self.last_cmd_time).nanoseconds / 1e6

        if age_ms > self.stop_on_timeout_ms:
            # keep sending stop so ESP never “runs away”
            line = "S\n"
            self.ser.write(line.encode('ascii'))
            return

        v = float(self.last_cmd.linear.x)
        w = float(self.last_cmd.angular.z)

        # Clamp to expected limits (prevents insane scaling)
        v = clamp(v, -self.max_lin, self.max_lin)
        w = clamp(w, -self.max_ang, self.max_ang)

        # Differential drive kinematics:
        # v_l = v - w*(L/2), v_r = v + w*(L/2)
        v_l = v - w * (self.wheel_base * 0.5)
        v_r = v + w * (self.wheel_base * 0.5)

        # Normalize to [-1..1] using max_lin as primary scale
        # (good enough to start; later you can calibrate)
        left = clamp(v_l / self.max_lin, -1.0, 1.0) if self.max_lin > 1e-6 else 0.0
        right = clamp(v_r / self.max_lin, -1.0, 1.0) if self.max_lin > 1e-6 else 0.0

        if self.invert_left:
            left *= -1.0
        if self.invert_right:
            right *= -1.0

        line = f"D {left:.3f} {right:.3f}\n"
        self.ser.write(line.encode('ascii'))


def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.ser.write(b"S\n")
        node.ser.close()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
