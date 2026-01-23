#!/usr/bin/env python3
import math
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__('cmdvel_to_esp32')

        self.declare_parameter('port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation_m', 0.30)
        self.declare_parameter('max_linear_mps', 0.6)     # cmd=1.0 corresponds to this speed
        self.declare_parameter('timeout_sec', 0.5)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.wheel_sep = float(self.get_parameter('wheel_separation_m').value)
        self.max_lin = float(self.get_parameter('max_linear_mps').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)

        self.get_logger().info(f"Opening ESP32 serial: {self.port} @ {self.baud}")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.last_cmd_time = time.time()
        self.last_sent = None

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.timer = self.create_timer(0.05, self.watchdog)  # 20 Hz

    def send_line(self, line: str):
        if not line.endswith('\n'):
            line += '\n'
        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = time.time()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Convert v,w -> left/right wheel linear velocity
        v_l = v - w * (self.wheel_sep / 2.0)
        v_r = v + w * (self.wheel_sep / 2.0)

        # Normalize into [-1..1]
        left = clamp(v_l / self.max_lin, -1.0, 1.0)
        right = clamp(v_r / self.max_lin, -1.0, 1.0)

        line = f"D {left:.3f} {right:.3f}"

        # reduce spam (only send if changed a bit)
        if self.last_sent is None or line != self.last_sent:
            self.send_line(line)
            self.last_sent = line

    def watchdog(self):
        if (time.time() - self.last_cmd_time) > self.timeout:
            # Stop if commands stop arriving
            if self.last_sent != "S":
                self.send_line("S")
                self.last_sent = "S"

def main():
    rclpy.init()
    node = CmdVelToESP32()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
