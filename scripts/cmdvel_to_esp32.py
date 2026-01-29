#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def sign(x: float) -> float:
    return 1.0 if x >= 0.0 else -1.0


class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__('cmdvel_to_esp32')

        # Topics / serial
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        # Robot geometry / scaling
        self.declare_parameter('wheel_separation_m', 0.30)
        self.declare_parameter('max_linear_mps', 0.6)     # cmd=1.0 corresponds to this speed
        self.declare_parameter('timeout_sec', 0.5)

        # “Breakaway” behavior
        self.declare_parameter('deadband', 0.05)          # ignore tiny commands
        self.declare_parameter('min_output', 0.50)        # your drivetrain needs ~50% to move
        self.declare_parameter('start_boost', 0.65)       # brief kick to overcome stiction
        self.declare_parameter('start_boost_ms', 200)     # boost duration (ms)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.wheel_sep = float(self.get_parameter('wheel_separation_m').value)
        self.max_lin = float(self.get_parameter('max_linear_mps').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)

        self.deadband = float(self.get_parameter('deadband').value)
        self.min_out = float(self.get_parameter('min_output').value)
        self.boost = float(self.get_parameter('start_boost').value)
        self.boost_ms = int(self.get_parameter('start_boost_ms').value)

        self.ser = None
        self._open_serial()

        self.last_cmd_time = time.time()
        self.last_sent = None

        # Track whether we are currently “stopped”
        self.was_stopped = True
        self.boost_until = 0.0

        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.timer = self.create_timer(0.05, self.watchdog)  # 20 Hz

        self.get_logger().info(
            f"Bridge ready. cmd_vel={self.cmd_vel_topic} port={self.port} "
            f"min_output={self.min_out} start_boost={self.boost} boost_ms={self.boost_ms}"
        )

    def _open_serial(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Opening ESP32 serial: {self.port} @ {self.baud}")
                self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                time.sleep(0.2)
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
                return
            except Exception as e:
                self.get_logger().warn(f"Serial open failed ({e}). Retrying in 1s...")
                time.sleep(1.0)

    def send_line(self, line: str):
        if not line.endswith('\n'):
            line += '\n'
        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}. Re-opening serial...")
            try:
                self.ser.close()
            except Exception:
                pass
            self._open_serial()

    def _apply_deadband_and_min(self, x: float) -> float:
        # deadband
        if abs(x) < self.deadband:
            return 0.0

        # min output (so 0.10 becomes 0.50, etc.)
        ax = abs(x)
        if ax < self.min_out:
            return sign(x) * self.min_out
        return clamp(x, -1.0, 1.0)

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = time.time()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # v,w -> left/right wheel linear velocity
        v_l = v - w * (self.wheel_sep / 2.0)
        v_r = v + w * (self.wheel_sep / 2.0)

        # Normalize into [-1..1]
        left = clamp(v_l / self.max_lin, -1.0, 1.0)
        right = clamp(v_r / self.max_lin, -1.0, 1.0)

        # If command is essentially stop:
        if abs(left) < self.deadband and abs(right) < self.deadband:
            self.was_stopped = True
            self.boost_until = 0.0
            self._send_drive(0.0, 0.0)
            return

        now = time.time()

        # If we were stopped and are now trying to move -> start boost window
        if self.was_stopped:
            self.was_stopped = False
            self.boost_until = now + (self.boost_ms / 1000.0)

        # During boost window: kick in the direction requested
        if now < self.boost_until:
            left_out = sign(left) * self.boost if abs(left) >= self.deadband else 0.0
            right_out = sign(right) * self.boost if abs(right) >= self.deadband else 0.0
        else:
            # Normal shaping: deadband + min output
            left_out = self._apply_deadband_and_min(left)
            right_out = self._apply_deadband_and_min(right)

        self._send_drive(left_out, right_out)

    def _send_drive(self, left: float, right: float):
        line = f"D {left:.3f} {right:.3f}"
        if self.last_sent is None or line != self.last_sent:
            self.send_line(line)
            self.last_sent = line

    def watchdog(self):
        if (time.time() - self.last_cmd_time) > self.timeout:
            if self.last_sent != "S":
                self.send_line("S")
                self.last_sent = "S"
            self.was_stopped = True
            self.boost_until = 0.0


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
