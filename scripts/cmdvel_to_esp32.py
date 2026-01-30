#!/usr/bin/env python3
import time
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelToESP32(Node):
    """
    Subscribes to geometry_msgs/Twist and sends ESP32 lines:
      D <left_cmd> <right_cmd>
    where left_cmd/right_cmd are floats in [-1..1].

    Key params to fix sign / mixing issues:
      invert_w (bool): flip angular.z sign before mixing (default True)
      swap_lr (bool): swap left/right after mixing
      mix_mode (str): "diff" uses left=v-w, right=v+w (common)
                      "diff_inv" uses left=v+w, right=v-w (if your robot wiring is opposite)
    """

    def __init__(self):
        super().__init__("cmdvel_to_esp32")

        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.5)

        # Scale Twist units -> unitless motor command
        self.declare_parameter("v_to_cmd", 1.0)
        self.declare_parameter("w_to_cmd", 0.6)

        # Safety + stability
        self.declare_parameter("max_cmd", 0.65)          # cap magnitude sent to ESP (prevents 180° snaps)
        self.declare_parameter("deadband", 0.04)         # ignore tiny noise
        self.declare_parameter("slew_rate", 2.5)         # cmd units per second (limits step changes)

        # Sign/mapping controls (most likely fix is invert_w=True)
        self.declare_parameter("invert_w", True)
        self.declare_parameter("swap_lr", False)
        self.declare_parameter("mix_mode", "diff")       # "diff" or "diff_inv"

        # Debug
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_every_s", 1.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)

        self.v_to_cmd = float(self.get_parameter("v_to_cmd").value)
        self.w_to_cmd = float(self.get_parameter("w_to_cmd").value)

        self.max_cmd = float(self.get_parameter("max_cmd").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.slew_rate = float(self.get_parameter("slew_rate").value)

        self.invert_w = bool(self.get_parameter("invert_w").value)
        self.swap_lr = bool(self.get_parameter("swap_lr").value)
        self.mix_mode = str(self.get_parameter("mix_mode").value).strip()

        self.debug = bool(self.get_parameter("debug").value)
        self.debug_every_s = float(self.get_parameter("debug_every_s").value)
        self._last_dbg_t = 0.0

        self._last_msg_time = time.time()
        self._latest_twist = Twist()
        self._lock = threading.Lock()

        # Output command state for slew limiting
        self._out_left = 0.0
        self._out_right = 0.0
        self._last_send_t = time.time()

        # ROS
        self.sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / max(1e-6, self.rate_hz), self.on_timer)

        # Serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")
        self.get_logger().info(
            f"CmdVelToESP32: cmd={self.cmd_topic} invert_w={self.invert_w} swap_lr={self.swap_lr} mix_mode={self.mix_mode} "
            f"max_cmd={self.max_cmd} deadband={self.deadband} slew_rate={self.slew_rate}"
        )

    def on_cmd(self, msg: Twist):
        with self._lock:
            self._latest_twist = msg
            self._last_msg_time = time.time()

    def apply_deadband(self, x: float) -> float:
        return 0.0 if abs(x) < self.deadband else x

    def slew(self, current: float, target: float, dt: float) -> float:
        max_step = self.slew_rate * dt
        return clamp(target, current - max_step, current + max_step)

    def mix(self, v_cmd: float, w_cmd: float):
        # Base mixing
        if self.mix_mode == "diff":
            left = v_cmd - w_cmd
            right = v_cmd + w_cmd
        elif self.mix_mode == "diff_inv":
            left = v_cmd + w_cmd
            right = v_cmd - w_cmd
        else:
            # fallback
            left = v_cmd - w_cmd
            right = v_cmd + w_cmd

        if self.swap_lr:
            left, right = right, left

        # Normalize if either exceeds 1
        m = max(1.0, abs(left), abs(right))
        left /= m
        right /= m

        return left, right

    def send_drive(self, left: float, right: float):
        line = f"D {left:.3f} {right:.3f}\n"
        try:
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def send_stop(self):
        try:
            self.ser.write(b"S\n")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def on_timer(self):
        now = time.time()
        dt = max(1e-3, now - self._last_send_t)
        self._last_send_t = now

        # Timeout safety
        if (now - self._last_msg_time) > self.cmd_timeout_s:
            self._out_left = self.slew(self._out_left, 0.0, dt)
            self._out_right = self.slew(self._out_right, 0.0, dt)
            if abs(self._out_left) < 1e-3 and abs(self._out_right) < 1e-3:
                self.send_stop()
            else:
                self.send_drive(self._out_left, self._out_right)
            return

        with self._lock:
            tw = self._latest_twist

        # Convert Twist -> unitless
        v = float(tw.linear.x) * self.v_to_cmd
        w = float(tw.angular.z) * self.w_to_cmd

        # Most common “it turns the wrong way” fix:
        if self.invert_w:
            w = -w

        v = self.apply_deadband(v)
        w = self.apply_deadband(w)

        # Mix
        left_t, right_t = self.mix(v, w)

        # Clamp overall command to avoid instant pivots
        left_t = clamp(left_t, -self.max_cmd, self.max_cmd)
        right_t = clamp(right_t, -self.max_cmd, self.max_cmd)

        # Slew limit
        self._out_left = self.slew(self._out_left, left_t, dt)
        self._out_right = self.slew(self._out_right, right_t, dt)

        self.send_drive(self._out_left, self._out_right)

        if self.debug and (now - self._last_dbg_t) > self.debug_every_s:
            self._last_dbg_t = now
            self.get_logger().info(
                f"twist v={tw.linear.x:+.2f} w={tw.angular.z:+.2f} -> "
                f"v_cmd={v:+.2f} w_cmd={w:+.2f} -> "
                f"LR={self._out_left:+.2f},{self._out_right:+.2f}"
            )


def main():
    rclpy.init()
    node = CmdVelToESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.send_stop()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
