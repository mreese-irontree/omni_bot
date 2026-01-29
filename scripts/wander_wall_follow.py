#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def median(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WanderWallFollow(Node):
    """
    LiDAR-only wander with:
      - Front safety (stop/turn) with hysteresis
      - Right-wall-follow when right sector is valid
      - Otherwise steer toward open space using front-left vs front-right
      - Anti-stuck escape (reverse + turn) if blocked too long
      - Angular smoothing to reduce jitter

    IMPORTANT: This version assumes scan angles are 0..2pi (0..360deg),
    like your LD19 output (angle_min=0, angle_max=2pi).
    """

    def __init__(self):
        super().__init__('wander_wall_follow')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')

        # Speeds
        self.declare_parameter('forward_speed', 0.14)
        self.declare_parameter('turn_speed', 0.80)
        self.declare_parameter('reverse_speed', 0.10)

        # Front safety distances
        self.declare_parameter('front_stop_m', 0.55)
        self.declare_parameter('front_slow_m', 0.85)
        self.declare_parameter('front_clear_m', 1.10)  # hysteresis: must exceed to exit stop state

        # Wall follow (RIGHT)
        self.declare_parameter('wall_target_m', 0.55)
        self.declare_parameter('wall_kp', 1.1)
        self.declare_parameter('max_wall_correction', 0.60)

        # Open-space steering when no wall
        self.declare_parameter('open_k', 0.9)
        self.declare_parameter('open_deadband_m', 0.15)

        # Limits / smoothing
        self.declare_parameter('max_total_angular', 0.9)
        self.declare_parameter('ang_smooth_alpha', 0.25)  # 0=no smoothing, 1=heavy

        # Anti-stuck escape
        self.declare_parameter('stuck_front_m', 0.60)
        self.declare_parameter('stuck_time_sec', 1.2)
        self.declare_parameter('escape_turn_dir', -1.0)  # -1 turn right, +1 turn left
        self.declare_parameter('escape_turn_sec', 0.7)

        # Sector definitions in degrees for 0..360 scan
        # Front wraps across 360->0
        self.declare_parameter('front_sector_deg', 12.0)   # +/- around 0deg
        self.declare_parameter('front_r_deg', (320.0, 345.0))
        self.declare_parameter('front_l_deg', (15.0, 40.0))
        self.declare_parameter('right_deg', (250.0, 290.0))  # around 270
        self.declare_parameter('left_deg', (70.0, 110.0))    # around 90

        # Loop rate + debug
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('debug_hz', 1.0)

        # Read params
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.fwd = float(self.get_parameter('forward_speed').value)
        self.turn = float(self.get_parameter('turn_speed').value)
        self.rev = float(self.get_parameter('reverse_speed').value)

        self.front_stop = float(self.get_parameter('front_stop_m').value)
        self.front_slow = float(self.get_parameter('front_slow_m').value)
        self.front_clear = float(self.get_parameter('front_clear_m').value)

        self.wall_target = float(self.get_parameter('wall_target_m').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)
        self.max_wall_corr = float(self.get_parameter('max_wall_correction').value)

        self.open_k = float(self.get_parameter('open_k').value)
        self.open_db = float(self.get_parameter('open_deadband_m').value)

        self.max_ang = float(self.get_parameter('max_total_angular').value)
        self.alpha = float(self.get_parameter('ang_smooth_alpha').value)

        self.stuck_front = float(self.get_parameter('stuck_front_m').value)
        self.stuck_time = float(self.get_parameter('stuck_time_sec').value)
        self.escape_dir = float(self.get_parameter('escape_turn_dir').value)
        self.escape_turn_sec = float(self.get_parameter('escape_turn_sec').value)

        self.front_sector = float(self.get_parameter('front_sector_deg').value)
        self.front_r_rng = tuple(self.get_parameter('front_r_deg').value)
        self.front_l_rng = tuple(self.get_parameter('front_l_deg').value)
        self.right_rng = tuple(self.get_parameter('right_deg').value)
        self.left_rng = tuple(self.get_parameter('left_deg').value)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.debug_hz = float(self.get_parameter('debug_hz').value)

        # ROS IO
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        # State
        self.last_scan = None
        self._dbg_last = 0.0
        self._blocked_since = None
        self._escape_until = 0.0
        self._escape_phase = "none"
        self._prev_ang = 0.0
        self._in_stop_state = False

        self.get_logger().info(
            f"WanderWallFollow running scan={self.scan_topic} cmd_vel={self.cmd_vel_topic} "
            f"(0..360 sectors; front wraps)"
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    @staticmethod
    def _wrap_rad_0_2pi(a: float) -> float:
        # normalize to [0, 2pi)
        twopi = 2.0 * math.pi
        a = a % twopi
        return a

    def _sector_median_deg_0_360(self, scan: LaserScan, deg_min: float, deg_max: float):
        """
        Sector selection for scans where angles are 0..2pi.
        Handles wrap-around if deg_min > deg_max (e.g., 350..10).
        """
        # Normalize degrees into [0,360)
        deg_min = deg_min % 360.0
        deg_max = deg_max % 360.0

        rmin = max(0.02, float(scan.range_min))
        rmax = float(scan.range_max)

        a0 = math.radians(deg_min)
        a1 = math.radians(deg_max)

        vals = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if not (rmin < r < rmax):
                continue

            a = scan.angle_min + i * scan.angle_increment
            a = self._wrap_rad_0_2pi(a)

            if deg_min <= deg_max:
                # normal sector
                if a0 <= a <= a1:
                    vals.append(r)
            else:
                # wrap sector (e.g., 350..10) means [350..360) U [0..10]
                if (a >= a0) or (a <= a1):
                    vals.append(r)

        return median(vals)

    def _front_median(self, scan: LaserScan):
        # Front is +/- front_sector around 0deg => wrap sector: (360-front_sector)..(front_sector)
        d = self.front_sector
        return self._sector_median_deg_0_360(scan, 360.0 - d, d)

    def _smooth_ang(self, target):
        # Exponential smoothing
        self._prev_ang = (1.0 - self.alpha) * target + self.alpha * self._prev_ang
        return self._prev_ang

    def _dbg(self, front, fr, fl, right, left, cmd):
        if self.debug_hz <= 0:
            return
        now = time.time()
        if now - self._dbg_last >= (1.0 / self.debug_hz):
            self._dbg_last = now
            self.get_logger().info(
                f"front={front} fr={fr} fl={fl} right={right} left={left} "
                f"stop={self._in_stop_state} esc={self._escape_phase} cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})"
            )

    def tick(self):
        if self.last_scan is None:
            return

        scan = self.last_scan
        now = time.time()

        # Sectors (0..360)
        front = self._front_median(scan)
        front_r = self._sector_median_deg_0_360(scan, self.front_r_rng[0], self.front_r_rng[1])
        front_l = self._sector_median_deg_0_360(scan, self.front_l_rng[0], self.front_l_rng[1])
        right = self._sector_median_deg_0_360(scan, self.right_rng[0], self.right_rng[1])
        left = self._sector_median_deg_0_360(scan, self.left_rng[0], self.left_rng[1])

        cmd = Twist()

        # ---------- Escape if stuck ----------
        if now < self._escape_until:
            if self._escape_phase == "reverse":
                cmd.linear.x = -self.rev
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_dir * self.turn
            cmd.angular.z = clamp(cmd.angular.z, -self.max_ang, self.max_ang)
            self.pub.publish(cmd)
            self._dbg(front, front_r, front_l, right, left, cmd)
            return

        # ---------- Stuck detection ----------
        if front is not None and front < self.stuck_front:
            if self._blocked_since is None:
                self._blocked_since = now
            elif (now - self._blocked_since) > self.stuck_time:
                # reverse briefly then turn
                self._escape_phase = "reverse"
                rev_end = now + 0.4
                turn_end = rev_end + self.escape_turn_sec
                self._escape_until = turn_end

                # immediately send reverse once
                cmd.linear.x = -self.rev
                cmd.angular.z = 0.0
                self.pub.publish(cmd)

                # store turn timing
                self._turn_start = rev_end
                self._turn_end = turn_end
                self._dbg(front, front_r, front_l, right, left, cmd)
                return
        else:
            self._blocked_since = None

        # If we are in reverse+turn window, handle phase switch
        if hasattr(self, "_turn_end") and now < getattr(self, "_turn_end", 0.0):
            if now < self._turn_start:
                self._escape_phase = "reverse"
                cmd.linear.x = -self.rev
                cmd.angular.z = 0.0
            else:
                self._escape_phase = "turn"
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_dir * self.turn
            cmd.angular.z = clamp(cmd.angular.z, -self.max_ang, self.max_ang)
            self.pub.publish(cmd)
            self._dbg(front, front_r, front_l, right, left, cmd)
            return
        else:
            self._escape_phase = "none"

        # ---------- Normal driving ----------
        cmd.linear.x = self.fwd
        ang = 0.0

        # Stop-state hysteresis
        if front is not None:
            if (not self._in_stop_state) and front < self.front_stop:
                self._in_stop_state = True
            elif self._in_stop_state and front > self.front_clear:
                self._in_stop_state = False

        # If stopped: turn away from closer side using front-left/front-right
        if self._in_stop_state:
            cmd.linear.x = 0.0
            if front_r is not None and front_l is not None:
                ang = self.turn if front_r < front_l else -self.turn
            elif front_r is not None:
                ang = self.turn
            else:
                ang = -self.turn
            cmd.angular.z = clamp(ang, -self.max_ang, self.max_ang)
            cmd.angular.z = self._smooth_ang(cmd.angular.z)
            self.pub.publish(cmd)
            self._dbg(front, front_r, front_l, right, left, cmd)
            return

        # Slow down when approaching something
        if front is not None and front < self.front_slow:
            t = (front - self.front_stop) / max(1e-6, (self.front_slow - self.front_stop))
            t = clamp(t, 0.0, 1.0)
            cmd.linear.x *= t

        # Right wall follow if available
        if right is not None:
            err = (self.wall_target - right)  # +err => too far => turn right
            ang += clamp(self.wall_kp * err, -self.max_wall_corr, self.max_wall_corr)
        else:
            # No right wall: steer toward open space (front-left vs front-right)
            if front_r is not None and front_l is not None:
                diff = front_l - front_r  # + => left more open
                if abs(diff) > self.open_db:
                    ang += clamp(self.open_k * diff, -0.6, 0.6)
            elif front_l is not None and front_r is None:
                ang += 0.25
            elif front_r is not None and front_l is None:
                ang += -0.25

        cmd.angular.z = clamp(ang, -self.max_ang, self.max_ang)
        cmd.angular.z = self._smooth_ang(cmd.angular.z)

        self.pub.publish(cmd)
        self._dbg(front, front_r, front_l, right, left, cmd)


def main():
    rclpy.init()
    node = WanderWallFollow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
