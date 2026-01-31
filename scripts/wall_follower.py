#!/usr/bin/env python3
import math
import time
import random
from dataclasses import dataclass
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def is_valid(scan: LaserScan, r: float) -> bool:
    return math.isfinite(r) and (scan.range_min <= r <= scan.range_max)


def deg_to_scan_rad(scan: LaserScan, deg_scan_0_360: float) -> float:
    """
    scan rad angle corresponding to a given 0..360 deg in scan frame.

    If scan angle_min is negative (e.g. -pi..pi), we map 0..360 -> -180..180.
    If scan angle_min is >= 0 (e.g. 0..2pi), keep 0..360.
    """
    d = deg_scan_0_360 % 360.0
    if scan.angle_min < 0.0:
        if d > 180.0:
            d -= 360.0
    return math.radians(d)


def get_range_at_deg_robot(scan: LaserScan, deg_robot: float, scan_yaw_offset_deg: float) -> Optional[float]:
    """
    deg_robot: 0=forward, +CCW left
    scan_yaw_offset_deg: rotate robot angles into scan angles.
    """
    deg_scan = (deg_robot + scan_yaw_offset_deg) % 360.0
    ang = deg_to_scan_rad(scan, deg_scan)
    if scan.angle_increment == 0.0:
        return None
    idx = int(round((ang - scan.angle_min) / scan.angle_increment))
    if idx < 0 or idx >= len(scan.ranges):
        return None
    r = float(scan.ranges[idx])
    if not is_valid(scan, r):
        return None
    return r


def robust_min_in_sector(scan: LaserScan, center_deg_robot: float, half_deg: float, scan_yaw_offset_deg: float) -> Optional[float]:
    vals: List[float] = []
    d = center_deg_robot - half_deg
    while d <= center_deg_robot + half_deg:
        r = get_range_at_deg_robot(scan, d, scan_yaw_offset_deg)
        if r is not None:
            vals.append(r)
        d += 1.0
    if not vals:
        return None
    vals.sort()
    # 20th percentile -> avoids single-ray spikes
    k = max(0, int(0.2 * (len(vals) - 1)))
    return vals[k]


@dataclass
class LineModel:
    a: float
    b: float
    c: float
    inliers: int


def fit_line_ransac(points: List[Tuple[float, float]],
                    iters: int,
                    thresh: float,
                    min_inliers: int) -> Optional[LineModel]:
    """
    Line in ax + by + c = 0 with normalized sqrt(a^2+b^2)=1
    RANSAC by sampling 2 points.
    """
    if len(points) < 2:
        return None

    best: Optional[LineModel] = None

    for _ in range(max(1, iters)):
        p1 = random.choice(points)
        p2 = random.choice(points)
        if p1 == p2:
            continue

        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            continue

        # line through p1,p2: normal = (dy, -dx)
        a = dy
        b = -dx
        norm = math.hypot(a, b)
        if norm < 1e-9:
            continue
        a /= norm
        b /= norm
        c = -(a * x1 + b * y1)

        inl = 0
        for (x, y) in points:
            dist = abs(a * x + b * y + c)  # since normalized
            if dist <= thresh:
                inl += 1

        if best is None or inl > best.inliers:
            best = LineModel(a=a, b=b, c=c, inliers=inl)

    if best is None or best.inliers < min_inliers:
        return None

    return best


class WallFollower(Node):
    """
    Lidar-only wall follower using:
      - Sector point extraction
      - RANSAC line fit -> wall distance + wall heading
      - State machine to avoid dithering at corners
      - Smooth angular output (low-pass + slew limit)
    """

    def __init__(self):
        super().__init__("wall_follower")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")

        # Side
        self.declare_parameter("follow_side", "left")  # left/right
        self.declare_parameter("scan_yaw_offset_deg", -90.0)

        # Speed & limits
        self.declare_parameter("desired_dist", 0.35)
        self.declare_parameter("v_nominal", 0.14)
        self.declare_parameter("v_min", 0.06)
        self.declare_parameter("v_max", 0.18)
        self.declare_parameter("w_max", 1.0)

        # Front safety
        self.declare_parameter("front_sector_half_deg", 18.0)
        self.declare_parameter("front_stop_dist", 0.35)
        self.declare_parameter("front_slow_dist", 0.80)

        # Wall sector (robot frame degrees, 0 forward, + left)
        self.declare_parameter("side_sector_center_deg", 90.0)
        self.declare_parameter("side_sector_half_deg", 35.0)

        # RANSAC
        self.declare_parameter("ransac_iters", 60)
        self.declare_parameter("ransac_thresh", 0.03)
        self.declare_parameter("min_inliers", 35)

        # Controller gains
        self.declare_parameter("k_dist", 2.2)
        self.declare_parameter("k_head", 1.6)

        # Smoothing
        self.declare_parameter("w_slew_rate", 2.5)       # rad/s per second
        self.declare_parameter("w_lowpass_alpha", 0.35)  # 0..1 (higher = less smoothing)

        # State behavior
        self.declare_parameter("corner_hold_s", 0.55)    # commit time
        self.declare_parameter("search_turn_w", 0.35)    # gentle bias to find wall
        self.declare_parameter("publish_rate_hz", 20.0)

        # Debug
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_every_s", 0.6)

        # Read params
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.follow_side = str(self.get_parameter("follow_side").value).strip().lower()
        if self.follow_side not in ("left", "right"):
            self.follow_side = "left"

        self.side_sign = +1.0 if self.follow_side == "left" else -1.0
        self.scan_yaw_offset_deg = float(self.get_parameter("scan_yaw_offset_deg").value)

        self.desired_dist = float(self.get_parameter("desired_dist").value)
        self.v_nominal = float(self.get_parameter("v_nominal").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.front_sector_half_deg = float(self.get_parameter("front_sector_half_deg").value)
        self.front_stop_dist = float(self.get_parameter("front_stop_dist").value)
        self.front_slow_dist = float(self.get_parameter("front_slow_dist").value)

        self.side_sector_center_deg = float(self.get_parameter("side_sector_center_deg").value)
        self.side_sector_half_deg = float(self.get_parameter("side_sector_half_deg").value)

        self.ransac_iters = int(self.get_parameter("ransac_iters").value)
        self.ransac_thresh = float(self.get_parameter("ransac_thresh").value)
        self.min_inliers = int(self.get_parameter("min_inliers").value)

        self.k_dist = float(self.get_parameter("k_dist").value)
        self.k_head = float(self.get_parameter("k_head").value)

        self.w_slew_rate = float(self.get_parameter("w_slew_rate").value)
        self.w_lowpass_alpha = float(self.get_parameter("w_lowpass_alpha").value)

        self.corner_hold_s = float(self.get_parameter("corner_hold_s").value)
        self.search_turn_w = float(self.get_parameter("search_turn_w").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.debug_every_s = float(self.get_parameter("debug_every_s").value)
        self._last_dbg = 0.0

        # ROS
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # State
        self.have_scan = False
        self.latest_scan: Optional[LaserScan] = None

        self.mode = "SEARCH"  # SEARCH, FOLLOW, CORNER
        self.corner_until = 0.0
        self.wall_good_recent = False
        self.wall_last_good_t = 0.0

        # Output smoothing
        self.w_prev = 0.0
        self.v_prev = 0.0
        self.t_prev = time.time()

        self.latest_cmd = Twist()

        self.get_logger().info(
            f"WallFollower ready: side={self.follow_side} desired={self.desired_dist:.2f} "
            f"yaw_offset={self.scan_yaw_offset_deg:.1f} cmd={self.cmd_topic}"
        )

    def on_timer(self):
        if not self.have_scan:
            return
        self.pub.publish(self.latest_cmd)

    def on_scan(self, scan: LaserScan):
        self.have_scan = True
        self.latest_scan = scan

        now = time.time()
        dt = max(1e-3, now - self.t_prev)
        self.t_prev = now

        # ----- FRONT SAFETY -----
        front = robust_min_in_sector(scan, 0.0, self.front_sector_half_deg, self.scan_yaw_offset_deg)

        # ----- Extract side sector points (robot frame) -----
        # For right wall, center is -90 (or 270). Keep in robot frame:
        side_center = +abs(self.side_sector_center_deg) if self.follow_side == "left" else -abs(self.side_sector_center_deg)
        side_half = abs(self.side_sector_half_deg)

        pts: List[Tuple[float, float]] = []
        d = side_center - side_half
        while d <= side_center + side_half:
            r = get_range_at_deg_robot(scan, d, self.scan_yaw_offset_deg)
            if r is not None:
                ang = math.radians(d)
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                # Keep points that are plausibly "side-ish" and not behind too much
                if x > -0.10:
                    pts.append((x, y))
            d += 1.0

        model = fit_line_ransac(
            pts,
            iters=self.ransac_iters,
            thresh=self.ransac_thresh,
            min_inliers=self.min_inliers,
        )

        wall_ok = (model is not None)

        if wall_ok:
            self.wall_good_recent = True
            self.wall_last_good_t = now
        else:
            # keep "recent" true for a short time to avoid instant mode flip
            if (now - self.wall_last_good_t) > 0.4:
                self.wall_good_recent = False

        # ----- Compute distance + heading from line -----
        dist = None
        phi = 0.0
        inliers = 0

        if model is not None:
            a, b, c = model.a, model.b, model.c
            inliers = model.inliers

            # distance from robot origin to line (since normalized):
            dist0 = abs(c)

            # heading of wall: tangent t = (-b, a)
            tx, ty = (-b), (a)
            # choose direction such that tx points forward-ish
            if tx < 0.0:
                tx, ty = -tx, -ty
            phi = wrap_pi(math.atan2(ty, tx))  # angle of wall tangent relative to robot x axis

            dist = dist0

        # ----- MODE LOGIC -----
        # Trigger CORNER if front is close (commit turn away from wall)
        if front is not None and front < self.front_stop_dist:
            self.mode = "CORNER"
            self.corner_until = now + self.corner_hold_s

        if self.mode == "CORNER":
            if now < self.corner_until:
                v_cmd = 0.0
                w_cmd = -0.9 * self.side_sign  # turn away from wall side
                self.latest_cmd = self.smooth_cmd(v_cmd, w_cmd, dt)
                self.maybe_debug(now, mode="CORNER", dist=dist, phi=phi, front=front, inliers=inliers)
                return
            # after hold, try to follow if wall is back
            self.mode = "FOLLOW" if self.wall_good_recent else "SEARCH"

        if self.mode == "SEARCH":
            # move slowly forward with a gentle bias toward the wall side
            # until we get a stable wall fit.
            v_cmd = self.v_min
            w_cmd = (self.search_turn_w * self.side_sign)  # gentle toward wall side

            # slow down if something ahead
            if front is not None:
                if front < self.front_slow_dist:
                    t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                    v_cmd = clamp(v_cmd * clamp(t, 0.0, 1.0), 0.0, self.v_min)

            if self.wall_good_recent:
                self.mode = "FOLLOW"

            self.latest_cmd = self.smooth_cmd(v_cmd, w_cmd, dt)
            self.maybe_debug(now, mode="SEARCH", dist=dist, phi=phi, front=front, inliers=inliers)
            return

        # ----- FOLLOW MODE -----
        if dist is None:
            # Lost wall: go SEARCH
            self.mode = "SEARCH"
            v_cmd = self.v_min
            w_cmd = (self.search_turn_w * self.side_sign)
            self.latest_cmd = self.smooth_cmd(v_cmd, w_cmd, dt)
            self.maybe_debug(now, mode="FOLLOW->SEARCH", dist=None, phi=phi, front=front, inliers=inliers)
            return

        # Distance error
        e = dist - self.desired_dist

        # Core control:
        # - distance term: for left wall, positive e (too far) -> turn left (+w)
        # - for right wall, positive e -> turn right (-w) => side_sign handles that
        w_dist = self.side_sign * (self.k_dist * e)

        # Heading term: align robot to wall direction
        w_head = self.k_head * phi

        w_cmd = w_dist + w_head
        w_cmd = clamp(w_cmd, -self.w_max, self.w_max)

        # Speed scheduling: slow down with more turn
        turn_mag = min(1.0, abs(w_cmd) / max(1e-6, self.w_max))
        v_cmd = self.v_nominal * (1.0 - 0.55 * turn_mag)
        v_cmd = clamp(v_cmd, self.v_min, self.v_max)

        # Front slow
        if front is not None and front < self.front_slow_dist:
            if front < self.front_stop_dist:
                self.mode = "CORNER"
                self.corner_until = now + self.corner_hold_s
                v_cmd = 0.0
                w_cmd = -0.9 * self.side_sign
            else:
                t = (front - self.front_stop_dist) / max(1e-3, (self.front_slow_dist - self.front_stop_dist))
                v_cmd = clamp(v_cmd * clamp(t, 0.0, 1.0), 0.0, v_cmd)

        self.latest_cmd = self.smooth_cmd(v_cmd, w_cmd, dt)
        self.maybe_debug(now, mode="FOLLOW", dist=dist, phi=phi, front=front, inliers=inliers)

    def smooth_cmd(self, v_cmd: float, w_cmd: float, dt: float) -> Twist:
        # Slew limit angular velocity (prevents flip-flop)
        max_dw = self.w_slew_rate * dt
        dw = clamp(w_cmd - self.w_prev, -max_dw, +max_dw)
        w_limited = self.w_prev + dw

        # Low-pass filter angular
        a = clamp(self.w_lowpass_alpha, 0.0, 1.0)
        w_smooth = (1.0 - a) * self.w_prev + a * w_limited

        # Slight v smoothing too (helps inching)
        v_smooth = 0.6 * self.v_prev + 0.4 * v_cmd

        self.w_prev = w_smooth
        self.v_prev = v_smooth

        msg = Twist()
        msg.linear.x = float(v_smooth)
        msg.angular.z = float(w_smooth)
        return msg

    def maybe_debug(self, now: float, mode: str, dist: Optional[float], phi: float,
                    front: Optional[float], inliers: int):
        if not self.debug:
            return
        if (now - self._last_dbg) < self.debug_every_s:
            return
        self._last_dbg = now

        d_str = "None" if dist is None else f"{dist:.3f}"
        f_str = "None" if front is None else f"{front:.3f}"
        self.get_logger().info(
            f"{mode}: dist={d_str} phi={math.degrees(phi):+.1f}deg front={f_str} "
            f"inl={inliers} cmd v={self.latest_cmd.linear.x:.2f} w={self.latest_cmd.angular.z:+.2f}"
        )


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
