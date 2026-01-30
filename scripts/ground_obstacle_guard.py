#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class GroundObstacleGuard(Node):
    """
    Subscribes: /point_cloud (PointCloud2)
    Publishes:  /safety/stop (Bool)

    Looks for any points in a "low obstacle" box ahead of the robot in the camera frame.
    Tune z_min/z_max heavily depending on camera tilt and frame convention.
    """

    def __init__(self):
        super().__init__("ground_obstacle_guard")

        self.declare_parameter("points_topic", "/point_cloud")
        self.declare_parameter("stop_topic", "/safety/stop")

        # ROI box in the pointcloud frame
        self.declare_parameter("x_min", 0.10)
        self.declare_parameter("x_max", 0.80)
        self.declare_parameter("y_half", 0.25)
        self.declare_parameter("z_min", -0.20)
        self.declare_parameter("z_max", 0.10)

        # decision thresholds
        self.declare_parameter("stop_dist", 0.25)
        self.declare_parameter("hold_stop_ms", 400)

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("decimation", 8)

        self.points_topic = self.get_parameter("points_topic").value
        self.stop_topic = self.get_parameter("stop_topic").value

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_half = float(self.get_parameter("y_half").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.hold_stop_ms = int(self.get_parameter("hold_stop_ms").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.decimation = int(self.get_parameter("decimation").value)

        self.pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.sub = self.create_subscription(PointCloud2, self.points_topic, self.on_points, 10)

        self.stop_until = 0.0
        self.latest_stop = False
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(
            f"GroundObstacleGuard: points={self.points_topic} stop={self.stop_topic} "
            f"ROI x[{self.x_min},{self.x_max}] y±{self.y_half} z[{self.z_min},{self.z_max}] stop_dist={self.stop_dist}"
        )

    def on_timer(self):
        now = time.time()
        stop = self.latest_stop or (now < self.stop_until)
        self.pub.publish(Bool(data=bool(stop)))

    def on_points(self, cloud: PointCloud2):
        decim = max(1, self.decimation)
        i = 0
        found = False

        for (x, y, z) in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            i += 1
            if (i % decim) != 0:
                continue

            x = float(x); y = float(y); z = float(z)

            if x < self.x_min or x > self.x_max:
                continue
            if abs(y) > self.y_half:
                continue
            if z < self.z_min or z > self.z_max:
                continue

            d = math.sqrt(x*x + y*y + z*z)
            if d < self.stop_dist:
                found = True
                break

        if found:
            self.latest_stop = True
            self.stop_until = time.time() + (self.hold_stop_ms / 1000.0)
        else:
            self.latest_stop = False


def main():
    rclpy.init()
    node = GroundObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
