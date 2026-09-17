#!/usr/bin/env python3
"""Print the bearing of the nearest lidar return, in the robot's own frame.

Use this to verify the lidar's angular alignment without guessing. Put an
object about 30 cm directly in front of the robot and run:

    ros2 run fossbot_bridge scan_bearing

Directly ahead should print a bearing near 0 deg. Then:

    reads ~+90   -> angle_offset_deg is 90 too low
    reads ~-90   -> angle_offset_deg is 90 too high
    reads ~180   -> angle_offset_deg is 180 out
    bearing sign is mirrored (object on the left reads right) -> set invert: true

Adjust `angle_offset_deg` in config/bridge.yaml, or live with:

    ros2 param set /fossbot_lidar angle_offset_deg 90.0

Note the live param only affects scans published after it is set, and the node
reads it once at startup, so prefer editing the yaml and relaunching.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import tf2_ros

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


class ScanBearing(Node):

    def __init__(self):
        super().__init__("scan_bearing")
        self.declare_parameter("base_frame", "base_footprint")
        self.base_frame = self.get_parameter("base_frame").value
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, QOS)
        self.get_logger().info(
            "place an object directly in front of the robot; "
            "bearing should read ~0 deg")

    def on_scan(self, msg):
        best_i, best_r = None, float("inf")
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min < r < best_r:
                best_i, best_r = i, r
        if best_i is None:
            print("no valid returns")
            return

        angle_in_scan = msg.angle_min + best_i * msg.angle_increment

        # Rotate the bearing into the robot frame using the real TF, so this
        # reports what RViz actually draws rather than re-deriving the mounting.
        try:
            tf = self.buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception as exc:
            print(f"no TF {self.base_frame} <- {msg.header.frame_id}: {exc}")
            return

        bearing = math.degrees(
            math.atan2(math.sin(angle_in_scan + yaw),
                       math.cos(angle_in_scan + yaw)))
        side = ("AHEAD" if abs(bearing) < 25 else
                "LEFT" if 25 <= bearing < 155 else
                "BEHIND" if abs(bearing) >= 155 else "RIGHT")
        print(f"nearest {best_r:5.2f} m at {bearing:+7.1f} deg in "
              f"{self.base_frame}  -> {side}    "
              f"(raw scan angle {math.degrees(angle_in_scan):6.1f} deg, "
              f"frame yaw {math.degrees(yaw):+6.1f} deg)")


def main(args=None):
    rclpy.init(args=args)
    node = ScanBearing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
