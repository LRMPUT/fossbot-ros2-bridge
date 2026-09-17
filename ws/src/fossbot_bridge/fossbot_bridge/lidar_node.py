#!/usr/bin/env python3
"""Publishes /scan from the RPLIDAR A1M8 streamed by the robot-side agent.

Kept as its own node and its own TCP connection so a lidar stall can never
delay telemetry or motor commands, and so the lidar can be restarted without
dropping the rest of the bridge.
"""

import json
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

from fossbot_bridge.protocol import LIDAR_PORT

# Reliable for the same reason as the sensor topics in bridge_node: RViz's
# default LaserScan display is Reliable and would receive nothing otherwise.
SCAN_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=5)


class LidarBridge(Node):

    def __init__(self):
        super().__init__("fossbot_lidar")

        self.declare_parameter("robot_host", "fossbotrpi1.local")
        self.declare_parameter("frame_id", "lidar_scan_frame")
        self.declare_parameter("angle_bins", 360)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("angle_offset_deg", 0.0)
        self.declare_parameter("invert", False)
        self.declare_parameter("reconnect_period", 3.0)
        self.declare_parameter("read_timeout", 30.0)

        p = self.get_parameter
        self.robot_host = p("robot_host").value
        self.frame_id = p("frame_id").value
        self.bins = int(p("angle_bins").value)
        self.range_min = p("range_min").value
        self.range_max = p("range_max").value
        self.angle_offset = math.radians(p("angle_offset_deg").value)
        self.invert = p("invert").value
        self.reconnect_period = p("reconnect_period").value
        self.read_timeout = p("read_timeout").value

        self.pub = self.create_publisher(LaserScan, "/scan", SCAN_QOS)
        self.running = True
        self.scans = 0
        self.last_scan_t = None
        self.scan_period = None
        threading.Thread(target=self.stream_loop, daemon=True).start()
        self.get_logger().info(f"lidar bridge up, robot={self.robot_host}")

    def stream_loop(self):
        while self.running:
            try:
                self.get_logger().info(
                    f"connecting to lidar at {self.robot_host}:{LIDAR_PORT}")
                sock = socket.create_connection((self.robot_host, LIDAR_PORT),
                                                timeout=10.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # create_connection's timeout applies to every later read too.
                # At 10 s that turns a slow lidar spin-up into a reconnect
                # storm, and each reconnect leaves the device's serial state
                # dirtier than the last. Give streaming reads their own, much
                # longer budget -- long enough to mean "the link is dead",
                # not "the lidar is still spinning up".
                sock.settimeout(self.read_timeout)
                reader = sock.makefile("r")
                self.get_logger().info("lidar connected")
                for line in reader:
                    if not self.running:
                        break
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line)
                    except ValueError:
                        continue
                    self.publish_scan(payload)
                sock.close()
            except Exception as exc:
                self.get_logger().warn(f"lidar link: {exc}")
            if self.running:
                time.sleep(self.reconnect_period)

    def publish_scan(self, payload):
        pts = payload.get("pts", [])
        if not pts:
            return

        # The A1M8 returns points at irregular angles and drops beams with no
        # echo, so rasterise into fixed bins. Empty bins stay inf, which is what
        # Nav2 and slam_toolbox expect for "no return", not zero.
        ranges = [float("inf")] * self.bins
        step = 2.0 * math.pi / self.bins
        for angle_deg, dist_mm in pts:
            if dist_mm <= 0:
                continue
            d = dist_mm / 1000.0
            if d < self.range_min or d > self.range_max:
                continue
            a = math.radians(angle_deg)
            if self.invert:
                a = -a
            a += self.angle_offset
            idx = int((a % (2.0 * math.pi)) / step)
            if 0 <= idx < self.bins and d < ranges[idx]:
                ranges[idx] = d

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi - step
        scan.angle_increment = step
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        # Measure the rotation period rather than assuming it: the A1M8's spin
        # rate drifts with supply voltage (observed between 3.3 and 6.7 Hz on
        # this robot), and Nav2 / slam_toolbox use scan_time to motion-compensate.
        now = time.monotonic()
        if self.last_scan_t is not None:
            dt = now - self.last_scan_t
            if 0.05 < dt < 2.0:
                self.scan_period = (0.9 * self.scan_period + 0.1 * dt
                                    if self.scan_period else dt)
        self.last_scan_t = now
        scan.scan_time = float(self.scan_period or 0.15)
        scan.time_increment = scan.scan_time / self.bins
        scan.ranges = ranges
        self.pub.publish(scan)
        self.scans += 1

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
