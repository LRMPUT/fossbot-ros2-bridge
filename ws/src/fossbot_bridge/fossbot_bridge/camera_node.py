#!/usr/bin/env python3
"""Publishes the FOSSBot's camera from the MJPEG stream the agent relays.

The robot sends JPEG because the alternative is untenable over wifi: 640x480
RGB8 raw is 921 kB a frame, 110 Mbit/s at 15 fps. JPEG at quality 80 is roughly
40 kB, about 5 Mbit/s, and every frame is independent so a dropped one costs
exactly one frame.

Publishes:
  /camera/image_raw/compressed   sensor_msgs/CompressedImage  (always)
  /camera/camera_info            sensor_msgs/CameraInfo       (always)
  /camera/image_raw              sensor_msgs/Image            (publish_raw)

/camera/image_raw is decoded here rather than on the robot so the wire stays
compressed. It matches the topic the Gazebo sim publishes, so lab code written
against the simulator works unchanged; turn it off with publish_raw:=false if
nothing needs raw and you want the CPU back.
"""

import math
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from fossbot_bridge.protocol import CAMERA_PORT, CAMERA_HDR_SIZE, unpack_camera_header

# Compressed frames and CameraInfo are small (tens of kB and a few hundred
# bytes), so they go Reliable like everything else in this bridge -- RViz's
# displays default to Reliable and receive nothing from a best-effort publisher.
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=2)

# Raw frames are a different animal: 640x480 bgr8 is 921 kB, and pushing that
# Reliable at 15 fps measurably starves the transport -- half the frames were
# being dropped in testing because the publisher history wrapped before the
# subscriber drained it. Best effort with depth 1 is the conventional choice
# for raw images and behaves correctly: the newest frame wins, stale ones are
# discarded rather than queued. Set RViz's Image display to Best Effort to see
# this topic, or point it at /camera/image_raw/compressed instead.
RAW_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class CameraBridge(Node):

    def __init__(self):
        super().__init__("fossbot_camera")

        self.declare_parameter("robot_host", "fossbotrpi1.local")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("horizontal_fov_deg", 66.0)
        self.declare_parameter("reconnect_period", 3.0)
        self.declare_parameter("read_timeout", 20.0)

        p = self.get_parameter
        self.robot_host = p("robot_host").value
        self.frame_id = p("frame_id").value
        self.publish_raw = p("publish_raw").value
        self.hfov = math.radians(p("horizontal_fov_deg").value)
        self.reconnect_period = p("reconnect_period").value
        self.read_timeout = p("read_timeout").value

        self.pub_compressed = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", IMAGE_QOS)
        self.pub_info = self.create_publisher(
            CameraInfo, "/camera/camera_info", IMAGE_QOS)
        self.pub_raw = None
        self.bridge = None
        self.cv2 = None
        if self.publish_raw:
            # Degrade to compressed-only rather than refusing to start: the
            # compressed topic is the useful one, and a leaner image without
            # OpenCV should still get pictures.
            try:
                import cv2
                from cv_bridge import CvBridge
                self.cv2 = cv2
                self.bridge = CvBridge()
                self.pub_raw = self.create_publisher(Image, "/camera/image_raw",
                                                     RAW_QOS)
            except ImportError as exc:
                self.publish_raw = False
                self.get_logger().warn(
                    f"publish_raw requested but unavailable ({exc}); "
                    f"publishing /camera/image_raw/compressed only")

        self.frames = 0
        self.running = True
        threading.Thread(target=self.stream_loop, daemon=True).start()
        self.get_logger().info(f"camera bridge up, robot={self.robot_host}, "
                               f"publish_raw={self.publish_raw}")

    # --- link ------------------------------------------------------------
    def stream_loop(self):
        while self.running:
            sock = None
            try:
                self.get_logger().info(
                    f"connecting to camera at {self.robot_host}:{CAMERA_PORT}")
                sock = socket.create_connection((self.robot_host, CAMERA_PORT),
                                                timeout=10.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # create_connection's timeout also governs every later read; at
                # 10 s a slow camera start would trigger a reconnect loop.
                sock.settimeout(self.read_timeout)
                self.get_logger().info("camera connected")
                self.read_frames(sock)
            except Exception as exc:
                self.get_logger().warn(f"camera link: {exc}")
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            if self.running:
                time.sleep(self.reconnect_period)

    @staticmethod
    def recv_exact(sock, n):
        """Read exactly n bytes, or return None if the peer closed."""
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def read_frames(self, sock):
        while self.running:
            header = self.recv_exact(sock, CAMERA_HDR_SIZE)
            if header is None:
                raise OSError("camera stream closed")
            meta = unpack_camera_header(header)
            if meta is None:
                # Desynchronised: the only safe recovery is a fresh connection,
                # since we cannot tell where the next header starts.
                raise OSError("bad camera frame header")
            if not (0 < meta["jpeg_len"] <= 8 * 1024 * 1024):
                raise OSError(f"implausible jpeg length {meta['jpeg_len']}")
            payload = self.recv_exact(sock, meta["jpeg_len"])
            if payload is None:
                raise OSError("camera stream closed mid-frame")
            self.publish(meta, payload)

    # --- publishing ------------------------------------------------------
    def publish(self, meta, jpeg):
        stamp = self.get_clock().now().to_msg()

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub_compressed.publish(msg)

        self.pub_info.publish(
            self.make_camera_info(meta["width"], meta["height"], stamp))

        if self.pub_raw is not None:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            img = self.cv2.imdecode(arr, self.cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn("dropped an undecodable JPEG frame")
                return
            raw = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            raw.header.stamp = stamp
            raw.header.frame_id = self.frame_id
            self.pub_raw.publish(raw)

        self.frames += 1

    def make_camera_info(self, width, height, stamp):
        """A nominal pinhole model from the lens FOV.

        This is NOT a calibration: distortion is assumed zero and the principal
        point assumed centred. It is enough for RViz and for anything that just
        wants image geometry, and wrong for photogrammetry or visual odometry.
        Run camera_calibration against a checkerboard and replace K and D for
        anything that depends on metric accuracy.
        """
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        fx = (width / 2.0) / math.tan(self.hfov / 2.0)
        fy = fx                      # square pixels
        cx, cy = width / 2.0, height / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
