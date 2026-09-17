#!/usr/bin/env python3
"""Verify the IMU gyro's sign against a commanded rotation.

Needed before enabling GYRO_HEADING_KP in the agent: closing a heading loop
with the wrong sign turns it into positive feedback.

Commands a slow LEFT (counter-clockwise, positive angular.z in ROS) rotation and
reports the measured yaw sign. Put the robot on the floor with room to turn.
"""

import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)


def main(args=None):
    rclpy.init(args=args)
    n = Node("gyro_sign_check")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    gz = []
    n.create_subscription(Imu, "/imu/data_raw",
                          lambda m: gz.append(m.angular_velocity.z), QOS)
    en = n.create_client(SetBool, "/fossbot/enable_motors")
    if not en.wait_for_service(timeout_sec=5.0):
        print("bridge not running", file=sys.stderr)
        return 1

    def enable(v):
        r = SetBool.Request()
        r.data = v
        f = en.call_async(r)
        rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)

    print("measuring gyro bias (keep the robot still) ...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not gz:
        print("no IMU data on /imu/data_raw", file=sys.stderr)
        return 1
    bias = statistics.mean(gz)
    print(f"  bias {bias:+.4f} rad/s")

    enable(True)
    gz.clear()
    print("commanding a LEFT turn (angular.z = +1.2) for 3 s ...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        m = Twist()
        m.angular.z = 1.2
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.02)
    pub.publish(Twist())
    t1 = time.time()
    while time.time() - t1 < 1.5:
        rclpy.spin_once(n, timeout_sec=0.05)
    enable(False)

    vals = [v - bias for v in gz]
    if len(vals) < 20:
        print("not enough samples -- did the robot reset mid-test?",
              file=sys.stderr)
        return 1
    mean = statistics.mean(vals)
    print(f"  measured yaw rate: {mean:+.3f} rad/s over {len(vals)} samples")
    if abs(mean) < 0.15:
        print("  INCONCLUSIVE: the robot barely rotated. Check that it actually "
              "turned; if it did not move, the duty was too low to overcome "
              "stiction, or the robot reset.")
        return 1
    if mean > 0:
        print("  gyro is CONVENTIONAL (+z = counter-clockwise). GYRO_SIGN = 1.0")
    else:
        print("  gyro is INVERTED relative to ROS. Set GYRO_SIGN = -1.0")
    print("  then set GYRO_HEADING_KP to about 0.25 in fossbot_agent.py "
          "and redeploy with ./restart_agent.sh")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
