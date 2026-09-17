#!/usr/bin/env python3
"""Keyboard teleop for the FOSSBot, with the motor-enable handshake built in.

Unlike teleop_twist_keyboard this also calls /fossbot/enable_motors, because the
agent refuses to drive until motors are explicitly enabled, and it re-disables
them on exit so a closed terminal cannot leave the robot live.

  w/s  forward / reverse      a/d  turn left / right
  x    stop                   e    toggle motors enabled
  +/-  speed up / down        q    quit
"""

import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool

KEYS = """  w/s  forward / reverse      a/d  turn left / right
  x    stop                   e    toggle motors enabled
  +/-  speed up / down        q    quit"""


class Teleop(Node):

    def __init__(self):
        super().__init__("fossbot_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.enable_cli = self.create_client(SetBool, "/fossbot/enable_motors")
        self.linear = 0.12
        self.angular = 1.2
        self.enabled = False

    def bridge_present(self, timeout=5.0):
        return self.enable_cli.wait_for_service(timeout_sec=timeout)

    def set_enabled(self, value):
        if not self.enable_cli.wait_for_service(timeout_sec=2.0):
            print("\r  /fossbot/enable_motors unavailable - is the bridge running?")
            return False
        req = SetBool.Request()
        req.data = value
        self.enable_cli.call_async(req)
        self.enabled = value
        return True

    def send(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def status(self):
        state = "ENABLED" if self.enabled else "disabled (press e)"
        return f"\r  motors: {state:<20}  speed: {self.linear:.2f} m/s        "


def main(args=None):
    # Fail loudly rather than with a bare termios traceback. Running this
    # through `docker exec` without -it is the usual cause, and the error you
    # get otherwise ("termios.error: (25, 'Inappropriate ioctl for device')")
    # says nothing about the actual fix.
    if not sys.stdin.isatty():
        print("teleop needs an interactive terminal (a TTY) to read keys.\n"
              "\n"
              "If you are running it through docker, pass -it:\n"
              "    docker exec -it fossbot_bridge bash\n"
              "    source /ws/install/setup.bash\n"
              "    ros2 run fossbot_bridge teleop\n"
              "\n"
              "To drive without a TTY, publish /cmd_vel directly, e.g.:\n"
              "    ros2 service call /fossbot/enable_motors std_srvs/srv/SetBool "
              "\"{data: true}\"\n"
              "    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
              "\"{linear: {x: 0.12}}\"",
              file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = Teleop()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(__doc__.split("\n\n")[0])
    print()
    print(KEYS)
    print()

    if not node.bridge_present():
        print("WARNING: /fossbot/enable_motors did not appear within 5 s.\n"
              "         The bridge is probably not running. Start it with:\n"
              "           ros2 launch fossbot_bridge bringup.launch.py\n")
    print("Motors start DISABLED. Press 'e' before the robot will move.\n")

    settings = termios.tcgetattr(sys.stdin)
    warned = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(node.status(), end="", flush=True)
        while True:
            key = sys.stdin.read(1)
            moved = False
            if key == "q":
                break
            elif key == "w":
                node.send(node.linear, 0.0)
                moved = True
            elif key == "s":
                node.send(-node.linear, 0.0)
                moved = True
            elif key == "a":
                node.send(0.0, node.angular)
                moved = True
            elif key == "d":
                node.send(0.0, -node.angular)
                moved = True
            elif key == "x":
                node.send(0.0, 0.0)
            elif key == "e":
                node.set_enabled(not node.enabled)
                warned = False
            elif key in "+=":
                node.linear = min(node.linear * 1.2, 0.35)
            elif key == "-":
                node.linear = max(node.linear / 1.2, 0.02)

            # A movement key with the motors still disabled is the single most
            # common "teleop does nothing" report, so say so explicitly instead
            # of silently publishing into the void.
            if moved and not node.enabled and not warned:
                print("\r  >> motors are DISABLED - press 'e' to enable."
                      "  /cmd_vel is being published but ignored.      ")
                warned = True
            print(node.status(), end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.send(0.0, 0.0)
        if node.enabled:
            node.set_enabled(False)
        print("\nstopped, motors disabled")
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
