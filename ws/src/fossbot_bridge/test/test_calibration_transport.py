"""Exercise real UDP/TCP against an in-process agent, never the robot."""

import json
import math
import select
import socket
import threading
import time
import unittest

from fossbot_bridge.protocol import (
    CMD_DIRECT, FLAG_MOTORS_ENABLED, pack_telemetry, unpack_command,
)
from fossbot_bridge.wheel_calibrate import RobotLink, summarize


class FakeAgent:
    def __init__(self, telemetry_port):
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp.bind(("127.0.0.1", 0))
        self.tcp.listen()
        self.destination = ("127.0.0.1", telemetry_port)
        self.enabled = False
        self.duty = 0.0
        self.echo = None
        self.counts = [0.0, 0.0]
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        last = time.monotonic()
        seq = 0
        while not self.stop.is_set():
            ready, _, _ = select.select([self.udp, self.tcp], [], [], .01)
            now = time.monotonic()
            dt, last = now - last, now
            if self.enabled:
                for i, gain in enumerate((1.0, 1.2)):
                    self.counts[i] += gain * self.duty * dt * 20 / (2 * math.pi * .03524)
            for sock in ready:
                if sock is self.udp:
                    command = unpack_command(sock.recvfrom(2048)[0])
                    if command is not None:
                        self.echo = command["seq"]
                        self.duty = command["a"] if command["type"] == CMD_DIRECT else 0
                else:
                    conn, _ = sock.accept()
                    with conn, conn.makefile("rb") as reader:
                        request = json.loads(reader.readline())
                        if request["op"] == "enable_motors":
                            self.enabled = request["enable"]
                            if not self.enabled:
                                self.duty = 0
                            reply = {"ok": True, "enabled": self.enabled}
                        else:
                            reply = {"ok": True, "motors_enabled": self.enabled,
                                     "target_mps": [0, 0]}
                        conn.sendall((json.dumps(reply) + "\n").encode())
            if self.echo is not None:
                seq += 1
                frame = pack_telemetry(
                    seq, self.echo, now, *(int(n) for n in self.counts),
                    [0] * 16, [0.] * 6, 25., float("nan"),
                    self.duty, self.duty, 4.95, 1., 0,
                    FLAG_MOTORS_ENABLED if self.enabled else 0)
                self.udp.sendto(frame, self.destination)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.udp.close()
        self.tcp.close()


class TransportTests(unittest.TestCase):
    def test_motor_response_and_confirmed_shutdown(self):
        # Port zero reserves a free receiver port without probing the LAN.
        link = RobotLink("127.0.0.1", telemetry_port=0)
        agent = FakeAgent(link.rx.getsockname()[1])
        link.command_port = agent.udp.getsockname()[1]
        link.service_port = agent.tcp.getsockname()[1]
        try:
            link.phase(0, .1, bootstrap=True)
            link.service("enable_motors", enable=True)
            link.phase(0, .1)
            frames = link.phase(.2, .8)
            result = summarize(frames, .03524, 20)
            self.assertGreater(result["right"]["ticks"], result["left"]["ticks"])
            self.assertAlmostEqual(result["left"]["speed_mps"], .2, delta=.025)
            self.assertEqual(link.stop_and_disable(), [])
            self.assertFalse(agent.enabled)
            self.assertEqual(agent.duty, 0)
        finally:
            link.close()
            agent.close()

    def test_refuses_port_owned_by_bridge(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            occupied.bind(("0.0.0.0", 0))
            with self.assertRaisesRegex(RuntimeError, "Telemetry port busy"):
                RobotLink("127.0.0.1", telemetry_port=occupied.getsockname()[1])


if __name__ == "__main__":
    unittest.main()
