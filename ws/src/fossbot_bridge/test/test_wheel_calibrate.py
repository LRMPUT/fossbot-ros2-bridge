"""Motor fitting and failure-path checks, with no robot or ROS installation."""

import contextlib
from collections import deque
import io
import math
import socket
import time
import unittest
from unittest.mock import Mock, patch

from fossbot_bridge import wheel_calibrate as calibration
from fossbot_bridge.protocol import (
    FLAG_ESTOP, FLAG_LOW_VOLTAGE, FLAG_MOTORS_ENABLED, pack_telemetry)


def point(duty, speed, ticks=30):
    return {"applied_duty": duty, "speed_mps": speed, "ticks": ticks}


class FitTests(unittest.TestCase):
    def test_higher_duty_requires_explicit_bounded_ceiling(self):
        parser = calibration.build_parser()
        for arguments in (["--duties", ".30", ".35", ".40"],
                          ["--max-duty", ".51"], ["--max-duty", "nan"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                calibration.validate_args(parser, parser.parse_args(arguments))
        args = parser.parse_args(["--max-duty", ".40", "--duties", ".30", ".35", ".40"])
        calibration.validate_args(parser, args)

    def test_recovers_separate_motor_curves(self):
        for static, gain in ((0.02, 1.1), (0.04, 1.4)):
            points = [point(d, gain * (d - static)) for d in (.06, .10, .15, .20)]
            result = calibration.fit_motor(points)
            self.assertTrue(result["valid"])
            self.assertAlmostEqual(result["static_duty"], static)
            self.assertAlmostEqual(result["duty_per_mps"], 1 / gain)

    def test_stalled_and_low_resolution_points_do_not_bias_fit(self):
        points = [point(.01, 0, 0), point(.03, .002, 1)]
        points += [point(d, 1.2 * (d - .02)) for d in (.08, .12, .20)]
        self.assertAlmostEqual(calibration.fit_motor(points)["static_duty"], .02)

    def test_rejects_insufficient_or_inconsistent_data(self):
        for points in ([point(.1, .1)],
                       [point(d, .1) for d in (.1, .15, .2)],
                       [point(d, v) for d, v in ((.08, .15), (.12, .04), (.2, .17))]):
            self.assertFalse(calibration.fit_motor(points)["valid"])

    def test_never_recommends_negative_static_duty(self):
        result = calibration.fit_motor([point(d, 1.5 * d + .005)
                                        for d in (.08, .12, .16, .20)])
        self.assertTrue(result["valid"])
        self.assertEqual(result["static_duty"], 0)

    def test_cumulative_counts_use_robot_time_and_reverse_magnitudes(self):
        frames = [dict(t_robot=1, enc_left=20, enc_right=30, duty_left=-.1,
                       duty_right=-.1, supply_v=4.9),
                  dict(t_robot=5, enc_left=0, enc_right=0, duty_left=-.1,
                       duty_right=-.1, supply_v=4.8)]
        result = calibration.summarize(frames, .03524, 20)
        self.assertAlmostEqual(result["left"]["speed_mps"], 2 * math.pi * .03524 / 4)
        self.assertEqual(result["left"]["ticks"], -20)
        self.assertEqual(result["min_supply_v"], 4.8)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.args = calibration.build_parser().parse_args([])
        self.report = {"complete": False, "points": []}
        self.link = Mock(owned=False)
        self.link.service.side_effect = lambda op, **kw: (
            {"motors_enabled": False, "estop": False, "target_mps": [0, 0]}
            if op == "status" else {"enabled": kw["enable"]})
        self.link.phase.return_value = [
            dict(t_robot=1, enc_left=0, enc_right=0, duty_left=.1, duty_right=.1, supply_v=4.9),
            dict(t_robot=5, enc_left=20, enc_right=22, duty_left=.1, duty_right=.1, supply_v=4.9)]
        self.link.stop_and_disable.return_value = []

    def run_with_mock(self):
        with patch.object(calibration, "RobotLink", return_value=self.link), \
                contextlib.redirect_stdout(io.StringIO()):
            calibration.run_sweep(self.args, self.report)

    def test_refuses_already_enabled_robot_without_taking_control(self):
        self.link.service.side_effect = None
        self.link.service.return_value = {"motors_enabled": True}
        with self.assertRaises(RuntimeError):
            self.run_with_mock()
        self.link.phase.assert_not_called()
        self.link.stop_and_disable.assert_not_called()
        self.link.close.assert_called_once()

    def test_stop_disable_on_interruption(self):
        self.link.phase.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.run_with_mock()
        self.link.stop_and_disable.assert_called_once()
        self.link.close.assert_called_once()
        self.assertFalse(self.report["complete"])

    def test_rejects_motion_during_baseline_and_cleans_up(self):
        with self.assertRaisesRegex(RuntimeError, "stationary baseline"):
            self.run_with_mock()
        self.link.stop_and_disable.assert_called_once()

    def test_sweep_stops_between_all_levels_and_directions(self):
        frames = self.link.phase.return_value
        baseline = [dict(f, enc_left=0, enc_right=0) for f in frames]
        self.link.phase.side_effect = [baseline, baseline, baseline] + [frames] * 30
        self.run_with_mock()
        commands = [c.args[0] for c in self.link.phase.call_args_list][3:]
        expected = []
        for sign in (1, -1):
            for duty in self.args.duties:
                expected.extend([0, sign * duty, sign * duty])
        self.assertEqual(commands, expected)
        self.assertTrue(self.report["complete"])
        self.link.stop_and_disable.assert_called_once()

    def test_dry_run_opens_no_connection(self):
        with patch.object(calibration, "RobotLink") as constructor, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calibration.main([]), 0)
        constructor.assert_not_called()


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.link = calibration.RobotLink.__new__(calibration.RobotLink)
        self.link.host = "127.0.0.1"
        self.link.rx = Mock()
        self.link.sent = deque([42])
        self.link.previous = None
        self.link.last_rx = time.monotonic()

    def frame(self, echo=42, flags=FLAG_MOTORS_ENABLED, supply=4.9):
        self.link.rx.recvfrom.return_value = (pack_telemetry(
            1, echo, 10., 0, 0, [0] * 16, [0.] * 6, 25., float("nan"),
            .1, .1, supply, 1., 0, flags), (self.link.host, 5005))

    def test_aborts_on_missing_telemetry(self):
        self.link.last_rx -= 1
        self.link.rx.recvfrom.side_effect = socket.timeout
        with self.assertRaisesRegex(RuntimeError, "300 ms"):
            self.link.receive(moving=True)

    def test_aborts_on_other_controller_low_voltage_and_disable(self):
        for kwargs in ({"echo": 43}, {"supply": 4.6}, {"supply": float("nan")}, {"flags": 0}):
            self.frame(**kwargs)
            with self.assertRaises(RuntimeError):
                self.link.receive(moving=True)

    def test_lowered_supply_guard_replaces_agent_low_voltage_flag(self):
        self.link.min_supply_v = 4.65
        self.frame(supply=4.7, flags=FLAG_MOTORS_ENABLED | FLAG_LOW_VOLTAGE)
        self.assertIsNotNone(self.link.receive(moving=True))
        for kwargs in ({"supply": 4.64}, {"flags": FLAG_MOTORS_ENABLED | FLAG_ESTOP}):
            self.link.previous = None
            self.frame(**kwargs)
            with self.assertRaises(RuntimeError):
                self.link.receive(moving=True)

    def test_min_supply_v_is_bounded(self):
        parser = calibration.build_parser()
        for value in ("4.6", "4.8", "nan"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                calibration.validate_args(parser, parser.parse_args(["--min-supply-v", value]))

    def test_tcp_disable_attempted_even_when_udp_stop_fails(self):
        self.link.send = Mock(side_effect=OSError("UDP unavailable"))
        self.link.service = Mock(side_effect=[{"enabled": False},
                                             {"motors_enabled": False, "target_mps": [0, 0]}])
        with patch.object(calibration.time, "sleep"):
            errors = self.link.stop_and_disable()
        self.assertTrue(errors)
        self.link.service.assert_any_call("enable_motors", enable=False)


if __name__ == "__main__":
    unittest.main()
