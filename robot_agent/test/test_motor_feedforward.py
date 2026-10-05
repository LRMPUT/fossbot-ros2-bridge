"""Exercise the actual agent controller with hardware imports stubbed out."""

from collections import deque
import importlib.util
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

from fossbot_bridge import protocol


def load_agent():
    path = Path(__file__).resolve().parents[1] / "fossbot_agent.py"
    spec = importlib.util.spec_from_file_location("calibration_test_agent", path)
    agent = importlib.util.module_from_spec(spec)
    original_path = list(sys.path)
    try:
        with patch.dict(sys.modules, {"lgpio": Mock(), "spidev": Mock(),
                                     "pinmap": Mock(), "protocol": protocol}):
            spec.loader.exec_module(agent)
    finally:
        sys.path[:] = original_path
    return agent


class FeedforwardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.agent = load_agent()

    def test_default_response_matches_previous_controller(self):
        a = self.agent
        for speed in (-.2, -.08, 0, .08, .2):
            for side in ("LEFT", "RIGHT"):
                actual = a.motor_feedforward(speed, getattr(a, f"MOTOR_FF_{side}_FORWARD"),
                                             getattr(a, f"MOTOR_FF_{side}_REVERSE"))
                self.assertAlmostEqual(actual, speed / a.MAX_LINEAR)

    def test_directional_friction_and_zero_target(self):
        f = self.agent.motor_feedforward
        self.assertEqual(f(0, (.1, 2), (.2, 3)), 0)
        self.assertAlmostEqual(f(.1, (.1, 2), (.2, 3)), .3)
        self.assertAlmostEqual(f(-.1, (.1, 2), (.2, 3)), -.5)

    def test_independent_models_reach_closed_loop_motor_output(self):
        a = self.agent
        hw = a.Hardware.__new__(a.Hardware)
        hw.lock = threading.Lock()
        hw.closed_loop = hw.motors_enabled = True
        hw.estop = False
        hw.target_l = hw.target_r = .1
        hw.des_l = hw.des_r = hw.corr_l = hw.corr_r = 0
        hw.enc_left = hw.enc_right = hw.ref_enc_l = hw.ref_enc_r = 0
        hw.set_motors = Mock()
        with patch.object(a, "MOTOR_FF_LEFT_FORWARD", (.02, 1)), \
                patch.object(a, "MOTOR_FF_RIGHT_FORWARD", (.03, 2)):
            hw.velocity_step(.001)  # Sub-tick position error is in the deadband.
        left, right = hw.set_motors.call_args.args
        self.assertAlmostEqual(left, .12)
        self.assertAlmostEqual(right, .23)

    def test_raw_geometry_is_unchanged_by_motor_tuning(self):
        a = self.agent
        hw = a.Hardware.__new__(a.Hardware)
        hw._hist = deque([(1.0, 0, 0)])
        hw.measure_speeds(1.2, 4, 4)
        self.assertAlmostEqual(hw.meas_l, hw.meas_r)


if __name__ == "__main__":
    unittest.main()
