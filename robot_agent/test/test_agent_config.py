"""Per-robot config: validated, applied to the agent's constants, optional."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from fossbot_bridge import protocol

AGENT_DIR = Path(__file__).resolve().parents[1]


def load_agent():
    spec = importlib.util.spec_from_file_location(
        "config_test_agent", AGENT_DIR / "fossbot_agent.py")
    agent = importlib.util.module_from_spec(spec)
    sys.modules["config_test_agent"] = agent
    with patch.dict(sys.modules, {"lgpio": Mock(), "spidev": Mock(),
                                 "pinmap": Mock(), "protocol": protocol}):
        spec.loader.exec_module(agent)
    return agent


def write_config(settings):
    f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump(settings, f)
    f.close()
    return f.name


class ApplyConfigTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(AGENT_DIR))
        self.agent = load_agent()

    def tearDown(self):
        sys.path.remove(str(AGENT_DIR))

    def test_generic_defaults_without_config(self):
        self.assertEqual(self.agent.apply_config("/nonexistent/agent.json"), {})
        self.assertFalse(self.agent.MOTOR_LEFT_INVERT)
        self.assertFalse(self.agent.MOTOR_RIGHT_INVERT)
        self.assertEqual(self.agent.LIDAR_DEVICE, "/dev/ttyUSB0")

    def test_overrides_reach_module_constants(self):
        path = write_config({"motor_left_invert": True, "motor_right_invert": True,
                             "motor_ff_left_forward": [0.1, 1.2],
                             "lidar_device": "/dev/ttyUSB1"})
        self.agent.apply_config(path)
        self.assertTrue(self.agent.MOTOR_LEFT_INVERT)
        self.assertTrue(self.agent.MOTOR_RIGHT_INVERT)
        self.assertEqual(self.agent.MOTOR_FF_LEFT_FORWARD, (0.1, 1.2))
        self.assertEqual(self.agent.LIDAR_DEVICE, "/dev/ttyUSB1")

    def test_example_config_is_valid(self):
        self.agent.apply_config(str(AGENT_DIR / "config.example.json"))

    def test_unknown_key_is_rejected(self):
        with self.assertRaises(ValueError):
            self.agent.apply_config(write_config({"motor_left_invertt": True}))

    def test_out_of_range_value_is_rejected(self):
        with self.assertRaises(ValueError):
            self.agent.apply_config(write_config({"duty_max": 1.5}))


if __name__ == "__main__":
    unittest.main()
