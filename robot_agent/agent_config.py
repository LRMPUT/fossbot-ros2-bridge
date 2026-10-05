"""Validate per-robot JSON settings before opening any hardware devices."""

import json
import math


BOOLEANS = {"motor_left_invert", "motor_right_invert"}
INTEGERS = {"gpio_chip": (0, 64), "encoder_ticks_per_rev": (1, 10000),
            "camera_width": (1, 4096), "camera_height": (1, 4096),
            "camera_fps": (1, 120), "camera_quality": (1, 100)}
NUMBERS = {"wheel_radius": (0.001, 1), "wheel_track": (0.01, 2),
           "duty_max": (0.01, 0.85), "duty_slew_per_s": (0.01, 10),
           "pos_kp": (0, 100), "pos_deadband_ticks": (0, 5),
           "pos_corr_alpha": (0, 1), "pos_err_clamp": (0.001, 1),
           "vel_window_s": (0.01, 5), "gyro_heading_kp": (0, 1),
           "gyro_bias_alpha": (0, 1), "supply_warn_v": (1, 20),
           "supply_critical_v": (1, 20), "supply_poll_s": (0.1, 10)}
PATHS = {"camera_prefix", "lidar_device"}
MODELS = {f"motor_ff_{side}_{direction}" for side in ("left", "right")
          for direction in ("forward", "reverse")}


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def load_config(path):
    """Return validated constant overrides; reject unknown keys and bad types."""
    with open(path) as source:
        settings = json.load(source)
    if not isinstance(settings, dict):
        raise ValueError("Agent config must be a JSON object")
    allowed = BOOLEANS | INTEGERS.keys() | NUMBERS.keys() | PATHS | MODELS | {"gyro_sign"}
    if settings.keys() - allowed:
        raise ValueError(f"Unknown agent settings: {sorted(settings.keys() - allowed)}")
    for key, value in settings.items():
        valid = False
        if key in BOOLEANS:
            valid = type(value) is bool
        elif key in INTEGERS:
            low, high = INTEGERS[key]
            valid = type(value) is int and low <= value <= high
        elif key in NUMBERS:
            low, high = NUMBERS[key]
            valid = finite_number(value) and low <= value <= high
        elif key in PATHS:
            valid = isinstance(value, str) and value.startswith("/") and "\0" not in value
        elif key == "gyro_sign":
            valid = finite_number(value) and value in (-1, 1)
        elif key in MODELS:
            valid = (isinstance(value, list) and len(value) == 2
                     and all(finite_number(v) for v in value)
                     and 0 <= value[0] <= 0.5 and 0 < value[1] <= 100)
        if not valid:
            raise ValueError(f"Invalid agent setting {key}: {value!r}")
    if settings.get("supply_critical_v", 4.63) >= settings.get("supply_warn_v", 4.75):
        raise ValueError("supply_critical_v must be below supply_warn_v")
    return {key.upper(): tuple(value) if key in MODELS else value
            for key, value in settings.items()}
