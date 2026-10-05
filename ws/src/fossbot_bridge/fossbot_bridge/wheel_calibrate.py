"""Measure motor duty/speed curves without ROS or changes to wheel geometry.

Stop bridge/teleop before running: this tool owns the telemetry port and sends
commands directly, so loss of this process lets the agent watchdog stop motors.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import secrets
import signal
import socket
import statistics
import sys
import time

from .protocol import (
    CMD_DIRECT, CMD_STOP, COMMAND_PORT, FLAG_ESTOP, FLAG_LOW_VOLTAGE,
    FLAG_MOTORS_ENABLED, SERVICE_PORT, TELEMETRY_PORT,
    pack_command, unpack_telemetry,
)

# Matches the agent's SUPPLY_WARN_V, below which it sets FLAG_LOW_VOLTAGE.
DEFAULT_MIN_SUPPLY_V = 4.75
# Stay above Pi 5 undervoltage (the agent's SUPPLY_CRITICAL_V = 4.63).
LOWEST_MIN_SUPPLY_V = 4.65


def summarize(frames, radius, ticks_per_rev):
    """Use cumulative counts over a settled interval, not noisy tick velocities."""
    if len(frames) < 2:
        raise ValueError("Not enough telemetry samples")
    elapsed = frames[-1]["t_robot"] - frames[0]["t_robot"]
    if elapsed <= 0:
        raise ValueError("Robot sample clock did not advance")
    result = {"seconds": elapsed, "frames": len(frames)}
    for side in ("left", "right"):
        ticks = frames[-1]["enc_" + side] - frames[0]["enc_" + side]
        result[side] = {
            "ticks": ticks,
            "speed_mps": abs(ticks) * 2 * math.pi * radius / ticks_per_rev / elapsed,
            "applied_duty": statistics.mean(abs(f["duty_" + side]) for f in frames),
        }
    result["min_supply_v"] = min(f["supply_v"] for f in frames)
    return result


def fit_motor(points):
    """Fit speed = gain*duty + intercept, then invert for feedforward.

    A running friction estimate is not a measured breakaway threshold. Reject
    weak data rather than deriving a large correction from one or two ticks.
    """
    moving = [p for p in points if abs(p["ticks"]) >= 8]
    if len(moving) < 3:
        return {"valid": False, "reason": "Need three moving levels with >=8 ticks each"}
    x = [p["applied_duty"] for p in moving]
    y = [p["speed_mps"] for p in moving]
    mx, my = statistics.mean(x), statistics.mean(y)
    xx = sum((v - mx) ** 2 for v in x)
    yy = sum((v - my) ** 2 for v in y)
    if xx < 1e-5 or yy < 1e-8:
        return {"valid": False, "reason": "Insufficient duty/speed variation"}
    gain = sum((a - mx) * (b - my) for a, b in zip(x, y)) / xx
    if gain <= 0:
        return {"valid": False, "reason": "Speed did not increase with duty"}
    intercept = my - gain * mx
    if intercept > 0:
        # A negative friction duty is unphysical. Refit with zero intercept.
        gain = sum(a * b for a, b in zip(x, y)) / sum(a * a for a in x)
        intercept = 0.0
    r_squared = 1 - sum((b - (gain * a + intercept)) ** 2
                        for a, b in zip(x, y)) / yy
    static = -intercept / gain
    if r_squared < 0.85 or static >= min(x):
        return {"valid": False, "reason": "Poor linear fit; repeat or inspect stalling",
                "r_squared": r_squared}
    return {"valid": True, "static_duty": static, "duty_per_mps": 1 / gain,
            "r_squared": r_squared, "speed_range_mps": [min(y), max(y)],
            "lowest_moving_tested_duty": min(p["applied_duty"] for p in points
                                            if abs(p["ticks"]) >= 2)}


class RobotLink:
    """Exclusive local UDP receiver plus bounded TCP service requests."""

    min_supply_v = DEFAULT_MIN_SUPPLY_V

    def __init__(self, host, telemetry_port=TELEMETRY_PORT,
                 command_port=COMMAND_PORT, service_port=SERVICE_PORT,
                 min_supply_v=DEFAULT_MIN_SUPPLY_V):
        self.min_supply_v = min_supply_v
        self.host = socket.gethostbyname(host)
        self.command_port = command_port
        self.service_port = service_port
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Intentionally no SO_REUSEADDR: refuse to compete with the bridge.
            self.rx.bind(("0.0.0.0", telemetry_port))
        except OSError:
            self.rx.close()
            raise RuntimeError("Telemetry port busy; stop ROS bringup/other calibration tools")
        self.rx.settimeout(0.02)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = secrets.randbelow(2**32)
        self.sent = deque(maxlen=256)
        self.previous = None
        self.last_rx = time.monotonic()
        self.owned = False

    def service(self, op, **kwargs):
        with socket.create_connection((self.host, self.service_port), timeout=1) as sock:
            sock.sendall((json.dumps(dict(op=op, **kwargs)) + "\n").encode())
            with sock.makefile("rb") as reader:
                reply = json.loads(reader.readline(65536))
        if not reply.get("ok"):
            raise RuntimeError(f"Agent rejected {op}: {reply}")
        return reply

    def send(self, duty=0.0):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        kind = CMD_DIRECT if duty else CMD_STOP
        self.tx.sendto(pack_command(self.seq, kind, duty, duty),
                       (self.host, self.command_port))
        self.sent.append(self.seq)

    def receive(self, moving=False, bootstrap=False):
        try:
            data, addr = self.rx.recvfrom(2048)
        except socket.timeout:
            if time.monotonic() - self.last_rx > 0.3:
                raise RuntimeError("Telemetry missing for 300 ms")
            return None
        if addr[0] != self.host:
            return None
        frame = unpack_telemetry(data)
        if frame is None:
            raise RuntimeError("Telemetry does not match the shared wire protocol")
        if frame["cmd_echo"] not in self.sent:
            if bootstrap:
                self.seq = frame["cmd_echo"]
                return None
            raise RuntimeError("Unrecognized command echo; another controller may be active")
        if self.previous is not None:
            delta = (frame["seq"] - self.previous["seq"]) & 0xFFFFFFFF
            if delta == 0 or delta > 2**31:
                raise RuntimeError("Telemetry reordered or agent restarted; repeat the run")
            dt = frame["t_robot"] - self.previous["t_robot"]
            if not math.isfinite(dt) or not 0 < dt < 0.3:
                raise RuntimeError("Robot clock jumped or telemetry stalled")
        if not math.isfinite(frame["supply_v"]) or frame["supply_v"] < self.min_supply_v:
            raise RuntimeError(
                f"Supply missing or below {self.min_supply_v:.2f} V; aborting sweep")
        # The agent flags low voltage at its fixed 4.75 V; a lowered guard
        # replaces that flag with the numeric check above.
        abort_flags = FLAG_ESTOP
        if self.min_supply_v >= DEFAULT_MIN_SUPPLY_V:
            abort_flags |= FLAG_LOW_VOLTAGE
        if frame["flags"] & abort_flags:
            raise RuntimeError("Robot reports emergency stop or low supply")
        if moving and not frame["flags"] & FLAG_MOTORS_ENABLED:
            raise RuntimeError("Motors became disabled during the sweep")
        self.previous = frame
        self.last_rx = time.monotonic()
        return frame

    def phase(self, duty, seconds, bootstrap=False):
        frames = []
        end = time.monotonic() + seconds
        next_send = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                self.send(duty)
                next_send = now + 0.02
            frame = self.receive(moving=bool(duty), bootstrap=bootstrap)
            if frame is not None:
                frames.append(frame)
        if not frames:
            raise RuntimeError("No usable telemetry")
        return frames

    def stop_and_disable(self):
        # Independent cleanup paths: TCP failure must not prevent UDP stop, and
        # UDP failure must not prevent disabling. No more commands after return.
        errors = []
        for _ in range(3):
            try:
                self.send()
            except OSError as exc:
                errors.append(str(exc))
            time.sleep(0.02)
        try:
            reply = self.service("enable_motors", enable=False)
            if reply.get("enabled") is not False:
                errors.append("Disable was not acknowledged")
            status = self.service("status")
            if status.get("motors_enabled") or any(status.get("target_mps", [1, 1])):
                errors.append("Robot did not confirm disabled motors and zero targets")
        except (OSError, ValueError, RuntimeError) as exc:
            errors.append(str(exc))
        return errors

    def close(self):
        self.rx.close()
        self.tx.close()


def run_sweep(args, report):
    link = RobotLink(args.host, min_supply_v=args.min_supply_v)
    try:
        status = link.service("status")
        report["initial_status"] = status
        if status.get("motors_enabled") or status.get("estop"):
            raise RuntimeError("Start with motors disabled and emergency stop released")
        if any(status.get("target_mps", [1, 1])):
            raise RuntimeError("Agent has pending motion; stop the existing controller first")
        link.owned = True
        baseline = link.phase(0, 1, bootstrap=True)
        if any(baseline[-1]["enc_" + s] != baseline[0]["enc_" + s]
               for s in ("left", "right")):
            raise RuntimeError("Wheels moved during stationary baseline")
        print("Starting in 3 seconds; Ctrl-C stops and disables motors.", flush=True)
        link.phase(0, 3)
        if link.service("enable_motors", enable=True).get("enabled") is not True:
            raise RuntimeError("Motor enable was not acknowledged")
        # Receive the enabled state before accepting motion samples.
        link.phase(0, 0.2)
        for direction, sign in (("forward", 1), ("reverse", -1)):
            for duty in args.duties:
                link.phase(0, args.rest)
                link.phase(sign * duty, args.settle)
                frames = link.phase(sign * duty, args.sample)
                point = summarize(frames, args.wheel_radius, args.ticks_per_rev)
                point.update(direction=direction, requested_duty=duty)
                report["points"].append(point)
                print(f"{direction:7} duty={duty:.3f}  "
                      f"left={point['left']['speed_mps']:.3f} m/s "
                      f"({point['left']['ticks']:+d} ticks)  "
                      f"right={point['right']['speed_mps']:.3f} m/s "
                      f"({point['right']['ticks']:+d} ticks)", flush=True)
        report["complete"] = True
    finally:
        if link.owned:
            errors = link.stop_and_disable()
            report["shutdown_errors"] = errors
            if errors:
                report["complete"] = False
                print("Stop/disable not fully confirmed: " + "; ".join(errors), file=sys.stderr)
            else:
                print("Motors stopped and disabled.", flush=True)
        link.close()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("FOSSBOT_HOST"),
                        help="Robot hostname/IP (or set FOSSBOT_HOST)")
    parser.add_argument("--run", action="store_true", help="Execute motor sweep (otherwise show plan only)")
    parser.add_argument("--condition", choices=("suspended", "floor"), default="suspended")
    parser.add_argument("--duties", type=float, nargs="+", default=[0.06, 0.09, 0.12, 0.16, 0.20])
    parser.add_argument("--max-duty", type=float, default=0.25,
                        help="Explicit sweep ceiling (default 0.25, maximum 0.50)")
    parser.add_argument("--min-supply-v", type=float, default=DEFAULT_MIN_SUPPLY_V,
                        help=f"Abort below this supply voltage (default {DEFAULT_MIN_SUPPLY_V}, "
                             f"lowest {LOWEST_MIN_SUPPLY_V}; low values risk Pi undervoltage)")
    parser.add_argument("--settle", type=float, default=1.0, help="Seconds to settle at each duty")
    parser.add_argument("--sample", type=float, default=4.0, help="Seconds of encoder measurement")
    parser.add_argument("--rest", type=float, default=1.0, help="Stopped interval between levels")
    parser.add_argument("--wheel-radius", type=float, default=0.03524, help="Existing radius, not a fit parameter")
    parser.add_argument("--ticks-per-rev", type=float, default=20.0)
    parser.add_argument("--output", type=Path, default=Path("motor-calibration.json"))
    return parser


def validate_args(parser, args):
    if args.run and not args.host:
        parser.error("--run requires --host or FOSSBOT_HOST")
    if not math.isfinite(args.max_duty) or not 0 < args.max_duty <= 0.50:
        parser.error("--max-duty must be in (0, 0.50]")
    if (not 3 <= len(args.duties) <= 8 or args.duties != sorted(set(args.duties)) or
            not all(math.isfinite(d) and 0 < d <= args.max_duty for d in args.duties)):
        parser.error("Use three to eight ascending, distinct duties within --max-duty")
    if (not math.isfinite(args.min_supply_v) or
            not LOWEST_MIN_SUPPLY_V <= args.min_supply_v <= DEFAULT_MIN_SUPPLY_V):
        parser.error(f"--min-supply-v must be between {LOWEST_MIN_SUPPLY_V} "
                     f"and {DEFAULT_MIN_SUPPLY_V}")
    for name, low, high in (("settle", 0.5, 5), ("sample", 2, 10), ("rest", 0.5, 5),
                            ("wheel_radius", 0.01, 0.1), ("ticks_per_rev", 1, 10000)):
        value = getattr(args, name)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f"--{name.replace('_', '-')} must be between {low} and {high}")


def print_fits(report):
    report["fits"] = {}
    for side in ("left", "right"):
        for direction in ("forward", "reverse"):
            points = [p[side] for p in report["points"] if p["direction"] == direction]
            fit = fit_motor(points)
            name = f"MOTOR_FF_{side.upper()}_{direction.upper()}"
            report["fits"][name] = fit
            if fit["valid"]:
                print(f"{name} = ({fit['static_duty']:.6f}, {fit['duty_per_mps']:.6f})"
                      f"  # R²={fit['r_squared']:.3f}")
            else:
                print(f"{name}: inconclusive — {fit['reason']}")


def main(args=None):
    parser = build_parser()
    options = parser.parse_args(args)
    validate_args(parser, options)
    duration = 2 * len(options.duties) * (options.rest + options.settle + options.sample) + 5
    print(f"Motor sweep: {options.condition}, both wheels, forward then reverse, "
          f"up to {max(options.duties):.0%} duty, about {duration:.0f} seconds.")
    print("Stop ROS bringup and all other command sources before running.")
    if options.condition == "floor":
        print("Open-loop speeds may differ: use a clear test area and expect curved travel.")
    else:
        print("Suspended measurements compare motors; do not apply as floor calibration.")
    if options.min_supply_v < DEFAULT_MIN_SUPPLY_V:
        print(f"Supply guard lowered to {options.min_supply_v:.2f} V: results reflect a "
              "sagging supply and may not transfer to a charged battery.")
    if not options.run:
        print("Plan only. Add --run to move the wheels. No connection was opened.")
        return 0
    report = {"version": 1, "complete": False, "condition": options.condition,
              "settings": {k: str(v) if isinstance(v, Path) else v
                           for k, v in vars(options).items()}, "points": []}
    # Check writable output before enabling anything; never overwrite a run.
    try:
        output = options.output.open("x")
    except OSError as exc:
        print(f"Cannot create report: {exc}", file=sys.stderr)
        return 1
    previous_handler = signal.getsignal(signal.SIGTERM)

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        run_sweep(options, report)
    except (KeyboardInterrupt, OSError, RuntimeError, ValueError) as exc:
        report["error"] = str(exc) or "Interrupted"
        print("Sweep aborted: " + report["error"], file=sys.stderr)
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        if report["complete"]:
            print("Candidate feedforward parameters (review before editing the agent):")
            print_fits(report)
        # The agent uses NaN for unavailable status fields. JSON null is portable.
        def clean(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {k: clean(v) for k, v in value.items()}
            if isinstance(value, list):
                return [clean(v) for v in value]
            return value
        with output:
            json.dump(clean(report), output, indent=2, allow_nan=False)
            output.write("\n")
        print(f"Report: {options.output}")
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    sys.exit(main())
