#!/usr/bin/env python3
"""At-a-glance FOSSBot status: link, battery, motors, motion and sensors.

Started by bringup.launch.py (use_dashboard:=false to skip). Opens a small Qt
window, with buttons to enable/disable the motors, toggle the e-stop and reset
odometry. Without a display (no $DISPLAY, or mode:=text) it prints a compact
summary to the terminal every few seconds instead.

Everything shown comes from the bridge's ordinary topics, so the dashboard
can run on any machine on the same ROS_DOMAIN_ID, not just the one running
the bridge.
"""

from collections import deque
import math
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (BatteryState, CompressedImage, Imu, JointState,
                             LaserScan, Range)
from std_srvs.srv import SetBool, Trigger

from fossbot_msgs.msg import Buttons, LineSensors, LinkStatus

# Best effort matches every publisher: a Reliable publisher satisfies a Best
# Effort subscriber, and the reverse is not needed here. The dashboard would
# rather drop a frame than queue one.
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

STALE_S = 2.0
IR_SIDES = ("front_left", "front_right", "back_left", "back_right")


class Rate:
    """Messages per second over a short sliding window."""

    def __init__(self, window=2.0):
        self.window = window
        self.stamps = deque()

    def tick(self):
        now = time.monotonic()
        self.stamps.append(now)
        while self.stamps and self.stamps[0] < now - self.window:
            self.stamps.popleft()

    def hz(self):
        now = time.monotonic()
        while self.stamps and self.stamps[0] < now - self.window:
            self.stamps.popleft()
        return len(self.stamps) / self.window


class DashboardNode(Node):

    def __init__(self):
        super().__init__("fossbot_dashboard")
        self.declare_parameter("mode", "auto")           # auto | gui | text
        self.declare_parameter("wheel_radius", 0.03524)  # display only
        self.declare_parameter("text_period_s", 3.0)
        self.mode = self.get_parameter("mode").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.text_period = self.get_parameter("text_period_s").value

        self.latest = {}
        self.seen = {}
        self.rates = {k: Rate() for k in ("scan", "camera", "imu", "odom")}
        self.ir = {}
        self.cmd = None
        self.notice = ""

        def keep(key, rate=None):
            def cb(msg):
                self.latest[key] = msg
                self.seen[key] = time.monotonic()
                if rate:
                    self.rates[rate].tick()
            return cb

        sub = self.create_subscription
        sub(LinkStatus, "/fossbot/link_status", keep("link"), QOS)
        sub(BatteryState, "/fossbot/battery", keep("battery"), QOS)
        sub(Odometry, "/odom", keep("odom", "odom"), QOS)
        sub(JointState, "/joint_states", keep("joints"), QOS)
        sub(LaserScan, "/scan", keep("scan", "scan"), QOS)
        sub(CompressedImage, "/camera/image_raw/compressed",
            keep("camera", "camera"), QOS)
        sub(Imu, "/imu/data_raw", keep("imu", "imu"), QOS)
        sub(LineSensors, "/fossbot/line_sensors", keep("line"), QOS)
        sub(Buttons, "/fossbot/buttons", keep("buttons"), QOS)
        sub(Twist, "/cmd_vel", keep("cmd"), 10)
        def keep_ir(msg, side):
            self.ir[side] = msg.range
            self.seen["ir"] = time.monotonic()
        for side in IR_SIDES:
            sub(Range, f"/fossbot/range/{side}",
                lambda m, s=side: keep_ir(m, s), QOS)

        self.cli_enable = self.create_client(SetBool, "/fossbot/enable_motors")
        self.cli_estop = self.create_client(SetBool, "/fossbot/estop")
        self.cli_reset = self.create_client(Trigger, "/fossbot/reset_odometry")

    # --- data helpers ------------------------------------------------------
    def fresh(self, key):
        t = self.seen.get(key)
        return t is not None and time.monotonic() - t < STALE_S

    def get(self, key):
        return self.latest.get(key) if self.fresh(key) else None

    def call(self, client, request, label):
        """Fire-and-forget service call; the result lands in self.notice."""
        if not client.service_is_ready():
            self.notice = f"{label}: bridge not reachable"
            return
        future = client.call_async(request)

        def done(f):
            try:
                r = f.result()
                ok = getattr(r, "success", False)
                msg = getattr(r, "message", "")
                self.notice = f"{label}: {'ok' if ok else 'FAILED'} {msg}".strip()
            except Exception as exc:
                self.notice = f"{label}: {exc}"
        future.add_done_callback(done)
        self.notice = f"{label} ..."

    def set_motors(self, enable):
        r = SetBool.Request()
        r.data = enable
        self.call(self.cli_enable, r, "motors on" if enable else "motors off")

    def toggle_estop(self):
        link = self.get("link")
        r = SetBool.Request()
        r.data = not (link.estop if link else False)
        self.call(self.cli_estop, r, "e-stop engage" if r.data else "e-stop release")

    def reset_odom(self):
        self.call(self.cli_reset, Trigger.Request(), "reset odometry")

    # --- one snapshot of everything, shared by the GUI and the text mode -----
    def snapshot(self):
        link, bat = self.get("link"), self.get("battery")
        odom, joints = self.get("odom"), self.get("joints")
        scan, line, buttons = self.get("scan"), self.get("line"), self.get("buttons")
        cmd = self.get("cmd")
        s = {"have_bridge": link is not None}

        s["host"] = link.robot_host if link else "?"
        s["connected"] = bool(link and link.connected)
        s["telemetry_hz"] = link.telemetry_hz if link else 0.0
        s["latency_ms"] = link.latency_ms if link else float("nan")
        s["loss"] = link.loss_percent if link else float("nan")

        s["volts"] = bat.voltage if bat else float("nan")
        s["amps"] = -bat.current if bat else float("nan")
        s["percent"] = bat.percentage * 100.0 if bat else float("nan")
        s["estimated"] = bool(bat and bat.location.startswith("estimated"))
        s["measured"] = bool(bat and bat.location == "pack_via_adc")
        # From the battery message itself, so it works without link_status:
        # the bridge reports HEALTH_UNKNOWN in the warning band (BatteryState
        # has no "degraded" value) and HEALTH_DEAD below the critical voltage.
        s["low"] = bool(link and link.low_voltage) or bool(
            bat and bat.voltage == bat.voltage
            and bat.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
        s["dead"] = bool(bat and bat.power_supply_health
                         == BatteryState.POWER_SUPPLY_HEALTH_DEAD)

        s["enabled"] = bool(link and link.motors_enabled)
        s["estop"] = bool(link and link.estop)
        s["duty"] = (link.duty_left, link.duty_right) if link else (0.0, 0.0)
        if joints and len(joints.velocity) >= 2:
            s["wheel_mps"] = tuple(v * self.wheel_radius for v in joints.velocity[:2])
        else:
            s["wheel_mps"] = (float("nan"), float("nan"))

        s["cmd"] = (cmd.linear.x, cmd.angular.z) if cmd else (0.0, 0.0)
        if odom:
            q = odom.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            s["pose"] = (odom.pose.pose.position.x, odom.pose.pose.position.y,
                         math.degrees(yaw))
            s["vel"] = (odom.twist.twist.linear.x, odom.twist.twist.angular.z)
        else:
            s["pose"] = s["vel"] = None

        s["imu_ok"] = link.imu_ok if link else None   # None = unknown
        s["imu_hz"] = self.rates["imu"].hz()
        s["lidar_ok"] = bool(link and link.lidar_ok)
        s["scan_hz"] = self.rates["scan"].hz()
        finite = [r for r in scan.ranges if math.isfinite(r)] if scan else []
        s["nearest"] = min(finite) if finite else float("nan")
        s["camera_hz"] = self.rates["camera"].hz()
        s["line"] = ((line.on_line_left, line.on_line_middle, line.on_line_right)
                     if line else None)
        s["ir"] = dict(self.ir) if self.fresh("ir") else {}
        s["buttons"] = ((buttons.sw1, buttons.sw2, buttons.sw3, buttons.sw4)
                        if buttons else None)
        s["notice"] = self.notice
        return s


def fmt(v, spec, unit=""):
    return "--" if v is None or v != v else f"{v:{spec}}{unit}"


# --- text mode -------------------------------------------------------------
def text_lines(s):
    if not s["have_bridge"]:
        link = "no /fossbot/link_status (bridge not running, or a different version)"
    else:
        link = "CONNECTED" if s["connected"] else "NO LINK"
    bat = ("CRITICAL" if s["dead"] else "LOW" if s["low"] else "ok")
    est = "~" if s["estimated"] else ""
    motors = "E-STOP" if s["estop"] else ("ENABLED" if s["enabled"] else "disabled")
    out = [
        f"robot {s['host']}: {link}  {s['telemetry_hz']:.0f} Hz  "
        f"rtt {fmt(s['latency_ms'], '.0f', ' ms')}  loss {fmt(s['loss'], '.1f', '%')}",
        f"battery {est}{fmt(s['percent'], '.0f', '%')}  {fmt(s['volts'], '.2f', ' V')}  "
        f"{fmt(s['amps'], '.2f', ' A')}  [{bat}]",
        f"motors {motors}  duty L {s['duty'][0]:+.2f} R {s['duty'][1]:+.2f}  "
        f"wheels {fmt(s['wheel_mps'][0], '+.2f')} / {fmt(s['wheel_mps'][1], '+.2f')} m/s",
    ]
    if s["pose"]:
        out.append(f"odom x {s['pose'][0]:+.2f} y {s['pose'][1]:+.2f} m  "
                   f"yaw {s['pose'][2]:+.0f} deg  v {s['vel'][0]:+.2f} m/s  "
                   f"w {s['vel'][1]:+.2f} rad/s")
    out.append(f"lidar {s['scan_hz']:.1f} Hz (nearest {fmt(s['nearest'], '.2f', ' m')})  "
               f"camera {s['camera_hz']:.0f} fps  imu {s['imu_hz']:.0f} Hz")
    return out


def spin_quietly(node):
    """rclpy.spin that ends silently when the context shuts down (Ctrl+C)."""
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def run_text(node):
    print("fossbot dashboard: no display, printing a summary every "
          f"{node.text_period:.0f} s (mode:=gui to force the window)", flush=True)
    # First summary only once the 2 s rate windows have filled, or every rate
    # in it reads low.
    next_t = time.monotonic() + 2.5
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() >= next_t:
            next_t += node.text_period
            print("\n".join(text_lines(node.snapshot())), flush=True)


# --- GUI mode --------------------------------------------------------------
def run_gui(node):
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import (QApplication, QGridLayout, QGroupBox,
                                             QHBoxLayout, QLabel, QProgressBar,
                                             QPushButton, QVBoxLayout, QWidget)

    app = QApplication.instance() or QApplication(sys.argv)
    win = QWidget()
    win.setWindowTitle("FOSSBot dashboard")
    win.setMinimumWidth(780)
    root = QVBoxLayout(win)

    GREEN, AMBER, RED, GREY = "#2e9d4b", "#d48a00", "#c62828", "#8a8a8a"

    def badge(text="", color=GREY):
        lab = QLabel(text)
        lab.setAlignment(Qt.AlignCenter)
        lab.setMinimumWidth(110)
        lab.setStyleSheet(f"background:{color};color:white;border-radius:4px;"
                          "padding:3px 8px;font-weight:bold;")
        return lab

    def paint(lab, text, color):
        lab.setText(text)
        lab.setStyleSheet(f"background:{color};color:white;border-radius:4px;"
                          "padding:3px 8px;font-weight:bold;")

    def bar():
        b = QProgressBar()
        b.setRange(0, 100)
        b.setTextVisible(True)
        return b

    def tint(b, color):
        b.setStyleSheet(f"QProgressBar::chunk{{background:{color};}}")

    def group(title):
        box = QGroupBox(title)
        grid = QGridLayout(box)
        return box, grid

    # header
    head = QHBoxLayout()
    host = QLabel()
    host.setStyleSheet("font-size:15px;font-weight:bold;")
    conn = badge()
    linkinfo = QLabel()
    head.addWidget(host)
    head.addStretch(1)
    head.addWidget(linkinfo)
    head.addWidget(conn)
    root.addLayout(head)

    cols = QGridLayout()
    root.addLayout(cols)

    # battery
    bbox, bg = group("Battery")
    bat_bar = bar()
    bat_badge = badge()
    bat_volts, bat_amps, bat_note = QLabel(), QLabel(), QLabel()
    bat_note.setStyleSheet(f"color:{GREY};font-size:11px;")
    bat_note.setWordWrap(True)
    bg.addWidget(bat_bar, 0, 0, 1, 2)
    bg.addWidget(bat_badge, 0, 2)
    bg.addWidget(QLabel("supply"), 1, 0)
    bg.addWidget(bat_volts, 1, 1)
    bg.addWidget(QLabel("current"), 2, 0)
    bg.addWidget(bat_amps, 2, 1)
    bg.addWidget(bat_note, 3, 0, 1, 3)
    cols.addWidget(bbox, 0, 0)

    # motors
    mbox, mg = group("Motors")
    mot_badge = badge()
    estop_badge = badge()
    duty_l, duty_r = bar(), bar()
    wheels = QLabel()
    mg.addWidget(mot_badge, 0, 0)
    mg.addWidget(estop_badge, 0, 1)
    mg.addWidget(QLabel("duty left"), 1, 0)
    mg.addWidget(duty_l, 1, 1)
    mg.addWidget(QLabel("duty right"), 2, 0)
    mg.addWidget(duty_r, 2, 1)
    mg.addWidget(QLabel("wheel speed"), 3, 0)
    mg.addWidget(wheels, 3, 1)
    cols.addWidget(mbox, 0, 1)

    # motion
    obox, og = group("Motion")
    labels = {}
    for i, name in enumerate(("commanded", "measured", "position", "heading")):
        og.addWidget(QLabel(name), i, 0)
        labels[name] = QLabel()
        labels[name + "2"] = QLabel()
        og.addWidget(labels[name], i, 1)
        og.addWidget(labels[name + "2"], i, 2)
    cols.addWidget(obox, 1, 0)

    # sensors
    sbox, sg = group("Sensors")
    for i, name in enumerate(("lidar", "camera", "imu", "line", "IR range", "buttons")):
        sg.addWidget(QLabel(name), i, 0)
        labels[name] = QLabel()
        sg.addWidget(labels[name], i, 1)
    cols.addWidget(sbox, 1, 1)

    # controls
    row = QHBoxLayout()
    b_on = QPushButton("Enable motors")
    b_off = QPushButton("Disable motors")
    b_estop = QPushButton("E-STOP")
    b_estop.setStyleSheet(f"background:{RED};color:white;font-weight:bold;padding:4px 14px;")
    b_reset = QPushButton("Reset odometry")
    b_on.clicked.connect(lambda: node.set_motors(True))
    b_off.clicked.connect(lambda: node.set_motors(False))
    b_estop.clicked.connect(node.toggle_estop)
    b_reset.clicked.connect(node.reset_odom)
    for b in (b_on, b_off, b_estop, b_reset):
        row.addWidget(b)
    root.addLayout(row)

    notice = QLabel()
    notice.setStyleSheet(f"color:{GREY};")
    root.addWidget(notice)

    def refresh():
        s = node.snapshot()
        host.setText(f"FOSSBot  {s['host']}")
        if not s["have_bridge"]:
            paint(conn, "NO BRIDGE", GREY)
            linkinfo.setText("no /fossbot/link_status yet ")
        elif s["connected"]:
            paint(conn, "CONNECTED", GREEN)
            linkinfo.setText(f"{s['telemetry_hz']:.0f} Hz   rtt {fmt(s['latency_ms'], '.0f', ' ms')}"
                             f"   loss {fmt(s['loss'], '.1f', '%')}   ")
        else:
            paint(conn, "NO LINK", RED)
            linkinfo.setText("robot not responding ")

        pct = s["percent"]
        if pct == pct:
            bat_bar.setValue(int(round(pct)))
            bat_bar.setFormat(("~" if s["estimated"] else "") + "%p %")
        else:
            bat_bar.setValue(0)
            bat_bar.setFormat("--")
        color = RED if s["dead"] else AMBER if s["low"] else GREEN
        tint(bat_bar, color)
        paint(bat_badge, "CRITICAL" if s["dead"] else "LOW" if s["low"] else "OK"
              if pct == pct else "--", color if pct == pct else GREY)
        bat_volts.setText(fmt(s["volts"], ".2f", " V"))
        bat_amps.setText(fmt(s["amps"], ".2f", " A"))
        if s["measured"]:
            bat_note.setText("Charge from the measured pack voltage. "
                             "Supply is the Pi's 5 V rail.")
        elif s["estimated"]:
            bat_note.setText("Charge is estimated from the Pi's 5 V rail (shown as "
                             "supply), which stays nearly flat until the battery "
                             "is almost empty.")
        else:
            bat_note.setText("")

        if s["estop"]:
            paint(mot_badge, "E-STOP", RED)
        elif s["enabled"]:
            paint(mot_badge, "ENABLED", GREEN)
        else:
            paint(mot_badge, "DISABLED", GREY)
        paint(estop_badge, "E-STOP ON" if s["estop"] else "e-stop off",
              RED if s["estop"] else GREY)
        b_estop.setText("Release e-stop" if s["estop"] else "E-STOP")
        for b, d in ((duty_l, s["duty"][0]), (duty_r, s["duty"][1])):
            b.setValue(int(round(abs(d) * 100)))
            b.setFormat(f"{d:+.2f}")
            tint(b, AMBER if abs(d) > 0.7 else GREEN)
        wheels.setText(f"L {fmt(s['wheel_mps'][0], '+.2f')}   "
                       f"R {fmt(s['wheel_mps'][1], '+.2f')}  m/s")

        labels["commanded"].setText(f"{s['cmd'][0]:+.2f} m/s")
        labels["commanded2"].setText(f"{s['cmd'][1]:+.2f} rad/s")
        if s["pose"]:
            labels["measured"].setText(f"{s['vel'][0]:+.2f} m/s")
            labels["measured2"].setText(f"{s['vel'][1]:+.2f} rad/s")
            labels["position"].setText(f"x {s['pose'][0]:+.2f} m")
            labels["position2"].setText(f"y {s['pose'][1]:+.2f} m")
            labels["heading"].setText(f"{s['pose'][2]:+.0f} deg")
        else:
            for k in ("measured", "measured2", "position", "position2", "heading"):
                labels[k].setText("--")

        labels["lidar"].setText(
            (f"{s['scan_hz']:.1f} Hz,  nearest {fmt(s['nearest'], '.2f', ' m')}"
             if s["scan_hz"] > 0 else "no scans") +
            ("" if s["lidar_ok"] or not s["have_bridge"] else "   (not ok)"))
        labels["camera"].setText(f"{s['camera_hz']:.0f} fps" if s["camera_hz"] > 0
                                 else "no frames")
        labels["imu"].setText(
            "not ok" if s["imu_ok"] is False else
            f"{s['imu_hz']:.0f} Hz" if s["imu_hz"] > 0 else "no data")
        # filled marker = on the line / pressed
        labels["line"].setText("  ".join(
            f"{'●' if v else '○'} {n}" for n, v in zip(("left", "mid", "right"), s["line"]))
            if s["line"] else "--")
        short = {"front_left": "FL", "front_right": "FR",
                 "back_left": "BL", "back_right": "BR"}
        labels["IR range"].setText("  ".join(
            f"{short[k]} {s['ir'][k]:.2f}" for k in IR_SIDES if k in s["ir"]) or "--")
        labels["buttons"].setText("  ".join(
            f"{'●' if v else '○'} {i + 1}" for i, v in enumerate(s["buttons"]))
            if s["buttons"] else "--")
        notice.setText(s["notice"])

    spin = threading.Thread(target=spin_quietly, args=(node,), daemon=True)
    spin.start()

    timer = QTimer()
    timer.timeout.connect(refresh)
    timer.start(200)
    refresh()

    # Let Ctrl+C from the launch terminal close the window: Qt's event loop
    # never returns to Python on its own, so poke it periodically.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    tick = QTimer()
    tick.timeout.connect(lambda: None)
    tick.start(250)

    win.show()
    return app.exec_()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    mode = node.mode
    if mode == "auto":
        mode = "gui" if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") \
            else "text"
    try:
        if mode == "gui":
            try:
                run_gui(node)
            except ImportError as exc:
                node.get_logger().warn(f"no Qt ({exc}); falling back to text mode")
                run_text(node)
        else:
            run_text(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
