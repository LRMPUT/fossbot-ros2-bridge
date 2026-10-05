#!/usr/bin/env python3
"""ROS 2 <-> FOSSBot bridge. Runs on the PC, talks to the agent on the robot.

Publishes the same interface the Gazebo sim publishes (/odom, /joint_states,
/tf, /cmd_vel, /scan), so Nav2 and SLAM configs written against the simulator
run unchanged against the real robot, plus fossbot-specific topics for the
hardware the sim has no equivalent for.
"""

import math
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Range, Illuminance, JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from fossbot_msgs.msg import (AnalogRaw, Buttons, LineSensors, LinkStatus,
                              MotorCommand)
from fossbot_msgs.srv import PlayTone, SetRGB

from fossbot_bridge.battery import Smoother, liion_fraction, rail_fraction
from fossbot_bridge.protocol import (
    TELEMETRY_PORT, COMMAND_PORT, SERVICE_PORT, TELEMETRY_SIZE,
    unpack_telemetry, pack_command,
    CMD_TWIST, CMD_DIRECT, CMD_STOP,
    FLAG_MOTORS_ENABLED, FLAG_ESTOP, FLAG_LIDAR_OK, FLAG_IMU_OK,
    FLAG_LOW_VOLTAGE,
    ADC_DIST_FL, ADC_DIST_FR, ADC_DIST_BL, ADC_DIST_BR,
    ADC_LINE_LEFT, ADC_LINE_MID, ADC_LINE_RIGHT,
    ADC_LDR, ADC_MIC, ADC_PHOTODIODE,
)

# Reliable, not best-effort. QoS matching is asymmetric: a RELIABLE publisher
# satisfies a BEST_EFFORT subscriber, but a BEST_EFFORT publisher is refused by
# a RELIABLE one. RViz defaults its displays to Reliable, so publishing
# best-effort makes topics appear in the list and silently deliver nothing --
# the single most confusing failure mode here. Nav2 and slam_toolbox subscribe
# with SensorDataQoS (best effort) and match a reliable publisher fine, so this
# is strictly more compatible. These rates are low enough that it costs nothing.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ServiceClient:
    """Request/response over the agent's TCP service port, with lazy reconnect."""

    def __init__(self, host, logger):
        self.host = host
        self.logger = logger
        self.lock = threading.Lock()
        self.sock = None
        self.reader = None

    def _connect(self):
        s = socket.create_connection((self.host, SERVICE_PORT), timeout=3.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        self.reader = s.makefile("r")

    def call(self, payload):
        import json
        with self.lock:
            for attempt in (1, 2):
                try:
                    if self.sock is None:
                        self._connect()
                    self.sock.sendall((json.dumps(payload) + "\n").encode())
                    line = self.reader.readline()
                    if not line:
                        raise OSError("service connection closed")
                    return json.loads(line)
                except (OSError, ValueError) as exc:
                    self.sock = None
                    self.reader = None
                    if attempt == 2:
                        self.logger.warn(f"service call failed: {exc}")
                        return {"ok": False, "error": str(exc)}
        return {"ok": False, "error": "unreachable"}


class FossbotBridge(Node):

    def __init__(self):
        super().__init__("fossbot_bridge")

        self.declare_parameter("robot_host", os.environ.get("FOSSBOT_HOST", ""))
        self.declare_parameter("wheel_radius", 0.03524)
        self.declare_parameter("wheel_track", 0.1866)
        self.declare_parameter("encoder_ticks_per_rev", 20.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("on_line_threshold", 200)
        self.declare_parameter("ir_range_min", 0.05)
        self.declare_parameter("ir_range_max", 0.40)
        self.declare_parameter("cmd_rate_hz", 50.0)
        # Pi 5 flags undervoltage near 4.63 V and resets below roughly 4.4 V.
        self.declare_parameter("supply_warn_v", 4.75)
        self.declare_parameter("supply_critical_v", 4.63)
        # Battery percentage. There is no battery sense on the FOSSBot PCB, so
        # by default this is an ESTIMATE from the Pi's regulated 5 V rail,
        # which stays nearly flat until the battery is close to empty. For a
        # real reading, wire a divider from the pack to a spare ADC header
        # (J10 = adc index 11, J11 = adc index 14) and set battery_source to
        # "adc". See README, "Battery".
        self.declare_parameter("battery_source", "rail")       # rail | adc
        self.declare_parameter("battery_rail_full_v", 4.90)
        self.declare_parameter("battery_rail_empty_v", 4.65)
        self.declare_parameter("battery_adc_index", 11)
        self.declare_parameter("battery_divider_ratio", 3.0)   # V_pack / V_adc
        self.declare_parameter("battery_cells", 2)             # Li-ion, series
        self.declare_parameter("battery_smoothing_s", 10.0)

        p = self.get_parameter
        self.robot_host = p("robot_host").value
        if not self.robot_host:
            raise ValueError("Set robot_host or FOSSBOT_HOST to select a robot")
        self.wheel_radius = p("wheel_radius").value
        self.wheel_track = p("wheel_track").value
        self.ticks_per_rev = p("encoder_ticks_per_rev").value
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.imu_frame = p("imu_frame").value
        self.publish_tf = p("publish_tf").value
        self.on_line_threshold = p("on_line_threshold").value
        self.ir_min = p("ir_range_min").value
        self.ir_max = p("ir_range_max").value
        self.supply_warn_v = p("supply_warn_v").value
        self.supply_critical_v = p("supply_critical_v").value
        self.battery_source = p("battery_source").value
        if self.battery_source not in ("rail", "adc"):
            raise ValueError("battery_source must be 'rail' or 'adc'")
        self.battery_rail_full_v = p("battery_rail_full_v").value
        self.battery_rail_empty_v = p("battery_rail_empty_v").value
        self.battery_adc_index = int(p("battery_adc_index").value)
        self.battery_divider_ratio = p("battery_divider_ratio").value
        self.battery_cells = int(p("battery_cells").value)
        self.battery_smoothing_s = p("battery_smoothing_s").value
        self.battery_smoother = Smoother(self.battery_smoothing_s)

        # --- publishers ---------------------------------------------------
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_joints = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", SENSOR_QOS)
        self.pub_line = self.create_publisher(LineSensors,
                                              "/fossbot/line_sensors", SENSOR_QOS)
        self.pub_analog = self.create_publisher(AnalogRaw,
                                                "/fossbot/analog_raw", SENSOR_QOS)
        self.pub_buttons = self.create_publisher(Buttons, "/fossbot/buttons", 10)
        self.pub_light = self.create_publisher(Illuminance, "/fossbot/light",
                                               SENSOR_QOS)
        self.pub_mic = self.create_publisher(Float32, "/fossbot/microphone",
                                             SENSOR_QOS)
        self.pub_photodiode = self.create_publisher(Float32,
                                                    "/fossbot/photodiode", SENSOR_QOS)
        self.pub_status = self.create_publisher(LinkStatus, "/fossbot/link_status", 10)
        self.pub_battery = self.create_publisher(BatteryState, "/fossbot/battery", 10)
        self.pub_ultra = self.create_publisher(Range, "/fossbot/ultrasonic",
                                               SENSOR_QOS)
        self.pub_ir = {
            name: self.create_publisher(Range, f"/fossbot/range/{name}", SENSOR_QOS)
            for name in ("front_left", "front_right", "back_left", "back_right")
        }
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- subscribers --------------------------------------------------
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(MotorCommand, "/fossbot/motor_cmd",
                                 self.on_motor_cmd, 10)

        # --- services -----------------------------------------------------
        self.svc = ServiceClient(self.robot_host, self.get_logger())
        self.create_service(SetRGB, "/fossbot/set_rgb", self.on_set_rgb)
        self.create_service(PlayTone, "/fossbot/play_tone", self.on_play_tone)
        self.create_service(SetBool, "/fossbot/enable_motors", self.on_enable_motors)
        self.create_service(Trigger, "/fossbot/reset_odometry", self.on_reset_odom)
        self.create_service(SetBool, "/fossbot/estop", self.on_estop)

        # --- link state ---------------------------------------------------
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(("0.0.0.0", TELEMETRY_PORT))
        self.rx_sock.settimeout(0.5)
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_seq = 0
        self.cmd_lock = threading.Lock()
        self.pending_cmd = (CMD_STOP, 0.0, 0.0)

        self.frames = 0
        self.lost = 0
        self.last_seq = None
        self.last_rx_t = 0.0
        self.rate_window = []
        self.latency_ms = 0.0
        self.flags = 0
        self.supply_v = float("nan")
        self.supply_a = float("nan")
        self.duty_l = 0.0
        self.duty_r = 0.0
        self.wheel_vel_l = 0.0
        self.wheel_vel_r = 0.0
        # seq -> monotonic send time, for round-trip latency. The robot's wall
        # clock cannot be trusted (it can be days off), so latency is measured
        # as a true round trip rather than by differencing the two clocks.
        self.cmd_sent_at = {}

        # Odometry integration state.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_enc = None
        self.last_odom_t = 0.0
        self.wheel_pos_l = 0.0
        self.wheel_pos_r = 0.0

        self.running = True
        self.rx_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.rx_thread.start()

        # A steady command stream keeps the robot's watchdog fed; without it the
        # agent cuts the motors after COMMAND_TIMEOUT_S.
        self.create_timer(1.0 / p("cmd_rate_hz").value, self.send_command)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f"bridge up, robot={self.robot_host}, "
            f"telemetry udp/{TELEMETRY_PORT}, commands udp/{COMMAND_PORT}")

        # Send one frame immediately so the agent learns our address and starts
        # streaming telemetry without waiting for a /cmd_vel.
        self.send_command()

    # --- telemetry ------------------------------------------------------
    def telemetry_loop(self):
        while self.running:
            try:
                data, _ = self.rx_sock.recvfrom(TELEMETRY_SIZE * 2)
            except socket.timeout:
                continue
            except OSError:
                break
            t = unpack_telemetry(data)
            if t is None:
                continue
            try:
                self.handle_telemetry(t)
            except Exception as exc:
                self.get_logger().error(f"telemetry handling failed: {exc}")

    def handle_telemetry(self, t):
        now = time.time()
        self.frames += 1
        if self.last_seq is not None:
            gap = t["seq"] - self.last_seq
            if gap > 1:
                self.lost += gap - 1
        self.last_seq = t["seq"]
        self.last_rx_t = now
        self.flags = t["flags"]
        self.duty_l = float(t["duty_left"])
        self.duty_r = float(t["duty_right"])

        # Round-trip: PC sent command N -> robot applied it -> robot echoed N
        # back in this telemetry frame. Independent of both wall clocks.
        sent = self.cmd_sent_at.pop(t["cmd_echo"], None)
        if sent is not None:
            self.latency_ms = (time.monotonic() - sent) * 1000.0
            # Drop everything older than the echoed command; those commands were
            # superseded and will never be echoed.
            stale = [k for k in self.cmd_sent_at if k < t["cmd_echo"]]
            for k in stale:
                del self.cmd_sent_at[k]

        self.rate_window.append(now)
        cutoff = now - 2.0
        while self.rate_window and self.rate_window[0] < cutoff:
            self.rate_window.pop(0)

        stamp = self.get_clock().now().to_msg()
        adc = t["adc"]

        self.publish_odometry(t, stamp)
        self.publish_imu(t, stamp)
        self.publish_analog(adc, stamp)
        self.publish_line(adc, stamp)
        self.publish_ranges(adc, t, stamp)
        self.publish_buttons(t, stamp)
        self.publish_battery(t, stamp)

    def publish_odometry(self, t, stamp):
        el, er = t["enc_left"], t["enc_right"]
        if self.last_enc is None:
            self.last_enc = (el, er)
            self.last_odom_t = t["t_robot"]
            return
        d_l_ticks = el - self.last_enc[0]
        d_r_ticks = er - self.last_enc[1]
        self.last_enc = (el, er)

        # Use the robot's own sample clock for dt: it is the only clock that saw
        # both encoder reads, and it is immune to jitter on the wifi link.
        dt = t["t_robot"] - self.last_odom_t
        self.last_odom_t = t["t_robot"]
        if dt <= 0.0:
            return

        m_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        d_l = d_l_ticks * m_per_tick
        d_r = d_r_ticks * m_per_tick
        d_center = (d_l + d_r) / 2.0
        d_yaw = (d_r - d_l) / self.wheel_track

        self.yaw = math.atan2(math.sin(self.yaw + d_yaw),
                              math.cos(self.yaw + d_yaw))
        self.x += d_center * math.cos(self.yaw)
        self.y += d_center * math.sin(self.yaw)
        self.wheel_pos_l += d_l / self.wheel_radius
        self.wheel_pos_r += d_r / self.wheel_radius
        self.wheel_vel_l = d_l / self.wheel_radius / dt
        self.wheel_vel_r = d_r / self.wheel_radius / dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        # Single-channel encoders give no direction feedback, so the covariance
        # is deliberately loose -- do not let a filter trust this too much.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_yaw / dt
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1
        self.pub_odom.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = yaw_to_quaternion(self.yaw)
            self.tf_broadcaster.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        # Names must match the URDF in fossbot_educational_description.
        js.name = ["Revolute 21", "Revolute 20"]
        js.position = [self.wheel_pos_l, self.wheel_pos_r]
        js.velocity = [self.wheel_vel_l, self.wheel_vel_r]   # rad/s
        self.pub_joints.publish(js)

    def publish_imu(self, t, stamp):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.linear_acceleration.x = t["imu"][0]
        imu.linear_acceleration.y = t["imu"][1]
        imu.linear_acceleration.z = t["imu"][2]
        imu.angular_velocity.x = t["imu"][3]
        imu.angular_velocity.y = t["imu"][4]
        imu.angular_velocity.z = t["imu"][5]
        # No magnetometer and no fusion here, so orientation is unknown.
        # -1 in element 0 is the REP-145 way of saying so.
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = 0.04
        imu.linear_acceleration_covariance[4] = 0.04
        imu.linear_acceleration_covariance[8] = 0.04
        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.02
        self.pub_imu.publish(imu)

    def publish_analog(self, adc, stamp):
        msg = AnalogRaw()
        msg.header.stamp = stamp
        msg.counts = [int(v) for v in adc]
        msg.volts = [v / 1023.0 * 3.3 for v in adc]
        self.pub_analog.publish(msg)

        light = Illuminance()
        light.header.stamp = stamp
        # The LDR divider is not photometrically calibrated; this is a relative
        # brightness figure, not real lux.
        light.illuminance = float(adc[ADC_LDR]) / 1023.0 * 100.0
        light.variance = 0.0
        self.pub_light.publish(light)

        self.pub_mic.publish(Float32(data=adc[ADC_MIC] / 1023.0 * 3.3))
        self.pub_photodiode.publish(
            Float32(data=adc[ADC_PHOTODIODE] / 1023.0 * 3.3))

    def publish_line(self, adc, stamp):
        msg = LineSensors()
        msg.header.stamp = stamp
        msg.left_raw = int(adc[ADC_LINE_LEFT])
        msg.middle_raw = int(adc[ADC_LINE_MID])
        msg.right_raw = int(adc[ADC_LINE_RIGHT])
        msg.left = msg.left_raw / 1023.0
        msg.middle = msg.middle_raw / 1023.0
        msg.right = msg.right_raw / 1023.0
        thr = self.on_line_threshold
        msg.on_line_left = msg.left_raw < thr
        msg.on_line_middle = msg.middle_raw < thr
        msg.on_line_right = msg.right_raw < thr
        self.pub_line.publish(msg)

    def publish_ranges(self, adc, t, stamp):
        # The Sharp-style IR sensors are not linear in distance and were never
        # characterised for this board, so the reported range is a monotonic
        # approximation only. Treat crossing a threshold as meaningful, not the
        # absolute number.
        for name, idx in (("front_left", ADC_DIST_FL),
                          ("front_right", ADC_DIST_FR),
                          ("back_left", ADC_DIST_BL),
                          ("back_right", ADC_DIST_BR)):
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = f"ir_{name}"
            r.radiation_type = Range.INFRARED
            r.field_of_view = 0.26
            r.min_range = self.ir_min
            r.max_range = self.ir_max
            counts = adc[idx]
            frac = 1.0 - (counts / 1023.0)
            r.range = float(self.ir_min + frac * (self.ir_max - self.ir_min))
            self.pub_ir[name].publish(r)

        u = Range()
        u.header.stamp = stamp
        u.header.frame_id = "ultrasonic"
        u.radiation_type = Range.ULTRASOUND
        u.field_of_view = 0.26
        u.min_range = 0.02
        u.max_range = 4.0
        u.range = float(t["ultrasonic"])
        self.pub_ultra.publish(u)

    def publish_buttons(self, t, stamp):
        b = Buttons()
        b.header.stamp = stamp
        mask = t["buttons"]
        b.sw1 = bool(mask & 1)
        b.sw2 = bool(mask & 2)
        b.sw3 = bool(mask & 4)
        b.sw4 = bool(mask & 8)
        self.pub_buttons.publish(b)

    def publish_battery(self, t, stamp):
        """Supply as a BatteryState.

        `voltage` is always the Pi's 5 V input rail (PMIC EXT5V_V): it is the
        rail that collapses in a brownout, so it drives the health field.
        `percentage` comes from battery_source -- an estimate from that same
        rail by default, or a real pack voltage from an ADC divider.
        """
        b = BatteryState()
        b.header.stamp = stamp
        b.header.frame_id = "base_link"
        v = float(t["supply_v"])
        self.supply_v = v
        self.supply_a = float(t["supply_a"])
        b.voltage = v
        b.current = -float(t["supply_a"])   # REP-147: discharge is negative
        b.charge = float("nan")
        b.capacity = float("nan")
        b.design_capacity = float("nan")
        b.percentage = self.battery_fraction(t)
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        b.present = v == v and v > 1.0

        if v != v:
            b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif v < self.supply_critical_v:
            # Not "dead" in the battery sense -- this is the rail about to drop
            # the Pi, which is what the operator needs to see.
            b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif v < self.supply_warn_v:
            # BatteryState has no "degraded" health, and every specific value
            # (OVERHEAT, OVERVOLTAGE, ...) would be a lie. UNKNOWN is honest;
            # the warning that matters is logged below and carried by the
            # FLAG_LOW_VOLTAGE bit in /fossbot/link_status.
            b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        b.location = ("pack_via_adc" if self.battery_source == "adc"
                      else "estimated_from_pi5_5v_rail")
        self.pub_battery.publish(b)

        if bool(t["flags"] & FLAG_LOW_VOLTAGE):
            self.get_logger().warn(
                f"supply rail low: {v:.2f} V (warn below "
                f"{self.supply_warn_v:.2f}, resets near 4.4) -- charge the "
                f"battery before it browns out mid-drive",
                throttle_duration_sec=15.0)

    def battery_fraction(self, t):
        """0..1 charge, NaN if unknown; smoothed (see battery.Smoother)."""
        if self.battery_source == "adc":
            if not 0 <= self.battery_adc_index < len(t["adc"]):
                return float("nan")
            v = t["adc"][self.battery_adc_index] / 1023.0 * 3.3 \
                * self.battery_divider_ratio
        else:
            v = float(t["supply_v"])
        if v != v or v <= 0.0:
            return float("nan")
        v = self.battery_smoother.update(v, time.monotonic())
        if self.battery_source == "adc":
            return liion_fraction(v / max(self.battery_cells, 1))
        return rail_fraction(v, self.battery_rail_empty_v, self.battery_rail_full_v)

    def publish_status(self):
        s = LinkStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.connected = (time.time() - self.last_rx_t) < 1.0
        s.telemetry_hz = len(self.rate_window) / 2.0 if self.rate_window else 0.0
        s.latency_ms = float(self.latency_ms)  # command -> robot -> telemetry
        s.frames_received = self.frames
        s.frames_lost = self.lost
        total = self.frames + self.lost
        s.loss_percent = (self.lost / total * 100.0) if total else 0.0
        s.motors_enabled = bool(self.flags & FLAG_MOTORS_ENABLED)
        s.estop = bool(self.flags & FLAG_ESTOP)
        s.imu_ok = bool(self.flags & FLAG_IMU_OK)
        s.lidar_ok = bool(self.flags & FLAG_LIDAR_OK)
        s.supply_volts = float(self.supply_v)
        s.supply_amps = float(self.supply_a)
        s.low_voltage = bool(self.flags & FLAG_LOW_VOLTAGE)
        s.robot_host = str(self.robot_host)
        s.duty_left = float(self.duty_l)
        s.duty_right = float(self.duty_r)
        self.pub_status.publish(s)

    # --- commands -------------------------------------------------------
    def on_cmd_vel(self, msg):
        with self.cmd_lock:
            self.pending_cmd = (CMD_TWIST, msg.linear.x, msg.angular.z)

    def on_motor_cmd(self, msg):
        with self.cmd_lock:
            self.pending_cmd = (CMD_DIRECT, msg.left, msg.right)

    def send_command(self):
        with self.cmd_lock:
            cmd_type, a, b = self.pending_cmd
        self.cmd_seq += 1
        frame = pack_command(self.cmd_seq, cmd_type, float(a), float(b))
        self.cmd_sent_at[self.cmd_seq] = time.monotonic()
        if len(self.cmd_sent_at) > 500:
            for k in sorted(self.cmd_sent_at)[:250]:
                del self.cmd_sent_at[k]
        try:
            self.tx_sock.sendto(frame, (self.robot_host, COMMAND_PORT))
        except OSError as exc:
            self.get_logger().warn(f"command send failed: {exc}", once=True)

    # --- services -------------------------------------------------------
    def on_set_rgb(self, req, resp):
        r = self.svc.call({"op": "set_rgb", "r": int(req.red),
                           "g": int(req.green), "b": int(req.blue)})
        resp.success = bool(r.get("ok"))
        resp.message = r.get("error", "")
        return resp

    def on_play_tone(self, req, resp):
        r = self.svc.call({"op": "play_tone", "freq": float(req.frequency),
                           "duration": float(req.duration)})
        resp.success = bool(r.get("ok"))
        resp.message = r.get("error", "")
        return resp

    def on_enable_motors(self, req, resp):
        r = self.svc.call({"op": "enable_motors", "enable": bool(req.data)})
        resp.success = bool(r.get("ok"))
        resp.message = f"motors_enabled={r.get('enabled')}" if r.get("ok") \
            else r.get("error", "")
        return resp

    def on_estop(self, req, resp):
        r = self.svc.call({"op": "estop", "engage": bool(req.data)})
        resp.success = bool(r.get("ok"))
        resp.message = f"estop={r.get('estop')}" if r.get("ok") \
            else r.get("error", "")
        return resp

    def on_reset_odom(self, req, resp):
        r = self.svc.call({"op": "reset_odometry"})
        self.x = self.y = self.yaw = 0.0
        self.last_enc = None
        self.wheel_pos_l = self.wheel_pos_r = 0.0
        resp.success = bool(r.get("ok"))
        resp.message = r.get("error", "")
        return resp

    def destroy_node(self):
        self.running = False
        try:
            self.tx_sock.sendto(pack_command(self.cmd_seq + 1, CMD_STOP, 0.0, 0.0),
                                (self.robot_host, COMMAND_PORT))
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FossbotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
