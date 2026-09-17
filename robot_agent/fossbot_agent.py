#!/usr/bin/env python3
"""FOSSBot robot-side agent. Runs on the Pi, inside the `fb` container.

Owns every piece of hardware and is the only process allowed to touch it. All
sensor reads happen locally in a tight loop (sub-millisecond over SPI/I2C) and
leave the robot as one batched frame, so the PC never pays a network round trip
per sensor.

Threads:
  sampler   reads ADC + IMU + encoders + buttons, sends a UDP telemetry frame
  commands  receives UDP command frames, applies them to the motors
  services  TCP request/response for discrete actions (LED, buzzer, odom reset)
  lidar     TCP push of complete scans
  camera    TCP push of JPEG frames from rpicam-vid
  power     polls the Pi's PMIC for the 5 V input rail
  watchdog  cuts the motors when commands stop arriving

Run:  python3 fossbot_agent.py [--pc-host 192.168.0.x]
The agent learns the PC's address from the first command frame it receives, so
--pc-host is only needed if you want telemetry before sending any command.
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lgpio
import spidev

from protocol import (
    TELEMETRY_PORT, COMMAND_PORT, SERVICE_PORT, LIDAR_PORT, CAMERA_PORT,
    pack_camera_header,
    TELEMETRY_HZ, COMMAND_TIMEOUT_S, N_ADC,
    pack_telemetry, unpack_command,
    CMD_TWIST, CMD_DIRECT, CMD_STOP,
    FLAG_MOTORS_ENABLED, FLAG_ESTOP, FLAG_LIDAR_OK, FLAG_IMU_OK,
    FLAG_LOW_VOLTAGE,
)
from pinmap import (
    I2C_BUS, MPU6050_ADDR,
    MOTOR_STBY, MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2,
    MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2,
    ENC_LEFT, ENC_RIGHT, LED_R, LED_G, LED_B, BUZZER, BUTTONS,
    ULTRA_A, ULTRA_B,
)

# On a Pi 5 the RP1 header GPIOs live on gpiochip4; gpiochip0 is root-only and
# opening it fails with 'can not open gpiochip'.
GPIO_CHIP = 4
PWM_FREQ = 1000

# Slots on each encoder disc. Matches the upstream fossbot library
# (fossbot_lib/real_robot/control.py: sensor_disc = 20) and was cross-checked
# against measured tick rates at known duty.
ENCODER_TICKS_PER_REV = 20

# Closed-loop wheel control, run locally at 50 Hz so wifi jitter can never
# destabilise it. Feedforward sets the duty, and a correction term removes the
# rest -- including the ~11% left/right motor mismatch measured on this robot.
#
# The correction tracks accumulated DISTANCE, not velocity. With 20 ticks/rev a
# velocity estimate over a 0.2 s window quantises to 0.055 m/s -- 46% of a
# 0.12 m/s setpoint -- so a velocity loop spends its whole time chasing whether
# one tick landed inside the window or not. Each wheel did that independently
# and they fought each other, which is exactly what "drives in waves" looks
# like. Accumulated tick counts have no such noise: they are exact. Integrating
# the commanded velocity and driving the position error to zero gives the same
# steady-state speed, keeps the two wheels in lockstep (so the robot holds a
# heading), and is inherently smooth.
# Gain is deliberately low, and there is a deadband, because the FEEDBACK is
# quantised: one tick is 11 mm of position. At POS_KP=5 that became a 0.055
# duty step every time a tick landed, and the loop limit-cycled at ~2.4 Hz --
# measured with the IMU gyro as 29 yaw-direction reversals in 6 s. Never react
# to error smaller than the sensor can resolve: inside one tick the controller
# simply does not know it is wrong.
# A FULL-tick deadband was too much: it cut chatter (yaw stdev 0.268 -> 0.114
# rad/s) but removed so much authority that heading drift went from +3.7 to
# -21.2 deg over 0.7 m, and low-speed commands risked not overcoming stiction
# at all. Half a tick keeps the sub-resolution chatter out without gutting the
# correction.
POS_KP = 3.0            # duty per metre of lag
POS_DEADBAND_TICKS = 0.5  # ignore error below this many encoder ticks
POS_CORR_ALPHA = 0.35   # low-pass on the correction, per control step
POS_ERR_CLAMP = 0.06    # m; bounds the correction if a wheel stalls or slips
VEL_WINDOW_S = 0.40     # only for reporting measured speed, not for control

# Gyro heading hold. The encoders cannot hold a heading -- 11 mm per tick is too
# coarse -- but the IMU measures yaw rate directly, continuously, and is already
# sampled at 100 Hz on the robot. Feeding (commanded yaw - measured yaw) into a
# differential duty correction is the right fix for weaving.
#
# DISABLED BY DEFAULT (gain 0) because the gyro's sign convention on this board
# has NOT been verified against physical rotation yet -- the test that would have
# confirmed it was cut short when the robot browned out. A wrong sign turns this
# into positive feedback and makes weaving far worse.
#
# To enable: verify the sign first with
#     ros2 run fossbot_bridge gyro_sign_check
# which commands a slow left turn and reports the measured yaw sign. If it
# reports NEGATIVE, set GYRO_SIGN = -1. Then set GYRO_HEADING_KP to ~0.25.
GYRO_HEADING_KP = 0.0
GYRO_SIGN = 1.0
GYRO_BIAS_ALPHA = 0.002   # slow bias tracking, only while stationary

# Physical constants, measured from the v2 URDF meshes. WHEEL_TRACK is the
# distance between the wheel *centre planes* (+-93.3 mm), not between the URDF
# joint origins (+-77.9 mm) -- using the latter puts ~17% error into rotation.
WHEEL_RADIUS = 0.03524
WHEEL_TRACK = 0.1866

# Camera. Ubuntu's libcamera cannot drive a Pi 5 camera, so the stack is built
# from source by build_camera_stack.sh and installed under this prefix, which
# lives on the robot's real filesystem and survives container recreation.
CAMERA_PREFIX = "/ws/opt/camera"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
CAMERA_QUALITY = 80

# The TB6612's IN1/IN2 polarity versus which way the wheel physically turns
# depends on how the motor leads are soldered, and on this robot both are
# reversed: commanding forward drove the robot backwards AND mirrored every
# turn. Those two symptoms together are the signature of exactly this fault --
# negating both wheel velocities gives v' = -v and w' = -w at the same time.
# A left/right channel swap would mirror only the turns, and a 180 deg frame
# error would mirror only forward/back, so neither fits.
#
# Flip these if a rebuilt robot drives the other way.
MOTOR_LEFT_INVERT = True
MOTOR_RIGHT_INVERT = True

# Slew-rate limit on duty, and a cap below 100%.
#
# This robot resets under motor load: after a drop-out `vcgencmd get_throttled`
# reads 0x0 with an uptime of seconds, which is a full power loss rather than
# undervoltage throttling -- a throttle event would stay up and latch bit 0/16,
# but the SoC cannot record anything if the rail collapses. Stepping the duty
# straight to its target draws peak inrush from a stalled rotor, so ramp
# instead. This is a mitigation, not a cure: the real fix is a supply that can
# hold up under motor current.
DUTY_SLEW_PER_S = 2.5   # full-scale duty change per second (0 -> 1 in 0.4 s)
DUTY_MAX = 0.85

# Supply monitoring. There is no battery divider on this PCB (all spare ADC
# channels read ~0), so the measurement comes from the Pi 5's own PMIC:
# `vcgencmd pmic_read_adc` reports EXT5V_V, the 5 V rail as seen at the Pi.
#
# Note what this is and is not. It is the rail that collapses during a brownout,
# so it is exactly the right thing to watch for the resets this robot suffers.
# It is NOT battery state of charge: if a regulator sits between the cells and
# the Pi, this reads flat until the battery can no longer hold it up.
#
# Pi 5 flags undervoltage around 4.63 V; below roughly 4.4 V it resets.
SUPPLY_POLL_S = 0.5
SUPPLY_WARN_V = 4.75
SUPPLY_CRITICAL_V = 4.63

# Feedforward only: duty = target / MAX_LINEAR, with the PI term removing
# whatever is left. Derived from a measured 40% duty giving ~0.58 m/s with the
# wheels free, then derated for rolling load. It does not need to be exact --
# a wrong value costs settling time, not steady-state accuracy.
MAX_LINEAR = 0.80


class Hardware:
    """All hardware state, guarded by one lock."""

    def __init__(self, claim_timeout=15.0):
        self.lock = threading.Lock()
        self.h = lgpio.gpiochip_open(GPIO_CHIP)

        # The kernel releases a dead process's GPIO lines asynchronously, so
        # restarting the agent immediately after killing it hits 'GPIO busy'.
        # Retry until the old owner is reaped rather than guessing at a sleep.
        deadline = time.monotonic() + claim_timeout
        while True:
            try:
                for pin in (MOTOR_STBY, MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2,
                            MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2,
                            LED_R, LED_G, LED_B, BUZZER):
                    lgpio.gpio_claim_output(self.h, pin, 0)
                for name, pin in BUTTONS.items():
                    lgpio.gpio_claim_input(self.h, pin, lgpio.SET_PULL_UP)
                break
            except lgpio.error as exc:
                if "busy" not in str(exc).lower() or time.monotonic() > deadline:
                    raise
                print(f"[agent] GPIO busy, waiting for previous owner to exit",
                      flush=True)
                time.sleep(1.0)

        # Encoders are single-channel slotted discs: they count edges but carry
        # no direction. Sign comes from the commanded motor direction below.
        #
        # No debounce filter: tick rate was measured at 0, 200, 1000 and 2000 us
        # of debounce at a fixed duty and varied by under 1%, so the optical
        # signal is clean and a filter would only risk dropping real ticks.
        for pin in (ENC_LEFT, ENC_RIGHT):
            lgpio.gpio_claim_alert(self.h, pin, lgpio.RISING_EDGE,
                                   lgpio.SET_PULL_UP)
        self.cb_left = lgpio.callback(self.h, ENC_LEFT, lgpio.RISING_EDGE)
        self.cb_right = lgpio.callback(self.h, ENC_RIGHT, lgpio.RISING_EDGE)
        self._last_tally_l = 0
        self._last_tally_r = 0
        self.enc_left = 0
        self.enc_right = 0

        self.spi_u8 = self._open_spi(0)
        self.spi_u7 = self._open_spi(1)

        self.i2c = None
        self.imu_ok = False
        try:
            self.i2c = lgpio.i2c_open(I2C_BUS, MPU6050_ADDR)
            # Wake from sleep, then set ranges: accel +-2g, gyro +-250 deg/s.
            lgpio.i2c_write_byte_data(self.i2c, 0x6B, 0x00)
            time.sleep(0.05)
            lgpio.i2c_write_byte_data(self.i2c, 0x1C, 0x00)
            lgpio.i2c_write_byte_data(self.i2c, 0x1B, 0x00)
            self.imu_ok = True
        except Exception as exc:
            print(f"[agent] IMU init failed: {exc}", flush=True)

        self.motors_enabled = False
        self.estop = False
        self.duty_left = 0.0
        self.duty_right = 0.0
        self.dir_left = 1
        self.dir_right = 1
        self.last_command_t = 0.0
        self.lidar_ok = False
        self.camera_ok = False

        # Closed-loop velocity state.
        self.closed_loop = True
        self.target_l = 0.0        # m/s at the wheel
        self.target_r = 0.0
        self.meas_l = 0.0          # measured wheel speed, m/s (telemetry only)
        self.meas_r = 0.0
        self.des_l = 0.0           # commanded distance, integrated, m
        self.des_r = 0.0
        self.ref_enc_l = 0         # encoder datum the desired distance runs from
        self.ref_enc_r = 0
        self.corr_l = 0.0          # smoothed correction terms
        self.corr_r = 0.0
        self.gyro_z = 0.0          # latest yaw rate, rad/s
        self.gyro_bias = 0.0
        self.supply_v = float("nan")
        self.supply_a = float("nan")
        # Lowest rail voltage seen since the last reset of this figure. The sag
        # under motor load is the number that predicts a brownout, and an
        # instantaneous reading at 2 Hz will usually miss it.
        self.supply_v_min = float("nan")
        self._last_slew_t = time.monotonic()
        self._hist = deque(maxlen=256)   # (t, enc_left, enc_right)
        # Latest ultrasonic reading, refreshed by its own thread. The HC-SR04
        # blocks for the full timeout when nothing echoes back, which would
        # otherwise stall the sampler loop and drag the telemetry rate down.
        self.ultrasonic = float("nan")

    @staticmethod
    def _open_spi(ce):
        s = spidev.SpiDev()
        s.open(0, ce)
        s.max_speed_hz = 1350000
        s.mode = 0
        return s

    # --- sensors ---------------------------------------------------------
    def read_adc(self):
        out = []
        for spi in (self.spi_u8, self.spi_u7):
            for ch in range(8):
                r = spi.xfer2([1, (8 + ch) << 4, 0])
                out.append(((r[1] & 3) << 8) | r[2])
        return out

    def read_imu(self):
        """Returns (ax, ay, az, gx, gy, gz, temp_c) in m/s^2 and rad/s."""
        if not self.imu_ok:
            return (0.0,) * 6 + (0.0,)
        try:
            _, data = lgpio.i2c_read_i2c_block_data(self.i2c, 0x3B, 14)
        except Exception:
            return (0.0,) * 6 + (0.0,)

        def s16(hi, lo):
            v = (data[hi] << 8) | data[lo]
            return v - 65536 if v >= 32768 else v

        # +-2g -> 16384 LSB/g ; +-250 deg/s -> 131 LSB/(deg/s)
        ax = s16(0, 1) / 16384.0 * 9.80665
        ay = s16(2, 3) / 16384.0 * 9.80665
        az = s16(4, 5) / 16384.0 * 9.80665
        temp = s16(6, 7) / 340.0 + 36.53
        gx = math.radians(s16(8, 9) / 131.0)
        gy = math.radians(s16(10, 11) / 131.0)
        gz = math.radians(s16(12, 13) / 131.0)
        return ax, ay, az, gx, gy, gz, temp

    def read_buttons(self):
        """Bitmask, bit0..bit3 = SW1..SW4. Buttons are active low."""
        mask = 0
        for i, name in enumerate(("SW1", "SW2", "SW3", "SW4")):
            if lgpio.gpio_read(self.h, BUTTONS[name]) == 0:
                mask |= 1 << i
        return mask

    def update_encoders(self):
        """Fold raw tallies into signed cumulative counts.

        The discs give no direction, so each tick is signed by the direction the
        motor was last commanded to turn. Counts are therefore only meaningful
        while the wheel actually follows the command -- a wheel pushed backwards
        by hand still counts up.
        """
        tl, tr = self.cb_left.tally(), self.cb_right.tally()
        dl, dr = tl - self._last_tally_l, tr - self._last_tally_r
        self._last_tally_l, self._last_tally_r = tl, tr
        self.enc_left += dl * self.dir_left
        self.enc_right += dr * self.dir_right
        return self.enc_left, self.enc_right

    def read_ultrasonic(self, timeout=0.02):
        """HC-SR04 on ULTRA_A=trig, ULTRA_B=echo. NaN when no echo."""
        try:
            lgpio.gpio_claim_output(self.h, ULTRA_A, 0)
            lgpio.gpio_claim_input(self.h, ULTRA_B)
            lgpio.gpio_write(self.h, ULTRA_A, 1)
            time.sleep(1e-5)
            lgpio.gpio_write(self.h, ULTRA_A, 0)

            deadline = time.monotonic() + timeout
            while lgpio.gpio_read(self.h, ULTRA_B) == 0:
                if time.monotonic() > deadline:
                    return float("nan")
            t0 = time.monotonic()
            while lgpio.gpio_read(self.h, ULTRA_B) == 1:
                if time.monotonic() > deadline:
                    return float("nan")
            return (time.monotonic() - t0) * 343.0 / 2.0
        except Exception:
            return float("nan")

    # --- actuators -------------------------------------------------------
    def _apply_motor(self, duty, pwm, in1, in2, invert=False):
        """duty in -1..1. Returns the direction sign actually applied.

        `invert` flips the IN1/IN2 pair so that a positive duty always means the
        wheel drives the robot forwards, whatever way the motor is wired.
        """
        if abs(duty) < 1e-3:
            lgpio.tx_pwm(self.h, pwm, PWM_FREQ, 0)
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 0)
            return 1
        fwd = (duty > 0) != invert
        # No stiction floor: remapping the magnitude into a [floor, 1] band
        # would put a nonlinearity inside the PI loop and set a high minimum
        # speed. The integrator walks the duty up through stiction instead.
        mag = min(abs(duty), 1.0)
        lgpio.gpio_write(self.h, in1, 1 if fwd else 0)
        lgpio.gpio_write(self.h, in2, 0 if fwd else 1)
        lgpio.tx_pwm(self.h, pwm, PWM_FREQ, mag * 100.0)
        return 1 if fwd else -1

    def set_motors(self, left, right, slew=True):
        with self.lock:
            if self.estop or not self.motors_enabled:
                left = right = 0.0
                slew = False          # stopping must be immediate
            left = max(-DUTY_MAX, min(DUTY_MAX, left))
            right = max(-DUTY_MAX, min(DUTY_MAX, right))

            if slew:
                now = time.monotonic()
                dt = now - self._last_slew_t
                self._last_slew_t = now
                if 0.0 < dt < 0.5:
                    step = DUTY_SLEW_PER_S * dt
                    left = self.duty_left + max(-step, min(step, left - self.duty_left))
                    right = self.duty_right + max(-step, min(step, right - self.duty_right))
            else:
                self._last_slew_t = time.monotonic()

            self.duty_left, self.duty_right = left, right
            lgpio.gpio_write(self.h, MOTOR_STBY,
                             1 if (self.motors_enabled and not self.estop) else 0)
            # Encoder ticks are signed by the COMMANDED direction, so use the
            # sign of the request, not the electrical direction the invert
            # produced -- otherwise odometry would run backwards.
            self._apply_motor(left, MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2,
                              MOTOR_LEFT_INVERT)
            self._apply_motor(right, MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2,
                              MOTOR_RIGHT_INVERT)
            self.dir_left = 1 if left >= 0 else -1
            self.dir_right = 1 if right >= 0 else -1

    def set_twist(self, v, w):
        """Differential drive. v m/s, w rad/s -> per-wheel target speed."""
        vl = v - w * WHEEL_TRACK / 2.0
        vr = v + w * WHEEL_TRACK / 2.0
        if self.closed_loop:
            with self.lock:
                if (vl == 0.0 and vr == 0.0) or \
                        (vl * self.target_l < 0) or (vr * self.target_r < 0):
                    # Stopping or reversing: re-datum so accumulated lag from
                    # the previous move cannot kick the wheels the wrong way.
                    self._reset_tracking()
                self.target_l, self.target_r = vl, vr
            if vl == 0.0 and vr == 0.0:
                self.set_motors(0.0, 0.0, slew=False)
        else:
            self.set_motors(max(-1.0, min(1.0, vl / MAX_LINEAR)),
                            max(-1.0, min(1.0, vr / MAX_LINEAR)))

    def measure_speeds(self, now, enc_l, enc_r):
        """Wheel speed in m/s from a ~VEL_WINDOW_S window of encoder ticks."""
        self._hist.append((now, enc_l, enc_r))
        cutoff = now - VEL_WINDOW_S
        ref = None
        for sample in self._hist:
            if sample[0] >= cutoff:
                ref = sample
                break
        if ref is None or ref[0] >= now:
            return
        dt = now - ref[0]
        m_per_tick = 2.0 * math.pi * WHEEL_RADIUS / ENCODER_TICKS_PER_REV
        self.meas_l = (enc_l - ref[1]) * m_per_tick / dt
        self.meas_r = (enc_r - ref[2]) * m_per_tick / dt

    def _reset_tracking(self):
        """Re-datum the distance tracker. Caller must hold self.lock."""
        self.des_l = 0.0
        self.des_r = 0.0
        self.ref_enc_l = self.enc_left
        self.ref_enc_r = self.enc_right
        self.corr_l = 0.0
        self.corr_r = 0.0

    def velocity_step(self, dt):
        """One control step. Runs on the robot, never across the network."""
        if not self.closed_loop or not self.motors_enabled or self.estop:
            return
        with self.lock:
            tl, tr = self.target_l, self.target_r
            if tl == 0.0 and tr == 0.0:
                self._reset_tracking()
                return

            m_per_tick = 2.0 * math.pi * WHEEL_RADIUS / ENCODER_TICKS_PER_REV
            self.des_l += tl * dt
            self.des_r += tr * dt
            act_l = (self.enc_left - self.ref_enc_l) * m_per_tick
            act_r = (self.enc_right - self.ref_enc_r) * m_per_tick

            err_l = max(-POS_ERR_CLAMP, min(POS_ERR_CLAMP, self.des_l - act_l))
            err_r = max(-POS_ERR_CLAMP, min(POS_ERR_CLAMP, self.des_r - act_r))
            # Clamping the error is also the anti-windup: a stalled wheel cannot
            # accumulate an unbounded demand that slams the duty on release.
            self.des_l = act_l + err_l
            self.des_r = act_r + err_r

            dead = POS_DEADBAND_TICKS * m_per_tick
            eff_l = math.copysign(max(0.0, abs(err_l) - dead), err_l)
            eff_r = math.copysign(max(0.0, abs(err_r) - dead), err_r)

            # Smooth what is left: the residual still steps by a tick at a time.
            self.corr_l += POS_CORR_ALPHA * (POS_KP * eff_l - self.corr_l)
            self.corr_r += POS_CORR_ALPHA * (POS_KP * eff_r - self.corr_r)

            duty_l = tl / MAX_LINEAR + self.corr_l
            duty_r = tr / MAX_LINEAR + self.corr_r

            if GYRO_HEADING_KP > 0.0:
                # Differential correction: speed up one wheel and slow the
                # other by the same amount, so heading is corrected without
                # disturbing forward speed.
                w_target = (tr - tl) / WHEEL_TRACK
                w_meas = GYRO_SIGN * (self.gyro_z - self.gyro_bias)
                d = GYRO_HEADING_KP * (w_target - w_meas)
                duty_l -= d
                duty_r += d

        self.set_motors(max(-1.0, min(1.0, duty_l)),
                        max(-1.0, min(1.0, duty_r)))

    def set_rgb(self, r, g, b):
        with self.lock:
            lgpio.gpio_write(self.h, LED_R, 1 if r else 0)
            lgpio.gpio_write(self.h, LED_G, 1 if g else 0)
            lgpio.gpio_write(self.h, LED_B, 1 if b else 0)

    def play_tone(self, freq, duration):
        def run():
            with self.lock:
                if freq <= 0:
                    lgpio.tx_pwm(self.h, BUZZER, PWM_FREQ, 0)
                    lgpio.gpio_write(self.h, BUZZER, 0)
                    return
                lgpio.tx_pwm(self.h, BUZZER, freq, 50)
            time.sleep(max(0.0, min(duration, 10.0)))
            with self.lock:
                lgpio.tx_pwm(self.h, BUZZER, freq, 0)
                lgpio.gpio_write(self.h, BUZZER, 0)
        threading.Thread(target=run, daemon=True).start()

    def close(self):
        try:
            self.set_motors(0.0, 0.0)
            lgpio.gpio_write(self.h, MOTOR_STBY, 0)
            self.set_rgb(0, 0, 0)
            lgpio.tx_pwm(self.h, BUZZER, PWM_FREQ, 0)
            self.cb_left.cancel()
            self.cb_right.cancel()
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(self.h)
        except Exception:
            pass


class Agent:
    def __init__(self, pc_host=None):
        self.hw = Hardware()
        self.pc_host = pc_host
        self.pc_lock = threading.Lock()
        self.running = True
        self.seq = 0
        self.last_cmd_seq = -1
        self.cmd_echo = 0
        self.stats = {"tx": 0, "rx": 0, "cmd_stale": 0}

        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cmd_sock.bind(("0.0.0.0", COMMAND_PORT))
        self.cmd_sock.settimeout(0.2)

    # --- threads ---------------------------------------------------------
    def sampler_loop(self):
        period = 1.0 / TELEMETRY_HZ
        next_t = time.monotonic()
        while self.running:
            next_t += period
            adc = self.hw.read_adc()
            imu = self.hw.read_imu()
            el, er = self.hw.update_encoders()
            self.hw.measure_speeds(time.monotonic(), el, er)
            self.hw.gyro_z = imu[5]
            if self.hw.target_l == 0.0 and self.hw.target_r == 0.0 and \
                    self.hw.duty_left == 0.0 and self.hw.duty_right == 0.0:
                # Only while truly stopped, or the bias absorbs real rotation.
                self.hw.gyro_bias += GYRO_BIAS_ALPHA * (imu[5] - self.hw.gyro_bias)
            buttons = self.hw.read_buttons()

            flags = 0
            if self.hw.motors_enabled:
                flags |= FLAG_MOTORS_ENABLED
            if self.hw.estop:
                flags |= FLAG_ESTOP
            if self.hw.lidar_ok:
                flags |= FLAG_LIDAR_OK
            if self.hw.imu_ok:
                flags |= FLAG_IMU_OK
            v = self.hw.supply_v
            if v == v and v < SUPPLY_WARN_V:
                flags |= FLAG_LOW_VOLTAGE

            self.seq += 1
            frame = pack_telemetry(
                self.seq, self.cmd_echo, time.time(), el, er, adc,
                imu[:6], imu[6], self.hw.ultrasonic,
                self.hw.duty_left, self.hw.duty_right,
                self.hw.supply_v, self.hw.supply_a, buttons, flags)

            with self.pc_lock:
                host = self.pc_host
            if host:
                try:
                    self.tx_sock.sendto(frame, (host, TELEMETRY_PORT))
                    self.stats["tx"] += 1
                except OSError:
                    pass

            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def command_loop(self):
        while self.running:
            try:
                data, addr = self.cmd_sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            cmd = unpack_command(data)
            if cmd is None:
                continue

            # Learn where to send telemetry from whoever is commanding us.
            with self.pc_lock:
                if self.pc_host != addr[0]:
                    self.pc_host = addr[0]
                    print(f"[agent] telemetry -> {addr[0]}", flush=True)

            # Drop reordered stragglers, allowing for sequence wrap.
            if cmd["seq"] <= self.last_cmd_seq and \
                    self.last_cmd_seq - cmd["seq"] < 1000:
                self.stats["cmd_stale"] += 1
                continue
            self.last_cmd_seq = cmd["seq"]
            self.cmd_echo = cmd["seq"]
            self.stats["rx"] += 1
            self.hw.last_command_t = time.monotonic()

            if cmd["type"] == CMD_TWIST:
                self.hw.set_twist(cmd["a"], cmd["b"])
            elif cmd["type"] == CMD_DIRECT:
                # Raw duty: suspend the tracking loop so it cannot fight the
                # caller. Still slew-limited -- the ramp is there to protect the
                # supply, and direct control has no reason to bypass it.
                with self.hw.lock:
                    self.hw.target_l = self.hw.target_r = 0.0
                    self.hw._reset_tracking()
                self.hw.set_motors(cmd["a"], cmd["b"])
            elif cmd["type"] == CMD_STOP:
                with self.hw.lock:
                    self.hw.target_l = self.hw.target_r = 0.0
                    self.hw._reset_tracking()
                self.hw.set_motors(0.0, 0.0, slew=False)

    def control_loop(self):
        """PI velocity loop at 50 Hz, entirely local to the robot."""
        period = 0.02
        last = time.monotonic()
        while self.running:
            time.sleep(period)
            now = time.monotonic()
            dt = now - last
            last = now
            if dt <= 0 or dt > 0.5:
                continue
            try:
                self.hw.velocity_step(dt)
            except Exception as exc:
                print(f"[agent] control step failed: {exc}", flush=True)

    def supply_loop(self):
        """Poll the PMIC for the 5 V input rail.

        vcgencmd is a subprocess, far too slow for the sampler thread, so it
        lives here at 2 Hz and the sampler just reports the latest value.
        """
        import subprocess
        while self.running:
            try:
                out = subprocess.run(["vcgencmd", "pmic_read_adc"],
                                     capture_output=True, text=True, timeout=4)
                volts, amps = float("nan"), 0.0
                for line in out.stdout.splitlines():
                    line = line.strip()
                    if line.startswith("EXT5V_V"):
                        volts = float(line.split("=")[1].rstrip("V"))
                    elif "current(" in line and line.endswith("A"):
                        try:
                            amps += float(line.split("=")[1].rstrip("A"))
                        except ValueError:
                            pass
                self.hw.supply_v = volts
                self.hw.supply_a = amps
                if volts == volts:      # not NaN
                    lo = self.hw.supply_v_min
                    if lo != lo or volts < lo:
                        self.hw.supply_v_min = volts
            except Exception as exc:
                print(f"[agent] supply poll failed: {exc}", flush=True)
            time.sleep(SUPPLY_POLL_S)

    def ultrasonic_loop(self):
        """Polls the HC-SR04 at 10 Hz off the critical path."""
        while self.running:
            self.hw.ultrasonic = self.hw.read_ultrasonic()
            time.sleep(0.1)

    def watchdog_loop(self):
        while self.running:
            time.sleep(0.05)
            if self.hw.duty_left == 0.0 and self.hw.duty_right == 0.0:
                continue
            if time.monotonic() - self.hw.last_command_t > COMMAND_TIMEOUT_S:
                print("[agent] command timeout -- stopping motors", flush=True)
                with self.hw.lock:
                    self.hw.target_l = self.hw.target_r = 0.0
                    self.hw._reset_tracking()
                self.hw.set_motors(0.0, 0.0, slew=False)

    def service_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", SERVICE_PORT))
        srv.listen(4)
        srv.settimeout(0.5)
        while self.running:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve_client, args=(conn,),
                             daemon=True).start()
        srv.close()

    def _serve_client(self, conn):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        f = conn.makefile("r")
        try:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    req = json.loads(line)
                except ValueError:
                    continue
                resp = self.handle_service(req)
                conn.sendall((json.dumps(resp) + "\n").encode())
        except OSError:
            pass
        finally:
            conn.close()

    def handle_service(self, req):
        op = req.get("op")
        try:
            if op == "ping":
                return {"ok": True, "t_robot": time.time()}
            if op == "set_rgb":
                self.hw.set_rgb(req.get("r", 0), req.get("g", 0),
                                req.get("b", 0))
                return {"ok": True}
            if op == "play_tone":
                self.hw.play_tone(float(req.get("freq", 1000)),
                                  float(req.get("duration", 0.2)))
                return {"ok": True}
            if op == "enable_motors":
                self.hw.motors_enabled = bool(req.get("enable", False))
                if not self.hw.motors_enabled:
                    self.hw.set_motors(0.0, 0.0, slew=False)
                return {"ok": True, "enabled": self.hw.motors_enabled}
            if op == "estop":
                self.hw.estop = bool(req.get("engage", True))
                self.hw.set_motors(0.0, 0.0, slew=False)
                return {"ok": True, "estop": self.hw.estop}
            if op == "closed_loop":
                self.hw.closed_loop = bool(req.get("enable", True))
                with self.hw.lock:
                    self.hw._reset_tracking()
                return {"ok": True, "closed_loop": self.hw.closed_loop}
            if op == "reset_supply_min":
                self.hw.supply_v_min = float("nan")
                return {"ok": True}
            if op == "reset_odometry":
                self.hw.enc_left = 0
                self.hw.enc_right = 0
                return {"ok": True}
            if op == "status":
                return {
                    "ok": True,
                    "motors_enabled": self.hw.motors_enabled,
                    "estop": self.hw.estop,
                    "imu_ok": self.hw.imu_ok,
                    "lidar_ok": self.hw.lidar_ok,
                    "camera_ok": self.hw.camera_ok,
                    "supply_v": self.hw.supply_v,
                    "supply_a": self.hw.supply_a,
                    "supply_v_min": self.hw.supply_v_min,
                    "closed_loop": self.hw.closed_loop,
                    "target_mps": [round(self.hw.target_l, 3),
                                   round(self.hw.target_r, 3)],
                    "measured_mps": [round(self.hw.meas_l, 3),
                                     round(self.hw.meas_r, 3)],
                    "stats": dict(self.stats),
                }
            return {"ok": False, "error": f"unknown op {op!r}"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def camera_loop(self):
        """Serve MJPEG frames to whoever connects, one client at a time."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", CAMERA_PORT))
        srv.listen(1)
        srv.settimeout(0.5)
        while self.running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[agent] camera client {addr}", flush=True)
            self._stream_camera(conn)
        srv.close()

    def _stream_camera(self, conn):
        """Pipe rpicam-vid's MJPEG output to the client as framed JPEGs.

        rpicam-vid is used rather than the libcamera API directly because it
        already runs the full ISP -- auto exposure, auto white balance and
        autofocus -- which raw V4L2 capture off the CSI does not.
        """
        import subprocess

        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = f"{CAMERA_PREFIX}/lib:" + env.get("LD_LIBRARY_PATH", "")
        # The IPA modules install one level deeper than the pipeline handlers;
        # pointing at lib/libcamera gives "No IPA found" and no auto-exposure.
        env["LIBCAMERA_IPA_MODULE_PATH"] = f"{CAMERA_PREFIX}/lib/libcamera/ipa"

        cmd = [
            f"{CAMERA_PREFIX}/bin/rpicam-vid",
            "--codec", "mjpeg",
            "--width", str(CAMERA_WIDTH), "--height", str(CAMERA_HEIGHT),
            "--framerate", str(CAMERA_FPS),
            "--quality", str(CAMERA_QUALITY),
            "--timeout", "0",              # run until killed
            "--nopreview",
            "--output", "-",
        ]
        proc = None
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, env=env, bufsize=0)
            self.hw.camera_ok = True
            seq = 0
            buf = bytearray()
            while self.running:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf.extend(chunk)
                # MJPEG is a bare concatenation of JPEGs: split on SOI..EOI.
                while True:
                    start = buf.find(b"\xff\xd8")
                    if start < 0:
                        if len(buf) > 1:
                            del buf[:-1]
                        break
                    end = buf.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        if start:
                            del buf[:start]
                        break
                    frame = bytes(buf[start:end + 2])
                    del buf[:end + 2]
                    seq += 1
                    header = pack_camera_header(seq, time.time(), CAMERA_WIDTH,
                                                CAMERA_HEIGHT, len(frame))
                    try:
                        conn.sendall(header + frame)
                    except OSError:
                        return
        except FileNotFoundError:
            print(f"[agent] camera: rpicam-vid not found under {CAMERA_PREFIX}. "
                  f"Run build_camera_stack.sh.", flush=True)
        except Exception as exc:
            print(f"[agent] camera stopped: {exc}", flush=True)
        finally:
            self.hw.camera_ok = False
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=3)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            conn.close()

    def lidar_loop(self):
        """Serves complete scans to whoever connects. Lidar is opened lazily so
        the agent still runs with no lidar plugged in."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", LIDAR_PORT))
        srv.listen(1)
        srv.settimeout(0.5)
        while self.running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[agent] lidar client {addr}", flush=True)
            self._stream_lidar(conn)
        srv.close()

    def _open_lidar(self, attempts=3):
        """Open the RPLIDAR, recovering from a dirty serial state.

        A session that did not shut down cleanly leaves scan bytes in the UART
        buffer; the next command then reads them as a malformed reply
        ("Incorrect descriptor starting bytes", "Descriptor length mismatch").
        Stop the device, drain, reset, drain again, and confirm with get_info
        before handing it to the caller.
        """
        from rplidar import RPLidar
        for attempt in range(1, attempts + 1):
            lidar = None
            try:
                lidar = RPLidar("/dev/ttyUSB0")
                for fn in (lidar.stop, lidar.stop_motor):
                    try:
                        fn()
                    except Exception:
                        pass
                time.sleep(0.3)
                lidar.clean_input()
                lidar.reset()
                time.sleep(2.0)          # A1 reset takes about two seconds
                lidar.clean_input()
                info = lidar.get_info()  # proves the link is sane again
                print(f"[agent] lidar ready: {info}", flush=True)
                return lidar
            except Exception as exc:
                print(f"[agent] lidar open attempt {attempt}/{attempts} "
                      f"failed: {exc}", flush=True)
                if lidar is not None:
                    try:
                        lidar.disconnect()
                    except Exception:
                        pass
                time.sleep(1.5)
        return None

    def _stream_lidar(self, conn):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        lidar = None
        try:
            from rplidar import RPLidar
            lidar = self._open_lidar()
            if lidar is None:
                return
            self.hw.lidar_ok = True
            for scan in lidar.iter_scans(max_buf_meas=3000):
                if not self.running:
                    break
                payload = {
                    "t": time.time(),
                    "pts": [[round(a, 2), round(d, 1)] for _, a, d in scan],
                }
                conn.sendall((json.dumps(payload) + "\n").encode())
        except Exception as exc:
            print(f"[agent] lidar stopped: {exc}", flush=True)
        finally:
            self.hw.lidar_ok = False
            if lidar is not None:
                for fn in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                    try:
                        fn()
                    except Exception:
                        pass
            conn.close()

    def run(self):
        threads = [
            threading.Thread(target=self.sampler_loop, daemon=True),
            threading.Thread(target=self.command_loop, daemon=True),
            threading.Thread(target=self.control_loop, daemon=True),
            threading.Thread(target=self.supply_loop, daemon=True),
            threading.Thread(target=self.ultrasonic_loop, daemon=True),
            threading.Thread(target=self.watchdog_loop, daemon=True),
            threading.Thread(target=self.service_loop, daemon=True),
            threading.Thread(target=self.lidar_loop, daemon=True),
            threading.Thread(target=self.camera_loop, daemon=True),
        ]
        for t in threads:
            t.start()
        print(f"[agent] up. telemetry udp/{TELEMETRY_PORT} "
              f"commands udp/{COMMAND_PORT} services tcp/{SERVICE_PORT} "
              f"lidar tcp/{LIDAR_PORT} camera tcp/{CAMERA_PORT}", flush=True)
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[agent] shutting down", flush=True)
        finally:
            self.running = False
            time.sleep(0.3)
            self.hw.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pc-host", default=os.environ.get("FOSSBOT_PC_HOST"),
                    help="where to send telemetry before any command arrives")
    args = ap.parse_args()
    Agent(args.pc_host).run()


if __name__ == "__main__":
    main()
