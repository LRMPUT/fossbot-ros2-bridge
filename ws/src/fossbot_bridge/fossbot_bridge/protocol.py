"""Wire protocol shared by the robot-side agent and the PC-side ROS 2 bridge.

This file is the single source of truth for the link. It is deployed verbatim to
the robot (see deploy_agent.sh), so the two ends cannot drift apart.

Transport layout, chosen for latency rather than tidiness:

  UDP 5005  telemetry   robot -> PC    fixed-size struct, TELEMETRY_HZ
  UDP 5006  commands    PC -> robot    fixed-size struct, on demand
  TCP 5007  services    PC <-> robot   newline-delimited JSON, request/response
  TCP 5008  lidar       robot -> PC    newline-delimited JSON, one line per scan
  TCP 5009  camera      robot -> PC    length-prefixed JPEG frames

Why UDP for the hot paths: telemetry and velocity commands are a continuous
stream where the newest sample fully supersedes the previous one. TCP would add
head-of-line blocking -- one lost packet stalls every later sample until it is
retransmitted, which on wifi turns a 4 ms link into an occasional 200 ms one.
With UDP a lost frame is simply replaced by the next one 10 ms later. Every
frame carries a sequence number so the receiver can drop reordered stragglers
and count loss.

Services keep TCP because they are discrete, rare, and must not be lost: setting
an LED colour or resetting odometry has no "newer value" to fall back on. Lidar
keeps TCP because a scan is large and only useful complete.

Camera keeps TCP for the same reason as lidar, more so: a JPEG frame is tens of
kilobytes, far past the MTU, so UDP would mean hand-rolled fragmentation where a
single lost datagram destroys the whole frame. The agent drops frames rather
than letting them queue, so a slow link costs frame rate, never latency.
"""

import struct

TELEMETRY_PORT = 5005
COMMAND_PORT = 5006
SERVICE_PORT = 5007
LIDAR_PORT = 5008
CAMERA_PORT = 5009

TELEMETRY_HZ = 100.0

# Motors are cut if no command frame arrives within this window. The robot must
# coast to a stop when the link dies, not keep driving on the last command.
COMMAND_TIMEOUT_S = 0.5

TELEMETRY_MAGIC = 0x46425431  # 'FBT1'
COMMAND_MAGIC = 0x46424331  # 'FBC1'

N_ADC = 16

# --- telemetry: robot -> PC -------------------------------------------------
# < little endian (both ends are ARM/x86 LE; explicit so it stays true)
#   I  magic
#   I  seq
#   I  cmd_echo       seq of the last command frame applied, echoed back so the
#                     PC can measure true round-trip latency without the two
#                     clocks having to agree
#   d  t_robot        time.time() on the robot when the frame was sampled
#   i  enc_left       cumulative ticks, signed by commanded direction
#   i  enc_right
#   16H adc           raw MCP3008 counts, 0..1023, U8 ch0-7 then U7 ch0-7
#   6f imu            ax ay az (m/s^2), gx gy gz (rad/s)
#   f  imu_temp       degC
#   f  ultrasonic     metres, NaN when no echo
#   f  duty_left      applied duty, -1..1
#   f  duty_right
#   f  supply_v       5V input rail at the Pi, from the PMIC. NaN if unknown.
#   f  supply_a       total current drawn across the PMIC rails, amps
#   B  buttons        bit0..bit3 = SW1..SW4, 1 = pressed
#   B  flags
TELEMETRY_FMT = "<IIIdii16H6fffffffBB"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FMT)

FLAG_MOTORS_ENABLED = 1 << 0
FLAG_ESTOP = 1 << 1
FLAG_LIDAR_OK = 1 << 2
FLAG_IMU_OK = 1 << 3
FLAG_LOW_VOLTAGE = 1 << 4

# --- commands: PC -> robot --------------------------------------------------
#   I  magic
#   I  seq
#   B  type
#   f  a
#   f  b
COMMAND_FMT = "<IIBff"
COMMAND_SIZE = struct.calcsize(COMMAND_FMT)

CMD_TWIST = 0      # a = linear.x m/s, b = angular.z rad/s
CMD_DIRECT = 1     # a = left duty -1..1, b = right duty -1..1
CMD_STOP = 2       # a, b ignored


def pack_telemetry(seq, cmd_echo, t_robot, enc_left, enc_right, adc, imu,
                   imu_temp, ultrasonic, duty_left, duty_right, supply_v,
                   supply_a, buttons, flags):
    """adc: 16 ints 0..1023. imu: (ax, ay, az, gx, gy, gz)."""
    return struct.pack(TELEMETRY_FMT, TELEMETRY_MAGIC, seq & 0xFFFFFFFF,
                       cmd_echo & 0xFFFFFFFF, t_robot, enc_left, enc_right,
                       *adc, *imu, imu_temp, ultrasonic, duty_left, duty_right,
                       supply_v, supply_a, buttons, flags)


def unpack_telemetry(buf):
    """Returns a dict, or None if the frame is not a well-formed telemetry frame."""
    if len(buf) != TELEMETRY_SIZE:
        return None
    f = struct.unpack(TELEMETRY_FMT, buf)
    if f[0] != TELEMETRY_MAGIC:
        return None
    return {
        "seq": f[1],
        "cmd_echo": f[2],
        "t_robot": f[3],
        "enc_left": f[4],
        "enc_right": f[5],
        "adc": list(f[6:6 + N_ADC]),
        "imu": list(f[22:28]),
        "imu_temp": f[28],
        "ultrasonic": f[29],
        "duty_left": f[30],
        "duty_right": f[31],
        "supply_v": f[32],
        "supply_a": f[33],
        "buttons": f[34],
        "flags": f[35],
    }


def pack_command(seq, cmd_type, a, b):
    return struct.pack(COMMAND_FMT, COMMAND_MAGIC, seq & 0xFFFFFFFF,
                       cmd_type, a, b)


def unpack_command(buf):
    if len(buf) != COMMAND_SIZE:
        return None
    f = struct.unpack(COMMAND_FMT, buf)
    if f[0] != COMMAND_MAGIC:
        return None
    return {"seq": f[1], "type": f[2], "a": f[3], "b": f[4]}


# --- camera frames: robot -> PC ---------------------------------------------
#   I  magic
#   I  seq
#   d  t_robot     time.time() on the robot when the frame was captured
#   H  width
#   H  height
#   I  jpeg_len    bytes of JPEG payload following this header
CAMERA_MAGIC = 0x46424931  # 'FBI1'
CAMERA_HDR_FMT = "<IIdHHI"
CAMERA_HDR_SIZE = struct.calcsize(CAMERA_HDR_FMT)


def pack_camera_header(seq, t_robot, width, height, jpeg_len):
    return struct.pack(CAMERA_HDR_FMT, CAMERA_MAGIC, seq & 0xFFFFFFFF,
                       t_robot, width, height, jpeg_len)


def unpack_camera_header(buf):
    if len(buf) != CAMERA_HDR_SIZE:
        return None
    f = struct.unpack(CAMERA_HDR_FMT, buf)
    if f[0] != CAMERA_MAGIC:
        return None
    return {"seq": f[1], "t_robot": f[2], "width": f[3], "height": f[4],
            "jpeg_len": f[5]}


# --- ADC channel map --------------------------------------------------------
# Index into the telemetry adc[] array. Order is U8 ch0..7 then U7 ch0..7,
# matching ~/dev/pinmap.py on the robot.
ADC_DIST_BL = 4
ADC_DIST_FL = 5
ADC_LINE_LEFT = 6
ADC_LINE_MID = 7
ADC_LDR = 8
ADC_MIC = 9
ADC_LINE_RIGHT = 10
ADC_SPARE_J10 = 11
ADC_PHOTODIODE = 12
ADC_DIST_BR = 13
ADC_SPARE_J11 = 14
ADC_DIST_FR = 15
