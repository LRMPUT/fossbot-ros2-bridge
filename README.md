# fossbot_ros2_bridge

A ROS 2 Jazzy bridge that exposes the FOSSBot edu robot's hardware as ROS
topics, services and TF. ROS runs in a container **on the PC**; a small agent
runs **on the robot** and owns the hardware.

The published interface deliberately matches what the Gazebo sim in
`FOSSBotEduSim` publishes (`/cmd_vel`, `/odom`, `/scan`, `/joint_states`, `/tf`,
frames `odom` → `base_footprint`), so Nav2 and slam_toolbox configs written
against the simulator run unchanged against the real robot.

## Why it is built this way

The upstream `fossbot-lib-real` API is call-per-read (`get_reading(pin)`,
`get_acceleration(axis)`), which is fine on the robot and wrong across a
network: every sensor read would cost a wifi round trip, and a single ROS cycle
touching a dozen sensors would cost a dozen of them.

Instead the robot-side agent samples **everything locally** at 100 Hz — SPI and
I2C reads are sub-millisecond there — and ships one batched 102-byte frame per
cycle. The PC pays one network hop per cycle regardless of how many sensors it
wants.

| Transport | Port | Direction | Why |
|---|---|---|---|
| UDP | 5005 | robot → PC | Telemetry. Newest sample supersedes the last, so TCP's head-of-line blocking would turn a 4 ms link into an occasional 200 ms one for no benefit. A lost frame is replaced 11 ms later. |
| UDP | 5006 | PC → robot | Velocity commands. Same reasoning, plus a sequence number so reordered stragglers are dropped. |
| TCP | 5007 | PC ↔ robot | Services (LED, buzzer, e-stop, odom reset). Discrete and rare — there is no "newer value" to fall back on, so these must not be lost. |
| TCP | 5008 | robot → PC | Lidar scans. A scan is large and only useful complete. |
| TCP | 5009 | robot → PC | Camera. A JPEG frame is tens of kB, far past the MTU, so UDP would mean hand-rolled fragmentation where one lost datagram destroys the frame. |

`protocol.py` is the single definition of the wire format and is deployed
verbatim to both ends, so they cannot drift apart.

**Wheel control is closed-loop and runs on the robot**, never across the
network, so wifi jitter cannot destabilise it. This matters: the two motors
differ by ~11% at identical duty, so open-loop driving veers noticeably.

The loop tracks **accumulated distance, not velocity**, and that choice is
forced by the encoders. With 20 ticks/rev, a velocity estimate over a 0.2 s
window quantises to 0.055 m/s — 46% of a 0.12 m/s setpoint. A velocity loop
therefore spends its time reacting to whether one tick landed inside the window,
each wheel independently, and the two fight each other: the robot visibly
weaves. Tick *counts* have no such noise. Integrating the commanded velocity
into a target distance and driving the position error to zero gives the same
steady-state speed, holds the wheels in lockstep so the robot keeps a heading,
and is inherently smooth. Measured: 0.97 m travelled against 0.96 m commanded,
wheels within 0.31 rad (~3 deg of heading) over 8 s.

## Measured performance

Over wifi (`M321-Lab`), robot at 192.168.0.100, PC at 192.168.0.102:

| | |
|---|---|
| Telemetry rate | 88–90 Hz (100 Hz target; the 16 SPI + 1 I2C reads per cycle are the limit) |
| Frame loss | 0.06% over 25,021 frames |
| Round-trip latency | median 7–11 ms, min 4.7 ms, p95 22 ms |
| `/scan` | 6.7 Hz, ~135 valid returns per rotation |
| `/camera/image_raw/compressed` | 15 Hz, ~22 kB per frame (~2.6 Mbit/s) |
| Velocity tracking | 2.6–18.5% error, see calibration below |

## Quick start

```bash
./run.sh
```

Builds the image if needed, starts the container, **builds the workspace**, and
drops you into a shell.

The workspace build is not optional. `/ws/install` lives in the container's
writable layer, not in a mounted volume, so `docker compose down` (or any
`docker rm`) discards it and the next container falls back to whatever was baked
into the image. Any package added since shows up as
`PackageNotFoundError: package 'fossbot_description' not found`. `run.sh` runs an
incremental `colcon build` every time, which costs a couple of seconds and makes
that impossible.
Then start the robot-side agent:

```bash
./restart_agent.sh
```

This needs passwordless ssh to the robot. If it prompts, install your key once:

```bash
ssh-copy-id admin@fossbotrpi1.local
```

Note there is **no sudo** anywhere in these scripts: `admin` is in the robot's
`docker` group, so docker works directly. Calling sudo over a non-interactive
ssh fails with "sudo: a terminal is required to read the password".

And in the container:

```bash
ros2 launch fossbot_bridge bringup.launch.py
```

Point it at a different robot with `robot_host:=fossbotrpi2.local`, or skip the
lidar with `use_lidar:=false`.

Teleop, which handles the motor-enable handshake for you:

```bash
ros2 run fossbot_bridge teleop
```

**Motors start disabled — press `e` before anything moves.** The agent ignores
`/cmd_vel` until `/fossbot/enable_motors` is called with `true`, so a teleop
session that looks dead is usually just a missing `e`; the status line at the
bottom says which state you are in. Teleop also needs a real terminal, so
attach with `docker exec -it`, not a bare `docker exec`.

## Interface

### Frames and the robot model

`fossbot_description` publishes the URDF and every fixed transform below
`base_footprint`. **The bridge alone is not enough**: it only publishes
`odom -> base_footprint`, so without `robot_state_publisher` there is no
transform for `lidar_scan_frame` and RViz cannot place `/scan` at all — the
topic lists, the rate looks healthy, and nothing is drawn. `bringup.launch.py`
starts it by default (`use_description:=false` to opt out).

The URDF is vendored from `fossbot_educational_description` in FOSSBotEduSim,
with the gazebo plugin block stripped (on real hardware the bridge provides
those topics). Re-run `sync_description.sh` if the sim model changes.

### Published

| Topic | Type | Rate |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | ~90 Hz |
| `/joint_states` | `sensor_msgs/JointState` | ~90 Hz |
| `/tf` | `odom` → `base_footprint` | ~90 Hz |
| `/scan` | `sensor_msgs/LaserScan` | ~6.7 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | ~90 Hz |
| `/fossbot/line_sensors` | `fossbot_msgs/LineSensors` | ~90 Hz |
| `/fossbot/analog_raw` | `fossbot_msgs/AnalogRaw` (all 16 ADC channels) | ~90 Hz |
| `/fossbot/range/{front,back}_{left,right}` | `sensor_msgs/Range` | ~90 Hz |
| `/fossbot/ultrasonic` | `sensor_msgs/Range` | ~90 Hz |
| `/fossbot/buttons` | `fossbot_msgs/Buttons` | ~90 Hz |
| `/fossbot/light` | `sensor_msgs/Illuminance` | ~90 Hz |
| `/fossbot/microphone`, `/fossbot/photodiode` | `std_msgs/Float32` | ~90 Hz |
| `/fossbot/link_status` | `fossbot_msgs/LinkStatus` | 2 Hz |
| `/fossbot/battery` | `sensor_msgs/BatteryState` | ~90 Hz |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (jpeg) | ~15 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ~15 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` (decoded on the PC) | ~15 Hz |

### Subscribed

- `/cmd_vel` — `geometry_msgs/Twist`, closed-loop velocity
- `/fossbot/motor_cmd` — `fossbot_msgs/MotorCommand`, raw per-wheel duty −1..1
  (suspends the PI loop while in use)

### Services

- `/fossbot/enable_motors` — `std_srvs/SetBool`. **Motors ignore `/cmd_vel`
  until this is called with `true`.**
- `/fossbot/estop` — `std_srvs/SetBool`
- `/fossbot/set_rgb` — `fossbot_msgs/SetRGB` (on/off per channel; the LED is
  driven straight off GPIO, so there is no brightness control)
- `/fossbot/play_tone` — `fossbot_msgs/PlayTone`
- `/fossbot/reset_odometry` — `std_srvs/Trigger`

### Safety

Motors are disabled at startup, and the agent cuts them if no command frame
arrives for 500 ms — pull the plug on the PC and the robot coasts to a stop
rather than driving on.

## If RViz shows nothing

Three separate causes, all of which produce exactly the same symptom — topics
listed, nothing rendered:

1. **No `robot_state_publisher`.** `/scan` lives in `lidar_scan_frame`, which
   has no transform without it. Launch with `use_description:=true` (the
   default), or check `ros2 run tf2_ros tf2_echo base_footprint lidar_scan_frame`.
2. **Fixed Frame set to `map`.** Nothing publishes a `map` frame until you start
   SLAM. Set it to `odom`. The shipped RViz config already does.
3. **QoS mismatch.** RViz displays default to Reliable. A Best Effort publisher
   never matches a Reliable subscriber, so the topic appears and delivers
   nothing. This bridge publishes everything **Reliable** for that reason — a
   Reliable publisher satisfies Best Effort subscribers (Nav2, slam_toolbox)
   too, so it is strictly the more compatible choice. If you add your own
   publishers here, do the same.

   **The one exception is `/camera/image_raw`.** A 640x480 bgr8 frame is 921 kB,
   and pushing that Reliable at 15 fps dropped half the frames in testing — the
   publisher's history wrapped before the subscriber drained it. It is Best
   Effort with depth 1, which restored the full 14.5 Hz. Set RViz's Image
   display to Best Effort for it, or use `/camera/image_raw/compressed`, which
   is Reliable and forty times smaller. The shipped RViz config already does.

Use the shipped config, which gets all three right:

```bash
ros2 launch fossbot_bridge bringup.launch.py use_rviz:=true
```

## Lidar orientation

Two independent corrections, both in `config/bridge.yaml`:

- **`invert: true`** — the RPLIDAR reports angles increasing *clockwise*, ROS
  `LaserScan` is *counter-clockwise*. Without this the scan is mirrored, and
  no rotation can fix it. The tell-tale is that adding offset rotates the
  picture the *wrong way*.
- **`angle_offset_deg: -90`** — `lidar_scan_frame` inherits `base_link`'s +90°
  yaw, because the CAD model was exported with the robot facing along its own
  Y axis.

To check the alignment rather than guess, put an object about 30 cm directly in
front of the robot and run:

```bash
ros2 run fossbot_bridge scan_bearing
```

It prints the bearing of the nearest return in `base_footprint` using the real
TF, so it reports what RViz actually draws. Straight ahead should read ~0°, an
object on the left ~+90°.

## Motor direction

`MOTOR_LEFT_INVERT` and `MOTOR_RIGHT_INVERT` in `fossbot_agent.py` are both
`True` on this robot: the motor leads are soldered such that the TB6612's
IN1/IN2 convention drives both wheels backwards. Symptom was `w` driving the
robot in reverse *and* `a`/`d` turning the wrong way.

Those two symptoms together identify the fault uniquely, which is worth
remembering:

| Symptom | Cause |
|---|---|
| Forward reversed, turns **also** reversed | Both motors wired backwards — negating both wheel velocities gives `v' = -v` and `w' = -w` at once |
| Turns reversed, forward **correct** | Left/right channels swapped |
| Forward reversed, turns **correct** | Robot frame is 180° out (rotation about +Z is unaffected by yawing the frame) |

Verified on this robot: channel A drives the `L_ODO` encoder and the physical
**left** wheel; channel B drives `R_ODO` and the **right** wheel. Positive duty
on either drives that wheel forwards.

**Do not use `/odom` to check drive direction.** The encoders are single
channel, so tick sign comes from the *commanded* direction — odometry
structurally cannot disagree with the command, and will happily report forward
motion while the robot reverses. Only physical observation, or the IMU gyro for
rotation, is ground truth here.

## Calibration

Two constants decide whether `/odom` means anything:

- `wheel_radius` = **0.03524 m**, from the v2 URDF wheel mesh (70.47 mm
  diameter). Note the upstream library says 6.65 cm for v1 hardware — if your
  wheels are the older ones, use 0.03325.
- `wheel_track` = **0.1866 m**. This is the distance between the wheel *centre
  planes*, not between the URDF joint origins (±0.0779, which would give
  0.1559). Using the joint origins puts about 17% error into every rotation.
- `encoder_ticks_per_rev` = **20**, matching the upstream library's
  `sensor_disc = 20` and consistent with measured tick rates at known duty.

**The encoders are the accuracy floor.** They are single-channel slotted discs:
20 ticks per revolution, and no direction information at all — tick sign comes
from the commanded motor direction, so a wheel pushed backwards by hand still
counts up. At 0.1 m/s a wheel produces about 9 ticks/s, so a 0.2 s velocity
window sees fewer than 2 ticks. That quantization, not the controller, is what
sets the 2.6–18.5% velocity error measured above. `/odom` covariance is set
loose on purpose; do not let a filter trust it.

To calibrate properly, put the robot on the floor (not suspended — free-spinning
wheels turn far faster than loaded ones), drive a measured straight line, and
compare `/odom` position against the tape measure.

## Camera

The robot carries an IMX708 (Camera Module 3, with autofocus). Getting it
working needed a detour worth recording:

**Ubuntu 24.04 cannot drive a Pi 5 camera as shipped.** It packages libcamera
0.2.0, whose only Raspberry Pi IPA is `ipa_rpi_vc4.so` — the Pi 4 ISP. The Pi 5
uses PiSP and needs `ipa_rpi_pisp.so`, which is absent, so `cam --list` comes up
empty even though the sensor probes fine. `rpicam-apps` and `python3-picamera2`
are not in the Ubuntu archive at all. There is no V4L2 shortcut either: the Pi 5
CSI emits only raw Bayer and PiSP does the debayer, auto-exposure and white
balance, so bypassing it gives dark green frames at a 1536x864 minimum mode.

`robot_agent/build_camera_stack.sh` builds the real stack — `libpisp`, then
Raspberry Pi's `libcamera` fork with `-Dpipelines=rpi/pisp`, then `rpicam-apps`.
It installs to `/ws/opt/camera`, which is on the robot's real filesystem, so it
survives recreating the `fb` container and does not add half an hour to every
image rebuild. Run it once:

```bash
sudo docker exec -it fb bash /ws/build_camera_stack.sh
```

It defaults to `JOBS=2` and runs under `nice`: a four-core `ninja` dropped this
robot off the network mid-build once, and the build tree lives under `/ws` so an
interrupted run resumes rather than restarting.

The agent then runs `rpicam-vid --codec mjpeg` and relays frames; the PC decodes.
JPEG rather than raw because 640x480 RGB8 is 921 kB a frame — 110 Mbit/s at
15 fps, against roughly 5 Mbit/s for JPEG at quality 80.

`CameraInfo` is a **nominal pinhole model derived from the lens FOV, not a
calibration**: zero distortion, centred principal point. Fine for RViz and for
anything that just wants image geometry; wrong for photogrammetry or visual
odometry. Run `camera_calibration` against a checkerboard and replace `K`/`D`
if you need metric accuracy.

## The robot's clock

There is no RTC battery and this lab network blocks NTP (UDP 123 to
ntp.ubuntu.com times out), so every cold boot came up months out of date, which
makes apt reject every repository with "Release file is not valid yet".
`http-time-sync.service` on the robot now sets the clock from an HTTP `Date`
header at boot (source kept in `robot_agent/http-time-sync.sh`). It retries for
a few minutes, because `network-online.target` fires before wifi has actually
associated and got a lease — the first version ran one second into boot, failed
every fetch, and left the clock wrong anyway. Nothing in the bridge depends on the robot's wall clock — ROS
stamps use the PC clock and odometry uses time *differences* — but apt and TLS
do.

## Power: the robot resets under motor load

This robot **cuts out while driving**. The evidence says brownout, not throttle:
after a drop-out `vcgencmd get_throttled` reads `0x0` with an uptime of seconds.
Undervoltage *throttling* would keep the Pi up and latch bit 0/bit 16 — but if
the rail collapses the SoC cannot record anything, and the register comes back
clear because it reset. Motor current is the trigger.

### Watching it

`/fossbot/battery` publishes a `sensor_msgs/BatteryState`, and
`/fossbot/link_status` carries `supply_volts`, `supply_amps` and a `low_voltage`
flag. The bridge logs a throttled warning below `supply_warn_v` (4.75 V).

```bash
ros2 topic echo /fossbot/battery --once
```

The measurement is the Pi 5's own PMIC (`vcgencmd pmic_read_adc`, field
`EXT5V_V`) — there is no battery divider on this PCB, every spare ADC channel
reads zero. Be clear about what that means:

- It **is** the 5 V rail that collapses in a brownout, so it is exactly the
  right thing to watch for these resets, and the sag under motor load is the
  number that predicts one.
- It is **not** battery state of charge. If a regulator sits between the cells
  and the Pi, it reads flat until the battery can no longer hold the rail up.
  `percentage` is therefore published as NaN rather than invented from voltage.

The agent tracks the minimum rail voltage seen (`supply_v_min` in the `status`
service, reset with the `reset_supply_min` op), because a 2 Hz sample will miss
the transient dip that actually trips the reset.

Software mitigations are in the agent, and they are mitigations, not a cure:

- `DUTY_SLEW_PER_S` ramps duty instead of stepping it, so a stalled rotor is
  never hit with full duty at once (stops bypass the ramp — stopping must be
  immediate).
- `DUTY_MAX` caps duty below 100%.

**The real fix is hardware.** Worth checking, in order: whether the Pi and the
motor driver share one supply (motor stall current sagging the 5 V rail is the
classic cause — separate them, or add bulk capacitance across the motor supply),
and whether the supply can actually deliver the Pi 5's rated current on top of
the motors.

Because the robot resets often, `fossbot-agent.service` on the robot restarts
the agent automatically at boot (the `fb` container already has
`restart: always`, but the agent inside it was started with `docker exec` and
would not come back on its own).

## Known gaps
- **The front-right IR sensor reads ~0 counts** while the other three read ~940.
  It is either unpopulated or faulty — check the hardware before trusting
  `/fossbot/range/front_right`.
- **The robot's clock is far off** (107 days behind when this was written).
  Nothing here depends on it — ROS stamps use the PC clock and odometry uses
  time *differences* on the robot clock — but it will break apt and TLS on the
  robot. Fix with `sudo timedatectl set-ntp true` on the Pi.
- **The ultrasonic sensor reports NaN**, which is correct behaviour for "no
  echo"; no HC-SR04 appears to be fitted.

## Layout

```
robot_agent/fossbot_agent.py        runs on the Pi, owns all hardware
ws/src/fossbot_msgs/                messages and services
ws/src/fossbot_bridge/
  fossbot_bridge/protocol.py        wire format, deployed to BOTH ends
  fossbot_bridge/bridge_node.py     telemetry -> topics, cmd_vel -> robot
  fossbot_bridge/lidar_node.py      scan stream -> /scan
  fossbot_bridge/teleop.py          keyboard teleop with enable handshake
  config/bridge.yaml                all tunable parameters
  launch/bringup.launch.py
Dockerfile, docker-compose.yml      the PC-side ROS 2 container
run.sh                              build + shell
deploy_agent.sh, restart_agent.sh   push the agent to the robot
```

### Container notes

`network_mode: host` is required, not convenience — the robot pushes telemetry
to UDP 5005 on this machine, and behind Docker's NAT the agent would be replying
to a translated address.

The `/var/run/avahi-daemon/socket` mount is what makes `fossbotrpi1.local`
resolve inside the container. nss-mdns 0.15 uses that socket, **not** the D-Bus
one — mounting the D-Bus socket instead looks right and silently fails.
