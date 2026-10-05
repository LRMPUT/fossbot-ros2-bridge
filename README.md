# fossbot-ros2-bridge

A ROS 2 Jazzy bridge for the [FOSSBot](https://github.com/eellak/fossbot)
educational robot. It exposes the robot's motors, encoders, IMU, analog sensors,
buttons, LED, buzzer, RPLIDAR and Pi camera as standard ROS 2 topics, services
and TF.

ROS 2 runs **on your PC**, in Docker. Each FOSSBot runs a small Python agent
that owns its hardware and needs no ROS at all. The two talk over the network.

The ROS interface mirrors the FOSSBot Gazebo simulation (`/cmd_vel`, `/odom`,
`/scan`, `/joint_states`, `/tf`, frames `odom` → `base_footprint`), so Nav2 and
slam_toolbox configurations written against the simulator work on the real
robot unchanged.

```
 PC: Docker, ROS 2 Jazzy                    Robot: Raspberry Pi 5, no ROS
+-----------------------------+            +-----------------------------+
| fossbot_bridge   -> topics  | <-UDP 5005-| fossbot_agent.py            |
| fossbot_lidar    -> /scan   |  telemetry |   GPIO, SPI ADCs, I2C IMU   |
| fossbot_camera   -> /camera | -UDP 5006->|   motors (closed loop)      |
| robot_state_publisher       |  commands  |   RPLIDAR, camera, PMIC     |
| RViz, Nav2, your nodes ...  | <-TCP 5007>|                             |
|                             |  services  |                             |
|                             | <-TCP 5008-|  lidar scans                |
|                             | <-TCP 5009-|  camera JPEG frames         |
+-----------------------------+            +-----------------------------+
```

## Requirements

**Robot**

- Reference platform: FOSSBot v2 PCB (TB6612, 2× MCP3008, MPU-6050/6500)
  and a Raspberry Pi running Ubuntu 24.04 (arm64)
- Docker, with the robot's login user in the `docker` group
- SPI and I2C enabled (`dtparam=spi=on` and `dtparam=i2c_arm=on` in
  `/boot/firmware/config.txt`)
- Optional: RPLIDAR A1 on USB, Raspberry Pi Camera Module 3 (IMX708)

**PC**

- Linux x86_64 with Docker and Docker Compose
- Optional: an X11 display for RViz; the bridge also works headless.
- Optional: Avahi for `<robot>.local` hostnames; IP addresses need no Avahi.

`run.sh` enables the GUI overlay when `DISPLAY` is set. Set `FOSSBOT_MDNS=1`
to pass the host's Avahi socket into the container when using `.local` names;
otherwise use the robot's IP address.

PC and robot must be on the same network, and the network must allow the UDP
and TCP ports above between them. Client-isolated guest Wi-Fi will not work.
`robot_agent/pinmap.py` targets the reference v2 PCB. Other FOSSBot boards
need a matching pin map and per-robot config; host selection and PC-side ROS
parameters are configurable for each robot.

## Robot setup (once per robot)

1. Install your SSH key on the robot:

   ```bash
   ssh-copy-id <user>@<robot>
   ```

2. Provision it from this repository:

   ```bash
   scripts/setup_robot.sh <user>@<robot> --camera
   ```

   This copies the agent to `~/fossbot` on the robot, builds the agent image,
   creates a privileged `fb` container that restarts with the robot, and
   installs a systemd unit that starts the agent at boot. It asks for the
   robot user's password once, for `sudo`.

   - `--camera` also builds the Raspberry Pi 5 camera stack (about 15 minutes;
     see [Camera](#camera) for why it must be built from source). Omit it if
     there is no camera.
   - `--http-time` installs a boot-time clock sync over HTTP, for networks that
     block NTP (see [Troubleshooting](#troubleshooting)).
   - `--config robot.json` installs a per-robot settings file.
   - `--invert both` (or `left`/`right`) if the motors are wired reversed —
     see [Motor direction](#1-motor-direction).

3. Check the motor direction and calibrate — see
   [Per-robot setup](#per-robot-setup). Every robot is wired slightly
   differently, and the defaults are not verified for yours.

The scripts also read `FOSSBOT_HOST` (robot hostname or IP), `FOSSBOT_USER`
(the robot's login, if it differs from yours) and `FOSSBOT_DIR` (agent directory
on the robot, default `~/fossbot`), so `scripts/restart_agent.sh` works without
arguments once those are set.

## Quick start

```bash
FOSSBOT_HOST=<robot> ./run.sh
```

`<robot>` is the robot's hostname or IP, for example `fossbot1.local`. The
script builds the image on first use, starts the container, builds the ROS
workspace, and opens a shell in the container. There:

```bash
ros2 launch fossbot_bridge bringup.launch.py use_rviz:=true
```

A **dashboard** window opens with it: link quality, battery, motor state and
duty, commanded versus measured motion, odometry and every sensor, plus buttons
to enable/disable the motors, toggle the e-stop and reset odometry. Without a
display it prints the same summary to the terminal every few seconds instead.
It only reads the bridge's topics, so you can also run it on its own on any
machine on the same `ROS_DOMAIN_ID`:

```bash
ros2 run fossbot_bridge dashboard
```

Drive with the keyboard from a **second** terminal:

```bash
docker exec -it fossbot_bridge bash -lc 'source /ws/install/setup.bash && ros2 run fossbot_bridge teleop'
```

**Motors start disabled. Press `e` in teleop before anything moves.** The agent
ignores `/cmd_vel` until `/fossbot/enable_motors` is called with `true`, and it
stops the motors if no command arrives for 500 ms — closing the PC side brings
the robot to a stop.

Useful launch arguments:

| Argument | Default | |
|---|---|---|
| `robot_host` | `$FOSSBOT_HOST` | Robot hostname or IP. Required. |
| `use_rviz` | `false` | Start RViz with the bundled config |
| `use_lidar` | `true` | Start the lidar node |
| `use_camera` | `true` | Start the camera node |
| `use_description` | `true` | Publish the URDF and TF below `base_footprint` |
| `use_dashboard` | `true` | Show the status dashboard |
| `params_file` | bundled `bridge.yaml` | ROS parameters |

### Several robots on one network

Give every robot/PC pair its own `ROS_DOMAIN_ID`. The bridge publishes on the
PC with host networking, so two PCs on the same domain see — and can command —
each other's robots:

```bash
ROS_DOMAIN_ID=3 FOSSBOT_MDNS=1 FOSSBOT_HOST=fossbot3.local ./run.sh
```

Run one bridge per workstation and one controlling workstation per robot.
The container name and host ports are shared; changing `ROS_DOMAIN_ID` alone
does not support multiple bridge containers on the same workstation.

## Interface

### Published

| Topic | Type | Rate |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | ~90 Hz |
| `/tf` | `odom` → `base_footprint` | ~90 Hz |
| `/joint_states` | `sensor_msgs/JointState` | ~90 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` (no orientation) | ~90 Hz |
| `/scan` | `sensor_msgs/LaserScan` | lidar rotation rate, ~6 Hz |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (JPEG) | 15 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` (decoded on the PC) | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz |
| `/fossbot/line_sensors` | `fossbot_msgs/LineSensors` | ~90 Hz |
| `/fossbot/range/{front,back}_{left,right}` | `sensor_msgs/Range` (IR) | ~90 Hz |
| `/fossbot/ultrasonic` | `sensor_msgs/Range` | ~90 Hz |
| `/fossbot/analog_raw` | `fossbot_msgs/AnalogRaw` (all 16 ADC channels) | ~90 Hz |
| `/fossbot/buttons` | `fossbot_msgs/Buttons` | ~90 Hz |
| `/fossbot/light` | `sensor_msgs/Illuminance` (relative) | ~90 Hz |
| `/fossbot/microphone`, `/fossbot/photodiode` | `std_msgs/Float32` (volts) | ~90 Hz |
| `/fossbot/battery` | `sensor_msgs/BatteryState` | ~90 Hz |
| `/fossbot/link_status` | `fossbot_msgs/LinkStatus` | 2 Hz |

`robot_state_publisher` also publishes `/robot_description` and the static TF
tree from `fossbot_description`.

### Subscribed

- `/cmd_vel` — `geometry_msgs/Twist`. Closed-loop wheel control on the robot.
- `/fossbot/motor_cmd` — `fossbot_msgs/MotorCommand`, raw per-wheel duty
  −1..1. Suspends the closed loop while in use.

### Services

- `/fossbot/enable_motors` — `std_srvs/SetBool`
- `/fossbot/estop` — `std_srvs/SetBool`
- `/fossbot/reset_odometry` — `std_srvs/Trigger`
- `/fossbot/set_rgb` — `fossbot_msgs/SetRGB` (each channel on/off; no brightness)
- `/fossbot/play_tone` — `fossbot_msgs/PlayTone`

### Notes on what the values mean

- **Odometry** comes from single-channel encoders: 20 ticks per wheel turn, and
  **no direction sensing** — tick sign is taken from the commanded direction.
  A wheel pushed backwards by hand still counts forwards. Covariances are set
  loose accordingly.
- **IR ranges** are a monotonic approximation of an uncharacterised sensor.
  Treat threshold crossings as meaningful, not the absolute distance.
- **`/fossbot/light`** is relative brightness, not calibrated lux.
- **`/fossbot/battery`**: `voltage` is the Pi 5's 5 V input rail as measured
  by its PMIC (`EXT5V_V`) — the rail that collapses in a brownout.
  `percentage` is an **estimate** from that rail unless you add a battery
  sense; see [Battery](#battery).
- **`CameraInfo`** is a nominal pinhole model from the lens field of view, not
  a calibration. Run `camera_calibration` if you need metric accuracy.

## Per-robot setup

Settings that differ between robots live in `agent_config.json` in the agent
directory on the robot (`~/fossbot` by default). Start from
[`robot_agent/config.example.json`](robot_agent/config.example.json); every key
is optional and validated at startup (see
[`robot_agent/agent_config.py`](robot_agent/agent_config.py) for the full list
and ranges). Install or update it with:

```bash
scripts/restart_agent.sh <user>@<robot> --config my-robot.json
```

Routine deployment preserves `agent_config.json`; `--config` explicitly replaces
it, and `--invert` updates only motor polarity. Keep your robots' files outside
this repository. Wi-Fi credentials, SSH logins, hostnames and deployment paths
belong in your site's connection instructions, not in the shared defaults.

### 1. Motor direction

Motor polarity depends on how the motor leads were soldered. Lift the robot so
the wheels are free, start teleop, press `e`, then `w` and `a`:

| `w` (forward) | `a` (turn left) | Fault | Fix |
|---|---|---|---|
| forward | left | none | |
| **backward** | **right** | both motors reversed | `--invert both` |
| forward | **right** | left/right channels swapped | swap the motor connectors |
| **backward** | left | robot frame 180° out | check which end you call the front |

Apply the fix when deploying; it is stored in the robot's `agent_config.json`
and kept on every later deploy:

```bash
scripts/restart_agent.sh <user>@<robot> --invert both
```

`--invert` takes `none`, `left`, `right` or `both`, and changes only those two
settings in the robot's config. It works on `setup_robot.sh` too.

**Do not use `/odom` to check this.** With direction-less encoders, odometry
always agrees with the command, even when the robot drives the other way.

### 2. Motor feedforward (optional, improves tracking)

`wheel_calibrate` drives both wheels at a series of raw duties, forward and
reverse, measures the speed each produces, and fits a per-motor, per-direction
model `duty = static_duty + duty_per_mps × |speed|`. It talks to the agent
directly, so stop the ROS bringup and teleop first.

```bash
# Print the plan without connecting or moving anything:
PYTHONPATH=ws/src/fossbot_bridge python3 -m fossbot_bridge.wheel_calibrate --host <robot>

# On the floor, with clear space for slightly curved travel:
PYTHONPATH=ws/src/fossbot_bridge python3 -m fossbot_bridge.wheel_calibrate \
  --host <robot> --run --condition floor --output floor-run1.json
```

If the default duties (6–20%) do not overcome rolling friction, raise them
explicitly, e.g. `--max-duty 0.40 --duties 0.30 0.33 0.36 0.40`. Each fit needs
at least three moving levels with eight encoder ticks each; inconclusive data is
reported rather than turned into a correction. The sweep aborts on telemetry
loss, low supply voltage or unexpected command echoes, and the robot's 500 ms
watchdog stops the motors if the tool dies.

Copy each valid fit into the config as `[static_duty, duty_per_mps]`:

```json
"motor_ff_left_forward": [0.13, 1.03]
```

**Repeat the run and compare before trusting it.** Fits from separate runs on
the same robot can differ substantially, especially near the stiction
threshold. Suspended runs show motor asymmetry but are not floor calibration.
The defaults reproduce a plain proportional feedforward, and the closed loop
corrects the rest either way.

### 3. Gyro heading hold (optional)

The encoders are too coarse to hold a heading well; the IMU gyro is not.
Heading hold is off by default because the gyro's sign must be verified on
each robot first — the wrong sign turns it into positive feedback:

```bash
ros2 run fossbot_bridge gyro_sign_check
```

Then set `"gyro_sign"` as reported and `"gyro_heading_kp": 0.25`.

### 4. Lidar alignment

`bridge.yaml` sets `invert: true` and `angle_offset_deg: -90` for the standard
mount. To check yours, put an object about 30 cm directly in front of the robot:

```bash
ros2 run fossbot_bridge scan_bearing
```

Straight ahead should read about 0°, an object on the left about +90°. If adding
offset rotates the scan the *wrong* way, the scan is mirrored: fix `invert`
before touching the offset — no rotation can correct a mirror.

## Battery

The reference FOSSBot v2 PCB has no battery voltage sense, so by default the charge in
`/fossbot/battery` and the dashboard is **estimated from the Pi's regulated
5 V rail**, mapped linearly between `battery_rail_empty_v` (4.65 V) and
`battery_rail_full_v` (4.90 V) and smoothed over 10 s. A regulated rail stays
near 5 V for most of the discharge and only sags once the battery can no longer
hold it, so this gauge is coarse: it says "fine" for most of the run and then
drops quickly. Treat it as an early warning, not a fuel gauge. The Pi 5 flags
undervoltage near 4.63 V and resets around 4.4 V.

For a real reading, wire a resistor divider from the battery pack to a spare
ADC header and tell the bridge about it in `bridge.yaml`:

```yaml
battery_source: "adc"
battery_adc_index: 11        # J10 (U7 ch3); J11 is 14
battery_divider_ratio: 3.0   # V_pack / V_adc; keep V_adc below 3.3 V
battery_cells: 2             # series Li-ion cells
```

The charge is then read off a Li-ion discharge curve per cell. For example,
20 kΩ from the pack to the header and 10 kΩ from the header to ground gives a
ratio of 3.0, which keeps a full 2S pack (8.4 V) at 2.8 V on the ADC.

## Camera

Ubuntu 24.04 cannot drive a Raspberry Pi 5 camera with its own packages: it
ships libcamera 0.2.0, whose only Raspberry Pi IPA is for the Pi 4 ISP (VC4).
The Pi 5's PiSP needs a newer libcamera, and `rpicam-apps` and `picamera2` are
not packaged. `cam --list` comes up empty even though the sensor probes fine.

`setup_robot.sh --camera` builds `libpisp`, Raspberry Pi's `libcamera` fork and
`rpicam-apps` into `~/fossbot/opt/camera` on the robot (it survives container
recreation). To build it later:

```bash
ssh -t <user>@<robot> docker exec -it fb bash /ws/build_camera_stack.sh
```

Frames travel as JPEG (about 22 kB at 640×480) and are decoded on the PC.
`/camera/image_raw` is published **Best Effort**: 921 kB frames pushed Reliable
at 15 Hz overrun the publisher's history and drop half the frames. Use Best
Effort in RViz for it, or `/camera/image_raw/compressed`, which is Reliable.

## Troubleshooting

**RViz shows topics but draws nothing.** Three independent causes:
`robot_state_publisher` is not running (so `lidar_scan_frame` has no
transform — check with `ros2 run tf2_ros tf2_echo base_footprint lidar_scan_frame`),
the Fixed Frame is `map` while nothing publishes `map` (use `odom`), or a QoS
mismatch. Use the bundled config: `use_rviz:=true`.

**`ros2 topic list` shows only `/parameter_events` and `/rosout`.** Nothing is
running. A launch started in an interactive shell dies with that shell.

**`package 'fossbot_description' not found`.** The container was recreated and
fell back to the workspace baked into the image. `./run.sh` rebuilds the
workspace every time; inside the container, run
`cd /ws && colcon build --symlink-install`.

**`robot_host` is required / `Set robot_host or FOSSBOT_HOST`.** Pass
`robot_host:=<robot>`, or start the container with `FOSSBOT_HOST` set.

**Teleop does nothing.** Press `e`. Teleop also needs a real terminal: attach
with `docker exec -it`, not plain `docker exec`.

**The robot resets while driving.** On the reference Raspberry Pi 5 platform,
this is usually a weak battery: motor current sags the supply until it browns
out. Watch `/fossbot/battery`; the Pi 5
flags undervoltage near 4.63 V and resets around 4.4 V. After a reset,
`vcgencmd get_throttled` reads `0x0` — not because nothing happened, but
because the reset cleared it. The agent ramps motor duty and caps it at 85% to
limit current spikes; the real fix is a charged battery or a stronger supply.

**apt on the robot fails with "Release file is not valid yet".** The clock is
wrong. Systems without a battery-backed RTC depend on NTP; on networks that
block NTP, install the HTTP clock sync with `setup_robot.sh --http-time`.

**The agent log says `GPIO busy` a few times.** Normal after a restart: the
kernel releases the previous process's GPIO lines asynchronously, and the agent
retries until it can claim them.

**No `/scan`, agent log shows lidar descriptor errors.** The lidar's serial
buffer held stale data. The agent resets the device on each connection; if it
persists, unplug and replug the lidar.

**`<robot>.local` does not resolve inside the container.** Start with
`FOSSBOT_MDNS=1 ./run.sh` so the container can ask the host's avahi-daemon. If
the PC has no avahi, use the robot's IP address.

## How it works

**One batched frame instead of per-sensor calls.** The upstream FOSSBot Python
API reads one sensor per call. Wrapped over Wi-Fi, every read would cost a round
trip. Instead the agent samples everything locally — SPI and I2C reads are
sub-millisecond on the robot — and sends one 110-byte frame per cycle.

**UDP where the newest sample wins, TCP where data must arrive.** Telemetry and
velocity commands are UDP: a lost frame is superseded ~11 ms later, while TCP's
head-of-line blocking would stall every later frame behind it. Services,
lidar scans and camera frames use TCP because they are discrete or only useful
complete. `protocol.py` defines the wire format and is deployed to both ends,
so they cannot drift apart.

**Wheel control is closed-loop on the robot and tracks distance, not
velocity.** With 20 ticks per turn, a velocity estimate over a 0.2 s window
moves in steps of 0.055 m/s — nearly half of a typical 0.12 m/s setpoint — and
a velocity loop chasing that noise makes the robot weave. Accumulated tick
counts are exact, so the agent integrates the commanded speed into a target
distance and drives the error to zero. It runs at 50 Hz on the robot, so Wi-Fi
jitter never enters the loop.

**Reliable QoS by default.** RViz subscribes Reliable, and a Best Effort
publisher never matches a Reliable subscriber, so best-effort sensor topics
would appear in RViz and deliver nothing. A Reliable publisher also satisfies
the Best Effort subscriptions Nav2 and slam_toolbox use. The one exception is
the raw camera image (see [Camera](#camera)).

**Wheel geometry.** `wheel_track` (0.1866 m) is the distance between the wheel
*centre planes* in the URDF meshes, not between the URDF joint origins
(0.1559 m), which would put ~17% error into every rotation.

## Development

```
robot_agent/               runs on the robot
  fossbot_agent.py           hardware agent
  agent_config.py            per-robot settings schema and validation
  config.example.json        starting point for a robot's agent_config.json
  pinmap.py                  FOSSBot v2 PCB pin map
  Dockerfile                 robot image
  build_camera_stack.sh      Pi 5 camera stack build
  http-time-sync.sh          optional clock sync for NTP-blocked networks
  systemd/                   units installed by setup_robot.sh
ws/src/fossbot_msgs/       messages and services
ws/src/fossbot_bridge/
  fossbot_bridge/protocol.py wire format, deployed to BOTH ends
  fossbot_bridge/*_node.py   bridge, lidar and camera nodes
  fossbot_bridge/dashboard.py status window / terminal summary
  fossbot_bridge/battery.py  charge estimation (no ROS dependency)
  fossbot_bridge/teleop.py, scan_bearing.py, gyro_sign_check.py, wheel_calibrate.py
  config/bridge.yaml         ROS parameters
  launch/bringup.launch.py
ws/src/fossbot_description/ URDF, meshes, RViz config
scripts/                   setup_robot.sh, restart_agent.sh, sync_description.sh
run.sh, Dockerfile, docker-compose.yml   PC side
```

After changing the agent or `protocol.py`, redeploy to the robot:

```bash
scripts/restart_agent.sh <user>@<robot>
```

Hardware-free tests:

```bash
PYTHONPATH=ws/src/fossbot_bridge python3 -m unittest discover -s ws/src/fossbot_bridge/test -v
PYTHONPATH=ws/src/fossbot_bridge python3 -m unittest discover -s robot_agent/test -v
```

`fossbot_description` is derived from the FOSSBot simulation's description
package; `scripts/sync_description.sh <path-to-fossbot_educational_description>`
re-vendors it. See [NOTICE](NOTICE).

## License

MIT — see [LICENSE](LICENSE). The robot design and the URDF meshes come from the
[FOSSBot](https://github.com/eellak/fossbot) project, also MIT; see
[NOTICE](NOTICE).
