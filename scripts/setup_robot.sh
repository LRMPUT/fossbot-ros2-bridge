#!/bin/bash
# One-time provisioning of a FOSSBot (Raspberry Pi 5, Ubuntu 24.04, Docker).
#
#   scripts/setup_robot.sh [user@]robot-host [--config robot.json]
#                          [--invert none|left|right|both]
#                          [--camera] [--http-time]
#
#   --config FILE  install FILE as the robot's agent_config.json
#   --invert MODE  none|left|right|both: which motors are wired reversed
#   --camera       also build the Pi 5 camera stack (~15 min)
#   --http-time    install a boot-time clock sync over HTTP, for networks that
#                  block NTP (otherwise apt fails with "not valid yet")
#
# Prerequisites on the robot: passwordless ssh for the user, Docker installed,
# that user in the docker group, SPI and I2C enabled. Installing the systemd
# unit needs sudo, so you will be asked for the robot user's password once.
set -euo pipefail
source "$(dirname "$0")/common.sh"

if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
  resolve_target "$1"
  shift
else
  resolve_target
fi
CONFIG=""; INVERT=""; CAMERA=0; HTTP_TIME=0
while [ $# -gt 0 ]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --invert) INVERT="$2"; check_invert_mode "$INVERT"; shift 2 ;;
    --camera) CAMERA=1; shift ;;
    --http-time) HTTP_TIME=1; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

require_key_login

echo "=== checking $TARGET ==="
rsh 'set -e
  echo "  $(. /etc/os-release; echo "$PRETTY_NAME") $(uname -m)"
  tr -d "\0" < /proc/device-tree/model 2>/dev/null | sed "s/^/  /"; echo
  command -v docker >/dev/null || { echo "  ERROR: docker is not installed"; exit 1; }
  docker ps >/dev/null 2>&1 || { echo "  ERROR: $USER cannot run docker (add it to the docker group, then log in again)"; exit 1; }
  for d in /dev/spidev0.0 /dev/spidev0.1 /dev/i2c-1; do
    [ -e "$d" ] || echo "  WARNING: $d missing - enable SPI/I2C (dtparam=spi=on, dtparam=i2c_arm=on)"
  done
  ls /dev/ttyUSB* >/dev/null 2>&1 || echo "  note: no /dev/ttyUSB* - lidar not connected (optional)"'

echo "=== uploading agent to ~/$FOSSBOT_DIR ==="
upload_agent
[ -n "$CONFIG" ] && upload_config "$CONFIG"
[ -n "$INVERT" ] && set_motor_invert "$INVERT"

echo "=== building image $IMAGE (several minutes the first time) ==="
# No build context: the image only installs packages, and the agent directory
# reaches gigabytes once the camera stack is built inside it.
rsh "docker build -q -t $IMAGE - < '$FOSSBOT_DIR/Dockerfile'"

echo "=== (re)creating container $CONTAINER ==="
# Privileged with /dev mounted: the agent needs GPIO, SPI, I2C, the lidar's
# serial port, the camera's media devices and /dev/vcio. Host networking so the
# PC can reach the agent's ports directly. restart=always, not unless-stopped:
# unless-stopped does not come back after a docker daemon restart or reboot.
rsh "docker rm -f $CONTAINER >/dev/null 2>&1 || true
     docker run -d --name $CONTAINER --privileged --restart always --network host \
       -v /dev:/dev -v \"\$HOME/$FOSSBOT_DIR:/ws\" $IMAGE >/dev/null
     docker ps --filter name=^$CONTAINER\$ --format '  {{.Names}}: {{.Status}}'"

echo "=== installing systemd units (sudo) ==="
# fossbot-agent.service starts the agent inside the container at boot; the
# container's restart policy alone does not, because the agent is started with
# docker exec rather than as the container's main process.
UNITS="sudo install -m 644 ~/$FOSSBOT_DIR/systemd/fossbot-agent.service /etc/systemd/system/"
if [ "$HTTP_TIME" = 1 ]; then
  UNITS="$UNITS && sudo install -m 755 ~/$FOSSBOT_DIR/http-time-sync.sh /usr/local/sbin/ \
         && sudo install -m 644 ~/$FOSSBOT_DIR/systemd/http-time-sync.service /etc/systemd/system/"
fi
UNITS="$UNITS && sudo systemctl daemon-reload && sudo systemctl enable --now fossbot-agent.service"
[ "$HTTP_TIME" = 1 ] && UNITS="$UNITS && sudo systemctl enable --now http-time-sync.service"
ssh -t "${SSH_OPTS[@]}" "$TARGET" "$UNITS"

if [ "$CAMERA" = 1 ]; then
  echo "=== building camera stack (~15 min) ==="
  ssh -t "${SSH_OPTS[@]}" "$TARGET" "docker exec -it $CONTAINER bash /ws/build_camera_stack.sh"
  rsh "docker exec $CONTAINER pkill -9 -f fossbot_agent.py 2>/dev/null || true
       docker exec -d $CONTAINER bash -c 'cd /ws && python3 fossbot_agent.py > /tmp/agent.log 2>&1'"
fi

sleep 5
echo "=== agent log ==="
rsh "docker exec $CONTAINER cat /tmp/agent.log" || true
warn_if_unconfigured
echo
echo "Done. Next: point the PC at it with FOSSBOT_HOST=${TARGET#*@} ./run.sh"
