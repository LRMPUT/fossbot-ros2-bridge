#!/bin/bash
# Bring the bridge container up (building it the first time) and drop into a
# shell inside it. Safe to run repeatedly: it reuses the container unless its
# settings (such as FOSSBOT_HOST) changed.
set -eo pipefail
cd "$(dirname "$0")"

COMPOSE_FILES=(-f docker-compose.yml)
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
  COMPOSE_FILES+=(-f docker-compose.gui.yml)
  xhost +local:docker >/dev/null 2>&1 || true
fi
if [ "${FOSSBOT_MDNS:-0}" = 1 ]; then
  COMPOSE_FILES+=(-f docker-compose.mdns.yml)
fi

if [ -z "$(docker images -q fossbot_bridge:jazzy 2>/dev/null)" ]; then
  echo "building fossbot_bridge:jazzy ..."
  docker compose "${COMPOSE_FILES[@]}" build
fi

# Always run this: it is a no-op when nothing changed, and it recreates the
# container when the environment did -- e.g. a different FOSSBOT_HOST, which
# is fixed at container creation and would otherwise be silently ignored.
docker compose "${COMPOSE_FILES[@]}" up -d

if [ -z "${FOSSBOT_HOST:-}" ]; then
  echo "note: FOSSBOT_HOST is not set; pass robot_host:=<robot> to the launch file," >&2
  echo "      or start with  FOSSBOT_HOST=<robot-hostname-or-ip> ./run.sh" >&2
fi

# XDG_RUNTIME_DIR must exist and be 0700 or Qt complains on every launch.
docker exec fossbot_bridge bash -c 'mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root'

# Always build the workspace. /ws/install lives in the container's writable
# layer, not in a volume, so `docker compose down` (or any `docker rm`) throws
# it away and a fresh container falls back to whatever was baked into the image
# -- which silently omits any package added since. That shows up as
# "PackageNotFoundError: package 'fossbot_description' not found".
# An incremental colcon build costs a couple of seconds and makes it impossible.
echo "building workspace ..."
docker exec fossbot_bridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install' \
  | tail -3   # pipefail above: a failed build stops here, not in a stale shell

exec docker exec -it fossbot_bridge bash
