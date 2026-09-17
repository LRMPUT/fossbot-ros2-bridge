#!/bin/bash
# Bring the bridge container up (building it the first time) and drop into a
# shell inside it. Safe to run repeatedly -- it reuses a running container.
set -e
cd "$(dirname "$0")"

# rviz2 and rqt need this once per login session to reach the host's X server.
xhost +local:docker >/dev/null 2>&1 || true

if [ -z "$(docker images -q fossbot_bridge:jazzy 2>/dev/null)" ]; then
  echo "building fossbot_bridge:jazzy ..."
  docker compose build
fi

if [ -z "$(docker ps -q -f name=^fossbot_bridge$)" ]; then
  docker compose up -d
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
  | tail -3

exec docker exec -it fossbot_bridge bash
