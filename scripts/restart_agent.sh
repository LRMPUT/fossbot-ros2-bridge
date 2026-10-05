#!/bin/bash
# Deploy the agent to a robot and restart it.
#
#   scripts/restart_agent.sh [user@]robot-host [--config robot.json]
#                            [--invert none|left|right|both]
#
# --config  installs a per-robot settings file as agent_config.json on the
#           robot. Without it, the robot's existing agent_config.json is kept.
# --invert  sets which motors are wired reversed, keeping the rest of the
#           robot's config. Applied after --config. See README, "Motor direction".
set -euo pipefail
source "$(dirname "$0")/common.sh"

if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
  resolve_target "$1"
  shift
else
  resolve_target
fi
CONFIG=""; INVERT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --invert) INVERT="$2"; check_invert_mode "$INVERT"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

require_key_login
echo "deploying agent to $TARGET:~/$FOSSBOT_DIR"
upload_agent
[ -n "$CONFIG" ] && upload_config "$CONFIG"
[ -n "$INVERT" ] && set_motor_invert "$INVERT"
warn_if_unconfigured

# No sudo anywhere: the ssh user must be in the robot's docker group, and sudo
# cannot prompt over a non-interactive ssh.
echo "restarting agent"
rsh "docker exec $CONTAINER pkill -9 -f fossbot_agent.py 2>/dev/null || true
     docker exec -d $CONTAINER bash -c 'cd /ws && python3 fossbot_agent.py > /tmp/agent.log 2>&1'"

# The agent retries its GPIO claim while the kernel reaps the previous owner,
# which takes a few seconds, so allow time before declaring failure.
echo -n "waiting for agent"
for _ in $(seq 1 20); do
  if rsh "docker exec $CONTAINER grep -q '^\[agent\] up\.' /tmp/agent.log" 2>/dev/null; then
    echo; rsh "docker exec $CONTAINER cat /tmp/agent.log"; exit 0
  fi
  echo -n "."; sleep 2
done
echo; echo "agent did not report ready within 40 s. Log so far:" >&2
rsh "docker exec $CONTAINER cat /tmp/agent.log" >&2 || true
exit 1
