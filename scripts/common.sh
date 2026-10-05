# shellcheck shell=bash
# Shared by the robot management scripts. Source, do not run.
#
# Environment:
#   FOSSBOT_HOST  robot hostname or IP (the ROS launch reads the same variable)
#   FOSSBOT_USER  ssh user on the robot, if it differs from your local user
#   FOSSBOT_DIR   agent directory on the robot, relative to that user's home
#                 (default: fossbot). Mounted at /ws inside the agent container.

FOSSBOT_DIR="${FOSSBOT_DIR:-fossbot}"
CONTAINER=fb
IMAGE=fossbot-agent
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=15)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Sets TARGET to user@host from the first argument or FOSSBOT_HOST.
resolve_target() {
  local host="${1:-${FOSSBOT_HOST:-}}"
  if [ -z "$host" ]; then
    echo "usage: $(basename "$0") [user@]robot-host [options]" >&2
    echo "       (or set FOSSBOT_HOST, and FOSSBOT_USER if needed)" >&2
    exit 2
  fi
  if [[ "$host" == *@* ]]; then
    TARGET="$host"
  else
    TARGET="${FOSSBOT_USER:+$FOSSBOT_USER@}$host"
  fi
}

rsh() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }

require_key_login() {
  if ! ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "Cannot log in to $TARGET without a password. Install your key once:" >&2
    echo "    ssh-copy-id $TARGET" >&2
    exit 1
  fi
}

# Copies the agent, its shared protocol module and the provisioning files.
# Never touches agent_config.json on the robot -- that file is per robot.
upload_agent() {
  local agent="$REPO_ROOT/robot_agent"
  tar -cf - \
      -C "$agent" fossbot_agent.py agent_config.py pinmap.py config.example.json \
                  Dockerfile build_camera_stack.sh http-time-sync.sh systemd \
      -C "$REPO_ROOT/ws/src/fossbot_bridge/fossbot_bridge" protocol.py \
    | rsh "mkdir -p '$FOSSBOT_DIR' && tar -C '$FOSSBOT_DIR' -xf -"
}

# Uploads a per-robot config file as the robot's agent_config.json.
upload_config() {
  local file="$1"
  [ -f "$file" ] || { echo "no such config file: $file" >&2; exit 1; }
  python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$file" \
    || { echo "$file is not valid JSON" >&2; exit 1; }
  rsh "cat > '$FOSSBOT_DIR/agent_config.json'" < "$file"
  echo "installed $file as $FOSSBOT_DIR/agent_config.json"
}

# Validates an --invert value before anything touches the robot.
check_invert_mode() {
  case "$1" in
    none|left|right|both) ;;
    *) echo "--invert must be one of: none, left, right, both (got '$1')" >&2; exit 2 ;;
  esac
}

# Sets motor_left_invert / motor_right_invert in the robot's agent_config.json,
# keeping every other key. Creates the file if the robot has none.
set_motor_invert() {
  local l=false r=false
  case "$1" in
    left) l=true ;; right) r=true ;; both) l=true; r=true ;;
  esac
  rsh "python3 - '$FOSSBOT_DIR/agent_config.json' $l $r" <<'PY_EOF'
import json, os, sys
path, left, right = sys.argv[1], sys.argv[2] == "true", sys.argv[3] == "true"
cfg = {}
if os.path.exists(path):
    with open(path) as f:
        cfg = json.load(f)
cfg["motor_left_invert"] = left
cfg["motor_right_invert"] = right
tmp = path + ".tmp"
with open(tmp, "w") as f:
    json.dump(cfg, f, indent=2)
    f.write("\n")
os.replace(tmp, path)
print(f"{path}: motor_left_invert={left}, motor_right_invert={right}")
PY_EOF
}

warn_if_unconfigured() {
  if ! rsh "test -f '$FOSSBOT_DIR/agent_config.json'"; then
    echo "WARNING: $TARGET has no $FOSSBOT_DIR/agent_config.json." >&2
    echo "         The agent runs on generic defaults, and motor direction is" >&2
    echo "         unverified for this robot. See README, 'Per-robot setup'." >&2
  fi
}
