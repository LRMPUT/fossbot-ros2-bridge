#!/bin/bash
# Deploy the current agent source to the robot and restart it.
#
#   ./restart_agent.sh [robot-host]
#
# No sudo: the `admin` user is in the robot's `docker` group, so docker works
# directly. Calling sudo over a non-interactive ssh fails with "sudo: a terminal
# is required to read the password", which is what this script used to do.
#
# Passwordless ssh is expected. If you are still being prompted, install your
# key once with:
#     ssh-copy-id admin@fossbotrpi1.local
set -euo pipefail

ROBOT="${1:-fossbotrpi1.local}"
USER="${FOSSBOT_USER:-admin}"
DEST="/home/$USER/dev"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=15)

cd "$(dirname "$0")"

if ! ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$USER@$ROBOT" true 2>/dev/null; then
    echo "Cannot log in to $USER@$ROBOT without a password."
    echo "Install your key once, then re-run this script:"
    echo "    ssh-copy-id $USER@$ROBOT"
    exit 1
fi

echo "deploying agent to $USER@$ROBOT:$DEST"
scp "${SSH_OPTS[@]}" -q \
    robot_agent/fossbot_agent.py \
    ws/src/fossbot_bridge/fossbot_bridge/protocol.py \
    "$USER@$ROBOT:$DEST/"

echo "restarting agent"
ssh "${SSH_OPTS[@]}" "$USER@$ROBOT" '
    docker exec fb pkill -9 -f fossbot_agent.py 2>/dev/null || true
    docker exec -d fb bash -c "cd /ws && python3 fossbot_agent.py > /tmp/agent.log 2>&1"
'

# The agent retries the GPIO claim while the kernel reaps the previous owner,
# which takes a few seconds, so do not declare failure too early.
echo -n "waiting for agent"
for _ in $(seq 1 20); do
    if ssh "${SSH_OPTS[@]}" "$USER@$ROBOT" \
        'docker exec fb grep -q "^\[agent\] up\." /tmp/agent.log' 2>/dev/null; then
        echo
        ssh "${SSH_OPTS[@]}" "$USER@$ROBOT" 'docker exec fb cat /tmp/agent.log'
        exit 0
    fi
    echo -n "."
    sleep 2
done

echo
echo "agent did not report ready within 40 s. Log so far:"
ssh "${SSH_OPTS[@]}" "$USER@$ROBOT" 'docker exec fb cat /tmp/agent.log' || true
exit 1
