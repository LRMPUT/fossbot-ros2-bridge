#!/bin/bash
# Copy the robot-side agent onto the robot without restarting it.
#
#   ./deploy_agent.sh [robot-host]
#
# protocol.py is copied from the ROS package rather than kept as a second copy,
# so the two ends of the link can never drift out of sync.
# Use restart_agent.sh if you also want the agent restarted.
set -euo pipefail

ROBOT="${1:-fossbotrpi1.local}"
USER="${FOSSBOT_USER:-admin}"
DEST="/home/$USER/dev"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=15)

cd "$(dirname "$0")"

echo "deploying to $USER@$ROBOT:$DEST"
scp "${SSH_OPTS[@]}" -q \
    robot_agent/fossbot_agent.py \
    robot_agent/build_camera_stack.sh \
    ws/src/fossbot_bridge/fossbot_bridge/protocol.py \
    "$USER@$ROBOT:$DEST/"
echo "done. restart the agent with:  ./restart_agent.sh $ROBOT"
