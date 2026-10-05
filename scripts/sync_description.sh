#!/bin/bash
# Re-vendor the URDF and meshes from the simulator's description package.
# Keeps fossbot_description in step with FOSSBotEduSim without making the
# bridge depend on that workspace at build time.
set -e
cd "$(dirname "$0")/.."
SRC="${1:-../FOSSBotEduSim/ws_fossbot/src/fossbot_educational_description}"
DST="ws/src/fossbot_description"
[ -d "$SRC" ] || { echo "no description package at $SRC"; exit 1; }

cp "$SRC"/meshes/*.stl "$DST"/meshes/
cp "$SRC"/urdf/materials.xacro "$DST"/urdf/
python3 - "$SRC" "$DST" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(f"{src}/urdf/fossbot_educational_gazebo.xacro").read()
s = s.replace(
    '<xacro:include filename="$(find fossbot_educational_description)/urdf/fossbot_educational_gazebo.gazebo" />',
    '<!-- gazebo plugins intentionally omitted: on the real robot the hardware\n     bridge publishes /scan, /odom and /joint_states instead. -->')
s = s.replace("fossbot_educational_description", "fossbot_description")
s = s.replace('<robot name="fossbot_educational_gazebo"', '<robot name="fossbot"')
open(f"{dst}/urdf/fossbot.urdf.xacro", "w").write(s)
PY
echo "re-vendored from $SRC"
