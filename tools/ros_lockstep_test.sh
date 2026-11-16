#!/bin/bash
# ROS 2 lockstep run of the MuJoCo validation without building the package:
# starts the plant node and the controller node as two processes and compares
# the result with the in-process run of the same mission.
#   bash tools/ros_lockstep_test.sh [algo] [payload] [gui: true|false] [phase: true|false] [gate_source]
ALGO=${1:-mpc}; PAYLOAD=${2:-on_top}; GUI=${3:-false}     # 3rd argument "true": MuJoCo window
PHASE=${4:-false}; GSRC=${5:-truth}                       # traffic phase; truth|audit|sensor|learned
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/../src/ros2_swarm_transport" || exit 1
OUT="$(cd ../..; pwd)/results/mujoco_ros"
ARGS="--ros-args -p algo:=$ALGO -p payload:=$PAYLOAD -p traffic_phase:=$PHASE -p gate_source:=$GSRC"
SFX=""; [ "$PHASE" = "true" ] && SFX="_ph1"; [ "$GSRC" != "truth" ] && SFX="${SFX}_$GSRC"
NAME="${ALGO}_square_${PAYLOAD}_s1${SFX}"
python3 -m ros2_swarm_transport.mujoco_plant $ARGS -p gui:=$GUI &
PLANT=$!
python3 -m ros2_swarm_transport.formation_controller $ARGS -p output_dir:=$OUT -p run_name:=ros_$NAME
wait $PLANT
PH=""; [ "$PHASE" = "true" ] && PH="--phase"
python3 -m ros2_swarm_transport.mujoco_validation --algo $ALGO --payload $PAYLOAD --output $OUT/direct $PH --gate-source $GSRC
python3 - <<PY
import json
a = json.load(open("$OUT/ros_$NAME.json"))
b = json.load(open("$OUT/direct/$NAME.json"))
keys = ["reached_goal", "total_time", "n_reject", "coll_mover", "formation_rms",
        "formation_err_max", "contact_steps_static", "contact_steps_mover", "pin_gap_max"]
print(f"{'metric':24s} {'ROS 2 lockstep':>16s} {'in-process':>16s}")
for k in keys:
    print(f"{k:24s} {str(round(a[k], 5) if isinstance(a[k], float) else a[k]):>16s} "
          f"{str(round(b[k], 5) if isinstance(b[k], float) else b[k]):>16s}")
PY
