#!/bin/bash
# ROS 2 runs recorded for the demo video (inside a Humble environment):
#   1. ros2 launch of the physics validation, video rendered by the mujoco_plant node
#   2. real-sensor deployment over DDS with a full trace (poses + published scans)
#   3. the watchdog test with a trace
REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/results/ros_humble/demo"; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
WS=/tmp/swarm_ws; rm -rf $WS; mkdir -p $WS/src; ln -s "$REPO/src/ros2_swarm_transport" $WS/src/
(cd $WS && colcon build --symlink-install 2>&1 | tail -1)
source $WS/install/setup.bash
export MUJOCO_GL=${MUJOCO_GL:-osmesa}
stop_launch() { kill -INT "$1" 2>/dev/null; for i in $(seq 10); do kill -0 "$1" 2>/dev/null || break; sleep 1; done
  kill -KILL "$1" 2>/dev/null; pkill -KILL -f "lib/ros2_swarm_transport/formation_controller" 2>/dev/null; wait "$1" 2>/dev/null; sleep 1; }
echo "=== ros2 launch mujoco_validation.launch.py algo:=pavfh gate_margin:=0.10 (video from the plant node)"
timeout 1500 ros2 launch ros2_swarm_transport mujoco_validation.launch.py algo:=pavfh payload:=on_top \
  traffic_phase:=true gate_margin:=0.10 video:=$OUT/ros2_launch_pavfh.mp4 output_dir:=$OUT \
  run_name:=ros2_launch_pavfh 2>&1 | tee $OUT/ros2_launch.log | grep -E "plant up|formed|mission" | tail -4
W=$REPO/src/ros2_swarm_transport/worlds
for run in "deploy 2 -1.0" "watchdog 1 20.0"; do
  set -- $run
  echo "=== robots.launch.py + fake robots over DDS: $1 (seed $2)"
  ros2 launch ros2_swarm_transport robots.launch.py world_sdf:=$W/swarm_world_dynamic.sdf \
      contract:=$W/dynamic_obstacles.json seed:=$2 traffic_phase:=true kin:=false > $OUT/$1_launch.log 2>&1 &
  L=$!
  timeout 600 python3 "$REPO/tools/ros_fake_robots.py" --ros-args -p seed:=$2 -p traffic_phase:=true \
      -p world_sdf:=$W/swarm_world_dynamic.sdf -p contract:=$W/dynamic_obstacles.json -p kin:=false \
      -p sigma:=0.02 -p gate_source:=sensor -p drop_after:=$3 -p out:=$OUT/$1 2>&1 | grep -E "^\[INFO\].*\{" | tail -1
  stop_launch $L
done
echo "=== DONE"
