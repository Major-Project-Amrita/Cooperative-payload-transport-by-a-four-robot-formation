#!/bin/bash
# Full ROS 2 Humble validation, run INSIDE a ROS 2 Humble environment
# (a Humble machine, or the container: see docs/RUN_GUIDE.md section 6).
#   bash tools/ros_humble_full.sh 2>&1 | tee results/ros_humble/full.log
# 1 colcon build + install of the package      4 ros2 launch of the physics validation (installed)
# 2 the test suite under Humble's Python       5 real-sensor deployment (robots.launch.py + fake robots over DDS)
# 3 lockstep ROS 2 vs in-process, 7 cases      6 watchdog: a robot's lidar fails mid-mission

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/results/ros_humble"; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
echo "=== 1. colcon build"
WS=/tmp/swarm_ws; rm -rf $WS; mkdir -p $WS/src; ln -s "$REPO/src/ros2_swarm_transport" $WS/src/
(cd $WS && colcon build --symlink-install --event-handlers console_cohesion- 2>&1 | tail -3)
source $WS/install/setup.bash
ros2 pkg executables ros2_swarm_transport
echo "=== 2. test suite (Python $(python3 -c 'import sys;print(sys.version.split()[0])'), real rclpy)"
(cd "$REPO/src/ros2_swarm_transport" && python3 -m pytest test -q -p no:cacheprovider 2>&1 | tail -3)
echo "=== 3. lockstep: ROS 2 processes vs in-process"
bash "$REPO/tools/ros_humble_validation.sh"
echo "=== 4. ros2 launch (installed package): PA-VFH, random phase, learned monitor"
timeout 900 ros2 launch ros2_swarm_transport mujoco_validation.launch.py algo:=pavfh \
    payload:=on_top traffic_phase:=true gate_source:=learned output_dir:=$OUT \
    run_name:=launch_pavfh_on_top_learned 2>&1 | grep -E "mission|formed|plant up" | tail -4
python3 -c "import json;d=json.load(open('$OUT/launch_pavfh_on_top_learned.json'));print({k:d[k] for k in ('reached_goal','total_time','contact_steps_static','contact_steps_mover','formation_rms','gate_source','gate_stats')})"
echo "=== 5. real-sensor deployment over DDS (robots.launch.py + fake robots)"
W=$REPO/src/ros2_swarm_transport/worlds
for s in 1 2 3; do
  ros2 launch ros2_swarm_transport robots.launch.py world_sdf:=$W/swarm_world_dynamic.sdf \
      contract:=$W/dynamic_obstacles.json seed:=$s traffic_phase:=true kin:=false \
      > $OUT/robots_launch_s$s.log 2>&1 &
  L=$!
  timeout 600 python3 "$REPO/tools/ros_fake_robots.py" --ros-args -p seed:=$s -p traffic_phase:=true \
      -p world_sdf:=$W/swarm_world_dynamic.sdf -p contract:=$W/dynamic_obstacles.json \
      -p kin:=false -p sigma:=0.02 -p out:=$OUT/real_sensor_s$s 2>&1 | grep -E "^\[INFO\].*\{" | tail -1
  kill -INT $L; wait $L 2>/dev/null
done
echo "=== 6. watchdog: robot1's lidar stops publishing at t = 20 s"
ros2 launch ros2_swarm_transport robots.launch.py world_sdf:=$W/swarm_world_dynamic.sdf \
    contract:=$W/dynamic_obstacles.json seed:=1 kin:=false > $OUT/watchdog_launch.log 2>&1 &
L=$!
timeout 300 python3 "$REPO/tools/ros_fake_robots.py" --ros-args -p seed:=1 -p kin:=false \
    -p world_sdf:=$W/swarm_world_dynamic.sdf -p contract:=$W/dynamic_obstacles.json \
    -p drop_after:=20.0 -p out:=$OUT/watchdog 2>&1 | grep -E "^\[INFO\].*\{" | tail -1
kill -INT $L; wait $L 2>/dev/null
grep -h "watchdog" $OUT/watchdog_launch.log | head -2
echo "=== DONE"
