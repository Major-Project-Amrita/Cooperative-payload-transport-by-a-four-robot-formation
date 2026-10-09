# Agent prompt — run the swarm mission in Gazebo (paste into a NEW session in ~/ros2_ws)

You are working in `~/ros2_ws` on Ubuntu 22.04 / ROS 2 Humble / gz-sim 8.
The package `ros2_swarm_transport` is already built and installed
(`source ~/ros2_ws/install/setup.bash` works; `ros2 pkg executables
ros2_swarm_transport` lists `formation_controller`, `mover_node`,
`swarm_logger`; all 32 pytest tests pass). Do not modify anything outside
`~/ros2_ws`.

## Context you need

- The MATLAB reference run is `~/ros2_bundle_reference/reference/…` (world
  copy) and the kinematics spec is `~/ros2_bundle_reference/ROS2_REPLICATION.md`
  — read §1 (kinematic model), §4 (dynamic-obstacle contract) and §6
  (architecture) first.
- Params live in `src/ros2_swarm_transport/params/swarm.yaml` (single source).
  `algo: pvfh` is the default; the four steering laws are
  `apf | vfh | pvfh | pvfh_prob`; sensing is `sense_kind: lidar | tof` with
  `sense_sigma0`, `sense_k_rng`, `sense_fov_deg`, `sense_max_range`.
- The launch file `src/ros2_swarm_transport/launch/transport.launch.py` has a
  TODO: it spawns no robots yet.

## Work through these steps in order, stop and report if a step fails

1. `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`.
2. Inspect `transport.launch.py` and `swarm.yaml`. Complete the robot-spawn
   TODO: four TurtleBot3-burger robots named robot1..robot4 at the MATLAB
   start ring (centre (−12, 0), radius 2.2 m, angles 0/90/180/270 deg, yaw
   facing +x), each with `parameter_bridge` for `/robotN/cmd_vel`
   (`@geometry_msgs/msg/Twist@gz.msgs.Twist`), `/robotN/odom`
   (`@nav_msgs/msg/Odometry[gz.msgs.Odometry]`) and `/robotN/scan`
   (`@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan]`), all namespaced.
   Footprint contract: `robot_radius: 0.18` in swarm.yaml (burger is 0.105 —
   either set robot_radius: 0.105 consistently or inflate the model; pick
   one, apply it everywhere, and note it in the report).
3. Smoke the nodes WITHOUT Gazebo first: run `mover_node` and confirm
   `/dynamic_obstacles` publishes three markers (it teleports nothing without
   a world, that is fine). `ros2 topic echo /dynamic_obstacles --once`.
4. Launch the world headless: `gz sim -r -s
   src/ros2_swarm_transport/worlds/swarm_world_dynamic.sdf` (server mode,
   no GUI), then the bridges + nodes. Check `gz topic -l` for the robots'
   topics before wiring bridges.
5. Full mission: start `swarm_logger` (output:=ros2_dynamic.csv rate:=10.0),
   then the launch. Success = `/swarm/phase` reaches 2 and `dist_to_goal` in
   the CSV drops below 0.6 m. If the controller stalls >120 s wall clock,
   dump `/swarm/phase`, the four `/robotN/odom` poses and the controller's
   last 50 log lines, and diagnose against
   `~/ros2_bundle_reference/ROS2_REPLICATION.md` §6.
6. Acceptance report: pytest summary, build warnings, CSV tail (last 5
   rows), min pairwise formation spacing vs 1.18 m nominal, final centroid
   vs (12, 0), wall/sim time, PASS/FAIL per criterion.

## If you only want ONE algorithm comparison run instead

```bash
cd ~/ros2_ws/src/ros2_swarm_transport
python3 -m ros2_swarm_transport.headless_runner --algo pvfh_prob --seed 7 \
    --output /tmp/ros2_headless.csv        # no Gazebo needed
```

`--algo` ∈ {apf, vfh, pvfh, pvfh_prob}, `--formation` ∈ {square, line,
triangle, diamond}, `--sense tof --n-beams 8` for the sparse-sensor runs.
