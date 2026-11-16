# ROS 2 validation and deployment readiness

What has been validated for the ROS 2 package, how, and what remains before
hardware. Numbers come from result files in `results/`.

## 1. Three levels of validation

| Level | What runs | Where | Status |
|---|---|---|---|
| **A. Node code, in-memory middleware** | The package's real `mujoco_plant` and `formation_controller` node classes. Topics go through an in-process stand-in for rclpy (`tools/ros_lockstep_stub.py`). | Any machine, no ROS 2 needed; part of the pytest suite | **Done.** Identical to the in-process run (every compared metric equal) for MPC, the DAgger policy (NN) and PA-VFH. Also identical for PA-VFH and NN with the learned monitor and randomised traffic phase. |
| **B. Real-sensor closed loop** | The controller node in free-running mode with `use_scans:=true`, so obstacles reach it only as per-robot `LaserScan` messages (`tools/ros_sensor_closedloop.py`). | Any machine | **Done.** Results in §2. |
| **C. Real ROS 2 (Humble), DDS** | Plant and controller as two ROS 2 processes over DDS (`tools/ros_lockstep_test.sh`; `tools/ros_humble_validation.sh` runs 7 cases). | Machine with ROS 2 Humble | MPC was run earlier and matched the in-process run. **The neural laws (NN, PA-VFH) and the monitor modes are still to be run over real DDS.** |

Level A checks node logic, parameters, message packing and the lockstep
protocol. It does not check DDS transport, QoS, executors, launch files or
the colcon install; only level C covers those.

### Running level C on a Humble machine

```bash
source /opt/ros/humble/setup.bash
bash tools/ros_humble_validation.sh 2>&1 | tee results/ros_humble_validation.log
```

The script runs seven cases:
- MPC, NN, PA-VFH and PA-VFH (no NN), all with the truth gate;
- PA-VFH and NN with the learned monitor and random phase;
- PA-VFH with the sensor monitor.

Each case prints the ROS 2 result next to the in-process result. They
should be identical.

## 2. Real-sensor closed loop (level B)

Setup:
- PA-VFH steering; randomised traffic phase.
- Ideal unicycle robots: kinematic, no physics.
- Each robot publishes a 360-beam `LaserScan` (8 m range, Gaussian range
  noise σ) ray-cast from its own pose. The scan sees the static world, the
  movers where they are now, and its three team-mates.
- The controller never reads the world model or the traffic contract for
  steering or for the gate decision.
- Contacts are scored afterwards with the benchmark's ground-truth contact
  test (`results/ros_sensor_closedloop.json`).

| world | monitor | σ (m) | missions | reached | runs with contact | runs with *static* contact | monitor false accepts / checks |
|---|---|---|---|---|---|---|---|
| AWS warehouse | sensor | 0.02 | 6 | **6** | 1 | 0 | 0 / 4,484 |
| AWS warehouse | learned | 0.02 | 6 | **6** | 1 | 0 | 0 / 5,171 |
| study warehouse (dynamic) | sensor | 0.02 | 6 | 5 | 6 | 0 | 2 / 4,591 |
| study warehouse (dynamic) | learned | 0.02 | 6 | 5 | 6 | 2 | 27 / 3,692 |
| study warehouse (dynamic) | sensor | 0.05 | 6 | 3 | 6 | 0 | 0 / 6,223 |
| study warehouse (dynamic) | learned | 0.05 | 6 | 3 | 6 | 0 | 29 / 4,944 |
| study warehouse (dynamic) | sensor | 0.10 | 6 | 0 | 6 | 4 | 0 / 4,908 |
| study warehouse (dynamic) | learned | 0.10 | 6 | 0 | 6 | 4 | 58 / 3,942 |

How to read it:
- **The real-sensor pipeline works end to end.** At low noise the
  formation completes 22 of 24 missions from LaserScans alone. In the AWS
  warehouse it completes 12 of 12, with no static contact.
- **The geometric sensor monitor almost never admits an unsafe pose:** 2
  false accepts in about 20,000 checks.
- **The learned monitor admits more unsafe poses** (27–58 per few thousand
  checks). Its conformal guarantee holds only in-distribution: it was
  calibrated on the simulated *centroid* scan, not on fused per-robot scans.
  Re-calibrate it on this sensor pipeline before using it.
- **Contacts in the study warehouse are mover contacts.** Movers there are
  kinematic and never yield, so they drive into the formation. The same
  effect shows in the benchmark's E17 (R5).
- **The pipeline breaks down at σ ≥ 0.05 m.** At 0.10 m it completes no
  mission, and the robots start to touch statics: the monitor's inflation
  grows with σ until the aisles look closed. This is the most important
  thing to fix before hardware.

## 3. Deployment interface (`launch/robots.launch.py`)

**Subscribes** (per robot `<r>`):
- `/<r>/odom` (`nav_msgs/Odometry`, map frame);
- `/<r>/scan` (`sensor_msgs/LaserScan`).

**Publishes:**
- `/<r>/cmd_vel` (`geometry_msgs/Twist`);
- `/swarm/phase` (`std_msgs/Int32`).

**Inputs:**
- `world_sdf`: the static site map, used for formation planning and the
  route;
- `contract`: start, waypoints and goal. Its mover list may be empty.

**Behaviour:**
- **Refuses** any configuration that needs ground truth or the traffic
  contract's future. It accepts only `algo` pavfh or pavfh_nonn with
  `gate_source` sensor or learned.
- **Holds still** until every robot has sent odometry and a scan.
- Scan mount offset: `scan_mount`; stale-data limit: `scan_max_age`.

## 4. Before hardware (open items)

1. **Run level C on Humble.** Then test on a robot simulator with real
   sensor plugins (Gazebo + TurtleBot-class robots) using `robots.launch.py`.
2. **Noise robustness.** At σ ≥ 0.05 m reach collapses.
   - Calibrate σ per sensor.
   - Make the monitor inflation σ-adaptive per beam rather than k·σ0
     globally.
   - Re-calibrate the learned monitor on fused per-robot scans.
3. **Physics margin.** In MuJoCo, PA-VFH touches static racks in every run
   ([PA_VFH.md](PA_VFH.md) §4). Enlarge the swept radius by the measured
   tracking error before running with a load.
4. **Safety layer independent of the planner.**
   - A hardware or software emergency stop on `/swarm/phase` and on stale
     scans or odometry. The node holds still on missing data, but a
     watchdog must also cut `cmd_vel`.
   - A stopping-distance check at the chosen top speed.
   - Load-attachment monitoring. The pin-gap signal exists only in the
     simulator.
5. **Localisation.** Odometry must be in the shared map frame. The only evidence so far is grid
   experiment E10: localisation noise of 0.05, 0.15 and 0.30 m, in
   simulation.
6. **Moving people.** All movers in the study are constant-velocity
   shuttles that do not react. Nothing here is validated around people.

Until items 1–4 are closed, the package is a **research benchmark and a
simulation-validated controller**, not a safety-rated product.
