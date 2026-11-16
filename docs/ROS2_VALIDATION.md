# ROS 2 validation and deployment readiness

What has been validated for the ROS 2 package, how, and what remains before
hardware. Numbers come from result files in `results/`.

## 1. Three levels of validation

| Level | What runs | Where | Status |
|---|---|---|---|
| **A. Node code, in-memory middleware** | The package's real `mujoco_plant` and `formation_controller` node classes. Topics go through an in-process stand-in for rclpy (`tools/ros_lockstep_stub.py`). | Any machine, no ROS 2 needed; part of the pytest suite | **Done.** Identical to the in-process run (every compared metric equal) for MPC, the DAgger policy (NN) and PA-VFH. Also identical for PA-VFH and NN with the learned monitor and randomised traffic phase. |
| **B. Real-sensor closed loop** | The controller node in free-running mode with `use_scans:=true`, so obstacles reach it only as per-robot `LaserScan` messages (`tools/ros_sensor_closedloop.py`). | Any machine | **Done.** Results in §2. |
| **C. Real ROS 2 (Humble), DDS** | Every check through real rclpy and DDS (`tools/ros_humble_full.sh`): colcon build, the test suite, lockstep, `ros2 launch`, real-sensor deployment, the watchdog. | Official `ros:humble-ros-base` Docker image, or any Humble machine | **Done** (§1b). |

Level A checks node logic, parameters, message packing and the lockstep
protocol. It does not check DDS transport, QoS, executors, launch files or
the colcon install; level C covers those.

### 1b. Level C results (ROS 2 Humble, Python 3.10, real rclpy, DDS)

Logs: `results/ros_humble/full.log`, `full_stage1_2.log`,
`full_stage2_5_6.log`.

| Stage | Result |
|---|---|
| 1. `colcon build`, then `ros2 pkg executables` | builds; 4 executables installed |
| 2. Test suite | **74 passed**, 3 skipped by design: the stand-in versions of checks that stages 3, 5 and 6 run over real DDS |
| 3. Lockstep, plant and controller as two ROS 2 processes vs in-process | **7 of 7 cases identical on all 9 metrics** (below) |
| 4. `ros2 launch mujoco_validation.launch.py` (installed package): PA-VFH, random traffic phase, learned monitor | reached in 115.5 s; the monitor admitted 0 unsafe poses in 1,203 checks; 4 static-contact steps (run without the 0.10 m margin) |
| 5. Real-sensor deployment, `robots.launch.py` with robots publishing `/odom` and `/scan` over DDS, σ = 0.02, random phase | **3 of 3 reached, 0 static-contact steps**; mover contacts in all 3 (shuttles that never yield) |
| 6. Watchdog: robot1's lidar stops at t = 20.0 s | `/swarm/safety` = 1 at **t = 20.6 s** (0.5 s timeout + one control period); every later `cmd_vel` is 0 (23 messages) |
| `cmd_vel` cadence over DDS | median 100 ms, 95th percentile 104 ms (target 100 ms) |

Lockstep cases (ROS 2 result = in-process result):

| Law | Payload | Traffic phase | Gate | Time (s) | Static-contact steps |
|---|---|---|---|---|---|
| MPC | on top | nominal | truth | 57.8 | 0 |
| NN (DAgger) | on top | nominal | truth | 58.4 | 0 |
| PA-VFH | on top | nominal | truth | 101.0 | 85 (no margin) |
| PA-VFH (no NN) | on top | nominal | truth | 97.6 | 44 (no margin) |
| PA-VFH | on top | random | learned monitor | 115.5 | 4 |
| NN (DAgger) | on top | random | learned monitor | 66.3 | 0 |
| PA-VFH | virtual | random | sensor monitor | 90.3 | 0 |

The signal plots S5 and S6 in [SIGNALS.md](SIGNALS.md) show the same runs
over time.

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

### 1c. MuJoCo physics validation through ROS 2 (R1, R2)

Full tables: [EXTENDED_RESULTS.md](EXTENDED_RESULTS.md) R10. Tool:
`tools/run_ros2_physics.py`, run inside Humble.

| Batch | What runs | Result |
|---|---|---|
| **R1** (90 runs) | The P3 physics configuration through ROS 2 lockstep: 6 laws × 3 payload modes × 5 seeds, 0.10 m margin, random traffic phase. The MuJoCo plant node and the controller node run over DDS, each job on its own DDS domain. | **90/90 identical** on every metric to the in-process physics run in the same environment, so ROS 2 adds no error. Against the host's run (a different MuJoCo version), 43/90 are identical; the rest diverge from about 1e-14 floating-point differences that grow in chaotic missions. Every run reached the goal. PA-VFH is clean in 8/10 runs with the carried or virtual load. The towed load is 0/5 for PA-VFH and at most 2/5 for any law. |
| **R2** (10 runs) | The full deployment stack in **real-time physics**: the MuJoCo plant runs free, applies `/robotN/cmd_vel`, and publishes `/robotN/odom` and `/robotN/scan` from the physical robots. The controller uses the `robots.launch.py` configuration (PA-VFH, sensor monitor, watchdog, 0.10 m margin) and sees only the scans. | **9/10 reached with zero contacts** in the physics engine. The tenth (load on top, seed 2) was hit by a shuttle (peak contact force 2,017 N) and timed out 7 m from the goal. Pin gap ≤ 0.76 mm and tilt ≤ 0.04° in the successful runs with the load on top. |
| **Watchdog in physics** | robot1's lidar stops publishing at t = 20.0 s | `/swarm/safety` = 1 at **t = 20.3 s**; every later command is 0 m/s and the MuJoCo robots stop. No contacts. |
| **Sim-time run** | The same deployment while rendering video, so the simulator runs about 3.5 × slower than real time | With the wall clock, the controller drifts ahead of the physics: watchdog trips, timeout and contacts. With **`use_sim_time:=true`** (the controller's timer and watchdog follow `/clock`), the run reached the goal with 0 contacts. **Lesson for deployment:** run the controller on sim time with any simulator that can fall behind real time. |

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
- **The pipeline breaks down at σ ≥ 0.05 m** (version 1 of the pipeline).
  At 0.10 m it completes no mission, and the robots start to touch statics:
  the monitor's inflation grows with σ until the aisles look closed.

### 2b. Noise robustness (version 2 of the sensing pipeline)

Two changes:
1. **The self-filter widens with noise** (r + 0.10 + 3σ). Noisy returns
   from team-mates had looked like obstacles next to the formation slots,
   and the team never finished forming.
2. **Each virtual-scan beam uses the median of its three nearest returns
   instead of the minimum.** The minimum of n noisy returns is biased short
   by about σ·√(2 ln n).

Study warehouse, sensor monitor, 6 seeds per row, random phase:

| σ (m) | reached v1 → v2 | runs with static contact v1 → v2 | monitor false accepts v1 → v2 |
|---|---|---|---|
| 0.02 | 5 → 5 | 0 → 0 | 2 / 4,591 → 8 / 4,411 |
| 0.05 | 3 → **5** | 0 → 0 | 0 / 6,223 → 19 / 4,770 |
| 0.10 | 0 → **3** | 4 → **0** | 0 / 4,908 → 25 / 29,076 |

**Trade-off.** Version 2 recovers reach and removes the static contacts at
high noise, but it is less conservative: it admits 0.1–0.4 % unsafe poses
where version 1 admitted almost none.

One idea was tried and **reverted** because it made things worse:
- **The idea:** let the static map exempt explained returns from the
  moving-obstacle allowance.
- **The result:** the monitor accepted 2 % of moves, and static contacts
  appeared.

**Sensor requirement that follows:** range noise σ ≤ 0.05 m, which typical
2-D lidars meet (±1–3 cm). At σ = 0.10 m, the 3σ inflation plus the robot
radius nearly fills the gap between the formation and the wall in the
2.45 m corridor. No monitor that keeps a 3σ margin can fix that.

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

1. ~~Run level C on Humble.~~ **Done** (§1b). Next: a robot simulator
   with real sensor plugins (Gazebo + TurtleBot-class robots) using
   `robots.launch.py`, then hardware.
2. **Noise robustness.** Partly done (§2b): reach recovers up to σ = 0.05 m.
   Still open:
   - calibrate σ per sensor;
   - re-calibrate the learned monitor on fused per-robot scans;
   - work at σ = 0.10 m, which needs a better sensor rather than a looser
     monitor.
3. **Physics margin.** Done for the carried and virtual load. With the
   0.10 m gate margin, PA-VFH is clean in 16 of 20 P3 runs (0 of 20
   before). In the robot-parameter sweep it is clean in 22 of 40 (0 of 40
   before), still behind MPC (39 of 40). The towed ("behind") load stays
   0 of 10 for PA-VFH and 2 of 10 for MPC. The towed load swings outside
   the formation footprint the planner models, so the open item is a
   planner model of the towed payload.
4. **Safety layer independent of the planner.**
   - ~~Software watchdog.~~ **Done**: stale `/odom` or `/scan` (0.5 s)
     stops every robot, `/swarm/safety` reads 1, every command is clamped
     to `max_v` and `max_w`, and zero velocity is sent on shutdown.
     Measured over DDS: stop 0.6 s after a lidar failure.
   - Still needed:
     - a **hardware** emergency stop;
     - a stopping-distance check at the chosen top speed;
     - load-attachment monitoring (the pin-gap signal exists only in the
       simulator).
5. **Localisation.** Odometry must be in the shared map frame. The only evidence so far is grid
   experiment E10: localisation noise of 0.05, 0.15 and 0.30 m, in
   simulation.
6. **Moving people.** All movers in the study are constant-velocity
   shuttles that do not react. Nothing here is validated around people.

Until the open parts of items 1–6 are closed, the package is a
**research benchmark and a simulation-validated, ROS 2-integrated
controller**, not a safety-rated product.
