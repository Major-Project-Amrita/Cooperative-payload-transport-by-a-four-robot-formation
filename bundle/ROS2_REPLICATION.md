# Replicating the MATLAB Swarm-Transport Model in ROS 2

This document specifies **exactly** what the MATLAB simulation computes, which
parts are the kinematic model proper, which parameters must be mirrored, and
how the dynamic-obstacle experiment carries over. It is the contract between
`Simulation8_dynamic.m` (this repo) and the ROS 2 package scaffolded in
`ros2_swarm_transport/`.

Read together with `README.md` (comparison protocol, metric definitions).

> **READ FIRST — October 2026 audit.** The MATLAB reference changed after this
> document's sections 1–8 were written, and the Python package in this bundle
> has **not** been updated to match. Section 9 at the end is the authoritative
> list of what the port must change; where sections 1–8 disagree with
> section 9, section 9 wins. Results produced by the un-updated package
> (`results_headless*.csv`) are superseded.

---

## 1. What the MATLAB model actually is (and is not)

The MATLAB pipeline is a **kinematic** formation-transport model. It contains
no dynamics: no mass, no inertia, no wheel slip. Every motion constraint is
imposed directly on the *rates*:

| Quantity | Model | Where |
|---|---|---|
| State (each robot, and the formation leader) | `p ∈ R²`, `θ ∈ S¹` | `Simulation8_dynamic.m` |
| Kinematic update | `p ← p + v·[cosθ, sinθ]·dt`, `θ ← θ + clip(wrap(θ_des − θ), ±ω_max)` | Phase loops |
| Speed | constant `V` per phase, scaled by proximity logic | `cfg.V`, `cfg.forming.V`, `speed_scale` |
| Max turn rate | hard clip `ω_max = 0.3 rad/s` (transport), `0.5` (forming) | `cfg.max_turn_rate` |
| Heading smoothing | first-order low-pass: `θ_des ← θ + α·wrap(θ_raw − θ)`, `α = 0.35` | both phases |
| Control period | `dt = 0.1 s` (10 Hz) | `cfg.dt` |

This is precisely the **unicycle / differential-drive kinematic model without
the wheel-level layer**. Its ROS 2 analogue is a `cmd_vel` publisher:
`v = V`, `ω = (θ_new − θ_old)/dt` — a differential-drive base with an internal
velocity controller (the TurtleBot3 `diff_drive` plugin, or your own
`DiffDriveController`) tracks these setpoints. Anything the ROS 2 stack adds
below `cmd_vel` (acceleration limits, wheel odometry, slip) is *extra* dynamics
that MATLAB does not model — expect it to show up as small formation_rms and
turning differences in `compare_runs`, not as a different trajectory shape.

### 1.1 The two-phase mission

**Phase 1 — forming up** (each robot independently, 10 Hz):

```
per robot i:
  e_i   = || slot_i − p_i ||                       # slot = rigid-frame target
  v_i   = V_form · min(1, e_i / slow_radius)       # decelerate on approach
  f_att = w_slot · (slot_i − p_i)/||slot_i − p_i|| # conic attraction
  f_rep = Σ_mates w_rep·(rep_dist − d)·unit(p_i − p_j)   # d < rep_dist
  g     = p_i + f_att + f_rep                      # CASCADED: virtual goal
  θ_raw = VFH(p_i, g, scan)                        # obstacle layer steers
```

Plus a **convoy/queueing** mode: robots within `convoy_trigger_dist` of a
*compact static* obstacle (`obstacle_extents ≤ convoy_max_obstacle`) queue
single-file behind the group leader along its trail (`point_along_trail_behind`,
spacing `convoy_spacing`). Movers are never convoy anchors. Success:
`max_i e_i < tolerance = 0.3 m`.

**Phase 2 — rigid transport** (one virtual leader carries the payload frame):

```
sense_margin = circumradius(offsets) + robot_radius      # VFH+ body inflation
vec_goal = (goal − c)/||goal − c||                       # pure attraction
vec_vfh  = VFH(c, goal, scan, margin)                    # obstacle steering
θ_raw    = atan2( w_goal·vec_goal + w_vfh·vec_vfh )      # WEIGHTED SUM blend
[slowdown: v = V · speed_scale(c, movers)]               # dynamic obstacles
c ← c + v·V·[cosθ sinθ] dt
commit only if formation_clearance(c, θ, movers) > 0 at t AND t+dt
```

The four robots are *slaves of the frame*: `p_i = c + R(θ)·offset_i`. In
MATLAB this is exact by construction; in ROS 2 each robot servo-references its
slot and the residual is real formation error (see §6).

---

## 2. Formation geometry and the payload

- Geometry: `N×N` grid, `N = grid_size = 2`, spacing `s = grid_spacing`
  (auto-reduced to fit the corridor — **use the value the MATLAB run prints**;
  the medium/dynamic world needs ≤ 1.48 m, 1.18 m is used).
- Slot offsets: `meshgrid(0:N-1)`·s, centred → body-frame `offsets` (K×2).
- The **payload** is the rigid body spanned by the formation:
  `collision_mode = 'payload'` treats the whole block (inflated by
  `robot_radius`) as solid via an exact separating-axis test
  (`formation_clearance.m`, ported to `geometry.py`). The load must never
  clip an obstacle *or a moving forklift*, and the heading is low-pass
  smoothed (`α = 0.35`) specifically so the payload is not yanked.
- **Slot assignment** at forming time: Hungarian assignment of robots to
  target slots (`assign_slots.m` → `assign_slots.py`, SciPy
  `linear_sum_assignment` is the same algorithm).
- Leader logging is the **centroid** and a **circular-mean yaw**, never a
  designated robot (README, §Shared CSV schema).

---

## 3. Perception: the sensing-model contract

`lidar_scan.m` is an analytic raycast LaserScan — the *same input modality*
as `/scan`. To replicate:

- **Beams:** `cfg.vfh.n_beams = 288` over ±π (open interval, no duplicate
  beam at −π/π). Match your real LiDAR's count or remap.
- **Range:** `sense_range_base + margin = 4.0 + (circumradius + robot_radius)`.
- **p-VFH** (`pvfh_scan.m` → `vfh.py`): 72 sectors, nearest-hit-per-sector,
  VFH+ enlarged obstacles (`asin(r_eff/d)` bearing blocking, `r_eff =
  margin`), `σ` auto-derived from the required clearance
  (`d_react = r_eff + react_gap = 1.0 m`), circular histogram smoothing
  (window 5), threshold 0.3, valley search limited to ±45° of goal bearing,
  hysteresis weight 0.6, close-range repulsion `avoid_gain 5`,
  `danger_dist = 1.2 + margin`.
- **P-VFH** (`pvfh_prob.m` → `vfh.py::pvfh_prob`): the probabilistic variant
  (`algo: pvfh_prob`). Two per-beam layers from the SAME noisy scan: the risk
  layer `p_b = Φ((d_block − d_meas)/s_b)` (noisy-OR per sector, `p_unknown`
  floor for unobserved/drop-cut beams) and the posterior-expected kernel
  `u_b = √(σ_h²/(σ_h²+s_b²))·exp(−d_meas²/(2(σ_h²+s_b²)))` that is actually
  thresholded (graded, never saturates; exact deterministic limit as s_b → 0;
  inflates monotonically with sensor noise). Control topology identical to
  the deterministic stack; plus the APF reflex fused with `avoid_gain`.
- **Noise model** (`sensor_noise.m` → `vfh.py::sensor_noise`): every variant
  consumes the same noisy scan — `'lidar'`: `σ(d) = σ0`; `'tof'`: FoV arc
  `fov_deg` centred on +x, `σ(d) = σ0 + k_rng·d`, plus a max-range drop-cut.
  `sigma0 = 0.02`, `k_rng = 0.03` replicate the deterministic reference runs.
- `danger_dist_base = 1.2` (NOT 2.5 — see README) and every other knob in
  §5 must be mirrored or the comparison is meaningless.

## 3.1 Dynamic obstacles in perception

`pvfh_predict.m` extends the scan: movers are raycast at their contracted
poses over the prediction horizon (`pred_lead = 0.5 s`, sampled at
`pred_dt = 0.1 s`) and each beam keeps the nearest blocking instance. The
**static** sensing path is untouched — do not shift the sensor position by
self-motion; that variant couples heading into static perception and causes
heading limit cycles (found and removed during development).

In ROS 2 the clean equivalent: maintain a **local mover track list** (from the
`/dynamic_obstacles` topic, §4.3), and when building the VFH histogram,
raycast the tracked movers at their predicted future poses for the next
`pred_lead` seconds into the sector histogram. Static costmap/LiDAR handling
stays exactly as in the static world.

---

## 4. The dynamic-obstacle experiment

### 4.1 World

`swarm_world_dynamic.sdf` = the medium warehouse **plus three movers**
(0.7 m forklift ×2, 1.0 m drum), declared `<static>true</static>` and
teleported every tick. Statics are identical to `swarm_world_medium.sdf`
(goal pad at (12, 0)).

### 4.2 The shared motion contract — `dynamic_obstacles.json`

ONE file drives both stacks. Frame: world/`map`, metres, ENU. Motion model
(`eval_mover_pose.m`, mirrored by `mover_node.py`):

```
t < t0        parked at route[0]
loop=false    translate route[0]→route[1] at speed, stop
loop=true     leg time T = L/speed + wait; motion first, then pause at the
              endpoint; direction alternates each leg; position continuous
```

Default scenario: mover1 (0.9 m/s) crosses west→east at y=+0.5 — the
formation must let it pass; mover2 (0.8 m/s) sweeps the aisle head-on
forever; mover3 patrols the south aisle as background traffic.

### 4.3 ROS 2 wiring

- **`mover_node.py`** loads the JSON, publishes
  `visualization_msgs/Marker` on `/dynamic_obstacles` (one Marker per mover,
  `pose` + `velocity` in `marker.color` is abuse-free enough; or extend with
  a custom msg) and teleports the Gazebo models each tick with
  `gz service -s /world/<world>/set_pose` (or `ros_gz_interfaces/SetEntityPose`).
- **Controller** subscribes to `/dynamic_obstacles` and feeds the predicted
  scan (§3.1). Latency tolerance: markers at ≥ 10 Hz keep prediction exact
  (piecewise-constant velocity ⇒ constant-velocity prediction is *not* an
  approximation between speed changes).
- The MATLAB and ROS 2 runs see **identical obstacle motion** because both
  read the same JSON with the same t=0 reference — start the mover node and
  the run clock together.

### 4.4 Controller-side dynamic logic (mirror these exactly)

| Mechanism | Parameter | Effect |
|---|---|---|
| Predictive scan | `dyn.pred_lead = 0.5 s` | VFH sees movers at future poses |
| Speed slowdown | `dyn.slow_margin = 0.25 m`, `dyn.slow_min = 0.4` | `v = V·scale`, scale linear in predicted gap below `0.25 + robot_radius` |
| Ignore parked movers | `dyn.avoid_vel_frac = 0.25` | movers slower than 25% of V don't trigger slowdown |
| t+dt commit | `cfg.dt = 0.1` | step commits only if body is clear at t and t+dt vs contracted mover poses |
| Escape hold | `dyn.hold_time = 1.5 s` | escape heading accepted only if the body can hold it while movers advance |

Verified MATLAB result on the dynamic world: goal reached, 0 collision steps,
min formation clearance 0.51 m, min gap to a mover 1.22 m (predicted worst
0.33 m, handled by slowdown + VFH), path efficiency 0.93.

### 4.5 The algorithm comparison (E1/E2/E3) and the headless runner

The paper's experiments are reproducible from the Python stack alone — no
Gazebo needed — with the headless mission runner:

```bash
cd ros2_swarm_transport
# full mission, all four steering laws (2-phase, same kinematics as MATLAB):
python3 -m ros2_swarm_transport.headless_runner --algo pvfh --seed 7 \
    --output logs/ros2_headless.csv
# sparse ToF axis:
python3 -m ros2_swarm_transport.headless_runner --algo pvfh_prob \
    --sense tof --n-beams 8 --sigma0 0.10 --seed 3 --output logs/ros2_tof8.csv
```

It runs the identical two-phase mission (forming + rigid transport), the
identical dispatch (`steering_rigid`/`steering_point` ports) and the shared
motion contract, and writes the shared CSV schema (+ `mover_vx`, `mover_vy`,
`speed_scale`, `mover_gap`, `mover_gap_pred`, `comp_ms`). Same-seed runs are
byte-identical (per-run RNG passed through the noise model).

Reference results (dynamic world, seed 7, lidar σ0 = 0.02):

| algo | reached | path (m) | eff | minclr (m) | collisions |
|---|---|---|---|---|---|
| apf | no (local minimum) | — | — | 0.56 | 0 |
| vfh | yes | 25.6–26.2 | 0.90–0.92 | 0.30–0.34 | 0 |
| pvfh | yes | 25.33 | 0.93 | 0.51 | 0 |
| pvfh_prob | yes | 25.6–25.9 | 0.91–0.92 | 0.39–0.47 | 0 |

`pvfh` is deterministic (seed-independent); `pvfh_prob` varies slightly with
seed because the noise actually influences its decisions — that is the point
of the probabilistic layer. Under high noise (σ0 = 0.40, Octave/MATLAB smoke
batch) the deterministic VFH fails entirely while `pvfh_prob` still reaches:
see `run_paper_experiments.m` (E2) and `matlab/make_paper_figures.py`.

### 4.6 The anti-deadlock layer (progress-window probe + committed escape)

The transport runner carries one stack-level mechanism that the steering
laws do not provide: **anti-deadlock**. The rejected-step escape probe alone
cannot see a stall made of *accepted* micro-steps — a long hull oscillating
between two shelf faces accepts every step yet nets zero progress (the E5
line-formation pivot). Two pieces fix this for the VFH-family laws
(`apf` deliberately keeps its textbook local-minimum failure — it is the E1
baseline):

1. **Progress-window detector**: if the formation centre gains < 0.30 m of
   net progress along the start→goal axis in 60 s, force the escape probe —
   regardless of accepted/rejected steps.
2. **Commit-after-probe**: the probe scans headings over *both* senses,
   ordered by deviation from the goal bearing (the old loop probed from the
   current heading and could spin the hull); once a validated heading is
   found, the formation **commits** to sliding along it for 40 steps
   (2.4 m at mission speed), re-validated each step against the moving
   world. Without the commit, the steering law re-asks its (rejected)
   proposal every step and the hull pivots forever.

Ablation (line formation, `pvfh_prob`, 5 seeds): without the layer 1/5
reaches with endless pivots (archived in
`logs/paper/results_headless_precommit.csv`); with it 5/5 at ~113 s with
zero collisions. The MATLAB reference (`Simulation8_dynamic.m`) keeps its
weaker one-shot probe — cross-stack differences are documented in the
report's E1 honesty box.

### 4.7 The full experiment grids (E1–E6) from the Python stack

The complete Monte Carlo study runs from one command (from the repo root of
the source project, not the bundle):

```bash
python3 matlab/run_headless_grid.py --save-runs
#   -> logs/paper/results_headless.csv  (one row per run, long format)
#   -> logs/paper/runs/*.csv            (per-step logs for the overlay figs)
```

Resumable: rows already in the CSV are skipped, so an interrupted batch can
be relaunched as-is. Grids (5 paired seeds each; same seed = same noise
draws across algorithms):

| grid | factors | runs |
|---|---|---|
| E1 | 4 algorithms, clean lidar (σ0 = 0.02) | 20 |
| E2 | vfh vs pvfh_prob × σ0 ∈ {0.02…0.40} | 50 |
| E3 | pvfh_prob × beams {288, 64, 16, 8} (tof) | 20 |
| E4 | pvfh_prob × formation {square, diamond, triangle, line} @ s = 0.60 | 20 |
| E5 | **crossed grid**: 4 formations × 4 algorithms @ s = 0.60 | 80 |
| E6 | **matched sensor**: {lidar-288, tof-16, tof-8} × σ0 {0.02, 0.10} × {vfh, pvfh_prob} | 60 |

Figures: `python3 matlab/make_paper_figures.py --csv
logs/paper/results_headless.csv` (E2 ablation, E3 sweep, E1 bars, E5
formation×algorithm heatmaps, E6 sensor comparison) and
`python3 matlab/make_log_figures.py` (trajectory overlays, paired
differences, formation trails). The MATLAB/Octave twin of the same grids is
`run_paper_experiments` (modes: none = full, `'smoke'`, `'e13'`, `'e4'`,
`'e5'`, `'e6'`; per-run checkpoint file `logs/paper/results_partial.csv`
makes it kill-and-resume safe).

---

## 5. Parameters to mirror 1:1 (the comparison contract)

```
dt = 0.1                       # control period (10 Hz)
robot_radius = 0.18            # set to YOUR robot (burger 0.105 / waffle 0.22)
robot_height = 0.35            # z-slice for the 2D map
grid_size = 2, grid_spacing = 1.18   # or the AUTO-FIX value MATLAB prints
V = 0.6, forming.V = 0.5
max_turn_rate = 0.3 (transport), 0.5 (forming)
heading_smooth_alpha = 0.35 (both phases)
w_goal = 1.0, w_vfh = 2.0
forming: tolerance 0.3, w_slot 2.0, w_repulse 1.5, repulse_dist 1.2,
         slow_radius 1.5, stuck_escape_steps 5, backoff_dist 0.3
convoy:  trigger 2.0, exit 3.0, max_obstacle 2.0, min_group 2, spacing 1.0
algo:    'pvfh' (default) | 'apf' | 'vfh' | 'pvfh_prob'   # steering law
vfh:     sectors 72, beams 288, sense_range_base 4.0, danger_dist_base 1.2,
         turn_limit pi/4, threshold 0.3, smooth_window 5, avoid_gain 5,
         hysteresis 0.6, falloff 3, enlarge true, auto_sigma true,
         react_gap 1.0, p_unknown 0.05
sense:   kind 'lidar'|'tof', sigma0 0.02, k_rng 0.03, fov_deg 60,
         max_range inf
dyn:     pred_lead 0.5, slow_margin 0.25, slow_min 0.4, hold_time 1.5,
         lookahead 1.0, avoid_vel_frac 0.25
```

The four `algo` variants all consume the SAME noisy scan — that is the
comparison contract: differences are the steering law, nothing else.
`swarm.yaml` exposes `algo` and every `sense_*` key.

`swarm_logger.py` already writes the shared CSV schema; it gains optional
columns (`mover_vx, mover_vy, speed_scale, mover_gap, mover_gap_pred`) when
the dynamic topics are present — `write_log_csv.m` appends the same columns,
and `compare_runs.m` ignores extras.

---

## 6. ROS 2 architecture (what goes where)

```
ros2_swarm_transport/
├── params/
│   └── swarm.yaml              # every parameter in §5, single source
├── worlds/
│   └── swarm_world_dynamic.sdf # same file, copied into the Gazebo world
├── scripts/ (or nodes/)
│   ├── mover_node.py           # JSON contract → Gazebo set_pose + /dynamic_obstacles
│   ├── vfh.py                  # pvfh_scan/pvfh_predict/pvfh_prob + noise + dispatch
│   ├── geometry.py             # world_dist / formation_clearance equivalents
│   ├── assign_slots.py         # Hungarian assignment (scipy)
│   ├── headless_runner.py      # full 2-phase mission, no ROS/Gazebo (all algos)
│   ├── formation_controller.py # Phase 1 + Phase 2 state machine, cmd_vel per robot
│   └── swarm_logger.py         # unchanged from this repo (already ROS 2)
├── launch/
│   └── transport.launch.py     # gz sim + spawn 4 TB3 + mover + controller + logger
└── test/
    ├── test_geometry.py        # unit tests vs hand-computed cases
    └── test_pvfh_prob.py       # noise model, expected kernel, dispatch tests
```

Node responsibilities:

1. **`formation_controller`** — the unicycle model from §1, the two-phase
   state machine, slot servoing (`cmd_vel` per robot: `v` from proximity
   logic, `ω = wrap(θ_des − θ)/dt` clipped to `ω_max`). Runs at 10 Hz
   (`dt = 0.1`) to match MATLAB exactly.
2. **Slot tracking:** each robot holds its body-frame offset; the frame
   `(c, θ)` comes from the virtual leader (publish `/formation/frame`). In
   the simplest replication the *same controller* computes the frame and each
   robot's slot; the robots then P-control to their slots — that residual is
   your real `formation_rms`.
3. **`mover_node`** — §4.3.
4. **`swarm_logger`** — records the CSV for `compare_runs`.

### 6.1 Coordinate conventions (get these wrong and nothing matches)

- World/`map` frame = Gazebo world = MATLAB axes: x east, y north, yaw CCW.
  **No transform.** Spawn the robots where `cfg.robots_start` puts them
  (ring of radius 2.2 around (−12, 0)) and publish the goal (12, 0).
- Odometry → map: identity (static world, no localization drift in Gazebo).
- Yaw from quaternion exactly as `swarm_logger.py` does it.

### 6.2 Carrying a load (the payload, physically)

The MATLAB payload is *virtual*: a rigid geometric block. Options in ROS 2,
in increasing fidelity:

1. **Virtual (matches MATLAB 1:1):** no physical load; the formation
   controller keeps the block rigid; a box mesh visualizes the payload
   attached to the leader frame. Use this for the comparison study.
2. **Passive load on the frame:** attach the payload visual to each robot
   with a `FixedJoint`-like constraint via `gz::sim` joints — adds contact
   forces the MATLAB model does not have; expect formation_rms to grow.
3. **Cooperative-carrying study (future):** physical payload with force
   sensing; the kinematic model becomes the *setpoint generator* for a
   compliant controller — outside this repo's scope.

Recommendation: option 1 for replication, and treat option 2 as a separate
experiment. The MATLAB model's predictions (clearances, path, timing) are
only meaningful for the virtual-payload case.

---

## 7. Verification protocol

1. `run_experiments.m` (or the batch call in this repo's README) →
   `logs/matlab_dynamic.csv`. Note the printed `grid_spacing`.
2. Launch the ROS 2 stack with the same parameters:
   `ros2 launch ros2_swarm_transport transport.launch.py`
3. `ros2 run ros2_swarm_transport swarm_logger.py --ros-args -p output:=logs/ros2_dynamic.csv ...`
4. `compare_runs('logs/matlab_dynamic.csv', 'logs/ros2_dynamic.csv', 'swarm_world_dynamic.sdf')`
5. Read the agreement table. Expected divergences and their causes:
   - `formation_rms`: MATLAB ≈ 0 (rigid by construction); ROS 2 > 0 (real
     servoing) — this measures your controller, not an error.
   - `total_turning`: MATLAB slightly lower (no latency/slip).
   - `min_clearance`: a mismatch means `danger_dist`/inflation differs.
   - Frechet/mean deviation: the honest trajectory-overlap numbers.

## 8. Known model limitations (inherit them consciously)

- No dynamics: acceleration is instantaneous; ROS 2's `cmd_vel` tracking will
  smooth corners slightly.
- Kinematic movers: obstacles never yield and never push the payload.
- 2D slice at z ∈ [0.03, 0.35]; overhangs are invisible.
- The convoy logic only anchors on static, compact obstacles.
- LiDAR is 2D; movers must intersect the robot's height band (they do).

---

## 9. October 2026 audit: what the Python/ROS 2 port must change

The MATLAB files named below are in `reference/matlab/` of this bundle. They
are the specification. `reference/AUDIT_CHANGELOG.md` explains why each
change was made. None of the MATLAB changes had been executed when this
bundle was built; if the MATLAB smoke test later forces a correction, the
MATLAB repo wins over this text.

### 9.1 Transport loop (`headless_runner.py`, `formation_controller.py`)

Mirror `Simulation8_dynamic.m`, phase 2:

1. **Atomic pose commit.** Candidate heading `cand_h = wrap(theta + dh)`,
   candidate position along `cand_h`. Commit both if the body is safe at t
   and t+dt. Else, if `dh != 0` and the body is safe at the *current*
   position with `cand_h`, commit the heading only (`reject = 1`). Else hold
   (`reject = 2`). The Python loop already refuses to rotate on a rejected
   step; it lacks the rotate-only fallback.
2. **Escape = steer, not teleport.** The Python loop sets
   `theta_l = found_h` instantly and then slides along `escape_heading`
   irrespective of the body yaw. MATLAB keeps unicycle kinematics: during a
   commit the desired heading is the escape heading, the turn is rate-limited
   (`max_turn_rate`, no low-pass), the body moves along its *current*
   heading, and every pose goes through the gate. A `reject = 2` step cancels
   the commit.
3. **Anti-deadlock layer for every algorithm** (remove the `apf` exclusion):
   window 150 steps (15 s; was 600), min progress 0.30 m along the current leg's
   start→goal axis, commit 40 steps, a window-triggered probe skips headings
   within 0.5 rad of the previous escape. `adl.enable = false` gives a
   one-step commit. A probe that finds nothing resets the stuck counter.
4. **Escape probe** is `find_escape_heading_dyn.m`: 36 headings ordered by
   deviation from the goal bearing, translate one `escape_step` with the body
   at that heading, and hold-safe for `hold_time` against the movers.
5. **Mission timeout** 3000 steps (300 s) per waypoint in the grids.
6. **Waypoints**: if the contract JSON has a `"waypoints"` array, it is the
   route (`contract_waypoints.m`); otherwise the goal pad.
7. **Mover speed scale**: `dyn.speed_scale` multiplies every contract speed.
8. **Mover count**: `dyn.max_movers = N` uses only the first N movers of the
   contract (0 = none). E11 uses `dynamic_obstacles_dense.json` (six movers).
9. **Localisation error** (`loc.sigma_pos`, `loc.sigma_yaw`): each steering
   step draws `e_pos ~ N(0, sigma_pos^2 I)`, `e_yaw ~ N(0, sigma_yaw^2)`
   (two position draws, then one yaw draw, BEFORE the scan noise) and
   steers at the believed goal
   `goal_b = pos + R(-e_yaw) (goal - (pos + e_pos))`; the same vector,
   normalised, replaces the goal-attraction term. The scan, arrival test and
   gate use the true pose. No draws when both sigmas are 0.
10. **Command-update loss** (`comm_drop = p`): after the steering update is
    computed, one uniform draw; if `< p` and a previous command exists, the
    previous `(desired_heading_raw, v_scale)` is applied instead. No draw
    when `p = 0`.

11. **Wedge recovery**: in the hold branch of the commit, once the stuck
    counter exceeds `stuck_escape_steps`, try in order (each through the
    gate): back up `0.5*V*dt` along the body axis with the candidate
    heading; back up with the current heading; rotate in place by `-dh`.
    The first safe one is committed and logged as `reject = 3`. A
    `reject = 3` step does not cancel an escape commit.
11b. **Load-carrying motion limits** (`kin`, default **off**; experiment E14), applied to the
    rate-limited turn `dh` before the pose candidate is built:
    `w = dh/dt`; `e = |wrap(desired_heading_raw - heading)|`;
    `v_flr = min(v_scale*V, 0.5*V)`;
    `w_cap = min(0.8*sqrt(2*yaw_acc_max*e), (v_robot_max - v_flr)/half_diag)`
    (yaw_acc_max = 2.5 rad/s², v_robot_max = 1.0 m/s); clamp `w` to
    `± w_cap`, then to `w_prev ± yaw_acc_max*dt`;
    `v_lim = max(v_flr, v_robot_max - |w|*half_diag)`, and if `|w| > 0` also
    `min(v_lim, lat_acc_max/|w|)` (0.8 m/s²); `v_scale = min(v_scale,
    v_lim/V)`; `dh = w*dt`. Copy the block from `Simulation8_dynamic.m`. `w_prev` is the yaw rate actually committed in
    the previous step (0 if the heading was held).
12. **Docking**: at the phase switch store the robots' slot residual in the
    body frame; logged robot positions are `slot + R*resid*exp(-k*dt/tau)`,
    `tau = 1.0 s`, for the first `6*tau` seconds. The gate uses the rigid hull.

### 9.2 Sensing and steering (`vfh.py`)

1. `scan_ranges_dyn`: ToF fan centred on the **body heading**
   (`sense.frame = 'body'`), angles wrapped to [-pi, pi); returns the beam
   width `dth` (2π/n lidar, fov/n ToF); movers are scanned at t_now even when
   `pred_lead = 0`.
2. `apf_obstacle_repulsion(..., dth)`: use the scan's beam width.
3. Dispatch (`steering_dispatch.m`):
   `apf`, `vfh`, `fgm`, `dwa` use the instantaneous noisy scan
   (`pred_lead = 0`); `pvfh` is the deterministic histogram on the **noisy**
   scan with the horizon; `pvfh_prob` unchanged apart from `dth`.
   `pvfh_predict` (noise-free) is no longer used by the comparison.
4. New laws `fgm` (`local_fgm`), `dwa` (`local_dwa`) and `mpc`
   (`local_mpc`, a 125-sequence turn-rate lattice): port line for line.
   They use the scan WITH the mover horizon (`baseline_horizon`, default
   true); only `apf` and `vfh` are instantaneous.
4b. **Recursive range filter** (`range_filter.m`): per-beam Kalman filter,
   process std 0.08 m, measurement std `s_b`, gain `K = (P+q²)/(P+q²+s²)`,
   3-sigma innovation reset, re-initialised on a time gap > 0.5 s or a
   backward step. Used by `pvfh_prob` (posterior mean AND posterior sigma
   feed the expected kernel and the reflex) and by the new ablation law
   `pvfh_filt` (deterministic kernel on the filtered ranges), in the
   transport phase with the lidar fan only.
4c. **Kernel check.** `expected_kernel` in `vfh.py` is element-wise and
   was always correct; the MATLAB kernel had a matrix-division bug until
   2026-10-02. Results from the two stacks before that date differ for
   this reason.
4d. **Blend fade** (transport): `w_goal_eff = w_goal * max(0, dot(vec_goal,
   vec_steer)/|vec_steer|)` for every law except `apf`.
5. The noise model is linear for ToF: `s = sigma0 + k_rng * d`.

### 9.3 Logging and metrics

Per-step CSV gains `reject` (0/1/2), `escape` (0/1), `clearance_static`
after `comp_ms`. Run summary (`summarize_run.m`): `collisions`,
`coll_static` (must be 0), `coll_mover`, `n_reject`, `n_escape` (rising
edges of `escape`), `comp_ms_mean` over phase-2 steps with `comp_ms > 0`.

### 9.4 Experiments

The manifest is `build_jobs` in `run_paper_experiments.m` (E1, E2, E3, E5–E14;
E4 is the `pvfh_prob` rows of E5). Results CSV columns:
`stack, exp, algo, sense_kind, sigma0, beams, seed, world, formation,
spacing_used, mover_scale, adl, contract, loc_sigma, comm_drop, n_movers, kin,
reached, collisions, coll_static, coll_mover,
n_reject, n_escape, path_length, path_eff, min_clear, mean_clear, total_time,
transport_time, total_turn, comp_ms_mean, run_name, key`. Write
`stack = ros2_headless` and the same `run_name` scheme so
`make_paper_figures.py --csv ... --runs ...` renders the port's results with
the identical script.

### 9.5 Worlds

`worlds/` now also holds `swarm_world_plaza.sdf`, `swarm_world_maze.sdf`, the
ten generated layouts `swarm_world_rand01..10.sdf` (E13) and all contracts
(`dynamic_obstacles_plaza/maze/randNN/dense.json`; the maze contract has
`"waypoints"`). Movers 4–6 of the dense contract have no model in
`swarm_world_dynamic.sdf`; add them there before a Gazebo run of E11.

### 9.6 Acceptance for the port

Different PRNGs mean no run-for-run equality with MATLAB. Accept the port
when, per cell of the MATLAB `results_all.csv`: (a) `coll_static` is 0 in
every run; (b) the reach count's Wilson interval overlaps MATLAB's; (c) the
bootstrap intervals of `n_reject` and of the minimum gap overlap MATLAB's,
or the disagreement is traced to a named cause. With zero noise
(`sigma0 = 0`) and a lidar the two stacks consume no random numbers that
matter and should agree far more closely: use that as the first check.
