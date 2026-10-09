# Running your MATLAB swarm in the ROS 2 / Gazebo world

> **October 2026 audit.** Several statements below predate the audit. Current
> behaviour: six algorithms (`apf`, `vfh`, `fgm`, `dwa`, `pvfh`, `pvfh_prob`)
> through one noisy sensing front end; the transport pose (position **and**
> heading) is committed atomically behind the collision gate; an
> anti-deadlock layer (`cfg.adl`) applies to every algorithm; the ToF fan is
> body-fixed; new log columns `reject`, `escape`, `clearance_static`. Details
> and evidence: `../docs/AUDIT_CHANGELOG.md`. The code changes are untested
> until `run_paper_experiments('smoke')` has been run.

This package makes `Simulation7.m` run in the *same* world your ROS 2 stack uses,
in the same coordinate frame, with the same sensing model, so the two runs can be
compared quantitatively instead of impressionistically.

---

## Read this first: your formation did not fit

Before any code, I measured the tightest squeeze on the start→goal path in both
worlds (exact signed-distance field + connectivity search over the clearance
threshold):

| World | Corridor width at the pinch | Max formation circumradius | `grid_spacing = 4.0` needs |
|---|---|---|---|
| `swarm_world.sdf` | 3.89 m | 1.95 m | **3.01 m** ❌ |
| `swarm_world_medium.sdf` | 2.45 m | 1.23 m | **3.01 m** ❌ |

A 2×2 grid at 4.0 m spacing is a rigid block with a 2.83 m circumradius; add the
0.18 m robot radius and it needs 3.01 m of clearance. The medium world's chicane
gives 1.23 m.

The SDF comment says gaps are "> ~2 m", and that is true — but it was sized for
*individual* robots, not for a 4 m rigid block that rotates with heading. Because
the block rotates, the quantity that must fit is the **circumradius**, not the
half-width.

Had you run the comparison as-is, MATLAB would have ground against its
stuck-escape logic while Gazebo threw contact forces, and you would have been
comparing two different failure modes rather than two implementations.

Working spacings: **≤ 2.50 m** (simple), **≤ 1.48 m** (medium). `run_experiments.m`
uses 1.5 m and auto-reduces to 1.18 m for the medium world. **Whatever spacing you
use, set the same value in ROS 2**, or the comparison is meaningless.

---

## Three bugs found in `Simulation7.m`

**1. Collision checked only the four slot points.**
Every pillar in these worlds is 0.8–0.9 m across, so one can sit between two
corners of a 4 m square and pass clean through the formation undetected — while
Gazebo registers a hard contact. Verified: a formation centred exactly on
`pillar4` reports "no collision" under the original logic.
Fixed in `formation_clearance.m`, which samples the body outline and adds an
exact separating-axis test.

**2. Phase 2 is rigid *transport*, so the block interior is solid** — there is a
load spanning it — but nothing tested the interior. Now handled by
`'payload'` mode (`'frame'` and `'slots'` are also available; `'slots'`
reproduces your original behaviour for A/B purposes).

**3. The stuck-escape never escaped.**
`find_escape_heading` rotated the formation about its current centre and asked
whether *that pose* was in collision. A body merely blocked ahead is not itself
in collision, so the first probe — the goal heading, the one already jammed —
came back clear and the function reported success without escaping. The
formation re-committed to the same blocked heading every step, forever. That was
the stall at x = −4.35 m in my first run. Now it tests whether the body can
actually *translate* along the candidate heading.

Two smaller ones: `linspace(-pi,pi,72)` puts a sector at both −π and +π (the same
bearing), and `movmean` smooths the polar histogram as a line segment, leaving a
seam directly behind the robot.

---

## The sensing-model problem (the important one)

Your original p-VFH consumed a point cloud sampled over obstacle **interiors**
on a 0.2 m grid. A robot in Gazebo consumes a `LaserScan`: nearest visible
surface per bearing, everything behind it occluded. Those are different inputs,
so identical gains still give different trajectories — and you would have no way
to tell whether a divergence came from the algorithm or from the sensing model.

`lidar_scan.m` replaces it with an analytic raycast (slab method for boxes,
quadratic for circles), cross-validated against the signed-distance field to
4×10⁻⁹ m. Same algorithm, same input modality, both sides.

Two further changes were needed to make a *rigid body* navigable at all:

- **Enlarged obstacles (VFH+).** The histogram was computed from the formation
  centre, so a bearing could read "free" while the 1.9 m-wide block clipped the
  obstacle. Each hit at range *d* now blocks bearings within `asin(r_eff/d)`.
- **`sigma` derived from required clearance.** Reaction distance is
  `sigma·sqrt(2·ln(1/threshold))`, so a fixed `sigma = 1.0` hard-codes a 1.55 m
  reaction — fine for one robot in an open 50×50 field, far too late for a
  2.5 m block. Now solved the other way round, from the body size.
  Set `cfg.vfh.auto_sigma = false` to restore the fixed value.

I also dropped `danger_dist_base` from 2.5 to 1.2. In a 10 m aisle a 2.5 m
repulsion radius sees both walls at once; they nearly cancel and the residue
fights the goal term.

---

## Files

| File | Purpose |
|---|---|
| `sdf2world.m` | SDF → 2D obstacle map. No Java, no toolboxes, no `xmlread`. |
| `xml_lite.m` | Dependency-free XML reader backing the above. |
| `world_dist.m` | Exact vectorised signed distance. The one geometric primitive. |
| `lidar_scan.m` | Analytic 2D LaserScan. |
| `scan_ranges_dyn.m` | Shared clean scan: statics + movers over the prediction horizon (ToF FoV arc supported). |
| `sensor_noise.m` | The range-noise model every algorithm variant consumes (lidar/tof). |
| `pvfh_scan.m` | p-VFH on scans, with enlarged obstacles. |
| `pvfh_predict.m` | Predictive variant: movers raycast at contracted future poses. |
| `pvfh_prob.m` | **P-VFH (the proposal)**: probabilistic occupancy + APF reflex. |
| `apf_obstacle_repulsion.m` | APF obstacle reflex on a scan (shared by all variants). |
| `steering_dispatch.m` | Algorithm dispatch ('apf'/'vfh'/'pvfh'/'pvfh_prob') for both phases. |
| `formation_clearance.m` | Whole-body collision, three fidelity modes. |
| `world_feasibility.m` | Corridor bottleneck + max usable `grid_spacing`. |
| `obstacle_extents.m` | Obstacle sizes; gates the convoy behaviour. |
| `find_escape_heading_sdf.m` | Escape probing that tests translation. |
| `assign_slots.m` | Hungarian assignment, with `matchpairs` fallback. |
| `draw_world.m`, `draw_grid.m`, `draw_robot.m` | Rendering at true footprint size. |
| `Simulation7_sdf.m` | The adapted main script. |
| `Simulation8_dynamic.m` | The dynamic-obstacle main script (algorithm axis, comp timing). |
| `run_experiments.m` | Headless batch driver over the static worlds. |
| `run_paper_experiments.m` | Paper Monte Carlo: E1 baselines, E2 noise ablation, E3 ToF sweep (MATLAB **and Octave**). |
| `make_paper_figures.py` | Paper figures from `logs/paper/results_all.csv` (matplotlib). |
| `compare_runs.m` | MATLAB vs ROS 2 metrics + overlay plots. |
| `swarm_logger.py` | ROS 2 node writing the identical CSV schema. |

Everything is in **Gazebo's own coordinates** — no rescaling, no re-origining.
(0,0) in MATLAB is (0,0) in Gazebo, so trajectories overlay without a transform.
Your `world_scale` multiplier is gone.

---

## Protocol

**1. MATLAB side**

```matlab
run_experiments        % writes logs/matlab_simple.csv, logs/matlab_medium.csv
```

Note the `grid_spacing` in the batch summary — including any AUTO-FIX value.

**2. ROS 2 side**

Set the same `grid_spacing`, `robot_radius`, and controller gains, then:

```bash
ros2 run <your_pkg> swarm_logger.py --ros-args \
    -p robots:="[robot1,robot2,robot3,robot4]" \
    -p goal:="[12.0, 0.0]" \
    -p output:=logs/ros2_simple.csv \
    -p rate:=10.0          # match 1/cfg.dt
```

**3. Compare**

```matlab
R = compare_runs('logs/matlab_simple.csv', 'logs/ros2_simple.csv', 'swarm_world.sdf');
```

### Shared CSV schema

```
t, phase, leader_x, leader_y, leader_yaw, dist_to_goal, clearance,
r1_x .. rK_x, r1_y .. rK_y
```

`leader_*` is the formation **centroid**, not a designated lead robot — logging a
lead robot instead offsets the trajectories by half the formation width and
corrupts every deviation metric. `leader_yaw` is a **circular** mean; plain
averaging is 180° wrong whenever the formation crosses ±π.

`clearance` is written as `NaN` by the ROS 2 node on purpose: `compare_runs`
recomputes it from the SDF for *both* runs, so they are scored by identical
geometry rather than by whatever a costmap reported.

---

## MATLAB figure rendering on Linux (broken/dashed lines fix)

If lines in MATLAB figures render as scattered bits or dashes wherever the
value CHANGES (flat lines fine, wiggly lines broken), that is the known
hardware-OpenGL line-dropping bug on Linux (Intel/Mesa), NOT a data problem.
Verify against the CSV: the logged series are continuous at exact 0.1 s steps.

- **Interactive runs:** `opengl save software` once per session, or
  per-figure `set(gcf,'Renderer','painters')`.
- **Report figures:** `make_figures.m` already works around it — it exports
  vector PDF via `exportgraphics(..., 'ContentType','vector')` (painters
  pipeline, no OpenGL; the plain `-dpng`/`-painters` print path silently
  rasterises through OpenGL anyway and stays broken), converts with
  `pdftoppm` (poppler-utils) and trims page margins with `crop_figures.py`.
  Requires `pdftoppm` and `python3 + PIL` on PATH.

## Interpreting the comparison

`compare_runs` reports per-run metrics plus agreement metrics.

- **`formation_rms`** is usually the most interesting number. MATLAB's block is
  rigid by construction, so it reads ~0; the ROS 2 value is a real measurement
  of your formation controller. A large gap here means the MATLAB model is
  optimistic about formation-keeping, not that ROS 2 is broken.
- **`frechet`** is the honest headline for "are these the same trajectory" —
  worst-case, order-preserving. **`mean_deviation`** is the better number for
  reporting trends; Fréchet is dominated by the single worst moment.
- **`min_clearance`** diverging means one stack hugs obstacles harder: usually
  a `danger_dist`/inflation-radius mismatch.
- **`total_turning`** is control effort, and a proxy for how much the load was
  shaken during transport.

Expect *some* divergence. MATLAB integrates a kinematic point at the formation
centre; Gazebo simulates four differential-drive bases with mass, wheel slip and
controller latency. The value of the comparison is in **which** metrics diverge
and by how much, not in making them identical.

---

## Two things to set before you run

1. **`cfg.robot_radius = 0.18`** is a placeholder. TurtleBot3 burger is 0.105,
   waffle 0.22. This feeds inflation, the feasibility check and the VFH
   enlargement, so it matters more than it looks.
2. **`cfg.vfh.n_beams = 288`** should match your real LiDAR's beam count.

## Limitations

- `<include>`'d models (`model://` URIs) are not resolved — inline their
  geometry if they matter. You get a warning.
- `<mesh>` collisions are unsupported; approximate them with box/cylinder. You
  get a warning.
- Non-static models are placed at their spawn pose and never move.
- Roll/pitch are ignored (exact for these worlds — all zero).
- The world is sliced at one height band (`cfg.robot_height`), the standard 2D
  navigation assumption.

---

## APF + VFH architecture — the four-variant algorithm comparison

The paper's central experiment compares FOUR steering laws that consume the
**same noisy scan** (`scan_ranges_dyn` + `sensor_noise`), so any performance
difference is attributable to the steering law alone, not the sensing:

| `cfg.algo` | Steering law | Mover horizon | Reference |
|---|---|---|---|
| `'apf'` | APF reflex only (Phase 1 cascaded with attraction; Phase 2 reflex only — the caller blends `vec_goal`) | none | classical APF baseline |
| `'vfh'` | deterministic VFH histogram on the noisy scan | none (classic stack) | VFH+ baseline |
| `'pvfh'` | deterministic predictive VFH (the previously verified stack) | `pred_lead` | APF+VFH hybrid, reference behaviour |
| `'pvfh_prob'` | **P-VFH: probabilistic occupancy + APF reflex (the proposal)** | `pred_lead` | this work |

All four are dispatched by `steering_dispatch.m` (a standalone file rather
than script-locals, so **Octave runs the same code path** — Octave defines
script-local functions only at their point of appearance).

**Phase 1 (forming up)** — cascaded for every variant: `apf_force` computes a
virtual goal from slot-attraction + inter-robot repulsion; the dispatcher
steers toward it while dodging obstacles. For the pure-APF baseline the
attraction is added inside the dispatcher (cascaded architecture).

**Phase 2 (rigid transport)** — weighted sum: the caller blends
`w_goal·vec_goal + w_vfh·vec_steer` for all variants except pure APF, whose
dispatcher output is reflex-only (attraction would double-count).

Obstacle avoidance is deliberately kept in the VFH family rather than APF: a
polar histogram degrades more gracefully in narrow corridors, where two nearby
obstacle faces can otherwise cancel and leave an APF repulsive term with zero
usable gradient (the classical "local minimum" trap — E1 shows exactly this:
APF never reaches the goal on the dynamic world).

### The P-VFH concept (`pvfh_prob.m`) — why a probabilistic histogram

The deterministic histogram thresholds a danger kernel `env(d) =
exp(−d²/2σ²)` of the measured range. With a noisy sensor that is brittle in
two opposite ways: a return measurably inside the reaction envelope can still
leave the histogram reading "free" (missed obstacle, collisions), while any
single low outlier seals a corridor the deterministic stack would use.
P-VFH keeps the per-bearing uncertainty and computes TWO layers per beam:

1. **Risk layer** — blocked probability under the sensor likelihood
   `d ~ N(d_meas, s_b²)`, one-sided (nothing behind a return):
   `p_b = Φ((d_block − d_meas)/s_b)`, `d_block = r_geom + react_gap` (the same
   envelope the deterministic `auto_sigma` contract encodes). Per sector the
   p_b are pooled by noisy-OR (the correct combiner for independent chances of
   the same fatal event); unobserved/drop-cut sectors floor at `p_unknown`.
2. **Selection layer** — the posterior-expected deterministic kernel
   `u_b = E[env(d_true)] = √(σ_h²/(σ_h²+s_b²))·exp(−d_meas²/(2(σ_h²+s_b²)))`.
   Thresholding p directly does NOT work: p saturates at 1 for anything inside
   d_block, every VFH+ enlarged arc becomes a full plateau, circular smoothing
   seals the gaps between obstacle clusters, and the stack stalls (this exact
   failure was reproduced and fixed during development). u_b instead is graded
   like the deterministic kernel, never saturates, and inflates monotonically
   with sensor noise — the graceful degradation E2 measures.

Properties that make u_b the right bridge: **(i) exact deterministic limit**
(s_b → 0 gives env(d_meas), so P-VFH inherits the verified low-noise
behaviour instead of re-tuning it); **(ii) bounded and graded**, so valleys
survive smoothing; **(iii) monotone caution** (∂u/∂s_b > 0: an uncertain
return may hide a closer obstacle). The control topology — VFH+ propagation,
threshold, valley search, hysteresis, APF reflex fusion — is IDENTICAL to the
deterministic stack; only the occupancy values change.

### Sensor noise model (`sensor_noise.m`)

Both modes apply a max-range drop-cut (a weak far return is indistinguishable
from no return; consumers floor those at `p_unknown`):
- `'lidar'`: dense 360°, homoscedastic `σ(d) = σ0`;
- `'tof'`: FoV arc (`fov_deg`, centred on +x), distance-dependent
  `σ(d) = σ0 + k_rng·d` — the standard direct-ToF error model
  (VL53L1X/VL53L5CX-class).

### Running the comparison

```matlab
% single runs: pick the axis
CFG_OVERRIDE = struct('algo','pvfh_prob','seed',7,'viz',struct('enable',false));
Simulation8_dynamic;

% the paper's Monte Carlo (MATLAB or Octave):
run_paper_experiments;          % E1 x E2 x E3, overnight
run_paper_experiments('smoke'); % 7 runs, minutes — pipeline check

% the three paper figures (machine-independent, matplotlib):
%   python3 matlab/make_paper_figures.py
```

Smoke-mode results on the dynamic world (Octave 10.3, seed 7): all variants
reach with 0 collisions at σ0 = 0.02; at σ0 = 0.40 the deterministic VFH
**fails** (0% goal rate, 250 collision steps) while P-VFH still reaches
(min clearance 0.25 m) — the graceful-degradation claim in one table. Dense
vs 8-beam ToF (E3): P-VFH reaches in both, clearance 0.35 → 0.30 m.
