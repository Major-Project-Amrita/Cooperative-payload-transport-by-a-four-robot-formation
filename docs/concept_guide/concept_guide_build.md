

This guide teaches every idea the project uses, from the ground up, and then
shows how to read each figure. Read Parts A–E in order the first time. After
that, use Part G (the graph guide) and Part H (the glossary) as references.
Part I has self-check questions with answers.

| Part | Topic |
|---|---|
| [A](#part-a--the-task-and-the-robots) | The task and the robots: kinematics, formations, the swept corridor, traffic |
| [B](#part-b--sensing-and-steering) | Sensing and steering: lidar, noise, Kalman range filter, VFH family, APF, FGM, DWA, MPC, MPPI |
| [C](#part-c--safety) | Safety: the collision gate, the separating-axis test and its defect, monitors, margins, watchdog |
| [D](#part-d--learning-the-neural-networks) | Learning: CNNs, circular convolution, DeepSets, imitation/DAgger, oracle labels, conformal prediction |
| [E](#part-e--measuring-fairly-statistics-and-protocol) | Measuring fairly: seeds, common random numbers, intervals, paired tests, tuning, held-out evaluation |
| [F](#part-f--physics-and-ros-2) | Physics (MuJoCo) and ROS 2 (nodes, topics, DDS, lockstep, deployment) |
| [G](#part-g--how-to-read-every-graph) | How to read every graph (G1–G27) |
| [H](#part-h--glossary) | Glossary |
| [I](#part-i--self-check) | Self-check questions |

Notation: *r* = robot radius (0.18 m), *dt* = control period (0.1 s), σ0 =
sensor range noise (standard deviation, in metres), *K* = number of robots.

---

## Part A — The task and the robots

### A1. The mission

Several small robots must carry **one rigid load** (a pallet or a plate)
from a start A to a goal B through a warehouse. Shuttles move along fixed
routes and **never give way**. A mission has three phases:

1. **Forming.** The robots start scattered. Each one drives to its *slot*,
   its place under the load.
2. **Docking.** The robots close the last few centimetres so the load can
   be pinned on.
3. **Transport.** The robots move as one rigid body until the load reaches
   B.

**Success** means the load reaches B. **Safety** means no robot or load
touches a static obstacle (a rack or wall) or a mover.

### A2. Unicycle kinematics: how a robot moves

A differential-drive robot has two driven wheels. If the wheels track their
commands well, the robot behaves like a **unicycle**. It has a position
(x, y) and a heading θ, and it can only move along its heading and turn at
a limited rate:

```
θ ← θ + clip(Δθ, ±ω_max)            turn, rate-limited
x ← x + v·cos θ·dt,  y ← y + v·sin θ·dt
```

It cannot slide sideways. This is called a **non-holonomic** constraint.
The control commands are a forward speed *v* and a turn rate *ω*. In ROS
these are the two fields of `cmd_vel`.

### A3. Formations and the rigid body

**Slots.** In transport the robots are slots of one rigid frame with centre
**c** and heading θ. Robot *i* sits at c + R(θ)·offsetᵢ, where R(θ) is the
2-D rotation matrix and the offsets define the shape:
- square (4 robots);
- triangle (3);
- 2×3 grid (6);
- 3×3 grid (9);
- line.

The **spacing *s*** is the distance between neighbouring slots.

**One virtual leader.** A single leader steers the frame. Every robot then
follows its own slot. This is **leader–follower formation control**.

**The payload body** used for collision checking is the **convex hull** of
the slots, grown by *r*. The convex hull is the smallest convex shape that
contains all the slots: imagine a rubber band around them.

**Slot assignment.** At the start, the slots are matched to robots by the
**Hungarian algorithm**, which finds the assignment with the smallest total
travel distance.

### A4. The swept corridor (why a 2 m-wide body needs a wider aisle)

The body rotates as it steers. The width it can need at some moment is the
diameter of its **circumscribed circle**, not its width when aligned with
the aisle. Some examples:
- **Square at s = 1.18 m:** the body is 1.54 m wide, but it needs a
  2.03 m corridor.
- **3×3 grid at s = 1.0 m:** circumradius √2 + 0.18 ≈ 1.59 m, so it needs
  about 3.2 m. That is why it fails in the 2.45 m corridor of the study
  warehouse (E16).

### A5. Movers and the traffic contract

Movers are kinematic shuttles that travel straight segments at constant
speed and pause at the ends. Their routes are stored in a JSON **traffic
contract**. Because the motion is piecewise linear, the future position of
a mover is *known exactly* from the contract. Some algorithms use that
knowledge (PA-VFH-K, and the baselines' predicted scans). A deployed robot
usually does not have it, which is why PA-VFH does without it.

### A6. Traffic phase

The *phase* of the traffic is *when* each mover starts its route. In the
original study every run used the same timetable. That is unfair, because
a planner may simply be lucky with that one timetable. **Randomising the
phase** gives each run (seed) its own start offsets, drawn from an
independent random stream, so the obstacle layout is unchanged but the
encounters differ. Part G13 shows what this did to the results.

### A7. The mission layer: the same for every law

Only the *steering vector* differs between algorithms. Everything else is
shared:
- **Heading blend.** The goal direction and the steering direction are
  combined, `θ_raw = angle(w_goal·goal + w_vfh·steer)`, then low-pass
  filtered (α = 0.35) and rate-limited.
- **Safety gate.** Every candidate pose is checked before it is committed
  (Part C).
- **Escape probe.** If the gate rejects the planned pose, the probe looks
  for any admissible heading nearby.
- **Anti-deadlock layer.** If progress along the route stalls for 15 s,
  the layer commits to a diversified heading for a while.
- **Motion limits.** These are load-carrying limits on speed, yaw
  acceleration and lateral acceleration, so the load is not yanked.

---

## Part B — Sensing and steering

### B1. Lidar and ToF sensors

A 2-D **lidar** sends out beams in a full circle (here 288 beams, 1.25°
apart) and returns, for each beam, the range to the first surface it hits.
A **ToF fan** is a cheaper sensor with a narrow field of view (60°).

**The noise model.** Each range is the true range plus Gaussian noise with
standard deviation σ0 (lidar). For ToF the noise is σ0 + k·range. Beyond
the maximum range there is "no return".

### B2. The Kalman range filter (per beam)

A single noisy reading is unreliable, so each beam is filtered over time.
Each beam has an **estimate** *x* and a **variance** *P*. At every new
reading *z* with noise variance *s²*:

```
predict:   P ← P + q²                    (the world may have moved a little)
gain:      K = P / (P + s²)
update:    x ← x + K·(z − x),   P ← (1 − K)·P
reset:     if |z − x| > 3·√(P + s²), set x ← z (something new entered the beam)
```

How to read it:
- **K near 1:** trust the new reading. This happens with a precise sensor.
- **K near 0:** trust the history. The filter averages a noisy sensor.
- **The reset** stops the filter lagging when a mover suddenly appears.
- **√P** is the filter's own uncertainty. It is reused downstream as σ per
  beam.

This one ingredient carries the histogram family's robustness to noise
(G9).

### B3. The polar histogram: VFH and VFH+

**VFH** (Vector Field Histogram) turns a scan into **sectors**: 72 sectors
of 5° around the robot. It then asks, for each sector, "is it dangerous to
go this way?":

1. **Nearest return per sector:** d = the smallest range in that sector.
2. **Danger** u = exp(−d² / 2σ_h²). Near obstacles give u ≈ 1, far ones
   u ≈ 0. The width σ_h is set so that u crosses the **threshold** (0.3)
   at the body radius plus a reaction gap.
3. **VFH+ enlargement.** An obstacle at distance d blocks not only its own
   sector but every sector within the angle γ = asin(r_body / d). That
   accounts for the size of the body.
4. **Smoothing** over neighbouring sectors, which wraps around at ±180°.
5. **Valleys.** Sectors below the threshold are free. Among them, the law
   picks the one closest to the goal direction, plus a **hysteresis**
   penalty for changing its mind.
6. **APF reflex.** A short-range push away from very close returns is
   added (B5).

**The ladder in this project** adds one ingredient at a time:
1. **VFH:** the basic histogram.
2. **pVFH:** adds the movers' predicted positions over a horizon. This is
   the contract's future.
3. **pVFH+F:** adds the per-beam range filter (B2).
4. **PA-VFH-K:** adds the *expected kernel* (B4). This was the original
   proposal, "P-VFH".
5. **PA-VFH (no NN):** the same pipeline on the robot's *own instantaneous
   scans*, with no contract.
6. **PA-VFH:** replaces the kernel with a learned predictor (Part D).

### B4. The expected kernel, and why it hardly matters

If the measured range has uncertainty s, the *expected* danger is

```
E[exp(−d²/2σ_h²)]  with d ~ N(d_meas, s²)  =  √(σ_h² / (σ_h² + s²)) · exp(−d_meas² / 2(σ_h² + s²))
```

Noise widens the kernel, so the robot reacts a little earlier. G3 shows
that, at realistic noise, the reaction distance moves by only about 3 %.
That is why the "uncertainty-aware" kernel adds nothing measurable. It is
a good example of checking a mechanism's *size* before claiming it.

### B5. APF: artificial potential field

The goal attracts and obstacles repel. The repulsion from each close return
falls off as 1/range^p. APF is simple and reacts fast, but it has
well-known **local minima**: places where attraction and repulsion cancel
and the robot stops.

### B6. FGM: follow the gap

FGM looks for the widest **gap** in the scan and steers towards its middle,
weighted towards the goal. It is very cheap. It can oscillate when gaps
open and close.

### B7. DWA: dynamic window approach

DWA samples (v, ω) pairs the robot can actually reach in the next instant:
the *dynamic window*. It simulates each pair forward for T seconds and
scores it on heading to the goal, clearance and speed. It picks the best
pair. Because it reasons about short trajectories rather than directions,
it handles a moving body well.

### B8. MPC: model predictive control (here, a lattice MPC)

MPC predicts the future with a model and optimises a cost over a horizon.
It applies the first step, then re-plans every step (**receding horizon**).
The lattice MPC here:
- builds a small tree of heading changes over two segments;
- scores each branch on progress, a smooth obstacle cost (soft safety
  distance `d_safe`) and a hard clearance;
- keeps the best branch.

`d_safe` keeps MPC away from walls, which is why it survived physics
without a margin (G21).

### B9. MPPI: model predictive path integral

MPPI is a sampling-based MPC:
1. Sample N = 256 noisy control sequences around the previous plan.
2. Roll each one out and compute its cost.
3. Average them with weights exp(−cost/λ).

λ (the temperature) sets how greedy the averaging is. MPPI handles
non-convex costs well, at higher compute (about 4 ms per step here).

### B10. The information each law uses

| Law | Reads the movers' future (contract)? | Notes |
|---|---|---|
| VFH, pVFH, pVFH+F, PA-VFH-K, FGM, DWA, MPC, MPPI | yes, through the predicted scan | the benchmark's standard sensing front end |
| PA-VFH (no NN), PA-VFH | **no**: own instantaneous scans only | deployable |
| NN (DAgger) | scan features as for MPC | learned |

So PA-VFH competes with *less information* than the baselines. Keep this in
mind when it loses on contacts with movers.

---

## Part C — Safety

### C1. The safety gate (atomic pose commit)

Before the frame moves, the gate checks the **next pose**: the body at the
new centre and heading, at t and t + dt. If any part overlaps an obstacle,
the move is **rejected** and the escape logic runs. Nothing is committed
half-way, which is why the commit is called atomic.

In `truth` mode the gate uses the simulator's exact geometry. That is a
**screening device**: it measures what a law asks for while an oracle
prevents static collisions. It is not something a real robot has.

### C2. The separating-axis test (SAT), and the defect that was found

**The theorem.** Two convex shapes do not overlap if and only if there is
a line (axis) onto which their projections do not overlap. For boxes it is
enough to test the axes perpendicular to each box's edges.

**The defect.** The MATLAB reference listed a box's corners in the order
(+,+), (+,−), (−,+), (−,−). That is **not cyclic**. Joining consecutive
corners gives two sides and two diagonals, so one of the box's two edge
normals was never tested. The test then sometimes found "no separating
axis" for a pose that was clear: a **false rejection**.

**Consequences.** The defect is **conservative**: it never accepted a real
collision, it only refused some safe poses. It inflated rejection counts
and lengthened some missions. Fixing the order to cyclic, (+,+), (+,−),
(−,−), (−,+), made MATLAB and Python agree in all 221 grid cells (G7).

### C3. Ground-truth screening vs deployable monitors

| Gate source | Who decides each pose | What it needs | Used for |
|---|---|---|---|
| `truth` | exact geometry | the simulator | reference behaviour |
| `audit` | truth decides; the sensor monitor is scored in its shadow | the simulator | measuring the monitor without changing the mission |
| `sensor` | the geometric monitor | only the robots' scans | deployable |
| `learned` | the learned monitor with a conformal margin | only the scans | deployable |

**The geometric monitor** converts the filtered scan into points. It
admits a pose only if every point stays clear of the body by

```
r + k·σ  (sensor uncertainty, k = 3)  +  v_obs·h  (a point may move up to v_obs = 1 m/s for h = 0.2 s)
```

**Two error types:**
- **False accept:** the monitor admitted a pose that the truth rejects.
  This is dangerous.
- **False reject:** the monitor refused a safe pose. This costs time.

### C4. Physical safety margin (`gate_margin`)

A kinematic gate with r = 0.18 m lets a planner pass 1 mm from a rack. A
real or physically simulated robot lags its command by centimetres, so it
touches the rack. `gate_margin = 0.10 m` makes the gate demand r + 0.10 m.
It is applied **to every law** in batches P3 and P4. G21 shows it working.

### C5. The ROS 2 watchdog

A software watchdog in the controller node does three things:
- If any robot's odometry or scan is older than 0.5 s, it **stops every
  robot** and publishes `/swarm/safety = 1`.
- It clamps every command to `max_v` and `max_w`.
- It sends zero velocity on shutdown.

Measured over DDS, the team stopped 0.6 s after a lidar failure (G26). It
complements a hardware e-stop; it is not a replacement for one.

---

## Part D — Learning: the neural networks

### D1. What a neural network is, in this project's terms

A neural network is a function with many tunable numbers (weights). It is
fitted to examples by gradient descent on a **loss**. The networks here
are small, about 11 k weights for the occupancy model. They are trained
with PyTorch and **run with numpy only**, so a robot needs no GPU and no
deep-learning library.

### D2. Convolution, and why it is *circular*

A **1-D convolution** slides a small filter (5 sectors wide here) along a
sequence. It detects the same local pattern wherever it appears.

A scan is a **ring**: sector 71 neighbours sector 0. A *circular*
convolution wraps around, so the network treats every bearing the same
way. Rotating the scan by k sectors rotates the output by k sectors. This
property is called **rotation equivariance**. It saves data and generalises
better.

### D3. The PA-VFH occupancy network: input, output, training

**Input:** a 6 × 72 array (6 channels per sector):
- three channels for the last three filtered scans, each holding the
  sector's minimum range minus the body radius;
- the range uncertainty σ per sector;
- an observed/unobserved mask;
- the formation's own closing speed towards each sector.

**Network:** 3 circular convolution layers (32 channels, width 5, ReLU),
then a 1×1 output layer and a sigmoid.

**Output:** a danger value u ∈ (0, 1) per sector, *for the next 0.5 s*.

**Labels (oracle):** the simulator knows where every mover will be, so it
can compute the *true* danger over the next 0.5 s exactly. That is the
training target. Training uses privileged information that the robot never
has at run time, sometimes called "learning by cheating". The network
learns to infer from three scans what the oracle knows.

**Loss:** binary cross-entropy with soft labels (the targets are numbers
between 0 and 1, not just 0 or 1). Sectors that really are dangerous get
more weight.

**On-policy rounds:**
1. Round 1 collects data while the non-learned PA-VFH drives.
2. Round 2 collects data while PA-VFH itself drives, and retrains on the
   union.

This fixes **distribution shift**: the learned planner visits states the
first driver never did.

**Where it sits:** it replaces only the danger kernel. Valley selection and
the gate remain classical, which keeps the system auditable.

### D4. DeepSets, and the team-size-agnostic DAgger policy

The **DAgger policy** steers the whole formation directly.
- It encodes the scan with a circular CNN.
- It encodes the slots with **DeepSets**: apply the same small network to
  each slot, then *sum*. A sum does not care about order or count, so one
  policy works for 3, 4, 6 or 9 robots.

### D5. Imitation learning and DAgger

**Behaviour cloning** copies an expert (here MPC) from recorded data.
Its weakness is that once the learner makes a small mistake, it reaches
states the expert never showed it.

**DAgger** (Dataset Aggregation) fixes this:
1. Let the *learner* drive.
2. Ask the *expert* what it would have done in the states the learner
   visited.
3. Add those labels to the dataset and retrain.
4. Repeat.

### D6. Conformal prediction: a calibrated error margin

The learned monitor regresses the clearance. A regression can be wrong,
and split conformal prediction turns it into a decision with a stated
error rate:
1. On a **calibration set** not used for training, compute the residuals
   *e = true − predicted*.
2. Take *q_α* = the ⌈(n+1)(1−α)⌉-th smallest residual. For α = 0.05,
   that is roughly the 95th percentile.
3. **Admit** a pose only if *predicted − q_α > r*.

**Guarantee:** if new checks are drawn like the calibration ones
(exchangeable), the true clearance falls below the corrected estimate at
most α of the time. The guarantee is **marginal** (on average over checks)
and **in-distribution only**. On fused per-robot scans, which it was never
calibrated on, it admitted more unsafe poses
(ROS2_VALIDATION.md §2).

### D7. Two ways to judge a learned predictor

- **Value error** (mean absolute error against the oracle): how close the
  numbers are.
- **Decision agreement:** do the network and the oracle agree on
  *blocked or free* at the 0.3 threshold?

Steering depends on the second. In G23 the network has a *larger* value
error (0.022 vs 0.012) but *better* decisions (98.2 % vs 98.0 %). It puts
small, hedged values on free sectors that do not change any decision.

---

## Part E — Measuring fairly: statistics and protocol

### E1. Monte Carlo runs, seeds and cells

One mission is one random outcome, so each configuration (a **cell**:
world × law × settings) is run with several **seeds**, usually 6–10.
Results are reported per cell or pooled.

### E2. Common random numbers (CRN)

All laws in a cell use the *same* seeds, so they face the same noise
samples and the same traffic. Differences between laws are then not
caused by luck. This is what makes **paired** comparisons possible.

### E3. Rates and the Wilson interval

A table entry like `69/80 [0.77, 0.92]` means 69 successes out of 80 runs,
with a **95 % Wilson interval** of 0.77–0.92 for the true rate. The Wilson
interval is preferred over "p ± 1.96·√(p(1−p)/n)" because it behaves well
near 0 and 1 and for small *n*. With 10 runs and 10/10 successes, the
interval is [0.72, 1.00]: a perfect score on 10 runs is weak evidence.

### E4. Means and the bootstrap interval

For averages (time, contact steps), the **bootstrap** resamples the runs
with replacement 2,000 times and recomputes the mean each time. The middle
95 % of those means is the interval. It needs no normality assumption.

### E5. Paired differences and "not significant"

In E19 each mission (world, noise, seed) is run *with* and *without* the
network. The difference per mission is computed, and then its bootstrap
interval, for example −19.3 [−57.9, +0.7] rejected steps.
- **Negative** favours the network.
- **The interval touches 0**, so the data are consistent with no effect.
  The honest wording is "consistent but not statistically significant".

### E6. Equal-budget tuning and held-out evaluation

**Equal-budget tuning:** every law gets the same search, 24 configurations
including its defaults, on the same 7 training missions. This stops
"tuned proposal vs untuned baseline" comparisons.

**Held-out evaluation:** the comparison then runs on worlds and seeds the
tuner never saw (E18). Training and tuning used seeds ≥ 101; evaluation
used seeds 1–10.

### E7. Cross-stack agreement

The same experiment was run in MATLAB and in Python. A cell **agrees** if
the 95 % intervals of reach, rejections and minimum gap overlap. 221/221
cells agreed. At zero noise the runs are compared step by step, and the
largest centroid difference was 7 × 10⁻⁷ m.

### E8. The metrics

| Metric | Meaning |
|---|---|
| reach | the load got to B within the time limit |
| contact steps / contact runs | steps, or runs, with any overlap with a static or a mover (ground truth) |
| min gap | the smallest clearance minus *r*. Negative means overlap |
| rejections | steps the gate refused |
| time | mission time of the runs that reached B |
| comp ms | the steering law's compute per step |
| clean run (physics) | reached, with no static and no mover contact in MuJoCo |
| false accept / reject | monitor errors, scored against the truth |

---

## Part F — Physics and ROS 2

### F1. Why physics

The kinematic model assumes perfect tracking. **MuJoCo** simulates
masses, wheel friction, motor torque limits, contacts and the load pinned
to each robot by a ball joint. The robots now **lag** and **slip**:
- the *formation error* (robot-to-slot distance) grows in turns;
- planners that drive close to obstacles touch them.

There are three **payload modes**:
- **virtual:** no physical load;
- **on top:** a plate and crate pinned to the robots;
- **behind:** a towed sled. The towed load swings *outside* the formation
  footprint the planner models, so every law struggles with it.

Signals measured in physics:
- **pin gap:** how far the joint between robot and load is pulled apart;
- **tilt:** the load's tilt;
- **contact force.**

### F2. The robot-parameter sweep (model mismatch)

P2 and P4 change one physical parameter at a time while the planner keeps
its nominal model:
- chassis mass ×0.5 / ×2;
- wheel speed 0.8 / 1.6 m/s;
- friction 0.6;
- payload ×0.5 / ×2;
- motor force ×0.5.

This measures robustness to a robot that is not what the planner thinks.

### F3. ROS 2 in one page

- **Node:** a process with a role. Here `mujoco_plant` (physics),
  `formation_controller` (mission and steering), `mover_node` and
  `swarm_logger`.
- **Topic:** a named message stream, such as `/robot1/cmd_vel`
  (geometry_msgs/Twist), `/robot1/odom` (nav_msgs/Odometry),
  `/robot1/scan` (sensor_msgs/LaserScan), and `/swarm/phase` and
  `/swarm/safety` (std_msgs/Int32).
- **DDS:** the middleware that carries messages between processes and
  machines. **QoS** settings choose reliable or best-effort delivery and
  queue depth.
- **Launch file:** starts several nodes with parameters, for example
  `ros2 launch ros2_swarm_transport robots.launch.py`.
- **colcon:** the build tool that installs the package (executables,
  launch files, worlds, model weights).

### F4. Lockstep vs free-running

- **Lockstep (validation):**
  1. The plant publishes the state.
  2. The controller replies with commands.
  3. Only then does physics advance one tick.

  The run is deterministic, so it can be compared to the in-process run
  *exactly* (G25).
- **Free-running (deployment):** the controller steps on a 10 Hz timer
  using the latest `/odom` and `/scan`. That is what real robots do (G26).

### F4b. Simulation time (`use_sim_time`)

A free-running controller steps on a timer. Two kinds of clock exist:
- **The wall clock.** If the simulator falls behind real time (for example
  while rendering video), the controller runs ahead of the physics. Its
  watchdog then sees "stale" data, and the two drift apart.
- **Simulation time.** With `use_sim_time:=true`, the timer and the
  watchdog follow `/clock`, which the MuJoCo plant publishes, so both sides
  stay in step whatever the simulator's speed.

This project measured the difference. On the wall clock, a rendered run
timed out with contacts; on sim time, the same run was clean
(ROS2_VALIDATION.md §1c).

### F4c. Physics through ROS 2: R1 and R2

- **R1:** the P3 physics batch with the plant and controller as ROS 2
  processes in lockstep. All 90 runs are identical to the in-process runs
  in the same environment.
- **Host vs container:** different MuJoCo versions differ in about the
  14th digit. In a chaotic mission, one flipped gate decision then changes
  the whole outcome. That is why identity is checked within one
  environment, and why environments are compared statistically.
- **R2:** the deployment stack, with the plant free-running and
  publishing `/odom` and `/scan` from the physical robots.

### F5. Real-sensor mode: from LaserScans to a formation scan

In deployment the controller must not read the simulator's map of
obstacles. Instead:
1. Each robot's `LaserScan` is converted to points in the map frame, using
   that robot's odometry pose.
2. **Self-filtering:** points on the team-mates, and under the carried
   load, are removed. The filter radius grows with noise, r + 0.10 + 3σ.
3. A **virtual scan** from the formation centre is built from the fused
   points. Each beam takes the *median of its three nearest points*, not
   the minimum. The minimum of *n* noisy points is biased short by about
   σ·√(2 ln n).
4. PA-VFH and the monitor then work exactly as in simulation.

Only PA-VFH (with `gate_source` sensor or learned) is allowed in this
mode. The node refuses configurations that need the oracle.

---

## Part G — How to read every graph

For each figure: **what is plotted**, **how to read it**, **what it shows
in this project**, and **pitfalls**.

**General rules:**
- **Error bars** are 95 % intervals: Wilson for rates, bootstrap for
  means. If two intervals don't overlap, the difference is very likely
  real. If they overlap, look for a *paired* comparison before concluding
  anything.
- **Count the runs.** "10/10" on 10 runs is weaker evidence than
  "130/150".
- **Check which gate was used.** In `truth` mode, static contacts mean
  the *physical* or *mover* side failed, not the oracle.

### G1. Formation shapes (`docs/figures/concept_formations.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/concept_formations.png){width=95%}


- **Plotted:** the four shapes. Disks are robots, the shaded area is the
  collision body (hull + r), the dashed circle is the circumscribed circle.
- **Read:** the corridor a shape needs is the *circle's diameter* (A4).
  Compare it with the aisle widths in G12.
- **Shows:** why the line formation needs more room than its width
  suggests, and why the large 3×3 payload fails in a 2.45 m corridor.

### G2. The polar histogram (`concept_histogram.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/concept_histogram.png){width=95%}


- **Plotted:**
  - **Left:** one noisy scan (blue beams) from the formation centre, a
    mover (orange) and the body outline.
  - **Right:** the 72-sector histogram. Grey bars are above the threshold
    (blocked), green bars are below it (free). The dashed green line is
    the goal bearing; the purple line is the chosen sector.
- **Read:** if the goal sector is green, the law goes nearly straight.
  Otherwise it picks the nearest green valley.
- **Pitfall:** the enlargement (B3) blocks *more* sectors than the
  obstacle itself covers. Wide grey regions near a narrow aisle are why
  histogram laws hesitate at aisle mouths (G11).

### G3. The expected kernel (`concept_kernel.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/concept_kernel.png){width=95%}


- **Plotted:**
  - **Left:** danger against measured range for four noise levels.
  - **Right:** the range where danger crosses the threshold (the reaction
    distance) against noise. The green band is the study's noise range.
- **Read:** inside the green band the curve barely moves, 2.01 m to about
  2.07 m.
- **Shows:** the whole effect of the "uncertainty-aware" kernel is about
  3 %. That predicts, correctly, that PA-VFH-K ≈ pVFH+F in experiments
  (G9).

### G4. The range filter (`concept_range_filter.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/concept_range_filter.png){width=95%}


- **Plotted:** one beam over time. Black is the true range, orange dots
  are measurements, purple is the filter estimate with ±2σ.
- **Read:**
  - Left (precise sensor): the purple line follows the dots, gain ≈ 1.
  - Right (noisy sensor): the purple line is smooth.
  - At 4.5 s a mover enters, and the reset makes the estimate jump at
    once instead of lagging.
- **Shows:** why pVFH+F is so much more robust than pVFH (G9).

### G5. The gate defect (`concept_gate.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/concept_gate.png){width=95%}


- **Plotted:** the same pose under two tests.
  - **Left:** the reference "edges" in red, two sides plus two diagonals.
    The pose is reported as overlapping although it is 0.29 m clear.
  - **Right:** with cyclic corners, all four sides are tested (green) and
    the pose is accepted.
- **Read:** a missing axis can only cause false *rejections* (C2).

### G6. Zero-noise cross-check (`xcheck_trajectories_fixedgate.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/xcheck_trajectories_fixedgate.png){width=95%}


- **Plotted:**
  - **Top:** the centroid path of each law. The wide pale line is MATLAB,
    the thin line Python. Grey is static, red dashed is the mover routes.
  - **Bottom:** the per-step position difference.
- **Read:** if the thin line sits exactly on the wide one and the
  difference stays near zero, the two implementations are the same
  algorithm.
- **Shows:** 7 × 10⁻⁷ m maximum, for every law. That is the rounding of
  the MATLAB logs.
- **Pitfall:** this holds only at zero noise. With noise the random
  numbers differ, so you compare statistics instead (G7).

### G7. Cross-stack scatter (`cross_stack_scatter_fixedgate.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/cross_stack_scatter_fixedgate.png){width=95%}


- **Plotted:** each dot is one experiment cell. x is the MATLAB mean, y
  the Python mean, the line is y = x. There are three panels: time,
  rejections (log scale) and minimum gap.
- **Read:**
  - Dots on the line mean the two stacks measured the same thing.
  - Blue means the intervals overlap.
  - A red diamond means a disagreement.
- **Shows:** with the corrected gate, 221/221 cells agree. The
  correlations are 0.98 (time), 0.95 (rejections) and 0.99 (gap).
- **Pitfall:** the spread at high rejection counts is expected. Those
  cells are chaotic failures, where a single seed changes the mean a lot.

### G8. E1 both stacks (`e1_both_stacks_fixedgate.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/e1_both_stacks_fixedgate.png){width=95%}


- **Plotted:** each law's metrics with 95 % whiskers. Blue is MATLAB,
  orange Python.
- **Read:** paired bars at the same height mean agreement.
- **Pitfall:** APF and FGM have erratic missions. One timeout among 10
  runs shifts their means, so the whiskers are long.

### G9. The ablation ladder against noise (`e2_noise_both_stacks_fixedgate.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/e2_noise_both_stacks_fixedgate.png){width=95%}


- **Plotted:** one column per law (VFH, pVFH, pVFH+F, PA-VFH-K). The top
  row is time, the bottom row is rejections, both against sensor noise σ0.
- **Read left to right:** each column adds one ingredient. The ingredient
  that matters is the one where the curves flatten.
- **Shows:**
  - VFH → pVFH: no change in robustness.
  - pVFH → pVFH+F: much flatter. At σ0 = 0.1, time 77 → 53 s and
    rejections about 40 → 3.
  - pVFH+F → PA-VFH-K: no difference.
- **Lesson:** the filter carries the robustness. The kernel adds nothing.

### G10. Gate effect (`gate_effect.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/gate_effect.png){width=95%}


- **Plotted:** the same seeds with the reference gate (grey) and the
  exact gate (purple), pooled over several experiments.
- **Read:** the pooled means move little, because cells where laws
  genuinely fail dominate them.
- **Shows:** 71–91 % of runs are bit-identical under the two gates. The
  defect shows only where a falsely refused pose occurred.

### G11. Warehouse paths (`warehouse_paths.png`, `warehouse_matlab_vs_python.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/warehouse_paths.png){width=95%}


![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/warehouse_matlab_vs_python.png){width=95%}


- **Plotted:** centroid paths of each law from the dock to the aisle. Left
  is without motion limits, right with them.
- **Read:**
  - Smooth paths mean decisive steering.
  - Zig-zags at aisle mouths mean histogram hesitation (G2 pitfall).
- **The second figure** overlays MATLAB and Python on the validation
  warehouse, a world neither implementation was developed against. They
  agree again to 7 × 10⁻⁷ m.

### G12. Generated layouts (`results/figures_ext/fig_layouts.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_layouts.png){width=95%}


- **Plotted:** the six generated warehouses (30×20 to 80×50 m). Racks and
  columns are dark grey. The blue line is the global A* route, from the
  start (blue dot) to the goal (green star). The dashed orange lines are
  the mover lanes. Each title gives the size and the number of statics and
  movers.
- **Read:** compare the aisle width with the swept diameter of the
  formation (G1). `wh_m_narrow` has 2.4 m aisles.

### G13. Traffic-phase effect (`fig_phase_effect.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_phase_effect.png){width=95%}


- **Plotted:** for each law, two markers per panel. The blue circle is
  the nominal timetable, the orange square is random phase, each with a
  95 % Wilson bar.
  - **Left:** fraction of runs reaching the goal.
  - **Right:** fraction of runs with any contact.
- **Read:** horizontal distance between circle and square is the effect
  of randomising traffic. Bars that don't overlap mean a real effect.
- **Shows:**
  - For every law, contact runs rise sharply. MPC goes from 19 % to 47 %.
  - Reach drops 5–15 points.
  - The fixed timetable had flattered every planner, MPC most of all,
    because its clean result depended on timing.
- **Pitfall:** the ranking among laws also changes. Results from a fixed
  timetable should not be used to rank planners.

### G14. E15 reach per layout (`fig_e15_reach.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_e15_reach.png){width=95%}


- **Plotted:** a heatmap. Rows are layouts, columns are laws, each cell
  is "reached / runs". Darker means a higher success rate.
- **Read:**
  - Look for **light rows**: layouts where every law fails.
  - Look for **light columns**: laws that fail everywhere.
- **Shows:**
  - The `wh_m_narrow` row is 0/10 for every law. That is an open problem,
    not a weakness of one law.
  - The study warehouse ("dynamic") is the hardest of the passable
    layouts.
  - PA-VFH and NN reach 69/80, the best.

### G15. E16 team size (`fig_e16_dynamic.png`, `fig_e16_wh_m_racks.png`, `fig_e16_wh_l_racks.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_e16_dynamic.png){width=95%}


![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_e16_wh_m_racks.png){width=95%}


![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_e16_wh_l_racks.png){width=95%}


- **Plotted:** a heatmap per world. Rows are teams (shape, K, spacing),
  columns are laws, each cell is "reached / 10".
- **Read:** compare rows in the *same* world.
- **Shows:**
  - The 3×3 grid at s = 1.0 (a 2 × 2 m payload, about 3.2 m swept
    diameter) fails in the study warehouse for every law except MPPI
    (7/10). It passes in the large rack warehouses.
  - K = 9 at s = 0.59 (the same small payload with more robots) is easy.
  - So it is **payload size relative to the corridor**, not robot count,
    that limits.
  - NN (DAgger) at 2×3, K = 6 in the study warehouse (2/10) shows a
    learned policy's weak spot on a team it was trained on but in a hard
    world.

### G16. E17 safety monitors (`fig_e17_monitor.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_ext/fig_e17_monitor.png){width=95%}


- **Plotted:** three gate types (ground truth, sensor monitor,
  learned + conformal) × two noise levels (circle 0.02, square 0.10).
  - **Left:** reach rate.
  - **Right:** contact-run rate.
- **Read:**
  - Compare each monitor with ground truth at the same noise.
  - A monitor that keeps reach but adds contacts is less safe.
  - A monitor that loses reach is too conservative.
- **Shows:**
  - The sensor monitor matches ground truth at σ0 = 0.02 but collapses
    at 0.10 (reach about 0.5). Its 3σ inflation closes the aisles.
  - The learned monitor keeps reach at both noise levels.
  - Both deployable monitors have more contact runs than ground truth.
    The contacts are mostly movers driving into the formation, which no
    pose check can prevent.

### G17. Robot platform and mission frames (`robot_platform.png`, `mujoco_forming_sequence.png`, `mujoco_frames.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/robot_platform.png){width=95%}


![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/mujoco_forming_sequence.png){width=95%}


![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/mujoco_frames.png){width=95%}


- **Plotted:** the MuJoCo robot (radius 0.18 m, 6.9 kg, two wheels plus
  casters), the square formation with and without the load, and frames
  from the mission: scattered start, forming, docking, transport.
- **Read:** use these to picture what the kinematic numbers mean
  physically.

### G18. Physics tracking (`mujoco_tracking.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/mujoco_tracking.png){width=95%}


- **Plotted:**
  - **Left:** the planner's kinematic frame (dashed), the physical robots
    (green) and the load (orange).
  - **Top right:** the worst slot error over time per payload mode.
  - **Bottom right:** the frame's yaw rate.
- **Read:** error peaks line up with yaw-rate peaks. Turns are where
  physics departs from the kinematic model.

### G19. Physics summary (`mujoco_summary.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/mujoco_summary.png){width=95%}


- **Plotted:** formation error and contact steps by law and payload mode.
- **Read:** the towed ("behind") mode has the largest errors for every
  law (F1).

### G20. Motion limits (`mujoco_motion_limits.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/docs/figures/mujoco_motion_limits.png){width=95%}


- **Plotted:** the worst slot error over time with the load-carrying
  motion limits off (red) and on (green). Left MPC, right PA-VFH-K.
- **Read:** with limits on, the error peaks are lower. The limits stop the
  planner asking for turns the robots cannot follow with a load.

### G21. S1: the physical margin (`results/figures_signals/S1_physics_margin_clearance.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_signals/S1_physics_margin_clearance.png){width=95%}


- **Plotted:**
  - **Top:** the planner's clearance to static obstacles during transport
    in MuJoCo, same seed and traffic. Red is PA-VFH without a margin, blue
    PA-VFH with the 0.10 m margin, amber MPC. The dashed red line is
    contact.
  - **Bottom:** the time steps where physics recorded a rack contact.
- **Read:** contacts in the bottom strip coincide with the red curve
  touching zero (around 33–46 s).
- **Shows:**
  - Without a margin, PA-VFH hugs the racks.
  - With the margin it keeps ≥ 0.1 m and the contacts disappear.
  - MPC keeps a wide berth by itself (its `d_safe`).
- **Batch result:** PA-VFH clean 0/20 → 16/20 (carried and virtual load).

### G22. S2: one physics mission, signal by signal (`S2_physics_signals.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_signals/S2_physics_signals.png){width=95%}


- **Plotted:** PA-VFH with the margin, load on top. The panels from top:
  - payload speed;
  - formation error;
  - payload tilt;
  - pin gap;
  - clearance;
  - an event strip with the transport phase, mover contact, static
    contact and gate rejections.
- **Read across panels at the same time:**
  - When speed drops (a slow-down near traffic), the formation error
    rises and gate rejections cluster.
  - The tilt stays below 0.04° and the pin gap below 1 mm: the load is
    level and attached.
- **Shows:** no contacts at all, and a stable load throughout.

### G23. S3: inside the neural network (`S3_pavfh_danger_maps.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_signals/S3_pavfh_danger_maps.png){width=95%}


- **Plotted:** three heatmaps of danger. x is time, y is bearing (−180°
  to 180°), colour is danger (white = free, dark blue = blocked).
  1. The oracle (true danger over the next 0.5 s).
  2. The PA-VFH network, which sees only three noisy scans.
  3. The analytic kernel, from the current scan only.

  The bottom panel shows each method's mean error against the oracle over
  time.
- **Read:**
  - Diagonal stripes are obstacles sweeping past as the formation moves
    or turns.
  - Compare the shapes in the network's map with the oracle's. A good
    predictor shows the same stripes, slightly ahead of where the current
    scan has them.
- **Shows:**
  - The network reproduces the oracle's structure closely.
  - Decision agreement is 98.2 % (network) vs 98.0 % (kernel); missed
    blocked sectors are 8.0 % vs 8.3 %.
  - Its value error is higher, 0.022 vs 0.012, because of hedged small
    values (D7).
  - There is a **start-up transient** at 5–10 s, before three frames of
    history exist, where it over-predicts danger everywhere.
- **Pitfall:** don't judge the network by value error alone. Steering uses
  only blocked vs free.

### G24. S4: the sensor monitor against ground truth (`S4_monitor_signals.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_signals/S4_monitor_signals.png){width=95%}


- **Plotted:**
  - **Left:** every gate check. x is the true clearance minus r, y is the
    monitor's clearance minus its required gap. Blue points were admitted,
    orange rejected. The shaded area left of x = 0 is truly unsafe.
  - **Right:** both signals over the sequence of checks.
- **Read:**
  - A **false accept** is a blue point in the shaded area.
  - Points below the diagonal mean the monitor is *more pessimistic* than
    the truth.
  - Orange points with x > 0 are false rejects, which cost time.
- **Shows:** the monitor stays below the truth (conservative) and made 0
  false accepts in this mission.

### G25. S5: ROS 2 lockstep equality (`S5_ros2_lockstep_equality.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_signals/S5_ros2_lockstep_equality.png){width=95%}


- **Plotted:**
  - **Left:** the centroid path from two ROS 2 processes over DDS (thin
    blue) on top of the in-process path (wide amber).
  - **Right:** their difference over time.
- **Read:** a flat line at 0 means identical.
- **Shows:** a 0 m difference over 1,010 steps. The ROS 2 wiring (messages,
  packing, timing protocol) adds no error.

### G26. S6: real-sensor deployment over DDS (`S6_ros2_deployment_signals.png`)

![](/home/user/Cooperative-payload-transport-by-a-four-robot-formation/results/figures_signals/S6_ros2_deployment_signals.png){width=95%}


- **Plotted:** four panels.
  1. A histogram of the time between `cmd_vel` messages.
  2. robot1's commanded speed.
  3. The true gap to the nearest obstacle, which the controller never
     sees.
  4. The watchdog test: commanded speed (blue) and `/swarm/safety` (red)
     around a lidar failure at 20 s.
- **Read:**
  - Panel 1 should be a sharp peak at 100 ms.
  - Panel 3 dipping below zero means contact. Here the dips are movers
    driving into the formation; there are no static contacts.
  - Panel 4: the flag rises and the speed drops to zero shortly after the
    failure.
- **Shows:**
  - Cadence: median 100 ms, 95th percentile 104 ms.
  - 3/3 missions reached the goal from scans alone.
  - The watchdog stopped the team 0.6 s after the failure.

### G27. Tables and the demo's bar charts

- `k/n [lo, hi]` is a rate with a Wilson interval (E3). `m [lo, hi]` is a
  mean with a bootstrap interval (E4).
- In R9, the "paired" rows are per-mission differences (E5).
- The demo's bar charts show raw counts. Read them together with the run
  counts in the axis label.

---

## Part H — Glossary

| Term | Meaning |
|---|---|
| ADL | anti-deadlock layer (A7) |
| APF | artificial potential field (B5) |
| audit mode | truth decides; the monitor is scored in its shadow (C3) |
| bootstrap | resampling interval for a mean (E4) |
| circular convolution | convolution that wraps around a ring of sectors (D2) |
| clean run | reached, with no contact in physics (E8) |
| conformal prediction | calibrated error margin for a learned predictor (D6) |
| contract | JSON file of mover routes and timing (A5) |
| CRN | common random numbers (E2) |
| DAgger | dataset aggregation for imitation learning (D5) |
| DDS | ROS 2 middleware (F3) |
| DeepSets | permutation-invariant set encoder: per-element net, then sum (D4) |
| DWA | dynamic window approach (B7) |
| enlargement (VFH+) | blocking neighbouring sectors by the body size (B3) |
| false accept / reject | monitor admitted an unsafe / refused a safe pose (C3) |
| FGM | follow-the-gap method (B6) |
| gate | pose check before every move (C1) |
| gate_margin | extra clearance the gate demands (C4) |
| held-out | worlds and seeds not used in training or tuning (E6) |
| hysteresis | penalty for switching direction (B3) |
| lockstep | plant and controller advance tick by tick (F4) |
| MPC / MPPI | model predictive control / its sampling-based variant (B8, B9) |
| oracle labels | training targets computed from privileged simulator knowledge (D3) |
| PA-VFH | the proposed law: VFH+ with a learned occupancy predictor |
| PA-VFH (no NN) | the same pipeline with the analytic kernel (the ablation) |
| PA-VFH-K | former "P-VFH": analytic kernel plus contract prediction |
| phase (traffic) | when each mover starts (A6) |
| pin gap | separation at the robot–load joint (F1) |
| SAT | separating-axis theorem (C2) |
| self-filter | removing team and load returns from fused scans (F5) |
| sector | 5° bin of the polar histogram (B3) |
| σ0 | sensor range noise, standard deviation (B1) |
| slot | a robot's place in the formation (A3) |
| swept corridor | width the rotating body can need (A4) |
| unicycle | kinematic model of a differential-drive robot (A2) |
| VFH / VFH+ | vector field histogram / with enlargement (B3) |
| watchdog | stops all robots on stale sensor data (C5) |
| Wilson interval | interval for a rate (E3) |

---

## Part I — Self-check

Answers follow each question.

1. **Why does the square formation at s = 1.18 m need a 2.03 m corridor
   when it is only 1.54 m wide?**
   Because it rotates. The widest extent over all headings is the
   circumscribed circle's diameter (A4).

2. **The reference gate never caused a collision. Why was fixing it still
   important?**
   It falsely rejected safe poses. That inflated rejections, lengthened
   missions, and broke agreement between the two stacks in one cell. A
   benchmark must measure the planners, not a test artefact (C2, G7).

3. **Which ingredient of the histogram ladder makes it robust to noise, and
   which figure shows it?**
   The per-beam Kalman range filter (pVFH → pVFH+F). Figure E2 (G9).

4. **Why does randomising the traffic phase matter more than adding seeds
   with the same timetable?**
   With a fixed timetable every seed meets the same encounters, so seeds
   only vary sensor noise. Planners can look safe through lucky timing. G13
   shows contact rates roughly doubling once the timing varies.

5. **E19 reports −19.3 [−57.9, +0.7] rejected steps for the network. What
   can you claim?**
   The network tends to reduce rejections (negative is good), but the
   interval touches 0. The evidence is consistent with no effect, so the
   claim is "consistent but not significant" (E5). The pre-registered E20,
   with 360 fresh pairs, confirmed this: the rejection effect vanished
   (+0.12 [−2.71, +3.00]). Only mission time improved significantly
   (−0.97 s [−1.66, −0.33]). That is why a small exploratory result needs a
   confirmatory run before it is claimed.

6. **In S3 the network has a larger mean error than the analytic kernel but
   is called better. Contradiction?**
   No. Steering depends on blocked vs free decisions, where the network
   agrees more often with the oracle. Its larger value error comes from
   small values on free sectors that never cross the threshold (D7).

7. **What is the difference between "oracle labels" and "the contract"?**
   Both use knowledge of the movers' future. PA-VFH-K uses the contract
   *at run time*, which is not deployable. PA-VFH uses future knowledge
   only *to make training labels*. At run time it sees only scans (D3,
   B10).

8. **The learned monitor's conformal guarantee says α = 5 %. Why did it
   admit more unsafe poses in ROS 2 real-sensor mode?**
   The guarantee holds for checks distributed like the calibration set.
   Fused per-robot scans are a different distribution (D6).

9. **Why did MPC survive physics without a margin while PA-VFH did not?**
   MPC's cost keeps a soft safety distance (`d_safe`) from obstacles. The
   histogram laws accept any pose the gate allows, which can be 1 mm from
   a rack (G21).

10. **In S6, panel 3 dips below zero but there are no static contacts. What
    happened?**
    Movers never yield and drove into the formation. The controller cannot
    prevent a collision that is entirely the mover's doing (A1, G26).

11. **Why must the self-filter grow with sensor noise?**
    Returns from team-mates scatter by about 3σ beyond the robot's
    radius. A fixed filter leaves them in the cloud as phantom obstacles
    next to the slots, and forming never finishes (F5).

12. **What sensor would you specify for a deployment, and why?**
    A 2-D lidar with range noise ≤ 5 cm. Above that, the 3σ inflation plus
    the robot radius nearly fills the gap between formation and wall in the
    narrowest corridor, and reach collapses (ROS2_VALIDATION §2b).