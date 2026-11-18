# PA-VFH: the proposed hybrid steering law

> **Nomenclature (November 2026).** The proposed algorithm is now called
> **PA-VFH**. It contains a learned component (below). The algorithm that
> earlier documents call **P-VFH** (code key `pvfh_prob`) is renamed
> **PA-VFH-K**: the same histogram pipeline with the *analytic* expected
> kernel and a contract-predicted scan. Its code key, run names and result
> files are unchanged, so the MATLAB/Python replication data stays
> traceable. The ablation ladder keeps its names (VFH → pVFH → pVFH+F →
> PA-VFH-K). The frozen snapshots in `bundle/` and `backup_preaudit/` still
> say "P-VFH".
>
> *TODO (authors): state the expansion of the acronym "PA-VFH" in the
> manuscript.*

## 1. Why change the algorithm

The Q2 review ([Q2_ASSESSMENT.md](Q2_ASSESSMENT.md)) found two things about
the former proposal:

1. **The named novelty was inactive.** The posterior-expected kernel changed
   the reaction distance by about 3 %. All of the noise robustness came from
   the per-beam Kalman range filter.
2. **It saw the future.** Its scan was ray-cast against the movers at their
   **future** positions taken from the traffic contract
   (`with_horizon=True`). A deployed robot has no such information.

PA-VFH keeps the component that works (the range filter). It replaces the
inactive one (the analytic kernel) with a learned **predictive occupancy**
model, and it removes the oracle: everything it uses is the robot's own
instantaneous scans.

## 2. The algorithm

At each 10 Hz transport step (`vfh.pavfh`):

1. **Sense.** One instantaneous noisy scan (288-beam lidar or ToF fan),
   with movers where they are now and no prediction.
2. **Filter.** The per-beam Kalman range filter gives a posterior mean and
   a standard deviation for every beam (unchanged from PA-VFH-K).
3. **Predict occupancy (neural).** The 72 histogram sectors are encoded
   over the last three filtered scans:
   - the sector minimum range minus the body's swept radius, one channel
     per scan;
   - the posterior sigma;
   - an observed/unobserved mask;
   - the formation's own closing speed toward each sector.

   This gives a 6 × 72 input. A fully convolutional circular CNN (3 layers
   of 32 channels, kernel 5, then a 1×1 output layer) maps it to a danger
   value u ∈ (0, 1) per sector. Circular convolutions make the model
   equivariant to rotations of the scan by whole sectors, and it has 11 k
   parameters.
4. **Histogram and decision.** VFH+ obstacle enlargement by the body
   radius, circular smoothing, threshold, valley selection with goal and
   hysteresis costs, and the APF reflex: all identical to the histogram
   family. The mission layer (atomic pose commit behind the gate,
   anti-deadlock, slow-down, motion limits) is the same for every law.

**Training target.** The danger an *oracle* would compute: the analytic
kernel exp(−d²/2σ_h²) applied to the **true** sector range minimum over the
0.5 s prediction horizon. The simulator knows the movers' future, so the
labels are exact. At run time the network has to infer it from scan history.

**Training procedure** (`tools/train_neural.py occupancy`):
- **Round 1.** Data collected while *PA-VFH (no NN)* drives.
- **Round 2.** Data collected on-policy while PA-VFH itself drives
  (DAgger-style), then retrained on the aggregate.
- **Training worlds:** rand01–05, the study warehouse and the plaza.
- **Teams:** 3, 4 and 6 robots.
- **Noise:** σ0 ∈ {0.02, 0.10, 0.20}.
- **Seeds:** ≥ 701, with a randomised traffic phase.
- **Loss:** soft-label binary cross-entropy, up-weighted for sectors with
  u > 0.1.

**Deployment.** Numpy inference (equal to the PyTorch model to 5e-8),
0.1-0.25 ms per step on top of the analytic pipeline (measured); weights are in
`src/ros2_swarm_transport/models/pavfh_occupancy.npz` and are installed
with the ROS 2 package.

**Forming phase.** Each robot steers to its slot with the analytic kernel
on its instantaneous scan (no filter or history state, and no contract).

## 3. How it is evaluated (no claim without these)

| Experiment | Question |
|---|---|
| **E19 ablation** | Does the network help? PA-VFH vs **PA-VFH (no NN)** (the identical pipeline with the analytic kernel) vs PA-VFH-K (former P-VFH, with contract prediction) vs MPC and MPPI. σ0 ∈ {0.02, 0.10, 0.20}; dynamic, awswh, wh_m_racks, rand08 and maze; 6 seeds; randomised phase. |
| E15 layouts | Generalisation to six unseen generated warehouses. |
| E16 team size | Unseen team sizes and payloads (9 robots; 2.36 × 1.18 m and 2 × 2 m payloads). |
| E17 monitors | Behaviour under the deployable sensor monitor and the learned conformal monitor, not only the ground-truth gate. |
| E18 tuning | Tuned with the same budget as every baseline; held-out worlds. |
| P1/P2 physics | MuJoCo, 3 payload modes × 10 phase-randomised seeds, and the robot-parameter sweep. |

The comparison is deliberately strict:
- PA-VFH uses **less** information than PA-VFH-K and the baselines, whose
  scans include the contract's prediction of mover motion.
- The fair, like-for-like test of the network is PA-VFH against PA-VFH
  (no NN).
- PA-VFH against MPC answers whether the hybrid is competitive with the
  strongest classical planner.

Results: [EXTENDED_RESULTS.md](EXTENDED_RESULTS.md), section R9 and the
PA-VFH rows of R3–R8.

## 4. Result of the E19 ablation (measured)

450 runs: 5 laws × 5 worlds × 3 noise levels × 6 seeds, random traffic
phase. None raised an exception. Numbers come from
[EXTENDED_RESULTS.md](EXTENDED_RESULTS.md) R9, with 95 % intervals.

| | PA-VFH | PA-VFH (no NN) | PA-VFH-K | MPC | MPPI |
|---|---|---|---|---|---|
| reached | **90/90** | 89/90 | 86/90 | 87/90 | 86/90 |
| runs with any contact | 32/90 | 31/90 | **14/90** | 26/90 | 30/90 |
| mean time, s | 90.3 | 91.4 | 88.8 | **74.7** | 79.1 |
| rejected steps | **19.6** | 38.9 | 83.0 | 71.3 | 100.9 |
| compute, ms/step | 0.77 | **0.64** | 1.12 | 2.46 | 4.06 |

**Paired, network vs no network** (the same 90 missions):
- **Reach:** 90 vs 89.
- **Time:** −0.9 s [−2.0, +0.3].
- **Rejected steps:** −19.3 [−57.9, +0.7].
- **Contact steps:** −0.3 [−1.8, +1.1].

**What this supports (E19, exploratory):**
- The learned occupancy model points in the right direction on every
  measure. It finished the one mission the analytic kernel did not, and it
  halved the number of steps the gate rejected.
- **No effect is statistically significant at this sample size.** Every
  paired interval contains or touches 0. The rejected-step gain comes mostly
  from a few long stalls of the analytic version at σ0 = 0.02.

**Confirmed by the pre-registered E20** (360 new pairs, seeds 7–30; plan in
[PREREGISTRATION_E20.md](PREREGISTRATION_E20.md), report section R11):

| Outcome (NN − no NN) | E20 | Pooled E19 + E20 (450 pairs) |
|---|---|---|
| Mission time (s) | **−0.97 [−1.66, −0.33]: improved** | −0.95 [−1.53, −0.38]: improved |
| Rejected steps | +0.12 [−2.71, +3.00]: inconclusive | −3.77 [−12.95, +1.76]: inconclusive |
| Contact steps | +0.17 [−0.93, +1.19]: inconclusive | +0.07 [−0.84, +0.95]: inconclusive |
| Reach | 351 vs 351 | 441 vs 440 |

So the network makes PA-VFH **slightly but significantly faster** (about
1 s, 1 %) and is otherwise neutral. **E19's halved rejections did not
replicate**; they came from a few long stalls.
- PA-VFH has the highest reach of the five laws. It uses no contract
  prediction and about a third of MPC's compute.
- **PA-VFH is not the best on every measure.** MPC is about 15 s faster.
  PA-VFH-K, which sees the movers' contract future, has fewer than half as
  many contact runs.
- Under the truth gate, contacts come from movers driving into a formation
  that cannot get out of the way in time. Seeing the future (PA-VFH-K)
  helps here; inferring it from three scans helps much less.

**What would be needed for a stronger claim:**
- more seeds on the dynamic worlds, where the laws differ;
- a longer scan history or an explicit mover-velocity channel;
- training on the held-out failure cases (on-policy aggregation, round 3).

### PA-VFH in the other experiments (measured)

All numbers are from [EXTENDED_RESULTS.md](EXTENDED_RESULTS.md); every
experiment uses random traffic phase.

| Experiment | PA-VFH | Strongest comparison |
|---|---|---|
| E15: 8 layouts, 80 runs per law | reach 69/80; 21 contact runs; 0.85 ms | reach: NN 69/80, MPC 68/80. Contact runs: pVFH 13, PA-VFH-K 15. No law passes `wh_m_narrow` (0/10 each). |
| E16: teams of 3–9, 150 runs per law | reach 138/150 (tied best with MPPI); 80 contact runs (the most); 0.82 ms | PA-VFH-K 135/150 with 59 contact runs; MPC 135/150 with 68 |
| E16, K = 9 only (unseen team size) | 51/60 | MPPI 55/60, MPC 50/60 |
| E17, audit gate, 48 runs per law | reach 48/48; 21 contact runs | PA-VFH-K 46/48 with 11 contact runs |
| E17, sensor monitor decides | reach 38/48 (best of five); 27 contact runs | MPC 36/48 with 29 |
| E17, learned monitor decides | reach 47/48 (best of five); 30 contact runs | MPPI 46/48 with 26 |
| E18: tuned with equal budget, held out | training objective 127.8 → 96.1; held-out change +0.5 [−3.3, 4.9]; 50/50 reached both ways | the defaults are already near the held-out optimum for every law except FGM and DWA |
| **P1 MuJoCo physics**, 10 runs per payload mode | **reached 30/30, but static contact in 30/30, so 0/30 clean** | MPC 22/30 clean; NN 26/30 |
| P2 robot sweep, 8 variants × 5 runs | 0/40 clean (static contact in every run) | MPC 39/40 clean |

How to read this:
- **Kinematic simulation.** PA-VFH is consistently among the most reliable
  laws in reach and the cheapest to compute. Its weak point is contacts with
  moving traffic: it has more contact runs than PA-VFH-K, which sees the
  movers' future, and more than MPC.
- **Physics.** PA-VFH inherits the histogram family's failure: it grazes
  static racks in every MuJoCo run. Its formation-keeping error is large
  (25 cm on top of the load, more than 1 m in the "behind" mode).
- **What causes it.** The histogram laws steer the centroid to the smallest
  clearance the kinematic gate admits. With wheel slip and tracking lag in
  physics, that clearance is used up. MPC and the DAgger policy keep a
  larger margin.
- **Fixed in part by a physical safety margin.** The fix is a 0.10 m
  margin in the gate, applied to every law equally (batches P3 and P4,
  [SIGNALS.md](SIGNALS.md) S1).

  | Batch | PA-VFH clean runs, no margin → 0.10 m margin | MPC, same |
  |---|---|---|
  | P1/P3, load virtual or carried on top | 0/20 → **16/20** | 20/20 → 20/20 |
  | P1/P3, load towed behind | 0/10 → 0/10 | 2/10 → 2/10 |
  | P2/P4, robot-parameter sweep | 0/40 → **22/40** | 39/40 → 39/40 |

- **What still fails.** Under model mismatch (heavier chassis, slower
  wheels, low friction) PA-VFH still trails MPC and the DAgger policy
  (38/40). The towed load fails for every law, because the planner does not
  model a load that swings outside the formation. Training the occupancy
  network on physics rollouts remains open.

## 5. Limits

- **Simulation only.** The network is trained and tested with a simulated
  2-D lidar.
- **Domain shift.** The labels come from the simulator's ground truth. On
  hardware the network would need data from the real sensor.
- **Kinematic movers.** Movers are kinematic and never yield. The network
  learns to anticipate constant-velocity shuttles, not people.
- **Safety still comes from the gate.** The network changes *where the
  formation wants to go*. Every pose is still committed by the gate
  (ground truth, sensor monitor, or learned conformal monitor). The network
  itself carries no safety guarantee.
