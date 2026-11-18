# The paper, re-framed

This is the blueprint of the manuscript, built on the final results. It
covers:
- what the paper claims and does not;
- the title and abstract;
- the section-by-section outline with the message of each section;
- the figure and table plan;
- a claims ledger that ties every sentence of the abstract to a result file.

Every number here comes from the repository: [EXTENDED_RESULTS.md](EXTENDED_RESULTS.md),
[RESULTS_TABLES.md](RESULTS_TABLES.md), [SIGNALS.md](SIGNALS.md) and
[ROS2_VALIDATION.md](ROS2_VALIDATION.md). Re-check it against the
regenerated files before submission.

To learn the concepts behind each section and how to read each figure, see
[CONCEPT_GUIDE.md](CONCEPT_GUIDE.md).

---

## 1. Why the framing changed

The first draft proposed a new steering law, P-VFH, and argued it was
better. The review and the extended experiments changed the story:

| Earlier framing | What the evidence showed | New framing |
|---|---|---|
| "P-VFH's expected kernel makes the histogram robust to noise" | The kernel moves the reaction distance by about 3 %. All of the robustness came from the per-beam range filter (E2). | Credit the filter. Keep the kernel only as the analytic baseline for the learned version. |
| "P-VFH anticipates moving obstacles" | It read the movers' future from the traffic contract, which a deployed robot does not have | PA-VFH predicts from the robots' own scans only. The contract-reading law is kept as an upper reference (PA-VFH-K). |
| "The proposal beats the baselines" | MPC is as good or better in clean conditions and about 15 s faster. Once traffic phase is randomised, every planner's contact rate roughly doubles. | The main contribution is a **fair, replicated benchmark**. The new algorithm is one entry, evaluated honestly. |
| "Collision-free" | The guarantee came from a ground-truth gate that only a simulator has | Ground-truth screening and deployable monitors are separated and both measured |
| Simulation results | Physics revealed contacts that kinematics hid | Physics (MuJoCo) and ROS 2 (DDS) are part of the evaluation, with a physical safety margin |

**The paper is now a benchmark-and-method paper.** Its first contribution
is knowing *how much* each design choice matters, under a protocol that
does not flatter anyone. Its second is a hybrid that uses a neural network
where it is defensible, together with an honest measurement of what the
network adds.

---

## 2. Title

**Primary:**

> **Steering a rigid multi-robot payload through dynamic warehouses:
> a replicated benchmark from kinematics to physics and ROS 2, and a
> hybrid planner with learned occupancy prediction**

**Shorter (for venues with a length limit):**

> **A replicated benchmark of local steering for cooperative payload
> transport, with a learned-occupancy VFH**

*TODO (authors): state the expansion of "PA" in PA-VFH at first use.*

---

## 3. Abstract (≈ 250 words)

> Teams of mobile robots that carry one rigid payload must steer the whole
> formation through clutter and moving traffic. Their local steering laws
> are rarely compared under a protocol that is reproducible, fair and
> carried through to physics. We present an open benchmark in which twelve
> steering laws move formations of three to nine differential-drive robots
> through eight warehouse layouts of up to 80 × 50 m. The laws are
> potential fields, VFH variants, the follow-the-gap method, DWA, a lattice
> MPC, MPPI, an imitation-learned policy, and a new hybrid, PA-VFH.
>
> Two independent implementations (MATLAB and Python) agree to below 1 µm
> at zero noise and in all 221 of 221 cells of a 2,730-run noisy grid,
> after a collision-test defect we found was corrected. Randomising the
> moving-traffic phase roughly doubles every planner's contact rate (MPC:
> 19 % → 47 % of runs), showing that a fixed timetable flatters all of
> them. We tune every planner under the same budget, separate ground-truth
> collision screening from deployable sensor-based and conformally
> calibrated learned safety monitors, and replicate selected cases in
> MuJoCo physics and in ROS 2 Humble over DDS.
>
> PA-VFH keeps a classical VFH+ decision layer and learns only one block: a
> circular CNN that predicts per-sector danger 0.5 s ahead from the robots'
> own scans. In a pre-registered confirmation on 360 fresh paired missions,
> the network shortened missions by about 1 s (1 %; 95 % CI 0.3–1.7 s) with
> no measurable change in gate rejections, contacts or reach. PA-VFH had the
> highest reach of the five laws compared, at a third of MPC's compute; MPC
> remained faster. In
> physics, a 0.10 m clearance margin applied to every law removed most
> histogram-planner contacts (PA-VFH: 0/20 → 16/20 clean runs). MPC stayed
> more robust to robot-model mismatch.
>
> Everything is simulated; code, scenarios, weights and all runs are
> released.

---

## 4. Contributions (as they will appear in the introduction)

1. **A replicated, fair benchmark for rigid-payload formation steering.**
   - Two independent stacks that agree step for step.
   - A collision-test defect found and corrected.
   - Randomised traffic phase, common random numbers and equal-budget
     tuning.
   - Held-out worlds and seeds, and every failed run reported with 95 %
     intervals.
2. **Evidence on what matters.**
   - The per-beam range filter, not the uncertainty-aware kernel, carries
     noise robustness.
   - A fixed traffic timetable hides half of all contacts.
   - Layout and team size change the ranking. Narrow 2.4 m aisles defeat
     every law. A 2 × 2 m nine-robot payload fails in the study
     warehouse's 2.45 m corridor for every law but MPPI (7/10), yet
     passes in the large rack warehouses.
   - Physics exposes clearance-hugging behaviour that kinematics hides, and
     a uniform physical margin repairs most of it.
3. **Safety screening separated from safety monitoring.**
   - A ground-truth gate (simulator only).
   - A geometric scan monitor (deployable).
   - A learned monitor with a split-conformal margin.
   - Each is scored for false accepts against ground truth.
4. **PA-VFH, a hybrid with learned occupancy.** A classical, auditable
   decision layer with a small learned predictor that uses only the robots'
   own scans. It is trained against oracle labels with on-policy
   aggregation and evaluated against its own non-learned ablation.
   - Alongside it, a team-size-agnostic DAgger policy as the end-to-end
     learned baseline.
5. **A path to deployment.**
   - A ROS 2 package that reproduces the in-process MuJoCo missions
     exactly over DDS (90/90 runs), and drives real-time MuJoCo physics
     from laser scans alone (9/10 clean).
   - A real-sensor mode (LaserScan input).
   - A safety watchdog.
   - A sensor-accuracy requirement (σ ≤ 5 cm), derived from the
     experiments.

---

## 5. Section-by-section outline

Each section lists its **one message**, then its content and evidence.

### I. Introduction

**Message:** cooperative payload transport needs local steering for a
rigid, rotating body among moving traffic. Today's comparisons don't tell
a practitioner which law to use, or why.

- **Setting:** warehouses, shared payloads, shuttles that do not yield.
- **Gap:** learned and classical planners are evaluated on different
  scenarios and with different safety assumptions; usually without
  replication, with a fixed traffic timetable, and in kinematics only.
- **Contributions:** §4 above.
- **Figure 1:** the system at a glance. Formation shapes (concept
  figure), a warehouse layout and the PA-VFH pipeline diagram.

### II. Related work

**Message:** each community covers one piece, and nobody tests all the
pieces together.
- Cooperative transport and formation control.
- Reactive local planners: VFH family, FGM, DWA, MPC, MPPI.
- Learned multi-robot navigation: MARL, GNN and DeepSets policies.
- Learned safety: neural barriers, conformal prediction.
- Benchmarking and reproducibility.

Source text: [RELATED_WORK.md](RELATED_WORK.md) and
[NEURAL.md](NEURAL.md) §1–2. End with the gap table G1–G5.

### III. Problem and system

**Message:** what is controlled, what is sensed, and what counts as
success.
- Unicycle robots.
- Forming, then docking, then rigid transport.
- The payload body is the convex hull of the slots plus the robot radius.
- The swept corridor of a rotating body.
- Movers follow a piecewise-linear contract and never yield.
- Sensing: 288-beam lidar or a 60° ToF fan, with range noise σ0.
- The mission layer, the same for every law: the atomic-pose safety gate,
  the escape probe, the anti-deadlock layer and load-carrying motion
  limits.

### IV. Methods: the steering laws

**Message:** twelve laws, one interface. Only the steering vector differs.
- Classical: APF, VFH, FGM, DWA, lattice MPC, MPPI.
- The histogram ladder, adding one ingredient at a time: VFH → pVFH (mover
  horizon) → pVFH+F (range filter) → PA-VFH-K (expected kernel, contract
  prediction).
- Learned:
  - **DAgger policy:** a circular CNN plus DeepSets over the slots,
    imitating MPC.
  - **PA-VFH:** the occupancy CNN inside VFH+. Give the architecture, the
    6 × 72 input, the oracle labels, the soft-label BCE loss, the two
    on-policy rounds and the 11 k parameters.
- **Figure 2:** the PA-VFH pipeline, with S3 inset (the network's danger
  map against the oracle).

### V. Safety: screening vs monitoring

**Message:** a guarantee is only as good as the information it uses.
- **Ground-truth gate (simulator):** an exact separating-axis test,
  including the defect we found in the reference.
- **Geometric sensor monitor:** a scan-only clearance check with
  k·σ + v·h inflation.
- **Learned monitor:** a regressed clearance with a split-conformal
  quantile q_α.
- **Audit mode:** the truth decides and the monitor is scored in its
  shadow.

### VI. Benchmark protocol

**Message:** every choice is made so that no law is flattered.
- Common random numbers.
- Randomised traffic phase on an independent RNG stream.
- Evaluation seeds 1–10; training and tuning seeds ≥ 101.
- Held-out worlds.
- Equal-budget tuning: 24 configurations × 7 missions, defaults included.
- Wilson and bootstrap 95 % intervals; failed runs counted.
- The cross-stack agreement criterion.

### VII. Results

| § | Question | Evidence | Message |
|---|---|---|---|
| A | Can the study be replicated? | T1c, Fig. xcheck, cross-stack scatter | Two stacks agree: < 1 µm at zero noise, 221/221 cells |
| B | What carries noise robustness? | E2 (ablation ladder) | The range filter. The kernel's effect is about 3 % (concept_kernel) |
| C | Does the traffic timetable matter? | R2, fig_phase_effect | Yes: contact runs roughly double for every law |
| D | Scale, layout, team size | E15, E16 heatmaps | Narrow aisles, and a large payload in a corridor narrower than its swept diameter, are open problems. PA-VFH is tied highest in reach on both E15 (69/80) and E16 (138/150) |
| E | What does deployable safety cost? | E17, fig_e17_monitor, S4 | The sensor monitor holds reach at low noise and fails at σ0 = 0.10. The learned monitor holds reach but admits more |
| F | Is the comparison fair? | E18 tuning | Tuning changes held-out cost little except for FGM and DWA. The defaults were not a straw man |
| G | Does the network help PA-VFH? | E19 (R9), E20 (R11, pre-registered), S3 | Significantly but slightly faster (about 1 %), otherwise neutral; E19's apparent rejection gain did not replicate. The highest reach of five laws, at a third of MPC's compute |
| H | Does it survive physics? | P1–P4, S1, S2 | Contacts hidden by kinematics appear. A 0.10 m margin fixes most for PA-VFH; MPC is more robust to model mismatch. Towed loads defeat all laws |
| I | Does it run in ROS 2, with physics? | ROS2_VALIDATION §1b–1c, R10, S5 | MuJoCo through ROS 2 lockstep: 90/90 runs identical to in-process. Full deployment stack in real-time MuJoCo (scans over DDS): 9/10 clean. Watchdog in physics: stop 0.3 s after a lidar failure |

### VIII. Discussion

1. **Where learning pays, and where it doesn't.**
   - The learned occupancy model helps a little. Its real value is that it
     removes the need for the traffic plan without losing reach.
   - The end-to-end DAgger policy matches MPC at 2.5 × less compute and is
     the most robust in physics. Learning to imitate a good planner is the
     strongest learned result here.
2. **Fixed timetables are a benchmark trap.** Report phase-randomised
   results.
3. **Kinematic safety is not physical safety.** Planners that hug the
   gate's boundary need a physical margin; MPC's internal `d_safe` already
   provides one.
4. **Sensor-accuracy requirement.** Above σ ≈ 5 cm, any monitor with a 3σ
   margin closes the aisles. This is a specification for hardware, not a
   tuning issue.
5. **Open problems:**
   - narrow aisles;
   - payloads whose swept diameter exceeds the corridor;
   - towed loads;
   - non-yielding traffic, and people.

### IX. Limitations

Copy and keep: [§7 below](#7-what-the-paper-does-not-claim).

### X. Conclusion and data availability

- Summarise contributions 1–5 in three sentences.
- Release: archive, `MANIFEST.sha256`, and the Zenodo DOI
  ([RELEASE.md](RELEASE.md)).

---

## 6. Figure and table plan (main paper ≈ 8 figures, 4 tables)

| # | Content | Source file | Reading guide |
|---|---|---|---|
| Fig. 1 | System overview: formations, layout, PA-VFH pipeline | `docs/figures/concept_formations.png`, `results/figures_ext/fig_layouts.png`, pipeline diagram (demo slide 5) | CONCEPT_GUIDE §G1, G12 |
| Fig. 2 | Inside PA-VFH: oracle vs network vs kernel | `results/figures_signals/S3_pavfh_danger_maps.png` | §G23 |
| Fig. 3 | Replication: zero-noise paths and cell scatter | `docs/figures/xcheck_trajectories_fixedgate.png`, `cross_stack_scatter_fixedgate.png` | §G6, G7 |
| Fig. 4 | Ablation ladder against noise | `docs/figures/e2_noise_both_stacks_fixedgate.png` | §G9 |
| Fig. 5 | Effect of the traffic phase | `results/figures_ext/fig_phase_effect.png` | §G13 |
| Fig. 6 | Layouts and team size | `fig_e15_reach.png`, `fig_e16_dynamic.png` | §G14, G15 |
| Fig. 7 | Safety: monitors and the monitor signal | `fig_e17_monitor.png`, `S4_monitor_signals.png` | §G16, G24 |
| Fig. 8 | Physics: the margin fix and one mission's signals | `S1_physics_margin_clearance.png`, `S2_physics_signals.png` | §G21, G22 |
| Table I | Steering laws: information used, compute | DOCUMENTATION §4, E19 comp_ms | – |
| Table II | Cross-stack and zero-noise agreement | T1c, CROSS_STACK_fixedgate | – |
| Table III | E19 ablation, pooled and paired | EXTENDED_RESULTS R9 | §F |
| Table IV | Physics P1/P3/P4 and ROS 2 summary | R8, ROS2_VALIDATION §1b | – |
| Supplement | Every run; E15–E18 per cell; S5–S6; demo video | EXTENDED_RESULTS, `results/demo/demo.mp4` | – |

---

## 7. What the paper does not claim

- **Simulation only.** Kinematic, MuJoCo physics and ROS 2 runs with
  simulated sensors. Nothing has run on hardware.
- **The ground-truth gate is a screening device.** In `truth` mode
  collisions with statics are prevented by an oracle. Those results
  measure what a law *asks for*, not deployable safety.
- **Deployable safety holds only for the monitors, and only as measured.**
  The figures are false-accept rates for this sensor model, these noise
  levels and these kinematic movers. The learned monitor's conformal
  guarantee is marginal and in-distribution, and it did not transfer to
  fused per-robot scans without re-calibration.
- **Learned components carry no guarantee.** The occupancy network and the
  DAgger policy change where the formation wants to go. The gate commits
  every pose.
- **The network's gain is small**: significant only for mission time (about 1 %, pre-registered E20); no effect on rejections, contacts or reach (E19 paired
  intervals include 0).
- **Movers are constant-velocity shuttles that never yield.** No people
  appear in any scene.
- **Sensor requirement:** range noise σ ≤ 0.05 m. Above it the real-sensor
  pipeline loses reach.

---

## 8. Claims ledger (abstract sentence → evidence)

| Claim | Number | Where |
|---|---|---|
| Two stacks agree | < 1 µm at zero noise (7 × 10⁻⁷ m); 221/221 cells | `results/deterministic_xcheck_fixedgate.csv`, `results/CROSS_STACK_fixedgate.md` |
| A defect was found and corrected | Non-cyclic corner order; one box normal never tested | DOCUMENTATION §5.3; `matlab_ref/formation_clearance_dyn.m` |
| Phase randomisation roughly doubles contacts | MPC 61/320 → 151/320 runs | EXTENDED_RESULTS R2 |
| Twelve laws, 3–9 robots, 8 layouts up to 80 × 50 m | – | `tools/run_extended.py` (E15, E16) |
| Network: about 1 s faster, otherwise neutral (pre-registered) | E20: time −0.97 [−1.66, −0.33] s; rejections +0.12 [−2.71, +3.00] | R11, PREREGISTRATION_E20.md |
| Highest reach of five, a third of MPC's compute | 0.77 vs 2.46 ms per step | R9 |
| MPC faster | 74.7 vs 90.3 s | R9 |
| Margin: PA-VFH 0/20 → 16/20 clean | P1 vs P3, virtual and on-top modes | R8; `results/mujoco_phase/` |
| MPC more robust to model mismatch | P4: 39/40 vs PA-VFH 22/40 | R8 |
| ROS 2 + MuJoCo identical to in-process | 90/90 runs, every metric (same environment) | `results/ros_physics/R1/*.row.json`, R10 |
| Deployment stack in real-time MuJoCo | 9/10 reached with zero physics contacts | `results/ros_physics/R2/`, R10 |
| Watchdog in physics | flag 0.3 s after a lidar failure; all commands 0 | `results/ros_physics/DEMO/DEMO_watchdog_on_top_s1.json` |

---

## 9. Venue fit

- **Best fit:** a journal that values benchmarks and negative results, such
  as *Robotics and Autonomous Systems* or the *Journal of Intelligent &
  Robotic Systems*.
- **Also possible:** IEEE RA-L, with the supplement carrying the per-cell
  results.

The released artefact (code, scenarios, weights, every run, the Docker
recipe for ROS 2) is a strength for reproducibility badges.
