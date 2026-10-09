# Is this at the level of a Q2 journal paper?

A review of the project as it stands on 3 October 2026, written the way a
referee would read it. "Q2" here means a second-quartile robotics or
automation journal (the tier of *Robotics and Autonomous Systems*,
*Journal of Intelligent & Robotic Systems*, *Robotica*, *Sensors*,
*IEEE Access*): sound method, honest evaluation, a clear but not
necessarily large contribution.

## Verdict

**Not yet, as the paper is currently framed. Close, if it is reframed.**

The engineering and the evaluation are above what most Q2 submissions in
this area offer. The problem is the claim. The proposed method, P-VFH, does
not outperform existing planners in this project's own data, and its named
novelty (the expected kernel) has no measurable effect. A paper titled
around P-VFH would be rejected on that point alone, however good the rest
is. A paper built on what the data does support is publishable at Q2 with
about two to four weeks of further work.

## Scorecard

| Criterion | State | Evidence |
|---|---|---|
| Sound, reproducible method | **Strong** | Two independent implementations identical step for step at zero noise (8 laws); 220 of 221 cells agree with noise; 52 tests; seeded; one-command grids |
| Statistical rigour | **Good** | 10–20 seeds per cell, Wilson and bootstrap intervals, paired tests, ablation ladder |
| Breadth of evaluation | **Strong** | 2,730 runs, 13 experiments, 3 designed worlds + 10 random layouts, noise, sensors, shapes, traffic, localisation error, command loss |
| Baselines | **Adequate, attackable** | FGM, DWA, lattice MPC are present, but are untuned re-implementations |
| Honest reporting | **Strong** | Negative results stated; a defect in the reference's own gate found and quantified |
| Novelty of the method | **Weak** | Expected kernel changes the reaction distance by 3 %; P-VFH ≡ pVFH + range filter in every cell |
| Does the proposal win? | **No** | MPC: 46 s vs 50 s in the study's warehouse, 51 s vs 93 s in the AWS warehouse, fewer rejections, larger gaps |
| Validation beyond kinematics | **Adequate** | MuJoCo with mass, wheels, a pinned 12 kg load; ROS 2 in the loop. One seed per configuration |
| Hardware | **Absent** | No real robot, no real sensor data |
| Related-work positioning | **Unknown** | Not assessed here; the report in the MATLAB repository was not reviewed |

## What a referee would object to, in order of severity

**1. The method does not beat the baselines (fatal for the current framing).**
MPC and DWA are faster and at least as safe wherever passages are narrow:
the plaza, the random layouts at low noise, and the AWS warehouse, where the
histogram laws take almost twice as long. Under physics, the
histogram laws were hit by a mover in all five seeds run, while MPC
delivered the load cleanly in all five (that contrast depends on traffic
timing, see point 7). The histogram laws are better only at double mover speed
and with six movers, where MPC and DWA lose 2–4 of 10 runs.

**2. The stated novelty is not the active ingredient.**
The ablation is clean and its answer is unambiguous: the recursive range
filter produces the noise robustness (at σ0 = 0.1 m, 77 s → 53 s and ~40 →
~3 rejections); the expected kernel adds nothing in any cell. A per-beam
Kalman filter with an innovation gate is standard. The analytic reason is
also in hand (kernel width 1.3 m against a posterior range sigma below
0.2 m), which makes this a well-explained negative result rather than a
loose end.

**3. No hardware.**
For a transport-by-robot paper most referees at this tier will ask for at
least a small physical demonstration, or will require the title and
abstract to say "simulation study".

**4. The safety gate uses ground truth.**
Gate rejections are a sound *metric*, but the system as described cannot be
deployed: a real gate would have to work from the same noisy scan. The
paper must say this plainly.

**5. Baselines are untuned and compact.**
"A navigation-stack MPPI/TEB would do better still" and "your DWA weights
are arbitrary" are both fair. Since the baselines already win, tuning them
only strengthens objection 1.

**6. The reference gate defect.**
The MATLAB tables in the bundle were produced with the defective collision
test. It is conservative and changes no ranking, but it inflates the
headline safety metric at low noise (3–7 rejections per run that are really
0–0.5). With the corrected gate the two stacks differ in 25 of 221 cells, and
21 of those are exactly these low-noise cells. The MATLAB grid should be
re-run with the fix before submission.

**7. Mover timing is not randomised (found in this work, affects the whole study).**
Movers follow a fixed timetable and a seed only changes sensor noise, so all
seeds of a cell meet the traffic at the same instant. On the AWS warehouse,
delaying the start by 4 s is enough for the histogram laws to be run into,
and by 6–8 s every law is, MPC included, although all show zero mover
contact at the nominal timing. Every mover-contact number in both grids is
therefore conditional on one traffic phase per world. A referee who notices
this will discount the safety comparison; it must be fixed, not explained.

**8. Thin physics statistics.**
Five seeds for the carried plate (MPC: 11–13 cm RMS, no contact, in all
five), one seed for the other configurations; one robot parameter set; a
generic platform rather than a named one.

**9. Kinematic movers that never yield** make "mover contact" partly a
property of the scenario. Already acknowledged in the documentation; needs
the same in the paper.

## What the data does support: three framings that would pass

**A. A reproducible benchmark and replication study (the strongest option).**
*"Local steering laws for rigid multi-robot payload transport: a replicated
benchmark from kinematics to physics."* Contributions: (i) an open benchmark
of eight laws over thirteen experiments with common random numbers and a
ground-truth safety gate as the metric; (ii) an independent
cross-implementation replication, exact at zero noise, which is rare in
this literature and found a real defect in the reference; (iii) a physics
validation that measures where the kinematic model stops being predictive
(planned gaps below ~0.4 m, towed loads outside the footprint, turn rates
beyond the robots' speed). Negative results are welcome in this framing.
Fits *Robotics and Autonomous Systems*, *JINT*, *Sensors*, *IEEE Access*.

**B. Noise-robust histogram steering by per-beam range filtering.**
A narrower, honest methods paper: the filter is the contribution, the
expected kernel is reported as an analysed negative result with the proof of
its bounded effect, and MPC is presented as the stronger but costlier
alternative. Needs a computational-cost comparison in the method's favour
(currently about 1–2 ms per step for the histogram laws against 2.5–5 ms
for MPC, which is not a decisive gap) and a regime where it wins (dense or
fast traffic, where MPC and DWA lose runs). Weaker than A.

**C. Formation-level mission layer for load-carrying robots.**
The atomic pose commit, the anti-deadlock layer (1–3 of 40 runs reach
without it, 40 of 40 with), the load-feasible motion limits and the pinned
rigid-twist tracking controller are law-independent and useful with any
planner, including MPC. Combined with the physics validation this is a
coherent systems paper; it would want hardware.

## What to do before submitting (ranked by value for effort)

| # | Action | Effort | Effect |
|---|---|---|---|
| 1 | Choose framing A (or A with C) and rewrite title, abstract and claims accordingly | days | Removes the fatal objection |
| 2 | Fix the corner order in `formation_clearance_dyn.m` and re-run the MATLAB grid (30 min on 6 processes) | 1 h | Clean safety metric; the two stacks then match with the exact gate too |
| 3 | Randomise the mover phase per run (a random start offset in the contract) and re-run both grids | 1 day incl. compute | Makes every mover-contact and reach statistic valid beyond one timetable; the largest fix to the evaluation itself |
| 3b | Physics validation with 10 seeds per configuration (with randomised mover phase) and a robot-parameter sweep (mass, speed limit) | 1 day of compute | Turns the physics section from a demonstration into evidence |
| 4 | Tune the baselines, or cite a navigation-stack planner run on the same worlds | 2–4 days | Closes the "weak baselines" objection |
| 5 | A sensor-based gate (same noisy scan, inflated by the filter's posterior sigma), reported next to the ground-truth one | 3–5 days | Makes the system deployable in principle; a real contribution for framing C |
| 6 | A named platform as a second physics configuration (TurtleBot 4 footprint and speed) | 0.5 day | Answers "why a generic robot" |
| 7 | A small hardware run: two to four robots carrying a board through a few obstacles, using the existing ROS 2 node | 1–2 weeks | The single largest increase in acceptance odds |
| 8 | Release code, worlds and result files with a DOI | 0.5 day | Expected for a benchmark paper |

Items 1–3b and 8 are enough for a credible Q2 submission as a simulation and
replication study. Item 7 is what would move it toward Q1.

## What should not be done

- Do not tune parameters or pick seeds until P-VFH wins. The data is clear,
  and the replication makes any such move easy to detect.
- Do not report "220 of 221 cells agree" without its caveat: interval
  overlap is lenient where a law fails in some seeds.
- Do not describe the MuJoCo result as validating the *method*. It validates
  the kinematic *model* for planners that keep their margins. Do not present
  "MPC succeeds, P-VFH fails under physics" as a physics finding either: it
  is a traffic-timing effect that the kinematic model reproduces.
- Do not call the ROS 2 run a deployment. It is the same simulation with
  the controller and the physics in separate processes.

## Limits of this review

It is based on the code and data in this workspace. I did not read the LaTeX
report in the MATLAB repository, so related work, writing quality and
figure quality of the manuscript itself are not assessed. The journal names
are examples of the tier, not a check of their current scope or ranking.
All batches were complete when this was last updated (both Python grids,
48 MATLAB warehouse missions, 200 Python warehouse missions, 45 MuJoCo
missions).
