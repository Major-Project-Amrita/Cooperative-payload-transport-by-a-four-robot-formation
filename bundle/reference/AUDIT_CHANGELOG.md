# Audit changelog — October 2026

What was found, what was changed, and what still has to be run. Evidence for
each defect is in the pre-fix per-step logs under `logs/` (repo root) and the
pre-fix table `matlab/logs/paper/results_all.csv` (Octave, September 2026).

**Execution status (2026-10-02, MATLAB R2025a).** The corrected code has been
run: all seven steering laws on the warehouse, plaza, maze and two random
layouts; the line formation; a ToF sensor; zero movers; localisation noise
with command loss; the motion limits on and off; and the driver's smoke test
(19 configurations, 35 s) followed by the figure/table generator on its
output. No run crashed and **no run had a static-contact step**. A mission
takes about 1.5–3.5 s of wall time, so the full grid (2,410 jobs, 2,130
unique configurations) is roughly 2 hours on one process and about 25
minutes on six. **The full grid itself has not been run**; the report still
shows the legacy table until it is.

## 1. Why tables and graphs disagreed

The report took E1–E3 tables from the Octave table, E4–E6 tables from the
Python headless port, and every per-step figure (trajectory overlays,
formation trails, time series) from the Python port. The two implementations
do not behave the same, so a table and the figure beside it described
different runs. The text then explained the differences as findings.

Fix: one results file (`matlab/logs/paper/results_all.csv`) feeds one script
(`matlab/make_paper_figures.py`) that writes every figure, every table
(`docs/simulation_report/generated/*.tex`, `\input` by the report) and
`docs/RESULTS_LOG.md`. The experiments chapter no longer contains hand-typed
result values.

## 2. Defects in the MATLAB reference (all fixed)

| # | Defect | Evidence | Fix |
|---|---|---|---|
| 1 | Heading was committed before, and regardless of, the collision check; only translation was gated. A blocked body rotated in place into the obstacle. | Every collision step examined in the old logs has zero translation and non-zero yaw change (e.g. `paper_e6_vfh_tof016_sg002_s1`: 30 collision steps, position frozen, yaw 3°→25°). | `Simulation8_dynamic.m`: atomic pose commit — translate+rotate, else rotate-only if safe, else hold. |
| 2 | `'pvfh'` used `pvfh_predict`, which raycasts a noise-free full-circle lidar and never calls the sensor model. | `paper_e1_pvfh_s1` and `_s2` are identical step for step (max difference 0.0). | `steering_dispatch.m`: `'pvfh'` senses through the shared noisy front end. |
| 3 | `'vfh'` rebuilt beam angles as a full circle for any sensor, so a 60° ToF fan was treated as 360°. | Code reading (`local_vfh_from_scan`); the Python port passes real angles, which is why the two stacks "disagreed" on ToF. | Bearings come from the scan. |
| 4 | The APF reflex used a beam width of 2π/n for a 60° fan (6× over-weight). | Code reading (`apf_obstacle_repulsion.m`). | `scan_ranges_dyn` returns the beam width; all callers pass it. |
| 5 | `'vfh'` was documented as horizon-free but inherited `pred_lead = 0.5 s`; and with `pred_lead = 0` the scan skipped movers entirely. | Code reading. | Baselines use an instantaneous scan that does see movers; `'pvfh'`/`'pvfh_prob'` use the horizon. |
| 6 | The ToF fan was fixed to world +x rather than the body heading. | Code comment. | `sense.frame = 'body'` (default); `'world'` restores the old fan. |
| 7 | `comp_ms_mean` averaged phase-1 values (summed over robots), phase-2 values and zeros from escape steps. | Code reading (`mean(LOG.comp)` over all steps in the old driver). | `summarize_run.m`: phase-2 steering steps only. |
| 9 | `world_dist_dyn.m` crashed with zero movers (`d = d(:)` on an undefined variable). | Code reading; the zero-mover path was never exercised. | `d = D(:)`. Needed by the static-clearance log and by E11. |
| 11 | **`pvfh_prob.m` evaluated the kernel with `/` instead of `./`.** For two row vectors MATLAB's `/` returns one least-squares scalar, so every beam got the same kernel value: the P-VFH histogram was identical in every direction. | At a plaza pose with obstacles 1.4 m away, max kernel value 0.003 (should be 0.56); all 72 sectors read free; P-VFH circled in place for 60 s. After the fix its histogram matches the deterministic one to 0.02. | `./`. **Every P-VFH result before this fix, including the first corrected grid of 2,410 runs, is invalid** — the law was steering by goal attraction and reflex only. The Python port had the formula right. |
| 10 | `merge_dyn.m`: with no circle (or no box) movers it appended a dummy element and deleted element 2, removing a real static obstacle and leaving an empty one. | Crash in `dyn_bounds` on the maze and random worlds (all-box movers). | The static list is kept unchanged when there are no movers of a kind. |
| 8 | Per-step logs were written relative to the start directory. | Stray `logs/` at the repo root. | The driver changes into `matlab/` and writes to `logs/paper/runs/`. |

## 3. Errors in the report text (all corrected)

- "∂u/∂s > 0 at every range" is false. True: it holds iff d² > σ_h² + s²; the
  reaction distance grows with noise up to s* = 1.757 σ_h. Now Proposition 2,
  with proof.
- The noisy-OR risk layer was credited with the sparse-sensor results; the
  code never used it for steering. Now described as a diagnostic.
- ToF noise was written in quadrature in the experiments chapter; the code
  and the algorithms chapter use the linear form σ0 + k·d. Text aligned.
- "Two laws sharing a seed see the identical noise for every scan" holds only
  until their trajectories diverge. Reworded as common random numbers.
- The E4 table quoted the square's circumradius at s = 1.18 m (0.834 m) while
  running at s = 0.60 m (0.424 m); the line's length was given as 3.0 m
  (it is 2.16 m).
- Explanations of cross-stack differences ("range-growing noise grinds the
  deterministic histogram", "narrow FoV protects") rationalised defects 1–4.
  Removed.

## 4. Figure defects (all regenerated)

- `e5_stall_timeseries`: vfh and pvfh colours swapped relative to every other
  figure; "escape commit" band and "graze" arrow hard-coded at fixed times.
- `trajectory_overlays`: empty sixth panel, no map, legend unreadable.
- `formation_sketch`: circumradii in units of s, inconsistent with the table.
- `e6_graze_timeseries`: caption described a below-zero burst the plotted
  column does not show.
- `e6_sensor`: x-groups sorted alphabetically (tof-8 before tof-16), not in
  table order; legend over the bars.
- `ablation_noise`, `sweep_tof`: twin axes with overlapping lines and no
  uncertainty.

Now: one colour and order per algorithm everywhere, paths drawn on the world
map, intervals on every mean, annotations taken from the logs, runs chosen by
rule (median duration / worst seed) rather than by hand.

## 5. Added (the simulation-side roadmap items)

- **Baselines**: Follow-the-Gap (`'fgm'`), a Dynamic-Window planner (`'dwa'`)
  and a receding-horizon lattice MPC (`'mpc'`, the optimisation-based class)
  in `steering_dispatch.m`, same scan and interface. The MPC is a compact
  re-implementation, not a navigation-stack TEB/MPPI.
- **Environments**: `swarm_world_plaza.sdf`, `swarm_world_maze.sdf` and their
  contracts; corridor bottlenecks checked (1.55 m and 1.60 m against the
  1.01 m the square needs). The maze contract carries a waypoint route
  (`contract_waypoints.m`).
- **Wedge recovery**: a body blocked in both translation and rotation for
  more than 5 steps backs up half a step (or turns the other way). Found by
  `check_motion_limits`: `pvfh` seed 1 held its pose for 2,857 steps and
  timed out, with the limits on *and* off, while the escape probe kept
  naming a heading the body could not rotate to.
- **Load-carrying motion limits** (`cfg.kin`, **off by default**, evaluated
  as experiment E14; every run now reports `max_robot_speed`): braking
  curve, yaw rate ≤ (v_robot_max − v_floor)/r_c, yaw acceleration ≤
  2.5 rad/s², speed reduced so the outer robot stays ≤ 1.0 m/s and lateral
  acceleration ≤ 0.8 m/s², with a speed floor of 0.5 V. The first version
  had no floor: the formation stopped to pivot in a mover's lane and was hit
  (28 mover-contact steps, seed 1). With the floor it is still ~7 s slower
  and still hit (17–18 mover-contact steps on 3 of 3 seeds, 0 with limits
  off), which is why the limits are an experiment and not the default.
  `check_motion_limits.m` is the quick on/off comparison. Evidence
  it was needed: the first corrected run (pvfh_prob, seed 1) asked a corner
  robot for 3.0 m/s and 13.7 rad/s² of yaw acceleration. Also **docking**:
  at the forming→transport switch the robots used to jump 0.24–0.35 m onto
  their slots; the residual now decays over ~1 s (`cfg.forming.dock_tau`).
- **Diagnostic plots**: line + markers (`pline.m`), vector PDF export to
  `logs/figs/`, a mission-path figure (works around the R2025a WebGL
  line-rendering glitch on Linux).
- **Anti-deadlock layer** ported into the MATLAB loop, applied to every
  algorithm, with an off switch (`cfg.adl.enable`).
- **Experiments**: E7 (environments), E8 (traffic speed,
  `cfg.dyn.speed_scale`), E9 (anti-deadlock ablation), E10 (localisation
  error, `cfg.loc`), E11 (traffic density, `cfg.dyn.max_movers` with the
  six-mover `dynamic_obstacles_dense.json`), E12 (command-update loss,
  `cfg.comm_drop`), E13 (ten randomly generated layouts,
  `make_random_worlds.py`; bottlenecks 1.35–1.93 m). E4 is now the P-VFH
  rows of E5 rather than duplicate runs. About 2,350 runs in total.
- **Statistics**: 10 seeds per cell (20 in E2), Wilson and bootstrap
  intervals, paired Wilcoxon and McNemar tests.
- **Theory**: Propositions 1–2 with proofs, cost statement, analytic figure.
- **Metrics**: gate rejections, static vs mover contact, escape count; new
  per-step columns `reject`, `escape`, `clearance_static`.
- **Driver**: manifest, shards, per-run checkpoint, merge
  (`run_paper_experiments.m`, `run_grid_parallel.sh`).

## 6. Files

Changed: `world_dist_dyn.m`, `Simulation8_dynamic.m`, `steering_dispatch.m`, `scan_ranges_dyn.m`,
`apf_obstacle_repulsion.m`, `pvfh_prob.m`, `find_escape_heading_dyn.m`,
`summarize_run.m`, `print_summary.m`, `write_log_csv.m`,
`run_paper_experiments.m`, `make_paper_figures.py`; report sections 02, 03,
05, 07, 08, 09, 10, 99 and `main.tex`.
New: `range_filter.m`, `pline.m`, `check_motion_limits.m`, `contract_waypoints.m`, `run_grid_parallel.sh`, `make_random_worlds.py`,
the plaza/maze worlds, ten `swarm_world_randNN.sdf` layouts, all contracts
(incl. `dynamic_obstacles_dense.json`), `docs/RUN_GUIDE.md`, `docs/simulation_report/generated/`, `docs/RESULTS_LOG.md`, this
file.
Removed: `make_log_figures.py`, `fill_e5e6_tables.py` (merged into
`make_paper_figures.py`).
Unused but kept: `pvfh_predict.m`, `pvfh_scan.m` (static `Simulation7_sdf`
path), `run_headless_grid.py` (drives the un-fixed Python port).

## 7. What the current PDF shows

`docs/simulation_report/main.pdf` currently displays the **pre-fix** Octave
data (the banner at the top of the experiments chapter says so), E7–E9 as
"pending". It becomes the real report after:

```bash
cd matlab && matlab -batch "run_paper_experiments('smoke')"
```
```bash
rm -rf matlab/logs/paper/parts && matlab/run_grid_parallel.sh 6
```
```bash
python3 matlab/make_paper_figures.py
```
```bash
cd docs/simulation_report && pdflatex main && pdflatex main
```

## 7a. After the kernel fix (2026-10-02, pilot runs only)

- With the kernel working, single-scan P-VFH behaves like deterministic
  pVFH at every noise level (warehouse, 5 seeds: 50 → 239 s vs 50 → 233 s
  as σ0 goes 0.02 → 0.40). The expected kernel alone gives no noise
  robustness.
- Added `range_filter.m`: a per-beam recursive Bayesian (Kalman) range
  filter whose gain is set by the sensor's s_b, with 3-sigma jump reset. It
  is now part of P-VFH (transport phase, lidar). Same pilot: 49 → 97 s,
  rejections 37 instead of 101 at σ0 = 0.40.
- Added the ablation law `'pvfh_filt'` (pVFH + the same filter). It does
  about as well as P-VFH (100 s, 28 rejections at σ0 = 0.40): the filter
  carries the effect, the expected kernel adds little.
- `cfg.blend_fade` (default on): the goal weight is scaled by the agreement
  between goal and steering directions. VFH/pVFH on the plaza: ~120 s with
  ~100 rejections → 59 s with none.
- The FGM, DWA and MPC baselines now get the same mover-prediction horizon
  (`cfg.baseline_horizon`, default on): their warehouse mover contacts went
  from 21–27 steps per run to 0.
- An "escalating escape" variant (`cfg.adl.escalate`) was tried and made
  reach and time worse; it is off.
- Motion limits (`cfg.kin`): no setting found that holds across worlds
  (a 1.5 s horizon helps the histogram laws in the warehouse and plaza but
  hurts the MPC baseline). Still off by default, still experiment E14.
- The first grid is archived in
  `matlab/logs/paper/archive_grid1_kernelbug/`. The manifest is now 2,730
  jobs (8 laws). The report's Findings section is withdrawn until the grid
  is re-run.

## 7e. THIRD grid of 2026-10-02 (2,730 runs, 0 static-contact steps) — current, final code

Report section "Findings" is written from this grid. Filter step at
σ0 = 0.10: 77 → 53 s, 39 → 2 rejections, 22 → 0 mover-contact steps; kernel
step: no resolved difference anywhere (the weak random-layout signal of
grid 2 did not repeat). Histogram laws now match or beat MPC on the random
layouts; MPC still best on the plaza and fastest in the warehouse at normal
traffic; MPC/DWA lose runs at 2× mover speed and with six movers.

## 7d. Method-improvement attempt after the second grid (pilot sweeps only)

Tested on the plaza (10 seeds), the ten random layouts, warehouse, maze and
line formation; a change was kept only if it won without breaking anything.

| Change | Result | Decision |
|---|---|---|
| Stall window 60 s → 15 s (`cfg.adl.window = 150`) | P-VFH plaza 109 → 79 s, random layouts 77 → 60 s (level with MPC's 57 s); nothing worse | **kept** |
| Stall window 10 s / 6 s | plaza a little faster at 10 s but mover contacts on the line formation; 6 s worse on the plaza | rejected |
| VFH+ "previous selected direction" cost term (`vfh.commit_weight`) | plaza 109 → 169 s, rejections ×3 | rejected (code kept, weight 0) |
| Chance-constrained range d − κ·s (`vfh.lcb_kappa`, κ = 1, 2) | slower and more rejections at every noise level; κ = 2 loses runs at σ0 = 0.4 | rejected (code kept, κ = 0) |

Net effect: the mission layer is better; **no P-VFH-specific gain was
found**. P-VFH and pVFH+F remain indistinguishable. Reason, in hindsight:
the kernel width σ_h (1.3 m) is far larger than any posterior range sigma
(≤ 0.2 m after filtering), so the expected kernel differs from the
deterministic one by under 2% — it cannot matter by construction.
The second grid's tables are now out of date for the histogram laws
(stall window changed); the grid has to be run once more.

## 7c. SECOND grid of 2026-10-02 (2,730 runs, 0 static-contact steps) — current

Findings are in the report, section "Findings". In short: the range filter
removes most of the histogram's noise sensitivity in the warehouse and maze
(pVFH → pVFH+F at σ0 = 0.10: 102 → 53 s, 127 → 2 rejections); the expected
kernel adds nothing measurable in the matched ablation (P-VFH vs pVFH+F,
every interval includes zero), apart from one weak signal on the random
layouts; nothing helps the histogram laws on the plaza, where MPC is best;
MPC and DWA are best in light traffic and worst in fast or dense traffic;
the ToF fan defeats every histogram law; with motion limits on, pVFH and
P-VFH stay clear of movers in the warehouse.

## 7b. FIRST grid of 2026-10-02 (2,410 runs) — SUPERSEDED, P-VFH rows invalid

Findings are written up in the report, section "Findings". In one line:
P-VFH's noise robustness holds in the warehouse (paired, significant, vs a
baseline differing only in the kernel) and with the line formation; it does
not generalise to the plaza or the random layouts (worse than pVFH there);
MPC is faster where traffic is light; ToF, fast/dense traffic and
load-feasible kinematics defeat every law.

## 8. Known risks and first observations

- Pilot runs (2 seeds, not evidence) suggest the picture the grid will give
  is mixed, not a clean win: P-VFH is clean on the warehouse and the line
  formation, but timed out on one random layout, and the DWA and MPC
  baselines did better than the histogram laws on the plaza. With a ToF
  fan all three histogram laws were run into from outside the field of
  view. Wait for the grid before writing any claim.
- With rotation gated, formations that previously "rotated through" a pinch
  may now stall; the anti-deadlock layer is meant to resolve that, and E9
  shows how much it does.
- The body-frame ToF fan is a different sensor from the old world-fixed one;
  E3/E6 results may change substantially.
- FGM, DWA and MPC parameters are defaults, not tuned.
- The perturbation models are simple: white localisation noise, independent
  per-step command loss.
- The headline claim of earlier drafts (deterministic VFH collapses, P-VFH
  does not) was measured on defect 1 and is not established until the grid
  is re-run.
