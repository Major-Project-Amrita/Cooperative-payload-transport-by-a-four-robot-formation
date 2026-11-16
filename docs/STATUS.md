# Status — 16 November 2026

## Review response: state of the six steps

| Step | State | Evidence |
|---|---|---|
| 1. Correct the MATLAB collision gate; regenerate grid, cross-stack, figures, tables | **Done** | Box and mover corners are in cyclic order (`matlab_ref/formation_clearance_dyn.m`). Zero noise: MATLAB and Python agree to 7e-7 m for every law. Noisy grid (2,730 runs): **221/221** cells agree, with no static contact in either stack (`results/CROSS_STACK_fixedgate.md`, T1c in `RESULTS_TABLES.md`, R1 in `EXTENDED_RESULTS.md`). |
| 2. Randomise traffic phase; report reach, contact and clearance intervals | **Done** | Every extended, physics and tuning run uses the per-seed random phase, with Wilson and bootstrap 95 % intervals (R2–R9). |
| 3. More physics replication, robot-parameter sweep, every run listed | **Done** | P1: 6 laws × 3 payload modes × 10 seeds. P2: 8 robot variants × 5 seeds. Every run is listed in R8. |
| 4. Fair tuning of the baselines, a standard planner, related work | **Done** | Equal-budget tuning for 7 laws, evaluated on held-out worlds (R6); MPPI added; `RELATED_WORK.md`. |
| 5. Separate ground-truth screening from a deployable monitor | **Done** | Audit, sensor and learned-conformal monitors (R5). ROS 2 real-sensor mode from LaserScans (`ROS2_VALIDATION.md`). Claim limits are in `PAPER_FRAMING.md`. |
| 6. Release with archive and DOI; revised title and abstract | **Archive built; DOI pending** | Run `python3 tools/make_release.py --version 1.0.0` to get `dist/*.tar.gz` and `MANIFEST.sha256`. Minting the DOI needs the authors, a LICENSE file and the Zenodo switch (`RELEASE.md`). Title and abstract: `PAPER_FRAMING.md`, `CITATION.cff`. |
| Neural networks | **Done** | PA-VFH, the proposed algorithm (learned occupancy inside VFH+; ablation in R9); DAgger policy; conformal learned monitor (`NEURAL.md`, `PA_VFH.md`). |
| ROS 2 | **Node code verified; real-DDS run of the neural laws pending** | `ROS2_VALIDATION.md`. On a Humble machine, run `bash tools/ros_humble_validation.sh`. |

## Open before submission

- Authors, affiliation, ORCIDs, and the expansion of "PA" in PA-VFH.
- LICENSE file (MIT is declared in `package.xml`).
- The real-ROS 2 run of the neural laws (`tools/ros_humble_validation.sh`).

## Open before hardware

See `ROS2_VALIDATION.md` §4:
- noise robustness above σ0 = 0.05 m;
- the physics clearance margin of PA-VFH;
- an emergency-stop watchdog;
- localisation;
- people.

---

*The status record of 3 October 2026 follows unchanged.*

# Status — 3 October 2026, 04:00

> **Nomenclature (Nov 2026):** the proposed algorithm is now **PA-VFH** (neural predictive occupancy inside the VFH+ pipeline; see [PA_VFH.md](PA_VFH.md)). The algorithm this document called "P-VFH" (code key `pvfh_prob`) is renamed **PA-VFH-K** throughout; its data and run names are unchanged.

What is finished, what was stopped part-way, and the exact commands to
complete it. Run everything from a terminal; each command is resumable (it
skips what is already done), so nothing below repeats finished work.

## 1. Finished and verified

| Item | Result | Where |
|---|---|---|
| Python port of the audited MATLAB reference (8 steering laws, mission layer, range filter, motion limits, perturbations) | 52 tests pass | `src/ros2_swarm_transport/`, `test/` |
| Zero-noise cross-check, MATLAB vs Python, all 8 laws | identical to 6 printed decimals at every step | `results/deterministic_xcheck_matlab.csv` |
| Python grid replicating MATLAB (2,730 rows / 2,420 simulations) | 0 failed, 0 static-contact steps | `results/results_headless.csv`, `results/runs/` |
| Cell-by-cell comparison with the MATLAB grid | 220 of 221 cells agree | `results/CROSS_STACK.md`, `results/cross_stack.csv` |
| MATLAB repo's report generator on the Python grid | 21 figures + log | `results/figures/`, `results/RESULTS_LOG_headless.md` |
| Defect found in the reference collision test (conservative false rejections) | reproduced, corrected mode added | documentation §5.3, `geometry.py` (`sat=`) |
| Validation scenario from your `aws_warehouse.xml` (one file for MATLAB, Python, MuJoCo) | 463 boxes + 75 cylinders, 3 movers, 4 waypoints | `worlds/swarm_world_awswh.sdf`, `dynamic_obstacles_awswh.json` |
| MuJoCo physics validation: scattered start → forming → docking → pick-up → transport; three payload modes | working, video rendered and inspected | `mujoco_validation.py` |
| Mission logic and physics split into `MissionController` and `MujocoPlant` | refactor reproduces the earlier run | `transport.py`, `mujoco_validation.py` |
| ROS 2 run of the physics validation (plant node + controller node, lockstep) | identical to the in-process run | `mujoco_plant.py`, `formation_controller.py`, `launch/mujoco_validation.launch.py` |
| Runbook, documentation, Q2 review | written | `docs/RUNBOOK.md`, `docs/DOCUMENTATION.md`, `docs/Q2_ASSESSMENT.md` |

## 2. Batches: all complete (3 October, 03:57)

| Job | Result |
|---|---|
| Python grid, corrected gate | 2,730 rows, 0 static-contact steps, 2,621 reached |
| MATLAB on the validation warehouse | 48 of 48 missions |
| MuJoCo batch | 45 of 45 missions; **7 of 10 showcase videos** |
| Python on the validation warehouse | 200 of 200 missions |
| Comparisons, figures, tables | regenerated from the final data |

## 3. Left to run

The three missing videos (the command skips the seven that exist):

```bash
cd ~/ros2_ws && python3 tools/run_mujoco_batch.py --videos
```

To regenerate everything derived, at any time:

```bash
cd ~/ros2_ws && python3 tools/cross_stack.py && python3 tools/cross_stack.py --py results/results_headless_exactgate.csv --tag exactgate && python3 tools/make_doc_figures.py && python3 tools/make_tables.py
```

(`make_doc_figures.py` takes about 10 minutes: two of its figures re-run 24
warehouse missions.)

## 4. Sanity checks on the final numbers

```bash
cd ~/ros2_ws && python3 -c "
import pandas as pd
for f in ['results/results_headless.csv','results/results_headless_exactgate.csv']:
    d=pd.read_csv(f); print(f, len(d), 'rows, static-contact steps:', int(d.coll_static.sum()), ', reached:', int(d.reached.sum()))"
```

Both must show `2730 rows` and `static-contact steps: 0`.

```bash
head -12 ~/ros2_ws/results/CROSS_STACK.md
```

Reference-gate comparison: 220 of 221 cells agree. Exact-gate comparison
(`results/CROSS_STACK_exactgate.md`): 196 of 221, the 25 differences being the
gate defect itself (documentation §8).

## 5. Known open points

- Mover timing is fixed per world, so mover-contact statistics hold for one
  traffic phase (documentation §9.3, `results/warehouse_mover_timing.csv`).
  Randomising the phase and re-running is the top item in
  `docs/Q2_ASSESSMENT.md`.
- The MATLAB reference still has the gate defect; the fix is two corner
  orders in `formation_clearance_dyn.m`, then a re-run of the MATLAB grid.
- `ros2 launch … gui:=true` needs one `colcon build` (the viewer option was
  added after the last build). The windowed two-node run without building,
  `bash tools/ros_lockstep_test.sh mpc on_top true`, is tested; the fix for
  the crash on closing the window is in but not re-tested.
- No RViz session was run. `launch/transport.launch.py` (Gazebo),
  `mover_node.py` and `swarm_logger.py` are unchanged from the bundle and
  unused.
