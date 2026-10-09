# Runbook — from a clean terminal to every result in the documentation

Everything runs from `~/ros2_ws`. Each step says what it does, how long it
took on this machine (12 cores), what it writes, and what a correct result
looks like. Steps 1–3 take ten minutes and prove the stack works; steps 4–8
regenerate every number and figure in `docs/DOCUMENTATION.md`.

Nothing here needs Gazebo. ROS 2 is needed only for step 9.

| Step | What | Time | Needs |
|---|---|---|---|
| 0 | Check the tools | 1 min | Python 3.10, MATLAB, ffmpeg |
| 1 | Unit tests | 1.5 min | Python |
| 2 | One mission, each stack | 2 min | Python (+ MATLAB) |
| 3 | Zero-noise MATLAB ↔ Python cross-check | 1 min | MATLAB + Python |
| 4 | The Monte Carlo grid on the Python stack | ~1.5 h | Python |
| 5 | Cell-by-cell comparison with the MATLAB grid | 1 min | Python |
| 6 | The validation warehouse (your `aws_warehouse.xml`) in MATLAB | ~40 min | MATLAB |
| 7 | Physics validation in MuJoCo | ~45 min | Python + MuJoCo |
| 8 | Figures and tables | 3 min | Python |
| 9 | The same physics run through ROS 2 (plant node + controller node) | 3 min | ROS 2 Humble |

---

## 0. Check the tools

```bash
cd ~/ros2_ws
```

```bash
python3 -c "import numpy, scipy, pandas, matplotlib, mujoco; print('python ok, mujoco', mujoco.__version__)"
```

```bash
matlab -batch "disp(version)"
```

```bash
ffmpeg -version | head -1
```

Expected: `python ok, mujoco 3.x`, a MATLAB version string (R2025a here), an
ffmpeg version. If MuJoCo is missing: `python3 -m pip install --user mujoco`.
MATLAB is needed only for steps 3 and 6; without it, skip them — the MATLAB
outputs they produce are already in `matlab_ref/logs/`.

---

## 1. Unit tests (1.5 min)

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m pytest test -q
```

Expected: `52 passed`. What they cover: geometry, the motion contract, every
steering law, the range filter, the kernel property (Proposition 2), the
safety gate against a brute-force reference, a full mission of each of the
eight laws with zero static contact, and the experiment manifest against the
MATLAB results table.

---

## 2. One mission on each stack

### 2a. Python headless (2 s)

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m ros2_swarm_transport.headless_runner --algo pvfh_prob --seed 1 --output ../../results/demo/pvfh_prob.csv
```

Expected last line (numbers will match exactly, the run is seeded):
`reached=True t=48.7 s path=25.89 m … coll=0 (static 0, mover 0) reject=0 …`

Options worth knowing: `--algo apf|vfh|fgm|dwa|mpc|pvfh|pvfh_filt|pvfh_prob`,
`--formation square|diamond|triangle|line` (use `--grid-spacing 0.6` for the
last three), `--sigma0 0.2` (sensor noise), `--sense tof --n-beams 16`,
`--kin` (load-carrying motion limits), `--no-adl` (anti-deadlock off),
`--mover-scale 2`, `--loc-sigma 0.15`, `--comm-drop 0.3`,
`--world … --contract …` (any world in `worlds/`),
`--gate matlab` (replicate the reference gate, see documentation §5.3).

### 2b. The validation warehouse, headless (15 s)

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m ros2_swarm_transport.headless_runner --world worlds/swarm_world_awswh.sdf --contract worlds/dynamic_obstacles_awswh.json --algo mpc --seed 1 --output ../../results/demo/wh_mpc.csv
```

Expected: `reached=True t=50.7 s … coll=0 … reject=0`.

### 2c. MATLAB, watched (optional)

```bash
cd ~/ros2_ws/matlab_ref && matlab -r "CFG_OVERRIDE = struct('algo','pvfh_prob','seed',1); Simulation8_dynamic"
```

A figure window shows the four robots forming up and the formation driving
to the goal; the run summary must say `collision steps : 0 (static 0, …)`.

### 2d. MuJoCo, one mission with a video (1–2 min)

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m ros2_swarm_transport.mujoco_validation --algo mpc --payload on_top --video
```

Expected: one line ending in
`physics[rms=…cm … contact s/m=0/0 … pin=0.7mm …]`, and the files
`results/mujoco/mpc_square_on_top_s1.{csv,json,mp4}`. Open the video:

```bash
xdg-open ~/ros2_ws/results/mujoco/mpc_square_on_top_s1.mp4
```

To watch it live instead, in the interactive MuJoCo window at real time
(camera follows the formation; drag to orbit, scroll to zoom):

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m ros2_swarm_transport.mujoco_validation --algo mpc --payload on_top --gui --output /tmp/gui_demo
```

`--payload virtual|on_top|behind`, `--formation …`, `--no-kin` (motion limits
off), `--no-hold` (open-loop frame), `--world worlds/swarm_world_dynamic.sdf
--contract worlds/dynamic_obstacles.json` (the MATLAB study's own warehouse).
If the machine has no GPU/EGL: `MUJOCO_GL=osmesa python3 -m …`.

---

## 3. Zero-noise cross-check, MATLAB ↔ Python (1 min)

With `sigma0 = 0` no random number matters, so the two implementations must
follow the same trajectory.

```bash
cd ~/ros2_ws/matlab_ref && matlab -batch "for a = {'apf','vfh','fgm','dwa','mpc','pvfh','pvfh_filt','pvfh_prob'}, xcheck(a{1}, 0, 1, ['x_' a{1} '_s0']); end"
```

```bash
cd ~/ros2_ws && python3 tools/deterministic_xcheck.py matlab
```

Expected: for all eight laws `max_centroid_diff_m` below `1e-6`
(`first_gt_1mm = never`), equal step counts and equal rejection counts.

```bash
cd ~/ros2_ws && python3 tools/deterministic_xcheck.py exact
```

With the corrected gate, `vfh / fgm / dwa / mpc` are unchanged and
`apf / pvfh / pvfh_filt / pvfh_prob` diverge at the first step where the
reference gate refuses a collision-free pose (documentation §5.3).

---

## 4. The Monte Carlo grid on the Python stack

The same 2,730 jobs (2,420 unique simulations) as the MATLAB grid.

```bash
cd ~/ros2_ws && python3 tools/run_headless_grid.py --list | tail -1
```

Expected: `2730 jobs, 2420 unique configurations`.

Smoke run first (one seed per experiment, 1 min):

```bash
cd ~/ros2_ws && python3 tools/run_headless_grid.py --mode smoke --jobs 10
```

Full grid, replicating the reference gate (≈ 70 min with 10 processes):

```bash
cd ~/ros2_ws && python3 tools/run_headless_grid.py --jobs 10
```

The same grid with the corrected gate (≈ 60 min):

```bash
cd ~/ros2_ws && python3 tools/run_headless_grid.py --jobs 10 --gate exact --tag exactgate --no-runs
```

Both are resumable: run the same command again after an interruption and
finished rows are skipped. Outputs: `results/results_headless.csv`,
`results/runs/*.csv` (per-step logs, ≈ 250 MB),
`results/results_headless_exactgate.csv`.
Check: the last line says `0 failed`, and

```bash
cd ~/ros2_ws && python3 -c "import pandas as pd; d=pd.read_csv('results/results_headless.csv'); print(len(d), 'rows; static-contact steps:', int(d.coll_static.sum()))"
```

must print `2730 rows; static-contact steps: 0`.

One experiment only: add `--only E2` (E1, E2, E3, E5 … E14).

---

## 5. Compare with the MATLAB grid (1 min)

```bash
cd ~/ros2_ws && python3 tools/cross_stack.py
```

```bash
cd ~/ros2_ws && python3 tools/cross_stack.py --py results/results_headless_exactgate.csv --tag exactgate
```

Outputs: `results/cross_stack.csv` (every cell, both stacks, intervals),
`results/CROSS_STACK.md` (summary + the list of disagreeing cells),
`results/generated/tab_cross.tex` (LaTeX table). Read the summary:

```bash
cat ~/ros2_ws/results/CROSS_STACK.md | head -30
```

The MATLAB repository's own report generator also reads the Python results:

```bash
cd ~/ros2_ws && python3 bundle/reference/matlab/make_paper_figures.py --csv results/results_headless.csv --runs results/runs --outdir results/figures --gendir results/generated --log results/RESULTS_LOG_headless.md
```

---

## 6. The validation warehouse in MATLAB (~40 min, two MATLAB processes)

The scenario files are generated from your MuJoCo scene:

```bash
cd ~/ros2_ws && python3 tools/make_validation_world.py && cp src/ros2_swarm_transport/worlds/swarm_world_awswh.sdf src/ros2_swarm_transport/worlds/dynamic_obstacles_awswh.json matlab_ref/
```

Expected: `wrote swarm_world_awswh.sdf (463 boxes, 75 cylinders) and
dynamic_obstacles_awswh.json`. Then, in two terminals:

```bash
cd ~/ros2_ws/matlab_ref && matlab -batch "wh_batch(1)"
```

```bash
cd ~/ros2_ws/matlab_ref && matlab -batch "wh_batch(2)"
```

Each prints one `RESULT wh_<law>_… reached=… t=… coll_s=0 …` line per
mission (8 zero-noise missions + 8 laws × 5 seeds). Finished missions are
skipped on restart. A single mission, watched:

```bash
cd ~/ros2_ws/matlab_ref && matlab -r "run_wh('mpc', 0.02, 1, 'wh_demo')"
```

---

## 7. Physics validation in MuJoCo (~45 min, one core)

```bash
cd ~/ros2_ws && python3 tools/run_mujoco_batch.py
```

45 missions (laws × payload modes, formation shapes, motion limits on/off,
seeds, the MATLAB study's own warehouse, formation feedback off). Resumable.
Each prints one summary line; outputs are
`results/mujoco/<run>.{csv,json}`. Add the showcase videos (10 MP4s,
+10 min):

```bash
cd ~/ros2_ws && python3 tools/run_mujoco_batch.py --videos
```

One group only: `--only V1` … `V6`.

---

## 8. Figures and tables (3 min)

```bash
cd ~/ros2_ws && python3 tools/make_doc_figures.py
```

Writes every figure of the documentation to `docs/figures/` and the tables
`results/mujoco_summary.csv`, `results/warehouse_xcheck.csv`. Figures whose
inputs are missing are skipped with a note, so this can be run after any
step. One figure only: `python3 tools/make_doc_figures.py e2`.

```bash
cd ~/ros2_ws && python3 tools/make_tables.py
```

Regenerates `docs/RESULTS_TABLES.md` (the numbers quoted in the
documentation) from the result files.

---

## 9. The same physics run through ROS 2

Two nodes: `mujoco_plant` (the physics world) and `formation_controller`
(the mission). They exchange one message per control tick in lockstep, so
the run is reproducible and equals the in-process run of step 7.

Without building the package (20 s); runs both nodes, then the in-process
run, and prints the two side by side:

```bash
cd ~/ros2_ws && bash tools/ros_lockstep_test.sh mpc on_top
```

Expected: `mission complete`, then a table whose two columns are equal
(`total_time 57.8`, `formation_rms 0.11268`, contacts 0).

With the MuJoCo window (third argument `true`):

```bash
cd ~/ros2_ws && bash tools/ros_lockstep_test.sh mpc on_top true
```

As a normal ROS 2 package:

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select ros2_swarm_transport && source install/setup.bash
```

```bash
ros2 launch ros2_swarm_transport mujoco_validation.launch.py algo:=mpc payload:=on_top output_dir:=$HOME/ros2_ws/results/mujoco_ros
```

Expected (about 1 min; the same numbers as the in-process run):

```
[formation_controller]: formed and docked at t=7.7 s -> transport
launch_test: reached=True t=57.8s … physics[rms=11.3cm … contact s/m=0/0 … pin=0.7mm …]
[formation_controller]: mission complete; wrote …/results/mujoco_ros/….csv/.json
[mujoco_plant]: mission finished at t=58.8 s; plant stopped
```

The launch then shuts down. A last line `[ERROR] [mujoco_plant-1]: process
has died … exit code -2` is the launcher interrupting a node that was already
exiting; it is harmless.

Launch arguments: `algo`, `formation`, `grid_spacing`, `payload`
(`virtual|on_top|behind`), `seed`, `kin`, `hold_err`, `world_sdf`,
`contract`, `gui:=true` (MuJoCo window), `video:=/path/out.mp4`,
`output_dir`, `run_name`. While it runs,
in a second terminal:

```bash
source /opt/ros/humble/setup.bash && ros2 topic list
```

shows `/robot1..4/odom`, `/robot1..4/cmd_vel`, `/payload/pose`,
`/dynamic_obstacles`, `/swarm/phase`, `/swarm/state`, `/swarm/command`,
`/clock`. `ros2 topic echo /swarm/phase` goes 1 (forming) → 2 (transport) →
3 (done). The markers and odometry can be viewed in RViz (fixed frame `map`).

The controller alone against an ideal robot, no simulator (free-running mode
on `/robotN/odom`, the mode to use with real robots):

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && python3 tools/ros_node_selftest.py
```

Expected: `mission complete`, `phase 3 … slot err max 0.06`.

Not done: `launch/transport.launch.py` (the original Gazebo launch file) is
unchanged and not usable; `mover_node` and `swarm_logger` from the bundle
were not exercised (the plant node publishes the movers and the controller
writes the log).

---

## If something goes wrong

| Symptom | Cause / fix |
|---|---|
| `ModuleNotFoundError: ros2_swarm_transport` | Run from `src/ros2_swarm_transport`, or use the `tools/` scripts (they add the path themselves). |
| MuJoCo: `GLFWError` / `EGL` errors | No offscreen GPU context. Use `MUJOCO_GL=osmesa`, or drop `--video` (physics needs no rendering). |
| `colcon build`: `Duplicate package names not supported` | The bundle holds a second copy of the package: `touch ~/ros2_ws/bundle/COLCON_IGNORE` (already done here). |
| `ros2 launch …`: `mujoco_validation.launch.py was not found` | The package was built before this launch file existed: rebuild (step 9) and `source install/setup.bash` again. |
| After editing Python files, `ros2 launch` runs the old code | Rebuild: the installed copy is separate from `src/`. `tools/ros_lockstep_test.sh` always runs the source tree. |
| `matlab: command not found` | Skip steps 3 and 6; their outputs are already in `matlab_ref/logs/`. |
| Grid stops (reboot, Ctrl-C) | Re-run the same command; it resumes. |
| A grid run prints `RUN FAILED` | The row is written with `reached = 0` and NaNs; send the message — none occurred here. |
| `UserWarning: Unable to import Axes3D` | Harmless (two matplotlib installs). |
| Step 3 shows a law diverging with `matlab` gate | The MATLAB copy and the Python port are out of sync; check that `matlab_ref/` is the unmodified `bundle/reference/matlab/`. |
| Many grid runs take 300 s of simulated time | Expected for failing cells (line formation without the anti-deadlock layer, FGM in the maze); they count as not reached. |

## Where things are

```
~/ros2_ws/
  docs/STATUS.md             what is finished / what to run next
  docs/DOCUMENTATION.md      concepts, methods, results, figures explained
  docs/RUNBOOK.md            this file
  docs/Q2_ASSESSMENT.md      publication-readiness review
  docs/RESULTS_TABLES.md     generated tables
  docs/figures/              generated figures
  src/ros2_swarm_transport/  the Python package
      ros2_swarm_transport/  geometry.py vfh.py transport.py headless_runner.py
                             mujoco_validation.py mjcf_world.py
                             formation_controller.py mujoco_plant.py ros_common.py …
      worlds/                all SDF worlds, contracts, aws_warehouse.xml
      test/                  52 tests
  tools/                     grid, comparison, cross-check, batch, figures
  matlab_ref/                working copy of the corrected MATLAB reference
                             (+ xcheck.m, run_wh.m, wh_batch.m), logs/
  bundle/                    the original bundle (reference, MATLAB results)
  results/                   everything the steps above write
  backup_preaudit/           the Python modules as they were before the port
```
