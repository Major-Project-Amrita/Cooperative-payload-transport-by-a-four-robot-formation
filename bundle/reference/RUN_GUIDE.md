# Run guide — from a clean checkout to the finished report, then ROS 2

Everything below is run from the repository root (`swarm_matlab/`) unless a
step says otherwise. Do the parts in order. Part A–D is the MATLAB side;
Part E starts the ROS 2 side in a new directory.

**Status you are starting from:** the corrected MATLAB code has been run and
smoke-tested on this machine (all eight laws, all worlds, every experiment
type; no crashes, no static contact). A mission takes about 1.5–3.5 s, so
the full grid is roughly **30 minutes with 6 processes** (about 2 hours with
one). What has not been done is the full grid itself.
The report PDF currently shows the old pre-fix data under a "LEGACY DATA"
banner.

---

## Part A — check that the code runs (10–30 minutes)

### A1. Prerequisites

- MATLAB (you have R2025a; no toolbox is needed). `matlab` must be on the PATH.
- Python 3 with `numpy pandas scipy matplotlib`:
  ```bash
  python3 -c "import numpy, pandas, scipy, matplotlib; print('python ok')"
  ```
- LaTeX (`pdflatex`). On this machine TinyTeX is at `~/.TinyTeX`:
  ```bash
  export PATH="$HOME/.TinyTeX/bin/x86_64-linux:$PATH"
  ```

### A2. One mission, watched

Open MATLAB in `matlab/` and run one mission with the proposal:

```matlab
cd matlab
CFG_OVERRIDE = struct('algo','pvfh_prob','seed',1);
Simulation8_dynamic
```

What you should see: the four robots form up, the formation drives to the
star, and a run summary prints. In the summary, **`static` collision steps
must be 0** and `reached goal` should be 1. If MATLAB stops with an error,
that is the first thing to fix (send me the message).

Try the other laws the same way: `'apf'`, `'vfh'`, `'fgm'`, `'dwa'`, `'mpc'`,
`'pvfh'`, `'pvfh_filt'`. They do not all have to reach the goal — that is what the
experiments measure — but none may crash.

### A3. Smoke test of the whole pipeline

```bash
cd matlab && matlab -batch "run_paper_experiments('smoke')"
```

This runs one seed of one cell of every experiment (about 20 runs).
Check three things:

1. No line says `RUN FAILED`.
2. Each run prints `reached=… reject=… coll=… minclr=… (NN s wall)`. Note the
   wall times: **(average seconds per run) × 2,730 ÷ (number of parallel
   processes)** is your estimate for the full grid.
3. Then render the smoke results (still from the repo root):
   ```bash
   python3 matlab/make_paper_figures.py
   ```
   It must finish with three lines `figures -> … tables -> … log -> …`.

### A4. Clean up the smoke rows

```bash
rm -rf matlab/logs/paper/parts matlab/logs/paper/runs
```

(Otherwise the seed-7 smoke rows would be merged into the real table.)

---

## Part B — the Monte Carlo grid (hours; resumable)

### B1. See what will run

```bash
cd matlab && matlab -batch "run_paper_experiments('list')" | tail -3
```

Prints the number of jobs and of unique configurations (about 2,730).

### B2. Launch

```bash
matlab/run_grid_parallel.sh 6
```

- `6` is the number of MATLAB processes. This machine has 12 cores; each
  process needs roughly 1.5 GB RAM. Expect about 30 minutes.
- Half-size study (5 seeds per cell, 10 in E2):
  ```bash
  matlab/run_grid_parallel.sh 6 quick
  ```
- One experiment only (`e1` … `e14`):
  ```bash
  matlab/run_grid_parallel.sh 6 e2
  ```

A sensible order if time is short: `e2` first (the central claim), then `e1`,
`e7`, `e13`, then the rest.

### B3. Watch progress

```bash
tail -n 3 matlab/logs/paper/shardlogs/*.log
```
```bash
cat matlab/logs/paper/parts/*.csv | wc -l
```

### B4. If it stops (reboot, crash, Ctrl-C)

Run the same launch command again **with the same number of processes**.
Every finished run is already saved in `matlab/logs/paper/parts/`; they are
skipped. If you change the number of processes, that is also fine — finished
configurations are still skipped.

To look for failed runs afterwards:

```bash
grep -h "RUN FAILED" matlab/logs/paper/shardlogs/*.log | sort | uniq -c
```

### B5. What you get

- `matlab/logs/paper/results_all.csv` — one row per run (the results table).
- `matlab/logs/paper/runs/*.csv` — one per-step log per run (the trajectory
  and time-series figures need these; several hundred MB, git-ignored).

---

## Part C — results into the documentation

### C1. Figures, tables, results log (one command)

```bash
python3 matlab/make_paper_figures.py
```

This rewrites, from `results_all.csv`:

- every evaluation figure in `docs/simulation_report/figures/`,
- every results table in `docs/simulation_report/generated/` (the report
  `\input`s them — you never copy a number by hand),
- the banner at the top of the experiments chapter (it will no longer say
  LEGACY),
- `docs/RESULTS_LOG.md` — all cells of all experiments, readable on its own.

### C2. Sanity checks before you believe anything

1. Open `docs/RESULTS_LOG.md`. In every table the **static contact** column
   must be 0. Anything else means the safety gate has a bug — stop and tell me.
2. The E6 `lidar-288` cells are the E2 simulations of seeds 1–10 at the same
   σ0 (run once, reported twice); E2 has 20 seeds, so its cell means may
   differ slightly, but the two must tell the same story.
3. E10 at noise 0, E12 at drop 0 and E8 at speed ×1 must equal the E1 rows of
   the same law.

### C3. Compile the report

```bash
cd docs/simulation_report && pdflatex main && pdflatex main
```

Open `docs/simulation_report/main.pdf`. The experiments chapter now shows
your results.

### C4. What still needs a human: the conclusions

The tables and figures update themselves; the **interpretation does not**.
The chapter deliberately contains no result sentences. After the run, read:

- Table "Paired-seed tests" (`tab:stats`): P-VFH minus pVFH+F, pVFH and VFH,
  per noise level, with confidence intervals and p-values.
- E2 figure and table: does the filter (pVFH → pVFH+F) cut rejections and
  time at high noise, and does the kernel (pVFH+F → P-VFH) add anything?
- E7 and E13: does the same ordering hold in other worlds?

Then either send me `docs/RESULTS_LOG.md` (I will write the findings
paragraphs, the abstract sentence and the conclusion from the actual
numbers), or write them yourself in
`docs/simulation_report/sections/08_experiments.tex` under each "How to
read" paragraph. Section 07 ("Towards a publishable claim") lists the three
possible outcomes and what each means for the paper.

### C5. Refresh the archives

```bash
release/build_release.sh
```

Rebuilds `release/matlab_reference.zip`,
`release/ros2_swarm_transport_bundle.zip` (now carrying your results as
`reference/results_all.csv`) and `docs/simulation_report.zip`.

### C6. Commit (optional)

```bash
git add -A && git commit -m "Corrected Monte Carlo grid E1-E13 and regenerated report"
```

---

## Part D — if something goes wrong

| Symptom | What to do |
|---|---|
| MATLAB error in the smoke test | Copy the error text and the line it names; that file needs a fix before the grid. |
| `merge: … different header` | Old checkpoint files from an earlier code version: `rm -rf matlab/logs/paper/parts` and relaunch. |
| A figure says "not in the results table yet" | That experiment has no rows; run it (`run_grid_parallel.sh 6 e7`, etc.) and rerun C1. |
| A trajectory figure says "per-step logs" | `matlab/logs/paper/runs/` is missing; the runs must be redone with logs. |
| `pdflatex` not found | `export PATH="$HOME/.TinyTeX/bin/x86_64-linux:$PATH"` |
| Many runs time out at 300 s | Expected for failing cells (e.g. APF); they count as "not reached". |

---

## Part E — starting the ROS 2 project in a new directory

Do this **after** Part C5, so the bundle contains the corrected MATLAB
results to replicate. The Python/ROS 2 package inside the bundle still has
the pre-audit behaviour; bringing it in line is the first job of the new
session.

### E1. Create the workspace and unpack

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
```
```bash
unzip -o "/home/siddx/Desktop/Major Project/Project/swarm_matlab/release/ros2_swarm_transport_bundle.zip" -d . && mv ros2_bundle bundle
```

You now have:

```
~/ros2_ws/bundle/
  AGENT_PROMPT.md            the prompt to paste (same as docs/ROS2_AGENT_PROMPT.md)
  ROS2_REPLICATION.md        the specification; section 9 = what to port
  ROS2_START_HERE.md
  ros2_swarm_transport/      the Python / ROS 2 package (pre-audit code)
  reference/
    matlab/                  the corrected MATLAB reference (the spec)
    AUDIT_CHANGELOG.md  RUN_GUIDE.md  RESULTS_LOG.md  results_all.csv
    *.sdf  *.json            all worlds and contracts
```

### E2. Python-only check (no ROS needed)

```bash
cd ~/ros2_ws/bundle/ros2_swarm_transport && python3 -m pip install --user pytest numpy scipy && python3 -m pytest test -q
```

The existing tests should pass; they test the *old* behaviour and will be
extended by the port.

### E3. Install ROS 2 (once)

Ubuntu 22.04 pairs with **ROS 2 Humble**. If `printenv ROS_DISTRO` prints
nothing, install it following the official instructions
(docs.ros.org → Humble → Installation → Ubuntu deb packages), then:

```bash
sudo apt install ros-humble-ros-gz python3-colcon-common-extensions python3-rosdep
```
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

`sudo` commands are yours to run; the agent session will stop and ask rather
than install system packages silently.

### E4. Open the new window and paste the prompt

1. Open a **new** Claude Code window with `~/ros2_ws` as its folder.
2. Open `~/ros2_ws/bundle/AGENT_PROMPT.md`, copy everything below the `---`
   line, and replace `<REPO>` with
   `/home/siddx/Desktop/Major Project/Project/swarm_matlab`.
3. Paste it as the first message.

The prompt makes the session, in order: (A) port the audit fixes into the
Python package with tests; (B) run the same E1–E14 grid on the Python stack;
(C) build the ROS 2 package, complete robot spawning, and run Gazebo missions;
(D) compare both stacks cell by cell against your MATLAB results; (E) add the
"Independent replication" section to the LaTeX report and recompile.

### E5. What to check when it finishes

- All tests pass, including the new ones.
- `results/cross_stack.csv` exists and the session named the disagreeing
  cells with causes, rather than reporting blanket agreement.
- `docs/simulation_report/sections/11_replication.tex` exists and
  `main.pdf` compiles without undefined references.
- Back in this repo, rebuild the archives once more:
  ```bash
  release/build_release.sh
  ```

---

## What is and is not covered

Covered in simulation: external baselines (FGM, DWA, lattice MPC), three
designed worlds plus ten random layouts, noise/beam/sensor/shape sweeps,
traffic speed and density, localisation error, command loss, anti-deadlock
ablation, paired statistics, two proved properties of the kernel.

Not covered: hardware; a navigation-stack TEB/MPPI; a sensor-based safety
supervisor (the gate uses true geometry); drifting localisation error and
bursty communication loss; the cross-implementation replication (Part E).
