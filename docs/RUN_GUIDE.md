# Run guide: every result, from a clean machine

This guide regenerates every number, table, figure and video in the
project, and runs the ROS 2 package. Times were measured on a 4-core cloud
machine. Every long step is **resumable**: re-running it skips finished
runs.

All commands run from the repository root unless a step says otherwise.

```bash
cd ~/Cooperative-payload-transport-by-a-four-robot-formation
```

| Part | What | Time | Needs |
|---|---|---|---|
| [0](#0-install) | Install | 10 min | Ubuntu 22.04/24.04, Python ≥ 3.10 |
| [1](#1-smoke-test-5-min) | Smoke test (unit tests + one mission) | 5 min | Python |
| [2](#2-the-reference-study-matlab-and-python) | Reference study: MATLAB grid, Python grid, cross-check | 1.5 h Python; 10 h MATLAB/Octave | Python (+ MATLAB or Octave) |
| [3](#3-extended-benchmark-e15e19) | Extended benchmark E15–E19 (layouts, team size, monitors, tuning, PA-VFH ablation) | ~6 h | Python |
| [4](#4-neural-networks-train-or-use-the-shipped-weights) | Neural networks (optional retraining) | 2–3 h | PyTorch |
| [5](#5-physics-mujoco) | Physics in MuJoCo (P1–P4) | ~5 h | MuJoCo |
| [6](#6-ros-2) | ROS 2: lockstep, deployment mode, watchdog | 30 min | ROS 2 Humble, or Docker |
| [7](#7-reports-figures-tables-signals) | Reports, figures, tables, signal plots | 20 min | Python |
| [8](#8-demo-video) | Demo video | 10 min | ffmpeg |
| [9](#9-release-archive-and-doi) | Release archive and DOI | 10 min | Python |

You don't have to recompute anything to read the results: every output is
committed under `results/` and `docs/`. Recompute only what you want to
verify.

---

## 0. Install

```bash
sudo apt-get install -y python3-pip ffmpeg octave
```

```bash
python3 -m pip install --user "numpy<3" scipy pandas matplotlib mujoco pytest
```

The MATLAB reference needs MATLAB (R2023a+) or GNU Octave 8+. The project
was validated with Octave 8.4, which ships a port of SciPy's assignment
solver in `matlab_ref/assign_slots.m`. MATLAB uses its own `matchpairs`.

Neural-network retraining (part 4 only) also needs PyTorch:

```bash
python3 -m pip install --user torch
```

Running the steering laws needs only numpy: the trained weights ship as
`.npz` files in `src/ros2_swarm_transport/models/`.

Check:

```bash
python3 -c "import numpy, scipy, pandas, matplotlib, mujoco; print('ok, mujoco', mujoco.__version__)"
```

---

## 1. Smoke test (5 min)

```bash
cd src/ros2_swarm_transport && python3 -m pytest test -q && cd ../..
```

Expected: **77 passed**. With real ROS 2 sourced, 3 tests that use the
in-memory ROS stand-in are skipped instead; part 6 covers them over DDS.
The tests cover:
- geometry and the collision gate;
- every steering law;
- the range filter;
- the neural components;
- the sensing pipeline;
- the ROS 2 nodes and the watchdog.

One mission of the proposed algorithm, PA-VFH, with a random traffic phase:

```bash
PYTHONPATH=src/ros2_swarm_transport python3 -c "
from ros2_swarm_transport.headless_runner import run_mission
W='src/ros2_swarm_transport/worlds/'
_, S = run_mission(W+'swarm_world_dynamic.sdf', W+'dynamic_obstacles.json', algo='pavfh', seed=1, phase=True, verbose=False)
print({k: S[k] for k in ('reached_goal','total_time','collisions','n_reject','comp_ms_mean')})"
```

Expected: `reached_goal: True`, about 60–90 s of mission time, about 1 ms
of compute per step.

Algorithm keys used throughout:

| Key | Name in the documents |
|---|---|
| `apf` `vfh` `fgm` `dwa` `mpc` `mppi` | classical baselines |
| `pvfh` `pvfh_filt` | histogram ablation ladder (pVFH, pVFH+F) |
| `pvfh_prob` | **PA-VFH-K**, formerly "P-VFH": analytic kernel with contract prediction |
| `nn` | DAgger imitation policy (neural baseline) |
| `pavfh_nonn` | PA-VFH (no NN): the same pipeline with the analytic kernel |
| `pavfh` | **PA-VFH**, the proposed algorithm: VFH+ with a learned occupancy network |

---

## 2. The reference study (MATLAB and Python)

### 2a. Python grid (2,730 runs, ~1.5 h)

```bash
python3 tools/run_headless_grid.py --gate exact --tag exactgate --jobs 4
```

```bash
python3 tools/run_headless_grid.py --gate exact --tag phase --phase --no-runs --jobs 4
```

These write `results/results_headless_exactgate.csv` (corrected gate) and
`results/results_headless_phase.csv` (the same grid with random traffic
phase, review step 2).

### 2b. MATLAB grid with the corrected collision gate (~10 h in Octave on 4 cores)

Run the four shards in parallel, then merge:

```bash
cd matlab_ref && for k in 1 2 3 4; do octave-cli --no-gui --quiet --eval "run_paper_experiments('full', $k, 4)" > logs/paper/shardlogs/shard_${k}of4.log 2>&1 & done; wait; octave-cli --no-gui --quiet --eval "run_paper_experiments('merge')"; cd ..
```

```bash
mkdir -p results/matlab_fixedgate && cp matlab_ref/logs/paper/results_all.csv results/matlab_fixedgate/
```

With MATLAB, replace `octave-cli --no-gui --quiet --eval` with
`matlab -batch`.

### 2c. Cross-checks

Zero-noise, step by step. Expected: maximum centroid difference below 1 µm
for every law.

```bash
cd matlab_ref && octave-cli --no-gui --quiet --eval "run_xcheck" && cd .. && python3 tools/deterministic_xcheck.py exact matlab_ref/logs/xcheck_fixedgate fixedgate
```

Noisy grid, cell by cell. Expected: **221/221 cells agree**.

```bash
python3 tools/cross_stack.py --matlab results/matlab_fixedgate/results_all.csv --py results/results_headless_exactgate.csv --tag fixedgate
```

Writes `results/CROSS_STACK_fixedgate.md`.

---

## 3. Extended benchmark (E15–E19)

Every run uses the random traffic phase and the corrected gate. Failed runs
are recorded, not dropped.

| Experiment | Question | Runs | Time (4 cores) |
|---|---|---|---|
| E15 | 8 warehouse layouts (30×20 m to 80×50 m) × 10 laws | 800 | ~2 h |
| E16 | Team sizes 3/4/6/9 and larger payloads | 900 | ~2 h |
| E17 | Ground-truth gate vs sensor monitor vs learned monitor | 816 | ~1.5 h |
| E18 | Default vs equal-budget-tuned parameters, held-out worlds | 700 | ~1 h |
| E19 | **PA-VFH ablation: does the network help?** | 450 | ~20 min |

Generate the warehouse layouts (already committed):

```bash
python3 tools/make_warehouse_layouts.py
```

Equal-budget tuning (needed before E18): 24 configurations × 7 missions per
law.

```bash
python3 tools/tune_baselines.py --jobs 4
```

```bash
python3 tools/run_extended.py --exp E19,E15,E16,E17,E18 --jobs 4
```

Output: `results/extended/E1x.csv`, one row per run.

---

## 4. Neural networks (train, or use the shipped weights)

The weights in `src/ros2_swarm_transport/models/` are what the results
used. Retraining reproduces them up to PyTorch nondeterminism.

| Model | What it is | Command | Time |
|---|---|---|---|
| `pavfh_occupancy` | **Inside PA-VFH**: a circular CNN that predicts per-sector danger over the next 0.5 s from 3 filtered scans | `python3 tools/train_neural.py occupancy` | ~40 min |
| `policy_dagger` | Team-size-agnostic steering policy, imitating MPC with DAgger | `python3 tools/train_neural.py policy` | ~1 h |
| `monitor_conformal` | Learned safety monitor with a split-conformal margin | `python3 tools/train_neural.py monitor` | ~30 min |

Training uses seeds ≥ 101 and only the training worlds (rand01–05, dynamic,
plaza). Evaluation uses seeds 1–10 and held-out worlds. Metadata and the
training log are in the `.json` next to each `.npz`.

---

## 5. Physics (MuJoCo)

One mission with a video (1–2 min):

```bash
PYTHONPATH=src/ros2_swarm_transport python3 -m ros2_swarm_transport.mujoco_validation --algo pavfh --payload on_top --phase --video --output results/mujoco_demo
```

Batches:

| Batch | Content | Runs |
|---|---|---|
| P1 | 6 laws × 3 payload modes (virtual, carried on top, towed behind) × 10 seeds | 180 |
| P2 | Robot-parameter sweep: mass, wheel speed, friction, payload, motor force; 3 laws × 8 variants × 5 seeds | 120 |
| P3 | P1 with the **0.10 m physical safety margin** in the gate, the same for every law | 180 |
| P4 | P2 with the 0.10 m margin, all six laws | 240 |

```bash
python3 tools/run_mujoco_batch.py --only P1,P2,P3,P4 --jobs 4
```

Output: `results/mujoco_phase/<run>.json` plus a per-step `.csv`. Every
run, failed ones included, is listed in `docs/EXTENDED_RESULTS.md` R8.

---

## 6. ROS 2

The package is `src/ros2_swarm_transport`.

**Nodes:**
- `mujoco_plant`
- `formation_controller`
- `mover_node`
- `swarm_logger`

**Launch files:**
- `mujoco_validation.launch.py` (physics, lockstep);
- `robots.launch.py` (real robots or any simulator that publishes
  odometry and laser scans).

### 6a. One command for everything (Humble machine)

```bash
source /opt/ros/humble/setup.bash && bash tools/ros_humble_full.sh 2>&1 | tee results/ros_humble/full.log
```

It runs six stages:
1. `colcon build`.
2. The test suite under Humble.
3. The lockstep check: the same mission as two ROS 2 processes and in one
   process must give identical results, in 7 cases (MPC, NN, PA-VFH, PA-VFH
   (no NN), plus the learned and sensor monitors).
4. `ros2 launch` of the installed package.
5. **Real-sensor deployment over DDS.** `robots.launch.py` runs with
   simulated robots publishing `/robotN/odom` and `/robotN/scan`
   (`tools/ros_fake_robots.py`). The controller sees obstacles only
   through the scans.
6. **Watchdog test.** robot1's lidar stops at t = 20 s; every robot must
   stop and `/swarm/safety` must read 1.

Results go to `results/ros_humble/`.

### 6b. Without a Humble machine: Docker

```bash
docker pull ros:humble-ros-base
```

```bash
docker build -t swarm-humble -f tools/docker/Dockerfile.humble tools/docker
```

```bash
docker run --rm --network host -v "$PWD":/ws -w /ws swarm-humble bash tools/ros_humble_full.sh 2>&1 | tee results/ros_humble/full.log
```

### 6c. Without ROS 2 at all

These use an in-memory stand-in for rclpy:

```bash
python3 tools/ros_lockstep_stub.py mpc nn pavfh
```

```bash
python3 tools/ros_sensor_closedloop.py --seeds 1 2 3 4 5 6 --gate-source sensor --sigma 0.02
```

### 6d. On robots

```bash
ros2 launch ros2_swarm_transport robots.launch.py world_sdf:=/path/site_map.sdf contract:=/path/route.json
```

| Interface | Topics |
|---|---|
| Inputs, per robot | `/<r>/odom` (map frame), `/<r>/scan` |
| Outputs | `/<r>/cmd_vel`, `/swarm/phase`, `/swarm/safety` |

Defaults:
- `algo:=pavfh` with `gate_source:=sensor`;
- watchdog 0.5 s;
- `max_v:=0.6`, `max_w:=1.0`.

Read [ROS2_VALIDATION.md](ROS2_VALIDATION.md) §4 first: the controller is
validated in simulation only.

---

## 7. Reports, figures, tables, signals

```bash
python3 tools/make_extended_report.py
```

Writes `docs/EXTENDED_RESULTS.md` (sections R1–R9) and
`results/figures_ext/`.

```bash
python3 tools/make_doc_figures.py && XSTACK=fixedgate python3 tools/make_doc_figures.py
```

```bash
python3 tools/make_tables.py
```

Writes `docs/RESULTS_TABLES.md`.

```bash
python3 tools/make_signal_figures.py
```

Writes `results/figures_signals/S1–S6`: time-series proofs, one per claim
(see [SIGNALS.md](SIGNALS.md)).

---

## 8. Demo video

```bash
python3 tools/make_demo_video.py
```

Writes `results/demo/demo.mp4`: a captioned walk-through with physics
clips, signal plots and results.

---

## 9. Release archive and DOI

```bash
python3 tools/make_release.py --version 1.0.0
```

This builds `dist/*.tar.gz` and `MANIFEST.sha256` (SHA-256 of every file).
[RELEASE.md](RELEASE.md) gives the Zenodo steps for the DOI.

---

## Where to look

| Question | File |
|---|---|
| Title, abstract, claims and their limits | [PAPER_FRAMING.md](PAPER_FRAMING.md) |
| The proposed algorithm and its evaluation | [PA_VFH.md](PA_VFH.md) |
| Neural components and the survey | [NEURAL.md](NEURAL.md) |
| Every number | [EXTENDED_RESULTS.md](EXTENDED_RESULTS.md), [RESULTS_TABLES.md](RESULTS_TABLES.md) |
| ROS 2 status and readiness for hardware | [ROS2_VALIDATION.md](ROS2_VALIDATION.md) |
| State of each review step | [STATUS.md](STATUS.md) |
| The original detailed runbook | [RUNBOOK.md](RUNBOOK.md) |
