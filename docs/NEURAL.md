# Neural networks for multi-robot transport: recent work, gaps, and what this project adds

This note has three parts: a short survey of learning-based multi-robot and
swarm navigation, with an emphasis on cooperative transport and safety; the
gaps that matter for this benchmark; and the two learned components added to
address some of them, with how they are evaluated. Results are in
[EXTENDED_RESULTS.md](EXTENDED_RESULTS.md) (R5, R7, and the `NN (DAgger)`
rows of R3/R4).

> **Scope of the survey.** It was compiled in November 2026 from a targeted
> literature search. Entries marked † were identified from arXiv listings and
> search snippets, and their abstracts were read; their full texts were
> not reviewed. Before citing them in a manuscript, check the exact claims,
> venues and author lists against the papers. Unmarked entries are
> established references.

## 0. In plain terms: the neural networks, the gaps they address, and what they change

The project uses three small neural networks. Each is evaluated against a
non-learned version of the same thing, and the numbers below come from the
result files.

### 0.1 Inside the proposed algorithm: the PA-VFH occupancy network

**What it is.** A circular 1-D convolutional network of about 11,000
parameters.
- **Input:** the last three filtered lidar scans, grouped into 72 bearing
  sectors, plus each sector's range uncertainty, which sectors were
  observed, and how fast the formation is closing on each sector.
- **Output:** a danger value per sector for the **next 0.5 s**.
- **Runtime:** numpy only, well under a millisecond per step, no GPU.
- **Training target:** what an oracle that knows where every shuttle will
  be computes. The labels come from the simulator. At run time the network
  sees only scans.

**Where it sits.** It replaces one block of a classical pipeline: the
analytic danger kernel. Everything after it stays classical and auditable:
- the VFH+ histogram;
- valley selection;
- the anti-deadlock layer;
- the safety gate that checks every move.

**Industry gaps it addresses:**
- **Prediction without a traffic plan.** The former proposal (PA-VFH-K,
  "P-VFH") read the movers' future from the traffic contract. Warehouses
  rarely publish that to robots in real time. PA-VFH infers motion from
  scan history alone.
- **Learned components that do not have to be trusted blindly.** End-to-end
  learned planners are hard to certify. Here the network only re-weights
  sectors, and the classical decision layer and the safety gate stay in
  charge. If the network is wrong, the gate still blocks unsafe poses.
- **Cheap and team-size independent.** The network sees the formation's
  scan, not individual robots. It runs unchanged for 3, 4, 6 or 9 robots
  and costs about a third of MPC's compute per step.

**How it changes the algorithm (measured):**

| Measure | With the network | Without (the identical pipeline with the analytic kernel) |
|---|---|---|
| Missions completed (E19, 90 paired missions) | 90 / 90 | 89 / 90 |
| Gate-rejected steps per mission | about half: −19 [−58, +1] paired | – |
| Blocked/free agreement with the oracle (S3) | 98.2 % | 98.0 % |
| Blocked sectors missed | 8.0 % | 8.3 % |

These gains are **consistent but not statistically significant** at this
sample size. The network also has a start-up transient before it has three
scans of history (S3). PA-VFH as a whole had the highest reach of the five
laws in E19 and the cheapest compute. MPC is faster on average.

### 0.2 A learned steering policy (DAgger imitation of MPC)

A team-size-agnostic policy: a circular CNN over the scan plus a DeepSets
encoder over the robot slots.
- **Gap:** learned swarm policies are usually tested on point robots. Here
  the team is one rigid body whose footprint changes with team size.
- **Result:** it matches its MPC teacher on held-out worlds and on the
  unseen 9-robot team at about 2.5 × less compute. In physics it was clean
  in 26 of 30 runs (P1).

### 0.3 A learned safety monitor with a conformal bound

The network estimates clearance from scans. A split-conformal margin turns
that estimate into an admit/reject decision with a stated error rate.
- **Gap:** learned safety layers rarely state, or measure, their
  false-accept rate.
- **Result:** in distribution, it admitted unsafe poses at the calibrated
  rate. On the fused per-robot scans of the ROS 2 real-sensor mode it
  admitted more (ROS2_VALIDATION §2), so it must be re-calibrated for each
  sensor setup. That is a limit we report, not hide.

## 1. Recent advances

### 1.1 Learned multi-robot navigation and swarm control
- **Graph neural network (GNN) policies.** Decentralised controllers that
  aggregate neighbours' features by message passing. Tolstaya et al.
  (CoRL 2019) learned flocking controllers for swarms. Li, Gama, Ribeiro and
  Prorok (IROS 2020, [arXiv:1912.06095](https://arxiv.org/abs/1912.06095))
  imitated a centralised planner with a GNN. Trained on 10 robots, it
  scales to teams six times larger, and training on larger teams
  generalised better than training on small ones. Follow-ups apply the
  same design to target tracking
  ([arXiv:2105.08601](https://arxiv.org/abs/2105.08601)) and motion
  planning with history
  ([Adv. Intell. Syst. 2023](https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/aisy.202300036)).
  Permutation-invariant set encoders (Deep Sets, Zaheer et al., NeurIPS 2017)
  are the degenerate, fully connected case.
- **Multi-agent reinforcement learning (MARL).** Decentralised collision
  avoidance with deep RL (Chen et al., ICRA 2017; Long et al., ICRA 2018).
  MAPPO (Yu et al., NeurIPS 2022) is the standard strong baseline. Two
  frameworks address reproducibility, which their authors call a crisis in
  the field: VMAS (Bettini et al., DARS 2022) and BenchMARL (Bettini, Prorok
  and Moens, JMLR 25, 2024;
  [paper](https://www.jmlr.org/papers/volume25/23-1612/23-1612.pdf)).
- **Generative (diffusion / transformer) planners.** Multi-robot motion
  planning with diffusion models (ICLR 2025). Diffusion transformers
  treating robots as tokens
  ([arXiv:2606.15550](https://arxiv.org/abs/2606.15550) †). Train-small /
  deploy-large diffusion planning
  ([arXiv:2604.06598](https://arxiv.org/abs/2604.06598) †). Scalable
  multi-agent diffusion policies for coverage
  ([arXiv:2509.17244](https://arxiv.org/abs/2509.17244) †).
- **Learned local planners replacing classical ones.** Survey by Xiao et
  al. (Auton. Robots 2022). LiCS, a transformer imitating a local planner on
  2-D lidar, with a geometric safety-check layer
  ([arXiv:2406.14947](https://arxiv.org/abs/2406.14947)). The BARN
  challenge lessons (ICRA 2024,
  [arXiv:2407.01862](https://arxiv.org/abs/2407.01862)). MPC-SafeGIL,
  imitation with MPC-guided disturbance injection, compared against DAgger
  ([arXiv:2508.03129](https://arxiv.org/abs/2508.03129) †). A 2026 survey
  of learning-based planning in dynamic environments recommends pairing
  learned planners with classical safety layers
  ([arXiv:2608.00625](https://arxiv.org/abs/2608.00625) †).

### 1.2 Learning for cooperative transport
- Classical background: the review by Tuci, Alkilabi and Akanyeti (Front.
  Robot. AI 2018), and constrained-optimisation formation transport
  (Alonso-Mora et al., IJRR 2017).
- Decentralised deep RL for object transport
  ([arXiv:2007.09243](https://arxiv.org/abs/2007.09243)).
- Graph RL for collective-transport planning (Paul et al., ICRA 2023 †; seen only in reference lists).
- Quadruped teams:
  - Dual-quadruped bilevel learning
    ([arXiv:2412.08644](https://arxiv.org/abs/2412.08644) †).
  - Decentralised MARL navigation of a cable-towed load
    ([arXiv:2503.18221](https://arxiv.org/abs/2503.18221) †).
  - Decentralised pinch-lift-move transport by several quadrupeds
    ([arXiv:2509.14342](https://arxiv.org/abs/2509.14342) †).
- MARL shape formation before transport, in VMAS
  ([arXiv:2606.09610](https://arxiv.org/abs/2606.09610) †).
- Safe MARL with decomposed joint constraints, including package–obstacle
  clearance (SafeDiSTL,
  [arXiv:2610.13737](https://arxiv.org/abs/2610.13737) †).
- Vision-language-guided leader-follower transport (CoLF,
  [arXiv:2602.07776](https://arxiv.org/abs/2602.07776) †).

### 1.3 Learned safety: neural barrier functions and conformal prediction
- Learned certificates: survey by Dawson, Gao and Fan (T-RO 2023). Training
  neural control barrier functions (CBFs) for input-constrained systems
  ([arXiv:2310.15478](https://arxiv.org/abs/2310.15478)). Neural CBFs for
  navigation ([arXiv:2407.19907](https://arxiv.org/abs/2407.19907)).
  Decentralised MPC with a learned barrier
  (NMPC-LBF, [arXiv:2208.07974](https://arxiv.org/abs/2208.07974)).
  Formation MPC with per-agent neural barriers
  (FORMULA, [arXiv:2604.04409](https://arxiv.org/abs/2604.04409) †).
- Conformal prediction for safe planning: Lindemann et al. (RA-L 2023),
  the tutorial by Angelopoulos and Bates (2023), and formal verification
  and control with conformal prediction
  ([arXiv:2409.00536](https://arxiv.org/abs/2409.00536)). Learned
  Hamilton-Jacobi safety filters with adaptive conformal inference for
  out-of-distribution states (ACoFi,
  [arXiv:2604.18482](https://arxiv.org/abs/2604.18482) †). Cost-aware
  adaptive conformal runtime assurance
  ([arXiv:2605.24463](https://arxiv.org/abs/2605.24463) †).

## 2. Gaps relevant to rigid multi-robot payload transport

| # | Gap | Why it matters here |
|---|---|---|
| G1 | Learned policies are rarely compared with **tuned** classical planners on the **same** scenarios with common random numbers, randomised traffic and failures reported. MARL papers mostly report normalised return; BenchMARL names reproducibility as the field's main problem. | Without it a learned planner cannot be placed against MPC/MPPI. This benchmark already has CRN, two implementations and a ground-truth screen. |
| G2 | Learned planners ship with **ad-hoc safety layers** (geometric checks, emergency stops), and the monitor's **false-accept rate is almost never measured** against ground truth or given a calibrated bound. Conformal methods give such bounds, but mostly for single robots and trajectory prediction. | The original gate here *is* the ground truth (review objection 4). A deployable monitor must be scored against it, and a learned one must state its error rate. |
| G3 | GNN and swarm policies generalise over **team size for point robots**. With a **rigid shared payload**, the team is one body whose footprint changes with K. That coupling is rarely learned, and transfer to unseen K or payload sizes is rarely tested. | Team size is a design choice for transport. One policy for all K is the useful object. |
| G4 | Learned navigation is usually trained and tested on the same layout family. Transfer to **larger, structurally different warehouses** is rarely measured. | Warehouses differ in scale and topology (racks, narrow aisles, block stacking). |
| G5 | The **physics gap** for learned transport policies (pinned load, wheel slip, actuator limits) is rarely quantified against the kinematic model the policy was trained in. | MuJoCo is already in the loop here. |

## 3. What was added, and how each gap is tested

### 3.1 A learned steering law for the rigid formation (G1, G3, G4, G5)
`ros2_swarm_transport/neural.py`, trained by `tools/train_neural.py policy`.

- **Input.** The same noisy scan every law consumes, min-pooled into 72
  body-frame sectors and expressed relative to the body's swept radius
  (range − margin). Also the goal in the body frame, and the robot slot
  offsets.
- **Architecture.**
  - A circular 1-D CNN over the sectors.
  - A Deep Sets encoder over the slot offsets (shared per-slot MLP, mean and
    max pooling). It is permutation-invariant and defined for any K, so one
    network serves 3, 4, 6 and 9 robots.
  - An MLP head that classifies over the expert's heading-change levels.
- **Training.** DAgger (Ross, Gordon and Bagnell, AISTATS 2011) with the
  lattice MPC as expert. β = 1, ½, ¼, ⅛ over four iterations.
- **Training and held-out data.** Training uses randomised traffic phase on
  the worlds rand01–05, the study warehouse and the plaza, with teams of
  3, 4 and 6 robots (6 only at s = 0.59 m), and training seeds ≥ 101.
  Held out: rand06–10, the maze, the AWS warehouse, all six generated
  warehouses, the 9-robot team, both large payloads, and evaluation
  seeds 1–10.
- **Deployment.** Inference is pure numpy (checked against torch to
  1e-7), so the ROS 2 node needs no deep-learning runtime. The policy runs
  inside the unchanged mission layer, so the pose commit is still gated
  like every other law's.
- **Where it is tested.**
  - E15: 8 layouts, against the expert and the tuned classical planners.
  - E16: team size, including unseen K and payloads.
  - P1: MuJoCo, 3 payload modes × 10 phase-randomised seeds.

### 3.2 A learned safety monitor with a conformal bound (G2)
`SensorSafetyMonitor.learned_safe`, trained by
`tools/train_neural.py monitor`.

- **What it predicts.** The true body clearance at t and t+dt (the quantity
  the ground-truth gate thresholds), from two consecutive filtered scans
  seen from the candidate pose. The second scan is what lets it account for
  moving obstacles without the mover contract.
- **Calibration.** Split conformal on runs held out from training. The
  one-sided residual quantile q_α gives the rule *admit iff
  prediction − q_α > robot radius*. Under exchangeability this bounds the
  probability that an admitted candidate is unsafe by α.
- **Comparisons (E17).** The learned monitor is compared with the
  ground-truth screen and the hand-designed geometric monitor (3σ range
  inflation + v_max·Δt motion allowance), on held-out worlds and seeds.
- **What it measures.** The monitor's false-accept rate per gate check on
  data that is *not* exchangeable with the calibration set (new layouts),
  which is where the guarantee formally ends.

### 3.3 What is *not* claimed
- No hardware. Both components are evaluated in simulation only, with a
  simulated 2-D lidar.
- The conformal bound is **marginal, per gate check**. It is not a
  per-mission safety guarantee, and it does not hold under distribution
  shift. E17 reports the empirical rate on shifted data.
- The policy imitates MPC. Unless the results show otherwise, it is not
  expected to beat its expert. Its value is the cost of a forward pass,
  team-size generality from one network, and a clean, fully reported
  comparison.
- Movers never yield, and every law, learned or not, is screened by the same
  gate. A contact therefore means a mover entered a formation that found no
  admissible escape.

## 4. Reproduce

```bash
python3 tools/train_neural.py policy  --jobs 4 --iters 4 --per-cell 2 --epochs 25
```

```bash
python3 tools/train_neural.py monitor --jobs 4 --per-cell 2 --epochs 25
```

```bash
python3 tools/run_extended.py --exp E15,E16,E17 --jobs 4
```

```bash
python3 tools/make_extended_report.py
```

Weights, training logs and calibration quantiles are committed under
`src/ros2_swarm_transport/models/`.
