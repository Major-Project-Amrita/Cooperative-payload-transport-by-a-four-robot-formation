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
about 0.1 ms per step; weights are in
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

## 4. Limits

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
