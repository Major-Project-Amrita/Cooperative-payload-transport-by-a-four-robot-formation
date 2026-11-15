# Related work and the contribution of the benchmark

Draft text for the manuscript's related-work section (review step 4). It
positions the work as a **benchmark and replication study**, not a new
planner. References marked † are recent arXiv items found by literature
search; check them against the full papers before submission (see
[NEURAL.md](NEURAL.md)).

## 1. Local steering for mobile robots

- **Potential fields and histograms.** Potential fields (Khatib, IJRR
  1986) and vector-field histograms: VFH (Borenstein and Koren, T-RA 1991)
  and VFH+ (Ulrich and Borenstein, ICRA 1998), with enlarged obstacles and
  hysteresis.
- **Gap-based steering.** Follow-the-Gap (Sezer and Gökaşan, RAS 2012).
- **Velocity-space search.** The Dynamic Window Approach (Fox, Burgard and
  Thrun, RAM 1997).
- **Optimisation-based planners.** Receding-horizon MPC. Sampling-based
  Model Predictive Path Integral control: MPPI (Williams et al., ICRA 2016;
  T-RO 2018), now the default local controller family in ROS 2 Nav2
  (Macenski et al., IROS 2020).

These are almost always evaluated for a *single* robot, and against one
another in only a handful of scenarios. Their behaviour when the "robot" is
a rigid multi-robot block with a 2 m swept diameter is rarely measured.

## 2. Cooperative transport and formations

- **Reviews.** Tuci, Alkilabi and Akanyeti (Front. Robot. AI 2018) review
  cooperative object transport.
- **Optimisation-based transport.** Formation control and object transport
  in dynamic environments by constrained optimisation (Alonso-Mora et al.,
  IJRR 2017).
- **Implicit coordination.** Force-based coordination without
  communication (Wang and Schwager, 2016).
- **Learning-based transport (2024–2026).** Decentralised RL for object
  transport. Graph RL for collective-transport planning (Paul et al. †).
  Quadruped teams carrying or towing loads (arXiv 2412.08644 †, 2503.18221 †,
  2509.14342 †). MARL shape formation before transport (arXiv 2606.09610 †).

Transport papers typically present one method in a few scenarios. The
mission layer (forming, docking, rigid transport) is coupled to the
proposed planner, so the planner's contribution cannot be separated from the
mission logic.

## 3. Benchmarks and reproducibility

- **MARL benchmarks.** VMAS (Bettini et al., DARS 2022) and BenchMARL
  (Bettini, Prorok and Moens, JMLR 2024) standardise tasks and reporting,
  and their authors name reproducibility as the field's central problem.
- **Navigation benchmarks.** BARN (Perille et al., SSRR 2020; ICRA 2024
  challenge report) and Bench-MR for single-robot motion planning.

None of these addresses rigid multi-robot payload transport with moving
traffic. Independent re-implementation of a robotics simulation study, with
step-for-step agreement, is rare in this literature.

## 4. Safety screening and monitors

- **Collision checking.** Separating-axis tests for convex bodies
  (Gottschalk et al., SIGGRAPH 1996).
- **Certificates.** Control barrier functions (Ames et al., ECC 2019) and
  learned certificates (Dawson, Gao and Fan, T-RO 2023).
- **Conformal methods.** Conformal prediction for safe planning among
  uncertain agents (Lindemann et al., RA-L 2023), and conformal runtime
  assurance († 2026).

Simulation studies often report safety from the simulator's ground-truth
geometry, which no deployed robot has. Few separate that **metric** from
the robot's **monitor**, or measure the monitor's false-accept rate.

## 5. The contribution of this work, stated against the above

1. **An open benchmark for rigid multi-robot payload transport** with
   moving traffic. It covers:
   - ten steering laws (APF, VFH, FGM, DWA, lattice MPC, MPPI, three
     histogram variants, and a learned DAgger policy) behind one
     law-independent mission layer;
   - 13 grid experiments (E1–E14, with E4 reported from E5) and 4 extended ones (E15–E18);
   - designed, random and generated warehouse layouts from 30 × 16 m to
     80 × 50 m;
   - teams of 3–9 robots;
   - common random numbers, a **randomised traffic phase**, and every run
     reported, failures included.
2. **Fairness of comparison.** Every law, the authors' own included, is
   tuned under the same budget on training worlds. Tuned and default
   parameters are evaluated on held-out worlds and seeds.
3. **An independent cross-implementation replication.** MATLAB and
   Python/ROS 2 agree step for step at zero noise for all eight original
   laws, under both the original gate and the corrected one. The
   replication found a defect in the reference's collision test, which is
   now corrected and quantified.
4. **Ground-truth screening separated from deployable monitors.** A
   geometric sensor monitor and a learned monitor with a split-conformal
   margin are scored against the ground-truth screen, with false-accept
   rates reported.
5. **A physics validation that measures where the kinematic model stops
   being predictive.** It is replicated over seeds and traffic phases, and
   covers payload modes and robot parameters.

**What is not claimed:** a new planner that outperforms the state of the art;
deployment on hardware. All results are from simulation (kinematic model,
MuJoCo, and ROS 2 nodes in lockstep with the simulator).
