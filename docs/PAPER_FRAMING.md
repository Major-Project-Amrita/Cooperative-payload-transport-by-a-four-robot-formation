# Paper framing: title, abstract, contribution and claim limits

This file holds the revised title and abstract (review step 6) and the
explicit limits on what the study claims (review step 5). Numbers come from
[EXTENDED_RESULTS.md](EXTENDED_RESULTS.md) (E19 = section R9). Re-check them
against the final regenerated report before submission.

## Title

**Local steering for rigid multi-robot payload transport in dynamic
warehouses: a replicated benchmark and a learned-occupancy hybrid (PA-VFH)**

Shorter alternative if the venue limits length: *A replicated benchmark of
local steering laws for cooperative payload transport, from kinematics to
physics.*

## Abstract (draft)

Teams of mobile robots that carry one rigid payload must steer the whole
formation through clutter and moving traffic, yet local steering laws for
this task are rarely compared under a common, reproducible protocol. We
present an open benchmark in which twelve steering laws (potential fields,
VFH, the follow-the-gap method, DWA, a lattice MPC, MPPI, four histogram
variants, an imitation-learned policy, and the proposed hybrid PA-VFH) move a
formation of three to nine differential-drive robots through eight
warehouse layouts from 30 × 20 m to 80 × 50 m. Two independent
implementations, in MATLAB and in Python/ROS 2, agree step for step at zero
sensor noise (maximum centroid difference below 1 µm). Every run randomises
the phase of the mover traffic, every planner is tuned under the same
budget, and results are reported with 95 % intervals over all runs, failed
ones included. We separate ground-truth collision screening from a
deployable sensor-based safety monitor and a conformally calibrated learned
monitor, and replicate selected cases in MuJoCo physics while varying robot
mass, speed, friction, payload and actuator force. The proposed PA-VFH uses
only the robots' own instantaneous scans: a small circular CNN predicts
per-sector danger over a 0.5 s horizon from filtered scan history and feeds
the VFH+ decision layer. Against the identical pipeline with an analytic
kernel it reached the goal in 90 of 90 ablation missions (89 of 90) and
halved the steps rejected by the collision gate. Paired intervals still
include no effect, so the learned component is a consistent but not yet
significant improvement. PA-VFH had the highest reach of the five laws
compared, at about a third of MPC's compute. MPC remained about 15 s faster,
and a variant that sees the movers' planned trajectories had fewer than half
as many contact runs. All results are simulated; we state the limits of
transfer to hardware.

## Contributions

1. **A replicated benchmark.** Two independent stacks, a corrected and
   cross-checked collision gate, common random numbers, randomised traffic
   phase, equal-budget tuning, held-out worlds and seeds, and every failed
   run reported. (Steps 1, 2 and 4.)
2. **Scale and layout.** Team sizes 3 to 9, payloads up to 2 × 2 m, and six
   generated warehouses including narrow-aisle and block-stacking layouts.
3. **Safety screening separated from safety monitoring.** Ground truth in
   the shadow (audit), a geometric scan monitor, and a learned monitor with a
   split-conformal margin, each scored for false accepts. (Step 5.)
4. **Physics replication.** MuJoCo runs with phase-randomised traffic and a
   robot-parameter sweep, all runs listed. (Step 3.)
5. **PA-VFH.** A hybrid that keeps the classical histogram decision layer
   and learns only the occupancy prediction, trained against oracle labels
   with on-policy aggregation, and evaluated against its own non-learned
   ablation. A learned team-size-agnostic imitation policy (DAgger of MPC)
   is included as a separate learned baseline.

Where the benchmark sits in the literature:
[RELATED_WORK.md](RELATED_WORK.md) and [NEURAL.md](NEURAL.md).

## What the study does not claim (step 5)

- **Simulation only.** Every result is simulated: a kinematic planner layer,
  MuJoCo physics for selected cases, simulated 2-D lidar/ToF. Nothing has
  been run on hardware.
- **The ground-truth gate is a screening device, not a safety system.** In
  `gate_source="truth"` mode the collision gate reads the simulator's exact
  obstacle and mover geometry. Results in that mode measure what a steering
  law *asks for*, with collisions prevented by an oracle. They make no claim
  about deployable safety.
- **Deployable safety is only claimed in monitor mode.** The sensor monitor
  and the learned monitor decide from the robots' own scans. Their
  false-accept rates (R5) are the only safety numbers the study offers, and
  they hold for the simulated sensor model, the noise levels tested and
  kinematic movers that never yield.
- **The conformal guarantee is marginal and in-distribution.** It covers
  exchangeable gate checks drawn like the calibration set. It does not cover
  new worlds, new sensors or adversarial traffic; R5 reports the shift.
- **Learned components carry no guarantee.** The PA-VFH occupancy network and
  the DAgger policy change where the formation wants to go. Every pose is
  still committed by the gate.
- **Mover model.** Constant-velocity shuttles on fixed routes; no people, no
  reactive agents.
