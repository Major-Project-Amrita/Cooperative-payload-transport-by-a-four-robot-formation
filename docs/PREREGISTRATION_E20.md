# Pre-registration: E20, does the occupancy network improve PA-VFH?

This analysis plan was written and committed **before any E20 run was
executed**; the commit timestamp records this. The analysis will be done
exactly as written, and the result reported whatever it is.

## Question

Does the learned occupancy network improve PA-VFH compared with the
identical pipeline using the analytic kernel, PA-VFH (no NN)?

## Why a new experiment

E19 (90 pairs, seeds 1–6) found differences favouring the network, with 95 %
intervals that include or touch 0:
- mission time −0.9 s [−2.0, +0.3];
- rejected steps −19.3 [−57.9, +0.7].

From E19's standard deviations, about 160 pairs (time) and 330 pairs
(rejections) are needed for an interval to exclude 0 *if* the observed
effect is real.

## Design (fixed)

| Item | Value |
|---|---|
| Laws | `pavfh` (with the network) and `pavfh_nonn` (without) |
| Worlds | dynamic, awswh, wh_m_racks, rand08, maze (as in E19) |
| Noise | σ0 ∈ {0.02, 0.10, 0.20} |
| Seeds | **7–30**, new and disjoint from E19 (1–6) and from all training and tuning seeds (≥ 101) |
| Traffic | random phase, keyed on the seed (as in every experiment) |
| Gate | ground truth, `truth` mode (as in E19) |
| Runs | 5 × 3 × 24 × 2 = **720** (360 pairs) |
| Weights | `pavfh_occupancy.npz` as committed; **no retraining, no tuning, no code change to either law** |

## Analysis (fixed)

- **Pairing:** by (world, σ0, seed); both laws face identical noise and
  traffic (common random numbers).
- **Primary outcomes:** the paired difference, PA-VFH minus PA-VFH (no NN),
  in:
  1. **rejected steps** (all pairs);
  2. **mission time** (pairs where both reached the goal).
- **Secondary outcomes:**
  - contact steps (all pairs; underpowered, expected inconclusive);
  - reach (counts, with McNemar-style discordant-pair counts reported).
- **Statistic:** mean paired difference with a 95 % percentile-bootstrap
  interval (10,000 resamples of pairs, fixed seed 2026).
- **Decision rule:**
  - An outcome counts as **improved by the network** if its whole interval
    is below 0.
  - It counts as **worsened** if its whole interval is above 0.
  - Otherwise it is **inconclusive**.
- **Reporting:**
  1. **E20 alone** is the confirmatory result. It uses fresh seeds, so it is
     independent of E19.
  2. **E19 + E20 pooled** (450 pairs) is reported as a secondary estimate.
- **Both reported regardless.** Both analyses are reported whatever the
  outcome, along with any failed runs. Multiple primary outcomes: each is
  reported separately; no claim is made from a single favourable outcome
  without stating the others.

## What will change in the paper

- **Improved:** if E20 shows a primary outcome improved, the paper may say
  the network significantly reduces that outcome, quoting E20.
- **Inconclusive:** if neither primary outcome improves, the paper states a
  firm null result: no measurable effect at 360 pairs.
- **Worsened:** if a primary outcome worsens, the paper says so.
