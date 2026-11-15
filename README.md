# Cooperative payload transport by a robot formation

> Repository: `Major Project - Amrita / cooperative-payload-transport-by-a-four-robot-formation`

A team of differential-drive robots (3 to 9; four in the reference study)
forms up and carries one rigid load through a warehouse with moving traffic.
This workspace holds:

- the MATLAB reference study (`matlab_ref/`);
- its Python/ROS 2 replication (`src/ros2_swarm_transport/`) and the
  cross-stack comparison;
- the MuJoCo physics validation;
- the extended benchmark: randomised traffic phase, eight warehouse
  layouts, team sizes 3 to 9, equal-budget tuning, sensor-based and learned
  safety monitors;
- the learned components, including the proposed hybrid **PA-VFH**.

> **Nomenclature.** The proposed algorithm is **PA-VFH** (code key `pavfh`),
> a VFH+ decision layer fed by a learned predictive-occupancy network. The
> algorithm earlier documents call **P-VFH** (key `pvfh_prob`) is now
> **PA-VFH-K**. See [docs/PA_VFH.md](docs/PA_VFH.md).

Start here:

| File | What it is |
|---|---|
| [docs/PAPER_FRAMING.md](docs/PAPER_FRAMING.md) | title, abstract, contributions and what the study does not claim |
| [docs/PA_VFH.md](docs/PA_VFH.md) | the proposed hybrid algorithm and how it is evaluated |
| [docs/EXTENDED_RESULTS.md](docs/EXTENDED_RESULTS.md) | generated results of the extended benchmark (`python3 tools/make_extended_report.py`) |
| [docs/NEURAL.md](docs/NEURAL.md) | survey of neural methods for multi-robot transport, gaps, learned components |
| [docs/RELATED_WORK.md](docs/RELATED_WORK.md) | positioning and the benchmark's contribution |
| [docs/STATUS.md](docs/STATUS.md) | what is finished, what is still running, the commands to finish |
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | every step from a clean terminal, with expected outputs |
| [docs/DOCUMENTATION.md](docs/DOCUMENTATION.md) | concepts, methods, results, every figure explained |
| [docs/RESULTS_TABLES.md](docs/RESULTS_TABLES.md) | generated tables (`python3 tools/make_tables.py`) |
| [docs/RELEASE.md](docs/RELEASE.md) | archive, checksums and how to mint the DOI |

Quick check (2 min):

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m pytest test -q
```

```bash
cd ~/ros2_ws && bash tools/ros_lockstep_test.sh mpc on_top
```

All results are simulated. `bundle/` and `backup_preaudit/` are frozen
snapshots of earlier sessions and keep the old name "P-VFH".

Cite with [CITATION.cff](CITATION.cff).
