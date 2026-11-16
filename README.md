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
| [docs/RUN_GUIDE.md](docs/RUN_GUIDE.md) | **start here**: every command to reproduce every result, with timings |
| [results/demo/demo.mp4](results/demo/demo.mp4) | 5-minute narrated demo video (captions: `demo.srt`) |
| [docs/paper/main.pdf](docs/paper/main.pdf) | **the manuscript** (LaTeX source `docs/paper/main.tex`) |
| [docs/concept_guide/concept_guide.pdf](docs/concept_guide/concept_guide.pdf) | **concept guide PDF**: every concept, every graph explained |
| [docs/PAPER_FRAMING.md](docs/PAPER_FRAMING.md) | title, abstract, contributions and what the study does not claim |
| [docs/SIGNALS.md](docs/SIGNALS.md) | signal plots, one per claim (S1–S6) |
| [docs/ROS2_VALIDATION.md](docs/ROS2_VALIDATION.md) | ROS 2 Humble validation over DDS, deployment mode, watchdog, readiness for hardware |
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
cd src/ros2_swarm_transport && python3 -m pytest test -q
```

Full ROS 2 Humble validation (a Humble machine, or Docker; see the run guide §6):

```bash
source /opt/ros/humble/setup.bash && bash tools/ros_humble_full.sh
```

All results are simulated. `bundle/` and `backup_preaudit/` are frozen
snapshots of earlier sessions and keep the old name "P-VFH".

Cite with [CITATION.cff](CITATION.cff).
