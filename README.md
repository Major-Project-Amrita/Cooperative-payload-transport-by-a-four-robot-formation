# Cooperative payload transport by a four-robot formation

> Repository: `Major Project - Amrita / cooperative-payload-transport-by-a-four-robot-formation`

Four differential-drive robots form up and carry one load through a
warehouse with moving traffic. This workspace holds the Python/ROS 2
replication of the MATLAB study, the cross-stack comparison, and the MuJoCo
physics validation on `aws_warehouse.xml`.

Start here:

| File | What it is |
|---|---|
| [docs/STATUS.md](docs/STATUS.md) | what is finished, what is still running, the commands to finish |
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | every step from a clean terminal, with expected outputs |
| [docs/DOCUMENTATION.md](docs/DOCUMENTATION.md) | concepts, methods, results, every figure explained |
| [docs/RESULTS_TABLES.md](docs/RESULTS_TABLES.md) | generated tables (`python3 tools/make_tables.py`) |
| [docs/Q2_ASSESSMENT.md](docs/Q2_ASSESSMENT.md) | publication-readiness review |

Quick check (2 min):

```bash
cd ~/ros2_ws/src/ros2_swarm_transport && python3 -m pytest test -q
```

```bash
cd ~/ros2_ws && bash tools/ros_lockstep_test.sh mpc on_top
```

`backup_preaudit/` holds the pre-port modules and the two superseded
hand-over notes of earlier sessions (the Gazebo plan and the first MuJoCo
draft).
