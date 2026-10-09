# Agent prompt — paste this into a NEW session opened in your ROS 2 workspace

Unzip `release/ros2_swarm_transport_bundle.zip` first (step 0), then paste
everything below the line. Replace `<REPO>` with the absolute path of the
MATLAB repository (`swarm_matlab`).

---

You are working in `~/ros2_ws`. The bundle is unpacked at `~/ros2_ws/bundle`:

- `bundle/ros2_swarm_transport/` — the Python/ROS 2 package (pre-audit code)
- `bundle/ROS2_REPLICATION.md` — the replication spec; **section 9 is the
  October 2026 audit port list and overrides sections 1–8**
- `bundle/reference/matlab/` — the corrected MATLAB reference (the
  specification); `bundle/reference/AUDIT_CHANGELOG.md` explains each change
- `bundle/reference/results_all.csv` and `bundle/reference/RESULTS_LOG.md` —
  the MATLAB Monte Carlo results to replicate, if present. If
  `RESULTS_LOG.md` says "LEGACY DATA", the corrected MATLAB grid has not been
  run yet: do Parts A–C, skip the comparison in Part D, and say so.
- the MATLAB repository itself is at `<REPO>` (read-only for you, except the
  LaTeX report in Part E).

The goal: bring the Python stack in line with the corrected MATLAB
reference, reproduce the experiment grid on it, run the ROS 2/Gazebo mission,
and update the report with a cross-implementation section. Work in order.
Do not change anything under `<REPO>/matlab`. Report plainly when something
fails; do not tune parameters to make numbers match.

0. Unpack if needed:
   `mkdir -p ~/ros2_ws && cd ~/ros2_ws && unzip -o <REPO>/release/ros2_swarm_transport_bundle.zip -d . && mv ros2_bundle bundle`
   (skip the `mv` if `bundle/` already exists).

**Part A — port the audit fixes (no ROS needed).**

1. Read `bundle/ROS2_REPLICATION.md` section 9 and
   `bundle/reference/AUDIT_CHANGELOG.md` completely, then read
   `reference/matlab/Simulation8_dynamic.m` (phase 2),
   `steering_dispatch.m`, `scan_ranges_dyn.m`, `apf_obstacle_repulsion.m`,
   `pvfh_prob.m`, `find_escape_heading_dyn.m`, `summarize_run.m`,
   `write_log_csv.m`, `run_paper_experiments.m`.
2. Run the existing tests first and record the baseline:
   `cd bundle/ros2_swarm_transport && python3 -m pytest test -q`.
3. Implement section 9.1 (transport loop: atomic pose commit with
   rotate-only fallback, steered escape instead of heading teleport,
   anti-deadlock for every algorithm with an off switch, hold-horizon escape
   probe, waypoints from the contract, mover speed scale, 300 s timeout) in
   `headless_runner.py`, and the same logic in `formation_controller.py`.
4. Implement section 9.2 in `vfh.py`: body-frame ToF fan with wrapped
   angles and a returned beam width, the beam width in the reflex,
   instantaneous-scan baselines that still see movers, `pvfh` on the noisy
   scan, the three new laws `fgm`, `dwa` and `mpc` ported line for line
   from `steering_dispatch.m`, the recursive range filter (`range_filter.m`)
   inside `pvfh_prob`, the ablation law `pvfh_filt`, the blend fade and the
   baseline horizon (section 9.2 items 4–4d). Add `--algo fgm|dwa|mpc|pvfh_filt`, `--world`,
   `--contract`, `--mover-scale`, `--max-movers`, `--loc-sigma`,
   `--comm-drop`, `--kin`, `--no-adl` to the CLI (section 9.1 items 7–12),
   and the summary field `max_robot_speed`.
5. Implement section 9.3: the `reject`, `escape`, `clearance_static` log
   columns and the summary fields `coll_static`, `coll_mover`, `n_reject`,
   `n_escape`, phase-2-only `comp_ms_mean`.
6. Add tests: (a) a blocked body never changes heading into a colliding
   pose (`coll_static == 0` on a full mission for each algorithm, lidar,
   seed 1); (b) the ToF fan follows the heading and its `dth` is fov/n;
   (c) `vfh` and `pvfh` differ only in the horizon (identical output with no
   movers and the same RNG state); (d) `pvfh_prob` equals `pvfh` as
   `sigma0 -> 0`; (e) the kernel reaction distance increases with `s_b` up
   to `1.757 * sigma_h` (Proposition 2 of the report); (f) `fgm`, `dwa` and `mpc`
   return unit vectors and steer away from a wall placed ahead. All tests
   must pass.
7. Deterministic cross-check: with `--sigma0 0` on the lidar no randomness
   matters. Run `pvfh` and `pvfh_prob`, seed 1, square, warehouse world, and
   compare against the MATLAB per-step log of the same configuration if one
   is in `bundle/reference/` (otherwise print the exact MATLAB command the
   user should run to produce it and continue). Report the maximum position
   difference over the run and where the two first diverge.

**Part B — the grid on the Python stack.**

8. Rewrite `<REPO>/matlab/run_headless_grid.py` *as a copy in*
   `~/ros2_ws/run_headless_grid.py` (do not edit the one in the repo) so its
   manifest equals `build_jobs` in `run_paper_experiments.m` (E1, E2, E3,
   E5–E14; shared configurations simulated once), with the CSV
   columns and `run_name` scheme of section 9.4, `stack = ros2_headless`,
   per-run checkpointing and a `--jobs N` process pool. Output to
   `~/ros2_ws/results/results_headless.csv` and `~/ros2_ws/results/runs/`.
9. Run a smoke subset (one seed, one cell per experiment), fix what breaks,
   then run the full grid. Print the wall time.
10. Render it with the MATLAB repo's script, into a scratch folder:
    `python3 <REPO>/matlab/make_paper_figures.py --csv results/results_headless.csv --runs results/runs --outdir results/figures --gendir results/generated --log results/RESULTS_LOG_headless.md`

**Part C — ROS 2 / Gazebo.**

11. Environment audit: `printenv ROS_DISTRO`, `gz sim --version`, `ros_gz`,
    `colcon`, `rosdep`. Report present/missing; install only what is missing.
12. `mkdir -p src && cp -r bundle/ros2_swarm_transport src/`,
    `rosdep install -i --from-path src -y`, `colcon build --symlink-install`,
    source `install/setup.bash`. Fix build errors.
13. Complete the robot spawning that `launch/transport.launch.py` marks as
    TODO: four robots `robot1..robot4` at the start ring (centre (-12, 0),
    radius 2.2 m, angles 0/90/180/270°), with `cmd_vel`, odom and scan
    bridges. The footprint must be consistent with `robot_radius` in
    `params/swarm.yaml`; state which you chose.
14. Launch on `swarm_world_dynamic.sdf`; verify the topics
    (`/dynamic_obstacles`, `/robotN/odom`, `/robotN/cmd_vel`,
    `/swarm/phase`); record one full mission per algorithm in
    {`pvfh`, `pvfh_prob`, `dwa`, `mpc`} with `swarm_logger` at 10 Hz. Then one
    `pvfh_prob` mission on the plaza and one on the maze world.
15. For each Gazebo run report: reached or not, time, minimum formation
    spacing error, closest approach to obstacles and movers, and whether the
    controller's gate ever rejected a pose. Gazebo has dynamics the kinematic
    model lacks; describe differences, do not hide them.

**Part D — comparison.**

16. If `bundle/reference/results_all.csv` is the corrected MATLAB grid:
    for every cell compute, for both stacks, reach with Wilson 95% interval
    and bootstrap 95% intervals of `n_reject` and of the minimum surface gap
    (`min_clear - 0.18`). Write `results/cross_stack.csv` and a LaTeX table
    `results/generated/tab_cross.tex` with one row per cell and an
    `agree` column (intervals overlap on all three). List every disagreeing
    cell and investigate the three largest down to a cause (first divergent
    step, which component). `coll_static` must be 0 everywhere on both.

**Part E — the report.**

17. In `<REPO>/docs/simulation_report/`: add a section file
    `sections/11_replication.tex` ("Independent replication: Python headless
    and ROS 2 / Gazebo") and `\input` it after `10_howto` in `main.tex`. It
    must contain: what was ported (reference section 9), the deterministic
    cross-check result, `tab_cross.tex` (copy it into
    `docs/simulation_report/generated/`), the agreement summary with the
    disagreeing cells named, the Gazebo mission results, and a limitations
    paragraph. Copy the headless figures you cite into
    `docs/simulation_report/figures/` with a `headless_` prefix — do not
    overwrite the MATLAB figures.
18. Update `sections/10_howto.tex` §"Which implementation the paper uses",
    the "Independent replication" row of the roadmap table and the first
    bullet of "Threats to validity" in `sections/08_experiments.tex`, and
    one sentence of the abstract in `main.tex`, to say what the replication
    did and did not establish. Do not edit the generated tables or the
    MATLAB result text.
19. Compile (`pdflatex main` twice), fix LaTeX errors, and confirm there are
    no undefined references.
20. Final report to the user: tests passed/failed, grid wall time, number of
    agreeing/disagreeing cells with the named causes, Gazebo outcomes, files
    changed in `<REPO>/docs`, and anything you could not do.
