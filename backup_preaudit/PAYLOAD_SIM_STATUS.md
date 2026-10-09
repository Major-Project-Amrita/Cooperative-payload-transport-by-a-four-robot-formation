# Payload-transport simulation — status & handover

Date: 2026-10-02. Decision locked by the user: **Gazebo will not run on their
system → use MuJoCo.** Deliverable: one final simulation per formation where 4
robots carry a load from A(−12, 0) to B(12, 0), in **both payload modes** —
`behind` (towed sled) and `on_top` (carried on the robots) — i.e. the matrix
{square, line, triangle, diamond} × {behind, on_top} = **8 runs**, with MP4
videos, CSV logs and an acceptance REPORT.md.

## Environment facts (verified in this sandbox)

- MuJoCo **3.14.0** installed; offscreen rendering works with `MUJOCO_GL=egl`
  (tested OK). ffmpeg at `/usr/bin/ffmpeg`. Python 3.10.
- ROS 2 Humble + gz ARE installed here, but per the user's instruction they are
  NOT to be used for this demo.
- Package `src/ros2_swarm_transport` is complete and its 32 pytest tests pass.
  The kinematic mission runner (`headless_runner.py`) and the deployed Gazebo
  controller (`formation_controller.py`) are the algorithm reference.

## What has been done

1. **`src/ros2_swarm_transport/worlds/payload_warehouse_mjcf.xml`** (NEW, done)
   — MuJoCo twin of the 30×16 m SDF warehouse: floor, four diff-drive robots
   (chassis cyl r=0.18, 2 wheel velocity actuators kv=0.4, 2 ball casters,
   roof pad, hitch site), decor from the user's `aws_warehouse.xml` palette
   (blue wall bands, ceiling trusses + lamps, all collision-free), and
   injection markers `<!--INJECT:OBSTACLES/GOAL/MOVERS/PAYLOAD/EQUALITY-->`.

2. **`src/ros2_swarm_transport/ros2_swarm_transport/mujoco_payload_sim.py`**
   (NEW, written but NOT yet executed — see bug list). Design that is
   implemented:
   - Static obstacle geometry is injected from the SDF via `world_from_sdf`,
     so algorithm and physics see identical geometry. Movers follow the shared
     `dynamic_obstacles.json` contract as mocap bodies.
   - Phase 1 forming: per-robot APF + `steering_point` (deployed law).
   - Phase 2 transport: leader frame advances exactly like `headless_runner`
     (`steering_rigid`, t/t+dt clearance commit, mover-gap slowdown, escape
     probes); robots P-servo to slots with the **verbatim deployed law** from
     `formation_controller.py` (`v_cmd = v_frame·e_hat[0] + min(2·en, 0.6)`,
     `w_cmd = clip(1.5·err_th, ±0.3)`).
   - Payload `on_top`: plate (+ crate) **welded** to the carrier robot
     (triangle: 3 crates, one per robot; a triangular plate would overhang the
     planned hull+r footprint). Plate footprint matches the algorithm's
     payload block (square/line/diamond boxes, diamond rotated 45°).
   - Payload `behind`: castor sled parked at (−13.0, 2.6) (clear of the ring
     and of mover2's y-band), **hooked at formation-complete** via a `connect`
     (ball) equality to the rear robot (slot with min x) with runtime
     `eq_data` anchors; sled qpos is set to the hitched pose first (no snap).
   - Outputs per run: CSV (same core schema as swarm_logger + payload/sled
     columns), MP4 (EGL, 1280×720, 10 fps, camera tracking the formation),
     trajectory PNG, and `REPORT.md` with PASS/FAIL per run.
   - CLI: `--formation all|square|line|triangle|diamond`,
     `--mode all|behind|on_top`, `--algo pvfh_prob`, `--seed 7`, `--no-video`,
     `--output logs/mujoco` (resolved to project root regardless of cwd).

## What must be done next — in order

### B1 (CRITICAL, blocks everything): payload bodies need free joints
In `_payload_xml`, the `payload_top` plate body and the triangle
`payload_top{i}` crate bodies were created WITHOUT a joint → MuJoCo treats
them as static; welding a static body pins the robot. Fix: add
`<joint name="payload_top_free" type="free"/>` (resp.
`payload_top{i}_free`) as the first child of those bodies, and set their
initial qpos in `run_payload_mission`'s init block:
- plate: `[START[0], START[1], 0.01, cos(yaw0/2), 0, 0, sin(yaw0/2)]`
- crate i: same pose as its carrier robot (geoms carry the z=0.312 offset, so
  crate frame == robot frame; then the weld relpose `0 0 0.312 1 0 0 0`
  matches reality → no snap).

### B2: plate weld relpose z must be 0.0, not 0.225
In `_payload_xml` (non-triangle branch) `rel = np.array([0,0,0.225])` → the
plate body origin sits at floor height and the geoms carry the z-offsets, so
relpose z must be `0.0` (z of plate origin minus carrier origin).

### B3: `_weld_err` reads the wrong eq_data slice
MuJoCo 3.x weld layout: `eq_data = [anchor(3), relpose(7), torquescale(1)]` →
relpose position is `eq_data[3:6]`, not `[0:3]`. Change
`rel = model.eq_data[eq_id, 0:7]` → `[3:10]` (use pos `[3:6]`).

### B4: verify equality conventions empirically BEFORE the full runs
Tiny script (run once, ~10 s): two free bodies, one weld with relpose computed
as "pose of body1 in body2 frame" (`pos = R2ᵀ(p1−p2)`, `quat = q2⁻¹⊗q1`),
simulate 300 steps, assert relative drift < 1 mm — if it drifts, the
convention is inverted; fix the helper + relpose accordingly. Same for
connect: set `eq_data[0:3]`=anchor1 (body1=sled frame), `[3:6]`=anchor2
(body2=robot frame), drive the robot 1 m, assert anchor distance < 1 cm.
`_sled_hook_err` / `hook_sled` in the module already use these conventions —
flip them only if the test says so.

### B5: mover teleports should use exact physics time
Both mover-update blocks inside the loop use `n_ticks * DT_CTRL`; change to
`n_ticks * DT_PHYS` (settle block too) so movers glide at contract speed
instead of jumping 9 cm per control tick. At control ticks the algorithm still
evaluates movers at the grid time — consistent by construction.

### B6: camera orientation check
Goal pad is green at x=+12. Render one frame at the start; if +x is not to the
right on screen, change `cam.azimuth = 90.0` to −90 (or 270).

### B7: smoke run (no video), then full matrix
```bash
python3 -m ros2_swarm_transport.mujoco_payload_sim \
    --formation square --mode behind --no-video     # fix issues
python3 -m ros2_swarm_transport.mujoco_payload_sim \
    --formation square --mode on_top --no-video
python3 -m ros2_swarm_transport.mujoco_payload_sim \
    --formation all --mode all --video              # 8 runs, ~15–25 min total
```
Outputs land in `logs/mujoco/`: CSVs, `videos/*.mp4`, `traj_*.png`,
`REPORT.md`. A run takes ~1–3 min (forming ≤2500 ticks, transport ≤6000 ticks
at 500 Hz physics, rendering dominates).

### B8: tests + report
- `python3 -m pytest test/ -q` must stay green (32 tests; the new module must
  not import-break anything). Optionally add `test/test_mujoco_sim.py` smoke
  test (square+behind, no video, capped ticks, assert weld/connect drift
  < 2 cm).
- REPORT.md criteria (from the original acceptance prompt): reached B
  (commanded frame < 0.5 m AND actual centroid < 0.6 m), min pairwise spacing
  vs 1.18 m nominal (report value; ≥ 0.95 m ≈ PASS), algorithm min clearance
  > robot radius, hard contacts (>2 N) during transport == 0, payload
  integrity (weld/hitch anchor drift) < 2 cm, wall vs sim time.

## Known design notes to repeat in the report

- `behind` sled is NOT in the algorithm's planned footprint (same as the
  MATLAB `point_along_trail_behind` reference); it can scrape obstacles —
  report the contact count, don't silently hide it.
- `on_top` plate footprint ≈ the algorithm's payload block (hull + r) ± 3 cm;
  triangle uses per-robot crates (strictly inside the planned footprint).
- Physics time between control ticks means movers are up to ~9 cm ahead of
  the controller's model — the honest "world moved while controller slept".
- go2 quadrupeds from the user's XML were replaced by diff-drive platforms:
  the steering contract is cmd_vel unicycle; trot control is out of scope.

---

## NEXT AGENT PROMPT (paste into a fresh session)

You are finishing the MuJoCo payload-transport demo in this workspace
(30×16 m warehouse twin of `src/ros2_swarm_transport/worlds/swarm_world_dynamic.sdf`).
Gazebo is NOT to be used (user's system can't run it); MuJoCo 3.14.0 +
`MUJOCO_GL=egl` + ffmpeg are installed and working. Everything below is
already written but NOT yet executed: `worlds/payload_warehouse_mjcf.xml`
(warehouse shell + 4 diff-drive robots + injection markers) and
`ros2_swarm_transport/mujoco_payload_sim.py` (full mission: forming with
`steering_point`, transport with `steering_rigid` exactly like
`headless_runner.py`, robots servoed with the deployed
`formation_controller.py` P-law, payload `on_top` = weld / `behind` = parked
sled hooked with a connect equality, CSV + MP4 + PNG + REPORT.md outputs).
Read `PAYLOAD_SIM_STATUS.md` first — it lists the exact known bugs.

Work through these steps in order; stop and report if a step fails:

1. Fix the bugs in `PAYLOAD_SIM_STATUS.md` §B1–B5 (free joints + initial qpos
   for payload bodies; plate weld relpose z=0; `_weld_err` eq_data[3:10];
   empirical weld/connect convention check; mover teleports at physics dt).
2. Convention check script: weld drift < 1 mm over 300 steps; connect anchors
   stay coincident under 1 m of driving. Flip the helper convention only if
   this fails.
3. Camera check: first frame must show +x (green goal pad at (12,0)) pointing
   right; else set `cam.azimuth = -90`.
4. Smoke runs, no video: square/behind, then square/on_top. Expect: forming
   converges (< 0.3 m slot error), phase 2 runs, dist_to_goal → < 0.5 m, no
   NaNs, sled hook drift < 2 cm, plate weld drift < 2 cm.
5. Full matrix: `python3 -m ros2_swarm_transport.mujoco_payload_sim
   --formation all --mode all --video` → 8 runs, 8 MP4s, 8 CSVs, 8 PNGs,
   REPORT.md in `logs/mujoco/`.
6. `python3 -m pytest test/ -q` — all green (the module must not break
   imports).
7. Final acceptance summary per run (PASS/FAIL): reached B (commanded < 0.5 m
   and actual centroid < 0.6 m), min pairwise spacing vs 1.18 m, min
   clearance, hard transport contacts, payload integrity, wall/sim time.
   Print the 8-video file list and keep REPORT.md consistent with it.

Do not modify the existing steering/geometry modules; fixes go in
`mujoco_payload_sim.py` / the MJCF template only.
