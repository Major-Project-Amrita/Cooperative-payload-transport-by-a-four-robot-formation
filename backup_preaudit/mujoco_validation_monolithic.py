"""Physics validation of the swarm-transport mission in MuJoCo.

The kinematic model (MATLAB reference and headless_runner.py) treats the
formation as a rigid frame that moves exactly as commanded. Here the SAME
mission logic (transport.FormingPhase / transport.TransportLeader -- no
re-implementation) drives four differential-drive robots with mass, wheel
contact, actuator limits and a real payload:

  payload "virtual"  no load (the kinematic model's assumption, with physics)
  payload "on_top"   a plate + crate carried on the robots' roofs, pinned to
                     every robot with a ball joint at pick-up
  payload "behind"   a castor sled hitched to the rear robot (it is NOT part
                     of the planned footprint, as in the kinematic model)

The scene is either the user's AWS-style warehouse MJCF (worlds/
aws_warehouse.xml, default) or any SDF world of the study; the planner's 2D
map is the same file the MATLAB reference loads (the SDF), so the planner,
MATLAB and the physics engine share one geometry. Movers follow the JSON
motion contract as mocap bodies.

    python3 -m ros2_swarm_transport.mujoco_validation --algo mpc --payload on_top
    python3 -m ros2_swarm_transport.mujoco_validation --matrix --video

Outputs (default results/mujoco/): <run>.csv, <run>.mp4, <run>.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np                                   # noqa: E402
import mujoco                                        # noqa: E402

from .geometry import formation_clearance, movers_distance   # noqa: E402
from .headless_runner import (contract_meta, contract_waypoints,  # noqa: E402
                              load_scenario, plan_formation, summarize_run)
from .transport import (FormingPhase, TransportLeader, eval_movers,  # noqa: E402
                        make_params, rot2d, slot_tracking_cmd,
                        velocity_cmd, wrap, _clip_norm)
from .vfh import ALGOS, VfhConfig                    # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(PKG, "worlds")
ROOT = os.path.dirname(os.path.dirname(PKG))

DT_CTRL, DT_PHYS = 0.1, 0.002
SUB = int(round(DT_CTRL / DT_PHYS))
# robot: 0.18 m footprint radius (the planner's robot_radius), diff drive
WHEEL_R, HALF_TRACK, ROOF_Z, PIN_Z = 0.05, 0.14, 0.20, 0.05
V_WHEEL_MAX = 1.2                 # m/s rim speed the actuators can command
HOLD_MAX = 20                     # control steps the frame may wait in a row
COLORS = ["0.15 0.35 0.75 1", "0.85 0.30 0.20 1", "0.20 0.55 0.30 1",
          "0.90 0.70 0.10 1"]


# ---------------------------------------------------------------------------
# scene
# ---------------------------------------------------------------------------

def _robot_xml(i, color):
    n = f"robot{i}"
    return f'''
    <body name="{n}" pos="0 0 0">
      <freejoint name="{n}_free"/>
      <geom name="{n}_chassis" type="cylinder" size="0.18 0.07" pos="0 0 0.11" rgba="{color}" mass="6.0" friction="0.4 0.005 0.0001"/>
      <geom name="{n}_roof" type="cylinder" size="0.12 0.01" pos="0 0 0.19" rgba="0.9 0.92 0.95 1" mass="0.2"/>
      <geom name="{n}_nose" type="box" size="0.04 0.02 0.005" pos="0.12 0 0.185" rgba="0.1 0.1 0.1 1" mass="0.01" contype="0" conaffinity="0"/>
      <geom name="{n}_caster_f" type="sphere" size="0.02" pos="0.14 0 0.02" rgba="0.2 0.2 0.2 1" mass="0.05" friction="0.02 0.001 0.0001" condim="1" priority="1"/>
      <geom name="{n}_caster_b" type="sphere" size="0.02" pos="-0.14 0 0.02" rgba="0.2 0.2 0.2 1" mass="0.05" friction="0.02 0.001 0.0001" condim="1" priority="1"/>
      <body name="{n}_wl" pos="0 {HALF_TRACK} {WHEEL_R}">
        <joint name="{n}_jl" type="hinge" axis="0 1 0" damping="0.01" armature="0.002"/>
        <geom name="{n}_wl_g" type="cylinder" size="{WHEEL_R} 0.015" quat="0.7071068 0.7071068 0 0" rgba="0.1 0.1 0.12 1" mass="0.3" friction="1.2 0.005 0.0001"/>
      </body>
      <body name="{n}_wr" pos="0 {-HALF_TRACK} {WHEEL_R}">
        <joint name="{n}_jr" type="hinge" axis="0 1 0" damping="0.01" armature="0.002"/>
        <geom name="{n}_wr_g" type="cylinder" size="{WHEEL_R} 0.015" quat="0.7071068 0.7071068 0 0" rgba="0.1 0.1 0.12 1" mass="0.3" friction="1.2 0.005 0.0001"/>
      </body>
    </body>'''


def _plate_footprint(formation, spacing):
    """(half_x, half_y, yaw_deg) of the carried plate in the frame: it spans
    the slots and stays inside the planned hull + robot radius."""
    if formation == "square":
        h = spacing / 2 + 0.12
        return h, h, 0.0
    if formation in ("diamond", "v"):
        h = spacing / 2 + 0.12
        return h, h, 45.0
    if formation == "line":
        return 1.5 * spacing + 0.12, 0.14, 0.0
    return spacing / math.sqrt(3) * 0.75 + 0.05, spacing / 2 * 0.6, 0.0   # triangle


def _world_scene_xml(world):
    """Plain scene for an SDF world: floor + the planner's obstacle geometry."""
    g = []
    for k, b in enumerate(world.boxes):
        hz, zc = max((b.zhi - b.zlo) / 2, 0.01), (b.zhi + b.zlo) / 2
        g.append(f'    <geom name="obs_b{k}" type="box" pos="{b.c[0]:.4f} {b.c[1]:.4f} {zc:.4f}" '
                 f'size="{b.h[0]:.4f} {b.h[1]:.4f} {hz:.4f}" '
                 f'quat="{math.cos(b.yaw/2):.6f} 0 0 {math.sin(b.yaw/2):.6f}" rgba="0.55 0.45 0.3 1"/>')
    for k, c in enumerate(world.circles):
        hz, zc = max((c.zhi - c.zlo) / 2, 0.01), (c.zhi + c.zlo) / 2
        g.append(f'    <geom name="obs_c{k}" type="cylinder" pos="{c.c[0]:.4f} {c.c[1]:.4f} {zc:.4f}" '
                 f'size="{c.r:.4f} {hz:.4f}" rgba="0.8 0.15 0.15 1"/>')
    return ('<mujoco model="world">\n  <visual><headlight ambient="0.45 0.45 0.45"/></visual>\n'
            '  <worldbody>\n    <light pos="0 0 8" dir="0 0 -1" directional="true"/>\n'
            '    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.56 0.57 0.58 1"/>\n'
            + "\n".join(g) + "\n  </worldbody>\n</mujoco>\n")


def build_scene(base_xml, movers, K, formation, spacing, payload, waypoints):
    """Inject robots, movers, payload, markers, actuators and equalities."""
    wb, eq, act = [], [], []
    for i in range(1, K + 1):
        wb.append(_robot_xml(i, COLORS[i - 1]))
        for side in ("l", "r"):
            act.append(f'    <velocity name="a_robot{i}_{side}" joint="robot{i}_j{side}" '
                       f'kv="1.5" ctrlrange="-{V_WHEEL_MAX/WHEEL_R:.1f} {V_WHEEL_MAX/WHEEL_R:.1f}" '
                       f'forcerange="-2 2"/>')
    for m in movers:
        if m.shape == "circle":
            g = f'<geom name="{m.name}_g" type="cylinder" pos="0 0 0.6" size="{m.r:.3f} 0.6" rgba="0.95 0.55 0.10 1"/>'
        else:
            g = (f'<geom name="{m.name}_g" type="box" pos="0 0 0.4" '
                 f'size="{m.size[0]/2:.3f} {m.size[1]/2:.3f} 0.4" rgba="0.95 0.55 0.10 1"/>')
        wb.append(f'    <body name="{m.name}" mocap="true" pos="{m.c[0]:.3f} {m.c[1]:.3f} 0">{g}</body>')
    for k, w in enumerate(waypoints):
        last = k == len(waypoints) - 1
        wb.append(f'    <geom name="wp{k}" type="cylinder" pos="{w[0]:.3f} {w[1]:.3f} 0.004" '
                  f'size="{0.6 if last else 0.15} 0.003" contype="0" conaffinity="0" '
                  f'rgba="{"0.1 0.85 0.2 0.8" if last else "0.1 0.6 0.9 0.6"}"/>')
    if payload == "on_top":
        hx, hy, yaw = _plate_footprint(formation, spacing)
        q = f"{math.cos(math.radians(yaw)/2):.6f} 0 0 {math.sin(math.radians(yaw)/2):.6f}"
        wb.append(f'''
    <body name="payload" pos="0 0 0">
      <freejoint name="payload_free"/>
      <geom name="payload_plate" type="box" pos="0 0 0.225" size="{hx:.3f} {hy:.3f} 0.015" quat="{q}" rgba="0.62 0.64 0.66 1" mass="3.0"/>
      <geom name="payload_crate" type="box" pos="0 0 {0.24 + min(hx, hy) * 0.5:.3f}" size="{hx*0.6:.3f} {hy*0.6:.3f} {min(hx, hy)*0.5:.3f}" quat="{q}" rgba="0.74 0.60 0.40 1" mass="9.0"/>
    </body>''')
        for i in range(1, K + 1):
            eq.append(f'    <connect name="pin{i}" body1="payload" body2="robot{i}" '
                      f'anchor="0 0 0" active="false"/>')
    elif payload == "behind":
        wb.append('''
    <body name="payload" pos="0 0 0">
      <freejoint name="payload_free"/>
      <geom name="payload_sled" type="box" pos="0 0 0.22" size="0.30 0.25 0.14" rgba="0.74 0.60 0.40 1" mass="10.0"/>
      <geom name="payload_c0" type="sphere" pos="0.24 0.19 0.04" size="0.04" mass="0.1" friction="0.02 0.001 0.0001" condim="1" priority="1"/>
      <geom name="payload_c1" type="sphere" pos="0.24 -0.19 0.04" size="0.04" mass="0.1" friction="0.02 0.001 0.0001" condim="1" priority="1"/>
      <geom name="payload_c2" type="sphere" pos="-0.24 0.19 0.04" size="0.04" mass="0.1" friction="0.02 0.001 0.0001" condim="1" priority="1"/>
      <geom name="payload_c3" type="sphere" pos="-0.24 -0.19 0.04" size="0.04" mass="0.1" friction="0.02 0.001 0.0001" condim="1" priority="1"/>
      <geom name="payload_bar" type="box" pos="0.42 0 0.12" size="0.12 0.015 0.015" rgba="0.3 0.3 0.3 1" mass="0.2" contype="0" conaffinity="0"/>
    </body>''')
        eq.append('    <connect name="hitch" body1="payload" body2="robot1" anchor="0 0 0" active="false"/>')
    excl = []
    if payload == "on_top":       # the plate is carried by the pins, not by
        excl = [f'    <exclude body1="payload" body2="robot{i}{sfx}"/>'  # contact
                for i in range(1, K + 1) for sfx in ("", "_wl", "_wr")]
    xml = base_xml.replace("</worldbody>", "\n".join(wb) + "\n  </worldbody>\n"
                           "  <actuator>\n" + "\n".join(act) + "\n  </actuator>\n"
                           "  <contact>\n" + "\n".join(excl) + "\n  </contact>\n"
                           "  <equality>\n" + "\n".join(eq) + "\n  </equality>")
    head = (f'<option timestep="{DT_PHYS}" integrator="implicitfast" cone="elliptic" impratio="10"/>\n'
            '  <size memory="200M"/>\n  <visual><global offwidth="1280" offheight="720"/></visual>')
    return xml.replace("<worldbody>", head + "\n  <worldbody>", 1)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _bid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def _yaw(data, b):
    m = data.xmat[b].reshape(3, 3)
    return math.atan2(m[1, 0], m[0, 0])


def _set_free(model, data, joint, x, y, z, yaw):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    a, d = model.jnt_qposadr[j], model.jnt_dofadr[j]
    data.qpos[a:a + 7] = [x, y, z, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    data.qvel[d:d + 6] = 0.0


def _activate_connect(model, data, name, b1, b2, p_world):
    """Pin body1 to body2 at the world point p_world (zero initial error)."""
    e = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
    model.eq_obj1id[e], model.eq_obj2id[e] = b1, b2
    for k, b in enumerate((b1, b2)):
        R = data.xmat[b].reshape(3, 3)
        model.eq_data[e, 3 * k:3 * k + 3] = R.T @ (p_world - data.xpos[b])
    data.eq_active[e] = 1
    return e


def _anchor_gap(model, data, e):
    b1, b2 = model.eq_obj1id[e], model.eq_obj2id[e]
    p1 = data.xpos[b1] + data.xmat[b1].reshape(3, 3) @ model.eq_data[e, 0:3]
    p2 = data.xpos[b2] + data.xmat[b2].reshape(3, 3) @ model.eq_data[e, 3:6]
    return float(np.linalg.norm(p1 - p2))


# ---------------------------------------------------------------------------
# mission
# ---------------------------------------------------------------------------

def run(world_sdf=None, contract=None, mjcf=None, algo="pvfh_prob",
        formation="square", spacing=1.18, payload="on_top", seed=1,
        sigma0=0.02, kin=True, video=None, out_dir=None, name=None,
        timesteps=3000, verbose=True, gate="exact", hold_err=0.6,
        _debug=None):
    """One physics mission. Returns the summary dict (also written as JSON)."""
    world_sdf = world_sdf or os.path.join(WORLDS, "swarm_world_awswh.sdf")
    contract = contract or os.path.join(WORLDS, "dynamic_obstacles_awswh.json")
    if mjcf is None and os.path.basename(world_sdf) == "swarm_world_awswh.sdf":
        mjcf = os.path.join(WORLDS, "aws_warehouse.xml")
    out_dir = out_dir or os.path.join(ROOT, "results", "mujoco")
    os.makedirs(out_dir, exist_ok=True)
    name = name or f"{algo}_{formation}_{payload}_s{seed}"

    world, movers = load_scenario(world_sdf, contract)
    meta = contract_meta(contract)
    waypoints = contract_waypoints(contract)
    if waypoints is None:
        waypoints = np.array([world.goal], dtype=float)
    start = np.asarray(meta.get("start", [-12.0, 0.0]), dtype=float)
    P = make_params(kin=kin, timesteps=timesteps, gate=gate)
    rng = np.random.default_rng(seed)
    cfg = VfhConfig(sense={"kind": "lidar", "sigma0": sigma0})
    F = plan_formation(formation, spacing, start, waypoints[0],
                       scatter_radius=meta.get("scatter_radius", 2.2))
    K, r = F["K"], P["robot_radius"]

    base = open(mjcf).read() if mjcf else _world_scene_xml(world)
    model = mujoco.MjModel.from_xml_string(
        build_scene(base, movers, K, formation, spacing, payload, waypoints))
    data = mujoco.MjData(model)
    rb = [_bid(model, f"robot{i+1}") for i in range(K)]
    pb = _bid(model, "payload") if payload != "virtual" else -1
    mocap = [model.body_mocapid[_bid(model, m.name)] for m in movers]
    robot_geoms = {g for g in range(model.ngeom)
                   if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "")
                   .startswith(("robot", "payload"))}
    mover_geoms = {g for g in range(model.ngeom)
                   if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "")
                   .startswith("mover")}
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    for g in range(model.ngeom):            # ceiling decor hides the top view
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
                ("truss", "lamp")):
            model.geom_rgba[g, 3] = 0.0

    h0 = F["heading0"]
    for i in range(K):
        d = F["assigned_slots"][i] - F["starts"][i]
        _set_free(model, data, f"robot{i+1}_free", F["starts"][i][0],
                  F["starts"][i][1], 0.0, math.atan2(d[1], d[0]))
    hold = None                      # (qpos address, pose) while on the stand
    if payload == "on_top":
        _set_free(model, data, "payload_free", start[0], start[1], 0.0, h0)
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "payload_free")
        a, dq = model.jnt_qposadr[j], model.jnt_dofadr[j]
        hold = (a, dq, data.qpos[a:a + 7].copy())
    elif payload == "behind":       # parked behind the assembly point
        park = start - 2.6 * np.array([math.cos(h0), math.sin(h0)])
        _set_free(model, data, "payload_free", park[0], park[1], 0.0, h0)
    mujoco.mj_forward(model, data)

    # ---- video ----
    renderer = ffmpeg = None
    if video:
        renderer = mujoco.Renderer(model, 720, 1280)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.azimuth, cam.elevation, cam.distance = 90.0, -78.0, 10.5
        ffmpeg = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
             "rgb24", "-s", "1280x720", "-r", "20", "-i", "-", "-c:v", "libx264",
             "-preset", "fast", "-crf", "24", "-pix_fmt", "yuv420p", video],
            stdin=subprocess.PIPE)

    def frame():
        if renderer is None:
            return
        c = np.mean([data.xpos[b] for b in rb], axis=0)
        cam.lookat[:] = [c[0], c[1], 0.3]
        renderer.update_scene(data, camera=cam)
        ffmpeg.stdin.write(renderer.render().tobytes())

    def state():
        return (np.array([data.xpos[b][:2] for b in rb]),
                np.array([_yaw(data, b) for b in rb]))

    def set_cmd(i, v, w):
        data.ctrl[2 * i] = (v - w * HALF_TRACK) / WHEEL_R
        data.ctrl[2 * i + 1] = (v + w * HALF_TRACK) / WHEEL_R

    contact = {"static": 0, "mover": 0, "fmax": 0.0}
    f6 = np.zeros(6)

    def physics(t0, count):
        """Advance one control period; movers glide on the contract."""
        hs = hm = False
        for k in range(SUB):
            tk = t0 + k * DT_PHYS
            for mi, m in enumerate(movers):
                c = m.eval_at(tk).c
                data.mocap_pos[mocap[mi], 0:2] = c
            if hold is not None:               # load waits at the pick-up pose
                data.qpos[hold[0]:hold[0] + 7] = hold[2]
                data.qvel[hold[1]:hold[1] + 6] = 0.0
            mujoco.mj_step(model, data)
            if not count:
                continue
            for ci in range(data.ncon):
                con = data.contact[ci]
                g1, g2 = con.geom1, con.geom2
                if g1 == floor or g2 == floor:
                    continue
                a, b = g1 in robot_geoms, g2 in robot_geoms
                if a == b:
                    continue                   # robot-robot / world-world
                mujoco.mj_contactForce(model, data, ci, f6)
                if f6[0] < 2.0:
                    continue
                contact["fmax"] = max(contact["fmax"], float(f6[0]))
                if (g2 if a else g1) in mover_geoms:
                    hm = True
                else:
                    hs = True
        contact["static"] += hs
        contact["mover"] += hm
        return hs, hm

    rows = []
    t_wall = time.perf_counter()
    sim_t = 0.0

    def log(phase, c, yaw, dist, clr, clr_s, pos, extra):
        mt = eval_movers(movers, sim_t)
        d_phys = float(world.distance(pos).min())
        if mt:
            d_phys = min(d_phys, float(movers_distance(pos, mt).min()))
        row = {"t": sim_t, "phase": phase, "leader_x": c[0], "leader_y": c[1],
               "leader_yaw": yaw, "dist_to_goal": dist, "clearance": clr,
               "clearance_static": clr_s, "robot_gap_phys": d_phys - r}
        for i in range(K):
            row[f"r{i+1}_x"], row[f"r{i+1}_y"] = pos[i, 0], pos[i, 1]
        row.update(dict(mover_vx=0.0, mover_vy=0.0, speed_scale=1.0, mover_gap=99.0,
                        mover_gap_pred=99.0, comp_ms=0.0, reject=0, escape=0,
                        slot_err_max=0.0, slot_err_rms=0.0, hold=0, contact_static=0,
                        contact_mover=0, payload_x=float("nan"),
                        payload_y=float("nan"), payload_yaw=float("nan"),
                        pin_gap=0.0, tilt_deg=0.0))
        if pb >= 0:
            row["payload_x"], row["payload_y"] = data.xpos[pb][0], data.xpos[pb][1]
            row["payload_yaw"] = _yaw(data, pb)
            row["tilt_deg"] = math.degrees(math.acos(
                max(-1.0, min(1.0, data.xmat[pb].reshape(3, 3)[2, 2]))))
        row.update(extra)
        rows.append(row)

    # ---------------- phase 1: forming (same FormingPhase as headless) -----
    forming = FormingPhase(world, movers, F["assigned_slots"], P, cfg, algo, rng)
    for _ in range(40):                       # let the robots settle on the floor
        mujoco.mj_step(model, data)
    formed = False
    for step in range(P["forming_max_steps"]):
        pos, yaw = state()
        new_pos, new_yaw, comp = forming.step(pos, yaw, sim_t + DT_CTRL)
        for i in range(K):
            u = np.array([math.cos(new_yaw[i]), math.sin(new_yaw[i])])
            set_cmd(i, float(np.dot(new_pos[i] - pos[i], u)) / DT_CTRL,
                    wrap(new_yaw[i] - yaw[i]) / DT_CTRL)
        physics(sim_t, False)
        sim_t += DT_CTRL
        pos, yaw = state()
        errs = np.linalg.norm(pos - F["assigned_slots"], axis=1)
        clr = float(world.distance(pos).min())
        log(1, pos.mean(axis=0), float(yaw.mean()), float(errs.max()), clr, clr,
            pos, {"comp_ms": comp, "slot_err_max": float(errs.max())})
        if step % 2 == 0:
            frame()
        if float(errs.max()) < P["forming_tolerance"]:
            formed = True
            break

    # ---------------- docking: close the residual, then pick up ------------
    leader = TransportLeader(world, movers, F["robot_offsets"], P, cfg, algo, rng,
                             F["assigned_slots"].mean(axis=0), h0)
    slots = leader.slots()
    for step in range(120):
        pos, yaw = state()
        errs = np.linalg.norm(pos - slots, axis=1)
        aligned = np.abs(wrap(yaw - h0)) < 0.08
        if float(errs.max()) < 0.03 and aligned.all():
            break
        for i in range(K):
            if errs[i] > 0.03:
                set_cmd(i, *slot_tracking_cmd(pos[i], yaw[i], slots[i], np.zeros(2),
                                              v_max=0.3))
            else:
                set_cmd(i, 0.0, max(-1.5, min(1.5, 3.0 * wrap(h0 - yaw[i]))))
        physics(sim_t, False)
        sim_t += DT_CTRL
        pos, yaw = state()
        clr = float(world.distance(pos).min())
        log(1, pos.mean(axis=0), float(yaw.mean()),
            float(np.linalg.norm(pos - slots, axis=1).max()), clr, clr, pos,
            {"slot_err_max": float(np.linalg.norm(pos - slots, axis=1).max())})
        if step % 2 == 0:
            frame()
    data.ctrl[:] = 0
    dock_err = float(np.linalg.norm(state()[0] - slots, axis=1).max())
    pins, rigid = [], None
    if payload == "on_top":
        data.qpos[hold[0]:hold[0] + 7] = hold[2]
        data.qvel[hold[1]:hold[1] + 6] = 0.0
        hold = None
        mujoco.mj_forward(model, data)
        for i in range(K):
            # The pin (a slewing bearing) sits at wheel-axle height: reaction
            # forces between the robots then have almost no lever arm about
            # the wheel contact line. Pinned at the roof (0.20 m) the same
            # forces tipped the 0.28 m-track robots over.
            p = data.xpos[rb[i]] + np.array([0.0, 0.0, PIN_Z])
            pins.append(_activate_connect(model, data, f"pin{i+1}", pb, rb[i], p))
        Rp = data.xmat[pb].reshape(3, 3)[:2, :2]
        rigid = (Rp.T @ (leader.pos - data.xpos[pb][:2]),
                 wrap(_yaw(data, pb) - leader.heading))
    elif payload == "behind":
        rear = int(np.argmin(F["robot_offsets"][:, 0]))
        p_r, y_r = data.xpos[rb[rear]], h0
        hitch = p_r + np.array([-0.20 * math.cos(y_r), -0.20 * math.sin(y_r), 0.12])
        sled = hitch[:2] - 0.54 * np.array([math.cos(y_r), math.sin(y_r)])
        _set_free(model, data, "payload_free", sled[0], sled[1], 0.0, y_r)
        mujoco.mj_forward(model, data)
        pins.append(_activate_connect(model, data, "hitch", pb, rb[rear], hitch))

    # ---------------- phase 2: transport (same TransportLeader) ------------
    reached_all, n_phase2, n_hold, st = True, 0, 0, None
    hold_run = hold_cool = 0
    slot_err_hist, pin_hist, tilt_hist = [], [], []
    for wp in waypoints:
        leader.set_goal(wp)
        reached = False
        for _ in range(P["timesteps"]):
            prev_slots, prev_h, prev_c = leader.slots(), leader.heading, leader.pos.copy()
            # Formation feedback: the kinematic frame is a setpoint generator
            # with no knowledge of the robots. If they have fallen more than
            # hold_err behind (shoved by a mover, wheels saturated in a turn)
            # the frame waits for them instead of driving away.
            e_now = float(np.linalg.norm(state()[0] - prev_slots, axis=1).max())
            # The wait is bounded (HOLD_MAX steps, then HOLD_MAX steps of
            # normal motion): an unbounded wait deadlocks when the robots
            # cannot close the error by themselves (a snagged sled), and it
            # parks the formation in a mover's lane.
            held = (hold_err > 0 and e_now > hold_err and st is not None
                    and hold_run < HOLD_MAX and hold_cool == 0)
            if held:
                hold_run += 1
                if hold_run == HOLD_MAX:
                    hold_cool = HOLD_MAX
            else:
                hold_run, hold_cool = 0, max(0, hold_cool - 1)
            if held:
                n_hold += 1
                st = dict(st, reject=0, escape=0, comp_ms=0.0, reached=False)
            else:
                st = leader.step(sim_t + DT_CTRL)
            slots = leader.slots()
            v_ref = (slots - prev_slots) / DT_CTRL
            pos, yaw = state()
            if rigid is None:
                for i in range(K):
                    set_cmd(i, *slot_tracking_cmd(pos[i], yaw[i], slots[i], v_ref[i]))
            else:
                # The robots are pinned to one rigid plate: their velocities
                # must form ONE rigid-body twist or the wheels fight each
                # other. Feed back the plate's pose error against the
                # planner frame and give every robot v + w x r of that twist.
                Rp = data.xmat[pb].reshape(3, 3)[:2, :2]
                c_p = data.xpos[pb][:2] + Rp @ rigid[0]
                th_p = wrap(_yaw(data, pb) - rigid[1])
                v_tw = (leader.pos - prev_c) / DT_CTRL + \
                    _clip_norm(1.5 * (leader.pos - c_p), 0.5)
                w_tw = wrap(leader.heading - prev_h) / DT_CTRL + \
                    max(-0.6, min(0.6, 2.0 * wrap(leader.heading - th_p)))
                for i in range(K):
                    rr = pos[i] - c_p
                    set_cmd(i, *velocity_cmd(
                        yaw[i], v_tw + w_tw * np.array([-rr[1], rr[0]])))
            hs, hm = physics(sim_t, True)
            if _debug is not None:
                _debug(model, data, rb, pb, n_phase2)
            sim_t += DT_CTRL
            n_phase2 += 1
            pos, yaw = state()
            e = np.linalg.norm(pos - slots, axis=1)
            slot_err_hist.append(e)
            pg = max((_anchor_gap(model, data, q) for q in pins), default=0.0)
            pin_hist.append(pg)
            log(2, leader.pos, leader.heading, st["dist"], st["clearance"],
                st["clearance_static"], pos,
                {"speed_scale": st["v_scale"], "mover_gap": st["near"],
                 "mover_gap_pred": st["near_pre"], "comp_ms": st["comp_ms"],
                 "reject": st["reject"], "escape": st["escape"],
                 "slot_err_max": float(e.max()),
                 "slot_err_rms": float(np.sqrt((e ** 2).mean())),
                 "contact_static": int(hs), "contact_mover": int(hm),
                 "pin_gap": pg, "hold": int(held)})
            tilt_hist.append(rows[-1]["tilt_deg"])
            frame()
            if st["reached"]:
                reached = True
                break
        if not reached:
            reached_all = False
            break
    data.ctrl[:] = 0
    for _ in range(10):
        physics(sim_t, False)
        sim_t += DT_CTRL
        frame()
    if ffmpeg is not None:
        ffmpeg.stdin.close()
        ffmpeg.wait()
        renderer.close()

    # ---------------- summary ----------------
    S = summarize_run(rows, K, DT_CTRL, r, reached_all)
    pos, _ = state()
    E = np.array(slot_err_hist) if slot_err_hist else np.zeros((1, K))
    p2 = [q for q in rows if q["phase"] == 2]
    S.update(
        name=name, algo=algo, formation=formation, spacing=spacing,
        payload=payload, seed=seed, sigma0=sigma0, kin=bool(kin), formed=formed,
        dock_err=dock_err,
        final_centroid_err=float(np.linalg.norm(pos.mean(axis=0) - waypoints[-1])),
        formation_rms=float(np.sqrt((E ** 2).mean())),
        formation_err_max=float(E.max()),
        formation_err_p95=float(np.percentile(E.max(axis=1), 95)),
        min_robot_gap_phys=float(min((q["robot_gap_phys"] for q in p2), default=float("nan"))),
        contact_steps_static=int(contact["static"]),
        contact_steps_mover=int(contact["mover"]),
        contact_force_max=float(contact["fmax"]),
        pin_gap_max=float(max(pin_hist, default=0.0)),
        hold_steps=int(n_hold), hold_err=hold_err,
        payload_tilt_max_deg=float(max(tilt_hist, default=0.0)),
        wall_s=time.perf_counter() - t_wall, world=os.path.basename(world_sdf))
    hdr = list(rows[0].keys())
    with open(os.path.join(out_dir, name + ".csv"), "w") as fh:
        fh.write(",".join(hdr) + "\n")
        for q in rows:
            fh.write(",".join(f"{float(q[h]):.6f}" for h in hdr) + "\n")
    with open(os.path.join(out_dir, name + ".json"), "w") as fh:
        json.dump(S, fh, indent=1)
    if verbose:
        print(f"{name}: reached={S['reached_goal']} t={S['total_time']:.1f}s "
              f"planner[minclr={S['min_clearance']:.2f} rej={S['n_reject']} "
              f"coll_m={S['coll_mover']}] physics[rms={S['formation_rms']*100:.1f}cm "
              f"max={S['formation_err_max']*100:.1f}cm gap={S['min_robot_gap_phys']:.2f}m "
              f"contact s/m={S['contact_steps_static']}/{S['contact_steps_mover']} "
              f"hold={S['hold_steps']} pin={S['pin_gap_max']*1000:.1f}mm tilt={S['payload_tilt_max_deg']:.1f}deg] "
              f"wall={S['wall_s']:.0f}s", flush=True)
    return S


MATRIX = ([("square", 1.18, p) for p in ("virtual", "on_top", "behind")]
          + [(f, 0.60, p) for f in ("diamond", "triangle", "line")
             for p in ("on_top", "behind")])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default=None, help="SDF world (default: the AWS warehouse slice)")
    ap.add_argument("--contract", default=None)
    ap.add_argument("--algo", default="pvfh_prob", choices=list(ALGOS))
    ap.add_argument("--formation", default="square",
                    choices=["square", "line", "triangle", "diamond"])
    ap.add_argument("--spacing", type=float, default=None)
    ap.add_argument("--payload", default="on_top", choices=["virtual", "on_top", "behind"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sigma0", type=float, default=0.02)
    ap.add_argument("--no-kin", action="store_true",
                    help="disable the load-carrying motion limits")
    ap.add_argument("--no-hold", action="store_true",
                    help="open-loop frame: never wait for the robots")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--matrix", action="store_true",
                    help="formations x payload modes for --algo")
    ap.add_argument("--output", default=None)
    a = ap.parse_args(argv)
    out = a.output or os.path.join(ROOT, "results", "mujoco")
    os.makedirs(out, exist_ok=True)
    jobs = MATRIX if a.matrix else [
        (a.formation, a.spacing or (1.18 if a.formation == "square" else 0.60), a.payload)]
    ok = True
    for formation, spacing, payload in jobs:
        name = f"{a.algo}_{formation}_{payload}_s{a.seed}" + ("_nokin" if a.no_kin else "")
        S = run(a.world, a.contract, algo=a.algo, formation=formation,
                spacing=spacing, payload=payload, seed=a.seed, sigma0=a.sigma0,
                kin=not a.no_kin, out_dir=out, name=name,
                hold_err=0.0 if a.no_hold else 0.6,
                video=os.path.join(out, name + ".mp4") if a.video else None)
        ok &= S["reached_goal"]
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
