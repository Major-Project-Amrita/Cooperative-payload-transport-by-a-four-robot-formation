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
from .transport import (MissionController, eval_movers,  # noqa: E402
                        make_params)
from .vfh import ALGOS, VfhConfig                    # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(PKG, "worlds")
ROOT = os.path.dirname(os.path.dirname(PKG))
if not os.path.isdir(WORLDS):          # installed with colcon: worlds are in share/
    try:
        from ament_index_python.packages import get_package_share_directory
        WORLDS = os.path.join(get_package_share_directory("ros2_swarm_transport"), "worlds")
        ROOT = os.getcwd()
    except Exception:                                   # noqa: BLE001
        pass

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
# the plant: MuJoCo side only (no mission logic)
# ---------------------------------------------------------------------------

class MujocoPlant:
    """The physical world: robots, movers, payload, contacts, rendering.
    It knows nothing about the mission; it applies wheel commands and reports
    poses. Used in-process by run() and behind ROS topics by
    mujoco_ros_bridge.py."""

    def __init__(self, world, movers, F, start, waypoints, formation, spacing,
                 payload, mjcf=None, video=None, gui=False):
        self.movers, self.F, self.payload, self.K = movers, F, payload, F["K"]
        self.viewer, self._t_wall = None, None
        K = self.K
        base = open(mjcf).read() if mjcf else _world_scene_xml(world)
        self.model = model = mujoco.MjModel.from_xml_string(
            build_scene(base, movers, K, formation, spacing, payload, waypoints))
        self.data = data = mujoco.MjData(model)
        self.rb = [_bid(model, f"robot{i+1}") for i in range(K)]
        self.pb = _bid(model, "payload") if payload != "virtual" else -1
        self.mocap = [model.body_mocapid[_bid(model, m.name)] for m in movers]
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
                 for g in range(model.ngeom)]
        self.robot_geoms = {g for g, n in enumerate(names) if n.startswith(("robot", "payload"))}
        self.mover_geoms = {g for g, n in enumerate(names) if n.startswith("mover")}
        self.floor = names.index("floor")
        for g, n in enumerate(names):           # ceiling decor hides the top view
            if n.startswith(("truss", "lamp")):
                model.geom_rgba[g, 3] = 0.0
        self.h0 = h0 = F["heading0"]
        for i in range(K):
            d = F["assigned_slots"][i] - F["starts"][i]
            _set_free(model, data, f"robot{i+1}_free", F["starts"][i][0],
                      F["starts"][i][1], 0.0, math.atan2(d[1], d[0]))
        self.hold = None                 # (qpos adr, dof adr, pose) on the stand
        if payload == "on_top":
            _set_free(model, data, "payload_free", start[0], start[1], 0.0, h0)
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "payload_free")
            a, dq = model.jnt_qposadr[j], model.jnt_dofadr[j]
            self.hold = (a, dq, data.qpos[a:a + 7].copy())
        elif payload == "behind":        # parked behind the assembly point
            park = start - 2.6 * np.array([math.cos(h0), math.sin(h0)])
            _set_free(model, data, "payload_free", park[0], park[1], 0.0, h0)
        mujoco.mj_forward(model, data)
        self.pins = []
        self.contact = {"static": 0, "mover": 0, "fmax": 0.0}
        self._f6 = np.zeros(6)
        self.renderer = self.ffmpeg = None
        if video:
            self.renderer = mujoco.Renderer(model, 720, 1280)
            self.cam = cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.azimuth, cam.elevation, cam.distance = 90.0, -78.0, 10.5
            self.ffmpeg = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
                 "rgb24", "-s", "1280x720", "-r", "20", "-i", "-", "-c:v", "libx264",
                 "-preset", "fast", "-crf", "24", "-pix_fmt", "yuv420p", video],
                stdin=subprocess.PIPE)
        for _ in range(40):                       # let the robots settle on the floor
            mujoco.mj_step(model, data)
        if gui:                                   # interactive window, real time
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(model, data)
            self.viewer.cam.azimuth, self.viewer.cam.elevation = 90.0, -60.0
            self.viewer.cam.distance = 12.0
            self._follow()
            self.viewer.sync()

    # ---- sensing ----
    def state(self):
        return (np.array([self.data.xpos[b][:2] for b in self.rb]),
                np.array([_yaw(self.data, b) for b in self.rb]))

    def plate(self):
        """(xy, yaw) of the payload body, or None."""
        if self.pb < 0:
            return None
        return self.data.xpos[self.pb][:2].copy(), _yaw(self.data, self.pb)

    def tilt_deg(self):
        if self.pb < 0:
            return 0.0
        return math.degrees(math.acos(max(-1.0, min(
            1.0, self.data.xmat[self.pb].reshape(3, 3)[2, 2]))))

    def pin_gap(self):
        return max((_anchor_gap(self.model, self.data, q) for q in self.pins), default=0.0)

    # ---- actuation ----
    def set_cmds(self, cmds):
        for i, (v, w) in enumerate(cmds):
            self.data.ctrl[2 * i] = (v - w * HALF_TRACK) / WHEEL_R
            self.data.ctrl[2 * i + 1] = (v + w * HALF_TRACK) / WHEEL_R

    def attach(self):
        """Pick the load up (on_top: pin it to every robot; behind: hitch the
        sled to the rear robot)."""
        model, data, rb, pb = self.model, self.data, self.rb, self.pb
        data.ctrl[:] = 0
        if self.payload == "on_top":
            data.qpos[self.hold[0]:self.hold[0] + 7] = self.hold[2]
            data.qvel[self.hold[1]:self.hold[1] + 6] = 0.0
            self.hold = None
            mujoco.mj_forward(model, data)
            for i in range(self.K):
                # The pin (a slewing bearing) sits at wheel-axle height: reaction
                # forces between the robots then have almost no lever arm about
                # the wheel contact line. Pinned at the roof (0.20 m) the same
                # forces tipped the 0.28 m-track robots over.
                p = data.xpos[rb[i]] + np.array([0.0, 0.0, PIN_Z])
                self.pins.append(_activate_connect(model, data, f"pin{i+1}", pb, rb[i], p))
        elif self.payload == "behind":
            rear = int(np.argmin(self.F["robot_offsets"][:, 0]))
            p_r, y_r = data.xpos[rb[rear]], self.h0
            hitch = p_r + np.array([-0.20 * math.cos(y_r), -0.20 * math.sin(y_r), 0.12])
            sled = hitch[:2] - 0.54 * np.array([math.cos(y_r), math.sin(y_r)])
            _set_free(model, data, "payload_free", sled[0], sled[1], 0.0, y_r)
            mujoco.mj_forward(model, data)
            self.pins.append(_activate_connect(model, data, "hitch", pb, rb[rear], hitch))

    def advance(self, t0, count):
        """One control period of physics from time t0; movers glide on the
        contract. Returns (static contact, mover contact) flags (> 2 N)."""
        model, data = self.model, self.data
        hs = hm = False
        for k in range(SUB):
            tk = t0 + k * DT_PHYS
            for mi, m in enumerate(self.movers):
                data.mocap_pos[self.mocap[mi], 0:2] = m.eval_at(tk).c
            if self.hold is not None:              # load waits at the pick-up pose
                data.qpos[self.hold[0]:self.hold[0] + 7] = self.hold[2]
                data.qvel[self.hold[1]:self.hold[1] + 6] = 0.0
            mujoco.mj_step(model, data)
            if not count:
                continue
            for ci in range(data.ncon):
                con = data.contact[ci]
                g1, g2 = con.geom1, con.geom2
                if g1 == self.floor or g2 == self.floor:
                    continue
                a, b = g1 in self.robot_geoms, g2 in self.robot_geoms
                if a == b:
                    continue                   # robot-robot / world-world
                mujoco.mj_contactForce(model, data, ci, self._f6)
                if self._f6[0] < 2.0:
                    continue
                self.contact["fmax"] = max(self.contact["fmax"], float(self._f6[0]))
                if (g2 if a else g1) in self.mover_geoms:
                    hm = True
                else:
                    hs = True
        self.contact["static"] += hs
        self.contact["mover"] += hm
        return hs, hm

    # ---- video / GUI ----
    def _follow(self):
        c = np.mean([self.data.xpos[b] for b in self.rb], axis=0)
        self.viewer.cam.lookat[:] = [c[0], c[1], 0.3]

    def frame(self):
        if self.viewer is not None and self.viewer.is_running():
            self._follow()                         # camera tracks the formation
            self.viewer.sync()
            now = time.perf_counter()              # pace the window at real time
            if self._t_wall is not None:
                time.sleep(max(0.0, DT_CTRL - (now - self._t_wall)))
            self._t_wall = time.perf_counter()
        if self.renderer is None:
            return
        c = np.mean([self.data.xpos[b] for b in self.rb], axis=0)
        self.cam.lookat[:] = [c[0], c[1], 0.3]
        self.renderer.update_scene(self.data, camera=self.cam)
        self.ffmpeg.stdin.write(self.renderer.render().tobytes())

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            time.sleep(0.5)                        # let the render thread stop
            self.viewer = None
        if self.ffmpeg is not None:
            self.ffmpeg.stdin.close()
            self.ffmpeg.wait()
            self.renderer.close()
            self.ffmpeg = self.renderer = None


# ---------------------------------------------------------------------------
# mission = MissionController (transport.py) + MujocoPlant
# ---------------------------------------------------------------------------

def setup(world_sdf=None, contract=None, mjcf=None, algo="pvfh_prob",
          formation="square", spacing=1.18, payload="on_top", seed=1,
          sigma0=0.02, kin=True, video=None, timesteps=3000, gate="exact",
          hold_err=0.6, gui=False):
    """Build the scenario, the controller and the plant (shared by run() and
    the ROS 2 nodes)."""
    world_sdf = world_sdf or os.path.join(WORLDS, "swarm_world_awswh.sdf")
    contract = contract or os.path.join(WORLDS, "dynamic_obstacles_awswh.json")
    if mjcf is None and os.path.basename(world_sdf) == "swarm_world_awswh.sdf":
        mjcf = os.path.join(WORLDS, "aws_warehouse.xml")
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
    S = dict(world=world, movers=movers, waypoints=waypoints, start=start, P=P,
             F=F, world_sdf=world_sdf, mjcf=mjcf, formation=formation,
             spacing=spacing, payload=payload, algo=algo, seed=seed,
             sigma0=sigma0, kin=kin, hold_err=hold_err, video=video)
    S["make_controller"] = lambda: MissionController(
        world, movers, F, waypoints, P, cfg, algo, rng, payload, hold_err)
    S["make_plant"] = lambda: MujocoPlant(world, movers, F, start, waypoints,
                                          formation, spacing, payload, mjcf, video, gui)
    return S


class MissionLog:
    """Per-tick log rows and the run summary (same for the direct and the
    ROS 2 run)."""

    def __init__(self, S):
        self.S, self.rows = S, []
        self.E, self.pin_hist, self.tilt_hist = [], [], []

    def add(self, sim_t, ctrl, plant, hs=False, hm=False, comp_ms=0.0):
        S = self.S
        K, r, world = ctrl.K, S["P"]["robot_radius"], S["world"]
        pos, yaw = plant.state()
        mt = eval_movers(S["movers"], sim_t)
        d_phys = float(world.distance(pos).min())
        if mt:
            d_phys = min(d_phys, float(movers_distance(pos, mt).min()))
        slots = ctrl.slots()
        e = np.linalg.norm(pos - slots, axis=1)
        st = ctrl.st if ctrl.phase == 2 else None
        if st is None:
            clr = float(world.distance(pos).min())
            row = {"t": sim_t, "phase": 1, "leader_x": pos[:, 0].mean(),
                   "leader_y": pos[:, 1].mean(), "leader_yaw": float(yaw.mean()),
                   "dist_to_goal": float(e.max()), "clearance": clr,
                   "clearance_static": clr}
        else:
            L = ctrl.leader
            row = {"t": sim_t, "phase": 2, "leader_x": L.pos[0], "leader_y": L.pos[1],
                   "leader_yaw": L.heading, "dist_to_goal": st["dist"],
                   "clearance": st["clearance"], "clearance_static": st["clearance_static"]}
        row["robot_gap_phys"] = d_phys - r
        for i in range(K):
            row[f"r{i+1}_x"], row[f"r{i+1}_y"] = pos[i, 0], pos[i, 1]
        pl = plant.plate()
        row.update(dict(
            mover_vx=0.0, mover_vy=0.0,
            speed_scale=st["v_scale"] if st else 1.0,
            mover_gap=st["near"] if st else 99.0,
            mover_gap_pred=st["near_pre"] if st else 99.0,
            comp_ms=comp_ms, reject=st["reject"] if st else 0,
            escape=st["escape"] if st else 0, slot_err_max=float(e.max()),
            slot_err_rms=float(np.sqrt((e ** 2).mean())) if st else 0.0,
            hold=int(ctrl.held) if st else 0, contact_static=int(hs),
            contact_mover=int(hm),
            payload_x=pl[0][0] if pl else float("nan"),
            payload_y=pl[0][1] if pl else float("nan"),
            payload_yaw=pl[1] if pl else float("nan"),
            pin_gap=plant.pin_gap() if st else 0.0, tilt_deg=plant.tilt_deg()))
        self.rows.append(row)
        if st is not None:
            self.E.append(e)
            self.pin_hist.append(row["pin_gap"])
            self.tilt_hist.append(row["tilt_deg"])

    def finish(self, ctrl, plant, name, out_dir, wall_s, verbose=True):
        S0, rows = self.S, self.rows
        K, r = ctrl.K, S0["P"]["robot_radius"]
        S = summarize_run(rows, K, DT_CTRL, r, bool(ctrl.reached_all))
        pos, _ = plant.state()
        E = np.array(self.E) if self.E else np.zeros((1, K))
        p2 = [q for q in rows if q["phase"] == 2]
        S.update(
            name=name, algo=S0["algo"], formation=S0["formation"],
            spacing=S0["spacing"], payload=S0["payload"], seed=S0["seed"],
            sigma0=S0["sigma0"], kin=bool(S0["kin"]), formed=ctrl.formed,
            dock_err=ctrl.dock_err,
            final_centroid_err=float(np.linalg.norm(pos.mean(axis=0) - S0["waypoints"][-1])),
            formation_rms=float(np.sqrt((E ** 2).mean())),
            formation_err_max=float(E.max()),
            formation_err_p95=float(np.percentile(E.max(axis=1), 95)),
            min_robot_gap_phys=float(min((q["robot_gap_phys"] for q in p2), default=float("nan"))),
            contact_steps_static=int(plant.contact["static"]),
            contact_steps_mover=int(plant.contact["mover"]),
            contact_force_max=float(plant.contact["fmax"]),
            pin_gap_max=float(max(self.pin_hist, default=0.0)),
            hold_steps=int(ctrl.n_hold), hold_err=S0["hold_err"],
            payload_tilt_max_deg=float(max(self.tilt_hist, default=0.0)),
            wall_s=wall_s, world=os.path.basename(S0["world_sdf"]))
        os.makedirs(out_dir, exist_ok=True)
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
                  f"hold={S['hold_steps']} pin={S['pin_gap_max']*1000:.1f}mm "
                  f"tilt={S['payload_tilt_max_deg']:.1f}deg] wall={S['wall_s']:.0f}s", flush=True)
        return S


def run(world_sdf=None, contract=None, mjcf=None, algo="pvfh_prob",
        formation="square", spacing=1.18, payload="on_top", seed=1,
        sigma0=0.02, kin=True, video=None, out_dir=None, name=None,
        timesteps=3000, verbose=True, gate="exact", hold_err=0.6, gui=False):
    """One physics mission, controller and plant in one process. Returns the
    summary dict (also written as JSON)."""
    out_dir = out_dir or os.path.join(ROOT, "results", "mujoco")
    name = name or f"{algo}_{formation}_{payload}_s{seed}"
    S = setup(world_sdf, contract, mjcf, algo, formation, spacing, payload, seed,
              sigma0, kin, video, timesteps, gate, hold_err, gui)
    ctrl, plant = S["make_controller"](), S["make_plant"]()
    log = MissionLog(S)
    t_wall = time.perf_counter()
    sim_t = 0.0
    while True:
        pos, yaw = plant.state()
        out = ctrl.step(sim_t, pos, yaw, plant.plate())
        if out["attach"]:
            plant.attach()
        plant.set_cmds(out["cmds"])
        hs, hm = plant.advance(sim_t, ctrl.phase == 2)
        sim_t += DT_CTRL
        log.add(sim_t, ctrl, plant, hs, hm, out["comp_ms"])
        if ctrl.phase == 2 or len(log.rows) % 2 == 0 or plant.viewer is not None:
            plant.frame()
        if ctrl.mode == "finishing":
            break
    plant.set_cmds([(0.0, 0.0)] * ctrl.K)          # stop and settle for 1 s
    for _ in range(10):
        plant.advance(sim_t, False)
        sim_t += DT_CTRL
        plant.frame()
    plant.close()
    return log.finish(ctrl, plant, name, out_dir, time.perf_counter() - t_wall, verbose)


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
    ap.add_argument("--gui", action="store_true",
                    help="open the interactive MuJoCo viewer and run in real time")
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
                hold_err=0.0 if a.no_hold else 0.6, gui=a.gui,
                video=os.path.join(out, name + ".mp4") if a.video else None)
        ok &= S["reached_goal"]
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
