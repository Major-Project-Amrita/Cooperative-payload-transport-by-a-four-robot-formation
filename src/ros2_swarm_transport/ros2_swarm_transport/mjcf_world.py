"""MJCF <-> 2D world model.

world_from_mjcf  -- slice the collision geometry of a MuJoCo scene (static
                    worldbody geoms: box / cylinder) at the robot height band,
                    exactly as sdf2world.m slices an SDF, and return the World
                    the planner uses.
world_to_sdf     -- write that World as an SDF file of static box / cylinder
                    models (+ goal_pad + mover placeholders), so the MATLAB
                    reference (sdf2world.m) and the Python stack load the SAME
                    geometry for the validation warehouse.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import Box, Circle, World


def _floats(s, n=None, default=None):
    if s is None:
        return default
    v = [float(x) for x in s.split()]
    return v if n is None else (v + [0.0] * n)[:n]


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def world_from_mjcf(path, robot_height=0.35, z_floor=0.03, name="aws_warehouse",
                    skip_prefixes=()):
    """Static collision geoms directly under <worldbody>, sliced at the
    z-band [z_floor, robot_height]. Geoms with contype="0" (decor) and planes
    are skipped. Boxes may be yawed about z; a cylinder lying on its side is
    represented by its bounding box."""
    root = ET.parse(path).getroot()
    wb = root.find("worldbody")
    boxes, circles = [], []
    for g in wb.findall("geom"):
        gname = g.get("name", "geom")
        if g.get("contype") == "0" or g.get("type", "sphere") == "plane":
            continue
        if any(gname.startswith(p) for p in skip_prefixes):
            continue
        pos = np.array(_floats(g.get("pos"), 3, [0.0, 0.0, 0.0]))
        size = _floats(g.get("size"))
        R = np.eye(3)
        if g.get("quat"):
            R = _quat_to_mat(_floats(g.get("quat"), 4))
        elif g.get("euler"):
            ez = _floats(g.get("euler"), 3)[2]
            R = np.array([[math.cos(ez), -math.sin(ez), 0],
                          [math.sin(ez), math.cos(ez), 0], [0, 0, 1]])
        typ = g.get("type")
        if typ == "box":
            if abs(R[2, 2]) < 0.999:
                raise ValueError(f"{gname}: tilted boxes are not supported")
            yaw = math.atan2(R[1, 0], R[0, 0])
            zl, zh = pos[2] - size[2], pos[2] + size[2]
            if zh > z_floor and zl < robot_height:
                boxes.append(Box(pos[:2].copy(), yaw, np.array(size[:2]), gname, zl, zh))
        elif typ == "cylinder":
            r, hl = size[0], size[1]
            axis = R[:, 2]
            if abs(axis[2]) > 0.999:
                zl, zh = pos[2] - hl, pos[2] + hl
                if zh > z_floor and zl < robot_height:
                    circles.append(Circle(pos[:2].copy(), r, gname, zl, zh))
            else:                                    # lying on its side
                zl, zh = pos[2] - r, pos[2] + r
                if zh > z_floor and zl < robot_height:
                    yaw = math.atan2(axis[1], axis[0])
                    boxes.append(Box(pos[:2].copy(), yaw, np.array([hl, r]),
                                     gname, zl, zh))
    return World(boxes=boxes, circles=circles, name=name)


def world_to_sdf(world: World, path, goal, goal_r=1.2, movers=(), comment=""):
    """Write the sliced world as static SDF models (collision + visual)."""
    out = ['<?xml version="1.0"?>', '<sdf version="1.9">',
           f'  <!-- {comment} -->', f'  <world name="{world.name}">',
           '    <model name="ground_plane"><static>true</static><link name="l">'
           '<collision name="c"><geometry><plane><normal>0 0 1</normal>'
           '<size>60 60</size></plane></geometry></collision></link></model>']

    def model(name, x, y, z, yaw, geom):
        return (f'    <model name="{name}"><static>true</static>'
                f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose><link name="l">'
                f'<collision name="c"><geometry>{geom}</geometry></collision>'
                f'<visual name="v"><geometry>{geom}</geometry></visual></link></model>')

    for b in world.boxes:
        zc, hz = (b.zlo + b.zhi) / 2, (b.zhi - b.zlo)
        out.append(model(b.name, b.c[0], b.c[1], zc, b.yaw,
                         f'<box><size>{2*b.h[0]:.4f} {2*b.h[1]:.4f} {hz:.4f}</size></box>'))
    for c in world.circles:
        zc, hz = (c.zlo + c.zhi) / 2, (c.zhi - c.zlo)
        out.append(model(c.name, c.c[0], c.c[1], zc, 0.0,
                         f'<cylinder><radius>{c.r:.4f}</radius><length>{hz:.4f}</length></cylinder>'))
    out.append(f'    <model name="goal_pad"><static>true</static>'
               f'<pose>{goal[0]:.4f} {goal[1]:.4f} 0.005 0 0 0</pose><link name="l">'
               f'<visual name="v"><geometry><cylinder><radius>{goal_r:.2f}</radius>'
               f'<length>0.01</length></cylinder></geometry></visual></link></model>')
    for m in movers:
        if m.shape == "circle":
            geom = f'<cylinder><radius>{m.r:.3f}</radius><length>1.0</length></cylinder>'
        else:
            geom = f'<box><size>{m.size[0]:.3f} {m.size[1]:.3f} 1.0</size></box>'
        out.append(model(m.name, m.route[0][0], m.route[0][1], 0.5, 0.0, geom))
    out += ['  </world>', '</sdf>']
    with open(path, "w") as fh:
        fh.write("\n".join(out) + "\n")
