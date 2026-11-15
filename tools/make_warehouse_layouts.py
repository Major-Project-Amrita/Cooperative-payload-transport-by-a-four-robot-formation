#!/usr/bin/env python3
"""Seeded generator for the extended warehouse layouts (experiment E15).

    python3 tools/make_warehouse_layouts.py          # writes worlds/swarm_world_wh_*.sdf
                                                     #    + dynamic_obstacles_wh_*.json

Six layouts that differ in size and in structure, not only in obstacle
count. Each is written as a Gazebo SDF (the same schema as every other
world, so MATLAB's sdf2world, the Python stack and the MuJoCo converter
read it) plus a motion contract with start, global waypoints and traffic.

  wh_s_racks      30 x 20 m   3 rack rows, 3.2 m aisles, 2 forklifts
  wh_m_racks      45 x 30 m   5 rack rows, 3.0 m aisles, cross-aisle, 4 forklifts + 2 walkers
  wh_l_racks      60 x 40 m   7 rack rows, 2.8 m aisles, 2 cross-aisles, 6 forklifts + 3 walkers
  wh_xl_racks     80 x 50 m   9 rack rows, 3.0 m aisles, 2 cross-aisles, 9 forklifts + 4 walkers
  wh_m_narrow     45 x 30 m   6 rack rows, 2.4 m aisles (tight for the 2.03 m swept body)
  wh_m_blockstack 45 x 30 m   block-stacked pallets at random + column grid, 5 forklifts

The global route is an A* path on a 0.2 m grid over cells whose clearance
exceeds the formation's swept radius (1.014 m for the 2x2 block at
s = 1.18 m, plus 0.15 m), pruned to line-of-sight waypoints. A layout is
rejected and redrawn until such a route exists, so every mission is
feasible for the reference formation; whether a planner completes it is
the experiment. Movers are axis-aligned and shuttle along aisles
(forklifts, 1.1 x 0.8 m boxes) or across them (walkers, r = 0.3 m).
"""
import heapq
import json
import math
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "src", "ros2_swarm_transport", "worlds")
SWEPT = 1.014 + 0.15          # body swept radius + planning margin (m)

HEAD = """<?xml version="1.0" ?>
<!--
  {name}: generated warehouse layout ({desc}), {Lx:.0f} m x {Ly:.0f} m,
  seed {seed}. Written by tools/make_warehouse_layouts.py; never edited by
  hand. {nobs} static obstacles. Start {start}, goal pad {goal}.
-->
<sdf version="1.8">
  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{G} {G}</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{G} {G}</size></plane></geometry></visual></link></model>
"""


def box_sdf(n, x, y, sx, sy, h=1.5, col="0.45 0.45 0.5"):
    return (f'    <model name="{n}"><static>true</static><pose>{x:.3f} {y:.3f} {h/2} 0 0 0</pose><link name="l">\n'
            f'      <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {h}</size></box></geometry></collision>\n'
            f'      <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {h}</size></box></geometry>\n'
            f'        <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual></link></model>\n')


def cyl_sdf(n, x, y, r, h=1.5, col="0.3 0.3 0.6"):
    return (f'    <model name="{n}"><static>true</static><pose>{x:.3f} {y:.3f} {h/2} 0 0 0</pose><link name="l">\n'
            f'      <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>\n'
            f'      <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>\n'
            f'        <material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual></link></model>\n')


def sdist(P, boxes, circles):
    """Signed distance of points P (N,2) to axis-aligned boxes + circles."""
    P = np.atleast_2d(P)
    d = np.full(len(P), np.inf)
    for (x, y, sx, sy) in boxes:
        qx = np.abs(P[:, 0] - x) - sx / 2
        qy = np.abs(P[:, 1] - y) - sy / 2
        d = np.minimum(d, np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
                       + np.minimum(np.maximum(qx, qy), 0))
    for (x, y, r) in circles:
        d = np.minimum(d, np.hypot(P[:, 0] - x, P[:, 1] - y) - r)
    return d


def astar_route(boxes, circles, Lx, Ly, start, goal, res=0.2):
    xs = np.arange(-Lx / 2, Lx / 2 + 1e-9, res)
    ys = np.arange(-Ly / 2, Ly / 2 + 1e-9, res)
    X, Y = np.meshgrid(xs, ys)
    D = sdist(np.c_[X.ravel(), Y.ravel()], boxes, circles).reshape(X.shape)
    free = D > SWEPT
    ij = lambda p: (int(round((p[1] + Ly / 2) / res)), int(round((p[0] + Lx / 2) / res)))  # noqa: E731
    s, g = ij(start), ij(goal)
    if not (free[s] and free[g]):
        return None, D
    nb = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
          (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    # mild clearance preference keeps the route in the aisle centre
    pen = 0.6 * np.clip(2.0 - D, 0, None)
    best = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    while pq:
        f, u = heapq.heappop(pq)
        if u == g:
            break
        gu = best[u]
        for di, dj, c in nb:
            v = (u[0] + di, u[1] + dj)
            if not (0 <= v[0] < free.shape[0] and 0 <= v[1] < free.shape[1]) or not free[v]:
                continue
            gv = gu + c * (1 + pen[v])
            if gv < best.get(v, np.inf):
                best[v] = gv
                prev[v] = u
                h = math.hypot(v[0] - g[0], v[1] - g[1])
                heapq.heappush(pq, (gv + h, v))
    if g not in prev:
        return None, D
    path = [g]
    while path[-1] != s:
        path.append(prev[path[-1]])
    path = path[::-1]
    P = np.array([[xs[j], ys[i]] for i, j in path])
    # line-of-sight pruning with a clearance-checked segment
    wps = [P[0]]
    k = 0
    while k < len(P) - 1:
        nxt = k + 1
        for m in range(len(P) - 1, k, -1):
            seg = np.linspace(P[k], P[m], max(2, int(np.linalg.norm(P[m] - P[k]) / 0.1)))
            if sdist(seg, boxes, circles).min() > SWEPT:
                nxt = m
                break
        wps.append(P[nxt])
        k = nxt
    return np.array(wps[1:]), D


def rack_layout(rng, Lx, Ly, n_rows, aisle, n_cross, rack_depth=1.2):
    """Parallel rack rows along x, separated by aisles, cut by cross-aisles."""
    boxes, circles = [], []
    x0, x1 = -Lx / 2 + 6.0, Lx / 2 - 5.0          # docks at both ends
    pitch = rack_depth + aisle
    total = n_rows * rack_depth + (n_rows - 1) * aisle
    y_first = -total / 2 + rack_depth / 2
    cuts = np.linspace(x0, x1, n_cross + 2)[1:-1] if n_cross else []
    cross_w = max(aisle, 3.0)
    for r in range(n_rows):
        y = y_first + r * pitch
        edges = [x0] + sum(([c - cross_w / 2, c + cross_w / 2] for c in cuts), []) + [x1]
        for a, b in zip(edges[0::2], edges[1::2]):
            # split each run into bays of ~4.8 m with small gaps
            n_bay = max(1, int(round((b - a) / 4.8)))
            w = (b - a) / n_bay
            for k in range(n_bay):
                boxes.append((a + (k + 0.5) * w, y, w - 0.1, rack_depth))
    # building columns in the dock zones
    for x in (-Lx / 2 + 2.5, Lx / 2 - 2.5):
        for y in np.arange(-Ly / 2 + 4.0, Ly / 2 - 3.9, 8.0):
            if abs(y) > 2.0:
                circles.append((x, y, 0.25))
    rows_y = [y_first + r * pitch for r in range(n_rows)]
    aisles_y = [(rows_y[r] + rows_y[r + 1]) / 2 for r in range(n_rows - 1)]
    return boxes, circles, aisles_y, list(cuts), (x0, x1)


def blockstack_layout(rng, Lx, Ly):
    boxes, circles = [], []
    for x in np.arange(-Lx / 2 + 6, Lx / 2 - 5, 7.5):          # column grid
        for y in np.arange(-Ly / 2 + 4, Ly / 2 - 3.9, 7.5):
            circles.append((float(x), float(y), 0.3))
    tries = 0
    while len(boxes) < 26 and tries < 5000:
        tries += 1
        sx, sy = rng.choice([2.4, 3.6, 4.8]), rng.choice([1.2, 2.4])
        if rng.random() < 0.5:
            sx, sy = sy, sx
        x = rng.uniform(-Lx / 2 + 7, Lx / 2 - 6)
        y = rng.uniform(-Ly / 2 + 2.5, Ly / 2 - 2.5)
        if sdist(np.array([[x, y]]), boxes, circles)[0] < max(sx, sy) / 2 + 2.6:
            continue
        boxes.append((round(x, 2), round(y, 2), sx, sy))
    return boxes, circles


def mover(name, shape, a, b, speed, t0, wait, size=None):
    if shape == "circle":
        return {"name": name, "shape": "circle", "size": [0.6, 0.6], "radius": 0.3,
                "route": [list(map(lambda v: round(float(v), 2), a)),
                          list(map(lambda v: round(float(v), 2), b))],
                "speed": round(float(speed), 2), "t0": round(float(t0), 1),
                "wait": float(wait), "loop": True}
    return {"name": name, "shape": "box", "size": size, "radius": 0.0,
            "route": [list(map(lambda v: round(float(v), 2), a)),
                      list(map(lambda v: round(float(v), 2), b))],
            "speed": round(float(speed), 2), "t0": round(float(t0), 1),
            "wait": float(wait), "loop": True}


SPECS = [
    dict(name="wh_s_racks", desc="3 rack rows, 3.2 m aisles", Lx=30, Ly=20, kind="racks",
         rows=3, aisle=3.2, cross=1, n_fork=2, n_walk=0, seed=2101),
    dict(name="wh_m_racks", desc="5 rack rows, 3.0 m aisles, 1 cross-aisle", Lx=45, Ly=30,
         kind="racks", rows=5, aisle=3.0, cross=1, n_fork=4, n_walk=2, seed=2102),
    dict(name="wh_l_racks", desc="7 rack rows, 2.8 m aisles, 2 cross-aisles", Lx=60, Ly=40,
         kind="racks", rows=7, aisle=2.8, cross=2, n_fork=6, n_walk=3, seed=2103),
    dict(name="wh_xl_racks", desc="9 rack rows, 3.0 m aisles, 2 cross-aisles", Lx=80, Ly=50,
         kind="racks", rows=9, aisle=3.0, cross=2, n_fork=9, n_walk=4, seed=2104),
    dict(name="wh_m_narrow", desc="6 rack rows, 2.4 m narrow aisles, 1 cross-aisle", Lx=45, Ly=30,
         kind="racks", rows=6, aisle=2.4, cross=1, n_fork=3, n_walk=2, seed=2105),
    dict(name="wh_m_blockstack", desc="block-stacked pallets + column grid", Lx=45, Ly=30,
         kind="block", n_fork=5, n_walk=0, seed=2106),
]


def make(spec):
    rng = np.random.default_rng(spec["seed"])
    Lx, Ly = spec["Lx"], spec["Ly"]
    for attempt in range(200):
        if spec["kind"] == "racks":
            boxes, circles, aisles_y, cuts, (x0, x1) = rack_layout(
                rng, Lx, Ly, spec["rows"], spec["aisle"], spec["cross"])
        else:
            boxes, circles = blockstack_layout(rng, Lx, Ly)
            aisles_y, cuts, (x0, x1) = [], [], (-Lx / 2 + 6, Lx / 2 - 5)
        # start in the west dock, goal in the east dock, opposite halves in y
        sy = float(rng.uniform(-Ly / 2 + 3.5, -1.0))
        gy = float(rng.uniform(1.0, Ly / 2 - 3.5))
        start = (round(-Lx / 2 + 3.2, 2), round(sy, 2))
        goal = (round(Lx / 2 - 2.8, 2), round(gy, 2))
        wps, D = astar_route(boxes, circles, Lx, Ly, start, goal)
        if wps is not None:
            break
    else:
        raise RuntimeError(f"{spec['name']}: no feasible route")
    # walls
    t = 0.3
    walls = [(0, Ly / 2, Lx, t), (0, -Ly / 2, Lx, t), (Lx / 2, 0, t, Ly), (-Lx / 2, 0, t, Ly)]
    movers = []
    # forklifts shuttle along aisles (or along random free lines in the block layout)
    lanes = list(aisles_y) if aisles_y else list(np.linspace(-Ly / 2 + 3, Ly / 2 - 3, 6))
    for k in range(spec["n_fork"]):
        y = lanes[k % len(lanes)] + (0.0 if aisles_y else float(rng.uniform(-0.5, 0.5)))
        a = [x0 + float(rng.uniform(0, 4)), y]
        b = [x1 - float(rng.uniform(0, 4)), y]
        if sdist(np.array([a, b]), boxes, circles).min() < 0.7:
            continue
        if k % 2:
            a, b = b, a
        movers.append(mover(f"forklift{k+1}", "box", a, b, rng.uniform(0.6, 1.0),
                            rng.uniform(0, 15), rng.uniform(1, 4), size=[1.1, 0.8]))
    # walkers cross the aisles along the cross-aisles / dock zones
    xs_cross = list(cuts) + [x0 - 2.0, x1 + 2.0]
    for k in range(spec["n_walk"]):
        x = xs_cross[k % len(xs_cross)] + float(rng.uniform(-0.6, 0.6))
        a, b = [x, -Ly / 2 + 1.5], [x, Ly / 2 - 1.5]
        if k % 2:
            a, b = b, a
        movers.append(mover(f"walker{k+1}", "circle", a, b, rng.uniform(0.4, 0.8),
                            rng.uniform(0, 10), rng.uniform(2, 5)))
    name = f"swarm_world_{spec['name']}.sdf"
    sdf = HEAD.format(name=name, desc=spec["desc"], Lx=Lx, Ly=Ly, seed=spec["seed"],
                      nobs=len(boxes) + len(circles), start=start, goal=goal,
                      G=int(max(Lx, Ly) * 2))
    for i, w in enumerate(walls):
        sdf += box_sdf(f"wall{i}", *w, h=2.0)
    for i, b in enumerate(boxes):
        sdf += box_sdf(f"rack{i}", *b, h=2.4, col="0.25 0.35 0.6")
    for i, c in enumerate(circles):
        sdf += cyl_sdf(f"column{i}", *c, h=4.0)
    for m in movers:
        if m["shape"] == "box":
            sdf += box_sdf(m["name"], *m["route"][0], *m["size"], h=1.6, col="0.95 0.55 0.1")
        else:
            sdf += cyl_sdf(m["name"], *m["route"][0], m["radius"], h=1.7, col="0.9 0.2 0.2")
    sdf += (f'    <model name="goal_pad"><static>true</static><pose>{goal[0]} {goal[1]} 0.01 0 0 0</pose><link name="l">\n'
            '      <visual name="v"><geometry><cylinder><radius>1.2</radius><length>0.02</length></cylinder></geometry>\n'
            '        <material><ambient>0 0.8 0 1</ambient><diffuse>0 0.9 0 1</diffuse></material></visual></link></model>\n'
            '  </world>\n</sdf>\n')
    with open(os.path.join(OUT, name), "w") as fh:
        fh.write(sdf)
    route_len = float(np.sum(np.linalg.norm(np.diff(np.vstack([start, wps]), axis=0), axis=1)))
    doc = {"comment": (f"Motion contract for {name} (generated by tools/make_warehouse_layouts.py, "
                       f"seed {spec['seed']}). 'waypoints' is an A* route feasible for a body of "
                       f"swept radius {SWEPT:.3f} m."),
           "frame": "world", "dt": 0.1, "start": list(start), "scatter_radius": 2.2,
           "waypoints": [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in wps],
           "layout": {"size": [Lx, Ly], "kind": spec["kind"], "desc": spec["desc"],
                      "n_static": len(boxes) + len(circles), "route_length_m": round(route_len, 1),
                      "min_route_clearance_m": round(float(SWEPT), 3)},
           "movers": movers}
    with open(os.path.join(OUT, f"dynamic_obstacles_{spec['name']}.json"), "w") as fh:
        json.dump(doc, fh, indent=2)
    return name, len(boxes) + len(circles), len(movers), len(wps), route_len


if __name__ == "__main__":
    for sp in SPECS:
        n, no, nm, nw, L = make(sp)
        print(f"{n}: {no} statics, {nm} movers, {nw} waypoints, route {L:.1f} m")
