"""Geometry primitives ported 1:1 from the MATLAB pipeline.

Sources:
    world_dist.m              -> World.distance
    sdf2world.m               -> world_from_sdf (boxes/circles/goal, z-band)
    lidar_scan.m              -> lidar_scan (analytic raycast)
    formation_clearance.m     -> formation_clearance (payload SAT test incl.)
    eval_mover_pose.m         -> eval_mover_pose  (single source of truth)
    update_world_dyn.m        -> Mover.eval_at / velocity
    merge_dyn.m               -> World.with_movers

This module is ROS-agnostic on purpose: everything here is unit-testable
without a ROS installation, and the controller nodes import only from here.
All functions are numpy-vectorised where it matters.

Conventions (identical to MATLAB): x east, y north, yaw CCW, metres.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


# ---------------------------------------------------------------------------
# world model
# ---------------------------------------------------------------------------

@dataclass
class Box:
    c: np.ndarray          # (2,) centre
    yaw: float
    h: np.ndarray          # (2,) half-extents
    name: str = ""
    zlo: float = 0.0
    zhi: float = 1.0


@dataclass
class Circle:
    c: np.ndarray          # (2,)
    r: float
    name: str = ""
    zlo: float = 0.0
    zhi: float = 1.0


@dataclass
class World:
    boxes: list = field(default_factory=list)
    circles: list = field(default_factory=list)
    goal: np.ndarray | None = None
    goal_r: float = math.nan
    bounds: np.ndarray | None = None   # [xmin xmax ymin ymax]
    name: str = "world"

    def __post_init__(self):
        self._compute_bounds()

    def _compute_bounds(self):
        xs, ys = [], []
        for b in self.boxes:
            R = np.array([[math.cos(b.yaw), -math.sin(b.yaw)],
                          [math.sin(b.yaw), math.cos(b.yaw)]])
            corners = np.array([[-b.h[0], -b.h[1]], [b.h[0], -b.h[1]],
                                [b.h[0], b.h[1]], [-b.h[0], b.h[1]]])
            cn = corners @ R.T + b.c
            xs += cn[:, 0].tolist()
            ys += cn[:, 1].tolist()
        for c in self.circles:
            xs += [c.c[0] - c.r, c.c[0] + c.r]
            ys += [c.c[1] - c.r, c.c[1] + c.r]
        if not xs:
            self.bounds = np.array([-1.0, 1.0, -1.0, 1.0])
        else:
            self.bounds = np.array([min(xs), max(xs), min(ys), max(ys)])

    def with_movers(self, movers) -> "World":
        """merge_dyn.m: movers become ordinary geometry for all helpers."""
        wm = World(boxes=list(self.boxes),
                   circles=list(self.circles),
                   goal=self.goal, goal_r=self.goal_r,
                   bounds=self.bounds, name=self.name)
        for m in movers:
            if m.shape == "circle":
                wm.circles.append(Circle(m.c.copy(), m.r, m.name))
            else:
                wm.boxes.append(Box(m.c.copy(), 0.0, np.array(m.size) / 2, m.name))
        wm._compute_bounds()
        return wm

    # ---- world_dist.m ----
    def _arrays(self):
        """Cached vectorised obstacle arrays (rebuilt if the lists change)."""
        key = (len(self.boxes), len(self.circles))
        c = getattr(self, "_cache", None)
        if c is None or c["key"] != key:
            nb = len(self.boxes)
            c = {"key": key,
                 "bc": np.array([b.c for b in self.boxes]).reshape(nb, 2),
                 "bh": np.array([b.h for b in self.boxes]).reshape(nb, 2),
                 "bcos": np.array([math.cos(-b.yaw) for b in self.boxes]),
                 "bsin": np.array([math.sin(-b.yaw) for b in self.boxes]),
                 "cc": np.array([ci.c for ci in self.circles]).reshape(
                     len(self.circles), 2),
                 "cr": np.array([ci.r for ci in self.circles])}
            self._cache = c
        return c

    def distance_matrix(self, P, subset=None) -> np.ndarray:
        """Signed distance of every point to every obstacle, (N, nB+nC),
        boxes first then circles (the owner indexing of world_dist.m).
        subset: optional obstacle indices (same indexing) to restrict to."""
        P = np.atleast_2d(np.asarray(P, dtype=float))
        a = self._arrays()
        nb = len(a["bc"])
        if subset is None:
            ib, ic = slice(None), slice(None)
            n_b, n_c = nb, len(a["cc"])
        else:
            subset = np.asarray(subset)
            ib, ic = subset[subset < nb], subset[subset >= nb] - nb
            n_b, n_c = len(ib), len(ic)
        cols = []
        if n_b:
            dx = P[:, 0:1] - a["bc"][ib, 0]
            dy = P[:, 1:2] - a["bc"][ib, 1]
            lx = a["bcos"][ib] * dx - a["bsin"][ib] * dy
            ly = a["bsin"][ib] * dx + a["bcos"][ib] * dy
            qx = np.abs(lx) - a["bh"][ib, 0]
            qy = np.abs(ly) - a["bh"][ib, 1]
            cols.append(np.sqrt(np.maximum(qx, 0.0) ** 2 + np.maximum(qy, 0.0) ** 2)
                        + np.minimum(np.maximum(qx, qy), 0.0))
        if n_c:
            cols.append(np.sqrt((P[:, 0:1] - a["cc"][ic, 0]) ** 2
                                + (P[:, 1:2] - a["cc"][ic, 1]) ** 2) - a["cr"][ic])
        if not cols:
            return np.full((P.shape[0], 0), np.inf)
        return np.hstack(cols)

    def distance(self, P) -> np.ndarray:
        """Exact signed distance, negative inside obstacles. P is (N,2)."""
        D = self.distance_matrix(P)
        return D.min(axis=1) if D.shape[1] else np.full(D.shape[0], np.inf)

    def distance_owner(self, P):
        """(distance, index of the nearest obstacle), boxes then circles."""
        D = self.distance_matrix(P)
        if not D.shape[1]:
            return np.full(D.shape[0], np.inf), np.full(D.shape[0], -1)
        j = D.argmin(axis=1)
        return D[np.arange(D.shape[0]), j], j


def movers_distance(P, movers) -> np.ndarray:
    """Signed distance of points (N,2) to each mover, (N, nM) (mover_gap.m /
    the mover term of world_dist_dyn.m). Movers are axis-aligned."""
    P = np.atleast_2d(np.asarray(P, dtype=float))
    D = np.full((P.shape[0], len(movers)), np.inf)
    for k, m in enumerate(movers):
        dx, dy = P[:, 0] - m.c[0], P[:, 1] - m.c[1]
        if m.shape == "circle":
            D[:, k] = np.hypot(dx, dy) - m.r
        else:
            qx = np.abs(dx) - m.size[0] / 2
            qy = np.abs(dy) - m.size[1] / 2
            D[:, k] = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + \
                np.minimum(np.maximum(qx, qy), 0)
    return D


# ---------------------------------------------------------------------------
# SDF parsing (sdf2world.m port)
# ---------------------------------------------------------------------------

def _pose_of(el) -> np.ndarray:
    p = np.zeros(6)
    if el is not None:
        txt = el.text.strip() if el.text else ""
        vals = [float(v) for v in txt.split()]
        p[:len(vals)] = vals
    return p


def world_from_sdf(path, robot_height=0.35, z_floor=0.03,
                   goal_model="goal_pad", skip_models=("ground_plane",),
                   movers_json=None) -> World:
    """Parse box/cylinder collision geometry from an SDF world at the robot's
    height band. model:// includes and meshes are skipped with a warning.

    movers_json: path to the dynamic-obstacle contract. The mover models exist
    in the SDF only so Gazebo has something to teleport; their geometry comes
    from the JSON, so they must NOT also enter the static map (same reason the
    MATLAB side passes SkipModels).
    """
    skip = set(skip_models)
    if movers_json is not None:
        ms, _ = load_dynamic_obstacles(movers_json)
        skip.update(m.name for m in ms)
    root = ET.parse(path).getroot()
    world_el = root.find("world")
    if world_el is None:
        raise ValueError(f"no <world> in {path}")

    zband = (z_floor, robot_height)
    boxes, circles = [], []
    goal, goal_r = None, math.nan

    for model in world_el.findall("model"):
        name = model.get("name", "model")
        mpose = _pose_of(model.find("pose"))

        if name == goal_model:
            goal = mpose[:2].copy()
            for cyl in model.iter("cylinder"):
                r = cyl.find("radius")
                if r is not None:
                    goal_r = float(r.text)
                    break
            continue

        if name in skip:
            continue

        for link in model.findall("link"):
            lpose = _pose_of(link.find("pose"))
            for col in link.findall("collision"):
                cpose = _pose_of(col.find("pose"))
                p = mpose.copy()
                cy, sy = math.cos(mpose[5]), math.sin(mpose[5])
                p[0] += cy * lpose[0] - sy * lpose[1]
                p[1] += sy * lpose[0] + cy * lpose[1]
                p[2] += lpose[2]
                p[5] += lpose[5] + cpose[5]
                cy2, sy2 = math.cos(lpose[5]), math.sin(lpose[5])
                p[0] += cy2 * cpose[0] - sy2 * cpose[1]
                p[1] += sy2 * cpose[0] + cy2 * cpose[1]
                p[2] += cpose[2]

                geom = col.find("geometry")
                if geom is None:
                    continue

                box = geom.find("box")
                if box is not None:
                    size = np.array([float(v) for v in box.find("size").text.split()])
                    size = np.pad(size, (0, 3 - len(size)))
                    zl, zh = p[2] - size[2] / 2, p[2] + size[2] / 2
                    if zh > zband[0] and zl < zband[1]:
                        boxes.append(Box(p[:2].copy(), p[5],
                                         np.array(size[:2]) / 2, name, zl, zh))
                    continue

                cyl = geom.find("cylinder")
                if cyl is not None:
                    r = float(cyl.find("radius").text)
                    ln = float(cyl.find("length").text)
                    zl, zh = p[2] - ln / 2, p[2] + ln / 2
                    if zh > zband[0] and zl < zband[1]:
                        circles.append(Circle(p[:2].copy(), r, name, zl, zh))
                    continue

                if geom.find("mesh") is not None or geom.find("plane") is not None:
                    continue

    w = World(boxes=boxes, circles=circles, goal=goal, goal_r=goal_r,
              name=world_el.get("name", "world"))
    return w


# ---------------------------------------------------------------------------
# analytic lidar (lidar_scan.m port)
# ---------------------------------------------------------------------------

def lidar_scan(pos, world: World, n_beams=288, max_range=6.0,
               angle_min=-math.pi, angle_max=math.pi):
    """Analytic 2D raycast (slab method for boxes, quadratic for circles),
    vectorised over obstacles and beams. Returns (ranges, angles); misses
    return max_range. Open interval at the top so -pi and +pi are not
    duplicated. Obstacles farther than max_range are culled first."""
    angles = angle_min + (angle_max - angle_min) * np.arange(n_beams) / n_beams
    ranges = np.full(n_beams, float(max_range))
    cx, sy = np.cos(angles), np.sin(angles)
    EPS = 1e-12
    a = world._arrays()
    near = world.distance_matrix(np.asarray(pos, dtype=float)[None, :])[0] < max_range
    nb = len(a["bc"])
    kb = np.where(near[:nb])[0]
    if len(kb):
        ox = (pos[0] - a["bc"][kb, 0])[:, None]
        oy = (pos[1] - a["bc"][kb, 1])[:, None]
        c, s = a["bcos"][kb][:, None], a["bsin"][kb][:, None]
        lox, loy = c * ox - s * oy, s * ox + c * oy
        ldx, ldy = c * cx - s * sy, s * cx + c * sy
        ldx = np.where(np.abs(ldx) < EPS, EPS, ldx)
        ldy = np.where(np.abs(ldy) < EPS, EPS, ldy)
        hx, hy = a["bh"][kb, 0][:, None], a["bh"][kb, 1][:, None]
        t1x, t2x = (-hx - lox) / ldx, (hx - lox) / ldx
        t1y, t2y = (-hy - loy) / ldy, (hy - loy) / ldy
        tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
        tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
        t = np.where(tmin < 0, tmax, tmin)          # inside: exit face
        hit = (tmax >= np.maximum(tmin, 0)) & (t > 0)
        ranges = np.minimum(ranges, np.where(hit, t, np.inf).min(axis=0))
    kc = np.where(near[nb:])[0]
    if len(kc):
        ox = (pos[0] - a["cc"][kc, 0])[:, None]
        oy = (pos[1] - a["cc"][kc, 1])[:, None]
        bq = ox * cx + oy * sy
        cq = ox * ox + oy * oy - a["cr"][kc][:, None] ** 2
        disc = bq * bq - cq
        sq = np.sqrt(np.maximum(disc, 0))
        t = -bq - sq
        t = np.where(t < 0, -bq + sq, t)
        hit = (disc >= 0) & (t > 0)
        ranges = np.minimum(ranges, np.where(hit, t, np.inf).min(axis=0))
    return ranges, angles


# ---------------------------------------------------------------------------
# movers: the shared motion contract (eval_mover_pose.m port)
# ---------------------------------------------------------------------------

def eval_mover_pose(mover, t: float) -> np.ndarray:
    """Centre of `mover` at absolute time t. THE single source of truth -
    identical to eval_mover_pose.m and the contract in dynamic_obstacles.json."""
    route = mover.route
    if t < mover.t0:
        return route[0].copy()
    L = float(np.linalg.norm(route[1] - route[0]))
    if L <= 0 or mover.speed <= 0:
        return route[0].copy()
    tau = t - mover.t0
    if mover.loop:
        T = L / mover.speed + mover.wait
        leg = int(math.floor(tau / T)) + 1
        t_in_leg = math.fmod(tau, T)
        frac = min(t_in_leg * mover.speed / L, 1.0)
        s = frac if leg % 2 == 1 else 1.0 - frac
        return route[0] + s * (route[1] - route[0])
    s = min(tau * mover.speed / L, 1.0)
    return route[0] + s * (route[1] - route[0])


def eval_mover_vel(mover, t: float) -> np.ndarray:
    """Exact velocity (derivative of the piecewise-constant model)."""
    route = mover.route
    v = np.zeros(2)
    if t < mover.t0:
        return v
    L = float(np.linalg.norm(route[1] - route[0]))
    if L <= 0 or mover.speed <= 0:
        return v
    tau = t - mover.t0
    d = (route[1] - route[0]) / L
    if mover.loop:
        T = L / mover.speed + mover.wait
        t_in_leg = math.fmod(tau, T)
        if t_in_leg * mover.speed >= L:
            return v                                   # paused at endpoint
        leg = int(math.floor(tau / T)) + 1
        return -mover.speed * d if leg % 2 == 0 else mover.speed * d
    if tau * mover.speed >= L:
        return v
    return mover.speed * d


@dataclass
class Mover:
    name: str
    shape: str            # 'box' | 'circle'
    size: np.ndarray      # (2,) full extents
    r: float              # effective radius (circumradius for boxes)
    route: np.ndarray     # (2,2)
    speed: float
    t0: float = 0.0
    wait: float = 0.0
    loop: bool = False
    c: np.ndarray = None  # current centre
    v: np.ndarray = None  # current velocity
    t: float = 0.0

    def __post_init__(self):
        if self.c is None:
            self.c = eval_mover_pose(self, 0.0)
        if self.v is None:
            self.v = eval_mover_vel(self, 0.0)

    def eval_at(self, t: float) -> "Mover":
        return Mover(self.name, self.shape, np.array(self.size), self.r,
                     self.route.copy(), self.speed, self.t0, self.wait,
                     self.loop, eval_mover_pose(self, t),
                     eval_mover_vel(self, t), t)

    @property
    def box(self) -> Box:
        return Box(self.c.copy(), 0.0, np.array(self.size) / 2, self.name)

    @property
    def circle(self) -> Circle:
        return Circle(self.c.copy(), self.r, self.name)


def load_dynamic_obstacles(json_path) -> tuple[list, dict]:
    """load_dynamic_obstacles.m port (json module instead of jsondecode)."""
    import json
    with open(json_path) as fh:
        doc = json.load(fh)
    movers = []
    for e in doc.get("movers", []):
        size = np.array(e["size"], dtype=float)
        radius = float(e.get("radius", 0.0))
        if radius > 0:
            r = radius
        elif e["shape"] == "circle":
            r = size[0] / 2
        else:
            r = 0.5 * float(np.hypot(size[0], size[1]))
        movers.append(Mover(
            name=e["name"], shape=e["shape"], size=size, r=r,
            route=np.array(e["route"], dtype=float),
            speed=float(e["speed"]), t0=float(e.get("t0", 0.0)),
            wait=float(e.get("wait", 0.0)), loop=bool(e.get("loop", False)),
        ))
    return movers, {"frame": doc.get("frame", "world"), "dt": doc.get("dt", 0.1)}


# ---------------------------------------------------------------------------
# whole-body formation clearance (formation_clearance.m port)
# ---------------------------------------------------------------------------

def _sat_separated(d, ext, a_axes, hb, b_axes) -> bool:
    sep = False
    for ax in np.vstack([a_axes, b_axes]):
        rA = ext[0] * abs(ax @ a_axes[0]) + ext[1] * abs(ax @ a_axes[1])
        rB = hb[0] * abs(ax @ b_axes[0]) + hb[1] * abs(ax @ b_axes[1])
        if abs(ax @ d) > rA + rB:
            sep = True
            break
    return sep


def _circle_hit(cc, r, c, ca, sa, ext) -> bool:
    d = cc - c
    lx = ca * d[0] + sa * d[1]
    ly = -sa * d[0] + ca * d[1]
    qx = max(abs(lx) - ext[0], 0.0)
    qy = max(abs(ly) - ext[1], 0.0)
    return qx * qx + qy * qy <= r * r


def formation_offsets(kind, K, s):
    """Body-frame slot offsets (K,2), centred, for the supported shapes:
    square (2x2), line (1xK), triangle (K=3 equilateral), diamond/v (K=4).
    s = centre-to-centre slot spacing. Mirrors matlab/formation_geometry.m."""
    kind = (kind or "square").lower()
    if kind == "square":
        gx, gy = np.meshgrid(np.arange(2), np.arange(2))
        offs = np.column_stack([gx.ravel(), gy.ravel()]) * s
        return offs - offs.mean(axis=0)
    if kind == "line":
        return np.column_stack([
            (np.arange(K) - (K - 1) / 2.0) * s, np.zeros(K)])
    if kind in ("diamond", "v"):
        d = s / math.sqrt(2.0)
        return np.array([[0.0, d], [-d, 0.0], [0.0, -d], [d, 0.0]])
    if kind == "triangle":
        R = s / math.sqrt(3.0)
        return R * np.column_stack([
            np.cos(2 * math.pi * np.arange(3) / 3 + math.pi / 2),
            np.sin(2 * math.pi * np.arange(3) / 3 + math.pi / 2)])
    raise ValueError(f"unknown formation '{kind}' "
                     "(square|line|triangle|diamond|v)")


def _convex_hull(points):
    """Andrew's monotone chain. points (K,2) -> hull vertices CCW (H,2)."""
    pts = sorted({(float(p[0]), float(p[1])) for p in np.asarray(points)})
    if len(pts) <= 2:
        return np.array(pts)
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and \
                    (out[-1][0] - out[-2][0]) * (p[1] - out[-2][1]) - \
                    (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(p)
        return out
    lower, upper = half(pts), half(reversed(pts))
    return np.array(lower[:-1] + upper[:-1])


def _hull_body_separated(hull_w, r, other_pts, other_axes, other_growth=0.0):
    """Exact SAT: robot body = conv(hull_w) + disk(r) (the payload block:
    slots' convex hull, corners rounded by the robot radius) vs a convex
    polygon given by its vertices (2,m) and edge normals (m,2).
    other_growth inflates the other body along every axis (circle radius).
    Axis set = both edge-normal families + vertex-vertex differences, which
    covers corner-vs-corner closest features for rounded bodies."""
    H = np.asarray(hull_w, dtype=float)
    P = np.asarray(other_pts, dtype=float)          # (2,m)
    axes = []
    n = len(H)
    for i in range(n):
        e = H[(i + 1) % n] - H[i]
        L = math.hypot(e[0], e[1])
        if L > 1e-9:
            axes.append((-e[1] / L, e[0] / L))
    m = P.shape[1]
    for j in range(m):
        e = P[:, (j + 1) % m] - P[:, j]
        L = math.hypot(e[0], e[1])
        if L > 1e-9:
            axes.append((-e[1] / L, e[0] / L))
    for i in range(n):
        for j in range(m):
            d = H[i] - P[:, j]
            L = math.hypot(d[0], d[1])
            if L > 1e-9:
                axes.append((d[0] / L, d[1] / L))
    for ax in axes:
        pa = H @ np.array(ax)
        pb = P.T @ np.array(ax)
        if pa.min() - r > pb.max() + other_growth or \
                pb.min() - other_growth > pa.max() + r:
            return True
    return False


def formation_clearance(center, heading, offsets, world: World,
                        robot_radius, mode="payload", edge_step=0.15,
                        movers=None, sat="exact"):
    """Returns (clearance, collision). Port of formation_clearance_dyn.m:
    outline sampling + exact SAT block test in payload mode, over the static
    world and (optionally) a list of movers at their current poses.
    The payload body is the CONVEX HULL of the slots with corners rounded by
    robot_radius (a segment for the collinear line formation).

    sat = "exact"  : box corners in cyclic order, so both box edge normals
                     are tested -- the separating-axis test is exact.
    sat = "matlab" : bug-compatible with formation_clearance_dyn.m, whose
                     corner order (+,+),(+,-),(-,+),(-,-) is not cyclic: the
                     "edges" are two parallel sides and the two diagonals, so
                     one box normal is never tested and some collision-free
                     poses are reported as overlapping (conservative: it
                     never misses a real overlap). Use it to replicate the
                     MATLAB tables step for step."""
    sx = (1, 1, -1, -1) if sat == "matlab" else (1, 1, -1, -1)
    sy = (1, -1, 1, -1) if sat == "matlab" else (1, -1, -1, 1)
    exact = sat != "matlab"
    ca, sa = math.cos(heading), math.sin(heading)
    R = np.array([[ca, -sa], [sa, ca]])
    center = np.asarray(center, dtype=float)
    offsets = np.atleast_2d(offsets)
    slot = (R @ offsets.T).T + center
    K = slot.shape[0]
    n = round(math.sqrt(K))
    movers = movers or []

    pts = [slot]
    if mode != "slots":
        if n * n == K and n > 1:
            G = np.arange(1, K + 1).reshape(n, n)
            ring = np.concatenate([G[0, :], G[1:, -1], G[-1, -2::-1], G[-2:0:-1, 0]])
            ring = np.append(ring, ring[0])
            for k in range(len(ring) - 1):
                a, b = slot[ring[k] - 1], slot[ring[k + 1] - 1]
                L = float(np.linalg.norm(b - a))
                if L > edge_step:
                    m = int(math.ceil(L / edge_step))
                    lam = (np.arange(1, m) / m)[:, None]
                    pts.append(a + lam * (b - a))
        elif K == 2:
            a, b = slot[0], slot[1]
            L = float(np.linalg.norm(b - a))
            if L > edge_step:
                m = int(math.ceil(L / edge_step))
                lam = (np.arange(1, m) / m)[:, None]
                pts.append(a + lam * (b - a))

    P = np.vstack(pts)
    # Exact culling: an obstacle at distance Dc from the centre is at least
    # Dc - Rp from every sample point (Rp = farthest sample from the centre),
    # so only obstacles with Dc - Rp <= min(Dc) + Rp can hold the minimum.
    Dc = world.distance_matrix(center[None, :])[0]
    if Dc.size:
        Rp = float(np.linalg.norm(P - center, axis=1).max())
        cand = np.where(Dc - Rp <= Dc.min() + Rp)[0]
        clr = float(world.distance_matrix(P, cand).min())
    else:
        clr = float("inf")
    if movers:
        clr = min(clr, float(movers_distance(P, movers).min()))
    coll = clr <= robot_radius

    if mode == "payload" and K > 1:
        hull_w = _convex_hull(slot) if K > 2 else slot
        # Exact prefilter: the body lies inside the disk of radius
        # circumradius + robot_radius around the centre, so an obstacle whose
        # surface is farther than that from the centre cannot overlap it.
        reach = float(np.linalg.norm(offsets, axis=1).max()) + robot_radius + 1e-9
        if not exact:
            reach = float("inf")        # MATLAB tests every obstacle
        nb = len(world.boxes)
        coll_block = False
        for i in np.where(Dc[:nb] <= reach)[0]:
            b = world.boxes[i]
            cb, sb = math.cos(b.yaw), math.sin(b.yaw)
            corners = np.array([[cb, -sb], [sb, cb]]) @ \
                np.array([[sx[0] * b.h[0], sx[1] * b.h[0], sx[2] * b.h[0], sx[3] * b.h[0]],
                          [sy[0] * b.h[1], sy[1] * b.h[1], sy[2] * b.h[1], sy[3] * b.h[1]]]) + \
                np.array([[b.c[0]], [b.c[1]]])
            if not _hull_body_separated(hull_w, robot_radius, corners, []):
                coll_block = True
                break
        if not coll_block:
            for k in np.where(Dc[nb:] <= reach)[0]:
                ci = world.circles[k]
                if not _hull_body_separated(
                        hull_w, robot_radius,
                        np.array([[ci.c[0]], [ci.c[1]]]), [], ci.r):
                    coll_block = True
                    break
        if not coll_block and movers:
            Dm = movers_distance(center[None, :], movers)[0]
            for k in np.where(Dm <= reach)[0]:
                m = movers[k]
                if m.shape == "circle":
                    sep = _hull_body_separated(
                        hull_w, robot_radius,
                        np.array([[m.c[0]], [m.c[1]]]), [], m.r)
                else:
                    hx, hy = m.size[0] / 2, m.size[1] / 2
                    corners = np.array([[sx[0] * hx, sx[1] * hx, sx[2] * hx, sx[3] * hx],
                                        [sy[0] * hy, sy[1] * hy, sy[2] * hy, sy[3] * hy]]) + \
                        np.array([[m.c[0]], [m.c[1]]])
                    sep = _hull_body_separated(hull_w, robot_radius, corners, [])
                if not sep:
                    coll_block = True
                    break
        if coll_block:
            coll = True
            clr = min(clr, 0.0)
    return clr, coll
