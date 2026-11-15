"""Neural components, numpy inference only (training: tools/train_neural.py).

Two learned components, each aimed at a gap identified in the literature
review (docs/NEURAL.md):

1. ``nn`` steering law -- a learned local planner for the RIGID formation.
   Input: the same noisy scan every other law consumes, binned into 72
   sectors in the body frame and expressed relative to the body's swept
   radius (range - margin); the goal in the body frame; and the robot slot
   offsets, encoded by a permutation-invariant DeepSets encoder
   (per-slot MLP, mean + max pooling). The slot encoder is what lets ONE
   network drive teams of 3, 4, 6 or 9 robots carrying a rigid load; it is
   trained on some team sizes and tested on an unseen one. Output: a
   classification over the expert's heading-change levels. Trained by
   DAgger (Ross et al., AISTATS 2011) from the lattice-MPC expert. The
   policy is used inside the unchanged mission layer, so the pose commit is
   still gated (ground truth or sensor monitor) like every other law.

2. Learned safety monitor with split-conformal calibration. A regressor
   predicts the TRUE body clearance at t and t+dt (the quantity the ground
   truth gate thresholds) from two consecutive scans, seen from the
   candidate pose. On a held-out calibration set the one-sided residual
   quantile q_alpha is computed; a pose is admitted only if
   pred - q_alpha > robot_radius. Under exchangeability this bounds the
   false-accept probability by alpha (Vovk et al. 2005; Angelopoulos &
   Bates 2023). The evaluation checks the bound on held-out layouts, where
   exchangeability does not hold.

Weights are .npz files under worlds/../models/; nothing here imports torch.
"""

from __future__ import annotations

import math
import os

import numpy as np

N_BINS = 72
MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "models")
_CACHE = {}


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# features (shared by data collection, training and inference)
# ---------------------------------------------------------------------------

def scan_bins(ranges, angles, heading, max_sense, n_bins=N_BINS):
    """Min-pool a scan into n_bins body-frame sectors; empty = max_sense."""
    ab = _wrap(np.asarray(angles) - heading)
    k = np.floor((ab + np.pi) / (2 * np.pi) * n_bins).astype(int) % n_bins
    out = np.full(n_bins, float(max_sense))
    r = np.where(np.isfinite(ranges), ranges, max_sense)
    np.minimum.at(out, k, np.minimum(r, max_sense))
    return out


def policy_features(sc, pos, goal, heading, margin, offsets, robot_radius):
    """-> (scan (2,72), slots (K,2), aux (5,))"""
    rb = scan_bins(sc["ranges"], sc["angles"], heading, sc["max_sense"])
    x_rng = np.clip((rb - margin) / 3.0, -0.5, 1.5)
    x_hit = (rb < sc["max_sense"] - 1e-6).astype(float)
    d = np.asarray(goal, float) - np.asarray(pos, float)
    dist = float(np.hypot(*d))
    b = _wrap(math.atan2(d[1], d[0]) - heading)
    aux = np.array([math.tanh(dist / 5.0), math.sin(b), math.cos(b),
                    margin / 2.0, robot_radius / 0.2])
    return (np.stack([x_rng, x_hit]), np.asarray(offsets, float) / 2.0, aux)


# ---------------------------------------------------------------------------
# numpy network primitives
# ---------------------------------------------------------------------------

def _conv1d_circ(x, W, b):
    """x (C,L), W (O,C,k), b (O,) -> (O,L), circular padding, stride 1."""
    O, C, k = W.shape
    p = k // 2
    xp = np.concatenate([x[:, -p:], x, x[:, :p]], axis=1)
    L = x.shape[1]
    cols = np.stack([xp[:, i:i + L] for i in range(k)], axis=1)   # (C,k,L)
    return W.reshape(O, C * k) @ cols.reshape(C * k, L) + b[:, None]


def _relu(x):
    return np.maximum(x, 0.0)


def _pool2(x):
    return np.maximum(x[:, 0::2], x[:, 1::2])


def _lin(x, W, b):
    return x @ W.T + b


def load_model(name):
    if name not in _CACHE:
        path = name if os.path.isabs(name) else os.path.join(MODELS, name)
        if not path.endswith(".npz"):
            path += ".npz"
        with np.load(path, allow_pickle=False) as z:
            _CACHE[name] = {k: z[k] for k in z.files}
    return _CACHE[name]


def policy_forward(m, scan, slots, aux):
    h = _relu(_conv1d_circ(scan, m["c1.weight"], m["c1.bias"]))
    h = _pool2(_relu(_conv1d_circ(h, m["c2.weight"], m["c2.bias"])))
    h = _pool2(_relu(_conv1d_circ(h, m["c3.weight"], m["c3.bias"])))
    hs = _relu(_lin(h.reshape(-1), m["fs.weight"], m["fs.bias"]))
    e = _relu(_lin(slots, m["s1.weight"], m["s1.bias"]))
    e = _relu(_lin(e, m["s2.weight"], m["s2.bias"]))
    he = np.concatenate([e.mean(axis=0), e.max(axis=0)])
    z = np.concatenate([hs, he, aux])
    z = _relu(_lin(z, m["h1.weight"], m["h1.bias"]))
    z = _relu(_lin(z, m["h2.weight"], m["h2.bias"]))
    return _lin(z, m["h3.weight"], m["h3.bias"])


# ---------------------------------------------------------------------------
# the steering law
# ---------------------------------------------------------------------------

def nn_steer(sc, pos, goal, margin, cfg, prev_heading, V):
    """Dispatch entry for algo 'nn'. cfg.nn keys:
       model    weights file (default 'policy_dagger')
       offsets  slot offsets (body frame), set by TransportLeader
       robot_radius
       record   optional list: (features, expert class) appended per call
       beta     probability of executing the EXPERT action (DAgger mixing)
       rng      generator for the beta draw (own stream)"""
    from .vfh import local_mpc
    o = cfg.nn
    offs = o.get("offsets")
    if offs is None:
        s = margin / math.sqrt(2.0)
        offs = np.array([[s, s], [-s, s], [-s, -s], [s, -s]]) * 0.7
    rr = float(o.get("robot_radius", 0.18))
    feats = policy_features(sc, pos, goal, prev_heading, margin, offs, rr)
    rec, beta = o.get("record"), float(o.get("beta", 0.0))
    expert_vec = None
    if rec is not None or beta > 0:
        expert_vec = local_mpc(sc, pos, goal, margin, cfg, prev_heading, V)
    if rec is not None:
        d = _wrap(math.atan2(expert_vec[1], expert_vec[0]) - prev_heading)
        rec.append((feats, float(d)))
    if beta > 0 and o["rng"].random() < beta:
        return expert_vec
    m = load_model(o.get("model", "policy_dagger"))
    logits = policy_forward(m, *feats)
    dphi = float(m["classes"][int(np.argmax(logits))])
    phi = prev_heading + dphi
    return np.array([math.cos(phi), math.sin(phi)])


# ---------------------------------------------------------------------------
# learned + conformal safety monitor
# ---------------------------------------------------------------------------

def monitor_features(pts_now, pts_prev, sig_now, cand_pos, cand_heading,
                     offsets, n_bins=N_BINS, cap=3.0):
    """Clearance of the candidate body to the observed points, per sector,
    for the current and the previous scan, plus the per-sector range sigma.
    -> (3, n_bins) array (each row clipped to [-0.5, cap])."""
    from .geometry import _convex_hull
    from .safety_monitor import hull_point_distance
    c, s = math.cos(cand_heading), math.sin(cand_heading)
    slots = (np.array([[c, -s], [s, c]]) @ np.asarray(offsets).T).T + cand_pos
    hull = _convex_hull(slots) if len(slots) > 2 else slots
    rows = []
    for P, extra in ((pts_now, sig_now), (pts_prev, None)):
        out = np.full(n_bins, cap)
        sg = np.zeros(n_bins)
        if len(P):
            d = hull_point_distance(hull, P)
            a = _wrap(np.arctan2(P[:, 1] - cand_pos[1], P[:, 0] - cand_pos[0])
                      - cand_heading)
            k = np.floor((a + np.pi) / (2 * np.pi) * n_bins).astype(int) % n_bins
            np.minimum.at(out, k, np.minimum(d, cap))
            if extra is not None:
                np.maximum.at(sg, k, extra)
        rows.append(np.clip(out, -0.5, cap))
        if extra is not None:
            sig_row = sg
    rows.append(sig_row)
    return np.stack(rows)


def monitor_forward(m, x):
    h = _relu(_conv1d_circ(x, m["c1.weight"], m["c1.bias"]))
    h = _pool2(_relu(_conv1d_circ(h, m["c2.weight"], m["c2.bias"])))
    z = np.concatenate([h.reshape(-1), [x[0].min(), x[1].min()]])
    z = _relu(_lin(z, m["h1.weight"], m["h1.bias"]))
    z = _relu(_lin(z, m["h2.weight"], m["h2.bias"]))
    return float(_lin(z, m["h3.weight"], m["h3.bias"])[0])
