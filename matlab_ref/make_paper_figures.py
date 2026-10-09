#!/usr/bin/env python3
"""ONE script, ONE results table -> every evaluation figure, every LaTeX
table and the results log of the report.

    python3 matlab/make_paper_figures.py                 # after the grid
    python3 matlab/make_paper_figures.py --csv X --runs D --legacy

Inputs (written by run_paper_experiments.m):
    matlab/logs/paper/results_all.csv     one row per run
    matlab/logs/paper/runs/<run_name>.csv per-step logs

Outputs:
    docs/simulation_report/figures/*.pdf      figures
    docs/simulation_report/generated/*.tex    tables + provenance banner
    docs/RESULTS_LOG.md                       every cell of every experiment

Why one script: the previous report typed table numbers by hand from one
stack (Octave) and drew half of the figures from another (Python headless),
so tables and figures disagreed. Nothing here is typed by hand: a number in
the PDF exists only because it is in the CSV named in the provenance banner.

Conventions used in every output:
  * surface gap = logged clearance - robot radius (contact at gap <= 0)
  * gate rejections = steps where the ground-truth safety gate refused the
    commanded motion (the collisions the steering law would have caused)
  * intervals: Wilson 95% for rates, percentile bootstrap 95% for means
  * one colour and one order per algorithm, everywhere
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

try:
    from scipy import stats as _st
except ImportError:  # pragma: no cover
    _st = None

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_R = 0.18
ALGOS = ["apf", "vfh", "fgm", "dwa", "mpc", "pvfh", "pvfh_filt", "pvfh_prob"]
COL = {"apf": "#d62728", "vfh": "#ff7f0e", "fgm": "#8c564b",
       "dwa": "#9467bd", "mpc": "#e377c2", "pvfh_filt": "#17becf", "pvfh": "#1f77b4", "pvfh_prob": "#2ca02c"}
LAB = {"apf": "APF", "vfh": "VFH", "fgm": "FGM", "dwa": "DWA", "mpc": "MPC", "pvfh_filt": "pVFH+F",
       "pvfh": "pVFH", "pvfh_prob": "P-VFH"}
FORMS = ["square", "diamond", "triangle", "line"]
RNG = np.random.default_rng(12345)          # bootstrap only; deterministic


# --------------------------------------------------------------------------
# statistics
# --------------------------------------------------------------------------
def wilson(k, n, z=1.96):
    if n == 0:
        return (np.nan, np.nan)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def boot_ci(x, n_boot=4000):
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if len(x) == 0:
        return (np.nan, np.nan, np.nan)
    if len(x) == 1 or np.all(x == x[0]):
        return (x.mean(), x.mean(), x.mean())
    m = RNG.choice(x, (n_boot, len(x)), replace=True).mean(axis=1)
    return (x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5))


def paired_test(d):
    """Paired differences -> (mean, lo, hi, p_wilcoxon, n)."""
    d = np.asarray(d, float)
    d = d[~np.isnan(d)]
    if len(d) == 0:
        return (np.nan,) * 4 + (0,)
    mean, lo, hi = boot_ci(d)
    p = np.nan
    if _st is not None and len(d) >= 5 and np.any(d != 0):
        try:
            p = float(_st.wilcoxon(d, zero_method="wilcox").pvalue)
        except ValueError:
            p = np.nan
    return (mean, lo, hi, p, len(d))


# --------------------------------------------------------------------------
# data
# --------------------------------------------------------------------------
def load(csv, legacy):
    T = pd.read_csv(csv)
    for c, v in (("stack", "octave (legacy, pre-fix)" if legacy else "matlab"),
                 ("formation", "square"), ("mover_scale", 1.0), ("adl", 1),
                 ("n_reject", np.nan), ("n_escape", np.nan),
                 ("coll_static", np.nan), ("coll_mover", np.nan),
                 ("transport_time", np.nan), ("run_name", ""),
                 ("loc_sigma", 0.0), ("comm_drop", 0.0), ("n_movers", -1),
                 ("kin", 0), ("max_robot_speed", np.nan)):
        if c not in T.columns:
            T[c] = v
    T["formation"] = T["formation"].fillna("square")
    T["world"] = T["world"].astype(str)
    T["gap"] = T["min_clear"] - ROBOT_R
    if legacy and (T["run_name"] == "").all():
        T["run_name"] = [legacy_name(r) for _, r in T.iterrows()]
    return T


def legacy_name(r):
    e, a, s = r["exp"], r["algo"], int(r["seed"])
    sg = f"sg{round(r['sigma0'] * 100):03d}"
    nb = int(r["beams"])
    if e == "E1":
        return f"paper_e1_{a}_s{s}"
    if e == "E2":
        return f"paper_e2_{a}_{sg}_s{s}"
    if e == "E3":
        return f"paper_e3_b{nb:03d}_s{s}"
    if e == "E4":
        return f"paper_e4_{r['formation']}_s{s}"
    if e == "E5":
        return f"paper_e5_{a}_{r['formation']}_s{s}"
    return f"paper_e6_{a}_{r['sense_kind']}{nb:03d}_{sg}_s{s}"


class Runs:
    def __init__(self, d):
        self.d = d
        self.cache = {}

    def get(self, name):
        if name in self.cache:
            return self.cache[name]
        p = os.path.join(self.d, f"{name}.csv")
        L = None
        if name and os.path.exists(p):
            L = pd.read_csv(p)
            L = L[L["phase"] == 2].reset_index(drop=True)
            if "escape" not in L.columns:
                L["escape"] = 0
            if "reject" not in L.columns:
                L["reject"] = 0
        self.cache[name] = L
        return L


def cell(T, **eq):
    m = np.ones(len(T), bool)
    for k, v in eq.items():
        if isinstance(v, float):
            m &= np.isclose(T[k].astype(float), v)
        else:
            m &= (T[k] == v).to_numpy()
    return T[m]


def agg(c):
    """Cell summary dict (c = runs of one cell)."""
    n = len(c)
    if n == 0:
        return None
    k = int(np.nansum(c["reached"]))
    lo, hi = wilson(k, n)
    return dict(n=n, k=k, rate=k / n, lo=lo, hi=hi,
                rej=boot_ci(c["n_reject"]), coll=np.nanmean(c["collisions"]),
                cs=np.nansum(c["coll_static"]) if c["coll_static"].notna().any() else np.nan,
                cm=np.nanmean(c["coll_mover"]) if c["coll_mover"].notna().any() else np.nan,
                gap=boot_ci(c["gap"]), eff=np.nanmean(c["path_eff"]),
                time=np.nanmean(c["total_time"]),
                comp=np.nanmean(c["comp_ms_mean"]),
                esc=np.nanmean(c["n_escape"]) if c["n_escape"].notna().any() else np.nan)


def present(T, exp, col, order):
    have = set(T[T["exp"] == exp][col])
    return [x for x in order if x in have]


# --------------------------------------------------------------------------
# world drawing (so trajectories are shown IN the map they were driven in)
# --------------------------------------------------------------------------
def load_world(sdf, contract):
    movers, wps = [], None
    if contract and os.path.exists(contract):
        c = json.load(open(contract))
        movers = c.get("movers", [])
        wps = c.get("waypoints")
    skip = {m["name"] for m in movers} | {"ground_plane", "goal_pad"}
    shapes = []
    goal = (12.0, 0.0)
    if os.path.exists(sdf):
        for mdl in ET.parse(sdf).getroot().iter("model"):
            name = mdl.get("name")
            pose = [float(v) for v in (mdl.findtext("pose") or "0 0 0 0 0 0").split()]
            if name == "goal_pad":
                goal = (pose[0], pose[1])
            if name in skip:
                continue
            col = mdl.find(".//collision")
            if col is None:
                continue
            b = col.find(".//box/size")
            cy = col.find(".//cylinder/radius")
            if b is not None:
                sx, sy = [float(v) for v in b.text.split()][:2]
                shapes.append(("box", pose[0], pose[1], sx, sy))
            elif cy is not None:
                shapes.append(("circle", pose[0], pose[1], float(cy.text), 0))
    return dict(shapes=shapes, movers=movers, goal=goal, wps=wps)


def draw_world(ax, W, start=(-12.0, 0.0)):
    for s in W["shapes"]:
        if s[0] == "box":
            ax.add_patch(Rectangle((s[1] - s[3] / 2, s[2] - s[4] / 2), s[3], s[4],
                                   fc="0.80", ec="0.55", lw=0.4, zorder=1))
        else:
            ax.add_patch(Circle((s[1], s[2]), s[3], fc="0.80", ec="0.55",
                                lw=0.4, zorder=1))
    for m in W["movers"]:
        (x0, y0), (x1, y1) = m["route"]
        ax.plot([x0, x1], [y0, y1], ls=(0, (1, 2)), color="0.35", lw=0.9, zorder=2)
    if W["wps"]:
        w = np.array(W["wps"])
        ax.plot(w[:-1, 0], w[:-1, 1], "kx", ms=4, zorder=5)
    ax.plot(*start, "k^", ms=5, zorder=6)
    ax.plot(*W["goal"], "k*", ms=10, zorder=6)
    ax.set_xlim(-15.3, 15.3)
    ax.set_ylim(-8.3, 8.3)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=6)


def worlds(T):
    out = {}
    for w in sorted(set(T["world"])):
        tag = w.replace("swarm_world_", "").replace(".sdf", "")
        cj = "dynamic_obstacles.json" if tag == "dynamic" else f"dynamic_obstacles_{tag}.json"
        out[w] = load_world(os.path.join(HERE, w), os.path.join(HERE, cj))
    return out


# --------------------------------------------------------------------------
# figure helpers
# --------------------------------------------------------------------------
def pending(path, what):
    fig, ax = plt.subplots(figsize=(6.0, 1.6))
    ax.axis("off")
    ax.text(0.5, 0.5, f"{what}\nnot in the results table yet -- run the grid "
            "(Section: How to run everything)", ha="center", va="center",
            fontsize=9, color="0.3")
    fig.savefig(path)
    plt.close(fig)


def save(fig, path):
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def errbar(ax, x, a_list, key, color, label, ls="-", rate=False):
    if rate:
        y = [100 * a["rate"] for a in a_list]
        lo = [100 * a["lo"] for a in a_list]
        hi = [100 * a["hi"] for a in a_list]
    else:
        y = [a[key][0] for a in a_list]
        lo = [a[key][1] for a in a_list]
        hi = [a[key][2] for a in a_list]
    y, lo, hi = map(np.array, (y, lo, hi))
    ax.errorbar(x, y, yerr=[np.clip(y - lo, 0, None), np.clip(hi - y, 0, None)],
                color=color, ls=ls, marker="o", ms=3.5, lw=1.4, capsize=2,
                label=label)


def three_metric_lines(T, exp, xcol, xvals, algos, xlabel, title, path,
                       xfmt="{:g}", extra=None):
    """Reach / gate rejections / min gap vs a factor, one line per algo.
    Categorical x positions with a small per-algo dodge so intervals that
    coincide stay visible (the old log-x twin-axis plot hid them)."""
    fig, axes = plt.subplots(3, 1, figsize=(5.6, 6.2), sharex=True)
    n = len(algos)
    for i, a in enumerate(algos):
        xs, cells = [], []
        for j, xv in enumerate(xvals):
            q = dict(exp=exp, algo=a)
            q[xcol] = xv
            if extra:
                q.update(extra)
            c = agg(cell(T, **q))
            if c:
                xs.append(j + (i - (n - 1) / 2) * 0.09)
                cells.append(c)
        if not cells:
            continue
        errbar(axes[0], xs, cells, None, COL[a], LAB[a], rate=True)
        if not np.isnan(cells[0]["rej"][0]):
            errbar(axes[1], xs, cells, "rej", COL[a], LAB[a])
        else:
            axes[1].plot(xs, [c["coll"] for c in cells], color=COL[a],
                         marker="o", ms=3.5, lw=1.4)
        errbar(axes[2], xs, cells, "gap", COL[a], LAB[a])
    legacy = T["n_reject"].isna().all()
    axes[0].set_ylabel("goal reached (%)")
    axes[0].set_ylim(-5, 105)
    axes[1].set_ylabel("collision steps / run\n(legacy metric)" if legacy
                       else "gate rejections / run")
    axes[1].set_yscale("symlog", linthresh=10)
    axes[1].set_ylim(-0.5, max(10.0, axes[1].get_ylim()[1]))
    axes[2].set_ylabel("min surface gap (m)")
    axes[2].axhline(0, color="k", lw=0.8, ls=":")
    axes[2].set_xticks(range(len(xvals)))
    axes[2].set_xticklabels([xfmt.format(v) for v in xvals])
    axes[2].set_xlabel(xlabel)
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7, ncol=min(n, 3), loc="lower left")
    axes[0].set_title(title, fontsize=10)
    save(fig, path)


# --------------------------------------------------------------------------
# figures
# --------------------------------------------------------------------------
def fig_e1(T, out):
    algos = present(T, "E1", "algo", ALGOS)
    if not algos:
        return pending(out, "E1")
    cs = [agg(cell(T, exp="E1", algo=a)) for a in algos]
    legacy = T["n_reject"].isna().all()
    fig, ax = plt.subplots(1, 4, figsize=(11.0, 2.9))
    x = np.arange(len(algos))
    cols = [COL[a] for a in algos]
    r = np.array([100 * c["rate"] for c in cs])
    ax[0].bar(x, r, color=cols, yerr=[r - [100 * c["lo"] for c in cs],
                                      [100 * c["hi"] for c in cs] - r], capsize=3)
    ax[0].set_ylabel("goal reached (%)")
    ax[0].set_ylim(0, 108)
    if legacy:
        ax[1].bar(x, [c["coll"] for c in cs], color=cols)
        ax[1].set_ylabel("collision steps / run (legacy)")
    else:
        v = np.array([c["rej"][0] for c in cs])
        ax[1].bar(x, v, color=cols, capsize=3,
                  yerr=[v - [c["rej"][1] for c in cs], [c["rej"][2] for c in cs] - v])
        ax[1].set_ylabel("gate rejections / run")
    g = np.array([c["gap"][0] for c in cs])
    ax[2].bar(x, g, color=cols, capsize=3,
              yerr=[g - [c["gap"][1] for c in cs], [c["gap"][2] for c in cs] - g])
    ax[2].axhline(0, color="k", lw=0.8)
    ax[2].set_ylabel("min surface gap (m)")
    ax[3].bar(x, [c["comp"] for c in cs], color=cols)
    ax[3].set_ylabel("steering compute (ms/step)")
    for a_ in ax:
        a_.set_xticks(x)
        a_.set_xticklabels([LAB[a] for a in algos], rotation=25, fontsize=8)
        a_.grid(alpha=0.3, axis="y")
    fig.suptitle(f"E1: steering laws on the precise lidar ({cs[0]['n']} seeds; "
                 "bars = mean, whiskers = 95% CI)", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_paired(T, out):
    E = T[T["exp"] == "E2"]
    if E.empty:
        return pending(out, "E2 paired analysis")
    sig = sorted(set(E["sigma0"]))
    pairs = [(a, b) for a, b in (("pvfh_prob", "pvfh_filt"), ("pvfh_prob", "pvfh"), ("pvfh_filt", "pvfh"))
             if a in set(E["algo"]) and b in set(E["algo"])]
    if not pairs:
        return pending(out, "E2 paired analysis (no pair of ladder laws in the table)")
    metric = "gap"
    fig, axes = plt.subplots(1, len(pairs), figsize=(5.0 * len(pairs), 3.3),
                             squeeze=False)
    for ax, (a, b) in zip(axes[0], pairs):
        P = E.pivot_table(index=["sigma0", "seed"], columns="algo", values=metric)
        d = (P[a] - P[b]).unstack("seed")           # rows sigma, cols seed
        for s in d.columns:
            ax.plot(range(len(sig)), d[s].to_numpy(), color="0.6", lw=0.6, alpha=0.6)
        st = [paired_test(d.loc[sg].to_numpy()) for sg in sig]
        m = np.array([s[0] for s in st])
        ax.errorbar(range(len(sig)), m, yerr=[m - [s[1] for s in st],
                    [s[2] for s in st] - m], color="k", lw=2, marker="o",
                    capsize=3, label="mean, 95% CI")
        ax.axhline(0, color="k", ls=":", lw=1)
        ax.set_xticks(range(len(sig)))
        ax.set_xticklabels([f"{s:g}" for s in sig])
        ax.set_xlabel(r"range-noise std $\sigma_0$ (m)")
        ax.set_ylabel(f"$\\Delta$ min gap: {LAB[a]} $-$ {LAB[b]} (m)")
        ax.set_title(f"{LAB[a]} vs {LAB[b]} (thin = one seed)", fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    save(fig, out)


def fig_overlays(T, R, W, out):
    E = T[T["exp"] == "E1"]
    algos = present(T, "E1", "algo", ALGOS)
    if not algos:
        return pending(out, "E1 trajectories")
    ncol = 3 if len(algos) > 4 else 2
    nrow = int(math.ceil(len(algos) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.4 * ncol, 2.6 * nrow),
                             squeeze=False)
    found = False
    for k, a in enumerate(algos):
        ax = axes[k // ncol][k % ncol]
        c = E[E["algo"] == a]
        draw_world(ax, W[c["world"].iloc[0]])
        for _, r in c.iterrows():
            L = R.get(r["run_name"])
            if L is None:
                continue
            found = True
            ax.plot(L["leader_x"], L["leader_y"], color=COL[a], lw=0.9,
                    alpha=0.7, zorder=4)
        ax.set_title(f"{LAB[a]}: reached {int(c['reached'].sum())}/{len(c)}",
                     fontsize=9)
    for k in range(len(algos), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    if not found:
        plt.close(fig)
        return pending(out, "E1 per-step logs")
    fig.suptitle("E1: transport-phase centroid paths, all seeds per law "
                 "(triangle = start, star = goal, dotted = mover routes)",
                 fontsize=10)
    fig.tight_layout()
    save(fig, out)


def formation_offsets(kind, s):
    if kind == "square":
        return np.array([[-.5, -.5], [-.5, .5], [.5, -.5], [.5, .5]]) * s
    if kind == "line":
        return np.c_[(np.arange(4) - 1.5) * s, np.zeros(4)]
    if kind == "triangle":
        th = np.arange(3) * 2 * np.pi / 3 + np.pi / 2
        return (s / math.sqrt(3)) * np.c_[np.cos(th), np.sin(th)]
    d = s / math.sqrt(2)
    return np.array([[0, d], [-d, 0], [0, -d], [d, 0]])


def fig_sketch(out, s=0.60):
    fig, axes = plt.subplots(1, 4, figsize=(10.5, 2.9))
    for ax, f in zip(axes, FORMS):
        off = formation_offsets(f, s)
        rc = float(np.linalg.norm(off, axis=1).max())
        for p in off:
            ax.add_patch(Circle(p, ROBOT_R, fc="#1f77b4", ec="k", zorder=3))
        ax.add_patch(Circle((0, 0), rc + ROBOT_R, fill=False, color="#d62728",
                            ls="--", lw=1.1))
        ax.set_title(f"{f}: circumradius {rc:.2f} m\n(+ robot radius = "
                     f"{rc + ROBOT_R:.2f} m)", fontsize=8)
        ax.set_xlim(-1.35, 1.35)
        ax.set_ylim(-1.35, 1.35)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=6)
    fig.suptitle(f"Slot layouts at s = {s:.2f} m, to scale (blue = robots of "
                 "radius 0.18 m; dashed = swept body radius the corridor must admit)",
                 fontsize=9)
    fig.tight_layout()
    save(fig, out)


def fig_trails(T, R, W, exp, algo, forms, title, out):
    E = cell(T, exp=exp, algo=algo)
    forms = [f for f in forms if f in set(E["formation"])]
    if not forms:
        return pending(out, title)
    fig, axes = plt.subplots(2, 2, figsize=(9.0, 5.2), squeeze=False)
    found = False
    for k, f in enumerate(forms[:4]):
        ax = axes[k // 2][k % 2]
        c = E[E["formation"] == f]
        draw_world(ax, W[c["world"].iloc[0]])
        for _, r in c.iterrows():
            L = R.get(r["run_name"])
            if L is None:
                continue
            found = True
            ax.plot(L["leader_x"], L["leader_y"], lw=0.9, alpha=0.75,
                    color=COL[algo], zorder=4)
        ax.set_title(f"{f}: reached {int(c['reached'].sum())}/{len(c)}", fontsize=9)
    for k in range(len(forms), 4):
        axes[k // 2][k % 2].axis("off")
    if not found:
        plt.close(fig)
        return pending(out, title + " (per-step logs)")
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_heatmap(T, out):
    algos = present(T, "E5", "algo", ALGOS)
    forms = present(T, "E5", "formation", FORMS)
    if not algos:
        return pending(out, "E5")
    legacy = T["n_reject"].isna().all()
    M = {k: np.full((len(algos), len(forms)), np.nan) for k in ("rate", "rej", "gap")}
    n = 0
    for i, a in enumerate(algos):
        for j, f in enumerate(forms):
            c = agg(cell(T, exp="E5", algo=a, formation=f))
            if c:
                n = max(n, c["n"])
                M["rate"][i, j] = 100 * c["rate"]
                M["rej"][i, j] = c["coll"] if legacy else c["rej"][0]
                M["gap"][i, j] = c["gap"][0]
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 0.55 * len(algos) + 1.9))
    spec = (("rate", "goal reached (%)", "RdYlGn", (0, 100), "{:.0f}"),
            ("rej", "collision steps / run (legacy)" if legacy else
             "gate rejections / run", "YlOrRd", (0, None), "{:.1f}"),
            ("gap", "mean min surface gap (m)", "RdYlGn", (-0.2, 0.7), "{:.2f}"))
    for ax, (k, ttl, cmap, lim, fmt) in zip(axes, spec):
        vmax = lim[1] if lim[1] is not None else max(1.0, np.nanmax(M[k]))
        im = ax.imshow(M[k], cmap=cmap, vmin=lim[0], vmax=vmax, aspect="auto")
        ax.set_xticks(range(len(forms)))
        ax.set_xticklabels(forms)
        ax.set_yticks(range(len(algos)))
        ax.set_yticklabels([LAB[a] for a in algos])
        ax.set_title(ttl, fontsize=9)
        for i in range(len(algos)):
            for j in range(len(forms)):
                v = M[k][i, j]
                ax.text(j, i, "--" if np.isnan(v) else fmt.format(v),
                        ha="center", va="center", fontsize=8, color="k",
                        bbox=dict(fc="white", ec="none", alpha=0.6, pad=0.6))
        fig.colorbar(im, ax=ax, shrink=0.85)
    fig.suptitle(f"E5: formation x steering law ({n} seeds per cell)", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def route_progress(L, W):
    """Distance travelled toward the goal: start distance minus the logged
    dist_to_goal is wrong on multi-waypoint routes, so use the x-coordinate
    on straight worlds and cumulative route completion otherwise."""
    return L["leader_x"].to_numpy()


def fig_line_anatomy(T, R, W, out):
    E = cell(T, exp="E5", formation="line")
    algos = [a for a in ALGOS if a in set(E["algo"])]
    if not algos:
        return pending(out, "E5 line anatomy")
    ncol = 3 if len(algos) > 4 else 2
    nrow = int(math.ceil(len(algos) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.4 * ncol, 2.6 * nrow),
                             squeeze=False)
    found = False
    for k, a in enumerate(algos):
        ax = axes[k // ncol][k % ncol]
        c = E[E["algo"] == a]
        draw_world(ax, W[c["world"].iloc[0]])
        for _, r in c.iterrows():
            L = R.get(r["run_name"])
            if L is None:
                continue
            found = True
            ax.plot(L["leader_x"], L["leader_y"], color=COL[a], lw=0.8,
                    alpha=0.6, zorder=4)
        ax.set_title(f"{LAB[a]}: reached {int(c['reached'].sum())}/{len(c)}, "
                     f"mean {c['total_time'].mean():.0f} s", fontsize=9)
    for k in range(len(algos), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    if not found:
        plt.close(fig)
        return pending(out, "E5 line per-step logs")
    fig.suptitle("E5, line formation (2.16 m hull): centroid paths of every seed, "
                 "one panel per steering law", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def median_run(c):
    """The run with the median mission time in a cell (a typical run, picked
    by rule -- the old figure hard-coded seeds and annotation times)."""
    c = c.sort_values("total_time")
    return c.iloc[len(c) // 2]


def fig_line_timeseries(T, R, out):
    E = cell(T, exp="E5", formation="line")
    algos = [a for a in ALGOS if a in set(E["algo"])]
    if not algos:
        return pending(out, "E5 line time series")
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.0), sharex=True)
    found = False
    for a in algos:
        r = median_run(E[E["algo"] == a])
        L = R.get(r["run_name"])
        if L is None:
            continue
        found = True
        t = L["t"].to_numpy() - L["t"].iloc[0]
        axes[0].plot(t, L["leader_x"], color=COL[a], lw=1.4,
                     label=f"{LAB[a]} (seed {int(r['seed'])})")
        axes[1].plot(t, L["clearance"] - ROBOT_R, color=COL[a], lw=0.9)
        e = L["escape"].to_numpy() > 0
        if e.any():
            axes[0].plot(t[e], L["leader_x"].to_numpy()[e], ".", color="k",
                         ms=2.5, zorder=5)
    if not found:
        plt.close(fig)
        return pending(out, "E5 line per-step logs")
    axes[0].plot([], [], "k.", ms=4, label="committed-escape steps")
    axes[0].axhline(12.0, color="k", ls=":", lw=1)
    axes[1].axhline(0.0, color="k", ls=":", lw=1)
    axes[0].set_ylabel("centroid x (m); goal at 12")
    axes[1].set_ylabel("surface gap (m)")
    axes[1].set_xlabel("time in transport phase (s)")
    axes[0].legend(fontsize=7, loc="lower right")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title("E5 line formation: the median-duration run of each law",
                      fontsize=10)
    fig.tight_layout()
    save(fig, out)


E6_CFG = [("lidar", 288), ("tof", 16), ("tof", 8)]


def fig_e6(T, out):
    E = T[T["exp"] == "E6"]
    if E.empty:
        return pending(out, "E6")
    algos = [a for a in ALGOS if a in set(E["algo"])]
    sig = sorted(set(E["sigma0"]))
    cfgs = [(k, b, s) for k, b in E6_CFG for s in sig]
    legacy = T["n_reject"].isna().all()
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 3.2))
    w = 0.8 / len(algos)
    for i, a in enumerate(algos):
        cs = [agg(cell(T, exp="E6", algo=a, sense_kind=k, beams=b, sigma0=float(s)))
              for k, b, s in cfgs]
        x = np.arange(len(cfgs)) + (i - (len(algos) - 1) / 2) * w
        ok = [c is not None for c in cs]
        cs = [c for c in cs if c]
        x = x[ok]
        r = np.array([100 * c["rate"] for c in cs])
        axes[0].bar(x, r, w, color=COL[a], label=LAB[a], capsize=2,
                    yerr=[r - [100 * c["lo"] for c in cs], [100 * c["hi"] for c in cs] - r])
        v = np.array([c["coll"] if legacy else c["rej"][0] for c in cs])
        axes[1].bar(x, v, w, color=COL[a])
        g = np.array([c["gap"][0] for c in cs])
        axes[2].bar(x, g, w, color=COL[a], capsize=2,
                    yerr=[g - [c["gap"][1] for c in cs], [c["gap"][2] for c in cs] - g])
    labels = [f"{k}-{b}\n$\\sigma_0$={s:g}" for k, b, s in cfgs]
    for ax, yl in zip(axes, ("goal reached (%)",
                             "collision steps / run (legacy)" if legacy
                             else "gate rejections / run",
                             "min surface gap (m)")):
        ax.set_xticks(range(len(cfgs)))
        ax.set_xticklabels(labels, fontsize=7)
        ax.set_ylabel(yl)
        ax.grid(alpha=0.3, axis="y")
    axes[0].set_ylim(0, 125)
    axes[0].set_yticks([0, 25, 50, 75, 100])
    axes[0].legend(fontsize=7, ncol=len(algos), loc="upper center")
    axes[2].axhline(0, color="k", lw=0.8)
    fig.suptitle("E6: matched sensor comparison (same seeds and world; "
                 "x-groups in the order of the table)", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_e6_timeseries(T, R, out):
    E = cell(T, exp="E6", algo="vfh")
    if E.empty:
        return pending(out, "E6 time series")
    sg = max(set(E["sigma0"]))
    A = cell(E, sense_kind="lidar", sigma0=float(sg))
    B = cell(E, sense_kind="tof", beams=16, sigma0=float(sg))
    if A.empty or B.empty:
        return pending(out, "E6 time series")
    key = "collisions" if A["n_reject"].isna().all() else "n_reject"
    seed = int(A.sort_values(key).iloc[-1]["seed"])     # lidar's worst seed
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 4.6), sharex=True)
    found = False
    for c, lab, col in ((A, "lidar-288", "#d62728"), (B, "tof-16", "#2ca02c")):
        r = c[c["seed"] == seed]
        if r.empty:
            continue
        L = R.get(r.iloc[0]["run_name"])
        if L is None:
            continue
        found = True
        t = L["t"].to_numpy() - L["t"].iloc[0]
        axes[0].plot(t, L["clearance"] - ROBOT_R, color=col, lw=1.1,
                     label=f"VFH, {lab}, $\\sigma_0$={sg:g} (seed {seed})")
        axes[1].plot(t, L["leader_x"], color=col, lw=1.2)
        rj = L["reject"].to_numpy() > 0
        if rj.any():
            axes[1].plot(t[rj], L["leader_x"].to_numpy()[rj], "|", color="k", ms=5)
    if not found:
        plt.close(fig)
        return pending(out, "E6 per-step logs")
    axes[1].plot([], [], "k|", ms=5, label="gate rejection")
    axes[0].axhline(0, color="k", ls=":", lw=1)
    axes[1].axhline(12, color="k", ls=":", lw=1)
    axes[0].set_ylabel("surface gap (m)")
    axes[1].set_ylabel("centroid x (m)")
    axes[1].set_xlabel("time in transport phase (s)")
    axes[0].legend(fontsize=7)
    axes[1].legend(fontsize=7, loc="lower right")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title("E6 matched pair: same law, noise level and seed; only "
                      "the sensor differs", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_e7(T, R, W, out):
    E = T[T["exp"] == "E7"]
    if E.empty:
        return pending(out, "E7 (second environments)")
    ws = sorted(set(E["world"]))
    algos = [a for a in ALGOS if a in set(E["algo"])]
    sig = sorted(set(E["sigma0"]))
    fig, axes = plt.subplots(len(ws), 2, figsize=(11.0, 3.0 * len(ws)),
                             squeeze=False, gridspec_kw=dict(width_ratios=[1.5, 1]))
    for i, w in enumerate(ws):
        ax = axes[i][0]
        draw_world(ax, W[w])
        for a in ("pvfh", "pvfh_prob"):
            for _, r in cell(E, world=w, algo=a, sigma0=float(sig[0])).iterrows():
                L = R.get(r["run_name"])
                if L is not None:
                    ax.plot(L["leader_x"], L["leader_y"], color=COL[a], lw=0.8,
                            alpha=0.6, zorder=4)
        ax.set_title(f"{w}: pVFH (blue) and P-VFH (green) paths, "
                     f"$\\sigma_0$={sig[0]:g}", fontsize=9)
        bx = axes[i][1]
        wd = 0.8 / len(sig)
        for j, s in enumerate(sig):
            cs = [agg(cell(E, world=w, algo=a, sigma0=float(s))) for a in algos]
            x = np.arange(len(algos)) + (j - (len(sig) - 1) / 2) * wd
            r = np.array([100 * c["rate"] if c else np.nan for c in cs])
            lo = np.array([100 * c["lo"] if c else np.nan for c in cs])
            hi = np.array([100 * c["hi"] if c else np.nan for c in cs])
            bx.bar(x, r, wd, yerr=[r - lo, hi - r], capsize=2,
                   color=[COL[a] for a in algos], alpha=1.0 if j == 0 else 0.45,
                   edgecolor="k", lw=0.4)
        bx.set_xticks(range(len(algos)))
        bx.set_xticklabels([LAB[a] for a in algos], fontsize=7, rotation=25)
        bx.set_ylim(0, 108)
        bx.set_ylabel("goal reached (%)")
        bx.set_title("solid: $\\sigma_0$=" + f"{sig[0]:g}" +
                     (f", faint: {sig[-1]:g}" if len(sig) > 1 else ""), fontsize=8)
        bx.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    save(fig, out)


def fig_e13(T, W, out):
    E = T[T["exp"] == "E13"]
    if E.empty:
        return pending(out, "E13 (randomised layouts)")
    ws = sorted(set(E["world"]))
    algos = [a for a in ALGOS if a in set(E["algo"])]
    sig = sorted(set(E["sigma0"]))
    ncol = 6
    nrow = int(math.ceil(len(ws) / ncol))
    fig = plt.figure(figsize=(12.0, 1.35 * nrow + 3.2))
    gs = fig.add_gridspec(nrow + 1, ncol, height_ratios=[1.0] * nrow + [1.9])
    for i, w in enumerate(ws):
        ax = fig.add_subplot(gs[i // ncol, i % ncol])
        draw_world(ax, W[w])
        ax.set_xticks([])
        ax.set_yticks([])
        c = E[E["world"] == w]
        ax.set_title(f"{w.replace('swarm_world_', '').replace('.sdf', '')}: "
                     f"{int(c['reached'].sum())}/{len(c)} runs reached", fontsize=7)
    legacy = E["n_reject"].isna().all()
    for j, (key, yl) in enumerate((("rate", "goal reached (%)"),
                                   ("rej", "gate rejections / run"),
                                   ("gap", "min surface gap (m)"))):
        ax = fig.add_subplot(gs[nrow, [slice(0, 2), slice(2, 4), slice(4, 6)][j]])
        wd = 0.8 / len(sig)
        for q, sg in enumerate(sig):
            cs = [agg(cell(E, algo=a, sigma0=float(sg))) for a in algos]
            x = np.arange(len(algos)) + (q - (len(sig) - 1) / 2) * wd
            if key == "rate":
                v = np.array([100 * c["rate"] for c in cs])
                lo = np.array([100 * c["lo"] for c in cs])
                hi = np.array([100 * c["hi"] for c in cs])
            elif key == "rej" and legacy:
                v = np.array([c["coll"] for c in cs]); lo = v; hi = v
            else:
                v = np.array([c[key][0] for c in cs])
                lo = np.array([c[key][1] for c in cs])
                hi = np.array([c[key][2] for c in cs])
            ax.bar(x, v, wd, yerr=[np.clip(v - lo, 0, None), np.clip(hi - v, 0, None)],
                   capsize=2, color=[COL[a] for a in algos],
                   alpha=1.0 if q == 0 else 0.45, edgecolor="k", lw=0.4)
        ax.set_xticks(range(len(algos)))
        ax.set_xticklabels([LAB[a] for a in algos], fontsize=7, rotation=30)
        ax.set_ylabel(yl, fontsize=8)
        ax.grid(alpha=0.3, axis="y")
        if key == "gap":
            ax.axhline(0, color="k", lw=0.8)
    fig.suptitle(f"E13: {len(ws)} randomised layouts, pooled over layouts and seeds "
                 f"(solid: $\\sigma_0$={sig[0]:g}" +
                 (f", faint: {sig[-1]:g}" if len(sig) > 1 else "") + ")", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_e14(T, out):
    on = T[T["exp"] == "E14"]
    if on.empty:
        return pending(out, "E14 (load-feasible motion limits)")
    algos = [a for a in ALGOS if a in set(on["algo"])]
    fig, axes = plt.subplots(1, 4, figsize=(12.0, 3.0))
    for j, (lab, D, alpha) in enumerate((("limits OFF (E1 rows)", T[T["exp"] == "E1"], 0.45),
                                         ("limits ON", on, 1.0))):
        cs = [agg(D[D["algo"] == a]) for a in algos]
        x = np.arange(len(algos)) + (j - 0.5) * 0.38
        col = [COL[a] for a in algos]
        r = np.array([100 * c["rate"] if c else np.nan for c in cs])
        lo = np.array([100 * c["lo"] if c else np.nan for c in cs])
        hi = np.array([100 * c["hi"] if c else np.nan for c in cs])
        kw = dict(color=col, alpha=alpha, edgecolor="k", lw=0.4)
        axes[0].bar(x, r, 0.38, yerr=[r - lo, hi - r], capsize=2, **kw)
        axes[1].bar(x, [c["cm"] if c else np.nan for c in cs], 0.38, **kw)
        axes[2].bar(x, [c["time"] if c else np.nan for c in cs], 0.38, **kw)
        axes[3].bar(x, [np.nanmean(D[D["algo"] == a]["max_robot_speed"])
                        if (D["algo"] == a).any() else np.nan for a in algos],
                    0.38, label=lab, **kw)
    for ax, yl in zip(axes, ("goal reached (%)", "mover-contact steps / run",
                             "mean mission time (s)", "peak robot speed needed (m/s)")):
        ax.set_xticks(range(len(algos)))
        ax.set_xticklabels([LAB[a] for a in algos], fontsize=8)
        ax.set_ylabel(yl)
        ax.grid(alpha=0.3, axis="y")
    axes[0].set_ylim(0, 108)
    axes[3].axhline(1.0, color="k", ls=":", lw=1)
    axes[3].legend(fontsize=7)
    fig.suptitle("E14: the same missions without (faint) and with (solid) the "
                 "load-carrying motion limits", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_e9(T, out):
    off = T[T["exp"] == "E9"]
    if off.empty:
        return pending(out, "E9 (anti-deadlock ablation)")
    algos = [a for a in ALGOS if a in set(off["algo"])]
    fig, axes = plt.subplots(1, 2, figsize=(7.4, 3.0))
    for j, (lab, D, alpha) in enumerate((("layer ON (E5 rows)",
                                          cell(T, exp="E5", formation="line"), 1.0),
                                         ("layer OFF", off, 0.45))):
        cs = [agg(D[D["algo"] == a]) for a in algos]
        x = np.arange(len(algos)) + (j - 0.5) * 0.38
        r = np.array([100 * c["rate"] if c else np.nan for c in cs])
        lo = np.array([100 * c["lo"] if c else np.nan for c in cs])
        hi = np.array([100 * c["hi"] if c else np.nan for c in cs])
        axes[0].bar(x, r, 0.38, yerr=[r - lo, hi - r], capsize=2,
                    color=[COL[a] for a in algos], alpha=alpha, edgecolor="k", lw=0.4)
        axes[1].bar(x, [c["time"] if c else np.nan for c in cs], 0.38,
                    color=[COL[a] for a in algos], alpha=alpha, edgecolor="k",
                    lw=0.4, label=lab)
    for ax in axes:
        ax.set_xticks(range(len(algos)))
        ax.set_xticklabels([LAB[a] for a in algos])
        ax.grid(alpha=0.3, axis="y")
    axes[0].set_ylabel("goal reached (%)")
    axes[0].set_ylim(0, 108)
    axes[1].set_ylabel("mean mission time (s)")
    axes[1].legend(fontsize=7)
    fig.suptitle("E9: line formation with (solid) and without (faint) the "
                 "anti-deadlock layer", fontsize=10)
    fig.tight_layout()
    save(fig, out)


def fig_theory(out, margin=0.834 + ROBOT_R, react=1.0, thr=0.3):
    """Analytic figure for the kernel propositions (no simulation data)."""
    sh = (margin + react) / math.sqrt(2 * math.log(1 / thr))
    d = np.linspace(0, 5, 400)
    fig, ax = plt.subplots(1, 2, figsize=(9.0, 3.1))
    for s, c in ((0.0, "k"), (0.2, "#1f77b4"), (0.4, "#2ca02c"), (0.8, "#d62728")):
        tau = sh * sh + s * s
        ax[0].plot(d, math.sqrt(sh * sh / tau) * np.exp(-d * d / (2 * tau)),
                   color=c, label=f"$s_b$ = {s:g} m" + (" (deterministic)" if s == 0 else ""))
    ax[0].axhline(thr, color="0.4", ls=":", lw=1)
    ax[0].text(4.95, thr + 0.02, "threshold", ha="right", fontsize=7, color="0.3")
    ax[0].set_xlabel("measured range $d_{meas}$ (m)")
    ax[0].set_ylabel("kernel value $u_b$")
    ax[0].legend(fontsize=7)
    ax[0].grid(alpha=0.3)
    s = np.linspace(0, 2.6, 400)
    tau = sh * sh + s * s
    arg = 2 * tau * (math.log(1 / thr) - 0.5 * np.log(tau / (sh * sh)))
    ax[1].plot(s, np.sqrt(np.clip(arg, 0, None)), color="#2ca02c", lw=1.8)
    s_star = sh * math.sqrt(math.exp(2 * math.log(1 / thr) - 1) - 1)
    ax[1].axvline(s_star, color="0.4", ls=":", lw=1)
    ax[1].text(s_star, 0.3, f" $s^*$ = {s_star:.2f} m", fontsize=7, color="0.3")
    ax[1].axhline(margin + react, color="k", ls="--", lw=0.8)
    ax[1].text(0.02, margin + react - 0.13, "deterministic $d_{block}$", fontsize=7)
    ax[1].set_xlabel("per-beam noise $s_b$ (m)")
    ax[1].set_ylabel("reaction distance $d_{react}(s_b)$ (m)")
    ax[1].set_ylim(0, None)
    ax[1].grid(alpha=0.3)
    fig.suptitle(f"Expected-kernel properties ($\\sigma_h$ = {sh:.2f} m, "
                 f"threshold {thr:g}): noise widens the reaction envelope", fontsize=10)
    fig.tight_layout()
    save(fig, out)


# --------------------------------------------------------------------------
# LaTeX tables + results log
# --------------------------------------------------------------------------
def tx(s):
    return str(s).replace("_", r"\_")


def f1(v, nd=1):
    return "--" if v is None or (isinstance(v, float) and np.isnan(v)) else f"{v:.{nd}f}"


def cell_cols(c, legacy):
    """reach | rejections (or legacy collisions) | static | mover | gap | eff | time"""
    if c is None:
        return ["--"] * 7
    rate = f"{c['k']}/{c['n']} [{100*c['lo']:.0f},{100*c['hi']:.0f}]"
    rej = f1(c["coll"]) + r"$^\dagger$" if legacy else f1(c["rej"][0])
    return [rate, rej, f1(c["cs"], 0), f1(c["cm"]),
            f"{c['gap'][0]:.2f} [{c['gap'][1]:.2f},{c['gap'][2]:.2f}]",
            f1(c["eff"], 2), f1(c["time"], 0)]


HEAD7 = (r"\textbf{Reach} [95\% CI, \%] & \textbf{Gate rej.} & \textbf{Static} & "
         r"\textbf{Mover} & \textbf{Min gap (m)} [95\% CI] & \textbf{Eff.} & \textbf{Time (s)}")
NOTE = (r"Reach: runs reaching the goal, Wilson interval. Gate rej.: mean steps per "
        r"run in which the safety gate refused the commanded motion. Static: total "
        r"static-contact steps over the cell (must be 0). Mover: mean steps per run "
        r"in contact with a mover. Min gap: mean over runs of the per-run minimum "
        r"surface gap, bootstrap interval. $\dagger$ = legacy pre-fix table, where "
        r"this column is the old collision-step count.")


def write_table(path, key_heads, rows, caption, label, legacy, align=None):
    nk = len(key_heads)
    align = align or ("l" * nk + "rrrrrrr")
    with open(path, "w") as fh:
        fh.write("% AUTO-GENERATED by matlab/make_paper_figures.py -- do not edit\n")
        fh.write("\\begin{table}[H]\n\\centering\\scriptsize\n")
        fh.write("\\begin{tabular}{@{}" + align + "@{}}\n\\toprule\n")
        fh.write(" & ".join(f"\\textbf{{{h}}}" for h in key_heads) + " & " + HEAD7 + " \\\\\n\\midrule\n")
        if not rows:
            fh.write(f"\\multicolumn{{{nk+7}}}{{c}}{{\\emph{{not in the results table yet -- run the grid}}}} \\\\\n")
        for keys, c in rows:
            if keys is None:
                fh.write("\\midrule\n")
                continue
            fh.write(" & ".join([tx(k) for k in keys] + cell_cols(c, legacy)) + " \\\\\n")
        fh.write("\\bottomrule\n\\end{tabular}\n")
        fh.write(f"\\caption{{{caption} {NOTE}}}\n\\label{{{label}}}\n\\end{{table}}\n")


def md_table(fh, title, key_heads, rows, legacy):
    fh.write(f"\n### {title}\n\n")
    if not rows:
        fh.write("_not in the results table yet -- run the grid_\n")
        return
    heads = key_heads + ["reach [95% CI %]", "collision steps (legacy)" if legacy
                         else "gate rej./run", "static contact (total)",
                         "mover contact/run", "min gap m [95% CI]", "path eff.", "time s"]
    fh.write("| " + " | ".join(heads) + " |\n|" + "---|" * len(heads) + "\n")
    for keys, c in rows:
        if keys is None:
            continue
        cols = [x.replace(r"$^\dagger$", "") for x in cell_cols(c, legacy)]
        fh.write("| " + " | ".join([str(k) for k in keys] + cols) + " |\n")


def build_tables(T):
    """exp tag -> (title, key heads, rows[(keys, cell)])"""
    out = {}

    def rows_for(exp, specs, block=None):
        rows, last = [], None
        for keys, q in specs:
            c = agg(cell(T, exp=exp, **q))
            if c is None:
                continue
            if block is not None and last is not None and keys[block] != last:
                rows.append((None, None))
            last = keys[block] if block is not None else None
            rows.append((keys, c))
        return rows

    E = lambda e: T[T["exp"] == e]  # noqa: E731
    out["e1"] = ("E1 -- steering laws (lidar-288, sigma0 = 0.02, square s = 1.18 m)",
                 ["Law"], rows_for("E1", [([LAB[a]], dict(algo=a)) for a in ALGOS]))
    sig = sorted(set(E("E2")["sigma0"]))
    out["e2"] = ("E2 -- noise ablation", ["Law", "sigma0"],
                 rows_for("E2", [([LAB[a], f"{s:g}"], dict(algo=a, sigma0=float(s)))
                                 for a in ALGOS for s in sig], block=0))
    out["e3"] = ("E3 -- beam sweep (sigma0 = 0.10)", ["Law", "Sensor"],
                 rows_for("E3", [([LAB[a], f"{'lidar' if b >= 288 else 'tof'}-{b}"],
                                  dict(algo=a, beams=b))
                                 for a in ALGOS for b in (288, 64, 16, 8)], block=0))
    out["e4"] = ("E4 -- formation shapes under P-VFH (the P-VFH rows of E5)", ["Shape"],
                 rows_for("E5", [([f], dict(algo="pvfh_prob", formation=f)) for f in FORMS]))
    out["e5"] = ("E5 -- formation x steering law (s = 0.60 m)", ["Law", "Shape"],
                 rows_for("E5", [([LAB[a], f], dict(algo=a, formation=f))
                                 for a in ALGOS for f in FORMS], block=0))
    sig6 = sorted(set(E("E6")["sigma0"]))
    out["e6"] = ("E6 -- matched sensor comparison", ["Sensor", "sigma0", "Law"],
                 rows_for("E6", [([f"{k}-{b}", f"{s:g}", LAB[a]],
                                  dict(sense_kind=k, beams=b, sigma0=float(s), algo=a))
                                 for k, b in E6_CFG for s in sig6 for a in ALGOS], block=0))
    ws = sorted(set(E("E7")["world"]))
    sig7 = sorted(set(E("E7")["sigma0"]))
    out["e7"] = ("E7 -- second and third environment classes", ["World", "sigma0", "Law"],
                 rows_for("E7", [([w.replace("swarm_world_", "").replace(".sdf", ""),
                                   f"{s:g}", LAB[a]], dict(world=w, sigma0=float(s), algo=a))
                                 for w in ws for s in sig7 for a in ALGOS], block=0))
    ms = sorted(set(E("E8")["mover_scale"]))
    out["e8"] = ("E8 -- traffic-speed sweep", ["Law", "Mover speed x"],
                 rows_for("E8", [([LAB[a], f"{m:g}"], dict(algo=a, mover_scale=float(m)))
                                 for a in ALGOS for m in ms], block=0))
    for tag, col, heads, title, fmt in (
            ("e10", "loc_sigma", ["Law", "Pose noise (m)"], "E10 -- localisation error", "{:g}"),
            ("e11", "n_movers", ["Law", "Movers"], "E11 -- traffic density", "{:g}"),
            ("e12", "comm_drop", ["Law", "Drop prob."], "E12 -- command-update loss", "{:g}")):
        ex = tag.upper()
        lv = sorted(set(E(ex)[col]))
        out[tag] = (title, heads,
                    rows_for(ex, [([LAB[a], fmt.format(v)], {"algo": a, col: float(v)})
                                  for a in ALGOS for v in lv], block=0))
    sig13 = sorted(set(E("E13")["sigma0"]))
    out["e13"] = ("E13 -- randomised layouts (pooled over layouts and seeds)",
                  ["sigma0", "Law"],
                  rows_for("E13", [([f"{s_:g}", LAB[a]], dict(sigma0=float(s_), algo=a))
                                   for s_ in sig13 for a in ALGOS], block=0))
    r9 = []
    for a in ALGOS:
        for lab, c in (("ON", agg(cell(T, exp="E5", formation="line", algo=a))),
                       ("OFF", agg(cell(T, exp="E9", algo=a)))):
            if c is not None and not E("E9").empty and a in set(E("E9")["algo"]):
                r9.append(([LAB[a], lab], c))
    out["e9"] = ("E9 -- anti-deadlock layer ablation (line formation)",
                 ["Law", "Layer"], r9)
    r14 = []
    for a in ALGOS:
        if E("E14").empty or a not in set(E("E14")["algo"]):
            continue
        for lab, c in (("OFF", agg(cell(T, exp="E1", algo=a))),
                       ("ON", agg(cell(T, exp="E14", algo=a)))):
            if c is not None:
                r14.append(([LAB[a], lab], c))
    out["e14"] = ("E14 -- load-carrying motion limits", ["Law", "Limits"], r14)
    return out


def stats_rows(T):
    """Paired tests on E2 (and E7): P-VFH minus each deterministic law."""
    rows = []
    for exp in ("E2", "E7", "E13"):
        E = T[T["exp"] == exp]
        if E.empty:
            continue
        legacy = E["n_reject"].isna().all()
        E = E.copy()
        if exp == "E13":                       # pool layouts: pair on (layout, seed)
            E["seed"] = E["world"] + "#" + E["seed"].astype(str)
            E["world"] = "10 random layouts"
        for w in sorted(set(E["world"])):
            Ew = E[E["world"] == w]
            for base in ("pvfh_filt", "pvfh", "vfh"):
                if base not in set(Ew["algo"]) or "pvfh_prob" not in set(Ew["algo"]):
                    continue
                for s in sorted(set(Ew["sigma0"])):
                    c = Ew[np.isclose(Ew["sigma0"], s)]
                    res = []
                    for metric in ("gap", "collisions" if legacy else "n_reject"):
                        P = c.pivot_table(index="seed", columns="algo", values=metric)
                        if "pvfh_prob" not in P or base not in P:
                            res.append((np.nan,) * 4 + (0,))
                            continue
                        res.append(paired_test((P["pvfh_prob"] - P[base]).to_numpy()))
                    R = c.pivot_table(index="seed", columns="algo", values="reached")
                    b = int(((R["pvfh_prob"] == 1) & (R[base] == 0)).sum())
                    cc = int(((R["pvfh_prob"] == 0) & (R[base] == 1)).sum())
                    pm = np.nan
                    if _st is not None and b + cc > 0:
                        pm = float(_st.binomtest(min(b, cc), b + cc, 0.5).pvalue)
                    rows.append((exp, w.replace("swarm_world_", "").replace(".sdf", ""),
                                 LAB[base], s, res[0], res[1], b, cc, pm))
    return rows


def pv(p):
    if p is None or (isinstance(p, float) and np.isnan(p)):
        return "--"
    return "$<$0.001" if p < 0.001 else f"{p:.3f}"


def write_stats(path, rows, legacy):
    with open(path, "w") as fh:
        fh.write("% AUTO-GENERATED by matlab/make_paper_figures.py -- do not edit\n")
        fh.write("\\begin{table}[H]\n\\centering\\scriptsize\n")
        fh.write("\\begin{tabular}{@{}lllrrrrrrr@{}}\n\\toprule\n")
        fh.write(r"\textbf{Exp.} & \textbf{World} & \textbf{vs} & $\sigma_0$ & "
                 r"$\Delta$\textbf{gap (m)} [95\% CI] & $p_W$ & "
                 + (r"$\Delta$\textbf{coll.}$^\dagger$" if legacy else r"$\Delta$\textbf{rej.}")
                 + r" [95\% CI] & $p_W$ & \textbf{win/loss} & $p_M$ \\" + "\n\\midrule\n")
        if not rows:
            fh.write("\\multicolumn{10}{c}{\\emph{not in the results table yet}} \\\\\n")
        for exp, w, base, s, g, r, b, c, pm in rows:
            fh.write(f"{exp} & {tx(w)} & {base} & {s:g} & "
                     f"{g[0]:+.2f} [{g[1]:+.2f},{g[2]:+.2f}] & {pv(g[3])} & "
                     f"{r[0]:+.1f} [{r[1]:+.1f},{r[2]:+.1f}] & {pv(r[3])} & "
                     f"{b}/{c} & {pv(pm)} \\\\\n")
        fh.write("\\bottomrule\n\\end{tabular}\n")
        fh.write(r"\caption{Paired-seed tests, P-VFH minus the named deterministic law "
                 r"(positive $\Delta$gap and negative $\Delta$rej.\ favour P-VFH). "
                 r"$p_W$: two-sided Wilcoxon signed-rank on the per-seed differences; "
                 r"win/loss: seeds where only P-VFH / only the baseline reached the goal, "
                 r"$p_M$: exact McNemar (binomial) test on those discordant pairs. "
                 r"Intervals are percentile bootstrap. No multiplicity correction is "
                 r"applied; read the $p$-values with the number of rows in mind.}"
                 "\n\\label{tab:stats}\n\\end{table}\n")


def write_provenance(path, T, csv, legacy):
    n = len(T)
    stack = ", ".join(sorted(set(T["stack"].astype(str))))
    exps = ", ".join(f"{e} ({(T['exp'] == e).sum()})" for e in sorted(set(T["exp"])))
    when = _dt.datetime.fromtimestamp(os.path.getmtime(csv)).strftime("%Y-%m-%d %H:%M")
    cs = T["coll_static"]
    gate = ("not recorded (legacy table)" if cs.isna().all()
            else f"{int(np.nansum(cs))} static-contact steps in total (must be 0)")
    with open(path, "w") as fh:
        fh.write("% AUTO-GENERATED by matlab/make_paper_figures.py -- do not edit\n")
        fh.write("\\begin{keybox}{Provenance of every number and figure in this chapter}\n")
        if legacy:
            fh.write("\\textbf{LEGACY DATA.} These tables and figures are drawn from the "
                     "\\emph{pre-fix} Octave results of September 2026, produced before the "
                     "audit of \\S\\ref{sec:audit}. They are shown only so the document "
                     "compiles with one consistent source; they are \\emph{superseded} the "
                     "moment the corrected grid is run (\\S\\ref{sec:howto}). Experiments "
                     "added by the audit (E7--E9, the DWA/FGM baselines) are absent.\n\n")
        fh.write(f"Source table: \\code{{{tx(os.path.relpath(csv, os.path.join(HERE, '..')))}}} "
                 f"(modified {when}); stack: {tx(stack)}; {n} runs: {exps}. "
                 f"Safety-gate soundness check: {gate}. Every table and figure below is "
                 "regenerated from this one file by \\code{make\\_paper\\_figures.py}; "
                 "none is typed by hand.\n")
        fh.write("\\end{keybox}\n")


def write_log(path, T, tabs, srows, csv, legacy):
    with open(path, "w") as fh:
        fh.write("# Results log (auto-generated)\n\n")
        fh.write("Generated by `matlab/make_paper_figures.py` from "
                 f"`{os.path.relpath(csv, os.path.join(HERE, '..'))}` "
                 f"({len(T)} runs, stack: {', '.join(sorted(set(T['stack'].astype(str))))}).\n\n")
        if legacy:
            fh.write("> **LEGACY DATA** -- pre-fix Octave results (September 2026). "
                     "Superseded as soon as the corrected grid is run; see "
                     "`docs/AUDIT_CHANGELOG.md`.\n\n")
        fh.write("Conventions: reach = runs reaching the goal with Wilson 95% interval; "
                 "gate rej. = mean steps/run where the safety gate refused the commanded "
                 "motion; static contact = total static-contact steps in the cell (must "
                 "be 0); min gap = mean of per-run minimum surface gap (clearance - 0.18 m) "
                 "with bootstrap 95% interval.\n")
        for k in ("e1", "e2", "e3", "e4", "e5", "e6", "e7", "e8", "e9", "e10",
                  "e11", "e12", "e13", "e14"):
            title, heads, rows = tabs[k]
            md_table(fh, title, heads, rows, legacy)
        fh.write("\n### Paired-seed tests (P-VFH minus baseline)\n\n")
        if srows:
            fh.write("| exp | world | vs | sigma0 | d gap m [CI] | p (Wilcoxon) | "
                     "d rej [CI] | p (Wilcoxon) | win/loss | p (McNemar) |\n" + "|---" * 10 + "|\n")
            for exp, w, base, s, g, r, b, c, pm in srows:
                fh.write(f"| {exp} | {w} | {base} | {s:g} | {g[0]:+.2f} [{g[1]:+.2f},{g[2]:+.2f}] "
                         f"| {pv(g[3]).replace('$<$', '<')} | {r[0]:+.1f} [{r[1]:+.1f},{r[2]:+.1f}] "
                         f"| {pv(r[3]).replace('$<$', '<')} | {b}/{c} | {pv(pm).replace('$<$', '<')} |\n")
        else:
            fh.write("_no paired cells in the table_\n")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="figures + tables + results log")
    ap.add_argument("--csv", default=os.path.join(HERE, "logs", "paper", "results_all.csv"))
    ap.add_argument("--runs", default=os.path.join(HERE, "logs", "paper", "runs"))
    ap.add_argument("--legacy", action="store_true",
                    help="input is the pre-fix Octave table (old column set and "
                         "old per-step log names)")
    doc = os.path.normpath(os.path.join(HERE, "..", "docs"))
    ap.add_argument("--outdir", default=os.path.join(doc, "simulation_report", "figures"))
    ap.add_argument("--gendir", default=os.path.join(doc, "simulation_report", "generated"))
    ap.add_argument("--log", default=os.path.join(doc, "RESULTS_LOG.md"))
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    os.makedirs(a.gendir, exist_ok=True)

    T = load(a.csv, a.legacy)
    legacy = bool(T["n_reject"].isna().all())
    R = Runs(a.runs)
    W = worlds(T)
    F = lambda n: os.path.join(a.outdir, n)  # noqa: E731
    print(f"{a.csv}: {len(T)} runs; legacy={legacy}")

    fig_e1(T, F("algo_bars.pdf"))
    ladder = [x for x in ALGOS if x in set(T[T["exp"] == "E2"]["algo"])]
    sig = sorted(set(T[T["exp"] == "E2"]["sigma0"]))
    if ladder:
        three_metric_lines(T, "E2", "sigma0", [float(s) for s in sig], ladder,
                           r"range-noise std $\sigma_0$ (m)",
                           "E2: noise ablation (mean, 95% CI)", F("ablation_noise.pdf"))
    else:
        pending(F("ablation_noise.pdf"), "E2")
    fig_paired(T, F("paired_difference.pdf"))
    a3 = [x for x in ALGOS if x in set(T[T["exp"] == "E3"]["algo"])]
    if a3:
        three_metric_lines(T, "E3", "beams", [288, 64, 16, 8], a3,
                           "beam count (288 = lidar, others = 60-deg ToF fan)",
                           "E3: beam sweep at $\\sigma_0$ = 0.10 (mean, 95% CI)",
                           F("sweep_tof.pdf"), xfmt="{}")
    else:
        pending(F("sweep_tof.pdf"), "E3")
    fig_overlays(T, R, W, F("trajectory_overlays.pdf"))
    fig_sketch(F("formation_sketch.pdf"))
    fig_trails(T, R, W, "E5", "pvfh_prob", FORMS,
               "E4: P-VFH centroid paths per formation shape, all seeds",
               F("formation_trails.pdf"))
    fig_heatmap(T, F("e5_heatmap.pdf"))
    fig_line_anatomy(T, R, W, F("e5_line_anatomy.pdf"))
    fig_line_timeseries(T, R, F("e5_stall_timeseries.pdf"))
    fig_e6(T, F("e6_sensor.pdf"))
    fig_e6_timeseries(T, R, F("e6_timeseries.pdf"))
    fig_e7(T, R, W, F("e7_environments.pdf"))
    a8 = [x for x in ALGOS if x in set(T[T["exp"] == "E8"]["algo"])]
    if a8:
        ms = sorted(set(T[T["exp"] == "E8"]["mover_scale"]))
        three_metric_lines(T, "E8", "mover_scale", [float(m) for m in ms], a8,
                           "mover speed multiplier (1 = the contract)",
                           "E8: traffic-speed sweep (mean, 95% CI)", F("e8_traffic.pdf"))
    else:
        pending(F("e8_traffic.pdf"), "E8 (traffic-speed sweep)")
    fig_e9(T, F("e9_adl.pdf"))
    for ex, col, xl, ttl, fn in (
            ("E10", "loc_sigma", "position-estimate noise $\\sigma_p$ (m); yaw noise = 20 deg per m",
             "E10: localisation error (mean, 95% CI)", "e10_localisation.pdf"),
            ("E11", "n_movers", "number of movers", "E11: traffic density (mean, 95% CI)",
             "e11_density.pdf"),
            ("E12", "comm_drop", "probability a steering update is lost",
             "E12: command-update loss (mean, 95% CI)", "e12_commloss.pdf")):
        ax_ = [x for x in ALGOS if x in set(T[T["exp"] == ex]["algo"])]
        if ax_:
            lv = sorted(set(T[T["exp"] == ex][col]))
            three_metric_lines(T, ex, col, [float(v) for v in lv], ax_, xl, ttl, F(fn))
        else:
            pending(F(fn), ex)
    fig_e13(T, W, F("e13_random.pdf"))
    fig_e14(T, F("e14_limits.pdf"))
    fig_theory(F("kernel_theory.pdf"))

    tabs = build_tables(T)
    caps = dict(e1="E1.", e2="E2, every cell.", e3="E3.", e4="E4.", e5="E5, every cell.",
                e6="E6, every cell.", e7="E7, every cell.", e8="E8.", e9="E9.",
                e10="E10.", e11="E11.", e12="E12.", e13="E13.", e14="E14.")
    for k, (title, heads, rows) in tabs.items():
        write_table(os.path.join(a.gendir, f"tab_{k}.tex"),
                    [h.replace("sigma0", r"$\sigma_0$") for h in heads], rows,
                    caps[k], f"tab:{k}", legacy)
    srows = stats_rows(T)
    write_stats(os.path.join(a.gendir, "tab_stats.tex"), srows, legacy)
    write_provenance(os.path.join(a.gendir, "provenance.tex"), T, a.csv, legacy)
    write_log(a.log, T, tabs, srows, a.csv, legacy)
    print(f"figures -> {a.outdir}\ntables  -> {a.gendir}\nlog     -> {a.log}")


if __name__ == "__main__":
    main()
