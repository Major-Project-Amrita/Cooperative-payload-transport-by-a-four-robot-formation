"""Every figure of docs/DOCUMENTATION.md, from the result files.

    python3 tools/make_doc_figures.py            # all figures -> docs/figures/
    python3 tools/make_doc_figures.py concept    # only names containing 'concept'

Figures whose input files are missing are skipped with a note, so the script
can be run at any stage of the runbook.
"""
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as MCircle, Polygon

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from ros2_swarm_transport.geometry import (Box, World, formation_clearance,   # noqa: E402
                                           formation_offsets, _convex_hull)
from ros2_swarm_transport.headless_runner import load_scenario, run_mission  # noqa: E402
from ros2_swarm_transport import vfh as V                                     # noqa: E402

FIG = os.path.join(ROOT, "docs", "figures")
RES = os.path.join(ROOT, "results")
WORLDS = os.path.join(PKG, "worlds")
os.makedirs(FIG, exist_ok=True)

# one colour per steering law, everywhere (validated categorical order)
ALGOS = ["vfh", "pvfh", "pvfh_filt", "pvfh_prob", "mpc", "dwa", "fgm", "apf"]
COL = {"vfh": "#2a78d6", "pvfh": "#eb6834", "pvfh_filt": "#1baf7a",
       "dwa": "#eda100", "fgm": "#e87ba4", "mpc": "#008300",
       "pvfh_prob": "#4a3aa7", "apf": "#e34948"}
NAME = {"apf": "APF", "vfh": "VFH", "fgm": "FGM", "dwa": "DWA", "mpc": "MPC",
        "pvfh": "pVFH", "pvfh_filt": "pVFH+F", "pvfh_prob": "P-VFH"}
C_M, C_P, C_X = "#2a78d6", "#eb6834", "#1baf7a"     # MATLAB, Python, physics
INK, MUTED, GRID = "#0b0b0b", "#898781", "#e6e5e0"
plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
    "savefig.facecolor": "#fcfcfb", "font.size": 9, "axes.edgecolor": MUTED,
    "axes.labelcolor": "#52514e", "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.6, "axes.titlesize": 10,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "legend.frameon": False,
    "lines.linewidth": 1.6, "figure.dpi": 150})


def save(fig, name):
    fig.savefig(os.path.join(FIG, name), bbox_inches="tight", dpi=150)
    plt.close(fig)
    print("  wrote docs/figures/" + name)


def draw_world(ax, world, movers=(), color="#c9c8c2"):
    for b in world.boxes:
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        cor = (np.array([[c, -s], [s, c]]) @ (np.array(
            [[1, 1], [1, -1], [-1, -1], [-1, 1]]) * b.h).T).T + b.c
        ax.add_patch(Polygon(cor, closed=True, fc=color, ec="none"))
    for ci in world.circles:
        ax.add_patch(MCircle(ci.c, ci.r, fc=color, ec="none"))
    for m in movers:
        ax.plot(m.route[:, 0], m.route[:, 1], color="#d03b3b", lw=1.0, ls=(0, (2, 2)))
        ax.plot(*m.route[0], marker="s", ms=4, color="#d03b3b")
    ax.set_aspect("equal")
    ax.grid(False)


def hull_patch(c, th, offs, r, **kw):
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    H = _convex_hull((R @ offs.T).T + c)
    pts = []
    n = len(H)
    for i in range(n):
        a, b = H[i], H[(i + 1) % n]
        e = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([e[1], -e[0]])
        prev = H[i - 1]
        e0 = (a - prev) / np.linalg.norm(a - prev)
        n0 = np.array([e0[1], -e0[0]])
        a0, a1 = math.atan2(n0[1], n0[0]), math.atan2(nrm[1], nrm[0])
        if a1 < a0:
            a1 += 2 * math.pi
        for t in np.linspace(a0, a1, 12):
            pts.append(a + r * np.array([math.cos(t), math.sin(t)]))
        pts.append(b + r * nrm)
    return Polygon(np.array(pts), closed=True, **kw)


# ------------------------------------------------------------------ concepts
def fig_concept_kernel():
    sh = 1.01 * 0 + (1.014 + 1.0) / math.sqrt(2 * math.log(1 / 0.3))
    d = np.linspace(0, 5, 400)
    fig, ax = plt.subplots(1, 2, figsize=(9.2, 3.3))
    for s, c in zip([0.0, 0.2, 0.5, 1.0], ["#104281", "#2a78d6", "#6da7ec", "#b7d3f6"]):
        ax[0].plot(d, V.expected_kernel(d, max(s, 1e-9), sh), color=c,
                   label=f"$s_b$ = {s:g} m" + (" (deterministic)" if s == 0 else ""))
    ax[0].axhline(0.3, color="#d03b3b", lw=1, ls="--")
    ax[0].text(4.95, 0.32, "threshold 0.3", ha="right", color="#d03b3b", fontsize=8)
    ax[0].set(xlabel="measured range $d$ (m)", ylabel="kernel value $u_b$",
              title="Expected danger kernel", xlim=(0, 5), ylim=(0, 1.02))
    ax[0].legend(fontsize=8)
    s = np.linspace(0, 3.2 * sh, 400)
    dr = [V.kernel_reaction_distance(x, sh) for x in s]
    ax[1].plot(s, dr, color="#4a3aa7")
    ax[1].axvline(1.757 * sh, color=MUTED, lw=1, ls="--")
    ax[1].text(1.757 * sh + 0.05, 0.3, r"$s^* = 1.757\,\sigma_h$", color="#52514e", fontsize=8)
    ax[1].axvspan(0, 0.4, color="#1baf7a", alpha=0.12, lw=0)
    ax[1].text(0.2, 0.3, "sensor range\nof the study", ha="center", fontsize=7, color="#52514e")
    ax[1].set(xlabel="range noise $s_b$ (m)", ylabel="reaction distance $d_{react}$ (m)",
              title="Reaction distance grows with noise (Prop. 2)", ylim=(0, None), xlim=(0, s[-1]))
    save(fig, "concept_kernel.png")


def fig_concept_histogram():
    world, movers = load_scenario(os.path.join(WORLDS, "swarm_world_dynamic.sdf"),
                                  os.path.join(WORLDS, "dynamic_obstacles.json"))
    pos, th, t = np.array([-6.2, 0.6]), 0.35, 14.0
    goal = np.array([12.0, 0.0])
    offs = formation_offsets("square", 4, 1.18)
    margin = float(np.linalg.norm(offs, axis=1).max()) + 0.18
    cfg = V.VfhConfig(sense={"sigma0": 0.05})
    vec, info = V.pvfh_prob(pos, world, movers, t, goal, margin, cfg, 1e-6, th,
                            np.random.default_rng(3), return_info=True)
    fig = plt.figure(figsize=(10, 3.9))
    ax = fig.add_axes([0.04, 0.1, 0.5, 0.82])
    draw_world(ax, world)
    for m in movers:
        mk = m.eval_at(t)
        ax.add_patch(plt.Rectangle(mk.c - np.array(mk.size) / 2, *mk.size, fc="#eb6834", ec="none")
                     if mk.shape == "box" else MCircle(mk.c, mk.r, fc="#eb6834", ec="none"))
    r, a = info["ranges"], info["angles"]
    for k in range(0, len(r), 3):
        ax.plot([pos[0], pos[0] + r[k] * math.cos(a[k])],
                [pos[1], pos[1] + r[k] * math.sin(a[k])], color="#2a78d6", lw=0.35, alpha=0.6)
    ax.add_patch(hull_patch(pos, th, offs, 0.18, fc="none", ec=INK, lw=1.2))
    ch = info["chosen_heading"]
    ax.annotate("", xy=pos + 2.2 * np.array([math.cos(ch), math.sin(ch)]), xytext=pos,
                arrowprops=dict(arrowstyle="-|>", color="#4a3aa7", lw=2))
    gh = math.atan2(*(goal - pos)[::-1])
    ax.annotate("", xy=pos + 2.2 * np.array([math.cos(gh), math.sin(gh)]), xytext=pos,
                arrowprops=dict(arrowstyle="-|>", color="#008300", lw=1.4, ls="--"))
    ax.set(xlim=(-12.5, 1.5), ylim=(-5, 5.6), title="One noisy scan from the formation centre",
           xlabel="x (m)", ylabel="y (m)")
    ax2 = fig.add_axes([0.6, 0.12, 0.38, 0.78])
    S = cfg.num_sectors
    ang = np.degrees(-math.pi + 2 * math.pi * np.arange(S) / S)
    free = info["hist"] < cfg.threshold
    ax2.bar(ang[~free], info["hist"][~free], width=4.2, color="#c9c8c2", label="blocked sector")
    ax2.bar(ang[free], info["hist"][free], width=4.2, color="#1baf7a", label="free sector")
    ax2.axhline(cfg.threshold, color="#d03b3b", lw=1, ls="--")
    ax2.axvline(math.degrees(ch), color="#4a3aa7", lw=2, label="chosen valley")
    ax2.axvline(math.degrees(gh), color="#008300", lw=1.4, ls="--", label="goal bearing")
    ax2.set(xlabel="bearing (deg)", ylabel="smoothed histogram value", xlim=(-180, 180),
            title="Polar histogram, threshold and choice")
    ax2.legend(fontsize=7, loc="upper right")
    save(fig, "concept_histogram.png")


def fig_concept_formations():
    fig, ax = plt.subplots(1, 4, figsize=(10, 2.9))
    for a, (kind, s) in zip(ax, [("square", 1.18), ("diamond", 0.6), ("triangle", 0.6), ("line", 0.6)]):
        offs = formation_offsets(kind, 3 if kind == "triangle" else 4, s)
        a.add_patch(hull_patch(np.zeros(2), 0.0, offs, 0.18, fc="#cde2fb", ec="#2a78d6", lw=1.2))
        for o in offs:
            a.add_patch(MCircle(o, 0.18, fc="#2a78d6", ec="#fcfcfb", lw=1.5))
        rc = float(np.linalg.norm(offs, axis=1).max())
        a.add_patch(MCircle((0, 0), rc + 0.18, fc="none", ec=MUTED, lw=0.8, ls="--"))
        a.annotate("", xy=(0.55, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=INK))
        a.set(xlim=(-1.25, 1.25), ylim=(-1.25, 1.25), aspect="equal",
              title=f"{kind}, s = {s:g} m\nneeds {2*(rc+0.18):.2f} m of corridor")
        a.grid(False)
        a.set_xticks([]); a.set_yticks([])
    save(fig, "concept_formations.png")


def fig_concept_gate():
    offs = formation_offsets("square", 4, 1.18)
    rng = np.random.default_rng(4)
    for _ in range(4000):
        box = Box(rng.uniform(-2.2, 2.2, 2), 0.0, rng.uniform(0.3, 0.9, 2), "b")
        th = rng.uniform(-3, 3)
        w = World(boxes=[box])
        ce, he = formation_clearance(np.zeros(2), th, offs, w, 0.18, sat="exact")
        _, hm = formation_clearance(np.zeros(2), th, offs, w, 0.18, sat="matlab")
        if hm and not he and ce > 0.45:
            break
    fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.6))
    for a, (order, ttl) in zip(ax, [
            ([(1, 1), (1, -1), (-1, 1), (-1, -1)], "Reference gate: corners (+,+) (+,-) (-,+) (-,-)\n"
             "'edges' = 2 sides + 2 diagonals -> one normal never tested"),
            ([(1, 1), (1, -1), (-1, -1), (-1, 1)], "Exact gate: corners in cyclic order\nall four sides tested")]):
        a.add_patch(hull_patch(np.zeros(2), th, offs, 0.18, fc="#cde2fb", ec="#2a78d6", lw=1.2))
        cor = np.array(order) * box.h + box.c
        a.add_patch(Polygon(np.array([(1, 1), (1, -1), (-1, -1), (-1, 1)]) * box.h + box.c,
                            closed=True, fc="#c9c8c2", ec="none"))
        for i in range(4):
            p, q = cor[i], cor[(i + 1) % 4]
            a.plot([p[0], q[0]], [p[1], q[1]], color="#d03b3b" if "Reference" in ttl else "#008300", lw=1.6)
        a.set(xlim=(-3, 3), ylim=(-3, 3), aspect="equal", title=ttl)
        a.title.set_fontsize(8)
        a.grid(False)
        a.text(0, -2.75, "verdict: " + ("OVERLAP (false)" if "Reference" in ttl else f"clear, gap {ce-0.18:.2f} m"),
               ha="center", fontsize=9, color=INK)
    save(fig, "concept_gate.png")


def fig_concept_filter():
    rng = np.random.default_rng(1)
    t = np.arange(0, 8, 0.1)
    truth = np.where(t < 4.5, 3.2 - 0.25 * t, 1.1 + 0.02 * (t - 4.5))
    fig, ax = plt.subplots(1, 2, figsize=(9.4, 3.1), sharey=True)
    for a, s in zip(ax, (0.02, 0.20)):
        z = truth + s * rng.normal(size=len(t))
        st, xs, sp = {}, [], []
        for k in range(len(t)):
            x, p = V.range_filter(np.array([z[k]]), np.array([s]), t[k], 0.08, st)
            xs.append(x[0]); sp.append(p[0])
        xs, sp = np.array(xs), np.array(sp)
        a.plot(t, truth, color=INK, lw=1, label="true range")
        a.plot(t, z, ".", color="#eb6834", ms=3, label="measurement")
        a.plot(t, xs, color="#4a3aa7", label="filtered mean")
        a.fill_between(t, xs - 2 * sp, xs + 2 * sp, color="#4a3aa7", alpha=0.15, lw=0, label=r"$\pm 2\sigma$ posterior")
        a.set(xlabel="time (s)", title=f"sensor $s_b$ = {s:g} m  (steady gain {((sp[-1]/s)**2):.2f})")
    ax[0].set_ylabel("range along one beam (m)")
    ax[0].annotate("a mover enters the beam:\n3-sigma reset, no lag", xy=(4.5, 1.2), xytext=(5.2, 2.6),
                   fontsize=8, arrowprops=dict(arrowstyle="->", color=MUTED), color="#52514e")
    ax[1].legend(fontsize=7)
    save(fig, "concept_range_filter.png")


# ------------------------------------------------------------- replication
def fig_xcheck():
    world, movers = load_scenario(os.path.join(WORLDS, "swarm_world_dynamic.sdf"),
                                  os.path.join(WORLDS, "dynamic_obstacles.json"))
    fig = plt.figure(figsize=(10, 6.2))
    ax = fig.add_axes([0.06, 0.47, 0.9, 0.5])
    draw_world(ax, world, movers)
    ax2 = fig.add_axes([0.06, 0.08, 0.9, 0.3])
    found = False
    for algo in ALGOS:
        mf = os.path.join(ROOT, "matlab_ref/logs", f"x_{algo}_s0.csv")
        pf = os.path.join(RES, "xcheck", f"py_matlab_{algo}_s0.csv")
        if not (os.path.exists(mf) and os.path.exists(pf)):
            continue
        found = True
        m, p = pd.read_csv(mf), pd.read_csv(pf)
        n = min(len(m), len(p))
        ax.plot(m.leader_x, m.leader_y, color=COL[algo], lw=3.2, alpha=0.35, solid_capstyle="round")
        ax.plot(p.leader_x, p.leader_y, color=COL[algo], lw=0.9, label=NAME[algo])
        d = np.hypot(p.leader_x[:n] - m.leader_x[:n], p.leader_y[:n] - m.leader_y[:n])
        ax2.semilogy(m.t[:n], np.maximum(d, 1e-9), color=COL[algo], lw=1)
    if not found:
        plt.close(fig); print("  (skip xcheck: run tools/deterministic_xcheck.py)"); return
    ax.plot(-12, 0, "s", color=INK, ms=6); ax.plot(12, 0, "*", color=INK, ms=11)
    ax.set(xlim=(-15.4, 15.4), ylim=(-8.3, 8.3), ylabel="y (m)",
           title="Zero-noise missions: MATLAB (wide, pale) under Python (thin) - centroid paths")
    ax.legend(ncol=8, fontsize=7, loc="upper center")
    ax2.axhline(5e-7, color=MUTED, lw=1, ls="--")
    ax2.text(1, 7e-7, "CSV print precision (6 decimals)", fontsize=7, color="#52514e")
    ax2.text(1, 2.5e-9, "all eight laws: the two logs are identical at the printed precision, every step "
             "(difference 0, drawn at the axis floor)", fontsize=7, color="#52514e")
    ax2.set(xlabel="mission time (s)", ylabel="|MATLAB - Python| (m)", ylim=(1e-9, 1e-2),
            title="Centroid position difference, every step of the mission")
    save(fig, "xcheck_trajectories.png")


def _cells():
    f = os.path.join(RES, "cross_stack.csv")
    return pd.read_csv(f) if os.path.exists(f) else None


def fig_cross_scatter():
    X = _cells()
    if X is None:
        print("  (skip cross scatter: run tools/cross_stack.py)"); return
    fig, ax = plt.subplots(1, 3, figsize=(10.5, 3.5))
    for a, (c, lab, lim) in zip(ax, [("time", "mean mission time (s)", None),
                                     ("rej", "gate rejections per run", None),
                                     ("gap", "mean minimum gap (m)", None)]):
        x, y = X[c + "_matlab"], X[c + "_py"]
        if c == "rej":
            x, y = x + 0.5, y + 0.5
            a.set_xscale("log"); a.set_yscale("log")
        lo, hi = min(x.min(), y.min()), max(x.max(), y.max())
        a.plot([lo, hi], [lo, hi], color=MUTED, lw=1)
        a.scatter(x[X.agree], y[X.agree], s=14, color=C_M, ec="#fcfcfb", lw=0.5, label="intervals overlap")
        a.scatter(x[~X.agree], y[~X.agree], s=22, color="#d03b3b", marker="D", ec="#fcfcfb", lw=0.5,
                  label="disagree")
        r = np.corrcoef(X[c + "_matlab"], X[c + "_py"])[0, 1]
        a.set(xlabel="MATLAB: " + lab + (" (+0.5)" if c == "rej" else ""), ylabel="Python",
              title=f"{lab.split('(')[0].strip()}  (r = {r:.3f})")
    ax[0].legend(fontsize=7)
    save(fig, "cross_stack_scatter.png")


def _both():
    fm, fp = os.path.join(ROOT, "bundle/reference/results_all.csv"), os.path.join(RES, "results_headless.csv")
    if not os.path.exists(fp):
        return None, None
    return pd.read_csv(fm), pd.read_csv(fp)


def _ci(x, rng):
    x = np.asarray(x, float)
    m = x[rng.integers(0, len(x), (1000, len(x)))].mean(axis=1)
    return x.mean(), x.mean() - np.percentile(m, 2.5), np.percentile(m, 97.5) - x.mean()


def fig_e1():
    M, P = _both()
    if M is None:
        print("  (skip e1: grid not run)"); return
    rng = np.random.default_rng(0)
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.3))
    for a, (col, lab) in zip(ax, [("total_time", "mission time (s)"), ("n_reject", "gate rejections per run"),
                                  ("gap", "minimum gap to anything (m)")]):
        for k, (D, c, nm) in enumerate(((M, C_M, "MATLAB"), (P, C_P, "Python"))):
            D = D[D.exp == "E1"].assign(gap=lambda d: d.min_clear - 0.18)
            st = [_ci(D[D.algo == al][col], rng) for al in ALGOS]
            x = np.arange(len(ALGOS)) + (k - 0.5) * 0.36
            a.bar(x, [s[0] for s in st], 0.32, color=c, label=nm)
            a.errorbar(x, [s[0] for s in st], yerr=[[s[1] for s in st], [s[2] for s in st]],
                       fmt="none", ecolor=INK, lw=0.8)
        a.set_xticks(range(len(ALGOS))); a.set_xticklabels([NAME[x] for x in ALGOS], rotation=35, ha="right")
        a.set(title=lab); a.axhline(0, color=MUTED, lw=0.8); a.grid(axis="x", visible=False)
    ax[0].legend(fontsize=8)
    fig.suptitle("E1 - eight steering laws, lidar-288, sigma0 = 0.02, 10 seeds per stack (bars: mean, whiskers: 95 % CI)",
                 fontsize=9, y=1.02)
    save(fig, "e1_both_stacks.png")


def fig_e2():
    M, P = _both()
    if M is None:
        print("  (skip e2: grid not run)"); return
    rng = np.random.default_rng(0)
    ladder = ["vfh", "pvfh", "pvfh_filt", "pvfh_prob"]
    fig, ax = plt.subplots(2, 4, figsize=(11, 5), sharex=True, sharey="row")
    for j, al in enumerate(ladder):
        for i, (col, lab) in enumerate([("total_time", "mission time (s)"), ("n_reject", "gate rejections per run")]):
            a = ax[i, j]
            for D, c, nm, mk in ((M, C_M, "MATLAB", "o"), (P, C_P, "Python", "s")):
                d = D[(D.exp == "E2") & (D.algo == al)]
                sg = sorted(d.sigma0.unique())
                st = [_ci(d[d.sigma0 == s][col], rng) for s in sg]
                a.errorbar(range(len(sg)), [s[0] for s in st], yerr=[[s[1] for s in st], [s[2] for s in st]],
                           color=c, marker=mk, ms=4, lw=1.4, capsize=2, label=nm)
            a.set_xticks(range(len(sg))); a.set_xticklabels([f"{s:g}" for s in sg])
            if i == 0:
                a.set_title(NAME[al], color=COL[al])
            if j == 0:
                a.set_ylabel(lab)
            if i == 1:
                a.set_xlabel("range noise sigma0 (m)")
    ax[0, 0].legend(fontsize=8)
    fig.suptitle("E2 - noise ablation ladder: horizon (VFH->pVFH), filter (pVFH->pVFH+F), kernel (pVFH+F->P-VFH); 20 seeds",
                 fontsize=9, y=1.0)
    save(fig, "e2_noise_both_stacks.png")


def fig_gate_effect():
    fp, fe = os.path.join(RES, "results_headless.csv"), os.path.join(RES, "results_headless_exactgate.csv")
    if not (os.path.exists(fp) and os.path.exists(fe)):
        print("  (skip gate effect: exact-gate grid not run)"); return
    A, B = pd.read_csv(fp), pd.read_csv(fe)
    k = ["exp", "key"]
    D = A.merge(B, on=k, suffixes=("_m", "_e"))
    D = D[D.exp.isin(["E1", "E2", "E5", "E7", "E13"])]
    rng = np.random.default_rng(0)
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.3))
    for a, (col, lab) in zip(ax, [("n_reject", "gate rejections per run"), ("total_time", "mission time (s)"),
                                  ("reached", "runs reaching the goal (%)")]):
        for kk, (sf, c, nm) in enumerate((("_m", "#898781", "reference gate"), ("_e", "#4a3aa7", "exact gate"))):
            st = [_ci(D[D.algo_m == al][col + sf] * (100 if col == "reached" else 1), rng) for al in ALGOS]
            x = np.arange(len(ALGOS)) + (kk - 0.5) * 0.36
            a.bar(x, [s[0] for s in st], 0.32, color=c, label=nm)
            a.errorbar(x, [s[0] for s in st], yerr=[[s[1] for s in st], [s[2] for s in st]], fmt="none",
                       ecolor=INK, lw=0.8)
        a.set_xticks(range(len(ALGOS))); a.set_xticklabels([NAME[x] for x in ALGOS], rotation=35, ha="right")
        a.set(title=lab); a.grid(axis="x", visible=False)
    ax[0].legend(fontsize=8)
    fig.suptitle(f"Effect of the gate's separating-axis defect: same seeds, same code, only the gate differs "
                 f"({len(D)} paired runs, E1+E2+E5+E7+E13)", fontsize=9, y=1.02)
    save(fig, "gate_effect.png")


# ----------------------------------------------------------- the warehouse
def _wh():
    return load_scenario(os.path.join(WORLDS, "swarm_world_awswh.sdf"),
                         os.path.join(WORLDS, "dynamic_obstacles_awswh.json"))


def _wh_axes(ax, world, movers):
    draw_world(ax, world, movers)
    c = json.load(open(os.path.join(WORLDS, "dynamic_obstacles_awswh.json")))
    wp = np.array([c["start"]] + c["waypoints"])
    ax.plot(wp[:, 0], wp[:, 1], color=MUTED, lw=0.8, ls=":")
    ax.plot(*wp[0], "s", color=INK, ms=6)
    ax.plot(wp[1:-1, 0], wp[1:-1, 1], "o", color=INK, ms=3)
    ax.plot(*wp[-1], "*", color=INK, ms=11)
    ax.text(wp[0, 0] + 0.4, wp[0, 1] - 0.9, "A", fontsize=10, weight="bold")
    ax.text(wp[-1, 0] + 0.4, wp[-1, 1] + 0.3, "B", fontsize=10, weight="bold")
    ax.set(xlim=(-11.5, 11.5), ylim=(-13.8, 13.8), xlabel="x (m)", ylabel="y (m)")


def fig_warehouse_paths():
    world, movers = _wh()
    fig, ax = plt.subplots(1, 2, figsize=(10, 6.4))
    for a, kin in zip(ax, (False, True)):
        _wh_axes(a, world, movers)
        for algo in ALGOS:
            rows, S = run_mission(os.path.join(WORLDS, "swarm_world_awswh.sdf"),
                                  os.path.join(WORLDS, "dynamic_obstacles_awswh.json"),
                                  algo=algo, seed=1, verbose=False, kin=kin)
            d = pd.DataFrame(rows); d = d[d.phase == 2]
            a.plot(d.leader_x, d.leader_y, color=COL[algo], lw=1.1,
                   label=f"{NAME[algo]}  {S['total_time']:.0f} s" + ("" if S["coll_mover"] == 0 else f", {S['coll_mover']} mover-contact steps"))
        a.set_title("motion limits " + ("ON (load-feasible)" if kin else "OFF (kinematic default)"))
        a.legend(fontsize=6.5, loc="lower left", framealpha=0.9, frameon=True, edgecolor="none")
    fig.suptitle("Validation warehouse, Python headless, seed 1: centroid paths of the eight laws", fontsize=10, y=0.98)
    save(fig, "warehouse_paths.png")


def fig_warehouse_matlab():
    world, movers = _wh()
    files = {a: os.path.join(ROOT, "matlab_ref/logs", f"wh_{a}_s0.csv") for a in ALGOS}
    if not any(os.path.exists(f) for f in files.values()):
        print("  (skip warehouse matlab: run the MATLAB warehouse batch)"); return
    fig = plt.figure(figsize=(10.5, 6.2))
    ax = fig.add_axes([0.04, 0.08, 0.42, 0.86])
    ax2 = fig.add_axes([0.56, 0.55, 0.42, 0.38]); ax3 = fig.add_axes([0.56, 0.08, 0.42, 0.36])
    _wh_axes(ax, world, movers)
    rows_tab = []
    for algo in ALGOS:
        if not os.path.exists(files[algo]):
            continue
        m = pd.read_csv(files[algo])
        rows, S = run_mission(os.path.join(WORLDS, "swarm_world_awswh.sdf"),
                              os.path.join(WORLDS, "dynamic_obstacles_awswh.json"),
                              algo=algo, seed=1, sigma0=0.0, verbose=False, gate="matlab")
        p = pd.DataFrame(rows)
        n = min(len(m), len(p))
        d = np.hypot(p.leader_x[:n] - m.leader_x[:n], p.leader_y[:n] - m.leader_y[:n])
        m2 = m[m.phase == 2]; p2 = p[p.phase == 2]
        ax.plot(m2.leader_x, m2.leader_y, color=COL[algo], lw=3.2, alpha=0.35)
        ax.plot(p2.leader_x, p2.leader_y, color=COL[algo], lw=0.9, label=NAME[algo])
        ax2.semilogy(m.t[:n], np.maximum(d, 1e-9), color=COL[algo], lw=1)
        rows_tab.append((algo, m.t.iloc[-1], p.t.iloc[-1], float(d.max())))
    ax.legend(fontsize=7, loc="lower left", frameon=True, edgecolor="none")
    ax.set_title("MATLAB (wide, pale) vs Python (thin), zero noise")
    ax2.axhline(5e-7, color=MUTED, lw=1, ls="--")
    ax2.set(xlabel="mission time (s)", ylabel="|MATLAB - Python| (m)", ylim=(1e-9, 1e-2),
            title="Centroid difference on the validation warehouse")
    x = np.arange(len(rows_tab))
    ax3.bar(x - 0.18, [r[1] for r in rows_tab], 0.32, color=C_M, label="MATLAB")
    ax3.bar(x + 0.18, [r[2] for r in rows_tab], 0.32, color=C_P, label="Python")
    ax3.set_xticks(x); ax3.set_xticklabels([NAME[r[0]] for r in rows_tab], rotation=35, ha="right")
    ax3.set(title="Mission time (s), zero noise"); ax3.legend(fontsize=7); ax3.grid(axis="x", visible=False)
    pd.DataFrame(rows_tab, columns=["algo", "t_matlab", "t_python", "max_diff_m"]).to_csv(
        os.path.join(RES, "warehouse_xcheck.csv"), index=False)
    save(fig, "warehouse_matlab_vs_python.png")


# ------------------------------------------------------------------ physics
def _mj(name):
    f = os.path.join(RES, "mujoco", name)
    return (pd.read_csv(f + ".csv"), json.load(open(f + ".json"))) if os.path.exists(f + ".json") else (None, None)


def fig_mujoco_tracking():
    world, movers = _wh()
    d, S = _mj("V1_mpc_square_on_top_s1")
    if d is None:
        print("  (skip mujoco tracking: run tools/run_mujoco_batch.py)"); return
    fig = plt.figure(figsize=(10.5, 6.4))
    ax = fig.add_axes([0.04, 0.08, 0.42, 0.86])
    _wh_axes(ax, world, movers)
    p = d[d.phase == 2]
    f = d[d.phase == 1]
    for i in range(4):
        ax.plot(f[f"r{i+1}_x"], f[f"r{i+1}_y"], color=MUTED, lw=0.6)
        ax.plot(p[f"r{i+1}_x"], p[f"r{i+1}_y"], color=C_X, lw=0.8, label="robots (physics)" if i == 0 else None)
    ax.plot(p.leader_x, p.leader_y, color=INK, lw=1.2, ls="--", label="planner frame (kinematic)")
    ax.plot(p.payload_x, p.payload_y, color=C_P, lw=1.0, label="payload body")
    ax.legend(fontsize=7, loc="lower left", frameon=True, edgecolor="none")
    ax.set_title("MPC, square, carried plate - MuJoCo")
    ax2 = fig.add_axes([0.56, 0.56, 0.42, 0.37]); ax3 = fig.add_axes([0.56, 0.08, 0.42, 0.37])
    for pl, c, lab in (("virtual", C_M, "no load"), ("on_top", C_P, "carried plate (pinned)"),
                       ("behind", C_X, "towed sled")):
        dd, _ = _mj(f"V1_mpc_square_{pl}_s1")
        if dd is None:
            continue
        q = dd[dd.phase == 2]
        ax2.plot(q.t - q.t.iloc[0], 100 * q.slot_err_max, color=c, lw=1.1, label=lab)
    ax2.set(xlabel="transport time (s)", ylabel="worst slot error (cm)", title="Formation tracking error under physics (MPC)")
    ax2.legend(fontsize=7)
    w = np.gradient(np.unwrap(p.leader_yaw.values), p.t.values)
    ax3.plot(p.t - p.t.iloc[0], np.abs(w), color=INK, lw=1.0)
    ax3.set(xlabel="transport time (s)", ylabel="|frame yaw rate| (rad/s)",
            title="Tracking error follows the frame's turns")
    save(fig, "mujoco_tracking.png")


def fig_mujoco_summary():
    rows = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(RES, "mujoco", "V*.json")))]
    if not rows:
        print("  (skip mujoco summary)"); return
    D = pd.DataFrame(rows)
    D["tag"] = D.name.str[:2]
    D.to_csv(os.path.join(RES, "mujoco_summary.csv"), index=False)
    v1 = D[D.tag == "V1"]
    laws = [a for a in ["mpc", "dwa", "pvfh_prob", "pvfh", "vfh"] if a in set(v1.algo)]
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    pls = [("virtual", C_M, "no load"), ("on_top", C_P, "carried plate"), ("behind", C_X, "towed sled")]
    for a, (col, lab, sc) in zip(ax, [("formation_rms", "formation RMS error (cm)", 100),
                                      ("formation_err_p95", "95th-percentile worst slot error (cm)", 100),
                                      ("contact", "steps with physical contact > 2 N", 1)]):
        for k, (pl, c, nm) in enumerate(pls):
            vals = []
            for al in laws:
                r = v1[(v1.algo == al) & (v1.payload == pl)]
                if len(r) == 0:
                    vals.append(0); continue
                r = r.iloc[0]
                vals.append((r.contact_steps_static + r.contact_steps_mover) if col == "contact" else r[col] * sc)
            a.bar(np.arange(len(laws)) + (k - 1) * 0.27, vals, 0.25, color=c, label=nm)
        a.set_xticks(range(len(laws))); a.set_xticklabels([NAME[x] for x in laws])
        a.set(title=lab); a.grid(axis="x", visible=False)
    ax[0].legend(fontsize=7)
    fig.suptitle("MuJoCo validation V1 - AWS warehouse, square formation, motion limits on, seed 1", fontsize=9, y=1.02)
    save(fig, "mujoco_summary.png")


def fig_mujoco_kin():
    names = [("V3_mpc_square_on_top_s1_nokin", "V1_mpc_square_on_top_s1", "MPC"),
             ("V3_pvfh_prob_square_on_top_s1_nokin", "V1_pvfh_prob_square_on_top_s1", "P-VFH")]
    if _mj(names[0][0])[0] is None:
        print("  (skip mujoco kin)"); return
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.2), sharey=True)
    for a, (off, on, lab) in zip(ax, names):
        for nm, c, l2 in ((off, "#d03b3b", "limits OFF"), (on, "#008300", "limits ON")):
            d, S = _mj(nm)
            if d is None:
                continue
            q = d[d.phase == 2]
            a.plot(q.t - q.t.iloc[0], 100 * q.slot_err_max, color=c, lw=1.0,
                   label=f"{l2}: rms {100*S['formation_rms']:.0f} cm, needs {S['max_robot_speed']:.1f} m/s")
        a.set(xlabel="transport time (s)", title=f"{lab}, carried plate")
        a.legend(fontsize=7)
    ax[0].set_ylabel("worst slot error (cm)")
    save(fig, "mujoco_motion_limits.png")


def fig_mujoco_frames():
    vids = sorted(glob.glob(os.path.join(RES, "mujoco", "V1_mpc_square_on_top_s1.mp4")))
    if not vids:
        print("  (skip frames: run tools/run_mujoco_batch.py --videos)"); return
    import subprocess
    tmp = os.path.join(FIG, "_f%d.png")
    ts = [3, 12, 30, 52]
    for k, t in enumerate(ts):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", str(t / 2.0), "-i", vids[0],
                        "-frames:v", "1", tmp % k], check=False)
    fig, ax = plt.subplots(1, 4, figsize=(12, 2.1))
    for k, a in enumerate(ax):
        if os.path.exists(tmp % k):
            a.imshow(plt.imread(tmp % k)); os.remove(tmp % k)
        a.set_axis_off()
        a.set_title(["forming under the load", "dock -> aisle", "turning into the cross-aisle", "west aisle, near B"][k], fontsize=8)
    fig.subplots_adjust(wspace=0.02)
    save(fig, "mujoco_frames.png")


ALL = [fig_concept_kernel, fig_concept_histogram, fig_concept_formations, fig_concept_gate,
       fig_concept_filter, fig_xcheck, fig_cross_scatter, fig_e1, fig_e2, fig_gate_effect,
       fig_warehouse_paths, fig_warehouse_matlab, fig_mujoco_tracking, fig_mujoco_summary,
       fig_mujoco_kin, fig_mujoco_frames]

if __name__ == "__main__":
    pat = sys.argv[1] if len(sys.argv) > 1 else ""
    for f in ALL:
        if pat in f.__name__:
            print(f.__name__)
            f()
