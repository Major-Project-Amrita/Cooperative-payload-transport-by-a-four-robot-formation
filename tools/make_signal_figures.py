#!/usr/bin/env python3
"""Signal-level figures: what happens inside a mission, step by step.

    python3 tools/make_signal_figures.py            # record (cached) + plot
    python3 tools/make_signal_figures.py --rerecord

Writes results/figures_signals/*.png and results/signals/*.npz. Each figure
backs one claim in the documentation:

  S1 physics_margin_clearance  PA-VFH grazes static racks in physics without a
                               margin; with the 0.10 m gate margin it does not
                               (MuJoCo, same seed and traffic phase), MPC as
                               reference.
  S2 physics_signals           one physics mission, signal by signal: speed,
                               formation error, payload tilt, pin gap, gate
                               rejections, contacts.
  S3 pavfh_danger_maps         PA-VFH's per-sector danger over a mission: the
                               neural network, the analytic kernel on the same
                               scan, and the oracle (true future) it was trained
                               to predict; plus the per-step error of each.
  S4 monitor_signals           the deployable sensor monitor's clearance
                               estimate vs the ground-truth clearance at every
                               gate check (false accepts would be in the
                               shaded quadrant).
  S5 ros2_lockstep_equality    centroid trajectory of the same mission run as
                               two ROS 2 processes over DDS and in one process,
                               and their difference over time.
  S6 ros2_deployment_signals   real-sensor deployment over DDS: cmd_vel
                               cadence, commanded speed, clearance, and the
                               watchdog stopping every robot when a lidar fails.
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
W = os.path.join(PKG, "worlds")
FIG = os.path.join(ROOT, "results", "figures_signals")
SIG = os.path.join(ROOT, "results", "signals")
sys.path.insert(0, PKG)
R_ROBOT = 0.18

# reference categorical palette (fixed order), text inks, surfaces
C = {"a": "#2a78d6", "b": "#eb6834", "c": "#1baf7a", "d": "#eda100", "e": "#6250d6",
     "f": "#e34948"}
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
CRIT = "#c6362f"


def plt_setup():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
        "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "text.color": INK, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
        "legend.frameon": False, "lines.linewidth": 1.6})
    return plt


def save(fig, name):
    os.makedirs(FIG, exist_ok=True)
    p = os.path.join(FIG, name + ".png")
    fig.savefig(p, dpi=150, bbox_inches="tight")
    print("  wrote", os.path.relpath(p, ROOT))


# ------------------------------------------------------------------ recording
def record(force=False):
    os.makedirs(SIG, exist_ok=True)
    from ros2_swarm_transport.headless_runner import run_mission
    from ros2_swarm_transport import safety_monitor as sm
    p3 = os.path.join(SIG, "pavfh_danger.npz")
    if force or not os.path.exists(p3):
        rec = []
        rows, S = run_mission(os.path.join(W, "swarm_world_dynamic.sdf"),
                              os.path.join(W, "dynamic_obstacles.json"), algo="pavfh",
                              seed=3, sigma0=0.10, phase=True, verbose=False,
                              timesteps=2000, vfh_over={"nn": {"signal_record": rec}})
        np.savez_compressed(p3, t=np.array([r["t"] for r in rec]),
                            u_nn=np.array([r["u_used"] for r in rec]),
                            u_an=np.array([r["u_analytic"] for r in rec]),
                            u_or=np.array([r["u_oracle"] for r in rec]),
                            observed=np.array([r["observed"] for r in rec]),
                            thr=rec[0]["threshold"] if rec else 0.3,
                            reached=S["reached_goal"], total_time=S["total_time"])
        print("  recorded PA-VFH danger signals:", len(rec), "steps")
    p4 = os.path.join(SIG, "monitor.npz")
    if force or not os.path.exists(p4):
        last = {}
        orig = sm.SensorSafetyMonitor.clearance

        def clr(self, p, h):
            v = orig(self, p, h)
            last["v"] = v
            return v
        sm.SensorSafetyMonitor.clearance = clr
        mrec, mon = [], []
        orig_score = sm.SensorSafetyMonitor.score

        def score(self, admitted, truth):
            mon.append((last.get("v", np.nan), bool(admitted), bool(truth)))
            return orig_score(self, admitted, truth)
        sm.SensorSafetyMonitor.score = score
        _, S = run_mission(os.path.join(W, "swarm_world_dynamic.sdf"),
                           os.path.join(W, "dynamic_obstacles.json"), algo="pavfh",
                           seed=2, sigma0=0.02, phase=True, verbose=False, timesteps=2000,
                           gate_source="sensor", mon_record=mrec)
        sm.SensorSafetyMonitor.clearance, sm.SensorSafetyMonitor.score = orig, orig_score
        n = min(len(mrec), len(mon))
        np.savez_compressed(p4, mon_clear=np.array([m[0] for m in mon[:n]]),
                            admitted=np.array([m[1] for m in mon[:n]]),
                            truth_safe=np.array([m[2] for m in mon[:n]]),
                            true_clear=np.array([m[1] for m in mrec[:n]]),
                            stats=json.dumps(S["gate_stats"]))
        print("  recorded monitor signals:", n, "gate checks")


# ------------------------------------------------------------------ figures
def s1(plt):
    files = {"PA-VFH, no margin (P1)": "P1_pavfh_square_on_top_s1.csv",
             "PA-VFH, 0.10 m margin (P3)": "P3_pavfh_square_on_top_s1.csv",
             "MPC, no margin (P1)": "P1_mpc_square_on_top_s1.csv"}
    D = {k: os.path.join(ROOT, "results", "mujoco_phase", f) for k, f in files.items()}
    if not all(os.path.exists(p) for p in D.values()):
        return
    fig, ax = plt.subplots(2, 1, figsize=(8, 5.2), sharex=True,
                           gridspec_kw={"height_ratios": [3, 1.2]})
    col = [C["f"], C["a"], C["d"]]
    for (k, p), c in zip(D.items(), col):
        d = pd.read_csv(p)
        d = d[d.phase == 2]
        ax[0].plot(d.t, d.clearance_static - R_ROBOT, color=c, label=k)
        cs = d[d.contact_static > 0]
        ax[1].scatter(cs.t, [k] * len(cs), s=10, color=c)
    ax[0].axhline(0, color=CRIT, lw=1, ls="--")
    ax[0].text(ax[0].get_xlim()[0], 0.01, "  robot touches a rack below this line",
               color=CRIT, fontsize=8, va="bottom")
    ax[0].set_ylabel("planner clearance to statics (m)")
    ax[0].set_title("S1. Physics: the 0.10 m gate margin removes PA-VFH's rack contacts")
    ax[0].legend(loc="upper right")
    ax[1].set_ylabel("physics\ncontact")
    ax[1].set_xlabel("mission time (s), transport phase, MuJoCo, seed 1, random traffic phase")
    ax[1].tick_params(axis="y", labelsize=7)
    save(fig, "S1_physics_margin_clearance")


def s2(plt):
    p = os.path.join(ROOT, "results", "mujoco_phase", "P3_pavfh_square_on_top_s1.csv")
    if not os.path.exists(p):
        return
    d = pd.read_csv(p)
    sp = np.r_[0, np.hypot(np.diff(d.payload_x), np.diff(d.payload_y)) / np.diff(d.t).clip(1e-6)]
    panels = [("payload speed (m/s)", sp, C["a"]),
              ("formation error, max over robots (m)", d.slot_err_max, C["b"]),
              ("payload tilt (deg)", d.tilt_deg, C["c"]),
              ("pin gap (mm)", 1000 * d.pin_gap, C["d"]),
              ("clearance to anything (m)", d.clearance - R_ROBOT, C["e"])]
    fig, ax = plt.subplots(len(panels) + 1, 1, figsize=(8, 9), sharex=True)
    for a, (lab, y, c) in zip(ax, panels):
        a.plot(d.t, y, color=c)
        a.set_ylabel(lab, fontsize=8)
    ax[4].axhline(0, color=CRIT, lw=1, ls="--")
    ev = [("gate reject", d.reject > 0, C["e"]), ("static contact", d.contact_static > 0, CRIT),
          ("mover contact", d.contact_mover > 0, C["f"]), ("transport phase", d.phase == 2, C["c"])]
    for i, (lab, m, c) in enumerate(ev):
        ax[-1].scatter(d.t[m], np.full(int(m.sum()), i), s=4, color=c, marker="|")
    ax[-1].set_yticks(range(len(ev)), [e[0] for e in ev], fontsize=7)
    ax[-1].set_xlabel("mission time (s)")
    ax[0].set_title("S2. One physics mission, signal by signal (PA-VFH, 0.10 m margin, "
                    "load on top, seed 1)")
    save(fig, "S2_physics_signals")


def s3(plt):
    p = os.path.join(SIG, "pavfh_danger.npz")
    if not os.path.exists(p):
        return
    z = np.load(p)
    t, nn, an, orc = z["t"], z["u_nn"], z["u_an"], z["u_or"]
    fig, ax = plt.subplots(4, 1, figsize=(8, 9.5), sharex=True,
                           gridspec_kw={"height_ratios": [1, 1, 1, 1.1]})
    ext = [t[0], t[-1], -180, 180]
    for a, (u, lab) in zip(ax[:3], ((orc, "oracle: true danger over the next 0.5 s"),
                                    (nn, "PA-VFH network (from 3 noisy scans)"),
                                    (an, "analytic kernel (current scan only)"))):
        im = a.imshow(u.T, aspect="auto", origin="lower", extent=ext, cmap="Blues",
                      vmin=0, vmax=1, interpolation="nearest")
        a.set_ylabel("bearing (deg)")
        a.set_title(lab, fontsize=9, loc="left")
        a.grid(False)
    cb = fig.colorbar(im, ax=ax[:3], fraction=0.02, pad=0.01)
    cb.set_label("danger u (0 free, 1 blocked)")
    obs = z["observed"] if "observed" in z else np.ones_like(orc, bool)
    thr = float(z["thr"]) if "thr" in z else 0.3
    e_nn = np.array([np.abs(nn[i] - orc[i])[obs[i]].mean() for i in range(len(t))])
    e_an = np.array([np.abs(an[i] - orc[i])[obs[i]].mean() for i in range(len(t))])
    blk = orc > thr
    miss_nn = (blk & (nn <= thr)).sum() / max(blk.sum(), 1)
    miss_an = (blk & (an <= thr)).sum() / max(blk.sum(), 1)
    fa_nn = (~blk & (nn > thr)).sum() / max((~blk).sum(), 1)
    fa_an = (~blk & (an > thr)).sum() / max((~blk).sum(), 1)
    ag_nn = ((nn > thr) == blk).mean()
    ag_an = ((an > thr) == blk).mean()
    ax[3].plot(t, e_an, color=C["b"], label=f"analytic kernel, mean {e_an.mean():.3f}")
    ax[3].plot(t, e_nn, color=C["a"], label=f"network, mean {e_nn.mean():.3f}")
    ax[3].set_ylabel("|u - oracle|, mean over\nobserved sectors")
    ax[3].set_title(f"blocked/free agreement with the oracle: network {ag_nn:.3f}, analytic "
                    f"{ag_an:.3f}; truly blocked sectors called free: network {miss_nn:.3f}, "
                    f"analytic {miss_an:.3f}", fontsize=8, loc="left")
    ax[3].set_xlabel("mission time (s); study warehouse, sigma0 = 0.10 m, seed 3, random phase")
    ax[3].legend(loc="upper right")
    fig.suptitle("S3. What the neural network inside PA-VFH predicts", fontweight="bold",
                 fontsize=10, x=0.02, ha="left")
    save(fig, "S3_pavfh_danger_maps")
    return dict(mae_network=float(e_nn.mean()), mae_analytic=float(e_an.mean()),
                agree_network=float(ag_nn), agree_analytic=float(ag_an),
                missed_block_network=float(miss_nn), missed_block_analytic=float(miss_an),
                false_block_network=float(fa_nn), false_block_analytic=float(fa_an),
                steps=int(len(t)))


def s4(plt):
    p = os.path.join(SIG, "monitor.npz")
    if not os.path.exists(p):
        return
    z = np.load(p)
    mc, tc, adm = z["mon_clear"], z["true_clear"] - R_ROBOT, z["admitted"]
    ok = np.isfinite(mc)
    fig, ax = plt.subplots(1, 2, figsize=(9.5, 4))
    a = ax[0]
    a.axvspan(min(tc.min(), -0.1), 0, color=CRIT, alpha=0.06)
    a.scatter(tc[ok & adm], mc[ok & adm], s=5, color=C["a"], label="admitted")
    a.scatter(tc[ok & ~adm], mc[ok & ~adm], s=5, color=C["b"], label="rejected")
    a.axhline(0, color=INK2, lw=0.8)
    a.axvline(0, color=CRIT, lw=0.8, ls="--")
    fa = int((adm & (tc < 0)).sum())
    a.set_xlabel("ground-truth clearance minus robot radius (m)")
    a.set_ylabel("monitor's clearance minus required gap (m)")
    a.set_title(f"every gate check (n={len(tc)}); false accepts: {fa}", fontsize=9, loc="left")
    a.legend(loc="lower right")
    b = ax[1]
    k = np.arange(len(tc))
    b.plot(k, tc, color=INK2, lw=1, label="ground truth")
    b.plot(k, np.where(ok, mc, np.nan), color=C["a"], lw=1, label="monitor (needs no map)")
    b.axhline(0, color=CRIT, lw=0.8, ls="--")
    b.set_xlabel("gate check index")
    b.set_ylabel("clearance signal (m)")
    b.legend(loc="upper right")
    b.set_title("the monitor is conservative: it stays below the truth", fontsize=9, loc="left")
    fig.suptitle("S4. Deployable sensor safety monitor vs ground truth (PA-VFH, sigma0 = 0.02 m)",
                 fontweight="bold", fontsize=10, x=0.02, ha="left")
    save(fig, "S4_monitor_signals")


def s5(plt):
    a_p = os.path.join(ROOT, "results", "mujoco_ros", "ros_pavfh_square_on_top_s1.csv")
    b_p = os.path.join(ROOT, "results", "mujoco_ros", "direct", "pavfh_square_on_top_s1.csv")
    if not (os.path.exists(a_p) and os.path.exists(b_p)):
        return
    a, b = pd.read_csv(a_p), pd.read_csv(b_p)
    n = min(len(a), len(b))
    a, b = a.iloc[:n], b.iloc[:n]
    fig, ax = plt.subplots(1, 2, figsize=(9.5, 3.8))
    ax[0].plot(b.leader_x, b.leader_y, color=C["d"], lw=4, alpha=0.5, label="in one process")
    ax[0].plot(a.leader_x, a.leader_y, color=C["a"], lw=1.2, label="two ROS 2 processes (DDS)")
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x (m)")
    ax[0].set_ylabel("y (m)")
    ax[0].legend(loc="best")
    ax[0].set_title("formation centroid, PA-VFH, MuJoCo", fontsize=9, loc="left")
    dd = np.hypot(a.leader_x.values - b.leader_x.values, a.leader_y.values - b.leader_y.values)
    ax[1].plot(a.t, dd, color=C["a"])
    ax[1].set_xlabel("mission time (s)")
    ax[1].set_ylabel("centroid difference (m)")
    ax[1].set_title(f"max difference {dd.max():.1e} m over {n} steps", fontsize=9, loc="left")
    fig.suptitle("S5. ROS 2 Humble over DDS reproduces the in-process mission exactly",
                 fontweight="bold", fontsize=10, x=0.02, ha="left")
    save(fig, "S5_ros2_lockstep_equality")
    return dict(max_diff=float(dd.max()), steps=int(n))


def s6(plt):
    base = os.path.join(ROOT, "results", "ros_humble")
    runs = sorted(glob.glob(os.path.join(base, "real_sensor_s*_signals.json")))
    wd = os.path.join(base, "watchdog_signals.json")
    if not runs:
        return
    sg = json.load(open(runs[0]))
    cmd = np.array(sg["cmd"])
    sig = np.array(sg["sig"])
    fig, ax = plt.subplots(4, 1, figsize=(8, 8.5))
    dtw = np.diff(cmd[:, 0]) * 1000
    ax[0].hist(dtw, bins=60, color=C["a"])
    ax[0].set_xlabel("time between cmd_vel messages, robot1 (ms)")
    ax[0].set_ylabel("count")
    ax[0].set_title(f"cadence over DDS: median {np.median(dtw):.0f} ms, "
                    f"95th percentile {np.percentile(dtw, 95):.0f} ms (target 100 ms)",
                    fontsize=9, loc="left")
    ax[1].plot(cmd[:, 1], cmd[:, 2], color=C["a"])
    ax[1].set_ylabel("commanded v, robot1 (m/s)")
    ax[1].set_xlabel("mission time (s)")
    ax[2].plot(sig[:, 0], sig[:, 1], color=C["e"])
    ax[2].axhline(0, color=CRIT, lw=0.8, ls="--")
    ax[2].set_ylabel("true gap to nearest\nobstacle (m)")
    ax[2].set_xlabel("mission time (s); controller sees only /scan")
    if os.path.exists(wd):
        w = json.load(open(wd))
        wc, ws = np.array(w["cmd"]), np.array(w["sig"])
        info = json.load(open(os.path.join(base, "watchdog.json"))).get("watchdog", {})
        ax[3].plot(wc[:, 1], np.abs(wc[:, 2]), color=C["a"], label="|commanded v|, robot1")
        ax[3].step(ws[:, 0], ws[:, 2], color=CRIT, where="post", label="/swarm/safety (1 = stop)")
        if info.get("scan_dropped_at") is not None:
            ax[3].axvline(info["scan_dropped_at"], color=INK2, ls=":", lw=1)
            ax[3].text(info["scan_dropped_at"], 0.9, " robot1 lidar fails", color=INK2, fontsize=8)
        ax[3].set_xlabel("mission time (s)")
        ax[3].set_ylabel("speed / flag")
        ax[3].legend(loc="upper left")
    fig.suptitle("S6. Real-sensor deployment over ROS 2 DDS (robots.launch.py)",
                 fontweight="bold", fontsize=10, x=0.02, ha="left")
    fig.tight_layout()
    save(fig, "S6_ros2_deployment_signals")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rerecord", action="store_true")
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    if not a.no_record:
        record(a.rerecord)
    plt = plt_setup()
    out = {}
    for f in (s1, s2, s3, s4, s5, s6):
        try:
            r = f(plt)
            if r:
                out[f.__name__] = r
        except Exception as e:                   # noqa: BLE001
            print(f"  {f.__name__} skipped: {type(e).__name__}: {e}")
        plt.close("all")
    json.dump(out, open(os.path.join(FIG, "signal_numbers.json"), "w"), indent=1)
    print(json.dumps(out))


if __name__ == "__main__":
    main()
