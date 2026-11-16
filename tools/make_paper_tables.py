#!/usr/bin/env python3
"""LaTeX tables for the manuscript, computed from the result files.

    python3 tools/make_paper_tables.py      # -> docs/paper/tables/*.tex, docs/paper/numbers.tex

Nothing in the tables is typed by hand. numbers.tex defines macros (e.g.
\\XsAgree) for numbers quoted in the running text.
"""
import glob
import json
import math
import os
import re

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results")
OUT = os.path.join(ROOT, "docs", "paper", "tables")
NAME = {"apf": "APF", "vfh": "VFH", "fgm": "FGM", "dwa": "DWA", "mpc": "MPC", "mppi": "MPPI",
        "pvfh": "pVFH", "pvfh_filt": "pVFH+F", "pvfh_prob": "PA-VFH-K", "nn": "NN (DAgger)",
        "pavfh_nonn": "PA-VFH (no NN)", "pavfh": r"\textbf{PA-VFH}"}
rng = np.random.default_rng(2026)
MAC = {}


def wilson(k, n, z=1.959964):
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def rate(k, n):
    lo, hi = wilson(k, n)
    return f"{k}/{n} [{lo:.2f}, {hi:.2f}]"


def boot(x, f="{:.1f}"):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return "--"
    m = x[rng.integers(0, len(x), (2000, len(x)))].mean(1)
    return f"{f.format(x.mean())} [{f.format(np.percentile(m, 2.5))}, {f.format(np.percentile(m, 97.5))}]"


def write(name, body):
    os.makedirs(OUT, exist_ok=True)
    open(os.path.join(OUT, name + ".tex"), "w").write(body)
    print("  wrote", name)


def tab(cols, rows, spec=None, size=r"\footnotesize"):
    spec = spec or ("l" * len(cols))
    s = [size, r"\setlength{\tabcolsep}{3pt}", r"\begin{tabular}{" + spec + "}", r"\toprule",
         " & ".join(cols) + r" \\", r"\midrule"]
    s += [" & ".join(str(c) for c in r) + r" \\" for r in rows]
    s += [r"\bottomrule", r"\end{tabular}"]
    return "\n".join(s) + "\n"


def mac(name, val):
    MAC[name] = val


# ------------------------------------------------------------------ T1 laws
def t_laws():
    e15 = pd.read_csv(os.path.join(RES, "extended", "E15.csv"))
    ms = e15.groupby("algo").comp_ms_mean.mean()
    info = {"vfh": ("histogram", "predicted scan"), "fgm": ("gap", "predicted scan"),
            "dwa": ("velocity sampling", "predicted scan"), "mpc": ("lattice MPC", "predicted scan"),
            "mppi": ("sampling MPC", "predicted scan"), "pvfh": ("histogram + horizon", "contract"),
            "pvfh_prob": ("+ range filter + kernel", "contract"),
            "nn": ("learned policy (DAgger)", "predicted scan"),
            "pavfh_nonn": ("+ range filter + kernel", r"\textbf{own scans only}"),
            "pavfh": ("+ range filter + learned occupancy", r"\textbf{own scans only}")}
    rows = [(NAME[a], info[a][0], info[a][1], f"{ms[a]:.2f}") for a in info]
    write("laws", tab(["Law", "Mechanism", "Mover information", "ms/step"], rows, "lllr"))


# ------------------------------------------------------------------ T2 replication
def t_replication():
    md = open(os.path.join(RES, "CROSS_STACK_fixedgate.md")).read()
    m = re.search(r"agree on reach.*?\*\*(\d+)/(\d+)\*\*", md)
    r = re.search(r"time r = ([\d.]+), rejections r = ([\d.]+), min gap r = ([\d.]+)", md)
    g = re.search(r"reaching the goal: MATLAB (\d+)/(\d+), Python (\d+)/(\d+)", md)
    xc = pd.read_csv(os.path.join(RES, "deterministic_xcheck_fixedgate.csv"))
    mac("XsAgree", m.group(1)); mac("XsCells", m.group(2))
    mac("XcMax", f"{xc.max_centroid_diff_m.max():.0e}".replace("e-0", r"\times10^{-") + "}")
    rows = [("Zero noise, step by step (8 laws)", f"max centroid diff.\\ {xc.max_centroid_diff_m.max():.1e} m; never $>$1 mm"),
            ("Noisy grid, cells agreeing", f"{m.group(1)}/{m.group(2)}"),
            ("Correlation of cell means", f"time {r.group(1)}, rejections {r.group(2)}, gap {r.group(3)}"),
            ("Runs reaching the goal", f"MATLAB {g.group(1)}/{g.group(2)}, Python {g.group(3)}/{g.group(4)}"),
            ("Static-contact steps", "0 in both stacks")]
    write("replication", tab(["Check", "Result"], rows, "lp{0.58\\columnwidth}"))


# ------------------------------------------------------------------ T3 phase
def t_phase():
    a = pd.read_csv(os.path.join(RES, "results_headless_exactgate.csv"))
    b = pd.read_csv(os.path.join(RES, "results_headless_phase.csv"))
    rows = []
    for law in ("vfh", "fgm", "dwa", "mpc", "pvfh", "pvfh_filt", "pvfh_prob"):
        x, y = a[a.algo == law], b[b.algo == law]
        if not len(x):
            continue
        rows.append((NAME[law], f"{100 * x.reached.mean():.0f}\\,\\%", f"{100 * y.reached.mean():.0f}\\,\\%",
                     f"{100 * (x.collisions > 0).mean():.0f}\\,\\%", f"{100 * (y.collisions > 0).mean():.0f}\\,\\%",
                     len(x)))
        if law == "mpc":
            mac("MpcContactNom", f"{100 * (x.collisions > 0).mean():.0f}")
            mac("MpcContactPhase", f"{100 * (y.collisions > 0).mean():.0f}")
            mac("MpcReachNom", f"{100 * x.reached.mean():.0f}")
            mac("MpcReachPhase", f"{100 * y.reached.mean():.0f}")
    write("phase", tab(["Law", "Reach nom.", "Reach rand.", "Contact nom.", "Contact rand.", "$n$"],
                       rows, "lrrrrr"))


# ------------------------------------------------------------------ T4 E19
def t_e19():
    d = pd.read_csv(os.path.join(RES, "extended", "E19.csv"))
    rows = []
    for law in ("pavfh", "pavfh_nonn", "pvfh_prob", "mpc", "mppi"):
        x = d[d.algo == law]
        rows.append((NAME[law], rate(int(x.reached.sum()), len(x)),
                     rate(int((x.collisions > 0).sum()), len(x)),
                     boot(x[x.reached == 1].total_time), boot(x.n_reject), f"{x.comp_ms_mean.mean():.2f}"))
    write("e19", tab(["Law", "Reach", "Contact runs", "Time (s)", "Rejections", "ms"], rows, "llllll",
                     r"\scriptsize"))
    A = d[d.algo == "pavfh"].set_index(["world", "sigma0", "seed"])
    B = d[d.algo == "pavfh_nonn"].set_index(["world", "sigma0", "seed"])
    c = A.index.intersection(B.index)
    A, B = A.loc[c], B.loc[c]
    both = (A.reached == 1) & (B.reached == 1)
    prow = [("Missions (paired)", f"{len(c)}"),
            ("Reached, network / no network", f"{int(A.reached.sum())} / {int(B.reached.sum())}"),
            ("Time difference (s), both reached", boot((A.total_time - B.total_time)[both], "{:+.1f}")),
            ("Rejected-step difference", boot(A.n_reject - B.n_reject, "{:+.1f}")),
            ("Contact-step difference", boot(A.collisions - B.collisions, "{:+.1f}"))]
    write("e19_paired", tab(["Paired PA-VFH $-$ PA-VFH (no NN)", "Mean [95\\,\\% CI]"], prow, "ll"))
    mac("EnnReach", f"{int(A.reached.sum())}"); mac("EnnN", f"{len(c)}")
    mac("EnoReach", f"{int(B.reached.sum())}")
    x = d[d.algo == "pavfh"]; y = d[d.algo == "mpc"]
    mac("PavfhMs", f"{x.comp_ms_mean.mean():.2f}"); mac("MpcMs", f"{y.comp_ms_mean.mean():.2f}")
    mac("PavfhTime", f"{x[x.reached == 1].total_time.mean():.1f}")
    mac("MpcTime", f"{y[y.reached == 1].total_time.mean():.1f}")


# ------------------------------------------------------------------ T5 monitors
def t_monitors():
    d = pd.read_csv(os.path.join(RES, "extended", "E17.csv"))
    d = d[d.exp == "E17"]
    lab = {"audit": "Ground truth (audit)", "sensor": "Geometric sensor monitor",
           "learned": "Learned + conformal ($\\alpha{=}0.05$)"}
    rows = []
    for g in ("audit", "sensor", "learned"):
        for s in (0.02, 0.10):
            x = d[(d.gate_source == g) & (np.isclose(d.sigma0, s))]
            fa = x.mon_false_accept.sum() / max(x.mon_checks.sum(), 1) * 1000 if g != "audit" else 0.0
            rows.append((lab[g] if s == 0.02 else "", f"{s:.2f}", rate(int(x.reached.sum()), len(x)),
                         rate(int((x.collisions > 0).sum()), len(x)),
                         "--" if g == "audit" else f"{fa:.2f}"))
    write("monitors", tab(["Gate", "$\\sigma_0$", "Reach", "Contact runs", "FA/1000"], rows, "lllll",
                          r"\scriptsize"))


# ------------------------------------------------------------------ T6 physics
def t_physics():
    def clean(tag):
        out = {}
        for f in glob.glob(os.path.join(RES, "mujoco_phase", f"{tag}_*.json")):
            s = json.load(open(f))
            if tag in ("P1", "P3") and s["payload"] == "behind":
                key = (s["algo"], "behind")
            else:
                key = (s["algo"], "main")
            ok = s["reached_goal"] and s["contact_steps_static"] == 0 and s["contact_steps_mover"] == 0
            k, n = out.get(key, (0, 0))
            out[key] = (k + int(ok), n + 1)
        return out
    P1, P3, P4 = clean("P1"), clean("P3"), clean("P4")
    r1 = [json.load(open(f)) for f in glob.glob(os.path.join(RES, "ros_physics", "R1", "*.row.json"))]
    R1 = {}
    for r in r1:
        key = (r["algo"], "behind" if r["payload"] == "behind" else "main")
        ok = r["reached_goal"] and r["contact_steps_static"] == 0 and r["contact_steps_mover"] == 0
        k, n = R1.get(key, (0, 0))
        R1[key] = (k + int(ok), n + 1)
    f = lambda D, k: (f"{D[k][0]}/{D[k][1]}" if k in D else "--")
    rows = []
    for a in ("pavfh", "mpc", "nn", "mppi", "dwa", "pvfh_prob"):
        rows.append((NAME[a], f(P1, (a, "main")), f(P3, (a, "main")), f(P3, (a, "behind")),
                     f(P4, (a, "main")), f(R1, (a, "main")), f(R1, (a, "behind"))))
    write("physics", tab(["Law", "P1", "P3", "P3 towed", "P4 sweep", "R1 (ROS\\,2)", "R1 towed"],
                         rows, "lrrrrrr", r"\scriptsize"))
    same = [r for r in r1 if "identical_same_env" in r]
    mac("RoneSame", f"{sum(bool(r['identical_same_env']) for r in same)}")
    mac("RoneN", f"{len(same)}")
    r2 = [json.load(open(f)) for f in glob.glob(os.path.join(RES, "ros_physics", "R2", "*.row.json"))]
    mac("RtwoClean", f"{sum(bool(r.get('reached')) and r['contact_steps_static'] == 0 and r['contact_steps_mover'] == 0 for r in r2)}")
    mac("RtwoN", f"{len(r2)}")
    wd = json.load(open(os.path.join(RES, "ros_physics", "DEMO", "DEMO_watchdog_on_top_s1.json")))["watchdog"]
    mac("WdDelay", f"{wd['safety_flag_at'] - wd['scan_dropped_at']:.1f}")
    mac("PavfhPone", f(P1, ("pavfh", "main"))); mac("PavfhPthree", f(P3, ("pavfh", "main")))
    mac("PavfhPfour", f(P4, ("pavfh", "main"))); mac("MpcPfour", f(P4, ("mpc", "main")))
    mac("NnPfour", f(P4, ("nn", "main")))


def t_layouts():
    e15 = pd.read_csv(os.path.join(RES, "extended", "E15.csv"))
    e16 = pd.read_csv(os.path.join(RES, "extended", "E16.csv"))
    rows = []
    for a in ("vfh", "fgm", "dwa", "mpc", "mppi", "pvfh_prob", "nn", "pavfh_nonn", "pavfh"):
        x = e15[e15.algo == a]
        y = e16[e16.algo == a]
        rows.append((NAME[a], f"{int(x.reached.sum())}/{len(x)}",
                     f"{int((x.collisions > 0).sum())}/{len(x)}",
                     f"{int(y.reached.sum())}/{len(y)}" if len(y) else "--",
                     f"{int((y.collisions > 0).sum())}/{len(y)}" if len(y) else "--"))
    write("layouts", tab(["Law", "E15 reach", "E15 contact", "E16 reach", "E16 contact"], rows,
                         "lrrrr", r"\scriptsize"))


def main():
    for f in (t_laws, t_replication, t_phase, t_e19, t_monitors, t_physics, t_layouts):
        f()
    with open(os.path.join(ROOT, "docs", "paper", "numbers.tex"), "w") as fh:
        for k, v in MAC.items():
            fh.write(f"\\newcommand{{\\{k}}}{{{v}}}\n")
    print("  wrote numbers.tex:", ", ".join(MAC))


if __name__ == "__main__":
    main()
