"""Cell-by-cell comparison of the MATLAB grid and the Python headless grid.

For every experiment cell and both stacks:
  reach            goal-reach count with the Wilson 95 % interval
  n_reject         mean gate rejections per run, bootstrap 95 % interval
  min gap          mean of (min_clear - 0.18 m), bootstrap 95 % interval
  time             mean mission time, bootstrap 95 % interval
`agree` = the reach, rejection and gap intervals of the two stacks overlap
(the acceptance rule of ROS2_REPLICATION.md section 9.6). The two stacks use
different random generators, so runs are not paired.

    python3 tools/cross_stack.py                       # matlab-gate replication
    python3 tools/cross_stack.py --py results/results_headless_exactgate.csv --tag exactgate

Writes results/cross_stack[_tag].csv, results/generated/tab_cross[_tag].tex
and results/CROSS_STACK[_tag].md.
"""
import argparse
import os

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = 0.18
CELL = ["exp", "algo", "sense_kind", "sigma0", "beams", "world", "formation",
        "mover_scale", "adl", "contract", "loc_sigma", "comm_drop", "n_movers",
        "kin"]
NAME = {"apf": "APF", "vfh": "VFH", "fgm": "FGM", "dwa": "DWA", "mpc": "MPC",
        "pvfh": "pVFH", "pvfh_filt": "pVFH+F", "pvfh_prob": "P-VFH"}


def wilson(k, n, z=1.959964):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def boot(x, rng, B=2000):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return (np.nan, np.nan, np.nan)
    m = x[rng.integers(0, len(x), (B, len(x)))].mean(axis=1)
    return (float(x.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5)))


def overlap(a, b, tol=0.0):
    return not (a[1] < b[0] - tol or b[1] < a[0] - tol)


def cell_label(k):
    d = dict(zip(CELL, k))
    w = str(d["world"]).replace("swarm_world_", "").replace(".sdf", "")
    bits = [NAME.get(d["algo"], d["algo"])]
    e = d["exp"]
    if e in ("E2", "E6", "E7", "E13", "E3"):
        bits.append(f"s0={d['sigma0']:g}")
    if e in ("E3", "E6"):
        bits.append(f"{d['sense_kind']}-{int(d['beams'])}")
    if e in ("E5", "E9"):
        bits.append(d["formation"])
    if e in ("E7", "E13"):
        bits.append(w)
    if e == "E8":
        bits.append(f"movers x{d['mover_scale']:g}")
    if e == "E9":
        bits.append("ADL off")
    if e == "E10":
        bits.append(f"loc {d['loc_sigma']:g} m")
    if e == "E11":
        bits.append(f"{int(d['n_movers'])} movers")
    if e == "E12":
        bits.append(f"drop {d['comm_drop']:g}")
    if e == "E14":
        bits.append("limits on")
    return ", ".join(bits)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--matlab", default=os.path.join(ROOT, "bundle/reference/results_all.csv"))
    ap.add_argument("--py", default=os.path.join(ROOT, "results/results_headless.csv"))
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    sfx = ("_" + a.tag) if a.tag else ""
    M, P = pd.read_csv(a.matlab), pd.read_csv(a.py)
    for D in (M, P):
        D.loc[D.exp == "E13", "world"] = "rand(pooled)"
        D.loc[D.exp == "E13", "contract"] = "rand(pooled)"
        D["gap"] = D.min_clear - R
    rng = np.random.default_rng(2026)
    rows = []
    gm, gp = M.groupby(CELL, dropna=False), P.groupby(CELL, dropna=False)
    for key, m in gm:
        if key not in gp.groups:
            continue
        p = gp.get_group(key)
        r = dict(zip(CELL, key))
        r["cell"] = cell_label(key)
        ok = True
        for nm, D in (("matlab", m), ("py", p)):
            n, k = len(D), int(D.reached.sum())
            lo, hi = wilson(k, n)
            r[f"n_{nm}"], r[f"reach_{nm}"] = n, k
            r[f"reach_lo_{nm}"], r[f"reach_hi_{nm}"] = lo, hi
            for col, out in (("n_reject", "rej"), ("gap", "gap"), ("total_time", "time")):
                mu, lo, hi = boot(D[col], rng)
                r[f"{out}_{nm}"], r[f"{out}_lo_{nm}"], r[f"{out}_hi_{nm}"] = mu, lo, hi
            r[f"coll_static_{nm}"] = int(D.coll_static.sum())
            r[f"coll_mover_{nm}"] = float(D.coll_mover.mean())
        r["agree_reach"] = overlap((r["reach_lo_matlab"], r["reach_hi_matlab"]),
                                   (r["reach_lo_py"], r["reach_hi_py"]))
        r["agree_rej"] = overlap((r["rej_lo_matlab"], r["rej_hi_matlab"]),
                                 (r["rej_lo_py"], r["rej_hi_py"]), 0.5)
        r["agree_gap"] = overlap((r["gap_lo_matlab"], r["gap_hi_matlab"]),
                                 (r["gap_lo_py"], r["gap_hi_py"]), 0.005)
        r["agree_time"] = overlap((r["time_lo_matlab"], r["time_hi_matlab"]),
                                  (r["time_lo_py"], r["time_hi_py"]), 0.5)
        r["agree"] = bool(r["agree_reach"] and r["agree_rej"] and r["agree_gap"])
        rows.append(r)
    X = pd.DataFrame(rows)
    order = {f"E{i}": i for i in range(1, 15)}
    X = X.sort_values(by=["exp", "cell"], key=lambda c: c.map(order) if c.name == "exp" else c)
    out_csv = os.path.join(ROOT, f"results/cross_stack{sfx}.csv")
    X.to_csv(out_csv, index=False)

    os.makedirs(os.path.join(ROOT, "results/generated"), exist_ok=True)
    with open(os.path.join(ROOT, f"results/generated/tab_cross{sfx}.tex"), "w") as fh:
        fh.write("% auto-generated by tools/cross_stack.py\n"
                 "\\begin{longtable}{llrrrrrrc}\n\\toprule\n"
                 "Exp & Cell & \\multicolumn{2}{c}{reach} & \\multicolumn{2}{c}{rej./run} & "
                 "\\multicolumn{2}{c}{min gap (m)} & agree\\\\\n"
                 " & & M & Py & M & Py & M & Py & \\\\\n\\midrule\n\\endhead\n")
        for _, r in X.iterrows():
            ag = "yes" if r.agree else "\\textbf{no}"
            fh.write(f"{r.exp} & {r.cell.replace('_', ' ')} & {r.reach_matlab}/{r.n_matlab} & "
                     f"{r.reach_py}/{r.n_py} & {r.rej_matlab:.1f} & {r.rej_py:.1f} & "
                     f"{r.gap_matlab:.2f} & {r.gap_py:.2f} & {ag}" + " \\\\\n")
        fh.write("\\bottomrule\n\\end{longtable}\n")

    n = len(X)
    L = [f"# Cross-stack comparison{' (' + a.tag + ')' if a.tag else ''}", "",
         f"MATLAB: `{os.path.relpath(a.matlab, ROOT)}` ({len(M)} runs). "
         f"Python: `{os.path.relpath(a.py, ROOT)}` ({len(P)} runs).", "",
         f"- cells compared: **{n}**",
         f"- agree on reach, rejections and minimum gap (all three intervals overlap): "
         f"**{int(X.agree.sum())}/{n}** ({100*X.agree.mean():.1f} %)",
         f"- reach intervals overlap: {int(X.agree_reach.sum())}/{n}; "
         f"rejection intervals: {int(X.agree_rej.sum())}/{n}; "
         f"gap intervals: {int(X.agree_gap.sum())}/{n}; "
         f"time intervals: {int(X.agree_time.sum())}/{n}",
         f"- static-contact steps, total: MATLAB {int(M.coll_static.sum())}, "
         f"Python {int(P.coll_static.sum())} (must be 0)",
         f"- runs reaching the goal: MATLAB {int(M.reached.sum())}/{len(M)}, "
         f"Python {int(P.reached.sum())}/{len(P)}", "",
         "Pearson correlation of the cell means across stacks: "
         + ", ".join(f"{lab} r = {np.corrcoef(X[c + '_matlab'], X[c + '_py'])[0, 1]:.3f}"
                     for lab, c in (("time", "time"), ("rejections", "rej"), ("min gap", "gap"))),
         "", "## Per experiment", "",
         "| exp | cells | agree | reach M | reach Py | time M (s) | time Py (s) |",
         "|---|---|---|---|---|---|---|"]
    for e, g in X.groupby("exp", sort=False):
        L.append(f"| {e} | {len(g)} | {int(g.agree.sum())} | "
                 f"{int(g.reach_matlab.sum())}/{int(g.n_matlab.sum())} | "
                 f"{int(g.reach_py.sum())}/{int(g.n_py.sum())} | "
                 f"{np.average(g.time_matlab, weights=g.n_matlab):.1f} | "
                 f"{np.average(g.time_py, weights=g.n_py):.1f} |")
    bad = X[~X.agree]
    L += ["", f"## Disagreeing cells ({len(bad)})", "",
          "| exp | cell | reach M | reach Py | rej M [CI] | rej Py [CI] | gap M [CI] | gap Py [CI] | differs in |",
          "|---|---|---|---|---|---|---|---|---|"]
    for _, r in bad.iterrows():
        why = ", ".join(w for w, f in (("reach", r.agree_reach), ("rejections", r.agree_rej),
                                       ("gap", r.agree_gap)) if not f)
        L.append(f"| {r.exp} | {r.cell} | {r.reach_matlab}/{r.n_matlab} | {r.reach_py}/{r.n_py} | "
                 f"{r.rej_matlab:.1f} [{r.rej_lo_matlab:.1f},{r.rej_hi_matlab:.1f}] | "
                 f"{r.rej_py:.1f} [{r.rej_lo_py:.1f},{r.rej_hi_py:.1f}] | "
                 f"{r.gap_matlab:.2f} [{r.gap_lo_matlab:.2f},{r.gap_hi_matlab:.2f}] | "
                 f"{r.gap_py:.2f} [{r.gap_lo_py:.2f},{r.gap_hi_py:.2f}] | {why} |")
    with open(os.path.join(ROOT, f"results/CROSS_STACK{sfx}.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L[:14]))
    print(f"-> {out_csv}")


if __name__ == "__main__":
    main()
