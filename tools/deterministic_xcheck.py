"""Zero-noise cross-check: Python headless vs MATLAB per-step logs.

With sigma0 = 0 on the lidar no random number matters, so the two
implementations should follow the same trajectory. Reports, per law, the
maximum position difference over the common steps and the first step where
the difference exceeds 1 mm.

    python3 tools/deterministic_xcheck.py            (after tools/run_matlab_xcheck.sh)
"""
import os, sys
import numpy as np, pandas as pd
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "ros2_swarm_transport"))
from ros2_swarm_transport.headless_runner import run_mission, write_csv
W = os.path.join(ROOT, "src/ros2_swarm_transport/worlds")
GATE = sys.argv[1] if len(sys.argv) > 1 else "matlab"
out = []
for algo in ["apf", "vfh", "fgm", "dwa", "mpc", "pvfh", "pvfh_filt", "pvfh_prob"]:
    mfile = os.path.join(ROOT, "matlab_ref/logs", f"x_{algo}_s0.csv")
    if not os.path.exists(mfile):
        print("missing", mfile); continue
    rows, S = run_mission(f"{W}/swarm_world_dynamic.sdf", f"{W}/dynamic_obstacles.json",
                          algo=algo, sigma0=0.0, seed=1, verbose=False, gate=GATE)
    write_csv(rows, os.path.join(ROOT, "results/xcheck", f"py_{GATE}_{algo}_s0.csv"))
    p = pd.DataFrame(rows); m = pd.read_csv(mfile)
    n = min(len(p), len(m))
    d = np.hypot(p.leader_x[:n].values - m.leader_x[:n].values,
                 p.leader_y[:n].values - m.leader_y[:n].values)
    dr = max(np.abs(p[c][:n].values - m[c][:n].values).max()
             for c in p.columns if c.startswith("r") and c[-2:] in ("_x", "_y"))
    bad = np.where(d > 1e-3)[0]
    first = "never" if len(bad) == 0 else f"t={m.t[bad[0]]:.1f}s (phase {int(m.phase[bad[0]])})"
    out.append(dict(algo=algo, steps_py=len(p), steps_matlab=len(m),
                    max_centroid_diff_m=d.max(), max_robot_diff_m=dr,
                    first_gt_1mm=first, t_py=p.t.iloc[-1], t_matlab=m.t.iloc[-1],
                    reject_py=int((p.reject[p.phase == 2] > 0).sum()),
                    reject_matlab=int((m.reject[m.phase == 2] > 0).sum())))
df = pd.DataFrame(out)
df.to_csv(os.path.join(ROOT, f"results/deterministic_xcheck_{GATE}.csv"), index=False)
pd.set_option("display.width", 200)
print(df.to_string(index=False, float_format=lambda x: f"{x:.2e}" if x < 0.01 else f"{x:.3f}"))
