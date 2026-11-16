#!/usr/bin/env python3
"""Video clips of the ROS 2 runs, for the demo video.

    python3 tools/make_ros_demo_clips.py

Inputs: the real ROS 2 Humble logs and traces (tools/ros_humble_full.sh,
tools/ros_demo_runs.sh). Outputs in results/ros_humble/demo/:
  terminal_replay.mp4   the actual console output of the Humble run, line by line
  deploy_topdown.mp4    deployment over DDS: robots, shuttles and the lidar
                        returns each robot published on /robotN/scan
  watchdog_topdown.mp4  robot1's lidar stops: /swarm/safety goes to 1, the team stops
"""
import json
import os
import re
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
RH = os.path.join(ROOT, "results", "ros_humble")
OUT = os.path.join(RH, "demo")
sys.path.insert(0, PKG)
W, H = 1280, 720
ROBOT_C = ["#2a78d6", "#1baf7a", "#eda100", "#6250d6"]


def plt_mod():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def encode(frames_dir, fps, out, hold_s=0.0):
    vf = "format=yuv420p"
    if hold_s > 0:
        vf = f"tpad=stop_mode=clone:stop_duration={hold_s},format=yuv420p"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i",
                    os.path.join(frames_dir, "f%05d.png"), "-vf", vf, "-c:v", "libx264",
                    "-preset", "veryfast", "-crf", "20", "-r", "30", out], check=True)
    print("  wrote", os.path.relpath(out, ROOT))


# ------------------------------------------------------------------ terminal
def terminal_lines():
    full = open(os.path.join(RH, "full.log")).read().splitlines()
    tests = open(os.path.join(RH, "full_stage1_2.log")).read().splitlines()
    L = [("$", "docker run --rm --network host -v $PWD:/ws swarm-humble \\"),
         ("$", "    bash tools/ros_humble_full.sh            # ROS 2 Humble, real rclpy + DDS")]
    keep = lambda l: not re.search(r"warn|^\s*$|^\[", l, re.I)
    for l in tests:
        if keep(l) and (l.startswith("===") or "Summary" in l or "ros2_swarm_transport " in l
                        or "passed" in l):
            L.append(("", l.strip()))
    case = None
    for l in full:
        m = re.match(r"=== (\S+) payload=(\S+) phase=(\S+) gate_source=(\S+)", l)
        if l.startswith("=== 3."):
            L.append(("", l.strip()))
        if m:
            case = m.groups()
            L.append(("", f"=== lockstep {case[0]:10s} payload={case[1]:7s} phase={case[2]:5s} gate={case[3]}"))
            continue
        if case and re.match(r"^metric\s", l):
            L.append(("", "    " + l.rstrip()))
        if case and re.match(r"^(reached_goal|total_time|contact_steps_static|formation_rms)\s", l):
            L.append(("", "    " + l.rstrip()))
        if l.startswith("=== 4"):
            break
    d = json.load(open(os.path.join(RH, "launch_pavfh_on_top_learned.json")))
    L.append(("", "=== 4. ros2 launch ros2_swarm_transport mujoco_validation.launch.py algo:=pavfh ..."))
    L.append(("", f"    reached={d['reached_goal']}  t={d['total_time']:.1f}s  monitor false accepts="
                  f"{d['gate_stats']['false_accept']}/{d['gate_stats']['checks']}"))
    L.append(("", "=== 5. real-sensor deployment: robots.launch.py + robots publishing /odom, /scan"))
    for s in (1, 2, 3):
        p = os.path.join(RH, f"real_sensor_s{s}.json")
        if os.path.exists(p):
            r = json.load(open(p))
            L.append(("", f"    seed {s}: reached={r['reached']}  t={r['sim_time']}s  "
                          f"static contact steps={r['contact_steps_static']}"))
    wd = json.load(open(os.path.join(RH, "watchdog.json")))["watchdog"]
    L.append(("", "=== 6. watchdog: robot1 lidar stops at t=20.0 s"))
    L.append(("", f"    /swarm/safety=1, every cmd_vel = 0 (flag at t=20.6 s)  max speed after: "
                  f"{wd['max_cmd_speed_after']}"))
    return L


def terminal_replay():
    plt = plt_mod()
    lines = terminal_lines()
    fd = os.path.join(OUT, "frames_term")
    os.makedirs(fd, exist_ok=True)
    for f in os.listdir(fd):
        os.remove(os.path.join(fd, f))
    rows = 30
    for k in range(1, len(lines) + 1):
        fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
        fig.patch.set_facecolor("#1e1f22")
        fig.text(0.02, 0.965, "ROS 2 Humble (Docker, official ros:humble-ros-base) - actual console output",
                 color="#9aa0a6", fontsize=11, family="monospace")
        vis = lines[max(0, k - rows):k]
        for i, (pr, txt) in enumerate(vis):
            col = "#e8eaed"
            if txt.startswith("==="):
                col = "#8ab4f8"
            if "passed" in txt or re.search(r"True\s+True", txt):
                col = "#81c995"
            if pr == "$":
                col = "#fdd663"
            fig.text(0.02, 0.92 - i * 0.03, (pr + " " if pr else "") + txt[:118], color=col,
                     fontsize=10.5, family="monospace", va="top")
        fig.savefig(os.path.join(fd, f"f{k:05d}.png"), dpi=100, facecolor=fig.get_facecolor())
        plt.close(fig)
    encode(fd, 4, os.path.join(OUT, "terminal_replay.mp4"), hold_s=40)


# ------------------------------------------------------------------ top-down
def topdown(trace_name, out_name, t_from=0.0, t_to=None, fps=20, title="", flag_t=None):
    p = os.path.join(OUT, trace_name + "_trace.json")
    if not os.path.exists(p):
        print("  missing", p)
        return
    from matplotlib.patches import Circle, Polygon, Rectangle
    from matplotlib.transforms import Affine2D
    from ros2_swarm_transport.geometry import world_from_sdf
    plt = plt_mod()
    tr = json.load(open(p))
    sigj = os.path.join(OUT, trace_name + "_signals.json")
    flags = {round(s[0], 2): s[2] for s in json.load(open(sigj))["sig"]} if os.path.exists(sigj) else {}
    world = world_from_sdf(os.path.join(PKG, "worlds", "swarm_world_dynamic.sdf"))
    P = tr["poses"]
    S = tr["scans"]
    idx = [i for i, q in enumerate(P) if q["t"] >= t_from and (t_to is None or q["t"] <= t_to)]
    fd = os.path.join(OUT, "frames_" + out_name)
    os.makedirs(fd, exist_ok=True)
    for f in os.listdir(fd):
        os.remove(os.path.join(fd, f))
    allpos = np.array([q["pos"] for q in P]).reshape(-1, 2)
    cx = allpos[:, 0].mean()
    trail = []
    for n, i in enumerate(idx):
        q = P[i]
        fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
        fig.patch.set_facecolor("#fcfcfb")
        ax = fig.add_axes([0.03, 0.04, 0.94, 0.84])
        ax.set_facecolor("#fcfcfb")
        for b in world.boxes:
            r = Rectangle((-b.h[0], -b.h[1]), 2 * b.h[0], 2 * b.h[1], color="#52514e", lw=0)
            r.set_transform(Affine2D().rotate(b.yaw).translate(*b.c) + ax.transData)
            ax.add_patch(r)
        for c in world.circles:
            ax.add_patch(Circle(c.c, c.r, color="#52514e", lw=0))
        if i < len(S):
            pos0 = np.array(q["pos"])
            for k, pts in enumerate(S[i]):
                if pts:
                    a = np.array(pts)
                    if k == 0:                      # robot1's beams, faint
                        for e in a:
                            ax.plot([pos0[0, 0], e[0]], [pos0[0, 1], e[1]], color=ROBOT_C[0],
                                    lw=0.4, alpha=0.18, zorder=4)
                    ax.scatter(a[:, 0], a[:, 1], s=14, color=ROBOT_C[k % 4], alpha=0.9, lw=0,
                               zorder=5)
        for m in q["movers"]:
            x, y, sx, sy, circ = m
            if circ:
                ax.add_patch(Circle((x, y), sx / 2, color="#eb6834"))
            else:
                ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color="#eb6834"))
        pos = np.array(q["pos"])
        c = pos.mean(axis=0)
        trail.append(c)
        tr_ = np.array(trail)
        ax.plot(tr_[:, 0], tr_[:, 1], color="#2a78d6", lw=1.2, alpha=0.6)
        ang = np.arctan2(pos[:, 1] - c[1], pos[:, 0] - c[0])
        ax.add_patch(Polygon(pos[np.argsort(ang)], closed=True, color="#2a78d6", alpha=0.12))
        for k, (pp, yw) in enumerate(zip(pos, q["yaw"])):
            ax.add_patch(Circle(pp, 0.18, color=ROBOT_C[k % 4], ec="white", lw=1))
            ax.plot([pp[0], pp[0] + 0.3 * np.cos(yw)], [pp[1], pp[1] + 0.3 * np.sin(yw)],
                    color="white", lw=1.2)
        ax.set_xlim(c[0] - 7.5, c[0] + 7.5)
        ax.set_ylim(c[1] - 4.2, c[1] + 4.2)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for s_ in ax.spines.values():
            s_.set_visible(False)
        fl = flags.get(round(q["t"], 2), 0)
        fig.text(0.03, 0.94, title, fontsize=17, weight="bold", color="#0b0b0b")
        fig.text(0.03, 0.895, f"t = {q['t']:5.1f} s   coloured dots: the lidar returns each robot "
                 f"published on /robotN/scan   orange: shuttles", fontsize=11, color="#52514e")
        if flag_t is not None:
            lost = q["t"] >= flag_t
            fig.text(0.70, 0.94, "robot1 lidar: " + ("LOST" if lost else "ok"), fontsize=14,
                     color="#c6362f" if lost else "#1baf7a", weight="bold")
            fig.text(0.70, 0.895, f"/swarm/safety = {fl}" + ("   -> all robots stopped" if fl else ""),
                     fontsize=13, color="#c6362f" if fl else "#52514e", weight="bold")
        fig.savefig(os.path.join(fd, f"f{n:05d}.png"), dpi=100, facecolor="#fcfcfb")
        plt.close(fig)
    encode(fd, fps, os.path.join(OUT, out_name + ".mp4"), hold_s=20)


def main():
    os.makedirs(OUT, exist_ok=True)
    terminal_replay()
    topdown("deploy", "deploy_topdown", fps=24,
            title="ROS 2 deployment over DDS: PA-VFH steering from the robots' own lidar")
    topdown("watchdog", "watchdog_topdown", t_from=14.0, fps=10, flag_t=20.0,
            title="ROS 2 watchdog: a lidar fails, the whole team stops")


if __name__ == "__main__":
    main()
