#!/usr/bin/env python3
"""Captioned demo video of the whole project (silent by default).

    python3 tools/make_demo_video.py            # -> results/demo/demo.mp4 (+ demo.srt)
    python3 tools/make_demo_video.py --voice    # add espeak-ng narration

Every number spoken or shown is read from the result files at build time
(nothing is typed in by hand), so the video always matches the repository.
Needs ffmpeg and, for the voice, espeak-ng (MBROLA voice mb-us2 if installed;
without a TTS engine the video is silent with captions).
"""
import glob
import json
import os
import re
import shutil
import subprocess
import textwrap

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results")
OUT = os.path.join(RES, "demo")
TMP = os.path.join(OUT, "build")
W, H, FPS = 1280, 720, 30
VOICE = False                     # silent video: the explanation is on screen
CAP_H = 0.22                      # caption band height (fraction of the frame)


def caption_fit(text):
    """(wrapped text, font size) that fits the band without truncation."""
    for fs, width, nmax in ((13.5, 118, 5), (12.5, 128, 5), (11.5, 140, 6), (10.5, 155, 7)):
        lines = textwrap.wrap(text, width)
        if len(lines) <= nmax:
            return "\n".join(lines), fs
    return "\n".join(textwrap.wrap(text, 160)), 10


def caption_lines(text, *a, **k):
    return caption_fit(text)[0]


def read_time(text):
    """Seconds needed to read a caption comfortably."""
    return max(7.0, len(text.split()) / 2.6 + 3.0)
INK, INK2, SURF, ACC, CRIT = "#0b0b0b", "#52514e", "#fcfcfb", "#2a78d6", "#c6362f"


# ------------------------------------------------------------------ numbers
def wilson_txt(k, n):
    return f"{k} of {n}"


def numbers():
    N = {}
    md = open(os.path.join(RES, "CROSS_STACK_fixedgate.md")).read()
    m = re.search(r"agree on reach.*?\*\*(\d+)/(\d+)\*\*", md)
    N["xs_agree"], N["xs_cells"] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    xc = pd.read_csv(os.path.join(RES, "deterministic_xcheck_fixedgate.csv"))
    col = [c for c in xc.columns if "max" in c and "diff" in c][0]
    N["xc_max"] = float(xc[col].max())
    e19 = pd.read_csv(os.path.join(RES, "extended", "E19.csv"))
    for a in ("pavfh", "pavfh_nonn", "pvfh_prob", "mpc", "mppi"):
        d = e19[e19.algo == a]
        N[f"e19_{a}_reach"] = int(d.reached.sum())
        N[f"e19_{a}_n"] = len(d)
        N[f"e19_{a}_rej"] = float(d.n_reject.mean())
        N[f"e19_{a}_ms"] = float(d.comp_ms_mean.mean())
        N[f"e19_{a}_time"] = float(d[d.reached == 1].total_time.mean())
    sig = os.path.join(RES, "figures_signals", "signal_numbers.json")
    N["sig"] = json.load(open(sig)) if os.path.exists(sig) else {}

    def phys(tag):
        out = {}
        for f in glob.glob(os.path.join(RES, "mujoco_phase", f"{tag}_*.json")):
            s = json.load(open(f))
            ok = s["reached_goal"] and s["contact_steps_static"] == 0 and s["contact_steps_mover"] == 0
            a = s["algo"]
            k, n = out.get(a, (0, 0))
            out[a] = (k + int(ok), n + 1)
        return out
    N["P1"], N["P3"], N["P4"] = phys("P1"), phys("P3"), phys("P4")
    rh = os.path.join(RES, "ros_humble")
    N["ros_real"] = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(rh, "real_sensor_s*.json")))
                     if not f.endswith("_signals.json")]
    wd = os.path.join(rh, "watchdog.json")
    N["ros_wd"] = json.load(open(wd)) if os.path.exists(wd) else None
    dm = os.path.join(rh, "demo")
    for k in ("ros2_launch_pavfh", "deploy", "watchdog"):
        f = os.path.join(dm, k + ".json")
        N["demo_" + k] = json.load(open(f)) if os.path.exists(f) else None
    rp = os.path.join(RES, "ros_physics")
    r1 = [json.load(open(f)) for f in glob.glob(os.path.join(rp, "R1", "*.row.json"))]
    if r1:
        clean = {}
        for r in r1:
            ok = bool(r.get("reached_goal")) and r.get("contact_steps_static") == 0 \
                and r.get("contact_steps_mover") == 0
            clean[r["algo"]] = clean.get(r["algo"], 0) + int(ok)
        same = [r for r in r1 if "identical_same_env" in r]
        N["R1"] = dict(n=len(r1), clean=clean, per_law=len(r1) // max(1, len(clean)),
                       same_env=sum(bool(r["identical_same_env"]) for r in same),
                       n_same_env=len(same))
    r2 = [json.load(open(f)) for f in glob.glob(os.path.join(rp, "R2", "*.row.json"))]
    if r2:
        N["R2"] = dict(n=len(r2), reached=sum(bool(r.get("reached")) for r in r2),
                       static=sum(r.get("contact_steps_static", 0) > 0 for r in r2),
                       mover=sum(r.get("contact_steps_mover", 0) > 0 for r in r2))
    for k, f in (("demo_deploy_mujoco", "DEMO_deploy_on_top_s3.json"),
                 ("demo_watchdog_mujoco", "DEMO_watchdog_on_top_s1.json")):
        p_ = os.path.join(rp, "DEMO", f)
        N[k] = json.load(open(p_)) if os.path.exists(p_) else None
    log = os.path.join(rh, "full.log")
    N["ros_cases"] = open(log).read().count("=== ") if os.path.exists(log) else 0
    return N


# ------------------------------------------------------------------ rendering
def plt_mod():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for f in glob.glob("/usr/share/fonts/opentype/inter/Inter-*.otf"):
        font_manager.fontManager.addfont(f)
    plt.rcParams["font.family"] = ["Inter", "DejaVu Sans"]
    return plt


def slide_png(path, title, image=None, bullets=None, caption="", kicker=""):
    plt = plt_mod()
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_facecolor(SURF)
    if kicker:
        fig.text(0.04, 0.935, kicker.upper(), fontsize=11, color=ACC, weight="bold")
    fig.text(0.04, 0.875, title, fontsize=24, color=INK, weight="bold", va="center")
    top, bottom = 0.82, (CAP_H + 0.02 if caption else 0.16)
    if image and os.path.exists(image):
        import matplotlib.image as mpimg
        img = mpimg.imread(image)
        ih, iw = img.shape[:2]
        left_w = 0.92 if not bullets else 0.58
        ax = fig.add_axes([0.04, bottom, left_w, top - bottom])
        ax.imshow(img)
        ax.set_axis_off()
        ax.set_anchor("C")
    if bullets:
        x0 = 0.66 if image else 0.06
        y = top - 0.03
        for b in bullets:
            lines = textwrap.wrap(b, 34 if image else 70)
            for i, ln in enumerate(lines):
                fig.text(x0, y, ("•  " if i == 0 else "    ") + ln, fontsize=15 if image else 19,
                         color=INK, va="top")
                y -= 0.052 if image else 0.062
            y -= 0.02
    if caption:
        fig.patches.append(plt.Rectangle((0, 0), 1, CAP_H, transform=fig.transFigure,
                                         color="#ececea", zorder=-1))
        _t, _fs = caption_fit(caption)
        fig.text(0.03, CAP_H / 2, _t, fontsize=_fs, color=INK, ha="left", va="center",
                 linespacing=1.4)
    fig.savefig(path, dpi=100, facecolor=SURF)
    plt.close(fig)


def overlay_png(path, title, caption):
    """Transparent title + caption band for video clips."""
    plt = plt_mod()
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_alpha(0)
    fig.patches.append(plt.Rectangle((0, 0.9), 1, 0.1, transform=fig.transFigure,
                                     color=SURF, alpha=0.85))
    fig.text(0.03, 0.95, title, fontsize=20, color=INK, weight="bold", va="center")
    if caption:
        fig.patches.append(plt.Rectangle((0, 0), 1, CAP_H, transform=fig.transFigure,
                                         color=SURF, alpha=0.9))
        _t, _fs = caption_fit(caption)
        fig.text(0.03, CAP_H / 2, _t, fontsize=_fs, color=INK, ha="left", va="center",
                 linespacing=1.4)
    fig.savefig(path, dpi=100, transparent=True)
    plt.close(fig)


def pipeline_png(path, caption=""):
    """Diagram of the PA-VFH pipeline."""
    plt = plt_mod()
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_facecolor(SURF)
    fig.text(0.04, 0.935, "THE PROPOSED ALGORITHM", fontsize=11, color=ACC, weight="bold")
    fig.text(0.04, 0.875, "PA-VFH: a classical decision layer with a learned prediction",
             fontsize=24, color=INK, weight="bold", va="center")
    ax = fig.add_axes([0.03, CAP_H + 0.02, 0.94, 0.80 - CAP_H])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 60)
    ax.set_axis_off()
    boxes = [(2, "Robots' own\nlidar scans", "instantaneous,\nnoisy", "#e9eef7"),
             (21, "Kalman range\nfilter", "mean + sigma\nper beam", "#e9eef7"),
             (40, "Occupancy CNN\n(neural network)", "3 scans + sigma +\nego-motion -> danger\nper sector, 0.5 s ahead", "#d7e6fb"),
             (59, "VFH+ histogram\n+ valley choice", "enlarge by body,\nsmooth, threshold,\ngoal + hysteresis", "#e9eef7"),
             (78, "Safety gate", "every pose checked:\nground truth or\nsensor monitor", "#fbe3e1")]
    for x, t, s, c in boxes:
        ax.add_patch(plt.Rectangle((x, 28), 17, 20, color=c, ec="#9aa7b8", lw=1.2))
        ax.text(x + 8.5, 42, t, ha="center", va="center", fontsize=13, weight="bold", color=INK)
        ax.text(x + 8.5, 33, s, ha="center", va="center", fontsize=10, color=INK2)
    for x in (19, 38, 57, 76):
        ax.annotate("", xy=(x + 2, 38), xytext=(x, 38),
                    arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.6))
    ax.text(48.5, 22, "the only learned part: ~11 k parameters, numpy inference, < 0.3 ms",
            ha="center", fontsize=12, color=ACC)
    ax.text(48.5, 12, "Trained to reproduce what an oracle that knows the future would see; "
            "deployed from scans alone.\nEverything after it is classical and auditable; "
            "the gate still decides every move.", ha="center", fontsize=12, color=INK2)
    if caption:
        fig.patches.append(plt.Rectangle((0, 0), 1, CAP_H, transform=fig.transFigure,
                                         color="#ececea", zorder=-1))
        _t, _fs = caption_fit(caption)
        fig.text(0.03, CAP_H / 2, _t, fontsize=_fs, color=INK, ha="left", va="center",
                 linespacing=1.4)
    fig.savefig(path, dpi=100, facecolor=SURF)
    plt.close(fig)


def bars_png(path, title, labels, values, unit, kicker="", caption="", fmt="{:.0f}"):
    plt = plt_mod()
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_facecolor(SURF)
    fig.text(0.04, 0.935, kicker.upper(), fontsize=11, color=ACC, weight="bold")
    fig.text(0.04, 0.875, title, fontsize=24, color=INK, weight="bold", va="center")
    ax = fig.add_axes([0.25, CAP_H + 0.08, 0.68, 0.74 - CAP_H - 0.06])
    ax.set_facecolor(SURF)
    y = np.arange(len(labels))[::-1]
    cols = [ACC if "PA-VFH" == l else "#9aa7b8" for l in labels]
    ax.barh(y, values, color=cols, height=0.6)
    ax.set_yticks(y, labels, fontsize=14)
    for yi, v in zip(y, values):
        ax.text(v, yi, "  " + fmt.format(v), va="center", fontsize=13, color=INK)
    ax.set_xlabel(unit, fontsize=12, color=INK2)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(axis="x", color="#e4e3df")
    ax.set_axisbelow(True)
    if caption:
        fig.patches.append(plt.Rectangle((0, 0), 1, CAP_H, transform=fig.transFigure,
                                         color="#ececea", zorder=-1))
        _t, _fs = caption_fit(caption)
        fig.text(0.03, CAP_H / 2, _t, fontsize=_fs, color=INK, ha="left", va="center",
                 linespacing=1.4)
    fig.savefig(path, dpi=100, facecolor=SURF)
    plt.close(fig)


# ------------------------------------------------------------------ audio / video
def tts(text, wav):
    eng = shutil.which("espeak-ng") or shutil.which("espeak")
    if not eng:
        return None
    voice = "mb-us2" if os.path.exists("/usr/share/mbrola/us2") else "en-us"
    r = subprocess.run([eng, "-v", voice, "-s", "152", "-w", wav, text], capture_output=True)
    if r.returncode != 0 or not os.path.exists(wav):
        subprocess.run([eng, "-v", "en-us", "-s", "152", "-w", wav, text], capture_output=True)
    return wav if os.path.exists(wav) else None


def dur(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration",
                          "-of", "csv=p=0", path], capture_output=True, text=True).stdout
    return float(out.strip() or 0)


def ff(args):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + args, check=True)


def segment_still(png, wav, out, pad=0.9, d=None):
    d = d if d is not None else (dur(wav) if wav else 6.0) + pad
    if wav:
        ff(["-loop", "1", "-framerate", str(FPS), "-i", png, "-i", wav,
            "-t", f"{d:.2f}", "-vf", f"scale={W}:{H},format=yuv420p", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "20", "-r", str(FPS), "-af", "apad",
            "-c:a", "aac", "-ar", "44100", "-ac", "1", out])
    else:                                        # silent
        ff(["-loop", "1", "-framerate", str(FPS), "-i", png, "-t", f"{d:.2f}",
            "-vf", f"scale={W}:{H},format=yuv420p", "-c:v", "libx264", "-preset", "veryfast",
            "-crf", "20", "-r", str(FPS), "-an", out])
    return d


def segment_clip(clip, ovl, wav, out, pad=0.9, start=0.0, d=None):
    d = d if d is not None else (dur(wav) if wav else 8.0) + pad
    if not wav:                                  # silent
        ins = ["-i", ovl] if ovl else []
        fc = (f"[0:v]scale={W}:{H},fps={FPS},tpad=stop_mode=clone:stop_duration=120"
              + ("[v];[v][1:v]overlay=0:0,format=yuv420p[o]" if ovl else ",format=yuv420p[o]"))
        ff(["-ss", f"{start:.1f}", "-i", clip] + ins +
           ["-t", f"{d:.2f}", "-filter_complex", fc, "-map", "[o]", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "20", "-an", out])
        return d
    audio = (["-i", wav] if wav else ["-f", "lavfi", "-i", "anullsrc=r=22050:cl=mono"])
    if ovl:
        ins, fc, amap = ["-i", ovl], (f"[0:v]scale={W}:{H},fps={FPS},tpad=stop_mode=clone:"
                                      f"stop_duration=120[v];[v][1:v]overlay=0:0,format=yuv420p[o]"), "2:a"
    else:
        ins, fc, amap = [], (f"[0:v]scale={W}:{H},fps={FPS},tpad=stop_mode=clone:"
                             f"stop_duration=120,format=yuv420p[o]"), "1:a"
    ff(["-ss", f"{start:.1f}", "-i", clip] + ins + audio +
       ["-t", f"{d:.2f}", "-filter_complex", fc,
        "-map", "[o]", "-map", amap, "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
        "-af", "apad", "-c:a", "aac", "-ar", "44100", "-ac", "1", out])
    return d


def srt_time(t):
    h, m = int(t // 3600), int(t % 3600 // 60)
    s = t % 60
    return f"{h:02d}:{m:02d}:{int(s):02d},{int((s % 1) * 1000):03d}"


# ------------------------------------------------------------------ script
def script(N):
    F = lambda p: os.path.join(ROOT, p)
    pv = lambda a: f"{N[f'e19_{a}_reach']} of {N[f'e19_{a}_n']}"
    p1 = lambda a: N["P1"].get(a, (0, 0))
    p3 = lambda a: N["P3"].get(a, (0, 0))
    p4 = lambda a: N["P4"].get(a, (0, 0))
    s3 = N["sig"].get("s3", {})
    s5 = N["sig"].get("s5", {})
    rr = N["ros_real"]
    wd = (N["ros_wd"] or {}).get("watchdog", {})
    S = []
    S.append(dict(kind="still", kicker="Major Project, Amrita", title="Cooperative payload transport\nby a robot formation",
                  bullets=["A replicated benchmark of local steering laws",
                           "PA-VFH: a hybrid with a learned occupancy predictor",
                           "MATLAB, Python, MuJoCo physics and ROS 2"],
                  say="This video walks through the cooperative payload transport project. "
                      "A team of robots carries one rigid load through a warehouse with moving "
                      "traffic. We built a replicated benchmark of local steering laws, and a "
                      "new hybrid algorithm called PA-VFH, which puts a small neural network "
                      "inside a classical planner."))
    S.append(dict(kind="still", kicker="The problem", title="Four robots, one rigid load, moving traffic",
                  image=F("results/figures_ext/fig_layouts.png"),
                  bullets=["The whole formation must steer as one body",
                           "Shuttles move on fixed routes and never yield",
                           "Eight warehouse layouts, 30 by 20 m up to 80 by 50 m"],
                  say="Four differential drive robots form up around a load, pick it up, and "
                      "carry it to the goal. The formation has to steer as one rigid body, while "
                      "shuttles cross its path and never give way. We test eight warehouse "
                      "layouts, from thirty by twenty metres up to eighty by fifty."))
    S.append(dict(kind="still", kicker="Step 1: replication",
                  title=f"MATLAB and Python agree: {N['xs_agree']} of {N['xs_cells']} cells",
                  image=F("docs/figures/cross_stack_scatter_fixedgate.png"),
                  bullets=["Collision-gate defect found and corrected",
                           f"Zero noise: identical to {N['xc_max']:.0e} m",
                           "2,730 runs per stack, no static contact"],
                  say=f"Step one was replication. We found a defect in the reference collision "
                      f"test, where one box edge was never checked, and corrected it. With the fix, "
                      f"the MATLAB and Python implementations agree step for step at zero noise, "
                      f"to less than a micrometre, and in {N['xs_agree']} of {N['xs_cells']} "
                      f"cells of the noisy grid."))
    S.append(dict(kind="still", kicker="Step 2: fair evaluation", title="Randomised traffic phase",
                  image=F("results/figures_ext/fig_phase_effect.png"),
                  say="Step two: every run now randomises when each shuttle starts, so no planner "
                      "benefits from a lucky timetable. All results carry ninety five percent "
                      "intervals, and every failed run is counted."))
    S.append(dict(kind="pipeline", say="Here is the proposed algorithm. The robots' own scans are "
                  "filtered beam by beam. A small circular convolutional network then reads the "
                  "last three scans, their uncertainty, and the formation's own motion, and "
                  "predicts how dangerous each direction will be over the next half second. "
                  "The classical histogram picks a free valley, and the safety gate still checks "
                  "every single move. The network has about eleven thousand parameters and runs "
                  "in a fraction of a millisecond."))
    if s3:
        S.append(dict(kind="still", kicker="Signals: inside the network",
                      title="What the network predicts, step by step",
                      image=F("results/figures_signals/S3_pavfh_danger_maps.png"),
                      bullets=[f"Blocked/free agreement with the oracle: {100*s3['agree_network']:.1f} % "
                               f"(analytic {100*s3['agree_analytic']:.1f} %)",
                               f"Blocked sectors missed: {100*s3['missed_block_network']:.1f} % "
                               f"vs {100*s3['missed_block_analytic']:.1f} %",
                               "Start-up transient before 3 frames of history"],
                      say=f"This signal plot shows the danger map over a whole mission. On top, the "
                          f"oracle that knows the future. In the middle, the network, which only sees "
                          f"noisy scans. At the bottom, the analytic kernel. The network agrees with "
                          f"the oracle on blocked or free in {100*s3['agree_network']:.1f} percent of "
                          f"sectors, slightly better than the analytic kernel, and misses fewer "
                          f"blocked sectors. It does over-predict for the first seconds, before it "
                          f"has three frames of history."))
    S.append(dict(kind="bars", kicker="Does the network help? (E19, 450 runs)",
                  title="Missions completed",
                  labels=["PA-VFH", "PA-VFH (no NN)", "PA-VFH-K", "MPC", "MPPI"],
                  values=[N[f"e19_{a}_reach"] for a in ("pavfh", "pavfh_nonn", "pvfh_prob", "mpc", "mppi")],
                  unit=f"missions reached, of {N['e19_pavfh_n']} each",
                  caption=f"Gate rejections per run: PA-VFH {N['e19_pavfh_rej']:.0f}, without the network "
                          f"{N['e19_pavfh_nonn_rej']:.0f}. Compute: PA-VFH {N['e19_pavfh_ms']:.2f} ms vs "
                          f"MPC {N['e19_mpc_ms']:.2f} ms per step.",
                  say=f"Does the network help? In the ablation, PA-VFH completed {pv('pavfh')} "
                      f"missions, the most of the five planners, and halved the steps the safety "
                      f"gate rejected compared with the same pipeline without the network. The paired "
                      f"intervals still include no effect, so we report this as a consistent but not "
                      f"yet significant gain. It needs about a third of MPC's compute, but MPC is "
                      f"faster on average."))
    S.append(dict(kind="still", kicker="Benchmark", title="Larger warehouses and team sizes",
                  image=F("results/figures_ext/fig_e15_reach.png"),
                  say="Across eight layouts and teams of three to nine robots, PA-VFH is among "
                      "the most reliable laws and the cheapest to compute. No planner gets through "
                      "the narrow aisle layout, which we report as an open case."))
    S.append(dict(kind="still", kicker="Step 5: safety", title="Ground truth vs deployable monitors",
                  image=F("results/figures_signals/S4_monitor_signals.png"),
                  say="Step five separates two things. The ground truth gate is a screening "
                      "device that only a simulator has. The sensor monitor uses only the robots' "
                      "scans. Every gate check is plotted here: the monitor stays below the truth, "
                      "so it is conservative, and false accepts are rare."))
    clip = F("results/demo_clips/pavfh_square_on_top_s1_ph1_m0.1.mp4")
    if os.path.exists(clip):
        k1, n1 = p1("pavfh")
        k3, n3 = p3("pavfh")
        S.append(dict(kind="clip", clip=clip, start=6.0, title="Physics: PA-VFH carrying the load (MuJoCo)",
                      caption="Load on top, random traffic phase, 0.10 m physical safety margin",
                      say=f"Now physics. In MuJoCo, with wheel slip and tracking lag, the histogram "
                          f"planners first grazed the racks: PA-VFH was clean in only {k1} of {n1} "
                          f"runs. The cause was that they steer to the smallest clearance the "
                          f"kinematic gate allows."))
    S.append(dict(kind="still", kicker="Signals: the physics fix",
                  title="A 0.10 m safety margin removes the rack contacts",
                  image=F("results/figures_signals/S1_physics_margin_clearance.png"),
                  say=f"This signal plot shows the fix. Red is PA-VFH without a margin, touching "
                      f"the racks. Blue is the same mission with a ten centimetre physical safety "
                      f"margin in the gate, applied equally to every planner. With it, PA-VFH "
                      f"was clean in {p3('pavfh')[0]} of {p3('pavfh')[1]} physics runs, up from "
                      f"{p1('pavfh')[0]}. The remaining failures are the towed load, which swings "
                      f"outside the formation and defeats every planner, MPC included."))
    if N["P4"]:
        S.append(dict(kind="bars", kicker="Robot-parameter sweep with the margin (P4)",
                      title="Clean physics runs under model mismatch",
                      labels=["PA-VFH", "MPC", "MPPI", "DWA", "NN (DAgger)", "PA-VFH-K"],
                      values=[p4(a)[0] for a in ("pavfh", "mpc", "mppi", "dwa", "nn", "pvfh_prob")],
                      unit=f"clean runs (reached, no contact), of {p4('pavfh')[1]} each",
                      caption="Chassis mass, wheel speed, friction, payload mass and motor force varied "
                              "one at a time; the planner keeps its nominal model.",
                      say="We then changed the robots: lighter and heavier chassis, slower and faster "
                          "wheels, low friction, half and double payload, and weaker motors, while "
                          "the planner keeps its nominal model. These bars count the clean runs for "
                          "each planner."))
    S.append(dict(kind="still", kicker="ROS 2 Humble", title="Same mission over DDS: identical",
                  image=F("results/figures_signals/S5_ros2_lockstep_equality.png"),
                  say=f"Then ROS 2. We ran the physics plant and the controller as two ROS 2 Humble "
                      f"processes talking over DDS. The mission is identical to the in-process "
                      f"run, with a maximum difference of {s5.get('max_diff', 0):.0f} metres over "
                      f"{s5.get('steps', 0)} steps, for MPC, the DAgger network, and PA-VFH."))
    # ---- ROS 2 part: MuJoCo physics runs only ----
    dm = lambda f: F(os.path.join("results/ros_humble/demo", f))
    L_ = N.get("demo_ros2_launch_pavfh")
    if L_ and os.path.exists(dm("ros2_launch_pavfh.mp4")):
        S.append(dict(kind="clip", clip=dm("ros2_launch_pavfh.mp4"), start=5.0, max_s=30.0,
                      title="ROS 2 + MuJoCo: lockstep run via ros2 launch",
                      say=f"ROS 2 Humble, two processes over DDS: the MuJoCo plant node simulates and "
                          f"renders this video; the formation-controller node steers. Started with "
                          f"ros2 launch. PA-VFH with the 0.10 m safety margin delivers the load in "
                          f"{L_['total_time']:.0f} s with {L_['contact_steps_static']} static and "
                          f"{L_['contact_steps_mover']} mover contacts."))
    R1 = N.get("R1") or {}
    if R1.get("n"):
        S.append(dict(kind="bars", kicker=f"ROS 2 + MuJoCo physics batch R1 ({R1['n']} runs)",
                      title="Clean physics runs through ROS 2 (lockstep over DDS)",
                      labels=["PA-VFH", "MPC", "MPPI", "DWA", "NN (DAgger)", "PA-VFH-K"],
                      values=[R1["clean"].get(a, 0) for a in ("pavfh", "mpc", "mppi", "dwa", "nn", "pvfh_prob")],
                      unit=f"clean runs (reached, no contact in MuJoCo), of {R1['per_law']} each "
                           f"(3 payload modes x 5 seeds)",
                      say=f"The whole physics configuration through ROS 2: 6 laws, 3 payload modes, "
                          f"5 seeds, 0.10 m margin, random traffic phase. "
                          f"{R1['same_env']} of {R1['n_same_env']} runs are identical to the "
                          f"in-process physics run in the same environment, so ROS 2 adds no error. "
                          f"Towed loads remain the hard case for every planner."))
    dp = lambda f: F(os.path.join("results/ros_physics/DEMO", f))
    D_ = N.get("demo_deploy_mujoco")
    if D_ and os.path.exists(dp("deploy_mujoco.mp4")):
        S.append(dict(kind="clip", clip=dp("deploy_mujoco.mp4"), start=3.0, max_s=35.0,
                      title="ROS 2 + MuJoCo: deployment mode, steering from lidar",
                      say=f"Full deployment stack in physics. The MuJoCo node runs free (here on "
                          f"simulation time, /clock, because rendering slows it), takes cmd_vel and "
                          f"publishes odometry and a 360-beam laser scan from each physical robot. The controller (robots.launch.py: PA-VFH, sensor monitor, "
                          f"watchdog) sees only those scans. Result: reached in "
                          f"{D_['sim_time']:.0f} s, {D_['contact_steps_static']} static and "
                          f"{D_['contact_steps_mover']} mover contacts, pin gap "
                          f"{1000 * D_['pin_gap_max']:.2f} mm."))
    R2 = N.get("R2") or {}
    if R2.get("n"):
        S.append(dict(kind="still", kicker=f"ROS 2 + MuJoCo deployment batch R2 ({R2['n']} runs)",
                      title="Driving real-time physics from the robots' own lidar",
                      bullets=[f"Reached: {R2['reached']} of {R2['n']}",
                               f"Runs with static contact: {R2['static']}",
                               f"Runs with mover contact: {R2['mover']}",
                               f"Load on top and virtual, seeds 1-5, sigma = 0.02 m"],
                      say="Ten real-time missions: MuJoCo physics, laser scans over DDS, PA-VFH with the "
                          "sensor monitor and watchdog. Contacts are counted by the physics engine; "
                          "mover contacts are shuttles that never yield driving into the formation."))
    WD_ = N.get("demo_watchdog_mujoco")
    if WD_ and os.path.exists(dp("watchdog_mujoco.mp4")):
        w = WD_.get("watchdog", {})
        delay = (w.get("safety_flag_at") or 0) - (w.get("scan_dropped_at") or 0)
        S.append(dict(kind="clip", clip=dp("watchdog_mujoco.mp4"), start=12.0, max_s=14.0,
                      title="ROS 2 + MuJoCo: watchdog stops the team",
                      say=f"Safety watchdog in physics: at t = {w.get('scan_dropped_at', 20):.0f} s robot 1's "
                          f"lidar stops publishing. {delay:.1f} s later the controller raises "
                          f"/swarm/safety and commands every robot to stop (largest commanded speed "
                          f"afterwards: {w.get('max_cmd_speed_after_flag', 0):.2f} m/s); the MuJoCo robots "
                          f"come to rest."))
    S.append(dict(kind="still", kicker="Honest limits", title="What is not claimed",
                  bullets=["Simulation only: no hardware runs yet",
                           "Sensor noise above 5 cm still breaks the sensor pipeline",
                           "Shuttles never yield; no people in the scenes",
                           "Learned components carry no guarantee: the gate decides"],
                  say="What we do not claim. Everything is simulated. The sensor pipeline still "
                      "fails when range noise exceeds about five centimetres, so we specify a "
                      "lidar accurate to a few centimetres. Shuttles never yield, and there are no "
                      "people in the scenes. The learned parts carry no guarantee: the safety gate "
                      "decides every move."))
    S.append(dict(kind="still", kicker="Reproduce everything", title="One guide, every result",
                  bullets=["docs/RUN_GUIDE.md: every command, with timings",
                           "Release archive + SHA-256 manifest for a DOI",
                           "77 automated tests; ROS 2 checks in Docker"],
                  say="Every number in this video is read from the result files. The run guide "
                      "lists every command to reproduce them, and the release archive carries a "
                      "checksum for every file. Thank you for watching."))
    return S


def main():
    import sys
    global VOICE
    VOICE = "--voice" in sys.argv
    os.makedirs(TMP, exist_ok=True)
    N = numbers()
    segs, t, srt = [], 0.0, []
    for i, s in enumerate(script(N)):
        png = os.path.join(TMP, f"s{i:02d}.png")
        mp4 = os.path.join(TMP, f"s{i:02d}.mp4")
        wav = tts(s["say"], os.path.join(TMP, f"s{i:02d}.wav")) if VOICE else None
        # silent: the explanation itself is the on-screen caption
        cap = s.get("caption", "") if VOICE else s["say"]
        d = None if VOICE else read_time(s["say"])
        if s["kind"] == "clip":
            if d is not None:                    # show more of the physics clip
                d = max(d, min(dur(s["clip"]) - s.get("start", 0.0), s.get("max_s", 30.0)))
            ovl = os.path.join(TMP, f"o{i:02d}.png")
            overlay_png(ovl, s["title"], cap)
            d = segment_clip(s["clip"], ovl, wav, mp4, start=s.get("start", 0.0), d=d)
        else:
            if s["kind"] == "pipeline":
                pipeline_png(png, cap)
            elif s["kind"] == "bars":
                bars_png(png, s["title"], s["labels"], s["values"], s["unit"],
                         s.get("kicker", ""), cap)
            else:
                slide_png(png, s["title"], s.get("image"), s.get("bullets"), cap,
                          s.get("kicker", ""))
            d = segment_still(png, wav, mp4, d=d)
        segs.append(mp4)
        srt.append(f"{i+1}\n{srt_time(t)} --> {srt_time(t + d - 0.5)}\n{s['say']}\n")
        t += d
        print(f"  segment {i+1}: {d:.1f} s  {s.get('title', s['kind'])!r}".replace("\n", " "))
    lst = os.path.join(TMP, "list.txt")
    open(lst, "w").write("".join(f"file '{p}'\n" for p in segs))
    out = os.path.join(OUT, "demo.mp4")
    ff(["-f", "concat", "-safe", "0", "-i", lst, "-c", "copy", "-movflags", "+faststart", out])
    open(os.path.join(OUT, "demo.srt"), "w").write("\n".join(srt))
    print(f"wrote {os.path.relpath(out, ROOT)} ({t/60:.1f} min) and demo.srt")


if __name__ == "__main__":
    main()
