#!/usr/bin/env python3
"""Build the archival release (code + scenarios + configurations + derived
results) with a SHA-256 manifest, ready for a Zenodo deposit (DOI).

    python3 tools/make_release.py --version 1.0.0
        -> dist/cooperative-transport-benchmark-1.0.0.tar.gz        (code, worlds,
           configs, models, summary results, docs)
        -> dist/cooperative-transport-benchmark-1.0.0-runs.tar.gz   (per-step logs)
        -> MANIFEST.sha256 (in the repo root and inside the archive)

Nothing is uploaded. Publishing a GitHub release with Zenodo's GitHub
integration enabled (or uploading the two archives to Zenodo by hand) mints
the DOI; see docs/RELEASE.md.
"""
import argparse
import hashlib
import os
import tarfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CORE = ["README.md", "CITATION.cff", ".zenodo.json", "docs", "tools",
        "matlab_ref", "src/ros2_swarm_transport", "bundle",
        "results/results_headless.csv", "results/results_headless_exactgate.csv",
        "results/results_headless_phase.csv", "results/matlab_fixedgate",
        "results/cross_stack.csv", "results/cross_stack_exactgate.csv",
        "results/cross_stack_fixedgate.csv", "results/CROSS_STACK.md",
        "results/CROSS_STACK_exactgate.md", "results/CROSS_STACK_fixedgate.md",
        "results/deterministic_xcheck_matlab.csv", "results/deterministic_xcheck_exact.csv",
        "results/deterministic_xcheck_fixedgate.csv",
        "results/warehouse_headless.csv", "results/warehouse_matlab.csv",
        "results/warehouse_mover_timing.csv", "results/warehouse_xcheck.csv",
        "results/mujoco_summary.csv", "results/mujoco_phase_summary.csv",
        "results/extended", "results/tuning", "results/figures", "results/figures_ext",
        "results/generated", "results/generated_matlabscript", "results/RESULTS_LOG_headless.md",
        "results/ros_sensor_closedloop.json", "results/mujoco_ros_stub/summary.json",
        "results/mujoco_ros_stub/summary_learned_phase.json",
        "results/mujoco_ros_stub/stub_truth.log", "results/xcheck_fixedgate",
        "results/CROSS_STACK_fixedgate.md", "results/cross_stack_fixedgate.csv",
        "results/figures_signals", "results/signals", "results/ros_humble",
        "results/demo/demo.mp4", "results/demo/demo.srt", "results/demo_clips",
        "results/ros_physics"]
RUNS = ["results/runs", "results/runs_exactgate", "results/xcheck", "results/mujoco",
        "results/mujoco_phase", "results/mujoco_ros", "results/mujoco_ros_stub",
        "matlab_ref/logs/paper/runs"]
SKIP_DIRS = {"__pycache__", ".git", "build", "install", "log"}
SKIP_IN_CORE = {"matlab_ref/logs/paper/runs"}


def files(paths, skip=()):
    for p in paths:
        full = os.path.join(ROOT, p)
        if os.path.isfile(full):
            yield p
        elif os.path.isdir(full):
            for d, dn, fn in os.walk(full):
                rel = os.path.relpath(d, ROOT)
                dn[:] = [x for x in dn if x not in SKIP_DIRS
                         and os.path.join(rel, x) not in skip]
                for f in sorted(fn):
                    if not f.endswith((".pyc", ".mp4")):
                        yield os.path.join(rel, f)


def sha(path):
    h = hashlib.sha256()
    with open(os.path.join(ROOT, path), "rb") as fh:
        for b in iter(lambda: fh.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True)
    ap.add_argument("--no-runs", action="store_true")
    a = ap.parse_args()
    dist = os.path.join(ROOT, "dist")
    os.makedirs(dist, exist_ok=True)
    name = f"cooperative-transport-benchmark-{a.version}"
    core = sorted(set(files(CORE, SKIP_IN_CORE)))
    runs = [] if a.no_runs else sorted(set(files(RUNS)))
    man = os.path.join(ROOT, "MANIFEST.sha256")
    with open(man, "w") as fh:
        for f in core + runs:
            fh.write(f"{sha(f)}  {f}\n")
    with tarfile.open(os.path.join(dist, name + ".tar.gz"), "w:gz") as tf:
        for f in core + ["MANIFEST.sha256"]:
            tf.add(os.path.join(ROOT, f), arcname=os.path.join(name, f))
    if runs:
        with tarfile.open(os.path.join(dist, name + "-runs.tar.gz"), "w:gz") as tf:
            for f in runs:
                tf.add(os.path.join(ROOT, f), arcname=os.path.join(name, f))
    print(f"{len(core)} core files, {len(runs)} run logs; manifest {man}; archives in {dist}")


if __name__ == "__main__":
    main()
