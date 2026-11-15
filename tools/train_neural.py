#!/usr/bin/env python3
"""Train the two neural components (src/ros2_swarm_transport/.../neural.py).

    python3 tools/train_neural.py policy  --jobs 2     # DAgger, lattice-MPC expert
    python3 tools/train_neural.py monitor --jobs 2     # learned monitor + conformal q
    python3 tools/train_neural.py occupancy --jobs 2   # PA-VFH predictive occupancy

Data split (fixed, so the evaluation is honest):
  TRAIN worlds   rand01..rand05, swarm_world_dynamic, plaza
  HELD-OUT       rand06..rand10, maze, awswh, every wh_* layout (never seen)
  TRAIN teams    triangle (3 robots), square (4), grid2x3 (6)
  HELD-OUT team  grid3x3 (9 robots)
Training missions use seeds 101.. (the evaluation uses 1..10), randomised
traffic phase, sigma0 drawn from {0.02, 0.10}.

Outputs: src/ros2_swarm_transport/models/{policy_dagger,monitor_conformal}.npz
plus a JSON training log next to each.
"""
import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
W = os.path.join(PKG, "worlds")
MODELS = os.path.join(PKG, "models")
sys.path.insert(0, PKG)

TRAIN_WORLDS = [f"rand{k:02d}" for k in range(1, 6)] + ["dynamic", "plaza"]
TRAIN_TEAMS = [("square", 1.18), ("triangle", 1.18), ("grid2x3", 0.59)]
MPC_LEVELS = np.array([-1.2, -0.6, 0.0, 0.6, 1.2])


def paths(w):
    sdf = os.path.join(W, f"swarm_world_{w}.sdf")
    js = os.path.join(W, "dynamic_obstacles.json" if w == "dynamic"
                      else f"dynamic_obstacles_{w}.json")
    return sdf, js


# ---------------------------------------------------------------------------
# rollouts (run in worker processes)
# ---------------------------------------------------------------------------

def rollout(args):
    kind, w, form, sp, seed, sigma0, beta, model, algo, gate_source = args
    from ros2_swarm_transport.headless_runner import run_mission
    sdf, js = paths(w)
    rec = []
    kw = dict(algo=algo, seed=seed, sigma0=sigma0, formation=form,
              grid_spacing=sp, verbose=False, phase=True, timesteps=1500)
    if kind == "occupancy":
        kw.update(vfh_over={"nn": {"occ_record": rec, "occ_model": model}})
        try:
            _, S = run_mission(sdf, js, **kw)
            ok = bool(S["reached_goal"])
        except Exception as e:                   # noqa: BLE001
            print(f"rollout failed {args}: {e}", flush=True)
            ok = False
        X = np.array([r[0] for r in rec], dtype=np.float32)
        Y = np.array([r[1] for r in rec], dtype=np.float32)
        return dict(X=X, Y=Y, ok=ok, run=args)
    if kind == "policy":
        kw.update(nn_record=rec, nn_beta=beta, nn_model=model)
    else:
        kw.update(mon_record=rec, gate_source=gate_source)
    try:
        _, S = run_mission(sdf, js, **kw)
        ok = bool(S["reached_goal"])
    except Exception as e:                       # noqa: BLE001
        print(f"rollout failed {args}: {e}", flush=True)
        ok = False
    if kind == "policy":
        scans = np.array([r[0][0] for r in rec], dtype=np.float32)
        slots = [r[0][1].astype(np.float32) for r in rec]
        aux = np.array([r[0][2] for r in rec], dtype=np.float32)
        y = np.array([r[1] for r in rec], dtype=np.float32)
        return dict(scans=scans, slots=slots, aux=aux, y=y, ok=ok, run=args)
    X = np.array([r[0] for r in rec], dtype=np.float32)
    c = np.array([r[1] for r in rec], dtype=np.float32)
    t = np.array([r[2] for r in rec], dtype=bool)
    return dict(X=X, c=c, t=t, ok=ok, run=args)


def run_pool(jobs, n):
    out = []
    with ProcessPoolExecutor(n) as ex:
        for r in ex.map(rollout, jobs):
            out.append(r)
    return out


# ---------------------------------------------------------------------------
# models (torch)
# ---------------------------------------------------------------------------

def build_policy(n_cls):
    import torch.nn as nn

    class Policy(nn.Module):
        def __init__(self):
            super().__init__()
            cc = dict(kernel_size=5, padding=2, padding_mode="circular")
            self.c1 = nn.Conv1d(2, 16, **cc)
            self.c2 = nn.Conv1d(16, 32, **cc)
            self.c3 = nn.Conv1d(32, 32, **cc)
            self.fs = nn.Linear(32 * 18, 128)
            self.s1 = nn.Linear(2, 32)
            self.s2 = nn.Linear(32, 32)
            self.h1 = nn.Linear(128 + 64 + 5, 128)
            self.h2 = nn.Linear(128, 128)
            self.h3 = nn.Linear(128, n_cls)

        def forward(self, scan, slots, mask, aux):
            import torch
            import torch.nn.functional as F
            h = F.relu(self.c1(scan))
            h = F.max_pool1d(F.relu(self.c2(h)), 2)
            h = F.max_pool1d(F.relu(self.c3(h)), 2)
            hs = F.relu(self.fs(h.flatten(1)))
            e = F.relu(self.s2(F.relu(self.s1(slots)))) * mask[..., None]
            mean = e.sum(1) / mask.sum(1, keepdim=True)
            mx = e.max(1).values            # relu >= 0, padded rows are 0
            z = torch.cat([hs, mean, mx, aux], 1)
            return self.h3(F.relu(self.h2(F.relu(self.h1(z)))))
    return Policy()


def build_monitor():
    import torch.nn as nn

    class Monitor(nn.Module):
        def __init__(self):
            super().__init__()
            cc = dict(kernel_size=5, padding=2, padding_mode="circular")
            self.c1 = nn.Conv1d(3, 16, **cc)
            self.c2 = nn.Conv1d(16, 32, **cc)
            self.h1 = nn.Linear(32 * 36 + 2, 128)
            self.h2 = nn.Linear(128, 64)
            self.h3 = nn.Linear(64, 1)

        def forward(self, x):
            import torch
            import torch.nn.functional as F
            h = F.relu(self.c1(x))
            h = F.max_pool1d(F.relu(self.c2(h)), 2)
            z = torch.cat([h.flatten(1), x[:, 0].min(1, keepdim=True).values,
                           x[:, 1].min(1, keepdim=True).values], 1)
            return self.h3(F.relu(self.h2(F.relu(self.h1(z))))).squeeze(1)
    return Monitor()


def build_occupancy(cin=6):
    import torch.nn as nn

    class Occ(nn.Module):
        def __init__(self):
            super().__init__()
            cc = dict(kernel_size=5, padding=2, padding_mode="circular")
            self.c1 = nn.Conv1d(cin, 32, **cc)
            self.c2 = nn.Conv1d(32, 32, **cc)
            self.c3 = nn.Conv1d(32, 32, **cc)
            self.out = nn.Conv1d(32, 1, kernel_size=1)

        def forward(self, x):
            import torch.nn.functional as F
            h = F.relu(self.c3(F.relu(self.c2(F.relu(self.c1(x))))))
            return self.out(h).squeeze(1)          # logits (B, S)
    return Occ()


def fit_occupancy(X, Y, epochs, seed=0):
    import torch
    torch.manual_seed(seed)
    torch.set_num_threads(max(1, int(os.environ.get("TORCH_THREADS", "1"))))
    n = len(Y)
    idx = np.random.default_rng(seed).permutation(n)
    nv = max(1, n // 10)
    va, tr = idx[:nv], idx[nv:]
    Xt, Yt = torch.as_tensor(X), torch.as_tensor(Y)
    model = build_occupancy(X.shape[1])
    opt = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    # sectors near the decision threshold matter most: up-weight u > 0.1
    for ep in range(epochs):
        perm = np.random.default_rng(seed + ep).permutation(tr)
        for b in range(0, len(perm), 256):
            j = perm[b:b + 256]
            opt.zero_grad()
            w = 1.0 + 3.0 * (Yt[j] > 0.1).float()
            loss = torch.nn.functional.binary_cross_entropy_with_logits(
                model(Xt[j]), Yt[j], weight=w)
            loss.backward()
            opt.step()
        sched.step()
    with torch.no_grad():
        pv = torch.sigmoid(model(Xt[va])).numpy()
    yv = Y[va]
    thr = 0.3
    stats = dict(val_mae=float(np.abs(pv - yv).mean()),
                 val_block_agree=float(((pv > thr) == (yv > thr)).mean()),
                 val_missed_block=float(((pv <= thr) & (yv > thr)).sum() / max((yv > thr).sum(), 1)))
    return model, stats


def cmd_occupancy(a):
    os.makedirs(MODELS, exist_ok=True)
    rng = np.random.default_rng(3)
    path = os.path.join(MODELS, "pavfh_occupancy.npz")
    log = dict(rounds=[], train_worlds=TRAIN_WORLDS,
               train_teams=[f"{f}@{s}" for f, s in TRAIN_TEAMS],
               label="oracle danger exp(-d^2/2 sigma_h^2) of the true sector range "
                     "minimum over the prediction horizon (0.5 s)")
    XS, YS = [], []
    seed = 701
    for rnd, driver in enumerate(("pavfh_nonn", "pavfh")):
        jobs = []
        for w in TRAIN_WORLDS:
            for form, sp in TRAIN_TEAMS:
                for _ in range(a.per_cell):
                    jobs.append(("occupancy", w, form, sp, seed,
                                 float(rng.choice([0.02, 0.10, 0.20])), 0.0, path,
                                 driver, "truth"))
                    seed += 1
        t0 = time.time()
        R = run_pool(jobs, a.jobs)
        for r in R:
            if len(r["Y"]):
                XS.append(r["X"])
                YS.append(r["Y"])
        X, Y = np.concatenate(XS), np.concatenate(YS)
        model, st = fit_occupancy(X, Y, a.epochs, seed=rnd)
        export(model, path, {})
        from ros2_swarm_transport import neural
        neural._CACHE.clear()
        rec = dict(round=rnd, driver=driver, rollouts=len(R),
                   rollout_reach=float(np.mean([r["ok"] for r in R])),
                   samples=int(len(Y)), wall_s=round(time.time() - t0, 1), **st)
        log["rounds"].append(rec)
        print(json.dumps(rec), flush=True)
        json.dump(log, open(os.path.join(MODELS, "pavfh_occupancy.json"), "w"), indent=2)


def export(model, path, extra):
    sd = {k: v.detach().cpu().numpy().astype(np.float64)
          for k, v in model.state_dict().items()}
    sd.update(extra)
    np.savez(path, **sd)


def pad_slots(slots, kmax=9):
    n = len(slots)
    S = np.zeros((n, kmax, 2), np.float32)
    M = np.zeros((n, kmax), np.float32)
    for i, s in enumerate(slots):
        S[i, :len(s)] = s
        M[i, :len(s)] = 1
    return S, M


def fit_policy(D, epochs, seed=0):
    import torch
    torch.manual_seed(seed)
    torch.set_num_threads(max(1, int(os.environ.get("TORCH_THREADS", "1"))))
    y = np.abs(D["y"][:, None] - MPC_LEVELS[None, :]).argmin(1)
    S, M = pad_slots(D["slots"])
    t = lambda a: torch.as_tensor(a)              # noqa: E731
    scans, slots, mask, aux, yy = t(D["scans"]), t(S), t(M), t(D["aux"]), t(y)
    n = len(yy)
    idx = np.random.default_rng(seed).permutation(n)
    nv = max(1, n // 10)
    va, tr = idx[:nv], idx[nv:]
    model = build_policy(len(MPC_LEVELS))
    # class weights: the expert goes straight most of the time
    cnt = np.bincount(y[tr], minlength=len(MPC_LEVELS)).astype(float)
    wts = torch.as_tensor((cnt.sum() / np.maximum(cnt, 1)) ** 0.5, dtype=torch.float32)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    lossf = torch.nn.CrossEntropyLoss(weight=wts)
    hist = []
    for ep in range(epochs):
        model.train()
        perm = np.random.default_rng(seed + ep).permutation(tr)
        for b in range(0, len(perm), 256):
            j = perm[b:b + 256]
            opt.zero_grad()
            loss = lossf(model(scans[j], slots[j], mask[j], aux[j]), yy[j])
            loss.backward()
            opt.step()
        sched.step()
        model.eval()
        with torch.no_grad():
            pv = model(scans[va], slots[va], mask[va], aux[va]).argmax(1).numpy()
        acc = float((pv == y[va]).mean())
        hist.append(dict(epoch=ep, loss=float(loss), val_acc=acc))
    return model, hist


def fit_monitor(X, c, epochs, seed=0):
    import torch
    torch.manual_seed(seed)
    torch.set_num_threads(max(1, int(os.environ.get("TORCH_THREADS", "1"))))
    yt = np.clip(c, -0.5, 3.0).astype(np.float32)
    w = np.where(yt < 1.0, 4.0, 1.0).astype(np.float32)   # weight the boundary
    Xt, Yt, Wt = (torch.as_tensor(a) for a in (X, yt, w))
    model = build_monitor()
    opt = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    n = len(Yt)
    for ep in range(epochs):
        perm = np.random.default_rng(seed + ep).permutation(n)
        for b in range(0, n, 512):
            j = perm[b:b + 512]
            opt.zero_grad()
            r = model(Xt[j]) - Yt[j]
            loss = (Wt[j] * torch.nn.functional.smooth_l1_loss(
                r, torch.zeros_like(r), reduction="none", beta=0.05)).mean()
            loss.backward()
            opt.step()
        sched.step()
    return model


def predict_monitor(model, X):
    import torch
    with torch.no_grad():
        return model(torch.as_tensor(X)).numpy()


def conformal_q(resid, alpha):
    """One-sided split-conformal quantile: P(resid_new > q) <= alpha."""
    n = len(resid)
    k = int(math.ceil((n + 1) * (1 - alpha)))
    if k > n:
        return float("inf")
    return float(np.sort(resid)[k - 1])


# ---------------------------------------------------------------------------

def cmd_policy(a):
    os.makedirs(MODELS, exist_ok=True)
    rng = np.random.default_rng(0)
    D = dict(scans=[], slots=[], aux=[], y=[])
    log = dict(iterations=[], train_worlds=TRAIN_WORLDS,
               train_teams=[f"{f}@{s}" for f, s in TRAIN_TEAMS], expert="mpc",
               classes=MPC_LEVELS.tolist())
    seed = 101
    model_path = os.path.join(MODELS, "policy_dagger.npz")
    for it in range(a.iters):
        beta = 1.0 if it == 0 else 0.5 ** it
        jobs = []
        for w in TRAIN_WORLDS:
            for form, sp in TRAIN_TEAMS:
                for _ in range(a.per_cell):
                    jobs.append(("policy", w, form, sp, seed,
                                 float(rng.choice([0.02, 0.10])), beta,
                                 model_path, "nn", "truth"))
                    seed += 1
        t0 = time.time()
        R = run_pool(jobs, a.jobs)
        n_new = 0
        for r in R:
            if len(r["y"]):
                D["scans"].append(r["scans"])
                D["aux"].append(r["aux"])
                D["y"].append(r["y"])
                D["slots"] += r["slots"]
                n_new += len(r["y"])
        reach = float(np.mean([r["ok"] for r in R]))
        Dc = dict(scans=np.concatenate(D["scans"]), aux=np.concatenate(D["aux"]),
                  y=np.concatenate(D["y"]), slots=D["slots"])
        model, hist = fit_policy(Dc, a.epochs, seed=it)
        export(model, model_path, dict(classes=MPC_LEVELS))
        from ros2_swarm_transport import neural
        neural._CACHE.clear()
        rec = dict(iteration=it, beta=beta, rollouts=len(R),
                   rollout_reach=reach, new_samples=n_new,
                   total_samples=int(len(Dc["y"])), val_acc=hist[-1]["val_acc"],
                   wall_s=round(time.time() - t0, 1))
        log["iterations"].append(rec)
        print(json.dumps(rec), flush=True)
        json.dump(log, open(os.path.join(MODELS, "policy_dagger.json"), "w"), indent=2)


def cmd_monitor(a):
    os.makedirs(MODELS, exist_ok=True)
    rng = np.random.default_rng(1)
    jobs = []
    seed = 501
    for w in TRAIN_WORLDS:
        for algo in ("mpc", "dwa", "pvfh_prob", "mppi"):
            for gs in ("audit", "sensor"):
                for _ in range(a.per_cell):
                    form, sp = TRAIN_TEAMS[int(rng.integers(len(TRAIN_TEAMS)))]
                    jobs.append(("monitor", w, form, sp, seed,
                                 float(rng.choice([0.02, 0.10])), 0.0, None, algo, gs))
                    seed += 1
    t0 = time.time()
    R = run_pool(jobs, a.jobs)
    # split by RUN (samples within a run are correlated): 60 % train,
    # 20 % calibration, 20 % in-distribution test
    order = np.random.default_rng(2).permutation(len(R))
    n = len(R)
    tr, ca, te = order[:int(.6 * n)], order[int(.6 * n):int(.8 * n)], order[int(.8 * n):]
    cat = lambda ids, k: np.concatenate([R[i][k] for i in ids if len(R[i][k])])  # noqa: E731
    Xtr, ctr = cat(tr, "X"), cat(tr, "c")
    model = fit_monitor(Xtr, ctr, a.epochs)
    Xca, cca = cat(ca, "X"), cat(ca, "c")
    res = predict_monitor(model, Xca) - np.clip(cca, -0.5, 3.0)
    qs = {f"q_{int(round(al * 1000)):03d}": conformal_q(res, al)
          for al in (0.01, 0.05, 0.10)}
    export(model, os.path.join(MODELS, "monitor_conformal.npz"), qs)
    Xte, cte = cat(te, "X"), cat(te, "c")
    pte = predict_monitor(model, Xte)
    rr = 0.18
    ev = {}
    for k, q in qs.items():
        adm = pte - q > rr
        unsafe = cte <= rr
        ev[k] = dict(q=q, admit_rate=float(adm.mean()),
                     false_accept_rate=float((adm & unsafe).mean()),
                     false_accept_given_unsafe=float((adm & unsafe).sum() / max(unsafe.sum(), 1)),
                     coverage=float((pte - q <= np.clip(cte, -0.5, 3.0)).mean()))
    log = dict(train_runs=len(tr), cal_runs=len(ca), test_runs=len(te),
               n_train=int(len(ctr)), n_cal=int(len(cca)), n_test=int(len(cte)),
               mae_test=float(np.mean(np.abs(pte - np.clip(cte, -0.5, 3.0)))),
               unsafe_frac_test=float((cte <= rr).mean()),
               in_distribution=ev, wall_s=round(time.time() - t0, 1))
    json.dump(log, open(os.path.join(MODELS, "monitor_conformal.json"), "w"), indent=2)
    print(json.dumps(log, indent=2))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["policy", "monitor", "occupancy"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--per-cell", type=int, default=2)
    ap.add_argument("--epochs", type=int, default=25)
    a = ap.parse_args()
    {"policy": cmd_policy, "monitor": cmd_monitor, "occupancy": cmd_occupancy}[a.what](a)
