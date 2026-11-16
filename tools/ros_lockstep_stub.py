#!/usr/bin/env python3
"""Lockstep check of the two ROS 2 nodes (mujoco_plant, formation_controller)
WITHOUT a ROS 2 installation.

    python3 tools/ros_lockstep_stub.py                 # mpc, nn, pavfh
    python3 tools/ros_lockstep_stub.py pavfh --gate-source learned --phase

What it does: it installs a minimal in-process stand-in for rclpy and the
message packages (topics are an in-memory FIFO queue), instantiates the
REAL node classes from the package, lets them exchange /swarm/state and
/swarm/command tick by tick, and compares the controller node's summary with
the in-process run of the same mission (mujoco_validation.run).

What it checks: the node code paths, parameter handling, message packing
(pack_state / unpack_state / the command layout), the lockstep protocol, and
that the learned components (NN policy, PA-VFH occupancy network, learned
monitor) load and run inside the nodes.

What it does NOT check: DDS transport, QoS behaviour, executors, launch
files or colcon installation. Those need a real ROS 2 install:
`bash tools/ros_lockstep_test.sh <algo> <payload>`.

Output: results/mujoco_ros_stub/<algo>_*.json and a comparison table;
exit status 0 only if every compared metric is identical.
"""
import argparse
import collections
import json
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "ros2_swarm_transport")
sys.path.insert(0, PKG)

PARAMS = {}                      # parameter overrides, like --ros-args -p
BUS = collections.defaultdict(list)
QUEUE = collections.deque()
COUNT = collections.Counter()


# --------------------------------------------------------------- stand-in
class Msg:
    """Message whose nested fields are created on first access."""

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        v = Msg()
        object.__setattr__(self, k, v)
        return v


class MarkerArray(Msg):
    def __init__(self):
        self.markers = []


class Marker(Msg):
    CYLINDER, CUBE = 3, 1


class Float64MultiArray(Msg):
    def __init__(self):
        self.data = []


class Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, m):
        COUNT[self.topic] += 1
        if BUS[self.topic]:
            QUEUE.append((self.topic, m))

    def get_subscription_count(self):
        return len(BUS[self.topic])


class Param:
    def __init__(self, v):
        self.value = v


class Logger:
    def __init__(self, name):
        self.name = name

    def info(self, s):
        print(f"  [{self.name}] {s}")

    warn = warning = error = info


class Timer:
    def __init__(self, period, cb):
        self.cb, self.cancelled = cb, False

    def cancel(self):
        self.cancelled = True


class Node:
    def __init__(self, name):
        self._name, self._p, self.timers = name, {}, []

    def declare_parameter(self, n, default=None):
        self._p[n] = PARAMS.get(n, default)

    def get_parameter(self, n):
        return Param(self._p[n])

    def create_publisher(self, typ, topic, qos):
        return Pub(topic)

    def create_subscription(self, typ, topic, cb, qos):
        BUS[topic].append(cb)

    def create_timer(self, period, cb):
        t = Timer(period, cb)
        self.timers.append(t)
        return t

    def get_logger(self):
        return Logger(self._name)

    def destroy_node(self):
        pass


def install_stub():
    def mod(name, **kw):
        m = types.ModuleType(name)
        m.__dict__.update(kw)
        sys.modules[name] = m
        return m

    class _E:
        RELIABLE = BEST_EFFORT = KEEP_LAST = 0
    mod("rclpy", init=lambda *a, **k: None, ok=lambda: False,
        shutdown=lambda: None, spin=lambda n: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=lambda **k: None, ReliabilityPolicy=_E, HistoryPolicy=_E)
    for pkg, names in (("geometry_msgs", ["Twist", "PoseStamped"]),
                       ("nav_msgs", ["Odometry"]), ("std_msgs", ["Int32"]),
                       ("rosgraph_msgs", ["Clock"]), ("visualization_msgs", []),
                       ("sensor_msgs", ["LaserScan"])):
        mod(pkg)
        m = mod(pkg + ".msg", **{n: type(n, (Msg,), {}) for n in names})
        if pkg == "std_msgs":
            m.Float64MultiArray = Float64MultiArray
        if pkg == "visualization_msgs":
            m.Marker, m.MarkerArray = Marker, MarkerArray


# --------------------------------------------------------------- run
KEYS = ["reached_goal", "total_time", "n_reject", "coll_mover", "formation_rms",
        "formation_err_max", "contact_steps_static", "contact_steps_mover", "pin_gap_max"]


def lockstep(algo, payload, seed, phase, gate_source, out):
    from ros2_swarm_transport.formation_controller import FormationController
    from ros2_swarm_transport.mujoco_plant import MujocoPlantNode
    BUS.clear(), QUEUE.clear(), COUNT.clear(), PARAMS.clear()
    name = f"{algo}_square_{payload}_s{seed}" + ("_ph1" if phase else "") + \
        ("" if gate_source == "truth" else f"_{gate_source}")
    PARAMS.update(algo=algo, payload=payload, seed=seed, traffic_phase=phase,
                  gate_source=gate_source, output_dir=out, run_name="ros_" + name, gui=False)
    plant = MujocoPlantNode()
    ctrl = FormationController()
    plant.timers[0].cb()                       # the plant's kick timer: tick 0
    done = 0
    while QUEUE and done < 2:
        topic, m = QUEUE.popleft()
        for cb in list(BUS[topic]):
            try:
                cb(m)
            except SystemExit:                 # a node finished its mission
                done += 1
    from ros2_swarm_transport.mujoco_validation import run
    run(algo=algo, payload=payload, seed=seed, out_dir=os.path.join(out, "direct"),
        name=name, verbose=False, phase=phase, gate_source=gate_source)
    a = json.load(open(os.path.join(out, f"ros_{name}.json")))
    b = json.load(open(os.path.join(out, "direct", f"{name}.json")))
    rows = [(k, a[k], b[k], a[k] == b[k]) for k in KEYS]
    return name, rows, dict(COUNT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("algos", nargs="*", default=["mpc", "nn", "pavfh"])
    ap.add_argument("--payload", default="on_top")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--phase", action="store_true")
    ap.add_argument("--gate-source", default="truth")
    a = ap.parse_args()
    install_stub()
    out = os.path.join(ROOT, "results", "mujoco_ros_stub")
    os.makedirs(out, exist_ok=True)
    ok, report = True, []
    for algo in a.algos:
        name, rows, cnt = lockstep(algo, a.payload, a.seed, a.phase, a.gate_source, out)
        same = all(r[3] for r in rows)
        ok &= same
        print(f"\n{name}: {'IDENTICAL' if same else 'DIFFERENT'} "
              f"({cnt.get('/swarm/state', 0)} state / {cnt.get('/swarm/command', 0)} command messages)")
        print(f"  {'metric':22s} {'ROS 2 nodes':>14s} {'in-process':>14s}")
        for k, x, y, s in rows:
            f = (lambda v: f"{v:.6g}" if isinstance(v, float) else str(v))
            print(f"  {k:22s} {f(x):>14s} {f(y):>14s} {'' if s else '  <-- differs'}")
        report.append(dict(run=name, identical=same, state_msgs=cnt.get("/swarm/state", 0),
                           command_msgs=cnt.get("/swarm/command", 0),
                           **{k: x for k, x, _, _ in rows}))
    with open(os.path.join(out, "summary.json"), "w") as fh:
        json.dump(report, fh, indent=1)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
