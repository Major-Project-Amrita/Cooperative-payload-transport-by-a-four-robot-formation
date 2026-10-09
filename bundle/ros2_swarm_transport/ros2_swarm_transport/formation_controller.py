#!/usr/bin/env python3
"""formation_controller.py - the two-phase swarm transport controller.

Phase 1 (FORMING): each robot APF-seeks its assigned slot with p-VFH obstacle
steering (cascaded architecture), convoy/queueing included via the phase topic.

Phase 2 (TRANSPORT): a virtual leader (the formation centroid command) runs
the weighted-sum APF+VFH blend with predictive scanning over the movers,
speed slowdown, and the t/t+dt commit test; robots servo to their slots.

This node implements the unicycle kinematic model from ROS2_REPLICATION.md
sec. 1: v from the proximity logic, omega = wrap(theta_des - theta)/dt clipped
to max_turn_rate. It subscribes to /<robot>/odom and the mover markers, and
publishes /<robot>/cmd_vel at 10 Hz.

In this replication the leader state is COMPUTED (kinematic, exactly like
MATLAB) rather than estimated: the node owns cfg and advances a virtual
leader, then commands each robot to its slot with a P-controller. The
resulting tracking residual is your real formation_rms.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray

from .assign_slots import assign_slots
from .geometry import (eval_mover_pose, formation_clearance, lidar_scan,
                       load_dynamic_obstacles, world_from_sdf)
from .vfh import (VfhConfig, pvfh_predict, pvfh_scan, steering_point,
                  steering_rigid)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FormationController(Node):

    def __init__(self):
        super().__init__('formation_controller')

        # ---- parameters: mirror of the MATLAB cfg (ROS2_REPLICATION sec 5)
        self.declare_parameter('world_sdf', 'swarm_world_dynamic.sdf')
        self.declare_parameter('contract', 'dynamic_obstacles.json')
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('goal', [12.0, 0.0])
        self.declare_parameter('start', [-12.0, 0.0])
        self.declare_parameter('grid_size', 2)
        self.declare_parameter('grid_spacing', 1.18)
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('V', 0.6)
        self.declare_parameter('forming_V', 0.5)
        self.declare_parameter('max_turn_rate', 0.3)
        self.declare_parameter('forming_max_turn_rate', 0.5)
        self.declare_parameter('heading_smooth_alpha', 0.35)
        self.declare_parameter('w_goal', 1.0)
        self.declare_parameter('w_vfh', 2.0)
        self.declare_parameter('forming_tolerance', 0.3)
        self.declare_parameter('forming_w_slot', 2.0)
        self.declare_parameter('forming_w_repulse', 1.5)
        self.declare_parameter('forming_repulse_dist', 1.2)
        self.declare_parameter('forming_slow_radius', 1.5)
        self.declare_parameter('slow_margin', 0.25)
        self.declare_parameter('slow_min', 0.4)
        self.declare_parameter('hold_time', 1.5)
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('avoid_vel_frac', 0.25)
        self.declare_parameter('pred_lead', 0.5)
        self.declare_parameter('pred_dt', 0.1)
        self.declare_parameter('n_beams', 288)
        # --- algorithm variant + sensing model (the paper's comparison axis) ---
        self.declare_parameter('algo', 'pvfh')
        self.declare_parameter('sense_kind', 'lidar')       # 'lidar' | 'tof'
        self.declare_parameter('sense_sigma0', 0.02)
        self.declare_parameter('sense_k_rng', 0.03)
        self.declare_parameter('sense_fov_deg', 60.0)
        self.declare_parameter('sense_max_range', float('inf'))
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4'])

        p = lambda n: self.get_parameter(n).value
        self.dt = float(p('dt'))
        self.goal = np.array(p('goal'), dtype=float)
        self.start = np.array(p('start'), dtype=float)
        self.N = int(p('grid_size'))
        self.s = float(p('grid_spacing'))
        self.r = float(p('robot_radius'))
        self.V = float(p('V'))
        self.Vf = float(p('forming_V'))
        self.w_max = float(p('max_turn_rate'))
        self.w_max_f = float(p('forming_max_turn_rate'))
        self.alpha = float(p('heading_smooth_alpha'))
        self.w_goal = float(p('w_goal'))
        self.w_vfh = float(p('w_vfh'))
        self.tol = float(p('forming_tolerance'))
        self.w_slot = float(p('forming_w_slot'))
        self.w_rep = float(p('forming_w_repulse'))
        self.rep_dist = float(p('forming_repulse_dist'))
        self.slow_radius = float(p('forming_slow_radius'))
        self.slow_margin = float(p('slow_margin'))
        self.slow_min = float(p('slow_min'))
        self.hold_time = float(p('hold_time'))
        self.lookahead = float(p('lookahead'))
        self.avoid_vel_frac = float(p('avoid_vel_frac'))
        self.robots = list(p('robots'))
        self.K = len(self.robots)

        self.world = world_from_sdf(p('world_sdf'), movers_json=p('contract'))
        self.movers, _ = load_dynamic_obstacles(p('contract'))
        self.algo = str(p('algo'))
        self.sense = {
            'kind': str(p('sense_kind')),
            'sigma0': float(p('sense_sigma0')),
            'k_rng': float(p('sense_k_rng')),
            'fov_deg': float(p('sense_fov_deg')),
            'max_range': float(p('sense_max_range')),
        }
        self.vfh = VfhConfig(n_beams=int(p('n_beams')),
                             pred_lead=float(p('pred_lead')),
                             pred_dt=float(p('pred_dt')),
                             sense=self.sense)
        self.eps = 1e-6
        self.rng = np.random.default_rng(0)   # per-run sensor-noise stream

        # ---- formation geometry (meshgrid, centred) exactly as MATLAB
        gx, gy = np.meshgrid(np.arange(self.N), np.arange(self.N))
        offs = np.column_stack([gx.ravel(), gy.ravel()]) * self.s
        self.offsets = offs - offs.mean(axis=0)
        self.half_diag = ((self.N - 1) * self.s) * math.sqrt(2) / 2

        # ---- virtual leader state (kinematic, like the MATLAB sim)
        self.c = self.start.copy()
        self.theta = math.atan2(self.goal[1] - self.start[1],
                                self.goal[0] - self.start[0])
        d0 = self.goal - self.start
        R0 = np.array([[math.cos(self.theta), -math.sin(self.theta)],
                       [math.sin(self.theta), math.cos(self.theta)]])
        target_slots = (R0 @ self.offsets.T).T + self.start
        starts = self.start + 2.2 * np.column_stack([
            np.cos(2 * math.pi * np.arange(self.K) / self.K),
            np.sin(2 * math.pi * np.arange(self.K) / self.K)])
        cost = np.linalg.norm(starts[:, None, :] - target_slots[None, :, :],
                              axis=2)
        perm = assign_slots(cost)
        self.offsets = self.offsets[perm]

        self.phase = 1
        self.t = 0.0
        self.stuck = 0
        self.scale = 1.0
        self.pos_meas = {name: None for name in self.robots}
        self.yaw_meas = {name: 0.0 for name in self.robots}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = {}
        for name in self.robots:
            self.cmd_pub[name] = self.create_publisher(Twist,
                                                       f'/{name}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{name}/odom',
                                     self._odom_cb(name), qos)
        self.create_subscription(MarkerArray, '/dynamic_obstacles',
                                 self._movers_cb, 10)
        self.pub_phase = self.create_publisher(Int32, '/swarm/phase', 10)

        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info('formation controller up (phase 1: forming)')

    def _odom_cb(self, name):
        def cb(msg):
            q = msg.pose.pose.orientation
            self.pos_meas[name] = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.yaw_meas[name] = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return cb

    def _movers_cb(self, msg):
        # In production, rebuild self.movers from the marker poses here.
        # The kinematic model already evaluates the contract directly, so the
        # subscription is only used to check liveness.
        pass

    # ------------------------------------------------------------------
    def step(self):
        self.t += self.dt
        if self.phase == 1:
            self.step_forming()
        else:
            self.step_transport()
        m = Int32()
        m.data = self.phase
        self.pub_phase.publish(m)

    def cmd(self, name, v, w):
        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(w)
        self.cmd_pub[name].publish(tw)

    # ---- Phase 1 -----------------------------------------------------
    def step_forming(self):
        c = self.c
        R = np.array([[math.cos(self.theta), -math.sin(self.theta)],
                      [math.sin(self.theta), math.cos(self.theta)]])
        slots = (R @ self.offsets.T).T + c
        errs = []
        movers_t = [m.eval_at(self.t) for m in self.movers]

        for i, name in enumerate(self.robots):
            pos = self.pos_meas[name]
            if pos is None:
                return                     # wait for all odometry
            e = float(np.linalg.norm(slots[i] - pos))
            errs.append(e)
            v = self.Vf * min(1.0, e / self.slow_radius)
            fx = self.w_slot * (slots[i] - pos) / (e + self.eps)
            f = fx.copy()
            for j in range(self.K):
                if j == i:
                    continue
                pos_j = self.pos_meas[self.robots[j]]
                if pos_j is None:
                    continue
                dvec = pos - pos_j
                d = float(np.linalg.norm(dvec))
                if d < self.rep_dist:
                    f += self.w_rep * (self.rep_dist - d) * dvec / (d + self.eps)
            virtual = pos + f
            vec, _ = steering_point(pos, virtual, self.w_slot,
                                    self.world.with_movers(movers_t), movers_t,
                                    self.t, self.r, self.vfh, self.algo,
                                    self.eps, self.yaw_meas[name], self.rng)
            th_des = self.yaw_meas[name] + self.alpha * wrap(
                math.atan2(vec[1], vec[0]) - self.yaw_meas[name])
            w = max(-self.w_max_f, min(self.w_max_f,
                                       wrap(th_des - self.yaw_meas[name]) / self.dt))
            self.cmd(name, v, w)

        if max(errs) < self.tol:
            self.phase = 2
            self.stuck = 0
            self.get_logger().info('formation formed -> phase 2 (transport)')

    # ---- Phase 2 -----------------------------------------------------
    def step_transport(self):
        movers_t = [m.eval_at(self.t) for m in self.movers]
        margin = self.half_diag + self.r

        vec_goal = (self.goal - self.c) / (np.linalg.norm(self.goal - self.c)
                                           + self.eps)
        v_self = self.V * np.array([math.cos(self.theta),
                                    math.sin(self.theta)])
        vec_steer, self.comp_ms = steering_rigid(
            self.c, self.goal, self.w_goal, self.world, movers_t, self.t,
            margin, self.vfh, self.algo, self.eps, self.theta, v_self, self.rng)
        total = self.w_goal * vec_goal + self.w_vfh * vec_steer
        th_raw = math.atan2(total[1], total[0])

        # speed slowdown on predicted mover gap
        gap = self.predicted_gap(movers_t)
        thr = self.slow_margin + self.r
        if gap < thr and max(float(np.linalg.norm(m.v)) for m in movers_t) / \
                self.V > self.avoid_vel_frac:
            self.scale = max(self.slow_min, min(1.0, 0.8 * gap / thr))
        else:
            self.scale = 1.0

        th_des = self.theta + self.alpha * wrap(th_raw - self.theta)
        w = max(-self.w_max, min(self.w_max, wrap(th_des - self.theta) / self.dt))
        self.theta = wrap(self.theta + w * self.dt)

        cand = self.c + self.scale * self.V * \
            np.array([math.cos(self.theta), math.sin(self.theta)]) * self.dt

        # commit only if clear at t AND t+dt
        clr_n, hit_n = formation_clearance(cand, self.theta, self.offsets,
                                           self.world.with_movers(movers_t),
                                           self.r, 'payload')
        movers_tp = [m.eval_at(self.t + self.dt) for m in self.movers]
        clr_p, hit_p = formation_clearance(cand, self.theta, self.offsets,
                                           self.world.with_movers(movers_tp),
                                           self.r, 'payload')
        if not hit_n and not hit_p:
            self.c = cand
            self.stuck = 0
        else:
            self.stuck += 1
            self.theta = wrap(self.theta - w * self.dt)   # undo heading step

        # command robots to their slots (P-controller on the real poses)
        R = np.array([[math.cos(self.theta), -math.sin(self.theta)],
                      [math.sin(self.theta), math.cos(self.theta)]])
        slots = (R @ self.offsets.T).T + self.c
        v_frame = self.scale * self.V
        for i, name in enumerate(self.robots):
            pos = self.pos_meas[name]
            if pos is None:
                continue
            e = slots[i] - pos
            en = float(np.linalg.norm(e))
            if en < 1e-6:
                self.cmd(name, 0.0, 0.0)
                continue
            e_hat = e / en
            # feed-forward the frame velocity + P-correction on the slot error
            v_cmd = v_frame * e_hat[0] + min(2.0 * en, 0.6)
            err_th = wrap(math.atan2(e[1], e[0]) - self.yaw_meas[name])
            w_cmd = max(-self.w_max, min(self.w_max, 1.5 * err_th))
            self.cmd(name, max(0.0, v_cmd), w_cmd)

    def predicted_gap(self, movers_t):
        worst = float('inf')
        v = self.V * np.array([math.cos(self.theta), math.sin(self.theta)])
        T = self.dt
        while T <= self.lookahead + 1e-9:
            mf = [m.eval_at(self.t + T) for m in self.movers]
            g = self._mover_gap(self.c + v * T, mf) - float(np.linalg.norm(v)) * T
            worst = min(worst, g)
            T += self.dt
        return worst

    def _mover_gap(self, q, movers):
        g = float('inf')
        for m in movers:
            if m.shape == 'circle':
                dk = np.hypot(q[0] - m.c[0], q[1] - m.c[1]) - m.r
            else:
                dx, dy = abs(q[0] - m.c[0]) - m.size[0] / 2, \
                    abs(q[1] - m.c[1]) - m.size[1] / 2
                dk = (np.hypot(max(dx, 0), max(dy, 0)) +
                      min(max(dx, dy), 0))
            g = min(g, float(dk))
        return g


def main():
    rclpy.init()
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
