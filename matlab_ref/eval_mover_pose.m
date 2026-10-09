function p = eval_mover_pose(m, t)
%EVAL_MOVER_POSE  Centre of mover m at absolute time t (single source of truth).
%
%   p = EVAL_MOVER_POSE(m, t)
%
%   The ONE implementation of the shared motion contract's kinematics. Every
%   consumer (load_dynamic_obstacles, update_world_dyn, pvfh_predict,
%   find_escape_heading_dyn, Simulation8_dynamic's helpers) calls this, so
%   the two stacks can never drift apart in where they think a mover will
%   be. The ROS 2 mover node (mover_node.py in ros2_swarm_transport)
%   reimplements exactly this function in Python - keep the two in sync.
%
%   Motion model (piecewise-constant velocity):
%     t < t0         parked at route(1,:)
%     loop = false   translates route(1,:) -> route(2,:) at 'speed', stops
%     loop = true    each leg takes T = L/speed + wait: motion first, then
%                    the pause at the reached endpoint; direction alternates.
%                    Position is continuous everywhere.
%
%   See also UPDATE_WORLD_DYN, LOAD_DYNAMIC_OBSTACLES, DYNAMIC_OBSTACLES.JSON.

p = m.route(1,:);
if t < m.t0, return; end
L = norm(m.route(2,:) - m.route(1,:));
if L <= 0 || m.speed <= 0, return; end
tau = t - m.t0;
if m.loop
    T        = L / m.speed + m.wait;
    leg      = floor(tau / T) + 1;          % 1 = A->B, 2 = B->A, ...
    t_in_leg = mod(tau, T);                 % 0..T within this leg
    frac     = min(t_in_leg * m.speed / L, 1);
    if mod(leg, 2) == 1, s = frac; else, s = 1 - frac; end
    p = m.route(1,:) + s * (m.route(2,:) - m.route(1,:));
else
    s = min(tau * m.speed / L, 1);
    p = m.route(1,:) + s * (m.route(2,:) - m.route(1,:));
end
end
