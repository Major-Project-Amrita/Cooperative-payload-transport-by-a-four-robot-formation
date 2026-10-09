function [world, movers] = update_world_dyn(world, movers, t)
%UPDATE_WORLD_DYN  Advance every dynamic obstacle to time t (kinematic model).
%
%   [world, movers] = UPDATE_WORLD_DYN(world, movers, t)
%
%   Evaluates the shared motion contract (dynamic_obstacles.json, loaded by
%   LOAD_DYNAMIC_OBSTACLES) at time t and writes the resulting poses into
%   world.dyn_boxes / world.dyn_circles. Merge them onto the static geometry
%   with MERGE_DYN and every existing helper (world_dist, lidar_scan,
%   pvfh_scan, formation_clearance, ...) keeps working unchanged.
%
%   The returned movers gain two fields: .v (exact velocity, the derivative of
%   the piecewise-constant motion model) and .t (the evaluation time).
%
%   See also LOAD_DYNAMIC_OBSTACLES, MERGE_DYN, SIMULATION8_DYNAMIC.

n    = numel(movers);
tmpl = struct('name', '', 'shape', '', 'size', [0 0], 'r', 0, ...
              'route', [0 0; 0 0], 'speed', 0, 't0', 0, 'wait', 0, ...
              'loop', false, 'c', [0 0], 'v', [0 0], 't', 0);
out  = repmat(tmpl, 1, n);

for k = 1:n
    m   = movers(k);
    c_t = eval_mover_pose(m, t);
    v_t = eval_mover_vel(m, t);
    % Explicit field-by-field construction keeps the struct array homogeneous:
    % m gains .v/.t here, so in-place assignment into the caller's array
    % (which lacks those fields) would error.
    out(k) = struct('name', m.name, 'shape', m.shape, 'size', m.size, ...
                    'r', m.r, 'route', m.route, 'speed', m.speed, ...
                    't0', m.t0, 'wait', m.wait, 'loop', m.loop, ...
                    'c', c_t, 'v', v_t, 't', t);
end
movers = out;

% ---- write into the world struct, mirroring the static fields ----
world.dyn_boxes   = struct('c', {}, 'yaw', {}, 'h', {}, 'name', {}, ...
                           'zlo', {}, 'zhi', {});
world.dyn_circles = struct('c', {}, 'r', {}, 'name', {}, 'zlo', {}, 'zhi', {});

for k = 1:n
    m = out(k);
    if strcmpi(m.shape, 'circle')
        world.dyn_circles(end+1) = struct('c', m.c, 'r', m.r, ...
            'name', m.name, 'zlo', 0.0, 'zhi', m.size(2)); %#ok<AGROW>
    else
        world.dyn_boxes(end+1) = struct('c', m.c, 'yaw', 0.0, ...
            'h', m.size/2, 'name', m.name, 'zlo', 0.0, 'zhi', m.size(2)); %#ok<AGROW>
    end
end
end

% ------------------------------------------------------------------------
function v = eval_mover_vel(m, t)
% Exact velocity of the motion model at time t (piecewise constant). During a
% pause (waiting for t0, or the wait window at a loop endpoint) velocity is 0.
v = [0 0];
if t < m.t0, return; end
L = norm(m.route(2,:) - m.route(1,:));
if L <= 0 || m.speed <= 0, return; end
tau = t - m.t0;
if m.loop
    T        = L / m.speed + m.wait;
    leg      = floor(tau / T) + 1;
    t_in_leg = mod(tau, T);
    if t_in_leg * m.speed >= L, return; end          % paused at endpoint
    dirn = 1;
    if mod(leg, 2) == 0, dirn = -1; end
    v = dirn * m.speed * (m.route(2,:) - m.route(1,:)) / L;
else
    if tau * m.speed >= L, return; end               % arrived, stopped
    v = m.speed * (m.route(2,:) - m.route(1,:)) / L;
end
end
