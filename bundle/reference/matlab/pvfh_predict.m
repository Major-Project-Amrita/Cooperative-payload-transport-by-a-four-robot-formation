function [vec, info] = pvfh_predict(pos, world, movers, v_self, t_now, goal, margin, v, eps_cfg, prev_heading)
%PVFH_PREDICT  p-VFH on a scan that includes the movers' FUTURE poses.
%
%   [vec, info] = PVFH_PREDICT(pos, world, movers, v_self, t_now, goal, ...
%                              margin, v, eps_cfg, prev_heading)
%
%   Identical algorithm, knobs and fallbacks to PVFH_SCAN (with empty movers
%   it reduces to pvfh_scan exactly - the static sensing path is untouched).
%   The one dynamic extension:
%
%   PER-BEAM MOVER SCAN OVER THE HORIZON. The beams are raycast against the
%   static world exactly as PVFH_SCAN does, then against each mover at its
%   contracted poses t_now, t_now+dt, ..., t_now+T_lead (eval_mover_pose -
%   the same function the ROS 2 side mirrors). Each beam keeps the NEAREST
%   blocking instance over the whole horizon, so a mover that will close into
%   the beam fan shows up at the range it will have when it does. Statics do
%   not move, so their sensing needs no prediction; and because the sensor
%   position itself is never shifted, the controller's perception of the
%   STATIC world is byte-identical to the original, proven stack.
%
%   T_lead (v.pred_lead, seconds) is the horizon; dt is v.pred_dt (defaults
%   to 0.1, the sim step). With movers at <= 1 m/s and 0.5 s of horizon a
%   head-on mover is seen ~0.75 m closer than an instantaneous scan would
%   show it - early enough to turn a 2.5 m-wide rigid block.
%
%   v_self is accepted for interface compatibility and logging only; it is
%   NOT used to shift the sensor position (that variant couples the current
%   heading into the perception of static geometry and causes heading limit
%   cycles - see the design notes in ROS2_REPLICATION.md).
%
%   See also PVFH_SCAN, LIDAR_SCAN_DYN, EVAL_MOVER_POSE, SIMULATION8_DYNAMIC.

S           = v.num_sectors;
max_sense   = v.sense_range_base + margin;
danger_dist = v.danger_dist_base + margin;

if isfield(v, 'n_beams') && ~isempty(v.n_beams)
    nb = v.n_beams;
else
    nb = 4*S;
end
if ~isfield(v, 'circular_smooth'), v.circular_smooth = true; end
if ~isfield(v, 'pred_lead'),       v.pred_lead       = 0.5; end
if ~isfield(v, 'pred_dt'),         v.pred_dt         = 0.1; end

T_lead = v.pred_lead;

% ---- 1. sense: statics exactly as PVFH_SCAN ----
[ranges, angles] = lidar_scan(pos, world, nb, max_sense);

% ---- 1b. movers: per-beam scan over the prediction horizon ----
% Sample the horizon at dt resolution; at each sample the movers are exactly
% where the shared contract puts them. Each beam keeps the nearest blocking
% range across all samples (current pose included), so approach geometry
% enters the histogram before an instantaneous scan would see it.
if ~isempty(movers) && T_lead > 0
    wempty        = world;
    wempty.boxes   = world.boxes([]);    % 0x0 struct, same fields
    wempty.circles = world.circles([]);
    samples = unique([0:v.pred_dt:T_lead, T_lead]);
    for j = 1:numel(samples)
        mk = movers;
        for k = 1:numel(mk)
            mk(k).c = eval_mover_pose(movers(k), t_now + samples(j));
        end
        [rk, ~] = lidar_scan_dyn(pos, wempty, mk, nb, max_sense);
        ranges  = min(ranges, rk);
    end
end

% ---- 2. polar obstacle density (as PVFH_SCAN) ----
sector_angles = -pi + (2*pi)*(0:S-1)/S;
dsec    = 2*pi/S;
sec_idx = floor((angles + pi) / (2*pi) * S) + 1;
sec_idx = min(max(sec_idx, 1), S);

rmin = accumarray(sec_idx', ranges', [S 1], @min, inf)';

if ~isfield(v, 'enlarge'),    v.enlarge    = true; end
if ~isfield(v, 'auto_sigma'), v.auto_sigma = true; end
if ~isfield(v, 'react_gap'),  v.react_gap  = 1.0;  end

r_eff = 0;
if v.enlarge
    r_eff = margin;
end

sigma = v.sigma;
if v.auto_sigma
    d_react = r_eff + v.react_gap;
    sigma   = d_react / sqrt(2*log(1/v.threshold));
end

hist_vals = zeros(1, S);
occ = find(rmin < max_sense);
for si = occ
    d   = rmin(si);
    val = exp(-(d^2) / (2*sigma^2));
    if r_eff > 0
        gamma = asin(min(1, r_eff / max(d, r_eff)));
        k     = ceil(gamma / dsec);
        idxs  = mod((si-1-k : si-1+k), S) + 1;
    else
        idxs = si;
    end
    hist_vals(idxs) = max(hist_vals(idxs), val);
end

hist_vals = smooth_hist(hist_vals, v.smooth_window, v.circular_smooth);

% ---- 4. candidate valleys ----
candidate_idx = find(hist_vals < v.threshold);
if isempty(candidate_idx)
    [~, candidate_idx] = min(hist_vals);
end

% ---- 5. restrict to sectors near the goal bearing ----
goal_heading = atan2(goal(2) - pos(2), goal(1) - pos(1));
dev          = abs(wrap_pi(sector_angles(candidate_idx) - goal_heading));
feasible_idx = candidate_idx(dev <= v.turn_limit);
if isempty(feasible_idx)
    feasible_idx = candidate_idx;
end

% ---- 6. cost = goal alignment + heading hysteresis ----
sh      = sector_angles(feasible_idx);
costs   = abs(wrap_pi(sh - goal_heading)) + ...
          v.hysteresis_weight * abs(wrap_pi(sh - prev_heading));
[~, ci] = min(costs);
chosen_heading = sh(ci);

vec = [cos(chosen_heading), sin(chosen_heading)];

% ---- 3. close-range repulsion on the predicted scan ----
close_b = ranges < danger_dist;
if any(close_b)
    r  = ranges(close_b);
    aB = angles(close_b);
    w  = 1 ./ (r.^v.avoid_falloff_power + eps_cfg);
    dth = 2*pi/nb;
    vx  = -sum(w .* cos(aB)) * dth;
    vy  = -sum(w .* sin(aB)) * dth;
    vec = vec + v.avoid_gain * [vx, vy];
end

nv = norm(vec);
if nv < eps_cfg
    vec = [cos(prev_heading), sin(prev_heading)];
else
    vec = vec / nv;
end

if nargout > 1
    info = struct('sigma', sigma, 'ranges', ranges, 'angles', angles, ...
                  'hist', hist_vals, 'chosen_heading', chosen_heading, ...
                  'min_range', min(ranges), 'T_lead', T_lead);
end
end

% ------------------------------------------------------------------------
function y = smooth_hist(x, w, circular)
w = max(1, round(w));
if w == 1, y = x; return; end
n    = numel(x);
half = floor(w/2);
k    = ones(1, 2*half+1) / (2*half+1);
if circular
    pad  = [x(end-half+1:end), x, x(1:half)];
    full = conv(pad, k, 'same');
    y    = full(half+1 : half+n);
else
    y = zeros(1, n);
    for i = 1:n
        lo = max(1, i-half); hi = min(n, i+half);
        y(i) = mean(x(lo:hi));
    end
end
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
