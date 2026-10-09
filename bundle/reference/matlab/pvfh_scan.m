function [vec, info] = pvfh_scan(pos, world, goal, margin, v, eps_cfg, prev_heading)
%PVFH_SCAN  Polar VFH steering vector computed from a synthetic laser scan.
%
%   [vec, info] = PVFH_SCAN(pos, world, goal, margin, vfh_cfg, eps, prev_heading)
%
%   Same algorithm and same tuning knobs as the original pVFH_body, but the
%   polar histogram is built from LIDAR_SCAN ranges instead of a point cloud
%   sampled over obstacle interiors. One sample per bearing, nearest surface
%   only, occlusion respected - i.e. exactly the information a ROS 2 node gets
%   from /scan.
%
%   vfh_cfg fields: num_sectors, sigma, sense_range_base, danger_dist_base,
%                   turn_limit, threshold, smooth_window, avoid_gain,
%                   hysteresis_weight, avoid_falloff_power, n_beams,
%                   circular_smooth
%
%   info.ranges / .angles / .hist / .chosen_heading / .min_range for logging
%   and debugging.
%
%   TWO DELIBERATE DEVIATIONS from the original, both toward correctness:
%     1. Sector angles use an OPEN interval. linspace(-pi,pi,S) puts a sector at
%        -pi and another at +pi, which are the same bearing, so the original had
%        one duplicated and slightly narrower sector. Fixed here.
%     2. Histogram smoothing wraps around (circular_smooth, default true).
%        movmean treats the histogram as a line segment, so the original had a
%        seam directly behind the robot where the smoothing window was truncated.
%
%   Both are small, but they are exactly the kind of thing that shows up as an
%   unexplained MATLAB-vs-ROS2 divergence during a hard turn. Set
%   vfh_cfg.circular_smooth = false to reproduce the old behaviour.
%
%   See also LIDAR_SCAN, SDF2WORLD.

S           = v.num_sectors;
max_sense   = v.sense_range_base + margin;
danger_dist = v.danger_dist_base + margin;

if isfield(v, 'n_beams') && ~isempty(v.n_beams)
    nb = v.n_beams;
else
    nb = 4*S;      % oversample so several beams land in each sector
end
if ~isfield(v, 'circular_smooth'), v.circular_smooth = true; end

% ---- 1. sense ----
[ranges, angles] = lidar_scan(pos, world, nb, max_sense);

% ---- 2. polar obstacle density, with the body's width folded in ----
sector_angles = -pi + (2*pi)*(0:S-1)/S;          % open interval, no duplicate
dsec    = 2*pi/S;
sec_idx = floor((angles + pi) / (2*pi) * S) + 1;
sec_idx = min(max(sec_idx, 1), S);

% Nearest surface per sector. min, not sum, because what decides whether a
% bearing is safe is the CLOSEST thing along it.
rmin = accumarray(sec_idx', ranges', [S 1], @min, inf)';

if ~isfield(v, 'enlarge'),    v.enlarge    = true; end
if ~isfield(v, 'auto_sigma'), v.auto_sigma = true; end
if ~isfield(v, 'react_gap'),  v.react_gap  = 1.0;  end

r_eff = 0;
if v.enlarge
    r_eff = margin;    % body circumradius + robot radius, passed in by the caller
end

% Reaction distance is where the Gaussian influence crosses the threshold:
%   exp(-d^2 / 2 sigma^2) = threshold   ->   d = sigma * sqrt(2 ln(1/threshold))
% A fixed sigma therefore hard-codes a fixed reaction distance, which is wrong
% the moment the body is not a point. sigma = 1.0 reacts at 1.55 m - fine for a
% single robot in an open 50x50 field, far too late for a 2.5 m rigid block that
% has to start its turn before the obstacle is inside its own footprint.
% Solving the other way round keeps the reaction distance a fixed margin beyond
% whatever the body actually needs.
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
        % VFH+ enlarged obstacle: a hit at range d blocks every bearing within
        % asin(r_eff/d) of it, because a body of radius r_eff steered down any
        % of those bearings would still clip it.
        gamma = asin(min(1, r_eff / max(d, r_eff)));
        k     = ceil(gamma / dsec);
        idxs  = mod((si-1-k : si-1+k), S) + 1;
    else
        idxs = si;
    end
    hist_vals(idxs) = max(hist_vals(idxs), val);
end

hist_vals = smooth_hist(hist_vals, v.smooth_window, v.circular_smooth);

% ---- 3. candidate valleys ----
candidate_idx = find(hist_vals < v.threshold);
if isempty(candidate_idx)
    [~, candidate_idx] = min(hist_vals);
end

% ---- 4. restrict to sectors near the goal bearing ----
goal_heading = atan2(goal(2) - pos(2), goal(1) - pos(1));
dev          = abs(wrap_pi(sector_angles(candidate_idx) - goal_heading));
feasible_idx = candidate_idx(dev <= v.turn_limit);
if isempty(feasible_idx)
    feasible_idx = candidate_idx;
end

% ---- 5. cost = goal alignment + heading hysteresis ----
sh       = sector_angles(feasible_idx);
costs    = abs(wrap_pi(sh - goal_heading)) + ...
           v.hysteresis_weight * abs(wrap_pi(sh - prev_heading));
[~, ci]  = min(costs);
chosen_heading = sh(ci);

vec = [cos(chosen_heading), sin(chosen_heading)];

% ---- 6. close-range repulsion, straight off the scan ----
close_b = ranges < danger_dist;
if any(close_b)
    r  = ranges(close_b);
    aB = angles(close_b);
    w  = 1 ./ (r.^v.avoid_falloff_power + eps_cfg);
    % Push directly away from each hit: direction is the beam reversed.
    % Weighting by the angular width of a beam (2*pi/nb) makes this an
    % integral over bearing, so the magnitude does not change when you
    % re-raycast at a different resolution. Summing raw (or dividing by nb)
    % would silently rescale repulsion against the unit steering vector and
    % break the avoid_gain tuning.
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
    info = struct('sigma', sigma, 'ranges', ranges, 'angles', angles, 'hist', hist_vals, ...
                  'chosen_heading', chosen_heading, 'min_range', min(ranges));
end
end

% ------------------------------------------------------------------------
function y = smooth_hist(x, w, circular)
%SMOOTH_HIST  Moving average over the polar histogram.
%   Self-contained (no movmean / Statistics Toolbox dependency). When
%   circular is true the window wraps around -pi/+pi, which is the correct
%   topology for a polar histogram.
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
