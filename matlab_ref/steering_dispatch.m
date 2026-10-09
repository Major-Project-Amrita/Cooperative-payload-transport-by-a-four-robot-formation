function [vec, comp_ms] = steering_dispatch(mode, q, goal, w_att, world, ...
                                            movers, t_now, margin, cfg, ...
                                            prev_heading, v_self)
%STEERING_DISPATCH  Algorithm dispatch shared by both mission phases.
%
%   [vec, comp_ms] = STEERING_DISPATCH('rigid', q, goal, w_att, world, ...
%       movers, t_now, margin, cfg, prev_heading, v_self)
%   [vec, comp_ms] = STEERING_DISPATCH('point', q, goal, w_att, world, ...
%       movers, t_now, robot_radius, cfg, prev_heading)
%
%   'rigid' -- Phase-2 transport leader: the steering vector is blended with
%       the caller's goal attraction (w_goal*vec_goal + w_vfh*vec) OUTSIDE.
%   'point' -- Phase-1 forming robot: the caller's virtual goal IS the
%       attraction target for the pure-APF baseline (cascaded architecture),
%       so the attraction is added here; every other variant steers at the
%       virtual goal like any other goal.
%
%   EVERY variant consumes a scan produced by the SAME front end
%   (SCAN_RANGES_DYN + SENSOR_NOISE, LOCAL_SENSE below): same beam fan, same
%   noise model, same seeded stream. They differ only in the steering law
%   and in whether the scan carries the mover prediction horizon:
%
%     algo         steering law                         mover horizon
%     'apf'        APF obstacle reflex (no histogram)   none (instantaneous)
%     'vfh'        deterministic VFH+ histogram         none (instantaneous)
%     'fgm'        Follow-the-Gap (Sezer & Gokasan)     none (instantaneous)
%     'dwa'        Dynamic-Window arc sampling (Fox)    none (instantaneous)
%     'mpc'        receding-horizon lattice MPC         none (instantaneous)
%     'pvfh'       deterministic VFH+ histogram         T_lead
%     'pvfh_filt'  pvfh + recursive range filter        T_lead   (ablation)
%     'pvfh_prob'  P-VFH: range filter + expected kernel T_lead  (proposal)
%
%   So vfh -> pvfh isolates the prediction horizon and pvfh -> pvfh_prob
%   isolates the probabilistic occupancy: a clean two-step ablation ladder.
%
%   AUDIT NOTE (2026-10): three defects of the previous dispatch are fixed
%   here. (1) 'pvfh' called PVFH_PREDICT, which raycasts a NOISE-FREE
%   full-circle lidar -- it ignored the sensor model altogether, so the
%   "verified reference" was a noise-free oracle and bit-identical across
%   seeds. (2) 'vfh' rebuilt its beam angles as a full circle regardless of
%   the sensor, so a 60-deg ToF fan was smeared over 360 deg. (3) 'vfh' was
%   documented as horizon-free but inherited pred_lead = 0.5 s from cfg.vfh.
%
%   comp_ms is the wall-clock steering cost in ms (sensing included, the
%   same for every variant). A standalone file so Octave can use it too.
%
%   See also SCAN_RANGES_DYN, SENSOR_NOISE, PVFH_PROB, APF_OBSTACLE_REPULSION.

tic_h = tic;
switch lower(mode)
    case 'rigid'
        vec = dispatch_one(q, goal, w_att, world, movers, t_now, margin, ...
                           cfg, prev_heading, v_self, false);
    case 'point'
        vec = dispatch_one(q, goal, w_att, world, movers, t_now, margin, ...
                           cfg, prev_heading, [], true);
    otherwise
        error('steering_dispatch: unknown mode ''%s''', mode);
end
comp_ms = 1000 * toc(tic_h);
end

% ------------------------------------------------------------------------
function vec = dispatch_one(q, goal, w_att, world, movers, t_now, margin, ...
                            cfg, prev_heading, v_self, forming) %#ok<INUSL>
%DISPATCH_ONE  The actual algorithm switch (shared body of both phases).

% Body-mounted sensor: the ToF boresight follows the body heading.
if isfield(cfg.vfh, 'sense') && isstruct(cfg.vfh.sense)
    cfg.vfh.sense.boresight = prev_heading;
end
if forming, V = cfg.forming.V; else, V = cfg.V; end
% The external baselines (fgm, dwa, mpc) get the SAME mover-prediction
% horizon as pvfh / pvfh_prob unless cfg.baseline_horizon = false, so the
% comparison with them is about the steering law, not about who was told
% where the movers are going. ('apf' and 'vfh' stay instantaneous: they are
% the classic laws, and vfh -> pvfh is the step that isolates the horizon.)
base_horizon = ~isfield(cfg, 'baseline_horizon') || cfg.baseline_horizon;
% Selection memory (VFH+ third cost term) is kept for the transport leader
% only: the forming robots share these functions and would mix their states.
mem_tag = '';
if ~forming, mem_tag = ['rigid_' lower(cfg.algo)]; end

switch lower(cfg.algo)
    case 'apf'
        % Pure APF baseline. 'rigid': obstacle reflex only -- the caller's
        % vec_goal already carries the attraction (adding it here would
        % double-count the goal term). 'point': the attraction toward the
        % caller's virtual goal is added HERE (cascaded input).
        sc = local_sense(q, world, movers, t_now, cfg, margin, false);
        a  = apf_obstacle_repulsion(sc.ranges, sc.angles, sc.danger, ...
                 cfg.vfh.avoid_falloff_power, cfg.epsilon, sc.dth);
        na = norm(a);
        if na > cfg.epsilon, a = a / na; end
        if forming
            vec = w_att * goal_vector(goal, q, cfg.epsilon) + cfg.vfh.avoid_gain * a;
            nv = norm(vec);
            if nv > cfg.epsilon, vec = vec / nv; end
        else
            vec = a;
        end
    case 'vfh'
        % Deterministic VFH+ on the instantaneous noisy scan.
        sc  = local_sense(q, world, movers, t_now, cfg, margin, false);
        vec = local_vfh_from_scan(sc, q, goal, margin, cfg, prev_heading, mem_tag, t_now);
    case 'pvfh'
        % Deterministic VFH+ on the noisy scan WITH the mover horizon.
        sc  = local_sense(q, world, movers, t_now, cfg, margin, true);
        vec = local_vfh_from_scan(sc, q, goal, margin, cfg, prev_heading, mem_tag, t_now);
    case 'pvfh_filt'
        % Ablation law: 'pvfh' + the SAME recursive range filter P-VFH uses,
        % but the deterministic kernel. pvfh -> pvfh_filt isolates the
        % filter; pvfh_filt -> pvfh_prob isolates the expected kernel.
        sc  = local_sense(q, world, movers, t_now, cfg, margin, true);
        if ~forming && strcmpi(cfg.vfh.sense.kind, 'lidar')
            s0 = cfg.vfh.sense.sigma0 * ones(size(sc.ranges));
            sc.ranges = range_filter(sc.ranges, s0, t_now, 0.08, 'pvfh_filt');
        end
        vec = local_vfh_from_scan(sc, q, goal, margin, cfg, prev_heading, mem_tag, t_now);
    case 'pvfh_prob'
        % P-VFH: probabilistic occupancy + APF reflex (the proposal). Senses
        % internally through the same front end (horizon on).
        % The recursive range filter needs one persistent scan history, so
        % it runs for the transport leader with a fixed (lidar) fan only.
        vv = cfg.vfh;
        vv.mem_tag = mem_tag;
        vv.rfilter = ~forming && (~isfield(vv, 'rfilter') || vv.rfilter) && ...
                     isfield(vv, 'sense') && strcmpi(vv.sense.kind, 'lidar');
        vec = pvfh_prob(q, world, movers, t_now, goal, margin, vv, ...
                        cfg.epsilon, prev_heading);
    case 'fgm'
        sc  = local_sense(q, world, movers, t_now, cfg, margin, base_horizon);
        vec = local_fgm(sc, q, goal, margin, cfg, prev_heading);
    case 'dwa'
        sc  = local_sense(q, world, movers, t_now, cfg, margin, base_horizon);
        vec = local_dwa(sc, q, goal, margin, cfg, prev_heading, V);
    case 'mpc'
        sc  = local_sense(q, world, movers, t_now, cfg, margin, base_horizon);
        vec = local_mpc(sc, q, goal, margin, cfg, prev_heading, V);
    otherwise
        error('Unknown cfg.algo ''%s''', cfg.algo);
end
end

% ------------------------------------------------------------------------
function sc = local_sense(q, world, movers, t_now, cfg, margin, with_horizon)
%LOCAL_SENSE  The one sensing path of every dispatch-level steering law.
%   Returns sc.ranges (noisy), sc.angles (absolute bearings, wrapped),
%   sc.dth (beam angular width), sc.max_sense, sc.danger.
v = cfg.vfh;
if ~with_horizon, v.pred_lead = 0; end
sc.max_sense = v.sense_range_base + margin;
if isfield(v, 'sense') && isfield(v.sense, 'max_range') && isfinite(v.sense.max_range)
    sc.max_sense = min(sc.max_sense, v.sense.max_range);
end
sc.danger = v.danger_dist_base + margin;
[clean, sc.angles, sc.dth] = scan_ranges_dyn(q, world, movers, t_now, ...
                                             v.n_beams, sc.max_sense, v);
if isfield(v, 'sense')
    [sc.ranges, ~, ~] = sensor_noise(clean, sc.angles, v, []);
else
    sc.ranges = clean;
end
end

% ------------------------------------------------------------------------
function vec = local_vfh_from_scan(sc, pos, goal, margin, cfg, prev_heading, mem_tag, t_now)
%LOCAL_VFH_FROM_SCAN  Deterministic VFH+ valley search on a provided scan.
%   Histogram/valley logic of PVFH_SCAN on the scan the caller sensed. Beam
%   bearings come from the scan itself (sc.angles) -- never rebuilt.
v = cfg.vfh;
S = v.num_sectors;
ranges = sc.ranges;
angles = sc.angles;

sector_angles = -pi + (2*pi)*(0:S-1)/S;
dsec = 2*pi/S;
sec_idx = min(max(floor((angles + pi) / (2*pi) * S) + 1, 1), S);

% Sectors no beam covers keep rmin = inf: the classic deterministic stack
% has no notion of "unknown", so an unobserved bearing reads free.
rmin = accumarray(sec_idx(:), ranges(:), [S 1], @min, inf)';

r_eff = 0;
if v.enlarge, r_eff = margin; end
sigma = v.sigma;
if v.auto_sigma
    sigma = (r_eff + v.react_gap) / sqrt(2*log(1/v.threshold));
end

hist_vals = zeros(1, S);
occ = find(rmin < sc.max_sense);
for si = occ
    d = max(rmin(si), 0);
    val = exp(-(d^2) / (2*sigma^2));
    if r_eff > 0
        gamma = asin(min(1, r_eff / max(d, r_eff)));
        k = ceil(gamma / dsec);
        idxs = mod((si-1-k : si-1+k), S) + 1;
    else
        idxs = si;
    end
    hist_vals(idxs) = max(hist_vals(idxs), val);
end

hist_vals = local_smooth_hist(hist_vals, v.smooth_window, v.circular_smooth);

candidate_idx = find(hist_vals < v.threshold);
if isempty(candidate_idx), [~, candidate_idx] = min(hist_vals); end

goal_heading = atan2(goal(2) - pos(2), goal(1) - pos(1));
dev = abs(wrap_pi(sector_angles(candidate_idx) - goal_heading));
feasible_idx = candidate_idx(dev <= v.turn_limit);
if isempty(feasible_idx), feasible_idx = candidate_idx; end

sh = sector_angles(feasible_idx);
costs = abs(wrap_pi(sh - goal_heading)) + v.hysteresis_weight * abs(wrap_pi(sh - prev_heading));
if nargin >= 8 && ~isempty(mem_tag) && isfield(v, 'commit_weight') && v.commit_weight > 0
    prev_choice = vfh_commit('get', mem_tag, t_now);
    if ~isempty(prev_choice)
        costs = costs + v.commit_weight * abs(wrap_pi(sh - prev_choice));
    end
end
[~, ci] = min(costs);
chosen_heading = sh(ci);
if nargin >= 8 && ~isempty(mem_tag), vfh_commit('set', mem_tag, t_now, chosen_heading); end
vec = [cos(chosen_heading), sin(chosen_heading)];

% close-range reflex on the same scan
vec = vec + v.avoid_gain * apf_obstacle_repulsion(ranges, angles, sc.danger, ...
                               v.avoid_falloff_power, cfg.epsilon, sc.dth);
nv = norm(vec);
if nv < cfg.epsilon
    vec = [cos(prev_heading), sin(prev_heading)];
else
    vec = vec / nv;
end
end

% ------------------------------------------------------------------------
function vec = local_fgm(sc, pos, goal, margin, cfg, prev_heading)
%LOCAL_FGM  Follow-the-Gap Method baseline (Sezer & Gokasan, RAS 2012).
%   1. Every return closer than d_thr is an obstacle; it blocks the bearings
%      within asin(margin/d) of it (the body-radius "bubble").
%   2. The widest run of free beams is the gap; its centre is phi_gap.
%   3. Final heading = weighted mean of phi_gap and the goal bearing, the
%      gap weight growing as alpha/d_min (the closer the nearest obstacle,
%      the more the gap centre dominates).
%   Knobs: cfg.vfh.fgm_alpha (default 2.0, metres). d_thr is the same
%   reaction envelope the histogram laws use (margin + react_gap), so all
%   laws start reacting at the same range.
v = cfg.vfh;
alpha = 2.0;
if isfield(v, 'fgm_alpha'), alpha = v.fgm_alpha; end
r  = max(sc.ranges, 1e-3);
nb = numel(r);
d_thr = margin + v.react_gap;
full_circle = abs(nb * sc.dth - 2*pi) < 1e-6;

blocked = false(1, nb);
for b = find(r < d_thr)
    gamma = asin(min(1, margin / max(r(b), margin)));
    k = ceil(gamma / sc.dth);
    if full_circle
        blocked(mod((b-1-k : b-1+k), nb) + 1) = true;
    else
        blocked(max(1, b-k) : min(nb, b+k)) = true;
    end
end

goal_heading = atan2(goal(2) - pos(2), goal(1) - pos(1));
free = ~blocked;
if ~any(free)
    vec = [cos(prev_heading), sin(prev_heading)];
    return;
end
if all(free)
    phi_gap = goal_heading;
else
    % longest run of free beams (circular for a full-circle fan)
    if full_circle
        first_blk = find(blocked, 1);
        shift = first_blk - 1;                 % rotate so index 1 is blocked
        f = free([first_blk:nb, 1:first_blk-1]);
    else
        shift = 0;
        f = free;
    end
    d  = diff([0, f, 0]);
    st = find(d == 1);  en = find(d == -1) - 1;
    [~, gi] = max(en - st);
    mid = (st(gi) + en(gi)) / 2 + shift;       % fractional beam index
    a0  = sc.angles(1);                        % fan is uniformly spaced
    phi_gap = wrap_pi(a0 + (mid - 1) * sc.dth);
end
d_min = max(min(r), 1e-3);
w_gap = alpha / d_min;
phi = goal_heading + wrap_pi(phi_gap - goal_heading) * w_gap / (w_gap + 1);
vec = [cos(phi), sin(phi)];
end

% ------------------------------------------------------------------------
function vec = local_dwa(sc, pos, goal, margin, cfg, prev_heading, V)
%LOCAL_DWA  Dynamic-Window-style arc-sampling baseline (Fox et al. 1997).
%   The mission layer owns the speed (constant V, mover slowdown outside),
%   so the window is over the turn rate only: candidate arcs (V, w) are
%   rolled out for T seconds against the obstacle points of the SAME noisy
%   scan, arcs whose body (radius = margin) would touch an obstacle point
%   within the admissibility horizon are discarded, and the survivor
%   maximising   a_h*heading + a_c*clearance   wins (the velocity term of
%   the original objective is constant here). The steering vector is the
%   arc's heading after tau seconds, handed to the same blend as every
%   other law.
%   Knobs (cfg.vfh.dwa, all optional): w_max 1.5 rad/s, n_w 31, T 2.0 s,
%   T_adm 1.0 s, dt 0.2 s, a_h 0.8, a_c 0.2, tau 1.0 s, clr_cap 1.0 m.
P = struct('w_max', 1.5, 'n_w', 31, 'T', 2.0, 'T_adm', 1.0, 'dt', 0.2, ...
           'a_h', 0.8, 'a_c', 0.2, 'tau', 1.0, 'clr_cap', 1.0);
if isfield(cfg.vfh, 'dwa') && isstruct(cfg.vfh.dwa)
    f = fieldnames(cfg.vfh.dwa);
    for i = 1:numel(f), P.(f{i}) = cfg.vfh.dwa.(f{i}); end
end

hit = sc.ranges < sc.max_sense & isfinite(sc.ranges);
ox = pos(1) + max(sc.ranges(hit), 0) .* cos(sc.angles(hit));
oy = pos(2) + max(sc.ranges(hit), 0) .* sin(sc.angles(hit));

ws = linspace(-P.w_max, P.w_max, P.n_w);
ts = P.dt:P.dt:P.T;
n_adm = sum(ts <= P.T_adm + 1e-9);
th0 = prev_heading;

best = -inf;  best_w = 0;
fb_clr = -inf; fb_w = 0;                 % fallback: max clearance arc
for w = ws
    th = th0 + w * ts;
    if abs(w) < 1e-6
        px = pos(1) + V * ts * cos(th0);
        py = pos(2) + V * ts * sin(th0);
    else
        px = pos(1) + (V / w) * (sin(th) - sin(th0));
        py = pos(2) - (V / w) * (cos(th) - cos(th0));
    end
    if isempty(ox)
        clr_all = inf(1, numel(ts));
    else
        dx = px(:) - ox(:)';  dy = py(:) - oy(:)';      % T x N
        clr_all = (min(sqrt(dx.^2 + dy.^2), [], 2) - margin)';
    end
    clr_adm = min(clr_all(1:n_adm));
    clr     = min(clr_all);
    if clr_adm > fb_clr, fb_clr = clr_adm; fb_w = w; end
    if clr_adm <= 0, continue; end                      % not admissible
    gb   = atan2(goal(2) - py(end), goal(1) - px(end));
    head = 1 - abs(wrap_pi(gb - th(end))) / pi;
    score = P.a_h * head + P.a_c * min(max(clr, 0), P.clr_cap) / P.clr_cap;
    if score > best, best = score; best_w = w; end
end
if ~isfinite(best), best_w = fb_w; end                  % boxed in
phi = th0 + max(min(best_w * P.tau, pi/2), -pi/2);
vec = [cos(phi), sin(phi)];
end

% ------------------------------------------------------------------------
function vec = local_mpc(sc, pos, goal, margin, cfg, prev_heading, V)
%LOCAL_MPC  Receding-horizon trajectory-optimisation baseline.
%   The optimisation-based class of local planner (TEB / MPC / MPPI),
%   reduced to its core at this mission's interface: every step, optimise a
%   turn-rate SEQUENCE over a horizon against the obstacle points of the
%   same noisy scan, apply the first move, re-plan. The search is an
%   exhaustive lattice (n_seg segments x n_lvl turn-rate levels) rather
%   than random shooting, so the planner draws no random numbers and the
%   common-random-numbers design of the study is preserved.
%     cost = -progress toward the goal
%            + w_obs * mean soft barrier  max(0, d_safe - clearance)^2
%            + w_smooth * mean |turn rate|  + w_jerk * mean |change of rate|
%     hard constraint: body (radius = margin) clear of every scan point for
%     the first T_hard seconds.
%   Unlike DWA (one constant arc), a sequence can plan an S-manoeuvre.
%   Knobs (cfg.vfh.mpc, all optional): n_seg 3, seg_T 0.8 s, levels
%   [-1.2 -0.6 0 0.6 1.2] rad/s, dt 0.2 s, T_hard 1.0 s, d_safe 0.5 m,
%   w_obs 4, w_smooth 0.05, w_jerk 0.05, tau 1.0 s.
P = struct('n_seg', 3, 'seg_T', 0.8, 'levels', [-1.2 -0.6 0 0.6 1.2], ...
           'dt', 0.2, 'T_hard', 1.0, 'd_safe', 0.5, 'w_obs', 4.0, ...
           'w_smooth', 0.05, 'w_jerk', 0.05, 'tau', 1.0);
if isfield(cfg.vfh, 'mpc') && isstruct(cfg.vfh.mpc)
    f = fieldnames(cfg.vfh.mpc);
    for i = 1:numel(f), P.(f{i}) = cfg.vfh.mpc.(f{i}); end
end

% ---- the lattice of turn-rate sequences (rows) ----
L  = numel(P.levels);
nq = L ^ P.n_seg;
idx = zeros(nq, P.n_seg);
for s = 1:P.n_seg
    idx(:, s) = mod(floor((0:nq-1)' / L^(s-1)), L) + 1;
end
Wseg = reshape(P.levels(idx), nq, P.n_seg);       % nq x n_seg (rad/s)
per  = max(1, round(P.seg_T / P.dt));             % steps per segment
W    = kron(Wseg, ones(1, per));                  % nq x T
T    = size(W, 2);

% ---- roll out every sequence (unicycle, constant speed V) ----
th = prev_heading + cumsum(W * P.dt, 2);
px = pos(1) + cumsum(V * cos(th) * P.dt, 2);
py = pos(2) + cumsum(V * sin(th) * P.dt, 2);

% ---- clearance of every rollout point to the scan's obstacle points ----
hit = sc.ranges < sc.max_sense & isfinite(sc.ranges);
ox = pos(1) + max(sc.ranges(hit), 0) .* cos(sc.angles(hit));
oy = pos(2) + max(sc.ranges(hit), 0) .* sin(sc.angles(hit));
if isempty(ox)
    clr = inf(nq, T);
else
    dx  = px(:) - ox(:)';  dy = py(:) - oy(:)';    % (nq*T) x N
    clr = reshape(min(sqrt(dx.^2 + dy.^2), [], 2), nq, T) - margin;
end

n_hard   = min(T, max(1, round(P.T_hard / P.dt)));
feasible = min(clr(:, 1:n_hard), [], 2) > 0;

d0   = norm(goal - pos);
prog = d0 - sqrt((goal(1) - px(:, end)).^2 + (goal(2) - py(:, end)).^2);
bar  = mean(max(0, P.d_safe - clr).^2, 2);
J    = -prog + P.w_obs * bar + P.w_smooth * mean(abs(W), 2) ...
       + P.w_jerk * mean(abs(diff([zeros(nq, 1), Wseg], 1, 2)), 2);
if any(feasible)
    J(~feasible) = inf;
else
    J = -min(clr(:, 1:n_hard), [], 2);            % boxed in: max clearance
end
[~, best] = min(J);
phi = prev_heading + max(min(Wseg(best, 1) * P.tau, pi/2), -pi/2);
vec = [cos(phi), sin(phi)];
end

% ------------------------------------------------------------------------
function y = local_smooth_hist(x, w, circular)
%LOCAL_SMOOTH_HIST  Circular (or clamped) moving-average smoothing.
w = max(1, round(w));
if w == 1, y = x; return; end
n = numel(x); half = floor(w/2);
k = ones(1, 2*half+1) / (2*half+1);
if circular
    pad = [x(end-half+1:end), x, x(1:half)];
    full = conv(pad, k, 'same');
    y = full(half+1 : half+n);
else
    y = zeros(1, n);
    for i = 1:n
        lo = max(1, i-half); hi = min(n, i+half);
        y(i) = mean(x(lo:hi));
    end
end
end
