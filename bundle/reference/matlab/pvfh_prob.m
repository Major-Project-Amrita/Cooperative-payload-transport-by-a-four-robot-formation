function [vec, info] = pvfh_prob(pos, world, movers, t_now, goal, margin, v, eps_cfg, prev_heading)
%PVFH_PROB  Probabilistic VFH: P-VFH over uncertain ranges + APF obstacle term.
%
%   [vec, info] = PVFH_PROB(pos, world, movers, t_now, goal, margin, v, eps_cfg, prev_heading)
%
%   The paper's steering layer. Two-stage architecture (both stages consume
%   the SAME noisy scan, so the composition is the contribution, not the
%   pieces):
%
%   STAGE 1 -- P-VFH sector probabilities (per bearing, then per sector)
%   ------------------------------------------------------------------------
%   The measured range of a beam is modelled as
%       d_meas = d + e,      e ~ N(0, s_b^2)              (1)
%   where the true range d is UNKNOWN (that is the whole point) and s_b is
%   the per-beam standard deviation from SENSOR_NOISE. Uncertainty in the
%   TRUE range given the measurement is modelled by the likelihood
%       L_b(d) = exp( -(d - d_meas,b)^2 / (2 s_b^2) ).        (2)
%   The physics is one-sided: the beam stopped where it stopped, so there is
%   (almost) nothing BEHIND the return. The belief about "is this bearing
%   blocked?" therefore lives on d in (0, d_meas]:
%       p_b = P(d <= d_block | d_meas)
%           = Phi((d_block - d_meas)/s_b)
%             - Phi((d_block - 0)/s_b)                            (3)
%           ~ Phi((d_block - d_meas)/s_b)   for d_block >> s_b,
%   with d_block the clearance a bearing must protect: the effective body
%   radius r_eff PLUS the reaction margin react_gap (the same envelope the
%   deterministic stack's auto_sigma contract encodes, so both stacks react
%   at the same distance). Phi is the standard normal CDF (erf). A
%   confidently-far return (d_meas >> d_block) gives p ~ 0, a confidently-near
%   return (d_meas < d_block) gives p ~ 1: the p = 0.5 crossing sits exactly
%   at d_meas = d_block, and the transition width is the sensor's own
%   uncertainty -- no hand-tuned safety inflation.
%
%   Per-beam probabilities are pooled per sector with a noisy-OR, which is
%   the correct combiner for independent chances of the SAME fatal event:
%       p_s = 1 - prod_b in s (1 - p_b).                      (4)
%   Dense LiDAR (288 beams) leaves it ~= max_b p_b (first hit wins, the rest
%   are occluded); sparse ToF (8-16 beams over the same arc) benefits because
%   each additional beam covering the sector adds evidence of blockage.
%
%   SELECTION KERNEL -- the posterior-expected danger envelope (5)
%   ------------------------------------------------------------------------
%   The deterministic stack thresholds the danger kernel
%       env(d) = exp( -d^2 / (2 sigma_hist^2) ),              (5)
%   whose auto_sigma width makes env(d_block) = threshold: the histogram
%   value is a GRADED function of range, and the valley search exploits that
%   grading. Thresholding the blockage PROBABILITY p directly does not work:
%   p saturates at 1 for any return measurably inside d_block, so every VFH+
%   enlarged arc becomes a full-amplitude plateau, circular smoothing seals
%   the gaps between obstacle clusters, and the stack stalls (found and fixed
%   during development). The selection layer is therefore the POSTERIOR
%   EXPECTATION of the deterministic kernel under the sensor likelihood
%   d ~ N(d_meas, s_b^2), which has the closed form
%       u_b = E[env(d_true)]
%           = sqrt( sig^2 / (sig^2 + s_b^2) )
%             * exp( -d_meas^2 / (2 (sig^2 + s_b^2)) ).        (6)
%   Three properties make this the right bridge between the probability layer
%   and the control law:
%     (i)  EXACT DETERMINISTIC LIMIT: as s_b -> 0, u_b -> env(d_meas). P-VFH
%          degenerates to the deterministic VFH histogram when the sensor is
%          precise -- the low-noise behaviour is inherited, not re-tuned.
%     (ii) BOUNDED AND GRADED: u_b < 1 always, and enlarged arcs carry
%          range-graded values, so valleys survive smoothing exactly as in
%          the deterministic stack.
%     (iii) NOISE-MONOTONE REACTION DISTANCE: the range at which a return
%          crosses the threshold, d_react(s_b) (solve u_b = P_th), grows
%          strictly with s_b for every s_b < 1.757*sig (Proposition 2 in
%          the report; sig is ~1.3 m here, so the bound is far beyond any
%          range sensor). Precisely: du_b/ds_b > 0 iff d_meas^2 > sig^2 +
%          s_b^2 -- far returns (the ones that decide free vs blocked)
%          inflate with noise, near returns deflate slightly but stay above
%          threshold. The earlier blanket claim "du_b/ds_b > 0 at every
%          range" was wrong and has been corrected.
%   The kernel composes per sector by MAX (the nearest-return rule, as in
%   the deterministic stack -- kernels are not probabilities, so noisy-OR
%   would double-count). Unknown bearings (no return) and max-range drop-
%   cuts are floored at p_unknown in BOTH layers: absence of evidence is not
%   evidence of absence. Free space is only ever concluded where a
%   confident, in-range return exists.
%
%   The sector kernel then passes through the VFH+ enlarged-obstacle
%   propagation and the standard threshold / valley / hysteresis selection,
%   all as in PVFH_SCAN -- the control topology is unchanged, only the
%   occupancy values are now posterior expectations:
%       hist(s)  = smoothed MAX-pooled kernel u_s over the enlarged arc
%       free(s)  = hist(s) < threshold = P_th                        (7)
%   The risk layer (p_beam, noisy-OR p_sector) is a DIAGNOSTIC output: it is
%   reported in info for logging/plots and does not enter the steering.
%
%   STAGE 2 -- APF obstacle reflex on the noisy scan
%   ------------------------------------------------------------------------
%   The close-range reflex of VFH is replaced by APF_OBSTACLE_REPULSION,
%   which pushes along the confidence-weighted negative range gradient. A
%   bearing that reads near with high confidence pushes hard; a noisy or
%   unknown bearing pushes little. The two stages are fused exactly like the
%   deterministic stack: the chosen valley direction is the P-VFH steering
%   vector, the reflex is added with avoid_gain.
%
%   info fields (for logging/plots):
%     .p_beam    per-beam blocked probability (risk layer)
%     .p_sector  noisy-OR pooled risk per sector
%     .u_beam    per-beam posterior-expected kernel (selection layer)
%     .u_sector  max-pooled kernel per sector (before smoothing/enlargement)
%     .hist      smoothed histogram actually thresholded
%     .sigma_eff median per-beam sigma actually used
%     .sigma_hist deterministic kernel width from the auto_sigma contract
%     .ranges/.angles the noisy scan consumed
%     .chosen_heading, .min_range
%
%   See also SENSOR_NOISE, APF_OBSTACLE_REPULSION, PVFH_SCAN, PVFH_PREDICT.

if ~isfield(v, 'enlarge'),    v.enlarge    = true; end
if ~isfield(v, 'auto_sigma'), v.auto_sigma = true; end
if ~isfield(v, 'react_gap'),  v.react_gap  = 1.0; end
if ~isfield(v, 'pred_lead'),  v.pred_lead  = 0.5; end
if ~isfield(v, 'pred_dt'),    v.pred_dt    = 0.1; end
if ~isfield(v, 'circular_smooth'), v.circular_smooth = true; end
if ~isfield(v, 'p_unknown'),  v.p_unknown  = 0.05; end

S = v.num_sectors;
max_sense = v.sense_range_base + margin;
if isfield(v, 'sense') && isfield(v.sense, 'max_range') && isfinite(v.sense.max_range)
    max_sense = min(max_sense, v.sense.max_range);
end
danger_dist = v.danger_dist_base + margin;

nb = v.n_beams;
if isempty(nb), nb = 4*S; end

% The histogram's Gaussian width follows from the required reaction margin,
% identical to the deterministic stack's auto_sigma contract: it sets
% d_react, i.e. where a fully-confident hit must have pushed p above
% threshold. Per-BEARING sigmas come from the sensor model; this keeps the
% two stacks commensurate and is reported in info.sigma_hist.
sigma_hist = v.sigma;
if v.auto_sigma
    r_eff0 = margin * double(v.enlarge);
    sigma_hist = (r_eff0 + v.react_gap) / sqrt(2*log(1/v.threshold));
end

r_geom  = margin * double(v.enlarge);          % body radius for VFH+ asin
d_block = r_geom + v.react_gap;                % what a bearing must protect

% ---- 1. sense: clean scan (statics + predictive movers) ----
[clean_ranges, angles, dth_beam] = scan_ranges_dyn(pos, world, movers, t_now, nb, max_sense, v);

% ---- 2. the sensor: apply the noise model ONCE, here ----
[ranges, valid, drop] = sensor_noise(clean_ranges, angles, v, []);

% per-beam sigmas from the sensor model
sig_b = sensor_default_sigma(v, ranges);

% ---- 2b. recursive Bayesian range filter (transport, fixed lidar fan) ----
% The single-scan expected kernel alone does not make the law robust to
% noise: the sector value is a MAX over noisy beams, which is biased toward
% the noisiest short return and flickers from scan to scan exactly like the
% deterministic histogram (measured 2026-10-02 once the kernel bug was
% fixed). What the sensor model buys is the knowledge of HOW MUCH to trust
% one scan: RANGE_FILTER fuses scans over time with a gain set by s_b, and
% hands back the posterior mean and the (smaller) posterior sigma, which
% then enter the expected kernel below. With a precise sensor the gain is
% ~1 and nothing is filtered (deterministic limit preserved).
if isfield(v, 'rfilter') && v.rfilter && all(valid)
    q_std = 0.08;  if isfield(v, 'rfilter_q'), q_std = v.rfilter_q; end
    [ranges, sig_b] = range_filter(ranges, sig_b, t_now, q_std, 'pvfh_prob');
end

% ---- 2c. chance-constrained range (optional; v.lcb_kappa, default 0) ----
% A bearing should read free only if it is free with confidence: use the
% lower confidence bound d - kappa*s of the (posterior) range in the kernel,
% the enlargement and the reflex. kappa = 0 is the plain posterior mean.
if isfield(v, 'lcb_kappa') && v.lcb_kappa > 0
    ranges = max(ranges - v.lcb_kappa * sig_b, 0.01);
end

% ---- 3. per-beam layers: blockage probability (risk) + expected kernel ----
% Risk layer, eq. (3): p_b = P(true range d <= d_block | measured d_meas).
% Selection layer, eq. (6): u_b = E[env(d_true)] under d ~ N(d_meas, s_b^2).
d_meas = ranges;
d_meas(~valid) = max_sense;
z     = (d_block - d_meas) ./ max(sig_b, 1e-6);
p_beam = 0.5 * (1 + erf(z / sqrt(2)));          % Phi(z)
p_beam(~valid) = 0;                             % NaN -> unknown (floor below)
p_beam(drop)   = max(p_beam(drop), v.p_unknown);

s2  = sig_b.^2;
sh2 = sigma_hist^2;
% NB: './' -- with '/' this was a MATRIX right-division of two row vectors,
% i.e. ONE least-squares scalar for the whole scan, so every beam got the
% same kernel value and the histogram was flat (found 2026-10-02; every
% P-VFH result before that date was produced with a direction-blind
% histogram, steering by goal attraction + reflex only).
u_beam = sqrt(sh2 ./ (sh2 + s2)) .* exp( -d_meas.^2 ./ (2*(sh2 + s2)) );
u_beam(~valid) = v.p_unknown;                   % no return -> floor
u_beam(drop)   = max(u_beam(drop), v.p_unknown);

% ---- 4. sector pooling: noisy-OR risk, MAX kernel ----
sector_angles = -pi + (2*pi)*(0:S-1)/S;
dsec    = 2*pi/S;
sec_idx = floor((angles + pi) / (2*pi) * S) + 1;
sec_idx = min(max(sec_idx, 1), S);

p_sec = zeros(1, S);
u_sec = zeros(1, S);
n_in  = zeros(1, S);
for b = 1:numel(p_beam)
    si = sec_idx(b);
    if p_beam(b) > 0
        p_sec(si) = 1 - (1 - p_sec(si)) * (1 - p_beam(b));   % noisy-OR
        n_in(si)  = n_in(si) + 1;
    end
    u_sec(si) = max(u_sec(si), u_beam(b));                   % nearest-return
end
p_sec(p_sec == 0 & n_in == 0) = v.p_unknown;    % unobserved sector floor
u_sec(u_sec == 0 & n_in == 0) = v.p_unknown;

% ---- 5. VFH+ enlarged-obstacle propagation (kernel values) ----
% A sector with kernel value u_s blocks every bearing within asin(r_geom/d)
% of it -- the body steered down any of those bearings would still clip it.
% d is the measured range of the sector's nearest confident return; sectors
% without a confident return propagate their floored value only (their
% geometry is unknown).
hist_vals = u_sec;
if r_geom > 0
    for si = 1:S
        if u_sec(si) <= v.p_unknown, continue; end
        d = sector_min_range(ranges, valid, sec_idx, si);
        if ~isfinite(d), continue; end
        gamma = asin(min(1, r_geom / max(d, r_geom)));
        k     = ceil(gamma / dsec);
        idxs  = mod((si-1-k : si-1+k), S) + 1;
        hist_vals(idxs) = max(hist_vals(idxs), u_sec(si));
    end
end

hist_vals = smooth_hist(hist_vals, v.smooth_window, v.circular_smooth);

% ---- 6. threshold, valley search, hysteresis: standard VFH ----
candidate_idx = find(hist_vals < v.threshold);
if isempty(candidate_idx)
    [~, candidate_idx] = min(hist_vals);
end

goal_heading = atan2(goal(2) - pos(2), goal(1) - pos(1));
dev          = abs(wrap_pi(sector_angles(candidate_idx) - goal_heading));
feasible_idx = candidate_idx(dev <= v.turn_limit);
if isempty(feasible_idx)
    feasible_idx = candidate_idx;
end

sh    = sector_angles(feasible_idx);
costs = abs(wrap_pi(sh - goal_heading)) + ...
        v.hysteresis_weight * abs(wrap_pi(sh - prev_heading));
use_mem = isfield(v, 'mem_tag') && ~isempty(v.mem_tag);
if use_mem && isfield(v, 'commit_weight') && v.commit_weight > 0
    prev_choice = vfh_commit('get', v.mem_tag, t_now);
    if ~isempty(prev_choice)
        costs = costs + v.commit_weight * abs(wrap_pi(sh - prev_choice));
    end
end
[~, ci] = min(costs);
chosen_heading = sh(ci);
if use_mem, vfh_commit('set', v.mem_tag, t_now, chosen_heading); end

vec = [cos(chosen_heading), sin(chosen_heading)];

% ---- 7. APF obstacle reflex on the SAME noisy scan ----
vec = vec + v.avoid_gain * apf_obstacle_repulsion(ranges, angles, danger_dist, ...
                                                  v.avoid_falloff_power, eps_cfg, dth_beam);

nv = norm(vec);
if nv < eps_cfg
    vec = [cos(prev_heading), sin(prev_heading)];
else
    vec = vec / nv;
end

if nargout > 1
    info = struct('p_beam', p_beam, 'p_sector', p_sec, ...
                  'u_beam', u_beam, 'u_sector', u_sec, ...
                  'hist', hist_vals, ...
                  'ranges', ranges, 'angles', angles, 'valid', valid, ...
                  'sigma_eff', median(sig_b(valid)), 'sigma_hist', sigma_hist, ...
                  'chosen_heading', chosen_heading, 'min_range', min(ranges));
end
end

% ------------------------------------------------------------------------
function s = sensor_default_sigma(v, ranges)
%SENSOR_DEFAULT_SIGMA  Per-beam sigma from the sensor model in v.sense.
kind   = 'lidar';  s0 = 0.02;  kr = 0.03;
if isfield(v, 'sense') && isstruct(v.sense)
    if isfield(v.sense, 'kind'),   kind = v.sense.kind;   end
    if isfield(v.sense, 'k_rng'),  kr   = v.sense.k_rng;  end
    if isfield(v.sense, 'sigma0'), s0   = v.sense.sigma0; end
end
switch lower(kind)
    case 'tof'
        s = s0 + kr * max(ranges, 0);
    otherwise
        s = s0 * ones(size(ranges));
end
end

function r = sector_min_range(ranges, valid, sec_idx, si)
%SECTOR_MIN_RANGE  Nearest confident return in sector si (inf if none).
sel = (sec_idx == si) & valid(:)';
if ~any(sel), r = inf; return; end
r = min(ranges(sel));
end

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
