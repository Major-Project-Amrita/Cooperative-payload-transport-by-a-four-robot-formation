function [ranges_noisy, valid, drop] = sensor_noise(ranges, angles, v, rng)
%SENSOR_NOISE  ToF/LiDAR range-noise model shared by all algorithm variants.
%
%   [ranges_noisy, valid, drop] = SENSOR_NOISE(ranges, angles, v, rng)
%
%   One model, two sensing modes (v.sense.kind):
%
%   'lidar'  -- dense 360-deg LiDAR. Homoscedastic Gaussian noise:
%                  d_meas = d + N(0, sigma0^2)
%   'tof'    -- sparse ToF array (the beam fan / FoV arc is already handled
%               upstream by SCAN_RANGES_DYN). Every range carries
%               DISTANCE-DEPENDENT Gaussian noise:
%                  d_meas = d + N(0, (sigma0 + k_rng * d)^2)
%               which is the standard error model for direct ToF ranging
%               (VL53L1X/VL53L5CX-class sensors): sigma grows roughly
%               linearly with range and with target reflectivity loss.
%
%   Both modes apply MAX-RANGE DROP-CUT: any beam whose (noisy) range
%   exceeds max_range reports max_range. This reproduces the real edge
%   behaviour where a weak return near the maximum is indistinguishable from
%   no return; consumers treat max-range returns with reduced confidence
%   (PVFH_PROB floors their blocked probability at p_unknown).
%
%   The beam fan is deterministic -- only the ranges are stochastic -- so a
%   fixed rng seed makes an entire experiment reproducible.
%
%   Inputs
%     ranges, angles  clean scan from SCAN_RANGES_DYN (angles in world frame)
%     v               vfh config; reads v.sense.kind ('lidar'|'tof'),
%                     v.sense.sigma0 (m, default 0.02),
%                     v.sense.k_rng  (-, default 0.03),
%                     v.sense.max_range (default inf)
%     rng             random stream handle (rng()); [] -> nondeterministic
%
%   Outputs
%     ranges_noisy  1xN; NaN only if the clean scan was already non-finite
%     valid         logical 1xN, true where a finite return exists
%     drop          logical 1xN, true where the drop-cut consumed the return
%
%   All algorithm variants share THIS model, so the paper's comparison
%   isolates the steering law, not the sensing. The deterministic and
%   probabilistic consumers differ only in what they DO with the same
%   measured ranges: VFH puts the raw number into the histogram; P-VFH keeps
%   the per-bearing uncertainty and accumulates occupancy probability.
%
%   See also SCAN_RANGES_DYN, PVFH_PROB.


kind   = 'lidar';
sigma0 = 0.02;
k_rng  = 0.03;
max_r  = inf;
if isfield(v, 'sense') && isstruct(v.sense)
    if isfield(v.sense, 'kind'),      kind    = v.sense.kind;      end
    if isfield(v.sense, 'sigma0'),    sigma0  = v.sense.sigma0;    end
    if isfield(v.sense, 'k_rng'),     k_rng   = v.sense.k_rng;     end
    if isfield(v.sense, 'max_range'), max_r   = v.sense.max_range; end
end

ranges_noisy = ranges(:)';          % row, like the input
n = numel(ranges_noisy);

% ---- stochastic range noise ----
switch lower(kind)
    case 'tof'
        s = sigma0 + k_rng * max(ranges_noisy, 0);   % distance-dependent sigma
    case 'lidar'
        s = sigma0 * ones(1, n);
    otherwise
        error('sensor_noise: unknown sense.kind ''%s''', kind);
end
if nargin >= 4 && ~isempty(rng)
    noise = s .* randn(rng, 1, n);   % injected RandStream handle (MATLAB)
else
    noise = s .* randn(1, n);        % global stream (MATLAB rng(seed) AND
end                                 % Octave randn('seed', s))
ranges_noisy = ranges_noisy + noise;

% ---- drop-cut at max range (weak return ~= no return) ----
cut = ranges_noisy >= max_r;
ranges_noisy(cut) = max_r;
drop = cut;

valid = isfinite(ranges_noisy);
end
