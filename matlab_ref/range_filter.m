function [xh, sp] = range_filter(z, s, t_now, q_std, tag)
%RANGE_FILTER  Per-beam recursive Bayesian (Kalman) filter of a range scan.
%
%   [xh, sp] = RANGE_FILTER(z, s, t_now, q_std, tag)
%
%   z      1xN measured ranges of this step (fixed beam fan: beam b always
%          looks along the same absolute bearing)
%   s      1xN measurement standard deviations (the sensor model's s_b)
%   t_now  simulation time; a step back in time, or a gap > 0.5 s, or a
%          change of N, re-initialises the filter (new mission)
%   q_std  process standard deviation per step: how much the true range
%          along a fixed bearing can change between two scans (sensor
%          motion + mover motion). Default 0.08 m.
%   tag    optional key so independent consumers keep independent state
%
%   xh     1xN posterior mean ranges
%   sp     1xN posterior standard deviations (<= s)
%
%   Model per beam: x_t = x_{t-1} + w, w ~ N(0, q^2);  z_t = x_t + e,
%   e ~ N(0, s^2). The gain K = P/(P + s^2) is set by the sensor noise: a
%   precise sensor (s << q) gives K ~ 1, i.e. NO filtering -- the
%   deterministic limit is preserved -- while a noisy one averages over
%   several scans. An innovation larger than 3 sigma is a discontinuity (a
%   beam sliding off an edge, a mover entering): that beam is re-initialised
%   to the measurement instead of being dragged slowly toward it, so a
%   newly visible obstacle is never seen late.
%
%   See also PVFH_PROB, SENSOR_NOISE.
persistent ST
if nargin < 4 || isempty(q_std), q_std = 0.08; end
if nargin < 5 || isempty(tag), tag = 'default'; end
if isempty(ST), ST = struct(); end
z = z(:)';  r = max(s(:)', 1e-9).^2;
fresh = ~isfield(ST, tag) || numel(ST.(tag).x) ~= numel(z) || ...
        t_now < ST.(tag).t - 1e-9 || t_now > ST.(tag).t + 0.5;
if fresh
    x = z;  P = r;
else
    x  = ST.(tag).x;
    Pp = ST.(tag).P + q_std^2;
    innov = z - x;
    Sg = Pp + r;
    K  = Pp ./ Sg;
    jump = abs(innov) > 3 * sqrt(Sg) | ~isfinite(x);
    x  = x + K .* innov;
    P  = (1 - K) .* Pp;
    x(jump) = z(jump);
    P(jump) = r(jump);
end
ST.(tag) = struct('x', x, 'P', P, 't', t_now);
xh = x;  sp = sqrt(P);
end
