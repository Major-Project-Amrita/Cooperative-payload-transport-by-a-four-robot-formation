function a = apf_obstacle_repulsion(ranges, angles, danger_dist, falloff_power, eps_cfg, dth)
%APF_OBSTACLE_REPULSION  APF repulsive term from a scan (obstacle layer only).
%
%   a = APF_OBSTACLE_REPULSION(ranges, angles, danger_dist, falloff_power, eps_cfg)
%   a = APF_OBSTACLE_REPULSION(..., dth)   beam angular width (rad); default
%       2*pi/numel(ranges), correct ONLY for a full-circle fan. Pass the dth
%       returned by SCAN_RANGES_DYN for a limited-FoV (ToF) array.
%
%   The APF half of the hybrid steering law, restricted to OBSTACLES.
%   For every beam with range inside danger_dist, a repulsive acceleration
%   along the beam's reversed direction, weighted by an inverse-power law of
%   the measured range:
%
%       a = -sum_b  w_b * [cos th_b, sin th_b] / d_b^p ,  d_b < d_danger
%       w_b = Dth (beam angular width)  -> an integral over bearing, so the
%             magnitude is resolution-independent (re-raycasting at a
%             different n_beams does not rescale the reflex against the unit
%             steering vector).
%
%   This is the classical APF repulsion shape (Khatib-style falloff) with two
%   deliberate deviations, both documented in apf_force.m:
%     * the falloff exponent p is a knob (avoid_falloff_power, default 3)
%       instead of the classical fixed 1/d^2, because at p = 2 the reflex of
%       a 2.5 m rigid block saturates its turn budget two metres early;
%     * the law is applied to SCAN BEAMS (a real sensor input), not to a
%       point-cloud sample of obstacle interiors.
%
%   ROLE IN THE HYBRID
%   The polar-histogram stage (VFH/P-VFH) chooses WHERE to steer; this term
%   supplies the graded "away from what is about to hit me" component that a
%   72-bin histogram with a 5 deg sector width cannot express. In the
%   deterministic stack it is inlined at the bottom of pvfh_scan.m /
%   pvfh_predict.m; pvfh_prob.m calls it from here so the probabilistic and
%   deterministic laws share one implementation.
%
%   Zero when nothing is inside danger_dist: the reflex never perturbs free
%   space, so the Valley/hysteresis logic fully owns the heading there.
%
%   See also PVFH_PROB, PVFH_SCAN, APF_FORCE (goal + inter-robot APF).

a = [0, 0];
if isempty(ranges), return; end

close_b = ranges(:)' < danger_dist & isfinite(ranges(:)');
if ~any(close_b), return; end

r  = ranges(close_b);
aB = angles(close_b);
w  = 1 ./ (r.^falloff_power + eps_cfg);
if nargin < 6 || isempty(dth), dth = 2*pi / numel(ranges); end

a = [-sum(w .* cos(aB)) * dth, -sum(w .* sin(aB)) * dth];
end
