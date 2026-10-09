function [ranges, angles, dth] = scan_ranges_dyn(pos, world, movers, t_now, nb, max_sense, v)
%SCAN_RANGES_DYN  Clean (noise-free) scan: statics + movers over the horizon.
%
%   [ranges, angles] = SCAN_RANGES_DYN(pos, world, movers, t_now, nb, max_sense, v)
%
%   The SHARED sensing front end for every steering algorithm:
%     PVFH_SCAN / PVFH_PREDICT / PVFH_PROB  and  APF_OBSTACLE_REPULSION.
%   One place that defines what "the robot sees" means, so the algorithm
%   variants in the paper's comparison are all fed by identical geometry.
%
%   Beam fan:
%     sense.kind = 'lidar'  nb beams over the full circle (-pi, pi].
%     sense.kind = 'tof'    nb beams over the FoV arc
%                           [-fov_deg/2, +fov_deg/2] centred on the sensor
%                           BORESIGHT. sense.frame = 'body' (default): the
%                           boresight is sense.boresight, the body heading
%                           set by STEERING_DISPATCH -- a body-mounted array
%                           looks where the body points. sense.frame =
%                           'world' reproduces the legacy world-+x fan.
%                           Bearings outside the arc are simply never
%                           generated: the consumers see them as UNKNOWN.
%
%   dth (3rd output) is the angular width one beam represents (2*pi/nb for
%   the lidar, fov/nb for the ToF fan). Consumers that integrate over
%   bearing (APF_OBSTACLE_REPULSION) must use it: assuming 2*pi/nb on a
%   60-deg fan over-weights every ToF return six-fold.
%
%   Movers are scanned at t_now and, when v.pred_lead > 0, also at their
%   contracted poses over the horizon. pred_lead = 0 is the INSTANTANEOUS
%   scan (movers visible where they are now) used by the classic baselines.
%
%   With empty MOVERS this is exactly LIDAR_SCAN (the static path).
%   With movers it min-combines each mover's contracted future poses over the
%   prediction horizon T_lead = v.pred_lead (sampled every v.pred_dt), the
%   dynamic extension introduced by PVFH_PREDICT: each beam keeps the nearest
%   blocking instance over the horizon, so an approaching forklift is seen at
%   the range it will have when it gets there.
%
%   v fields used: sense (struct), pred_lead (default 0.5), pred_dt (0.1).
%
%   See also LIDAR_SCAN, LIDAR_SCAN_DYN, SENSOR_NOISE, PVFH_PROB.

if nargin < 7, v = struct(); end
if ~isfield(v, 'pred_lead'), v.pred_lead = 0.5; end
if ~isfield(v, 'pred_dt'),   v.pred_dt   = 0.1; end

% ---- beam fan geometry ----
kind = 'lidar';  fov_deg = 60;
if isfield(v, 'sense') && isstruct(v.sense)
    if isfield(v.sense, 'kind'),    kind    = v.sense.kind;    end
    if isfield(v.sense, 'fov_deg'), fov_deg = v.sense.fov_deg; end
end
if strcmpi(kind, 'tof')
    bore = 0;
    frame = 'body';
    if isfield(v.sense, 'frame'), frame = v.sense.frame; end
    if strcmpi(frame, 'body') && isfield(v.sense, 'boresight') && ...
            ~isempty(v.sense.boresight)
        bore = v.sense.boresight;
    end
    angle_min = bore - pi/180 * fov_deg/2;
    angle_max = bore + pi/180 * fov_deg/2;
else
    angle_min = -pi;
    angle_max =  pi;
end
dth = (angle_max - angle_min) / nb;

% ---- 1. statics: analytic raycast, exactly as PVFH_SCAN ----
[ranges, angles] = lidar_scan(pos, world, nb, max_sense, angle_min, angle_max);

% ---- 1b. movers: per-beam scan at t_now (+ the prediction horizon) ----
% The t_now sample is ALWAYS taken: with pred_lead = 0 the old guard skipped
% the movers altogether, i.e. an "instantaneous" scan could not see them.
if ~isempty(movers)
    wempty        = world;
    wempty.boxes   = world.boxes([]);    % 0x0 struct, same fields
    wempty.circles = world.circles([]);
    samples = unique([0, 0:v.pred_dt:v.pred_lead, max(v.pred_lead, 0)]);
    for j = 1:numel(samples)
        mk = movers;
        for k = 1:numel(mk)
            mk(k).c = eval_mover_pose(movers(k), t_now + samples(j));
        end
        [rk, ~] = lidar_scan_dyn(pos, wempty, mk, nb, max_sense, angle_min, angle_max);
        ranges  = min(ranges, rk);
    end
end

% A body-frame fan can straddle +/-pi; consumers bin by absolute bearing.
angles = mod(angles + pi, 2*pi) - pi;
end
