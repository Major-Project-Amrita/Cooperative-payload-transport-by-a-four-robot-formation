function [found, escape_heading] = find_escape_heading_dyn(pos, world, movers, t_now, ...
                                    offsets, robot_radius, preferred_heading, ...
                                    num_probe, mode, step, T_hold, dt, ...
                                    avoid_heading, avoid_width)
%FIND_ESCAPE_HEADING_DYN  Escape probing against MOVING obstacles.
%
%   [found, escape_heading] = FIND_ESCAPE_HEADING_DYN(pos, world, movers, ...
%       t_now, offsets, robot_radius, preferred_heading, num_probe, mode, ...
%       step, T_hold, dt)
%
%   Same probe order as FIND_ESCAPE_HEADING_SDF (headings sorted by deviation
%   from preferred, first translatable one wins), with the dynamic addition:
%   a candidate is accepted only if the body can (a) TRANSLATE one step along
%   it right now AND (b) HOLD that pose safely for T_hold seconds while the
%   movers keep moving. Condition (b) evaluates the full body at t_now,
%   t_now+dt, ..., t_now+T_hold against mover poses from the same shared
%   motion contract - a mover sweeping through the candidate pose rejects it.
%
%   The hold test is conservative by design: it assumes the body pauses while
%   yielding. Anything that passes it is safe to commit to; anything that
%   fails it is rejected even if it is momentarily free. With no movers the
%   hold test degenerates to the static translate test and behaviour matches
%   FIND_ESCAPE_HEADING_SDF.
%
%   See also FIND_ESCAPE_HEADING_SDF, FORMATION_CLEARANCE_DYN, UPDATE_WORLD_DYN.

if nargin < 9  || isempty(mode),      mode      = 'payload'; end
if nargin < 10 || isempty(step),      step      = 0.3;      end
if nargin < 11 || isempty(T_hold),    T_hold    = 1.5;      end
if nargin < 12 || isempty(dt),        dt        = 0.1;      end
% Optional diversification (anti-deadlock layer): skip candidates within
% avoid_width rad of avoid_heading, so two successive probes do not commit
% to the same dead end.
if nargin < 13, avoid_heading = []; end
if nargin < 14 || isempty(avoid_width), avoid_width = 0.5; end

angles = -pi + 2*pi*(0:num_probe-1)/num_probe;
devs   = abs(mod(angles - preferred_heading + pi, 2*pi) - pi);
[~, order] = sort(devs);

% Precompute the mover poses over the hold horizon once - the movers follow
% the shared contract, so their future does not depend on the probe.
n_hold = round(T_hold / dt);
hold_times = t_now + (0:n_hold)' * dt;
hold_movers = cell(n_hold+1, 1);
for j = 1:n_hold+1
    hold_movers{j} = eval_all_movers(movers, hold_times(j));
end

found          = false;
escape_heading = preferred_heading;

for oi = order(:)'
    theta = angles(oi);
    if ~isempty(avoid_heading) && ...
            abs(mod(theta - avoid_heading + pi, 2*pi) - pi) < avoid_width
        continue;
    end
    cand  = pos + step*[cos(theta) sin(theta)];

    % (a) static-and-mover collision right now, after the translation
    [~, hit_now] = formation_clearance_dyn(cand, theta, offsets, world, ...
                                           robot_radius, mode, hold_movers{1});
    if hit_now, continue; end

    % (b) hold the pose while the movers keep moving
    safe = true;
    for j = 2:n_hold+1
        [~, hit_j] = formation_clearance_dyn(cand, theta, offsets, world, ...
                                             robot_radius, mode, hold_movers{j});
        if hit_j, safe = false; break; end
    end
    if safe
        found          = true;
        escape_heading = theta;
        return;
    end
end
end

% ------------------------------------------------------------------------
function movers_t = eval_all_movers(movers, t)
movers_t = movers;
for k = 1:numel(movers_t)
    m           = movers_t(k);
    m.c         = eval_mover_pose(m, t);   % standalone file: single source of truth
    m.t         = t;
    movers_t(k) = m;
end
end
