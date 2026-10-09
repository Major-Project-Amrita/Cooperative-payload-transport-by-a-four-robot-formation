function [found, escape_heading] = find_escape_heading_sdf(pos, offsets, world, ...
                                       robot_radius, preferred_heading, num_probe, mode, step)
%FIND_ESCAPE_HEADING_SDF  Pick a heading that actually gets the body moving again.
%
%   Probes num_probe headings, in order of how little they deviate from
%   preferred_heading, and returns the first one the body can actually
%   TRANSLATE along without collision.
%
%   step  (0.3) lookahead distance, metres.
%
%   THE BUG THIS FIXES
%   The original tested the formation IN PLACE: it rotated the body about its
%   current centre and asked whether that pose was in collision. But a body
%   that is merely blocked ahead is not itself in collision, so the very first
%   probe - the goal heading, the one already jammed - came back clear and the
%   function reported "escape found" without escaping anything. The formation
%   re-committed to the same blocked heading every step, forever. That is the
%   stall at x = -4.35 m.
%
%   Testing pos + step*[cos sin] asks the question that matters: can the body
%   MOVE this way? Rotation alone never frees a roughly square block.
%
%   See also FORMATION_CLEARANCE.

if nargin < 8 || isempty(step), step = 0.3; end

angles = -pi + 2*pi*(0:num_probe-1)/num_probe;
devs   = abs(mod(angles - preferred_heading + pi, 2*pi) - pi);
[~, order] = sort(devs);

found          = false;
escape_heading = preferred_heading;

for oi = order(:)'
    theta = angles(oi);
    cand  = pos + step*[cos(theta) sin(theta)];
    [~, hit] = formation_clearance(cand, theta, offsets, world, robot_radius, mode);
    if ~hit
        found          = true;
        escape_heading = theta;
        return;
    end
end
end
