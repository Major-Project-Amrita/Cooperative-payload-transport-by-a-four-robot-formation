function [ranges, angles, owner] = lidar_scan_dyn(pos, world, movers, n_beams, max_range, angle_min, angle_max)
%LIDAR_SCAN_DYN  Analytic 2D laser scan of the static world + movers.
%
%   [ranges, angles] = LIDAR_SCAN_DYN(pos, world, movers, n_beams, max_range)
%   [ranges, angles, owner] = ...   owner(b) = 0 static hit / k mover-k hit
%
%   LIDAR_SCAN against the static geometry, then the same exact ray/primitive
%   intersections against every mover at its CURRENT pose. Because each beam
%   keeps the nearest hit, occlusion works across the static/dynamic boundary
%   for free: a mover hides the shelf behind it exactly as a shelf hides a
%   mover.
%
%   This is what the VFH layer consumes, so a mover appears in the polar
%   histogram the moment it is visible - the controller never sees a special
%   "dynamic" flag in its scan, which keeps the avoidance path identical to
%   the static one.
%
%   See also LIDAR_SCAN, WORLD_DIST_DYN, PVFH_SCAN.

if nargin < 6 || isempty(angle_min), angle_min = -pi; end
if nargin < 7 || isempty(angle_max), angle_max =  pi; end

% ---- static part ----
[ranges, angles] = lidar_scan(pos, world, n_beams, max_range, angle_min, angle_max);

nM = numel(movers);
if nargout > 2
    owner = zeros(1, numel(ranges));   % 0 = the static world owns this beam
end
if nM == 0, return; end

cx  = cos(angles);
sy  = sin(angles);
EPS = 1e-12;

for k = 1:nM
    m  = movers(k);
    ox = pos(1) - m.c(1);
    oy = pos(2) - m.c(2);

    if strcmpi(m.shape, 'circle')
        b    = ox*cx + oy*sy;
        cq   = ox*ox + oy*oy - m.r^2;
        disc = b.*b - cq;
        ok   = disc >= 0;
        sq   = sqrt(max(disc, 0));
        t    = -b - sq;
        tfar = -b + sq;
        t(t < 0) = tfar(t < 0);
        hit = ok & (t > 0) & (t < ranges);
        ranges(hit) = t(hit);
        if nargout > 2, owner(hit) = k; end
    else
        yaw = 0;
        if isfield(m, 'yaw'), yaw = m.yaw; end
        if yaw ~= 0
            c   = cos(-yaw); s = sin(-yaw);
            lox =  c*ox - s*oy;
            loy =  s*ox + c*oy;
            ldx =  c*cx - s*sy;
            ldy =  s*cx + c*sy;
        else
            lox = ox;  loy = oy;
            ldx = cx;  ldy = sy;
        end
        ldx(abs(ldx) < EPS) = EPS;
        ldy(abs(ldy) < EPS) = EPS;

        hx = m.size(1)/2; hy = m.size(2)/2;
        t1x = (-hx - lox) ./ ldx;   t2x = ( hx - lox) ./ ldx;
        t1y = (-hy - loy) ./ ldy;   t2y = ( hy - loy) ./ ldy;

        tmin = max(min(t1x, t2x), min(t1y, t2y));
        tmax = min(max(t1x, t2x), max(t1y, t2y));

        t = tmin;
        inside = tmin < 0;
        t(inside) = tmax(inside);

        hit = (tmax >= max(tmin, 0)) & (t > 0) & (t < ranges);
        ranges(hit) = t(hit);
        if nargout > 2, owner(hit) = k; end
    end
end
end
