function [ranges, angles] = lidar_scan(pos, world, n_beams, max_range, angle_min, angle_max)
%LIDAR_SCAN  Analytic 2D laser scan of an SDF world (a synthetic /scan).
%
%   [ranges, angles] = LIDAR_SCAN(pos, world, n_beams, max_range)
%   [ranges, angles] = LIDAR_SCAN(pos, world, n_beams, max_range, angle_min, angle_max)
%
%   pos        1x2 sensor position [x y] in world frame.
%   world      struct from SDF2WORLD.
%   n_beams    number of beams (match your real LiDAR, e.g. 360).
%   max_range  metres; beams that hit nothing return max_range.
%   angle_min/max  beam fan in WORLD frame (default -pi to pi).
%
%   ranges     1xN distances, angles 1xN absolute bearings.
%
%   WHY THIS MATTERS FOR THE COMPARISON
%   The original MATLAB p-VFH consumed a dense point cloud sampled over the
%   *interior* of every obstacle. A real robot in Gazebo consumes a LaserScan:
%   only the nearest visible surface along each bearing, with everything behind
%   it occluded. Those two inputs produce genuinely different VFH histograms,
%   so identical gains would still give you different trajectories and you would
%   have no way to tell whether a divergence came from the algorithm or from the
%   sensing model. Raycasting removes that confound: same algorithm, same input
%   modality, on both sides.
%
%   Exact analytic intersection (slab method for boxes, quadratic for circles),
%   so there is no rasterisation error. Cost is O(n_beams x numObstacles).
%
%   See also SDF2WORLD, WORLD_DIST, PVFH_SCAN.

if nargin < 5 || isempty(angle_min), angle_min = -pi; end
if nargin < 6 || isempty(angle_max), angle_max =  pi; end

% Open interval at the top so -pi and +pi are not duplicated beams.
angles = angle_min + (angle_max - angle_min) * (0:n_beams-1) / n_beams;
ranges = repmat(max_range, 1, n_beams);

cx = cos(angles);
sy = sin(angles);
EPS = 1e-12;

% ---------------- boxes: slab method in the box's local frame ----------------
for i = 1:numel(world.boxes)
    B = world.boxes(i);
    ox = pos(1) - B.c(1);
    oy = pos(2) - B.c(2);
    if B.yaw ~= 0
        c = cos(-B.yaw); s = sin(-B.yaw);
        lox =  c*ox - s*oy;
        loy =  s*ox + c*oy;
        ldx =  c*cx - s*sy;
        ldy =  s*cx + c*sy;
    else
        lox = ox;  loy = oy;
        ldx = cx;  ldy = sy;
    end

    % Nudge away from exact zero so parallel beams give +/-Inf cleanly.
    ldx(abs(ldx) < EPS) = EPS;
    ldy(abs(ldy) < EPS) = EPS;

    t1x = (-B.h(1) - lox) ./ ldx;   t2x = ( B.h(1) - lox) ./ ldx;
    t1y = (-B.h(2) - loy) ./ ldy;   t2y = ( B.h(2) - loy) ./ ldy;

    tmin = max(min(t1x, t2x), min(t1y, t2y));
    tmax = min(max(t1x, t2x), max(t1y, t2y));

    t = tmin;
    inside = tmin < 0;
    t(inside) = tmax(inside);          % sensor inside the box: exit face

    hit = (tmax >= max(tmin, 0)) & (t > 0) & (t < ranges);
    ranges(hit) = t(hit);
end

% ---------------- circles: ray/circle quadratic ----------------
for k = 1:numel(world.circles)
    C  = world.circles(k);
    ox = pos(1) - C.c(1);
    oy = pos(2) - C.c(2);

    b    = ox*cx + oy*sy;             % direction is unit length, so a == 1
    cq   = ox*ox + oy*oy - C.r^2;
    disc = b.*b - cq;

    ok   = disc >= 0;
    sq   = sqrt(max(disc, 0));
    t    = -b - sq;
    tfar = -b + sq;
    t(t < 0) = tfar(t < 0);           % sensor inside the circle

    hit = ok & (t > 0) & (t < ranges);
    ranges(hit) = t(hit);
end
end
