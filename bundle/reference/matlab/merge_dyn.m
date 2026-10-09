function wm = merge_dyn(world)
%MERGE_DYN  Static world + current dynamic obstacles -> one world struct.
%
%   wm = MERGE_DYN(world)
%
%   Concatenates world.dyn_boxes / world.dyn_circles (maintained by
%   UPDATE_WORLD_DYN) onto world.boxes / world.circles and recomputes the
%   bounds. The returned struct is a drop-in for EVERY geometry helper:
%
%     world_dist(P, wm)                       exact signed distance
%     lidar_scan(pos, wm, nb, range)          synthetic /scan, occlusion incl.
%     pvfh_scan(pos, wm, ...)                 unchanged controller
%     find_escape_heading_sdf(..., wm, ...)   unchanged escape probe
%     formation_clearance(c, h, off, wm, ...) whole-body collision
%
%   That is the whole integration trick: dynamic obstacles are geometry, and
%   the geometry primitives are already exact and obstacle-agnostic. The one
%   thing callers must NOT do is cache wm across sim steps - rebuild it every
%   step after UPDATE_WORLD_DYN, or the LiDAR will report obstacles where
%   they used to be.
%
%   Missing dyn fields (e.g. a world straight from SDF2WORLD) are treated as
%   "no dynamic obstacles", so static runs are unaffected.
%
%   See also UPDATE_WORLD_DYN, SIMULATION8_DYNAMIC.

wm = world;
% No movers of a kind -> the static list unchanged. (The old fallback
% appended a dummy element and then deleted element 2 -- which removed a
% REAL static obstacle and left the empty dummy behind, so any world whose
% movers were all boxes, or all circles, crashed in dyn_bounds.)
if isfield(world, 'dyn_boxes') && ~isempty(world.dyn_boxes)
    wm.boxes = [world.boxes, world.dyn_boxes];
end
if isfield(world, 'dyn_circles') && ~isempty(world.dyn_circles)
    wm.circles = [world.circles, world.dyn_circles];
end
wm.bounds = dyn_bounds(wm.boxes, wm.circles, world.bounds);
end

% ------------------------------------------------------------------------
function b = dyn_bounds(boxes, circles, static_bounds)
% Tightest bounds over everything. Union with the static bounds so a mover
% outside the static footprint (never happens here, but cheap to be safe)
% does not shrink the plot or the feasibility grid.
xs = []; ys = [];
for i = 1:numel(boxes)
    B = boxes(i);
    R = [cos(B.yaw) -sin(B.yaw); sin(B.yaw) cos(B.yaw)];
    cn = [-B.h(1) -B.h(2); B.h(1) -B.h(2); B.h(1) B.h(2); -B.h(1) B.h(2)];
    cn = (R*cn')' + B.c;
    xs = [xs; cn(:,1)]; ys = [ys; cn(:,2)]; %#ok<AGROW>
end
for k = 1:numel(circles)
    C = circles(k);
    xs = [xs; C.c(1)-C.r; C.c(1)+C.r]; %#ok<AGROW>
    ys = [ys; C.c(2)-C.r; C.c(2)+C.r]; %#ok<AGROW>
end
if isempty(xs)
    b = static_bounds; return;
end
b = [min(static_bounds(1), min(xs)), max(static_bounds(2), max(xs)), ...
     min(static_bounds(3), min(ys)), max(static_bounds(4), max(ys))];
end
