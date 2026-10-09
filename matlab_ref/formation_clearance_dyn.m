function [clr, tf, pts] = formation_clearance_dyn(center, heading, offsets, world, ...
                                     robot_radius, mode, movers, edge_step)
%FORMATION_CLEARANCE_DYN  Whole-body clearance/collision vs static + dynamic.
%
%   [clr, tf, pts] = FORMATION_CLEARANCE_DYN(center, heading, offsets, world, ...
%                                            robot_radius, mode, movers, edge_step)
%
%   FORMATION_CLEARANCE extended over the movers: same three fidelity modes,
%   same outline sampling, same exact separating-axis test for 'payload' mode
%   - with the movers added to both the sampled-distance term (via
%   WORLD_DIST_DYN) and the SAT overlap test. A mover parked anywhere inside
%   the formation's footprint registers as a collision, exactly like a static
%   pillar would.
%
%   The 'payload' interior test is the one that matters here: the formation is
%   carrying a load, so a mover sliding into the block interior (not just the
%   slot points or the outline) must be seen.
%
%   See also FORMATION_CLEARANCE, WORLD_DIST_DYN, SIMULATION8_DYNAMIC.

if nargin < 6 || isempty(mode),      mode      = 'payload'; end
if nargin < 7 || isempty(movers),    movers    = struct([]); end
if nargin < 8 || isempty(edge_step), edge_step = 0.15;      end

R    = [cos(heading) -sin(heading); sin(heading) cos(heading)];
slot = (R*offsets')' + center;
K    = size(slot,1);
n    = round(sqrt(K));

% ---------------- outline sample points ----------------
pts = slot;
if ~strcmpi(mode, 'slots')
    if n*n == K && n > 1
        G    = reshape(1:K, n, n);
        ring = [G(1,:), G(2:end,end)', fliplr(G(end,1:end-1)), flipud(G(2:end-1,1))'];
        ring = [ring, ring(1)];
        for k = 1:numel(ring)-1
            pts = [pts; sample_seg(slot(ring(k),:), slot(ring(k+1),:), edge_step)]; %#ok<AGROW>
        end
    elseif K == 2
        pts = [pts; sample_seg(slot(1,:), slot(2,:), edge_step)];
    end
end

clr = min(world_dist_dyn(pts, world, movers));
tf  = clr <= robot_radius;

% ---------------- payload mode: exact hull-vs-obstacle overlap ----------------
% The carried body is the CONVEX HULL of the slots with corners rounded by
% robot_radius. For the square grid the hull equals the old bounding box
% (bit-identical decisions); for line/triangle/diamond shapes it removes the
% bounding-box phantom corners.
if strcmpi(mode, 'payload') && K > 1
    if K > 2
        try
            idx = convhull(slot(:,1), slot(:,2));
            hull_w = slot(idx(1:end-1), :);  % convhull repeats the first vertex
        catch
            % Collinear slots (the 'line' formation): the hull is degenerate
            % and qhull refuses it. The exact body is then just the segment
            % between the two farthest slots.
            bd = -1; pa = 1; pb = 2;
            for a = 1:K-1
                for b2 = a+1:K
                    dd = norm(slot(a,:) - slot(b2,:));
                    if dd > bd, bd = dd; pa = a; pb = b2; end
                end
            end
            hull_w = slot([pa pb], :);
        end
    else
        hull_w = slot;
    end
    if hull_overlaps_dyn(hull_w, robot_radius, world, movers)
        tf  = true;
        clr = min(clr, 0);
    end
end
end

% ------------------------------------------------------------------------
function p = sample_seg(a, b, step)
p = zeros(0,2);
L = norm(b-a);
if L > step
    m   = ceil(L/step);
    lam = (1:m-1)'/m;
    p   = a + lam.*(b-a);
end
end

function tf = hull_overlaps_dyn(hull_w, r, world, movers)
%HULL_OVERLAPS_DYN  Separating-axis test: conv(hull)+disk(r) vs obstacles.
%   Axis set = hull edge normals + obstacle edge normals + vertex-vertex
%   differences, which is exact for the Minkowski-rounded bodies (the robot
%   radius acts as disk inflation on the hull).
tf = true;
H = hull_w;
n = size(H, 1);

% ---- candidate separating axes (unit row vectors) ----
axes_v = zeros(0, 2);
for i = 1:n
    e = H(mod(i, n) + 1, :) - H(i, :);
    L = norm(e);
    if L > 1e-9
        axes_v(end+1, :) = [-e(2), e(1)] / L; %#ok<AGROW>
    end
end

% ---- static boxes: 4 corners as vertices ----
for i = 1:numel(world.boxes)
    B  = world.boxes(i);
    cb = cos(B.yaw); sb = sin(B.yaw);
    corners = ([cb -sb; sb cb] * [B.h(1)  B.h(1) -B.h(1) -B.h(1); ...
                                  B.h(2) -B.h(2)  B.h(2) -B.h(2)])' + repmat(B.c, 4, 1);
    P = corners';
    if ~hull_poly_separated(H, r, P, axes_v), return; end
end

% ---- static circles: single vertex + growth ----
for k = 1:numel(world.circles)
    C = world.circles(k);
    if ~hull_poly_separated(H, r, C.c(:), axes_v, C.r), return; end
end

% ---- movers ----
for k = 1:numel(movers)
    m = movers(k);
    if strcmpi(m.shape, 'circle')
        if ~hull_poly_separated(H, r, m.c(:), axes_v, m.r), return; end
    else
        yaw_m = 0;
        if isfield(m, 'yaw'), yaw_m = m.yaw; end
        cm = cos(yaw_m); sm = sin(yaw_m);
        corners = ([cm -sm; sm cm] * [m.size(1)/2  m.size(1)/2 -m.size(1)/2 -m.size(1)/2; ...
                                      m.size(2)/2 -m.size(2)/2  m.size(2)/2 -m.size(2)/2])' + repmat(m.c, 4, 1);
        P = corners';
        if ~hull_poly_separated(H, r, P, axes_v), return; end
    end
end

tf = false;
end

function tf = hull_poly_separated(H, r, P, axes_v, growth)
%HULL_POLY_SEPARATED  One SAT: conv(H)+disk(r) vs conv(P)+disk(growth).
%   P is (2,m) vertex matrix; growth inflates P along every axis (circles).
if nargin < 5, growth = 0; end
m = size(P, 2);
ax = axes_v;
for j = 1:m
    e = P(:, mod(j, m) + 1) - P(:, j);
    L = norm(e);
    if L > 1e-9
        ax(end+1, :) = [-e(2), e(1)] / L; %#ok<AGROW>
    end
end
for i = 1:size(H, 1)
    for j = 1:m
        d = H(i, :) - P(:, j)';
        L = norm(d);
        if L > 1e-9
            ax(end+1, :) = d / L; %#ok<AGROW>
        end
    end
end
for k = 1:size(ax, 1)
    pa = H * ax(k, :)';
    pb = P' * ax(k, :)';
    if min(pa) - r > max(pb) + growth || min(pb) - growth > max(pa) + r
        tf = true;
        return;
    end
end
tf = false;
end
