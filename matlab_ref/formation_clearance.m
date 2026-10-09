function [clr, tf, pts] = formation_clearance(center, heading, offsets, world, robot_radius, mode, edge_step)
%FORMATION_CLEARANCE  Clearance and collision state of a whole rigid formation.
%
%   [clr, tf, pts] = FORMATION_CLEARANCE(center, heading, offsets, world, ...
%                                        robot_radius, mode, edge_step)
%
%   center        1x2 formation centre.
%   heading       formation yaw, rad.
%   offsets       Kx2 slot offsets in the body frame.
%   robot_radius  physical robot radius, metres.
%   mode          'payload' (default) the block is solid - it carries a load,
%                             so anything inside the square is a collision.
%                 'frame'    only the robots and the links between them are
%                             solid; the interior is open.
%                 'slots'    robots only. Matches the ORIGINAL script, kept so
%                             you can reproduce your existing results.
%   edge_step     (0.15) sampling step along the body outline, metres.
%
%   clr   minimum signed clearance of the body to any obstacle (m).
%   tf    true if the formation is in collision (clr <= robot_radius, or, in
%         'payload' mode, an obstacle lies wholly inside the block).
%   pts   sampled outline points, for plotting.
%
%   TWO THINGS THE ORIGINAL CHECK MISSED
%   1. Edges. The original tested only the slot positions - four points on a
%      4 m square. Every pillar in these worlds is 0.8-0.9 m across, so one can
%      sit between two slots and pass clean through the formation undetected,
%      while Gazebo registers a hard contact.
%   2. Interior. Phase 2 is rigid TRANSPORT: there is a load spanning the
%      block. An obstacle inside the square hits that load. Boundary sampling
%      alone cannot see it, so 'payload' mode adds an exact separating-axis
%      test of the whole block against every obstacle.
%
%   The SAT test is exact, not sampled, and costs O(numObstacles).
%
%   See also WORLD_DIST, SDF2WORLD.

if nargin < 6 || isempty(mode),      mode      = 'payload'; end
if nargin < 7 || isempty(edge_step), edge_step = 0.15;      end

R    = [cos(heading) -sin(heading); sin(heading) cos(heading)];
slot = (R*offsets')' + center;
K    = size(slot,1);
n    = round(sqrt(K));

% ---------------- outline sample points ----------------
pts = slot;
if ~strcmpi(mode, 'slots')
    if n*n == K && n > 1
        G    = reshape(1:K, n, n);      % offsets come from meshgrid(0:n-1,0:n-1)
        ring = [G(1,:), G(2:end,end)', fliplr(G(end,1:end-1)), flipud(G(2:end-1,1))'];
        ring = [ring, ring(1)];
        for k = 1:numel(ring)-1
            pts = [pts; sample_seg(slot(ring(k),:), slot(ring(k+1),:), edge_step)]; %#ok<AGROW>
        end
    elseif K == 2
        pts = [pts; sample_seg(slot(1,:), slot(2,:), edge_step)];
    end
end

clr = min(world_dist(pts, world));
tf  = clr <= robot_radius;

% ---------------- payload mode: exact block-vs-obstacle overlap ----------------
if strcmpi(mode, 'payload') && K > 1
    ext = max(abs(offsets), [], 1) + robot_radius;   % inflated block half-extents
    if block_overlaps(center, heading, ext, world)
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

function tf = block_overlaps(c, yaw, ext, world)
%BLOCK_OVERLAPS  Separating-axis test: oriented block vs every world obstacle.
tf = true;
ca = cos(yaw); sa = sin(yaw);
Aaxes = [ca sa; -sa ca];            % rows are the block's local axes

for i = 1:numel(world.boxes)
    B  = world.boxes(i);
    cb = cos(B.yaw); sb = sin(B.yaw);
    Baxes = [cb sb; -sb cb];
    d = B.c - c;
    sep = false;
    for ax = [Aaxes; Baxes]'
        ax = ax';
        rA = ext(1)*abs(ax*Aaxes(1,:)') + ext(2)*abs(ax*Aaxes(2,:)');
        rB = B.h(1)*abs(ax*Baxes(1,:)') + B.h(2)*abs(ax*Baxes(2,:)');
        if abs(ax*d') > rA + rB
            sep = true; break;
        end
    end
    if ~sep, return; end
end

for k = 1:numel(world.circles)
    C = world.circles(k);
    d = C.c - c;
    lx = ca*d(1) + sa*d(2);         % circle centre in block frame
    ly = -sa*d(1) + ca*d(2);
    qx = max(abs(lx) - ext(1), 0);
    qy = max(abs(ly) - ext(2), 0);
    if (qx*qx + qy*qy) <= C.r^2, return; end
end

tf = false;
end
