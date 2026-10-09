function [d, owner] = world_dist(P, world)
%WORLD_DIST  Exact signed distance from points to the nearest obstacle surface.
%
%   [d, owner] = WORLD_DIST(P, world)
%
%   P      Nx2 query points [x y].
%   world  struct from SDF2WORLD.
%   d      Nx1 signed distance in metres. NEGATIVE inside an obstacle,
%          POSITIVE in free space, 0 on the surface.
%   owner  Nx1 index of the nearest obstacle. Boxes are 1..numel(world.boxes);
%          circles continue from numel(world.boxes)+1.
%
%   This is the single geometric primitive the whole pipeline rests on:
%
%     collision test        d <= 0
%     inflated collision    d <= robot_radius
%     clearance metric      d itself
%     feasibility analysis  thresholding d over a grid
%
%   Because it is exact (not a rasterised approximation) and consistent
%   everywhere, a MATLAB "collision" means the same thing a Gazebo contact
%   means, which is what makes the two runs comparable.
%
%   Fully vectorised: cost is O(N x numObstacles) with no loops over points.
%
%   See also SDF2WORLD, LIDAR_SCAN, WORLD_FEASIBILITY.

if isempty(P)
    d = zeros(0,1); owner = zeros(0,1); return;
end
N  = size(P,1);
nB = numel(world.boxes);
nC = numel(world.circles);
M  = nB + nC;

if M == 0
    d = inf(N,1); owner = zeros(N,1); return;
end

D = inf(N, M);

for i = 1:nB
    B  = world.boxes(i);
    dx = P(:,1) - B.c(1);
    dy = P(:,2) - B.c(2);
    if B.yaw ~= 0
        c = cos(-B.yaw); s = sin(-B.yaw);
        lx = c*dx - s*dy;
        ly = s*dx + c*dy;
    else
        lx = dx; ly = dy;
    end
    qx = abs(lx) - B.h(1);
    qy = abs(ly) - B.h(2);
    % Standard exact box SDF: outside term + inside (negative) term.
    outside = sqrt(max(qx,0).^2 + max(qy,0).^2);
    inside  = min(max(qx, qy), 0);
    D(:,i)  = outside + inside;
end

for k = 1:nC
    C = world.circles(k);
    D(:, nB+k) = sqrt((P(:,1)-C.c(1)).^2 + (P(:,2)-C.c(2)).^2) - C.r;
end

[d, owner] = min(D, [], 2);
d = d(:); owner = owner(:);
end
