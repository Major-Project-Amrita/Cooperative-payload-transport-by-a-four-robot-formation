function [d, owner] = world_dist_dyn(P, world, movers)
%WORLD_DIST_DYN  Exact signed distance, static world + dynamic obstacles.
%
%   [d, owner] = WORLD_DIST_DYN(P, world, movers)
%
%   Same contract as WORLD_DIST (exact signed distance, negative inside),
%   extended over the movers. The owner index is:
%       1..numel(world.boxes)+numel(world.circles)   static (as WORLD_DIST)
%       -k                                           movers(k)
%
%   The mover term is the same exact primitive WORLD_DIST uses for static
%   boxes/circles (slab/SDF for boxes, quadratic for circles), evaluated at
%   the mover's CURRENT centre. Movers are axis-aligned; a future contract
%   with a yaw field only needs the rotation added here and in
%   LIDAR_SCAN_DYN - both already have the rotation branch in place.
%
%   See also WORLD_DIST, LIDAR_SCAN_DYN, FORMATION_CLEARANCE_DYN.

if isempty(P)
    d = zeros(0,1); owner = zeros(0,1); return;
end
N   = size(P,1);
nB  = numel(world.boxes);
nC  = numel(world.circles);
nM  = numel(movers);

% ---- static part (indices 1..nB+nC) ----
[D, owner] = world_dist(P, world);

if nM == 0, d = D(:); owner = owner(:); return; end   % was d(:): undefined

Dm = inf(N, nM);
for k = 1:nM
    m  = movers(k);
    dx = P(:,1) - m.c(1);
    dy = P(:,2) - m.c(2);
    if strcmpi(m.shape, 'circle')
        Dm(:,k) = sqrt(dx.^2 + dy.^2) - m.r;
    else
        yaw = 0;
        if isfield(m, 'yaw'), yaw = m.yaw; end
        if yaw ~= 0
            c  = cos(-yaw); s = sin(-yaw);
            lx = c*dx - s*dy;
            ly = s*dx + c*dy;
        else
            lx = dx; ly = dy;
        end
        qx      = abs(lx) - m.size(1)/2;
        qy      = abs(ly) - m.size(2)/2;
        outside = sqrt(max(qx,0).^2 + max(qy,0).^2);
        inside  = min(max(qx, qy), 0);
        Dm(:,k) = outside + inside;
    end
end

[Dall, owner_all] = min([D, Dm], [], 2);
neg = owner_all > nB + nC;               % a mover is nearest
owner = zeros(N,1);
owner(~neg) = owner_all(~neg);
owner(neg)  = -(owner_all(neg) - nB - nC);
d = Dall;
d = d(:); owner = owner(:);
end
