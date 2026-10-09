function ext = obstacle_extents(world)
%OBSTACLE_EXTENTS  Characteristic size of every obstacle, in metres.
%
%   ext = OBSTACLE_EXTENTS(world)
%
%   ext(k) is the largest footprint dimension of obstacle k, indexed exactly
%   as WORLD_DIST returns its owner: boxes first, then circles.
%
%   Used to decide which obstacles the convoy/queueing behaviour should apply
%   to. Queueing up single file makes sense for rounding a compact obstacle
%   like a pillar or a pallet. It makes no sense for a 30 m wall or a 26 m
%   shelf rack: in a warehouse aisle every robot is permanently within a few
%   metres of one of those, so a distance-only trigger latches on at spawn and
%   never releases, and the swarm trails a leader instead of taking its slots.
%
%   See also WORLD_DIST, SDF2WORLD.

nB  = numel(world.boxes);
nC  = numel(world.circles);
ext = zeros(nB + nC, 1);

for i = 1:nB
    ext(i) = 2 * max(world.boxes(i).h);
end
for k = 1:nC
    ext(nB + k) = 2 * world.circles(k).r;
end
end
