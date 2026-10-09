function [vx, vy] = nearest_mover_vel(q, movers)
%NEAREST_MOVER_VEL  Velocity of the mover nearest to query point q (1x2).
vx = 0; vy = 0;
if isempty(movers), return; end
best = inf; bestk = 0;
for k = 1:numel(movers)
    d = norm(q - movers(k).c) - movers(k).r;
    if d < best, best = d; bestk = k; end
end
if bestk > 0, vx = movers(bestk).v(1); vy = movers(bestk).v(2); end
end
