function [vx, vy] = phase1_nearest_mover_vel(P, movers)
%PHASE1_NEAREST_MOVER_VEL  Velocity of the mover nearest to any robot.
vx = 0; vy = 0;
if isempty(movers), return; end
best = inf; bestk = 0;
for i = 1:size(P,1)
    for k = 1:numel(movers)
        d = norm(P(i,:) - movers(k).c) - movers(k).r;
        if d < best, best = d; bestk = k; end
    end
end
if bestk > 0, vx = movers(bestk).v(1); vy = movers(bestk).v(2); end
end
