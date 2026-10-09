function sf = mover_speed_frac(movers, V)
%MOVER_SPEED_FRAC  Fastest mover's speed relative to the formation's V.
sf = 0;
for k = 1:numel(movers)
    sf = max(sf, norm(movers(k).v) / V);
end
end
