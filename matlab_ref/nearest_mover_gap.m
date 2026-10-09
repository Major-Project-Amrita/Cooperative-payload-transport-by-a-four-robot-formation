function g = nearest_mover_gap(P, movers)
%NEAREST_MOVER_GAP  Min gap over all robots (P is Nx2). inf with no movers.
if isempty(movers), g = inf; return; end
g = mover_gap(P, movers);
end
