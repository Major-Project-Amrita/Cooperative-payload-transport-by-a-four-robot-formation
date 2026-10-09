function g = mover_gap(q, movers)
%MOVER_GAP  Min distance from query points (Nx2) to mover surfaces.
g = inf;
for k = 1:numel(movers)
    m = movers(k);
    if strcmpi(m.shape, 'circle')
        dk = sqrt((q(:,1)-m.c(1)).^2 + (q(:,2)-m.c(2)).^2) - m.r;
    else
        dx = q(:,1) - m.c(1);  dy = q(:,2) - m.c(2);
        qx = abs(dx) - m.size(1)/2;
        qy = abs(dy) - m.size(2)/2;
        outside = sqrt(max(qx,0).^2 + max(qy,0).^2);
        inside  = min(max(qx, qy), 0);
        dk = outside + inside;
    end
    g = min(g, min(dk));
end
end
