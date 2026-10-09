function pt = point_along_trail_behind(trail, arc_dist, leader_pos, fallback_from_pos)
    if size(trail,1) < 2
        dir = leader_pos - fallback_from_pos;
        dn  = norm(dir);
        if dn < 1e-6, pt = leader_pos - [arc_dist 0];
        else,         pt = leader_pos - arc_dist*(dir/dn); end
        return;
    end
    remaining = arc_dist;
    pt = trail(end,:);
    for k = size(trail,1):-1:2
        seg = trail(k-1,:) - trail(k,:);
        L   = norm(seg);
        if L < 1e-9, continue; end
        if remaining <= L, pt = trail(k,:) + (remaining/L)*seg; return; end
        remaining = remaining - L;
        pt = trail(k-1,:);
    end
end
