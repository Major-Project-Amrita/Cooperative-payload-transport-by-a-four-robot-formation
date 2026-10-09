function S = summarize_run(LOG, cfg, reached)
%SUMMARIZE_RUN  Per-run aggregate metrics (phase 2 = rigid transport).
%
%   Clearance convention (unchanged): LOG.clr is the distance from the
%   body's sample points to the nearest surface, so CONTACT is
%   clr <= robot_radius; the surface gap is clr - robot_radius.
%
%   collisions        steps in contact with anything
%   coll_static       ... with static geometry. The commit gate checks the
%                     whole pose (translation AND rotation), so this must be
%                     0; a non-zero value flags a gate bug, not a result.
%   coll_mover        ... caused by a mover (kinematic movers never yield and
%                     can drive into a body that is holding still)
%   n_reject          steps where the safety gate refused the commanded
%                     motion (translation blocked). The gate uses ground-
%                     truth geometry, so this counts the collisions the
%                     steering law WOULD have caused: the safety metric.
%   n_escape          escape probes fired (stuck counter or progress window)
    p2 = LOG.phase(:) == 2;
    S.reached_goal    = reached;
    S.total_time      = LOG.t(end);
    S.transport_time  = sum(p2)*cfg.dt;
    S.path_length     = sum(sqrt(diff(LOG.lx(p2)).^2 + diff(LOG.ly(p2)).^2));
    sx = LOG.lx(find(p2,1)); sy = LOG.ly(find(p2,1));
    ex = LOG.lx(end);        ey = LOG.ly(end);
    straight          = hypot(ex-sx, ey-sy);
    S.path_efficiency = straight / max(S.path_length, eps);
    clr               = LOG.clr(:);
    S.min_clearance   = min(clr(p2));
    S.mean_clearance  = mean(clr(p2));
    hit               = p2 & clr <= cfg.robot_radius;
    S.collisions      = sum(hit);
    S.coll_static     = NaN;  S.coll_mover = NaN;
    if isfield(LOG, 'clr_s') && numel(LOG.clr_s) == numel(clr)
        hs            = p2 & LOG.clr_s(:) <= cfg.robot_radius;
        S.coll_static = sum(hs);
        S.coll_mover  = sum(hit & ~hs);
    end
    S.n_reject = NaN;  S.n_escape = NaN;
    if isfield(LOG, 'rej') && numel(LOG.rej) == numel(clr)
        S.n_reject = sum(p2 & LOG.rej(:) > 0);
    end
    if isfield(LOG, 'esc') && numel(LOG.esc) == numel(clr)
        e = LOG.esc(:) > 0;
        S.n_escape = sum(p2 & e & ~[false; e(1:end-1)]);   % rising edges
    end
    % Peak speed any robot would need to follow its slot (docking phase
    % excluded): v + |w|*r_c at the outer slot. The feasibility number.
    S.max_robot_speed = NaN;
    i2 = find(p2);
    if numel(i2) > 70
        i2 = i2(61:end);
        vx = diff(LOG.rx(i2, :)) / cfg.dt;  vy = diff(LOG.ry(i2, :)) / cfg.dt;
        sp = sqrt(vx.^2 + vy.^2);
        S.max_robot_speed = max(sp(:));
    end
    comp = [];
    if isfield(LOG, 'comp') && numel(LOG.comp) == numel(clr)
        comp = LOG.comp(p2 & LOG.comp(:) > 0);
    end
    % Phase-2 steering cost only: phase-1 entries are summed over K robots
    % and escape steps carry 0, so the old all-steps mean mixed three things.
    if isempty(comp), S.comp_ms_mean = NaN; else, S.comp_ms_mean = mean(comp); end
    dh                = abs(diff(unwrap(LOG.lh(p2))));
    S.total_turning   = sum(dh);
    S.mean_turn_rate  = mean(dh)/cfg.dt;
    S.max_turn_rate   = max(dh)/cfg.dt;
    S.final_dist      = LOG.dist(end);
end
