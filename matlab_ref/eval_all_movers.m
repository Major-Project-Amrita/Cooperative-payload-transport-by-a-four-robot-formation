function movers_t = eval_all_movers(movers, t)
%EVAL_ALL_MOVERS  Mover poses at absolute time t (t is absolute sim time).
movers_t = movers;
for k = 1:numel(movers_t)
    m           = movers_t(k);
    m.c         = eval_mover_pose(m, t);   % standalone file: single source of truth
    m.t         = t;
    movers_t(k) = m;
end
end
