function [gap_pre, gap_now] = predicted_min_mover_gap(q, v_self, movers, t_now, dt, lookahead)
%PREDICTED_MIN_MOVER_GAP  Worst-case gap to any mover over the lookahead.
%   Samples horizons T = dt..lookahead; at each, the mover is evaluated at
%   t_now+T from the contract and the query is projected by v_self*T. The
%   self-displacement is SUBTRACTED from the gap, so head-on geometry (the
%   worst case for a rigid block) is what sets the reported number. With no
%   movers both outputs are 99 (a sentinel meaning "nothing moving nearby",
%   keeps logs finite).
gap_pre = 99; gap_now = 99;
if isempty(movers), return; end
gap_now = mover_gap(q, movers);
worst = gap_now;
T = dt;
while T <= lookahead + 1e-9
    mf = eval_all_movers(movers, t_now + T);
    g  = mover_gap(q + v_self*T, mf) - norm(v_self)*T;
    worst = min(worst, g);
    T = T + dt;
end
gap_pre = worst;
end
