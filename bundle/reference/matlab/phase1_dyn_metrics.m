function [gap_pre_min, gap_now_min] = phase1_dyn_metrics(pos_prev, pos_new, ...
                                                         movers, t_now, dt, lookahead)
%PHASE1_DYN_METRICS  Predicted/current mover gaps, min over robots.
%   Uses each robot's realised step velocity ((new-old)/dt), which is exactly
%   the velocity the t+dt commit logic assumed for it.
K = size(pos_new,1);
gap_pre_min = inf; gap_now_min = inf;
for i = 1:K
    v_i = (pos_new(i,:) - pos_prev(i,:)) / dt;
    [gp, gn] = predicted_min_mover_gap(pos_new(i,:), v_i, movers, ...
                                       t_now, dt, lookahead);
    gap_pre_min = min(gap_pre_min, gp);
    gap_now_min = min(gap_now_min, gn);
end
end
