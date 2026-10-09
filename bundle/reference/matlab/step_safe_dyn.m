function safe = step_safe_dyn(cand, cand_heading, world, movers, t_now, dt, ...
                              robot_radius, mode, offsets)
%STEP_SAFE_DYN  True if the body fits at cand BOTH now and at t_now+dt.
%   The t+dt test is what stops the formation from stepping into geometry a
%   mover is about to occupy. Movers at t_now+dt come from the same shared
%   contract, so both stacks agree on the future.
movers_tp = eval_all_movers(movers, t_now + dt);
[~, hit_now]  = formation_clearance_dyn(cand, cand_heading, offsets, world, ...
                                        robot_radius, mode, movers);
[~, hit_next] = formation_clearance_dyn(cand, cand_heading, offsets, world, ...
                                        robot_radius, mode, movers_tp);
safe = ~hit_now && ~hit_next;
end
