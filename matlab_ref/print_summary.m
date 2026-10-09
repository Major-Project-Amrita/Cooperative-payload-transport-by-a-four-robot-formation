function print_summary(S)
    fprintf('\n===== RUN SUMMARY =====\n');
    fprintf('  reached goal      : %d\n',        S.reached_goal);
    fprintf('  total sim time    : %.1f s\n',    S.total_time);
    fprintf('  transport time    : %.1f s\n',    S.transport_time);
    fprintf('  path length       : %.2f m\n',    S.path_length);
    fprintf('  path efficiency   : %.3f  (1.0 = straight line)\n', S.path_efficiency);
    fprintf('  min clearance     : %.3f m\n',    S.min_clearance);
    fprintf('  mean clearance    : %.3f m\n',    S.mean_clearance);
    fprintf('  collision steps   : %d  (static %g, mover %g)\n', S.collisions, S.coll_static, S.coll_mover);
    fprintf('  gate rejections   : %g   escape probes: %g\n', S.n_reject, S.n_escape);
    if isfield(S, 'max_robot_speed'), fprintf('  peak robot speed  : %.2f m/s  (needed by the outer robot)\n', S.max_robot_speed); end
    fprintf('  total turning     : %.2f rad\n',  S.total_turning);
    fprintf('  mean |turn rate|  : %.3f rad/s\n',S.mean_turn_rate);
    fprintf('  final dist to goal: %.3f m\n',    S.final_dist);
    fprintf('=======================\n\n');
end
