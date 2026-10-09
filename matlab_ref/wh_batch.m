function wh_batch(part)
A = {'apf','vfh','fgm','dwa','mpc','pvfh','pvfh_filt','pvfh_prob'};
J = {};
for a = A, J{end+1} = {a{1}, 0, 1, sprintf('wh_%s_s0', a{1})}; end
for sd = 1:5, for a = A, J{end+1} = {a{1}, 0.02, sd, sprintf('wh_%s_sg002_s%d', a{1}, sd)}; end, end
for k = part:2:numel(J)
    j = J{k};
    if exist(fullfile('logs', [j{4} '.csv']), 'file') == 2, continue; end
    try, run_wh(j{1}, j{2}, j{3}, j{4}); catch e, fprintf('RUN FAILED %s: %s\n', j{4}, e.message); end
end
end
