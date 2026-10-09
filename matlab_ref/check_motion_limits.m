function check_motion_limits(seeds)
%CHECK_MOTION_LIMITS  Quick A/B of the load-carrying motion limits (cfg.kin).
%
%   check_motion_limits            seeds 1:3
%   check_motion_limits(1:5)
%
%   Runs pvfh_prob / pvfh / vfh on the warehouse world with the limits ON
%   and OFF (no animation) and prints one line per run, so the effect of the
%   limits can be judged before the Monte Carlo grid is started. Results are
%   also written to logs/check_motion_limits.csv; per-step logs go to
%   logs/chk_*.csv.
if nargin < 1, seeds = 1:3; end
algos = {'pvfh_prob', 'pvfh', 'vfh'};
fid = fopen(fullfile('logs', 'check_motion_limits.csv'), 'w');
hdr = 'algo,seed,kin,reached,time,reject,coll_static,coll_mover,min_gap,max_corner_speed';
fprintf(fid, '%s\n', hdr);
fprintf('%s\n', strrep(hdr, ',', '  '));
for a = 1:numel(algos)
    for sd = seeds
        for kin = [1 0]
            r = one(algos{a}, sd, kin);
            line = sprintf('%s,%d,%d,%d,%.1f,%d,%d,%d,%.3f,%.2f', algos{a}, sd, kin, ...
                r.reached, r.time, r.reject, r.cs, r.cm, r.gap, r.vmax);
            fprintf(fid, '%s\n', line);
            fprintf('%s\n', strrep(line, ',', '  '));
        end
    end
end
fclose(fid);
fprintf('written: logs/check_motion_limits.csv\n');
end

function r = one(algo, seed, kin)
name = sprintf('chk_%s_s%d_k%d', algo, seed, kin);
CFG_OVERRIDE = struct('algo', algo, 'seed', seed, 'run_name', name, ...
    'timesteps', 3000, 'viz', struct('enable', false), ...
    'kin', struct('enable', logical(kin))); %#ok<NASGU>
evalc('Simulation8_dynamic');             % quiet; leaves S, cfg here
r = struct('reached', S.reached_goal, 'time', S.total_time, 'reject', S.n_reject, ...
    'cs', S.coll_static, 'cm', S.coll_mover, ...
    'gap', S.min_clearance - cfg.robot_radius, 'vmax', S.max_robot_speed);
end
