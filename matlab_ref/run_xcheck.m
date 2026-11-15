function run_xcheck(outdir)
%RUN_XCHECK  Zero-noise per-step logs of all eight laws (sigma0 = 0), for the
%   step-by-step MATLAB-vs-Python cross-check (tools/deterministic_xcheck.py).
%   run_xcheck('logs/xcheck_fixedgate')
if nargin < 1, outdir = fullfile('logs', 'xcheck_fixedgate'); end
if exist(outdir, 'dir') ~= 7, mkdir(outdir); end
algos = {'apf', 'vfh', 'fgm', 'dwa', 'mpc', 'pvfh', 'pvfh_filt', 'pvfh_prob'};
for i = 1:numel(algos)
    run_x(algos{i}, outdir);
end
end

function run_x(a, outdir)
CFG_OVERRIDE = struct('algo', a, 'seed', 1, ...
    'world_file', 'swarm_world_dynamic.sdf', 'dynamic_json', 'dynamic_obstacles.json', ...
    'run_name', ['x_' a '_s0'], 'formation', 'square', 'grid_spacing', 1.18, ...
    'interactive', false, 'timesteps', 3000, 'viz', struct('enable', false), ...
    'log', struct('enable', true, 'file', fullfile(outdir, ['x_' a '_s0.csv'])));
CFG_OVERRIDE.vfh = struct('sense', struct('kind', 'lidar', 'sigma0', 0, 'k_rng', 0.03, ...
    'fov_deg', 60, 'max_range', inf, 'frame', 'body'), 'n_beams', 288);
Simulation8_dynamic; %#ok<NASGU>
end
