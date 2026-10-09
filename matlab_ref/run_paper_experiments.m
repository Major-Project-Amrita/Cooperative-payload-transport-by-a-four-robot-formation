function run_paper_experiments(mode, shard, nshards, nseeds)
%RUN_PAPER_EXPERIMENTS  The paper's Monte Carlo study (E1-E9), MATLAB stack.
%
%   run_paper_experiments()                 full grid, one process
%   run_paper_experiments('smoke')          1 seed, one cell per experiment
%   run_paper_experiments('quick')          5 seeds everywhere (half budget)
%   run_paper_experiments('full', k, N)     shard k of N (run N processes,
%                                           k = 1..N, then 'merge')
%   run_paper_experiments('merge')          parts/*.csv -> results_all.csv
%   run_paper_experiments('list')           print the manifest, run nothing
%   run_paper_experiments('e2')             one experiment only ('e1'..'e14')
%   run_paper_experiments(mode, k, N, S)    override the seed count
%
%   matlab/run_grid_parallel.sh launches the shards and the merge for you.
%
%  EXPERIMENTS (dynamic warehouse unless stated; every algorithm consumes
%  the same seeded noisy-scan front end, see STEERING_DISPATCH):
%   E1  algorithm comparison   8 laws: apf vfh fgm dwa mpc pvfh pvfh_filt pvfh_prob
%   E2  noise ablation         vfh / pvfh / pvfh_filt / pvfh_prob x sigma0 (2x seeds)
%   E3  beam sweep             pvfh / pvfh_prob x {288,64,16,8} beams
%   E5  formation x algorithm  4 shapes x 7 laws at s = 0.60 m
%       (E4, "formation shapes", is the pvfh_prob row of E5 -- reported
%        from the same runs instead of re-running identical configs)
%   E6  matched sensor         lidar-288 / tof-16 / tof-8 x sigma0 x 3 laws
%   E7  second environments    plaza + maze x 7 laws x sigma0 {0.02,0.20}
%   E8  traffic speed          mover speed x{0.5,1,1.5,2} x 5 laws
%   E9  anti-deadlock ablation line formation, layer OFF (ON = E5 rows)
%   E10 localisation error     pose-estimate noise, 4 levels x 4 laws
%   E11 traffic density        0 / 1 / 3 / 6 movers x 5 laws
%   E12 command-update loss    drop probability {0,.1,.3,.5} x 4 laws
%   E13 randomised layouts     10 generated worlds x 7 laws x sigma0 x seeds
%   E14 load-feasible motion   motion limits (cfg.kin) ON x 5 laws (OFF = E1)
%
%  OUTPUT (all under matlab/logs/paper/, whatever the start directory):
%   parts/part_<k>of<N>.csv   crash-safe checkpoint, one row per run
%   results_all.csv           merged long-format table (one row per run)
%   runs/<run_name>.csv       per-step log of every run (figures need them)
%
%  Identical configurations shared by two experiments (e.g. the E6 lidar
%  cells and the E2 cells at the same sigma0) are simulated ONCE and the row
%  is emitted under both labels: equality is by construction, not by luck.
%
%  Figures, LaTeX tables and the results log are produced by
%      python3 matlab/make_paper_figures.py
%  from results_all.csv + runs/ -- nothing in the report is typed by hand.

if nargin < 1 || isempty(mode), mode = 'full'; end
if nargin < 2 || isempty(shard), shard = 1; end
if nargin < 3 || isempty(nshards), nshards = 1; end
if nargin < 4, nseeds = []; end
if ischar(shard), shard = str2double(shard); end
if ischar(nshards), nshards = str2double(nshards); end
if ischar(nseeds), nseeds = str2double(nseeds); end
mode = lower(mode);

ROOT   = fileparts(mfilename('fullpath'));        % .../swarm_matlab/matlab
LOGDIR = fullfile(ROOT, 'logs', 'paper');
PARTS  = fullfile(LOGDIR, 'parts');
RUNS   = fullfile(LOGDIR, 'runs');
mk(LOGDIR); mk(PARTS); mk(RUNS);

if strcmp(mode, 'merge')
    merge_parts(PARTS, fullfile(LOGDIR, 'results_all.csv'));
    return;
end

% Simulation8_dynamic resolves the world / contract files by relative name.
old_dir = cd(ROOT);
restore = onCleanup(@() cd(old_dir)); %#ok<NASGU>

J = build_jobs(mode, nseeds);
if strcmp(mode, 'list')
    for i = 1:numel(J), fprintf('%4d  %-3s %s\n', i, J(i).exp, J(i).key); end
    fprintf('%d jobs, %d unique configurations\n', numel(J), ...
            numel(unique({J.key})));
    return;
end

% ---- one simulation per UNIQUE configuration, sharded by configuration ----
keys = {J.key};
[ukeys, first] = unique(keys, 'stable');
part = fullfile(PARTS, sprintf('part_%dof%d.csv', shard, nshards));
done_keys = read_done_keys(PARTS);
mine = find(mod((1:numel(ukeys)) - 1, nshards) == shard - 1);
fprintf('RUN_PAPER_EXPERIMENTS[%s] shard %d/%d: %d jobs, %d unique configs, %d in this shard\n', ...
        mode, shard, nshards, numel(J), numel(ukeys), numel(mine));

t_all = tic;
n_run = 0;
for ii = 1:numel(mine)
    u = mine(ii);
    twins = find(strcmp(keys, ukeys{u}));
    todo  = twins(~ismember(arrayfun(@(j) [J(j).exp '|' J(j).key], twins, ...
                  'UniformOutput', false), done_keys));
    if isempty(todo)
        fprintf('[%d/%d] %s  (checkpointed, skipping)\n', ii, numel(mine), ukeys{u});
        continue;
    end
    j0 = J(first(u));
    fprintf('[%d/%d] %s\n', ii, numel(mine), ukeys{u});
    t0 = tic;
    r  = run_one(j0, RUNS);
    n_run = n_run + 1;
    fprintf('    reached=%d reject=%g coll=%g minclr=%.3f  (%.0f s wall)\n', ...
            r.reached, r.n_reject, r.collisions, r.min_clear, toc(t0));
    for j = todo
        r.exp = J(j).exp;
        append_row(part, r);
    end
end
fprintf('shard %d/%d: %d simulations in %.0f s\n', shard, nshards, n_run, toc(t_all));

if nshards == 1
    merge_parts(PARTS, fullfile(LOGDIR, 'results_all.csv'));
    fprintf('\nNext: python3 matlab/make_paper_figures.py   (figures + tables + results log)\n');
end
end

%% ========================================================================
function J = build_jobs(mode, nseeds)
%BUILD_JOBS  The experiment manifest as a struct array (one entry per row).
ALL6  = {'apf', 'vfh', 'fgm', 'dwa', 'mpc', 'pvfh', 'pvfh_filt', 'pvfh_prob'};   % all laws
LADDER = {'vfh', 'pvfh', 'pvfh_filt', 'pvfh_prob'};   % horizon -> filter -> kernel
FORMS = {'square', 'diamond', 'triangle', 'line'};
SIG   = [0.02 0.05 0.10 0.20 0.40];
W0 = 'swarm_world_dynamic.sdf';  C0 = 'dynamic_obstacles.json';

n = 10;                                   % seeds per cell
if strcmp(mode, 'quick'), n = 5; end
if strcmp(mode, 'smoke'), n = 1; end
if ~isempty(nseeds), n = nseeds; end
seeds  = 1:n;
seeds2 = 1:2*n;                           % E2: the small-effect cells
if strcmp(mode, 'smoke'), seeds = 7; seeds2 = 7; end

only = '';
if numel(mode) >= 2 && mode(1) == 'e' && ~isnan(str2double(mode(2:end))), only = upper(mode); end
smoke = strcmp(mode, 'smoke');

J = struct('exp', {}, 'algo', {}, 'kind', {}, 'sigma0', {}, 'beams', {}, ...
           'seed', {}, 'formation', {}, 'spacing', {}, 'world', {}, ...
           'contract', {}, 'mover_scale', {}, 'adl', {}, 'loc', {}, ...
           'drop', {}, 'nmov', {}, 'kin', {}, 'key', {});

    function add(ex, algo, kind, sg, nb, sd, form, sp, w, c, ms, adl, loc, drop, nmov, kin)
        % loc: localisation sigma_pos (m; sigma_yaw = loc * 20 deg/m);
        % drop: command-loss probability; nmov: movers used (-1 = all)
        if nargin < 13, loc = 0; end
        if nargin < 14, drop = 0; end
        if nargin < 15, nmov = -1; end
        if nargin < 16, kin = 0; end     % load-carrying motion limits on/off
        if ~isempty(only) && ~strcmp(only, ex), return; end
        % NB: a nested function shares every non-argument variable with
        % its parent, so this must not be called k (the E6 loop index).
        cfg_key = sprintf('%s|%s|%g|%d|s%d|%s|%g|%s|%s|m%g|a%d|l%g|d%g|n%d|k%d', algo, ...
                    kind, sg, nb, sd, form, sp, strrep(w, '.sdf', ''), ...
                    strrep(c, '.json', ''), ms, adl, loc, drop, nmov, kin);
        J(end+1) = struct('exp', ex, 'algo', algo, 'kind', kind, ...
            'sigma0', sg, 'beams', nb, 'seed', sd, 'formation', form, ...
            'spacing', sp, 'world', w, 'contract', c, 'mover_scale', ms, ...
            'adl', adl, 'loc', loc, 'drop', drop, 'nmov', nmov, 'kin', kin, ...
            'key', cfg_key); %#ok<AGROW>
    end

% E1 -- six steering laws, precise lidar
A = ALL6;  if smoke, A = {'pvfh_prob', 'dwa', 'fgm'}; end
for a = A, for sd = seeds
    add('E1', a{1}, 'lidar', 0.02, 288, sd, 'square', 1.18, W0, C0, 1, 1);
end, end

% E2 -- noise ablation over the ablation ladder (double seeds)
S2 = SIG;  if smoke, S2 = [0.02 0.40]; end
A = LADDER;  if smoke, A = {'vfh', 'pvfh_prob'}; end
for a = A, for sg = S2, for sd = seeds2
    add('E2', a{1}, 'lidar', sg, 288, sd, 'square', 1.18, W0, C0, 1, 1);
end, end, end

% E3 -- beam sweep at sigma0 = 0.10 (288 = lidar reference, else ToF)
B = [288 64 16 8];  if smoke, B = 8; end
A = {'pvfh', 'pvfh_prob'};  if smoke, A = {'pvfh_prob'}; end
for a = A, for nb = B, for sd = seeds
    kind = 'tof';  if nb >= 288, kind = 'lidar'; end
    add('E3', a{1}, kind, 0.10, nb, sd, 'square', 1.18, W0, C0, 1, 1);
end, end, end

% E5 -- formation x algorithm at formation-safe spacing (E4 = pvfh_prob row)
A = ALL6;  F = FORMS;  if smoke, A = {'pvfh_prob'}; F = {'line'}; end
for f = F, for a = A, for sd = seeds
    add('E5', a{1}, 'lidar', 0.02, 288, sd, f{1}, 0.60, W0, C0, 1, 1);
end, end, end

% E6 -- matched sensor comparison
K = {'lidar', 'tof', 'tof'};  NB = [288 16 8];  S6 = [0.02 0.10];
A = LADDER;
if smoke, K = {'tof'}; NB = 16; S6 = 0.10; A = {'vfh'}; end
for k = 1:numel(K), for sg = S6, for a = A, for sd = seeds
    add('E6', a{1}, K{k}, sg, NB(k), sd, 'square', 1.18, W0, C0, 1, 1);
end, end, end, end

% E7 -- second and third environment classes
WW = {'swarm_world_plaza.sdf', 'swarm_world_maze.sdf'};
CC = {'dynamic_obstacles_plaza.json', 'dynamic_obstacles_maze.json'};
A = ALL6;  S7 = [0.02 0.20];
if smoke, A = {'pvfh_prob'}; S7 = 0.02; end
for w = 1:numel(WW), for a = A, for sg = S7, for sd = seeds
    add('E7', a{1}, 'lidar', sg, 288, sd, 'square', 1.18, WW{w}, CC{w}, 1, 1);
end, end, end, end

% E8 -- traffic-speed sweep
MS = [0.5 1.0 1.5 2.0];  A = {'vfh', 'dwa', 'mpc', 'pvfh', 'pvfh_prob'};
if smoke, MS = 2.0; A = {'pvfh_prob'}; end
for ms = MS, for a = A, for sd = seeds
    add('E8', a{1}, 'lidar', 0.02, 288, sd, 'square', 1.18, W0, C0, ms, 1);
end, end, end

% E9 -- anti-deadlock layer OFF on the hard shape (ON = the E5 line rows)
A = LADDER;  if smoke, A = {'pvfh_prob'}; end
for a = A, for sd = seeds
    add('E9', a{1}, 'lidar', 0.02, 288, sd, 'line', 0.60, W0, C0, 1, 0);
end, end

% E10 -- localisation error (pose-estimate noise; yaw sigma = 20 deg per m)
LOC = [0 0.05 0.15 0.30];  A = {'dwa', 'mpc', 'pvfh', 'pvfh_prob'};
if smoke, LOC = 0.15; A = {'pvfh_prob'}; end
for lc = LOC, for a = A, for sd = seeds
    add('E10', a{1}, 'lidar', 0.02, 288, sd, 'square', 1.18, W0, C0, 1, 1, lc);
end, end, end

% E11 -- traffic density: the first N movers of the six-mover contract
CD = 'dynamic_obstacles_dense.json';
NM = [0 1 3 6];  A = {'vfh', 'dwa', 'mpc', 'pvfh', 'pvfh_prob'};
if smoke, NM = [0 6]; A = {'pvfh_prob'}; end
for nm = NM, for a = A, for sd = seeds
    add('E11', a{1}, 'lidar', 0.02, 288, sd, 'square', 1.18, W0, CD, 1, 1, 0, 0, nm);
end, end, end

% E12 -- command-update loss
DR = [0 0.1 0.3 0.5];  A = {'dwa', 'mpc', 'pvfh', 'pvfh_prob'};
if smoke, DR = 0.3; A = {'pvfh_prob'}; end
for dr = DR, for a = A, for sd = seeds
    add('E12', a{1}, 'lidar', 0.02, 288, sd, 'square', 1.18, W0, C0, 1, 1, 0, dr);
end, end, end

% E13 -- randomised layouts (make_random_worlds.py; 3 noise seeds each)
d = dir(fullfile(fileparts(mfilename('fullpath')), 'swarm_world_rand*.sdf'));
RW = sort({d.name});  A = ALL6;  S13 = [0.02 0.20];
sd13 = seeds(1:min(3, numel(seeds)));
if smoke, RW = RW(1:min(1, numel(RW))); A = {'pvfh_prob'}; S13 = 0.02; end
for w = 1:numel(RW)
    cj = strrep(strrep(RW{w}, 'swarm_world_', 'dynamic_obstacles_'), '.sdf', '.json');
    for a = A, for sg = S13, for sd = sd13
        add('E13', a{1}, 'lidar', sg, 288, sd, 'square', 1.18, RW{w}, cj, 1, 1);
    end, end, end
end

% E14 -- load-feasible motion limits ON (the OFF rows are E1)
A = {'vfh', 'dwa', 'mpc', 'pvfh', 'pvfh_prob'};
if smoke, A = {'pvfh_prob'}; end
for a = A, for sd = seeds
    add('E14', a{1}, 'lidar', 0.02, 288, sd, 'square', 1.18, W0, C0, 1, 1, 0, 0, -1, 1);
end, end
end

%% ========================================================================
function r = run_one(j, RUNS)
%RUN_ONE  One mission; returns the result row (same fields on every path).
run_name = sprintf('%s_%s%03d_sg%03d_%s_sp%03d_%s_m%03d_a%d_l%03d_d%02d_n%d_k%d_s%d', ...
    j.algo, j.kind, j.beams, round(j.sigma0*100), j.formation, ...
    round(j.spacing*100), strrep(strrep(j.world, 'swarm_world_', ''), '.sdf', ''), ...
    round(j.mover_scale*100), j.adl, round(j.loc*100), round(j.drop*100), ...
    max(j.nmov, -1) + (j.nmov < 0) * 100, j.kin, j.seed);   % n99 = all movers
max_movers = inf;  if j.nmov >= 0, max_movers = j.nmov; end
CFG_OVERRIDE = struct('algo', j.algo, 'seed', j.seed, ...
    'world_file', j.world, 'dynamic_json', j.contract, ...
    'run_name', run_name, 'formation', j.formation, ...
    'grid_spacing', j.spacing, 'interactive', false, ...
    'timesteps', 3000, ...                       % 300 s mission timeout
    'viz', struct('enable', false), ...
    'log', struct('enable', true, 'file', fullfile(RUNS, [run_name '.csv'])), ...
    'dyn', struct('speed_scale', j.mover_scale, 'max_movers', max_movers), ...
    'loc', struct('sigma_pos', j.loc, 'sigma_yaw', j.loc * 20 * pi/180), ...
    'comm_drop', j.drop, 'kin', struct('enable', logical(j.kin)), ...
    'adl', struct('enable', logical(j.adl)));
CFG_OVERRIDE.vfh = struct('sense', struct('kind', j.kind, 'sigma0', j.sigma0, ...
    'k_rng', 0.03, 'fov_deg', 60, 'max_range', inf, 'frame', 'body'), ...
    'n_beams', j.beams);
r = struct('stack', 'matlab', 'exp', j.exp, 'algo', j.algo, ...
    'sense_kind', j.kind, 'sigma0', j.sigma0, 'beams', j.beams, ...
    'seed', j.seed, 'world', j.world, 'formation', j.formation, ...
    'spacing_used', j.spacing, 'mover_scale', j.mover_scale, 'adl', j.adl, ...
    'contract', j.contract, 'loc_sigma', j.loc, 'comm_drop', j.drop, ...
    'n_movers', j.nmov, 'kin', j.kin, ...
    'reached', 0, 'collisions', NaN, 'coll_static', NaN, 'coll_mover', NaN, ...
    'n_reject', NaN, 'n_escape', NaN, 'path_length', NaN, 'path_eff', NaN, ...
    'min_clear', NaN, 'mean_clear', NaN, 'total_time', NaN, ...
    'transport_time', NaN, 'total_turn', NaN, 'comp_ms_mean', NaN, ...
    'max_robot_speed', NaN, ...
    'run_name', run_name, 'key', j.key);
try
    Simulation8_dynamic;                    % leaves S, cfg in this workspace
    r.spacing_used   = cfg.grid_spacing;    % feasibility auto-fix may shrink it
    r.reached        = double(S.reached_goal);
    r.collisions     = S.collisions;
    r.coll_static    = S.coll_static;
    r.coll_mover     = S.coll_mover;
    r.n_reject       = S.n_reject;
    r.n_escape       = S.n_escape;
    r.path_length    = S.path_length;
    r.path_eff       = S.path_efficiency;
    r.min_clear      = S.min_clearance;
    r.mean_clear     = S.mean_clearance;
    r.total_time     = S.total_time;
    r.transport_time = S.transport_time;
    r.total_turn     = S.total_turning;
    r.comp_ms_mean   = S.comp_ms_mean;
    r.max_robot_speed = S.max_robot_speed;
catch err
    fprintf(2, '  RUN FAILED (%s): %s\n', run_name, err.message);
end
end

%% ========================================================================
function mk(d)
if exist(d, 'dir') ~= 7, mkdir(d); end
end

function append_row(path, r)
%APPEND_ROW  Crash-safe checkpoint: one result row appended to PATH.
f = fieldnames(r);
if exist(path, 'file') ~= 2
    fid = fopen(path, 'w');
    if fid < 0, error('cannot create %s', path); end
    fprintf(fid, '%s\n', strjoin(f', ','));
    fclose(fid);
end
fid = fopen(path, 'a');
if fid < 0, error('cannot append to %s', path); end
parts = cell(1, numel(f));
for i = 1:numel(f)
    v = r.(f{i});
    if ischar(v),    parts{i} = v;
    elseif isnan(v), parts{i} = 'NaN';
    else,            parts{i} = sprintf('%.6g', v);
    end
end
fprintf(fid, '%s\n', strjoin(parts, ','));
fclose(fid);
end

function [hdr, rows] = read_csv_rows(path)
hdr = {}; rows = {};
fid = fopen(path, 'r');
if fid < 0, return; end
ln = fgetl(fid);
if ischar(ln), hdr = strsplit(strtrim(ln), ','); end
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    if isempty(strtrim(ln)), continue; end
    rows{end+1} = strsplit(strtrim(ln), ','); %#ok<AGROW>
end
fclose(fid);
end

function keys = read_done_keys(PARTS)
%READ_DONE_KEYS  'exp|key' of every row already checkpointed in ANY part.
keys = {};
d = dir(fullfile(PARTS, 'part_*.csv'));
for i = 1:numel(d)
    [hdr, rows] = read_csv_rows(fullfile(PARTS, d(i).name));
    ie = find(strcmp(hdr, 'exp'), 1);  ik = find(strcmp(hdr, 'key'), 1);
    if isempty(ie) || isempty(ik), continue; end
    for k = 1:numel(rows)
        if numel(rows{k}) >= ik
            keys{end+1} = [rows{k}{ie} '|' rows{k}{ik}]; %#ok<AGROW>
        end
    end
end
end

function merge_parts(PARTS, out)
%MERGE_PARTS  parts/part_*.csv -> results_all.csv (dedup on exp|key, last wins).
d = dir(fullfile(PARTS, 'part_*.csv'));
if isempty(d), error('merge: no checkpoint parts in %s', PARTS); end
H = {}; R = {}; K = {};
for i = 1:numel(d)
    [hdr, rows] = read_csv_rows(fullfile(PARTS, d(i).name));
    if isempty(hdr), continue; end
    if isempty(H), H = hdr; end
    if ~isequal(H, hdr)
        error('merge: %s has a different header (delete logs/paper/parts and re-run)', d(i).name);
    end
    ie = find(strcmp(hdr, 'exp'), 1);  ik = find(strcmp(hdr, 'key'), 1);
    for k = 1:numel(rows)
        if numel(rows{k}) ~= numel(hdr), continue; end   % torn last line
        key = [rows{k}{ie} '|' rows{k}{ik}];
        at = find(strcmp(K, key), 1);
        if isempty(at), K{end+1} = key; R{end+1} = rows{k}; %#ok<AGROW>
        else, R{at} = rows{k}; end
    end
end
[~, order] = sort(K);
fid = fopen(out, 'w');
if fid < 0, error('cannot write %s', out); end
fprintf(fid, '%s\n', strjoin(H, ','));
for i = order(:)'
    fprintf(fid, '%s\n', strjoin(R{i}, ','));
end
fclose(fid);
fprintf('MERGE: wrote %s (%d rows from %d part file(s))\n', out, numel(R), numel(d));
end
