function R = compare_runs(matlab_csv, ros2_csv, world_file, varargin)
%COMPARE_RUNS  Quantitative and visual comparison of a MATLAB run and a ROS 2 run.
%
%   R = COMPARE_RUNS(matlab_csv, ros2_csv, world_file)
%   R = COMPARE_RUNS(matlab_csv, ros2_csv, world_file, 'Name', value, ...)
%
%   Both CSVs must use the shared schema (see README):
%     t, phase, leader_x, leader_y, leader_yaw, dist_to_goal, clearance,
%     r1_x..rK_x, r1_y..rK_y
%
%   OPTIONS
%     'Labels'   ({'MATLAB','ROS2'})
%     'Plot'     (true)
%     'Phase'    (2) which phase to compare; [] for the whole run
%     'Recompute' (true) recompute clearance from the SDF for BOTH runs rather
%                 than trusting each stack's own number
%
%   RETURNS a struct of per-run metrics plus the between-run agreement metrics.
%
%   WHAT IT MEASURES AND WHY
%   Comparing two navigation stacks on "did it reach the goal" tells you almost
%   nothing. These are the quantities that actually separate implementations:
%
%     path_length / efficiency  did one stack take a systematically longer route
%     min/mean clearance        did one hug obstacles harder (tuning difference)
%     total_turning             control effort; a proxy for how much the load
%                               was shaken about during transport
%     frechet                   worst-case spatial disagreement between the two
%                               paths, order-preserving. The honest headline
%                               number for "are these the same trajectory"
%     mean_deviation            typical spatial disagreement, less alarmist
%                               than Frechet and better for reporting trends
%     formation_rms             how well each stack held the rigid shape. In
%                               MATLAB the block is rigid by construction, so
%                               this is near zero; in ROS 2 it is a real
%                               measurement of the formation controller. This
%                               is usually the most interesting divergence.
%
%   Clearance is recomputed from the SDF by default so both runs are scored by
%   the same geometry, rather than comparing MATLAB's number against whatever
%   the ROS 2 costmap happened to report.
%
%   See also SDF2WORLD, WORLD_DIST.

opt = struct('Labels', {{'MATLAB','ROS2'}}, 'Plot', true, 'Phase', 2, 'Recompute', true);
for k = 1:2:numel(varargin)
    if isfield(opt, varargin{k}), opt.(varargin{k}) = varargin{k+1}; end
end

world = sdf2world(world_file, 'Verbose', false);

A = load_run(matlab_csv, opt.Phase);
B = load_run(ros2_csv,   opt.Phase);

if opt.Recompute
    A.clr = recompute_clearance(A, world);
    B.clr = recompute_clearance(B, world);
end

R = struct();
R.(safename(opt.Labels{1})) = run_metrics(A, world);
R.(safename(opt.Labels{2})) = run_metrics(B, world);

% ---------------- between-run agreement ----------------
PA = [A.lx(:) A.ly(:)];
PB = [B.lx(:) B.ly(:)];
R.agreement.frechet        = discrete_frechet(PA, PB);
R.agreement.mean_deviation = mean_pathwise_deviation(PA, PB);
R.agreement.max_deviation  = max(pointwise_min_dist(PA, PB));
R.agreement.endpoint_gap   = norm(PA(end,:) - PB(end,:));
R.agreement.time_ratio     = (B.t(end)-B.t(1)) / max(A.t(end)-A.t(1), eps);
R.agreement.length_ratio   = R.(safename(opt.Labels{2})).path_length / ...
                             max(R.(safename(opt.Labels{1})).path_length, eps);

print_comparison(R, opt.Labels);

if opt.Plot
    plot_comparison(A, B, world, opt.Labels);
end
end

% ========================================================================
function S = load_run(fn, phase)
if exist(fn, 'file') ~= 2
    error('compare_runs:noFile', 'CSV not found: %s', fn);
end
fid = fopen(fn, 'r');
hdr = strtrim(fgetl(fid));
fclose(fid);
cols = strsplit(hdr, ',');

M = dlmread(fn, ',', 1, 0);

idx = @(name) find(strcmp(cols, name), 1);
need = {'t','phase','leader_x','leader_y','leader_yaw'};
for i = 1:numel(need)
    if isempty(idx(need{i}))
        error('compare_runs:badSchema', ...
              'Column "%s" missing from %s. See the README for the shared CSV schema.', ...
              need{i}, fn);
    end
end

keep = true(size(M,1),1);
if ~isempty(phase)
    keep = M(:, idx('phase')) == phase;
end
if ~any(keep)
    error('compare_runs:noRows', 'No phase-%d rows in %s.', phase, fn);
end

S.t  = M(keep, idx('t'));
S.lx = M(keep, idx('leader_x'));
S.ly = M(keep, idx('leader_y'));
S.lh = M(keep, idx('leader_yaw'));
S.clr = nan(sum(keep),1);
if ~isempty(idx('clearance')), S.clr = M(keep, idx('clearance')); end
S.dist = nan(sum(keep),1);
if ~isempty(idx('dist_to_goal')), S.dist = M(keep, idx('dist_to_goal')); end

% robot columns, however many there are
rx = {}; ry = {};
for i = 1:64
    cx = idx(sprintf('r%d_x', i)); cy = idx(sprintf('r%d_y', i));
    if isempty(cx) || isempty(cy), break; end
    rx{end+1} = M(keep, cx); ry{end+1} = M(keep, cy); %#ok<AGROW>
end
S.rx = cell2mat(rx);
S.ry = cell2mat(ry);
S.K  = size(S.rx, 2);
S.file = fn;
end

function clr = recompute_clearance(S, world)
if S.K == 0
    clr = world_dist([S.lx S.ly], world);
    return;
end
clr = inf(numel(S.lx),1);
for i = 1:S.K
    clr = min(clr, world_dist([S.rx(:,i) S.ry(:,i)], world));
end
end

function m = run_metrics(S, world) %#ok<INUSD>
d = sqrt(diff(S.lx).^2 + diff(S.ly).^2);
m.duration      = S.t(end) - S.t(1);
m.path_length   = sum(d);
m.straight_dist = hypot(S.lx(end)-S.lx(1), S.ly(end)-S.ly(1));
m.efficiency    = m.straight_dist / max(m.path_length, eps);
m.mean_speed    = m.path_length / max(m.duration, eps);
m.min_clearance = min(S.clr);
m.mean_clearance= mean(S.clr);
dh              = abs(diff(unwrap(S.lh)));
m.total_turning = sum(dh);
dt              = max(median(diff(S.t)), eps);
m.mean_turn_rate= mean(dh)/dt;
m.max_turn_rate = max(dh)/dt;
m.formation_rms = formation_error(S);
end

function e = formation_error(S)
%FORMATION_ERROR  RMS deviation of inter-robot distances from their own mean.
%   Scale-free: it does not need to know the nominal spacing, so it works even
%   if the two stacks were configured with slightly different geometry.
e = NaN;
if S.K < 2, return; end
n = numel(S.lx);
P = zeros(n, S.K*(S.K-1)/2);
c = 0;
for i = 1:S.K-1
    for j = i+1:S.K
        c = c + 1;
        P(:,c) = hypot(S.rx(:,i)-S.rx(:,j), S.ry(:,i)-S.ry(:,j));
    end
end
e = sqrt(mean(mean((P - mean(P,1)).^2)));
end

% ---------------- path agreement ----------------
function d = pointwise_min_dist(PA, PB)
d = zeros(size(PA,1),1);
for i = 1:size(PA,1)
    d(i) = min(hypot(PB(:,1)-PA(i,1), PB(:,2)-PA(i,2)));
end
end

function m = mean_pathwise_deviation(PA, PB)
m = 0.5*(mean(pointwise_min_dist(PA, PB)) + mean(pointwise_min_dist(PB, PA)));
end

function d = discrete_frechet(PA, PB)
%DISCRETE_FRECHET  Order-preserving worst-case path distance.
%   Paths are resampled to at most 400 points each first: the DP is O(n*m) and
%   a 500-step log against a 3000-step bag would otherwise be slow with no gain
%   in fidelity.
PA = resample_path(PA, 400);
PB = resample_path(PB, 400);
n  = size(PA,1); m = size(PB,1);
C  = inf(n, m);
for i = 1:n
    dx = PB(:,1)-PA(i,1); dy = PB(:,2)-PA(i,2);
    dij = hypot(dx, dy);
    for j = 1:m
        if i == 1 && j == 1
            C(i,j) = dij(j);
        elseif i == 1
            C(i,j) = max(C(i,j-1), dij(j));
        elseif j == 1
            C(i,j) = max(C(i-1,j), dij(j));
        else
            C(i,j) = max(min([C(i-1,j), C(i,j-1), C(i-1,j-1)]), dij(j));
        end
    end
end
d = C(n,m);
end

function Q = resample_path(P, n)
if size(P,1) <= n, Q = P; return; end
s = [0; cumsum(hypot(diff(P(:,1)), diff(P(:,2))))];
if s(end) <= 0, Q = P(1:min(n,size(P,1)),:); return; end
si = linspace(0, s(end), n)';
[s, ia] = unique(s);
Q = [interp1(s, P(ia,1), si), interp1(s, P(ia,2), si)];
end

% ---------------- reporting ----------------
function s = safename(x)
s = regexprep(x, '[^A-Za-z0-9_]', '_');
if ~isletter(s(1)), s = ['r' s]; end
end

function print_comparison(R, L)
a = R.(safename(L{1})); b = R.(safename(L{2}));
fprintf('\n=========== RUN COMPARISON ===========\n');
fprintf('%-22s %12s %12s %10s\n', 'metric', L{1}, L{2}, 'delta');
row('duration (s)',        a.duration,       b.duration);
row('path length (m)',     a.path_length,    b.path_length);
row('efficiency',          a.efficiency,     b.efficiency);
row('mean speed (m/s)',    a.mean_speed,     b.mean_speed);
row('min clearance (m)',   a.min_clearance,  b.min_clearance);
row('mean clearance (m)',  a.mean_clearance, b.mean_clearance);
row('total turning (rad)', a.total_turning,  b.total_turning);
row('mean turn (rad/s)',   a.mean_turn_rate, b.mean_turn_rate);
row('formation RMS (m)',   a.formation_rms,  b.formation_rms);
fprintf('--------------------------------------\n');
fprintf('  Frechet distance     : %.3f m   (worst-case path disagreement)\n', R.agreement.frechet);
fprintf('  mean deviation       : %.3f m   (typical disagreement)\n',        R.agreement.mean_deviation);
fprintf('  max deviation        : %.3f m\n',                                R.agreement.max_deviation);
fprintf('  endpoint gap         : %.3f m\n',                                R.agreement.endpoint_gap);
fprintf('  time ratio  (B/A)    : %.3f\n',                                  R.agreement.time_ratio);
fprintf('  length ratio (B/A)   : %.3f\n',                                  R.agreement.length_ratio);
fprintf('======================================\n\n');
end

function row(name, x, y)
fprintf('%-22s %12.3f %12.3f %10.3f\n', name, x, y, y-x);
end

function plot_comparison(A, B, world, L)
figure('Color','w','Name','MATLAB vs ROS2','Position',[80 80 1100 760]);

subplot(2,2,[1 2]); hold on;
draw_world(world);
plot(A.lx, A.ly, '-',  'LineWidth', 2,   'Color', [0 0.35 0.85]);
plot(B.lx, B.ly, '--', 'LineWidth', 2,   'Color', [0.9 0.25 0.1]);
plot(A.lx(1), A.ly(1), 'ks', 'MarkerFaceColor','g','MarkerSize',9);
plot(A.lx(end), A.ly(end), 'ko','MarkerFaceColor',[0 0.35 0.85],'MarkerSize',8);
plot(B.lx(end), B.ly(end), 'ko','MarkerFaceColor',[0.9 0.25 0.1],'MarkerSize',8);
legend({'', '', L{1}, L{2}}, 'Location','best');
title('Leader trajectory overlay (same world frame, no transform applied)');
xlabel('x (m)'); ylabel('y (m)');

subplot(2,2,3); hold on; grid on;
plot(A.t - A.t(1), A.clr, 'LineWidth', 1.5, 'Color', [0 0.35 0.85]);
plot(B.t - B.t(1), B.clr, 'LineWidth', 1.5, 'Color', [0.9 0.25 0.1]);
xlabel('time (s)'); ylabel('clearance (m)');
title('Clearance to nearest obstacle'); legend(L, 'Location','best');

subplot(2,2,4); hold on; grid on;
plot(A.t - A.t(1), rad2deg(unwrap(A.lh)), 'LineWidth', 1.5, 'Color', [0 0.35 0.85]);
plot(B.t - B.t(1), rad2deg(unwrap(B.lh)), 'LineWidth', 1.5, 'Color', [0.9 0.25 0.1]);
xlabel('time (s)'); ylabel('heading (deg)');
title('Leader heading (smooth = load stays level)'); legend(L, 'Location','best');
end
