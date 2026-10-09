%% MAKE_FIGURES  Generate the PNG figures for the LaTeX simulation report.
%
%  Run from the matlab/ directory:  >> make_figures
%
%  Produces, in ../docs/simulation_report/figures/:
%    fig_trajectory.png   world + full mission (assembly AND transport)
%    fig_diagnostics.png  two-phase diagnostics with correct per-phase
%                         semantics (slot error | dist-to-goal, clearance,
%                         mover gaps incl. the predicted one)
%    fig_graphs.png       per-robot target distance + inter-robot distances
%    fig_formation.png    formation geometry: slots, payload block, radii
%    fig_vfh.png          p-VFH histogram + predictive scan of an encounter
%
%  GAP HANDLING. The log uses two conventions this script must respect:
%    - Sentinels: dynamic columns hold 99 where a quantity is undefined
%      (e.g. no mover nearby). Those samples are MASKED (NaN), never drawn,
%      otherwise every plot shows a fake cliff between 10 m and 99 m.
%    - Phase semantics: 'dist_to_goal' is the MAX SLOT ERROR in phase 1 and
%      the CENTROID-TO-GOAL distance in phase 2. They are different physical
%      quantities and are plotted as two panels, never joined into one line.
%  Phase boundaries are marked as vertical lines on every time-series figure.
%
%  RENDERING NOTE (verified on this machine). MATLAB's PNG export always
%  rasterises through OpenGL on headless Linux, and that pipeline DROPS line
%  segments wherever the line is not axis-aligned - curves that actually
%  change value render as scattered bits even though the logged data is
%  continuous (verified: the same CSV drawn with PIL is a clean curve, and
%  'print -painters' claimed success while producing the same broken PNG -
%  the -painters flag is silently ignored for raster output). The fix:
%  export VECTOR PDF (painters genuinely applies there), then rasterise
%  with pdftoppm (poppler - no OpenGL involved). save_fig below does both.
%  If pdftoppm is not installed, the PDFs are left in the figures dir and
%  the PNGs from the previous run are untouched.

outdir = '../docs/simulation_report/figures';
if exist(outdir, 'dir') ~= 7, mkdir(outdir); end
set(groot, 'DefaultFigureRenderer', 'painters');

L   = read_log_csv(fullfile('logs', 'matlab_dynamic.csv'));
robot_radius = 0.18;
grid_size    = 2;
grid_spacing = 1.18;
K            = 4;

world = sdf2world('swarm_world_dynamic.sdf', 'RobotHeight', 0.35, ...
                  'SkipModels', {'ground_plane','mover1','mover2','mover3'}, ...
                  'Verbose', false);
[movers, ~] = load_dynamic_obstacles('dynamic_obstacles.json');

p1 = L.phase == 1;
p2 = L.phase == 2;
t_switch = L.t(find(p2, 1) - 1);          % last forming tick (or 0)
fprintf('log: %d rows (phase1 %d, phase2 %d), phase switch at t = %.1f s\n', ...
        numel(L.t), sum(p1), sum(p2), t_switch);

%% ---- FIG 1: trajectory map (FULL mission) ------------------------------
fig = figure('Color','w','Position',[60 60 1150 640]);
draw_world(world, 'Inflate', robot_radius);
hold on;

% mover swept segments + start markers + poses at the two yield instants
h_sweep = [];
for k = 1:numel(movers)
    m = movers(k);
    h_sweep = plot(m.route(:,1), m.route(:,2), '-', ...
                   'Color', [0.95 0.55 0.1], 'LineWidth', 2);
    plot(m.route(1,1), m.route(1,2), 'o', 'MarkerSize', 8, ...
         'MarkerEdgeColor', [0.6 0.3 0.05], 'MarkerFaceColor', [0.95 0.55 0.1]);
end
for tk = [16.1 46.3]
    for k = 1:numel(movers)
        draw_mover(eval_mover_pose(movers(k), tk), movers(k));
    end
end

% robots across BOTH phases (the assembly is part of the mission)
rcol = [0.85 0.2 0.2; 0.2 0.6 0.3; 0.9 0.6 0.1; 0.5 0.2 0.7];
h_r = gobjects(1, K);
for i = 1:K
    h_r(i) = plot(L.rx(:,i), L.ry(:,i), '-', 'Color', [rcol(i,:) 0.55], ...
                  'LineWidth', 1.0);
end
h_lead = plot(L.lx, L.ly, '-', 'Color', [0 0.35 0.85], 'LineWidth', 2);
plot(L.lx(p1), L.ly(p1), '.', 'Color', [0 0.35 0.85], 'MarkerSize', 9);
plot(L.lx(1), L.ly(1), 'ks', 'MarkerFaceColor','g', 'MarkerSize', 10);
plot(L.lx(end), L.ly(end), 'kp', 'MarkerFaceColor',[0 0.35 0.85], 'MarkerSize', 14);
text(L.lx(1)+0.4, L.ly(1)+0.9, 'scatter start', 'FontSize', 11, 'FontWeight','bold');
text(L.lx(end)-2.2, L.ly(end)+1.4, 'goal', 'FontSize', 11, 'FontWeight','bold');
axis equal;
xlim([world.bounds(1)-1, world.bounds(2)+1]);
ylim([world.bounds(3)-1, world.bounds(4)+1]);
xlabel('x (m)'); ylabel('y (m)');
title('Full mission: scattered start \rightarrow assembly \rightarrow rigid transport');
legend([h_sweep, h_lead, h_r(1), h_r(2), h_r(3), h_r(4)], ...
       {'mover swept path', 'leader (centroid)', 'robot 1', 'robot 2', ...
        'robot 3', 'robot 4'}, 'Location', 'southeast', 'FontSize', 9);
save_fig(fig, fullfile(outdir, 'fig_trajectory.png'), 160);

%% ---- FIG 2: diagnostics, correct per-phase semantics --------------------
fig = figure('Color','w','Position',[60 60 940 1050]);

% -- panel 1: slot error (phase 1) then dist-to-goal (phase 2), two colours,
%    marked boundary -- never one continuous line, the semantics change.
subplot(4,1,1); hold on; grid on;
plot(L.t(p1), L.dist(p1), 'LineWidth', 1.4, 'Color', [0.2 0.6 0.3]);
plot(L.t(p2), L.dist(p2), 'LineWidth', 1.4, 'Color', [0 0.35 0.85]);
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylabel('slot err (m) / dist (m)');
legend({'p1: slot error', 'p2: dist to goal', 'switch'}, ...
       'Location', 'northeast', 'FontSize', 8);
title('Approach metric: assembly error, then distance to goal');

% -- panel 2: clearance across the whole run --
subplot(4,1,2); hold on; grid on;
plot(L.t, L.clr, 'LineWidth', 1.4, 'Color', [0 0.35 0.85]);
yline_compat(robot_radius, 'r--');
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylabel('clearance (m)');
legend({'clearance', 'collision thr'}, 'Location', 'southeast', 'FontSize', 8);
title('Minimum clearance of the formation body');

% -- panel 3: mover gap, now vs predicted, sentinels masked --
subplot(4,1,3); hold on; grid on;
near = mask99(L.near);
near_pre = mask99(L.near_pre);
plot(L.t, near,     'LineWidth', 1.4, 'Color', [0.55 0.2 0.7]);
plot(L.t, near_pre, 'LineWidth', 1.4, 'Color', [0.9 0.45 0.1]);
yline_compat(robot_radius + 0.25, 'm--');
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylabel('mover gap (m)');
legend({'gap now', 'predicted', 'slowdown thr', 'switch'}, ...
       'Location', 'northeast', 'FontSize', 8);
title('Gap to the nearest mover: predicted stays below current when closing');

% -- panel 4: speed scale --
subplot(4,1,4); hold on; grid on;
plot(L.t, L.vscale, 'LineWidth', 1.4, 'Color', [0 0.35 0.85]);
yline_compat(0.4, 'r--');
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylim([0.3 1.1]);
ylabel('speed scale'); xlabel('time (s)');
legend({'speed scale', 'floor 0.4'}, 'Location', 'southeast', 'FontSize', 8);
title('Speed scale (phase 1 is 1 by definition: per-robot slow\_radius logic)');
save_fig(fig, fullfile(outdir, 'fig_diagnostics.png'), 160);

%% ---- FIG 3: per-robot + inter-robot graphs -----------------------------
target = [12 0];
dist_to_target = zeros(numel(L.t), K);
for i = 1:K
    dist_to_target(:,i) = hypot(L.rx(:,i) - target(1), L.ry(:,i) - target(2));
end
fig = figure('Color','w','Position',[60 60 900 800]);
subplot(2,1,1); hold on; grid on;
cols = lines(K);
for i = 1:K
    plot(L.t, dist_to_target(:,i), 'LineWidth', 1.4, 'Color', cols(i,:));
end
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylabel('robot - target (m)');
title('Distance from each robot to the final target (flat tail = convoyed arrival)');
legend([arrayfun(@(i) sprintf('r%d', i), 1:K, 'UniformOutput', false), ...
        {'phase switch'}], 'Location', 'northeast');

subplot(2,1,2); hold on; grid on;
pairs = nchoosek(1:K, 2);
hpair = gobjects(size(pairs,1),1);
nleg = cell(size(pairs,1),1);
for pp = 1:size(pairs,1)
    i = pairs(pp,1); j = pairs(pp,2);
    d = hypot(L.rx(:,i) - L.rx(:,j), L.ry(:,i) - L.ry(:,j));
    hpair(pp) = plot(L.t, d, 'LineWidth', 1.4);
    nleg{pp} = sprintf('r%d-r%d', i, j);
end
yline_compat(grid_spacing, 'k--');
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylabel('inter-robot (m)'); xlabel('time (s)');
title('Pairwise separations converge onto the 1.18 m slot spacing (dashed)');
legend([hpair, gobjects(0)], nleg, 'Location', 'southeast');
save_fig(fig, fullfile(outdir, 'fig_graphs.png'), 160);

%% ---- FIG 4: formation geometry -----------------------------------------
offs = [-0.59 -0.59; -0.59 0.59; 0.59 -0.59; 0.59 0.59];
fig = figure('Color','w','Position',[60 60 760 640]); hold on; grid on; axis equal;
ext = max(abs(offs),[],1) + robot_radius;
patch([-ext(1) ext(1) ext(1) -ext(1)], [-ext(2) -ext(2) ext(2) ext(2)], ...
      [1 0.9 0.75], 'FaceAlpha', 0.5, 'EdgeColor', [0.8 0.4 0]);
plot([-0.59 0.59 -0.59 -0.59 0.59 0.59 -0.59], ...
     [0.59 0.59 -0.59 -0.59 -0.59 0.59 0.59], 'k:', 'LineWidth', 0.8);
th = linspace(0, 2*pi, 64);
for i = 1:K
    fill(offs(i,1)+robot_radius*cos(th), offs(i,2)+robot_radius*sin(th), ...
         cols(i,:), 'EdgeColor', 'k', 'FaceAlpha', 0.9);
end
plot(offs(:,1), offs(:,2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
cr = norm(offs(1,:)) + robot_radius;
plot(cr*cos(th), cr*sin(th), 'b-', 'LineWidth', 1.8);
text(cr*0.55, -cr*0.75, sprintf('circumradius %.2f m', cr), 'Color', 'b');
text(-0.55, -2.45, sprintf('slot spacing %.2f m', grid_spacing), 'FontSize', 10);
xlabel('x (m)'); ylabel('y (m)');
title('2x2 formation: slots, footprints, payload block, circumradius');
save_fig(fig, fullfile(outdir, 'fig_formation.png'), 160);

%% ---- FIG 5: p-VFH histogram + predictive scan ---------------------------
S = 72;  dsec = 2*pi/S;
max_sense = 4.0 + robot_radius;
pos = [-9.0 0.5];  goal = [12 0.5];  margin = robot_radius;

m1 = movers(1);  m1.c = eval_mover_pose(movers(1), 9.0);
[r_inst, ang] = lidar_scan_dyn(pos, world, m1, 288, max_sense);

ranges = r_inst;
wempty = world;
wempty.boxes = world.boxes([]);  wempty.circles = world.circles([]);
for tj = 0:0.1:0.5
    mkj = m1;  mkj.c = eval_mover_pose(movers(1), 9.0 + tj);
    rk = lidar_scan_dyn(pos, wempty, mkj, 288, max_sense);
    ranges = min(ranges, rk);
end

sec_idx = min(max(floor((ang + pi)/(2*pi)*S) + 1, 1), S);
rmin = accumarray(sec_idx', ranges', [S 1], @min, inf)';
sigma = (margin + 1.0) / sqrt(2*log(1/0.3));
hist_vals = zeros(1,S);
for si = find(rmin < max_sense)
    val = exp(-(rmin(si)^2)/(2*sigma^2));
    gamma = asin(min(1, margin/max(rmin(si), margin)));
    kk = ceil(gamma/dsec);
    idxs = mod((si-1-kk : si-1+kk), S) + 1;
    hist_vals(idxs) = max(hist_vals(idxs), val);
end
hist_vals = movmean(hist_vals, [2 2]);

fig = figure('Color','w','Position',[60 60 1150 430]);
subplot(1,2,1);
bar(rad2deg(-pi + (2*pi)*(0:S-1)/S), hist_vals, 1); grid on; hold on;
yline_compat(0.3, 'r--');
xlabel('bearing (deg)'); ylabel('polar obstacle density');
title(sprintf('p-VFH histogram from (%.0f, %.1f), t = 9 s', pos(1), pos(2)));
xticks(-180:60:180); xlim([-200 200]);

subplot(1,2,2);
draw_world(world, 'Inflate', margin);
hold on;
th_ = linspace(0,2*pi,32);
for tj = 0:0.1:0.5
    p_ = eval_mover_pose(movers(1), 9.0 + tj);
    plot(p_(1)+0.35*cos(th_), p_(2)+0.35*sin(th_), '-', ...
         'Color', [0.95 0.55 0.1], 'LineWidth', 1.4);
end
plot(pos(1), pos(2), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
plot(goal(1), goal(2), 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 15);
axis equal; xlim([-11 -2]); ylim([-3 4]);
title('Predictive scan: orange = mover at t .. t+0.5 s');
save_fig(fig, fullfile(outdir, 'fig_vfh.png'), 160);

%% ---- FIG 6 (new): the gaps themselves ----------------------------------
% Dedicated figure for "what are the gaps in the graphs": every place where
% the log legitimately has no data, and every discontinuity of meaning.
fig = figure('Color','w','Position',[60 60 940 780]);

subplot(2,2,1); hold on; grid on;
tt = L.t; v = mask99(L.near_pre);
plot(tt, L.near, 'Color', [0.55 0.2 0.7], 'LineWidth', 1.3);
plot(tt(isfinite(v)), v(isfinite(v)), '.', 'Color', [0.9 0.45 0.1], ...
     'MarkerSize', 8);
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylabel('gap (m)');
title(sprintf('Predicted-gap coverage (%d/%d)', sum(isfinite(v)), numel(v)));
legend({'gap now', 'predicted (masked where undefined)'}, ...
       'Location', 'best', 'FontSize', 8);

subplot(2,2,2); hold on; grid on;
plot(L.t(p1), L.dist(p1), '-o', 'Color', [0.2 0.6 0.3], 'MarkerSize', 3, ...
     'LineWidth', 1.2);
plot(L.t(p2), L.dist(p2), 'LineWidth', 1.4, 'Color', [0 0.35 0.85]);
xline(t_switch, 'k:', 'LineWidth', 1.2);
yline(0.3, 'g--', 'LineWidth', 1);
yline(0.5, 'b--', 'LineWidth', 1);
ylabel('m');
title('Two metrics \neq one series: never joined');
legend({'p1: max slot error (conv. to 0.3)', 'p2: dist to goal (conv. to 0.5)', ...
        'phase switch'}, 'Location', 'northeast', 'FontSize', 8);

subplot(2,2,3); hold on; grid on;
ss = L.vscale;
plot(L.t, ss, 'LineWidth', 1.3, 'Color', [0 0.35 0.85]);
yline(0.4, 'r--');
xline(t_switch, 'k:', 'LineWidth', 1.2);
ylim([0.3 1.1]);
ylabel('speed scale'); xlabel('time (s)');
title('Slowdown dips at the two crossings');
legend({'speed scale', 'floor 0.4'}, 'Location', 'southeast', 'FontSize', 8);

subplot(2,2,4); hold on; grid on;
% per-robot mover gap: the centroid value hides which robot was threatened
gaps = zeros(numel(L.t), K);
p2t = find(p2, 1);
for i = 1:K
    gaps(:, i) = NaN;
end
% recompute per-robot gaps from logged robot poses against the contract
for r = 1:numel(L.t)
    tk = L.t(r);
    mk = movers;
    for k = 1:numel(mk), mk(k).c = eval_mover_pose(movers(k), tk); end
    for i = 1:K
        d = inf;
        for k = 1:numel(mk)
            if strcmpi(mk(k).shape, 'circle')
                dk = hypot(L.rx(r,i)-mk(k).c(1), L.ry(r,i)-mk(k).c(2)) - mk(k).r;
            else
                dx = abs(L.rx(r,i)-mk(k).c(1)) - mk(k).size(1)/2;
                dy = abs(L.ry(r,i)-mk(k).c(2)) - mk(k).size(2)/2;
                dk = hypot(max(dx,0), max(dy,0)) + min(max(dx,dy),0);
            end
            d = min(d, dk);
        end
        gaps(r,i) = d;
    end
end
plot(L.t, gaps, 'LineWidth', 1.2);
yline_compat(robot_radius + 0.25, 'm--');
xlabel('time (s)'); ylabel('robot \rightarrow mover gap (m)');
title('Per-robot mover gaps (exposure)');
legend([arrayfun(@(i) sprintf('r%d', i), 1:K, 'UniformOutput', false), ...
        {'slowdown thr'}], 'Location', 'northeast', 'FontSize', 8);
save_fig(fig, fullfile(outdir, 'fig_gaps.png'), 160);

convert_all_pdfs(outdir);

disp('figures written:');
disp({dir(fullfile(outdir, 'fig_*.png')).name});

%% ---------------------------------------------------------------------
function save_fig(fig, pdf_path, ~)
%SAVE_FIG  Export a figure as a tightly-cropped VECTOR PDF.
%   exportgraphics with ContentType='vector' uses the painters pipeline
%   (no OpenGL -> no dropped segments) and crops to the figure content,
%   unlike print -dpdf which re-lays-out onto a paper page.
[p, n, e] = fileparts(pdf_path);
assert(strcmp(e, '.png'));                       % caller passes the png name
target = fullfile(p, [n '.pdf']);                % fileparts -> [path name ext]
exportgraphics(fig, target, 'ContentType', 'vector');
end

function convert_all_pdfs(pdfdir)
%CONVERT_ALL_PDFS  Rasterise every exported fig_*.pdf to a same-name PNG.
pdfs = dir(fullfile(pdfdir, 'fig_*.pdf'));
if isempty(pdfs)
    error('make_figures:noPdfs', 'No fig_*.pdf written; export failed.');
end
ok = true;
for k = 1:numel(pdfs)
    [~, b] = fileparts(pdfs(k).name);
    cmd = sprintf('pdftoppm -png -r 160 -singlefile "%s" "%s"', ...
                  fullfile(pdfdir, [b '.pdf']), fullfile(pdfdir, b));
    st = system(cmd);
    if st ~= 0
        ok = false;
        fprintf(2, 'pdftoppm failed for %s (status %d)\n', b, st);
    end
end
if ~ok
    fprintf(2, ['PDF->PNG conversion failed; install poppler-utils ' ...
                '(pdftoppm) and rerun, or convert the fig_*.pdf files ' ...
                'manually.\n']);
    return;
end
% trim the PDF page margins off the PNGs (poppler keeps the page box)
st = system(sprintf('python3 "%s/crop_figures.py" "%s"', ...
                    fileparts(mfilename('fullpath')), pdfdir));
if st ~= 0
    fprintf(2, ['crop_figures.py failed (python3/PIL missing?). Figures ' ...
                'are correct but keep their page margins.\n']);
end
end

%% ---------------------------------------------------------------------
function y = mask99(x)
%MASK99  Sentinel 99 -> NaN so undefined samples are not drawn.
y = x;
y(x >= 98.9) = NaN;
end

function L = read_log_csv(fn)
M = readmatrix(fn, 'NumHeaderLines', 1);
L.t = M(:,1); L.phase = M(:,2); L.lx = M(:,3); L.ly = M(:,4);
L.lh = M(:,5); L.dist = M(:,6); L.clr = M(:,7);
Kk = 4;
L.rx = M(:, 8:8+Kk-1); L.ry = M(:, 8+Kk:8+2*Kk-1);
% dynamic columns in schema order
i = 8 + 2*Kk;
L.cvx = M(:,i); L.cvy = M(:,i+1); L.vscale = M(:,i+2);
L.near = M(:,i+3); L.near_pre = M(:,i+4);
end

function draw_mover(p, m)
if strcmpi(m.shape, 'circle')
    th = linspace(0, 2*pi, 32);
    fill(p(1)+m.r*cos(th), p(2)+m.r*sin(th), [0.95 0.55 0.1], ...
         'EdgeColor', [0.5 0.25 0], 'FaceAlpha', 0.9);
else
    patch(p(1)+[-1 1 1 -1]*m.size(1)/2, p(2)+[-1 -1 1 1]*m.size(2)/2, ...
          [0.95 0.55 0.1], 'EdgeColor', [0.5 0.25 0], 'FaceAlpha', 0.9);
end
end
