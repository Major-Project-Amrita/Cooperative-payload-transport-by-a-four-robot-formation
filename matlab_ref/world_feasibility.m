function rep = world_feasibility(world, start_xy, goal_xy, cfg_in)
%WORLD_FEASIBILITY  Can this rigid formation actually get through this world?
%
%   rep = WORLD_FEASIBILITY(world, start_xy, goal_xy)
%   rep = WORLD_FEASIBILITY(world, start_xy, goal_xy, cfg)
%
%   cfg fields (all optional):
%     .res           (0.05) grid resolution, metres
%     .robot_radius  (0.18) physical robot radius
%     .grid_size     (2)    N for an NxN formation
%     .grid_spacing  (4.0)  slot spacing, metres
%     .verbose       (true)
%
%   rep.bottleneck      widest half-corridor on the best start->goal path (m).
%                       This is the max-min clearance over all paths, i.e. the
%                       tightest squeeze you are forced through.
%     .point_ok         can a point robot get through at all
%     .single_ok        can ONE robot of robot_radius get through
%     .formation_ok     can the RIGID formation get through
%     .need_circum      circumradius the current formation demands
%     .max_spacing      largest grid_spacing that still fits
%     .clearance        clearance grid (m), .xs/.ys the axes
%
%   WHY THIS EXISTS
%   A rigid formation is not a point robot. The VFH layer steers a single
%   virtual leader, but the body that must fit is the whole NxN block, and
%   because the block rotates with heading, the quantity that has to fit is
%   the CIRCUMRADIUS, not the half-width. If the circumradius exceeds the
%   bottleneck, no amount of gain tuning will get the swarm through: it will
%   grind against the stuck-escape logic forever and your MATLAB-vs-ROS2
%   comparison will be a comparison of two different failure modes. Run this
%   before every experiment.
%
%   Uses a BFS flood fill over a thresholded clearance grid inside a bisection
%   on the threshold. No Image Processing Toolbox required.
%
%   See also SDF2WORLD, WORLD_DIST.

cfg = struct('res', 0.05, 'robot_radius', 0.18, 'grid_size', 2, ...
             'grid_spacing', 4.0, 'verbose', true);
if nargin >= 4 && ~isempty(cfg_in)
    f = fieldnames(cfg_in);
    for i = 1:numel(f)
        if isfield(cfg, f{i}), cfg.(f{i}) = cfg_in.(f{i}); end
    end
end

b   = world.bounds;
xs  = b(1):cfg.res:b(2);
ys  = b(3):cfg.res:b(4);
[X, Y] = meshgrid(xs, ys);
D   = reshape(world_dist([X(:) Y(:)], world), size(X));   % signed clearance

si = xy2idx(start_xy, xs, ys);
gi = xy2idx(goal_xy,  xs, ys);

if D(si(1), si(2)) <= 0
    error('world_feasibility:startBlocked', ...
        'Start [%.2f %.2f] is inside an obstacle (clearance %.2f m).', ...
        start_xy, D(si(1), si(2)));
end
if D(gi(1), gi(2)) <= 0
    error('world_feasibility:goalBlocked', ...
        'Goal [%.2f %.2f] is inside an obstacle (clearance %.2f m).', ...
        goal_xy, D(gi(1), gi(2)));
end

% Pad with a blocked border so the BFS needs no bounds checks.
[R0, C0] = size(D);
Dpad = -inf(R0+2, C0+2);
Dpad(2:end-1, 2:end-1) = D;
Rp   = R0 + 2;
sLin = (si(2)+1-1)*Rp + (si(1)+1);
gLin = (gi(2)+1-1)*Rp + (gi(1)+1);

% ---- bisection on the clearance threshold that keeps start and goal joined ----
lo = 0; hi = max(D(:));
if ~connected(Dpad, 0, sLin, gLin, Rp)
    rep = emptyreport(cfg, D, xs, ys);
    rep.bottleneck = 0;
    if cfg.verbose
        fprintf(2, 'WORLD_FEASIBILITY: start and goal are not connected at all.\n');
    end
    return;
end
% 1 cm of precision is far finer than any tuning decision this feeds, so stop
% there rather than burning fixed iterations.
it = 0;
while (hi - lo) > 0.01 && it < 14
    mid = 0.5*(lo + hi);
    if connected(Dpad, mid, sLin, gLin, Rp), lo = mid; else, hi = mid; end
    it = it + 1;
end

rep            = emptyreport(cfg, D, xs, ys);
rep.bottleneck = lo;

n = cfg.grid_size;
rep.need_circum  = ((n-1)*cfg.grid_spacing)*sqrt(2)/2 + cfg.robot_radius;
rep.point_ok     = rep.bottleneck > 0;
rep.single_ok    = rep.bottleneck >= cfg.robot_radius;
rep.formation_ok = rep.bottleneck >= rep.need_circum;
if n > 1
    rep.max_spacing = max(0, (rep.bottleneck - cfg.robot_radius) * 2/((n-1)*sqrt(2)));
else
    rep.max_spacing = inf;
end

if cfg.verbose
    fprintf('--- world_feasibility: %s ---\n', world.file);
    fprintf('  start [%.2f %.2f] -> goal [%.2f %.2f]\n', start_xy, goal_xy);
    fprintf('  bottleneck half-corridor : %.2f m  (corridor width %.2f m)\n', ...
            rep.bottleneck, 2*rep.bottleneck);
    fprintf('  single robot (r=%.2f)    : %s\n', cfg.robot_radius, yn(rep.single_ok));
    fprintf('  %dx%d formation @ s=%.2f  : needs circumradius %.2f m -> %s\n', ...
            n, n, cfg.grid_spacing, rep.need_circum, yn(rep.formation_ok));
    if ~rep.formation_ok
        fprintf(2, '  >> INFEASIBLE. Reduce cfg.grid_spacing to <= %.2f m (suggest %.2f m\n', ...
                rep.max_spacing, 0.8*rep.max_spacing);
        fprintf(2, '     for margin), or let the formation deform through the pinch.\n');
    else
        fprintf('  margin                   : %.2f m\n', rep.bottleneck - rep.need_circum);
        fprintf('  max usable grid_spacing  : %.2f m\n', rep.max_spacing);
    end
end
end

% ------------------------------------------------------------------------
function rep = emptyreport(cfg, D, xs, ys)
rep = struct('bottleneck', 0, 'point_ok', false, 'single_ok', false, ...
             'formation_ok', false, 'need_circum', NaN, 'max_spacing', 0, ...
             'clearance', D, 'xs', xs, 'ys', ys, 'cfg', cfg);
end

function s = yn(tf)
if tf, s = 'OK'; else, s = 'NOT POSSIBLE'; end
end

function idx = xy2idx(p, xs, ys)
[~, c] = min(abs(xs - p(1)));
[~, r] = min(abs(ys - p(2)));
idx = [r c];
end

function tf = connected(Dpad, thr, sLin, gLin, R)
%CONNECTED  4-connected BFS over cells with clearance >= thr.
%
%   Dpad is pre-padded with a one-cell -Inf border, so the border is never
%   free and the four neighbours of any interior linear index i are simply
%   i-1, i+1, i-R, i+R with no bounds checking. That removes the sub2ind /
%   mod / floor arithmetic from the hot loop, which is most of the cost.
free = Dpad >= thr;
if ~free(sLin) || ~free(gLin), tf = false; return; end

n     = numel(free);
seen  = false(n, 1);
queue = zeros(n, 1);
head  = 1; tail = 2;
queue(1)    = sLin;
seen(sLin)  = true;

while head < tail
    cur = queue(head); head = head + 1;
    if cur == gLin, tf = true; return; end

    k = cur - 1;
    if free(k) && ~seen(k), seen(k) = true; queue(tail) = k; tail = tail + 1; end
    k = cur + 1;
    if free(k) && ~seen(k), seen(k) = true; queue(tail) = k; tail = tail + 1; end
    k = cur - R;
    if free(k) && ~seen(k), seen(k) = true; queue(tail) = k; tail = tail + 1; end
    k = cur + R;
    if free(k) && ~seen(k), seen(k) = true; queue(tail) = k; tail = tail + 1; end
end
tf = false;
end
