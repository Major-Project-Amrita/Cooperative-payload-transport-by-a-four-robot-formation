function world = sdf2world(sdf_file, varargin)
%SDF2WORLD  Convert a Gazebo/gz-sim SDF world into a 2D obstacle map for MATLAB.
%
%   world = SDF2WORLD(sdf_file)
%   world = SDF2WORLD(sdf_file, 'Name', value, ...)
%
%   Reads <collision> geometry from every model in the SDF, composes the
%   model/link/collision poses, slices the result at the robot's height band,
%   and returns axis-aligned-or-rotated boxes and circles in the SAME world
%   coordinates the ROS 2 / Gazebo simulation uses. No rescaling, no
%   re-origining: (0,0) in MATLAB is (0,0) in Gazebo, so trajectories from the
%   two stacks can be overlaid directly.
%
%   OPTIONS
%     'RobotHeight'  (0.35) Only obstacles whose vertical extent overlaps
%                    [ZFloor, RobotHeight] are kept. This is what makes the
%                    goal pad (2 cm tall) a non-obstacle and keeps the 60 cm
%                    pallets as real obstacles.
%     'ZFloor'       (0.03) Bottom of the robot's body band, in metres.
%     'GoalModel'    ('goal_pad') Model whose pose is exported as world.goal.
%                    Searched in visuals too, since a goal pad usually has no
%                    collision geometry.
%     'SkipModels'   ({'ground_plane'}) Model names never treated as obstacles.
%     'Verbose'      (true) Print a parse summary.
%
%   OUTPUT STRUCT
%     world.boxes    1xB struct: .c [x y], .yaw, .h [hx hy] (half-extents),
%                                .name, .zlo, .zhi
%     world.circles  1xC struct: .c [x y], .r, .name, .zlo, .zhi
%     world.bounds   [xmin xmax ymin ymax] tight bounds of all obstacles
%     world.goal     [x y] goal pad centre, or [] if absent
%     world.goal_r   goal pad radius, or NaN
%     world.name     world name attribute
%     world.file     source file
%     world.skipped  names of models dropped, with the reason
%
%   Every obstacle is stored parametrically (centre / yaw / half-extents)
%   rather than as a polygon so that WORLD_DIST can return an exact signed
%   distance. That single signed distance then drives collision checking,
%   robot-radius inflation, clearance metrics and feasibility analysis, which
%   keeps all of them consistent with each other.
%
%   See also WORLD_DIST, LIDAR_SCAN, WORLD_FEASIBILITY, DRAW_WORLD.

opt = struct('RobotHeight', 0.35, 'ZFloor', 0.03, 'GoalModel', 'goal_pad', ...
             'SkipModels', {{'ground_plane'}}, 'Verbose', true);
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(opt, f)
        error('sdf2world:badOption', 'Unknown option "%s".', f);
    end
    opt.(f) = varargin{k+1};
end

if exist(sdf_file, 'file') ~= 2
    error('sdf2world:noFile', 'SDF file not found: %s', sdf_file);
end

root  = xml_lite('parse_file', sdf_file);
sdfn  = xml_lite('child', root, 'sdf');
if isempty(sdfn)
    error('sdf2world:noSdf', 'No <sdf> element in %s', sdf_file);
end
wnode = xml_lite('child', sdfn, 'world');
if isempty(wnode)
    error('sdf2world:noWorld', 'No <world> element in %s', sdf_file);
end

world = struct();
world.file    = sdf_file;
world.name    = 'unnamed';
if isfield(wnode.attrs, 'name'), world.name = wnode.attrs.name; end
world.boxes   = struct('c', {}, 'yaw', {}, 'h', {}, 'name', {}, 'zlo', {}, 'zhi', {});
world.circles = struct('c', {}, 'r', {}, 'name', {}, 'zlo', {}, 'zhi', {});
world.goal    = [];
world.goal_r  = NaN;
world.skipped = {};

inc = xml_lite('children', wnode, 'include');
if ~isempty(inc)
    warning('sdf2world:include', ...
        ['%d <include> element(s) found. sdf2world does not resolve model:// ' ...
         'URIs; those models are NOT in the MATLAB obstacle map. Inline their ' ...
         'geometry in the SDF if they matter for navigation.'], numel(inc));
end

models = xml_lite('children', wnode, 'model');
zband  = [opt.ZFloor, opt.RobotHeight];

for m = 1:numel(models)
    M     = models{m};
    mname = 'model';
    if isfield(M.attrs, 'name'), mname = M.attrs.name; end

    mpose = xml_lite('num', M, 'pose', [0 0 0 0 0 0]);
    mpose = pad_pose(mpose);

    % ---- goal pad: read from visuals as well, it has no collision ----
    if strcmp(mname, opt.GoalModel)
        world.goal = mpose(1:2);
        r = find_first_cylinder_radius(M);
        if ~isempty(r), world.goal_r = r; end
        world.skipped{end+1} = sprintf('%s (goal marker, not an obstacle)', mname);
        continue;
    end

    if any(strcmp(mname, opt.SkipModels))
        world.skipped{end+1} = sprintf('%s (in SkipModels)', mname);
        continue;
    end

    isStatic = strcmpi(xml_lite('str', M, 'static', 'false'), 'true');
    if ~isStatic
        warning('sdf2world:dynamic', ...
            ['Model "%s" is not static. It is added to the MATLAB map at its ' ...
             'spawn pose, but MATLAB will not move it as Gazebo physics would.'], mname);
    end

    links = xml_lite('children', M, 'link');
    nGeom = 0;
    for li = 1:numel(links)
        L     = links{li};
        lpose = pad_pose(xml_lite('num', L, 'pose', [0 0 0 0 0 0]));
        cols  = xml_lite('children', L, 'collision');
        for ci = 1:numel(cols)
            C     = cols{ci};
            cpose = pad_pose(xml_lite('num', C, 'pose', [0 0 0 0 0 0]));
            p     = compose_pose(compose_pose(mpose, lpose), cpose);

            G = xml_lite('child', C, 'geometry');
            if isempty(G), continue; end

            if ~isempty(xml_lite('child', G, 'plane'))
                world.skipped{end+1} = sprintf('%s (plane / ground)', mname);
                continue;
            end

            B = xml_lite('child', G, 'box');
            if ~isempty(B)
                sz = xml_lite('num', B, 'size', [1 1 1]);
                sz = [sz ones(1, 3-numel(sz))];
                zl = p(3) - sz(3)/2;  zh = p(3) + sz(3)/2;
                if ~overlaps(zl, zh, zband)
                    world.skipped{end+1} = sprintf('%s (z=[%.2f %.2f] outside robot band)', mname, zl, zh);
                    continue;
                end
                world.boxes(end+1) = struct('c', p(1:2), 'yaw', p(6), ...
                    'h', [sz(1) sz(2)]/2, 'name', mname, 'zlo', zl, 'zhi', zh);
                nGeom = nGeom + 1;
                continue;
            end

            Cy = xml_lite('child', G, 'cylinder');
            if ~isempty(Cy)
                r  = xml_lite('num', Cy, 'radius', 0.5);
                Ln = xml_lite('num', Cy, 'length', 1.0);
                zl = p(3) - Ln/2;  zh = p(3) + Ln/2;
                if ~overlaps(zl, zh, zband)
                    world.skipped{end+1} = sprintf('%s (z=[%.2f %.2f] outside robot band)', mname, zl, zh);
                    continue;
                end
                world.circles(end+1) = struct('c', p(1:2), 'r', r(1), ...
                    'name', mname, 'zlo', zl, 'zhi', zh);
                nGeom = nGeom + 1;
                continue;
            end

            Sp = xml_lite('child', G, 'sphere');
            if ~isempty(Sp)
                r  = xml_lite('num', Sp, 'radius', 0.5);
                zl = p(3) - r(1);  zh = p(3) + r(1);
                if ~overlaps(zl, zh, zband)
                    world.skipped{end+1} = sprintf('%s (sphere outside robot band)', mname);
                    continue;
                end
                % Slice the sphere at the robot's eye height for the 2D radius.
                zc   = min(max(p(3), zband(1)), zband(2));
                rEff = sqrt(max(r(1)^2 - (zc - p(3))^2, 1e-6));
                world.circles(end+1) = struct('c', p(1:2), 'r', rEff, ...
                    'name', mname, 'zlo', zl, 'zhi', zh);
                nGeom = nGeom + 1;
                continue;
            end

            if ~isempty(xml_lite('child', G, 'mesh'))
                warning('sdf2world:mesh', ...
                    ['Model "%s" uses a <mesh> collision. Meshes are not supported; ' ...
                     'this obstacle is MISSING from the MATLAB map. Replace it with a ' ...
                     'box/cylinder approximation in the SDF for a fair comparison.'], mname);
                world.skipped{end+1} = sprintf('%s (mesh collision, UNSUPPORTED)', mname);
            end
        end
    end

    if nGeom == 0 && ~any(strcmp(mname, opt.SkipModels))
        hasVisual = false;
        for li = 1:numel(links)
            if ~isempty(xml_lite('children', links{li}, 'visual')), hasVisual = true; end
        end
        if hasVisual
            world.skipped{end+1} = sprintf('%s (visual only, no collision -> not an obstacle)', mname);
        end
    end
end

world.bounds = compute_bounds(world);

if opt.Verbose
    fprintf('--- sdf2world: %s ---\n', sdf_file);
    fprintf('  world name      : %s\n', world.name);
    fprintf('  boxes           : %d\n', numel(world.boxes));
    fprintf('  circles         : %d\n', numel(world.circles));
    fprintf('  robot z-band    : [%.2f %.2f] m\n', zband(1), zband(2));
    fprintf('  obstacle bounds : x [%.2f %.2f]  y [%.2f %.2f]\n', world.bounds);
    if ~isempty(world.goal)
        fprintf('  goal pad        : [%.2f %.2f], r = %.2f\n', world.goal, world.goal_r);
    else
        fprintf('  goal pad        : not found (model "%s")\n', opt.GoalModel);
    end
    if ~isempty(world.skipped)
        fprintf('  skipped         : %s\n', strjoin(unique(world.skipped), ', '));
    end
end
end

% ------------------------------------------------------------------------
function p = pad_pose(p)
p = p(:)';
if numel(p) < 6, p = [p zeros(1, 6-numel(p))]; end
p = p(1:6);
end

function p = compose_pose(parent, child)
% 2D-relevant pose composition: xy rotated by the parent yaw, z and yaw added.
% Roll/pitch are ignored, which is exact for these worlds (all zero) and is
% the standard assumption when flattening a 3D world to a 2D navigation map.
cy = cos(parent(6)); sy = sin(parent(6));
p    = parent;
p(1) = parent(1) + cy*child(1) - sy*child(2);
p(2) = parent(2) + sy*child(1) + cy*child(2);
p(3) = parent(3) + child(3);
p(4) = parent(4) + child(4);
p(5) = parent(5) + child(5);
p(6) = parent(6) + child(6);
end

function tf = overlaps(zl, zh, band)
tf = (zh > band(1)) && (zl < band(2));
end

function r = find_first_cylinder_radius(M)
r = [];
links = xml_lite('children', M, 'link');
for li = 1:numel(links)
    for kind = {'visual', 'collision'}
        els = xml_lite('children', links{li}, kind{1});
        for ei = 1:numel(els)
            G = xml_lite('child', els{ei}, 'geometry');
            if isempty(G), continue; end
            Cy = xml_lite('child', G, 'cylinder');
            if ~isempty(Cy)
                v = xml_lite('num', Cy, 'radius', []);
                if ~isempty(v), r = v(1); return; end
            end
        end
    end
end
end

function b = compute_bounds(world)
xs = []; ys = [];
for i = 1:numel(world.boxes)
    B = world.boxes(i);
    R = [cos(B.yaw) -sin(B.yaw); sin(B.yaw) cos(B.yaw)];
    cn = [-B.h(1) -B.h(2); B.h(1) -B.h(2); B.h(1) B.h(2); -B.h(1) B.h(2)];
    cn = (R*cn')' + B.c;
    xs = [xs; cn(:,1)]; ys = [ys; cn(:,2)]; %#ok<AGROW>
end
for i = 1:numel(world.circles)
    C = world.circles(i);
    xs = [xs; C.c(1)-C.r; C.c(1)+C.r]; %#ok<AGROW>
    ys = [ys; C.c(2)-C.r; C.c(2)+C.r]; %#ok<AGROW>
end
if isempty(xs)
    b = [-1 1 -1 1];
else
    b = [min(xs) max(xs) min(ys) max(ys)];
end
end
