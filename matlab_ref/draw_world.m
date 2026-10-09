function h = draw_world(world, varargin)
%DRAW_WORLD  Render an SDF-derived 2D obstacle map, colour-coded by model type.
%
%   h = DRAW_WORLD(world)
%   h = DRAW_WORLD(world, 'Name', value, ...)
%
%   OPTIONS
%     'Inflate'   (0)     draw a dashed robot-radius inflation outline
%     'Labels'    (false) print model names on the obstacles
%     'Alpha'     (1)     face alpha
%     'Goal'      (true)  draw the goal pad
%
%   Colours follow the SDF semantics so the MATLAB figure reads like the
%   Gazebo scene: grey walls, brown shelves/pallets, red pillars.
%
%   See also SDF2WORLD, WORLD_DIST.

opt = struct('Inflate', 0, 'Labels', false, 'Alpha', 1, 'Goal', true);
for k = 1:2:numel(varargin)
    if isfield(opt, varargin{k}), opt.(varargin{k}) = varargin{k+1}; end
end

h = [];
hold on;

for i = 1:numel(world.boxes)
    B = world.boxes(i);
    R = [cos(B.yaw) -sin(B.yaw); sin(B.yaw) cos(B.yaw)];
    c = [-B.h(1) -B.h(2); B.h(1) -B.h(2); B.h(1) B.h(2); -B.h(1) B.h(2)];
    c = (R*c')' + B.c;
    col = model_color(B.name);
    h(end+1) = fill(c(:,1), c(:,2), col, 'EdgeColor', 'k', ...
                    'FaceAlpha', opt.Alpha, 'LineWidth', 0.5); %#ok<AGROW>
    if opt.Inflate > 0
        ci = (R*([-B.h(1)-opt.Inflate -B.h(2)-opt.Inflate; ...
                   B.h(1)+opt.Inflate -B.h(2)-opt.Inflate; ...
                   B.h(1)+opt.Inflate  B.h(2)+opt.Inflate; ...
                  -B.h(1)-opt.Inflate  B.h(2)+opt.Inflate])')' + B.c;
        plot(ci([1:4 1],1), ci([1:4 1],2), ':', 'Color', [0.45 0.45 0.45]);
    end
    if opt.Labels
        text(B.c(1), B.c(2), strrep(B.name,'_','\_'), 'FontSize', 7, ...
             'HorizontalAlignment', 'center', 'Color', [0.15 0.15 0.15]);
    end
end

th = linspace(0, 2*pi, 48);
for i = 1:numel(world.circles)
    C   = world.circles(i);
    col = model_color(C.name);
    h(end+1) = fill(C.c(1)+C.r*cos(th), C.c(2)+C.r*sin(th), col, ...
                    'EdgeColor', 'k', 'FaceAlpha', opt.Alpha); %#ok<AGROW>
    if opt.Inflate > 0
        plot(C.c(1)+(C.r+opt.Inflate)*cos(th), C.c(2)+(C.r+opt.Inflate)*sin(th), ...
             ':', 'Color', [0.45 0.45 0.45]);
    end
    if opt.Labels
        text(C.c(1), C.c(2), strrep(C.name,'_','\_'), 'FontSize', 7, ...
             'HorizontalAlignment', 'center');
    end
end

if opt.Goal && ~isempty(world.goal)
    r = world.goal_r; if isnan(r), r = 0.5; end
    fill(world.goal(1)+r*cos(th), world.goal(2)+r*sin(th), [0 0.85 0], ...
         'EdgeColor', [0 0.5 0], 'FaceAlpha', 0.35);
end

axis equal;
b = world.bounds;
xlim([b(1)-0.5 b(2)+0.5]); ylim([b(3)-0.5 b(4)+0.5]);
end

% ------------------------------------------------------------------------
function col = model_color(name)
if     ~isempty(strfind(lower(name), 'wall'));   col = [0.50 0.50 0.55];
elseif ~isempty(strfind(lower(name), 'shelf'));  col = [0.60 0.42 0.20];
elseif ~isempty(strfind(lower(name), 'pallet')); col = [0.45 0.30 0.14];
elseif ~isempty(strfind(lower(name), 'pillar')); col = [0.80 0.20 0.20];
elseif ~isempty(strfind(lower(name), 'mover'));  col = [0.95 0.55 0.10];  % dynamic
else                                             col = [0.60 0.60 0.60];
end
end
