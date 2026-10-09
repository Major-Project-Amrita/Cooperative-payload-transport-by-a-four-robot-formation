function [movers, meta] = load_dynamic_obstacles(json_file, world)
%LOAD_DYNAMIC_OBSTACLES  Read the shared dynamic-obstacle motion contract.
%
%   [movers, meta] = LOAD_DYNAMIC_OBSTACLES(json_file, world)
%
%   json_file  path to dynamic_obstacles.json - the SAME file the ROS 2
%              mover node consumes. One contract, two stacks, so the obstacle
%              motion in MATLAB and in Gazebo is identical by construction.
%   world      (optional) static world struct from SDF2WORLD. When given, the
%              spawn-time poses are checked against static obstacles and the
%              goal pad, the same validation the static pipeline applies to
%              robots and slots.
%
%   movers     1xM struct array, one entry per mover in file order:
%       .name     model name, matches the SDF model (mover1, mover2, ...)
%       .shape    'box' | 'circle'
%       .size     [sx sy] full footprint extents (boxes)
%       .r        effective 2D radius (boxes use the footprint circumradius
%                 for prediction and VFH+ enlargement; the exact box geometry
%                 is still what WORLD_DIST tests)
%       .route    2x2 [x y; x y] waypoints
%       .speed    m/s
%       .t0       seconds the mover waits at route(1,:) before departing
%       .wait     seconds to pause at each endpoint (loop=true only)
%       .loop     true = shuttle forever, false = stop at route(2,:)
%       .c        1x2 current centre (evaluated at t = 0)
%   meta       .frame, .dt as declared in the JSON header.
%
%   Parsing uses jsondecode (MATLAB R2016b+, Octave 7+); mini_json below is a
%   dependency-free recursive-descent fallback for anything older.
%
%   See also UPDATE_WORLD_DYN, SIMULATION8_DYNAMIC.

if nargin < 2, world = []; end
if exist(json_file, 'file') ~= 2
    error('load_dynamic_obstacles:noFile', 'JSON not found: %s', json_file);
end

txt = fileread(json_file);
if exist('jsondecode', 'builtin') == 5 || exist('jsondecode', 'file') == 2
    doc = jsondecode(txt);
else
    doc = mini_json(txt);
end

frame = 'world';
if isfield(doc, 'frame') && ~isempty(doc.frame), frame = doc.frame; end
dt = 0.1;
if isfield(doc, 'dt') && ~isempty(doc.dt), dt = doc.dt; end

% jsondecode returns a struct ARRAY when the JSON array has >1 element and a
% scalar struct when it has exactly one - normalise both to a cell array.
if ~isfield(doc, 'movers') || isempty(doc.movers)
    error('load_dynamic_obstacles:noMovers', 'No "movers" array in %s.', json_file);
end
mv = doc.movers;
if isstruct(mv)
    entries = arrayfun(@(k) mv(k), 1:numel(mv), 'UniformOutput', false);
elseif iscell(mv)
    entries = mv;
else
    error('load_dynamic_obstacles:badMovers', '"movers" must be an array of objects.');
end

movers = struct('name', {}, 'shape', {}, 'size', {}, 'r', {}, ...
                'route', {}, 'speed', {}, 't0', {}, 'wait', {}, ...
                'loop', {}, 'c', {});

for k = 1:numel(entries)
    e = entries{k};
    m.name  = char(e.name);
    m.shape = char(e.shape);
    m.size  = [e.size(1) e.size(2)];
    if isfield(e, 'radius') && ~isempty(e.radius) && e.radius > 0
        m.r = e.radius;
    elseif strcmpi(m.shape, 'circle')
        m.r = m.size(1) / 2;
    else
        % Box: circumradius of the footprint. Slightly conservative (a square
        % of side s is circumscribed by this circle), which is the safe
        % direction for the prediction margin and for VFH+ enlargement.
        m.r = 0.5 * hypot(m.size(1), m.size(2));
    end
    rt = e.route;
    if isstruct(rt)
        m.route = rt;   % cannot happen for a numeric array; guards cells
    else
        m.route = [rt(1,:) ; rt(2,:)];
    end
    m.speed = e.speed;
    m.t0    = e.t0;
    m.wait  = 0;
    if isfield(e, 'wait'), m.wait = e.wait; end
    m.loop  = false;
    if isfield(e, 'loop')
        if ischar(e.loop) || isstring(e.loop)
            m.loop = strcmpi(char(e.loop), 'true');
        else
            m.loop = ~isequal(e.loop, false) && e.loop ~= 0;
        end
    end
    m.c     = eval_mover_pose(m, 0);

    movers(end+1) = m; %#ok<AGROW>
end

meta.frame = frame;
meta.dt    = dt;

if ~isempty(world) && isfield(world, 'boxes')
    validate_spawn(movers, world);
end
end

% ------------------------------------------------------------------------
function validate_spawn(movers, world)
% Spawn-time sanity: a mover that starts inside a wall, or parked on the goal
% pad, would poison the comparison before either stack has done anything.
if ~isfield(world, 'goal'), return; end
for k = 1:numel(movers)
    m  = movers(k);
    pc = mover_corners(m, m.c);
    [d, ~] = world_dist(pc, world);
    if min(d) <= 0
        error('load_dynamic_obstacles:spawnOverlap', ...
            ['Mover %s intersects a static obstacle at spawn. Its route ' ...
             '[%.2f %.2f] -> [%.2f %.2f] must lie inside the aisle.'], ...
            m.name, m.route(1,1), m.route(1,2), m.route(2,1), m.route(2,2));
    end
    if ~isnan(world.goal_r) && norm(m.c - world.goal) < world.goal_r + m.r
        error('load_dynamic_obstacles:spawnOnGoal', ...
            'Mover %s spawns inside the goal pad radius.', m.name);
    end
end
end

function p = mover_corners(m, c)
% Axis-aligned footprint sample points (movers never rotate).
if strcmpi(m.shape, 'circle')
    th = linspace(0, 2*pi, 12);
    p  = [c(1)+m.r*cos(th)', c(2)+m.r*sin(th)'];
else
    p = [c + [-m.size(1)/2, -m.size(2)/2];
         c + [ m.size(1)/2, -m.size(2)/2];
         c + [ m.size(1)/2,  m.size(2)/2];
         c + [-m.size(1)/2,  m.size(2)/2]];
end
end

% ------------------------------------------------------------------------
% eval_mover_pose lives in its own file (eval_mover_pose.m): it is the single
% source of truth for the motion model, shared by every prediction consumer.
% ------------------------------------------------------------------------

% ========================================================================
function val = mini_json(txt)
%MINI_JSON  Minimal recursive-descent JSON parser (fallback, Octave < 7).
%   Handles objects, arrays, numbers, strings, true/false/null. Sufficient
%   for dynamic_obstacles.json; not a general-purpose JSON library.
pos = 1;
val = parse_value();
if skip_ws() <= numel(txt)
    error('mini_json:trailing', 'Trailing characters after JSON value at %d.', pos);
end

    function v = parse_value()
        pos = skip_ws();
        if pos > numel(txt), error('mini_json:unexpectedEnd', 'Unexpected end of input.'); end
        c = txt(pos);
        switch c
            case '{', v = parse_object();
            case '[', v = parse_array();
            case '"', v = parse_string();
            otherwise
                if strncmp(txt(pos:min(end,pos+3)), 'true', 4)
                    v = true; pos = pos + 4;
                elseif strncmp(txt(pos:min(end,pos+4)), 'false', 5)
                    v = false; pos = pos + 5;
                elseif strncmp(txt(pos:min(end,pos+3)), 'null', 4)
                    v = []; pos = pos + 4;
                else
                    v = parse_number();
                end
        end
    end

    function v = parse_object()
        pos = pos + 1;  % consume '{'
        v = struct();
        pos = skip_ws();
        if pos <= numel(txt) && txt(pos) == '}'
            pos = pos + 1; return;
        end
        while true
            pos = skip_ws();
            key = parse_string();
            pos = skip_ws();
            if pos > numel(txt) || txt(pos) ~= ':'
                error('mini_json:colon', 'Expected '':'' at %d.', pos);
            end
            pos = pos + 1;
            if isvarname(key), fname = key; else, fname = ['f' num2str(pos)]; end
            v.(fname) = parse_value();
            pos = skip_ws();
            if pos > numel(txt), error('mini_json:objectEnd', 'Unterminated object.'); end
            if txt(pos) == ','
                pos = pos + 1;
            elseif txt(pos) == '}'
                pos = pos + 1; return;
            else
                error('mini_json:objectSep', 'Expected '','' or ''}'' at %d.', pos);
            end
        end
    end

    function v = parse_array()
        pos = pos + 1;  % consume '['
        items = {};
        pos = skip_ws();
        if pos <= numel(txt) && txt(pos) == ']'
            pos = pos + 1; v = {}; return;
        end
        while true
            items{end+1} = parse_value(); %#ok<AGROW>
            pos = skip_ws();
            if pos > numel(txt), error('mini_json:arrayEnd', 'Unterminated array.'); end
            if txt(pos) == ','
                pos = pos + 1;
            elseif txt(pos) == ']'
                pos = pos + 1; break;
            else
                error('mini_json:arraySep', 'Expected '','' or '']'' at %d.', pos);
            end
        end
        % Homogeneous numeric arrays become matrices, like jsondecode.
        nums = cellfun(@isnumeric, items);
        if all(nums) && ~isempty(items)
            try
                v = vertcat(items{:});
                return;
            catch
            end
        end
        v = items;
    end

    function s = parse_string()
        pos = pos + 1;  % consume opening quote
        p0 = pos;
        while pos <= numel(txt) && txt(pos) ~= '"'
            if txt(pos) == '\', pos = pos + 1; end
            pos = pos + 1;
        end
        if pos > numel(txt), error('mini_json:stringEnd', 'Unterminated string.'); end
        s = txt(p0:pos-1);
        pos = pos + 1;  % consume closing quote
    end

    function v = parse_number()
        p0 = pos;
        while pos <= numel(txt) && ismember(txt(pos), '+-0123456789.eE')
            pos = pos + 1;
        end
        v = str2double(txt(p0:pos-1));
        if isnan(v), error('mini_json:number', 'Bad number at %d.', p0); end
    end

    function p = skip_ws()
        p = pos;
        ws = [' ' char(10) char(13) char(9)];
        while p <= numel(txt) && ismember(txt(p), ws)
            p = p + 1;
        end
        pos = p;
    end
end
