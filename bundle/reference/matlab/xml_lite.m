function out = xml_lite(action, varargin)
%XML_LITE  Minimal dependency-free XML reader (pure MATLAB, no Java, no toolboxes).
%
%   Works identically in MATLAB and GNU Octave. Written this way on purpose so
%   that sdf2world does not depend on xmlread (Java) or readstruct (R2020b+),
%   which makes the pipeline portable to any MATLAB install and testable offline.
%
%   A node is a struct with fields:
%       .name      char, tag name
%       .attrs     struct of attributes
%       .text      char, concatenated direct text content
%       .kids      cell array of child nodes
%
%   USAGE
%       root = xml_lite('parse_file', filename)
%       root = xml_lite('parse',      xmlstring)
%       c    = xml_lite('child',   node, 'tagname')          % first match or []
%       cs   = xml_lite('children',node, 'tagname')          % cell of matches
%       v    = xml_lite('num',     node, 'tagname', default) % numeric vector
%       s    = xml_lite('str',     node, 'tagname', default) % trimmed text
%
%   See also SDF2WORLD.

switch lower(action)
    case 'parse_file'
        out = xl_parse(fileread(varargin{1}));
    case 'parse'
        out = xl_parse(varargin{1});
    case 'child'
        out = xl_child(varargin{1}, varargin{2});
    case 'children'
        out = xl_children(varargin{1}, varargin{2});
    case 'num'
        out = xl_num(varargin{1}, varargin{2}, varargin{3});
    case 'str'
        out = xl_str(varargin{1}, varargin{2}, varargin{3});
    otherwise
        error('xml_lite:badAction', 'Unknown action "%s".', action);
end
end

% ------------------------------------------------------------------------
function root = xl_parse(s)
% Strip comments, XML declarations, DOCTYPE and CDATA wrappers.
s = regexprep(s, '<!--.*?-->',   '');
s = regexprep(s, '<\?.*?\?>',    '');
s = regexprep(s, '<!DOCTYPE[^>]*>', '');
s = regexprep(s, '<!\[CDATA\[(.*?)\]\]>', '$1');

[tstart, tend, tags] = regexp(s, '<[^>]+>', 'start', 'end', 'match');

root  = xl_node('#document');
stack = {root};
prevEnd = 0;

for k = 1:numel(tags)
    % --- text that sits between the previous tag and this one ---
    if tstart(k) > prevEnd + 1
        txt = s(prevEnd+1 : tstart(k)-1);
        if ~isempty(strtrim(txt))
            stack{end}.text = [stack{end}.text txt];
        end
    end
    prevEnd = tend(k);

    tag  = tags{k};
    body = strtrim(tag(2:end-1));

    if isempty(body)
        continue;
    end

    if body(1) == '/'
        % ---- closing tag: pop and attach to parent ----
        if numel(stack) < 2
            continue;   % stray close tag, ignore
        end
        done  = stack{end};
        stack(end) = [];
        stack{end}.kids{end+1} = done;
        continue;
    end

    selfClosing = (body(end) == '/');
    if selfClosing
        body = strtrim(body(1:end-1));
    end

    % ---- tag name and attributes ----
    sp = find(isspace(body), 1, 'first');
    if isempty(sp)
        name     = body;
        attrText = '';
    else
        name     = body(1:sp-1);
        attrText = body(sp+1:end);
    end

    node = xl_node(name);
    if ~isempty(strtrim(attrText))
        at = regexp(attrText, '([\w:.\-]+)\s*=\s*"([^"]*)"', 'tokens');
        for a = 1:numel(at)
            fld = xl_validname(at{a}{1});
            node.attrs.(fld) = at{a}{2};
        end
    end

    if selfClosing
        stack{end}.kids{end+1} = node;
    else
        stack{end+1} = node; %#ok<AGROW>
    end
end

% Unwind anything left unclosed (tolerates slightly malformed files).
while numel(stack) > 1
    done = stack{end};
    stack(end) = [];
    stack{end}.kids{end+1} = done;
end
root = stack{1};
end

% ------------------------------------------------------------------------
function n = xl_node(name)
n = struct('name', name, 'attrs', struct(), 'text', '', 'kids', {{}});
end

function f = xl_validname(s)
f = regexprep(s, '[^A-Za-z0-9_]', '_');
if isempty(f) || ~isletter(f(1))
    f = ['x' f];
end
end

function c = xl_child(node, name)
c = [];
if isempty(node) || ~isstruct(node) || ~isfield(node, 'kids'), return; end
for i = 1:numel(node.kids)
    if strcmp(node.kids{i}.name, name)
        c = node.kids{i};
        return;
    end
end
end

function cs = xl_children(node, name)
cs = {};
if isempty(node) || ~isstruct(node) || ~isfield(node, 'kids'), return; end
for i = 1:numel(node.kids)
    if strcmp(node.kids{i}.name, name)
        cs{end+1} = node.kids{i}; %#ok<AGROW>
    end
end
end

function v = xl_num(node, name, default)
v = default;
c = xl_child(node, name);
if isempty(c), return; end
t = sscanf(strtrim(c.text), '%f');
if ~isempty(t), v = t(:)'; end
end

function s = xl_str(node, name, default)
s = default;
c = xl_child(node, name);
if isempty(c), return; end
s = strtrim(c.text);
end
