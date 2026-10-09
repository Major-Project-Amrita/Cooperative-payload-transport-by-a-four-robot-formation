function wp = contract_waypoints(json_file)
%CONTRACT_WAYPOINTS  Optional global route stored in a motion contract.
%
%   wp = CONTRACT_WAYPOINTS(json_file)
%
%   Returns the Nx2 "waypoints" array of the contract JSON, or [] when the
%   file has none (then the mission drives straight for the SDF goal pad).
%   The maze world needs it: a purely reactive steering law has no reason to
%   drive away from the goal toward a gate, so the global route is part of
%   the scenario and identical for every algorithm and both stacks.
wp = [];
if isempty(json_file) || exist(json_file, 'file') ~= 2, return; end
try
    c = jsondecode(fileread(json_file));
catch
    return;
end
if isfield(c, 'waypoints') && ~isempty(c.waypoints)
    wp = c.waypoints;
    if iscell(wp), wp = cell2mat(cellfun(@(r) r(:)', wp(:), 'UniformOutput', false)); end
    if size(wp, 2) ~= 2, wp = reshape(wp, [], 2); end
end
end
