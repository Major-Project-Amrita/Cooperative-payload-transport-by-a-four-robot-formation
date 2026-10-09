%% RUN_EXPERIMENTS  Batch-run the MATLAB side of the comparison.
%
%   Runs the swarm through every world in the list, headless, and writes one
%   CSV per run into logs/. Use this rather than running Simulation7_sdf.m by
%   hand when you are generating data for comparison: visualisation is off, so
%   it is roughly 20x faster and, more importantly, the runs are identical
%   every time because nothing depends on figure timing.
%
%   After this, run the ROS 2 side with swarm_logger.py using the SAME
%   grid_spacing and robot_radius, then call compare_runs.
%
%   Read the console output. If a world reports AUTO-FIX, the spacing printed
%   there is the one you must also set in ROS 2.

clear; clc;

worlds = { 'swarm_world.sdf',        'matlab_simple'
           'swarm_world_medium.sdf', 'matlab_medium' };

% Keep these in one place: they are the numbers that MUST match the ROS 2 side.
SHARED = struct( ...
    'grid_size',    2,     ...
    'grid_spacing', 1.5,   ...   % may be auto-reduced per world; watch the log
    'robot_radius', 0.18,  ...   % TurtleBot3 burger 0.105, waffle 0.22
    'dt',           0.1,   ...
    'V',            0.6);

if exist('logs','dir') ~= 7, mkdir('logs'); end

results = struct('world', {}, 'run', {}, 'spacing', {}, 'summary', {});

for k = 1:size(worlds,1)
    fprintf('\n################ %s ################\n', worlds{k,1});

    CFG_OVERRIDE = SHARED;
    CFG_OVERRIDE.world_file = worlds{k,1};
    CFG_OVERRIDE.run_name   = worlds{k,2};
    CFG_OVERRIDE.viz        = struct('enable', false);
    CFG_OVERRIDE.log        = struct('enable', true);

    Simulation7_sdf;   % reads CFG_OVERRIDE, leaves cfg / S in the workspace

    results(end+1).world   = worlds{k,1}; %#ok<SAGROW>
    results(end).run       = worlds{k,2};
    results(end).spacing   = cfg.grid_spacing;
    results(end).summary   = S;

    clearvars -except worlds SHARED results k
end

fprintf('\n================ BATCH SUMMARY ================\n');
fprintf('%-26s %9s %8s %9s %9s %7s\n', 'world', 'spacing', 'reached', 'path(m)', 'minclr', 'time(s)');
for k = 1:numel(results)
    S = results(k).summary;
    fprintf('%-26s %9.2f %8d %9.2f %9.3f %7.1f\n', results(k).world, ...
        results(k).spacing, S.reached_goal, S.path_length, ...
        S.min_clearance, S.total_time);
end
fprintf('==============================================\n');
fprintf(['\nNow run the ROS 2 side with the SAME grid_spacing shown above,\n' ...
         'then: compare_runs(''logs/matlab_simple.csv'', ''logs/ros2_simple.csv'', ''swarm_world.sdf'')\n\n']);
