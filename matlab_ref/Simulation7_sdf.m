%% =====================================================================
%  Simulation7_sdf.m
%  4-ROBOT SWARM: SCATTERED START -> FORM SQUARE -> RIGID TRANSPORT
%  running in the SAME world your ROS 2 / Gazebo stack uses.
%
%  Adapted from Simulation7.m. The control laws (APF slot-seeking, convoy
%  queueing, p-VFH, heading smoothing, stuck-escape) are unchanged. What
%  changed is everything around them:
%
%    1. The map comes from the SDF, in Gazebo's own coordinates. No rescaling
%       and no re-origining, so a MATLAB trajectory and a ROS 2 trajectory can
%       be plotted on the same axes without a transform.
%    2. Perception is a raycast LaserScan, not a point cloud over obstacle
%       interiors - the same information a ROS 2 node reads off /scan.
%    3. Collision uses an exact signed distance inflated by the robot radius,
%       and tests the whole formation body, not just the four slot points.
%    4. A feasibility check runs before the sim, because a rigid formation
%       that cannot fit through the corridor will otherwise "fail" for reasons
%       that have nothing to do with the controller.
%    5. Every step is logged to CSV for quantitative comparison.
%
%  See README.md for the comparison protocol.
% =====================================================================
% Running this file directly clears the workspace as usual. A batch driver can
% instead define CFG_OVERRIDE (a struct of cfg fields) before calling it, and
% those fields are merged in after the defaults below.
if ~exist('CFG_OVERRIDE', 'var')
    clc; clear; close all;
end

%% =====================================================================
%  CONFIGURATION
% =====================================================================
cfg.world_file   = 'swarm_world_medium.sdf';   % or 'swarm_world_medium.sdf'
cfg.run_name     = 'matlab_simple';     % used for the log filename

cfg.grid_size    = 2;                   % NxN grid of robots
cfg.num_robots   = cfg.grid_size^2;
cfg.dt           = 0.1;
cfg.viz.colors   = hsv(cfg.num_robots);

% ---- Physical parameters. MATCH THESE TO YOUR ROS 2 ROBOT ----
cfg.robot_radius  = 0.18;    % TurtleBot3 burger ~0.105, waffle ~0.22
cfg.robot_height  = 0.35;    % z-band used to slice the 3D world into 2D
cfg.grid_spacing  = 1.5;     % set by the feasibility check below - see notes
cfg.collision_mode = 'payload';  % 'payload' | 'frame' | 'slots'

% ---- Start / goal. Defaults are read from the SDF itself. ----
cfg.interactive        = false;   % true -> click start/goal as before
cfg.spawn_in_formation = false;
cfg.auto_fix_spacing   = true;    % shrink grid_spacing if the corridor is too tight
cfg.start_point        = [-12  0];          % clear start area in both worlds
cfg.waypoints          = [];                % [] -> use the SDF goal_pad
cfg.scatter_center     = [-12  0];
cfg.scatter_radius     = 2.2;

% --- Phase 2 (rigid transport) ---
cfg.timesteps       = 5000;
cfg.goal_reach_dist = 0.5;
cfg.max_turn_rate   = 0.3;
cfg.V               = 0.6;
cfg.w_goal          = 1.0;
cfg.w_vfh           = 2.0;
cfg.heading_smooth_alpha = 0.35;

% --- Phase 1 (forming up) ---
cfg.forming.tolerance     = 0.3;
cfg.forming.max_steps     = 2500;
cfg.forming.w_slot        = 2.0;
cfg.forming.w_repulse     = 1.5;
cfg.forming.repulse_dist  = 1.2;
cfg.forming.max_turn_rate = 0.5;
cfg.forming.V             = 0.5;
cfg.forming.slow_radius   = 1.5;
cfg.forming.heading_smooth_alpha = 0.35;
cfg.forming.stuck_escape_steps   = 5;
cfg.forming.backoff_dist         = 0.3;

% --- Phase 1 convoy/queueing behavior ---
cfg.forming.convoy_trigger_dist = 2.0;
cfg.forming.convoy_exit_dist    = 3.0;
cfg.forming.convoy_max_obstacle = 2.0;  % only queue behind obstacles this wide
                                        % or narrower - see obstacle_extents.m
cfg.forming.convoy_min_group    = 2;
cfg.forming.convoy_spacing      = 1.0;
cfg.forming.trail_max_len       = 4000;

% --- p-VFH parameters ---
cfg.vfh.num_sectors      = 72;
cfg.vfh.n_beams          = 288;   % set to your LiDAR's beam count
cfg.vfh.sigma            = 1.0;
cfg.vfh.sense_range_base = 4.0;
cfg.vfh.danger_dist_base = 1.2;   % was 2.5; see NOTE below
cfg.vfh.turn_limit       = pi/4;
cfg.vfh.threshold        = 0.3;
cfg.vfh.smooth_window    = 5;
cfg.vfh.avoid_gain       = 5;
cfg.vfh.hysteresis_weight = 0.6;
cfg.vfh.avoid_falloff_power = 3;
cfg.vfh.circular_smooth  = true;
cfg.vfh.enlarge          = true;  % VFH+ enlarged obstacles, sized by the body
cfg.vfh.auto_sigma       = true;  % derive sigma from the required clearance
cfg.vfh.react_gap        = 1.0;   % start reacting this far beyond the body
cfg.stuck_escape_steps   = 5;
cfg.escape_step          = 0.4;   % lookahead used when probing escape headings
cfg.epsilon              = 1e-6;

% NOTE ON danger_dist_base
% The original 2.5 m was tuned for a 50x50 open world. This warehouse aisle is
% only ~10 m wide and the chicane pinches to ~2.5 m, so at 2.5 m the repulsion
% term sees both walls at once, they very nearly cancel, and what is left is
% noise that fights the goal term. 1.2 m keeps repulsion as a genuine
% last-resort reflex. If you change it, change it on the ROS 2 side too.

% --- Visualization / logging ---
cfg.viz.enable  = true;      % false -> headless, much faster, for batch runs
cfg.viz.every   = 5;         % redraw every k steps
cfg.viz.pause_time = 0.001;
cfg.viz.robot_marker_size = 8;
cfg.viz.square_line_width = 1.5;
cfg.viz.waypoint_marker_size = 10;
cfg.viz.leader_marker_width = 2;
cfg.log.enable  = true;
cfg.log.file    = '';        % '' -> auto: logs/<run_name>.csv

%% --- merge batch overrides, if any ---
if exist('CFG_OVERRIDE', 'var') && isstruct(CFG_OVERRIDE)
    ov = fieldnames(CFG_OVERRIDE);
    for oi = 1:numel(ov)
        val = CFG_OVERRIDE.(ov{oi});
        if isstruct(val) && isfield(cfg, ov{oi}) && isstruct(cfg.(ov{oi}))
            sv = fieldnames(val);
            for si = 1:numel(sv), cfg.(ov{oi}).(sv{si}) = val.(sv{si}); end
        else
            cfg.(ov{oi}) = val;
        end
    end
    cfg.num_robots  = cfg.grid_size^2;
    cfg.viz.colors  = hsv(cfg.num_robots);
end

%% =====================================================================
%  LOAD THE WORLD FROM SDF
% =====================================================================
world = sdf2world(cfg.world_file, 'RobotHeight', cfg.robot_height);

if isempty(cfg.waypoints)
    if isempty(world.goal)
        error('No goal_pad in the SDF and cfg.waypoints is empty.');
    end
    cfg.waypoints = world.goal;
end

cfg.viz.xlim = [world.bounds(1)-0.5, world.bounds(2)+0.5];
cfg.viz.ylim = [world.bounds(3)-0.5, world.bounds(4)+0.5];

%% =====================================================================
%  FEASIBILITY CHECK  (run this before you trust any comparison)
% =====================================================================
fcfg = struct('res', 0.05, 'robot_radius', cfg.robot_radius, ...
              'grid_size', cfg.grid_size, 'grid_spacing', cfg.grid_spacing);
rep = world_feasibility(world, cfg.start_point, cfg.waypoints(end,:), fcfg);

if ~rep.single_ok
    error(['This world is impassable even for a single robot of radius %.2f m. ' ...
           'Fix the world or the robot radius before comparing anything.'], cfg.robot_radius);
end
if ~rep.formation_ok
    if cfg.auto_fix_spacing
        newspace = 0.8 * rep.max_spacing;
        fprintf(2, ['\n>> AUTO-FIX: grid_spacing %.2f m -> %.2f m so the formation ' ...
                    'fits the %.2f m corridor.\n'], cfg.grid_spacing, newspace, 2*rep.bottleneck);
        fprintf(2, '   Apply the SAME spacing in ROS 2 or the comparison is meaningless.\n\n');
        cfg.grid_spacing = newspace;
    else
        error(['Formation does not fit (needs %.2f m circumradius, corridor gives ' ...
               '%.2f m). Reduce cfg.grid_spacing to <= %.2f m.'], ...
               rep.need_circum, rep.bottleneck, rep.max_spacing);
    end
end

%% =====================================================================
%  FORMATION GEOMETRY
% =====================================================================
n = cfg.grid_size;
half_diag = ((n-1)*cfg.grid_spacing)*sqrt(2)/2;

[gx, gy] = meshgrid(0:n-1, 0:n-1);
offsets_body = [gx(:) gy(:)] * cfg.grid_spacing;
offsets_body = offsets_body - mean(offsets_body, 1);

% Scattered start ring, kept inside the clear start area.
scatter_theta = linspace(0, 2*pi, cfg.num_robots+1); scatter_theta(end) = [];
cfg.robots_start = cfg.scatter_center + ...
    cfg.scatter_radius*[cos(scatter_theta') sin(scatter_theta')];

if cfg.interactive
    [cfg.robots_start, cfg.start_point, cfg.waypoints] = pick_points_sdf(cfg, world);
end

start_pos     = cfg.start_point;
sp_dir        = cfg.waypoints(1,:) - start_pos;
start_heading = atan2(sp_dir(2), sp_dir(1));
R0            = rot2d(start_heading);
target_slots  = (R0*offsets_body')' + start_pos;

if cfg.spawn_in_formation
    assigned_slots = target_slots;
    robot_offsets  = offsets_body;
else
    cost_matrix = zeros(cfg.num_robots);
    for i = 1:cfg.num_robots
        for j = 1:cfg.num_robots
            cost_matrix(i,j) = norm(cfg.robots_start(i,:) - target_slots(j,:));
        end
    end
    best_perm      = assign_slots(cost_matrix);
    assigned_slots = target_slots(best_perm,:);
    robot_offsets  = offsets_body(best_perm,:);
end

%% =====================================================================
%  SPAWN VALIDATION  (now uses inflated, exact clearance)
% =====================================================================
for i = 1:cfg.num_robots
    d = world_dist(cfg.robots_start(i,:), world);
    if d <= cfg.robot_radius
        error(['SPAWN ERROR: robot %d at [%.2f %.2f] has clearance %.2f m, ' ...
               'less than its radius %.2f m.'], i, cfg.robots_start(i,:), d, cfg.robot_radius);
    end
end
if world_dist(cfg.start_point, world) <= cfg.robot_radius
    error('SPAWN ERROR: formation start point is inside/too near an obstacle.');
end
for i = 1:size(cfg.waypoints,1)
    if world_dist(cfg.waypoints(i,:), world) <= cfg.robot_radius
        error('WAYPOINT ERROR: goal waypoint %d is inside/too near an obstacle.', i);
    end
end
for i = 1:cfg.num_robots
    d = world_dist(assigned_slots(i,:), world);
    if d <= cfg.robot_radius
        error(['SLOT ERROR: assembly slot %d at [%.2f %.2f] has clearance %.2f m. ' ...
               'Move cfg.start_point or reduce cfg.grid_spacing.'], ...
               i, assigned_slots(i,:), d);
    end
end

%% --- Visualization setup ---
if cfg.viz.enable
    figure('Color','w','Name','Swarm in SDF world'); hold on; grid on;
    draw_world(world, 'Inflate', cfg.robot_radius);
    xlim(cfg.viz.xlim); ylim(cfg.viz.ylim);
end

%% --- Logging buffers ---
LOG = struct('t', [], 'phase', [], 'lx', [], 'ly', [], 'lh', [], ...
             'dist', [], 'clr', [], 'rx', [], 'ry', []);

%% =====================================================================
%  PHASE 1 -- FORMING UP
% =====================================================================
sim_t = 0;

if cfg.spawn_in_formation
    pos     = assigned_slots;
    heading = repmat(start_heading, cfg.num_robots, 1);
    formed  = true;
else
    pos     = cfg.robots_start;
    heading = zeros(cfg.num_robots,1);
    for i = 1:cfg.num_robots
        d0 = assigned_slots(i,:) - pos(i,:);
        heading(i) = atan2(d0(2), d0(1));
    end

    num_obstacles      = numel(world.boxes) + numel(world.circles);
    obs_ext            = obstacle_extents(world);
    convoy_ok          = obs_ext <= cfg.forming.convoy_max_obstacle;
    stuck_counter_form = zeros(cfg.num_robots,1);
    in_convoy          = zeros(cfg.num_robots,1);
    convoy_trail       = cell(num_obstacles,1);
    for c = 1:num_obstacles, convoy_trail{c} = zeros(0,2); end

    if ~any(convoy_ok)
        fprintf('  (no compact obstacles: convoy/queueing disabled in this world)\n');
    end

    formed = false;
    for t = 1:cfg.forming.max_steps
        new_pos = pos;
        sim_t   = sim_t + cfg.dt;

        % Nearest obstacle per robot, straight off the exact signed distance.
        [nearest_d, nearest_id] = world_dist(pos, world);

        new_in_convoy = zeros(cfg.num_robots,1);
        for c = 1:num_obstacles
            if ~convoy_ok(c), continue; end   % walls and shelf racks never trigger convoy
            eligible = find((nearest_id==c & nearest_d<cfg.forming.convoy_trigger_dist) | ...
                            (in_convoy==c  & nearest_d<cfg.forming.convoy_exit_dist));
            if numel(eligible) >= cfg.forming.convoy_min_group
                new_in_convoy(eligible) = c;
            end
        end
        in_convoy = new_in_convoy;

        leader_of = zeros(cfg.num_robots,1);
        for c = 1:num_obstacles
            members = find(in_convoy==c);
            if ~isempty(members), leader_of(members) = min(members); end
        end

        for i = 1:cfg.num_robots
            goal_heading_i = atan2(assigned_slots(i,2)-pos(i,2), ...
                                   assigned_slots(i,1)-pos(i,1));

            if in_convoy(i) ~= 0 && leader_of(i) ~= i
                c         = in_convoy(i);
                leader_id = leader_of(i);
                members   = find(in_convoy==c);
                followers = members(members ~= leader_id);
                d_to_lead = sqrt(sum((pos(followers,:) - pos(leader_id,:)).^2, 2));
                [~, order] = sort(d_to_lead);
                rank      = find(followers(order) == i, 1);
                target_pt = point_along_trail_behind(convoy_trail{c}, ...
                                rank*cfg.forming.convoy_spacing, pos(leader_id,:), pos(i,:));
                desired_h_raw = atan2(target_pt(2)-pos(i,2), target_pt(1)-pos(i,1));
                speed_i = cfg.forming.V;

            elseif stuck_counter_form(i) > cfg.forming.stuck_escape_steps
                [found_escape, escape_heading] = find_escape_heading_sdf( ...
                    pos(i,:), [0 0], world, cfg.robot_radius, goal_heading_i, 36, ...
                    'slots', cfg.forming.backoff_dist);
                if found_escape, desired_h_raw = escape_heading;
                else,            desired_h_raw = heading(i); end
                e_i = norm(assigned_slots(i,:) - pos(i,:));
                speed_i = cfg.forming.V * min(1, e_i/cfg.forming.slow_radius);

            else
                % ---- APF: attractive pull to this robot's assigned slot,
                %      repulsive push from nearby formation-mates (skip
                %      anyone already queued in the same convoy, since they
                %      are handled by the convoy trail-following logic above,
                %      not by potential-field repulsion). ----
                is_mate = (1:cfg.num_robots)' ~= i & ...
                          ~(in_convoy(i) ~= 0 & in_convoy == in_convoy(i));
                neighbors_i = pos(is_mate, :);

                apf_cfg_i = struct('w_att', cfg.forming.w_slot, ...
                                   'w_rep', cfg.forming.w_repulse, ...
                                   'rep_dist', cfg.forming.repulse_dist);
                apf_vec = apf_force(pos(i,:), assigned_slots(i,:), neighbors_i, ...
                                    apf_cfg_i, cfg.epsilon);

                % ---- p-VFH: steer toward the APF result while dodging
                %      obstacles (CASCADED APF -> VFH architecture). ----
                virtual_goal = pos(i,:) + apf_vec;
                vec_vfh = pvfh_scan(pos(i,:), world, virtual_goal, ...
                                    cfg.robot_radius, cfg.vfh, cfg.epsilon, heading(i));

                desired_h_raw = atan2(vec_vfh(2), vec_vfh(1));
                e_i = norm(assigned_slots(i,:) - pos(i,:));
                speed_i = cfg.forming.V * min(1, e_i/cfg.forming.slow_radius);
            end

            desired_h = wrap_pi(heading(i) + cfg.forming.heading_smooth_alpha * ...
                                wrap_pi(desired_h_raw - heading(i)));
            dh = wrap_pi(desired_h - heading(i));
            dh = max(min(dh, cfg.forming.max_turn_rate), -cfg.forming.max_turn_rate);
            heading(i) = wrap_pi(heading(i) + dh);

            cand = pos(i,:) + speed_i*[cos(heading(i)) sin(heading(i))]*cfg.dt;
            if world_dist(cand, world) > cfg.robot_radius
                new_pos(i,:) = cand;
                stuck_counter_form(i) = 0;
            else
                stuck_counter_form(i) = stuck_counter_form(i) + 1;
                backoff = pos(i,:) - cfg.forming.backoff_dist*[cos(heading(i)) sin(heading(i))];
                if world_dist(backoff, world) > cfg.robot_radius
                    new_pos(i,:) = backoff;
                end
            end

            if in_convoy(i) ~= 0 && leader_of(i) == i
                c = in_convoy(i);
                convoy_trail{c} = [convoy_trail{c}; new_pos(i,:)];
                if size(convoy_trail{c},1) > cfg.forming.trail_max_len
                    convoy_trail{c}(1,:) = [];
                end
            end
        end
        pos  = new_pos;
        errs = sqrt(sum((pos-assigned_slots).^2,2));

        LOG = log_step(LOG, cfg, sim_t, 1, mean(pos,1), mean(heading), ...
                       max(errs), min(world_dist(pos, world)), pos);

        if cfg.viz.enable && (mod(t, cfg.viz.every)==0 || t==1)
            cla; draw_world(world, 'Inflate', cfg.robot_radius);
            plot(cfg.start_point(1), cfg.start_point(2), 'gp', 'MarkerSize',14, ...
                 'MarkerFaceColor','g','MarkerEdgeColor','k');
            plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'r*', ...
                 'MarkerSize', cfg.viz.waypoint_marker_size, 'LineWidth', 2);
            draw_grid(target_slots, n, 1, 'k--');
            for i = 1:cfg.num_robots
                draw_robot(pos(i,:), cfg.robot_radius, cfg.viz.colors(i,:));
                plot(assigned_slots(i,1), assigned_slots(i,2), 'x', 'MarkerSize', 8, ...
                     'Color', cfg.viz.colors(i,:), 'LineWidth', 1.5);
            end
            xlim(cfg.viz.xlim); ylim(cfg.viz.ylim);
            title(sprintf('PHASE 1: Forming | t=%.1fs step %d/%d | max slot error %.2f m', ...
                          sim_t, t, cfg.forming.max_steps, max(errs)));
            drawnow; pause(cfg.viz.pause_time);
        end

        if max(errs) < cfg.forming.tolerance, formed = true; break; end
    end
    if ~formed
        fprintf(2, 'WARNING: formation not converged after %d steps (max error %.3f m).\n', ...
                cfg.forming.max_steps, max(errs));
    end
end

%% =====================================================================
%  PHASE 2 -- RIGID TRANSPORT
% =====================================================================
leader_pos     = mean(pos,1);
leader_heading = start_heading;
stuck_counter  = 0;

heading_history = [];
dist_history    = [];
reached_all     = true;

for wp = 1:size(cfg.waypoints,1)
    goal         = cfg.waypoints(wp,:);
    reached_this = false;

    for t = 1:cfg.timesteps
        sim_t = sim_t + cfg.dt;

        % Sense from the formation centre, with the body's circumradius folded
        % into the margin so the VFH valleys are wide enough for the whole block.
        sense_margin = half_diag + cfg.robot_radius;

        % ---- APF: pure attractive term (the leader has no formation-mates
        %      to repel from - repulsion is empty by construction here). ----
        apf_cfg_leader = struct('w_att', 1.0, 'w_rep', 0, 'rep_dist', 0);
        vec_goal = apf_force(leader_pos, goal, zeros(0,2), apf_cfg_leader, cfg.epsilon);
        goal_heading_now = atan2(vec_goal(2), vec_goal(1));

        if stuck_counter > cfg.stuck_escape_steps
            [found_escape, escape_heading] = find_escape_heading_sdf( ...
                leader_pos, robot_offsets, world, cfg.robot_radius, ...
                goal_heading_now, 36, cfg.collision_mode, cfg.escape_step);
            if found_escape, desired_heading_raw = escape_heading;
            else,            desired_heading_raw = leader_heading; end
        else
            % ---- p-VFH: obstacle steering, blended with the APF attractive
            %      term by WEIGHTED SUM (Phase 2's APF -> VFH architecture,
            %      as opposed to Phase 1's cascaded virtual-goal version). ----
            vec_vfh   = pvfh_scan(leader_pos, world, goal, sense_margin, ...
                                  cfg.vfh, cfg.epsilon, leader_heading);
            total_vec = cfg.w_goal*vec_goal + cfg.w_vfh*vec_vfh;
            desired_heading_raw = atan2(total_vec(2), total_vec(1));
        end

        desired_heading = wrap_pi(leader_heading + cfg.heading_smooth_alpha * ...
                                  wrap_pi(desired_heading_raw - leader_heading));
        dh = wrap_pi(desired_heading - leader_heading);
        dh = max(min(dh, cfg.max_turn_rate), -cfg.max_turn_rate);
        leader_heading = wrap_pi(leader_heading + dh);

        new_pos = leader_pos + cfg.V*[cos(leader_heading) sin(leader_heading)]*cfg.dt;

        [clr_new, collision] = formation_clearance(new_pos, leader_heading, ...
            robot_offsets, world, cfg.robot_radius, cfg.collision_mode);

        if ~collision
            leader_pos    = new_pos;
            stuck_counter = 0;
        else
            stuck_counter = stuck_counter + 1;
        end

        R           = rot2d(leader_heading);
        corners     = (R*robot_offsets')' + leader_pos;
        grid_points = (R*offsets_body')'  + leader_pos;
        clr_now     = formation_clearance(leader_pos, leader_heading, ...
                          robot_offsets, world, cfg.robot_radius, cfg.collision_mode);

        heading_history(end+1) = leader_heading;   %#ok<SAGROW>
        dist_history(end+1)    = norm(goal - leader_pos); %#ok<SAGROW>

        LOG = log_step(LOG, cfg, sim_t, 2, leader_pos, leader_heading, ...
                       dist_history(end), clr_now, corners);

        if cfg.viz.enable && (mod(t, cfg.viz.every)==0 || t==1)
            cla; draw_world(world, 'Inflate', cfg.robot_radius);
            plot(cfg.start_point(1), cfg.start_point(2), 'gp','MarkerSize',14, ...
                 'MarkerFaceColor','g','MarkerEdgeColor','k');
            plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'r*', ...
                 'MarkerSize', cfg.viz.waypoint_marker_size, 'LineWidth', 2);
            if ~isempty(LOG.lx), plot(LOG.lx, LOG.ly, '-', 'Color',[0 0.4 0.8], 'LineWidth',1); end
            draw_grid(grid_points, n, cfg.viz.square_line_width, 'k-');
            for i = 1:cfg.num_robots
                draw_robot(corners(i,:), cfg.robot_radius, cfg.viz.colors(i,:));
            end
            plot(leader_pos(1), leader_pos(2), 'k+', 'MarkerSize', 10, ...
                 'LineWidth', cfg.viz.leader_marker_width);
            xlim(cfg.viz.xlim); ylim(cfg.viz.ylim);
            title(sprintf(['PHASE 2: Transport | t=%.1fs step %d | hdg %.0f deg | ' ...
                           'dist %.2f m | clearance %.2f m'], sim_t, t, ...
                           rad2deg(leader_heading), dist_history(end), clr_now));
            drawnow; pause(cfg.viz.pause_time);
        end

        if dist_history(end) < cfg.goal_reach_dist
            reached_this = true; break;
        end
    end

    if ~reached_this
        reached_all = false;
        fprintf(2, 'WARNING: waypoint %d NOT reached in %d steps (dist %.2f m).\n', ...
                wp, cfg.timesteps, dist_history(end));
    end
end

%% =====================================================================
%  RESULTS
% =====================================================================
S = summarize_run(LOG, cfg, reached_all);
print_summary(S);

if cfg.log.enable
    fn = cfg.log.file;
    if isempty(fn)
        if exist('logs','dir') ~= 7, mkdir('logs'); end
        fn = fullfile('logs', [cfg.run_name '.csv']);
    end
    write_log_csv(fn, LOG, cfg);
    fprintf('Log written to %s\n', fn);
end

%% --- Diagnostic plots ---
if cfg.viz.enable
figure("Color","w","Name","Diagnostics");
subplot(3,1,1);
plot(rad2deg(heading_history), 'LineWidth', 1.5); grid on;
xlabel('Timestep (transport)'); ylabel('Heading (deg)');
title('Rigid frame heading (smooth = load stays level)');

subplot(3,1,2);
plot(dist_history, 'LineWidth', 1.5); grid on;
xlabel('Timestep (transport)'); ylabel('Distance to goal (m)');
title('Distance to goal');

subplot(3,1,3);
p2 = LOG.phase == 2;
plot(LOG.t(p2), LOG.clr(p2), 'LineWidth', 1.5); grid on; hold on;
yline_compat(cfg.robot_radius, 'r--');
xlabel('Time (s)'); ylabel('Clearance (m)');
title('Formation clearance to nearest obstacle (red = collision threshold)');

end

%% ============================================================
%  GRAPH A: distance between each robot and the target vs time
%  GRAPH B: distance between each pair of robots vs time
%
%  Drop this in AFTER the simulation loop, once LOG is fully populated
%  (same place as the existing diagnostic plots at the end of
%  Simulation7_sdf.m). Needs: LOG.t, LOG.rx, LOG.ry, cfg.num_robots,
%  cfg.waypoints, cfg.viz.colors - all already produced by that script.
% ============================================================

target = cfg.waypoints(end,:);      % final goal; change to a row of
                                     % cfg.waypoints for a specific leg
K = cfg.num_robots;

%% ---- GRAPH A: robot-to-target distance ----
dist_to_target = zeros(numel(LOG.t), K);
for i = 1:K
    dist_to_target(:,i) = hypot(LOG.rx(:,i) - target(1), LOG.ry(:,i) - target(2));
end

figure('Color','w','Name','Distance to Target');
hold on; grid on;
for i = 1:K
    plot(LOG.t, dist_to_target(:,i), 'LineWidth', 1.5, 'Color', cfg.viz.colors(i,:));
end
xlabel('Time (s)');
ylabel('Distance to target (m)');
title('Distance between each robot and the target');
legend(arrayfun(@(i) sprintf('Robot %d', i), 1:K, 'UniformOutput', false), ...
       'Location', 'best');

%% ---- GRAPH B: inter-robot distance ----
pairs = nchoosek(1:K, 2);           % every unique robot pair, e.g. [1 2;1 3;...]

figure('Color','w','Name','Inter-robot Distance');
hold on; grid on;
leg = cell(size(pairs,1), 1);
for p = 1:size(pairs,1)
    i = pairs(p,1); j = pairs(p,2);
    d = hypot(LOG.rx(:,i) - LOG.rx(:,j), LOG.ry(:,i) - LOG.ry(:,j));
    plot(LOG.t, d, 'LineWidth', 1.5);
    leg{p} = sprintf('R%d - R%d', i, j);
end
xlabel('Time (s)');
ylabel('Inter-robot distance (m)');
title('Pairwise distance between robots');
legend(leg, 'Location', 'best');
