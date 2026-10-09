%% =====================================================================
%  Simulation8_dynamic.m
%  4-ROBOT SWARM: SCATTERED START -> FORM SQUARE -> RIGID TRANSPORT
%  WITH DYNAMIC (MOVING) OBSTACLES
%
%  Built on Simulation7_sdf.m. The controller (APF slot-seeking, convoy
%  queueing, p-VFH, heading smoothing, stuck-escape) is UNCHANGED in
%  structure; this file adds moving obstacles and the three things a rigid
%  transport formation needs to survive them:
%
%    1. PREDICTIVE SENSING (pvfh_predict.m): the VFH histogram is built from
%       a scan taken from pos + v_self*T_lead, with every beam shortened by
%       the self-displacement over that horizon - the controller reacts to
%       where the geometry will be, not where it is.
%    2. SPEED SLOWDOWN: when the predicted minimum clearance to any mover is
%       below a threshold, the transport speed is scaled down (min 40%).
%       The load must not be yanked around by hard reflexes.
%    3. t+dt COLLISION CHECK: a step is committed only if the whole body is
%       collision-free at BOTH t and t+dt, evaluated with the movers at
%       their contracted future poses.
%
%  The movers are kinematic and follow dynamic_obstacles.json - the SAME
%  file the ROS 2 mover node consumes, so obstacle motion is identical in
%  both stacks by construction.
%
%  A batch driver can define CFG_OVERRIDE (struct of cfg fields) before
%  calling; those fields are merged over the defaults below.
% =====================================================================
if ~exist('CFG_OVERRIDE', 'var')
    clc; clear; close all;
end

%% =====================================================================
%  CONFIGURATION
% =====================================================================
cfg.world_file       = 'swarm_world_dynamic.sdf';
cfg.dynamic_json     = 'dynamic_obstacles.json';   % '' -> static run (no movers)
cfg.run_name         = 'matlab_dynamic';
cfg.seed             = [];      % [] -> nondeterministic; set for batch studies
cfg.formation        = 'square'; % 'square'|'line'|'triangle'|'diamond' (slot layout)

% --- ALGORITHM VARIANT (the paper's comparison axis) ---
%   'apf'       pure APF: goal attraction + APF obstacle reflex (no histogram)
%   'vfh'       deterministic VFH+ on the instantaneous noisy scan (classic)
%   'fgm'       Follow-the-Gap baseline (instantaneous noisy scan)
%   'dwa'       Dynamic-Window arc-sampling baseline (instantaneous scan)
%   'mpc'       receding-horizon lattice-MPC baseline (instantaneous scan)
%   'pvfh'      deterministic VFH+ with the mover prediction horizon
%   'pvfh_prob' P-VFH: probabilistic occupancy + APF reflex (the proposal)
% Every variant consumes the SAME noisy scan from the SAME sensor model, so
% the comparison isolates the steering law, not the sensing.
cfg.algo             = 'pvfh';
cfg.w_att            = 1.0;     % APF goal-attraction gain (transport baseline)

cfg.grid_size        = 2;
cfg.num_robots       = cfg.grid_size^2;
cfg.dt               = 0.1;
cfg.viz.colors       = hsv(cfg.num_robots);

% ---- Physical parameters. MATCH THESE TO YOUR ROS 2 ROBOT ----
cfg.robot_radius   = 0.18;
cfg.robot_height   = 0.35;
cfg.grid_spacing   = 1.18;    % medium-world safe value; feasibility check may auto-fix
cfg.collision_mode = 'payload';   % the formation carries a load

% ---- Start / goal ----
cfg.interactive        = false;
cfg.spawn_in_formation = false;
cfg.auto_fix_spacing   = true;
cfg.start_point        = [-12 0];
cfg.waypoints          = [];      % [] -> SDF goal_pad
cfg.scatter_center     = [-12 0];
cfg.scatter_radius     = 2.2;

% --- Phase 2 (rigid transport) ---
cfg.timesteps       = 6000;
cfg.goal_reach_dist = 0.5;
cfg.max_turn_rate   = 0.3;
cfg.V               = 0.6;
cfg.w_goal          = 1.0;
cfg.w_vfh           = 2.0;
cfg.heading_smooth_alpha = 0.35;

% --- DYNAMIC OBSTACLES: reaction knobs (mirror these in ROS 2) ---
cfg.dyn.pred_lead     = 0.5;   % s, prediction horizon for the VFH scan
cfg.dyn.slow_margin   = 0.25;  % m, predicted clearance below which we slow
cfg.dyn.slow_min      = 0.4;   % -,  minimum speed scale during slowdown
cfg.dyn.hold_time     = 1.5;   % s, escape-pose hold horizon vs movers
cfg.dyn.lookahead     = 1.0;   % s, worst-case closing distance lookahead
cfg.dyn.avoid_vel_frac = 0.25; % -, mover speeds at/below this fraction of V
                               %     are treated as parked (no slowdown debt)
cfg.dyn.speed_scale   = 1.0;   % -, multiplies every mover's contract speed
                               %     (E8 traffic-speed sweep; 1 = the contract)
cfg.dyn.max_movers    = inf;   % use only the first N movers of the contract
                               %     (E11 traffic-density sweep; 0 = none)

% --- ROBUSTNESS PERTURBATIONS (all off by default; transport phase only) ---
% Localisation error (E10): each step the controller believes it is at
% pos + N(0, sigma_pos^2 I) with heading + N(0, sigma_yaw^2). The scan is
% physical (taken at the TRUE pose, in the body frame); what the error
% corrupts is the believed goal direction. Goal arrival and the safety gate
% use the true pose.
cfg.loc = struct('sigma_pos', 0, 'sigma_yaw', 0);
% Command-update loss (E12): with this probability a step's steering update
% is lost and the previous heading/speed command is held (a dropped packet
% between the planner and the formation). 0 = never.
cfg.comm_drop = 0;

% --- LOAD-CARRYING MOTION LIMITS (mission level, identical for EVERY law) ---
% The frame is a rigid payload on K robots. A slot at distance r_c from the
% centre moves at up to v + |w|*r_c, so an unconstrained yaw rate asks the
% outer robot for speeds it does not have (the pre-limit reference run
% needed 3.0 m/s from a corner robot while the centre drove 0.6 m/s, and
% 13.7 rad/s^2 of yaw acceleration). Each step the commanded turn is
% therefore shaped, in this order:
%   1. braking curve     |w| <= brake_gain*sqrt(2*yaw_acc_max*|heading error|)
%                        (never turn faster than can be stopped on target)
%   2. yaw rate          |w| <= (v_robot_max - v_floor) / r_c
%   3. yaw acceleration  |dw/dt| <= yaw_acc_max          (no jerked load)
%   4. speed             v <= v_robot_max - |w|*r_c      (outer robot limit)
%                        v <= lat_acc_max / |w|          (load does not slide)
% with v_floor = v_floor_frac * V: the formation slows to turn but never
% stops to pivot. (A first version let the yaw rate rise until the speed hit
% zero; the formation then pivoted in place in a mover's lane and was run
% into - reference run, t = 19 s. Standing still in traffic is the worst
% place to be: movers do not yield.) A pivot in place still happens when the
% safety gate blocks the translation (rotate-only fallback of the commit).
% enable = false is the unconstrained turn (0.3 rad per step).
%
% DEFAULT OFF. Measured on the warehouse (check_motion_limits, 2026-10-02):
% with the limits on, the formation is ~7 s slower through the first
% chicane, is still crossing the main aisle when the head-on forklift
% arrives, and is run into (17-18 mover-contact steps on 3 of 3 seeds; 0
% with the limits off). The limits are right physically, but the steering
% stack (0.5 s prediction horizon, slow-down-near-movers rule) was tuned for
% the agile body, so switching them on is a re-tuning job, not a flag flip.
% They are therefore evaluated as their own experiment (E14), and every run
% reports the peak robot speed its motion would have required
% (max_robot_speed), so the infeasibility is measured rather than hidden.
cfg.kin = struct('enable', false, 'v_robot_max', 1.0, 'yaw_acc_max', 2.5, ...
                 'lat_acc_max', 0.8, 'v_floor_frac', 0.5, 'brake_gain', 0.8);

% --- ANTI-DEADLOCK LAYER (mission level, identical for EVERY algorithm) ---
% (i) progress window: `window` steps without `min_progress` metres of net
%     motion along the start->goal axis force an escape probe even when
%     every step was accepted (a stall made of legal micro-steps).
% (ii) commit-after-probe: a found escape heading is held for
%     `commit_steps` steps (rate-limited turn, every pose still gated).
% enable = false reproduces the legacy one-shot probe (E9 ablation).
% (iii) escalation: every escape that does not bring the formation at least
%     `trap_gain` metres closer to the goal than it has ever been counts as
%     one more visit to the same trap. The k-th such escape aims
%     k * `trap_step` rad away from the goal bearing (side kept, flipped
%     every `trap_flip` visits) and commits (1 + k) times as long, so the
%     formation walks out of a dead end behind which the goal lies instead
%     of re-entering it (the random-layout failure: a 1.3 m gate straight
%     toward the goal, 1,270 escape steps without getting round it).
% window = 150 steps (15 s). It was 600 (60 s): on the plaza the histogram
% laws then hovered in front of an obstacle cluster for a minute before the
% detector fired (pilot, 10 seeds: P-VFH 109 s -> 79 s; random layouts
% 77 s -> 60 s; nothing got worse). 100 and 60 were tried: faster on the
% plaza but with mover contacts on the line formation / more rejections.
cfg.adl = struct('enable', true, 'window', 150, 'min_progress', 0.30, ...
                 'commit_steps', 40, 'diversify', 0.5, 'escalate', false, ...
                 'trap_gain', 0.5, 'trap_step', pi/6, 'trap_flip', 4, ...
                 'trap_max', 5);

% --- GOAL/STEERING BLEND (transport) ---
% heading = w_goal * goal direction + w_vfh * steering direction. When the
% steering law has chosen a detour, the fixed goal term drags the heading
% back toward the obstacle (a 90-deg detour becomes 63 deg). With
% blend_fade the goal weight is scaled by max(0, cos(angle between the
% two)): full weight when they agree, none when the law steers away. Not
% applied to 'apf', whose steering vector is a repulsion, not a direction.
cfg.blend_fade = true;

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
cfg.forming.dock_tau             = 1.0;   % s; see "docking" at the phase switch

% --- Phase 1 convoy/queueing behavior ---
cfg.forming.convoy_trigger_dist = 2.0;
cfg.forming.convoy_exit_dist    = 3.0;
cfg.forming.convoy_max_obstacle = 2.0;
cfg.forming.convoy_min_group    = 2;
cfg.forming.convoy_spacing      = 1.0;
cfg.forming.trail_max_len       = 4000;

% --- SENSING MODEL (shared by every algorithm variant) ---
% kind 'lidar': dense 360-deg scan, homoscedastic noise sigma0.
% kind 'tof'  : sparse array (n_beams over fov_deg), range-dependent noise
%               sigma(d) = sigma0 + k_rng*d, max-range drop-cut. The beam
%               arc is centred on world +x (the transport direction here);
%               unobserved bearings are UNKNOWN, never free.
cfg.vfh.sense = struct('kind', 'lidar', 'sigma0', 0.02, 'k_rng', 0.03, ...
                       'fov_deg', 60, 'max_range', inf, 'frame', 'body');
% frame 'body': the ToF fan is centred on the body heading (a mounted
% sensor). 'world' reproduces the legacy fan fixed on world +x.
cfg.vfh.p_unknown = 0.05;   % P-VFH: occupancy floor for unobserved sectors

% --- p-VFH parameters (unchanged from Simulation7_sdf) ---
cfg.vfh.num_sectors      = 72;
cfg.vfh.n_beams          = 288;
cfg.vfh.sigma            = 1.0;
cfg.vfh.sense_range_base = 4.0;
cfg.vfh.danger_dist_base = 1.2;
cfg.vfh.turn_limit       = pi/4;
cfg.vfh.threshold        = 0.3;
cfg.vfh.smooth_window    = 5;
cfg.vfh.avoid_gain       = 5;
cfg.vfh.hysteresis_weight = 0.6;
cfg.vfh.avoid_falloff_power = 3;
cfg.vfh.circular_smooth  = true;
cfg.vfh.enlarge          = true;
cfg.vfh.auto_sigma       = true;
cfg.vfh.react_gap        = 1.0;
cfg.vfh.pred_lead        = cfg.dyn.pred_lead;   % consumed by pvfh_predict

cfg.stuck_escape_steps   = 5;
cfg.escape_step          = 0.4;
cfg.epsilon              = 1e-6;

% --- Visualization / logging ---
cfg.viz.enable  = true;
cfg.viz.every   = 5;
cfg.viz.pause_time = 0.001;
cfg.viz.robot_marker_size = 8;
cfg.viz.square_line_width = 1.5;
cfg.viz.waypoint_marker_size = 10;
cfg.viz.leader_marker_width = 2;
cfg.viz.show_predicted_movers = true;  % ghost movers at t + pred_lead
cfg.log.enable  = true;
cfg.log.file    = '';

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
    cfg.num_robots = cfg.grid_size^2;
    cfg.viz.colors = hsv(cfg.num_robots);
end

%% =====================================================================
%  LOAD THE WORLD + THE SHARED MOTION CONTRACT
% =====================================================================
% The mover models exist in the SDF only so Gazebo has something to teleport.
% Their GEOMETRY comes from the JSON contract, so they must NOT also enter
% the static obstacle map - hence SkipModels. Without this, each mover would
% collide with its own spawn pose.
[movers0, ~] = load_dynamic_obstacles(cfg.dynamic_json);
skip = {'ground_plane'};
for k = 1:numel(movers0), skip{end+1} = movers0(k).name; end %#ok<AGROW>
world  = sdf2world(cfg.world_file, 'RobotHeight', cfg.robot_height, ...
                   'SkipModels', skip);
if isempty(cfg.waypoints)
    cfg.waypoints = contract_waypoints(cfg.dynamic_json);   % optional global route
end
if isempty(cfg.waypoints)
    if isempty(world.goal)
        error('No goal_pad in the SDF and cfg.waypoints is empty.');
    end
    cfg.waypoints = world.goal;
end
cfg.viz.xlim = [world.bounds(1)-0.5, world.bounds(2)+0.5];
cfg.viz.ylim = [world.bounds(3)-0.5, world.bounds(4)+0.5];

[movers, dyn_meta] = load_dynamic_obstacles(cfg.dynamic_json, world);
if ~strcmp(dyn_meta.frame, 'world')
    error('Only the ''world'' frame is supported; got ''%s''.', dyn_meta.frame);
end
if abs(dyn_meta.dt - cfg.dt) > 1e-9
    fprintf(2, ['WARNING: contract dt (%.3g) != sim dt (%.3g). The sim uses its ' ...
                'own dt; prediction and the t+dt check stay consistent because ' ...
                'both use cfg.dt.\n'], dyn_meta.dt, cfg.dt);
end
if isfinite(cfg.dyn.max_movers)
    movers = movers(1:min(numel(movers), max(0, round(cfg.dyn.max_movers))));
end
for k = 1:numel(movers)
    movers(k).speed = movers(k).speed * cfg.dyn.speed_scale;
end
nM = numel(movers);

%% --- deterministic seed for reproducible batch studies ---
if isfield(cfg, 'seed') && ~isempty(cfg.seed)
    % Seed the global stream. MATLAB: rng(seed) (Mersenne Twister). Octave:
    % randn/rand('seed', s) (legacy PRNG, fine for Monte Carlo).
    if exist('OCTAVE_VERSION') > 0
        randn('seed', cfg.seed); rand('seed', cfg.seed);
    else
        try, rng(cfg.seed); catch, randn('seed', cfg.seed); rand('seed', cfg.seed); end
    end
end

%% =====================================================================
%  FEASIBILITY CHECK (statics only - movers are avoided, not planned through)
% =====================================================================
fcfg = struct('res', 0.05, 'robot_radius', cfg.robot_radius, ...
              'grid_size', cfg.grid_size, 'grid_spacing', cfg.grid_spacing);
rep = world_feasibility(world, cfg.start_point, cfg.waypoints(end,:), fcfg);

if ~rep.single_ok
    error(['This world is impassable even for a single robot of radius %.2f m. ' ...
           'Fix the world or the robot radius.'], cfg.robot_radius);
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
if isfield(cfg, 'formation') && ~strcmpi(cfg.formation, 'square')
    % non-square shapes: slots from FORMATION_GEOMETRY (grid_size ignored)
    [offsets_body, half_diag, n_robots] = formation_geometry(cfg.formation, ...
                                                             cfg.grid_spacing);
    cfg.num_robots = n_robots;
else
    [offsets_body, half_diag] = formation_geometry('square', cfg.grid_spacing);
end

scatter_theta = linspace(0, 2*pi, cfg.num_robots+1); scatter_theta(end) = [];
cfg.robots_start = cfg.scatter_center + ...
    cfg.scatter_radius*[cos(scatter_theta') sin(scatter_theta')];

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
%  SPAWN VALIDATION (against statics AND mover t=0 poses)
% =====================================================================
world0 = merge_dyn(update_world_dyn(world, movers, 0));

for i = 1:cfg.num_robots
    d = world_dist(cfg.robots_start(i,:), world0);
    if d <= cfg.robot_radius
        error(['SPAWN ERROR: robot %d at [%.2f %.2f] has clearance %.2f m.'], ...
              i, cfg.robots_start(i,:), d);
    end
end
if world_dist(cfg.start_point, world0) <= cfg.robot_radius
    error('SPAWN ERROR: formation start point is inside/too near an obstacle.');
end
for i = 1:size(cfg.waypoints,1)
    if world_dist(cfg.waypoints(i,:), world0) <= cfg.robot_radius
        error('WAYPOINT ERROR: goal waypoint %d is inside/too near an obstacle.', i);
    end
end
for i = 1:cfg.num_robots
    d = world_dist(assigned_slots(i,:), world0);
    if d <= cfg.robot_radius
        error(['SLOT ERROR: assembly slot %d at [%.2f %.2f] has clearance %.2f m. ' ...
               'Move cfg.start_point or reduce cfg.grid_spacing.'], ...
               i, assigned_slots(i,:), d);
    end
end

%% --- Visualization setup ---
if cfg.viz.enable
    fig_main = figure('Color','w','Name','Swarm with dynamic obstacles'); hold on; grid on;
    draw_world(world0, 'Inflate', cfg.robot_radius);
    xlim(cfg.viz.xlim); ylim(cfg.viz.ylim);
end

%% --- Logging buffers ---
LOG = struct('t', [], 'phase', [], 'lx', [], 'ly', [], 'lh', [], ...
             'dist', [], 'clr', [], 'rx', [], 'ry', []);
LOG.cvx = []; LOG.cvy = []; LOG.vscale = []; LOG.near = []; LOG.near_pre = [];
LOG.comp = [];
LOG.rej = []; LOG.esc = []; LOG.clr_s = [];   % audit columns (see summarize_run)

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

    % Convoy/queueing applies to STATIC obstacles only: movers move, and the
    % trail-following logic assumes a fixed thing to queue behind.
    obs_ext   = obstacle_extents(world);
    convoy_ok = obs_ext <= cfg.forming.convoy_max_obstacle;
    nStatic   = numel(world.boxes) + numel(world.circles);

    stuck_counter_form = zeros(cfg.num_robots,1);
    in_convoy          = zeros(cfg.num_robots,1);
    convoy_trail       = cell(nStatic,1);
    for c = 1:nStatic, convoy_trail{c} = zeros(0,2); end
    if ~any(convoy_ok)
        fprintf('  (no compact static obstacles: convoy/queueing disabled)\n');
    end

    formed = false;
    for t = 1:cfg.forming.max_steps
        new_pos = pos;
        sim_t   = sim_t + cfg.dt;
        comp_ms = 0;    % steering compute time, summed over the robots

        % Advance the movers along the shared contract.
        [world_t, movers_t] = update_world_dyn(world, movers, sim_t);
        wm = merge_dyn(world_t);

        % Nearest obstacle per robot over statics + movers.
        [nearest_d, nearest_id] = world_dist(pos, wm);

        new_in_convoy = zeros(cfg.num_robots,1);
        for c = 1:nStatic
            if c > numel(convoy_ok) || ~convoy_ok(c), continue; end
            eligible = find((nearest_id==c & nearest_d<cfg.forming.convoy_trigger_dist) | ...
                            (in_convoy==c  & nearest_d<cfg.forming.convoy_exit_dist));
            if numel(eligible) >= cfg.forming.convoy_min_group
                new_in_convoy(eligible) = c;
            end
        end
        in_convoy = new_in_convoy;

        leader_of = zeros(cfg.num_robots,1);
        for c = 1:nStatic
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
                [found_escape, escape_heading] = find_escape_heading_dyn( ...
                    pos(i,:), world, movers_t, sim_t, [0 0], ...
                    cfg.robot_radius, goal_heading_i, 36, 'slots', ...
                    cfg.forming.backoff_dist, cfg.dyn.hold_time, cfg.dt);
                if found_escape, desired_h_raw = escape_heading;
                else,            desired_h_raw = heading(i); end
                e_i = norm(assigned_slots(i,:) - pos(i,:));
                speed_i = cfg.forming.V * min(1, e_i/cfg.forming.slow_radius);

            else
                % ---- APF: pull to slot, repel from mates (cascade input) ----
                is_mate = (1:cfg.num_robots)' ~= i & ...
                          ~(in_convoy(i) ~= 0 & in_convoy == in_convoy(i));
                neighbors_i = pos(is_mate, :);

                apf_cfg_i = struct('w_att', cfg.forming.w_slot, ...
                                   'w_rep', cfg.forming.w_repulse, ...
                                   'rep_dist', cfg.forming.repulse_dist);
                apf_vec = apf_force(pos(i,:), assigned_slots(i,:), neighbors_i, ...
                                    apf_cfg_i, cfg.epsilon);

                % ---- steer toward the APF virtual goal (algorithm dispatch) ----
                virtual_goal = pos(i,:) + apf_vec;
                [vec_steer, comp_ms_i] = steering_dispatch('point', pos(i,:), virtual_goal, ...
                    cfg.forming.w_slot, world, movers_t, sim_t, ...
                    cfg.robot_radius, cfg, heading(i));
                comp_ms = comp_ms + comp_ms_i;

                desired_h_raw = atan2(vec_steer(2), vec_steer(1));
                e_i = norm(assigned_slots(i,:) - pos(i,:));
                speed_i = cfg.forming.V * min(1, e_i/cfg.forming.slow_radius);
            end

            desired_h = wrap_pi(heading(i) + cfg.forming.heading_smooth_alpha * ...
                                wrap_pi(desired_h_raw - heading(i)));
            dh = wrap_pi(desired_h - heading(i));
            dh = max(min(dh, cfg.forming.max_turn_rate), -cfg.forming.max_turn_rate);
            heading(i) = wrap_pi(heading(i) + dh);

            cand = pos(i,:) + speed_i*[cos(heading(i)) sin(heading(i))]*cfg.dt;
            % Commit only if safe now AND at t+dt with the movers advancing.
            % Offsets are [0 0]: this is a SINGLE-robot point test (the robot
            % is its own centre), exactly like the original world_dist check.
            if step_safe_dyn(cand, heading(i), world, movers_t, sim_t, cfg.dt, ...
                             cfg.robot_radius, 'slots', [0 0])
                new_pos(i,:) = cand;
                stuck_counter_form(i) = 0;
            else
                stuck_counter_form(i) = stuck_counter_form(i) + 1;
                backoff = pos(i,:) - cfg.forming.backoff_dist*[cos(heading(i)) sin(heading(i))];
                if world_dist(backoff, wm) > cfg.robot_radius
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
                       max(errs), min(world_dist_dyn(pos, world, movers_t)), pos);
        % Phase-1 dynamic metrics. cvx/cvy: velocity of the mover nearest to
        % ANY robot (the centroid is a pseudo-entity while scattered).
        % vscale is 1 by definition here: forming speed is governed by each
        % robot's own slow_radius logic, not by the transport slowdown.
        % near: current gap, min over robots. near_pre: PREDICTED worst gap,
        % min over robots, using each robot's realised step velocity - the
        % same prediction the transport phase uses, so the metric is defined
        % across the whole run (no sentinel cliffs in the logs).
        [gap1, gapn1] = phase1_dyn_metrics(pos, new_pos, movers_t, sim_t, ...
                                           cfg.dt, cfg.dyn.lookahead);
        [cvx1, cvy1] = phase1_nearest_mover_vel(new_pos, movers_t);
        LOG.cvx(end+1,1) = cvx1; LOG.cvy(end+1,1) = cvy1;
        LOG.vscale(end+1,1) = 1;
        LOG.near(end+1,1) = gapn1;
        LOG.near_pre(end+1,1) = gap1;
        LOG.comp(end+1,1) = comp_ms;     % summed over the robots this step
        LOG.rej(end+1,1) = 0; LOG.esc(end+1,1) = 0;
        LOG.clr_s(end+1,1) = LOG.clr(end);

        if cfg.viz.enable && (mod(t, cfg.viz.every)==0 || t==1)
            cla; draw_world(wm, 'Inflate', cfg.robot_radius);
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
%  PHASE 2 -- RIGID TRANSPORT WITH DYNAMIC OBSTACLES
% =====================================================================
leader_pos     = mean(pos,1);
leader_heading = start_heading;
stuck_counter  = 0;
yaw_rate_prev  = 0;         % committed yaw rate of the previous step (rad/s)

% DOCKING. Forming ends when every robot is within cfg.forming.tolerance of
% its slot, so at the switch the robots are up to that far from the rigid
% frame's slots. The old code simply logged the slot positions from the next
% step on: the robots jumped (0.24-0.35 m in the reference run). Now the
% residual slot error, in the body frame, decays exponentially with time
% constant dock_tau while the frame starts to move - the robots close the
% last few centimetres instead of teleporting. The collision gate keeps
% using the rigid hull (the residual is smaller than the forming tolerance
% and the start area is open).
dock_resid = (rot2d(leader_heading)' * (pos - ((rot2d(leader_heading)*robot_offsets')' + leader_pos))')';
dock_steps = 0;

heading_history = [];
dist_history    = [];
reached_all     = true;
speed_scale     = 1.0;

% anti-deadlock state
escape_commit   = 0;        % committed-escape steps left
escape_heading  = 0;
last_escape     = [];       % previous escape heading (diversification)
last_probe_step = -inf;
last_cmd        = [];       % [desired_heading_raw, v_scale] of the last update
trap_count      = 0;        % escapes since the goal distance last improved
trap_side       = 0;        % +1 / -1 side the escalation leans to (0 = unset)

for wp = 1:size(cfg.waypoints,1)
    goal         = cfg.waypoints(wp,:);
    reached_this = false;
    prog_origin  = leader_pos;
    prog_axis    = (goal - leader_pos) / max(norm(goal - leader_pos), cfg.epsilon);
    prog_hist    = [];
    last_probe_step = -inf;
    best_dist    = norm(goal - leader_pos);   % closest approach so far
    trap_count   = 0;  trap_side = 0;

    for t = 1:cfg.timesteps
        sim_t = sim_t + cfg.dt;

        % ---- advance the movers along the shared contract ----
        [world_t, movers_t] = update_world_dyn(world, movers, sim_t);
        wm = merge_dyn(world_t);

        sense_margin = half_diag + cfg.robot_radius;

        % ---- APF: pure attractive term (leader has no formation-mates) ----
        apf_cfg_leader = struct('w_att', 1.0, 'w_rep', 0, 'rep_dist', 0);
        vec_goal = apf_force(leader_pos, goal, zeros(0,2), apf_cfg_leader, cfg.epsilon);
        goal_heading_now = atan2(vec_goal(2), vec_goal(1));

        % ---- anti-deadlock (ii): progress-window stall detector ----
        prog_hist(end+1) = dot(leader_pos - prog_origin, prog_axis); %#ok<SAGROW>
        if numel(prog_hist) > cfg.adl.window, prog_hist(1) = []; end
        want_probe = stuck_counter > cfg.stuck_escape_steps;
        avoid_h    = [];
        if cfg.adl.enable && escape_commit == 0 && ...
                numel(prog_hist) >= cfg.adl.window && ...
                t - last_probe_step > cfg.adl.commit_steps && ...
                prog_hist(end) - prog_hist(1) < cfg.adl.min_progress
            want_probe = true;
            avoid_h    = last_escape;       % do not re-commit the same way
        end

        if escape_commit == 0 && want_probe
            % ---- escalation bookkeeping: same trap, or real progress? ----
            d_now = norm(goal - leader_pos);
            if d_now < best_dist - cfg.adl.trap_gain
                best_dist = d_now;  trap_count = 0;  trap_side = 0;
            end
            pref_h = goal_heading_now;
            k_trap = 0;
            if cfg.adl.enable && cfg.adl.escalate && trap_count > 0
                if trap_side == 0, trap_side = 1; end
                k_trap = min(trap_count, cfg.adl.trap_max);
                side   = trap_side * (-1)^floor((trap_count - 1) / cfg.adl.trap_flip);
                pref_h = wrap_pi(goal_heading_now + side * k_trap * cfg.adl.trap_step);
            end
            % ---- escape probing against MOVERS (hold-horizon test) ----
            [found_escape, eh] = find_escape_heading_dyn( ...
                leader_pos, world, movers_t, sim_t, robot_offsets, ...
                cfg.robot_radius, pref_h, 36, cfg.collision_mode, ...
                cfg.escape_step, cfg.dyn.hold_time, cfg.dt, ...
                avoid_h, cfg.adl.diversify);
            last_probe_step = t;
            if found_escape
                escape_heading = eh;
                last_escape    = eh;
                if trap_side == 0
                    trap_side = sign(wrap_pi(eh - goal_heading_now));
                    if trap_side == 0, trap_side = 1; end
                end
                trap_count = trap_count + 1;
                if cfg.adl.enable
                    escape_commit = cfg.adl.commit_steps * (1 + k_trap * double(cfg.adl.escalate));
                else
                    escape_commit = 1;      % legacy one-shot probe
                end
            else
                % nothing translatable right now (e.g. a mover is passing):
                % wait a few rejected steps before paying for another probe
                stuck_counter = 0;
            end
        end

        if escape_commit > 0
            % ---- committed escape: steer at the validated heading ----
            desired_heading_raw = escape_heading;
            v_scale  = 1.0;
            comp_ms  = 0;    % no steering computed this step
            near     = 99;   % clamped: Inf would leak into the CSV
            near_pre = 99;
            esc_flag = 1;
            turn_gain = 1.0; % decisive: rate limit only, no low-pass
        elseif want_probe
            % probe found nothing: hold heading, try again next step
            desired_heading_raw = leader_heading;
            v_scale = 1.0; comp_ms = 0; near = 99; near_pre = 99;
            esc_flag = 0; turn_gain = cfg.heading_smooth_alpha;
        else
            % ---- steering from the formation centre (algorithm dispatch) ----
            v_self  = cfg.V * [cos(leader_heading) sin(leader_heading)];
            % ---- believed goal under localisation error (E10) ----
            goal_b = goal;  vec_goal_b = vec_goal;
            if cfg.loc.sigma_pos > 0 || cfg.loc.sigma_yaw > 0
                e_pos  = cfg.loc.sigma_pos * randn(1, 2);
                e_yaw  = cfg.loc.sigma_yaw * randn;
                rel    = (rot2d(-e_yaw) * (goal - (leader_pos + e_pos))')';
                goal_b = leader_pos + rel;
                vec_goal_b = rel / max(norm(rel), cfg.epsilon);
            end
            [vec_steer, comp_ms] = steering_dispatch('rigid', leader_pos, goal_b, cfg.w_att, ...
                world, movers_t, sim_t, sense_margin, cfg, leader_heading, v_self);
            w_goal_eff = cfg.w_goal;
            if cfg.blend_fade && ~strcmpi(cfg.algo, 'apf') && norm(vec_steer) > cfg.epsilon
                w_goal_eff = cfg.w_goal * max(0, dot(vec_goal_b, vec_steer) / norm(vec_steer));
            end
            total_vec = w_goal_eff*vec_goal_b + cfg.w_vfh*vec_steer;
            desired_heading_raw = atan2(total_vec(2), total_vec(1));

            % ---- SPEED SLOWDOWN near predicted movers ----
            [gap_pre, gap_now] = predicted_min_mover_gap(leader_pos, v_self, ...
                                         movers_t, sim_t, cfg.dt, cfg.dyn.lookahead);
            thr = cfg.dyn.slow_margin + cfg.robot_radius;
            if ~isempty(gap_pre) && gap_pre < thr && ...
                    mover_speed_frac(movers_t, cfg.V) > cfg.dyn.avoid_vel_frac
                v_scale = max(cfg.dyn.slow_min, min(1.0, 0.8*gap_pre/thr));
            else
                v_scale = 1.0;
            end
            near     = gap_now;
            near_pre = gap_pre;
            esc_flag = 0;
            turn_gain = cfg.heading_smooth_alpha;
            % ---- command-update loss (E12): hold the previous command ----
            if cfg.comm_drop > 0 && ~isempty(last_cmd) && rand < cfg.comm_drop
                desired_heading_raw = last_cmd(1);
                v_scale             = last_cmd(2);
            else
                last_cmd = [desired_heading_raw, v_scale];
            end
        end
        speed_scale = v_scale;

        desired_heading = wrap_pi(leader_heading + turn_gain * ...
                                  wrap_pi(desired_heading_raw - leader_heading));
        dh = wrap_pi(desired_heading - leader_heading);
        dh = max(min(dh, cfg.max_turn_rate), -cfg.max_turn_rate);
        % ---- load-carrying motion limits (see cfg.kin) ----
        if cfg.kin.enable
            w_cmd  = dh / cfg.dt;
            e_head = abs(wrap_pi(desired_heading_raw - leader_heading));
            w_brk  = cfg.kin.brake_gain * sqrt(2 * cfg.kin.yaw_acc_max * e_head);
            v_want = v_scale * cfg.V;
            v_flr  = min(v_want, cfg.kin.v_floor_frac * cfg.V);
            w_cap  = min(w_brk, (cfg.kin.v_robot_max - v_flr) / max(half_diag, cfg.epsilon));
            w_cmd  = max(min(w_cmd, w_cap), -w_cap);
            dw_max = cfg.kin.yaw_acc_max * cfg.dt;
            w_cmd  = max(min(w_cmd, yaw_rate_prev + dw_max), yaw_rate_prev - dw_max);
            v_lim  = max(v_flr, cfg.kin.v_robot_max - abs(w_cmd) * half_diag);
            if abs(w_cmd) > cfg.epsilon
                v_lim = min(v_lim, cfg.kin.lat_acc_max / abs(w_cmd));
            end
            v_scale = min(v_scale, v_lim / cfg.V);
            speed_scale = v_scale;
            dh = w_cmd * cfg.dt;
        end
        cand_heading = wrap_pi(leader_heading + dh);
        heading_before = leader_heading;

        new_pos = leader_pos + v_scale*cfg.V*[cos(cand_heading) sin(cand_heading)]*cfg.dt;

        % ---- ATOMIC POSE COMMIT: the WHOLE pose (position AND heading)
        % must be safe at t and t+dt. The previous code committed the new
        % heading before -- and regardless of -- the collision check, so a
        % blocked body kept rotating in place INTO the obstacle: every
        % "collision step" of the old result tables was this artefact.
        %   0  translation + rotation accepted
        %   1  translation blocked; rotation in place is safe -> pivot
        %   2  both blocked -> hold the pose
        %   3  wedged (blocked for > stuck_escape_steps): RECOVERY move --
        %      back up half a step along the body axis (with the turn if
        %      that is safe, else without), or turn the other way round.
        %      Without it a body that can neither advance nor rotate holds
        %      forever while the escape probe keeps naming a heading it
        %      cannot rotate to (pvfh, seed 1: 2,857 held steps, timeout).
        if step_safe_dyn(new_pos, cand_heading, world, movers_t, sim_t, ...
                         cfg.dt, cfg.robot_radius, cfg.collision_mode, robot_offsets)
            leader_pos     = new_pos;
            leader_heading = cand_heading;
            stuck_counter  = 0;
            rej_flag       = 0;
        elseif abs(dh) > 0 && step_safe_dyn(leader_pos, cand_heading, world, ...
                         movers_t, sim_t, cfg.dt, cfg.robot_radius, ...
                         cfg.collision_mode, robot_offsets)
            leader_heading = cand_heading;
            stuck_counter  = stuck_counter + 1;
            rej_flag       = 1;
        else
            stuck_counter  = stuck_counter + 1;
            rej_flag       = 2;
            if stuck_counter > cfg.stuck_escape_steps
                back_pos = leader_pos - 0.5*cfg.V*cfg.dt * ...
                           [cos(leader_heading) sin(leader_heading)];
                other_h  = wrap_pi(leader_heading - dh);
                if step_safe_dyn(back_pos, cand_heading, world, movers_t, sim_t, ...
                        cfg.dt, cfg.robot_radius, cfg.collision_mode, robot_offsets)
                    leader_pos = back_pos;  leader_heading = cand_heading;  rej_flag = 3;
                elseif step_safe_dyn(back_pos, leader_heading, world, movers_t, sim_t, ...
                        cfg.dt, cfg.robot_radius, cfg.collision_mode, robot_offsets)
                    leader_pos = back_pos;  rej_flag = 3;
                elseif abs(dh) > 0 && step_safe_dyn(leader_pos, other_h, world, ...
                        movers_t, sim_t, cfg.dt, cfg.robot_radius, ...
                        cfg.collision_mode, robot_offsets)
                    leader_heading = other_h;  rej_flag = 3;
                end
            end
        end
        if escape_commit > 0
            escape_commit = escape_commit - 1;
            if rej_flag == 2, escape_commit = 0; end   % blocked mid-escape
            if rej_flag == 0, stuck_counter = 0; end
        end

        yaw_rate_prev = wrap_pi(leader_heading - heading_before) / cfg.dt;

        R           = rot2d(leader_heading);
        corners     = (R*robot_offsets')' + leader_pos;
        if cfg.forming.dock_tau > 0 && dock_steps * cfg.dt < 6 * cfg.forming.dock_tau
            dock_steps = dock_steps + 1;
            corners = corners + (R * dock_resid')' * ...
                      exp(-dock_steps * cfg.dt / cfg.forming.dock_tau);
        end
        grid_points = (R*offsets_body')'  + leader_pos;
        clr_now     = formation_clearance_dyn(leader_pos, leader_heading, ...
                          robot_offsets, world, cfg.robot_radius, ...
                          cfg.collision_mode, movers_t);
        clr_static  = formation_clearance_dyn(leader_pos, leader_heading, ...
                          robot_offsets, world, cfg.robot_radius, ...
                          cfg.collision_mode, struct([]));

        heading_history(end+1) = leader_heading;   %#ok<SAGROW>
        dist_history(end+1)    = norm(goal - leader_pos); %#ok<SAGROW>

        LOG = log_step(LOG, cfg, sim_t, 2, leader_pos, leader_heading, ...
                       dist_history(end), clr_now, corners);
        [cvx, cvy] = nearest_mover_vel(leader_pos, movers_t);
        LOG.cvx(end+1,1)    = cvx;
        LOG.cvy(end+1,1)    = cvy;
        LOG.vscale(end+1,1) = speed_scale;
        LOG.near(end+1,1)   = min(near, 99);
        LOG.near_pre(end+1,1) = min(near_pre, 99);
        LOG.comp(end+1,1)   = comp_ms;   % steering compute time this step (ms)
        LOG.rej(end+1,1)    = rej_flag;
        LOG.esc(end+1,1)    = esc_flag;
        LOG.clr_s(end+1,1)  = clr_static;

        if cfg.viz.enable && (mod(t, cfg.viz.every)==0 || t==1)
            cla; draw_world(wm, 'Inflate', cfg.robot_radius);
            plot(cfg.start_point(1), cfg.start_point(2), 'gp','MarkerSize',14, ...
                 'MarkerFaceColor','g','MarkerEdgeColor','k');
            plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'r*', ...
                 'MarkerSize', cfg.viz.waypoint_marker_size, 'LineWidth', 2);
            if ~isempty(LOG.lx), pline(LOG.lx, LOG.ly, 'Color',[0 0.4 0.8], 'LineWidth',1); end
            draw_grid(grid_points, n, cfg.viz.square_line_width, 'k-');
            for i = 1:cfg.num_robots
                draw_robot(corners(i,:), cfg.robot_radius, cfg.viz.colors(i,:));
            end
            plot(leader_pos(1), leader_pos(2), 'k+', 'MarkerSize', 10, ...
                 'LineWidth', cfg.viz.leader_marker_width);
            if cfg.viz.show_predicted_movers && nM > 0
                draw_mover_ghosts(movers_t, sim_t + cfg.dyn.pred_lead, cfg);
            end
            xlim(cfg.viz.xlim); ylim(cfg.viz.ylim);
            title(sprintf(['PHASE 2: Transport | t=%.1fs step %d | hdg %.0f deg | ' ...
                           'dist %.2f m | clearance %.2f m | v x %.2f'], sim_t, t, ...
                           rad2deg(leader_heading), dist_history(end), clr_now, speed_scale));
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

% ---- dynamic-obstacle extras ----
p2 = LOG.phase == 2;
if any(p2) && nM > 0
    fprintf('--- dynamic-obstacle metrics (phase 2) ---\n');
    fprintf('  mean speed scale    : %.3f  (1.0 = never slowed)\n', mean(LOG.vscale(p2)));
    fprintf('  time in slowdown    : %.1f%% of transport steps\n', ...
            100 * mean(LOG.vscale(p2) < 0.999));
    fprintf('  min mover gap (now) : %.3f m\n', min(LOG.near(p2)));
    fprintf('  min mover gap (pred): %.3f m\n', min(LOG.near_pre(p2)));
end

if cfg.log.enable
    fn = cfg.log.file;
    if isempty(fn)
        if exist('logs','dir') ~= 7, mkdir('logs'); end
        fn = fullfile('logs', [cfg.run_name '.csv']);
    end
    write_log_csv(fn, LOG, cfg);   % prints the ACTUAL path (may fall back to tempdir)
end

%% --- Diagnostic plots ---
% Every curve goes through PLINE (line + point markers) and every figure is
% also exported as a vector PDF into logs/figs/. Reason: MATLAB R2025a's
% WebGL figure engine on some Linux GPU drivers drops the line segments
% wherever a curve bends, so on screen only the straight stretches showed.
% Markers are drawn independently of the segments, and the PDF export does
% not go through the screen renderer.
if cfg.viz.enable
ts2 = (0:numel(heading_history)-1) * cfg.dt;      % transport time axis (s)
fig_diag = figure("Color","w","Name","Diagnostics");
subplot(4,1,1);
pline(ts2, rad2deg(heading_history), 'LineWidth', 1.5); grid on;
xlabel('Transport time (s)'); ylabel('Heading (deg)');
title('Rigid frame heading (smooth = load stays level)');

subplot(4,1,2);
pline(ts2, dist_history, 'LineWidth', 1.5); grid on;
xlabel('Transport time (s)'); ylabel('Distance to goal (m)');
title('Distance to goal');

subplot(4,1,3);
pline(LOG.t(p2), LOG.clr(p2), 'LineWidth', 1.5); grid on; hold on;
yline_compat(cfg.robot_radius, 'r--');
xlabel('Time (s)'); ylabel('Clearance (m)');
title('Formation clearance (static + dynamic obstacles; red = contact)');

subplot(4,1,4);
pline(LOG.t(p2), LOG.near(p2), 'LineWidth', 1.5); grid on; hold on;
yline_compat(cfg.robot_radius + cfg.dyn.slow_margin, 'm--');
xlabel('Time (s)'); ylabel('Gap to nearest mover (m)');
title('Gap to nearest moving obstacle (magenta = slowdown threshold)');

% GRAPH A: distance between each robot and the target vs time
target = cfg.waypoints(end,:);
K = cfg.num_robots;
dist_to_target = zeros(numel(LOG.t), K);
for i = 1:K
    dist_to_target(:,i) = hypot(LOG.rx(:,i) - target(1), LOG.ry(:,i) - target(2));
end
fig_target = figure('Color','w','Name','Distance to Target');
hold on; grid on;
for i = 1:K
    pline(LOG.t, dist_to_target(:,i), 'LineWidth', 1.5, 'Color', cfg.viz.colors(i,:));
end
xlabel('Time (s)'); ylabel('Distance to target (m)');
title('Distance between each robot and the target');
legend(arrayfun(@(i) sprintf('Robot %d', i), 1:K, 'UniformOutput', false), ...
       'Location', 'best');

% GRAPH B: inter-robot distance vs time
pairs = nchoosek(1:K, 2);
fig_pairs = figure('Color','w','Name','Inter-robot Distance');
hold on; grid on;
leg = cell(size(pairs,1), 1);
for p = 1:size(pairs,1)
    i = pairs(p,1); j = pairs(p,2);
    d = hypot(LOG.rx(:,i) - LOG.rx(:,j), LOG.ry(:,i) - LOG.ry(:,j));
    pline(LOG.t, d, 'LineWidth', 1.5);
    leg{p} = sprintf('R%d - R%d', i, j);
end
xlabel('Time (s)'); ylabel('Inter-robot distance (m)');
title('Pairwise distance between robots');
legend(leg, 'Location', 'best');

% GRAPH C: the whole mission on the map (centroid path, final formation)
fig_path = figure('Color','w','Name','Mission path'); hold on; grid on;
draw_world(merge_dyn(update_world_dyn(world, movers, sim_t)), 'Inflate', cfg.robot_radius);
p1 = LOG.phase(:) == 1;
if any(p1), pline(LOG.lx(p1), LOG.ly(p1), 'Color', [0.6 0.6 0.6], 'LineWidth', 1); end
pline(LOG.lx(p2), LOG.ly(p2), 'Color', [0 0.4 0.8], 'LineWidth', 1.5);
plot(cfg.start_point(1), cfg.start_point(2), 'gp', 'MarkerSize', 14, ...
     'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
plot(cfg.waypoints(:,1), cfg.waypoints(:,2), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
for i = 1:cfg.num_robots
    draw_robot(corners(i,:), cfg.robot_radius, cfg.viz.colors(i,:));
end
xlim(cfg.viz.xlim); ylim(cfg.viz.ylim); axis equal;
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('%s: centroid path (grey = forming, blue = transport), reached = %d', ...
              cfg.algo, reached_all), 'Interpreter', 'none');

% ---- vector export of every figure (bypasses the screen renderer) ----
figdir = fullfile('logs', 'figs');
if exist(figdir, 'dir') ~= 7, mkdir(figdir); end
figs  = {fig_path, fig_diag, fig_target, fig_pairs};
names = {'path', 'diagnostics', 'dist_to_target', 'inter_robot'};
for fi = 1:numel(figs)
    fn_fig = fullfile(figdir, sprintf('%s_%s.pdf', cfg.run_name, names{fi}));
    try
        exportgraphics(figs{fi}, fn_fig, 'ContentType', 'vector');
    catch
        try, print(figs{fi}, fn_fig, '-dpdf', '-bestfit'); catch, fn_fig = ''; end
    end
    if ~isempty(fn_fig), fprintf('Figure written to %s\n', fn_fig); end
end
end

%% =====================================================================
%  LOCAL HELPERS
% =====================================================================
function draw_mover_ghosts(movers, t_future, cfg)
%DRAW_MOVER_GHOSTS  Dashed outlines where the movers will be at t_future.
for k = 1:numel(movers)
    m = movers(k);
    p = eval_mover_pose(m, t_future);
    if strcmpi(m.shape, 'circle')
        th = linspace(0, 2*pi, 40);
        plot(p(1)+m.r*cos(th), p(2)+m.r*sin(th), '--', 'Color', [0.95 0.55 0.1]);
    else
        plot(p(1)+[-1 1 1 -1 -1]*m.size(1)/2, ...
             p(2)+[-1 -1 1 1 -1]*m.size(2)/2, '--', 'Color', [0.95 0.55 0.1]);
    end
end
end
