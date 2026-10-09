function [robots_start, start_point, waypoints] = pick_points_sdf(cfg, world)
    f = figure('Color','w'); hold on; grid on;
    draw_world(world, 'Inflate', cfg.robot_radius);
    xlim(cfg.viz.xlim); ylim(cfg.viz.ylim);
    robots_start = [];
    if ~cfg.spawn_in_formation
        title(sprintf('Click %d SCATTERED robot start positions', cfg.num_robots));
        [xs, ys] = ginput(cfg.num_robots);
        robots_start = [xs(:) ys(:)];
    end
    title('Click the FORMATION START POINT');
    [xp, yp] = ginput(1);
    start_point = [xp yp];
    title('Click GOAL waypoint(s), then press ENTER');
    [xw, yw] = ginput;
    if isempty(xw), error('At least one goal waypoint is required.'); end
    waypoints = [xw(:) yw(:)];
    close(f);
end
