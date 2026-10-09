function write_log_csv(fn, LOG, cfg)
%WRITE_LOG_CSV  Write the shared-schema run log (MATLAB + Octave).
%   The mission is expensive and the log is a by-product, so an unwritable
%   path must never discard the run: the target folder is created on demand
%   and, if the file still cannot be opened, the CSV falls back to tempdir
%   with a loud warning instead of erroring after a completed mission.
    K = cfg.num_robots;
    hdr = {'t','phase','leader_x','leader_y','leader_yaw','dist_to_goal','clearance'};
    for i = 1:K, hdr{end+1} = sprintf('r%d_x', i); end %#ok<AGROW>
    for i = 1:K, hdr{end+1} = sprintf('r%d_y', i); end %#ok<AGROW>
    M = [LOG.t(:), LOG.phase(:), LOG.lx(:), LOG.ly(:), LOG.lh(:), ...
         LOG.dist(:), LOG.clr(:), LOG.rx, LOG.ry];

    % ---- optional dynamic-obstacle columns (Simulation8_dynamic) ----
    % Appended only when present, so static runs keep the exact original
    % schema and compare_runs.m / swarm_logger.py stay unchanged.
    has = @(f) isfield(LOG, f) && numel(LOG.(f)) == size(M,1);
    if has('cvx') && has('cvy') && has('vscale') && has('near') && has('near_pre')
        hdr{end+1} = 'mover_vx';       %#ok<AGROW> velocity of nearest mover
        hdr{end+1} = 'mover_vy';       %#ok<AGROW>
        hdr{end+1} = 'speed_scale';    %#ok<AGROW> 1 = never slowed
        hdr{end+1} = 'mover_gap';      %#ok<AGROW> gap to nearest mover, now
        hdr{end+1} = 'mover_gap_pred'; %#ok<AGROW> worst gap over lookahead
        D = [LOG.cvx(:), LOG.cvy(:), LOG.vscale(:), LOG.near(:), LOG.near_pre(:)];
        D(~isfinite(D)) = 99;   % sentinel: no mover nearby / nothing to report
        M = [M, D];
    end
    if has('comp')   % steering compute time this step (ms)
        hdr{end+1} = 'comp_ms';        %#ok<AGROW>
        C = LOG.comp(:);
        M = [M, C];
    end

    % ---- optional audit columns: safety-gate rejections, escape commits,
    %      static-only clearance (separates mover hits from static hits) ----
    if has('rej') && has('esc') && has('clr_s')
        hdr{end+1} = 'reject';            %#ok<AGROW> 0 ok, 1 translation, 2 pose
        hdr{end+1} = 'escape';            %#ok<AGROW> 1 = committed escape step
        hdr{end+1} = 'clearance_static';  %#ok<AGROW>
        M = [M, LOG.rej(:), LOG.esc(:), LOG.clr_s(:)];
    end

    % ---- open robustly: mkdir the folder, fall back to tempdir ----
    [fdir, fname, fext] = fileparts(fn);
    if ~isempty(fdir) && exist(fdir, 'dir') ~= 7
        try
            mkdir(fdir);
        catch
            % leave it to fopen to fail and take the fallback path
        end
    end
    fid = fopen(fn, 'w');
    if fid == -1
        fallback = fullfile(tempdir, ['swarm_log_' fname fext]);
        fid = fopen(fallback, 'w');
        if fid == -1
            warning('write_log_csv:cannotWrite', ...
                ['could not open %s (or a tempdir fallback) for writing; ' ...
                 'run metrics are printed above but NO CSV was saved.'], fn);
            return;
        end
        warning('write_log_csv:fallbackPath', ...
            'could not write %s (folder missing or no permission); CSV saved to %s instead.', ...
            fn, fallback);
        fn = fallback;
    end

    fprintf(fid, '%s\n', strjoin(hdr, ','));
    fmt = [repmat('%.6f,', 1, size(M,2)-1) '%.6f\n'];
    fprintf(fid, fmt, M');
    fclose(fid);
    fprintf('Log written to %s\n', fn);
end
