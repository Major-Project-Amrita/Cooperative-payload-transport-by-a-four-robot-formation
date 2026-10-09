function val = vfh_commit(action, tag, t_now, val)
%VFH_COMMIT  One-step memory of the sector a histogram law chose last.
%
%   prev = VFH_COMMIT('get', tag, t_now)        [] if none or stale
%          VFH_COMMIT('set', tag, t_now, hdg)
%
%   VFH+ (Ulrich & Borenstein 1998) weighs three terms when it picks a free
%   sector: deviation from the goal, from the current heading, and from the
%   PREVIOUSLY SELECTED direction. The third term is what stops the law
%   from flipping between "left around" and "right around" an obstacle
%   cluster that blocks the goal bearing; without it the formation can
%   hover in front of the cluster for as long as the stall detector allows.
%   The steering functions are otherwise stateless, so the previous choice
%   lives here, keyed by tag and invalidated when time goes backward or
%   more than 0.5 s passes (a new mission, or an escape in between).
persistent M
if isempty(M), M = struct(); end
switch action
    case 'get'
        val = [];
        if isfield(M, tag) && t_now >= M.(tag).t - 1e-9 && t_now <= M.(tag).t + 0.5
            val = M.(tag).h;
        end
    case 'set'
        M.(tag) = struct('h', val, 't', t_now);
    otherwise
        error('vfh_commit: unknown action ''%s''', action);
end
end
