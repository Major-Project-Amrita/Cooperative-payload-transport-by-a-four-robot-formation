function perm = assign_slots(cost)
%ASSIGN_SLOTS  Minimum-cost robot-to-slot assignment.
%
%   perm = ASSIGN_SLOTS(cost)  with cost(i,j) = cost of robot i taking slot j.
%   perm(i) is the slot index assigned to robot i.
%
%   Uses MATCHPAIRS when available (MATLAB R2019a+). Falls back to an exact
%   exhaustive search for small swarms and a greedy pass beyond that, so the
%   pipeline still runs on older MATLAB and on Octave.

K = size(cost,1);

if exist('matchpairs', 'file') == 2 || exist('matchpairs', 'builtin') == 5
    pairings = matchpairs(cost, 1e5);
    [~, si]  = sort(pairings(:,1));
    perm     = pairings(si, 2);
    return;
end

% Octave / pre-R2019a MATLAB: the shortest-augmenting-path Jonker-Volgenant
% algorithm of Crouse (2016), statement for statement as in SciPy's
% linear_sum_assignment (same scan order, same tie rules). Optimal
% assignments are often TIED here (the start ring is symmetric); an
% exhaustive search would return a different optimum than MATCHPAIRS/SciPy
% and break the step-for-step replication. Verified against SciPy on
% 20,000 random and tie-heavy matrices (tools/check_lsap_port.py).
perm = lsap_crouse(cost);
return;

if K <= 8 %#ok<UNRCH>  (kept for reference: the former fallback)
    P    = perms(1:K);
    best = inf; perm = (1:K)';
    for i = 1:size(P,1)
        c = sum(cost(sub2ind([K K], (1:K)', P(i,:)')));
        if c < best, best = c; perm = P(i,:)'; end
    end
    return;
end

% Greedy fallback: repeatedly take the cheapest remaining pair.
perm = zeros(K,1);
C    = cost;
for k = 1:K
    [~, idx] = min(C(:));
    [r, c]   = ind2sub(size(C), idx);
    perm(r)  = c;
    C(r,:)   = inf;
    C(:,c)   = inf;
end
end

function col4row = lsap_crouse(C)
[nr, nc] = size(C);
u = zeros(nr,1); v = zeros(nc,1); spc = inf(nc,1);
path = zeros(nc,1); col4row = zeros(nr,1); row4col = zeros(nc,1);
for cur = 1:nr
    minVal = 0; remaining = (nc:-1:1)'; nrem = nc;
    SR = false(nr,1); SC = false(nc,1); spc(:) = inf;
    sink = 0; i = cur;
    while sink == 0
        index = 0; lowest = inf; SR(i) = true;
        for it = 1:nrem
            j = remaining(it);
            r = minVal + C(i,j) - u(i) - v(j);
            if r < spc(j)
                path(j) = i; spc(j) = r;
            end
            if spc(j) < lowest || (spc(j) == lowest && row4col(j) == 0)
                lowest = spc(j); index = it;
            end
        end
        minVal = lowest;
        j = remaining(index);
        if row4col(j) == 0, sink = j; else, i = row4col(j); end
        SC(j) = true;
        remaining(index) = remaining(nrem); nrem = nrem - 1;
    end
    u(cur) = u(cur) + minVal;
    for i = 1:nr
        if SR(i) && i ~= cur, u(i) = u(i) + minVal - spc(col4row(i)); end
    end
    for j = 1:nc
        if SC(j), v(j) = v(j) - (minVal - spc(j)); end
    end
    j = sink;
    while true
        i = path(j); row4col(j) = i;
        tmp = col4row(i); col4row(i) = j; j = tmp;
        if i == cur, break; end
    end
end
end
