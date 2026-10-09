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

if K <= 8
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
