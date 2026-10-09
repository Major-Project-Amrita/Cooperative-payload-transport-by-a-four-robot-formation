"""Robot-to-slot assignment, port of assign_slots.m.

scipy.optimize.linear_sum_assignment solves exactly the same problem as
MATLAB's matchpairs (Jonker-Volgenant variant); the exhaustive and greedy
fallbacks mirror the MATLAB file so behaviour is identical everywhere.
"""

from __future__ import annotations

from itertools import permutations

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment as _lsa
    HAVE_SCIPY = True
except ImportError:                                   # pragma: no cover
    HAVE_SCIPY = False


def assign_slots(cost) -> np.ndarray:
    """perm[i] = slot index assigned to robot i (minimum total cost)."""
    cost = np.asarray(cost, dtype=float)
    K = cost.shape[0]

    if HAVE_SCIPY:
        rows, cols = _lsa(cost)
        perm = np.empty(K, dtype=int)
        perm[rows] = cols
        return perm

    if K <= 8:                                        # exhaustive (as MATLAB)
        best, perm = np.inf, np.arange(K)
        for p in permutations(range(K)):
            c = sum(cost[i, p[i]] for i in range(K))
            if c < best:
                best, perm = c, np.array(p)
        return perm

    C = cost.copy()                                   # greedy fallback
    perm = np.empty(K, dtype=int)
    for _ in range(K):
        idx = int(np.argmin(C))
        r, c = divmod(idx, K)
        perm[r] = c
        C[r, :] = np.inf
        C[:, c] = np.inf
    return perm
