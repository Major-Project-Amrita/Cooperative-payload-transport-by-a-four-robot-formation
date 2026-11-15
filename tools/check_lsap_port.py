#!/usr/bin/env python3
"""Check that matlab_ref/assign_slots.m (Octave fallback, lsap_crouse) returns
exactly SciPy's linear_sum_assignment on random and tie-heavy matrices.

    python3 tools/check_lsap_port.py [n_matrices]
"""
import os, subprocess, sys, tempfile
import numpy as np
from scipy.optimize import linear_sum_assignment

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
n = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
rng = np.random.default_rng(0)
Ms = []
for t in range(n):
    K = int(rng.integers(2, 10))
    Ms.append(rng.integers(0, 4, (K, K)).astype(float) if t % 2 else rng.normal(size=(K, K)))
with tempfile.TemporaryDirectory() as d:
    for i, M in enumerate(Ms):
        np.savetxt(os.path.join(d, f"m{i}.txt"), M, fmt="%.17g")
    script = (f"cd('{os.path.join(ROOT, 'matlab_ref')}'); fid=fopen('{d}/out.txt','w'); "
              f"for i=0:{n-1}, C=load(sprintf('{d}/m%d.txt',i)); p=assign_slots(C); "
              "fprintf(fid,'%d ',p); fprintf(fid,'\\n'); end; fclose(fid);")
    subprocess.run(["octave-cli", "--no-gui", "--quiet", "--eval", script], check=True)
    out = open(os.path.join(d, "out.txt")).read().splitlines()
bad = 0
for M, line in zip(Ms, out):
    r, c = linear_sum_assignment(M)
    p = np.empty(len(M), int); p[r] = c
    bad += not np.array_equal(p + 1, np.array(line.split(), int))
print(f"{bad} mismatches out of {n}")
sys.exit(1 if bad else 0)
