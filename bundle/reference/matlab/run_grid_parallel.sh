#!/usr/bin/env bash
# Launch the paper's Monte Carlo grid as N parallel MATLAB processes, then
# merge. Resumable: re-running skips every configuration already in
# logs/paper/parts/. Usage:
#     matlab/run_grid_parallel.sh [N_PROCESSES] [MODE] [SEEDS]
#       N_PROCESSES  default 6   (one MATLAB per shard; ~1.5 GB RAM each)
#       MODE         full | quick | smoke | e1..e9      (default full)
#       SEEDS        seeds per cell (default: 10 full, 5 quick)
# Octave instead of MATLAB:  MATLAB_CMD="octave --no-gui --quiet --eval" ...
set -euo pipefail
N="${1:-6}"; MODE="${2:-full}"; SEEDS="${3:-[]}"
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNNER="${MATLAB_CMD:-matlab -batch}"
mkdir -p "$HERE/logs/paper/shardlogs"
pids=()
for k in $(seq 1 "$N"); do
  $RUNNER "cd('$HERE'); run_paper_experiments('$MODE', $k, $N, $SEEDS)" \
      > "$HERE/logs/paper/shardlogs/shard_${k}of${N}.log" 2>&1 &
  pids+=($!)
  echo "shard $k/$N started (pid $!), log: logs/paper/shardlogs/shard_${k}of${N}.log"
done
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=1; done
[ "$fail" -eq 0 ] || echo "WARNING: at least one shard exited non-zero (see shardlogs/); merging what exists."
$RUNNER "cd('$HERE'); run_paper_experiments('merge')"
echo "Done. Next: python3 $HERE/make_paper_figures.py"
