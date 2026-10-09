# Cross-stack comparison

MATLAB: `bundle/reference/results_all.csv` (2730 runs). Python: `results/results_headless.csv` (2730 runs).

- cells compared: **221**
- agree on reach, rejections and minimum gap (all three intervals overlap): **220/221** (99.5 %)
- reach intervals overlap: 221/221; rejection intervals: 221/221; gap intervals: 220/221; time intervals: 219/221
- static-contact steps, total: MATLAB 0, Python 0 (must be 0)
- runs reaching the goal: MATLAB 2626/2730, Python 2613/2730

Pearson correlation of the cell means across stacks: time r = 0.964, rejections r = 0.903, min gap r = 0.991

## Per experiment

| exp | cells | agree | reach M | reach Py | time M (s) | time Py (s) |
|---|---|---|---|---|---|---|
| E1 | 8 | 8 | 80/80 | 77/80 | 60.8 | 68.8 |
| E2 | 20 | 20 | 397/400 | 395/400 | 69.8 | 71.2 |
| E3 | 8 | 8 | 80/80 | 80/80 | 52.3 | 52.4 |
| E5 | 32 | 32 | 305/320 | 307/320 | 71.3 | 70.0 |
| E6 | 24 | 24 | 240/240 | 240/240 | 51.2 | 51.3 |
| E7 | 32 | 32 | 308/320 | 306/320 | 112.3 | 116.8 |
| E8 | 20 | 20 | 194/200 | 198/200 | 61.3 | 56.5 |
| E9 | 4 | 4 | 3/40 | 1/40 | 295.6 | 305.5 |
| E10 | 16 | 16 | 160/160 | 160/160 | 49.4 | 49.5 |
| E11 | 20 | 20 | 194/200 | 189/200 | 58.5 | 64.5 |
| E12 | 16 | 15 | 160/160 | 160/160 | 49.6 | 49.7 |
| E13 | 16 | 16 | 455/480 | 450/480 | 80.4 | 81.8 |
| E14 | 5 | 5 | 50/50 | 50/50 | 53.8 | 53.7 |

## Disagreeing cells (1)

| exp | cell | reach M | reach Py | rej M [CI] | rej Py [CI] | gap M [CI] | gap Py [CI] | differs in |
|---|---|---|---|---|---|---|---|---|
| E12 | MPC, drop 0.5 | 10/10 | 10/10 | 0.0 [0.0,0.0] | 4.4 [0.0,10.1] | 0.20 [0.18,0.22] | 0.06 [-0.08,0.17] | gap |
