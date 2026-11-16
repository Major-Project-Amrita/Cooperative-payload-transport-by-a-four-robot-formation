# Cross-stack comparison (fixedgate)

MATLAB: `results/matlab_fixedgate/results_all.csv` (2730 runs). Python: `results/results_headless_exactgate.csv` (2730 runs).

- cells compared: **221**
- agree on reach, rejections and minimum gap (all three intervals overlap): **221/221** (100.0 %)
- reach intervals overlap: 221/221; rejection intervals: 221/221; gap intervals: 221/221; time intervals: 220/221
- static-contact steps, total: MATLAB 0, Python 0 (must be 0)
- runs reaching the goal: MATLAB 2605/2730, Python 2621/2730

Pearson correlation of the cell means across stacks: time r = 0.981, rejections r = 0.949, min gap r = 0.990

## Per experiment

| exp | cells | agree | reach M | reach Py | time M (s) | time Py (s) |
|---|---|---|---|---|---|---|
| E1 | 8 | 8 | 79/80 | 78/80 | 63.3 | 66.1 |
| E2 | 20 | 20 | 392/400 | 396/400 | 72.4 | 70.8 |
| E3 | 8 | 8 | 80/80 | 80/80 | 53.0 | 52.4 |
| E5 | 32 | 32 | 306/320 | 309/320 | 71.5 | 69.0 |
| E6 | 24 | 24 | 240/240 | 240/240 | 51.6 | 51.2 |
| E7 | 32 | 32 | 307/320 | 305/320 | 111.6 | 116.8 |
| E8 | 20 | 20 | 199/200 | 200/200 | 55.0 | 54.0 |
| E9 | 4 | 4 | 1/40 | 4/40 | 301.4 | 301.2 |
| E10 | 16 | 16 | 160/160 | 160/160 | 49.0 | 49.1 |
| E11 | 20 | 20 | 192/200 | 189/200 | 60.6 | 64.3 |
| E12 | 16 | 16 | 160/160 | 160/160 | 49.3 | 49.4 |
| E13 | 16 | 16 | 439/480 | 450/480 | 87.4 | 81.9 |
| E14 | 5 | 5 | 50/50 | 50/50 | 53.0 | 53.1 |

## Disagreeing cells (0)

| exp | cell | reach M | reach Py | rej M [CI] | rej Py [CI] | gap M [CI] | gap Py [CI] | differs in |
|---|---|---|---|---|---|---|---|---|
