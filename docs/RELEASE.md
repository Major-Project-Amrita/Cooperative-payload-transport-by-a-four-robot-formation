# Release and DOI (review step 6)

The benchmark is released as two archives with a SHA-256 manifest:

| Archive | Contents |
|---|---|
| `cooperative-transport-benchmark-<v>.tar.gz` | code (MATLAB reference, Python/ROS 2 package, tools), scenario files (SDF worlds, JSON traffic contracts, generated warehouse layouts), configurations (`results/tuning/best_params.json`, experiment lists in `tools/run_extended.py`), trained model weights (`src/ros2_swarm_transport/models/`), every derived result table, figures and docs |
| `cooperative-transport-benchmark-<v>-runs.tar.gz` | per-step run logs |
| `MANIFEST.sha256` | SHA-256 of every file in both archives |

## Build

```bash
python3 tools/make_release.py --version 1.0.0
```

Nothing is uploaded by this command.

## Mint the DOI (done by a repository owner)

1. Complete the author list, affiliation and ORCIDs in `CITATION.cff` and
   `.zenodo.json` (the `TODO` entries).
2. Add a `LICENSE` file. The ROS 2 package already declares MIT
   (`package.xml`, `setup.py`); the release should use the same licence
   unless the authors decide otherwise.
3. Sign in to zenodo.org with GitHub, open *GitHub* in the account menu and
   switch the repository on.
4. On GitHub, create a release with tag `v1.0.0`. Zenodo archives the tagged
   tree and mints a version DOI plus a concept DOI (the concept DOI always
   resolves to the latest version).
5. Upload `dist/*-runs.tar.gz` to the same Zenodo record (it is too large for
   the git tree).
6. Put the concept DOI into `CITATION.cff` (`doi:`), the README badge and the
   manuscript's data-availability statement.

## Data-availability statement (draft)

> All code, scenario files, configurations, trained model weights and derived
> results are archived at Zenodo, DOI 10.5281/zenodo.XXXXXXX (version 1.0.0),
> with SHA-256 checksums for every file. The study is entirely simulated; no
> hardware data were collected.
