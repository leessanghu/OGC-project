# OGC 2026 Third Submission

This branch contains the third submission algorithm:

```text
submit/myalgorithm.py
```

The submitted file is copied from `baseline/myalgorithm6.py`.

## Summary

`myalgorithm6.py` keeps the stable `myalgorithm4.py` core:

- EDD greedy placement
- adaptive K-best candidate search
- due-date shielding
- same-time ENTRY/EXIT topological ordering
- greedy repair with bounded destroy/repair fallback

Compared with `myalgorithm5.py`, this version does not rely on the discarded experimental SPT ordering, polygon-tight candidate mode, left-shift pass, or tardiness-LNS toggles. Those experiments were useful diagnostically, but they were unstable or slower on dense cases such as `prob_38`.

The third submission focuses on two AABB-based changes that directly reduce late forced placements.

## New In `myalgorithm6.py`

### 1. AABB-Disjoint Force Placement

The fallback `_force_place()` no longer only waits for a fully empty bay window. It first searches for a placement whose bounding box is disjoint from all blocks active in the target time window.

If such a placement exists, the expensive geometric checks are structurally avoided by the same AABB pre-filter logic used inside the feasibility predicates. This helps dense instances where many blocks would otherwise be pushed far into the future.

### 2. AABB Immediate-Entry Priority Before K-Best

Right before K-best collection in `_place_blocks()`, candidate positions are stably reordered:

- positions that can enter immediately at `release_time`
- and whose AABB is disjoint from all blocks overlapping `[release_time, release_time + processing_time)`
- are moved to the front of the candidate list

The original bottom-left order is preserved inside each group. This keeps the algorithm conservative while making K-best spend its limited checks on candidates that are more likely to avoid tardiness.

## Measured Improvements

Local feasibility checks remain PASS on the measured instances.

| Instance | Feasibility | Blocks | Previous Tardiness | New Tardiness | Improvement |
|---|---:|---:|---:|---:|---:|
| `prob_38` | FEASIBLE | 250/250 | 11,393 | 8,484 | -2,909 |
| `prob_25` | FEASIBLE | 100/100 | 2,428 | 1,488 | -940 |

Reference objective snapshots:

```text
prob_38
[Feasibility] FEASIBLE  (250/250 blocks)
  objective = 154432149.0000
  T (tardiness) = 11393.0000 -> 8484

prob_25
[Feasibility] FEASIBLE  (100/100 blocks)
  objective = 1662795.0000
  T (tardiness) = 2428.0000 -> 1488
```

## Files

```text
submit/myalgorithm.py        # official submission entry point
baseline/myalgorithm6.py     # development copy of the same algorithm
baseline/myalgorithm4.py     # stable previous baseline
```

## Run Locally

```powershell
conda activate ogc2026
cd alg_tester
python alg_tester_app.py
```

Select:

```text
Instance folder/file: train/prob_*.json
Algorithm folder: submit
```
