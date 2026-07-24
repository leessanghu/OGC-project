# OGC 2026 Optimization Challenge

This repository contains my work for the **OGC 2026 Optimization Challenge**.
The task is to build a feasible schedule and spatial layout for shipyard-like
blocks inside multiple bays while minimizing a weighted objective:

- **obj1**: tardiness penalty
- **obj2**: workload imbalance between bays
- **obj3**: bay preference penalty

The submitted solver is implemented in pure Python as `submit/myalgorithm.py`.
It follows the contest-style interface:

```python
def algorithm(prob_info: dict, timelimit: float = 60, repair_mode: str = "greedy") -> dict:
    return {"operations": ...}
```

## Repository Layout

- `submit/` - current submission algorithm
- `baseline/` - earlier baseline algorithms and experiments
- `train/` - public training instances
- `alg_tester/` - local tester and official-style feasibility checker
- `results/` - experiment logs and comparison results
- `Idea/` - research notes, failed ideas, and future improvement plans
- `P3 240k/`, `P3 360k/`, `P6/`, `P6 worst/` - important historical algorithm snapshots

## Main Techniques Used

### Greedy Construction With Spatial Feasibility

The base solver constructs a schedule by ordering blocks and placing each block
into a bay, orientation, position, and time window. Candidate placements are
checked against geometric feasibility and crane entry/exit constraints.

### Portfolio of Construction Orders

Different instances respond very differently to block order. I tested and used
multiple ordering strategies such as:

- EDD, earliest due date
- latest-start / slack based ordering
- SPT-like variants
- tardy-first reservation ordering
- bay-time bucket ordering
- GRASP-style randomized restricted candidate lists
- BRKGA-inspired rank-key order perturbations

The solver often compares candidate constructions by the official objective and
keeps only the best feasible one.

### Tail Portfolio and Tail Local Search

For large saturated instances, much of the objective comes from late tail blocks.
I used tail-specific order simulation, late-block destroy/reinsert, and bounded
local search to improve the most expensive tail portion without rebuilding the
entire solution.

### AABB / Raster-Based Placement Repair

Several repair phases try to move tardy blocks earlier using fast bounding-box
and raster-style scans. This was useful for reducing tardiness while staying
inside the strict 60-second budget.

### Zero-Tardy Controller

For easier instances where `obj1 == 0`, the problem becomes mostly an `obj2` and
`obj3` optimization problem. The solver has dedicated deterministic polish paths
that avoid breaking zero tardiness while improving workload balance and bay
preference cost.

### ALNS / Causal Repair Ideas

I experimented with large-neighborhood search ideas:

- remove high-tardiness blocks
- remove blocks that physically block their earlier placement
- reinsert by regret-style priorities
- route mid-tardy instances into a CausalALNS-heavy path when it shows promise

Some variants helped specific public proxies, while others were rejected because
they overfit local instances or consumed too much time.

### Geometry and Density Experiments

A major bottleneck was geometric packing density. I tested:

- raster scan cap tuning
- layer-aware placement rescue
- EMS / extreme-point style construction prototypes
- bay-pair transfer and large-window repair

Layer-aware placement proved that additional legal geometric placements exist,
but naive insertion could break later crane exits or worsen global scheduling.
Those risky versions were kept out of the final default path unless protected by
official-objective checks.

## Running Locally

Create the environment:

```bash
conda env create -f ogc2026_env.yml
conda activate ogc2026
```

Run the local tester:

```bash
python alg_tester/alg_tester_app.py
```

Run a batch test against selected training problems:

```bash
python tools/run_submit_batch.py 8 2 27 38 --timelimit 60
```

## Notes

The project contains many experimental branches and snapshots because hidden
leaderboard behavior differed significantly from public proxy instances. A large
part of the work was not only improving objective values, but also identifying
which ideas were robust and which were local overfitting.
