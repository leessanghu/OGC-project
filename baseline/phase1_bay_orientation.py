"""
phase1_bay_orientation.py — Phase 1: Bay Assignment + Orientation Selection

For each block, picks the best (bay_id, orient_idx) pair by:
  1. Feasibility filter: bounding box must fit inside the bay
  2. Scoring: weighted combination of bay preference (Z3) and workload balance (Z2)

Blocks are processed in EDD order (earliest due_date first); cumulative workload
is tracked so each assignment sees the current imbalance state of the bays.

Output: {block_id: {"bay_id": int, "orient_idx": int}}

Phase 2 will extend each entry with (x, y, entry_time, exit_time).
"""

from __future__ import annotations

import logging
import sys
import os

logger = logging.getLogger(__name__)

# utils.py lives alongside this file in the baseline/ directory.
# When imported from elsewhere (e.g. root myalgorithm.py) the caller is
# responsible for ensuring utils is importable — just as the competition
# server guarantees utils.py is present at root alongside myalgorithm.py.
try:
    from utils import Block  # type: ignore
except ImportError:
    # Fallback: try the baseline/ sibling directory (local dev from project root)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    from utils import Block  # type: ignore


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _compute_imbalance(loads: list[float], u: list[float], n_bays: int) -> float:
    """
    Max pairwise normalised workload imbalance — identical to check_feasibility's
    obj2 formula, but returns a float (not math.floor'd) so that continuous
    scoring differences are preserved during bay selection.

    Returns 0.0 when n_bays < 2 (no pairs exist).
    """
    if n_bays < 2:
        return 0.0
    return max(
        abs(u[j1] * loads[j1] - u[j2] * loads[j2])
        for j1 in range(n_bays)
        for j2 in range(n_bays)
        if j1 != j2
    )


def _precompute_footprints(
    blocks_data: list[dict],
) -> list[list[tuple[float, float, float]]]:
    """
    For every (block_i, orient_o) pair, compute (bw, bh, bbox_area) once.

    Uses Block(x=0, y=0, orient_idx=o).bounding_rect() which gives
    (min_x, min_y, max_x, max_y) in world coordinates.  Because the instance
    generator guarantees layer[0][0] == [0, 0] (reference point), world coords
    equal local coords at (x=0, y=0), so min_x == min_y == 0 always holds and
    bw == max_x, bh == max_y.

    Returns list[per-block list[per-orient (bw, bh, area)]].
    """
    result: list[list[tuple[float, float, float]]] = []
    for i, blk_data in enumerate(blocks_data):
        n_orient = len(blk_data["shape"])
        orient_fps: list[tuple[float, float, float]] = []
        for o in range(n_orient):
            blk  = Block(block_id=i, block_data=blk_data, x=0, y=0, orient_idx=o)
            bb   = blk.bounding_rect()        # (min_x, min_y, max_x, max_y)
            bw   = bb[2] - bb[0]
            bh   = bb[3] - bb[1]
            orient_fps.append((bw, bh, bw * bh))
        result.append(orient_fps)
    return result


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def assign_bay_and_orientation(prob_info: dict) -> dict[int, dict]:
    """
    Phase 1 entry point.

    Parameters
    ----------
    prob_info : instance dict (same structure as competition JSON)

    Returns
    -------
    dict mapping block_id -> {"bay_id": int, "orient_idx": int}
    Every block in prob_info["blocks"] appears exactly once in the output.
    """
    blocks_data = prob_info["blocks"]
    bays_data   = prob_info["bays"]
    weights     = prob_info.get("weights", {})

    n_blocks = len(blocks_data)
    n_bays   = len(bays_data)
    w2       = float(weights.get("w2", 1.0))
    w3       = float(weights.get("w3", 1.0))

    # u_j = avg_bay_area / (W_j * H_j)  — matches check_feasibility's obj2 formula
    bay_areas = [bays_data[j]["width"] * bays_data[j]["height"] for j in range(n_bays)]
    avg_area  = sum(bay_areas) / n_bays
    u         = [avg_area / area for area in bay_areas]

    # Pre-compute bounding-box footprints for every (block, orientation) pair.
    # This avoids constructing O(n_blocks * n_orient * n_bays) Block objects
    # inside the main loop; each block is constructed once per orientation here.
    footprints = _precompute_footprints(blocks_data)

    # Running per-bay workload (updated after each assignment)
    cumulative_workload = [0.0] * n_bays

    # EDD order: smallest due_date first, ties broken by block_id (deterministic)
    order = sorted(range(n_blocks),
                   key=lambda i: (blocks_data[i]["due_date"], i))

    result: dict[int, dict] = {}

    for i in order:
        blk_data = blocks_data[i]
        prefs    = blk_data["bay_preferences"]   # len == n_bays, sums to 100
        workload = float(blk_data["workload"])
        s_max    = max(prefs)
        n_orient = len(blk_data["shape"])

        # ------------------------------------------------------------------
        # Step 1: feasibility filter
        # For each bay j, find every orientation o whose bounding box fits
        # inside bay j's width × height.
        # Condition: bw <= W_j  AND  bh <= H_j
        # (Small float tolerance for coordinates that are almost exactly at
        #  the boundary due to floating-point representation.)
        # From the feasible set for bay j, keep the one with the smallest
        # bounding-box area (aids Phase 2 packing); ties broken by orient_idx.
        # ------------------------------------------------------------------
        # bay_best[j] = (orient_idx, bbox_area)  for the best feasible orient in bay j
        bay_best: dict[int, tuple[int, float]] = {}

        for j in range(n_bays):
            bay_w = bays_data[j]["width"]
            bay_h = bays_data[j]["height"]
            best_o:    int   = -1
            best_area: float = float("inf")

            for o in range(n_orient):
                bw, bh, area = footprints[i][o]
                if bw <= bay_w + 1e-6 and bh <= bay_h + 1e-6:
                    # Prefer smallest area; tie-break by lower orient_idx
                    if area < best_area or (area == best_area and o < best_o):
                        best_area = area
                        best_o    = o

            if best_o >= 0:
                bay_best[j] = (best_o, best_area)

        # ------------------------------------------------------------------
        # Fallback: no orientation fits in any bay
        # This should not happen for well-formed instances, but we must not
        # crash. Pick the (bay, orient) pair with the smallest total overflow.
        # ------------------------------------------------------------------
        if not bay_best:
            logger.warning(
                "Block %d: no orientation fits in any bay — "
                "assigning least-bad (smallest overflow) combination.", i
            )
            fb_j = 0
            fb_o = 0
            fb_overflow = float("inf")
            for j in range(n_bays):
                bay_w = bays_data[j]["width"]
                bay_h = bays_data[j]["height"]
                for o in range(n_orient):
                    bw, bh, _ = footprints[i][o]
                    overflow = max(0.0, bw - bay_w) + max(0.0, bh - bay_h)
                    if overflow < fb_overflow:
                        fb_overflow = overflow
                        fb_j        = j
                        fb_o        = o
            result[i] = {"bay_id": fb_j, "orient_idx": fb_o}
            cumulative_workload[fb_j] += workload
            continue

        # ------------------------------------------------------------------
        # Step 2: score each feasible bay and pick the best
        #
        # score(j) = -w3 * (S_max - S_j)   [preference: higher S_j is better]
        #          - w2 * imbalance_after_j  [balance: lower imbalance is better]
        #
        # We pick argmax(score).  Using -w3*(S_max-S_j) is equivalent to
        # maximising the bay preference score at Z3 cost w3 per unit.
        # Using -w2*imbalance_after_j is a greedy one-step approximation of the
        # Z2 contribution; it correctly penalises assigning to an already-heavy
        # bay when bays are imbalanced.
        # ------------------------------------------------------------------
        best_score:  float | None = None
        best_j:      int          = next(iter(bay_best))   # default: first feasible bay
        best_orient: int          = bay_best[best_j][0]

        for j, (orient_idx, _) in bay_best.items():
            # Compute hypothetical imbalance after assigning block i to bay j
            tentative = cumulative_workload[:]
            tentative[j] += workload
            new_imbalance = _compute_imbalance(tentative, u, n_bays)

            score = -w3 * (s_max - prefs[j]) - w2 * new_imbalance

            if best_score is None or score > best_score:
                best_score  = score
                best_j      = j
                best_orient = orient_idx

        result[i] = {"bay_id": best_j, "orient_idx": best_orient}
        cumulative_workload[best_j] += workload

    return result


# ---------------------------------------------------------------------------
# CLI smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import json
    import math

    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <instance.json>")
        sys.exit(1)

    path = sys.argv[1]
    with open(path) as fh:
        prob_info = json.load(fh)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    assignment = assign_bay_and_orientation(prob_info)

    blocks_data = prob_info["blocks"]
    bays_data   = prob_info["bays"]
    weights     = prob_info.get("weights", {})
    n_bays      = len(bays_data)
    n_blocks    = len(blocks_data)

    # Aggregate per-bay statistics
    bay_count    = [0]   * n_bays
    bay_workload = [0.0] * n_bays
    bay_pref_sum = [0.0] * n_bays

    for bid, asgn in assignment.items():
        j = asgn["bay_id"]
        bay_count[j]    += 1
        bay_workload[j] += blocks_data[bid]["workload"]
        bay_pref_sum[j] += blocks_data[bid]["bay_preferences"][j]

    # --- Summary header ---
    print(f"\nInstance : {prob_info.get('name', path)}")
    print(f"Blocks   : {n_blocks}   Bays: {n_bays}")
    print(f"Weights  : {weights}")
    print()
    print(f"{'Bay':>4}  {'Size':>10}  {'#Blocks':>8}  {'Workload':>10}  {'AvgPref':>8}")
    print("-" * 50)
    for j in range(n_bays):
        size_str = f"{bays_data[j]['width']}x{bays_data[j]['height']}"
        avg_pref = bay_pref_sum[j] / bay_count[j] if bay_count[j] > 0 else 0.0
        print(f"  {j:2d}  {size_str:>10}  {bay_count[j]:8d}  "
              f"{bay_workload[j]:10.0f}  {avg_pref:8.1f}")
    print()

    # Obj2 imbalance estimate (same formula as check_feasibility)
    bay_areas = [bays_data[j]["width"] * bays_data[j]["height"] for j in range(n_bays)]
    avg_area  = sum(bay_areas) / n_bays
    u         = [avg_area / a for a in bay_areas]
    if n_bays >= 2:
        raw_imbalance = max(
            abs(u[j1] * bay_workload[j1] - u[j2] * bay_workload[j2])
            for j1 in range(n_bays)
            for j2 in range(n_bays)
            if j1 != j2
        )
        obj2_est = math.floor(raw_imbalance)
        print(f"Obj2 imbalance estimate : {obj2_est}  (raw={raw_imbalance:.2f})")
    else:
        print("Obj2 imbalance estimate : 0  (single bay, no pairs)")

    # Obj3 preference penalty estimate
    obj3 = sum(
        max(blocks_data[bid]["bay_preferences"])
        - blocks_data[bid]["bay_preferences"][assignment[bid]["bay_id"]]
        for bid in assignment
    )
    print(f"Obj3 preference penalty : {obj3:.0f}")

    # Orientation distribution (optional diagnostic)
    orient_counts: dict[int, int] = {}
    for asgn in assignment.values():
        o = asgn["orient_idx"]
        orient_counts[o] = orient_counts.get(o, 0) + 1
    print(f"\nOrientation usage : {dict(sorted(orient_counts.items()))}")
