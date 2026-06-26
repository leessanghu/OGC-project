"""
myalgorithm.py — OGC 2026 submission entry point
=================================================
Two-phase pipeline:
  Phase 1  placement: bay + orient + position + entry time, all at once
           (SPT order, best-score across all feasible (bay, orient, pos))
  Phase 2  exit-time refinement
           (EDD order, single sequential pass)

Placement scoring per candidate (bay, orient, position):
  (tardiness_proxy, w3*pref_loss, w2*balance, entry_time)  — minimised lexicographically

Packages used: only utils (provided by server) + Python stdlib.
"""

from __future__ import annotations

import math
import time
import logging

from utils import Bay, Block, check_entry, check_exit, check_collisions, check_feasibility

logger = logging.getLogger(__name__)
_LP = "[alg]"   # log prefix

# ─────────────────────────────────────────────────────────────────────────────
# Shared geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _time_overlaps(a0: int, a1: int, b0: int, b1: int) -> bool:
    """True if half-open intervals [a0,a1) and [b0,b1) share any point."""
    return a0 < b1 and b0 < a1


def _precompute_footprints(blocks_data: list[dict]) -> list[list[tuple]]:
    """
    For every (block_i, orient_o) pair, compute and cache the bounding rect.

    footprints[i][o] = (lx0, ly0, lx1, ly1, bbox_area)
    where (lx0,ly0,lx1,ly1) = Block(x=0,y=0,orient_idx=o).bounding_rect().

    Because the instance generator guarantees layer[0][0]==[0,0] as the
    reference vertex, lx0≈0 and ly0≈0 for all well-formed blocks; we store
    the full bounding rect anyway for generality.
    """
    fp: list[list[tuple]] = []
    for i, bd in enumerate(blocks_data):
        row: list[tuple] = []
        for o in range(len(bd["shape"])):
            blk = Block(block_id=i, block_data=bd, x=0, y=0, orient_idx=o)
            lx0, ly0, lx1, ly1 = blk.bounding_rect()
            row.append((lx0, ly0, lx1, ly1, (lx1 - lx0) * (ly1 - ly0)))
        fp.append(row)
    return fp


def _compute_imbalance(loads: list[float], u: list[float], n_bays: int) -> float:
    """
    Max pairwise normalised workload imbalance — identical to check_feasibility
    obj2 formula (float, not floor'd, so continuous scoring differences are kept).
    Returns 0.0 for n_bays < 2.
    """
    if n_bays < 2:
        return 0.0
    return max(
        abs(u[j1] * loads[j1] - u[j2] * loads[j2])
        for j1 in range(n_bays)
        for j2 in range(n_bays)
        if j1 != j2
    )


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 helpers
# ─────────────────────────────────────────────────────────────────────────────

def _candidate_positions(
    bay_w: float,
    bay_h: float,
    placed_blocks: list[Block],
    blk_bb: tuple[float, float, float, float],
) -> list[tuple[int, int]]:
    """
    Bottom-left fill: integer (x, y) reference-point candidates sorted by (x, y).
    Seed set = {bay origin} ∪ {right/top edges of already-placed blocks}.
    Only positions where the block's world bbox stays within the bay are returned.
    """
    lx0, ly0, lx1, ly1 = blk_bb
    xs: set[int] = {max(0, math.ceil(-lx0))}
    ys: set[int] = {max(0, math.ceil(-ly0))}
    for b in placed_blocks:
        bb = b.bounding_rect()
        xs.add(math.ceil(bb[2] - lx0))
        ys.add(math.ceil(bb[3] - ly0))

    candidates: list[tuple[int, int]] = []
    for x in sorted(xs):
        for y in sorted(ys):
            if x + lx1 <= bay_w + 1e-6 and y + ly1 <= bay_h + 1e-6:
                candidates.append((int(x), int(y)))
    return candidates


def _empty_bay_entry(
    schedule_in_bay: list[tuple[int, int]],
    r_time: int,
    proc: int,
) -> int:
    """
    Earliest entry >= r_time such that the bay is completely empty for the
    window [entry, entry+proc).  Guaranteed crane-feasible (no blocks = no
    obstructions), so this always produces a valid Stage-2/3/4 placement.
    """
    entry   = int(r_time)
    changed = True
    while changed:
        changed = False
        exit_t  = entry + proc
        for a, e in schedule_in_bay:
            if _time_overlaps(entry, exit_t, a, e):
                entry   = max(entry, e)
                changed = True
    return entry


def _find_earliest_slot(
    new_blk:          Block,
    bay:              Bay,
    placed_in_bay:    list[Block],
    schedule_in_bay:  list[tuple[int, int]],
    r_time:           int,
    proc:             int,
) -> tuple[int | None, int | None]:
    """
    Earliest (entry, provisional_exit) with provisional_exit = entry + proc
    that passes Stage-2 (crane entry), Stage-3 (crane exit), Stage-4 (collision).

    Candidate entries: {r_time} | {exit times of already-placed blocks > r_time}.
    Returns (None, None) if no feasible slot exists for this (position, bay).

    Mirrors check_feasibility's stage semantics exactly:
      Stage-2 present: a_k < entry < e_k  (strict lower bound)
      Stage-3 present: a_k < exit_t < e_k (strict both ends)
      Stage-4 interior: blocks fully contained within [entry, exit_t)
    """
    candidates = sorted(
        {r_time} | {e for _, e in schedule_in_bay if e > r_time}
    )

    for entry_cand in candidates:
        entry  = max(r_time, entry_cand)
        exit_t = entry + proc

        # Stage-2: crane entry
        present_entry = [
            b for b, (a, e) in zip(placed_in_bay, schedule_in_bay)
            if a < entry < e
        ]
        if check_entry(bay, present_entry, new_blk, fast=True):
            continue

        # Stage-3: crane exit at provisional_exit
        present_exit = [new_blk] + [
            b for b, (a, e) in zip(placed_in_bay, schedule_in_bay)
            if a < exit_t < e
        ]
        if check_exit(bay, present_exit, new_blk, fast=True):
            continue

        # Stage-4 pre-check: interior blocks (a_other >= entry AND e_other <= exit_t)
        s4_ok = True
        for b_other, (a_other, e_other) in zip(placed_in_bay, schedule_in_bay):
            if a_other < entry or e_other > exit_t:
                continue   # covered by Stage-2 / Stage-3 above
            if not _time_overlaps(entry, exit_t, a_other, e_other):
                continue
            if check_collisions(bay, [new_blk, b_other]):
                s4_ok = False
                break
        if not s4_ok:
            continue

        return entry, exit_t

    return None, None


def _force_place_block(
    i:              int,
    blocks_data:    list[dict],
    bays:           list[Bay],
    bays_data:      list[dict],
    bay_schedule:   list[list[tuple[int, int]]],
    footprints:     list[list[tuple]],
) -> tuple[int, int, int, int, int, int]:
    """
    Guaranteed-feasible fallback placement: empty-bay window at min-valid position.
    Tries bays in decreasing preference order.
    Returns (bay_id, x, y, orient_idx, entry_time, provisional_exit_time).
    """
    bd     = blocks_data[i]
    r_time = bd["release_time"]
    proc   = bd["processing_time"]
    prefs  = bd["bay_preferences"]
    n_bays = len(bays)

    for j in sorted(range(n_bays), key=lambda j: prefs[j], reverse=True):
        for o in range(len(bd["shape"])):
            lx0, ly0, lx1, ly1, _ = footprints[i][o]
            px_lo = math.ceil(-lx0)
            px_hi = math.floor(bays_data[j]["width"]  - lx1)
            py_lo = math.ceil(-ly0)
            py_hi = math.floor(bays_data[j]["height"] - ly1)
            if px_lo <= px_hi and py_lo <= py_hi:
                px    = max(0, px_lo)
                py    = max(0, py_lo)
                entry = _empty_bay_entry(bay_schedule[j], r_time, proc)
                return (j, px, py, o, entry, entry + proc)

    # Absolute last resort — should be unreachable for valid instances
    logger.error("%s Block %d: _force_place_block found no valid position!", _LP, i)
    return (0, 0, 0, 0, r_time, r_time + proc)


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — spatial placement + entry timing
# ─────────────────────────────────────────────────────────────────────────────

def _place_all(
    prob_info:  dict,
    footprints: list[list[tuple]],
    t_start:    float,
    timelimit:  float,
) -> dict[int, dict]:
    """
    Single placement pass: bay selection + spatial placement + entry timing together.

    Sort order: SPT (processing_time asc), tie-break release_time asc, block_id.

    For each block, every fitting (bay, orient) combination is tried.
    The first BLF-feasible position for each (bay, orient) is scored as:
        (max(0, exit_t - due_date),          # tardiness proxy  — primary
         w3 * (s_max - prefs[j]),            # preference loss  — secondary
         w2 * workload_imbalance_after,      # balance penalty  — tertiary
         entry_time)                         # enter early      — tiebreak

    The minimum-score combination is committed.
    Fallback: _force_place_block if nothing fits.

    Time guard: if elapsed > timelimit*0.80, remaining blocks are force-placed.
    """
    blocks_data = prob_info["blocks"]
    bays_data   = prob_info["bays"]
    weights     = prob_info.get("weights", {})
    n_blocks    = len(blocks_data)
    n_bays      = len(bays_data)
    w2          = float(weights.get("w2", 1.0))
    w3          = float(weights.get("w3", 1.0))

    bays      = [Bay.from_dict(bays_data[j], j) for j in range(n_bays)]
    bay_areas = [bays_data[j]["width"] * bays_data[j]["height"] for j in range(n_bays)]
    avg_area  = sum(bay_areas) / n_bays
    u         = [avg_area / a for a in bay_areas]

    bay_placed:   list[list[Block]]           = [[] for _ in range(n_bays)]
    bay_schedule: list[list[tuple[int, int]]] = [[] for _ in range(n_bays)]
    bay_workload: list[float]                  = [0.0] * n_bays

    order    = sorted(range(n_blocks), key=lambda i: (
        blocks_data[i]["processing_time"],
        blocks_data[i]["release_time"],
        i,
    ))
    result:   dict[int, dict] = {}
    n_forced: int              = 0

    for rank, i in enumerate(order):
        # ── Time budget guard ──────────────────────────────────────────────
        if time.time() - t_start > timelimit * 0.80:
            logger.warning("%s Placement time budget at block %d/%d; force-placing %d.",
                           _LP, i, n_blocks, n_blocks - rank)
            for bid in order[rank:]:
                pl = _force_place_block(bid, blocks_data, bays, bays_data,
                                        bay_schedule, footprints)
                j_f, cx, cy, oi, entry, exit_t = pl
                bay_placed[j_f].append(Block(block_id=bid, block_data=blocks_data[bid],
                                             x=cx, y=cy, orient_idx=oi))
                bay_schedule[j_f].append((entry, exit_t))
                bay_workload[j_f] += blocks_data[bid]["workload"]
                result[bid] = {"block_id": bid, "bay_id": j_f, "orient_idx": oi,
                               "x": cx, "y": cy,
                               "entry_time": entry, "provisional_exit_time": exit_t}
                n_forced += 1
            break

        bd     = blocks_data[i]
        r_time = bd["release_time"]
        proc   = bd["processing_time"]
        prefs  = bd["bay_preferences"]
        due    = bd["due_date"]
        wl     = float(bd["workload"])
        s_max  = max(prefs)
        n_ori  = len(bd["shape"])

        best_score:     tuple | None = None
        best_placement: tuple | None = None

        # ── Score every fitting (bay, orient), take the best ──────────────
        # Sort bays by preference desc so that early-termination favours the
        # most-preferred bay when a zero-tardiness solution is found there.
        for j in sorted(range(n_bays), key=lambda jj: prefs[jj], reverse=True):
            for oi in sorted(range(n_ori), key=lambda o: footprints[i][o][4]):
                lx0, ly0, lx1, ly1, _ = footprints[i][oi]
                if lx1 - lx0 > bays_data[j]["width"]  + 1e-6: continue
                if ly1 - ly0 > bays_data[j]["height"] + 1e-6: continue

                active = [b for b, (_, e) in zip(bay_placed[j], bay_schedule[j])
                          if e > r_time]
                cands  = _candidate_positions(bays[j].width, bays[j].height,
                                              active, (lx0, ly0, lx1, ly1))
                for cx, cy in cands:
                    new_blk = Block(block_id=i, block_data=bd, x=cx, y=cy, orient_idx=oi)
                    if not bays[j].contains_block(new_blk):
                        continue
                    entry, exit_t = _find_earliest_slot(
                        new_blk, bays[j],
                        bay_placed[j], bay_schedule[j],
                        r_time, proc,
                    )
                    if entry is None:
                        continue
                    tentative_wl    = bay_workload[:]
                    tentative_wl[j] += wl
                    score = (
                        max(0, exit_t - due),
                        w3 * (s_max - prefs[j]),
                        w2 * _compute_imbalance(tentative_wl, u, n_bays),
                        entry,
                    )
                    if best_score is None or score < best_score:
                        best_score     = score
                        best_placement = (j, cx, cy, oi, entry, exit_t)
                    break  # BLF: first feasible position per (bay, orient)

            # Early termination: zero tardiness in the best-preference bay —
            # no other bay can improve the primary or secondary score component.
            if (best_score is not None
                    and best_score[0] == 0
                    and best_score[1] == 0):
                break

        # ── Force-place fallback ───────────────────────────────────────────
        if best_placement is None:
            logger.warning("%s Block %d: no BLF placement found, force-placing", _LP, i)
            best_placement = _force_place_block(i, blocks_data, bays, bays_data,
                                                bay_schedule, footprints)
            n_forced += 1

        # ── Commit ────────────────────────────────────────────────────────
        j_f, cx, cy, oi, entry, exit_t = best_placement
        bay_placed[j_f].append(Block(block_id=i, block_data=bd, x=cx, y=cy, orient_idx=oi))
        bay_schedule[j_f].append((entry, exit_t))
        bay_workload[j_f] += wl

        result[i] = {
            "block_id":              i,
            "bay_id":                j_f,
            "orient_idx":            oi,
            "x":                     int(round(cx)),
            "y":                     int(round(cy)),
            "entry_time":            int(round(entry)),
            "provisional_exit_time": int(round(exit_t)),
        }

    if n_forced:
        logger.info("%s Placement: %d block(s) force-placed", _LP, n_forced)
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Phase 3 — exit-timing refinement (Option A continuation)
# ─────────────────────────────────────────────────────────────────────────────

def _phase3_finalize_exit(
    prob_info:     dict,
    phase2_result: dict[int, dict],
    t_start:       float,
    timelimit:     float,
) -> dict[int, dict]:
    """
    Refine provisional_exit_time -> exit_time for every block.

    Single sequential pass in EDD order.  For each block i:
      naive_floor = entry_time + processing_time  (minimum possible exit)
      Candidate exit times: {naive_floor} ∪ {entry/exit times of every other
          block in the same bay that are >= naive_floor}.
      Present-at-exit set uses FINALIZED exit times for already-processed
      blocks and provisional_exit_time for not-yet-processed ones.
      First candidate where check_exit passes -> exit_time.
      Fallback (if no candidate passes): bay-empty time = max exit time of
          all co-bay blocks -> at that point the bay is empty and check_exit
          trivially succeeds (no obstructions).

    Time guard: if elapsed > timelimit*0.92, keep provisional_exit for the
    remaining blocks (Phase 2 already ensured those exits were stage-3 feasible
    given the partial block set, so they are a safe fallback).

    Entry/exit coupling recap (Option A):
      Phase 2's provisional_exit = entry + proc (the absolute minimum).
      Phase 3 may push this later if the crane path is blocked; it never
      brings it earlier (it's already at the floor).
    """
    blocks_data = prob_info["blocks"]
    bays_data   = prob_info["bays"]
    n_blocks    = len(blocks_data)
    n_bays      = len(bays_data)

    bays = [Bay.from_dict(bays_data[j], j) for j in range(n_bays)]

    # Pre-build Block objects once (reused in every check_exit call)
    block_objs: dict[int, Block] = {
        bid: Block(
            block_id=bid,
            block_data=blocks_data[bid],
            x=r["x"],
            y=r["y"],
            orient_idx=r["orient_idx"],
        )
        for bid, r in phase2_result.items()
    }

    # Per-bay block lists for fast iteration (avoid scanning all n_blocks each time)
    bay_block_ids: list[list[int]] = [[] for _ in range(n_bays)]
    for bid, r in phase2_result.items():
        bay_block_ids[r["bay_id"]].append(bid)

    # exit_times: starts at provisional values; updated as each block is finalized
    exit_times: dict[int, int] = {
        bid: r["provisional_exit_time"] for bid, r in phase2_result.items()
    }

    order = sorted(range(n_blocks), key=lambda i: (blocks_data[i]["due_date"], i))
    n_fallback = 0
    finalized_bids: set[int] = set()   # blocks whose exit_time is already committed

    for i in order:
        # ── Time guard ─────────────────────────────────────────────────────
        if time.time() - t_start > timelimit * 0.92:
            logger.warning(
                "%s Phase 3 time budget reached at block %d; "
                "keeping provisional exits for remaining blocks.", _LP, i
            )
            break

        r          = phase2_result[i]
        bay_id     = r["bay_id"]
        entry_time = r["entry_time"]
        proc       = blocks_data[i]["processing_time"]
        naive_floor = entry_time + proc

        target_blk = block_objs[i]
        bay        = bays[bay_id]

        # Other blocks sharing the same bay
        co_bay = [bid for bid in bay_block_ids[bay_id] if bid != i]

        # Candidate exit times: floor + all boundary times of co-bay blocks.
        # Also add (finalized_exit + 1) for any finalized co-bay block: if that
        # block's exit coincides with our naive_floor, Stage-5 simulation would
        # see it still in bay_present.  +1 jumps past the collision.
        cand_set: set[int] = {naive_floor}
        for bid2 in co_bay:
            r2 = phase2_result[bid2]
            if r2["entry_time"] >= naive_floor:
                cand_set.add(r2["entry_time"])
            et2 = exit_times[bid2]   # finalized or provisional
            if et2 >= naive_floor:
                cand_set.add(et2)
            if bid2 in finalized_bids and et2 >= naive_floor:
                cand_set.add(et2 + 1)

        # Guaranteed-feasible ceiling: after every co-bay block has exited,
        # present_at_exit is empty and check_exit trivially passes.
        all_gone = max((exit_times[bid2] for bid2 in co_bay), default=naive_floor)
        cand_set.add(max(naive_floor, all_gone))

        found_exit: int | None = None
        for t_cand in sorted(cand_set):
            if t_cand < naive_floor:
                continue

            # Build present_at_exit using two criteria:
            #   (a) Standard Stage-3 presence: a_k < t_cand < e_k  (strict)
            #   (b) Same-time finalized exits: block bid2 whose exit_time == t_cand
            #       and was finalized earlier in this EDD pass.  In Stage-5's
            #       sequential simulation those blocks are still in bay_present
            #       when we are processed, so we must treat them as present here
            #       to avoid creating EXIT ordering conflicts.
            present: list[Block] = [target_blk]
            for bid2 in co_bay:
                r2  = phase2_result[bid2]
                et2 = exit_times[bid2]
                if r2["entry_time"] < t_cand < et2:
                    present.append(block_objs[bid2])
                elif (et2 == t_cand
                      and bid2 in finalized_bids
                      and r2["entry_time"] < t_cand):
                    # Same-time finalized block: include to prevent Stage-5 conflict
                    present.append(block_objs[bid2])

            if not check_exit(bay, present, target_blk, fast=True):
                found_exit = t_cand
                break

        if found_exit is None:
            # Should be unreachable (all_gone candidate always works), but guard anyway
            logger.warning("%s Block %d: Phase 3 all candidates failed, using all_gone", _LP, i)
            found_exit = max(naive_floor, all_gone)
            n_fallback += 1

        exit_times[i] = found_exit
        finalized_bids.add(i)

    if n_fallback:
        logger.info("%s Phase 3: %d block(s) used all-gone fallback", _LP, n_fallback)

    # Assemble final result
    final: dict[int, dict] = {}
    for bid, r in phase2_result.items():
        final[bid] = {
            "block_id":   bid,
            "bay_id":     r["bay_id"],
            "orient_idx": r["orient_idx"],
            "x":          r["x"],
            "y":          r["y"],
            "entry_time": r["entry_time"],
            "exit_time":  exit_times[bid],
        }
    return final


# ─────────────────────────────────────────────────────────────────────────────
# Serialization
# ─────────────────────────────────────────────────────────────────────────────

def _topo_sort_bay_entries(
    bay_entries: list[dict],
    bay:         Bay,
    blocks_data: list[dict],
    pre_present: list[Block],
) -> list[dict]:
    """
    Sort ENTRY ops for one bay at one time step so that each block's crane
    descent is feasible given the blocks that have already entered in this
    same time step.

    Dependency rule: if block B's presence would obstruct block A's crane
    descent (check_entry with B in present fails for A), then A must enter
    before B  (A → B directed edge).

    pre_present: Block objects already in the bay from earlier time steps
    (not subject to reordering; used to detect true Stage-5 entry failures
    vs. ordering conflicts).

    Uses Kahn's topological sort.  Falls back to block_id order on cycle.
    """
    from collections import deque

    n = len(bay_entries)
    if n == 1:
        return bay_entries

    blks = [
        Block(block_id=op["block_id"],
              block_data=blocks_data[op["block_id"]],
              x=op["x"], y=op["y"], orient_idx=op["orient_idx"])
        for op in bay_entries
    ]

    # Build dependency graph: if B obstructs A's entry, A must enter before B.
    # adj[i] = list of j where i must precede j (i → j)
    adj   = [[] for _ in range(n)]
    indeg = [0]  * n

    for a_idx in range(n):
        for b_idx in range(n):
            if a_idx == b_idx:
                continue
            # Does block b obstruct block a's entry?
            present_test = pre_present + [blks[b_idx]]
            if check_entry(bay, present_test, blks[a_idx], fast=True):
                # b obstructs a → a must enter before b
                adj[a_idx].append(b_idx)
                indeg[b_idx] += 1

    # Kahn's algorithm
    queue  = deque(k for k in range(n) if indeg[k] == 0)
    result = []
    while queue:
        k = queue.popleft()
        result.append(k)
        for m in adj[k]:
            indeg[m] -= 1
            if indeg[m] == 0:
                queue.append(m)

    if len(result) < n:
        # Cycle: fall back to block_id order
        return sorted(bay_entries, key=lambda op: op["block_id"])

    return [bay_entries[k] for k in result]


def _topo_sort_bay_exits(
    bay_exits:   list[dict],
    bay:         Bay,
    blocks_data: list[dict],
) -> list[dict]:
    """
    Sort EXIT ops for one bay at one time step so that each block's crane exit
    is feasible given the blocks still present ahead of it in the sequence.

    Dependency rule: if block B's body obstructs block A's crane ascent
    (check_exit with B present returns a violation), then B must exit before A
    (B → A directed edge).

    Uses Kahn's topological sort.  Falls back to LIFO (latest entry first) if
    a dependency cycle is detected (two blocks mutually obstruct each other —
    infeasible in theory but handled gracefully here).
    """
    from collections import deque

    n = len(bay_exits)
    if n == 1:
        return bay_exits

    blks = [
        Block(block_id=op["block_id"],
              block_data=blocks_data[op["block_id"]],
              x=op["x"], y=op["y"], orient_idx=op["orient_idx"])
        for op in bay_exits
    ]

    # Build dependency graph: adj[j] = list of i where j must precede i
    adj   = [[] for _ in range(n)]
    indeg = [0]  * n

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            # Does block j obstruct block i's crane exit?
            if check_exit(bay, [blks[i], blks[j]], blks[i], fast=True):
                adj[j].append(i)   # j → i  (j exits first)
                indeg[i] += 1

    # Kahn's algorithm
    queue  = deque(k for k in range(n) if indeg[k] == 0)
    result = []
    while queue:
        k = queue.popleft()
        result.append(k)
        for m in adj[k]:
            indeg[m] -= 1
            if indeg[m] == 0:
                queue.append(m)

    if len(result) < n:
        # Cycle: fall back to LIFO (latest entry exits first, higher id as tie-break)
        return sorted(bay_exits,
                      key=lambda op: (-op["_entry_time"], -op["block_id"]))

    return [bay_exits[k] for k in result]


def _build_operations(
    assignments: list[dict],
    prob_info:   dict | None = None,
) -> dict[str, list[dict]]:
    """
    Convert flat assignment list to the operations dict required by the evaluator.

    EXIT ops always precede ENTRY ops at the same time point.

    When prob_info is provided:
      - Same-time EXIT ops in the same bay are topologically sorted so that
        each block's crane ascent is not obstructed by a block that hasn't
        yet exited (Stage-5 EXIT ordering safety).
      - Same-time ENTRY ops in the same bay are topologically sorted so that
        if block B's presence would obstruct block A's crane descent, A enters
        before B (Stage-5 ENTRY ordering safety).
    Without prob_info: LIFO exit ordering + block_id entry ordering (fallback).
    """
    exit_by_time:  dict[int, list[dict]] = {}
    entry_by_time: dict[int, list[dict]] = {}

    for a in assignments:
        t_en = int(a["entry_time"])
        t_ex = int(a["exit_time"])
        bid  = a["block_id"]
        bay  = a["bay_id"]
        exit_by_time.setdefault(t_ex, []).append({
            "block_id": bid, "bay_id": bay,
            "x": a["x"], "y": a["y"], "orient_idx": a["orient_idx"],
            "_entry_time": t_en,
        })
        entry_by_time.setdefault(t_en, []).append({
            "block_id": bid, "bay_id": bay,
            "x": a["x"], "y": a["y"], "orient_idx": a["orient_idx"],
        })

    bays_objs:   list[Bay] | None      = None
    blocks_data: list[dict] | None     = None
    asgn_map:    dict[int, dict] | None = None
    if prob_info is not None:
        blocks_data = prob_info["blocks"]
        bays_objs   = [Bay.from_dict(prob_info["bays"][j], j)
                       for j in range(len(prob_info["bays"]))]
        asgn_map    = {a["block_id"]: a for a in assignments}

    # bay_present: tracks which block IDs are in each bay between time steps
    bay_present: dict[int, set[int]] = (
        {j: set() for j in range(len(prob_info["bays"]))}
        if prob_info is not None else {}
    )

    all_times  = sorted(set(exit_by_time) | set(entry_by_time))
    operations: dict[str, list[dict]] = {}

    for t in all_times:
        exits_t   = exit_by_time.get(t, [])
        entries_t = entry_by_time.get(t, [])

        # ── Sort EXIT ops ──────────────────────────────────────────────────
        if bays_objs is not None and len(exits_t) >= 2:
            by_bay: dict[int, list[dict]] = {}
            for op in exits_t:
                by_bay.setdefault(op["bay_id"], []).append(op)
            sorted_exits: list[dict] = []
            for bay_id, grp in by_bay.items():
                sorted_exits.extend(
                    _topo_sort_bay_exits(grp, bays_objs[bay_id], blocks_data)
                    if len(grp) >= 2 else grp
                )
        else:
            sorted_exits = sorted(exits_t,
                                  key=lambda op: (-op["_entry_time"], -op["block_id"]))

        # Apply EXIT ops to bay_present tracking
        for op in sorted_exits:
            bay_present.get(op["bay_id"], set()).discard(op["block_id"])

        # ── Sort ENTRY ops ─────────────────────────────────────────────────
        if bays_objs is not None and len(entries_t) >= 2:
            by_bay_e: dict[int, list[dict]] = {}
            for op in entries_t:
                by_bay_e.setdefault(op["bay_id"], []).append(op)
            sorted_entries: list[dict] = []
            for bay_id, grp in by_bay_e.items():
                if len(grp) < 2:
                    sorted_entries.extend(grp)
                else:
                    pre_blks = [
                        Block(block_id=bid,
                              block_data=blocks_data[bid],
                              x=asgn_map[bid]["x"],
                              y=asgn_map[bid]["y"],
                              orient_idx=asgn_map[bid]["orient_idx"])
                        for bid in bay_present.get(bay_id, set())
                        if bid in asgn_map
                    ]
                    sorted_entries.extend(
                        _topo_sort_bay_entries(
                            grp, bays_objs[bay_id], blocks_data, pre_blks
                        )
                    )
        else:
            sorted_entries = sorted(entries_t, key=lambda op: op["block_id"])

        # Apply ENTRY ops to bay_present tracking
        for op in sorted_entries:
            bay_present.get(op["bay_id"], set()).add(op["block_id"])

        # ── Assemble day_ops ────────────────────────────────────────────────
        day_ops: list[dict] = []
        for op in sorted_exits:
            day_ops.append({"type": "EXIT",
                            "block_id": op["block_id"], "bay_id": op["bay_id"]})
        for op in sorted_entries:
            day_ops.append({"type": "ENTRY",
                            "block_id": op["block_id"], "bay_id": op["bay_id"],
                            "x": int(op["x"]), "y": int(op["y"]),
                            "orient_idx": op["orient_idx"]})
        operations[str(t)] = day_ops

    return operations


# ─────────────────────────────────────────────────────────────────────────────
# Repair pass — fix Stage-2/3/4 violations created by Phase 3's exit shifts
# ─────────────────────────────────────────────────────────────────────────────

def _parse_violating_blocks(violations: list[str]) -> list[int]:
    """
    Extract IDs of the *violating* (not the obstructing) blocks from
    check_feasibility violation strings.

    Stage-2: "Stage2: t=T: block Y entry obstructed by block Z" → Y
    Stage-3: "Stage3: t=T: block Y exit  obstructed by block Z" → Y
    Stage-4/5: first block ID found.
    """
    import re
    seen: set[int] = set()
    result: list[int] = []
    for v in violations:
        m = re.search(r"block\s+(\d+)\s+(?:entry|exit)\s+obstructed", v, re.IGNORECASE)
        if m:
            bid = int(m.group(1))
        else:
            nums = re.findall(r"block\s+(\d+)", v, re.IGNORECASE)
            if not nums:
                continue
            bid = int(nums[0])
        if bid not in seen:
            seen.add(bid)
            result.append(bid)
    return result


def _repair_violations(
    prob_info:        dict,
    final_assignment: dict[int, dict],
    footprints:       list[list[tuple]],
    t_start:          float,
    timelimit:        float,
    max_passes:       int = 6,
) -> dict[int, dict]:
    """
    Iteratively repair Stage-2/3/4 violations that Phase 3 may have introduced
    by delaying a block's exit_time past another block's entry_time.

    Each pass:
      1. Run check_feasibility on the current assignments.
      2. If feasible → done.
      3. Otherwise parse violating block IDs, remove them, rebuild full bay
         state from the surviving assignments (using their exit_time fields),
         and re-place each violating block with _find_earliest_slot (full
         state visibility) → exit_time = entry + proc (Stage-3-minimal).
      4. Fallback to _force_place_block if BLF finds nothing.

    The assignments dict uses "exit_time" (not "provisional_exit_time").
    """
    blocks_data = prob_info["blocks"]
    bays_data   = prob_info["bays"]
    n_bays      = len(bays_data)
    bays        = [Bay.from_dict(bays_data[j], j) for j in range(n_bays)]

    assignments = dict(final_assignment)

    def _to_sol(asgns: dict) -> dict:
        return {"operations": _build_operations(list(asgns.values()), prob_info)}

    for pass_idx in range(max_passes):
        if time.time() - t_start > timelimit * 0.95:
            logger.warning("%s Repair: time budget at pass %d, stopping", _LP, pass_idx + 1)
            break

        res = check_feasibility(prob_info, _to_sol(assignments))
        if res["feasible"]:
            logger.info("%s Repair done after %d pass(es)", _LP, pass_idx)
            return assignments

        to_repair = _parse_violating_blocks(res["violations"])
        if not to_repair:
            logger.error("%s Repair: no block IDs parsed from violations, aborting")
            break

        logger.info("%s Repair pass %d: stage=%d  violating=%s",
                    _LP, pass_idx + 1, res["stage"], to_repair)

        # Remove violating blocks from the working set
        for bid in to_repair:
            assignments.pop(bid, None)

        # Rebuild full bay state from the surviving assignments
        bay_placed:   list[list[Block]]           = [[] for _ in range(n_bays)]
        bay_schedule: list[list[tuple[int, int]]] = [[] for _ in range(n_bays)]
        for a in assignments.values():
            j   = a["bay_id"]
            blk = Block(block_id=a["block_id"],
                        block_data=blocks_data[a["block_id"]],
                        x=a["x"], y=a["y"], orient_idx=a["orient_idx"])
            bay_placed[j].append(blk)
            bay_schedule[j].append((a["entry_time"], a["exit_time"]))

        # Re-place each violating block in EDD order.
        # If time budget is hit mid-loop, force-place the remaining blocks so
        # they are never left unassigned (Stage-1 failure).
        for bid in sorted(to_repair,
                          key=lambda b: (blocks_data[b]["due_date"], b)):
            if time.time() - t_start > timelimit * 0.95:
                pl = _force_place_block(bid, blocks_data, bays, bays_data,
                                        bay_schedule, footprints)
                j_f, cx, cy, oi, entry, exit_t = pl
                bay_placed[j_f].append(Block(block_id=bid, block_data=blocks_data[bid],
                                             x=cx, y=cy, orient_idx=oi))
                bay_schedule[j_f].append((entry, exit_t))
                assignments[bid] = {"block_id": bid, "bay_id": j_f, "orient_idx": oi,
                                    "x": int(round(cx)), "y": int(round(cy)),
                                    "entry_time": int(round(entry)),
                                    "exit_time":  int(round(exit_t))}
                continue
            bd       = blocks_data[bid]
            r_time   = bd["release_time"]
            proc     = bd["processing_time"]
            prefs    = bd["bay_preferences"]
            n_orient = len(bd["shape"])

            # (bay, orient) candidate list — preference-ordered, fitting first
            seen_bo: set[tuple[int, int]] = set()
            bo_list: list[tuple[int, int]] = []
            for j in sorted(range(n_bays), key=lambda jj: prefs[jj], reverse=True):
                for o in sorted(range(n_orient),
                                key=lambda oo: footprints[bid][oo][4]):
                    lx0, ly0, lx1, ly1, _ = footprints[bid][o]
                    fits = ((lx1 - lx0) <= bays_data[j]["width"]  + 1e-6 and
                            (ly1 - ly0) <= bays_data[j]["height"] + 1e-6)
                    if fits and (j, o) not in seen_bo:
                        seen_bo.add((j, o))
                        bo_list.append((j, o))

            placement: tuple | None = None
            for bay_id, oi in bo_list:
                bay    = bays[bay_id]
                lx0, ly0, lx1, ly1, _ = footprints[bid][oi]
                blk_bb = (lx0, ly0, lx1, ly1)
                active = [b for b, (_, e) in
                          zip(bay_placed[bay_id], bay_schedule[bay_id])
                          if e > r_time]
                cands  = _candidate_positions(bay.width, bay.height, active, blk_bb)
                for cx, cy in cands:
                    new_blk = Block(block_id=bid, block_data=bd,
                                   x=cx, y=cy, orient_idx=oi)
                    if not bay.contains_block(new_blk):
                        continue
                    entry, exit_t = _find_earliest_slot(
                        new_blk, bay,
                        bay_placed[bay_id], bay_schedule[bay_id],
                        r_time, proc)
                    if entry is not None:
                        placement = (bay_id, cx, cy, oi, entry, exit_t)
                        break
                if placement is not None:
                    break

            if placement is None:
                logger.warning("%s Repair: block %d no BLF slot → force-place", _LP, bid)
                pl        = _force_place_block(bid, blocks_data, bays, bays_data,
                                               bay_schedule, footprints)
                placement = pl

            bay_id, cx, cy, oi, entry, exit_t = placement
            final_blk = Block(block_id=bid, block_data=blocks_data[bid],
                              x=int(round(cx)), y=int(round(cy)),
                              orient_idx=oi)
            bay_placed[bay_id].append(final_blk)
            bay_schedule[bay_id].append((entry, exit_t))

            assignments[bid] = {
                "block_id":   bid,
                "bay_id":     bay_id,
                "orient_idx": oi,
                "x":          int(round(cx)),
                "y":          int(round(cy)),
                "entry_time": int(round(entry)),
                "exit_time":  int(round(exit_t)),
            }

    # Final feasibility check (informational)
    res = check_feasibility(prob_info, _to_sol(assignments))
    if not res["feasible"]:
        logger.error("%s Repair exhausted max_passes=%d, still stage=%d",
                     _LP, max_passes, res["stage"])
    return assignments


# ─────────────────────────────────────────────────────────────────────────────
# Main entry point
# ─────────────────────────────────────────────────────────────────────────────

def algorithm(prob_info: dict, timelimit: float = 60) -> dict:
    """
    OGC 2026 solver — two-phase pipeline.

    Phase 1 (_place_all, SPT order): bay + orient + position + entry time
    Phase 2 (_phase3_finalize_exit, EDD order): exit-time refinement
    Repair (_repair_violations): fix any Stage-2/3 violations from Phase 2

    Time budget: placement stops at 80%, exit refinement stops at 92%.
    """
    t_start = time.time()

    blocks_data = prob_info["blocks"]
    n_blocks    = len(blocks_data)
    n_bays      = len(prob_info["bays"])

    logger.info("%s Instance: %s  blocks=%d  bays=%d  timelimit=%.1fs",
                _LP, prob_info.get("name", "?"), n_blocks, n_bays, timelimit)

    footprints = _precompute_footprints(blocks_data)

    placement_result = _place_all(prob_info, footprints, t_start, timelimit)
    logger.info("%s Phase 1 done: %.2fs  (%d blocks)",
                _LP, time.time() - t_start, len(placement_result))

    exit_result = _phase3_finalize_exit(prob_info, placement_result, t_start, timelimit)
    logger.info("%s Phase 2 done: %.2fs", _LP, time.time() - t_start)

    final_assignment = _repair_violations(prob_info, exit_result, footprints, t_start, timelimit)
    logger.info("%s Repair done: %.2fs", _LP, time.time() - t_start)

    return {"operations": _build_operations(list(final_assignment.values()), prob_info)}
