"""
test_phase1_synthetic.py — Edge-case stress tests for assign_bay_and_orientation.

All test instances are constructed in-memory (no file I/O) to verify that the
Phase 1 implementation is robust to structural variation that may not appear in
any single training file.

Edge cases covered
------------------
  T1: n_bays=1, n_blocks=1, 1 orientation, 1 layer  (minimal instance)
  T2: block that is larger than every bay in every orientation  (forced fallback)
  T3: multiple orientations where only one fits; must choose smallest-area fit
  T4: n_blocks=1 with n_bays=3  (single block, multi-bay preference check)
  T5: n_bays=2, equal bays, equal blocks, high w2  (workload balance check)
  T6: n_bays=1, n_blocks=5  (single-bay: no pairs, obj2 always 0)
  T7: block with 2 orientations, both fit, different areas  (orient tie-break)
"""

from __future__ import annotations

import sys
import os
import traceback
import io

# Force UTF-8 output so em-dashes in test names render correctly on Windows
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
elif sys.stdout.encoding and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

# Ensure utils is importable from the baseline/ directory
sys.path.insert(0, os.path.dirname(__file__))

from phase1_bay_orientation import assign_bay_and_orientation  # type: ignore


# ---------------------------------------------------------------------------
# Helpers for building minimal prob_info dicts
# ---------------------------------------------------------------------------

def _bay(w: int, h: int) -> dict:
    return {"width": w, "height": h}


def _block(
    due: int = 10,
    workload: int = 100,
    prefs: list[int] | None = None,
    shapes: list[list[list[list[float]]]] | None = None,
    release: int = 0,
    proc: int = 5,
) -> dict:
    """
    Build a minimal block dict.

    shapes: list of per-orientation layer lists.
            Each orientation is [[layer0_verts], [layer1_verts], ...].
            Each layer is a list of [x, y] pairs.
    prefs: bay_preferences list (must sum to 100 if not None).
           Defaults to uniform distribution over n_bays inferred from len(prefs).
    """
    if shapes is None:
        # Default: single orientation, single layer, 10x10 square
        shapes = [
            [[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]]
        ]
    if prefs is None:
        prefs = [100]   # placeholder; caller should supply correct length

    oriented_shapes = [
        {"orientation": o, "layers": layer_list}
        for o, layer_list in enumerate(shapes)
    ]
    return {
        "release_time":    release,
        "due_date":        due,
        "processing_time": proc,
        "workload":        workload,
        "bay_preferences": prefs,
        "shape":           oriented_shapes,
    }


def _prob(bays: list[dict], blocks: list[dict],
          w1: float = 100.0, w2: float = 5.0, w3: float = 10.0) -> dict:
    return {
        "name":    "synthetic",
        "bays":    bays,
        "blocks":  blocks,
        "weights": {"w1": w1, "w2": w2, "w3": w3},
    }


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

_PASS = 0
_FAIL = 0


def _run(name: str, prob_info: dict, checks: list[tuple]) -> None:
    """
    Run assign_bay_and_orientation, then apply assertion checks.

    checks: list of (description, callable) where callable(result) -> bool.
    """
    global _PASS, _FAIL
    sep = "-" * 60
    print(f"\n{sep}")
    print(f"TEST: {name}")
    n_bays   = len(prob_info["bays"])
    n_blocks = len(prob_info["blocks"])
    print(f"  n_bays={n_bays}, n_blocks={n_blocks}, "
          f"weights={prob_info['weights']}")
    try:
        result = assign_bay_and_orientation(prob_info)

        # --- Structural validity (always checked) ---
        assert len(result) == n_blocks, (
            f"Expected {n_blocks} assignments, got {len(result)}"
        )
        for bid, asgn in result.items():
            assert "bay_id" in asgn and "orient_idx" in asgn, (
                f"block {bid} missing required keys: {asgn}"
            )
            assert 0 <= asgn["bay_id"] < n_bays, (
                f"block {bid}: bay_id={asgn['bay_id']} out of [0, {n_bays})"
            )
            n_orient = len(prob_info["blocks"][bid]["shape"])
            assert 0 <= asgn["orient_idx"] < n_orient, (
                f"block {bid}: orient_idx={asgn['orient_idx']} out of [0, {n_orient})"
            )

        # --- Custom checks ---
        all_ok = True
        for desc, check_fn in checks:
            ok = check_fn(result)
            status = "OK  " if ok else "FAIL"
            print(f"  [{status}] {desc}")
            if not ok:
                all_ok = False

        if all_ok:
            _PASS += 1
            print(f"  => PASS")
        else:
            _FAIL += 1
            print(f"  => FAIL (some checks did not pass)")

        # Print assignment summary
        for bid in sorted(result):
            asgn = result[bid]
            prefs = prob_info["blocks"][bid]["bay_preferences"]
            print(f"     block{bid}: bay={asgn['bay_id']} "
                  f"orient={asgn['orient_idx']} "
                  f"pref={prefs[asgn['bay_id']]}")

    except Exception as exc:
        _FAIL += 1
        print(f"  => FAIL (exception): {exc}")
        traceback.print_exc()


# ---------------------------------------------------------------------------
# Individual tests
# ---------------------------------------------------------------------------

def test_minimal():
    """T1: absolute minimum — 1 bay, 1 block, 1 orientation, 1 layer."""
    prob = _prob(
        bays=[_bay(50, 50)],
        blocks=[_block(prefs=[100])],
    )
    _run(
        "T1: n_bays=1, n_blocks=1, 1 orientation, 1 layer",
        prob,
        [
            ("block 0 assigned to bay 0",
             lambda r: r[0]["bay_id"] == 0),
            ("block 0 orient_idx == 0",
             lambda r: r[0]["orient_idx"] == 0),
        ],
    )


def test_oversized_fallback():
    """T2: block is larger than every bay in every orientation — fallback path."""
    prob = _prob(
        bays=[_bay(5, 5), _bay(8, 8)],
        blocks=[_block(
            prefs=[50, 50],
            shapes=[
                # orientation 0: 100×100 — too big for both bays
                [[[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]],
            ],
        )],
    )
    _run(
        "T2: oversized block - all bays too small, must use fallback",
        prob,
        [
            ("must return exactly 1 assignment",
             lambda r: len(r) == 1),
            ("bay_id is valid (0 or 1)",
             lambda r: r[0]["bay_id"] in (0, 1)),
            ("orient_idx is 0 (only orientation available)",
             lambda r: r[0]["orient_idx"] == 0),
        ],
    )


def test_orientation_selection():
    """T3: 4 orientations, only 2 fit; must choose smallest-area one."""
    # Bay: 15×10
    # orient 0: 20×5  -> bw=20 > 15: DOES NOT FIT
    # orient 1: 5×20  -> bh=20 > 10: DOES NOT FIT
    # orient 2: 10×8  -> fits, area=80  (SMALLER AREA → should be chosen)
    # orient 3: 12×6  -> fits, area=72  (SMALLER AREA — this one should win)
    # Wait, 72 < 80, so orient 3 wins.
    prob = _prob(
        bays=[_bay(15, 10)],
        blocks=[_block(
            prefs=[100],
            shapes=[
                [[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [0.0, 5.0]]],   # orient 0: 20×5, doesn't fit
                [[[0.0, 0.0], [5.0, 0.0], [5.0, 20.0], [0.0, 20.0]]],   # orient 1: 5×20, doesn't fit
                [[[0.0, 0.0], [10.0, 0.0], [10.0, 8.0], [0.0, 8.0]]],   # orient 2: 10×8, area=80
                [[[0.0, 0.0], [12.0, 0.0], [12.0, 6.0], [0.0, 6.0]]],   # orient 3: 12×6, area=72
            ],
        )],
    )
    _run(
        "T3: 4 orientations, 2 fit; must pick smallest-area (orient 3, area=72)",
        prob,
        [
            ("orient_idx == 3 (area=72, smallest among fitting)",
             lambda r: r[0]["orient_idx"] == 3),
        ],
    )


def test_single_block_multi_bay():
    """T4: n_blocks=1, n_bays=3, preference clearly points to bay 1."""
    prob = _prob(
        bays=[_bay(50, 50), _bay(50, 50), _bay(50, 50)],
        blocks=[_block(
            prefs=[10, 80, 10],   # bay 1 is strongly preferred
            shapes=[
                [[[0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]]],
            ],
        )],
        w2=1.0, w3=50.0,   # w3 >> w2 so preference dominates
    )
    _run(
        "T4: 1 block, 3 equal bays, pref=[10,80,10], w3>>w2 => bay 1",
        prob,
        [
            ("block 0 assigned to bay 1 (preference 80)",
             lambda r: r[0]["bay_id"] == 1),
        ],
    )


def test_workload_balance():
    """T5: 2 equal bays, 4 equal-workload blocks, equal prefs, high w2.
    Expect roughly balanced assignment (2 blocks per bay)."""
    shape = [[[[0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]]]]
    blocks = [_block(prefs=[50, 50], workload=100, shapes=shape) for _ in range(4)]
    prob = _prob(
        bays=[_bay(50, 50), _bay(50, 50)],
        blocks=blocks,
        w1=100.0, w2=200.0, w3=1.0,   # very high w2 forces balance
    )

    def _balanced(r: dict) -> bool:
        counts = [0, 0]
        for asgn in r.values():
            counts[asgn["bay_id"]] += 1
        return counts[0] == 2 and counts[1] == 2

    _run(
        "T5: 4 equal blocks, 2 equal bays, high w2 => balanced 2/2",
        prob,
        [("assignment is balanced: 2 blocks per bay", _balanced)],
    )


def test_single_bay_multi_block():
    """T6: n_bays=1, n_blocks=5 — no imbalance computation needed."""
    shape = [[[[0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]]]]
    blocks = [_block(prefs=[100], shapes=shape, due=i * 2) for i in range(5)]
    prob = _prob(
        bays=[_bay(100, 100)],
        blocks=blocks,
    )
    _run(
        "T6: n_bays=1, n_blocks=5 — all must go to bay 0",
        prob,
        [
            ("all blocks assigned to bay 0",
             lambda r: all(a["bay_id"] == 0 for a in r.values())),
        ],
    )


def test_orient_area_tiebreak():
    """T7: 2 orientations, both fit, same due_date.
    orient 0 has larger bbox area; orient 1 has smaller.  Must pick orient 1."""
    # Bay: 20×20
    # orient 0: 15×15, area=225
    # orient 1: 10×10, area=100  <- should win
    prob = _prob(
        bays=[_bay(20, 20)],
        blocks=[_block(
            prefs=[100],
            shapes=[
                [[[0.0, 0.0], [15.0, 0.0], [15.0, 15.0], [0.0, 15.0]]],
                [[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]],
            ],
        )],
    )
    _run(
        "T7: 2 orientations both fit; orient 1 has smaller area => must choose orient 1",
        prob,
        [
            ("orient_idx == 1 (area=100 < 225)",
             lambda r: r[0]["orient_idx"] == 1),
        ],
    )


def test_preference_vs_balance_tradeoff():
    """T8: 2 bays, 2 blocks. Block 0 strongly prefers bay 0. Block 1 strongly
    prefers bay 1. With w3 >> w2, each should go to its preferred bay."""
    shape = [[[[0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]]]]
    prob = _prob(
        bays=[_bay(50, 50), _bay(50, 50)],
        blocks=[
            _block(prefs=[90, 10], shapes=shape, workload=100, due=5),
            _block(prefs=[10, 90], shapes=shape, workload=100, due=5),
        ],
        w1=100.0, w2=1.0, w3=100.0,   # w3 >> w2
    )

    def _pref_respected(r: dict) -> bool:
        return r[0]["bay_id"] == 0 and r[1]["bay_id"] == 1

    _run(
        "T8: strong per-block preference, w3>>w2 => each block in preferred bay",
        prob,
        [("block0→bay0, block1→bay1", _pref_respected)],
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 60)
    print("Phase 1 Synthetic Edge-Case Tests")
    print("=" * 60)

    test_minimal()
    test_oversized_fallback()
    test_orientation_selection()
    test_single_block_multi_bay()
    test_workload_balance()
    test_single_bay_multi_block()
    test_orient_area_tiebreak()
    test_preference_vs_balance_tradeoff()

    print("\n" + "=" * 60)
    print(f"Results: {_PASS} passed, {_FAIL} failed  (total {_PASS + _FAIL})")
    print("=" * 60)
    sys.exit(0 if _FAIL == 0 else 1)
