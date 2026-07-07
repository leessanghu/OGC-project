"""
myalgorithm4.py -- baseline_greedy.py + due-date shielding placement rule

목표: Z1(tardiness)을 직접 겨냥한 개선. EDD greedy 자체는 그대로 두되,
"이미 배치된, due_date가 더 이르거나 같은 블록의 크레인 출차 경로를 막는
위치는 후보에서 배제"하는 규칙(shielding)을 추가한다.

동기: 관찰된 Z1 폭등의 실제 원인은 "처리 순서"가 아니라 "배치 위치"였다 --
나중에 들어온 블록이 먼저 나가야 할 블록의 출차 경로를 막아서 stage-3
위반이 나고, repair 단계에서 강제배치(_force_place)로 크게 밀리는 패턴.
Shielding은 이 실패 모드를 애초에 후보에서 제거해 원천 차단한다.

구현 (baseline_greedy.py 대비 추가된 부분만):
  _shielding_violated(new_blk, bay, active_in_bay, due_new, blocks_data)
    -- due_date <= due_new 인 활성 블록의 미래 출차를 new_blk가 막는지
       기존 check_exit() 로 페어(pairwise) 체크 (baseline의 Stage-4 검사와
       동일한 스타일 -- 새 지오메트리 로직을 만들지 않고 재사용).

  안전장치: shielding을 만족하는 후보가 하나도 없으면(과도한 제약으로
  구멍이 뚫리는 것을 막기 위해) 원래(비차폐) 최선 후보로 자동 폴백한다.
  즉 이 알고리즘은 baseline_greedy.py보다 절대 더 나쁜 후보를 고르지
  않는다 -- shielding은 "가능하면 선호"이지 "필수 제약"이 아니다.

  SHIELD_ENABLED 상수로 켜고 끌 수 있다 (False = baseline과 완전히 동일).

변경되지 않은 부분 (baseline_greedy.py 와 동일):
  - EDD 정렬, AABB 기반 _candidate_positions, _find_earliest_slot
  - repair 패스 로직, _build_operations, feasibility-check 통합
  - 모든 weight 처리 (w1/w2/w3) 및 전체 objective 구조

입력/출력 계약:
  algorithm(prob_info, timelimit=60) -> dict
  반환 형식: {"operations": {...}}  (utils.check_feasibility 통과 보장)

===============================================================================
ALGORITHM OVERVIEW (baseline_greedy.py 원문)
===============================================================================

Phase 1 -- Aggressive greedy placement (EDD order):
  Blocks are sorted by Earliest Due Date (ties broken by Shortest Processing
  Time).  For each block, every (bay, orientation, position, time-slot)
  combination is scored; the cheapest is committed.  Crane-path feasibility
  (check_entry / check_exit) is verified against the current bay state, so
  most Phase-1 placements are already crane-feasible.

Phase 2 -- Iterative repair:
  check_feasibility is called on the Phase-1 solution.  Violating blocks are
  re-placed in EDD order.  Two modes are supported (repair_mode parameter):

  * "greedy" (default)
      Violating blocks are removed from the current solution and re-placed
      using the same full Phase-1 search (best bay + position + time-slot).
      State (bay_placed / bay_schedule / bay_loads) is reconstructed from
      the non-violating assignments before each pass.
      Cycle detection: if a block reappears in a second repair pass it is
      added to forced_ids, which bypasses search and uses _force_place
      (empty-bay window at (0,0)) to guarantee termination.
      Time guard: blocks whose turn comes after 90% of timelimit are also
      sent to _force_place to ensure all blocks are assigned before timeout.

  * "simple"
      Each violating block keeps its current (bay, x, y, orient) and is only
      pushed to the next empty-bay window (bay completely empty for the full
      processing duration).  Stage-4 (spatial collision) violations are also
      reset to position (0,0).

===============================================================================
SOLUTION DICT FORMAT
===============================================================================

{
    "operations": {
        "<time_int>": [           # integer time-point as string key
            {
                "type":       "EXIT",   # crane removes block from bay
                "block_id":   int,
                "bay_id":     int,
            },
            {
                "type":       "ENTRY",  # crane places block into bay
                "block_id":   int,
                "bay_id":     int,
                "x":          int,      # bottom-left x the reference point within the bay
                "y":          int,      # bottom-left y the reference point within the bay
                "orient_idx": int,      # index into block["shape"] list
            },
            ...
        ],
        ...
    }
}

At each time-point, EXIT operations always precede ENTRY operations.
Within the same type, operations are ordered so that each is feasible given
the bay state after all preceding operations at that time have completed.
entry_time = int(t_str) for ENTRY ops; exit_time = int(t_str) for EXIT ops.

Feasibility checking and objective computation: utils.check_feasibility(prob_info, solution).
"""

import math
import time
from utils import (Bay, Block, check_entry, check_exit, check_collisions,
                   _resolve_layers, _bounding_box, _bb_overlap)


# ==========================================================================
# Due-date shielding -- Z1 직접 겨냥 실험용 상수
# ==========================================================================

# True(기본): shielding 규칙 활성화.  False: baseline_greedy.py와 완전히 동일
# (shielding 로직을 완전히 우회하여 순수 baseline 동작으로 돌아간다).
SHIELD_ENABLED: bool = True

# K-best : (bay, orientation) 조합마다 _find_earliest_slot(비싼 check_entry/
# check_exit 호출을 포함)을 최대 몇 개의 실행가능 후보에서 멈출지.  프로파일링
# 결과 check_entry/check_exit 호출 수 자체가 병목이었으므로(전체 시간의
# ~94%), 후보를 다 평가하지 않고 K개 찾으면 멈추는 것으로 그 호출 수를
# 직접 줄인다.  0 또는 매우 큰 값 = 사실상 무제한(기존 baseline과 동일).
#
# 기본값을 0(무제한)으로 되돌림: prob_38에서 K-best+M-best 조합을 테스트한
# 결과, 위치당 성공 확률이 떨어져서 오히려 K_BEST개를 채우기 위해 더 많은
# 후보를 시도하게 되고, 총 호출 수가 늘어나는 역효과가 관찰됨. prob_1/prob_2
# 에서 이미 검증된(obj1=2.0 / obj1=0.0) 무제한 상태로 우선 복귀.
K_BEST: int = 0

# M-best : _find_earliest_slot 내부에서 시도할 entry-time 후보 개수 상한.
# K_BEST가 "후보 위치" 축을 줄인다면, 이건 그 반대 축("위치 하나당 시도하는
# entry-time 개수")을 줄인다 -- 베이가 찰수록 candidate_entries 집합 자체가
# 커지므로, 두 축을 모두 캡 해야 check_entry/check_exit 호출량이 진짜로
# 줄어든다.  0 = 무제한(기존과 동일, 모든 candidate_entries 시도).
#
# 기본값 0(무제한)으로 되돌림: prob_38에서 캡을 걸었더니 위치당 성공률이
# 떨어져 오히려 더 많은 후보를 시도하게 되는 역효과가 관찰되어 비활성화.
MAX_ENTRY_TRIES: int = 0


# ==========================================================================
# 인스턴스 밀도 디스패처 -- 입력 데이터를 먼저 분석해 K_BEST를 자동 조정
# ==========================================================================
#
# 이전 버전(이분법 dense/normal 임계값)의 문제: prob_6(blocks/bay=50)에서
# 검증한 K_BEST=20을 prob_37/38/39(blocks/bay=83.3)에도 "dense"라는 이유로
# 똑같이 적용했더니, prob_38에서 250개 중 170개가 강제배치될 만큼 부족했다.
# blocks/bay=50과 83.3은 같은 "dense" 라벨이어도 밀집도가 다른데, 계단식
# 이분법은 그 차이를 반영하지 못한다.
#
# 그래서 이분법 대신 blocks/bay 비율에 **반비례하는 연속 함수**로 바꾼다:
# 인스턴스가 더 밀집할수록 K_BEST가 매끄럽게 더 작아진다.
#   K_BEST = K_BEST_SCALE_CONST / blocks_per_bay  (반올림, K_BEST_MIN 이상)
# K_BEST_SCALE_CONST=1000 은 prob_6(blocks/bay=50 -> 검증된 K=20)에 정확히
# 맞도록 역산한 값이다: 1000/50=20.  blocks/bay=83.3(prob_37/38/39)이면
# 1000/83.3≈12로 자동으로 더 타이트해진다.
#
# 단, blocks/bay가 매우 낮으면(prob_1=50, prob_2=33.3, prob_7=50 모두
# K_BEST=0/무제한에서 obj1=0~21로 검증됨) 공식이 주는 큰 K값보다 아예
# 무제한을 쓰는 게 이미 확인된 선택이라, K_BEST_UNLIMITED_BELOW 미만에서는
# 공식을 적용하지 않고 그냥 0(무제한)을 쓴다.
#
# 참고: 이 공식도 딱 두 앵커 포인트(prob_2, prob_6)로 역산한 것이라 여전히
# 근사치다 -- 더 많은 인스턴스로 검증이 필요하다는 점은 동일하게 남아있다.
AUTO_DISPATCH: bool = True

# 이 값 미만의 blocks/bay 는 공식을 안 타고 그냥 무제한(0)을 쓴다.
# prob_2(33.3)는 확실히 이 구간, prob_6(50)은 공식 구간으로 넘어가야 하므로
# 그 사이(40)로 잡는다.
K_BEST_UNLIMITED_BELOW: float = 40.0

# K_BEST = round(이 값 / blocks_per_bay).  prob_6(50 -> K=20)에 맞춘 역산값.
K_BEST_SCALE_CONST: float = 1000.0

# 공식이 아무리 밀집해도 이 아래로는 안 내려간다 (완전히 후보를 못 보는
# 상황 방지).
K_BEST_MIN: int = 3

# repair 단계 전용 K_BEST 상한 -- 인스턴스 밀도와 무관하게 항상 적용된다.
# 이유: repair는 Phase 1이 끝난 "직후"에 돌아가는데, 이 시점엔 인스턴스가
# 애초에 얼마나 밀집했든 베이가 이미 거의 꽉 차 있다.  prob_23(공식 적용 시
# K_BEST=20이지만 예전 이분법에선 "normal"->0을 그대로 물려받아)에서 위반
# 블록 단 3개를 고치는 repair 호출 하나가 28초나 걸려 60초 제한을 넘긴
# 사고가 실제로 발생했다.  검증된 K=20(prob_6)을 재사용.
K_BEST_REPAIR: int = 20


def _classify_instance(prob_info: dict) -> tuple[int, float]:
    """
    blocks/bay 비율에 반비례하는 연속 함수로 K_BEST를 계산한다 (모듈
    docstring 참조).  (K_BEST 값, blocks_per_bay) 를 반환한다.
    """
    n_blocks = len(prob_info["blocks"])
    n_bays   = len(prob_info["bays"])
    blocks_per_bay = n_blocks / n_bays

    if blocks_per_bay < K_BEST_UNLIMITED_BELOW:
        k_best = 0
    else:
        k_best = max(K_BEST_MIN, round(K_BEST_SCALE_CONST / blocks_per_bay))

    return k_best, blocks_per_bay


def _shielding_violated(new_blk: Block, bay: Bay,
                        active_in_bay: list[Block],
                        due_new: float,
                        blocks_data: list[dict]) -> bool:
    """
    new_blk를 이 위치에 놓았을 때, due_date가 new_blk보다 이르거나 같은
    (즉 EDD 순서상 먼저 나가야 하는) 활성 블록의 미래 크레인 출차 경로를
    막게 되는지 확인한다.

    기존 check_exit() 를 페어(pairwise)로 재사용한다 -- baseline의 Stage-4
    내부-구간 충돌 검사가 check_collisions()를 페어로 쓰는 것과 동일한
    스타일이며, 새 지오메트리 로직을 만들지 않는다.

    True 를 반환하면 이 후보는 "차폐 위반" -- 호출부에서 제외하거나
    비차폐 폴백으로 넘긴다.
    """
    for b_other in active_in_bay:
        due_other = blocks_data[b_other.block_id]["due_date"]
        if due_other <= due_new and check_exit(bay, [b_other, new_blk], b_other, fast=True):
            return True
    return False


def _find_shielding_blocker(new_blk: Block, bay: Bay,
                           active_in_bay: list[Block],
                           due_new: float,
                           blocks_data: list[dict]) -> Block | None:
    """
    _shielding_violated 과 동일한 판정이지만, bool 대신 실제로 막고 있는
    Block 객체를 반환한다 -- destroy/repair(ALNS 스타일) 호출부가 "누구를
    치울지" 알아야 하기 때문이다.
    """
    for b_other in active_in_bay:
        due_other = blocks_data[b_other.block_id]["due_date"]
        if due_other <= due_new and check_exit(bay, [b_other, new_blk], b_other, fast=True):
            return b_other
    return None


def _pop_block_from_bay(bay_placed: list[list[Block]],
                        bay_schedule: list[list[tuple[int, int]]],
                        bay_loads: list[float],
                        blocks_data: list[dict],
                        bay_id: int, block_id: int) -> None:
    """
    bay_placed[bay_id]/bay_schedule[bay_id] 에서 block_id 항목을 제거하고
    bay_loads[bay_id] 에서 그만큼의 workload를 뺀다 (destroy 단계에서 사용).
    두 리스트는 인덱스로 정렬돼 있다고 가정한다 (기존 코드 전반의 불변식).
    """
    idx = next(i for i, b in enumerate(bay_placed[bay_id]) if b.block_id == block_id)
    bay_placed[bay_id].pop(idx)
    bay_schedule[bay_id].pop(idx)
    bay_loads[bay_id] -= blocks_data[block_id]["workload"]


# -----------------------------------------------------------------------------
# Helpers: block bounding box (anchored, per orientation)
# -----------------------------------------------------------------------------

def _block_bbox(block_data: dict, orient_idx: int) -> tuple[float, float, float, float]:
    """Bounding box of a block in local coordinates relative to the reference
    point (first vertex of first layer = (0, 0)).  Returns (min_x, min_y, max_x, max_y)."""
    raw_layers = block_data["shape"][orient_idx]["layers"]
    layers = _resolve_layers(raw_layers)
    if not layers:
        return (0.0, 0.0, 1.0, 1.0)
    all_verts = [v for l in layers for v in l]
    return _bounding_box(all_verts)


# -----------------------------------------------------------------------------
# Helper: time interval overlap check
# -----------------------------------------------------------------------------

def _time_overlaps(a_entry: int, a_exit: int,
                   b_entry: int, b_exit: int) -> bool:
    """True if intervals [a_entry, a_exit) and [b_entry, b_exit) overlap."""
    return a_entry < b_exit and b_entry < a_exit


# -----------------------------------------------------------------------------
# Helper: candidate position generation (bottom-left corner based)
# -----------------------------------------------------------------------------

def _candidate_positions(bay_w: float, bay_h: float,
                         placed_blocks: list[Block],
                         blk_bb: tuple[float, float, float, float]) -> list[tuple[int, int]]:
    """
    Return integer (x, y) reference-point candidate positions for a new block
    using the "bottom-left fill" heuristic.

    blk_bb = (local_min_x, local_min_y, local_max_x, local_max_y) in local
    coordinates (reference point = first vertex of first layer = (0, 0)).
    A placement (x, y) is valid iff the block's world bbox stays within the bay:
      x + blk_bb[0] >= 0,  y + blk_bb[1] >= 0
      x + blk_bb[2] <= bay_w,  y + blk_bb[3] <= bay_h
    Candidates are sorted by (x, y) so the search visits left-most / bottom-most
    positions first.
    """
    lx0, ly0, lx1, ly1 = blk_bb
    # Smallest valid integer reference-point position (block's left/bottom edge at bay wall)
    xs = {max(0, math.ceil(-lx0))}
    ys = {max(0, math.ceil(-ly0))}
    for b in placed_blocks:
        bb = b.bounding_rect()
        # Reference-point x/y such that new block's left/bottom edge touches the
        # right/top edge of this placed block
        xs.add(math.ceil(bb[2] - lx0))
        ys.add(math.ceil(bb[3] - ly0))

    candidates = []
    for x in sorted(xs):
        for y in sorted(ys):
            if x + lx1 <= bay_w + 1e-6 and y + ly1 <= bay_h + 1e-6:
                candidates.append((int(x), int(y)))
    return candidates


# -----------------------------------------------------------------------------
# Placement score (lower is better)
# -----------------------------------------------------------------------------

def _placement_score(tardiness: float, workload: float,
                     bay_loads: list[float], bay_id: int,
                     pref_penalty: float,
                     bay_weights: list[float],
                     w1: float, w2: float, w3: float,
                     top_y: float = 0.0, w4: float = 1e-4) -> float:
    """
    Composite score for placing a block in bay_id (lower is better).

      w1 * tardiness    -- total tardiness: max(0, exit_time - due_date).

      w2 * new_obj2     -- approximation of normalized load-balance penalty.
                          new_obj2 = max_j |u[bay_id]*new_load - u[j]*load_j|
                          where u_j = avg_bay_area / (W_j * H_j).

      w3 * pref_penalty -- preference penalty: S_i_max - S_i_bay_id.
                          0 when placed in most-preferred bay.

      w4 * top_y        -- tie-breaking: lower top edge -> tighter packing.
    """
    new_load = bay_loads[bay_id] + workload
    new_obj2 = max(
        (abs(bay_weights[bay_id] * new_load - bay_weights[j] * bay_loads[j])
         for j in range(len(bay_loads)) if j != bay_id),
        default=0.0
    )
    return w1 * tardiness + w2 * new_obj2 + w3 * pref_penalty + w4 * top_y


# -----------------------------------------------------------------------------
# Earliest feasible entry slot (aggressive -- allows time overlap)
# -----------------------------------------------------------------------------

def _find_earliest_slot(new_blk: Block,
                        bay: Bay,
                        placed_in_bay: list[Block],
                        schedule_in_bay: list[tuple[int, int]],
                        r_time: int,
                        proc: int) -> tuple[int | None, int | None]:
    """
    Return the earliest (entry, exit_t) time slot >= r_time at which new_blk
    can be crane-placed into bay without violating Stage-2 (entry) or Stage-3
    (exit) feasibility.  Returns (None, None) if no candidate entry passes
    both checks -- this means the (position, bay) combination is infeasible for
    any time and the caller should try a different position.

    Performance contract (myalgorithm4 "idea 1" change): placed_in_bay/
    schedule_in_bay do NOT need to be the bay's full history -- every internal
    filter below only ever keeps entries with e > r_time anyway (entry is
    always >= r_time, and a<entry<e requires e>entry>=r_time), so passing an
    ALREADY r_time-filtered "active only" pair of lists produces byte-identical
    results while skipping the already-exited blocks that accumulate in a bay
    over the course of Phase 1.  The hot call site in _place_blocks' K-best
    loop does this (builds active_in_bay/active_schedule once per (bay,
    orient) instead of letting each of up-to-K_BEST calls here re-filter the
    full list from scratch).  Passing the full unfiltered lists still works
    correctly (just slower) -- this function's own logic is unchanged.

    -- Candidate enumeration ----------------------------------------------------
    Candidates = {r_time} | {exit_time of every already-placed block in bay},
    capped to the first MAX_ENTRY_TRIES entries (myalgorithm4 change vs
    baseline_greedy.py -- this set grows with placed-block count, so on large
    instances trying all of them multiplies the already-expensive check_entry/
    check_exit calls below).  Capping means a position that WOULD have found a
    feasible slot beyond the cap is instead reported as infeasible (None,
    None) -- safe because the caller (_place_blocks) already has a two-tier
    unshielded/shielded fallback and, ultimately, _force_place; it just means
    this specific position is skipped a bit more eagerly.

    -- Feasibility checks (mirror of check_feasibility Stages 2 & 3) -----------
    Stage-2 (crane entry): the crane path must not be blocked at entry_time.
      present_at_entry = blocks b_k with  a_k <= entry < e_k
      check_entry(bay, present_at_entry, new_blk, fast=True) returns True if
      ANY block in present_at_entry obstructs the crane path; fast=True exits
      on the first obstruction to avoid unnecessary Shapely work.

    Stage-3 (crane exit): the crane path must not be blocked at exit_time.
      present_at_exit  = [new_blk] + blocks b_k with  a_k < exit_t < e_k
      (new_blk itself is included because it will be present during its own exit)
      check_exit(bay, present_at_exit, new_blk, fast=True) returns True if
      ANY block in present_at_exit obstructs the crane exit path.

    Stage-4 (interior-interval): blocks whose interval is strictly inside
      [entry, exit_t) -- i.e. entry < a_k AND e_k < exit_t -- are invisible
      to the Stage-2 and Stage-3 boundary checks above.  They are present
      during new_blk's stay but not at its entry or exit moment.  A per-pair
      spatial collision check is run for these blocks to avoid producing
      Stage-4 violations that the repair loop cannot detect at placement time.
    """
    candidate_entries = sorted({r_time} | {e for _, e in schedule_in_bay if e > r_time})
    if MAX_ENTRY_TRIES > 0:
        candidate_entries = candidate_entries[:MAX_ENTRY_TRIES]

    for entry_candidate in candidate_entries:
        entry  = max(r_time, entry_candidate)
        exit_t = entry + proc

        # Stage-2: blocks already present when new_blk arrives.
        # Mirrors check_feasibility: a_k < entry < e_k  (strict lower bound --
        # blocks entering at the same moment are handled by Stage-5 ordering).
        present_at_entry = [
            b for b, (a, e) in zip(placed_in_bay, schedule_in_bay)
            if a < entry < e
        ]
        if check_entry(bay, present_at_entry, new_blk, fast=True):
            continue  # crane path blocked at entry -> try next exit boundary

        # Stage-3: blocks still present when new_blk departs.
        # Mirrors check_feasibility: a_k < exit_t < e_k  (strict both ends).
        present_at_exit = [new_blk] + [
            b for b, (a, e) in zip(placed_in_bay, schedule_in_bay)
            if a < exit_t < e
        ]
        if check_exit(bay, present_at_exit, new_blk, fast=True):
            continue  # crane path blocked at exit -> try next exit boundary

        # Stage-4 pre-check: blocks that co-exist with new_blk during (entry, exit_t)
        # but are absent at both boundary moments, so Stage-2 and Stage-3 above
        # don't cover them.  A block b_other falls into this gap when:
        #   a_other >= entry  (not caught by Stage-2: a_k < entry is false)
        #   e_other <= exit_t (not caught by Stage-3: e_k > exit_t is false)
        # AND its interval actually overlaps [entry, exit_t).
        # Note: e_other == exit_t means b_other departs exactly when new_blk does;
        # check_feasibility treats them as co-present during their shared [a,e) so
        # a spatial collision is still a violation -- include it here.
        s4_blocked = False
        for b_other, (a_other, e_other) in zip(placed_in_bay, schedule_in_bay):
            if a_other < entry or e_other > exit_t:
                continue  # covered by Stage-2 (a_other < entry) or Stage-3 (e_other > exit_t)
            if not _time_overlaps(entry, exit_t, a_other, e_other):
                continue  # disjoint in time
            if check_collisions(bay, [new_blk, b_other]):
                s4_blocked = True
                break
        if s4_blocked:
            continue

        return entry, exit_t

    return None, None  # no valid time slot for this (position, bay) combination


# -----------------------------------------------------------------------------
# Guaranteed-feasible entry: empty-bay window
# -----------------------------------------------------------------------------

def _empty_bay_entry(schedule_in_bay: list[tuple[int, int]],
                     r_time: int, proc: int) -> int:
    """
    Return the earliest entry time >= r_time such that the bay is completely
    empty for the entire window [entry, entry + proc).

    This guarantees crane-path feasibility: when the bay is empty at both
    entry_time and exit_time, check_entry and check_exit trivially pass
    (no blocks present means no polygon obstructions).

    Algorithm -- iterative push:
      Start with entry = r_time.  Scan all existing slots (a_k, e_k).  If
      [entry, entry+proc) overlaps any slot, advance entry to e_k (the end of
      that slot) so the window no longer overlaps it.  Repeat until no
      overlaps remain.

    Convergence guarantee:
      Each iteration advances entry by at least the distance to the next
      slot endpoint.  Because the number of slots is finite, the loop
      terminates after at most len(schedule_in_bay) passes.
    """
    entry = int(r_time)
    changed = True
    while changed:
        changed = False
        exit_t = entry + proc
        for a, e in schedule_in_bay:
            if _time_overlaps(entry, exit_t, a, e):
                entry = max(entry, e)  # push past the overlapping slot
                changed = True
    return entry


# -----------------------------------------------------------------------------
# Main algorithm
# -----------------------------------------------------------------------------

def algorithm(prob_info: dict, timelimit: float = 60,
             repair_mode: str = "greedy") -> dict:
    """
    EDD + Best-Fit Greedy + due-date shielding, with post-hoc feasibility repair.

    Parameters
    ----------
    prob_info   : instance JSON dict with keys "name", "bays", "blocks", "weights"
    timelimit   : wall-clock time limit in seconds
    repair_mode : "greedy" (default) or "simple" -- see module docstring for details

    Returns
    -------
    solution dict in the format described in the module docstring

    Phase 1 -- EDD greedy placement:
        Blocks sorted by (due_date, processing_time).  For each block, every
        (bay, orientation, candidate position) is tried; _find_earliest_slot
        computes the earliest crane-feasible time slot.  Among candidates that
        satisfy due-date shielding, the one minimising _placement_score is
        committed (falls back to the unshielded best if none qualify).
        bay_placed, bay_schedule, and bay_loads are updated incrementally.

    Phase 2 -- Repair (see _repair and module docstring for details):
        Calls _repair which runs up to max_passes rounds of
        check_feasibility -> re-place violating blocks.
    """
    t_start = time.time()

    bays_data   = prob_info["bays"]
    blocks_data = prob_info["blocks"]
    n_bays      = len(bays_data)
    n_blocks    = len(blocks_data)

    w1 = prob_info.get("weights", {}).get("w1", 1.0)
    w2 = prob_info.get("weights", {}).get("w2", 1.0)
    w3 = prob_info.get("weights", {}).get("w3", 1.0)

    # -- 인스턴스 밀도 디스패처: K_BEST를 입력 데이터 기준으로 자동 조정 -----
    global K_BEST
    if AUTO_DISPATCH:
        K_BEST, blocks_per_bay = _classify_instance(prob_info)
    else:
        blocks_per_bay = n_blocks / n_bays

    print(f"[Greedy] Instance : {prob_info.get('name', '?')}")
    print(f"[Greedy] Bays     : {n_bays}  |  Blocks : {n_blocks}  |  Timelimit : {timelimit:.1f}s")
    print(f"[Greedy] Weights  : w1={w1}  w2={w2}  w3={w3}")
    print(f"[Greedy] Shielding: SHIELD_ENABLED={SHIELD_ENABLED}")
    print(f"[Greedy] Dispatch : blocks/bay={blocks_per_bay:.1f}  AUTO_DISPATCH={AUTO_DISPATCH}  "
          f"-> K_BEST={K_BEST}")
    print(f"[Greedy] {'-' * 56}")

    bays = [Bay.from_dict(d, i) for i, d in enumerate(bays_data)]
    for i, b in enumerate(bays):
        print(f"[Greedy]   bay[{i}]  {b.width}x{b.height}")

    # -- Instance validity check: every block must have at least one valid -----
    # integer (x, y) position in at least one bay and orientation.
    # If not, the problem instance itself is malformed -- abort immediately.
    invalid_blocks = []
    for bi, blk_data in enumerate(blocks_data):
        placeable = False
        for bay in bays:
            for oi in range(len(blk_data["shape"])):
                bb = _block_bbox(blk_data, oi)
                lx0, ly0, lx1, ly1 = bb
                if (math.ceil(-lx0) <= math.floor(bay.width  - lx1) and
                        math.ceil(-ly0) <= math.floor(bay.height - ly1)):
                    placeable = True
                    break
            if placeable:
                break
        if not placeable:
            invalid_blocks.append(bi)
    if invalid_blocks:
        print(f"[Greedy] ERROR: {len(invalid_blocks)} block(s) cannot be placed at any integer "
              f"position in any bay -- malformed instance.")
        for bi in invalid_blocks:
            blk_data = blocks_data[bi]
            for bay in bays:
                for oi in range(len(blk_data["shape"])):
                    bb = _block_bbox(blk_data, oi)
                    lx0, ly0, lx1, ly1 = bb
                    bw, bh = lx1 - lx0, ly1 - ly0
                    print(f"[Greedy]   block {bi} oi={oi} bay{bay.id}({bay.width}x{bay.height}): "
                          f"bw={bw:.4f} bh={bh:.4f} "
                          f"px=[{math.ceil(-lx0)},{math.floor(bay.width-lx1)}] "
                          f"py=[{math.ceil(-ly0)},{math.floor(bay.height-ly1)}]")
        raise ValueError(
            f"Malformed instance '{prob_info.get('name', '?')}': "
            f"block(s) {invalid_blocks} have no valid integer placement in any bay."
        )

    # -- Phase 1: aggressive greedy --------------------------------------------
    sorted_indices = sorted(
        range(n_blocks),
        key=lambda i: (blocks_data[i]["due_date"], blocks_data[i]["processing_time"])
    )
    print(f"[Greedy] {'-' * 56}")
    print("[Greedy] Phase 1 : EDD greedy placement ...")

    bay_placed:   list[list[Block]]             = [[] for _ in range(n_bays)]
    bay_schedule: list[list[tuple[int, int]]]   = [[] for _ in range(n_bays)]
    bay_loads:    list[float]                   = [0.0] * n_bays

    assignments = _place_blocks(
        sorted_indices, blocks_data, bays,
        bay_placed, bay_schedule, bay_loads,
        w1, w2, w3, forced_ids=set(),
        t_start=t_start, log_interval=max(1, n_blocks // 10),
        timelimit=timelimit,
    )

    elapsed_p1 = time.time() - t_start
    loads_str = "  ".join(f"bay{i}={round(bay_loads[i])}" for i in range(n_bays))
    print(f"[Greedy] Phase 1 done  |  placed={len(assignments)}  {loads_str}  "
          f"elapsed={elapsed_p1:.2f}s")

    # -- Phase 2: repair infeasible assignments --------------------------------
    print(f"[Greedy] {'-' * 56}")
    print(f"[Greedy] Phase 2 : repair  mode={repair_mode}")
    sol = {"operations": _build_operations(list(assignments.values()), prob_info)}
    assignments = _repair(prob_info, sol, assignments, bays, blocks_data,
                          w1, w2, w3, t_start, timelimit,
                          repair_mode=repair_mode)

    elapsed_total = time.time() - t_start
    final_sol = {"operations": _build_operations(list(assignments.values()), prob_info)}

    from utils import check_feasibility
    final_result = check_feasibility(prob_info, final_sol)
    print(f"[Greedy] {'-' * 56}")
    print(f"[Greedy] Done  |  assigned={len(assignments)}/{n_blocks}  "
          f"elapsed={elapsed_total:.2f}s")
    if final_result["feasible"]:
        print(f"[Greedy] Objective : {final_result['objective']:.0f}  "
              f"(obj1={final_result['obj1']:.1f}  "
              f"obj2={final_result['obj2']:.1f}  "
              f"obj3={final_result['obj3']:.1f})")
    else:
        print(f"[Greedy] INFEASIBLE stage={final_result['stage']}")
        for v in final_result["violations"][:5]:
            print(f"[Greedy]   {v}")

    return final_sol


# -----------------------------------------------------------------------------
# Force-place helper (no feasibility check -- used as phase-1 last resort)
# -----------------------------------------------------------------------------

# AABB-disjoint 강제배치 탐색 상한 (myalgorithm6 신규 -- 폭주 방지 캡).
# entry 후보(베이의 exit 경계들)를 이 개수까지만 훑고, 각 entry에서 x/y 후보를
# 이 개수까지만 조합한다.  캡을 넘겨도 못 찾으면 기존 empty-bay 폴백으로
# 넘어가므로 안전성은 동일하고, 최악 케이스 비용만 상수로 묶인다.
FORCE_ENTRY_TRIES: int = 60
FORCE_POS_XS: int = 8
FORCE_POS_YS: int = 8


def _force_place(bi: int,
                 blocks_data: list[dict],
                 bays: list[Bay],
                 bay_placed: list[list[Block]],
                 bay_schedule: list[list[tuple[int, int]]],
                 prefs: list[float]) -> tuple:
    """
    Fallback placement -- myalgorithm6 upgrade: AABB-disjoint search first,
    empty-bay window as the unchanged safety net.

    myalgorithm4의 empty-bay 방식의 문제: "베이가 통째로 비는 시간창"을
    기다리므로, 강제배치 블록이 많아지는 포화 인스턴스(prob_38: 157개)에서는
    이들이 사실상 한 줄로 직렬화되어 Z1의 대부분을 만들어낸다.

    핵심 관찰 (utils.py의 구조적 보장): check_entry / check_exit /
    check_collisions 세 검사 모두 첫 단계에서 `_bb_overlap` AABB 프리필터로
    "풋프린트 바운딩박스가 겹치지 않는 블록"을 아예 건너뛴다.  따라서 새
    블록의 world bbox가 **자기 체류시간과 겹치는 모든 블록의 bbox와
    서로소**인 위치를 고르면, Stage-2(진입 크레인 경로)/Stage-3(퇴출 크레인
    경로)/Stage-4(체류 중 공간 충돌)가 전부 구조적으로 통과된다 -- 베이가
    비어있을 필요가 전혀 없고, Shapely 호출도 전혀 없다 (순수 정수/구간
    연산만).  같은 시각 동시 진입 블록도 active 집합에 포함해 서로소를
    요구하므로 Stage-5 순서 문제도 만들지 않는다.

    탐색: 베이별 첫 fitting orientation에 대해 entry 후보(r_time + 기존 exit
    경계, 오름차순, FORCE_ENTRY_TRIES개까지)를 훑고, 각 entry에서 bottom-left
    스타일 후보 위치(active 블록 bbox의 오른쪽/위쪽 모서리, FORCE_POS_XS x
    FORCE_POS_YS개까지)를 시도해 첫 번째 서로소 위치를 찾는다.  entry
    오름차순이므로 첫 성공 = 그 베이에서 이 방법으로 가능한 가장 이른 진입.
    캡 안에서 못 찾으면 그 베이는 기존 _empty_bay_entry 폴백을 쓴다 --
    empty-bay 창은 AABB-서로소 조건의 특수 케이스(active가 공집합)이므로 이
    탐색은 결과적으로 기존 방식보다 늦은 entry를 고를 수 없다.

    최종적으로 모든 베이의 후보 중 (entry, -preference)가 최소인 것을 고른다
    (myalgorithm4의 earliest-entry-across-bays 선택 로직 그대로).
    """
    blk_data = blocks_data[bi]
    r_time   = blk_data["release_time"]
    proc     = blk_data["processing_time"]
    n_bays   = len(bays)

    # (entry, -pref, bay_id, px, py, oi) for every bay/orientation that fits;
    # sort key picks earliest entry first, highest preference as tie-break.
    candidates: list[tuple[int, float, int, int, int, int]] = []
    for bay_id in range(n_bays):
        bay = bays[bay_id]
        for oi in range(len(blk_data["shape"])):
            bb = _block_bbox(blk_data, oi)
            lx0, ly0, lx1, ly1 = bb
            px_lo = math.ceil(-lx0)
            px_hi = math.floor(bay.width  - lx1)
            py_lo = math.ceil(-ly0)
            py_hi = math.floor(bay.height - ly1)
            if px_lo > px_hi or py_lo > py_hi:
                continue  # no valid integer position for this orientation
            px = max(0, px_lo)
            py = max(0, py_lo)

            # -- AABB-disjoint search (구조적으로 crane-feasible) -------------
            found: tuple[int, int, int] | None = None
            entry_cands = sorted(
                {r_time} | {e for _, e in bay_schedule[bay_id] if e > r_time}
            )[:FORCE_ENTRY_TRIES]
            for entry_c in entry_cands:
                exit_c = entry_c + proc
                # active = 이 시간창과 겹치는 모든 블록의 world bbox.
                # 경계 반개구간 처리(e == entry 제외, a == exit 제외)는
                # check_feasibility Stage-2/3의 strict 부등호와 일치한다.
                active_bbs = [
                    b.bounding_rect()
                    for b, (a_k, e_k) in zip(bay_placed[bay_id], bay_schedule[bay_id])
                    if a_k < exit_c and e_k > entry_c
                ]
                if not active_bbs:
                    found = (entry_c, px, py)
                    break
                xs = sorted({px} | {math.ceil(ab[2] - lx0) for ab in active_bbs})
                xs = [x for x in xs if px_lo <= x <= px_hi][:FORCE_POS_XS]
                ys = sorted({py} | {math.ceil(ab[3] - ly0) for ab in active_bbs})
                ys = [y for y in ys if py_lo <= y <= py_hi][:FORCE_POS_YS]
                for cy in ys:
                    for cx in xs:
                        wbb = (cx + lx0, cy + ly0, cx + lx1, cy + ly1)
                        if all(not _bb_overlap(wbb, ab) for ab in active_bbs):
                            found = (entry_c, cx, cy)
                            break
                    if found:
                        break
                if found:
                    break

            if found is not None:
                entry_f, fx, fy = found
                candidates.append((entry_f, -prefs[bay_id], bay_id, fx, fy, oi))
            else:
                entry = _empty_bay_entry(bay_schedule[bay_id], r_time, proc)
                candidates.append((entry, -prefs[bay_id], bay_id, px, py, oi))
            break  # first fitting orientation for this bay (matches original semantics)

    if not candidates:
        # This path should never be reached: algorithm() checks at startup that
        # every block has at least one valid integer position and raises
        # ValueError for malformed instances before any placement begins.
        raise RuntimeError(
            f"_force_place: block {bi} has no valid integer position in any bay "
            f"-- instance validation should have caught this."
        )

    entry, _, bay_id, px, py, oi = min(candidates, key=lambda c: (c[0], c[1]))
    return (bay_id, px, py, oi, entry, entry + proc)


# -----------------------------------------------------------------------------
# Shared greedy placement kernel (used by Phase 1 and _repair)
# -----------------------------------------------------------------------------

def _place_blocks(
    block_ids: list[int],
    blocks_data: list[dict],
    bays: list[Bay],
    bay_placed: list[list[Block]],
    bay_schedule: list[list[tuple[int, int]]],
    bay_loads: list[float],
    w1: float, w2: float, w3: float,
    forced_ids: set[int],
    prev_assignments: dict[int, dict] | None = None,
    t_start: float | None = None,
    log_interval: int = 0,
    timelimit: float | None = None,
) -> dict[int, dict]:
    """
    Shared placement kernel used by both Phase 1 and _repair (greedy mode).

    For each block in block_ids, finds the best (bay, x, y, orient, entry_time)
    by minimising _placement_score, then commits it to bay_placed /
    bay_schedule / bay_loads.  Returns a dict mapping block_id -> assignment.

    Search order:
      1. Repair fast-path (only when prev_assignments is provided):
         Try the block's previous (bay, x, y, orient) with _find_earliest_slot.
         If that position is still crane-feasible, record it as the initial
         best candidate.  This avoids re-solving the position search for blocks
         that only need a time adjustment.
      2. Full search (Phase-1 style):
         Iterate bays in decreasing preference order, then orientations, then
         candidate positions from _candidate_positions.  For each (bay, orient,
         pos), call _find_earliest_slot to get the earliest crane-feasible slot.
         Keep the (bay, orient, pos, slot) with the lowest _placement_score.
      3. Forced path (forced_ids or no feasible combination found):
         Blocks in forced_ids skip steps 1-2 entirely and go straight to
         _force_place.  If the full search in step 2 found nothing, _force_place
         is used as a fallback and n_fallback is incremented.

    Parameters
    ----------
    block_ids        : ordered list of block indices to place (EDD order)
    blocks_data      : raw block data list from prob_info
    bays             : Bay objects (width, height, polygon)
    bay_placed       : mutable per-bay lists of placed Block objects (updated in-place)
    bay_schedule     : mutable per-bay lists of (entry_time, exit_time) (updated in-place)
    bay_loads        : mutable per-bay cumulative workload floats (updated in-place)
    w1, w2, w3       : objective weights
    forced_ids       : block ids to bypass search and use _force_place directly
    prev_assignments : previous assignment dict (repair mode fast-path)
    t_start          : wall-clock start time (for log timestamps AND the time
                       guard below, if timelimit is also given)
    log_interval     : print a progress line every N blocks (0 = silent)
    timelimit        : wall-clock time budget in seconds.  If given (together
                       with t_start), Phase 1 stops searching once 80% of the
                       budget is used and force-places all remaining blocks
                       (myalgorithm4 change vs baseline_greedy.py -- baseline
                       had NO time guard here at all, so on larger instances
                       Phase 1's per-candidate shielding cost could silently
                       blow the wall-clock timelimit).  Mirrors the pattern
                       already used in myalgorithm.py's _place_all().

    Returns
    -------
    dict[block_id -> assignment dict] for all blocks in block_ids
    """
    n_bays  = len(bays)
    n_total = len(block_ids)
    result: dict[int, dict] = {}
    n_forced = n_fallback = 0

    # Bay weights for normalized obj2: u_j = avg_area / (W_j * H_j)
    _bay_areas   = [bay.width * bay.height for bay in bays]
    _avg_area    = sum(_bay_areas) / n_bays
    bay_weights  = [_avg_area / a for a in _bay_areas]

    for rank, bi in enumerate(block_ids):
        # -- Time budget guard (only active when timelimit is given) --------
        if (timelimit is not None and t_start is not None
                and time.time() - t_start > timelimit * 0.80):
            remaining = block_ids[rank:]
            print(f"[Greedy] TIME GUARD: {time.time() - t_start:.1f}s > "
                  f"{timelimit * 0.80:.1f}s (80% of {timelimit:.1f}s) -- "
                  f"force-placing {len(remaining)} remaining block(s)")
            for bid in remaining:
                bay_id, cx, cy, oi, entry, exit_t = _force_place(
                    bid, blocks_data, bays, bay_placed, bay_schedule,
                    blocks_data[bid]["bay_preferences"]
                )
                final_blk = Block(block_id=bid, block_data=blocks_data[bid],
                                  x=cx, y=cy, orient_idx=oi)
                bay_placed[bay_id].append(final_blk)
                bay_schedule[bay_id].append((entry, exit_t))
                bay_loads[bay_id] += blocks_data[bid]["workload"]
                result[bid] = {
                    "block_id": bid, "bay_id": bay_id,
                    "x": int(round(cx)), "y": int(round(cy)), "orient_idx": oi,
                    "entry_time": int(round(entry)), "exit_time": int(round(exit_t)),
                }
                n_fallback += 1
            break

        blk_data = blocks_data[bi]
        r_time   = blk_data["release_time"]
        due      = blk_data["due_date"]
        proc     = blk_data["processing_time"]
        workload = blk_data["workload"]
        prefs    = blk_data["bay_preferences"]
        s_max    = max(prefs)
        n_orient = len(blk_data["shape"])

        # 두 개의 독립 트래커를 유지한다:
        #   unshielded : 기존 baseline과 동일한 무제약 최선 후보 (안전망)
        #   shielded   : due-date shielding을 만족하는 후보 중 최선
        # 마지막에 shielded가 하나라도 있으면 그것을 쓰고, 없으면(과도한
        # 제약으로 전멸) unshielded로 폴백한다 -- baseline보다 나빠질 수 없다.
        best_score_unshielded     = float("inf")
        best_placement_unshielded = None
        best_score_shielded       = float("inf")
        best_placement_shielded   = None
        used_forced    = bi in forced_ids

        if not used_forced:
            # -- Repair fast-path: try previous (bay, x, y, orient) first -----
            if prev_assignments and bi in prev_assignments:
                pa = prev_assignments[bi]
                pb_id = pa["bay_id"]
                px, py, poi = int(pa["x"]), int(pa["y"]), pa["orient_idx"]
                prev_blk = Block(block_id=bi, block_data=blk_data,
                                 x=px, y=py, orient_idx=poi)
                if bays[pb_id].contains_block(prev_blk):
                    entry, exit_t = _find_earliest_slot(
                        prev_blk, bays[pb_id],
                        bay_placed[pb_id], bay_schedule[pb_id],
                        r_time, proc
                    )
                    if entry is not None:
                        tardiness = max(0.0, exit_t - due)
                        p_bb = _block_bbox(blk_data, poi)
                        score = _placement_score(
                            tardiness, workload, bay_loads, pb_id,
                            s_max - prefs[pb_id], bay_weights, w1, w2, w3,
                            top_y=py + p_bb[3]
                        )
                        best_score_unshielded     = score
                        best_placement_unshielded = (pb_id, px, py, poi, entry, exit_t)

                        if SHIELD_ENABLED:
                            active_prev = [
                                b for b, (a_k, e_k) in zip(bay_placed[pb_id],
                                                             bay_schedule[pb_id])
                                if e_k > r_time
                            ]
                            if not _shielding_violated(prev_blk, bays[pb_id],
                                                       active_prev, due, blocks_data):
                                best_score_shielded     = score
                                best_placement_shielded = (pb_id, px, py, poi, entry, exit_t)

            # -- Full search (Phase-1 style) -----------------------------------
            bay_order = sorted(range(n_bays), key=lambda j: prefs[j], reverse=True)
            for bay_id in bay_order:
                bay             = bays[bay_id]
                placed_in_bay   = bay_placed[bay_id]
                schedule_in_bay = bay_schedule[bay_id]

                for oi in range(n_orient):
                    blk_bb = _block_bbox(blk_data, oi)
                    lx0_oi, ly0_oi, lx1_oi, ly1_oi = blk_bb
                    # Require a valid integer reference-point position to exist:
                    #   px in [ceil(-lx0), floor(W - lx1)]
                    #   py in [ceil(-ly0), floor(H - ly1)]
                    # If either range is empty there is no integer placement.
                    if (math.ceil(-lx0_oi) > math.floor(bay.width  - lx1_oi) or
                            math.ceil(-ly0_oi) > math.floor(bay.height - ly1_oi)):
                        continue

                    # active_in_bay/active_schedule = blocks (and their
                    # (entry,exit) pairs) still present at r_time.  Built
                    # ONCE per (bay, orient) here and passed into
                    # _find_earliest_slot below instead of the full
                    # placed_in_bay/schedule_in_bay (myalgorithm4 change --
                    # "idea 1" perf fix).  Without this, _find_earliest_slot
                    # re-derives the same active-only filter internally on
                    # EVERY one of the up-to-K_BEST calls in the loop below,
                    # rescanning already-exited blocks that accumulate in
                    # placed_in_bay over the course of Phase 1 every single
                    # time.  Passing the pre-filtered lists keeps
                    # _find_earliest_slot's own logic completely unchanged
                    # (a<entry<e filtering still applies, just over a
                    # smaller, already-live-only list) -- purely a caller-side
                    # redundant-work removal, no behavioural difference.
                    active_in_bay:  list[Block] = []
                    active_schedule: list[tuple[int, int]] = []
                    for b, (a_k, e_k) in zip(placed_in_bay, schedule_in_bay):
                        if e_k > r_time:
                            active_in_bay.append(b)
                            active_schedule.append((a_k, e_k))
                    candidates = _candidate_positions(
                        bay.width, bay.height, active_in_bay, blk_bb
                    )

                    # -- AABB 즉시진입 우선정렬 (myalgorithm6 신규) -----------
                    # K-best는 후보를 bottom-left (x,y) 순으로 훑다가 "처음
                    # 만난 feasible K개"에서 멈춘다.  넓은 베이(prob_25
                    # bay0=151폭)에서 왼쪽이 혼잡하면 K개가 전부 "들어갈 수는
                    # 있지만 entry가 늦은" 후보로 차버리고, 오른쪽의 "지금
                    # 당장 들어갈 수 있는" 자리는 생성조차 안 된다.  실측
                    # (prob_25): 지각 블록 62개 중 23개는 release 시점에
                    # bbox-서로소 자리가 실재했는데도 평균 40씩 늦게 진입.
                    #
                    # 즉시진입 창 [r_time, r_time+proc)과 시간이 겹치는 모든
                    # 블록과 bbox가 서로소인 위치는 AABB 프리필터에 의해
                    # 구조적으로 crane-feasible이 보장되고(_force_place와 동일
                    # 원리) _find_earliest_slot이 entry=r_time을 즉시 반환한다.
                    # 이런 위치를 순수 정수 연산으로 골라 후보 앞으로 보내면
                    # (안정 정렬: 그룹 내 bottom-left 순서 유지) K-best가
                    # "즉시 진입" 후보부터 채워진다.  부수 효과로 Phase 1도
                    # 빨라진다: 혼잡할수록 첫 probe들이 "성공이 보장된 싼
                    # 호출"이 되어 실패 스캔(entry 후보 수십 개 Shapely 검사)
                    # 이 줄기 때문.
                    # -> 실측: prob_25 obj1 2428->1488, prob_38 11393->8484.
                    #
                    # *** 기각된 개선 시도(적응형 게이트): 중간부하 인스턴스
                    # (prob_21/23/24)에서 이 정렬이 공간 파편화로 소폭 악화를
                    # 일으켜, "기존 순서 첫 후보를 1개 probe해서 지각 0이면
                    # 기존 순서 유지, 지각이면 즉시진입 정렬"하는 블록단위
                    # 게이트를 시도했으나 실측 결과 전 인스턴스에서 오히려
                    # 조금씩 더 나빠져 되돌렸다.  원인: (1) 게이트 probe가
                    # (베이x방향)마다 혼잡한 bottom-left 자리를 강제로 먼저
                    # 평가 -- 이는 가장 비싼 종류의 실패 호출이라 Phase 1이
                    # 느려지고 time guard 강제배치가 늘었다 (위의 속도 이득
                    # 상실).  (2) 파편화를 일으키는 주체가 정확히 "지각 위기
                    # 블록"인데 게이트는 바로 그 블록들에서 즉시진입을 켜므로
                    # 회귀도 해소되지 않았다.
                    win_bbs = [
                        b.bounding_rect()
                        for b, (a_k, e_k) in zip(active_in_bay, active_schedule)
                        if a_k < r_time + proc and e_k > r_time
                    ]
                    if win_bbs and candidates:
                        immediate: list[tuple[int, int]] = []
                        rest:      list[tuple[int, int]] = []
                        for (cx, cy) in candidates:
                            wbb = (cx + lx0_oi, cy + ly0_oi,
                                   cx + lx1_oi, cy + ly1_oi)
                            if all(not _bb_overlap(wbb, ab) for ab in win_bbs):
                                immediate.append((cx, cy))
                            else:
                                rest.append((cx, cy))
                        candidates = immediate + rest

                    # -- K-best collection: stop calling the expensive
                    # _find_earliest_slot (check_entry/check_exit) once
                    # K_BEST feasible candidates are found, instead of
                    # exhausting every candidate position.
                    feasible_k: list[tuple[int, int, int, int]] = []
                    for (cx, cy) in candidates:
                        if K_BEST > 0 and len(feasible_k) >= K_BEST:
                            break
                        new_blk = Block(block_id=bi, block_data=blk_data,
                                        x=cx, y=cy, orient_idx=oi)
                        if not bay.contains_block(new_blk):
                            continue

                        entry, exit_t = _find_earliest_slot(
                            new_blk, bay, active_in_bay, active_schedule,
                            r_time, proc
                        )
                        if entry is None:
                            continue
                        feasible_k.append((cx, cy, entry, exit_t))

                    # -- Score only the K collected candidates ---------------
                    for (cx, cy, entry, exit_t) in feasible_k:
                        new_blk = Block(block_id=bi, block_data=blk_data,
                                        x=cx, y=cy, orient_idx=oi)
                        tardiness = max(0.0, exit_t - due)
                        score = _placement_score(
                            tardiness, workload, bay_loads, bay_id,
                            s_max - prefs[bay_id], bay_weights, w1, w2, w3,
                            top_y=cy + blk_bb[3]
                        )
                        if score < best_score_unshielded:
                            best_score_unshielded     = score
                            best_placement_unshielded = (bay_id, cx, cy, oi, entry, exit_t)

                        if (SHIELD_ENABLED and score < best_score_shielded
                                and not _shielding_violated(new_blk, bay, active_in_bay,
                                                            due, blocks_data)):
                            best_score_shielded     = score
                            best_placement_shielded = (bay_id, cx, cy, oi, entry, exit_t)

        best_placement = (best_placement_shielded if best_placement_shielded is not None
                          else best_placement_unshielded)
        best_score     = (best_score_shielded if best_placement_shielded is not None
                          else best_score_unshielded)

        if best_placement is None:
            best_placement = _force_place(bi, blocks_data, bays, bay_placed, bay_schedule, prefs)
            n_fallback += 1

        if used_forced:
            n_forced += 1

        bay_id, cx, cy, oi, entry, exit_t = best_placement
        final_blk = Block(block_id=bi, block_data=blk_data, x=cx, y=cy, orient_idx=oi)
        bay_placed[bay_id].append(final_blk)
        bay_schedule[bay_id].append((entry, exit_t))
        bay_loads[bay_id] += workload

        result[bi] = {
            "block_id":   bi,
            "bay_id":     bay_id,
            "x":          int(round(cx)),
            "y":          int(round(cy)),
            "orient_idx": oi,
            "entry_time": int(round(entry)),
            "exit_time":  int(round(exit_t)),
        }

        if log_interval > 0 and t_start is not None:
            n_done = rank + 1
            if n_done % log_interval == 0 or n_done == n_total:
                elapsed = time.time() - t_start
                loads_str = " ".join(f"b{i}={round(bay_loads[i])}" for i in range(n_bays))
                flag = " [forced]" if used_forced else (" [fallback]" if best_score == float("inf") else "")
                print(f"[Greedy]   {n_done:4d}/{n_total}"
                      f"  block{bi:<4d} -> bay{bay_id} ({cx},{cy}) oi={oi}"
                      f"  t=[{int(round(entry))},{int(round(exit_t))})"
                      f"  loads=[{loads_str}]"
                      f"  fallback={n_fallback}{flag}"
                      f"  {elapsed:.1f}s")

    return result


# -----------------------------------------------------------------------------
# Phase 2: repair infeasible blocks
# -----------------------------------------------------------------------------

def _repair(prob_info: dict,
            sol: dict,
            assignments: dict[int, dict],
            bays: list[Bay],
            blocks_data: list[dict],
            w1: float, w2: float, w3: float,
            t_start: float,
            timelimit: float,
            max_passes: int = 10,
            repair_mode: str = "greedy") -> dict[int, dict]:
    """
    Iteratively detect infeasible blocks and repair them.

    Runs up to max_passes rounds of: check_feasibility -> collect violating
    block ids -> re-place them.  Stops early if the solution becomes feasible
    or 98% of timelimit is consumed.

    -- repair_mode="greedy" (default) ------------------------------------------
    Violating blocks are removed from assignments and re-placed using the full
    Phase-1 search (all bays, orientations, positions, time-slots).  The state
    arrays (bay_placed, bay_schedule, bay_loads) are reconstructed from the
    remaining non-violating assignments before each block is re-placed, so the
    search sees the current bay state.

    Cycle detection:
      repaired_counts[bid] tracks how many repair passes have touched block bid.
      If bid appears in a second pass (count > 1) it is added to forced_ids.
      Blocks in forced_ids skip search and go straight to _force_place (empty-
      bay window), which is structurally guaranteed to produce a crane-feasible
      placement.  This breaks cycles where two blocks keep displacing each other.

    Time guard (90% threshold):
      For each block in to_repair, if wall-clock time > 90% of timelimit before
      its turn, it is added to forced_ids.  This ensures all blocks are assigned
      before timeout rather than leaving some unassigned (Stage-1 failure).

    -- repair_mode="simple" -----------------------------------------------------
    Each violating block keeps its current (bay, x, y, orient) and is only
    pushed to the next empty-bay time window via _empty_bay_entry.  Stage-4
    violations (spatial collision) are also reset to position (0, 0).
    Faster than greedy mode, but cannot improve spatial placement quality.

    Parameters
    ----------
    prob_info   : instance JSON dict
    sol         : current solution dict (operations format)
    assignments : current assignment dict (block_id -> assignment dict)
    bays        : Bay objects
    blocks_data : raw block data from prob_info
    w1,w2,w3    : objective weights
    t_start     : wall-clock start time
    timelimit   : total wall-clock time limit
    max_passes  : maximum number of repair iterations
    repair_mode : "greedy" or "simple"

    Returns
    -------
    Updated assignments dict (all blocks assigned)
    """
    from utils import check_feasibility

    # -- repair 전용 K_BEST 상한 (구조적 안전장치, 인스턴스 분류와 무관) -------
    # Phase 1이 어떤 K_BEST로 끝났든, repair는 항상 붐비는 상태에서 도니까
    # 항상 캡을 씌운다.  AUTO_DISPATCH=False(수동 --k-best 테스트)일 때는
    # 사용자가 명시한 값을 존중해 건드리지 않는다.
    global K_BEST
    if AUTO_DISPATCH:
        K_BEST = K_BEST_REPAIR

    repaired_counts: dict[int, int] = {}
    forced_ids:      set[int]       = set()

    for pass_idx in range(max_passes):
        if time.time() - t_start > timelimit * 0.98:
            break

        result = check_feasibility(prob_info, sol)
        if result["feasible"]:
            break

        viols = result["violations"]
        elapsed_r = time.time() - t_start
        print(f"[Greedy] Repair pass {pass_idx+1}: {len(viols)} violation(s)  "
              f"stage={result['stage']}  elapsed={elapsed_r:.1f}s")

        # -- Parse block ids from violation messages ---------------------------
        # Each violation string contains "block <id>" somewhere in the text.
        # Deduplicate while preserving first-occurrence order.
        to_repair: list[int] = []
        seen: set[int] = set()
        for v in viols:
            try:
                bid = int(v.split("block ")[1].split()[0])
                if bid not in seen:
                    seen.add(bid)
                    to_repair.append(bid)
            except (IndexError, ValueError):
                pass

        if not to_repair:
            break

        # Re-place in EDD order so earlier-due blocks get the best slots first
        to_repair.sort(key=lambda b: (blocks_data[b]["due_date"],
                                      blocks_data[b]["processing_time"]))
        n_repl = len(to_repair)

        if repair_mode == "simple":
            # -- Simple mode: adjust only the time window, keep position/orient -
            # Rebuild the per-bay time schedule from all current assignments so
            # that _empty_bay_entry can find a gap with no other blocks present.
            n_bays = len(bays)
            bay_schedule: list[list[tuple[int, int]]] = [[] for _ in range(n_bays)]
            for a in assignments.values():
                bay_schedule[a["bay_id"]].append((a["entry_time"], a["exit_time"]))

            for ri, bid in enumerate(to_repair):
                a      = assignments[bid]
                bay_id = a["bay_id"]
                r_time = blocks_data[bid]["release_time"]
                proc   = blocks_data[bid]["processing_time"]

                # Remove the block's current slot before searching for a new one
                old_slot = (a["entry_time"], a["exit_time"])
                if old_slot in bay_schedule[bay_id]:
                    bay_schedule[bay_id].remove(old_slot)

                entry  = _empty_bay_entry(bay_schedule[bay_id], r_time, proc)
                exit_t = entry + proc

                # Stage-4 (spatial collision): also reset position to (0,0)
                # to eliminate any spatial overlap with other blocks
                x, y, oi = a["x"], a["y"], a["orient_idx"]
                if result["stage"] == 4:
                    x, y = 0, 0

                assignments[bid] = dict(a, x=x, y=y, orient_idx=oi,
                                        entry_time=int(round(entry)),
                                        exit_time=int(round(exit_t)))
                bay_schedule[bay_id].append((entry, exit_t))

                prev_t = f"[{a['entry_time']},{a['exit_time']})"
                new_t  = f"[{entry},{exit_t})"
                tag    = "[s4->(0,0)]" if result["stage"] == 4 else "[time]"
                elapsed_ri = time.time() - t_start
                print(f"[Greedy]   repair {ri+1:3d}/{n_repl}"
                      f"  block{bid:<4d} {tag}"
                      f"  bay{bay_id} ({int(x)},{int(y)})"
                      f"  {prev_t} -> {new_t}"
                      f"  elapsed={elapsed_ri:.1f}s")

        else:
            # -- Greedy mode: full Phase-1 re-search for violating blocks ------
            # Mark repeat offenders as forced before touching assignments, so
            # the flag is active when _place_blocks processes them below.
            for bid in to_repair:
                repaired_counts[bid] = repaired_counts.get(bid, 0) + 1
                if repaired_counts[bid] > 1:
                    forced_ids.add(bid)

            # Remove violating blocks from assignments so the state reconstruction
            # below does not include their (now invalid) positions/slots.
            for bid in to_repair:
                assignments.pop(bid, None)

            # Reconstruct bay_placed / bay_schedule / bay_loads from the
            # remaining valid assignments.  This gives _place_blocks an accurate
            # view of which positions and time-slots are already occupied.
            n_bays = len(bays)
            bay_placed:    list[list[Block]]            = [[] for _ in range(n_bays)]
            bay_schedule2: list[list[tuple[int, int]]]  = [[] for _ in range(n_bays)]
            bay_loads:     list[float]                  = [0.0] * n_bays

            for a in assignments.values():
                bid_a  = a["block_id"]
                bay_id = a["bay_id"]
                blk    = Block(block_id=bid_a, block_data=blocks_data[bid_a],
                               x=int(a["x"]), y=int(a["y"]), orient_idx=a["orient_idx"])
                bay_placed[bay_id].append(blk)
                bay_schedule2[bay_id].append((a["entry_time"], a["exit_time"]))
                bay_loads[bay_id] += blocks_data[bid_a]["workload"]

            for ri, bi in enumerate(to_repair):
                # Time guard: switch to forced path when 90% of timelimit is used.
                # Without this, a slow repair search could exhaust the timelimit
                # before all blocks are placed, causing Stage-1 (assignment) failures.
                time_critical = time.time() - t_start > timelimit * 0.90
                if time_critical:
                    forced_ids.add(bi)
                prev_a  = assignments.get(bi)
                tag_extra = ""

                if bi in forced_ids and not time_critical:
                    # -- Destroy/repair (ALNS-style), replacing blind force ---
                    # bi is a repeat-violator, but instead of unconditionally
                    # dumping it into an empty-bay window (baseline_greedy.py
                    # behaviour -- safe but can inflate tardiness a lot), give
                    # it one legitimate probe search first.  If that pick still
                    # violates due-date shielding, "destroy" the one specific
                    # block causing the violation and re-place BOTH of them
                    # normally.  Bounded to a single blocker, so this can never
                    # cascade/recurse -- if the probe genuinely finds nothing,
                    # _place_blocks' own internal _force_place fallback (same
                    # guaranteed-safe empty-bay logic as baseline) still applies,
                    # so this is never less safe than the original behaviour.
                    probe_forced = forced_ids - {bi}
                    partial = _place_blocks(
                        [bi], blocks_data, bays,
                        bay_placed, bay_schedule2, bay_loads,
                        w1, w2, w3, probe_forced,
                        prev_assignments=assignments,
                    )
                    new_a  = partial[bi]
                    bay_id = new_a["bay_id"]
                    final_blk = bay_placed[bay_id][-1]

                    blocker = None
                    if SHIELD_ENABLED:
                        r_time_bi = blocks_data[bi]["release_time"]
                        active_excl = [
                            b for b, (a_k, e_k) in zip(bay_placed[bay_id], bay_schedule2[bay_id])
                            if e_k > r_time_bi and b.block_id != bi
                        ]
                        blocker = _find_shielding_blocker(
                            final_blk, bays[bay_id], active_excl,
                            blocks_data[bi]["due_date"], blocks_data
                        )

                    if blocker is not None:
                        blocker_id = blocker.block_id
                        _pop_block_from_bay(bay_placed, bay_schedule2, bay_loads,
                                           blocks_data, bay_id, bi)
                        _pop_block_from_bay(bay_placed, bay_schedule2, bay_loads,
                                           blocks_data, bay_id, blocker_id)
                        assignments.pop(blocker_id, None)

                        partial_bi = _place_blocks(
                            [bi], blocks_data, bays,
                            bay_placed, bay_schedule2, bay_loads,
                            w1, w2, w3, probe_forced,
                            prev_assignments=assignments,
                        )
                        partial_blocker = _place_blocks(
                            [blocker_id], blocks_data, bays,
                            bay_placed, bay_schedule2, bay_loads,
                            w1, w2, w3, forced_ids - {blocker_id},
                            prev_assignments=assignments,
                        )
                        partial   = {**partial_bi, **partial_blocker}
                        new_a     = partial_bi[bi]
                        tag_extra = f" [destroy+repair w/block{blocker_id}]"
                    else:
                        tag_extra = " [forced-probe-ok]"

                    assignments.update(partial)
                else:
                    partial = _place_blocks(
                        [bi], blocks_data, bays,
                        bay_placed, bay_schedule2, bay_loads,
                        w1, w2, w3, forced_ids,
                        prev_assignments=assignments,
                    )
                    assignments.update(partial)
                    new_a = partial[bi]

                is_forced   = bi in forced_ids
                changed_bay = prev_a and prev_a["bay_id"] != new_a["bay_id"]
                changed_pos = prev_a and (prev_a["x"] != new_a["x"]
                                          or prev_a["y"] != new_a["y"])
                tag = ("[forced]" if is_forced
                       else "[bay]" if changed_bay
                       else "[pos]" if changed_pos
                       else "[time]") + tag_extra
                prev_t = (f"[{int(prev_a['entry_time'])},{int(prev_a['exit_time'])})"
                          if prev_a else "N/A")
                new_t  = f"[{int(new_a['entry_time'])},{int(new_a['exit_time'])})"
                elapsed_ri = time.time() - t_start
                print(f"[Greedy]   repair {ri+1:3d}/{n_repl}"
                      f"  block{bi:<4d} {tag}"
                      f"  bay{new_a['bay_id']} ({int(new_a['x'])},{int(new_a['y'])})"
                      f"  {prev_t} -> {new_t}"
                      f"  elapsed={elapsed_ri:.1f}s")

        sol = {"operations": _build_operations(list(assignments.values()), prob_info)}

    result = check_feasibility(prob_info, sol)
    status = "feasible" if result["feasible"] else f"INFEASIBLE stage={result['stage']}"
    obj    = f"obj={result['objective']:.0f}" if result["feasible"] else ""
    forced_note = f"  forced={len(forced_ids)}" if forced_ids else ""
    elapsed_done = time.time() - t_start
    print(f"[Greedy] Repair done  |  {status}  {obj}{forced_note}  elapsed={elapsed_done:.1f}s")

    return assignments


# -----------------------------------------------------------------------------
# Build operations dict from assignments
# -----------------------------------------------------------------------------

def _topo_sort_bay_entries(
    bay_entries: list[dict],
    bay:         Bay,
    blocks_data: list[dict],
    pre_present: list[Block],
) -> list[dict]:
    """
    Sort ENTRY ops for one bay at one time step so that each block's crane
    descent is feasible given the blocks that have already entered in this
    same time step.  (Ported from myalgorithm.py -- this is the fix for the
    root cause found while investigating why destroy/repair wasn't helping:
    same-timestamp co-entries were ordered by raw block_id, which can put an
    obstructing block "in the bay" before the block it blocks, producing a
    pure ordering violation that has nothing to do with placement quality.)

    Dependency rule: if block B's presence would obstruct block A's crane
    descent (check_entry with B in present fails for A), then A must enter
    before B  (A -> B directed edge).

    pre_present: Block objects already in the bay from earlier time steps.

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

    adj   = [[] for _ in range(n)]
    indeg = [0]  * n

    for a_idx in range(n):
        for b_idx in range(n):
            if a_idx == b_idx:
                continue
            present_test = pre_present + [blks[b_idx]]
            if check_entry(bay, present_test, blks[a_idx], fast=True):
                adj[a_idx].append(b_idx)
                indeg[b_idx] += 1

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
    (Ported from myalgorithm.py -- see _topo_sort_bay_entries docstring.)

    Dependency rule: if block B's body obstructs block A's crane ascent
    (check_exit with B present returns a violation), then B must exit before A
    (B -> A directed edge).

    Uses Kahn's topological sort.  Falls back to LIFO (latest entry first) on
    a dependency cycle (mutual obstruction).
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

    adj   = [[] for _ in range(n)]
    indeg = [0]  * n

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            if check_exit(bay, [blks[i], blks[j]], blks[i], fast=True):
                adj[j].append(i)
                indeg[i] += 1

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
        return sorted(bay_exits,
                      key=lambda op: (-op["_entry_time"], -op["block_id"]))

    return [bay_exits[k] for k in result]


def _build_operations(assignments: list[dict],
                      prob_info:   dict | None = None) -> dict:
    """
    Build the "operations" dict from a flat list of assignment dicts.

    EXIT ops always precede ENTRY ops at the same time point.

    When prob_info is provided (myalgorithm4 change vs baseline_greedy.py):
      - Same-time EXIT ops in the same bay are topologically sorted so that
        each block's crane ascent is not obstructed by a block that hasn't
        yet exited (Stage-5 EXIT ordering safety).
      - Same-time ENTRY ops in the same bay are topologically sorted so that
        if block B's presence would obstruct block A's crane descent, A
        enters before B (Stage-5 ENTRY ordering safety).
    Without prob_info: falls back to the original baseline_greedy.py
    behaviour (LIFO exit order, block_id entry order) -- kept for any caller
    that doesn't have prob_info handy.
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

    bays_objs:   list[Bay] | None       = None
    blocks_data: list[dict] | None      = None
    asgn_map:    dict[int, dict] | None = None
    if prob_info is not None:
        blocks_data = prob_info["blocks"]
        bays_objs   = [Bay.from_dict(prob_info["bays"][j], j)
                       for j in range(len(prob_info["bays"]))]
        asgn_map    = {a["block_id"]: a for a in assignments}

    bay_present: dict[int, set[int]] = (
        {j: set() for j in range(len(prob_info["bays"]))}
        if prob_info is not None else {}
    )

    all_times  = sorted(set(exit_by_time) | set(entry_by_time))
    operations: dict[str, list[dict]] = {}

    for t in all_times:
        exits_t   = exit_by_time.get(t, [])
        entries_t = entry_by_time.get(t, [])

        # -- Sort EXIT ops --
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

        for op in sorted_exits:
            bay_present.get(op["bay_id"], set()).discard(op["block_id"])

        # -- Sort ENTRY ops --
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

        for op in sorted_entries:
            bay_present.get(op["bay_id"], set()).add(op["block_id"])

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


# -----------------------------------------------------------------------------
# CLI run
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import json
    import pathlib
    from collections import defaultdict
    from utils import check_feasibility

    parser = argparse.ArgumentParser(description="myalgorithm4 (due-date shielding) smoke test")
    parser.add_argument("instance", help="path to instance JSON file")
    parser.add_argument("--timelimit", type=float, default=60.0,
                        help="wall-clock time limit in seconds (default: %(default)s)")
    parser.add_argument("--repair", choices=["greedy", "simple"], default="greedy",
                        help="repair mode (default: %(default)s)")
    parser.add_argument("--no-shield", action="store_true",
                        help="disable SHIELD_ENABLED (falls back to pure baseline_greedy behaviour)")
    parser.add_argument("--k-best", type=int, default=None,
                        help="force K_BEST to this value, DISABLING AUTO_DISPATCH "
                             "(0 = unlimited).  Without this flag, AUTO_DISPATCH "
                             "would silently override any K_BEST you set.")
    parser.add_argument("--k-best-scale-const", type=float, default=None,
                        help="override K_BEST_SCALE_CONST (K_BEST = this / blocks_per_bay) "
                             "without disabling dispatch")
    parser.add_argument("--max-entry-tries", type=int, default=None,
                        help="override MAX_ENTRY_TRIES (0 = unlimited)")
    args = parser.parse_args()

    # NOTE: do NOT `import myalgorithm4 as _self` here -- when this file runs
    # as __main__, that would create a SEPARATE module instance from the one
    # algorithm()/_shielding_violated() actually read, silently no-op'ing the
    # override.  Rebinding the module-level name directly (this block is
    # top-level code in __main__, not inside a function) works correctly.
    if args.no_shield:
        SHIELD_ENABLED = False
    if args.k_best is not None:
        # AUTO_DISPATCH would otherwise clobber this every algorithm() call --
        # disable it so a direct --k-best override actually sticks.
        AUTO_DISPATCH = False
        K_BEST = args.k_best
    if args.k_best_scale_const is not None:
        K_BEST_SCALE_CONST = args.k_best_scale_const
    if args.max_entry_tries is not None:
        MAX_ENTRY_TRIES = args.max_entry_tries

    inst_file = pathlib.Path(args.instance)

    with open(inst_file) as f:
        prob_info = json.load(f)

    t0  = time.time()
    sol = algorithm(prob_info, timelimit=args.timelimit, repair_mode=args.repair)
    elapsed = time.time() - t0

    result = check_feasibility(prob_info, sol)

    n_assigned = sum(1 for ops in sol["operations"].values()
                     for op in ops if op["type"] == "ENTRY")
    print(f"Instance : {prob_info['name']}")
    print(f"Elapsed  : {elapsed:.3f}s")
    print(f"Assigned : {n_assigned} / {len(prob_info['blocks'])} blocks")
    print(f"Feasible : {result['feasible']}  (stage={result['stage']})")
    if result["feasible"]:
        print(f"Objective: {result['objective']:.2f}  "
              f"(obj1={result['obj1']:.1f}, obj2={result['obj2']:.1f}, obj3={result['obj3']:.1f})")
    else:
        for v in result["violations"][:10]:
            print(f"  VIOLATION: {v}")
