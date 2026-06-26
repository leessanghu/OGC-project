"""
myalgorithm2.py — OGC 2026 : baseline_greedy.py 기반 개선 알고리즘
==========================================================================

baseline_greedy.py를 그대로 복사한 후 아래 세 가지만 변경:
  A. _candidate_positions 세 가지 변형 (v0/v1/v2) — CANDIDATE_MODE로 선택
  B. _placement_score 에 B(x,y) 블로킹 항 추가 — W5 상수로 가중치 조정
  C. K-best 후보 선택 — K_BEST 상수로 조정

변경되지 않은 부분:
  - EDD/release_time 정렬 순서
  - repair 패스 로직 (greedy/simple 모드, cycle 감지, force-place, time-guard)
  - _find_earliest_slot 의 time-slot 탐색 로직
  - _build_operations, feasibility-check 통합, checkpoint/fallback 동작
  - 모든 weight 처리 (w1/w2/w3) 및 전체 objective 구조

입력/출력 계약:
  algorithm(prob_info, timelimit=60) -> dict
  반환 형식: {"operations": {...}}  (utils.check_feasibility 통과 보장)
"""

import math
import time
from utils import Bay, Block, check_entry, check_exit, check_collisions, _resolve_layers, _bounding_box


# ==========================================================================
# 조정 가능한 상수 (코드 수정 없이 동작 변경)
# ==========================================================================

# 후보 위치 생성 모드:
#   "v0" = 원본 AABB 기반 (baseline과 동일)
#   "v1" = 레이어별 bounding box 기반 (Step 1)
#   "v2" = 다각형 꼭짓점 기반 touching corners (Step 2)
CANDIDATE_MODE: str = "v1"

# K-best : (bay, orientation) 쌍당 평가할 최대 실행 가능 후보 수
K_BEST: int = 6

# B(x,y) 블로킹 항의 가중치 (w1 대비 매우 작게 유지)
W5: float = 1e-3


# ==========================================================================
# Helper: 블록 bounding box (앵커 기준, 방향별)
# ==========================================================================

def _block_bbox(block_data: dict, orient_idx: int) -> tuple[float, float, float, float]:
    """블록의 로컬 좌표 bounding box (참조점 = 첫 번째 레이어 첫 꼭짓점 = (0,0)).
    반환: (min_x, min_y, max_x, max_y)"""
    raw_layers = block_data["shape"][orient_idx]["layers"]
    layers = _resolve_layers(raw_layers)
    if not layers:
        return (0.0, 0.0, 1.0, 1.0)
    all_verts = [v for l in layers for v in l]
    return _bounding_box(all_verts)


# ==========================================================================
# Helper: 시간 구간 겹침 확인
# ==========================================================================

def _time_overlaps(a_entry: int, a_exit: int,
                   b_entry: int, b_exit: int) -> bool:
    """[a_entry, a_exit) 와 [b_entry, b_exit) 가 겹치면 True."""
    return a_entry < b_exit and b_entry < a_exit


# ==========================================================================
# 후보 위치 생성 — 세 가지 변형
# ==========================================================================

def _candidate_positions_v0(bay_w: float, bay_h: float,
                             placed_blocks: list[Block],
                             blk_bb: tuple[float, float, float, float]) -> list[tuple[int, int]]:
    """
    v0 (원본 baseline): 전체 블록 AABB의 우측/상단 에지를 후보 x/y로 사용.

    placed_blocks 각각의 bounding_rect() 전체 bbox 기준:
      x 후보 = 각 블록 bbox 우측 에지 (bb[2])
      y 후보 = 각 블록 bbox 상단 에지 (bb[3])
    cross-product 후 베이 경계 내 필터링.
    """
    lx0, ly0, lx1, ly1 = blk_bb
    xs = {max(0, math.ceil(-lx0))}
    ys = {max(0, math.ceil(-ly0))}
    for b in placed_blocks:
        bb = b.bounding_rect()
        xs.add(math.ceil(bb[2] - lx0))
        ys.add(math.ceil(bb[3] - ly0))

    candidates = []
    for x in sorted(xs):
        for y in sorted(ys):
            if x + lx1 <= bay_w + 1e-6 and y + ly1 <= bay_h + 1e-6:
                candidates.append((int(x), int(y)))
    return candidates


def _candidate_positions_v1(bay_w: float, bay_h: float,
                             placed_blocks: list[Block],
                             blk_bb: tuple[float, float, float, float]) -> list[tuple[int, int]]:
    """
    v1 (Step 1): 레이어별 bounding box의 우측/상단 에지를 후보로 사용.

    전체 블록의 단일 bbox 대신, 각 레이어의 세계 좌표 bbox를 개별 계산.
    블록이 단일 레이어면 v0과 수치적으로 동일.
    계단형/다층 블록의 경우 v0보다 더 타이트한 후보를 생성.

      x 후보 = 모든 배치 블록의 각 레이어 bbox 우측 에지
      y 후보 = 모든 배치 블록의 각 레이어 bbox 상단 에지
    """
    lx0, ly0, lx1, ly1 = blk_bb
    xs = {max(0, math.ceil(-lx0))}
    ys = {max(0, math.ceil(-ly0))}

    for b in placed_blocks:
        for layer in b.layers_at_pos():
            if not layer:
                continue
            x_coords = [v[0] for v in layer]
            y_coords = [v[1] for v in layer]
            layer_x2 = max(x_coords)
            layer_y2 = max(y_coords)
            xs.add(math.ceil(layer_x2 - lx0))
            ys.add(math.ceil(layer_y2 - ly0))

    candidates = []
    for x in sorted(xs):
        for y in sorted(ys):
            if x + lx1 <= bay_w + 1e-6 and y + ly1 <= bay_h + 1e-6:
                candidates.append((int(x), int(y)))
    return candidates


def _candidate_positions_v2(bay_w: float, bay_h: float,
                             placed_blocks: list[Block],
                             blk_bb: tuple[float, float, float, float]) -> list[tuple[int, int]]:
    """
    v2 (Step 2): 실제 다각형 꼭짓점 기반 touching corners.

    배치된 블록의 각 레이어 다각형 꼭짓점 (세계 좌표)에서 후보 생성:
      x 후보: 새 블록의 왼쪽 에지가 꼭짓점 x에 닿는 위치 (ceil(vx - lx0))
      y 후보: 새 블록의 하단 에지가 꼭짓점 y에 닿는 위치 (ceil(vy - ly0))

    NFP 근사 (전체 NFP 계산은 아님) — 실제 유효성은 기존 충돌 검사가 담당.
    v1보다 더 많은 후보를 생성하므로 연산 비용이 높을 수 있음.
    """
    lx0, ly0, lx1, ly1 = blk_bb
    xs = {max(0, math.ceil(-lx0))}
    ys = {max(0, math.ceil(-ly0))}

    for b in placed_blocks:
        for layer in b.layers_at_pos():
            if not layer:
                continue
            for vx, vy in layer:
                # 새 블록의 왼쪽 에지(x + lx0)가 꼭짓점 vx에 닿는 참조점 x
                cand_x = math.ceil(vx - lx0)
                if cand_x >= 0:
                    xs.add(cand_x)
                # 새 블록의 하단 에지(y + ly0)가 꼭짓점 vy에 닿는 참조점 y
                cand_y = math.ceil(vy - ly0)
                if cand_y >= 0:
                    ys.add(cand_y)

    candidates = []
    for x in sorted(xs):
        for y in sorted(ys):
            if x + lx1 <= bay_w + 1e-6 and y + ly1 <= bay_h + 1e-6:
                candidates.append((int(x), int(y)))
    return candidates


def _candidate_positions(bay_w: float, bay_h: float,
                          placed_blocks: list[Block],
                          blk_bb: tuple[float, float, float, float]) -> list[tuple[int, int]]:
    """
    CANDIDATE_MODE 상수에 따라 v0/v1/v2 중 하나를 호출하는 디스패처.

    호출부 계약 (v0/v1/v2 모두 동일):
      - 입력: (bay_w, bay_h, placed_blocks, blk_bb)
      - 출력: 정수 (x, y) 튜플 리스트, (x, y) 오름차순 정렬
    """
    if CANDIDATE_MODE == "v2":
        return _candidate_positions_v2(bay_w, bay_h, placed_blocks, blk_bb)
    elif CANDIDATE_MODE == "v1":
        return _candidate_positions_v1(bay_w, bay_h, placed_blocks, blk_bb)
    else:  # "v0" or default
        return _candidate_positions_v0(bay_w, bay_h, placed_blocks, blk_bb)


# ==========================================================================
# 배치 점수 (낮을수록 좋음)
# ==========================================================================

def _placement_score(tardiness: float, workload: float,
                     bay_loads: list[float], bay_id: int,
                     pref_penalty: float,
                     bay_weights: list[float],
                     w1: float, w2: float, w3: float,
                     top_y: float = 0.0, w4: float = 1e-4,
                     blocking: float = 0.0) -> float:
    """
    블록을 bay_id에 배치할 때의 복합 점수 (낮을수록 좋음).

      w1 * tardiness    — 지연 시간: max(0, exit_time - due_date)
      w2 * new_obj2     — 정규화된 부하 균형 페널티 근사값
      w3 * pref_penalty — 선호도 페널티: S_i_max - S_i_bay_id
      w4 * top_y        — 타이브레이킹: 상단 에지 높이 (낮을수록 타이트한 패킹)
      W5 * blocking     — B(x,y) 블로킹 휴리스틱:
                          새 블록 앞에 있는(x 방향 입구 쪽) y-range 겹침 블록 수
    """
    new_load = bay_loads[bay_id] + workload
    new_obj2 = max(
        (abs(bay_weights[bay_id] * new_load - bay_weights[j] * bay_loads[j])
         for j in range(len(bay_loads)) if j != bay_id),
        default=0.0
    )
    return (w1 * tardiness + w2 * new_obj2 + w3 * pref_penalty
            + w4 * top_y + W5 * blocking)


# ==========================================================================
# 최조 실현 가능 진입 슬롯 (적극적 — 시간 겹침 허용)
# ==========================================================================

def _find_earliest_slot(new_blk: Block,
                        bay: Bay,
                        placed_in_bay: list[Block],
                        schedule_in_bay: list[tuple[int, int]],
                        r_time: int,
                        proc: int) -> tuple[int | None, int | None]:
    """
    r_time 이상의 가장 이른 (entry, exit_t) 시간 슬롯 반환.
    크레인 경로 Stage-2 (진입) 및 Stage-3 (퇴출) 검사 통과 필요.
    불가능하면 (None, None) 반환.
    """
    candidate_entries = sorted({r_time} | {e for _, e in schedule_in_bay if e > r_time})

    for entry_candidate in candidate_entries:
        entry  = max(r_time, entry_candidate)
        exit_t = entry + proc

        # Stage-2: 새 블록 진입 시 이미 존재하는 블록들
        present_at_entry = [
            b for b, (a, e) in zip(placed_in_bay, schedule_in_bay)
            if a < entry < e
        ]
        if check_entry(bay, present_at_entry, new_blk, fast=True):
            continue

        # Stage-3: 새 블록 퇴출 시 존재하는 블록들
        present_at_exit = [new_blk] + [
            b for b, (a, e) in zip(placed_in_bay, schedule_in_bay)
            if a < exit_t < e
        ]
        if check_exit(bay, present_at_exit, new_blk, fast=True):
            continue

        # Stage-4: Stage-2/3에서 검출되지 않는 내부 구간 블록
        s4_blocked = False
        for b_other, (a_other, e_other) in zip(placed_in_bay, schedule_in_bay):
            if a_other < entry or e_other > exit_t:
                continue
            if not _time_overlaps(entry, exit_t, a_other, e_other):
                continue
            if check_collisions(bay, [new_blk, b_other]):
                s4_blocked = True
                break
        if s4_blocked:
            continue

        return entry, exit_t

    return None, None


# ==========================================================================
# 보장된 실현 가능 진입: 빈 베이 창
# ==========================================================================

def _empty_bay_entry(schedule_in_bay: list[tuple[int, int]],
                     r_time: int, proc: int) -> int:
    """
    r_time 이상이면서 베이가 완전히 비어 있는 [entry, entry+proc) 창의
    가장 이른 entry 시간 반환.
    """
    entry = int(r_time)
    changed = True
    while changed:
        changed = False
        exit_t = entry + proc
        for a, e in schedule_in_bay:
            if _time_overlaps(entry, exit_t, a, e):
                entry = max(entry, e)
                changed = True
    return entry


# ==========================================================================
# 강제 배치 헬퍼 (feasibility 검사 없음 — 최후 수단)
# ==========================================================================

def _force_place(bi: int,
                 blocks_data: list[dict],
                 bays: list[Bay],
                 bay_schedule: list[list[tuple[int, int]]],
                 prefs: list[float]) -> tuple:
    """
    block bi를 최소 유효 위치에 강제 배치 (선호도 가장 높은 베이 우선).
    빈 베이 창을 사용하므로 항상 crane-path 실현 가능.
    """
    blk_data = blocks_data[bi]
    r_time   = blk_data["release_time"]
    proc     = blk_data["processing_time"]
    n_bays   = len(bays)

    for bay_id in sorted(range(n_bays), key=lambda j: prefs[j], reverse=True):
        bay = bays[bay_id]
        for oi in range(len(blk_data["shape"])):
            bb = _block_bbox(blk_data, oi)
            lx0, ly0, lx1, ly1 = bb
            px_lo = math.ceil(-lx0)
            px_hi = math.floor(bay.width  - lx1)
            py_lo = math.ceil(-ly0)
            py_hi = math.floor(bay.height - ly1)
            if px_lo > px_hi or py_lo > py_hi:
                continue
            px = max(0, px_lo)
            py = max(0, py_lo)
            entry = _empty_bay_entry(bay_schedule[bay_id], r_time, proc)
            return (bay_id, px, py, oi, entry, entry + proc)

    raise RuntimeError(
        f"_force_place: block {bi} has no valid integer position in any bay "
        f"-- instance validation should have caught this."
    )


# ==========================================================================
# 공유 탐욕 배치 커널 (Phase 1 및 _repair 에서 사용)
# ==========================================================================

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
) -> dict[int, dict]:
    """
    Phase 1 및 repair(greedy 모드) 공용 배치 커널.

    변경 사항 (baseline 대비):
      1. K-best: (bay, orientation)별 최대 K_BEST 개의 실현 가능 후보를 수집 후
                 그 중 _placement_score 최소값 선택. (모든 후보를 평가하는 대신)
      2. B(x,y): _placement_score 호출 시 블로킹 항(blocking) 계산 후 전달.
    """
    n_bays  = len(bays)
    n_total = len(block_ids)
    result: dict[int, dict] = {}
    n_forced = n_fallback = 0

    _bay_areas  = [bay.width * bay.height for bay in bays]
    _avg_area   = sum(_bay_areas) / n_bays
    bay_weights = [_avg_area / a for a in _bay_areas]

    for rank, bi in enumerate(block_ids):
        blk_data = blocks_data[bi]
        r_time   = blk_data["release_time"]
        due      = blk_data["due_date"]
        proc     = blk_data["processing_time"]
        workload = blk_data["workload"]
        prefs    = blk_data["bay_preferences"]
        s_max    = max(prefs)
        n_orient = len(blk_data["shape"])

        best_score     = float("inf")
        best_placement = None
        used_forced    = bi in forced_ids

        if not used_forced:
            # ── repair fast-path: 이전 (bay, x, y, orient) 먼저 시도 ──────────
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
                        # B(x,y) 계산: 이전 위치 기준
                        active_prev = [
                            b for b, (a_k, e_k) in zip(bay_placed[pb_id],
                                                         bay_schedule[pb_id])
                            if e_k > r_time
                        ]
                        p_y_min = py + p_bb[1]
                        p_y_max = py + p_bb[3]
                        prev_blocking = sum(
                            1 for b in active_prev
                            if b.x < px and (
                                b.bounding_rect()[1] < p_y_max and
                                p_y_min < b.bounding_rect()[3]
                            )
                        )
                        best_score = _placement_score(
                            tardiness, workload, bay_loads, pb_id,
                            s_max - prefs[pb_id], bay_weights, w1, w2, w3,
                            top_y=py + p_bb[3],
                            blocking=prev_blocking
                        )
                        best_placement = (pb_id, px, py, poi, entry, exit_t)

            # ── 전체 탐색 (Phase-1 스타일) + K-best ──────────────────────────
            bay_order = sorted(range(n_bays), key=lambda j: prefs[j], reverse=True)
            for bay_id in bay_order:
                bay             = bays[bay_id]
                placed_in_bay   = bay_placed[bay_id]
                schedule_in_bay = bay_schedule[bay_id]

                for oi in range(n_orient):
                    blk_bb = _block_bbox(blk_data, oi)
                    lx0_oi, ly0_oi, lx1_oi, ly1_oi = blk_bb
                    if (math.ceil(-lx0_oi) > math.floor(bay.width  - lx1_oi) or
                            math.ceil(-ly0_oi) > math.floor(bay.height - ly1_oi)):
                        continue

                    active_in_bay = [
                        b for b, (a_k, e_k) in zip(placed_in_bay, schedule_in_bay)
                        if e_k > r_time
                    ]
                    candidates = _candidate_positions(
                        bay.width, bay.height, active_in_bay, blk_bb
                    )

                    # ── K-best 수집: 첫 K_BEST 개 실현 가능 후보 ─────────────
                    feasible_k: list[tuple[int, int, int, int]] = []
                    for (cx, cy) in candidates:
                        if len(feasible_k) >= K_BEST:
                            break
                        new_blk = Block(block_id=bi, block_data=blk_data,
                                        x=cx, y=cy, orient_idx=oi)
                        if not bay.contains_block(new_blk):
                            continue
                        entry, exit_t = _find_earliest_slot(
                            new_blk, bay, placed_in_bay, schedule_in_bay,
                            r_time, proc
                        )
                        if entry is None:
                            continue
                        feasible_k.append((cx, cy, entry, exit_t))

                    # ── 수집된 K개 후보 점수 평가 ────────────────────────────
                    for (cx, cy, entry, exit_t) in feasible_k:
                        tardiness = max(0.0, exit_t - due)
                        # B(x,y): 새 블록 앞에 있으면서 y-range 겹치는 블록 수
                        new_y_min = cy + ly0_oi
                        new_y_max = cy + ly1_oi
                        blocking = sum(
                            1 for b in active_in_bay
                            if b.x < cx and (
                                b.bounding_rect()[1] < new_y_max and
                                new_y_min < b.bounding_rect()[3]
                            )
                        )
                        score = _placement_score(
                            tardiness, workload, bay_loads, bay_id,
                            s_max - prefs[bay_id], bay_weights, w1, w2, w3,
                            top_y=cy + ly1_oi,
                            blocking=blocking
                        )
                        if score < best_score:
                            best_score     = score
                            best_placement = (bay_id, cx, cy, oi, entry, exit_t)

        if best_placement is None:
            best_placement = _force_place(bi, blocks_data, bays, bay_schedule, prefs)
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
                flag = (" [forced]" if used_forced
                        else (" [fallback]" if best_score == float("inf") else ""))
                print(f"[alg2]   {n_done:4d}/{n_total}"
                      f"  block{bi:<4d} -> bay{bay_id} ({cx},{cy}) oi={oi}"
                      f"  t=[{int(round(entry))},{int(round(exit_t))})"
                      f"  loads=[{loads_str}]"
                      f"  fallback={n_fallback}{flag}"
                      f"  {elapsed:.1f}s")

    return result


# ==========================================================================
# Phase 2: 비실행 가능 블록 수리
# ==========================================================================

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
    위반 블록을 반복 감지하여 수리. (baseline과 동일 — 변경 없음)

    greedy 모드: 위반 블록을 제거하고 전체 Phase-1 탐색으로 재배치.
                 반복 위반자 → forced_ids (빈 베이 창), 90% time-guard.
    simple  모드: 현재 위치 유지, 시간 창만 빈 베이로 이동.
    """
    from utils import check_feasibility

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
        print(f"[alg2] Repair pass {pass_idx+1}: {len(viols)} violation(s)  "
              f"stage={result['stage']}  elapsed={elapsed_r:.1f}s")

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

        to_repair.sort(key=lambda b: (blocks_data[b]["due_date"],
                                      blocks_data[b]["processing_time"]))
        n_repl = len(to_repair)

        if repair_mode == "simple":
            n_bays = len(bays)
            bay_schedule: list[list[tuple[int, int]]] = [[] for _ in range(n_bays)]
            for a in assignments.values():
                bay_schedule[a["bay_id"]].append((a["entry_time"], a["exit_time"]))

            for ri, bid in enumerate(to_repair):
                a      = assignments[bid]
                bay_id = a["bay_id"]
                r_time = blocks_data[bid]["release_time"]
                proc   = blocks_data[bid]["processing_time"]

                old_slot = (a["entry_time"], a["exit_time"])
                if old_slot in bay_schedule[bay_id]:
                    bay_schedule[bay_id].remove(old_slot)

                entry  = _empty_bay_entry(bay_schedule[bay_id], r_time, proc)
                exit_t = entry + proc

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
                print(f"[alg2]   repair {ri+1:3d}/{n_repl}"
                      f"  block{bid:<4d} {tag}"
                      f"  bay{bay_id} ({int(x)},{int(y)})"
                      f"  {prev_t} -> {new_t}"
                      f"  elapsed={elapsed_ri:.1f}s")

        else:  # greedy mode
            for bid in to_repair:
                repaired_counts[bid] = repaired_counts.get(bid, 0) + 1
                if repaired_counts[bid] > 1:
                    forced_ids.add(bid)

            for bid in to_repair:
                assignments.pop(bid, None)

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
                if time.time() - t_start > timelimit * 0.90:
                    forced_ids.add(bi)
                prev_a  = assignments.get(bi)
                partial = _place_blocks(
                    [bi], blocks_data, bays,
                    bay_placed, bay_schedule2, bay_loads,
                    w1, w2, w3, forced_ids,
                    prev_assignments=assignments,
                )
                assignments.update(partial)
                new_a       = partial[bi]
                is_forced   = bi in forced_ids
                changed_bay = prev_a and prev_a["bay_id"] != new_a["bay_id"]
                changed_pos = prev_a and (prev_a["x"] != new_a["x"]
                                          or prev_a["y"] != new_a["y"])
                tag = ("[forced]" if is_forced
                       else "[bay]" if changed_bay
                       else "[pos]" if changed_pos
                       else "[time]")
                prev_t = (f"[{int(prev_a['entry_time'])},{int(prev_a['exit_time'])})"
                          if prev_a else "N/A")
                new_t  = f"[{int(new_a['entry_time'])},{int(new_a['exit_time'])})"
                elapsed_ri = time.time() - t_start
                print(f"[alg2]   repair {ri+1:3d}/{n_repl}"
                      f"  block{bi:<4d} {tag}"
                      f"  bay{new_a['bay_id']} ({int(new_a['x'])},{int(new_a['y'])})"
                      f"  {prev_t} -> {new_t}"
                      f"  elapsed={elapsed_ri:.1f}s")

        sol = {"operations": _build_operations(list(assignments.values()))}

    result = check_feasibility(prob_info, sol)
    status = "feasible" if result["feasible"] else f"INFEASIBLE stage={result['stage']}"
    obj    = f"obj={result['objective']:.0f}" if result["feasible"] else ""
    forced_note = f"  forced={len(forced_ids)}" if forced_ids else ""
    elapsed_done = time.time() - t_start
    print(f"[alg2] Repair done  |  {status}  {obj}{forced_note}  elapsed={elapsed_done:.1f}s")

    return assignments


# ==========================================================================
# operations dict 빌드
# ==========================================================================

def _build_operations(assignments: list[dict]) -> dict:
    """
    assignment dict 리스트에서 operations dict 생성.
    EXIT → ENTRY 순서 보장, 각 타임포인트 내 block_id 기준 정렬.
    (baseline과 동일 — 변경 없음)
    """
    buckets: dict[int, list[tuple]] = {}
    for a in assignments:
        t_entry = int(a["entry_time"])
        t_exit  = int(a["exit_time"])
        bid     = a["block_id"]
        bay     = a["bay_id"]
        buckets.setdefault(t_exit,  []).append((0, "EXIT",  bid, bay, None,    None,    None))
        buckets.setdefault(t_entry, []).append((1, "ENTRY", bid, bay, a["x"], a["y"], a["orient_idx"]))

    operations: dict[str, list[dict]] = {}
    for t in sorted(buckets):
        ops = sorted(buckets[t], key=lambda x: (x[0], x[2]))
        result = []
        for _, kind, bid, bay, x, y, orient_idx in ops:
            op: dict = {"type": kind, "block_id": bid, "bay_id": bay}
            if kind == "ENTRY":
                op["x"] = x
                op["y"] = y
                op["orient_idx"] = orient_idx
            result.append(op)
        operations[str(t)] = result
    return operations


# ==========================================================================
# 메인 알고리즘 진입점
# ==========================================================================

def algorithm(prob_info: dict, timelimit: float = 60,
              repair_mode: str = "greedy") -> dict:
    """
    OGC 2026 제출 진입점. baseline_greedy.py의 greedyalgorithm()을 대체.

    변경 사항:
      - CANDIDATE_MODE (v0/v1/v2) 에 따라 후보 위치 생성 방식 선택
      - K_BEST 개의 실현 가능 후보 비교 후 최저 점수 선택
      - _placement_score 에 W5*B(x,y) 블로킹 항 추가

    나머지 파이프라인 (EDD 정렬, repair 패스, force-place, feasibility 검사 등)
    은 baseline과 동일.
    """
    t_start = time.time()

    bays_data   = prob_info["bays"]
    blocks_data = prob_info["blocks"]
    n_bays      = len(bays_data)
    n_blocks    = len(blocks_data)

    w1 = prob_info.get("weights", {}).get("w1", 1.0)
    w2 = prob_info.get("weights", {}).get("w2", 1.0)
    w3 = prob_info.get("weights", {}).get("w3", 1.0)

    print(f"[alg2] Instance : {prob_info.get('name', '?')}")
    print(f"[alg2] Bays     : {n_bays}  |  Blocks : {n_blocks}  |  Timelimit : {timelimit:.1f}s")
    print(f"[alg2] Weights  : w1={w1}  w2={w2}  w3={w3}")
    print(f"[alg2] Mode     : candidate={CANDIDATE_MODE}  K={K_BEST}  W5={W5}")
    print(f"[alg2] {'-' * 56}")

    bays = [Bay.from_dict(d, i) for i, d in enumerate(bays_data)]
    for i, b in enumerate(bays):
        print(f"[alg2]   bay[{i}]  {b.width}x{b.height}")

    # ── 인스턴스 유효성 검사 ──────────────────────────────────────────────
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
        print(f"[alg2] ERROR: {len(invalid_blocks)} block(s) cannot be placed -- malformed instance.")
        raise ValueError(
            f"Malformed instance '{prob_info.get('name', '?')}': "
            f"block(s) {invalid_blocks} have no valid integer placement in any bay."
        )

    # ── Phase 1: EDD greedy 배치 ─────────────────────────────────────────
    sorted_indices = sorted(
        range(n_blocks),
        key=lambda i: (blocks_data[i]["due_date"], blocks_data[i]["processing_time"])
    )
    print(f"[alg2] {'-' * 56}")
    print("[alg2] Phase 1 : EDD greedy placement ...")

    bay_placed:   list[list[Block]]             = [[] for _ in range(n_bays)]
    bay_schedule: list[list[tuple[int, int]]]   = [[] for _ in range(n_bays)]
    bay_loads:    list[float]                   = [0.0] * n_bays

    assignments = _place_blocks(
        sorted_indices, blocks_data, bays,
        bay_placed, bay_schedule, bay_loads,
        w1, w2, w3, forced_ids=set(),
        t_start=t_start, log_interval=max(1, n_blocks // 10),
    )

    elapsed_p1 = time.time() - t_start
    loads_str = "  ".join(f"bay{i}={round(bay_loads[i])}" for i in range(n_bays))
    print(f"[alg2] Phase 1 done  |  placed={len(assignments)}  {loads_str}  "
          f"elapsed={elapsed_p1:.2f}s")

    # ── Phase 2: repair ───────────────────────────────────────────────────
    print(f"[alg2] {'-' * 56}")
    print(f"[alg2] Phase 2 : repair  mode={repair_mode}")
    sol = {"operations": _build_operations(list(assignments.values()))}
    assignments = _repair(prob_info, sol, assignments, bays, blocks_data,
                          w1, w2, w3, t_start, timelimit,
                          repair_mode=repair_mode)

    elapsed_total = time.time() - t_start
    final_sol = {"operations": _build_operations(list(assignments.values()))}

    from utils import check_feasibility
    final_result = check_feasibility(prob_info, final_sol)
    print(f"[alg2] {'-' * 56}")
    print(f"[alg2] Done  |  assigned={len(assignments)}/{n_blocks}  "
          f"elapsed={elapsed_total:.2f}s")
    if final_result["feasible"]:
        print(f"[alg2] Objective : {final_result['objective']:.0f}  "
              f"(obj1={final_result['obj1']:.1f}  "
              f"obj2={final_result['obj2']:.1f}  "
              f"obj3={final_result['obj3']:.1f})")
    else:
        print(f"[alg2] INFEASIBLE stage={final_result['stage']}")
        for v in final_result["violations"][:5]:
            print(f"[alg2]   {v}")

    return final_sol


# ==========================================================================
# CLI 실행 (벤치마크 / 스모크 테스트)
# ==========================================================================

if __name__ == "__main__":
    import argparse
    import json
    import pathlib
    from utils import check_feasibility

    parser = argparse.ArgumentParser(
        description="myalgorithm2 벤치마크 실행"
    )
    parser.add_argument("instance", help="인스턴스 JSON 파일 경로")
    parser.add_argument("--timelimit", type=float, default=60.0)
    parser.add_argument("--repair",    choices=["greedy", "simple"], default="greedy")
    parser.add_argument("--mode",      choices=["v0", "v1", "v2"], default=None,
                        help="후보 생성 모드 (기본: 모듈 상수 CANDIDATE_MODE)")
    parser.add_argument("--k",         type=int, default=None,
                        help="K-best 후보 수 (기본: 모듈 상수 K_BEST)")
    parser.add_argument("--w5",        type=float, default=None,
                        help="B(x,y) 가중치 (기본: 모듈 상수 W5)")
    args = parser.parse_args()

    # 명령줄 인수로 상수 오버라이드
    import myalgorithm2 as _self
    if args.mode is not None:
        _self.CANDIDATE_MODE = args.mode
    if args.k is not None:
        _self.K_BEST = args.k
    if args.w5 is not None:
        _self.W5 = args.w5

    inst_file = pathlib.Path(args.instance)
    with open(inst_file) as f:
        prob_info = json.load(f)

    t0  = time.time()
    sol = algorithm(prob_info, timelimit=args.timelimit, repair_mode=args.repair)
    elapsed = time.time() - t0

    result = check_feasibility(prob_info, sol)
    n_assigned = sum(1 for ops in sol["operations"].values()
                     for op in ops if op["type"] == "ENTRY")
    print(f"\n{'='*60}")
    print(f"Instance   : {prob_info['name']}")
    print(f"Mode       : candidate={_self.CANDIDATE_MODE}  K={_self.K_BEST}  W5={_self.W5}")
    print(f"Elapsed    : {elapsed:.3f}s")
    print(f"Assigned   : {n_assigned} / {len(prob_info['blocks'])} blocks")
    print(f"Feasible   : {result['feasible']}  (stage={result['stage']})")
    if result["feasible"]:
        print(f"Objective  : {result['objective']:.2f}  "
              f"(obj1={result['obj1']:.1f}, obj2={result['obj2']:.1f}, "
              f"obj3={result['obj3']:.1f})")
    else:
        for v in result["violations"][:10]:
            print(f"  VIOLATION: {v}")
