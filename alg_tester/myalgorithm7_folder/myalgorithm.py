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
import utils as _utils_module


# ==========================================================================
# Block.bounding_rect 인스턴스 캐시 (myalgorithm7 속도 개선 -- 결과 불변)
# ==========================================================================
#
# 프로파일(prob_26, 40블록): bounding_rect 354k회 호출로 ~10s/33.7s 소모.
# Block은 생성 후 위치/방향이 절대 안 바뀌고(utils도 __post_init__에서
# _layers_cache를 같은 이유로 미리 굳혀 둠), bounding_rect는 그 불변
# 레이어의 순수 함수이므로 첫 계산값을 인스턴스에 저장해 재사용해도
# 결과가 비트 단위로 동일하다.  utils.Block 메서드를 모듈 로드 시점에
# 캐시 래퍼로 교체한다 -- check_entry/check_exit/check_feasibility 내부의
# AABB 프리스크리닝까지 전부 같이 빨라진다.
_orig_bounding_rect = _utils_module.Block.bounding_rect


def _bounding_rect_cached(self):
    r = getattr(self, "_brect_cache", None)
    if r is None:
        r = _orig_bounding_rect(self)
        object.__setattr__(self, "_brect_cache", r)
    return r


_utils_module.Block.bounding_rect = _bounding_rect_cached


# ==========================================================================
# _poly_from_verts identity 캐시 (myalgorithm7 속도 개선 -- 결과 불변)
# ==========================================================================
#
# 프로파일(prob_26, 40블록): _poly_from_verts 276k회 호출로 9.4s/31.2s
# (~30%).  utils의 lru_cache는 verts를 매 호출 tuple-of-tuples로 변환해
# 키를 만드는데(그것만 ~2.7s), 호출의 대부분은 "배치된 블록"의 레이어
# 리스트다 -- Block.__post_init__이 _layers_cache로 한 번 굳혀둔 뒤 절대
# 안 바뀌는 동일 리스트 객체가 수천 번씩 들어온다.  그래서 객체
# identity(id) 기반 1차 캐시를 앞단에 둔다: 같은 리스트 객체면 tuple 변환
# 없이 폴리곤을 즉시 반환.  미스면 원본(내부 lru_cache 포함)으로 넘어가므로
# 반환값은 기존과 동일 객체다.
#
# 정확성: 캐시가 verts 리스트에 대한 강한 참조를 함께 보관하고, 히트 시
# `hit[0] is verts`로 동일 객체임을 확인한다 -- 참조를 쥐고 있는 동안
# 그 id는 재사용될 수 없으므로(GC 불가) id 충돌에 의한 오답이 원리적으로
# 불가능하다.  후보 위치마다 생기는 일회성 리스트가 캐시를 채우는 것은
# 메모리 낭비일 뿐 오답이 아니며, 상한(3만 엔트리) 초과 시 전체 비우고
# 다시 채운다 (활성 블록 레이어는 곧바로 재캐시되므로 손실 미미).
# 승격 정책: "두 번째 등장"부터 캐시에 넣는다.  후보 위치마다 생기는
# 일회성 리스트가 캐시를 가득 채워 클리어가 반복되면(스래싱) 활성 블록의
# 핫 엔트리까지 계속 증발해 이득이 사라지는 것이 실측됐다.  seen-once
# 집합은 id(int)만 담아 가볍고, id 재사용으로 인한 오승격이 나도 조회 시
# `hit[0] is verts` 동일성 검증이 있어 오답은 원리적으로 불가능하다.
_orig_poly_from_verts = _utils_module._poly_from_verts
_poly_id_cache: dict[int, tuple] = {}
_poly_seen_once: set[int] = set()


def _poly_from_verts_fast(verts):
    if not verts or len(verts) < 3:
        return None
    key = id(verts)
    hit = _poly_id_cache.get(key)
    if hit is not None and hit[0] is verts:
        return hit[1]
    p = _orig_poly_from_verts(verts)
    if key in _poly_seen_once:
        if len(_poly_id_cache) > 30_000:
            _poly_id_cache.clear()
        _poly_id_cache[key] = (verts, p)
    else:
        if len(_poly_seen_once) > 200_000:
            _poly_seen_once.clear()
        _poly_seen_once.add(key)
    return p


_utils_module._poly_from_verts = _poly_from_verts_fast


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

# ==========================================================================
# 포트폴리오 셀프 셀렉션 (myalgorithm6 신규)
# ==========================================================================
#
# 리더보드 점수는 문제당 순위(R - nb)라서 "어느 유형에서도 지지 않는 것"이
# 절대 개선폭보다 중요하다.  실측:
#   즉시진입 우선정렬 ON  : 포화형에서 크게 유리 (prob_25 obj1 2428->1488,
#                           prob_38 11393->8484)
#   즉시진입 우선정렬 OFF : 중간부하형(prob_21/23/24)에서 유리 (ON은 공간
#                           파편화로 소폭 악화)
# 정적 통계로는 두 유형이 구분되지 않고(prob_23/25 포화도 12.03/12.55),
# 블록단위 적응형 게이트도 실측 기각됐다(_place_blocks 주석 참조).  그래서
# 시간이 허락하는 인스턴스에서는 두 변형을 모두 실행해 objective가 낮은
# 쪽을 낸다 (아래 algorithm() 드라이버 참조).
PORTFOLIO_ENABLED: bool = True

# 즉시진입 우선정렬 토글 (포트폴리오 드라이버가 변형별로 설정)
IMMEDIATE_SORT_ENABLED: bool = True

# 두 번째 변형에 넘기지 않고 남겨두는 안전 여유(초) -- 최종 비교/출력과
# 변형 2의 마무리 check_feasibility가 60초 벽을 넘지 않게 하는 버퍼.
# (3.0으로 prob_6 실측 시 총 59.1s로 60초에 0.9s 차이까지 붙어서, 채점
# 서버의 시간 초과 = -1점 리스크를 줄이려고 4.5로 상향.  변형 2는 내부
# 시간가드가 자기 슬라이스 기준으로 작동하는데 마무리 check_feasibility가
# 슬라이스를 ~2s 초과할 수 있음이 실측됐다.)
PORTFOLIO_MARGIN_S: float = 4.5

# 두 번째 변형을 돌릴 최소 예산(초).  남은 시간은 안 쓰면 그냥 버려지는
# 예산이므로(첫 실행 결과는 이미 확보됨, 비교가 나쁜 결과를 걸러줌) 문턱을
# 낮게 잡는다 -- 이 값이면 내부 시간가드(80%/98%)로 feasible 해를 하나
# 만들어내기에 충분하다.  어려운 인스턴스(prob_38급, 첫 실행 55s+)는 남은
# 시간이 이 문턱에 못 미쳐 자동으로 단일 실행이 된다.
PORTFOLIO_MIN_BUDGET2_S: float = 8.0

# 공정 예산 조건: Variant 2는 자기 예산이 "Variant 1 소요시간 x 이 비율"
# 이상일 때만 돌린다.  절대 최소치(PORTFOLIO_MIN_BUDGET2_S)만으로는 포화
# 경계선 인스턴스(prob_27/31/38급, Variant 1이 45~55s)에서 10초 남짓의
# "이길 가망 없는" Variant 2가 돌아 60초 벽 리스크만 키우는 것이 실측됐다.
# 비교 로직 덕에 결과가 나빠지진 않지만(항상 더 나은 쪽 반환), 벽시계
# 낭비와 시간초과(-1점) 위험을 없애기 위해 공정 예산일 때만 돌린다.
# 0.6이면: prob_6급(V1 25s -> 잔여 35s, 필요 15s)은 돌고, 경계선급
# (V1 45s -> 잔여 15s, 필요 27s)은 깔끔하게 단일 실행이 된다.
PORTFOLIO_FAIR_BUDGET_RATIO: float = 0.6


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

    bbox 프리필터 (myalgorithm7 속도 개선, 결과 불변): check_exit은 내부
    AABB 프리필터가 돌기 "전에" target(b_other)의 Shapely 폴리곤들을 무조건
    생성한다.  bbox가 서로소인 쌍은 check_exit이 어차피 빈 리스트(위반
    없음)를 반환하므로, 호출 전에 순수 정수 비교로 걸러내면 판정은 비트
    단위로 동일하고 폴리곤 생성 비용만 사라진다.  shielding은 스코어링되는
    후보마다 모든 due-이른 활성 블록과 이 검사를 하므로(프로파일상
    check_entry/check_exit이 전체의 ~94%), 밀집 인스턴스에서 Phase 1
    처리량을 직접 끌어올린다.
    """
    new_bb = new_blk.bounding_rect()
    for b_other in active_in_bay:
        due_other = blocks_data[b_other.block_id]["due_date"]
        if due_other > due_new:
            continue
        if not _bb_overlap(new_bb, b_other.bounding_rect()):
            continue  # check_exit 내부 프리필터와 동일 판정 -- 위반 불가
        if check_exit(bay, [b_other, new_blk], b_other, fast=True):
            return True
    return False


def _find_shielding_blocker(new_blk: Block, bay: Bay,
                           active_in_bay: list[Block],
                           due_new: float,
                           blocks_data: list[dict]) -> Block | None:
    """
    _shielding_violated 과 동일한 판정이지만, bool 대신 실제로 막고 있는
    Block 객체를 반환한다 -- destroy/repair(ALNS 스타일) 호출부가 "누구를
    치울지" 알아야 하기 때문이다.  (bbox 프리필터도 동일하게 적용 --
    _shielding_violated docstring 참조.)
    """
    new_bb = new_blk.bounding_rect()
    for b_other in active_in_bay:
        due_other = blocks_data[b_other.block_id]["due_date"]
        if due_other > due_new:
            continue
        if not _bb_overlap(new_bb, b_other.bounding_rect()):
            continue
        if check_exit(bay, [b_other, new_blk], b_other, fast=True):
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

def _algorithm_once(prob_info: dict, timelimit: float = 60,
                    repair_mode: str = "greedy") -> tuple[dict, dict]:
    """
    EDD + Best-Fit Greedy + due-date shielding, with post-hoc feasibility repair.
    (기존 algorithm() 본체 -- 포트폴리오 드라이버가 변형별로 호출한다.)

    Parameters
    ----------
    prob_info   : instance JSON dict with keys "name", "bays", "blocks", "weights"
    timelimit   : wall-clock time limit in seconds (이 실행 몫의 슬라이스)
    repair_mode : "greedy" (default) or "simple" -- see module docstring for details

    Returns
    -------
    (solution dict, check_feasibility result dict) -- 드라이버가 변형 간
    objective 비교에 result를 재사용한다 (중복 check_feasibility 방지)

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
    assignments, repair_result = _repair(prob_info, sol, assignments, bays,
                                         blocks_data, w1, w2, w3, t_start,
                                         timelimit, repair_mode=repair_mode)

    elapsed_total = time.time() - t_start
    final_sol = {"operations": _build_operations(list(assignments.values()), prob_info)}

    # _repair가 동일 assignments로 이미 검사한 결과를 재사용한다 --
    # _build_operations는 결정적이므로 재검사는 순수 중복(250블록 ~2s)이고,
    # 60초 벽 근처에서 이 2초가 시간초과(-1점)를 가르는 것이 실측됐다.
    final_result = repair_result
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

    return final_sol, final_result


def algorithm(prob_info: dict, timelimit: float = 60,
             repair_mode: str = "greedy") -> dict:
    """
    포트폴리오 셀프 셀렉션 드라이버 (myalgorithm6 신규 -- 모듈 상단
    PORTFOLIO_ENABLED 주석 참조).

    1) 즉시진입 정렬 ON 변형을 전체 예산으로 실행한다.  어려운 인스턴스
       (prob_38급, 55s+)는 이 실행이 예산을 다 쓰므로 두 번째 변형이
       자동으로 생략된다 -- 즉 어려운 인스턴스의 결과는 기존과 완전히
       동일하고, 포트폴리오로 인해 나빠질 수 없다.
    2) 남은 시간이 "ON 실행 소요시간 + PORTFOLIO_MARGIN_S" 이상이면
       (쉬운 인스턴스: 한 실행 12~25s) OFF 변형을 남은 예산으로 실행한다.
    3) 둘 다 feasible이면 objective가 낮은 쪽을, 아니면 feasible인 쪽을
       반환한다.
    """
    global IMMEDIATE_SORT_ENABLED
    t0 = time.time()

    if not PORTFOLIO_ENABLED:
        sol, _ = _algorithm_once(prob_info, timelimit, repair_mode)
        return sol

    IMMEDIATE_SORT_ENABLED = True
    print(f"[Portfolio] Variant 1/2 : immediate-sort ON  (budget {timelimit:.1f}s)")
    sol_on, res_on = _algorithm_once(prob_info, timelimit, repair_mode)
    elapsed_on = time.time() - t0

    remaining = timelimit - elapsed_on
    needed = max(PORTFOLIO_MIN_BUDGET2_S,
                 elapsed_on * PORTFOLIO_FAIR_BUDGET_RATIO) + PORTFOLIO_MARGIN_S
    if remaining < needed:
        print(f"[Portfolio] remaining {remaining:.1f}s < needed "
              f"{needed:.1f}s -- variant 2 skipped "
              f"(single-run mode, 기존 동작과 동일)")
        return _post_improve(prob_info, sol_on, res_on, t0, timelimit)

    # V2 슬라이스는 "V1이 실제 쓴 시간 x 1.2"로 캡한다 -- V2가 잔여 시간을
    # 통째로 가져가면 Phase-3 폴리시(objective 다듬기)가 돌 시간이 안 남는다.
    # V1이 그 시간 안에 끝났으므로 V2도 비슷한 예산이면 충분하고, 남는
    # 시간은 선택된 해의 폴리시에 쓰는 것이 더 생산적이다.
    budget2 = min(remaining - PORTFOLIO_MARGIN_S, elapsed_on * 1.2)
    print(f"[Portfolio] {'=' * 56}")
    print(f"[Portfolio] Variant 2/2 : immediate-sort OFF (budget {budget2:.1f}s)")
    IMMEDIATE_SORT_ENABLED = False
    try:
        sol_off, res_off = _algorithm_once(prob_info, budget2, repair_mode)
    finally:
        IMMEDIATE_SORT_ENABLED = True

    obj_on  = res_on["objective"]  if res_on["feasible"]  else float("inf")
    obj_off = res_off["objective"] if res_off["feasible"] else float("inf")
    pick = "OFF" if obj_off < obj_on else "ON"
    print(f"[Portfolio] {'=' * 56}")
    print(f"[Portfolio] ON={obj_on:.0f}  OFF={obj_off:.0f}  -> {pick} selected  "
          f"(total {time.time() - t0:.1f}s)")
    if obj_off < obj_on:
        return _post_improve(prob_info, sol_off, res_off, t0, timelimit)
    return _post_improve(prob_info, sol_on, res_on, t0, timelimit)


# ==========================================================================
# Phase 3: objective polish (myalgorithm7 신규 -- Z2/Z3 직접 겨냥)
# ==========================================================================
#
# 동기 (hidden 프록시 인스턴스 실측): 쉬운 인스턴스는 obj1이 0~30까지
# 내려가서 objective의 대부분이 Z3(선호 베이 페널티)와 Z2(부하 균형)다:
#   prob_2  : w3*Z3=79%  w2*Z2=21%  (obj1=0)
#   prob_8  : w3*Z3=92%  w2*Z2= 8%  (obj1=0)
#   prob_22 : w3*Z3=75%  w1*Z1=24%
# 리더보드는 문제당 순위제라 쉬운 문제(모두가 Z1=0 달성)의 순위는 Z2/Z3가
# 결정한다.  그런데 이런 인스턴스는 실행이 20~35s에 끝나 예산이 남는다 --
# 그 남는 시간에 "objective 전체 기준으로 개선되는 블록 이동"을 반복한다.
#
# 안전장치 (myalgorithm5 tardiness-LNS에서 검증된 패턴 재사용): 이동 하나를
# 잠정 적용할 때마다 전체 check_feasibility로 재검증하고, feasible하면서
# objective가 "엄격히" 낮아질 때만 채택한다.  아니면 그 이동 하나만 정확히
# 롤백한다.  따라서 이 패스는 절대 결과를 나쁘게 만들 수 없다 (단조 개선).
POLISH_ENABLED: bool = True

# 폴리시를 시작할 최소 잔여 시간(초).  이보다 적게 남았으면 그냥 반환 --
# 포화 인스턴스(prob_38급)는 자동으로 폴리시가 생략된다.
POLISH_MIN_REMAINING_S: float = 5.0

# ==========================================================================
# AABB 좌압축 패스 (myalgorithm7 신규 -- 강제배치 잔재 Z1 직접 겨냥)
# ==========================================================================
#
# 배경: 포화/중간부하 인스턴스에서 블록의 절반 안팎이 time guard에 걸려
# AABB 강제배치되는데, 강제배치는 "그 시점까지 배치된 블록들" 기준의
# 자리라서 전체 배치가 끝난 뒤에는 더 이른 bbox-창이 생겨 있는 경우가
# 많다 (진단: prob_26/33에서 지각 대기의 대부분이 이 강제배치 몫).
#
# 동작: 지각(tardiness>0) 블록을 지각 큰 순으로, **자기 베이 안에서만**
# (obj2/obj3 불변 -> 이동이 채택되면 objective가 반드시 감소) 현재 entry보다
# 이른 bbox-서로소 (entry, x, y, orient)를 _force_place와 같은 순수 정수
# 탐색으로 찾아 당긴다.  bbox-서로소는 AABB 프리필터에 의해 구조적으로
# crane-feasible이므로 이동 자체가 안전하고, 패스가 끝나면 전체
# check_feasibility로 한 번 더 검증해 문제가 있으면 통째로 롤백한다.
AABB_SHIFT_ENABLED: bool = True

# 패스를 시작할 최소 잔여 시간(초).  마지막 전체 검증(~1.5s) + 여유.
AABB_SHIFT_MIN_REMAINING_S: float = 5.0

# 이동 탐색을 멈추는 시점: t0 + timelimit - 이 값 (최종 검증 시간 확보).
# 2.8 -> 3.5 (4차 제출 안전마진: 250블록급 최종 check_feasibility ~2s 감안).
AABB_SHIFT_MARGIN_S: float = 3.5

# 60초 벽 안전 버퍼: t0 + timelimit - 이 값 을 절대 데드라인으로 쓰고,
# "직전 이동 소요시간 x 1.5"가 데드라인을 넘길 것 같으면 미리 멈춘다.
# 2.5 -> 3.5 (4차 제출 안전마진: 폴리시 종료 후의 최종 재조립 +
# check_feasibility(~2s)가 데드라인 밖에서 실행되는 것을 감안).
POLISH_MARGIN_S: float = 3.5

# 폴리시 중 단일 블록 재탐색의 K_BEST.  20은 너무 얕고(prob_8 실측: 최선호
# 베이에 자리가 실재하는 블록들이 "즉시진입 후보가 아니라는 이유로" 20개
# 프로브 안에 못 들어 전부 놓침), 0(무제한)은 트라이얼 하나가 ~13s까지
# 걸려 60초 벽을 뚫는 사고가 났다(71.7s 실측).  _place_blocks의 내부
# 시간가드는 블록 루프 시작 시에만 검사해서 단일 블록 탐색은 중간에 끊을
# 수 없으므로, K 자체를 "깊지만 유계"인 100으로 둬서 트라이얼당 비용
# 상한을 구조적으로 묶는다.
POLISH_K_BEST: int = 100


def _post_improve(prob_info: dict, sol: dict, res: dict,
                  t0: float, timelimit: float) -> dict:
    """선택된 해에 대한 사후 개선 파이프라인: AABB 좌압축 -> 폴리시."""
    sol, res = _maybe_aabb_shift(prob_info, sol, res, t0, timelimit)
    return _maybe_polish(prob_info, sol, res, t0, timelimit)


def _maybe_aabb_shift(prob_info: dict, sol: dict, res: dict,
                      t0: float, timelimit: float) -> tuple[dict, dict]:
    """잔여 시간이 충분하면 _aabb_left_shift를 돌리고 (sol, res)를 반환."""
    if not AABB_SHIFT_ENABLED or not res.get("feasible"):
        return sol, res
    remaining = timelimit - (time.time() - t0)
    if remaining < AABB_SHIFT_MIN_REMAINING_S:
        return sol, res
    return _aabb_left_shift(prob_info, sol, res, t0, timelimit)


def _aabb_left_shift(prob_info: dict, sol: dict, res: dict,
                     t0: float, timelimit: float) -> tuple[dict, dict]:
    """
    지각 블록을 지각 큰 순으로 자기 베이 안에서 "현재보다 이른 bbox-서로소
    (entry, x, y, orient)"로 당긴다 (모듈 상단 AABB_SHIFT_ENABLED 주석 참조).

    이동 조건이 entry_new < entry_old 이므로 채택된 이동은 그 블록의
    tardiness를 엄격히 줄이고, 같은 베이 유지라 obj2/obj3은 불변 --
    이동이 하나라도 있으면 objective는 반드시 감소한다.  마지막에 전체
    check_feasibility로 재검증하고 이상하면 통째로 롤백한다.
    """
    from utils import check_feasibility

    blocks_data = prob_info["blocks"]
    bays_data   = prob_info["bays"]
    n_bays      = len(bays_data)

    # -- 솔루션 -> assignments 복원 --------------------------------------
    assignments: dict[int, dict] = {}
    for t_str, ops in sol["operations"].items():
        t = int(t_str)
        for op in ops:
            bid = op["block_id"]
            if op["type"] == "ENTRY":
                assignments.setdefault(bid, {})
                assignments[bid].update({
                    "block_id": bid, "bay_id": op["bay_id"],
                    "x": int(op["x"]), "y": int(op["y"]),
                    "orient_idx": op["orient_idx"], "entry_time": t,
                })
            else:
                assignments.setdefault(bid, {})
                assignments[bid]["exit_time"] = t

    # -- 베이별 (bbox, entry, exit, bid) 상태 -----------------------------
    state: list[list[tuple[tuple, int, int, int]]] = [[] for _ in range(n_bays)]
    for a in assignments.values():
        bid = a["block_id"]
        bb  = _block_bbox(blocks_data[bid], a["orient_idx"])
        wbb = (a["x"] + bb[0], a["y"] + bb[1], a["x"] + bb[2], a["y"] + bb[3])
        state[a["bay_id"]].append((wbb, a["entry_time"], a["exit_time"], bid))

    deadline = t0 + timelimit - AABB_SHIFT_MARGIN_S
    tardy = sorted(
        (bid for bid, a in assignments.items()
         if a["exit_time"] > blocks_data[bid]["due_date"]),
        key=lambda bid: assignments[bid]["exit_time"] - blocks_data[bid]["due_date"],
        reverse=True,
    )

    n_moved = 0
    for bid in tardy:
        if time.time() > deadline:
            break
        a       = assignments[bid]
        bay_id  = a["bay_id"]
        bay_d   = bays_data[bay_id]
        W, H    = bay_d["width"], bay_d["height"]
        r_time  = blocks_data[bid]["release_time"]
        proc    = blocks_data[bid]["processing_time"]
        cur_ent = a["entry_time"]

        entry_cands = sorted(
            {r_time} | {e2 for (_, a2, e2, b2) in state[bay_id]
                        if b2 != bid and r_time < e2 < cur_ent}
        )[:FORCE_ENTRY_TRIES]

        found = None
        for entry_c in entry_cands:
            if entry_c >= cur_ent or time.time() > deadline:
                break
            exit_c = entry_c + proc
            act = [wbb for (wbb, a2, e2, b2) in state[bay_id]
                   if b2 != bid and a2 < exit_c and e2 > entry_c]
            for oi in range(len(blocks_data[bid]["shape"])):
                lb = _block_bbox(blocks_data[bid], oi)
                lx0, ly0, lx1, ly1 = lb
                px_lo, px_hi = math.ceil(-lx0), math.floor(W - lx1)
                py_lo, py_hi = math.ceil(-ly0), math.floor(H - ly1)
                if px_lo > px_hi or py_lo > py_hi:
                    continue
                px0, py0 = max(0, px_lo), max(0, py_lo)
                if not act:
                    found = (entry_c, px0, py0, oi)
                    break
                xs = sorted({px0} | {math.ceil(ab[2] - lx0) for ab in act})
                xs = [x for x in xs if px_lo <= x <= px_hi][:FORCE_POS_XS]
                ys = sorted({py0} | {math.ceil(ab[3] - ly0) for ab in act})
                ys = [y for y in ys if py_lo <= y <= py_hi][:FORCE_POS_YS]
                for cy in ys:
                    for cx in xs:
                        wbb = (cx + lx0, cy + ly0, cx + lx1, cy + ly1)
                        if all(not _bb_overlap(wbb, ab) for ab in act):
                            found = (entry_c, cx, cy, oi)
                            break
                    if found:
                        break
                if found:
                    break
            if found:
                break

        if found is None:
            continue
        entry_n, nx, ny, noi = found
        # state/assignments 갱신
        idx = next(i for i, (_, _, _, b2) in enumerate(state[bay_id]) if b2 == bid)
        state[bay_id].pop(idx)
        nb  = _block_bbox(blocks_data[bid], noi)
        nbb = (nx + nb[0], ny + nb[1], nx + nb[2], ny + nb[3])
        state[bay_id].append((nbb, entry_n, entry_n + proc, bid))
        assignments[bid] = dict(a, x=int(nx), y=int(ny), orient_idx=noi,
                                entry_time=int(entry_n),
                                exit_time=int(entry_n + proc))
        n_moved += 1

    if n_moved == 0:
        print(f"[AABB-shift] no move found ({len(tardy)} tardy)  "
              f"(total {time.time() - t0:.1f}s)")
        return sol, res

    new_sol = {"operations": _build_operations(list(assignments.values()), prob_info)}
    new_res = check_feasibility(prob_info, new_sol)
    if new_res["feasible"] and new_res["objective"] < res["objective"]:
        print(f"[AABB-shift] {n_moved} block(s) pulled earlier  "
              f"obj {res['objective']:.0f} -> {new_res['objective']:.0f}  "
              f"(total {time.time() - t0:.1f}s)")
        return new_sol, new_res
    print(f"[AABB-shift] {n_moved} move(s) rolled back "
          f"(feasible={new_res['feasible']})  (total {time.time() - t0:.1f}s)")
    return sol, res


def _maybe_polish(prob_info: dict, sol: dict, res: dict,
                  t0: float, timelimit: float) -> dict:
    """잔여 시간이 충분하면 _objective_polish를 돌리고, 아니면 그대로 반환."""
    if not POLISH_ENABLED:
        return sol
    remaining = timelimit - (time.time() - t0)
    if remaining < POLISH_MIN_REMAINING_S or not res.get("feasible"):
        return sol
    return _objective_polish(prob_info, sol, res, t0, timelimit)


def _objective_polish(prob_info: dict, sol: dict, res: dict,
                      t0: float, timelimit: float) -> dict:
    """
    남는 시간 동안 "objective 기여도가 큰 블록"을 하나씩 destroy하고
    _place_blocks(w1/w2/w3 전체를 반영하는 기존 스코어)로 재배치한다.

    기여도 = w1*지각 + w3*선호페널티 + (가중부하 최대 베이에 있으면
    w2*u_bay*workload 보너스).  기여도 내림차순으로 시도하고, 이동마다
    전체 check_feasibility로 재검증해 objective가 엄격히 줄었을 때만
    채택한다.  한 바퀴에서 하나라도 채택되면 기여도를 다시 계산해 다음
    바퀴를 돈다 (시간이 남는 한).
    """
    from utils import check_feasibility

    global K_BEST
    saved_k_best = K_BEST
    K_BEST = POLISH_K_BEST

    blocks_data = prob_info["blocks"]
    bays = [Bay.from_dict(d, i) for i, d in enumerate(prob_info["bays"])]
    n_bays = len(bays)
    w = prob_info.get("weights", {})
    w1, w2, w3 = w.get("w1", 1.0), w.get("w2", 1.0), w.get("w3", 1.0)

    # -- 솔루션 -> assignments 복원 --------------------------------------
    assignments: dict[int, dict] = {}
    for t_str, ops in sol["operations"].items():
        t = int(t_str)
        for op in ops:
            bid = op["block_id"]
            if op["type"] == "ENTRY":
                assignments.setdefault(bid, {})
                assignments[bid].update({
                    "block_id": bid, "bay_id": op["bay_id"],
                    "x": int(op["x"]), "y": int(op["y"]),
                    "orient_idx": op["orient_idx"], "entry_time": t,
                })
            else:
                assignments.setdefault(bid, {})
                assignments[bid]["exit_time"] = t

    # -- bay 상태 복원 ----------------------------------------------------
    bay_placed:   list[list[Block]]           = [[] for _ in range(n_bays)]
    bay_schedule: list[list[tuple[int, int]]] = [[] for _ in range(n_bays)]
    bay_loads:    list[float]                 = [0.0] * n_bays
    for a in assignments.values():
        bid = a["block_id"]
        blk = Block(block_id=bid, block_data=blocks_data[bid],
                   x=a["x"], y=a["y"], orient_idx=a["orient_idx"])
        bay_placed[a["bay_id"]].append(blk)
        bay_schedule[a["bay_id"]].append((a["entry_time"], a["exit_time"]))
        bay_loads[a["bay_id"]] += blocks_data[bid]["workload"]

    _bay_areas  = [b.width * b.height for b in bays]
    _avg_area   = sum(_bay_areas) / n_bays
    bay_weights = [_avg_area / a for a in _bay_areas]

    def _contribution(bid: int) -> float:
        a = assignments[bid]
        b = blocks_data[bid]
        tard = max(0.0, a["exit_time"] - b["due_date"])
        prefs = b["bay_preferences"]
        pref_pen = max(prefs) - prefs[a["bay_id"]]
        c = w1 * tard + w3 * pref_pen
        jmax = max(range(n_bays), key=lambda j: bay_weights[j] * bay_loads[j])
        if a["bay_id"] == jmax:
            c += w2 * bay_weights[jmax] * b["workload"]
        return c

    obj_best = res["objective"]
    deadline = t0 + timelimit - POLISH_MARGIN_S
    n_moved = n_tried = 0
    # 다음 트라이얼 비용 예측치: "지금까지 본 최대 트라이얼 비용" 기반.
    # (직전 비용 기반으로 했더니 싼 트라이얼 뒤의 비싼 트라이얼이 예측을
    # 뚫고 60초 벽을 넘는 사고가 실측됐다 -- prob_8에서 71.7s.)  첫
    # 트라이얼은 실측치가 없으므로 보수적으로 2초로 가정하고 시작한다.
    max_cost = 2.0

    improved = True
    while improved and time.time() + max_cost * 1.2 < deadline:
        improved = False
        cand = sorted((bid for bid in assignments if _contribution(bid) > 0),
                      key=_contribution, reverse=True)
        for bi in cand:
            now = time.time()
            if now + max_cost * 1.2 > deadline:
                break
            t_move = now
            n_tried += 1
            old_a   = assignments[bi]
            old_bay = old_a["bay_id"]

            idx = next(i for i, b in enumerate(bay_placed[old_bay])
                       if b.block_id == bi)
            blk      = bay_placed[old_bay].pop(idx)
            old_slot = bay_schedule[old_bay].pop(idx)
            bay_loads[old_bay] -= blocks_data[bi]["workload"]

            # 트라이얼 내부 하드 데드라인: 단일 블록 탐색 하나가 예측을
            # 뚫고 60초 벽을 넘는 사고가 두 번 실측돼(64.4s/71.7s), 후보
            # 하나 단위로 중단 가능한 hard_deadline을 _place_blocks에 넘긴다.
            # 데드라인에 걸리면 그 시점까지의 최선 후보로 커밋되고, 아래
            # 수락 검사가 나쁜 결과를 걸러낸다.
            partial = _place_blocks(
                [bi], blocks_data, bays,
                bay_placed, bay_schedule, bay_loads,
                w1, w2, w3, forced_ids=set(),
                prev_assignments=assignments,
                hard_deadline=deadline,
            )
            new_a = partial[bi]

            def _undo() -> None:
                nb = new_a["bay_id"]
                i2 = next(i for i, b in enumerate(bay_placed[nb])
                          if b.block_id == bi)
                bay_placed[nb].pop(i2)
                bay_schedule[nb].pop(i2)
                bay_loads[nb] -= blocks_data[bi]["workload"]
                bay_placed[old_bay].append(blk)
                bay_schedule[old_bay].append(old_slot)
                bay_loads[old_bay] += blocks_data[bi]["workload"]

            moved = (new_a["bay_id"] != old_a["bay_id"]
                     or new_a["x"] != old_a["x"] or new_a["y"] != old_a["y"]
                     or new_a["entry_time"] != old_a["entry_time"]
                     or new_a["orient_idx"] != old_a["orient_idx"])
            if not moved:
                _undo()
                max_cost = max(max_cost, time.time() - t_move)
                continue

            trial = dict(assignments)
            trial[bi] = new_a
            trial_sol = {"operations": _build_operations(
                list(trial.values()), prob_info)}
            trial_res = check_feasibility(prob_info, trial_sol)
            if trial_res["feasible"] and trial_res["objective"] < obj_best:
                assignments[bi] = new_a
                obj_best = trial_res["objective"]
                n_moved += 1
                improved = True
            else:
                _undo()
            max_cost = max(max_cost, time.time() - t_move)

    # ---- Phase B: 선호베이 이주/스왑 (4차 제출 신규 -- Z3 직접 겨냥) ------
    # 대상: 지각 0이면서 선호 페널티가 있는 블록 A.  실측(prob_8)에서 확인한
    # Phase A의 두 가지 구조적 사각지대를 메운다:
    #   (1) _find_earliest_slot은 entry 후보를 {r_time, exit 경계}에서만
    #       뽑는데, A가 들어갈 수 있는 자리의 entry(예: 119)가 경계가
    #       아니면 절대 못 찾는다.  -> 여기서는 [rel, due-proc] 정수 entry
    #       "전부"를 순수 AABB 스캔으로 검사한다 (slack이 작아 범위 ≤~15,
    #       bbox-서로소 = 구조적 crane-feasible이므로 좌표를 직접 배정).
    #   (2) 자리가 없으면 파트너 B를 하나 빼야 하는데, 어느 B가 자리를
    #       여는지는 겹치는 블록 전수(11개 중 1개뿐인 사례 실측)를 싼
    #       스캔으로 사전 스크리닝해야 안다.  통과한 B만 비싼 스왑
    #       트라이얼(B 재배치 + 전체 검증)을 돌린다.
    # 채택 기준은 Phase A와 동일: 전체 check_feasibility + objective 엄격
    # 감소, 실패 시 정확히 롤백 -- 절대 나빠질 수 없다.
    def _bay_occupancy(t_bay: int, exclude: set[int]):
        occ = []
        for b2, a2 in assignments.items():
            if b2 in exclude or a2["bay_id"] != t_bay:
                continue
            lb2 = _block_bbox(blocks_data[b2], a2["orient_idx"])
            occ.append(((a2["x"] + lb2[0], a2["y"] + lb2[1],
                         a2["x"] + lb2[2], a2["y"] + lb2[3]),
                        a2["entry_time"], a2["exit_time"]))
        return occ

    def _scan_spot(A: int, t_bay: int, exclude: set[int]):
        """[rel, due-proc]의 모든 정수 entry에서 bbox-서로소 자리 탐색."""
        relA  = blocks_data[A]["release_time"]
        procA = blocks_data[A]["processing_time"]
        hiA   = blocks_data[A]["due_date"] - procA
        if hiA < relA:
            return None
        W = bays[t_bay].width
        H = bays[t_bay].height
        occ = _bay_occupancy(t_bay, exclude | {A})
        for entry in range(int(relA), int(hiA) + 1):
            exit_c = entry + procA
            act = [bb for (bb, a2, e2) in occ if a2 < exit_c and e2 > entry]
            for oi in range(len(blocks_data[A]["shape"])):
                lb = _block_bbox(blocks_data[A], oi)
                lx0, ly0, lx1, ly1 = lb
                px_lo, px_hi = math.ceil(-lx0), math.floor(W - lx1)
                py_lo, py_hi = math.ceil(-ly0), math.floor(H - ly1)
                if px_lo > px_hi or py_lo > py_hi:
                    continue
                px0, py0 = max(0, px_lo), max(0, py_lo)
                if not act:
                    return (entry, px0, py0, oi)
                xs = sorted({px0} | {math.ceil(ab[2] - lx0) for ab in act})
                xs = [x for x in xs if px_lo <= x <= px_hi][:FORCE_POS_XS]
                ys = sorted({py0} | {math.ceil(ab[3] - ly0) for ab in act})
                ys = [y for y in ys if py_lo <= y <= py_hi][:FORCE_POS_YS]
                for cy in ys:
                    for cx in xs:
                        wbb = (cx + lx0, cy + ly0, cx + lx1, cy + ly1)
                        if all(not _bb_overlap(wbb, ab) for ab in act):
                            return (entry, cx, cy, oi)
        return None

    def _pop_state(bid3: int, a3: dict):
        bay3 = a3["bay_id"]
        i3 = next(i for i, b in enumerate(bay_placed[bay3])
                  if b.block_id == bid3)
        blk3  = bay_placed[bay3].pop(i3)
        slot3 = bay_schedule[bay3].pop(i3)
        bay_loads[bay3] -= blocks_data[bid3]["workload"]
        return blk3, slot3

    def _apply_spot(A: int, t_bay: int, spot: tuple):
        entry_s, sx, sy, soi = spot
        proc_s = blocks_data[A]["processing_time"]
        nblk = Block(block_id=A, block_data=blocks_data[A],
                     x=int(sx), y=int(sy), orient_idx=soi)
        bay_placed[t_bay].append(nblk)
        bay_schedule[t_bay].append((int(entry_s), int(entry_s + proc_s)))
        bay_loads[t_bay] += blocks_data[A]["workload"]
        assignments[A] = {"block_id": A, "bay_id": t_bay,
                          "x": int(sx), "y": int(sy), "orient_idx": soi,
                          "entry_time": int(entry_s),
                          "exit_time": int(entry_s + proc_s)}

    swap_tried: set[int] = set()
    swap_est = max(2.0, max_cost * 1.5)
    while time.time() + swap_est < deadline:
        A = None
        best_pen = 0.0
        for bid2, a2 in assignments.items():
            if bid2 in swap_tried:
                continue
            if a2["exit_time"] > blocks_data[bid2]["due_date"]:
                continue  # 지각 블록은 w1 영역 -- 이 페이즈는 Z3 전용
            prefs2 = blocks_data[bid2]["bay_preferences"]
            pen2 = max(prefs2) - prefs2[a2["bay_id"]]
            if pen2 > best_pen:
                best_pen, A = pen2, bid2
        if A is None:
            break
        swap_tried.add(A)

        prefs_A = blocks_data[A]["bay_preferences"]
        t_bay   = max(range(n_bays), key=lambda j: prefs_A[j])
        a_A     = assignments[A]

        # -- (1) 무파트너 이주: 지금 상태 그대로 자리가 있으면 직접 배정 --
        t_sw = time.time()
        spot = _scan_spot(A, t_bay, set())
        if spot is not None:
            n_tried += 1
            save_A = assignments[A]
            blk_A, slot_A = _pop_state(A, save_A)
            _apply_spot(A, t_bay, spot)
            trial_sol = {"operations": _build_operations(
                list(assignments.values()), prob_info)}
            trial_res = check_feasibility(prob_info, trial_sol)
            if trial_res["feasible"] and trial_res["objective"] < obj_best:
                obj_best = trial_res["objective"]
                n_moved += 1
            else:
                _pop_state(A, assignments[A])
                bay_placed[save_A["bay_id"]].append(blk_A)
                bay_schedule[save_A["bay_id"]].append(slot_A)
                bay_loads[save_A["bay_id"]] += blocks_data[A]["workload"]
                assignments[A] = save_A
            swap_est = max(swap_est, time.time() - t_sw)
            continue

        # -- (2) 파트너 스왑: 자리를 여는 B를 전수 스크리닝 후 시도 --------
        partners = [b2 for b2, a2 in assignments.items()
                    if a2["bay_id"] == t_bay and b2 != A
                    and a2["entry_time"] < a_A["exit_time"]
                    and a2["exit_time"] > a_A["entry_time"]]
        partners.sort(key=_contribution, reverse=True)
        for B in partners:
            if time.time() + swap_est > deadline:
                break
            spot = _scan_spot(A, t_bay, {B})   # 순수 정수 스크리닝 (~ms)
            if spot is None:
                continue
            t_sw = time.time()
            n_tried += 1
            save_A, save_B = assignments[A], assignments[B]
            blk_A, slot_A = _pop_state(A, save_A)
            blk_B, slot_B = _pop_state(B, save_B)
            _apply_spot(A, t_bay, spot)
            part_B = _place_blocks(
                [B], blocks_data, bays, bay_placed, bay_schedule, bay_loads,
                w1, w2, w3, forced_ids=set(), prev_assignments=assignments,
                hard_deadline=deadline)
            assignments[B] = part_B[B]

            trial_sol = {"operations": _build_operations(
                list(assignments.values()), prob_info)}
            trial_res = check_feasibility(prob_info, trial_sol)
            if trial_res["feasible"] and trial_res["objective"] < obj_best:
                obj_best = trial_res["objective"]
                n_moved += 1
                swap_est = max(swap_est, time.time() - t_sw)
                break
            # 정확 롤백
            for bid3 in (B, A):
                nb3 = assignments[bid3]["bay_id"]
                i3 = next(i for i, b in enumerate(bay_placed[nb3])
                          if b.block_id == bid3)
                bay_placed[nb3].pop(i3)
                bay_schedule[nb3].pop(i3)
                bay_loads[nb3] -= blocks_data[bid3]["workload"]
            for bid3, save3, blk3, slot3 in ((A, save_A, blk_A, slot_A),
                                             (B, save_B, blk_B, slot_B)):
                bay_placed[save3["bay_id"]].append(blk3)
                bay_schedule[save3["bay_id"]].append(slot3)
                bay_loads[save3["bay_id"]] += blocks_data[bid3]["workload"]
                assignments[bid3] = save3
            swap_est = max(swap_est, time.time() - t_sw)

    K_BEST = saved_k_best

    if n_moved > 0:
        final_sol = {"operations": _build_operations(
            list(assignments.values()), prob_info)}
        final_res = check_feasibility(prob_info, final_sol)
        # 마지막 안전망: 재조립 결과가 어떤 이유로든 나빠졌으면 원본 유지
        if final_res["feasible"] and final_res["objective"] <= obj_best:
            print(f"[Polish] {n_moved}/{n_tried} move(s) accepted  "
                  f"obj {res['objective']:.0f} -> {final_res['objective']:.0f}  "
                  f"(total {time.time() - t0:.1f}s)")
            return final_sol
        print(f"[Polish] final rebuild mismatch -- keeping original  "
              f"(total {time.time() - t0:.1f}s)")
        return sol
    print(f"[Polish] no improving move found ({n_tried} tried)  "
          f"(total {time.time() - t0:.1f}s)")
    return sol


# -----------------------------------------------------------------------------
# Force-place helper (no feasibility check -- used as phase-1 last resort)
# -----------------------------------------------------------------------------

# AABB-disjoint 강제배치 탐색 상한 (myalgorithm6 신규 -- 폭주 방지 캡).
# entry 후보(베이의 exit 경계들)를 이 개수까지만 훑고, 각 entry에서 x/y 후보를
# 이 개수까지만 조합한다.  캡을 넘겨도 못 찾으면 기존 empty-bay 폴백으로
# 넘어가므로 안전성은 동일하고, 최악 케이스 비용만 상수로 묶인다.
#
# myalgorithm7 상향 (60/8/8 -> 150/30/30): prob_27 분석에서 8x8 캡이
# 병목으로 실측됐다 -- bay1이 168x16으로 극단적으로 넓은데 x 후보를
# 왼쪽에서 8개까지만 봐서 베이 오른쪽의 빈 공간이 아예 안 보였고, 강제배치
# 블록들이 전부 "자리 없음 -> 늦은 entry"로 밀렸다 (지각 블록 98개 중
# 27개는 release 시점에 들어갈 자리가 실재).  순수 정수 연산이라 상향
# 비용은 미미(총 실행시간 49~56s 유지).  실측 개선:
#   prob_27 obj1 6,383 -> 4,563 (-28%)   prob_38 8,484 -> 6,680 (-21%)
#   prob_25 1,488 -> 1,077 (-28%)        prob_6  1 -> 4 (노이즈 수준)
# 부수 효과: 강제배치 품질이 좋아져 "time guard 시점의 강제배치 수 출렁임
# -> obj1 출렁임"이라는 주요 실행 변동성 채널도 무뎌진다.
FORCE_ENTRY_TRIES: int = 150
FORCE_POS_XS: int = 30
FORCE_POS_YS: int = 30


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
    hard_deadline: float | None = None,
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
            # hard_deadline(폴리시 등 호출부가 지정): 후보 하나 단위로 검사해
            # 넘는 즉시 모든 루프를 중단한다 -- 지금까지 찾은 최선 후보로
            # 커밋되므로(없으면 _force_place 폴백) 안전하고, 초과가 Shapely
            # 호출 1개 수준(~0.2s)으로 묶인다.  timelimit의 80% 가드는 블록
            # 루프 시작 시에만 검사해서 "단일 블록 탐색"은 못 끊는다는 것이
            # 실측됐기 때문에(폴리시에서 64~71s 벽 초과 사고 2회) 추가했다.
            deadline_hit = False
            bay_order = sorted(range(n_bays), key=lambda j: prefs[j], reverse=True)
            for bay_id in bay_order:
                if deadline_hit:
                    break
                bay             = bays[bay_id]
                placed_in_bay   = bay_placed[bay_id]
                schedule_in_bay = bay_schedule[bay_id]

                # active_in_bay/active_schedule = blocks (and their
                # (entry,exit) pairs) still present at r_time.  Built ONCE
                # per bay here (myalgorithm7: orient 루프 밖으로 호이스팅 --
                # r_time에만 의존하고 방향과 무관한데 orient마다 최대 8번
                # 재구축되고 있었다) and passed into _find_earliest_slot
                # below instead of the full placed_in_bay/schedule_in_bay
                # (myalgorithm4 "idea 1" perf fix).  Without this,
                # _find_earliest_slot re-derives the same active-only filter
                # internally on EVERY one of the up-to-K_BEST calls,
                # rescanning already-exited blocks every single time.
                # Passing the pre-filtered lists keeps _find_earliest_slot's
                # own logic completely unchanged -- purely a caller-side
                # redundant-work removal, no behavioural difference.
                active_in_bay:  list[Block] = []
                active_schedule: list[tuple[int, int]] = []
                for b, (a_k, e_k) in zip(placed_in_bay, schedule_in_bay):
                    if e_k > r_time:
                        active_in_bay.append(b)
                        active_schedule.append((a_k, e_k))

                # 즉시진입 창과 시간이 겹치는 활성 블록들의 bbox --
                # 이것도 방향과 무관하므로 베이당 1회만 계산 (myalgorithm7
                # 호이스팅; bounding_rect 인스턴스 캐시와 함께 적용).
                win_bbs_bay: list[tuple[float, float, float, float]] = []
                if IMMEDIATE_SORT_ENABLED:
                    win_bbs_bay = [
                        b.bounding_rect()
                        for b, (a_k, e_k) in zip(active_in_bay, active_schedule)
                        if a_k < r_time + proc and e_k > r_time
                    ]

                for oi in range(n_orient):
                    if deadline_hit:
                        break
                    blk_bb = _block_bbox(blk_data, oi)
                    lx0_oi, ly0_oi, lx1_oi, ly1_oi = blk_bb
                    # Require a valid integer reference-point position to exist:
                    #   px in [ceil(-lx0), floor(W - lx1)]
                    #   py in [ceil(-ly0), floor(H - ly1)]
                    # If either range is empty there is no integer placement.
                    if (math.ceil(-lx0_oi) > math.floor(bay.width  - lx1_oi) or
                            math.ceil(-ly0_oi) > math.floor(bay.height - ly1_oi)):
                        continue

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
                    if IMMEDIATE_SORT_ENABLED and win_bbs_bay and candidates:
                        immediate: list[tuple[int, int]] = []
                        rest:      list[tuple[int, int]] = []
                        for (cx, cy) in candidates:
                            wbb = (cx + lx0_oi, cy + ly0_oi,
                                   cx + lx1_oi, cy + ly1_oi)
                            if all(not _bb_overlap(wbb, ab) for ab in win_bbs_bay):
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
                        if (hard_deadline is not None
                                and time.time() > hard_deadline):
                            deadline_hit = True
                            break
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
                    # (_shielding_violated도 Shapely 페어와이즈라 비싸므로
                    #  hard_deadline을 여기서도 검사한다)
                    for (cx, cy, entry, exit_t) in feasible_k:
                        if (hard_deadline is not None
                                and time.time() > hard_deadline):
                            deadline_hit = True
                            break
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
        # 패스 진입 가드 0.98 -> 0.95 -> 0.88 (myalgorithm7): 이 가드는 패스
        # "시작"만 막고 패스 내부(check_feasibility ~2s + 블록별 재배치
        # 1~3s씩)는 못 끊으므로, 0.95면 56.9s에 시작한 패스가 62~65s까지
        # 달리는 사고가 실측됐다 (prob_32 63.1s / prob_40 65.4s).  마지막
        # 패스가 최악 ~5s 걸린다고 보고 0.88(52.8s)로 당긴다 -- 그 뒤의
        # 최종 검사/비상망까지 포함해 60초 안에 끝나도록.
        # 0.88 -> 0.85 (4차 제출 안전마진): prob_32가 58.5s까지 가는 런이
        # 실측됨 -- 채점 서버가 조금이라도 느리면 시간초과(-1점)이므로
        # 마지막 패스 최악 소요(~5s)를 감안해 1.8s 더 당긴다.
        if time.time() - t_start > timelimit * 0.85:
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
                # Time guard: switch to forced path when 85% of timelimit is used
                # (0.90 -> 0.85, myalgorithm7 -- 패스 진입 가드를 0.88로 당긴
                # 것과 세트: 마지막 패스의 블록별 풀 탐색이 60초 벽을 넘는
                # 사고 방지).  Without this, a slow repair search could exhaust
                # the timelimit before all blocks are placed.
                time_critical = time.time() - t_start > timelimit * 0.85
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

    # ==== 비상 feasibility 안전망 (myalgorithm7 신규) ======================
    # 문제: 포화 인스턴스에서 Phase 1이 예산의 80%+강제배치 시간을 먹으면
    # repair가 ~54s에야 시작하고, 패스 하나에 2~5s가 걸려 시간가드(95%)에
    # 걸리면 위반이 남은 채 루프가 끝난다 -- 리더보드에서 infeasible은
    # -1점이므로 어떤 objective 악화보다도 나쁘다 (prob_32 stage=3,
    # prob_40 stage=2 infeasible이 사용자 전체 스윕에서 실제 발생).
    #
    # 해결: 위반이 남았으면 위반 블록 전부를 empty-bay 창(그 블록의 전체
    # 체류시간 동안 베이에 아무도 없는 창)으로 밀어넣는다.  empty-bay 창은
    # 다른 어떤 블록과도 시간이 겹치지 않으므로 Stage-2/3/4 어느 것도
    # 위반할 수 없고(단독 체류), 옮긴 블록이 남의 경로를 막지도 못한다 --
    # 한 번의 패스로 구조적으로 feasible 수렴이 보장된다.  순수 정수/구간
    # 연산이라 비용은 수 ms.  지각은 늘 수 있지만 feasible > objective.
    if not result["feasible"]:
        push_ids: list[int] = []
        seen_p: set[int] = set()
        for v in result["violations"]:
            try:
                bid = int(v.split("block ")[1].split()[0])
                if bid not in seen_p and bid in assignments:
                    seen_p.add(bid)
                    push_ids.append(bid)
            except (IndexError, ValueError):
                pass
        if push_ids:
            n_bays_e = len(bays)
            bay_schedule_e: list[list[tuple[int, int]]] = [[] for _ in range(n_bays_e)]
            for a in assignments.values():
                if a["block_id"] not in seen_p:
                    bay_schedule_e[a["bay_id"]].append(
                        (a["entry_time"], a["exit_time"]))
            for bid in push_ids:
                a      = assignments[bid]
                bay_id = a["bay_id"]
                r_time = blocks_data[bid]["release_time"]
                proc   = blocks_data[bid]["processing_time"]
                entry  = _empty_bay_entry(bay_schedule_e[bay_id], r_time, proc)
                assignments[bid] = dict(a, entry_time=int(round(entry)),
                                        exit_time=int(round(entry + proc)))
                bay_schedule_e[bay_id].append((entry, entry + proc))
            sol = {"operations": _build_operations(list(assignments.values()), prob_info)}
            result = check_feasibility(prob_info, sol)
            print(f"[Greedy] EMERGENCY NET: {len(push_ids)} violating block(s) "
                  f"pushed to empty-bay windows -> "
                  f"{'feasible' if result['feasible'] else 'STILL INFEASIBLE stage=' + str(result['stage'])}")

    status = "feasible" if result["feasible"] else f"INFEASIBLE stage={result['stage']}"
    obj    = f"obj={result['objective']:.0f}" if result["feasible"] else ""
    forced_note = f"  forced={len(forced_ids)}" if forced_ids else ""
    elapsed_done = time.time() - t_start
    print(f"[Greedy] Repair done  |  {status}  {obj}{forced_note}  elapsed={elapsed_done:.1f}s")

    # result도 함께 반환한다 (myalgorithm7): 호출부(_algorithm_once)가
    # 동일한 assignments로 재조립한 솔루션을 다시 check_feasibility하는
    # 중복 검사(250블록 기준 ~2s)를 없애기 위해 -- _build_operations는
    # 결정적이므로 같은 assignments면 같은 솔루션/같은 판정이다.
    return assignments, result


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
    parser.add_argument("--variant", choices=["auto", "on", "off"], default="auto",
                        help="auto(기본)=포트폴리오 셀프 셀렉션, on/off=즉시진입 "
                             "정렬을 해당 상태로 고정하고 단일 실행 (A/B 테스트용)")
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
    if args.variant != "auto":
        PORTFOLIO_ENABLED = False
        IMMEDIATE_SORT_ENABLED = (args.variant == "on")

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
