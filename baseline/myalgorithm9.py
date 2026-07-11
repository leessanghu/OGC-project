"""
myalgorithm9.py -- myalgorithm8.py + P4 밴드(util>=10.0) 전용
                   force-place 꼬리 개선 3종

목표: P4 밴드(util = 총 processing / (베이수 x max_due) >= 10)에서
TIME GUARD가 걸리면 남은 블록 전부가 _force_place로 넘어가는데, 이 꼬리가
obj1(지각)의 대부분을 만든다(project_phase1_forceplace_beats_search 계측,
prob37 89%).  myalgorithm9는 이 꼬리를 겨냥한 3개의 독립 개선을 추가한다
(모두 util>=10 게이트로 P1~P3 코드 경로는 완전히 그대로):

  (B+A) TAIL_PORTFOLIO + TAIL_LS -- Park, Lee, Park, Kim (1996, 서울대
      이경식 교수 "Spatial Block Scheduling in a Shipbuilding Company")의
      부분열거+분해 구조를 이식.  꼬리 순서를 EDD 및 4개 미세변형
      (edd_area/edd_proc/edd_wl/latest_start)으로 각각 순수 정수
      시뮬레이션해 꼬리 지각 합이 최소인 순서를 채택하고(포트폴리오),
      승자 순서 위에서 지각 기여 상위 블록을 접미사 증분 재시뮬레이션으로
      당겨보는 국소탐색(pull+swap 이동)을 추가로 돌린다.  실측(prob_33
      런내부 격리): 포트폴리오 -8.3~-9.2%, 그 위에 LS가 추가 -1.1~-1.4%.

  (C) FORCE_BESTFIT -- Jeong, Ju, Shen, Lee, Shin, Ryu (2018, IJAMT)
      "spatial arrangement algorithm considering free space and unplaced
      block"의 아이디어 이식 (SNU 조선해양공학과 계열, 이번 대회 주제와
      직결).  _force_place가 각 진입시각에서 첫 disjoint 위치를 그냥
      쓰던 것을, 후보 최대 5개를 모아 "다른 블록이 못 쓸 만큼 작은
      자투리(dead-sliver)"를 가장 적게 만드는 후보로 교체.  실측(prob_23
      런내부, 2런 평균): obj1 742.5->731 (-1.5%), 부수적으로 재현성도
      향상(두 런이 완전히 동일값).

  안전장치: 세 개선 모두 P4 게이트 밖에서는 코드 경로가 아예 안 바뀌고,
  게이트 안에서도 (B+A)는 "현행 EDD 대비 엄격 개선일 때만 교체"라
  회귀가 구조적으로 불가능하다.  (C)는 동점(자투리 차이 없음)이면 기존
  bottom-left 선택과 완전히 동일 -- 순수 첨가형.

  기각된 실험 (참고용, 코드에 없음): P4 스코어에 entry 항을 더해
  "entry 최소화 사전식 정렬"을 시도했으나(ADP cost-to-go 아이디어),
  부하균형(obj2)이 무너지고 그 부작용이 지각까지 악화시켜 6개 프록시 중
  4개에서 obj1이 나빠져 기각함.

===============================================================================
myalgorithm8.py 원문 (아래는 누적된 원본 설명, 유지)
===============================================================================

myalgorithm8.py -- myalgorithm7.py + 여유형(obj1=0) 인스턴스 전용
                   목표 배정(target-assignment) 유도 폴리시

목표: myalgorithm7 실측에서 obj1=0(지각 없음)인 "여유형" 인스턴스
(prob_2/3/8류, 실제 P1/P2 프록시)는 objective가 사실상 순수 배정 문제로
붕괴한다 -- utils.check_feasibility의 obj2/obj3 공식이 베이 배정에만
의존하고 기하/시간에는 의존하지 않기 때문(모듈 하단 [E] 참조).  기하를
무시한 배정만의 로컬서치로 하한을 구해보면 현재 해 대비 49~91% 남은
개선폭이 있었다(prob_2/3/8 실측).

원인 진단(실측): _objective_polish의 Phase A(destroy-reinsert)가 전체
폴리시 예산을 거의 다 써버려서(trial당 전체 check_feasibility 비용이
큼) obj2는 몇 라운드만에 근사 최적에 도달하지만 obj3(선호 페널티)를
전담하는 Phase B(스왑)가 실행될 시간이 거의 안 남는 것을 prob_2에서
직접 확인했다 (obj2 931->148 vs obj3 240->242, 사실상 정체).

변경 (myalgorithm7.py 대비 추가된 부분만, Phase 1/2/AABB-shift/Phase A
내부 로직은 전혀 손대지 않음):
  [E] _solve_target_assignment(prob_info, bays) -- 기하 무시, "블록->베이"
      배정만으로 실제 obj2+obj3 공식을 최소화하는 목표 배정을 로컬서치로
      계산한다 (멀티 리스타트 + 단일이동 + 페어스왑, 시간상한 있음).
      각 블록은 bbox가 맞는 베이로만 이동 가능(_feasible_bays_for_block).
  [F] Phase B 목표 개선: 스왑 대상 베이를 "최선호 베이 1개 고정"에서
      [E]의 목표 배정으로 교체(가능한 경우), 후보 선정도 선호페널티만
      보던 것에서 Phase A와 동일한 _contribution(부하 보너스 포함)으로
      교체 -- 부하 균형까지 함께 고려한 이주를 시도할 수 있게 됨.
  [G] Phase B 예산 예약 + Phase A 정체 감지: obj1==0(여유형)이면 Phase A
      데드라인을 앞당겨 Phase B에 정적으로 예약한다.  obj1>0 구간은 정적
      예약 대신 "PHASE_A_STALL_S초 동안 수락 없으면 Phase A 조기 종료"의
      적응형 양보를 쓴다 -- 격리 실험에서 정적 예약은 Phase A가 생산적인
      인스턴스(prob_25/23)에 순손실, A가 무익한 인스턴스(prob_36/4)에만
      이득이었기 때문 (수락이 나오면 카운터 리셋이라 생산적인 A는 계속 돈다).
  [H] Phase C: 지각 감소 스왑 (지각 블록 수 <= TARDY_SWAP_MAX_TARDY 전용).
      지각 블록 A를 현재 entry보다 이른 슬롯으로 옮긴다 -- 빈자리(0퇴출)
      직접 이동 또는 비지각 파트너 B 하나를 빼는 1퇴출 스왑.  v1은 완전
      정시 구제만 시도했으나(near-feasible 실측: prob_1 171k->33k, prob_10
      202k->116k), v2는 entry 탐색을 [rel, 현재 entry-1]로 확장해 정시가
      불가능한 중간포화(P3/P4급, 지각 수십~수백) 블록도 "이른 만큼 w1*유닛"
      을 벌 수 있게 일반화했다 (진단: 이 유형 잔여 지각의 23~59%가 1퇴출
      자리로 회수 가능).  reservation과 결합해 Z1(스왑)과 Z3(pen2 Phase B에
      시간 확보)를 동시에 줄인다.

    참고: [F]의 목표배정-유도 Phase B는 obj1==0 여유형에만 적용한다.
    near-feasible(0<obj1<=임계값)까지 확장해봤으나 시리얼 무경합 격리
    실험에서 기존 pen2 Phase B 대비 이득이 없어(오히려 무력화 위험) 되돌렸다.

안전장치: [E]~[H] 전부 "힌트/시도"일 뿐, 실제 이동 채택 기준은 기존과
동일한 전체 check_feasibility 재검증 + objective 엄격 감소뿐이다.  즉 목표
배정이 기하적으로 틀렸거나 스왑이 쓸모없어도 결과가 나빠질 수 없다(단조
개선 보장은 myalgorithm7과 동일하게 유지됨).

===============================================================================
myalgorithm7.py 원문 (아래는 myalgorithm4 이후 누적된 원본 설명, 유지)
===============================================================================

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
import random
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


# ==========================================================================
# 목표 배정(target-assignment) 솔버 -- myalgorithm8 신규 ([E] 모듈 docstring)
# ==========================================================================
#
# obj1(지각)=0인 인스턴스는 objective = w2*obj2 + w3*obj3 이고, 둘 다
# "블록 -> 베이" 배정에만 의존한다(utils.check_feasibility 공식 참조,
# 기하/시간 무관).  그래서 기하를 완전히 무시하고 이 배정 문제만 따로
# 풀면 몇백 ms 안에 실제 objective 공식 기준 매우 좋은 해를 얻을 수 있고,
# 이걸 Phase B(스왑 폴리시)의 이주 목표로 쓴다.
#
# 이 결과는 어디까지나 "힌트"다 -- 실제 채택은 항상 기존과 동일한 전체
# check_feasibility + objective 엄격 감소 기준을 통과해야 하므로, 힌트가
# 기하적으로 실현 불가능해도 결과가 나빠지지 않는다.
TARGET_ASSIGN_ENABLED: bool = True
TARGET_ASSIGN_TIME_BUDGET_S: float = 1.5
TARGET_ASSIGN_RESTARTS: int = 6
TARGET_ASSIGN_SEED: int = 20260709  # 고정 시드 -- 실행마다 동일한 힌트 보장


def _feasible_bays_for_block(block_data: dict, bays: list[Bay]) -> list[int]:
    """block_data가 정수 좌표로 bbox가 들어맞는 베이 id 목록(방향 무관 OR)."""
    result = []
    for bay in bays:
        for oi in range(len(block_data["shape"])):
            bb = _block_bbox(block_data, oi)
            lx0, ly0, lx1, ly1 = bb
            if (math.ceil(-lx0) <= math.floor(bay.width - lx1) and
                    math.ceil(-ly0) <= math.floor(bay.height - ly1)):
                result.append(bay.id)
                break
    return result


def _solve_target_assignment(prob_info: dict, bays: list[Bay],
                             time_budget: float = TARGET_ASSIGN_TIME_BUDGET_S,
                             restarts: int = TARGET_ASSIGN_RESTARTS,
                             seed: int = TARGET_ASSIGN_SEED,
                             pinned: dict[int, int] | None = None) -> list[int] | None:
    """
    기하/시간을 완전히 무시하고 "블록 -> 베이" 배정만으로 실제
    obj2+obj3 공식(utils.check_feasibility와 동일)을 최소화하는 목표
    배정을 로컬서치로 계산한다.  각 블록은 _feasible_bays_for_block로
    구한, bbox가 맞는 베이로만 이동 가능(물리적으로 못 들어가는 베이를
    목표로 제안하지 않는다).

    멀티 리스타트(첫 리스타트는 "가능한 베이 중 최선호" 탐욕해, 나머지는
    고정 시드 의사난수 초기해) + 단일이동/페어스왑 로컬서치, 매 이동은
    증분(O(1)) 평가.  시간상한(time_budget) 안에서 최선을 반환하고,
    인스턴스가 malformed거나(블록 하나라도 갈 수 있는 베이가 없음)
    n_bays<=1이면 각각 None / [0]*n을 반환한다.

    pinned : {block_id: bay_id} -- 이 블록들은 지정된 베이에 고정되어
        로컬서치가 옮기지 못한다 (near-feasible 인스턴스에서 현재 지각
        블록을 현 베이에 고정해, 지각은 Phase C에 맡기고 타겟배정은
        비지각 블록의 Z3만 겨냥하게 하는 용도).  고정 베이는 그 블록의
        부하로서 Z2 계산에는 그대로 반영된다.

    반환값은 힌트일 뿐이다 -- 호출부는 항상 전체 check_feasibility로
    재검증한 뒤 objective가 엄격히 개선될 때만 채택하므로, 이 함수가
    나쁘거나 기하적으로 실현 불가능한 힌트를 내놔도 결과는 나빠지지
    않는다.
    """
    blocks_data = prob_info["blocks"]
    n = len(blocks_data)
    n_bays = len(bays)
    if n_bays <= 1:
        return [0] * n

    w = prob_info.get("weights", {})
    w2, w3 = w.get("w2", 1.0), w.get("w3", 1.0)

    feas = [_feasible_bays_for_block(b, bays) for b in blocks_data]
    if any(not f for f in feas):
        return None
    # 고정 블록은 후보 베이를 지정 베이 하나로 좁힌다 -- 아래 로컬서치의
    # `len(feas[i]) < 2` 가드와 `jk not in feas[i]` 검사가 자동으로 이
    # 블록들을 이동 대상에서 제외한다 (초기해 생성도 이 단일 후보를 쓴다).
    if pinned:
        for bid, bay_id in pinned.items():
            if 0 <= bid < n and bay_id in feas[bid]:
                feas[bid] = [bay_id]

    wl    = [b["workload"] for b in blocks_data]
    prefs = [b["bay_preferences"] for b in blocks_data]
    smax  = [max(p) for p in prefs]

    bay_areas = [bays[j].width * bays[j].height for j in range(n_bays)]
    avg_area  = sum(bay_areas) / n_bays
    u = [avg_area / a for a in bay_areas]

    def _obj23(loads: list[float], pref_pen_sum: float) -> float:
        L = math.floor(max(
            abs(u[a] * loads[a] - u[b] * loads[b])
            for a in range(n_bays) for b in range(n_bays) if a != b
        ))
        return w2 * L + w3 * pref_pen_sum

    rng = random.Random(seed)
    deadline = time.time() + time_budget
    best_assign: list[int] | None = None
    best_val = float("inf")
    check_ctr = 0

    def _time_up() -> bool:
        nonlocal check_ctr
        check_ctr += 1
        return check_ctr % 2000 == 0 and time.time() > deadline

    for r in range(restarts):
        if time.time() > deadline:
            break
        if r == 0:
            assign = [max(feas[i], key=lambda j: prefs[i][j]) for i in range(n)]
        else:
            assign = [rng.choice(feas[i]) for i in range(n)]

        loads = [0.0] * n_bays
        pen   = 0.0
        for i, j in enumerate(assign):
            loads[j] += wl[i]
            pen += smax[i] - prefs[i][j]
        cur = _obj23(loads, pen)

        improved = True
        while improved and time.time() < deadline:
            improved = False
            for i in range(n):
                if len(feas[i]) < 2:
                    continue
                oj = assign[i]
                for j in feas[i]:
                    if j == oj:
                        continue
                    loads[oj] -= wl[i]; loads[j] += wl[i]
                    dp = prefs[i][oj] - prefs[i][j]
                    v = _obj23(loads, pen + dp)
                    if v < cur - 1e-9:
                        cur = v; pen += dp; assign[i] = j; oj = j
                        improved = True
                    else:
                        loads[j] -= wl[i]; loads[oj] += wl[i]
                if _time_up():
                    break
            if time.time() > deadline:
                break
            for i in range(n):
                ji = assign[i]
                for k in range(i + 1, n):
                    jk = assign[k]
                    if ji == jk or jk not in feas[i] or ji not in feas[k]:
                        continue
                    dl = wl[i] - wl[k]
                    loads[ji] -= dl; loads[jk] += dl
                    dp = ((prefs[i][ji] - prefs[i][jk]) +
                          (prefs[k][jk] - prefs[k][ji]))
                    v = _obj23(loads, pen + dp)
                    if v < cur - 1e-9:
                        cur = v; pen += dp
                        assign[i], assign[k] = jk, ji
                        ji = jk
                        improved = True
                    else:
                        loads[ji] += dl; loads[jk] -= dl
                    if _time_up():
                        break
                if time.time() > deadline:
                    break
        if cur < best_val:
            best_val = cur
            best_assign = list(assign)

    return best_assign


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


# ==========================================================================
# myalgorithm9 신규: force-place 꼬리 순서 부분열거 (tail-order portfolio)
# ==========================================================================
#
# 배경 (project_phase1_forceplace_beats_search 계측): P4 밴드(util>=10.7)
# 에서 TIME GUARD가 걸리면 남은 블록(prob37: 148/250개)이 전부
# _force_place로 넘어가는데, 이 꼬리가 obj1(지각)의 89%를 만든다.  현재
# 꼬리는 EDD 순서 하나로만, 블록당 한 번씩 그리디 커밋된다 -- 순서가
# 공간 점유 패턴을 결정하므로 순서 선택 자체가 큰 레버인데 열거가 없다.
#
# 처방 (Park, Lee, Park, Kim 1996, "Spatial Block Scheduling in a
# Shipbuilding Company"의 부분열거+분해 구조 이식): 스케줄 결정(=꼬리
# 순서)을 소수의 후보로 열거하고, 각 후보를 빠른 공간 휴리스틱
# (_force_place, 순수 정수 연산)으로 시뮬레이션한 뒤 꼬리 지각 합이
# 최소인 순서를 커밋한다.  후보:
#   v0 edd          -- 현행 순서 (반드시 포함: 절대 현행보다 나빠질 수
#                      없음을 구조적으로 보장, 엄격 개선일 때만 교체)
#   v1 area_desc    -- 풋프린트 큰 블록 먼저 (큰 블록은 나중에 자리가
#                      없어 밀리는 손실이 크므로 선배치; 2D 패킹 상식)
#   v2 latest_start -- (due - proc) 오름차순 (여유 없는 블록 먼저)
#
# 비용 안전장치: v0 시뮬 소요를 실측해, 추가 변형은 "예상 소요를 더해도
# 예산의 TAIL_PORTFOLIO_TIME_FRAC을 안 넘을 때"만 시도한다.  넘으면
# 그냥 v0(현행)로 커밋 -- 기존 동작과 동일.
#
# 게이트: util = total_proc / (n_bays * max_due) >= TAIL_PORTFOLIO_UTIL_MIN.
# 실측 분포(instance_feature_analysis.json)에서 prob_29(8.898)와
# prob_34(10.672) 사이 갭이 있어 10.0이면 P4 밴드/초포화만 걸리고
# P1~P3는 코드 경로가 아예 바뀌지 않는다.  (밴드 밖은 꼬리 자체가
# 거의 없거나 obj1 비중이 작아 폴리시 시간을 아끼는 쪽이 낫다.)
TAIL_PORTFOLIO_ENABLED: bool = True
TAIL_PORTFOLIO_UTIL_MIN: float = 10.0
# 추가 변형 시뮬레이션을 허용하는 예산 상한 비율 (t_start 기준).
TAIL_PORTFOLIO_TIME_FRAC: float = 0.93

# --------------------------------------------------------------------------
# (A) 순서 국소탐색 -- 부분열거의 확장.  승자 순서에서 지각 기여가 큰
# 블록을 더 앞 위치로 당겨보는 이웃 이동을, "순서의 j번째를 바꾸면 앞
# j-1개의 배치는 그대로"라는 구조를 이용해 접미사만 재시뮬레이션(비용
# (L-j)/L)으로 평가한다.  지각 블록은 순서 뒤쪽에 몰려 있으므로 당기기
# 이동의 접미사는 짧다 = 싸다.  채택은 꼬리 지각 합의 엄격 감소일 때만
# (포트폴리오와 동일한 단조 보장), 예산은 "가드 시점 잔여 시간의
# TAIL_LS_REMAIN_FRAC"까지만 -- 나머지 절반은 repair/폴리시 몫으로 남긴다.
TAIL_LS_ENABLED: bool = True
TAIL_LS_REMAIN_FRAC: float = 0.5
# 한 패스에서 이동을 시도할 지각 기여 상위 블록 수.
TAIL_LS_TOP_BLOCKS: int = 8
# 이동 후보 종류 (prob_33 격리 비교로 채택 결정):
#   "pull"    -- 고정 위치(i-1, i-L/8, i/2, 0)로 당기기 (v1 기본)
#   "cluster" -- 순서상 "due >= 자기 due"가 처음 나오는 위치(=자기 due
#                클러스터의 머리)로 삽입 -- EDD tie-break 논리상 블록이
#                '있어야 할' 자리로 직행하는 조준된 당기기
#   "swap"    -- 가까운 비지각 선행 블록(최대 3개)과 자리 교환 --
#                삽입과 달리 중간 블록들을 밀지 않는 국소 교란
# prob_33 격리 비교(2런/구성, V1 가드 기준): pull -0.68% < pull+cluster
# -0.85% < pull+swap -0.89% (시도당 채택 효율도 swap 최고 ~7%).  cluster는
# 채택 수는 가장 많지만(7) 개별 폭이 작았다.  꼬리 크기 confound가 있어
# 마진은 얇음 -- 재검토 시 cluster 재후보.
TAIL_LS_MOVES: tuple = ("pull", "swap")

# algorithm()이 인스턴스 로드 직후 게이트를 평가해 설정하는 런타임 플래그.
_TAIL_PORTFOLIO_ACTIVE: bool = False

# --------------------------------------------------------------------------
# (C) free-space 인지 force-place -- Jeong, Ju, Shen, Lee, Shin, Ryu (2018,
# IJAMT) "spatial arrangement algorithm considering free space and unplaced
# block"의 핵심 아이디어 이식: 배치 후보를 "최이른 진입"만으로 고르지 말고,
# 남는 빈 공간의 모양(다른 블록이 못 쓸 만큼 작은 자투리를 만드는지)도
# 함께 본다.
#
# 배경 (prob_23 진단): 베이가 단 2개(64x25 / 150x23)로 극단적으로
# 비대칭이고 TIME GUARD가 블록 30~40/100에서 조기 발동해 인스턴스의
# 40~60%가 force-place로 넘어간다.  그런데 _force_place는 각 진입시각의
# 후보 중 bottom-left 스캔에서 "가장 먼저 찾은" 위치를 그냥 채택한다 --
# 남는 자투리가 이후 블록이 못 쓸 만큼 작아도 무시.  베이 수가 적을수록
# (여기서는 단 2개) 한 번의 나쁜 자투리가 그 베이 전체의 활용도를
# 떨어뜨리는 영향이 크다.
#
# 처방: 같은 진입시각에서 disjoint 후보를 최대 FORCE_BESTFIT_MAX_CAND개
# 모으고, 각 후보가 만드는 우측/상단 gap이 "0보다 크고 인스턴스의 전형적
# 블록 크기(_MIN_BLOCK_DIM, 인스턴스 전체 블록 최소변의 20th percentile)
# 보다 작은" 자투리를 만드는지 세어 dead-sliver 수가 가장 적은 후보를
# 채택한다.  동률이면 기존 bottom-left 순서(스캔에서 먼저 나온 것)를
# 유지 -- 자투리 차이가 없으면 기존 동작과 완전히 동일.
FORCE_BESTFIT_ENABLED: bool = True
FORCE_BESTFIT_MAX_CAND: int = 5

# algorithm()이 인스턴스 로드 직후 계산해 설정 (0.0 = 미계산/비활성).
_MIN_BLOCK_DIM: float = 0.0


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
    global IMMEDIATE_SORT_ENABLED, _TAIL_PORTFOLIO_ACTIVE, _MIN_BLOCK_DIM
    t0 = time.time()

    # -- P4 밴드 게이트 (myalgorithm9, TAIL_PORTFOLIO 모듈 주석 참조) ------
    _TAIL_PORTFOLIO_ACTIVE = False
    _MIN_BLOCK_DIM = 0.0
    if TAIL_PORTFOLIO_ENABLED:
        try:
            _bd = prob_info["blocks"]
            _bvals = list(_bd.values()) if isinstance(_bd, dict) else list(_bd)
            _total_proc = sum(b["processing_time"] for b in _bvals)
            _max_due = max(b["due_date"] for b in _bvals)
            _util = _total_proc / (len(prob_info["bays"]) * _max_due)
            _TAIL_PORTFOLIO_ACTIVE = _util >= TAIL_PORTFOLIO_UTIL_MIN
            print(f"[TailPortfolio] util={_util:.3f} "
                  f"(threshold {TAIL_PORTFOLIO_UTIL_MIN}) -> "
                  f"{'ACTIVE' if _TAIL_PORTFOLIO_ACTIVE else 'off'}")
            if _TAIL_PORTFOLIO_ACTIVE and FORCE_BESTFIT_ENABLED:
                # 인스턴스 전체 블록의 (orientation 0) 최소변 20th percentile
                # -- "전형적으로 못 쓰는 자투리" 임계값 (모듈 상단 (C) 참조).
                _dims = sorted(
                    min(bb[2] - bb[0], bb[3] - bb[1])
                    for bb in (_block_bbox(bv, 0) for bv in _bvals)
                )
                _MIN_BLOCK_DIM = _dims[max(0, len(_dims) // 5)]
                print(f"[ForceBestFit] min_block_dim={_MIN_BLOCK_DIM:.1f} "
                      f"(20th pct of {len(_dims)} blocks)")
        except Exception as exc:  # 게이트 실패 = 기존 동작 (안전)
            print(f"[TailPortfolio] gate check failed ({exc!r}) -- off")

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

# ==========================================================================
# Phase B 예산 예약 -- myalgorithm8 신규 ([G] 모듈 docstring 참조)
# ==========================================================================
#
# 실측(prob_2): Phase A(destroy-reinsert)가 폴리시 예산을 거의 다 써버려서
# obj2는 몇 라운드 안에 근사 최적(148, LB=144)에 도달하지만 obj3(선호
# 페널티)를 전담하는 Phase B가 실행될 시간이 거의 안 남아 정체됐다
# (obj3 240 -> 242, 사실상 무변화).  obj1(지각)이 이미 0인 인스턴스에서만
# -- 그래야 리더보드 순위가 obj2/obj3로만 갈리는 "여유형"이 확실하므로 --
# Phase A의 내부 데드라인을 앞당겨 Phase B에 최소 시간을 예약한다.
#
# obj1>0인 인스턴스는 reserved=0 이라 Phase A/B 데드라인이 기존과 완전히
# 같다 -- 포화형 인스턴스(prob_23/25/27/38/39급)는 이 예약 로직으로 전혀
# 영향받지 않는다 (동작 100% 동일).
PHASE_B_RESERVE_ENABLED: bool = True

# 폴리시 전체 잔여시간 중 Phase B에 예약하는 비율 (obj1==0일 때만 적용).
PHASE_B_RESERVE_FRAC: float = 0.5

# 예약 시간의 절대 상한(초) -- 잔여시간이 아주 넉넉해도 이 이상은 안 묶는다.
PHASE_B_RESERVE_MAX_S: float = 20.0

# Phase A 정체 감지: 마지막 수락 이후 이 시간(초) 동안 수락이 없으면
# Phase A를 조기 종료하고 남은 시간을 Phase C/B에 넘긴다.  격리 실험
# (고정 시작해, 동일 예산) 실측 근거:
#   - Phase A가 생산적인 인스턴스(prob_25 T 1203->1055, prob_23): A의
#     시간을 정적으로 뺏는 예약은 순손실 (C가 회수 못함) -> 수락이 계속
#     나오는 동안은 A가 끝까지 돈다 (타이머가 수락마다 리셋).
#   - Phase A가 무익한 인스턴스(prob_36: 전 트라이얼 기각, prob_4): A가
#     창 전체를 소모해 C/B가 못 돌던 것이 손실 (정적 예약 실험에서
#     prob_36 -4.7% 확인) -> 몇 초 안에 양보.
# "연속 N회 기각" 방식은 250블록급(트라이얼 2~3s)에서 N회가 쌓이기 전에
# 창이 끝나 무용함이 실측돼 시간 기반으로 교체했다.
# 이 양보는 Phase C 대상(지각 블록 수 <= TARDY_SWAP_MAX_TARDY) 또는
# obj1==0일 때만 켠다 -- 그 외(초포화형)에서는 Phase A가 myalgorithm7과
# 완전히 동일하게 동작한다.
PHASE_A_STALL_S: float = 3.5

# ==========================================================================
# Phase C: 지각 감소 스왑 -- myalgorithm8 신규 (v2: P3/P4 중간포화 일반화)
# ==========================================================================
#
# 배경 1 (near-feasible, obj1 1~8: prob_1/4/6/7/10~13/15/29 실측):
#   잔여 지각 블록은 "단일 이동으로는 정시 자리가 없지만, 자리를 막는
#   비지각 파트너 B를 하나 빼면 정시 배치가 가능"한 경우가 대부분
#   (거의 전부 1개 퇴출로 구제 가능).  Phase A(단일이동)와 Phase B(비지각
#   전용)가 못 메우는 사각지대.
#
# 배경 2 (v2 일반화 -- P3/P4급 중간포화, obj1 146~1,258: prob_23/24/25/30/36
# 진단 실측):
#   - release 시점 "빈자리"(0퇴출)는 사실상 0개 -- 좌압축/캡 계열이 수확할
#     과실은 소진됨.
#   - 반면 "블록 하나만 치우면 열리는 release 시점 자리"(1퇴출)는 인스턴스당
#     10~16블록씩 실재하고, 그 잠재 Z1 감소가 현재 objective의 23~59%.
#   - 지각이 커서 "완전 정시"가 불가능한 블록도, entry를 이른 만큼
#     w1(13k~27k)/유닛씩 벌 수 있다.
#   그래서 v2는 (a) entry 탐색을 [rel, due-proc]에서 [rel, 현재 entry-1]로
#   확장하고(스캔이 오름차순이라 정시 슬롯이 있으면 그것부터 찾음 -- 기존
#   near-feasible 동작의 상위호환), (b) 게이트를 obj1<=30에서 "지각 블록 수"
#   기반으로 교체하고, (c) 비싼 트라이얼을 이득 큰 곳에 먼저 쓰도록 싼 정수
#   프로브로 추정 이득을 계산해 내림차순으로 시도하며, (d) 프로브 자체도
#   delay 상위 TARDY_SWAP_SCREEN_MAX개로 제한한다(지각 70~111개 인스턴스는
#   전수 프로빙만으로 확보한 창을 다 쓰는 것이 격리 실험으로 실측됨).
#
# Phase A와의 시간 배분: obj1==0(Phase B 타겟배정)이 아닌 한 정적 예약은
# 쓰지 않고, Phase A가 PHASE_A_STALL_S초 동안 수락이 없으면 조기 종료해
# 남은 시간을 Phase C에 넘기는 적응형 방식을 쓴다 -- 정적 예약은 격리
# 실험에서 Phase A가 생산적인 인스턴스(prob_23/25)에 순손실이었다.
#
# 실측(격리, Phase-1 변동성 제거): prob_24 지각 146->140(6유닛), objective
# -3.4%.  나머지 진단 인스턴스(23/25/30/36)는 이번 상태에서 old와 동일
# (Phase A가 이미 그 지점을 충분히 다뤄 Phase C가 추가로 찾을 게 없었음) --
# 즉 손해는 없지만 진단 당시 추정한 잠재치(23~59%)가 항상 실현되진 않는다.
# 전체 파이프라인 반복 실행에서는 Phase 1 자체의 wall-clock 변동성(같은
# 코드로 prob_24가 2.4M~4.8M까지 관측됨, [[feedback_myalgorithm8_validation]]
# 참조)이 이 폴리시 단계의 효과보다 훨씬 커서 종단간 비교가 어렵다.
#
# 채택 기준은 Phase A/B와 동일: 매 이동 전체 check_feasibility + objective
# 엄격 감소, 실패 시 정확 롤백 -- 절대 나빠질 수 없다(단조 개선).
TARDY_SWAP_ENABLED: bool = True

# 지각 블록 수가 이 값 이하일 때만 Phase C를 켠다.  P3/P4급 중간포화는
# 24~111블록이라 포함되고, 초포화형(prob_38/40급, 지각 수백 블록 + V1이
# 예산 대부분을 소비)은 제외 -- 트라이얼당 전체 재검증 비용 때문에 지각
# 블록이 아주 많으면 스왑으로 유의미하게 줄일 수 없고, 그 시간은 Phase A에
# 두는 것이 낫다.
TARDY_SWAP_MAX_TARDY: int = 120

# 블록 하나당 1퇴출 스왑 트라이얼 상한 (전체 check_feasibility가 비싸므로
# 한 블록에 예산을 다 쓰지 않고 여러 블록에 분산한다).
TARDY_SWAP_TRIALS_PER_BLOCK: int = 3

# 스크리닝(프로브) 대상 상한: delay 상위 이 개수의 지각 블록만 _best_option
# 프로브를 돌린다.  지각 블록이 70~111개인 인스턴스(prob_30/36)에서 전수
# 프로빙이 정체감지로 확보한 창(2~5s)을 통째로 먹어 트라이얼이 0개가 되는
# 것이 격리 실험에서 실측됐다.  진단상 회수 가능 이득은 delay 상위권에
# 집중돼 있어(top5가 top40 잠재치의 대부분) 상위 30개로 충분하다.
TARDY_SWAP_SCREEN_MAX: int = 30


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

    # -- 목표 배정 힌트 계산 (myalgorithm8 신규, [E] 참조) --------------------
    # obj1==0인 "여유형"에서만 계산한다.  near-feasible(0<obj1<=임계값)에도
    # 확장해봤으나(타겟배정 pinning 포함), 시리얼 무경합 3-way 격리 실험에서
    # 기존 pen2 Phase B 대비 깨끗한 이득이 전혀 없고(노이즈 실행에선 오히려
    # Phase B를 무력화하는 경우도 관측) 검증되지 않은 리스크만 있어 obj1==0
    # 전용으로 되돌렸다.  near-feasible의 Z3 개선은 아래 Phase B/C 예약이
    # 기존 pen2 Phase B에 시간을 확보해주는 것으로 충분히 달성된다(prob_4
    # 172.7k->138.6k 실측).  obj1>0인 인스턴스는 이 계산의 벽시계 비용조차
    # 물지 않는다.  폴리시 최소 예비(POLISH_MIN_REMAINING_S)는 건드리지
    # 않고 그 위의 여유분에서만 예산을 쓴다.
    target_bay = None
    if TARGET_ASSIGN_ENABLED and res.get("obj1", 0.0) == 0.0:
        budget = min(TARGET_ASSIGN_TIME_BUDGET_S,
                     max(0.0, remaining - POLISH_MIN_REMAINING_S))
        if budget > 0.05:
            bays = [Bay.from_dict(d, i) for i, d in enumerate(prob_info["bays"])]
            target_bay = _solve_target_assignment(prob_info, bays, time_budget=budget)
    return _objective_polish(prob_info, sol, res, t0, timelimit, target_bay)


def _objective_polish(prob_info: dict, sol: dict, res: dict,
                      t0: float, timelimit: float,
                      target_bay: list[int] | None = None) -> dict:
    """
    남는 시간 동안 "objective 기여도가 큰 블록"을 하나씩 destroy하고
    _place_blocks(w1/w2/w3 전체를 반영하는 기존 스코어)로 재배치한다.

    기여도 = w1*지각 + w3*선호페널티 + (가중부하 최대 베이에 있으면
    w2*u_bay*workload 보너스).  기여도 내림차순으로 시도하고, 이동마다
    전체 check_feasibility로 재검증해 objective가 엄격히 줄었을 때만
    채택한다.  한 바퀴에서 하나라도 채택되면 기여도를 다시 계산해 다음
    바퀴를 돈다 (시간이 남는 한).

    target_bay : _solve_target_assignment가 계산한 "블록 -> 목표 베이"
        힌트(myalgorithm8 신규, [E]/[F] 참조).  주어지면 Phase B(스왑)의
        이주 목표를 "최선호 베이 1개 고정" 대신 이 배정으로 사용한다.
        None이면(target 계산 스킵 등) myalgorithm7과 동일하게 최선호
        베이를 그대로 쓴다.
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

    # -- Phase B 예산 예약 (myalgorithm8 신규, [G] 참조) ----------------------
    # obj1==0(여유형, Phase B 타겟배정)일 때만 정적 예약을 쓴다.  obj1>0
    # 구간은 정적 예약 대신 아래 Phase A 정체 감지(PHASE_A_STALL_S)가
    # 적응형으로 시간을 양보한다 -- 격리 실험에서 정적 예약은 Phase A가
    # 생산적인 인스턴스(prob_25/23)에 순손실이었다.
    o1 = res.get("obj1", 0.0)
    phase_a_deadline = deadline
    if PHASE_B_RESERVE_ENABLED and o1 == 0.0:
        slack = max(0.0, deadline - time.time())
        reserved = min(PHASE_B_RESERVE_MAX_S, slack * PHASE_B_RESERVE_FRAC)
        phase_a_deadline = deadline - reserved

    # 정체 감지는 "양보 받을 페이즈가 실제로 있을 때"만 켠다 -- Phase C
    # 대상(지각 블록 수 <= TARDY_SWAP_MAX_TARDY) 또는 obj1==0(Phase B
    # 타겟배정).  그 외(초포화형)는 Phase A가 myalgorithm7과 동일하게 돈다.
    n_tardy0 = sum(1 for bid, a in assignments.items()
                   if a["exit_time"] > blocks_data[bid]["due_date"])
    stall_enabled = (o1 == 0.0 or
                     (TARDY_SWAP_ENABLED and
                      0 < n_tardy0 <= TARDY_SWAP_MAX_TARDY))

    improved = True
    last_accept = time.time()   # 정체 타이머 기준점
    stalled = False
    while improved and not stalled and \
            time.time() + max_cost * 1.2 < phase_a_deadline:
        improved = False
        cand = sorted((bid for bid in assignments if _contribution(bid) > 0),
                      key=_contribution, reverse=True)
        for bi in cand:
            now = time.time()
            if now + max_cost * 1.2 > phase_a_deadline:
                break
            if stall_enabled and now - last_accept > PHASE_A_STALL_S:
                stalled = True
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
                last_accept = time.time()
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

    def _scan_spot(A: int, t_bay: int, exclude: set[int],
                   entry_hi: int | None = None):
        """[rel, hi]의 모든 정수 entry에서 bbox-서로소 자리 탐색 (오름차순 =
        가장 이른 슬롯 우선).  hi는 기본 due-proc(정시 한정, Phase B용)이고,
        entry_hi를 주면 그 값으로 대체된다 (Phase C v2의 지각 감소 스캔:
        정시가 불가능해도 현재 entry보다 이르면 w1*유닛 이득)."""
        relA  = blocks_data[A]["release_time"]
        procA = blocks_data[A]["processing_time"]
        hiA   = (blocks_data[A]["due_date"] - procA) if entry_hi is None else entry_hi
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

    # ---- Phase C: 지각 감소 스왑 (myalgorithm8 신규 -- Z1 직접 겨냥) --------
    # v2 일반화 (모듈 상단 TARDY_SWAP_ENABLED 주석 참조): 지각 블록 A를
    # "현재 entry보다 이른" 슬롯으로 옮긴다.  빈자리(0퇴출)가 있으면 직접
    # 이동, 없으면 자리를 막는 비지각 파트너 B를 하나 빼고(1퇴출) A를 넣은
    # 뒤 B를 재배치한다.  entry가 1유닛 이르면 w1(수천~수만)씩 벌므로,
    # B 재배치로 Z2/Z3가 소폭 나빠져도 순 objective는 보통 크게 준다.
    #
    # 비용 구조가 설계를 결정했다: 트라이얼(전체 check_feasibility)이 개당
    # 0.5~2s로 비싸므로, (a) 어느 (entry, 위치)에 몇 개의 블록이 겹치는지를
    # 순수 정수 프로브로 먼저 훑어 "빈자리/1퇴출 자리 + 그 파트너"를 직접
    # 찾아내고(파트너별 전체 스캔의 조합폭발 회피), (b) 블록별 추정 이득
    # (당길 유닛 수) 내림차순으로 트라이얼 순서를 정하고, (c) 블록당 트라이얼
    # 수를 캡한다.  채택 기준은 Phase A/B와 동일: 전체 재검증 + 엄격 감소,
    # 실패 시 정확 롤백 -- 절대 나빠질 수 없다.
    n_tardy_now = sum(1 for bid2, a2 in assignments.items()
                      if a2["exit_time"] > blocks_data[bid2]["due_date"])
    if TARDY_SWAP_ENABLED and 0 < n_tardy_now <= TARDY_SWAP_MAX_TARDY:

        def _probe_entry(A: int, entry: int, banned: set[int]):
            """entry 시각에 A가 들어갈 자리를 순수 정수 스캔으로 찾는다.
            반환: (spot(entry,x,y,oi), t_bay, evict_id|None) -- 빈자리 우선,
            없으면 '정확히 1개 블록과만 겹치는' 위치 중 파트너 slack이 가장
            큰 것.  banned에 든 블록은 퇴출 파트너로 선택하지 않는다(트라이얼
            실패 후 재시도용 -- 공간 점유 판정에는 그대로 반영됨).
            둘 다 없으면 None."""
            blkA = blocks_data[A]
            exit_c = entry + blkA["processing_time"]
            best1 = None   # (slack_B, spot, t_bay, B)
            for t_bay2 in range(n_bays):
                W2, H2 = bays[t_bay2].width, bays[t_bay2].height
                act = []
                for b2, a2 in assignments.items():
                    if b2 == A or a2["bay_id"] != t_bay2:
                        continue
                    if a2["entry_time"] < exit_c and a2["exit_time"] > entry:
                        lb2 = _block_bbox(blocks_data[b2], a2["orient_idx"])
                        act.append(((a2["x"] + lb2[0], a2["y"] + lb2[1],
                                     a2["x"] + lb2[2], a2["y"] + lb2[3]), b2))
                for oi in range(len(blkA["shape"])):
                    lb = _block_bbox(blkA, oi)
                    lx0, ly0, lx1, ly1 = lb
                    px_lo, px_hi = math.ceil(-lx0), math.floor(W2 - lx1)
                    py_lo, py_hi = math.ceil(-ly0), math.floor(H2 - ly1)
                    if px_lo > px_hi or py_lo > py_hi:
                        continue
                    px0, py0 = max(0, px_lo), max(0, py_lo)
                    xs = sorted({px0} | {math.ceil(ab[2] - lx0) for ab, _ in act})
                    xs = [x for x in xs if px_lo <= x <= px_hi][:FORCE_POS_XS]
                    ys = sorted({py0} | {math.ceil(ab[3] - ly0) for ab, _ in act})
                    ys = [y for y in ys if py_lo <= y <= py_hi][:FORCE_POS_YS]
                    for cy in ys:
                        for cx in xs:
                            wbb = (cx + lx0, cy + ly0, cx + lx1, cy + ly1)
                            conf = None
                            n_conf = 0
                            for ab, b2 in act:
                                if _bb_overlap(wbb, ab):
                                    n_conf += 1
                                    conf = b2
                                    if n_conf > 1:
                                        break
                            if n_conf == 0:
                                return ((entry, cx, cy, oi), t_bay2, None)
                            if n_conf == 1 and conf not in banned:
                                slack_B = (blocks_data[conf]["due_date"]
                                           - assignments[conf]["exit_time"])
                                if best1 is None or slack_B > best1[0]:
                                    best1 = (slack_B, (entry, cx, cy, oi),
                                             t_bay2, conf)
            if best1 is not None:
                return (best1[1], best1[2], best1[3])
            return None

        def _entry_ladder(rel2: int, cur2: int):
            """rel(이득 최대)부터 cur 직전까지 성긴 사다리 -- 프로브 지점."""
            cands = {rel2, rel2 + 1, rel2 + 2, rel2 + 4, rel2 + 8,
                     (rel2 + cur2) // 2, (rel2 + 3 * cur2) // 4}
            return sorted(e for e in cands if rel2 <= e < cur2)

        def _best_option(A: int, banned: set[int]):
            """A의 최선 이동 옵션: 사다리에서 가장 이른(=이득 최대) 프로브
            성공 지점.  반환 (gain_units, spot, t_bay, evict|None) 또는 None."""
            a2 = assignments[A]
            rel2 = int(blocks_data[A]["release_time"])
            cur2 = int(a2["entry_time"])
            delay2 = a2["exit_time"] - blocks_data[A]["due_date"]
            if delay2 <= 0 or cur2 - 1 < rel2:
                return None
            for entry in _entry_ladder(rel2, cur2):
                r = _probe_entry(A, entry, banned)
                if r is not None:
                    spot, t_bay2, evict = r
                    return (min(cur2 - entry, delay2), spot, t_bay2, evict)
            return None

        # -- 스크리닝: 추정 이득 내림차순 트라이얼 순서 ----------------------
        # 지각 블록이 많으면(_best_option 프로브 자체가 블록당 사다리 x
        # 베이 x 방향 전수 스캔이라 값싸지 않음) 전수 프로빙이 정체감지로
        # 확보한 창을 통째로 먹어 트라이얼이 0개가 되는 것이 격리 실험에서
        # 실측됐다 (prob_30/36, 지각 70~111개).  그래서 프로빙 자체는
        # delay가 큰 상위 TARDY_SWAP_SCREEN_MAX개에만 수행한다 -- 진단상
        # 회수 가능 이득은 delay 상위권에 몰려 있어(top5가 top40 잠재치의
        # 대부분) 손실이 적다.
        tardy_ids = sorted(
            (bid2 for bid2, a2 in assignments.items()
             if a2["exit_time"] > blocks_data[bid2]["due_date"]),
            key=lambda b: assignments[b]["exit_time"] - blocks_data[b]["due_date"],
            reverse=True,
        )[:TARDY_SWAP_SCREEN_MAX]
        gains: dict[int, float] = {}
        for bid2 in tardy_ids:
            if time.time() > deadline:
                break   # 스크리닝(순수 파이썬 프로브)은 트라이얼보다 훨씬
                        # 싸므로 여유 버퍼 없이 데드라인만 본다
            r = _best_option(bid2, set())
            gains[bid2] = r[0] if r is not None else 0.0
        order_c = [b for b in sorted(gains, key=lambda b: gains[b],
                                     reverse=True) if gains[b] > 0]

        # Phase C 트라이얼 비용은 Phase A 트라이얼(전탐색 _place_blocks)보다
        # 싸므로 max_cost*1.5 대신 max_cost를 초기 추정으로 쓴다 (자체 실측
        # 으로 갱신됨).  중간포화에서 트라이얼 1~2개가 더 들어가는 차이.
        swap_est_c = max(2.0, max_cost)
        for A in order_c:
            if time.time() + swap_est_c > deadline:
                break
            banned: set[int] = set()
            for _trial in range(TARDY_SWAP_TRIALS_PER_BLOCK):
                if time.time() + swap_est_c > deadline:
                    break
                # 이전 수락/실패로 상태가 바뀌었으므로 트라이얼 직전에 새로 스캔
                r = _best_option(A, banned)
                if r is None:
                    break
                _, spot, t_bay, evict = r
                t_sw = time.time()
                n_tried += 1
                accepted = False
                if evict is None:
                    # 직접 이동 (빈자리 -- bbox-서로소 = 구조적 crane-feasible)
                    save_A = assignments[A]
                    blk_A, slot_A = _pop_state(A, save_A)
                    _apply_spot(A, t_bay, spot)
                    trial_sol = {"operations": _build_operations(
                        list(assignments.values()), prob_info)}
                    trial_res = check_feasibility(prob_info, trial_sol)
                    if trial_res["feasible"] and trial_res["objective"] < obj_best:
                        obj_best = trial_res["objective"]
                        n_moved += 1
                        accepted = True
                    else:
                        _pop_state(A, assignments[A])
                        bay_placed[save_A["bay_id"]].append(blk_A)
                        bay_schedule[save_A["bay_id"]].append(slot_A)
                        bay_loads[save_A["bay_id"]] += blocks_data[A]["workload"]
                        assignments[A] = save_A
                    swap_est_c = max(swap_est_c, time.time() - t_sw)
                    if not accepted:
                        break   # 빈자리 트라이얼 실패는 재시도 무의미 (같은 자리)
                else:
                    # 1퇴출 스왑: evict를 빼고 A를 넣은 뒤 evict 재배치
                    B = evict
                    save_A, save_B = assignments[A], assignments[B]
                    blk_A, slot_A = _pop_state(A, save_A)
                    blk_B, slot_B = _pop_state(B, save_B)
                    _apply_spot(A, t_bay, spot)
                    part_B = _place_blocks(
                        [B], blocks_data, bays, bay_placed, bay_schedule,
                        bay_loads, w1, w2, w3, forced_ids=set(),
                        prev_assignments=assignments, hard_deadline=deadline)
                    assignments[B] = part_B[B]
                    trial_sol = {"operations": _build_operations(
                        list(assignments.values()), prob_info)}
                    trial_res = check_feasibility(prob_info, trial_sol)
                    if trial_res["feasible"] and trial_res["objective"] < obj_best:
                        obj_best = trial_res["objective"]
                        n_moved += 1
                        accepted = True
                    else:
                        # 정확 롤백 (A, B 둘 다 원위치로)
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
                        banned.add(B)   # 이 파트너로는 재시도하지 않음
                    swap_est_c = max(swap_est_c, time.time() - t_sw)
                if accepted:
                    break   # 블록당 1개 수락이면 충분 -- 다음 블록으로

    swap_tried: set[int] = set()
    swap_est = max(2.0, max_cost * 1.5)
    while time.time() + swap_est < deadline:
        A = None
        # myalgorithm8: target_bay가 있을 때만(obj1==0 확인된 인스턴스)
        # 후보 선정을 pen2(선호페널티만)에서 Phase A와 동일한 _contribution
        # (부하 최대 베이 보너스 포함)으로 바꿔 부하균형 목적 이주도 후보에
        # 잡히게 한다.  target_bay가 None이면(포화형 등) pen2 기준의
        # myalgorithm7 원본 로직과 완전히 동일하게 동작한다.
        if target_bay is not None:
            best_c = 0.0
            for bid2, a2 in assignments.items():
                if bid2 in swap_tried:
                    continue
                if a2["exit_time"] > blocks_data[bid2]["due_date"]:
                    continue  # 지각 블록은 w1 영역 -- 이 페이즈는 Z2/Z3 전용
                c2 = _contribution(bid2)
                if c2 > best_c:
                    best_c, A = c2, bid2
        else:
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

        a_A = assignments[A]
        if target_bay is not None:
            t_bay = target_bay[A]
        else:
            prefs_A = blocks_data[A]["bay_preferences"]
            t_bay = max(range(n_bays), key=lambda j: prefs_A[j])
        if t_bay == a_A["bay_id"]:
            continue  # 이미 목표 베이 -- 이 메커니즘으로는 더 할 게 없음

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

# Phase 1 시간가드 비율: _place_blocks가 예산의 이 비율을 쓰면 남은 블록을
# 전부 _force_place로 넘긴다.  0.80 = 기존 동작(변경 없음).  포화형 인스턴스
# 실험용 노브: 이 값을 낮추면 탐색을 일찍 끊고 남은 시간을 폴리시/Phase C에
# 넘길 수 있다 -- 계측상 force-place가 포화 꼬리의 지각을 균형형 greedy 탐색
# 보다 잘 줄이므로(project_phase1_forceplace_beats_search 참조), w1 지배
# 인스턴스에서는 탐색시간을 지각 감소 폴리시로 재분배하는 것이 유리할 수 있다.
PHASE1_GUARD_FRAC: float = 0.80

# ==========================================================================
# Phase 1 적응형 시간가드 -- 정적 컷오프 스윕(prob37/33) 실측 결과, 최적
# 컷오프가 인스턴스 구조(베이 수/밀도)마다 달라서(prob33은 0.55가 이득,
# prob37은 0.80이 그대로 최선) 정적 상수 하나로는 해결이 안 됨.  그래서
# "몇 % 지점에서 끊을지"를 손튜닝하는 대신, 최근 블록들의 **실측 탐색
# 비용**으로 "이 속도로는 남은 블록을 다 못 끝낸다"를 실시간으로 투영해
# 판단한다 -- 밀집도가 다른 인스턴스에 자동으로 맞춰진다(조밀할수록 비용
# 곡선이 일찍 가팔라지므로 더 일찍 트리거됨).
#
# 안전장치: 이 가드는 PHASE1_GUARD_FRAC 고정 가드보다 "더 일찍" 끊을 수만
# 있다 -- 고정 가드는 그대로 최후 안전망(절대 이 시각 이후로는 안 감)으로
# 남아 있으므로, 이 기능을 꺼도(PHASE1_ADAPTIVE_GUARD_ENABLED=False) 기존
# 동작과 완전히 동일하고, 켜져 있어도 최악의 경우 기존과 동일한 시점에
# 끊긴다(투영이 절대 안 맞아도 고정 가드가 뒤를 받침).
PHASE1_ADAPTIVE_GUARD_ENABLED: bool = True

# 이 개수만큼 실제 탐색(강제배치 아님) 비용 표본이 쌓이기 전에는 투영하지
# 않는다 -- 초반 몇 개의 싼 표본만으로 성급하게 판단하는 것을 방지.
PHASE1_ADAPTIVE_WARMUP: int = 15

# 투영에 쓰는 이동평균 창(최근 이 개수 블록의 평균 비용) -- 비용이 뒤로
# 갈수록 커지는 추세(초선형 폭발, 실측)를 cumulative 평균보다 더 빠르게
# 반영하도록 trailing window를 쓴다.
PHASE1_ADAPTIVE_WINDOW: int = 15

# 투영된 완료 시각이 "timelimit - 예약분"을 넘으면 즉시 중단 -- repair 1패스
# + 최소한의 폴리시(AABB 압축 등)를 위한 예약분.  60s 예산 기준 절대값.
PHASE1_ADAPTIVE_RESERVE_S: float = 8.0

# 예약분의 상한을 "이 값 x timelimit"으로도 캡한다 -- 포트폴리오 2차 변형이나
# 폴리시 내부 재호출처럼 짧은 예산 슬라이스를 받을 때 절대값 예약이 예산의
# 큰 비율을 먹어버리는 것을 막는다 (실측: prob_2 variant2 budget=21.9s에서
# 8초 고정 예약 = 36% -> 조기 트리거).  60s 예산에서는 8s/60s≈13%라 이 값
# (15%)이 사실상 절대값과 같게 작동한다.
PHASE1_ADAPTIVE_RESERVE_MAX_FRAC: float = 0.15


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

                if FORCE_BESTFIT_ENABLED and _MIN_BLOCK_DIM > 0:
                    # -- (C) free-space 인지 선택 (모듈 상단 주석 참조) -------
                    # bottom-left 스캔 순서 그대로 최대 FORCE_BESTFIT_MAX_CAND개
                    # disjoint 후보를 모아, dead-sliver(0 < gap < 전형적
                    # 블록크기)를 가장 적게 만드는 것을 채택.  동률이면 스캔
                    # 순서상 먼저 나온 것(=기존 bottom-left 동작과 동일).
                    disjoint: list[tuple[int, int]] = []
                    for cy in ys:
                        for cx in xs:
                            wbb = (cx + lx0, cy + ly0, cx + lx1, cy + ly1)
                            if all(not _bb_overlap(wbb, ab) for ab in active_bbs):
                                disjoint.append((cx, cy))
                                if len(disjoint) >= FORCE_BESTFIT_MAX_CAND:
                                    break
                        if len(disjoint) >= FORCE_BESTFIT_MAX_CAND:
                            break
                    if disjoint:
                        best_dead = None
                        best_cand = None
                        for cx, cy in disjoint:
                            wbb = (cx + lx0, cy + ly0, cx + lx1, cy + ly1)
                            next_x = min(
                                (ab[0] for ab in active_bbs
                                 if ab[1] < wbb[3] and ab[3] > wbb[1]
                                 and ab[0] > wbb[2]),
                                default=bay.width)
                            next_y = min(
                                (ab[1] for ab in active_bbs
                                 if ab[0] < wbb[2] and ab[2] > wbb[0]
                                 and ab[1] > wbb[3]),
                                default=bay.height)
                            gap_x = next_x - wbb[2]
                            gap_y = next_y - wbb[3]
                            dead = int(0 < gap_x < _MIN_BLOCK_DIM) \
                                 + int(0 < gap_y < _MIN_BLOCK_DIM)
                            if best_dead is None or dead < best_dead:
                                best_dead, best_cand = dead, (cx, cy)
                        found = (entry_c, best_cand[0], best_cand[1])
                else:
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
    search_costs: list[float] = []  # per-block wall time, search path only

    # Bay weights for normalized obj2: u_j = avg_area / (W_j * H_j)
    _bay_areas   = [bay.width * bay.height for bay in bays]
    _avg_area    = sum(_bay_areas) / n_bays
    bay_weights  = [_avg_area / a for a in _bay_areas]

    for rank, bi in enumerate(block_ids):
        # -- Time budget guard (only active when timelimit is given) --------
        guard_reason = None
        if timelimit is not None and t_start is not None:
            now = time.time()
            elapsed = now - t_start
            if elapsed > timelimit * PHASE1_GUARD_FRAC:
                guard_reason = (f"{elapsed:.1f}s > {timelimit * PHASE1_GUARD_FRAC:.1f}s "
                                f"({PHASE1_GUARD_FRAC*100:.0f}% of {timelimit:.1f}s, fixed)")
            elif (PHASE1_ADAPTIVE_GUARD_ENABLED
                  and len(search_costs) >= PHASE1_ADAPTIVE_WARMUP):
                window = search_costs[-PHASE1_ADAPTIVE_WINDOW:]
                avg_cost = sum(window) / len(window)
                n_remaining_now = n_total - rank
                projected = elapsed + avg_cost * n_remaining_now
                # 예약은 절대값이 아니라 예산 비례 상한으로 캡한다 -- 그렇지
                # 않으면 짧은 예산 슬라이스(포트폴리오 2차 변형, 폴리시 내부
                # 호출 등)에서 예약분이 예산의 큰 비율을 먹어 조기 트리거된다
                # (실측: prob_2 variant2, budget=21.9s에서 8s 고정 예약이
                # 36%를 차지해 불필요하게 일찍 끊김).
                reserve = min(PHASE1_ADAPTIVE_RESERVE_S,
                             timelimit * PHASE1_ADAPTIVE_RESERVE_MAX_FRAC)
                budget_ceiling = timelimit - reserve
                if projected > budget_ceiling:
                    guard_reason = (f"projected finish {projected:.1f}s > "
                                    f"{budget_ceiling:.1f}s budget ceiling "
                                    f"(avg_cost={avg_cost:.3f}s/blk x {n_remaining_now} remaining, "
                                    f"adaptive)")
        if guard_reason is not None:
            remaining = block_ids[rank:]
            print(f"[Greedy] TIME GUARD: {guard_reason} -- "
                  f"force-placing {len(remaining)} remaining block(s)")

            # -- 꼬리 순서 부분열거 (모듈 상단 TAIL_PORTFOLIO 주석 참조) ----
            # 시뮬레이션은 bay_placed/bay_schedule의 얕은 복사 위에서 돈다
            # (이 루프는 append만 하고 기존 원소를 변형하지 않으므로 안전).
            # start_state=(placed, schedule)를 주면 그 상태에서 시작
            # (국소탐색의 접미사 재시뮬용), collect_snaps=True면 각 위치
            # "배치 직전" 상태의 얕은 복사 목록도 함께 돌려준다.
            def _sim_tail(order: list[int],
                          start_state: tuple | None = None,
                          collect_snaps: bool = False
                          ) -> tuple[float, list[tuple], list | None]:
                src_placed   = bay_placed   if start_state is None else start_state[0]
                src_schedule = bay_schedule if start_state is None else start_state[1]
                sim_placed   = [list(pb) for pb in src_placed]
                sim_schedule = [list(sc) for sc in src_schedule]
                snaps: list | None = [] if collect_snaps else None
                placements: list[tuple] = []
                tail_tardy = 0.0
                for bid in order:
                    if collect_snaps:
                        snaps.append(([list(pb) for pb in sim_placed],
                                      [list(sc) for sc in sim_schedule]))
                    bd = blocks_data[bid]
                    bay_id, cx, cy, oi, entry, exit_t = _force_place(
                        bid, blocks_data, bays, sim_placed, sim_schedule,
                        bd["bay_preferences"]
                    )
                    sim_placed[bay_id].append(
                        Block(block_id=bid, block_data=bd,
                              x=cx, y=cy, orient_idx=oi))
                    sim_schedule[bay_id].append((entry, exit_t))
                    placements.append((bid, bay_id, cx, cy, oi, entry, exit_t))
                    tail_tardy += max(0.0, exit_t - bd["due_date"])
                return tail_tardy, placements, snaps

            _t_sim = time.time()
            best_tardy, best_placements, _ = _sim_tail(remaining)  # v0 = 현행 EDD
            best_name = "edd"
            sim_cost = time.time() - _t_sim

            if (_TAIL_PORTFOLIO_ACTIVE and TAIL_PORTFOLIO_ENABLED
                    and timelimit is not None and t_start is not None
                    and len(remaining) > 1):
                def _fp_area(bid: int) -> float:
                    bb = _block_bbox(blocks_data[bid], 0)
                    return (bb[2] - bb[0]) * (bb[3] - bb[1])
                # 실측(prob_33/37): due-우선 계열만 이기고 EDD에서 크게
                # 벗어난 순서(area_desc/release 전역 정렬)는 전패 -> 후보를
                # EDD 미세변형(due 동률 tie-break 교체) 중심으로 재편.
                variants = [
                    ("edd_area", sorted(
                        remaining,
                        key=lambda b: (blocks_data[b]["due_date"], -_fp_area(b)))),
                    ("edd_proc", sorted(
                        remaining,
                        key=lambda b: (blocks_data[b]["due_date"],
                                       -blocks_data[b]["processing_time"]))),
                    ("edd_wl", sorted(
                        remaining,
                        key=lambda b: (blocks_data[b]["due_date"],
                                       -blocks_data[b]["workload"]))),
                    ("latest_start", sorted(
                        remaining,
                        key=lambda b: (blocks_data[b]["due_date"]
                                       - blocks_data[b]["processing_time"],
                                       blocks_data[b]["due_date"]))),
                ]
                deadline = t_start + timelimit * TAIL_PORTFOLIO_TIME_FRAC
                for vname, vorder in variants:
                    # 직전 시뮬 실측 비용으로 예산 초과를 사전 차단.
                    if time.time() + sim_cost * 1.2 > deadline:
                        print(f"[TailPortfolio] budget: '{vname}' 이후 생략 "
                              f"(sim_cost {sim_cost:.1f}s)")
                        break
                    _t_v = time.time()
                    v_tardy, v_placements, _ = _sim_tail(vorder)
                    sim_cost = max(sim_cost, time.time() - _t_v)
                    print(f"[TailPortfolio] {vname}: tail tardiness "
                          f"{v_tardy:.0f} (best {best_tardy:.0f})")
                    if v_tardy < best_tardy:  # 엄격 개선일 때만 교체
                        best_tardy, best_placements = v_tardy, v_placements
                        best_name = vname
                print(f"[TailPortfolio] selected '{best_name}' "
                      f"(tail tardiness {best_tardy:.0f}, "
                      f"{len(remaining)} blocks)")

            # -- (A) 순서 국소탐색 (모듈 상단 TAIL_LS 주석 참조) ------------
            if (_TAIL_PORTFOLIO_ACTIVE and TAIL_PORTFOLIO_ENABLED
                    and TAIL_LS_ENABLED
                    and timelimit is not None and t_start is not None
                    and len(remaining) > 2):
                _now = time.time()
                ls_deadline = min(
                    _now + TAIL_LS_REMAIN_FRAC
                        * max(0.0, (t_start + timelimit) - _now),
                    t_start + timelimit * TAIL_PORTFOLIO_TIME_FRAC)
                L = len(remaining)
                per_blk_cost = sim_cost / max(1, L)
                # 스냅샷 포함 기준 재시뮬 1회 (이후 모든 이동 평가의 기반)
                if time.time() + sim_cost * 1.15 < ls_deadline:
                    order = [p[0] for p in best_placements]
                    cur_tardy, cur_pl, snaps = _sim_tail(order,
                                                         collect_snaps=True)
                    ls_t0_tardy = cur_tardy
                    n_acc = n_try = 0
                    stall = False
                    while not stall and time.time() < ls_deadline:
                        stall = True
                        # 지각 기여 내림차순 상위 블록부터 당기기 시도
                        tardies = sorted(
                            ((max(0.0, pl[6]
                                  - blocks_data[pl[0]]["due_date"]), idx)
                             for idx, pl in enumerate(cur_pl)),
                            reverse=True)
                        for tval, i in tardies[:TAIL_LS_TOP_BLOCKS]:
                            if tval <= 0 or time.time() >= ls_deadline:
                                break
                            moved = False
                            due_i = blocks_data[order[i]]["due_date"]
                            # 이동 후보 (start_idx, new_order) 목록 구성
                            moves: list[tuple[int, list[int]]] = []
                            if "pull" in TAIL_LS_MOVES:
                                for j in {i - 1, i - max(1, L // 8),
                                          i // 2, 0}:
                                    if 0 <= j < i:
                                        moves.append((j, order[:j]
                                                      + [order[i]]
                                                      + order[j:i]
                                                      + order[i + 1:]))
                            if "cluster" in TAIL_LS_MOVES:
                                k = next((k for k in range(i)
                                          if blocks_data[order[k]]
                                          ["due_date"] >= due_i), None)
                                if k is not None:
                                    moves.append((k, order[:k]
                                                  + [order[i]]
                                                  + order[k:i]
                                                  + order[i + 1:]))
                            if "swap" in TAIL_LS_MOVES:
                                n_sw = 0
                                for k in range(i - 1, -1, -1):
                                    pl_k = cur_pl[k]
                                    if (pl_k[6] - blocks_data[pl_k[0]]
                                            ["due_date"]) <= 0:  # 비지각
                                        no = list(order)
                                        no[i], no[k] = no[k], no[i]
                                        moves.append((k, no))
                                        n_sw += 1
                                        if n_sw >= 3:
                                            break
                            # 싼 접미사(늦은 start_idx)부터 평가
                            moves.sort(key=lambda mv: -mv[0])
                            for j, new_order in moves:
                                sfx_len = L - j
                                if (time.time()
                                        + per_blk_cost * sfx_len * 1.3
                                        > ls_deadline):
                                    continue  # 이 접미사는 예산 초과 위험
                                sfx_tardy, sfx_pl, sfx_snaps = _sim_tail(
                                    new_order[j:], start_state=snaps[j],
                                    collect_snaps=True)
                                n_try += 1
                                pre_tardy = sum(
                                    max(0.0, pl[6]
                                        - blocks_data[pl[0]]["due_date"])
                                    for pl in cur_pl[:j])
                                new_total = pre_tardy + sfx_tardy
                                if new_total < cur_tardy - 1e-9:  # 엄격 개선
                                    order = new_order
                                    cur_pl = cur_pl[:j] + sfx_pl
                                    snaps = snaps[:j] + sfx_snaps
                                    cur_tardy = new_total
                                    n_acc += 1
                                    stall = False
                                    moved = True
                                    break
                            if moved:
                                break  # 갱신된 지각 순위로 다시 패스
                    if cur_tardy < best_tardy - 1e-9:
                        best_tardy, best_placements = cur_tardy, cur_pl
                    print(f"[TailLS] tail tardiness {ls_t0_tardy:.0f} -> "
                          f"{cur_tardy:.0f} ({n_acc} accepts / {n_try} trials)")
                else:
                    print(f"[TailLS] skipped (sim_cost {sim_cost:.1f}s, "
                          f"no budget)")

            for (bid, bay_id, cx, cy, oi, entry, exit_t) in best_placements:
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

        _t_blk_start = time.time()
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
        else:
            search_costs.append(time.time() - _t_blk_start)

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

    parser = argparse.ArgumentParser(description="myalgorithm8 (target-assignment polish) smoke test")
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
    parser.add_argument("--no-target-assign", action="store_true",
                        help="disable TARGET_ASSIGN_ENABLED (Phase B가 target_bay 힌트 "
                             "없이 myalgorithm7의 argmax-pref/pen2 로직으로 동작 -- "
                             "PHASE_B_RESERVE_ENABLED는 별도 플래그이니 완전한 "
                             "myalgorithm7 재현에는 --no-phase-b-reserve도 같이 필요)")
    parser.add_argument("--no-phase-b-reserve", action="store_true",
                        help="disable PHASE_B_RESERVE_ENABLED (Phase A가 myalgorithm7과 "
                             "동일하게 전체 폴리시 예산을 씀 -- A/B 테스트용)")
    parser.add_argument("--no-tardy-swap", action="store_true",
                        help="disable TARDY_SWAP_ENABLED (Phase C 지각 스왑 + "
                             "near-feasible 타겟배정 Z3 개선 OFF -- A/B 테스트용)")
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
    if args.no_target_assign:
        TARGET_ASSIGN_ENABLED = False
    if args.no_phase_b_reserve:
        PHASE_B_RESERVE_ENABLED = False
    if args.no_tardy_swap:
        # Phase C(지각 감소 스왑)와 그에 딸린 예약을 끈다.  obj1>0 인스턴스는
        # myalgorithm7과 동일하게 동작 (obj1==0 여유형의 타겟배정/예약은 유지).
        TARDY_SWAP_ENABLED = False

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
