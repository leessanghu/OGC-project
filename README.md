# 두 번째 알고리즘 (`submit/myalgorithm.py` ← `baseline/myalgorithm4.py`)

`baseline_greedy.py`(EDD + Best-Fit Greedy + 사후 repair)를 베이스로,
지배적 목적항인 **Z1(tardiness)** 을 줄이는 데 집중해서 개선한 버전이다.
입력/출력 계약은 동일하다.

```python
def algorithm(prob_info: dict, timelimit: float = 60, repair_mode: str = "greedy") -> dict:
    ...
    return {"operations": {...}}
```

## 왜 이 방향인가

`train/` 40개 인스턴스를 전수 확인한 결과, w1(tardiness 가중치)이 w2/w3보다
**최소 30배, 대부분 100배 이상** 크다. 즉 obj2(작업부하 불균형)·obj3(선호도
손실)을 다소 희생하더라도 obj1을 줄이는 쪽이 총 objective에 압도적으로
유리하다. 이 알고리즘의 모든 개선은 이 전제 위에서 설계됐다.

## 베이스 대비 변경 사항 (영향이 큰 순서대로)

### 1. 같은 시각 ENTRY/EXIT 위상정렬 (가장 큰 개선)
`_build_operations`가 같은 시각에 여러 블록이 동시에 진입/퇴출할 때, 기존
baseline은 `block_id` 순서로만 정렬했다. 이 경우 실제로는 문제없는 배치인데도
**정렬 순서 때문에** Stage-5 위반이 발생할 수 있다 — 위치·시간이 아니라
순서 버그였다. Kahn 위상정렬(`_topo_sort_bay_entries`/`_topo_sort_bay_exits`,
"누가 누구의 크레인 경로를 막는지" 그래프 기반)로 교체해, 이런 "가짜 위반"을
원천 차단했다. 단독으로 prob_1 기준 obj1 **294 → 2** (99% 감소)를 만들어낸
가장 큰 단일 개선.

### 2. Due-date shielding
새 블록을 놓을 때, **자신보다 due_date가 이르거나 같은 활성 블록의 미래
크레인 출차 경로를 막는 위치는 후보에서 배제**한다 (`_shielding_violated`,
`check_exit`를 페어로 재사용). "이미 배치된 블록이 나중에 막혀서 강제로
지연되는" 연쇄 실패를 배치 시점에 예방한다. 항상 무제약(unshielded) 최선
후보를 안전망으로 함께 추적해, shielding 때문에 후보가 하나도 안 남는 경우
자동으로 폴백한다 — baseline보다 나빠질 수 없는 구조.

### 3. Destroy/repair (반복 위반 블록 처리)
repair 단계에서 같은 블록이 두 번째로 위반에 걸리면(cycle 의심), baseline은
무조건 "베이 전체 비우기"로 강제배치한다. 이 알고리즘은 먼저 정상 탐색을
한 번 더 시도하고, 그래도 shielding을 위반하면 그 위반을 유발하는 블록 딱
하나만 같이 빼내(destroy) 둘을 함께 재배치한다(repair). 실패해도 원래의
안전한 강제배치로 자동 폴백한다.

### 4. 인스턴스 밀도 기반 K-best 디스패처
`_find_earliest_slot`(크레인 진입/퇴출 판정, Shapely 기반)이 병목이라는 걸
프로파일링으로 확인했다 (인스턴스 하나 처리 시간의 ~94%). 후보 위치를
전부 평가하지 않고 **K개 찾으면 멈추는** K-best로 호출 수를 줄인다. K 값은
인스턴스마다 고정하지 않고 **블록/베이 비율에 반비례하는 연속 함수**로
자동 결정한다:

```
K_BEST = round(1000 / blocks_per_bay)          (blocks_per_bay >= 40일 때)
K_BEST = 0 (무제한)                             (blocks_per_bay < 40일 때)
```

밀집한 인스턴스일수록 더 타이트하게, 여유 있는 인스턴스는 무제한 탐색을
유지한다. (이분법 dense/normal 임계값 방식은 실측 결과 인스턴스 크기 차이를
못 잡아내 폐기 — 자세한 배경은 커밋 히스토리 참고.)

### 5. repair 전용 K-best 상한
repair는 Phase 1 직후, 즉 베이가 이미 거의 꽉 찬 상태에서 돈다. Phase 1
디스패처가 "여유롭다"고 판단한 인스턴스도 repair에서는 무제약 탐색이 실제로
60초 제한을 넘긴 사고가 있었다(위반 3개 처리에 28초). 그래서 repair는
인스턴스 분류와 무관하게 **항상** `K_BEST_REPAIR=20`을 적용한다.

### 6. Phase 1 시간 가드
baseline에는 Phase 1 자체에 시간 제한 로직이 없었다. 80% 시점을 넘기면
남은 블록을 즉시 안전한 강제배치로 처리해 60초 제한을 항상 지키도록 했다.

### 7. 중복 재계산 제거 (품질 손실 없는 순수 속도 개선)
K-best 후보 평가 루프가 `_find_earliest_slot`을 최대 K번 호출하는데, 그때마다
베이 전체 이력(`placed_in_bay`)을 처음부터 다시 필터링하고 있었다. 이미 나간
블록까지 포함해서. `(bay, orientation)`당 한 번만 필터링한 활성 블록
리스트(`active_in_bay`/`active_schedule`)를 만들어 넘기도록 바꿨다 —
결과는 수학적으로 100% 동일하고(내부 필터가 결국 같은 조건으로 수렴)
호출 비용만 줄어든다. K-best의 K값을 낮추지 않고도 속도를 벌 수 있어,
디스패처가 계산한 K를 그대로 쓰면서 품질 저하 없이 시간 여유를 확보한다.

## 조정 가능한 상수

| 상수 | 기본값 | 의미 |
|---|---|---|
| `SHIELD_ENABLED` | `True` | due-date shielding on/off |
| `AUTO_DISPATCH` | `True` | K-best 밀도 디스패처 on/off |
| `K_BEST_UNLIMITED_BELOW` | `40.0` | 이 blocks/bay 미만이면 무제한 탐색 |
| `K_BEST_SCALE_CONST` | `1000.0` | `K_BEST = round(이 값 / blocks_per_bay)` |
| `K_BEST_MIN` | `3` | K-best 하한 |
| `K_BEST_REPAIR` | `20` | repair 전용 고정 상한 |
| `MAX_ENTRY_TRIES` | `0`(무제한) | entry-time 후보 상한 — **실측상 역효과 확인, 비활성 유지 권장** |

CLI 오버라이드: `python myalgorithm4.py <instance> --k-best N` (AUTO_DISPATCH
비활성화하고 강제 적용), `--k-best-scale-const`, `--no-shield`,
`--max-entry-tries` 등.

## 검증 결과 요약 (train 인스턴스 기준, timelimit=60s)

| 인스턴스 | 특징 | baseline obj1 | 이 알고리즘 obj1 |
|---|---|---|---|
| prob_1 (100블록/2베이) | 기준 | 294.0 | 2~21 |
| prob_2 (100블록/3베이) | 기준 | - | **0.0** |
| prob_6 (150블록/3베이) | K-best 도입 계기 | - | 124 (K=20) |
| prob_23 (100블록/2베이) | repair 60초 초과 버그 발견·수정 | - | feasible, 60s 이내 |
| prob_38 (250블록/3베이) | 가장 어려운 케이스 | - | 74,269 (K=12, 이 세션 최선) |

**한계**: prob_37/38/39 같은 250블록/3베이급 고밀도 인스턴스는 이 구조로도
완전히 만족스럽게 풀리지 않는다 (여전히 일부 블록이 시간 가드에 의해
강제배치됨). `_find_earliest_slot`의 근본 비용(활성 블록 수에 비례해 커지는
Shapely 기반 크레인 판정)을 줄이는 추가 작업(예: 베이별 병렬 처리, 블록별
공정 시간 배당)이 다음 후보로 남아있다.

## 파일 위치

- 개발/실험: `baseline/myalgorithm4.py`
- 제출본: `submit/myalgorithm.py` (`myalgorithm4.py`와 동일 내용)
- GUI 테스트: `alg_tester/myalgorithm4_folder/myalgorithm.py` (+ `utils.py` 사본)
