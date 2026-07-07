# OGC 2026 3번째 제출 알고리즘

이 브랜치는 3번째 제출용 브랜치입니다.

제출 파일은 아래 파일입니다.

```text
submit/myalgorithm.py
```

현재 `submit/myalgorithm.py`는 `baseline/myalgorithm6.py`를 그대로 복사한 버전입니다.

## 알고리즘 요약

`myalgorithm6.py`는 안정적으로 동작하던 `myalgorithm4.py` 구조를 기반으로 합니다.

- EDD 기준 greedy 배치
- 인스턴스 밀도 기반 K-best 후보 탐색
- due-date shielding
- 같은 시각 ENTRY/EXIT 위상정렬
- greedy repair와 제한된 destroy/repair fallback

`myalgorithm5.py`에서 시도했던 SPT 정렬, polygon-tight 후보 생성, left-shift, tardiness-LNS 계열 실험은 최종 제출에서는 제외했습니다. 해당 실험들은 원인 분석에는 도움이 되었지만, `prob_38` 같은 고밀도 인스턴스에서 불안정하거나 실행 시간이 늘어나는 문제가 있었습니다.

3번째 제출의 핵심은 **AABB 기반으로 늦은 강제배치를 줄이는 것**입니다.

## `myalgorithm6.py` 주요 변경점

### 1. AABB-서로소 강제배치

기존 `_force_place()`는 베이가 완전히 비는 시간창을 기다린 뒤 블록을 넣었습니다. 이 방식은 안전하지만, 고밀도 인스턴스에서는 강제배치 블록이 뒤로 길게 밀려서 tardiness가 크게 증가합니다.

`myalgorithm6.py`는 먼저 새 블록의 AABB가 해당 시간창의 활성 블록들과 서로 겹치지 않는 위치를 찾습니다. 그런 위치가 있으면, `check_entry`, `check_exit`, `check_collisions` 내부의 AABB pre-filter 구조상 비싼 geometry 검사를 피하면서도 crane-feasible한 배치를 만들 수 있습니다.

즉, 베이가 완전히 빌 때까지 기다리지 않고도 안전하게 들어갈 수 있는 자리를 먼저 찾습니다.

### 2. K-best 직전 AABB 즉시진입 우선정렬

`_place_blocks()`에서 K-best 후보를 수집하기 직전에 후보 위치를 한 번 정렬합니다.

우선순위가 올라가는 후보는 다음 조건을 만족하는 위치입니다.

- `release_time`에 바로 진입 가능
- `[release_time, release_time + processing_time)` 구간과 겹치는 활성 블록들과 AABB가 서로소
- 기존 bottom-left 후보 순서는 그룹 안에서 그대로 유지

이 변경의 목적은 K-best가 제한된 개수의 후보만 평가할 때, 늦게 들어가는 후보보다 **즉시 들어갈 수 있는 후보**를 먼저 보게 만드는 것입니다. 특히 넓은 베이에서 왼쪽 후보만 먼저 보다가 오른쪽의 좋은 자리를 놓치는 문제를 줄입니다.

## 측정된 개선 결과

아래 결과는 로컬 feasibility check 기준이며, 둘 다 feasible을 유지했습니다.

| 인스턴스 | Feasibility | 블록 수 | 기존 tardiness | 개선 후 tardiness | 감소량 |
|---|---:|---:|---:|---:|---:|
| `prob_38` | FEASIBLE | 250/250 | 11,393 | 8,484 | -2,909 |
| `prob_25` | FEASIBLE | 100/100 | 2,428 | 1,488 | -940 |

참고 로그:

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

## 파일 구성

```text
submit/myalgorithm.py        # 실제 제출 진입점
baseline/myalgorithm6.py     # 동일 알고리즘의 개발/기록용 파일
baseline/myalgorithm4.py     # 이전 안정 버전
```

## 로컬 실행 방법

```powershell
conda activate ogc2026
cd alg_tester
python alg_tester_app.py
```

GUI에서 아래처럼 선택하면 됩니다.

```text
Instance: train/prob_*.json
Algorithm folder: submit
```

