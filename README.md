# 네 번째 알고리즘 (`submit/myalgorithm.py` ← `baseline/myalgorithm7.py`)

EDD greedy + repair 위에 due-date shielding·밀도기반 K-best·크레인 경로
위상정렬을 쌓은 이전 버전들에서 한 단계 더 나아가, **포트폴리오 셀프
셀렉션**과 **사후 objective 폴리시**를 추가한 버전이다. 입력/출력 계약은
동일하다.

```python
def algorithm(prob_info: dict, timelimit: float = 60, repair_mode: str = "greedy") -> dict:
    ...
    return {"operations": {...}}
```

## 개선 결과 (프록시 인스턴스 기준, objective — 낮을수록 좋음)

| 문제 | 이전 제출(3차, myalgorithm6) | 이번 제출(4차, myalgorithm7) | 개선폭 |
|---|---|---|---|
| P1 | 40,560 | 12,550 | -69% |
| P2 | 117,532 | 54,696 | -53% |
| P3 | 1,171,247 | 1,131,659 | -3% |
| P4 | 15,600,438 | 12,487,463 | -20% |
| P5 | 39,594,087 | 33,774,026 | -15% |
| P6 | 130,733,035 | 58,323,300 | -55% |

전 문제 feasible, 전부 60초 제한 이내.

## 알고리즘 구조

1. **Phase 1 — EDD greedy 배치**: due_date 기준 정렬 후 (베이, 방향, 위치,
   시간창) 조합을 점수화해 배치. Due-date shielding으로 "먼저 나가야 할
   블록의 크레인 경로를 막는 위치"를 후보에서 배제.
2. **Phase 2 — repair**: 위반 블록을 재배치. 두 번째 위반이면 원인 블록을
   함께 destroy해 같이 재배치(그래도 안 되면 안전한 강제배치로 폴백).
3. **포트폴리오 셀프 셀렉션**: "즉시진입 우선정렬" ON/OFF 두 변형을 실행해
   objective가 낮은 쪽을 채택. 정적 통계로는 어느 쪽이 유리한 인스턴스
   유형인지 구분이 안 돼(실측으로만 드러남), 리더보드가 순위제라는 점에
   맞춰 "어느 유형에서도 지지 않는" 전략을 택했다. 시간이 부족한 인스턴스는
   자동으로 단일 실행이 되어 항상 안전하다.
4. **AABB 좌압축**: 지각 블록을 자기 베이 안에서 더 이른 시각으로 당긴다
   (bbox-서로소 위치는 크레인 간섭이 구조적으로 없다는 성질을 이용해 순수
   정수 연산으로 탐색 — Shapely 호출 없이 빠르다).
5. **Phase 3 — objective 폴리시**: 남는 시간 동안 objective 기여도가 큰
   블록을 하나씩 destroy-reinsert(부하·지각 공략) + 선호 베이 스왑(선호도
   손실 공략)으로 다듬는다. 매 이동마다 전체 feasibility를 재검증하고
   objective가 **엄격히** 개선될 때만 채택하므로, 이 패스는 원리적으로
   결과를 절대 악화시킬 수 없다.

밀도기반 K-best 디스패처, 크레인 경로 위상정렬 등 이전 버전의 개선은 모두
그대로 유지된다.

## alg_tester 사용법

```
conda env create -f ogc2026_env.yml   # 최초 1회
conda activate ogc2026
python alg_tester/alg_tester_app.py
```

1. **Instance** 옆 `...` 클릭 → 풀어볼 문제 인스턴스 JSON 선택
2. **Algorithm** 옆 `...` 클릭 → `myalgorithm.py`가 들어있는 폴더 선택
   (예: `alg_tester/myalgorithm7_folder`)
3. 시간제한을 설정하고 **[Run]** 클릭 → Solution 탭에 feasibility 검사
   결과와 objective 값이 표시됨

- 알고리즘은 서브프로세스로 실행되므로, 코드의 print 출력은 로그 패널에
  그대로 나온다.
- `alg_tester/utils.py`의 feasibility checker는 실제 채점에 쓰이는 것과
  동일한 로직이다 — 여기서 PASS면 유효한 해다.
- `alg_tester/myalgorithm{4,6,7}_folder/`는 각 회차 제출 버전의 스냅샷이다.
  새 버전을 테스트하려면 새 폴더를 만들어 `myalgorithm.py`(+ 참조하는
  `utils.py`)를 복사해 넣으면 된다.
