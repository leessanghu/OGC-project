"""
benchmark2.py — myalgorithm2.py 3가지 설정 비교 벤치마크

실행:
    cd baseline
    python benchmark2.py

비교 대상:
  cfg-A  v0 baseline (K=1 equivalent by running original code path, no B term)
         실제로는 K_BEST=999(사실상 무제한), W5=0, CANDIDATE_MODE="v0"
         → baseline_greedy.py 와 동일한 후보 생성 + 무제한 후보 평가

  cfg-B  v1 only (K=1 equivalent: K_BEST=999, W5=0, CANDIDATE_MODE="v1")
         → 레이어별 bbox 후보만 적용, 점수/K-best는 baseline과 동일

  cfg-C  v1 + Part B (W5=1e-3) + Part C (K_BEST=6)
         → CANDIDATE_MODE="v1", K_BEST=6, W5=1e-3

인스턴스: prob_1 (소형), prob_5 (중형 3-bay), prob_38 (대형)
"""

import json
import pathlib
import sys
import time

# baseline 디렉토리가 현재 경로에 있어야 함
sys.path.insert(0, str(pathlib.Path(__file__).parent))

import myalgorithm2 as alg
from utils import check_feasibility

INSTANCES = [
    pathlib.Path(__file__).parent.parent / "train" / "prob_1.json",
    pathlib.Path(__file__).parent.parent / "train" / "prob_5.json",
    pathlib.Path(__file__).parent.parent / "train" / "prob_38.json",
]

CONFIGS = [
    {
        "name":           "cfg-A (v0, K=∞, W5=0)",
        "CANDIDATE_MODE": "v0",
        "K_BEST":         999,   # 사실상 무제한 → baseline과 동일
        "W5":             0.0,
    },
    {
        "name":           "cfg-B (v1, K=∞, W5=0)",
        "CANDIDATE_MODE": "v1",
        "K_BEST":         999,
        "W5":             0.0,
    },
    {
        "name":           "cfg-C (v1, K=6, W5=1e-3)",
        "CANDIDATE_MODE": "v1",
        "K_BEST":         6,
        "W5":             1e-3,
    },
]

TIMELIMIT = 60.0


def run_one(prob_info: dict, cfg: dict) -> dict:
    """한 설정 × 한 인스턴스 실행. 결과 dict 반환."""
    alg.CANDIDATE_MODE = cfg["CANDIDATE_MODE"]
    alg.K_BEST         = cfg["K_BEST"]
    alg.W5             = cfg["W5"]

    t0 = time.time()
    try:
        sol = alg.algorithm(prob_info, timelimit=TIMELIMIT, repair_mode="greedy")
        elapsed = time.time() - t0
        r = check_feasibility(prob_info, sol)
        # 베이 동시 점유 블록 수 계산 (단순 평균)
        ops = sol["operations"]
        bay_concurrent = _avg_concurrent(prob_info, sol)
        return {
            "feasible":       r["feasible"],
            "stage":          r["stage"],
            "objective":      r["objective"],
            "obj1":           r["obj1"],
            "obj2":           r["obj2"],
            "obj3":           r["obj3"],
            "elapsed":        elapsed,
            "avg_concurrent": bay_concurrent,
        }
    except Exception as e:
        elapsed = time.time() - t0
        return {
            "feasible":       False,
            "stage":          "ERROR",
            "objective":      None,
            "obj1":           None,
            "obj2":           None,
            "obj3":           None,
            "elapsed":        elapsed,
            "avg_concurrent": None,
            "error":          str(e),
        }


def _avg_concurrent(prob_info: dict, sol: dict) -> float:
    """베이당 평균 동시 점유 블록 수 (시간 가중 평균)."""
    n_bays = len(prob_info["bays"])
    blocks_data = prob_info["blocks"]
    ops = sol["operations"]

    # ENTRY/EXIT에서 (block_id, bay_id, entry_time, exit_time) 재구성
    entry_map = {}  # block_id -> (bay_id, entry_time)
    exit_map  = {}  # block_id -> exit_time

    for t_str, op_list in ops.items():
        t = int(t_str)
        for op in op_list:
            if op["type"] == "ENTRY":
                entry_map[op["block_id"]] = (op["bay_id"], t)
            elif op["type"] == "EXIT":
                exit_map[op["block_id"]] = t

    if not entry_map:
        return 0.0

    # 베이별 [entry, exit) 구간 수집
    bay_intervals: list[list[tuple[int, int]]] = [[] for _ in range(n_bays)]
    for bid, (bay_id, et) in entry_map.items():
        xt = exit_map.get(bid, et)
        bay_intervals[bay_id].append((et, xt))

    # 각 베이에서 시간별 동시 점유 블록 수의 최대값 평균
    max_concurrents = []
    for j in range(n_bays):
        ivs = bay_intervals[j]
        if not ivs:
            max_concurrents.append(0)
            continue
        events = []
        for a, e in ivs:
            events.append((a, +1))
            events.append((e, -1))
        events.sort()
        cur = mx = 0
        for _, delta in events:
            cur += delta
            if cur > mx:
                mx = cur
        max_concurrents.append(mx)

    return sum(max_concurrents) / n_bays


def fmt(val, fmt_str=".0f", none_str="N/A"):
    if val is None:
        return none_str
    return format(val, fmt_str)


def main():
    print("=" * 80)
    print("myalgorithm2.py 벤치마크 (3인스턴스 × 3설정)")
    print("=" * 80)

    # 인스턴스 로드
    instances = []
    for p in INSTANCES:
        if not p.exists():
            print(f"[경고] 파일 없음: {p}")
            instances.append(None)
        else:
            with open(p) as f:
                instances.append(json.load(f))

    # 결과 수집
    all_results = {}  # (inst_name, cfg_name) -> result_dict

    for inst_path, prob_info in zip(INSTANCES, instances):
        if prob_info is None:
            continue
        inst_name = prob_info["name"]
        n_blocks  = len(prob_info["blocks"])
        n_bays    = len(prob_info["bays"])
        print(f"\n{'─'*70}")
        print(f"인스턴스: {inst_name}  ({n_blocks}블록, {n_bays}베이)")
        print(f"{'─'*70}")

        for cfg in CONFIGS:
            print(f"\n  설정: {cfg['name']}")
            print(f"  실행 중...")
            r = run_one(prob_info, cfg)
            all_results[(inst_name, cfg["name"])] = r

            if r["feasible"]:
                print(f"  ✓ 실현 가능  |  obj={fmt(r['objective'])}  "
                      f"(obj1={fmt(r['obj1'])}, obj2={fmt(r['obj2'])}, "
                      f"obj3={fmt(r['obj3'])})  "
                      f"시간={r['elapsed']:.1f}s  "
                      f"최대동시={fmt(r['avg_concurrent'], '.1f')}")
            else:
                err = r.get("error", "")
                print(f"  ✗ 비실현가능  stage={r['stage']}  "
                      f"시간={r['elapsed']:.1f}s"
                      + (f"  오류: {err}" if err else ""))

    # 결과 비교 표 출력
    print(f"\n\n{'='*80}")
    print("비교 결과 표")
    print("=" * 80)
    inst_names = [p.stem for p in INSTANCES if (p.exists())]

    header = f"{'인스턴스':<12} {'설정':<32} {'실현가능':<8} {'목적함수':<12} {'obj1':<10} {'obj2':<8} {'obj3':<8} {'시간(s)':<8} {'최대동시':<8}"
    print(header)
    print("─" * len(header))

    for inst_path, prob_info in zip(INSTANCES, instances):
        if prob_info is None:
            continue
        inst_name = prob_info["name"]
        for cfg in CONFIGS:
            r = all_results.get((inst_name, cfg["name"]))
            if r is None:
                continue
            feasible_str = "O" if r["feasible"] else f"X(s{r['stage']})"
            print(f"{inst_name:<12} {cfg['name']:<32} {feasible_str:<8} "
                  f"{fmt(r['objective'], '.0f'):<12} "
                  f"{fmt(r['obj1'], '.1f'):<10} "
                  f"{fmt(r['obj2'], '.1f'):<8} "
                  f"{fmt(r['obj3'], '.1f'):<8} "
                  f"{r['elapsed']:<8.1f} "
                  f"{fmt(r['avg_concurrent'], '.1f'):<8}")

    print("─" * len(header))
    print("\n[범례] obj1=지연시간, obj2=부하불균형, obj3=선호도페널티")
    print("[참고] K=∞는 모든 후보 평가(baseline 동작), K=6은 첫 6개만 평가")


if __name__ == "__main__":
    main()
