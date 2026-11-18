#!/usr/bin/env python3
"""
make_scenario.py

규칙
- 시나리오당 구간 수: 3~6개
- 전체 시나리오 길이: 20초
- 없음(label 0)은 시나리오에 포함하지 않음
- label 1(정지): 최소 2초
- label 2(걷기 접근): 최소 3초
- label 3(뛰기 접근): 최소 1.5초
- label 4(이탈): 최소 2초
- 각 구간의 최소 시간을 확보한 뒤 남은 시간을 무작위로 분배
- 시간은 0.05초 단위로 생성하여 50ms 센서 샘플링 주기에 맞춤
"""

import argparse
import csv
import random
from typing import Optional, List, Dict
from pathlib import Path

TOTAL_DURATION = 20.0
TIME_STEP = 0.05
DURATION_STEP = 0.5

# 구간별 최소 시간
LABELS = {
    1: {"name": "정지", "min_duration": 2.0},
    2: {"name": "걷기 접근", "min_duration": 3.0},
    3: {"name": "뛰기 접근", "min_duration": 1.5},
    4: {"name": "이탈", "min_duration": 2.0},
}


def generate_one_scenario(scenario_id: int) -> list[dict]:
    """규칙에 맞는 시나리오 하나를 생성한다."""
    # 3~6개 중 하나 뽑기
    segment_count = random.randint(3, 6)

    # 구간 후보 뽑기
    selected_labels = []
    for _ in range(segment_count):
        candidates = list(LABELS.keys())
        if selected_labels:
            candidates.remove(selected_labels[-1])
        selected_labels.append(random.choice(candidates))

    total_ticks = round(TOTAL_DURATION / DURATION_STEP)
    min_ticks = {
    label: round(info["min_duration"] / DURATION_STEP)
    for label, info in LABELS.items()
    }

    durations_ticks = [min_ticks[label] for label in selected_labels]
    remaining_ticks = total_ticks - sum(durations_ticks)

    if remaining_ticks < 0:
        # 총 시간이 전체를 초과하는지 검사
        raise ValueError("선택된 구간들의 최소 시간 합이 전체 시간을 초과합니다.")

    # 남은 시간을 구간마다 무작위로 분배한다.
    for _ in range(remaining_ticks):
        index = random.randrange(segment_count)
        durations_ticks[index] += 1

    # 실제 시작/종료 시각을 누적 계산
    rows = []
    current_time = 0.0

    for order, (label, duration_tick) in enumerate(
        zip(selected_labels, durations_ticks), start=1
    ):
        duration = duration_tick * DURATION_STEP
        start_time = current_time
        end_time = start_time + duration

        rows.append({
            "scenario_id": f"scenario_{scenario_id:03d}",
            "segment_order": order,
            "label": label,
            "label_name": LABELS[label]["name"],
            "duration_sec": round(duration, 2),
            "start_sec": round(start_time, 2),
            "end_sec": round(end_time, 2),
        })

        current_time = end_time

    # 부동소수점 오차와 전체 길이를 검사
    if round(current_time, 2) != TOTAL_DURATION:
        raise RuntimeError("시나리오 길이가 20초가 아닙니다.")

    return rows

def generate_scenarios(count: int, seed: Optional[int] = None) -> List[Dict]:
    """지정한 개수만큼 시나리오를 생성"""
    if count <= 0:
        raise ValueError("시나리오 개수는 1 이상이어야 함")

    if seed is not None:
        random.seed(seed)

    all_rows = []
    for scenario_id in range(1, count + 1):
        all_rows.extend(generate_one_scenario(scenario_id))

    return all_rows


def save_csv(rows: list[dict], output_path: str) -> None:
    output_path = Path(__file__).parent / output_path

    fieldnames = ["scenario_id"]

    for i in range(1, 7):
        fieldnames.extend([
            f"segment_{i}_label",
            f"segment_{i}_duration"
        ])

    scenarios = {}

    for row in rows:
        scenario_id = row["scenario_id"]

        if scenario_id not in scenarios:
            scenarios[scenario_id] = {"scenario_id": scenario_id}

        i = row["segment_order"]

        scenarios[scenario_id][f"segment_{i}_label"] = (
        f'{row["label_name"]}({row["label"]})'
        )
        scenarios[scenario_id][f"segment_{i}_duration"] = row["duration_sec"]

    with open(output_path, "w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(scenarios.values())


def main():
    parser = argparse.ArgumentParser(
        description="20초 행동 시나리오를 생성하여 CSV로 저장"
    )
    parser.add_argument(
        "--count",
        type=int,
        default=100,
        help="생성할 시나리오 개수 (기본값: 100)",
    )
    parser.add_argument(
        "--output",
        default="scenarios.csv",
        help="출력 CSV 파일명 (기본값: scenarios.csv)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="시드",
    )
    args = parser.parse_args()

    rows = generate_scenarios(args.count, args.seed)
    save_csv(rows, args.output)

    print(f"시나리오 {args.count}개 생성 완료")
    print(f"총 구간 수: {len(rows)}개")
    print(f"저장 위치: {args.output}")


if __name__ == "__main__":
    main()
