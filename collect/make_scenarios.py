import csv
import random
from pathlib import Path

# 기본 설정
NUM_SCENARIOS = 100
TOTAL_DURATION = 20
NUM_SEGMENTS = 4

TIME_STEP = 0.05
METRONOME_BPM = 120

MOVE_DISTANCE = 25
MAX_DISTANCE = 150

DURATION_STEP = 0.5

# 행동별 설정
# 레이블: (행동 이름, 25cm 이동 시간)
ACTIONS = {
    1: ("정지", None),
    2: ("걷기 접근", 3),
    3: ("뛰기 접근", 1),
    4: ("이탈", 1),
}


# 구간 지속시간 생성

def generate_durations(labels):
    """전체 20초를 행동별 시간 규칙에 맞게 분배한다."""

    candidates = []

    total_units = int(TOTAL_DURATION / DURATION_STEP)

    def backtrack(index, remaining, durations):

        if index == NUM_SEGMENTS:
            if remaining == 0:
                candidates.append(durations[:])
            return

        label = labels[index]

        if label == 1:
            # 정지: 최소 2초, 0.5초 단위
            min_units = int(2 / DURATION_STEP)
            step_units = 1

        else:
            # 이동 행동: 25cm 이동 시간의 배수
            unit_time = ACTIONS[label][1]
            min_units = int(unit_time / DURATION_STEP)
            step_units = min_units

        for duration_units in range(
            min_units,
            remaining + 1,
            step_units
        ):

            duration = duration_units * DURATION_STEP

            durations.append(duration)

            backtrack(
                index + 1,
                remaining - duration_units,
                durations
            )

            durations.pop()

    backtrack(0, total_units, [])

    if not candidates:
        return None

    return random.choice(candidates)


# 이동 거리 계산
def calculate_distances(labels, durations):
    """구간별 시작 위치와 종료 위치를 계산한다."""

    # 첫 구간이 이탈이면 임의 위치에서 시작
    # 그 외에는 150cm에서 시작
    if labels[0] == 4:
        current_distance = random.randint(0, 5) * MOVE_DISTANCE
    else:
        current_distance = MAX_DISTANCE

    distances = []

    for label, duration in zip(labels, durations):

        start_distance = current_distance

        # 행동별 이동 횟수 계산
        if label == 1:
            move_count = 0

        else:
            unit_time = ACTIONS[label][1]
            move_count = int(duration / unit_time)

        total_distance = move_count * MOVE_DISTANCE

        # 접근: 센서 방향으로 이동
        if label in (2, 3):
            current_distance -= total_distance

        # 이탈: 센서 반대 방향으로 이동
        elif label == 4:
            current_distance += total_distance

        # 측정 범위 확인
        if not (0 <= current_distance <= MAX_DISTANCE):
            return None

        distances.append({
            "start": start_distance,
            "end": current_distance,
            "distance": total_distance,
            "move_count": move_count
        })

    return distances


# 시나리오 생성
def generate_scenario():

    while True:

        # 행동 4개 무작위 선택
        labels = random.choices(
            list(ACTIONS.keys()),
            k=NUM_SEGMENTS
        )

        # 지속시간 생성
        durations = generate_durations(labels)

        if durations is None:
            continue

        # 이동 거리 및 범위 검증
        distances = calculate_distances(labels, durations)

        if distances is None:
            continue

        return labels, durations, distances


# CSV 저장
def save_scenarios(output_path="scenarios.csv"):

    output_path = Path(__file__).parent / output_path

    rows = []

    for i in range(NUM_SCENARIOS):

        labels, durations, distances = generate_scenario()

        row = {
            "scenario_id": f"scenario_{i + 1:03d}"
        }

        for j in range(NUM_SEGMENTS):

            label = labels[j]
            duration = durations[j]
            distance = distances[j]["distance"]

            action_name = ACTIONS[label][0]

            # 메트로놈 횟수 계산
            if label == 1:
                metronome_count = 0
            else:
                metronome_count = round(
                    duration * METRONOME_BPM / 60
                )

            # 레이블
            row[f"segment_{j + 1}_label"] = (
                f"{action_name}({label})"
            )

            # 이동 거리(소요 시간, 메트로놈 횟수)
            row[f"segment_{j + 1}_duration"] = (
                f"{distance}cm "
                f"({duration:g}초, "
                f"메트로놈 {metronome_count}회)"
            )

        rows.append(row)

    # 기존 CSV 컬럼 구조 유지
    fieldnames = ["scenario_id"]

    for i in range(NUM_SEGMENTS):

        fieldnames.extend([
            f"segment_{i + 1}_label",
            f"segment_{i + 1}_duration"
        ])

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        writer.writeheader()
        writer.writerows(rows)

    print(f"시나리오 {NUM_SCENARIOS}개 생성 완료")
    print(f"저장 위치: {output_path}")


# 실행
if __name__ == "__main__":
    save_scenarios()