"""
접근/이탈 구간의 '라벨은 시작됐는데 거리는 아직 안 변한' 앞부분을 정지(1)로 바꾼 CSV를 만든다.

위치: data/relabel.py        실행(어느 폴더에서든):  python data/relabel.py
입력: data/raw/data.csv      출력: data/processed/data_relabel.csv   (원본은 건드리지 않음)

판단 방법 (구간마다):
  1) 구간 시작 직전 10샘플의 중앙값을 '원래 거리'로 잡는다.
  2) 거리가 원래 거리에서 10cm 넘게 벗어난 상태가 3샘플 이어지는 첫 지점부터 15샘플에 직선을 맞춘다.
  3) 그 직선을 원래 거리까지 거슬러 올라간 시점 = 일정한 속도라면 움직임이 시작됐을 시점.
  4) 라벨 시작 ~ 그 시점 사이가 min_lag 샘플 이상이고, 바로 앞 구간 라벨이 정지(1)일 때만
     그 샘플들의 라벨을 정지(1)로 바꾼다.
원래 라벨은 'label_orig' 행으로 같이 저장한다 (train.py가 원래 라벨 기준 점수도 같이 낸다).
"""
import argparse
import csv
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent                  # 이 파일이 있는 data/ 폴더

UP = {2: -1, 3: -1, 4: +1}        # 접근(2,3)은 거리가 줄고, 이탈(4)은 늘어난다


def segments(label):
    out, i = [], 0
    while i < len(label):
        j = i
        while j < len(label) and label[j] == label[i]:
            j += 1
        out.append((int(label[i]), i, j - 1))             # (라벨, 시작, 끝)
        i = j
    return out


def start_lag(d, label, s, e, sign):
    """구간 [s, e]에서 '움직임 시작 - 라벨 시작'(샘플 수). 못 재면 None."""
    if s < 10:
        return None                                       # 앞 구간이 없어 원래 거리를 못 잡음
    base = np.median(d[s - 10:s])
    if base >= 149:
        return None                                       # 범위 밖(150)에서 시작
    t10 = next((t for t in range(s, e - 1)
                if all(sign * (d[t + j] - base) > 10 for j in range(3))), None)
    if t10 is None or t10 + 15 > len(d):
        return None
    k = np.arange(t10, t10 + 15)
    slope, icpt = np.polyfit(k, d[k], 1)
    if abs(slope) < 0.15:
        return None
    return (base - icpt) / slope - s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(HERE / "raw" / "data.csv"))
    ap.add_argument("--dst", default=str(HERE / "processed" / "data_relabel.csv"))
    ap.add_argument("--labels", type=int, nargs="+", default=[3], help="바꿀 구간의 라벨 (기본 3=뛰기 접근)")
    ap.add_argument("--min_lag", type=float, default=3, help="이 샘플 수 이상 차이 날 때만 바꿈 (20샘플=1초)")
    args = ap.parse_args()

    with open(args.src, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    data = {}
    for r in rows[1:]:
        if r:
            data.setdefault(r[0], {})[r[1]] = np.array(r[2:], dtype=float)

    new_label, log, skipped = {}, [], []
    for sid, v in data.items():
        d, lab = v["distance"], v["label"].astype(int)
        new = lab.copy()
        for L, s, e in segments(lab):
            if L not in args.labels:
                continue
            lag = start_lag(d, lab, s, e, UP[L])
            if lag is None or lag < args.min_lag:
                continue
            prev = int(lab[s - 1])
            if prev != 1:
                skipped.append((sid, L, s, lag, prev))
                continue
            n = min(int(round(lag)), e - s)               # 구간 전체를 지우지는 않는다
            new[s:s + n] = 1
            log.append((sid, L, s, lag, n))
        new_label[sid] = new

    out = [rows[0]]
    for r in rows[1:]:
        if not r:
            continue
        if r[1] == "label":
            out.append(r[:2] + [str(x) for x in new_label[r[0]]])
            out.append([r[0], "label_orig"] + r[2:])
        else:
            out.append(r)
    Path(args.dst).parent.mkdir(parents=True, exist_ok=True)
    with open(args.dst, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(out)

    print(f"saved: {args.dst}")
    print(f"정지로 바꾼 구간 {len(log)}개, 샘플 {sum(x[4] for x in log)}개")
    for sid, L, s, lag, n in log:
        print(f"  {sid}  라벨{L} 시작 {s * 0.05:.1f}초  차이 {lag:.1f}샘플  -> 앞 {n}샘플을 정지로")
    if skipped:
        print("앞 구간이 정지가 아니라 건너뜀:")
        for sid, L, s, lag, prev in skipped:
            print(f"  {sid}  라벨{L} 시작 {s * 0.05:.1f}초  차이 {lag:.1f}샘플  (앞 구간 라벨 {prev})")


if __name__ == "__main__":
    main()