import csv
import os
import re
import time

import RPi.GPIO as GPIO

TRIG1 = 23
ECHO1 = 22

# ===== Settings =====
SAMPLE_INTERVAL = 0.05          # s (20 samples per second)
N_SAMPLES = 400                 # 20 s -> t_0 ~ t_399
DIST_MIN = 3.0                  # cm
DIST_MAX = 150.0                # cm (also used when no echo / timeout)

# label value : (name, instruction shown on screen)
LABELS = {
    0: ("NONE",    "stay OUT of sensor range"),
    1: ("STOP",    "stand STILL in front of sensor"),
    2: ("WALK_IN", "WALK toward the sensor"),
    3: ("RUN_IN",  "RUN toward the sensor"),
    4: ("LEAVE",   "move AWAY from the sensor"),
}

# where to stand when a segment starts
START_POSITION = {
    0: "out of range",
    1: "within 150cm",
    2: "about 4m away",
    3: "about 4m away",
    4: "close to sensor (~30cm)",
}

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENARIO_CSV = os.path.join(BASE_DIR, "scenarios.csv")   # team scenario list (GitHub)
DATA_CSV = os.path.join(BASE_DIR, "dataset.csv")         # collected data
# ====================


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(TRIG1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO1, GPIO.IN)


def measure_distance():
    GPIO.output(TRIG1, True)
    time.sleep(0.00001)
    GPIO.output(TRIG1, False)

    timeout = time.perf_counter() + 0.02
    pulse_start = time.perf_counter()
    while GPIO.input(ECHO1) == 0:
        pulse_start = time.perf_counter()
        if pulse_start > timeout:
            return DIST_MAX

    timeout = time.perf_counter() + 0.02
    pulse_end = time.perf_counter()
    while GPIO.input(ECHO1) == 1:
        pulse_end = time.perf_counter()
        if pulse_end > timeout:
            return DIST_MAX

    d = (pulse_end - pulse_start) * 17150
    d = max(DIST_MIN, min(DIST_MAX, d))
    return round(d, 1)


def load_scenarios():
    """Read team scenarios.csv -> {scenario_id: [(label, n_samples), ...]} (file order kept)."""
    if not os.path.exists(SCENARIO_CSV):
        raise SystemExit(f"scenarios.csv not found: {SCENARIO_CSV}")

    scenarios = {}
    with open(SCENARIO_CSV, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if "segment_1_label" not in (reader.fieldnames or []):
            raise SystemExit("scenarios.csv is not the team format "
                             "(no segment_1_label column). Download it again from GitHub.")
        for row in reader:
            sid = row["scenario_id"].strip()
            segments = []
            for i in range(1, 7):
                label_text = (row.get(f"segment_{i}_label") or "").strip()
                if not label_text:
                    continue
                m = re.search(r"\((\d)\)", label_text)      # "walk(2)" -> 2
                if not m:
                    raise SystemExit(f"{sid}: cannot read label number in segment {i}")
                label = int(m.group(1))
                seconds = float(row[f"segment_{i}_duration"])
                segments.append((label, round(seconds / SAMPLE_INTERVAL)))

            total = sum(n for _, n in segments)
            if total != N_SAMPLES:
                raise SystemExit(f"{sid}: total {total * SAMPLE_INTERVAL:.2f}s is not 20s")
            scenarios[sid] = segments
    return scenarios


def scenario_text(scenario):
    return " -> ".join(f"{LABELS[v][0]}({n * SAMPLE_INTERVAL:.1f}s)" for v, n in scenario)


def expand_labels(scenario):
    labels = []
    for v, n in scenario:
        labels += [v] * n
    return labels


def collect(scenario):
    labels = expand_labels(scenario)
    dist = []
    late = 0

    starts = {}
    idx = 0
    for v, n in scenario:
        starts[idx] = (v, n)
        idx += n
    warn_ahead = round(1.0 / SAMPLE_INTERVAL)  # announce next segment 1s early

    t0 = time.perf_counter()
    for i in range(N_SAMPLES):
        target = t0 + i * SAMPLE_INTERVAL
        wait = target - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        elif wait < -SAMPLE_INTERVAL / 2:
            late += 1

        if i in starts:
            v, n = starts[i]
            name, guide = LABELS[v]
            print(f"[{i * SAMPLE_INTERVAL:5.2f}s] NOW: {name:<7} ({n * SAMPLE_INTERVAL:.1f}s)  {guide}")
        if (i + warn_ahead) in starts and i + warn_ahead > 0:
            nv = starts[i + warn_ahead][0]
            print(f"          next in 1s -> {LABELS[nv][0]} (start: {START_POSITION[nv]})")

        dist.append(measure_distance())

    print(f"[{N_SAMPLES * SAMPLE_INTERVAL:5.2f}s] END")
    return dist, labels, late


def read_dataset():
    """Return (header, rows) of dataset.csv, or (None, [])."""
    if not os.path.exists(DATA_CSV):
        return None, []
    with open(DATA_CSV, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        return None, []
    return rows[0], rows[1:]


def collected_ids():
    _, rows = read_dataset()
    return {row[0] for row in rows if row}


def save(sid, dist, labels):
    """Save one scenario. If sid already exists, its old rows are replaced."""
    header, rows = read_dataset()
    if header is None:
        header = ["scenario_id", "row_type"] + [f"t_{i}" for i in range(N_SAMPLES)]

    replaced = any(row and row[0] == sid for row in rows)
    rows = [row for row in rows if row and row[0] != sid]
    rows.append([sid, "distance"] + dist)
    rows.append([sid, "label"] + labels)
    rows.sort(key=lambda r: (r[0], 0 if r[1] == "distance" else 1))  # keep id order

    tmp = DATA_CSV + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    os.replace(tmp, DATA_CSV)   # write fully, then swap (safe if power is lost)

    print(f"{'Replaced' if replaced else 'Saved'} {sid} -> {DATA_CSV}")


def pick_scenario(cmd, ids, done):
    """Return scenario_id for the command, or None."""
    if cmd == "":
        for sid in ids:
            if sid not in done:
                return sid
        print("All scenarios are collected.")
        return None

    if cmd.isdigit():                       # "5" -> scenario_005
        cmd = f"scenario_{int(cmd):03d}"
    if cmd in ids:
        return cmd
    print("Unknown scenario. Type a number like 5 or scenario_005")
    return None


def main():
    scenarios = load_scenarios()
    ids = list(scenarios.keys())
    setup_gpio()

    try:
        print("Data collector ready.")
        print(f"  {len(ids)} scenarios loaded from {SCENARIO_CSV}")
        print(f"  {N_SAMPLES} samples x {SAMPLE_INTERVAL}s = {N_SAMPLES * SAMPLE_INTERVAL:.0f}s per scenario")

        while True:
            done = collected_ids()
            print(f"\nProgress: {len([i for i in ids if i in done])}/{len(ids)} collected")
            cmd = input("Enter = next scenario, number = pick (e.g. 5), q = quit > ").strip().lower()
            if cmd == "q":
                break

            sid = pick_scenario(cmd, ids, done)
            if sid is None:
                continue

            scenario = scenarios[sid]
            print(f"{sid}: {scenario_text(scenario)}")
            if sid in done:
                print("  (already collected - saving again will REPLACE it)")
            first = scenario[0][0]
            print(f"  Start position for {LABELS[first][0]}: {START_POSITION[first]}")

            while True:
                input("Press Enter to START (3s countdown) > ")
                for c in (3, 2, 1):
                    print(f"  {c}...")
                    time.sleep(1)
                print("  GO!")

                dist, labels, late = collect(scenario)

                print(f"Done. dist min {min(dist)} / max {max(dist)} cm, late samples {late}")

                c = input("s = save, r = retry same scenario, d = discard > ").strip().lower()
                if c == "s":
                    save(sid, dist, labels)
                    break
                elif c == "r":
                    continue
                else:
                    print("Discarded.")
                    break

    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        GPIO.cleanup()
        print("end")


if __name__ == "__main__":
    main()
