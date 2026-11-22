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

METRONOME_BPM = 120             # same as make_scenarios.py
STEP_CM = 25                    # one move = 25 cm (same as make_scenarios.py)
MAX_POS = 150                   # scenarios start at 150 cm
# time for one 25 cm move (same as make_scenarios.py ACTIONS)
UNIT_TIME = {2: 3.0, 3: 1.0, 4: 3.0}

# label value : (name, instruction shown on screen)
LABELS = {
    0: ("NONE",    "stay OUT of sensor range"),
    1: ("STOP",    "stand STILL"),
    2: ("WALK_IN", "WALK toward sensor, 25cm every 3s (6 beats)"),
    3: ("RUN_IN",  "RUN toward sensor, 25cm every 1s (2 beats)"),
    4: ("LEAVE",   "WALK AWAY, 25cm every 3s (6 beats)"),
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


def direction(label):
    """-1 = toward sensor, +1 = away, 0 = no move."""
    return {2: -1, 3: -1, 4: 1}.get(label, 0)


def start_position(segments):
    """Start 150 cm, except when the first move is LEAVE (then pick the lowest valid spot)."""
    if segments[0]["label"] != 4:
        return MAX_POS
    offsets, pos = [0], 0
    for s in segments:
        pos += direction(s["label"]) * s["cm"]
        offsets.append(pos)
    lo, hi = -min(offsets), MAX_POS - max(offsets)
    for p in range(0, MAX_POS + 1, STEP_CM):    # same 25 cm grid as the generator
        if lo <= p <= hi and p >= STEP_CM:
            return p
    return max(lo, 0)


def load_scenarios():
    """Read team scenarios.csv -> {scenario_id: [segment dict, ...]} (file order kept)."""
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
            i = 1
            while f"segment_{i}_label" in row:
                label_text = (row.get(f"segment_{i}_label") or "").strip()
                info = (row.get(f"segment_{i}_duration") or "").strip()
                i += 1
                if not label_text:
                    continue

                m_label = re.search(r"\((\d)\)", label_text)        # "...(2)"   -> 2
                m_sec = re.search(r"\(\s*([\d.]+)", info)            # "(6s, ..." -> 6
                m_cm = re.match(r"\s*(\d+)\s*cm", info)              # "50cm ..." -> 50
                if not (m_label and m_sec and m_cm):
                    raise SystemExit(f"{sid}: cannot read segment {i - 1}: {label_text} / {info}")

                label = int(m_label.group(1))
                seconds = float(m_sec.group(1))
                segments.append({
                    "label": label,
                    "n": round(seconds / SAMPLE_INTERVAL),
                    "sec": seconds,
                    "cm": int(m_cm.group(1)),
                })

            total = sum(s["n"] for s in segments)
            if total != N_SAMPLES:
                raise SystemExit(f"{sid}: total {total * SAMPLE_INTERVAL:.2f}s is not 20s")

            # position at the start / end of each segment
            pos = start_position(segments)
            for s in segments:
                s["start"] = pos
                pos += direction(s["label"]) * s["cm"]
                s["end"] = pos
            scenarios[sid] = segments
    return scenarios


def segment_text(s):
    name = LABELS[s["label"]][0]
    beats = round(s["sec"] * METRONOME_BPM / 60)
    if s["label"] == 1:
        return f"{name} at {s['start']}cm ({s['sec']:g}s, {beats} beats)"
    return f"{name} {s['start']}->{s['end']}cm ({s['sec']:g}s, {beats} beats)"


def scenario_text(scenario):
    return "\n".join(f"  {k}. {segment_text(s)}" for k, s in enumerate(scenario, 1))


def expand_labels(scenario):
    labels = []
    for s in scenario:
        labels += [s["label"]] * s["n"]
    return labels


def collect(scenario):
    labels = expand_labels(scenario)
    dist = []
    late = 0

    # events by sample index: segment start, 25 cm step targets
    starts, steps = {}, {}
    idx = 0
    for s in scenario:
        starts[idx] = s
        if s["label"] in UNIT_TIME:
            unit = round(UNIT_TIME[s["label"]] / SAMPLE_INTERVAL)
            target = s["start"]
            for k in range(s["n"] // unit):
                target += direction(s["label"]) * STEP_CM
                steps[idx + k * unit] = target   # reach this by the end of the step
        idx += s["n"]
    warn_ahead = round(1.0 / SAMPLE_INTERVAL)   # announce next segment 1s early

    t0 = time.perf_counter()
    for i in range(N_SAMPLES):
        target_t = t0 + i * SAMPLE_INTERVAL
        wait = target_t - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        elif wait < -SAMPLE_INTERVAL / 2:
            late += 1

        now = dist[-1] if dist else None
        if i in starts:
            s = starts[i]
            print(f"[{i * SAMPLE_INTERVAL:5.2f}s] NOW: {segment_text(s)}  - {LABELS[s['label']][1]}")
        if i in steps:
            now_txt = f"(now {now}cm)" if now is not None else ""
            print(f"          step -> {steps[i]}cm {now_txt}")
        if (i + warn_ahead) in starts and i + warn_ahead > 0:
            print(f"          next in 1s -> {segment_text(starts[i + warn_ahead])}")

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


def mismatched_ids(scenarios):
    """Collected scenarios whose saved labels differ from the current scenarios.csv."""
    _, rows = read_dataset()
    bad = []
    for row in rows:
        if len(row) < 2 or row[1] != "label":
            continue
        sid = row[0]
        expected = [str(v) for v in expand_labels(scenarios[sid])] if sid in scenarios else None
        if row[2:] != expected:
            bad.append(sid)
    return bad


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
        print(f"  Use a {METRONOME_BPM} BPM metronome while collecting.")

        bad = mismatched_ids(scenarios)
        if bad:
            print(f"\n  WARNING: {len(bad)} scenario(s) in dataset.csv do not match the current")
            print(f"  scenarios.csv (e.g. {', '.join(bad[:3])}). They were collected with an")
            print("  old scenario list. Move dataset.csv to another folder before collecting.")

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
            print(f"{sid}:")
            print(scenario_text(scenario))
            if sid in done:
                print("  (already collected - saving again will REPLACE it)")
            print(f"  >> Stand at {scenario[0]['start']}cm from the sensor before START")

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
                    print(f"  >> Go back to {scenario[0]['start']}cm")
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

