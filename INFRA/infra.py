import time
from collections import deque

import numpy as np
import serial
import torch
import torch.nn as nn
import RPi.GPIO as GPIO
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont   # frames are pre-drawn once (see section 4)

# ==========================================
# 1. Hardware Pin & Device Configuration
# ==========================================
TRIG1, ECHO1 = 23, 22   # Ultrasonic 1: Pedestrian risk (1D-CNN), same pins as collect/collect.py
TRIG2, ECHO2 = 5, 6     # Ultrasonic 2: Illegal parking (voltage divider on ECHO2)
PIR_PIN = 17            # PIR: Auxiliary motion display only

OLED_ADDR = 0x3C
OLED_W, OLED_H = 128, 64   # same 128x64 panel as the UNO OLED

HC12_PORT = "/dev/serial0"
HC12_BAUD = 9600

MODEL_PATH = "/home/user2/cnn3_base_e150.pt"

# ------------------------------------------
# Conditions used in training (must stay the same as train/train.py, collect/collect.py)
# ------------------------------------------
SAMPLE_INTERVAL = 0.05   # s, ultrasonic 1 sampling period (20 Hz)
WIN = 40                 # window length (samples) = 2 s
MIN_LEN = 5              # shortest window used in training (--min_len)
DIST_MIN = 3.0           # cm, lower clip of the measured distance
DIST_MAX = 150.0         # cm, upper clip (also the value used when no echo / timeout)
APPROACH = 1             # class index: 0 = stop, 1 = approach, 2 = leave
APPROACH_BIAS = 0.5      # added to the 'approach' score before argmax (train.py BIAS)
# SAFE : class 0 (stop), class 2 (leave)   CAUTION : class 1 (approach)

# Illegal parking
PARKING_DIST_CM = 25.0
PARKING_HOLD_SEC = 2.0
PARKING_EVERY = 4        # read ultrasonic 2 and PIR every N samples (4 x 0.05 s = 0.2 s)

# Output
BLINK_SEC = 0.3
OLED_MIN_INTERVAL = 0.25   # limit OLED redraw rate of the normal screen so sampling is not delayed
                           # (a 128x64 redraw is ~1 KB over I2C: set i2c_arm_baudrate=400000 to make it ~4x faster)
STATUS_PRINT_EVERY = 10    # print terminal status every N samples (0.5 s) (only if SHOW_AI_DETAILS)

# Link to the terminal (UNO) and risk release
TX_INTERVAL = 0.5          # s, repeat the current state (D / N): the terminal treats 2 s of silence as a lost link
RELEASE_HOLD_SEC = 2.0     # s, the risk must stay cleared this long before D is released (0 = release at once)
SHOW_AI_DETAILS = False    # True: also print the long status table and AI scores every 0.5 s

oled = None
hc12 = None


def init_hardware():
    global oled, hc12
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for t, e in ((TRIG1, ECHO1), (TRIG2, ECHO2)):
        GPIO.setup(t, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(e, GPIO.IN)
    GPIO.setup(PIR_PIN, GPIO.IN)

    oled = ssd1306(i2c(port=1, address=OLED_ADDR), width=OLED_W, height=OLED_H)
    hc12 = serial.Serial(HC12_PORT, baudrate=HC12_BAUD, timeout=0.2)
    build_frames()


# ==========================================
# 2. 1D-CNN (same structure as model/model.py) & Loader
# ==========================================
class CNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.layer1 = nn.Conv1d(2, 32, 5, 1, padding='same')
        self.layer2 = nn.Conv1d(32, 64, 5, 1, padding='same')
        self.relu = nn.ReLU()
        self.pooling = nn.AdaptiveAvgPool1d(1)
        self.flatten = nn.Flatten()
        self.dropout = nn.Dropout(p=0.3)
        self.linear = nn.Linear(64, 3)

    def forward(self, x):                       # x: (batch, 2, WIN)
        x1 = self.relu(self.layer1(x))          # (batch, 32, WIN)
        x2 = self.relu(self.layer2(x1))         # (batch, 64, WIN)
        features = self.flatten(self.pooling(x2))   # (batch, 64)
        features = self.dropout(features)
        return self.linear(features)            # (batch, 3) scores (no softmax)


def load_cnn_model(path):
    """Load the state_dict saved by train/train.py. Returns None on failure."""
    try:
        try:
            state = torch.load(path, map_location="cpu", weights_only=True)
        except TypeError:                       # old torch without weights_only
            state = torch.load(path, map_location="cpu")
        model = CNN()
        model.load_state_dict(state)            # strict: every key and shape must match
        model.eval()
        print("[MODEL] Loaded:", path)
        return model
    except Exception as e:
        print(f"[MODEL ERROR] {e}")
        return None


def build_input(window):
    """Recent distances (cm, 1..WIN samples) -> (1, 2, WIN) float32.
    Same as train/train.py make_input(): ch0 = distance / DIST_MAX, ch1 = mask (1 = real sample),
    samples are right-aligned and the front is zero padded."""
    d = np.asarray(window, dtype=np.float32)[-WIN:]
    n = len(d)
    x = np.zeros((1, 2, WIN), dtype=np.float32)
    x[0, 0, WIN - n:] = d / DIST_MAX
    x[0, 1, WIN - n:] = 1.0
    return x


def pick_class(logits, bias=APPROACH_BIAS):
    """Same as train/train.py apply_bias(): add bias to the 'approach' score, then argmax."""
    z = np.array(logits, dtype=np.float32, copy=True)
    z[APPROACH] += bias
    return int(z.argmax())


def predict_pedestrian_risk(model, window, dist1):
    # Return the original risk decision and diagnostic details.
    if dist1 is None:
        return "CAUTION", {"reason": "SENSOR ERROR"}

    window.append(float(dist1))
    if model is None:
        return "CAUTION", {"reason": "MODEL NOT LOADED"}
    if len(window) < MIN_LEN:
        return "CAUTION", {"reason": "WAITING FOR SAMPLES"}

    try:
        x = torch.from_numpy(build_input(window))
        with torch.no_grad():
            logits = model(x)[0]
            if logits.shape != (3,) or not torch.isfinite(logits).all():
                raise ValueError("Invalid model output")
            biased = logits.clone()
            biased[APPROACH] += APPROACH_BIAS
            raw_probs = torch.softmax(logits, dim=0)
            biased_probs = torch.softmax(biased, dim=0)
            pred = pick_class(logits.numpy())
            details = {
                "reason": "OK",
                "logits": logits.tolist(),
                "biased_logits": biased.tolist(),
                "raw_probs": raw_probs.tolist(),
                "biased_probs": biased_probs.tolist(),
                "raw_pred": int(logits.argmax().item()),
                "pred": pred,
            }
        return ("CAUTION" if pred == APPROACH else "SAFE"), details
    except Exception as e:
        return "CAUTION", {"reason": f"INFERENCE ERROR: {e}"}


def print_ai_details(details, window, d2):
    names = ["STOP", "APPROACH", "LEAVE"]
    print(f"Parking distance (d2): {d2} cm")
    print(f"Window: {len(window)}/{WIN} | AI reason: {details['reason']}")
    if window:
        print(f"Window range: {min(window):.1f} ~ {max(window):.1f} cm")
    if details["reason"] != "OK":
        print("Class scores unavailable (no valid prediction)")
        return
    print("CLASS       LOGIT   BIASED    RAW %   BIASED %")
    for i, name in enumerate(names):
        print(f"{name:<10} {details['logits'][i]:7.3f} "
              f"{details['biased_logits'][i]:8.3f} "
              f"{details['raw_probs'][i]*100:7.2f} "
              f"{details['biased_probs'][i]*100:9.2f}")
    print(f"Raw prediction: {names[details['raw_pred']]} | "
          f"Final prediction: {names[details['pred']]}")


# ==========================================
# 3. Sensor Readers & Illegal Parking Logic
# ==========================================
def measure_cm(trig, echo):
    """Same procedure as collect/collect.py measure_distance():
    no echo / timeout -> DIST_MAX, result clipped to DIST_MIN..DIST_MAX, rounded to 0.1 cm.
    Returns None only when a GPIO error occurs."""
    try:
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        timeout = time.perf_counter() + 0.02
        pulse_start = time.perf_counter()
        while GPIO.input(echo) == 0:
            pulse_start = time.perf_counter()
            if pulse_start > timeout:
                return DIST_MAX

        timeout = time.perf_counter() + 0.02
        pulse_end = time.perf_counter()
        while GPIO.input(echo) == 1:
            pulse_end = time.perf_counter()
            if pulse_end > timeout:
                return DIST_MAX

        dist = (pulse_end - pulse_start) * 17150.0
        return round(max(DIST_MIN, min(DIST_MAX, dist)), 1)
    except Exception as e:
        print(f"[US ERROR] {e}")
        return None


def read_pir():
    try:
        val = GPIO.input(PIR_PIN)
        return "DETECTED" if val == 1 else "NONE"
    except Exception as e:
        print(f"[PIR ERROR] {e}")
        return "NONE"


parking_start_time = None


def update_illegal_parking(dist2, now):
    global parking_start_time

    # Sensor fault -> do not treat as clear
    if dist2 is None:
        return True

    if dist2 <= PARKING_DIST_CM:
        if parking_start_time is None:
            parking_start_time = now
        return (now - parking_start_time) >= PARKING_HOLD_SEC
    else:
        parking_start_time = None
        return False


# ==========================================
# 4. OLED Renderers (128x64)
#    All frames are drawn ONCE at start (7 images of 1 KB each) and only displayed later:
#    no canvas, no font loading and no new images while running, so RAM stays flat.
#    The risk screen flashes with the OLED's own invert command (1 byte, no redraw).
# ==========================================
FONT_PATHS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf"]
frames = {}
oled_inverted = False


def _font(size):
    for path in FONT_PATHS:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass
    return ImageFont.load_default()


def _text(d, s, cx, top, font, fill):
    """Text centered on cx, glyph top at `top`."""
    x0, y0, x1, y1 = d.textbbox((0, 0), s, font=font)
    d.text((cx - (x0 + x1) / 2, top - y0), s, font=font, fill=fill)


def _stand(d, x, y, c):                       # standing person  (SAFE)
    d.ellipse([x + 5, y, x + 10, y + 5], fill=c)
    d.line([(x + 7, y + 7), (x + 7, y + 15)], fill=c, width=3)
    d.line([(x + 7, y + 8), (x + 4, y + 15)], fill=c, width=2)
    d.line([(x + 7, y + 8), (x + 10, y + 15)], fill=c, width=2)
    d.line([(x + 7, y + 15), (x + 5, y + 22)], fill=c, width=2)
    d.line([(x + 7, y + 15), (x + 9, y + 22)], fill=c, width=2)


def _run(d, x, y, c):                         # running person   (CAUTION)
    d.ellipse([x + 9, y, x + 14, y + 5], fill=c)
    d.line([(x + 10, y + 7), (x + 6, y + 15)], fill=c, width=3)
    d.line([(x + 9, y + 9), (x + 15, y + 8)], fill=c, width=2)
    d.line([(x + 9, y + 9), (x + 3, y + 8)], fill=c, width=2)
    d.line([(x + 6, y + 15), (x + 12, y + 17), (x + 11, y + 22)], fill=c, width=2)
    d.line([(x + 6, y + 15), (x + 1, y + 19), (x + 0, y + 22)], fill=c, width=2)
    d.line([(x - 5, y + 7), (x - 1, y + 7)], fill=c)           # speed lines
    d.line([(x - 6, y + 12), (x - 1, y + 12)], fill=c)


def _waves(d, x, y, c):                       # motion sensor waves (PIR)
    d.ellipse([x, y + 8, x + 4, y + 12], fill=c)
    for r in (7, 12, 17):
        d.arc([x + 2 - r, y + 10 - r, x + 2 + r, y + 10 + r], -45, 45, fill=c, width=2)


def _warn(d, x, y, c):                        # warning triangle (ERROR)
    d.polygon([(x + 8, y + 1), (x, y + 20), (x + 16, y + 20)], fill=c)
    d.line([(x + 8, y + 7), (x + 8, y + 14)], fill=1 - c, width=2)
    d.point((x + 7, y + 17), fill=1 - c)
    d.point((x + 8, y + 17), fill=1 - c)


def _normal_frame(ai, pir, font):
    """Left cell = AI (SAFE / CAUTION / ERROR), right cell = PIR (NONE / DETECTED)."""
    im = Image.new("1", (OLED_W, OLED_H), 0)
    d = ImageDraw.Draw(im)
    if ai == "SAFE":
        _stand(d, 24, 8, 1)
        _text(d, "SAFE", 32, 42, font, 1)
    else:                                     # CAUTION / ERROR: white cell
        d.rectangle([0, 0, 63, 63], fill=1)
        if ai == "CAUTION":
            _run(d, 26, 8, 0)
        else:
            _warn(d, 24, 8, 0)
        _text(d, ai, 32, 42, font, 0)
    _waves(d, 86, 10, 1)
    if pir != "DETECTED":
        d.line([(84, 29), (106, 13)], fill=1, width=2)         # crossed-out waves = NONE
    _text(d, pir, 96, 42, font, 1)
    d.line([(64, 0), (64, 63)], fill=1)       # divider
    return im


def _risk_frame(font):
    """Same screen as the terminal (UNO 0002): triangle with '!' + WATCH / OUT!!"""
    im = Image.new("1", (OLED_W, OLED_H), 0)
    d = ImageDraw.Draw(im)
    d.polygon([(64, 0), (44, 26), (84, 26)], fill=1)
    _text(d, "!", 64, 9, font, 0)
    _text(d, "WATCH", 64, 30, font, 1)
    _text(d, "OUT!!", 64, 48, font, 1)
    return im


def build_frames():
    small, big = _font(10), _font(16)
    for ai in ("SAFE", "CAUTION", "ERROR"):
        for pir in ("NONE", "DETECTED"):
            frames[("normal", ai, pir)] = _normal_frame(ai, pir, small)
    frames[("risk",)] = _risk_frame(big)


def show_frame(key):
    oled.display(frames[key])


def set_invert(on):
    """Flash effect (same as the UNO invertDisplay): one command byte, no redraw."""
    global oled_inverted
    if on != oled_inverted:
        oled.command(0xA7 if on else 0xA6)
        oled_inverted = on


def display_ai(ai_status, ai_details):
    """What the AI cell shows. The decision (ai_status) is unchanged: a sensor error,
    a missing model or an inference failure is never shown as SAFE."""
    reason = ai_details.get("reason", "OK")
    if reason.startswith(("SENSOR ERROR", "MODEL NOT LOADED", "INFERENCE ERROR")):
        return "ERROR"
    return ai_status


def print_terminal_status(ai_status, pir_status, parking_detected, combined_risk, d1, late):
    l_col = "CAUTION" if ai_status == "CAUTION" else "SAFE"
    r_col = "DETECTED" if pir_status == "DETECTED" else "NONE"
    print("+-----------------------+")
    print("|           |           |")
    print(f"|  {l_col:<7}  |  {r_col:<8} |")
    print("|           |           |")
    print("+-----------------------+")
    print(f"Parking: {'DETECTED' if parking_detected else 'NONE'} | Combined Risk: {'YES (D)' if combined_risk else 'NO (N)'}")
    print(f"dist1: {d1} cm | late samples: {late}")


def send_hc12(code_char):
    try:
        hc12.write(f"{code_char}\n".encode("ascii"))
    except Exception as e:
        log_event(f"[HC-12 ERROR] {e}")


# ---- live status line (one line, updated in place) + event lines ----
_live_len = 0


def log_event(msg):
    """Print an event line without leaving the live status line half overwritten."""
    global _live_len
    print("\r" + " " * _live_len + "\r" + msg)
    _live_len = 0


def print_live_status(now, d2, parking_detected, disp_ai, risk_raw, risk_on, last_risk_t):
    """PARK: illegal parking state | AI | RISK: combined danger (what is sent to the terminal)."""
    global _live_len
    if parking_detected:
        park = "DETECTED"
    elif parking_start_time is not None:
        park = f"COUNTING {max(0.0, now - parking_start_time):.1f}/{PARKING_HOLD_SEC:.1f}s"
    else:
        park = "NONE"
    dtxt = "ERR" if d2 is None else f"{d2:.1f}cm"
    if not risk_on:
        risk = "NO  -> N"
    elif risk_raw:
        risk = "YES -> D"
    else:
        risk = f"HOLD {now - last_risk_t:.1f}/{RELEASE_HOLD_SEC:.1f}s -> D"
    line = f"PARK: {park:<17} d2={dtxt:>8} | AI: {disp_ai:<7} | RISK: {risk}"
    pad = " " * max(0, _live_len - len(line))
    print("\r" + line + pad, end="", flush=True)
    _live_len = len(line)


# ==========================================
# 5. Main Loop (20 Hz sampling, 300ms Blink During Risk)
# ==========================================
def main():
    try:
        init_hardware()
        cnn_model = load_cnn_model(MODEL_PATH)
        dist1_window = deque(maxlen=WIN)     # recent valid distances of ultrasonic 1

        print("Stabilizing PIR sensor (2s)...")
        time.sleep(2.0)

        prev_risk = None
        prev_parking = False
        last_risk_t = 0.0                    # last time the raw risk (parking AND CAUTION) was true
        last_tx_t = 0.0                      # last time D / N was sent
        invert_state = False
        last_blink_t = time.perf_counter()
        shown = None                         # what is currently on the OLED
        last_draw_t = 0.0

        ai_status = "CAUTION"                # Safe initial assumption until the window has MIN_LEN samples
        pir_status = "NONE"
        parking_detected = False
        d1 = None
        d2 = None

        tick = 0
        late = 0
        next_t = time.perf_counter()

        while True:
            # Fixed 20 Hz schedule (same timing as data collection)
            wait = next_t - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            elif wait < -SAMPLE_INTERVAL:    # more than one sample behind: skip ahead instead of bursting
                late += 1
                next_t = time.perf_counter()
            next_t += SAMPLE_INTERVAL
            now = time.perf_counter()

            # Ultrasonic 1 every sample -> 1D-CNN
            d1 = measure_cm(TRIG1, ECHO1)
            ai_status, ai_details = predict_pedestrian_risk(cnn_model, dist1_window, d1)
            disp_ai = display_ai(ai_status, ai_details)

            # Ultrasonic 2 and PIR at a slower rate
            if tick % PARKING_EVERY == 0:
                time.sleep(0.01)             # let the echo of ultrasonic 1 die out
                d2 = measure_cm(TRIG2, ECHO2)
                pir_status = read_pir()
                parking_detected = update_illegal_parking(d2, time.perf_counter())

            # Combined risk: Illegal parking AND AI CAUTION (PIR excluded)
            risk_raw = bool(parking_detected and (ai_status == "CAUTION"))
            if risk_raw:
                last_risk_t = now
                combined_risk = True
            else:
                # release only after the risk stayed cleared for RELEASE_HOLD_SEC
                combined_risk = bool(prev_risk) and (now - last_risk_t < RELEASE_HOLD_SEC)

            if parking_detected != prev_parking:
                prev_parking = parking_detected
                dtxt = "ERR" if d2 is None else f"{d2:.1f}cm"
                log_event(f"[PARKING] DETECTED (d2={dtxt})" if parking_detected else "[PARKING] CLEARED")

            if combined_risk != prev_risk:
                # First sample, entering risk, or leaving risk (parking cleared / AI no longer CAUTION)
                if prev_risk is None:
                    log_event("[LINK] start -> N")
                elif combined_risk:
                    log_event("[RISK] START (parking + CAUTION) -> D sent, WATCH OUT!! on OLED")
                else:
                    log_event(f"[RISK] END (cleared for {RELEASE_HOLD_SEC:.1f}s) -> N sent")
                send_hc12("D" if combined_risk else "N")
                prev_risk = combined_risk
                invert_state = False             # start with the plain WATCH OUT!! image, flash from 0.3 s
                last_blink_t = now
                last_tx_t = now
            elif now - last_tx_t >= TX_INTERVAL:
                send_hc12("D" if combined_risk else "N")   # keep repeating so the terminal never times out
                last_tx_t = now

            if SHOW_AI_DETAILS:
                if tick % STATUS_PRINT_EVERY == 0:
                    print_terminal_status(ai_status, pir_status, parking_detected, combined_risk, d1, late)
                    print_ai_details(ai_details, dist1_window, d2)
            elif tick % PARKING_EVERY == 0:
                print_live_status(now, d2, parking_detected, disp_ai, risk_raw, combined_risk, last_risk_t)

            # OLED update (only when something changed)
            if prev_risk and now - last_blink_t >= BLINK_SEC:
                last_blink_t = now
                invert_state = not invert_state      # flash = hardware invert, no redraw

            if not prev_risk:
                set_invert(False)                    # leaving risk: un-invert BEFORE drawing the normal screen
            want = ("risk",) if prev_risk else ("normal", disp_ai, pir_status)
            if want != shown:
                mode_changed = shown is None or want[0] != shown[0]
                if mode_changed or now - last_draw_t >= OLED_MIN_INTERVAL:
                    show_frame(want)
                    shown = want
                    last_draw_t = now
            if prev_risk:
                set_invert(invert_state)             # entering risk: draw first, then invert

            tick += 1

    except KeyboardInterrupt:
        print("\n\nStopped by User")
    finally:
        if oled is not None:
            set_invert(False)
            oled.clear()
        if hc12 is not None:
            hc12.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()