import cv2
import mediapipe as mp
import serial
import time
import math

# =============================
# SETTINGS
# =============================
CAM_INDEX = 0
SERIAL_PORT = "COM9"
BAUD = 115200

FRAME_W, FRAME_H = 1920, 1080
STEPS_PER_REV = 2048

# Right hand jog
PINCH_ON_PX  = 45
PINCH_OFF_PX = 65
STEPS_PER_DEG = 7.0
FLIP_STEPPER_DIRECTION = False
SEND_EVERY_MS = 20
DEADBAND_STEPS = 2

# Robust fist detect (right hand)
FIST_ON_RATIO  = 0.55
FIST_OFF_RATIO = 0.70

# Mode switch: hold fist
MODE_HOLD_MS = 2000
MODE_COOLDOWN_MS = 1200

# Short fist toggle action (mode 2): require a small hold to avoid noise
SHORT_FIST_MIN_MS = 180
SHORT_FIST_MAX_MS = 900
ACTION_COOLDOWN_MS = 700

# Left hand brightness / speed
BRI_ANGLE_MIN = -90
BRI_ANGLE_MAX = 90
BRI_ALPHA = 0.20
BRI_SEND_EVERY_MS = 40
BRI_DEADBAND = 2

# Speed mapping (Mode 3)
SPEED_MIN_RPM = 5
SPEED_MAX_RPM = 16
SPEED_ALPHA = 0.25
SPEED_SEND_EVERY_MS = 120
SPEED_DEADBAND = 1

# If labels swapped due to flip
SWAP_HAND_LABELS = False

# =============================
# HELPERS
# =============================
def px(lm, W, H):
    return int(lm.x * W), int(lm.y * H)

def dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5

def wrap_steps(x):
    x %= STEPS_PER_REV
    if x < 0:
        x += STEPS_PER_REV
    return x

def wrap_angle_deg(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def map_range(x, in_min, in_max, out_min, out_max):
    if in_max - in_min == 0:
        return out_min
    t = (x - in_min) / (in_max - in_min)
    t = clamp(t, 0.0, 1.0)
    return out_min + t * (out_max - out_min)

def robust_fist_ratio(l, W, H):
    wrist = px(l[0], W, H)
    index_mcp = px(l[5], W, H)
    pinky_mcp = px(l[17], W, H)
    hand_scale = max(dist2(index_mcp, pinky_mcp), 1.0)

    tips = [px(l[8], W, H), px(l[12], W, H), px(l[16], W, H), px(l[20], W, H)]
    avg_tip_dist = sum(dist2(t, wrist) for t in tips) / 4.0
    return avg_tip_dist / hand_scale

def roll_deg_from_hand(l, W, H):
    p5 = px(l[5], W, H)
    p17 = px(l[17], W, H)
    dx = p17[0] - p5[0]
    dy = p17[1] - p5[1]
    return wrap_angle_deg(math.degrees(math.atan2(dy, dx)))

def pinch_dist_px(l, W, H):
    a = px(l[4], W, H)
    b = px(l[8], W, H)
    return int(dist2(a, b))

# =============================
# SERIAL + CAMERA + MEDIAPIPE
# =============================
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
time.sleep(2)
print("[INFO] Serial connected:", SERIAL_PORT)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
if not cap.isOpened():
    ser.close()
    raise RuntimeError("Camera not opened. Try CAM_INDEX=1 or close other apps.")

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

# =============================
# STATE MACHINE
# =============================
MODE = 1  # 1=Stepper, 2=LED, 3=Speed

def next_mode():
    global MODE
    MODE = 1 if MODE == 3 else MODE + 1

# Right jog state
pinch_state = False
prev_roll_right = None
target_steps = 0
last_sent_steps = None
last_send_t = 0

# Right fist state (for mode switch + LED toggle)
fist_state = False
fist_start_ms = None
last_mode_change_ms = 0
last_action_ms = 0
just_changed_mode = False

# Left brightness state
bri_sm = 128
last_bri_sent = None
last_bri_send_t = 0

# Left speed state
speed_sm = 12
last_speed_sent = None
last_speed_send_t = 0

prev_time = time.time()
print("[INFO] Hold RIGHT fist 2s = change mode | ESC exit")

try:
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            break

        frame = cv2.flip(frame, 1)
        H, W, _ = frame.shape

        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        # FPS
        now = time.time()
        fps = 1.0 / max(1e-6, now - prev_time)
        prev_time = now

        got_right = False
        got_left = False

        right_roll = None
        left_roll = None
        pinch_px = None
        fist_ratio = None

        now_ms = int(time.time() * 1000)
        just_changed_mode = False

        if res.multi_hand_landmarks and res.multi_handedness:
            for hand_lm, handed in zip(res.multi_hand_landmarks, res.multi_handedness):
                label = handed.classification[0].label
                if SWAP_HAND_LABELS:
                    label = "Left" if label == "Right" else "Right"

                mp_draw.draw_landmarks(frame, hand_lm, mp_hands.HAND_CONNECTIONS)
                l = hand_lm.landmark

                if label == "Right" and not got_right:
                    got_right = True

                    # --- FIST robust ---
                    fist_ratio = robust_fist_ratio(l, W, H)

                    if not fist_state:
                        is_fist_now = (fist_ratio < FIST_ON_RATIO)
                    else:
                        is_fist_now = (fist_ratio < FIST_OFF_RATIO)

                    # detect fist press/release with timing
                    if is_fist_now and not fist_state:
                        fist_state = True
                        fist_start_ms = now_ms
                    elif (not is_fist_now) and fist_state:
                        # fist released
                        dur = now_ms - (fist_start_ms or now_ms)
                        fist_state = False

                        # If mode didn't just change, allow short-fist action in MODE 2
                        if MODE == 2 and not just_changed_mode and (now_ms - last_action_ms) > ACTION_COOLDOWN_MS:
                            if SHORT_FIST_MIN_MS <= dur <= SHORT_FIST_MAX_MS:
                                ser.write(b"LED\n")
                                last_action_ms = now_ms

                        fist_start_ms = None

                    # mode switch if explaining hold
                    if fist_state and fist_start_ms is not None:
                        hold = now_ms - fist_start_ms
                        if hold >= MODE_HOLD_MS and (now_ms - last_mode_change_ms) > MODE_COOLDOWN_MS:
                            next_mode()
                            last_mode_change_ms = now_ms
                            just_changed_mode = True
                            # reset to avoid double
                            fist_state = False
                            fist_start_ms = None
                            # reset jog reference
                            prev_roll_right = None

                    # --- Right roll + pinch ---
                    right_roll = roll_deg_from_hand(l, W, H)
                    pinch_px = pinch_dist_px(l, W, H)

                    if not pinch_state and pinch_px <= PINCH_ON_PX:
                        pinch_state = True
                        prev_roll_right = None
                    elif pinch_state and pinch_px >= PINCH_OFF_PX:
                        pinch_state = False

                    # MODE 1 and MODE 3 allow stepper jog
                    if MODE in (1, 3):
                        if pinch_state:
                            if prev_roll_right is None:
                                prev_roll_right = right_roll

                            d = wrap_angle_deg(right_roll - prev_roll_right)
                            d = max(-25.0, min(25.0, d))
                            step_delta = int(d * STEPS_PER_DEG)
                            if FLIP_STEPPER_DIRECTION:
                                step_delta = -step_delta

                            if step_delta != 0:
                                target_steps = wrap_steps(target_steps + step_delta)
                            prev_roll_right = right_roll

                            if (now_ms - last_send_t) >= SEND_EVERY_MS:
                                if last_sent_steps is None or abs(target_steps - last_sent_steps) >= DEADBAND_STEPS:
                                    ser.write(f"{target_steps}\n".encode("utf-8"))
                                    last_sent_steps = target_steps
                                    last_send_t = now_ms
                        else:
                            prev_roll_right = right_roll

                if label == "Left" and not got_left:
                    got_left = True
                    left_roll = roll_deg_from_hand(l, W, H)

                    # wrap to [-90..90] for stable mapping
                    wrapped = left_roll
                    while wrapped > 90:
                        wrapped -= 180
                    while wrapped < -90:
                        wrapped += 180

                    # MODE 2: brightness
                    if MODE == 2:
                        bri_target = int(map_range(wrapped, BRI_ANGLE_MIN, BRI_ANGLE_MAX, 0, 255))
                        bri_sm = int(bri_sm + BRI_ALPHA * (bri_target - bri_sm))

                        if (now_ms - last_bri_send_t) >= BRI_SEND_EVERY_MS:
                            if last_bri_sent is None or abs(bri_sm - last_bri_sent) >= BRI_DEADBAND:
                                ser.write(f"B:{bri_sm}\n".encode("utf-8"))
                                last_bri_sent = bri_sm
                                last_bri_send_t = now_ms

                    # MODE 3: speed
                    if MODE == 3:
                        rpm_target = int(map_range(wrapped, BRI_ANGLE_MIN, BRI_ANGLE_MAX, SPEED_MIN_RPM, SPEED_MAX_RPM))
                        speed_sm = int(speed_sm + SPEED_ALPHA * (rpm_target - speed_sm))

                        if (now_ms - last_speed_send_t) >= SPEED_SEND_EVERY_MS:
                            if last_speed_sent is None or abs(speed_sm - last_speed_sent) >= SPEED_DEADBAND:
                                ser.write(f"S:{speed_sm}\n".encode("utf-8"))
                                last_speed_sent = speed_sm
                                last_speed_send_t = now_ms

        # =============================
        # UI
        # =============================
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        mode_text = {1: "MODE 1: STEPPER JOG", 2: "MODE 2: LED + BRIGHTNESS", 3: "MODE 3: SPEED"}[MODE]
        cv2.putText(frame, mode_text, (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

        if got_right:
            cv2.putText(frame, f"RIGHT roll: {right_roll:.1f} | pinch(px): {pinch_px}", (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if fist_ratio is not None:
                cv2.putText(frame, f"Fist ratio: {fist_ratio:.2f} (hold 2s = mode)", (20, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            cv2.putText(frame, "RIGHT: not detected", (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if got_left:
            cv2.putText(frame, f"LEFT roll: {left_roll:.1f}", (20, 175),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if MODE == 2:
                cv2.putText(frame, f"Brightness: {bri_sm}", (20, 205),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            if MODE == 3:
                cv2.putText(frame, f"Speed RPM: {speed_sm}", (20, 205),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            cv2.putText(frame, "LEFT: not detected", (20, 175),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Gesture Modes Controller", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("[INFO] Closed camera + serial")
