import argparse
import csv
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import serial
from flask import Flask, jsonify, request, send_from_directory

# ======= Hardware / tuning defaults =======
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
CAMERA_INDEX = 0

LOWER_YELLOW = np.array([20, 60, 60])
UPPER_YELLOW = np.array([40, 255, 255])
ROI_TOP_RATIO = 0.6
MIN_CONTOUR_AREA = 1200

STEER_CENTER = 90
STEER_RANGE = 30
STEER_ALPHA = 0.25  # 0..1; lower = more smoothing

ESC_NEUTRAL_US = 1500
ESC_FORWARD_MIN_US = 1520
ESC_FORWARD_MAX_US = 1700

COMMAND_PERIOD_S = 0.05
DISTANCE_PRESETS_M = [100, 200, 400, 800, 1600]
WHEEL_DIAMETER_INCHES = 4.2
WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_INCHES * 0.0254 * math.pi
METERS_PER_ROTATION = (2.0 * math.pi * (WHEEL_DIAMETER_INCHES / 2.0)) * 0.0254
PACE_FEEDFORWARD_TABLE = [
    (0.4, 1520),
    (1.0, 1540),
    (2.0, 1580),
    (3.0, 1620),
    (4.0, 1660),
]
PACE_KP_SPEED_US_PER_MPS = 45.0
PACE_KI_SPEED_US_PER_MPS_S = 4.0
PACE_KT_SCHEDULE_US_PER_S = 7.0
PACE_KSPLIT_MPS_PER_S = 0.10
PACE_MIN_TARGET_SPEED_MPS = 0.4
PACE_MAX_TARGET_SPEED_MPS = 8.0
PACE_SPEED_LPF_ALPHA = 0.20
PACE_THROTTLE_LPF_ALPHA = 0.25
PACE_MAX_THROTTLE_RATE_US_PER_S = 80.0
PACE_SCHEDULE_ERROR_CLAMP_S = 3.0
PACE_SPEED_ERROR_DEADBAND_MPS = 0.08
PACE_INTEGRAL_SPEED_ERROR_LIMIT_MPS = 0.60
PACE_MIN_ROLLING_THROTTLE_US = 1540
TELEMETRY_STALE_TIMEOUT_S = 0.35
PACE_LOG_PATH = Path("pace_run.csv")
PACE_MODEL_PATH = Path("pace_model.json")
PACE_MODEL_BIAS_MIN_US = -120.0
PACE_MODEL_BIAS_MAX_US = 120.0
PACE_LEARN_RATE_US_PER_S = 6.0
PACE_SCHEDULE_TRIM_US_PER_S = 3.0
PACE_LOG_FIELDS = [
    "timestamp_s",
    "distance_m",
    "target_speed_mps",
    "v_ref_mps",
    "measured_speed_mps",
    "elapsed_s",
    "expected_elapsed_s",
    "schedule_error_s",
    "speed_error_mps",
    "u_ff",
    "u_fb_speed",
    "u_fb_i",
    "u_fb_schedule",
    "u_model_bias",
    "throttle_cmd_us",
]
# ==========================================

app = Flask(__name__, static_folder="web/static")


@dataclass
class RobotState:
    armed: bool = False
    running: bool = False
    steering_angle: int = STEER_CENTER
    throttle_us: int = ESC_NEUTRAL_US
    vision_ok: bool = False
    serial_ok: bool = False
    manual_steer_mode: str = "vision"  # vision|left|center|right
    last_error: str = ""
    target_distance_m: float = 100.0
    target_time_s: float = 25.0
    target_speed_mps: float = 4.0
    encoder_count: int = 0
    measured_distance_m: float = 0.0
    measured_speed_mps: float = 0.0
    pacing_complete: bool = False
    target_rotations: float = 0.0
    expected_elapsed_s: float = 0.0
    actual_elapsed_s: float = 0.0
    schedule_error_s: float = 0.0
    telemetry_timestamp_s: float = 0.0
    motor_power_percent: float = 0.0
    finish_time_error_s: float = 0.0


@dataclass
class PaceControllerState:
    active: bool = False
    start_time_s: float = 0.0
    speed_filtered_mps: float = 0.0
    speed_integral: float = 0.0
    throttle_filtered_us: float = float(ESC_FORWARD_MIN_US)
    last_throttle_cmd_us: float = float(ESC_FORWARD_MIN_US)
    last_update_s: float = 0.0


state = RobotState()
state_lock = threading.Lock()
pace_state = PaceControllerState()
pace_lock = threading.Lock()
pace_log_lock = threading.Lock()
pace_model_bias_us = 0.0
pace_model_lock = threading.Lock()


def append_pace_log(sample: dict):
    with pace_log_lock:
        new_file = not PACE_LOG_PATH.exists()
        with PACE_LOG_PATH.open("a", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=PACE_LOG_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(sample)


def load_pace_model():
    global pace_model_bias_us
    with pace_model_lock:
        if not PACE_MODEL_PATH.exists():
            pace_model_bias_us = 0.0
            return
        try:
            model = json.loads(PACE_MODEL_PATH.read_text())
            pace_model_bias_us = float(model.get("bias_us", 0.0))
            pace_model_bias_us = clamp(pace_model_bias_us, PACE_MODEL_BIAS_MIN_US, PACE_MODEL_BIAS_MAX_US)
        except (ValueError, OSError):
            pace_model_bias_us = 0.0


def save_pace_model():
    with pace_model_lock:
        payload = {"bias_us": pace_model_bias_us}
    PACE_MODEL_PATH.write_text(json.dumps(payload, indent=2))


def learn_pace_model(finish_time_error_s: float):
    global pace_model_bias_us
    with pace_model_lock:
        pace_model_bias_us += PACE_LEARN_RATE_US_PER_S * finish_time_error_s
        pace_model_bias_us = clamp(pace_model_bias_us, PACE_MODEL_BIAS_MIN_US, PACE_MODEL_BIAS_MAX_US)
    save_pace_model()


def distance_for_counts(counts: int) -> float:
    return counts * METERS_PER_ROTATION


def calculate_target_speed(distance_m: float, time_s: float) -> float:
    if time_s <= 0:
        raise ValueError("time_must_be_positive")
    if distance_m <= 0:
        raise ValueError("distance_must_be_positive")
    return distance_m / time_s

def rotations_for_distance(distance_m: float) -> float:
    return distance_m / METERS_PER_ROTATION

def rotations_for_distance(distance_m: float) -> float:
    return distance_m / METERS_PER_ROTATION


def sync_workout_metrics():
    with state_lock:
        state.target_speed_mps = calculate_target_speed(state.target_distance_m, state.target_time_s)
        state.target_rotations = rotations_for_distance(state.target_distance_m)


def interp_feedforward_throttle(target_speed_mps: float) -> float:
    table = PACE_FEEDFORWARD_TABLE
    if target_speed_mps <= table[0][0]:
        return table[0][1]
    if target_speed_mps >= table[-1][0]:
        return table[-1][1]
    for idx in range(1, len(table)):
        v0, u0 = table[idx - 1]
        v1, u1 = table[idx]
        if v0 <= target_speed_mps <= v1:
            ratio = (target_speed_mps - v0) / (v1 - v0)
            return u0 + ratio * (u1 - u0)
    return table[-1][1]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def low_pass(prev: float, new: float, alpha: float) -> float:
    return alpha * new + (1.0 - alpha) * prev


def compute_pacing_throttle(distance_m: float, measured_speed_mps: float, now_s: float, target_distance_m: float, target_time_s: float):
    with pace_lock:
        if not pace_state.active:
            pace_state.active = True
            pace_state.start_time_s = now_s
            pace_state.last_update_s = now_s
            pace_state.speed_filtered_mps = measured_speed_mps
            pace_state.speed_integral = 0.0
            pace_state.throttle_filtered_us = float(ESC_FORWARD_MIN_US)
            pace_state.last_throttle_cmd_us = float(ESC_FORWARD_MIN_US)

        dt = max(1e-3, now_s - pace_state.last_update_s)
        pace_state.last_update_s = now_s
        elapsed_s = now_s - pace_state.start_time_s

        pace_state.speed_filtered_mps = low_pass(
            pace_state.speed_filtered_mps, measured_speed_mps, PACE_SPEED_LPF_ALPHA
        )

        expected_elapsed_s = target_time_s * clamp(distance_m / max(target_distance_m, 1e-6), 0.0, 1.0)
        schedule_error_s = clamp(elapsed_s - expected_elapsed_s, -PACE_SCHEDULE_ERROR_CLAMP_S, PACE_SCHEDULE_ERROR_CLAMP_S)
        target_speed_mps = target_distance_m / target_time_s
        v_ref_mps = clamp(
            target_speed_mps + (PACE_KSPLIT_MPS_PER_S * schedule_error_s),
            PACE_MIN_TARGET_SPEED_MPS,
            PACE_MAX_TARGET_SPEED_MPS,
        )

        speed_error_mps = v_ref_mps - pace_state.speed_filtered_mps
        with pace_model_lock:
            model_bias_us = pace_model_bias_us

        u_ff = interp_feedforward_throttle(v_ref_mps)
        u_fb_speed = 0.0
        u_fb_i = 0.0
        u_fb_schedule = PACE_SCHEDULE_TRIM_US_PER_S * schedule_error_s
        u_raw = u_ff + model_bias_us + u_fb_schedule

        u_clamped = clamp(u_raw, ESC_FORWARD_MIN_US, ESC_FORWARD_MAX_US)
        pace_state.throttle_filtered_us = low_pass(
            pace_state.throttle_filtered_us, u_clamped, PACE_THROTTLE_LPF_ALPHA
        )
        max_step = PACE_MAX_THROTTLE_RATE_US_PER_S * dt
        u_rate_limited = clamp(
            pace_state.throttle_filtered_us,
            pace_state.last_throttle_cmd_us - max_step,
            pace_state.last_throttle_cmd_us + max_step,
        )
        throttle_cmd_us = int(clamp(u_rate_limited, ESC_FORWARD_MIN_US, ESC_FORWARD_MAX_US))
        if distance_m < target_distance_m and v_ref_mps > 0.7:
            throttle_cmd_us = max(throttle_cmd_us, PACE_MIN_ROLLING_THROTTLE_US)
        pace_state.last_throttle_cmd_us = throttle_cmd_us

    return {
        "throttle_cmd_us": throttle_cmd_us,
        "target_speed_mps": target_speed_mps,
        "v_ref_mps": v_ref_mps,
        "elapsed_s": elapsed_s,
        "expected_elapsed_s": expected_elapsed_s,
        "schedule_error_s": schedule_error_s,
        "speed_error_mps": speed_error_mps,
        "u_ff": u_ff,
        "u_fb_speed": u_fb_speed,
        "u_fb_i": u_fb_i,
        "u_fb_schedule": u_fb_schedule,
        "model_bias_us": model_bias_us,
        "measured_speed_mps": pace_state.speed_filtered_mps,
    }


def reset_pacing_controller():
    with pace_lock:
        pace_state.active = False
        pace_state.speed_integral = 0.0
        pace_state.last_throttle_cmd_us = float(ESC_FORWARD_MIN_US)
        pace_state.throttle_filtered_us = float(ESC_FORWARD_MIN_US)


class SerialLink:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()

    def ensure_connected(self) -> bool:
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"[SERIAL] Connected to {self.port} @ {self.baud}")
            return True
        except serial.SerialException:
            self.ser = None
            return False

    def send_line(self, msg: str) -> bool:
        with self.lock:
            if not self.ensure_connected():
                return False
            try:
                self.ser.write((msg + "\n").encode("utf-8"))
                return True
            except serial.SerialException:
                self.ser = None
                return False

    def read_line(self):
        with self.lock:
            if not self.ensure_connected():
                return None
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                self.ser = None
                return None
        if not raw:
            return None
        return raw.decode("utf-8", errors="ignore").strip()

    def close(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None


serial_link = SerialLink(SERIAL_PORT, BAUD_RATE)


def set_safe_stop(reason: str):
    with state_lock:
        state.running = False
        state.throttle_us = ESC_NEUTRAL_US
        state.last_error = reason
        state.expected_elapsed_s = 0.0
        state.actual_elapsed_s = 0.0
        state.schedule_error_s = 0.0
        state.motor_power_percent = 0.0
    reset_pacing_controller()
    serial_link.send_line("STOP")
    print(f"[SAFETY] STOP triggered: {reason}")


def disarm(reason: str):
    set_safe_stop(reason)
    ok = serial_link.send_line("DISARM")
    with state_lock:
        state.armed = False
        state.serial_ok = ok


def arm_system() -> bool:
    ok1 = serial_link.send_line("STOP")
    ok2 = serial_link.send_line("ARM")
    with state_lock:
        state.armed = ok1 and ok2
        state.running = False
        state.throttle_us = ESC_NEUTRAL_US
        if not state.armed:
            state.last_error = "arm_failed_serial"
    return ok1 and ok2


def serial_reader_loop():
    while True:
        line = serial_link.read_line()
        if not line:
            time.sleep(0.05)
            continue

        if not line.startswith("TELEMETRY:"):
            continue

        payload = {}
        for item in line[len("TELEMETRY:") :].split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            payload[key.strip()] = value.strip()

        now = time.time()
        with state_lock:
            if "count" in payload:
                state.encoder_count = int(payload["count"])
                state.measured_distance_m = distance_for_counts(state.encoder_count)
            if "meters" in payload:
                state.measured_distance_m = float(payload["meters"])
            if "mps" in payload:
                state.measured_speed_mps = float(payload["mps"])
            state.telemetry_timestamp_s = now


def vision_and_control_loop(show_debug: bool):
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        disarm("camera_open_failed")
        print("[CAMERA] Failed to open camera.")
        return

    smoothed_steer = STEER_CENTER
    last_send = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            with state_lock:
                state.vision_ok = False
            disarm("camera_frame_failed")
            time.sleep(0.2)
            continue

        h, w, _ = frame.shape
        roi = frame[int(h * ROI_TOP_RATIO) : h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        steer_from_vision = STEER_CENTER
        detected = False

        if contours:
            biggest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest)
            if area >= MIN_CONTOUR_AREA:
                m = cv2.moments(biggest)
                if m["m00"] != 0:
                    cx = int(m["m10"] / m["m00"])
                    error = (w // 2) - cx
                    raw = STEER_CENTER + int((error / (w // 2)) * STEER_RANGE)
                    steer_from_vision = int(np.clip(raw, STEER_CENTER - STEER_RANGE, STEER_CENTER + STEER_RANGE))
                    detected = True

        smoothed_steer = int((STEER_ALPHA * steer_from_vision) + ((1.0 - STEER_ALPHA) * smoothed_steer))

        now = time.time()
        with state_lock:
            state.vision_ok = detected
            manual_mode = state.manual_steer_mode
            if manual_mode == "left":
                steer_angle = STEER_CENTER - STEER_RANGE
            elif manual_mode == "right":
                steer_angle = STEER_CENTER + STEER_RANGE
            elif manual_mode == "center":
                steer_angle = STEER_CENTER
            else:
                steer_angle = smoothed_steer

            state.steering_angle = int(np.clip(steer_angle, 0, 180))

            running = state.running and state.armed
            steering = state.steering_angle
            distance_m = state.measured_distance_m
            measured_speed_mps = state.measured_speed_mps
            target_distance_m = state.target_distance_m
            target_time_s = state.target_time_s
            telemetry_age_s = now - state.telemetry_timestamp_s if state.telemetry_timestamp_s > 0 else 999.0
        if now - last_send >= COMMAND_PERIOD_S:
            ok1 = serial_link.send_line(f"STEER:{steering}")
            ok2 = True
            if running and telemetry_age_s <= TELEMETRY_STALE_TIMEOUT_S:
                control = compute_pacing_throttle(
                    distance_m=distance_m,
                    measured_speed_mps=measured_speed_mps,
                    now_s=now,
                    target_distance_m=target_distance_m,
                    target_time_s=target_time_s,
                )
                throttle_cmd_us = control["throttle_cmd_us"]
                ok2 = serial_link.send_line(f"THROTTLE:{throttle_cmd_us}")
                with state_lock:
                    state.throttle_us = throttle_cmd_us
                    state.actual_elapsed_s = control["elapsed_s"]
                    state.expected_elapsed_s = control["expected_elapsed_s"]
                    state.schedule_error_s = control["schedule_error_s"]
                    state.motor_power_percent = (
                        (throttle_cmd_us - ESC_FORWARD_MIN_US) / max(1, (ESC_FORWARD_MAX_US - ESC_FORWARD_MIN_US))
                    ) * 100.0
                    if state.measured_distance_m >= state.target_distance_m:
                        finish_time_error_s = control["elapsed_s"] - state.target_time_s
                        learn_pace_model(finish_time_error_s)
                        state.running = False
                        state.pacing_complete = True
                        state.finish_time_error_s = finish_time_error_s
                        state.last_error = "workout_complete"
                append_pace_log(
                    {
                        "timestamp_s": now,
                        "distance_m": distance_m,
                        "target_speed_mps": control["target_speed_mps"],
                        "v_ref_mps": control["v_ref_mps"],
                        "measured_speed_mps": control["measured_speed_mps"],
                        "elapsed_s": control["elapsed_s"],
                        "expected_elapsed_s": control["expected_elapsed_s"],
                        "schedule_error_s": control["schedule_error_s"],
                        "speed_error_mps": control["speed_error_mps"],
                        "u_ff": control["u_ff"],
                        "u_fb_speed": control["u_fb_speed"],
                        "u_fb_i": control["u_fb_i"],
                        "u_fb_schedule": control["u_fb_schedule"],
                        "u_model_bias": control["model_bias_us"],
                        "throttle_cmd_us": throttle_cmd_us,
                    }
                )
            elif running:
                throttle_cmd_us = int(clamp(pace_state.last_throttle_cmd_us, ESC_FORWARD_MIN_US, ESC_FORWARD_MAX_US))
                ok2 = serial_link.send_line(f"THROTTLE:{throttle_cmd_us}")
                with state_lock:
                    state.motor_power_percent = (
                        (throttle_cmd_us - ESC_FORWARD_MIN_US) / max(1, (ESC_FORWARD_MAX_US - ESC_FORWARD_MIN_US))
                    ) * 100.0
            else:
                ok2 = serial_link.send_line("STOP")

            with state_lock:
                state.serial_ok = ok1 and ok2
                if not state.serial_ok:
                    state.running = False
                    state.throttle_us = ESC_NEUTRAL_US
                    state.armed = False
                    state.last_error = "serial_send_failed"

            last_send = now

        if show_debug:
            cv2.imshow("Track Mask", mask)
            cv2.imshow("Track Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


@app.route("/")
def index():
    return send_from_directory("web", "index.html")


def _start_logic():
    with state_lock:
        if not state.armed:
            return False
        state.running = True
        state.pacing_complete = False
        state.measured_distance_m = 0.0
        state.measured_speed_mps = 0.0
        state.encoder_count = 0
        state.last_error = ""
        state.expected_elapsed_s = 0.0
        state.actual_elapsed_s = 0.0
        state.schedule_error_s = 0.0
        state.finish_time_error_s = 0.0
        distance_m = state.target_distance_m
        time_s = state.target_time_s
        target_speed_mps = state.target_speed_mps
        target_rotations = state.target_rotations
    reset_pacing_controller()
    ok = serial_link.send_line(f"JETSON:{distance_m:.2f},{time_s:.2f},{target_speed_mps:.3f},{target_rotations:.2f}")
    with state_lock:
        state.serial_ok = ok
        if not ok:
            state.running = False
            state.last_error = "workout_send_failed"
    return ok


def _stop_logic(reason="user_stop"):
    set_safe_stop(reason)


def _set_workout_logic(distance_m: float, time_s: float):
    target_speed_mps = calculate_target_speed(distance_m, time_s)
    target_rotations = rotations_for_distance(distance_m)
    with state_lock:
        state.target_distance_m = distance_m
        state.target_time_s = time_s
        state.target_speed_mps = target_speed_mps
        state.target_rotations = target_rotations
        state.pacing_complete = False
        state.last_error = ""
    ok = serial_link.send_line(f"WORKOUTCFG:{distance_m:.2f},{time_s:.2f},{target_speed_mps:.3f},{target_rotations:.2f}")
    with state_lock:
        state.serial_ok = ok
    return ok


@app.post("/api/arm")
@app.post("/arm")
def api_arm():
    arm_system()
    return jsonify(current_status())


@app.post("/api/disarm")
@app.post("/disarm")
def api_disarm():
    disarm("user_disarm")
    return jsonify(current_status())


@app.post("/api/start")
@app.post("/start")
def api_start():
    started = _start_logic()
    if not started:
        with state_lock:
            state.last_error = "not_armed_or_workout_failed"
    return jsonify(current_status())


@app.post("/api/stop")
@app.post("/stop")
def api_stop():
    _stop_logic("user_stop")
    return jsonify(current_status())


@app.post("/api/workout")
def api_workout():
    body = request.get_json(force=True, silent=True) or {}
    try:
        distance_m = float(body.get("distance_m", state.target_distance_m))
        time_s = float(body.get("time_s", state.target_time_s))
        _set_workout_logic(distance_m, time_s)
    except (TypeError, ValueError) as exc:
        with state_lock:
            state.last_error = str(exc)
        return jsonify(current_status()), 400
    return jsonify(current_status())


@app.post("/api/manual_steer")
def api_manual_steer():
    body = request.get_json(force=True, silent=True) or {}
    mode = body.get("mode", "vision")
    if mode not in {"vision", "left", "center", "right"}:
        mode = "vision"
    with state_lock:
        state.manual_steer_mode = mode
    return jsonify(current_status())


@app.get("/api/status")
@app.get("/status")
def api_status():
    return jsonify(current_status())


def current_status() -> dict:
    with state_lock:
        return {
            "armed": state.armed,
            "running": state.running,
            "steering_angle": state.steering_angle,
            "throttle_us": state.throttle_us,
            "vision_ok": state.vision_ok,
            "serial_ok": state.serial_ok,
            "manual_steer_mode": state.manual_steer_mode,
            "last_error": state.last_error,
            "target_distance_m": state.target_distance_m,
            "target_time_s": state.target_time_s,
            "target_speed_mps": state.target_speed_mps,
            "target_rotations": state.target_rotations,
            "encoder_count": state.encoder_count,
            "measured_distance_m": state.measured_distance_m,
            "measured_speed_mps": state.measured_speed_mps,
            "pacing_complete": state.pacing_complete,
            "expected_elapsed_s": state.expected_elapsed_s,
            "actual_elapsed_s": state.actual_elapsed_s,
            "schedule_error_s": state.schedule_error_s,
            "motor_power_percent": state.motor_power_percent,
            "finish_time_error_s": state.finish_time_error_s,
            "wheel_diameter_inches": WHEEL_DIAMETER_INCHES,
            "wheel_circumference_m": WHEEL_CIRCUMFERENCE_M,
            "meters_per_rotation": METERS_PER_ROTATION,
            "distance_presets_m": DISTANCE_PRESETS_M,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--debug-view", action="store_true", help="Show OpenCV debug windows")
    args = parser.parse_args()

    load_pace_model()
    sync_workout_metrics()
    disarm("startup_default")

    serial_reader = threading.Thread(target=serial_reader_loop, daemon=True)
    serial_reader.start()

    worker = threading.Thread(target=vision_and_control_loop, args=(args.debug_view,), daemon=True)
    worker.start()

    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        disarm("shutdown")
        serial_link.close()
