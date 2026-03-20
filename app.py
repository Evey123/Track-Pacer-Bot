import argparse
import math
import threading
import time
from dataclasses import dataclass

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
SUPPORTED_DISTANCES_M = [100, 200, 400, 800, 1600]
WHEEL_DIAMETER_INCHES = 4.2
WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_INCHES * 0.0254 * math.pi
METERS_PER_ROTATION = (2.0 * math.pi * (WHEEL_DIAMETER_INCHES / 2.0)) * 0.0254
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
    target_distance_m: int = 100
    target_time_s: float = 25.0
    target_speed_mps: float = 4.0
    encoder_count: int = 0
    measured_distance_m: float = 0.0
    measured_speed_mps: float = 0.0
    pacing_complete: bool = False


state = RobotState()
state_lock = threading.Lock()


def distance_for_counts(counts: int) -> float:
    return counts * METERS_PER_ROTATION


def calculate_target_speed(distance_m: float, time_s: float) -> float:
    if time_s <= 0:
        raise ValueError("time_must_be_positive")
    return distance_m / time_s


def sync_workout_metrics():
    with state_lock:
        state.target_speed_mps = calculate_target_speed(state.target_distance_m, state.target_time_s)


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

        with state_lock:
            if "count" in payload:
                state.encoder_count = int(payload["count"])
                state.measured_distance_m = distance_for_counts(state.encoder_count)
            if "meters" in payload:
                state.measured_distance_m = float(payload["meters"])
            if "mps" in payload:
                state.measured_speed_mps = float(payload["mps"])
            if payload.get("done") == "1":
                state.running = False
                state.throttle_us = ESC_NEUTRAL_US
                state.pacing_complete = True
                state.last_error = "workout_complete"


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

        now = time.time()
        if now - last_send >= COMMAND_PERIOD_S:
            ok1 = serial_link.send_line(f"STEER:{steering}")
            ok2 = True
            if not running:
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
        distance_m = state.target_distance_m
        time_s = state.target_time_s
        target_speed_mps = state.target_speed_mps
    ok = serial_link.send_line(f"WORKOUT:{distance_m:.2f},{time_s:.2f},{target_speed_mps:.3f}")
    with state_lock:
        state.serial_ok = ok
        if not ok:
            state.running = False
            state.last_error = "workout_send_failed"
    return ok


def _stop_logic(reason="user_stop"):
    set_safe_stop(reason)


def _set_workout_logic(distance_m: int, time_s: float):
    if distance_m not in SUPPORTED_DISTANCES_M:
        raise ValueError("unsupported_distance")
    target_speed_mps = calculate_target_speed(distance_m, time_s)
    with state_lock:
        state.target_distance_m = distance_m
        state.target_time_s = time_s
        state.target_speed_mps = target_speed_mps
        state.pacing_complete = False
        state.last_error = ""
    ok = serial_link.send_line(f"WORKOUTCFG:{distance_m:.2f},{time_s:.2f},{target_speed_mps:.3f}")
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
        distance_m = int(body.get("distance_m", state.target_distance_m))
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
            "encoder_count": state.encoder_count,
            "measured_distance_m": state.measured_distance_m,
            "measured_speed_mps": state.measured_speed_mps,
            "pacing_complete": state.pacing_complete,
            "wheel_diameter_inches": WHEEL_DIAMETER_INCHES,
            "wheel_circumference_m": WHEEL_CIRCUMFERENCE_M,
            "meters_per_rotation": METERS_PER_ROTATION,
            "supported_distances_m": SUPPORTED_DISTANCES_M,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--debug-view", action="store_true", help="Show OpenCV debug windows")
    args = parser.parse_args()

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
