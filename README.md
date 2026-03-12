# Track Pacer Bot Control System

Complete integrated system with:
- Web UI (phone/laptop) for **ARM / START / STOP / speed**
- Jetson Python backend (web server + OpenCV vision + serial control)
- Teensy firmware (servo + ESC control with safety timeout)

## Folder structure

```text
Track-Pacer-Bot/
├── app.py
├── requirements.txt
├── web/
│   ├── index.html
│   └── static/
│       ├── style.css
│       └── script.js
└── teensy/
    └── teensy_control.ino
```

## Serial protocol (Jetson -> Teensy)

ASCII lines ending with `\n`:
- `ARM`
- `DISARM`
- `STOP`
- `STEER:<angle>` (0..180)
- `THROTTLE:<microseconds>` (typically 1500..1700)

Example:

```text
ARM
STEER:92
THROTTLE:1570
STOP
DISARM
```

## How website + backend run together

You run **one command only** (`python app.py`).

- Backend API and frontend are served from the same Flask process.
- Frontend URL: `http://<jetson-ip>:5000/`
- API URLs: `/api/*` and compatibility aliases `/start`, `/stop`, `/set_speed`, `/status`

## Jetson setup instructions

1. Connect Teensy by USB.
2. Install system dependencies if needed:
   - `sudo apt update`
   - `sudo apt install -y python3-pip python3-venv`
3. In project folder:
   - `python3 -m venv .venv`
   - `source .venv/bin/activate`
   - `pip install -r requirements.txt`
4. Find serial port:
   - `ls /dev/ttyACM* /dev/ttyUSB*`
5. Edit constants in `app.py` for your hardware (port, camera index, tuning).

## Run instructions (Jetson)

```bash
cd /workspace/Track-Pacer-Bot
source .venv/bin/activate
python app.py --host 0.0.0.0 --port 5000
```

Optional OpenCV debug windows:

```bash
python app.py --host 0.0.0.0 --port 5000 --debug-view
```

From phone/laptop on same hotspot/LAN, open:

```text
http://<jetson-local-ip>:5000
```

## Operation sequence (important)

1. Press **ARM** in web UI.
2. Press **START** in web UI.
3. Adjust speed slider.
4. Press **STOP** anytime for immediate neutral throttle.
5. Press **DISARM** when done.

If you press START before ARM, backend returns state with `last_error: "not_armed"`.

## API quick reference

- `POST /api/arm` (or `/arm`)
- `POST /api/disarm` (or `/disarm`)
- `POST /api/start` (or `/start`)
- `POST /api/stop` (or `/stop`)
- `POST /api/set_speed` (or `/set_speed`) body: `{"speed": 0..100}`
- `GET /api/status` (or `/status`)

## Teensy upload instructions

1. Install Arduino IDE + Teensyduino.
2. Open `teensy/teensy_control.ino`.
3. Select Teensy board + USB port in **Tools**.
4. **Important:** close any program using the Teensy serial port before upload (stop `python app.py`, close Serial Monitor/Plotter).
5. Click **Upload**.
6. If upload fails after successful compile, press the physical **PROGRAM** button on Teensy once during upload.
7. Keep ESC powered and allow neutral boot window (`ARMING_NEUTRAL_MS`, default 2s).
8. Verify Teensy appears on Jetson as `/dev/ttyACM0` (or update `SERIAL_PORT` in `app.py`).

### Upload error troubleshooting (compile succeeds, upload fails)

If you see memory usage and then only **"An error occurred while uploading the sketch"**, the sketch compiled but the PC/Jetson could not switch Teensy into loader mode.

Try in this exact order:

1. Stop backend so serial is free:
   - `pkill -f "python app.py"`
2. Unplug/replug Teensy USB cable (data cable, not charge-only).
3. In Arduino IDE set:
   - **Tools > Board:** `Teensy 4.1`
   - **Tools > USB Type:** `Serial`
   - **Tools > Port:** select Teensy port
4. Click Upload and press Teensy **PROGRAM** button once if it does not auto-load.
5. Enable **File > Preferences > Show verbose output during upload** and retry.
6. On Linux, verify device access:
   - `ls -l /dev/ttyACM* /dev/hidraw*`
   - If permission denied, add your user to `dialout` and re-login:
     - `sudo usermod -aG dialout $USER`
7. If still failing, reboot Jetson and try upload before starting any Python process.


## Data flow: button press to motor movement

1. Tap **ARM** -> web sends `POST /api/arm`.
2. Backend sends `STOP` then `ARM` over serial.
3. Tap **START** -> backend sets `running=true`.
4. Vision computes steering and backend sends `STEER:<angle>` continuously.
5. Backend maps slider speed to throttle and sends `THROTTLE:<us>` continuously.
6. Teensy applies servo steering + ESC throttle (only when armed).

## Tuning values you will likely adjust

In `app.py`:
- `SERIAL_PORT`, `BAUD_RATE`, `CAMERA_INDEX`
- `STEER_CENTER`, `STEER_RANGE`, `STEER_ALPHA`
- `ESC_NEUTRAL_US`, `ESC_FORWARD_MIN_US`, `ESC_FORWARD_MAX_US`
- `LOWER_YELLOW`, `UPPER_YELLOW`
- `MIN_CONTOUR_AREA`, `ROI_TOP_RATIO`

In `teensy/teensy_control.ino`:
- `STEERING_PIN`, `THROTTLE_PIN`
- `STEER_CENTER_DEG`, `STEER_MIN_DEG`, `STEER_MAX_DEG`
- `ESC_NEUTRAL_US`, `ESC_MIN_US`, `ESC_MAX_US`
- `COMMAND_TIMEOUT_MS`, `ARMING_NEUTRAL_MS`

## Things to edit for my hardware

- **serial port name:** `SERIAL_PORT` in `app.py` (ex `/dev/ttyACM0`)
- **baud rate:** `BAUD_RATE` in `app.py` and Teensy code (default `115200`)
- **steering pin:** `STEERING_PIN` in Teensy code (default `5`)
- **throttle pin:** `THROTTLE_PIN` in Teensy code (default `6`)
- **servo center value:** `STEER_CENTER` (`app.py`) and `STEER_CENTER_DEG` (Teensy)
- **ESC neutral / stop value:** `ESC_NEUTRAL_US` in both files (default `1500`)
- **camera index or camera pipeline:** `CAMERA_INDEX` in `app.py`
- **color threshold values for vision:** `LOWER_YELLOW` / `UPPER_YELLOW` in `app.py`
