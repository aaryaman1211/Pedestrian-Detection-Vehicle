# Project 7: Pedestrian Detection Vehicle

YOLO-based pedestrian detection with monocular distance estimation, a 3-zone safety policy, and Arduino Uno motor control. Two runtimes share the same core logic:

| Target | Folder | Purpose |
|--------|--------|---------|
| **Mac** | `mac/` | Webcam testing with a local browser dashboard and in-process motor-policy simulation |
| **Raspberry Pi** | `pi/` | Pi Camera inference + UART to a real Arduino Uno + ultrasonic sensor |

## Current status

Verified working end-to-end on real hardware (Raspberry Pi + Pi Camera v1 + Arduino Uno, no motors/ultrasonic sensor wired yet):

- Camera → YOLOv8n detection → distance estimate → 3-zone safety classification, running live on the Pi.
- Pi ↔ Arduino UART link: stable, self-recovering if the connection ever drops.
- Arduino firmware: correctly parses zone commands, tracks the heartbeat, latches on emergency stop, and resumes after a clear 2-second window — confirmed by comparing its own `STATUS`/`EMERGENCY_STOP`/`RESUMED` serial output against what the Pi detected.

**Not done yet** (hardware, not software): the L298N motor driver + motors, buzzer, LED, and the real HC-SR04 ultrasonic sensor are not physically wired. `pi/main.py` is run with `--no-ultrasonic` until the sensor is connected. `shared/config.py`'s `distance_k` is still the uncalibrated placeholder — run `pi/calibrate.py` once the camera setup is final.

**Important hardware note**: the actual board in this build is an **Arduino Uno** (a SunFounder Uno-compatible clone), not the Arduino Due or ESP32 the project name/older comments might suggest. The firmware sketch lives at `arduinodue/arduino_uno/arduino_uno.ino` — the `arduinodue/` folder name is a holdover from an earlier (incorrect) assumption about the hardware and does not reflect what's actually used. `arduinodue/Arduino_due.rtf` is the original, unused Due-targeted draft, kept only as historical reference.

**Known limitation**: the Arduino's heartbeat timeout is 2000ms (raised from a stricter 300ms) because `pi/main.py` only sends a heartbeat once per camera-frame loop iteration, and YOLO inference on this hardware runs at ~1-3 FPS — a 300ms budget fired on nearly every cycle in practice. This works reliably but isn't a true fix; the heartbeat send is still coupled to inference speed rather than an independent clock. Worth revisiting if inference gets faster or tighter safety margins are needed later.

**If testing without the ultrasonic sensor wired**: jumper the Arduino's ECHO pin (12) to GND. Left floating, it can read spurious noise as a close-range obstacle and falsely trigger `EMERGENCY_STOP,ULTRASONIC_DANGER`. Remove the jumper once the real sensor is connected there.

## Quick start (Mac)

```bash
cd mac
chmod +x setup.sh run.sh
./setup.sh
./run.sh
```

Options:

```bash
python main.py --camera 0              # default webcam
python main.py --video path/to/clip.mp4
python main.py --mock-ultrasonic 0.8   # force DANGER zone via ultrasonic
```

## Quick start (Raspberry Pi)

On Raspberry Pi OS (64-bit, Python 3.10+):

```bash
# System packages (once)
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-venv

cd pi
chmod +x setup.sh run.sh
./setup.sh

# If installing torch/ultralytics pulls in multi-GB NVIDIA CUDA packages
# and fails with "No space left on device", install the CPU-only build
# first, then the rest of requirements.txt:
#   pip install torch --index-url https://download.pytorch.org/whl/cpu
#   pip install -r requirements.txt

# Vision-only test (no Arduino / ultrasonic)
./run.sh --no-serial --no-ultrasonic

# Full deployment (find the actual port with `ls /dev/ttyACM*`)
./run.sh --port /dev/ttyACM0 --log
```

If the OpenCV preview window fails to open under a Wayland desktop session, run with `QT_QPA_PLATFORM=xcb` set.

Calibrate distance constant `k` (used in `d = k / h_bbox`):

```bash
python calibrate.py --distance 1.5 --samples 10
```

## Architecture

```
Pi Camera / Webcam
       ↓
  YOLOv8n (person class, 320×320)
       ↓
  DistanceEstimator  (d = k / h_bbox)
       ↓
  SafetyController   (FAR / CAUTION / DANGER + 3-frame debounce)
       ↓
  UART text protocol (HB / ZONE,.. / CLEAR)  →  Arduino Uno FSM (motor + buzzer + latched e-stop)
       ↑
  HC-SR04 confirms DANGER zone (Pi only)
```

### Safety zones

| Zone | Distance | Vehicle behaviour |
|------|----------|-------------------|
| **FAR** | ≥ 2 m | Cruise (100% speed) |
| **CAUTION** | < 2 m | 40% speed + buzzer |
| **DANGER** | < 1 m | Latched emergency stop |

Resume after path clear for **2 s**. Heartbeat loss **> 2000 ms** triggers stop (see "Known limitation" above for why this isn't the original 300ms).

## Project layout

```
shared/          Core detection, distance, safety, UART protocol
mac/             Mac simulator + local browser dashboard
pi/              Pi runtime (camera, UART, ultrasonic)
arduinodue/      Arduino firmware
  arduino_uno/     The actual sketch in use (targets the real hardware: an Uno)
  Arduino_due.rtf  Original draft targeting a Due; unused, kept for reference only
calibration/     Saved distance-k values (generated)
logs/            Trial CSV logs from Pi runs
```

## Arduino firmware

Flash with the [Arduino IDE](https://www.arduino.cc/en/software):

1. **Tools → Board → Arduino AVR Boards → Arduino Uno**
2. **Tools → Port** → select the Arduino's port
3. Open `arduinodue/arduino_uno/arduino_uno.ino` → **Upload**

UART: **115200 baud**, newline-delimited plain-text commands from the Pi (`HB`, `ZONE,<zone>,<distance_m>,<confidence>`, `CLEAR`, `RESET`) — see `shared/protocol.py`'s `encode_heartbeat`/`encode_zone_command`/`encode_clear`.

## Acceptance criteria mapping

| Criterion | Where implemented |
|-----------|-------------------|
| ≥ 5 FPS @ 320×320 | `shared/config.py` → `target_fps`, `input_size` (target only — real-world Pi CPU inference currently runs slower, ~1-3 FPS) |
| Person class filter | `shared/detector.py` |
| Zone debounce (3 frames) | `shared/safety.py` → `SafetyController` |
| Latched stop + 2 s resume | `shared/safety.py` → `Esp32PolicyController` (Mac simulator) + `arduinodue/arduino_uno/arduino_uno.ino` (real hardware) |
| Heartbeat failsafe | 100 ms heartbeat send interval, 2000 ms Arduino-side timeout |
| Trial logging | `pi/main.py --log` |

## Tuning

Edit `shared/config.py`:

- `distance_k` — monocular calibration constant
- `caution_distance_m` / `danger_distance_m` — zone thresholds
- `debounce_frames` — consecutive frames before zone change
- `caution_speed_factor` — motor speed in CAUTION (default 40%)
- `heartbeat_timeout_ms` — keep in sync with `HEARTBEAT_TIMEOUT_MS` in the Arduino sketch if you change either

## License note

Ultralytics YOLOv8 is **AGPL-3.0**. For coursework this is fine; check license terms if you deploy commercially.
