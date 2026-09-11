# Project 7: Pedestrian Detection Vehicle

YOLO-based pedestrian detection with monocular distance estimation, 3-zone safety policy, and ESP32 motor control. Two runtimes share the same core logic:

| Target | Folder | Purpose |
|--------|--------|---------|
| **Mac** | `mac/` | Webcam testing with a local browser dashboard and in-process ESP32 simulation |
| **Raspberry Pi** | `pi/` | Pi Camera inference + UART to real ESP32 + ultrasonic sensor |

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

# Vision-only test (no ESP32 / ultrasonic)
./run.sh --no-serial --no-ultrasonic

# Full deployment
./run.sh --port /dev/ttyUSB0 --log
```

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
  UART JSON @ 10 Hz  →  ESP32 FSM (motor + buzzer + latched e-stop)
       ↑
  HC-SR04 confirms DANGER zone (Pi only)
```

### Safety zones

| Zone | Distance | Vehicle behaviour |
|------|----------|-------------------|
| **FAR** | ≥ 2 m | Cruise (100% speed) |
| **CAUTION** | < 2 m | 40% speed + buzzer |
| **DANGER** | < 1 m | Latched emergency stop |

Resume after path clear for **2 s**. Heartbeat loss **> 300 ms** triggers stop.

## Project layout

```
shared/          Core detection, distance, safety, UART protocol
mac/             Mac simulator + local browser dashboard
pi/              Pi runtime (camera, UART, ultrasonic)
esp32/           PlatformIO firmware stub for ESP32
calibration/     Saved distance-k values (generated)
logs/            Trial CSV logs from Pi runs
```

## ESP32 firmware

Flash with [PlatformIO](https://platformio.org/):

```bash
cd esp32
pio run -t upload
```

UART: **115200 baud**, newline-delimited JSON from Pi.

## Acceptance criteria mapping

| Criterion | Where implemented |
|-----------|-------------------|
| ≥ 5 FPS @ 320×320 | `shared/config.py` → `target_fps`, `input_size` |
| Person class filter | `shared/detector.py` |
| Zone debounce (3 frames) | `shared/safety.py` → `SafetyController` |
| Latched stop + 2 s resume | `shared/safety.py` → `Esp32PolicyController` + `esp32/` |
| Heartbeat failsafe | 100 ms heartbeat, 300 ms timeout |
| Trial logging | `pi/main.py --log` |

## Tuning

Edit `shared/config.py`:

- `distance_k` — monocular calibration constant
- `caution_distance_m` / `danger_distance_m` — zone thresholds
- `debounce_frames` — consecutive frames before zone change
- `caution_speed_factor` — ESP32 speed in CAUTION (default 40%)

## License note

Ultralytics YOLOv8 is **AGPL-3.0**. For coursework this is fine; check license terms if you deploy commercially.
