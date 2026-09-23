"""Configuration shared between Raspberry Pi and Mac simulator."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class SafetyZone(str, Enum):
    FAR = "FAR"
    CAUTION = "CAUTION"
    DANGER = "DANGER"


@dataclass
class AppConfig:
    """Tunable parameters for detection, distance, and safety policy."""

    # Vision
    model_name: str = "yolov8n.pt"
    input_size: int = 320
    confidence_threshold: float = 0.45
    person_class_id: int = 0
    target_fps: float = 5.0

    # Distance calibration: distance (m) = k / bbox_height (px)
    # Default k ≈ 1.7 m * typical bbox height at 1 m; recalibrate on hardware.
    distance_k: float = 850.0
    max_detection_distance_m: float = 4.0

    # Zone thresholds (metres)
    caution_distance_m: float = 2.0
    danger_distance_m: float = 1.0

    # Safety policy
    debounce_frames: int = 3
    resume_clear_seconds: float = 2.0
    heartbeat_interval_ms: int = 100
    # 2000ms (not the original 300ms) because the Pi's UART send only
    # happens once per camera-frame loop iteration, and YOLO inference on
    # this hardware runs at ~1-3 FPS (330ms-1000ms/frame) -- a 300ms
    # timeout fires on nearly every cycle, permanently re-latching the
    # emergency stop before the 2s resume window can ever complete. The
    # Arduino-side constant below must be kept in sync with this value.
    heartbeat_timeout_ms: int = 2000

    # ESP32 behaviour
    caution_speed_factor: float = 0.20  # keep in sync with CAUTION_SPEED in the Arduino sketch

    # Ultrasonic (Pi only; Mac simulator can mock values)
    ultrasonic_danger_confirm_m: float = 1.0

    # Paths
    log_dir: str = "logs"

    zone_colors: dict = field(
        default_factory=lambda: {
            SafetyZone.FAR: (46, 204, 113),      # green (BGR)
            SafetyZone.CAUTION: (0, 165, 255),   # orange
            SafetyZone.DANGER: (0, 0, 255),      # red
        }
    )

    def zone_for_distance(self, distance_m: float) -> SafetyZone:
        if distance_m < self.danger_distance_m:
            return SafetyZone.DANGER
        if distance_m < self.caution_distance_m:
            return SafetyZone.CAUTION
        return SafetyZone.FAR
