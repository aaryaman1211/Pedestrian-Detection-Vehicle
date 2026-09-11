#!/usr/bin/env python3
"""
Raspberry Pi main loop: camera capture, YOLO detection, distance estimation,
zone policy, and UART bridge to ESP32.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from shared.config import AppConfig, SafetyZone
from shared.detector import PedestrianDetector
from shared.distance import DistanceEstimator
from shared.protocol import DetectionMessage, encode_message
from shared.safety import SafetyController

from camera import Camera
from uart_bridge import UartBridge
from ultrasonic import UltrasonicSensor


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pedestrian detection vehicle — Pi runtime")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0, help="Camera index if not using Pi Camera")
    parser.add_argument("--no-serial", action="store_true", help="Run without UART (vision-only test)")
    parser.add_argument("--no-ultrasonic", action="store_true", help="Skip HC-SR04 reads")
    parser.add_argument("--log", action="store_true", help="Write trial CSV logs")
    return parser.parse_args()


def ensure_log_dir(config: AppConfig) -> Path:
    log_dir = ROOT / config.log_dir
    log_dir.mkdir(exist_ok=True)
    return log_dir


def draw_hud(
    frame: np.ndarray,
    config: AppConfig,
    zone: SafetyZone,
    fps: float,
    distance_m: float | None,
) -> np.ndarray:
    color = config.zone_colors[zone]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (frame.shape[1], 48), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    dist_text = f"{distance_m:.2f} m" if distance_m is not None else "no target"
    cv2.putText(
        frame,
        f"Zone: {zone.value}  |  {dist_text}  |  {fps:.1f} FPS",
        (12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )
    return frame


def main() -> None:
    args = parse_args()
    config = AppConfig()

    camera = Camera(index=args.camera)
    detector = PedestrianDetector(config)
    distance_estimator = DistanceEstimator(config)
    safety = SafetyController(config)
    uart = None if args.no_serial else UartBridge(args.port, args.baud)
    ultrasonic = None if args.no_ultrasonic else UltrasonicSensor()

    log_path = None
    log_file = None
    if args.log:
        log_dir = ensure_log_dir(config)
        log_path = log_dir / f"trial_{datetime.now():%Y%m%d_%H%M%S}.csv"
        log_file = open(log_path, "w", newline="")
        csv.writer(log_file).writerow(
            ["timestamp", "zone", "distance_m", "confidence", "bbox_height_px", "fps"]
        )

    print("Starting Pi pedestrian detection loop. Press 'q' to quit.")
    if log_path:
        print(f"Logging to {log_path}")

    frame_interval = 1.0 / config.target_fps
    last_heartbeat = 0.0
    fps = 0.0

    try:
        with camera:
            while True:
                loop_start = time.perf_counter()
                frame = camera.read()
                if frame is None:
                    continue

                detection = detector.detect(frame)
                distance_m = None
                bbox_height = 0.0
                confidence = 0.0

                if detection is not None:
                    reading = distance_estimator.estimate(detection.bbox_height_px)
                    if reading is not None:
                        distance_m = reading.distance_m
                        bbox_height = reading.bbox_height_px
                    confidence = detection.confidence

                ultrasonic_m = ultrasonic.read_distance_m() if ultrasonic else None
                state = safety.update(distance_m, confidence, bbox_height, ultrasonic_m)

                now = time.time()
                if uart and (now - last_heartbeat) >= config.heartbeat_interval_ms / 1000.0:
                    msg = DetectionMessage(
                        zone=state.zone,
                        distance_m=distance_m or 0.0,
                        confidence=confidence,
                        bbox_height_px=bbox_height,
                        heartbeat=True,
                    )
                    uart.send(encode_message(msg))
                    last_heartbeat = now

                elapsed = time.perf_counter() - loop_start
                fps = 1.0 / max(elapsed, 1e-6)

                annotated = detector.draw_detection(
                    frame, detection, state.zone.value, distance_m
                )
                annotated = draw_hud(annotated, config, state.zone, fps, distance_m)
                cv2.imshow("Pedestrian Detection Vehicle (Pi)", annotated)

                if log_file:
                    csv.writer(log_file).writerow(
                        [
                            datetime.now().isoformat(timespec="milliseconds"),
                            state.zone.value,
                            f"{distance_m:.3f}" if distance_m else "",
                            f"{confidence:.3f}",
                            f"{bbox_height:.1f}",
                            f"{fps:.2f}",
                        ]
                    )

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                sleep_time = frame_interval - (time.perf_counter() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    finally:
        if log_file:
            log_file.close()
        if uart:
            uart.close()
        if ultrasonic:
            ultrasonic.cleanup()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
