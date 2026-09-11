#!/usr/bin/env python3
"""
Mac simulator: webcam feed, YOLO detection, safety zones, and in-process
ESP32 policy simulation with a desktop dashboard.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from shared.config import AppConfig, SafetyZone
from shared.detector import PedestrianDetector
from shared.distance import DistanceEstimator
from shared.protocol import DetectionMessage
from shared.safety import SafetyController, Esp32PolicyController

from ui import DashboardApp
from esp32_simulator import Esp32Simulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pedestrian detection vehicle — Mac simulator")
    parser.add_argument("--camera", type=int, default=0, help="Webcam index")
    parser.add_argument("--video", type=str, default="", help="Use a video file instead of webcam")
    parser.add_argument("--mock-ultrasonic", type=float, default=None, help="Fixed ultrasonic reading (m)")
    return parser.parse_args()


class PerceptionLoop:
    """Runs detection in a background thread and pushes frames to the UI."""

    def __init__(
        self,
        config: AppConfig,
        camera_index: int = 0,
        video_path: str = "",
        mock_ultrasonic: float | None = None,
    ) -> None:
        self.config = config
        self.camera_index = camera_index
        self.video_path = video_path
        self.mock_ultrasonic = mock_ultrasonic

        self.detector = PedestrianDetector(config)
        self.distance_estimator = DistanceEstimator(config)
        self.safety = SafetyController(config)
        self.esp32 = Esp32PolicyController(config)
        self.simulator = Esp32Simulator(self.esp32, config)

        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._latest_frame: np.ndarray | None = None
        self._fps = 0.0
        self._status: dict = {"status_text": "Initializing…"}
        self._error: str | None = None
        self._cap: cv2.VideoCapture | None = None

    def prepare(self, ui_pump=None) -> None:
        """
        Open the camera on the main thread.

        macOS requires camera authorization on the main thread — opening
        VideoCapture from a background thread produces a blank feed. Model
        loading deliberately stays in the perception thread so the dashboard
        remains responsive while YOLO downloads or initializes.
        """
        if ui_pump:
            ui_pump()

        with self._lock:
            self._status["status_text"] = "Opening camera…"

        if self.video_path:
            self._cap = cv2.VideoCapture(self.video_path)
        else:
            self._cap = cv2.VideoCapture(self.camera_index, cv2.CAP_AVFOUNDATION)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        if not self._cap.isOpened():
            raise RuntimeError(
                "Could not open camera. Check System Settings → Privacy → Camera "
                "and allow Terminal (or Cursor) access."
            )

        ok, _ = self._cap.read()
        if not ok and not self.video_path:
            raise RuntimeError("Camera opened but returned no frames.")

        with self._lock:
            self._status["status_text"] = "Camera ready — starting detector…"
        if ui_pump:
            ui_pump()

    def start(self) -> None:
        if self._cap is None:
            raise RuntimeError("Call prepare() on the main thread before start()")
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_frame(self) -> tuple[np.ndarray | None, dict, str | None]:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            status = dict(self._status)
            error = self._error
        return frame, status, error

    def _run(self) -> None:
        cap = self._cap
        frame_interval = 1.0 / self.config.target_fps
        last_heartbeat = 0.0

        try:
            with self._lock:
                self._status["status_text"] = "Loading YOLO model (first run may download ~6 MB)…"
            self.detector._load_model()

            while self._running:
                loop_start = time.perf_counter()
                ok, frame = cap.read()
                if not ok:
                    if self.video_path:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    time.sleep(0.05)
                    continue

                detection = self.detector.detect(frame)
                distance_m = None
                bbox_height = 0.0
                confidence = 0.0

                if detection is not None:
                    reading = self.distance_estimator.estimate(detection.bbox_height_px)
                    if reading is not None:
                        distance_m = reading.distance_m
                        bbox_height = reading.bbox_height_px
                    confidence = detection.confidence

                ultrasonic_m = self.mock_ultrasonic
                state = self.safety.update(distance_m, confidence, bbox_height, ultrasonic_m)

                now = time.time()
                if (now - last_heartbeat) >= self.config.heartbeat_interval_ms / 1000.0:
                    msg = DetectionMessage(
                        zone=state.zone,
                        distance_m=distance_m or 0.0,
                        confidence=confidence,
                        bbox_height_px=bbox_height,
                        heartbeat=True,
                    )
                    self.simulator.receive(msg)
                    last_heartbeat = now

                self.simulator.tick()
                esp32_state = self.simulator.state

                elapsed = time.perf_counter() - loop_start
                fps = 1.0 / max(elapsed, 1e-6)

                annotated = self._annotate(frame, detection, state.zone, distance_m, fps, esp32_state)

                with self._lock:
                    self._latest_frame = annotated
                    self._fps = fps
                    self._status = {
                        "status_text": "Running",
                        "zone": state.zone.value,
                        "distance_m": distance_m,
                        "confidence": confidence,
                        "fps": fps,
                        "person_detected": state.person_detected,
                        "debounced": state.debounced,
                        "esp32": esp32_state.as_text(),
                        "speed_factor": esp32_state.speed_factor,
                        "latched_stop": esp32_state.latched_stop,
                        "buzzer_on": esp32_state.buzzer_on,
                        "timestamp": datetime.now().strftime("%H:%M:%S"),
                    }

                sleep_time = frame_interval - (time.perf_counter() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            traceback.print_exc()
        finally:
            if cap is not None:
                cap.release()

    def _annotate(self, frame, detection, zone, distance_m, fps, esp32_state):
        color = self.config.zone_colors[zone]
        output = self.detector.draw_detection(frame, detection, zone.value, distance_m)

        overlay = output.copy()
        h, w = output.shape[:2]
        cv2.rectangle(overlay, (0, 0), (w, 56), (18, 18, 24), -1)
        cv2.addWeighted(overlay, 0.7, output, 0.3, 0, output)

        dist = f"{distance_m:.2f} m" if distance_m is not None else "no target"
        cv2.putText(
            output,
            f"{zone.value}  |  {dist}  |  {fps:.1f} FPS",
            (14, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )

        bar_y = h - 28
        cv2.rectangle(output, (14, bar_y), (w - 14, bar_y + 10), (40, 40, 40), -1)
        speed_w = int((w - 28) * esp32_state.speed_factor)
        bar_color = (80, 80, 255) if esp32_state.latched_stop else color
        cv2.rectangle(output, (14, bar_y), (14 + speed_w, bar_y + 10), bar_color, -1)

        return output


def main() -> None:
    args = parse_args()
    config = AppConfig()
    loop = PerceptionLoop(
        config,
        camera_index=args.camera,
        video_path=args.video,
        mock_ultrasonic=args.mock_ultrasonic,
    )

    app = DashboardApp(loop, config)

    def boot() -> None:
        try:
            loop.prepare(ui_pump=lambda: app.pump())
            loop.start()
        except Exception as exc:
            app.show_error(str(exc))

    app.show()
    try:
        boot()
        app.run()
    finally:
        loop.stop()
        app.close()


if __name__ == "__main__":
    main()
