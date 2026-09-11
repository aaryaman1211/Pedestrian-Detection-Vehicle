"""Camera capture for Raspberry Pi (Pi Camera v2 or USB fallback)."""

from __future__ import annotations

import cv2
import numpy as np


class Camera:
    """Try picamera2 first, fall back to OpenCV VideoCapture."""

    def __init__(self, index: int = 0, width: int = 640, height: int = 480) -> None:
        self.index = index
        self.width = width
        self.height = height
        self._picam = None
        self._cap = None

    def __enter__(self) -> "Camera":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        try:
            from picamera2 import Picamera2

            self._picam = Picamera2()
            config = self._picam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            self._picam.configure(config)
            self._picam.start()
            print("Using Pi Camera (picamera2)")
            return
        except Exception:
            self._picam = None

        self._cap = cv2.VideoCapture(self.index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.index}")
        print(f"Using OpenCV camera index {self.index}")

    def read(self) -> np.ndarray | None:
        if self._picam is not None:
            frame = self._picam.capture_array()
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        if self._picam is not None:
            self._picam.stop()
            self._picam = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
