"""YOLO-based pedestrian detector filtered to the person class."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from shared.config import AppConfig


@dataclass
class Detection:
    bbox_xyxy: tuple[int, int, int, int]  # x1, y1, x2, y2
    confidence: float
    bbox_height_px: float

    @property
    def center_x(self) -> float:
        x1, _, x2, _ = self.bbox_xyxy
        return (x1 + x2) / 2.0


class PedestrianDetector:
    """Wraps Ultralytics YOLOv8n and returns the best person detection."""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self._model = None

    def _load_model(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self.config.model_name)
        return self._model

    def detect(self, frame: np.ndarray) -> Detection | None:
        model = self._load_model()
        results = model.predict(
            source=frame,
            imgsz=self.config.input_size,
            conf=self.config.confidence_threshold,
            classes=[self.config.person_class_id],
            verbose=False,
        )

        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return None

        boxes = results[0].boxes
        best_idx = int(boxes.conf.argmax())
        xyxy = boxes.xyxy[best_idx].cpu().numpy().astype(int)
        conf = float(boxes.conf[best_idx].cpu().numpy())
        x1, y1, x2, y2 = map(int, xyxy)
        height = float(y2 - y1)

        return Detection(
            bbox_xyxy=(x1, y1, x2, y2),
            confidence=conf,
            bbox_height_px=height,
        )

    def draw_detection(
        self,
        frame: np.ndarray,
        detection: Detection | None,
        zone_label: str = "",
        distance_m: float | None = None,
    ) -> np.ndarray:
        import cv2

        output = frame.copy()
        if detection is None:
            return output

        x1, y1, x2, y2 = detection.bbox_xyxy
        color = (0, 255, 0)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)

        label_parts = [f"person {detection.confidence:.2f}"]
        if distance_m is not None:
            label_parts.append(f"{distance_m:.2f} m")
        if zone_label:
            label_parts.append(zone_label)

        label = " | ".join(label_parts)
        cv2.putText(
            output,
            label,
            (x1, max(y1 - 8, 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        return output
