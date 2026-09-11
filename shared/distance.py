"""Monocular distance estimation from bounding-box height."""

from __future__ import annotations

from dataclasses import dataclass

from shared.config import AppConfig


@dataclass
class DistanceReading:
    distance_m: float
    bbox_height_px: float
    source: str = "bbox"


class DistanceEstimator:
    """
    Estimate range using d = k / h_bbox.

    Calibrate `k` by measuring bbox height at known distances for each
    pedestrian figure size used in trials.
    """

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.k = config.distance_k

    def estimate(self, bbox_height_px: float) -> DistanceReading | None:
        if bbox_height_px <= 1.0:
            return None

        distance_m = self.k / bbox_height_px
        if distance_m > self.config.max_detection_distance_m:
            return None

        return DistanceReading(distance_m=distance_m, bbox_height_px=bbox_height_px)

    def calibrate(self, known_distance_m: float, bbox_height_px: float) -> float:
        """Update k from a calibration sample and return the new k."""
        if bbox_height_px <= 0:
            raise ValueError("bbox height must be positive")
        self.k = known_distance_m * bbox_height_px
        return self.k
