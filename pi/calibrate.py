#!/usr/bin/env python3
"""Calibrate monocular distance constant k from known range samples."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from shared.config import AppConfig
from shared.detector import PedestrianDetector
from shared.distance import DistanceEstimator

from camera import Camera


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate distance k = d * h_bbox")
    parser.add_argument("--distance", type=float, required=True, help="Known distance in metres")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--samples", type=int, default=10)
    args = parser.parse_args()

    config = AppConfig()
    detector = PedestrianDetector(config)
    estimator = DistanceEstimator(config)
    heights: list[float] = []

    print(f"Place pedestrian figure at {args.distance:.2f} m. Capturing {args.samples} samples...")

    with Camera(index=args.camera) as cam:
        for i in range(args.samples):
            frame = cam.read()
            if frame is None:
                continue
            det = detector.detect(frame)
            if det:
                heights.append(det.bbox_height_px)
                print(f"  sample {i + 1}: bbox height = {det.bbox_height_px:.1f} px")

    if not heights:
        print("No detections captured. Check camera and target placement.")
        sys.exit(1)

    avg_height = sum(heights) / len(heights)
    k = estimator.calibrate(args.distance, avg_height)

    out = ROOT / "calibration" / "distance_k.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"distance_k": k, "samples": len(heights)}, indent=2))
    print(f"Calibrated k = {k:.1f}. Saved to {out}")


if __name__ == "__main__":
    main()
