"""UART message format between Raspberry Pi and ESP32."""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from typing import Any

from shared.config import SafetyZone


@dataclass
class DetectionMessage:
    zone: SafetyZone
    distance_m: float
    confidence: float
    bbox_height_px: float = 0.0
    heartbeat: bool = False

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["zone"] = self.zone.value
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "DetectionMessage":
        return cls(
            zone=SafetyZone(data["zone"]),
            distance_m=float(data["distance_m"]),
            confidence=float(data["confidence"]),
            bbox_height_px=float(data.get("bbox_height_px", 0.0)),
            heartbeat=bool(data.get("heartbeat", False)),
        )


def encode_message(message: DetectionMessage) -> bytes:
    """Single-line JSON terminated by newline for UART framing."""
    return (json.dumps(message.to_dict(), separators=(",", ":")) + "\n").encode("utf-8")


def parse_message(raw: bytes | str) -> DetectionMessage:
    text = raw.decode("utf-8") if isinstance(raw, bytes) else raw
    return DetectionMessage.from_dict(json.loads(text.strip()))


# --- Arduino Due firmware protocol ---------------------------------------
# The Due sketch (arduinodue/arduino_due/arduino_due.ino) speaks a plain-text
# line protocol, not the JSON format above. These helpers encode the lines
# it expects: HB (heartbeat), ZONE,<zone>,<distance_m>,<confidence>, and
# CLEAR (sent while in FAR to accumulate the 2 s latched-stop resume timer).


def encode_heartbeat() -> bytes:
    return b"HB\n"


def encode_clear() -> bytes:
    return b"CLEAR\n"


def encode_zone_command(message: DetectionMessage) -> bytes:
    return (
        f"ZONE,{message.zone.value},{message.distance_m:.2f},{message.confidence:.2f}\n"
    ).encode("utf-8")
