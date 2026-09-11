"""Zone-based safety policy with debouncing (Pi-side perception layer)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Deque

from shared.config import AppConfig, SafetyZone


@dataclass
class SafetyState:
    zone: SafetyZone = SafetyZone.FAR
    distance_m: float | None = None
    confidence: float = 0.0
    bbox_height_px: float = 0.0
    person_detected: bool = False
    debounced: bool = False


@dataclass
class SafetyController:
    """
    Applies 3-zone policy with frame debouncing on the Pi.

    ESP32 owns motor latching; this module decides the zone sent over UART.
    """

    config: AppConfig
    _history: Deque[SafetyZone] = field(default_factory=deque, init=False)
    _state: SafetyState = field(default_factory=SafetyState, init=False)

    def __post_init__(self) -> None:
        self._history = deque(maxlen=self.config.debounce_frames)

    def update(
        self,
        distance_m: float | None,
        confidence: float = 0.0,
        bbox_height_px: float = 0.0,
        ultrasonic_m: float | None = None,
    ) -> SafetyState:
        person_detected = distance_m is not None
        if distance_m is None:
            raw_zone = SafetyZone.FAR
        else:
            raw_zone = self.config.zone_for_distance(distance_m)

        # The ultrasonic sensor is an independent physical safety input.  It
        # must be able to stop the vehicle even when vision has no usable box.
        ultrasonic_danger = (
            ultrasonic_m is not None
            and ultrasonic_m < self.config.ultrasonic_danger_confirm_m
        )
        if ultrasonic_danger:
            raw_zone = SafetyZone.DANGER

        if ultrasonic_danger:
            # An ultrasonic obstacle is a direct failsafe signal, not merely
            # another vision frame. Fill the debounce window so the DANGER
            # command is sent on this update rather than several frames later.
            self._history.clear()
            self._history.extend([SafetyZone.DANGER] * self.config.debounce_frames)
        else:
            self._history.append(raw_zone)

        if len(self._history) == self.config.debounce_frames and len(
            set(self._history)
        ) == 1:
            debounced_zone = raw_zone
            debounced = True
        else:
            debounced_zone = self._state.zone
            debounced = False

        self._state = SafetyState(
            zone=debounced_zone,
            distance_m=distance_m,
            confidence=confidence,
            bbox_height_px=bbox_height_px,
            person_detected=person_detected,
            debounced=debounced,
        )
        return self._state


@dataclass
class Esp32PolicyState:
    """Motor policy state executed on ESP32 (also simulated on Mac)."""

    zone: SafetyZone = SafetyZone.FAR
    speed_factor: float = 0.0
    latched_stop: bool = False
    buzzer_on: bool = False
    heartbeat_received: bool = False
    last_heartbeat: datetime | None = None
    clear_since: datetime | None = None

    def as_text(self) -> str:
        if not self.heartbeat_received:
            mode = "WAITING FOR HEARTBEAT"
        elif self.latched_stop:
            mode = "EMERGENCY STOP (latched)"
        elif self.zone == SafetyZone.CAUTION:
            mode = f"CAUTION ({self.speed_factor:.0%} speed)"
        else:
            mode = "CRUISE"
        buzzer = "BUZZER ON" if self.buzzer_on else "buzzer off"
        return f"{mode} · {buzzer}"


class Esp32PolicyController:
    """
    ESP32 finite-state policy:
      FAR     -> cruise (100%)
      CAUTION -> 40% speed + buzzer
      DANGER  -> latched stop until path clear for 2 s
    Heartbeat loss > 300 ms -> latched stop.
    """

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.state = Esp32PolicyState()

    def on_message(
        self,
        zone: SafetyZone,
        now: datetime | None = None,
        heartbeat: bool = False,
    ) -> Esp32PolicyState:
        now = now or datetime.now()
        self.state.last_heartbeat = now
        self.state.heartbeat_received = True

        if zone == SafetyZone.DANGER:
            self.state.latched_stop = True
            self.state.clear_since = None

        if self.state.latched_stop:
            if zone == SafetyZone.FAR:
                if self.state.clear_since is None:
                    self.state.clear_since = now
                elif (now - self.state.clear_since).total_seconds() >= self.config.resume_clear_seconds:
                    self.state.latched_stop = False
                    self.state.clear_since = None
            else:
                self.state.clear_since = None

        self.state.zone = zone

        if self.state.latched_stop:
            self.state.speed_factor = 0.0
            self.state.buzzer_on = True
        elif zone == SafetyZone.CAUTION:
            self.state.speed_factor = self.config.caution_speed_factor
            self.state.buzzer_on = True
        else:
            self.state.speed_factor = 1.0
            self.state.buzzer_on = False

        return self.state

    def check_heartbeat_timeout(self, now: datetime | None = None) -> Esp32PolicyState:
        now = now or datetime.now()
        last = self.state.last_heartbeat
        if last is None:
            return self.state

        elapsed_ms = (now - last).total_seconds() * 1000.0
        if elapsed_ms > self.config.heartbeat_timeout_ms:
            self.state.latched_stop = True
            self.state.speed_factor = 0.0
            self.state.buzzer_on = True

        return self.state
