"""In-process ESP32 UART receiver and policy tick loop."""

from __future__ import annotations

from datetime import datetime

from shared.config import AppConfig
from shared.protocol import DetectionMessage
from shared.safety import Esp32PolicyController, Esp32PolicyState


class Esp32Simulator:
    """Mirrors ESP32 behaviour without hardware serial."""

    def __init__(self, controller: Esp32PolicyController, config: AppConfig) -> None:
        self.controller = controller
        self.config = config
        self.state: Esp32PolicyState = controller.state
        self._last_msg: DetectionMessage | None = None

    def receive(self, message: DetectionMessage) -> None:
        self._last_msg = message
        self.state = self.controller.on_message(
            message.zone,
            now=datetime.now(),
            heartbeat=message.heartbeat,
        )

    def tick(self) -> Esp32PolicyState:
        self.state = self.controller.check_heartbeat_timeout(datetime.now())
        return self.state
