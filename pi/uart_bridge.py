"""UART bridge to Arduino over USB serial."""

from __future__ import annotations

import time


class UartBridge:
    def __init__(self, port: str, baud: int = 115200) -> None:
        import serial

        # DIAGNOSTIC: write_timeout temporarily raised to 3s (from 0.2s) and
        # every send() logs elapsed time, to determine whether writes are
        # genuinely stalling forever or just taking longer than 0.2s to
        # complete on this board.
        self._serial = serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=3.0)
        print(f"UART open on {port} @ {baud} baud")

    def send(self, payload: bytes) -> None:
        start = time.perf_counter()
        try:
            self._serial.write(payload)
            elapsed = time.perf_counter() - start
            print(f"UART write OK, {len(payload)} bytes in {elapsed*1000:.0f}ms")
        except Exception as exc:
            elapsed = time.perf_counter() - start
            print(f"UART write failed after {elapsed*1000:.0f}ms ({exc})")

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
