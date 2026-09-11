"""UART bridge to ESP32 over USB serial."""

from __future__ import annotations


class UartBridge:
    def __init__(self, port: str, baud: int = 115200) -> None:
        import serial

        self._serial = serial.Serial(port, baudrate=baud, timeout=0.1)
        print(f"UART open on {port} @ {baud} baud")

    def send(self, payload: bytes) -> None:
        self._serial.write(payload)

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
