"""UART bridge to Arduino over USB serial."""

from __future__ import annotations


class UartBridge:
    def __init__(self, port: str, baud: int = 115200) -> None:
        import serial

        # write_timeout bounds how long a write can block: without it, a
        # stalled/unresponsive Arduino leaves the whole vision+safety loop
        # frozen indefinitely inside serial.write()'s select() call, since
        # nothing is ever draining the port on the other end. The Arduino's
        # own 300ms heartbeat timeout is the real safety net if commands
        # stop getting through -- this just keeps the Pi side from hanging.
        self._serial = serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=0.2)
        print(f"UART open on {port} @ {baud} baud")

    def send(self, payload: bytes) -> None:
        try:
            self._serial.write(payload)
        except Exception as exc:
            print(f"UART write failed ({exc}); Arduino should self-stop via heartbeat timeout")

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
