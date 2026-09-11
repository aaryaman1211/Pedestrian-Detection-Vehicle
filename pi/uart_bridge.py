"""UART bridge to Arduino over USB serial."""

from __future__ import annotations

import time


class UartBridge:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self._port = port
        self._baud = baud
        self._serial = None
        self._open()

    def _open(self) -> None:
        import serial

        # write_timeout keeps a stalled write from blocking the loop forever.
        # On this hardware, a write either completes near-instantly or the
        # connection has entered a stuck state that only clears by closing
        # and reopening the port (observed repeatedly: a fresh Serial object
        # on the same physical connection, Arduino never power-cycled,
        # reliably recovers it) -- so a short timeout plus reconnect-on-
        # failure below is the actual fix, not just a hang guard.
        self._serial = serial.Serial(self._port, baudrate=self._baud, timeout=0.1, write_timeout=0.3)
        print(f"UART open on {self._port} @ {self._baud} baud")

        # Opening the port toggles DTR, which resets the Arduino into its
        # bootloader and back -- it takes ~1-2s to reboot and start reading
        # serial again. The very first open has this covered incidentally
        # by YOLO model-load time before the first send(), but a mid-loop
        # reconnect has no such gap: without this, the next write (100ms
        # later) hits the board mid-reboot, fails, reconnects (resetting it
        # again), forever -- an infinite reset loop that never succeeds.
        time.sleep(2.0)

    def send(self, payload: bytes) -> None:
        try:
            self._serial.write(payload)
        except Exception as exc:
            print(f"UART write failed ({exc}); reopening port")
            self._reconnect()

    def _reconnect(self) -> None:
        try:
            self._serial.close()
        except Exception:
            pass
        try:
            self._open()
        except Exception as exc:
            print(f"UART reconnect failed ({exc}); Arduino should self-stop via heartbeat timeout")

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
