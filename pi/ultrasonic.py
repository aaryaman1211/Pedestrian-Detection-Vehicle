"""
HC-SR04 ultrasonic sensor reader.

Wiring (BCM):
  TRIG -> GPIO 23
  ECHO -> GPIO 24 (via voltage divider on 5 V echo)
"""

from __future__ import annotations

import time


class UltrasonicSensor:
    TRIG_PIN = 23
    ECHO_PIN = 24

    def __init__(self) -> None:
        try:
            import RPi.GPIO as GPIO

            self._gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.TRIG_PIN, GPIO.OUT)
            GPIO.setup(self.ECHO_PIN, GPIO.IN)
            self._available = True
            print("HC-SR04 ultrasonic sensor ready")
        except Exception as exc:
            self._available = False
            print(f"Ultrasonic unavailable ({exc}); continuing without it")

    def read_distance_m(self) -> float | None:
        if not self._available:
            return None

        GPIO = self._gpio
        GPIO.output(self.TRIG_PIN, False)
        time.sleep(0.0002)
        GPIO.output(self.TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG_PIN, False)

        timeout = time.time() + 0.04
        pulse_start = pulse_end = None

        while GPIO.input(self.ECHO_PIN) == 0:
            if time.time() > timeout:
                return None
            pulse_start = time.time()

        while GPIO.input(self.ECHO_PIN) == 1:
            if time.time() > timeout:
                return None
            pulse_end = time.time()

        if pulse_start is None or pulse_end is None:
            return None

        duration = pulse_end - pulse_start
        distance_cm = (duration * 34300) / 2.0
        return distance_cm / 100.0

    def cleanup(self) -> None:
        if self._available:
            self._gpio.cleanup()
