"""Gamepad input for manual drive control (tank drive, forward only).

Left stick's vertical axis -> left motor speed, right stick's vertical axis
-> right motor speed. Pulling a stick back (or releasing it) just means no
throttle on that side -- there is no reverse and no brake command; only the
camera's DANGER zone can actually stop the vehicle (see arduino_uno.ino's
header comment for why).

Run this file directly (`python gamepad.py`) to print live axis values --
useful for confirming LEFT_STICK_Y_AXIS/RIGHT_STICK_Y_AXIS below actually
match your controller, since SDL's axis numbering varies between pads and
isn't something to assume blindly.
"""

from __future__ import annotations

DEADZONE = 0.15

# Standard Xbox-style mapping under SDL2 on Linux. Confirm with
# `python gamepad.py` before trusting this for your specific controller.
LEFT_STICK_Y_AXIS = 1
RIGHT_STICK_Y_AXIS = 3


class GamepadController:
    def __init__(self) -> None:
        import pygame

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError(
                "No gamepad detected. Plug it in before starting the script."
            )

        self._pygame = pygame
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()
        print(f"Gamepad connected: {self._joystick.get_name()}")

    def read_speeds(self) -> tuple[int, int]:
        """Return (left_speed, right_speed), each 0-255, forward only."""
        self._pygame.event.pump()

        # SDL reports stick-up as negative; flip so "forward" is positive.
        left_raw = -self._joystick.get_axis(LEFT_STICK_Y_AXIS)
        right_raw = -self._joystick.get_axis(RIGHT_STICK_Y_AXIS)

        return self._axis_to_speed(left_raw), self._axis_to_speed(right_raw)

    @staticmethod
    def _axis_to_speed(value: float) -> int:
        if value < DEADZONE:
            return 0
        return int(min(value, 1.0) * 255)

    def close(self) -> None:
        self._pygame.joystick.quit()
        self._pygame.quit()


def _calibrate() -> None:
    import time

    import pygame

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No gamepad detected. Plug it in and try again.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}  Buttons: {joystick.get_numbuttons()}")
    print("Move each stick and watch which index changes. Ctrl+C to quit.\n")

    try:
        while True:
            pygame.event.pump()
            values = [f"{i}:{joystick.get_axis(i):+.2f}" for i in range(joystick.get_numaxes())]
            print("\r" + "  ".join(values), end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    _calibrate()
