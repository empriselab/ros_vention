#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

from vention_arduino_control import VentionBase


COMMAND_HZ = 20.0
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

DEFAULT_MAX_TRANSLATION_SPEED = 500
DEFAULT_MAX_ROTATION_SPEED = 400
TURN_IN_PLACE_THRESHOLD = 0.20

# A direction is considered "held" as long as a press for it has arrived
# within the last DECAY_SECONDS. Terminals don't deliver key-up, so we
# simulate hold via repeated key-repeat presses + this timeout.
DECAY_SECONDS = 0.30


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def compute_wheel_speeds(
    x_axis: float,
    y_axis: float,
    max_translation: int,
    max_rotation: int,
) -> tuple[int, int]:
    v = -y_axis
    w = x_axis

    if abs(v) < TURN_IN_PLACE_THRESHOLD:
        v = 0.0

    speed_v = v * max_translation
    speed_w = w * max_rotation

    speed_a = int(clamp(speed_v - speed_w, -max_translation, max_translation))
    speed_b = int(clamp(speed_v + speed_w, -max_translation, max_translation))

    return speed_a, speed_b


def drain_stdin() -> list[str]:
    """Non-blocking read of all currently-available bytes from stdin."""
    chars: list[str] = []
    while select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        if not ch:
            break
        chars.append(ch)
    return chars


def main() -> None:
    parser = argparse.ArgumentParser(
        description="WASD terminal teleop for Vention base via Arduino bridge"
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Arduino baud rate")
    parser.add_argument(
        "--max_translation",
        type=int,
        default=DEFAULT_MAX_TRANSLATION_SPEED,
        help="Maximum forward/backward wheel speed",
    )
    parser.add_argument(
        "--max_rotation",
        type=int,
        default=DEFAULT_MAX_ROTATION_SPEED,
        help="Maximum turning contribution",
    )
    parser.add_argument(
        "--decay",
        type=float,
        default=DECAY_SECONDS,
        help="Seconds without a repeat keypress before a direction is released",
    )
    args = parser.parse_args()

    base = VentionBase(args.port, args.baud)
    if not base.bridge.connection_status:
        raise RuntimeError(f"Failed to connect to Arduino on {args.port}")

    print("WASD to drive, space=stop, q or ESC to quit. Hold a key to keep moving.")

    period = 1.0 / COMMAND_HZ
    last_sent: tuple[int, int] | None = None
    last_press: dict[str, float] = {}  # key -> monotonic timestamp

    fd = sys.stdin.fileno()
    old_termios = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        running = True
        while running:
            now = time.monotonic()

            for ch in drain_stdin():
                lower = ch.lower()
                if ch == "\x1b" or lower == "q":
                    running = False
                    break
                if ch == " ":
                    last_press.clear()
                    continue
                if lower in ("w", "a", "s", "d"):
                    last_press[lower] = now

            if not running:
                break

            x_axis = 0.0
            y_axis = 0.0
            if now - last_press.get("w", 0.0) < args.decay:
                y_axis -= 1.0
            if now - last_press.get("s", 0.0) < args.decay:
                y_axis += 1.0
            if now - last_press.get("a", 0.0) < args.decay:
                x_axis -= 1.0
            if now - last_press.get("d", 0.0) < args.decay:
                x_axis += 1.0

            speed_a, speed_b = compute_wheel_speeds(
                x_axis,
                y_axis,
                args.max_translation,
                args.max_rotation,
            )

            cmd = (speed_a, speed_b)
            if cmd != last_sent:
                print(f"[→] A={speed_a} B={speed_b}\r")
                last_sent = cmd

            base.set_speeds(speed_a, speed_b)

            if base.bridge.ser and base.bridge.ser.in_waiting:
                data = base.bridge.ser.read(base.bridge.ser.in_waiting).decode(errors="replace")
                for line in data.splitlines():
                    if line.strip():
                        print(f"[Arduino] {line.strip()}\r")

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n[!] Stopping.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_termios)
        try:
            base.stop()
        except Exception:
            pass
        try:
            base.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
