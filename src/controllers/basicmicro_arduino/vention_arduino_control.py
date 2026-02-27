#!/usr/bin/env python3
"""
PC -> Arduino speed bridge (maintains your class-style layout).

Instead of using the Basicmicro Python library, this script sends commands to an Arduino
over USB serial. The Arduino then forwards the speeds to two motor drivers:

- Driver A (TX=11 RX=10) gets speed A
- Driver B (TX=9  RX=8 ) gets speed B

Protocol sent to Arduino (one line):
    A=<int> B=<int>\n
Example:
    A=-300 B=200

Your Arduino sketch must parse that format and set M1=M2 per driver accordingly
(which matches the working Arduino code you already have).

Usage examples:
    python3 pc_to_arduino_dual.py --port "/dev/serial/by-id/usb-Arduino_..." --speedA -300 --speedB 200 --run_s 5
    python3 pc_to_arduino_dual.py --port "/dev/serial/by-id/usb-Arduino_..." --mode rotate --w 150 --run_s 5
"""

import time
import logging
import argparse
import errno
from dataclasses import dataclass
from typing import Optional, Tuple

import serial
import serial.tools.list_ports


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


# ---- Defaults (override via CLI) ----
ARDUINO_PORT_DEFAULT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00"
ARDUINO_BAUD_DEFAULT = 115200


@dataclass
class ABCommand:
    a: int
    b: int

    def to_line(self) -> bytes:
        return f"A={int(self.a)} B={int(self.b)}\n".encode("utf-8")


class ArduinoSerialBridge:
    """
    Owns the single USB serial connection to Arduino and provides:
    - connect/disconnect/reconnect
    - safe_call() wrapper for robust retry on EIO / timeouts
    - send_ab(a, b)
    """

    def __init__(self, port_id: str, baud: int = 115200):
        self.port_id = port_id
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.connection_status = self.connect()

    def _open(self) -> serial.Serial:
        s = serial.Serial(
            self.port_id,
            self.baud,
            timeout=0.2,
            write_timeout=0.2,
        )
        # Arduino resets on open; allow bootloader/firmware to start
        time.sleep(2.0)
        return s

    def connect(self) -> bool:
        logger.info(f"Connecting to Arduino at {self.port_id}, baud={self.baud}")
        try:
            try:
                self.disconnect()
            except Exception:
                pass

            self.ser = self._open()
            logger.info(f"Connected to Arduino on {self.port_id}")
            return True
        except Exception as e:
            logger.error(f"Connection error on {self.port_id}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None
            logger.info(f"Disconnected from Arduino ({self.port_id})")

    def reconnect(self, attempts: int = 10, delay_s: float = 0.25) -> bool:
        for k in range(1, attempts + 1):
            logger.warning(f"[{self.port_id}] Reconnecting attempt {k}/{attempts}...")
            if self.connect():
                return True
            time.sleep(delay_s)
        logger.error(f"[{self.port_id}] Failed to reconnect after {attempts} attempts")
        return False

    def safe_call(self, fn, *args, retries: int = 1, reconnect_attempts: int = 10, **kwargs):
        """
        Execute a call; on USB EIO (Errno 5) or write timeout, reconnect and retry.
        """
        last_exc = None
        for _attempt in range(retries + 1):
            try:
                return fn(*args, **kwargs)
            except OSError as e:
                last_exc = e
                if getattr(e, "errno", None) == errno.EIO:  # Errno 5
                    logger.warning(f"[{self.port_id}] USB EIO (Errno 5) in {fn.__name__}: {e}")
                    if not self.reconnect(attempts=reconnect_attempts):
                        raise
                    continue
                raise
            except (serial.SerialTimeoutException, serial.serialutil.SerialTimeoutException) as e:
                last_exc = e
                logger.warning(f"[{self.port_id}] Write-timeout in {fn.__name__}: {e}")
                if not self.reconnect(attempts=reconnect_attempts):
                    raise
                continue
            except Exception as e:
                # Some drivers throw generic exceptions containing "timeout"
                last_exc = e
                msg = str(e).lower()
                if "timeout" in msg:
                    logger.warning(f"[{self.port_id}] Timeout in {fn.__name__}: {e}")
                    if not self.reconnect(attempts=reconnect_attempts):
                        raise
                    continue
                raise
        raise last_exc

    # ---- low-level send ----
    def _write_line(self, payload: bytes):
        if self.ser is None:
            raise RuntimeError("Arduino serial not connected")
        self.ser.write(payload)
        self.ser.flush()

    def send_ab(self, a: int, b: int):
        cmd = ABCommand(a=a, b=b)
        self.safe_call(self._write_line, cmd.to_line(), retries=2)


class VentionMotorDriverChannel:
    """
    Represents ONE motor driver channel (A or B), but sends via the shared Arduino bridge.
    It keeps a local setpoint and delegates combined (A,B) sends through the base object.
    """

    def __init__(self, channel: str, base: "VentionBase"):
        assert channel in ("A", "B")
        self.channel = channel
        self.base = base

    def speed_control(self, motor1_speed: int, motor2_speed: int, reset_encoders: bool = False):
        """
        Keep your old signature. Your requirement says both motors must have same speed,
        so we enforce that here.
        """
        if motor1_speed != motor2_speed:
            raise ValueError(
                f"Driver {self.channel}: motor1_speed must equal motor2_speed "
                f"(got {motor1_speed} vs {motor2_speed})."
            )
        self.set_speed(motor1_speed)

    def set_speed(self, speed: int):
        if self.channel == "A":
            self.base._setpoints.a = int(speed)
        else:
            self.base._setpoints.b = int(speed)
        self.base._send_setpoints()

    def stop(self):
        self.set_speed(0)


class VentionBase:
    """
    Maintains your class organization:
    - base_r and base_l exist (now they map to Driver A and Driver B)
    - translate/rotate keep same semantics as your old code
    - robust reconnect is handled in ArduinoSerialBridge.safe_call()
    """

    def __init__(self, port_id: str, baud: int = 115200):
        self.bridge = ArduinoSerialBridge(port_id, baud)
        self._setpoints = ABCommand(a=0, b=0)

        # Keep your naming: base_r/base_l
        # Here: base_r -> Driver A, base_l -> Driver B
        self.base_r = VentionMotorDriverChannel("A", self)
        self.base_l = VentionMotorDriverChannel("B", self)

        if not self.bridge.connection_status:
            logger.error("Failed to connect to Arduino. Exiting.")
            return
        logger.info("Successfully connected to Arduino speed bridge.")

        self._last_sent = None
        self._last_sent_time = 0.0
        self._min_send_period = 1.0 / 20.0  # 20 Hz
        # Start stopped
        self._send_setpoints()

    def _send_setpoints(self):
        now = time.time()
        ab = (self._setpoints.a, self._setpoints.b)

        # only send if changed
        if self._last_sent == ab:
            return

        # rate limit
        if (now - self._last_sent_time) < self._min_send_period:
            return

        logger.info(f"[Arduino] Send A={ab[0]} B={ab[1]}")
        self.bridge.send_ab(ab[0], ab[1])

        self._last_sent = ab
        self._last_sent_time = now
    # --- motion commands (same idea as your old code) ---
    def translate(self, linear_speed: int):
        """
        '+' forward, '-' backward.
        Both drivers same sign for translation.
        """
        self._setpoints.a = int(linear_speed)
        self._setpoints.b = int(linear_speed)
        self._send_setpoints()

    def rotate(self, angular_speed: int):
        """
        '+' CCW, '-' CW (in-place).
        Driver A gets +w, Driver B gets -w (matches your old logic).
        """
        self._setpoints.a = int(angular_speed)
        self._setpoints.b = int(-angular_speed)
        self._send_setpoints()

    def set_speeds(self, speed_a: int, speed_b: int):
        """
        Directly set different speeds to the two motor drivers.
        """
        self._setpoints.a = int(speed_a)
        self._setpoints.b = int(speed_b)
        self._send_setpoints()

    def stop(self):
        try:
            self.set_speeds(0, 0)
        except Exception as e:
            logger.warning(f"Stop failed (ignored): {e}")

    def disconnect(self):
        self.stop()
        self.bridge.disconnect()

    def reconnect(self):
        return self.bridge.reconnect()

    def read_encoders(self) -> Tuple[None, None]:
        """
        Not supported here (Arduino-only send bridge).
        Keeping method to preserve class interface.
        """
        return None, None


def list_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"{p.device} | {p.description} | {p.hwid}")


def main():
    parser = argparse.ArgumentParser(description="Send different speeds to two motor drivers via Arduino bridge.")
    parser.add_argument("--port", type=str, default=ARDUINO_PORT_DEFAULT,
                        help="Arduino port (stable): /dev/serial/by-id/... or /dev/serial/by-path/...")
    parser.add_argument("--baud", type=int, default=ARDUINO_BAUD_DEFAULT, help="Arduino baud (default: 115200)")
    parser.add_argument("--list_ports", action="store_true", help="List available serial ports and exit.")

    # Direct speed mode
    parser.add_argument("--speedA", type=int, default=0, help="Driver A speed (counts/sec)")
    parser.add_argument("--speedB", type=int, default=0, help="Driver B speed (counts/sec)")

    # Convenience motion modes (optional)
    parser.add_argument("--mode", choices=["direct", "translate", "rotate"], default="direct")
    parser.add_argument("--v", type=int, default=0, help="Translate speed for mode=translate")
    parser.add_argument("--w", type=int, default=0, help="Rotate speed for mode=rotate")

    parser.add_argument("--run_s", type=float, default=5.0, help="Run duration seconds")
    parser.add_argument("--hz", type=float, default=10.0, help="Send rate (Hz) (default: 10)")

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    if "REPLACE_ME" in args.port:
        logger.error("Set --port to your Arduino by-id/path. Try: ls -l /dev/serial/by-id/")
        return

    base = VentionBase(args.port, args.baud)
    if not base.bridge.connection_status:
        return

    try:
        dt = 1.0 / max(args.hz, 1e-6)
        start = time.time()

        while time.time() - start < args.run_s:
            if args.mode == "direct":
                base.set_speeds(args.speedA, args.speedB)
            elif args.mode == "translate":
                base.translate(args.v)
            elif args.mode == "rotate":
                base.rotate(args.w)

            time.sleep(dt)

        base.stop()

    finally:
        base.disconnect()


if __name__ == "__main__":
    main()