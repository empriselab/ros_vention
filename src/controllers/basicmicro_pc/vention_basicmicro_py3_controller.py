#!/usr/bin/env python3
"""
Basicmicro dual-controller example with robust reconnect using stable USB IDs.

Key idea:
- DO NOT use /dev/ttyACM0/1/2/3 directly (they renumber on disconnect).
- Use stable udev symlinks:
    /dev/serial/by-id/...
  (or /dev/serial/by-path/... if by-id is not unique)

What this script does:
- Connects to two RoboClaw/Basicmicro controllers via their by-id paths.
- On Errno 5 (EIO) or write-timeout, it will:
    1) close
    2) reopen by the SAME by-id path (so it reconnects to the correct physical device)
    3) retry the command

IMPORTANT:
- You must set CTRL1_PORT_ID and CTRL2_PORT_ID to the correct by-id strings.
  Run:  ls -l /dev/serial/by-id/
"""

import time
import logging
import argparse
import errno
from basicmicro import Basicmicro

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# ---- SET THESE (or pass via CLI) ----
# Example (yours will be different):
# "/dev/serial/by-id/usb-Basicmicro_Inc_USB_Roboclaw_2x45A_ABC123-if00"
CTRL1_PORT_ID_DEFAULT = "/dev/serial/by-path/pci-0000:04:00.3-usb-0:2.2:1.0"
CTRL2_PORT_ID_DEFAULT = "/dev/serial/by-path/pci-0000:04:00.3-usb-0:2.4:1.0"


class VentionBasicmicroPy3Controller:
    def __init__(self, port_id: str, baud: int = 9600, address: int = 0x80):
        self.port_id = port_id  # stable path (by-id or by-path)
        self.baud = baud
        self.address = address
        self.controller = None
        self.connection_status = self.connect()

        if self.connection_status:
            self.safe_call(self.controller.ResetEncoders, self.address)

    def _open(self) -> Basicmicro:
        """Open a Basicmicro connection using the stable port_id."""
        bm = Basicmicro(self.port_id, self.baud)
        bm.__enter__()  # Basicmicro relies on __enter__ for opening/init
        return bm

    def connect(self) -> bool:
        logger.info(f"Connecting to controller at {self.port_id}, baud={self.baud}")
        try:
            # best-effort close previous connection
            try:
                self.disconnect()
            except Exception:
                pass

            self.controller = self._open()

            ok, ver = self.controller.ReadVersion(self.address)
            if ok:
                logger.info(f"Connected on {self.port_id} | firmware: {ver}")
                return True

            logger.error(f"Failed to read firmware version on {self.port_id}")
            return False
        except Exception as e:
            logger.error(f"Connection error on {self.port_id}: {e}")
            self.controller = None
            return False

    def disconnect(self):
        if self.controller is not None:
            try:
                self.controller.__exit__(None, None, None)
            finally:
                self.controller = None
            logger.info(f"Disconnected from controller ({self.port_id})")

    def reconnect(self, attempts: int = 10, delay_s: float = 0.25) -> bool:
        """Reconnect to the SAME stable USB ID path."""
        for k in range(1, attempts + 1):
            logger.warning(f"[{self.port_id}] Reconnecting attempt {k}/{attempts}...")
            if self.connect():
                return True
            time.sleep(delay_s)
        logger.error(f"[{self.port_id}] Failed to reconnect after {attempts} attempts")
        return False

    def safe_call(self, fn, *args, retries: int = 1, reconnect_attempts: int = 10, **kwargs):
        """
        Execute controller call; if USB drops (Errno 5) or write-timeout occurs,
        reconnect via stable USB ID and retry.
        """
        last_exc = None
        for attempt in range(retries + 1):
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
            except Exception as e:
                last_exc = e
                msg = str(e).lower()
                if "timeout" in msg and ("write" in msg or "sending" in msg):
                    logger.warning(f"[{self.port_id}] Write-timeout in {fn.__name__}: {e}")
                    if not self.reconnect(attempts=reconnect_attempts):
                        raise
                    continue
                raise
        raise last_exc

    # --- Commands ---
    def speed_control(self, motor1_speed, motor2_speed, reset_encoders: bool = False):
        logger.info(f"[{self.port_id}] SpeedM1M2: m1={motor1_speed}, m2={motor2_speed}")
        if reset_encoders:
            self.safe_call(self.controller.ResetEncoders, self.address, retries=2)

        self.safe_call(self.controller.SpeedM1M2, self.address, motor1_speed, motor2_speed, retries=2)

    def read_encoders(self):
        enc1 = self.safe_call(self.controller.ReadEncM1, self.address, retries=1)
        enc2 = self.safe_call(self.controller.ReadEncM2, self.address, retries=1)
        return enc1, enc2

    def stop(self):
        # Best-effort stop (try speed stop, then duty stop)
        try:
            self.safe_call(self.controller.SpeedM1M2, self.address, 0, 0, retries=2)
        except Exception:
            pass
        try:
            self.safe_call(self.controller.DutyM1M2, self.address, 0, 0, retries=2)
        except Exception:
            pass


class VentionBase(VentionBasicmicroPy3Controller):
    def __init__(self, port_id: str, baud: int = 9600, address: int = 0x80):
        super().__init__(port_id, baud, address)
        # You can add more base-specific methods here if needed
        self.base_r = VentionBasicmicroPy3Controller(CTRL1_PORT_ID_DEFAULT, baud, address)
        self.base_l = VentionBasicmicroPy3Controller(CTRL2_PORT_ID_DEFAULT, baud, address)
        if not self.base_r.connection_status or not self.base_l.connection_status:
            logger.error("Failed to connect to one or both base controllers. Exiting.")
            return
        else:
            logger.info("Successfully connected to both base controllers.")

    def translate(self, linear_speed):
        '''
        Translate linear speed in encoder counts/sec,
        - "+" to move forward,
        - "-" to move backward.
        '''
        self.base_r.speed_control(linear_speed, linear_speed)
        self.base_l.speed_control(linear_speed, linear_speed)
    
    def rotate(self, angular_speed):
        '''
        Rotate in place with angular speed in encoder counts/sec,
        - "+" to rotate counterclockwise,
        - "-" to rotate clockwise.
        '''
        self.base_r.speed_control(angular_speed, angular_speed)
        self.base_l.speed_control(-angular_speed, -angular_speed)   

    def stop(self):
        self.base_r.stop()
        self.base_l.stop()  

    def disconnect(self):
        self.base_r.disconnect()
        self.base_l.disconnect()
        return super().disconnect() 
    
    def read_encoders(self):
        enc_r1, enc_r2 = self.base_r.read_encoders()
        enc_l1, enc_l2 = self.base_l.read_encoders()
        return (enc_r1, enc_r2), (enc_l1, enc_l2)
    
def main():
    parser = argparse.ArgumentParser(description="Dual RoboClaw/Basicmicro controller test using /dev/serial/by-id")
    parser.add_argument("--ctrl1", type=str, default=CTRL1_PORT_ID_DEFAULT,
                        help="Controller 1 port (stable): /dev/serial/by-id/... or /dev/serial/by-path/...")
    parser.add_argument("--ctrl2", type=str, default=CTRL2_PORT_ID_DEFAULT,
                        help="Controller 2 port (stable): /dev/serial/by-id/... or /dev/serial/by-path/...")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("-a", "--address", type=lambda x: int(x, 0), default=0x80,
                        help="Controller address (default: 0x80)")
    parser.add_argument("--speed", type=int, default=-100, help="Speed setpoint (counts/sec)")
    parser.add_argument("--run_s", type=float, default=5.0, help="Run duration seconds")
    args = parser.parse_args()

    if "REPLACE_ME" in args.ctrl1 or "REPLACE_ME" in args.ctrl2:
        logger.error("You must set --ctrl1 and --ctrl2 to real /dev/serial/by-id/* paths.")
        logger.error("Run: ls -l /dev/serial/by-id/")
        return

    c1 = VentionBasicmicroPy3Controller(args.ctrl1, args.baud, args.address)
    c2 = VentionBasicmicroPy3Controller(args.ctrl2, args.baud, args.address)

    if not c1.connection_status or not c2.connection_status:
        logger.error("Failed to connect to one or both controllers. Exiting.")
        return

    try:
        # Start both
        c1.speed_control(args.speed, args.speed)
        c2.speed_control(args.speed, args.speed)

        start = time.time()
        while time.time() - start < args.run_s:
            e1, e2 = c1.read_encoders()
            e1b, e2b = c2.read_encoders()
            print(f"C1 [{c1.port_id}] enc1={e1} enc2={e2} | C2 [{c2.port_id}] enc1={e1b} enc2={e2b}")
            time.sleep(0.2)

        # Stop both
        c1.stop()
        c2.stop()

    finally:
        c1.disconnect()
        c2.disconnect()


if __name__ == "__main__":
    main()