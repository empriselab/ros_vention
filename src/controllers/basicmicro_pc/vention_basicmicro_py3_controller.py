#!/usr/bin/env python3
"""
Basic Movement Example for Basicmicro motor controllers.

This example demonstrates the fundamental movement commands:
- Setting duty cycle (analog to PWM)
- Speed control
- Mixed mode control for differential drive
"""

import time
import logging
import argparse
from basicmicro import Basicmicro

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class VentionBasicmicroPy3Controller:
    def __init__(self, port='/dev/ttyACM0', baud=9600, address=0x80):
        self.port = port
        self.baud = baud
        self.address = address
        self.controller = None
        self.connection_status = self.connect()

        if self.connection_status:
            self.controller.ResetEncoders(self.address)

    def connect(self):
        logger.info(f"Connecting to controller at {self.port}, baud rate {self.baud}")
        try:
            self.controller = Basicmicro(self.port, self.baud)

            # IMPORTANT: actually open/init the port (same as "with Basicmicro(...) as controller")
            self.controller.__enter__()

            version_result = self.controller.ReadVersion(self.address)
            if version_result[0]:
                logger.info(f"Connected to controller with firmware version: {version_result[1]}")
                return True
            else:
                logger.error("Failed to read firmware version!")
                return False
        except Exception as e:
            logger.error(f"Connection error: {str(e)}")
            return False

    def disconnect(self):
        if self.controller:
            # Properly close context
            self.controller.__exit__(None, None, None)
            logger.info("Disconnected from controller")

    def pwm_control(self, motor1_pwm, motor2_pwm):
        logger.info(f"Setting Motor 1 PWM: {motor1_pwm}, Motor 2 PWM: {motor2_pwm}")
        self.controller.DutyM1(self.address, motor1_pwm)
        self.controller.DutyM2(self.address, motor2_pwm)
    
    def speed_control(self, motor1_speed, motor2_speed, reset_encoders=False):
        logger.info(f"Setting Motor 1 Speed: {motor1_speed} counts/sec, Motor 2 Speed: {motor2_speed} counts/sec")
        if reset_encoders:
            self.controller.ResetEncoders(self.address)
        
        self.controller.SpeedM1M2(self.address, motor1_speed, motor2_speed)
        enc1 = self.controller.ReadEncM1(self.address)
        enc2 = self.controller.ReadEncM2(self.address)
        print(f"enc1={enc1}, enc2={enc2}")

    def speed_controlled_position_move(
        self,
        motor1_speed,
        motor2_speed,
        motor1_target,
        motor2_target,
        timeout_s=10.0,
        poll_s=0.2,
        settle_reads=3,
        stop_on_timeout=True,
    ):
        logger.info("\n=== SPEED-CONTROLLED POSITION MOVEMENT ===")

        buffer = 0

        logger.info(f"M1: target={motor1_target}, speed={motor1_speed}")
        logger.info(f"M2: target={motor2_target}, speed={motor2_speed}")

        # Issue commands
        M1_status = None
        M2_status = None
        M1_status = self.controller.SpeedPositionM1(self.address, motor1_speed, motor1_target, buffer)
        # M2_status = self.controller.SpeedPositionM2(self.address, motor2_speed, motor2_target, buffer)

        print(f"M1 command status: {M1_status}, M2 command status: {M2_status}")

        start_t = time.time()
        zero_count = 0

        while True:
            elapsed = time.time() - start_t

            # Timeout
            if elapsed > timeout_s:
                logger.error(f"Timeout after {timeout_s:.1f}s waiting for motion complete.")
                if stop_on_timeout:
                    try:
                        self.controller.SpeedM1M2(self.address, 0, 0)
                    except Exception as e:
                        logger.error(f"Failed to stop motors on timeout: {e}")
                raise TimeoutError(f"SpeedPosition timed out after {timeout_s:.1f}s")

            # Read status
            buffer_status = self.controller.ReadBuffers(self.address)
            enc1 = self.controller.ReadEncM1(self.address)
            enc2 = self.controller.ReadEncM2(self.address)
            print(f"enc1={enc1}, enc2={enc2}")

            # Log if buffer read succeeded
            if buffer_status[0]:
                b = buffer_status[1]
                e1 = enc1[1] if enc1[0] else None
                e2 = enc2[1] if enc2[0] else None
                logger.info(f"t={elapsed:5.2f}s  enc1={e1} enc2={e2}  buffer={b}")
            else:
                logger.warning(f"t={elapsed:5.2f}s  ReadBuffers failed")

            # Completion condition (debounced)
            if buffer_status[0] and buffer_status[1] == 0:
                zero_count += 1
                if zero_count >= settle_reads:
                    logger.info("Movement complete")
                    break
            else:
                zero_count = 0

            time.sleep(poll_s)
            


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Basicmicro motor controller basic movement example')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyACM0', help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=9600, help='Baud rate (default: 38400)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x80, 
                        help='Controller address (default: 0x80)')
    args = parser.parse_args()
    
    logger.info(f"Connecting to controller at {args.port}, baud rate {args.baud}")
    
    try:
        vention = VentionBasicmicroPy3Controller(args.port, args.baud, args.address)
        vention_2 = VentionBasicmicroPy3Controller('/dev/ttyACM1', args.baud, args.address)
        if not vention.connection_status:
            logger.error("Failed to connect to controller. Exiting.")
            return
        # example usage of the controller methods

        speed = -1000  # counts per second
        vention.speed_control(speed, speed)  # Motor 1 forward, Motor 2 forward at 1500 counts/sec
        start_t = time.time()
        vention_2.speed_control(speed, speed)  # Motor 1 forward, Motor 2 forward at 1500 counts/sec
        while time.time() - start_t < 5.0:
            enc1 = vention.controller.ReadEncM1(vention.address)
            enc2 = vention.controller.ReadEncM2(vention.address)
            print(f"Controller 1 - enc1={enc1}, enc2={enc2}")
            enc1_2 = vention_2.controller.ReadEncM1(vention_2.address)
            enc2_2 = vention_2.controller.ReadEncM2(vention_2.address)
            print(f"Controller 2 - enc1={enc1_2}, enc2={enc2_2}")
        vention.speed_control(0, 0)  # Stop both motors
        vention_2.speed_control(0, 0)  # Stop both motors

        # vention_2.speed_controlled_position_move(
        #     motor1_speed=1000,
        #     motor2_speed=1000,
        #     motor1_target=10000,
        #     motor2_target=10000,
        #     timeout_s=15.0
        # )
        vention.disconnect()
        vention_2.disconnect()


    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()