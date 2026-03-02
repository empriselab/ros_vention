import pygame
import serial
import time

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyACM0'   # arduio port
BAUD_RATE = 9600
DEADBAND = 0.4
COMMAND_INTERVAL = 0.01  # seconds

# === INIT SERIAL ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
time.sleep(2)  # Wait for Arduino to reset

# === INIT PYGAME ===
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise Exception("No game controller connected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller: {joystick.get_name()}")

last_cmd = ''

def send_command(cmd: str):
    """Send a single-character command if it changed."""
    global last_cmd
    if cmd != last_cmd:
        ser.write(cmd.encode('utf-8'))
        print(f"[→] Sent command: '{cmd}'")
        last_cmd = cmd

# Track previous button state for edge detection
prev_lb = 0  # LB (often button 4)
prev_rb = 0  # RB (often button 5)

try:
    while True:
        pygame.event.pump()

        # Axes: left stick
        x_axis = joystick.get_axis(0)  # Left stick X
        y_axis = joystick.get_axis(1)  # Left stick Y

        # Buttons (Xbox mapping; may vary by controller/OS)
        lb = joystick.get_button(4)  # LB = speed down
        rb = joystick.get_button(5)  # RB = speed up

        # Hat/D-pad (optional; not used for speed, but you could extend)
        # hat_x, hat_y = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)

        # ---- Movement command from stick with deadband ----
        cmd = 's'  # Default: stop
        if abs(y_axis) > DEADBAND:
            cmd = 'f' if y_axis < 0 else 'b'
        elif abs(x_axis) > DEADBAND:
            cmd = 'l' if x_axis < 0 else 'r'

        send_command(cmd)

        # ---- Speed adjust (edge-triggered) ----
        # RB rising edge → '+'
        if rb and not prev_rb:
            ser.write(b'+')
            print("[→] Speed +")
            # don't change last_cmd so movement cmd keeps flowing

        # LB rising edge → '-'
        if lb and not prev_lb:
            ser.write(b'-')
            print("[→] Speed -")

        prev_lb, prev_rb = lb, rb

        time.sleep(COMMAND_INTERVAL)

except KeyboardInterrupt:
    send_command('s')
    print("\n[!] Stopped. Exiting...")
finally:
    try:
        joystick.quit()
    except Exception:
        pass
    pygame.quit()
    try:
        ser.close()
    except Exception:
        pass
