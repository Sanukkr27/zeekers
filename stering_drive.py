import RPi.GPIO as GPIO
import time
import curses  # Library for keyboard input

# Define GPIO pins for TB6600 driver
DIR_PIN_1 = 17  # Direction Pin for Motor 1
STEP_PIN_1 = 27  # Step Pin for Motor 1
DIR_PIN_2 = 22  # Direction Pin for Motor 2
STEP_PIN_2 = 23  # Step Pin for Motor 2

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN_1, GPIO.OUT)
GPIO.setup(STEP_PIN_1, GPIO.OUT)
GPIO.setup(DIR_PIN_2, GPIO.OUT)
GPIO.setup(STEP_PIN_2, GPIO.OUT)

# Motor speed settings
motor_speed = 0.001  # Adjust speed if needed
steps_per_cycle = 10  # Adjust for smoother movement

# Function to move the motors
def move_motors(direction):
    if direction == "forward":
        GPIO.output(DIR_PIN_1, GPIO.HIGH)
        GPIO.output(DIR_PIN_2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(DIR_PIN_1, GPIO.LOW)
        GPIO.output(DIR_PIN_2, GPIO.HIGH)
    elif direction == "left":
        GPIO.output(DIR_PIN_1, GPIO.LOW)
        GPIO.output(DIR_PIN_2, GPIO.LOW)
    elif direction == "right":
        GPIO.output(DIR_PIN_1, GPIO.HIGH)
        GPIO.output(DIR_PIN_2, GPIO.HIGH)

    for _ in range(steps_per_cycle):
        GPIO.output(STEP_PIN_1, GPIO.HIGH)
        GPIO.output(STEP_PIN_2, GPIO.HIGH)
        time.sleep(motor_speed)
        GPIO.output(STEP_PIN_1, GPIO.LOW)
        GPIO.output(STEP_PIN_2, GPIO.LOW)
        time.sleep(motor_speed)

# Keyboard control function
def main(stdscr):
    stdscr.clear()
    stdscr.addstr("Use W to move forward\n")
    stdscr.addstr("Use S to move backward\n")
    stdscr.addstr("Use A to turn left\n")
    stdscr.addstr("Use D to turn right\n")
    stdscr.addstr("Use T to stop\n")
    stdscr.addstr("Use Q to quit\n")
    stdscr.refresh()

    direction = None

    while True:
        key = stdscr.getch()

        if key == ord('w'):
            direction = "forward"
        elif key == ord('s'):
            direction = "backward"
        elif key == ord('a'):
            direction = "left"
        elif key == ord('d'):
            direction = "right"
        elif key == ord('t'):
            direction = None  # Stop movement
        elif key == ord('q'):
            break  # Exit

        stdscr.clear()
        stdscr.addstr("Press W, A, S, D to move | T to stop | Q to quit\n")
        if direction:
            stdscr.addstr(f"Moving {direction}...\n")
        else:
            stdscr.addstr("Stopped\n")
        stdscr.refresh()

        if direction:
            move_motors(direction)

# Run the program using curses
try:
    curses.wrapper(main)
except KeyboardInterrupt:
    print("\nProgram interrupted")
finally:
    GPIO.cleanup()

