import RPi.GPIO as GPIO
import time

# Define GPIO pins for TB6600 driver
DIR_PIN_1 = 17  # Motor 1 Direction
STEP_PIN_1 = 27  # Motor 1 Step
DIR_PIN_2 = 22  # Motor 2 Direction
STEP_PIN_2 = 23  # Motor 2 Step
ENA_PIN_1 = 18  # Motor 1 Enable (Set Low to Enable)
ENA_PIN_2 = 24  # Motor 2 Enable (Set Low to Enable)

# Define GPIO pins for encoders
ENCODER_A1 = 5  # Encoder 1 Channel A
ENCODER_B1 = 6  # Encoder 1 Channel B
ENCODER_A2 = 13 # Encoder 2 Channel A
ENCODER_B2 = 19 # Encoder 2 Channel B

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([DIR_PIN_1, STEP_PIN_1, DIR_PIN_2, STEP_PIN_2, ENA_PIN_1, ENA_PIN_2], GPIO.OUT)
GPIO.setup([ENCODER_A1, ENCODER_B1, ENCODER_A2, ENCODER_B2], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Motor movement settings
motor_speed = 0.0005  # Step delay (controls speed)
steps_per_revolution = 400  # Full step count for NEMA 23
gear_ratio = 10  # 10:1 planetary gear
effective_steps_per_rev = steps_per_revolution * gear_ratio  # 2000 steps per rev
target_steps =4000  # Steps to move forward/backward

# Encoder counts
encoder_1_count = 0
encoder_2_count = 0

# PID controller parameters
Kp = 0.1  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.05  # Derivative gain
previous_error = 0
integral = 0

# Encoder callback functions
def encoder1_callback(channel):
    global encoder_1_count
    a_state = GPIO.input(ENCODER_A1)
    b_state = GPIO.input(ENCODER_B1)
    if a_state == b_state:  
        encoder_1_count += 1  # Clockwise
    else:
        encoder_1_count -= 1  # Counter-clockwise

def encoder2_callback(channel):
    global encoder_2_count
    a_state = GPIO.input(ENCODER_A2)
    b_state = GPIO.input(ENCODER_B2)
    if a_state == b_state:  
        encoder_2_count += 1  # Clockwise
    else:
        encoder_2_count -= 1  # Counter-clockwise

# Attach encoder interrupts
GPIO.add_event_detect(ENCODER_A1, GPIO.BOTH, callback=encoder1_callback)
GPIO.add_event_detect(ENCODER_A2, GPIO.BOTH, callback=encoder2_callback)

# Fine-tuning function to ensure motors are synchronized
def fine_tune_motor_movement():
    """Correct motor synchronization by adjusting for discrepancies between encoders."""
    global encoder_1_count, encoder_2_count
    error = encoder_1_count - encoder_2_count
    if abs(error) > 100:  # If error exceeds 100 steps, adjust
        # Fine-tune by adjusting the step delay proportionally to the error
        correction_factor = 1 - min(0.9, abs(error) / 1000)
        return correction_factor
    return 1  # No fine-tuning needed if error is small

# Function to move motors with PID feedback correction
def move_motor_with_pid(direction_1, direction_2, steps):
    global encoder_1_count, encoder_2_count, previous_error, integral

    # Set direction
    GPIO.output(DIR_PIN_1, direction_1)
    GPIO.output(DIR_PIN_2, direction_2)

    # Enable motors
    GPIO.output(ENA_PIN_1, GPIO.LOW)
    GPIO.output(ENA_PIN_2, GPIO.LOW)

    # Reset encoder values
    encoder_1_count = 0
    encoder_2_count = 0

    for step in range(steps):
        # Calculate error
        error = encoder_1_count - encoder_2_count

        # PID calculations
        proportional = Kp * error
        integral += error
        integral_term = Ki * integral
        derivative = error - previous_error
        derivative_term = Kd * derivative

        # Total PID output
        pid_output = proportional + integral_term + derivative_term

        # Fine-tune based on error between encoders
        fine_tune_factor = fine_tune_motor_movement()
        
        # Adjust step timing based on PID output
        if abs(pid_output) > 0.1:  # Avoid over-correction
            if pid_output > 0:
                time.sleep(motor_speed * (1 - min(0.9, abs(pid_output)) * fine_tune_factor))
            else:
                time.sleep(motor_speed * (1 - min(0.9, abs(pid_output)) * fine_tune_factor))
        else:
            time.sleep(motor_speed * fine_tune_factor)  # Normal speed with fine-tuning

        # Update step signals for both motors
        GPIO.output(STEP_PIN_1, GPIO.HIGH)
        GPIO.output(STEP_PIN_2, GPIO.HIGH)
        time.sleep(motor_speed)
        GPIO.output(STEP_PIN_1, GPIO.LOW)
        GPIO.output(STEP_PIN_2, GPIO.LOW)
        time.sleep(motor_speed)

        # Update previous error for derivative calculation
        previous_error = error

        if step % 100 == 0:  # Print every 100 steps
            print(f"Step {step+1}/{steps} | Encoder 1: {encoder_1_count} | Encoder 2: {encoder_2_count} | Error: {error} | Fine-tune factor: {fine_tune_factor}")

    # Disable motors after movement
    GPIO.output(ENA_PIN_1, GPIO.HIGH)
    GPIO.output(ENA_PIN_2, GPIO.HIGH)

    print("Movement complete.")

# Main loop: move forward & backward automatically with PID feedback correction
try:
    while True:
        print("\nMoving Forward...")
        move_motor_with_pid(GPIO.HIGH, GPIO.LOW, target_steps)

        time.sleep(1)  # Pause before changing direction

        print("\nMoving Backward...")
        move_motor_with_pid(GPIO.LOW, GPIO.HIGH, target_steps)

        time.sleep(1)  # Pause before restarting loop

except KeyboardInterrupt:
    print("\nProgram interrupted. Cleaning up GPIO.")  
    GPIO.cleanup()

