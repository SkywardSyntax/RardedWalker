import RPi.GPIO as GPIO
import time

def measure_distance(trigger_pin, echo_pin):
    """
    Measure distance using an HC-SR04 ultrasonic sensor.
    Returns the distance in centimeters.
    """
    GPIO.output(trigger_pin, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, GPIO.LOW)

    while GPIO.input(echo_pin) == 0:
        pulse_start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end_time = time.time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    return distance

try:
    GPIO.setmode(GPIO.BOARD)

    # Pins for Sensor A
    SENSOR_A_TRIGGER = 13
    SENSOR_A_ECHO = 11

    # Pin for the QYF740 haptic motor signal line (assuming the third pin is ground)
    MOTOR_PIN = 12  # Adjust according to your wiring

    # Set up sensor pins
    GPIO.setup(SENSOR_A_TRIGGER, GPIO.OUT)
    GPIO.output(SENSOR_A_TRIGGER, GPIO.LOW)
    GPIO.setup(SENSOR_A_ECHO, GPIO.IN)

    # Set up motor pin
    GPIO.setup(MOTOR_PIN, GPIO.OUT)
    GPIO.output(MOTOR_PIN, GPIO.LOW)

    print("Waiting for sensors to settle...")
    time.sleep(2)

    while True:
        distance_a = measure_distance(SENSOR_A_TRIGGER, SENSOR_A_ECHO)

        # Check distance
        if distance_a < 5:
            # Turn motor ON
            GPIO.output(MOTOR_PIN, GPIO.HIGH)
        else:
            # Turn motor OFF
            GPIO.output(MOTOR_PIN, GPIO.LOW)

        print(f"SENSOR A: {distance_a} cm", end='\r')
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nMeasurement stopped by user.")
finally:
    GPIO.cleanup()