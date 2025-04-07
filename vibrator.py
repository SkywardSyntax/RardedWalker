import RPi.GPIO as GPIO
import time

try:
    GPIO.setmode(GPIO.BOARD)
    MOTOR_PIN = 12  # Change this pin if your wiring differs

    GPIO.setup(MOTOR_PIN, GPIO.OUT)

    print("Motor control started. Press Ctrl+C to stop.")

    while True:
        print("Motor ON")
        GPIO.output(MOTOR_PIN, GPIO.HIGH)
        time.sleep(1)

        print("Motor OFF")
        GPIO.output(MOTOR_PIN, GPIO.LOW)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nMotor control stopped by user.")
finally:
    GPIO.cleanup() 