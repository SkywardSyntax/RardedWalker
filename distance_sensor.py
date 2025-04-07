import RPi.GPIO as GPIO
import time

# GPIO Setup
GPIO.setmode(GPIO.BOARD)

# Ultrasonic Sensor Pins
TRIGGER_PIN = 13
ECHO_PIN = 11

# Configure GPIO pins
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.output(TRIGGER_PIN, GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Let sensor settle
time.sleep(2)

class DistanceSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

    def measure_distance(self):
        # Send trigger pulse
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

        # Wait for echo pulse
        start_time = time.time()
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
        while GPIO.input(self.echo_pin) == 1:
            end_time = time.time()

        # Calculate distance
        distance = (end_time - start_time) * 34300 / 2
        return distance

# Initialize distance sensor
distance_sensor = DistanceSensor(TRIGGER_PIN, ECHO_PIN)

# Moving average filter parameters
FILTER_SIZE = 5  # Number of measurements to average
measurements = []  # List to store recent measurements

try:
    while True:
        distance = distance_sensor.measure_distance()
        measurements.append(distance)
        
        # Limit the number of measurements to FILTER_SIZE
        if len(measurements) > FILTER_SIZE:
            measurements.pop(0)
        
        # Calculate the moving average
        average_distance = sum(measurements) / len(measurements)
        
        print(f"Distance: {average_distance:.1f} cm")
        
        time.sleep(.5)  # Take measurement every second

except KeyboardInterrupt:
    print("Measurement stopped by user")

finally:
    GPIO.cleanup()


