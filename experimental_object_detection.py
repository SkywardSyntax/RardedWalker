# filepath: /workspaces/RardedWalker/object_detection.py
import cv2 as cv
import numpy as np
import time
import RPi.GPIO as GPIO
import threading
import argparse
from collections import deque
import math

# Global variables for sensor fusion
distance_cm = 100  # Default safe distance
is_visual_obstructed = False # Kept for potential overall assessment, but might be replaced
obstruction_percent = 0 # Kept for potential overall assessment
scene_complexity = 0 # Kept for potential overall assessment
num_contours = 0 # Kept for potential overall assessment

# New global variables for left/right assessment
left_obstruction_score = 0
right_obstruction_score = 0
left_contour_area_percent = 0
right_contour_area_percent = 0
left_num_contours = 0
right_num_contours = 0


# Updated weights: 15% camera, 85% ultrasonic - trusting distance sensor more
camera_weight = 0.15
ultrasonic_weight = 0.85

# Lock for thread safety when accessing shared variables
sensor_lock = threading.Lock()

# Moving average filter for ultrasonic sensor readings
FILTER_LENGTH = 5
distance_readings = deque(maxlen=FILTER_LENGTH)

# Initialize with default safe values
for _ in range(FILTER_LENGTH):
    distance_readings.append(100)

# Variable to track if we need to update the motor state (might need adjustment for L/R)
motor_update_needed = False
last_fused_assessment = 0 # Will be replaced by left/right assessments

# Haptic feedback variables
vibration_active_left = False
vibration_active_right = False
vibration_intensity_left = 0  # 0-100%
vibration_intensity_right = 0 # 0-100%
PWM_FREQUENCY = 100  # Hz for PWM control
haptic_left_pwm = None  # Will hold the left PWM object
haptic_right_pwm = None  # Will hold the right PWM object

def measure_distance(trigger_pin, echo_pin):
    """
    Optimized function to measure distance using an HC-SR04 ultrasonic sensor.
    Returns the distance in centimeters.
    """
    # Send pulse
    GPIO.output(trigger_pin, GPIO.LOW)
    # Removed unnecessary sleep for LOW state
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(trigger_pin, GPIO.LOW)

    # More efficient timing with single time reference
    start_time = time.time()
    timeout = start_time + 0.03  # 30ms timeout (reduced from 100ms)

    # Wait for echo to start with timeout
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return 400  # Return max range if timeout

    # Wait for echo to end with timeout
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return 400  # Return max range if timeout

    pulse_duration = pulse_end - pulse_start
    distance = round(pulse_duration * 17150, 2)

    # Limit to reasonable range (2cm-400cm)
    return max(2, min(distance, 400))

def apply_median_filter(readings):
    """
    Apply a median filter to remove outliers from readings.
    """
    sorted_readings = sorted(readings)
    mid = len(sorted_readings) // 2

    # If even number of readings, average the two middle values
    if len(sorted_readings) % 2 == 0:
        # Handle empty list case
        if not sorted_readings:
             return 100 # Return default safe distance or handle appropriately
        # Ensure mid-1 is not negative for lists with 2 elements
        if mid > 0:
             return (sorted_readings[mid-1] + sorted_readings[mid]) / 2
        else: # Only one element after potential filtering? Return that element.
             return sorted_readings[0] # Or handle as error/default
    else:
        return sorted_readings[mid]


def set_haptic_intensity(left_intensity, right_intensity):
    """
    Set the left and right haptic motor vibration intensity independently using PWM.
    intensity: 0-100 where 0 is off and 100 is maximum vibration.
    """
    global haptic_left_pwm, haptic_right_pwm
    global vibration_active_left, vibration_active_right
    global vibration_intensity_left, vibration_intensity_right

    # Limit intensity to valid range
    left_intensity = max(0, min(100, left_intensity))
    right_intensity = max(0, min(100, right_intensity))

    vibration_intensity_left = left_intensity
    vibration_intensity_right = right_intensity

    # Update vibration state for left motor
    if left_intensity > 0 and not vibration_active_left:
        vibration_active_left = True
    elif left_intensity == 0 and vibration_active_left:
        vibration_active_left = False

    # Update vibration state for right motor
    if right_intensity > 0 and not vibration_active_right:
        vibration_active_right = True
    elif right_intensity == 0 and vibration_active_right:
        vibration_active_right = False

    # Set PWM duty cycle based on intensity for each motor
    if haptic_left_pwm:
        haptic_left_pwm.ChangeDutyCycle(left_intensity)
    if haptic_right_pwm:
        haptic_right_pwm.ChangeDutyCycle(right_intensity)


def get_haptic_intensity_for_side(side_score, distance):
    """
    Generates a haptic vibration intensity based on a side's score and overall distance.
    Returns intensity value between 0-100.
    """
    # Threshold might need tuning based on how side_score is calculated
    score_threshold = 30 # Example threshold - adjust as needed
    if side_score <= score_threshold:
        return 0  # No vibration if score is low

    # Base intensity increases with assessment score (scale from threshold-100 to 0-100)
    base_intensity = ((side_score - score_threshold) / (100 - score_threshold)) * 100
    base_intensity = max(0, min(100, base_intensity)) # Ensure it's within 0-100

    # Distance factor - stronger vibration for closer objects
    # Using the same distance logic as before, adjust sensitivity if needed
    distance_factor = 1.0
    close_distance_threshold = 40 # cm
    if distance < close_distance_threshold:
        # Increase factor from 1.0 up to 2.0 as distance approaches 0
        distance_factor = 1 + ((close_distance_threshold - distance) / close_distance_threshold)

    # Calculate final intensity with distance consideration
    intensity = min(100, base_intensity * distance_factor)

    return intensity

def haptic_feedback_thread(left_motor_pin, right_motor_pin):
    """Thread for controlling haptic feedback patterns for left and right motors independently."""
    global vibration_active_left, vibration_active_right
    global vibration_intensity_left, vibration_intensity_right
    global haptic_left_pwm, haptic_right_pwm
    global left_obstruction_score, right_obstruction_score, distance_cm

    # Setup PWM for both vibration motors
    haptic_left_pwm = GPIO.PWM(left_motor_pin, PWM_FREQUENCY)
    haptic_left_pwm.start(0)  # Start with 0% duty cycle (off)

    haptic_right_pwm = GPIO.PWM(right_motor_pin, PWM_FREQUENCY)
    haptic_right_pwm.start(0)  # Start with 0% duty cycle (off)

    try:
        while True:
            # Get current sensor values under lock
            with sensor_lock:
                current_distance = distance_cm
                current_left_score = left_obstruction_score
                current_right_score = right_obstruction_score

            # Calculate appropriate intensity for each side based on its score and the overall distance
            intensity_left = get_haptic_intensity_for_side(current_left_score, current_distance)
            intensity_right = get_haptic_intensity_for_side(current_right_score, current_distance)

            # Update motor intensities independently
            set_haptic_intensity(intensity_left, intensity_right)

            # Brief sleep to avoid excessive CPU usage
            time.sleep(0.05) # 50ms interval
    except Exception as e:
        print(f"Haptic feedback thread error: {e}")
    finally:
        if haptic_left_pwm:
            haptic_left_pwm.stop()
        if haptic_right_pwm:
            haptic_right_pwm.stop()

def ultrasonic_thread_function(trigger_pin, echo_pin):
    """Thread function to continuously read the ultrasonic sensor with optimized performance"""
    global distance_cm, distance_readings, ultrasonic_weight, motor_update_needed # motor_update_needed might be less relevant now

    # Track timing for adaptive sensing
    last_significant_change_time = time.time()
    baseline_delay = 0.05  # 50ms base delay between readings
    current_delay = baseline_delay
    last_distance = 100
    significant_change_threshold = 5  # cm

    try:
        while True:
            # Measure and filter distance (only if ultrasonic has weight, though it's 0.85)
            measured_distance = measure_distance(trigger_pin, echo_pin)

            # Quick pre-check if value is reasonable before more processing
            if measured_distance < 2 or measured_distance > 400:
                 # If reading is bad, maybe use the last good reading?
                 # Or keep the previous filtered_distance? For now, use last_distance.
                 measured_distance = last_distance # Reject obviously bad readings

            # Apply filtering (using median filter)
            distance_readings.append(measured_distance)
            filtered_distance = apply_median_filter(list(distance_readings)) # Pass a list copy

            # Detect significant changes to adapt sensing rate (optional, could simplify)
            distance_change = abs(filtered_distance - last_distance)
            now = time.time()

            if distance_change > significant_change_threshold:
                last_significant_change_time = now
                current_delay = baseline_delay  # Fast update when distance is changing
                # motor_update_needed = True # Flag might be less useful if haptics update continuously
            else:
                time_since_change = now - last_significant_change_time
                # Gradually slow down updates if distance is stable
                if time_since_change > 1.0:  # After 1 second of stability
                    # Slow down up to 200ms max delay
                    current_delay = min(0.2, baseline_delay * (1 + time_since_change/5))

            # Update global distance under lock
            with sensor_lock:
                distance_cm = filtered_distance
                # Update last_distance *after* the lock to reflect the value used
                last_distance = filtered_distance


            time.sleep(current_delay)  # Adaptive delay
    except Exception as e:
        print("Ultrasonic thread encountered an error:", e)


def calculate_obstruction_scores():
    """
    Calculate fused obstruction scores for LEFT and RIGHT sides (0-100%).
    Combines distance sensor (equally for both sides) and visual analysis (specific to each side).
    """
    global distance_cm, camera_weight, ultrasonic_weight
    global left_contour_area_percent, right_contour_area_percent # Using area percentage now
    # Add other visual metrics if needed (e.g., left_num_contours)

    # --- Distance Score (same for both sides) ---
    distance_score = 0
    if ultrasonic_weight > 0:
        # Score based on distance: 0cm -> 100%, 80cm -> 0% (double sensitivity range)
        distance_score = max(0, min(100, 100 - (distance_cm * 1.25)))

        # Apply additional emphasis for very close distances (e.g., < 40cm)
        close_distance_threshold = 40
        if distance_cm < close_distance_threshold:
            # Boost score proportionally as distance decreases below threshold
            distance_boost = max(0, close_distance_threshold - distance_cm) * 1.5 # Adjust multiplier as needed
            distance_score = min(100, distance_score + distance_boost)

    # --- Visual Scores (specific to each side) ---
    left_visual_score = 0
    right_visual_score = 0
    if camera_weight > 0:
        # Simple visual score based on contour area percentage for each side
        # Scale the percentage (0-100) directly. Could add complexity here.
        left_visual_score = max(0, min(100, left_contour_area_percent))
        right_visual_score = max(0, min(100, right_contour_area_percent))

        # --- Optional: Add complexity based on number of contours ---
        # Example: Reduce score slightly if many small contours (less likely a single large obstacle)
        # complexity_factor_left = max(0.5, 1 - (left_num_contours / 50.0)) # Example scaling
        # left_visual_score *= complexity_factor_left
        # complexity_factor_right = max(0.5, 1 - (right_num_contours / 50.0))
        # right_visual_score *= complexity_factor_right

    # --- Fused Scores ---
    # Combine distance and visual scores using weights
    # Ensure weights sum to 1 (or handle cases where one is 0)
    effective_cam_weight = camera_weight
    effective_ultra_weight = ultrasonic_weight
    if camera_weight + ultrasonic_weight != 1.0:
         # Basic normalization if they don't sum to 1
         total_weight = camera_weight + ultrasonic_weight
         if total_weight > 0:
              effective_cam_weight = camera_weight / total_weight
              effective_ultra_weight = ultrasonic_weight / total_weight
         else: # Both zero? Scores will be zero anyway.
              effective_cam_weight = 0
              effective_ultra_weight = 0


    fused_left_score = (left_visual_score * effective_cam_weight) + (distance_score * effective_ultra_weight)
    fused_right_score = (right_visual_score * effective_cam_weight) + (distance_score * effective_ultra_weight)

    # Ensure scores are within 0-100
    fused_left_score = max(0, min(100, fused_left_score))
    fused_right_score = max(0, min(100, fused_right_score))

    return fused_left_score, fused_right_score


def process_frame(raw_img):
    """
    Process camera frame to detect obstructions, analyzing left and right halves separately.
    Updates global variables for left/right contour area and count.
    Returns visualization images.
    """
    global left_contour_area_percent, right_contour_area_percent
    global left_num_contours, right_num_contours
    global camera_weight # Use camera_weight to decide if processing is needed

    # Default return values if camera is disabled or frame is invalid
    default_shape = (170, 420, 3) # Based on the ROI dimensions used below
    default_thresh_shape = (170, 420)
    default_edge_img = np.zeros(default_shape, dtype=np.uint8)
    default_thresh = np.zeros(default_thresh_shape, dtype=np.uint8)
    default_edge_only = np.zeros(default_shape, dtype=np.uint8)

    if camera_weight <= 0 or raw_img is None:
        # Reset visual scores if camera is off
        with sensor_lock:
            left_contour_area_percent = 0
            right_contour_area_percent = 0
            left_num_contours = 0
            right_num_contours = 0
        # Add text overlay indicating camera is disabled
        if raw_img is not None: # Check if we can even draw on raw_img
             h, w = raw_img.shape[:2]
             # Try to use the shape of the expected output if raw_img is bad
             cv.putText(default_edge_img, "Camera disabled", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
             cv.putText(default_edge_only, "Camera disabled", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return default_edge_img, default_thresh, default_edge_only


    # --- Image Preprocessing ---
    # Define Region of Interest (ROI) - same as before
    roi_y_start, roi_y_end = 70, 240
    roi_x_start, roi_x_end = 20, 440
    img = raw_img[roi_y_start:roi_y_end, roi_x_start:roi_x_end].copy()
    img_h, img_w = img.shape[:2]
    img_area = img_h * img_w
    if img_area == 0: return default_edge_img, default_thresh, default_edge_only # Avoid division by zero

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), cv.BORDER_DEFAULT)

    # Adaptive thresholding - applied to the whole ROI first
    thresh = cv.adaptiveThreshold(
        blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2 # Inverted threshold might work better
    )

    # --- Split into Left and Right Halves ---
    mid_x = img_w // 2
    left_thresh = thresh[:, :mid_x]
    right_thresh = thresh[:, mid_x:]
    left_area = left_thresh.shape[0] * left_thresh.shape[1]
    right_area = right_thresh.shape[0] * right_thresh.shape[1]

    # --- Contour Analysis for Each Half ---
    min_contour_area = 30 # Minimum area to consider a contour significant

    # Find contours in the left half
    contours_left, _ = cv.findContours(left_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    significant_contours_left = [c for c in contours_left if cv.contourArea(c) > min_contour_area]
    total_contour_area_left = sum(cv.contourArea(c) for c in significant_contours_left)
    local_left_num_contours = len(significant_contours_left)
    local_left_area_percent = (total_contour_area_left / left_area) * 100 if left_area > 0 else 0

    # Find contours in the right half
    # Adjust contour coordinates found in the right half to be relative to the original image ROI
    contours_right, _ = cv.findContours(right_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    significant_contours_right = []
    total_contour_area_right = 0
    for c in contours_right:
        area = cv.contourArea(c)
        if area > min_contour_area:
            # Offset contour points by mid_x
            c_offset = c + (mid_x, 0)
            significant_contours_right.append(c_offset)
            total_contour_area_right += area # Use original area for calculation

    local_right_num_contours = len(significant_contours_right)
    local_right_area_percent = (total_contour_area_right / right_area) * 100 if right_area > 0 else 0


    # --- Update Global Variables ---
    with sensor_lock:
        left_contour_area_percent = local_left_area_percent
        right_contour_area_percent = local_right_area_percent
        left_num_contours = local_left_num_contours
        right_num_contours = local_right_num_contours
        # Update overall metrics if needed (e.g., average or max)
        obstruction_percent = max(local_left_area_percent, local_right_area_percent) # Example: use max
        num_contours = local_left_num_contours + local_right_num_contours # Example: use sum

    # --- Visualization ---
    edge_img = img.copy() # Draw on the color ROI
    edge_only = np.zeros_like(img) # Draw contours on a black background

    # Draw dividing line
    cv.line(edge_img, (mid_x, 0), (mid_x, img_h), (255, 0, 0), 1)
    cv.line(edge_only, (mid_x, 0), (mid_x, img_h), (255, 0, 0), 1)

    # Draw left contours (green)
    cv.drawContours(edge_img, significant_contours_left, -1, (0, 255, 0), 1)
    cv.drawContours(edge_only, significant_contours_left, -1, (0, 255, 0), 1)

    # Draw right contours (blue) - use the offset contours
    cv.drawContours(edge_img, significant_contours_right, -1, (255, 100, 0), 1)
    cv.drawContours(edge_only, significant_contours_right, -1, (255, 100, 0), 1)

    # Add text overlays for left/right info
    cv.putText(edge_img, f"L:{local_left_area_percent:.1f}% ({local_left_num_contours})", (10, img_h - 10),
               cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv.putText(edge_img, f"R:{local_right_area_percent:.1f}% ({local_right_num_contours})", (mid_x + 10, img_h - 10),
               cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

    # Return the visualization images and the original threshold (optional)
    # Note: The returned 'thresh' is for the whole ROI, not split.
    return edge_img, thresh, edge_only


def validate_weights(cam_weight, ultra_weight):
    """
    Validate and normalize sensor weights. Ensures they sum to 1.0 if both > 0.
    """
    cam_weight = max(0.0, min(1.0, cam_weight))
    ultra_weight = max(0.0, min(1.0, ultra_weight))

    if cam_weight == 0 and ultra_weight == 0:
        print("Warning: Both weights are 0. Obstacle detection might not function. Using 0/0.")
        return 0.0, 0.0 # Or revert to defaults?

    # If one is zero, the other must be 1.0 for normalization logic to make sense,
    # but the fusion function handles the weights directly, so just return them.
    # Normalization happens within calculate_obstruction_scores if needed.
    # total = cam_weight + ultra_weight
    # if total > 0 and total != 1.0:
    #     print(f"Normalizing weights: Cam {cam_weight/total:.2f}, Ultra {ultra_weight/total:.2f}")
    #     return cam_weight / total, ultra_weight / total

    return cam_weight, ultra_weight


def main():
    parser = argparse.ArgumentParser(description="Experimental sensor fusion system with independent L/R haptic feedback")
    parser.add_argument("--camera-weight", type=float, default=0.3, help="Weight for camera input (0.0-1.0)") # Increased default camera weight
    parser.add_argument("--ultrasonic-weight", type=float, default=0.7, help="Weight for ultrasonic sensor input (0.0-1.0)") # Decreased default ultrasonic weight
    parser.add_argument("--filter-length", type=int, default=5, help="Length of median filter for distance (1-20)")
    # Add arguments for GPIO pins if they might change
    parser.add_argument("--trig-pin", type=int, default=13, help="GPIO pin (BOARD mode) for ultrasonic trigger")
    parser.add_argument("--echo-pin", type=int, default=11, help="GPIO pin (BOARD mode) for ultrasonic echo")
    parser.add_argument("--left-motor-pin", type=int, default=12, help="GPIO pin (BOARD mode) for left haptic motor")
    parser.add_argument("--right-motor-pin", type=int, default=16, help="GPIO pin (BOARD mode) for right haptic motor")

    args = parser.parse_args()

    global camera_weight, ultrasonic_weight, FILTER_LENGTH, distance_readings
    # Validate and set filter length
    FILTER_LENGTH = max(1, min(20, args.filter_length))
    distance_readings = deque(maxlen=FILTER_LENGTH)
    for _ in range(FILTER_LENGTH):
        distance_readings.append(100) # Initialize with safe distance

    # Validate and set sensor weights
    camera_weight, ultrasonic_weight = validate_weights(args.camera_weight, args.ultrasonic_weight)
    print(f"Using weights: Camera {camera_weight:.2f}, Ultrasonic {ultrasonic_weight:.2f}")
    print(f"Distance filter length: {FILTER_LENGTH}")

    # Assign GPIO pins from args
    SENSOR_A_TRIGGER = args.trig_pin
    SENSOR_A_ECHO = args.echo_pin
    LEFT_VIBRATION_MOTOR_PIN = args.left_motor_pin
    RIGHT_VIBRATION_MOTOR_PIN = args.right_motor_pin
    print(f"Using Pins: Trig={SENSOR_A_TRIGGER}, Echo={SENSOR_A_ECHO}, LeftMotor={LEFT_VIBRATION_MOTOR_PIN}, RightMotor={RIGHT_VIBRATION_MOTOR_PIN}")


    try:
        GPIO.setmode(GPIO.BOARD) # Using physical pin numbering

        # Setup GPIO pins
        GPIO.setup(SENSOR_A_TRIGGER, GPIO.OUT)
        GPIO.output(SENSOR_A_TRIGGER, GPIO.LOW)
        GPIO.setup(SENSOR_A_ECHO, GPIO.IN)

        GPIO.setup(LEFT_VIBRATION_MOTOR_PIN, GPIO.OUT)
        GPIO.output(LEFT_VIBRATION_MOTOR_PIN, GPIO.LOW)
        GPIO.setup(RIGHT_VIBRATION_MOTOR_PIN, GPIO.OUT)
        GPIO.output(RIGHT_VIBRATION_MOTOR_PIN, GPIO.LOW)

        print("Waiting for sensors to settle...")
        time.sleep(1) # Short delay

        # --- Start Sensor and Feedback Threads ---
        ultrasonic_thread = threading.Thread(
            target=ultrasonic_thread_function,
            args=(SENSOR_A_TRIGGER, SENSOR_A_ECHO),
            daemon=True # Allows main thread to exit even if this thread is running
        )
        ultrasonic_thread.start()
        print("Ultrasonic sensor thread started.")

        haptic_thread = threading.Thread(
            target=haptic_feedback_thread,
            args=(LEFT_VIBRATION_MOTOR_PIN, RIGHT_VIBRATION_MOTOR_PIN),
            daemon=True
        )
        haptic_thread.start()
        print("Independent L/R haptic feedback thread started.")

        # --- Initialize Camera ---
        capture = None
        if camera_weight > 0:
            capture = cv.VideoCapture(0) # Use camera index 0
            if not capture.isOpened():
                print("Error: Could not open camera. Disabling camera input.")
                camera_weight = 0 # Disable camera processing if it fails to open
                # Re-validate weights if camera is disabled after initial setup
                camera_weight, ultrasonic_weight = validate_weights(camera_weight, ultrasonic_weight)
                print(f"Updated weights: Camera {camera_weight:.2f}, Ultrasonic {ultrasonic_weight:.2f}")
            else:
                 print("Camera initialized successfully.")
                 # Optional: Set camera properties like resolution if needed
                 # capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                 # capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            print("Camera processing is disabled (weight=0).")

        # --- Logging Setup ---
        log_file = "experimental_fusion_log.txt" # Use a different log file
        last_log_time = 0
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_file, "a") as f:
            f.write(f"\n--- New Experimental Session Started: {timestamp} ---\n")
            f.write(f"Weights: Camera={camera_weight:.2f}, Ultrasonic={ultrasonic_weight:.2f}\n")
            f.write(f"Filter: Length={FILTER_LENGTH}\n")
            f.write(f"Pins: Trig={SENSOR_A_TRIGGER}, Echo={SENSOR_A_ECHO}, LeftMotor={LEFT_VIBRATION_MOTOR_PIN}, RightMotor={RIGHT_VIBRATION_MOTOR_PIN}\n")
            f.write(f"Haptic Feedback: Independent L/R Enabled\n")

        print("Starting main loop... Press 'q' in OpenCV window to quit.")
        # --- Main Loop ---
        while True:
            # --- Read Camera Frame ---
            raw_img = None
            if camera_weight > 0 and capture is not None and capture.isOpened():
                isTrue, raw_img = capture.read()
                if not isTrue:
                    print("Error: Failed to capture frame from camera. Exiting.")
                    break # Exit loop if camera fails mid-run
            elif camera_weight <= 0:
                 # Create a blank image if camera is disabled to keep loop structure
                 raw_img = np.zeros((480, 640, 3), dtype=np.uint8)
                 cv.putText(raw_img, "Camera Disabled", (150, 240),
                            cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)


            # --- Process Frame for Visual Obstructions (L/R) ---
            # process_frame now updates global L/R visual metrics
            overlay, thresh, edge_only = process_frame(raw_img)

            # --- Calculate Fused Scores (L/R) ---
            # This reads global distance_cm and L/R visual metrics
            current_left_score, current_right_score = calculate_obstruction_scores()

            # --- Update Global Scores Under Lock ---
            # (haptic thread reads these)
            with sensor_lock:
                left_obstruction_score = current_left_score
                right_obstruction_score = current_right_score
                # Read current distance for display consistency
                current_distance = distance_cm
                # Read visual metrics for display
                vis_left_perc = left_contour_area_percent
                vis_right_perc = right_contour_area_percent
                vis_left_count = left_num_contours
                vis_right_count = right_num_contours
                # Read haptic intensities for display
                haptic_l = vibration_intensity_left
                haptic_r = vibration_intensity_right


            # --- Display / Visualization ---
            if camera_weight > 0:
                cv.imshow("ROI Overlay (L/R Contours)", overlay)
                # cv.imshow("Thresholded ROI", thresh) # Optional: show threshold
                cv.imshow("Edge Detection (L/R)", edge_only)

            # Create status display window
            status_h, status_w = 220, 550 # Increased height slightly
            status_display = np.ones((status_h, status_w, 3), dtype=np.uint8) * 255 # White background

            # Determine overall status based on max score (for simple text/border)
            max_score = max(current_left_score, current_right_score)
            status_threshold = 30 # Match haptic threshold? Or use a different one for display?

            if max_score > status_threshold:
                status_text = "WARNING: OBSTACLE NEAR"
                status_color = (0, 0, 255) # Red
                border_color = [0, 0, 255]
            else:
                status_text = "Path Appears Clear"
                status_color = (0, 128, 0) # Green
                border_color = [0, 128, 0]

            cv.putText(status_display, status_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

            # Display sensor readings and scores
            y_pos = 60
            if ultrasonic_weight > 0:
                cv.putText(status_display, f"Distance: {current_distance:.1f} cm", (10, y_pos), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
                y_pos += 25

            if camera_weight > 0:
                 cv.putText(status_display, f"Vis L: {vis_left_perc:.1f}% ({vis_left_count}) | R: {vis_right_perc:.1f}% ({vis_right_count})",
                            (10, y_pos), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
                 y_pos += 25

            # Display fused scores
            cv.putText(status_display, f"Fused L: {current_left_score:.1f}% | R: {current_right_score:.1f}%",
                       (10, y_pos), cv.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            y_pos += 30

            # Display haptic status
            haptic_status_l = f"Haptic L: {'ON' if haptic_l > 0 else 'OFF'} ({haptic_l:.0f}%)"
            haptic_status_r = f"Haptic R: {'ON' if haptic_r > 0 else 'OFF'} ({haptic_r:.0f}%)"
            cv.putText(status_display, haptic_status_l, (status_w // 2, 60), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0) if haptic_l == 0 else status_color, 1)
            cv.putText(status_display, haptic_status_r, (status_w // 2, 85), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0) if haptic_r == 0 else status_color, 1)


            # Display weights
            cv.putText(status_display, f"Weights: Cam {camera_weight:.2f}, Ultra {ultrasonic_weight:.2f}",
                       (10, status_h - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

            # Add border
            status_display = cv.copyMakeBorder(status_display, 3, 3, 3, 3, cv.BORDER_CONSTANT, value=border_color)

            cv.imshow("Sensor Fusion Status (Experimental L/R)", status_display)


            # --- Logging ---
            # Log if either score is high
            log_threshold = 50 # Log more frequently if score is moderately high
            current_time = time.time()
            if (current_left_score > log_threshold or current_right_score > log_threshold) and (current_time - last_log_time > 3): # Log every 3s if obstacle
                 timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                 with open(log_file, "a") as f:
                      log_line = f"{timestamp} - Dist:{current_distance:.1f} | Vis L:{vis_left_perc:.1f}({vis_left_count}) R:{vis_right_perc:.1f}({vis_right_count}) | Fused L:{current_left_score:.1f} R:{current_right_score:.1f} | Haptic L:{haptic_l:.0f} R:{haptic_r:.0f}\n"
                      f.write(log_line)
                 last_log_time = current_time


            # --- Check for Quit Key ---
            if cv.waitKey(1) & 0xFF == ord('q'):
                print("Quit key pressed. Exiting loop.")
                break

        # --- End of Loop Cleanup ---
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_file, "a") as f:
            f.write(f"--- Session Ended: {timestamp} ---\n")

        if capture is not None:
            capture.release()
            print("Camera released.")
        cv.destroyAllWindows()
        print("OpenCV windows closed.")

    except KeyboardInterrupt:
        print("\nProgram stopped by user (KeyboardInterrupt).")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        # Log the error
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_file, "a") as f:
             f.write(f"--- ERROR at {timestamp}: {e} ---\n")

    finally:
        # Ensure GPIO cleanup happens
        print("Cleaning up GPIO...")
        # Stop PWM first (inside haptic thread's finally block, but also good to try here)
        if 'haptic_left_pwm' in globals() and haptic_left_pwm is not None:
             try: haptic_left_pwm.stop()
             except: pass # Ignore errors if already stopped
        if 'haptic_right_pwm' in globals() and haptic_right_pwm is not None:
             try: haptic_right_pwm.stop()
             except: pass
        GPIO.cleanup()
        print("GPIO cleanup complete.")


if __name__ == "__main__":
    # Ensure necessary libraries are imported at the top
    import cv2 as cv
    import numpy as np
    import time
    try:
        import RPi.GPIO as GPIO
    except RuntimeError:
        print("Error importing RPi.GPIO! This is probably because you need superuser privileges. You can achieve this using 'sudo'.")
        # You might want to exit here or use a mock GPIO for testing
        # For now, let's try to define a mock GPIO for basic structure testing
        class MockGPIO:
             BOARD = 1
             OUT = 1
             IN = 1
             LOW = 0
             HIGH = 1
             PWM = lambda *args: MockPWM()
             def setmode(self, mode): print(f"MockGPIO: setmode({mode})")
             def setup(self, pin, mode): print(f"MockGPIO: setup({pin}, {mode})")
             def output(self, pin, state): print(f"MockGPIO: output({pin}, {state})")
             def input(self, pin): print(f"MockGPIO: input({pin}) -> LOW"); return self.LOW
             def cleanup(self): print("MockGPIO: cleanup()")
        class MockPWM:
             def start(self, dc): print(f"MockPWM: start({dc})")
             def ChangeDutyCycle(self, dc): print(f"MockPWM: ChangeDutyCycle({dc})")
             def stop(self): print("MockPWM: stop()")
        GPIO = MockGPIO()
        print("\n*** Using Mock RPi.GPIO for testing purposes ***\n")


    import threading
    import argparse
    from collections import deque
    import math

    main()
