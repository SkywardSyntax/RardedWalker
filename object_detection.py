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
is_visual_obstructed = False
obstruction_percent = 0
scene_complexity = 0
num_contours = 0

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

# Variable to track if we need to update the motor state
motor_update_needed = False
last_fused_assessment = 0

# Haptic feedback variables
vibration_active = False
vibration_intensity = 0  # 0-100%
PWM_FREQUENCY = 100  # Hz for PWM control
haptic_pwm = None  # Will hold the PWM object

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
        return (sorted_readings[mid-1] + sorted_readings[mid]) / 2
    else:
        return sorted_readings[mid]

def set_haptic_intensity(intensity):
    """
    Set the haptic motor vibration intensity using PWM
    intensity: 0-100 where 0 is off and 100 is maximum vibration
    """
    global haptic_pwm, vibration_active, vibration_intensity
    
    # Limit intensity to valid range
    intensity = max(0, min(100, intensity))
    vibration_intensity = intensity
    
    # Update vibration state
    if intensity > 0 and not vibration_active:
        vibration_active = True
    elif intensity == 0 and vibration_active:
        vibration_active = False

    # Set PWM duty cycle based on intensity
    if haptic_pwm:
        haptic_pwm.ChangeDutyCycle(intensity)

def get_haptic_pattern_intensity(assessment, distance):
    """
    Generates a haptic vibration pattern based on obstacle assessment and distance
    Returns intensity value between 0-100
    """
    if assessment <= 70:
        return 0  # No vibration when path is clear
    
    # Base intensity increases with assessment score
    base_intensity = (assessment - 70) * (100/30)  # Scale from 70-100 to 0-100
    
    # Distance factor - stronger vibration for closer objects
    # Increase intensity for very close objects (under 30cm)
    distance_factor = 1.0
    if distance < 30:
        distance_factor = 1 + ((30 - distance) / 30)  # 1.0-2.0 factor
    
    # Calculate final intensity with distance consideration
    intensity = min(100, base_intensity * distance_factor)
    
    return intensity

def haptic_feedback_thread(motor_pin):
    """Thread for controlling haptic feedback patterns"""
    global vibration_active, vibration_intensity, haptic_pwm
    
    # Setup PWM for vibration motor
    haptic_pwm = GPIO.PWM(motor_pin, PWM_FREQUENCY)
    haptic_pwm.start(0)  # Start with 0% duty cycle (off)
    
    try:
        while True:
            # Get current values under lock
            with sensor_lock:
                current_distance = distance_cm
                current_assessment = calculate_obstruction_score()
            
            # Calculate appropriate intensity based on assessment and distance
            intensity = get_haptic_pattern_intensity(current_assessment, current_distance)
            
            # Update motor intensity
            set_haptic_intensity(intensity)
            
            # Brief sleep to avoid excessive CPU usage
            time.sleep(0.05)
    except Exception as e:
        print(f"Haptic feedback thread error: {e}")
    finally:
        if haptic_pwm:
            haptic_pwm.stop()

def ultrasonic_thread_function(trigger_pin, echo_pin, motor_pin):
    """Thread function to continuously read the ultrasonic sensor with optimized performance"""
    global distance_cm, distance_readings, ultrasonic_weight, motor_update_needed, last_fused_assessment
    
    # Track timing for adaptive sensing
    last_significant_change_time = time.time()
    baseline_delay = 0.05  # 50ms base delay between readings (reduced from 200ms)
    current_delay = baseline_delay
    last_distance = 100
    significant_change_threshold = 5  # cm
    
    try:
        while True:
            if ultrasonic_weight > 0:
                # Measure and filter distance
                measured_distance = measure_distance(trigger_pin, echo_pin)
                
                # Quick pre-check if value is reasonable before more processing
                if measured_distance < 2 or measured_distance > 400:
                    measured_distance = last_distance  # Reject obviously bad readings
                
                # More efficient outlier detection
                if distance_readings:
                    avg = sum(distance_readings) / len(distance_readings)
                    if abs(measured_distance - avg) > (avg * 0.5):
                        measured_distance = max(min(measured_distance, avg * 1.5), avg * 0.5)
                
                distance_readings.append(measured_distance)
                filtered_distance = apply_median_filter(distance_readings)
                
                # Detect significant changes to adapt sensing rate
                distance_change = abs(filtered_distance - last_distance)
                now = time.time()
                
                if distance_change > significant_change_threshold:
                    last_significant_change_time = now
                    current_delay = baseline_delay  # Fast update when distance is changing
                    motor_update_needed = True
                else:
                    time_since_change = now - last_significant_change_time
                    # Gradually slow down updates if distance is stable
                    if time_since_change > 1.0:  # After 1 second of stability
                        current_delay = min(0.2, baseline_delay * (1 + time_since_change/5))  # Max 200ms
                
                with sensor_lock:
                    distance_cm = filtered_distance
                    last_distance = filtered_distance
            
            time.sleep(current_delay)  # Adaptive delay
    except Exception as e:
        print("Ultrasonic thread encountered an error:", e)

def calculate_obstruction_score():
    """
    Calculate a fused obstruction score (0-100%).
    Updated to heavily favor the ultrasonic sensor readings.
    """
    global distance_cm, is_visual_obstructed, obstruction_percent
    global camera_weight, ultrasonic_weight, scene_complexity

    # Setup effective sensor weights
    actual_camera_weight = camera_weight
    actual_ultrasonic_weight = ultrasonic_weight

    if camera_weight == 0 and ultrasonic_weight > 0:
        actual_ultrasonic_weight = 1.0
    elif ultrasonic_weight == 0 and camera_weight > 0:
        actual_camera_weight = 1.0
    elif camera_weight == 0 and ultrasonic_weight == 0:
        return 0  # Default to safe state if no sensor is trusted

    # Only calculate distance score if ultrasonic has weight
    distance_score = 0
    if actual_ultrasonic_weight > 0:
        # Distance-based score: lower distance yields higher score.
        # More aggressive curve: 0cm -> 100%, 40cm -> 0% (changed from 50cm)
        distance_score = max(0, min(100, 100 - (distance_cm * 2.5)))
        
        # Apply additional emphasis to close distances
        if distance_cm < 20:
            # Further boost score for very close objects
            distance_boost = max(0, 20 - distance_cm) * 1.5
            distance_score = min(100, distance_score + distance_boost)
    
    # Only calculate visual score if camera has weight
    visual_score = 0
    if actual_camera_weight > 0:
        if is_visual_obstructed:
            visual_score = obstruction_percent
        else:
            visual_score = max(0, 50 - scene_complexity / 2)
    
    # Fused score combining both, with more trust on distance sensor
    fused_score = (visual_score * actual_camera_weight) + (distance_score * actual_ultrasonic_weight)
    return fused_score

def process_frame(raw_img):
    """Process camera frame to detect obstructions."""
    global is_visual_obstructed, obstruction_percent, scene_complexity, num_contours, camera_weight
    global motor_update_needed
    
    if camera_weight <= 0:
        if raw_img is not None:
            h, w = raw_img.shape[:2]
            edge_img = np.zeros((h, w, 3), dtype=np.uint8)
            thresh = np.zeros((h, w), dtype=np.uint8)
            edge_only = np.zeros((h, w, 3), dtype=np.uint8)
            cv.putText(edge_img, "Camera disabled (weight=0)", (10, 30), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv.putText(edge_only, "Camera disabled (weight=0)", (10, 30), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return edge_img, thresh, edge_only
        else:
            return np.zeros((100, 100, 3), dtype=np.uint8), \
                   np.zeros((100, 100), dtype=np.uint8), \
                   np.zeros((100, 100, 3), dtype=np.uint8)
    
    img = raw_img[70:240, 20:440].copy()
    img_area = img.shape[0] * img.shape[1]

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), cv.BORDER_DEFAULT)
    
    thresh = cv.adaptiveThreshold(
        blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2
    )
    
    contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    edge_img = img.copy()
    edge_only = np.zeros_like(img)
    
    min_contour_area = 30
    significant_contours = [c for c in contours if cv.contourArea(c) > min_contour_area]
    
    for contour in significant_contours:
        cv.drawContours(edge_img, [contour], -1, (0, 255, 0), 2)
        cv.drawContours(edge_only, [contour], -1, (0, 255, 0), 2)
    
    local_num_contours = len(significant_contours)
    local_obstruction_percent = 0
    local_is_obstructed = False
    
    if significant_contours:
        sorted_contours = sorted(significant_contours, key=cv.contourArea, reverse=True)
        largest_contour_area = cv.contourArea(sorted_contours[0])
        local_obstruction_percent = (largest_contour_area / img_area) * 100
        
        area_threshold = 30
        contour_count_threshold = 15
        local_is_obstructed = (local_obstruction_percent > area_threshold and 
                               local_num_contours < contour_count_threshold)
        
        if len(sorted_contours) > 1:
            c = sorted_contours[1]
            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            box = np.int32(box)
            cv.polylines(edge_img, [box], isClosed=True, color=(0, 0, 255), thickness=2)
            cv.polylines(edge_only, [box], isClosed=True, color=(0, 0, 255), thickness=2)
            centerX, centerY = map(int, rect[0])
            cv.circle(edge_img, (centerX, centerY), 5, (255, 0, 0), -1)
            cv.circle(edge_only, (centerX, centerY), 5, (255, 0, 0), -1)
    
    local_scene_complexity = min(100, local_num_contours * 5)
    
    # Update global variables and flag for motor update if there's a significant change
    prev_obstruction = is_visual_obstructed
    prev_percent = obstruction_percent
    
    with sensor_lock:
        is_visual_obstructed = local_is_obstructed
        obstruction_percent = local_obstruction_percent
        scene_complexity = local_scene_complexity
        num_contours = local_num_contours
        
        # Signal motor update if visual obstruction state changed significantly
        if (is_visual_obstructed != prev_obstruction or 
                abs(obstruction_percent - prev_percent) > 15):
            motor_update_needed = True
    
    if local_is_obstructed:
        warning_text = f"CAUTION: VIEW OBSTRUCTED ({local_obstruction_percent:.1f}%)"
        cv.putText(edge_img, warning_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv.putText(edge_only, warning_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        edge_img = cv.copyMakeBorder(edge_img, 5, 5, 5, 5, cv.BORDER_CONSTANT, value=[0, 0, 255])
        edge_only = cv.copyMakeBorder(edge_only, 5, 5, 5, 5, cv.BORDER_CONSTANT, value=[0, 0, 255])
    
    complexity_text = f"Scene Complexity: {local_scene_complexity}/100"
    cv.putText(edge_img, complexity_text, (10, img.shape[0] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv.putText(edge_img, f"Contours: {local_num_contours}", (10, img.shape[0] - 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return edge_img, thresh, edge_only

def validate_weights(cam_weight, ultra_weight):
    """
    Validate and normalize sensor weights.
    """
    if cam_weight == 0 and ultra_weight == 0:
        print("Warning: Both weights were set to 0. Using default weights.")
        return 0.15, 0.85
        
    if cam_weight == 0:
        return 0, 1.0
    if ultra_weight == 0:
        return 1.0, 0
    
    total = cam_weight + ultra_weight
    if total != 1.0:
        return cam_weight / total, ultra_weight / total
    
    return cam_weight, ultra_weight

def main():
    parser = argparse.ArgumentParser(description="Sensor fusion system for obstacle detection with haptic feedback")
    parser.add_argument("--camera-weight", type=float, default=0.15, help="Weight for camera input (0.0-1.0)")
    parser.add_argument("--ultrasonic-weight", type=float, default=0.85, help="Weight for ultrasonic sensor input (0.0-1.0)")
    parser.add_argument("--filter-length", type=int, default=5, help="Length of moving average filter (1-20)")
    args = parser.parse_args()
    
    global camera_weight, ultrasonic_weight, FILTER_LENGTH, distance_readings
    FILTER_LENGTH = max(1, min(20, args.filter_length))
    distance_readings = deque(maxlen=FILTER_LENGTH)
    for _ in range(FILTER_LENGTH):
        distance_readings.append(100)
    
    camera_weight, ultrasonic_weight = validate_weights(args.camera_weight, args.ultrasonic_weight)
    print(f"Using weights: Camera {camera_weight:.2f}, Ultrasonic {ultrasonic_weight:.2f}")
    print(f"Filter length: {FILTER_LENGTH}")
        
    try:
        GPIO.setmode(GPIO.BOARD)
        
        SENSOR_A_TRIGGER = 13
        SENSOR_A_ECHO = 11
        VIBRATION_MOTOR_PIN = 12  # Renamed for clarity
        
        GPIO.setup(SENSOR_A_TRIGGER, GPIO.OUT)
        GPIO.output(SENSOR_A_TRIGGER, GPIO.LOW)
        GPIO.setup(SENSOR_A_ECHO, GPIO.IN)
        GPIO.setup(VIBRATION_MOTOR_PIN, GPIO.OUT)
        GPIO.output(VIBRATION_MOTOR_PIN, GPIO.LOW)
        
        print("Waiting for sensors to settle...")
        time.sleep(1)  # Reduced from 2 seconds
        
        # Start the ultrasonic sensing thread
        ultrasonic_thread = threading.Thread(
            target=ultrasonic_thread_function,
            args=(SENSOR_A_TRIGGER, SENSOR_A_ECHO, VIBRATION_MOTOR_PIN),
            daemon=True
        )
        ultrasonic_thread.start()
        
        # Start the haptic feedback thread
        haptic_thread = threading.Thread(
            target=haptic_feedback_thread,
            args=(VIBRATION_MOTOR_PIN,),
            daemon=True
        )
        haptic_thread.start()
        print("Haptic feedback system initialized")
        
        capture = None
        if camera_weight > 0:
            capture = cv.VideoCapture(0)
            if not capture.isOpened():
                print("Error: Could not open camera.")
                return
        else:
            print("Camera disabled (weight=0)")
        
        log_file = "fusion_log.txt"
        last_log_time = 0
        
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_file, "a") as f:
            f.write(f"\n--- New session started at {timestamp} by SkywardSyntax ---\n")
            f.write(f"Sensor weights: Camera {camera_weight:.2f}, Ultrasonic {ultrasonic_weight:.2f}\n")
            f.write(f"Filter settings: Length={FILTER_LENGTH}\n")
            f.write(f"Haptic feedback: Enabled\n")
        
        print("Press 'q' to quit the loop.")
        while True:
            if camera_weight > 0 and capture is not None:
                isTrue, raw_img = capture.read()
                if not isTrue:
                    print("Failed to capture frame from camera.")
                    break
            else:
                raw_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv.putText(raw_img, "Camera disabled (weight=0)", (150, 240), 
                           cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                
            overlay, thresh, edge_only = process_frame(raw_img)
            
            with sensor_lock:
                current_distance = distance_cm
                current_visual_obstructed = is_visual_obstructed
                current_obstruction_percent = obstruction_percent
                current_scene_complexity = scene_complexity
                current_num_contours = num_contours
                fused_score = calculate_obstruction_score()
            
            if camera_weight > 0:
                cv.imshow("Overlay with Edges", overlay)
                cv.imshow("Processed Binary", thresh)
                cv.imshow("Edge Detection", edge_only)
            
            status_display = np.ones((200, 500, 3), dtype=np.uint8) * 255
            
            if fused_score > 70:
                status_text = "WARNING: OBSTACLE DETECTED"
                status_color = (0, 0, 255)
                border_color = [0, 0, 255]
                haptic_status = f"Haptic: ON ({vibration_intensity:.0f}%)"
                
                current_time = time.time()
                if current_time - last_log_time > 3:
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    with open(log_file, "a") as f:
                        if camera_weight > 0:
                            f.write(f"{timestamp} - Obstacle detected: Visual {current_obstruction_percent:.1f}%, "
                                   f"Distance {current_distance:.1f}cm, Fused score: {fused_score:.1f}%\n")
                        else:
                            f.write(f"{timestamp} - Obstacle detected: Distance {current_distance:.1f}cm, "
                                   f"Fused score: {fused_score:.1f}%\n")
                    last_log_time = current_time
            else:
                status_text = "Path Clear"
                status_color = (0, 128, 0)
                border_color = [0, 128, 0]
                haptic_status = "Haptic: OFF"
                
            cv.putText(status_display, status_text, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1.2, status_color, 2)
            y_pos = 80
            if ultrasonic_weight > 0:
                cv.putText(status_display, f"Distance: {current_distance:.1f} cm", (10, y_pos), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
                y_pos += 30
            
            if camera_weight > 0:
                cv.putText(status_display, f"Visual obstruction: {current_obstruction_percent:.1f}%", (10, y_pos), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
                y_pos += 30
                cv.putText(status_display, f"Scene complexity: {current_scene_complexity:.1f}/100", (10, y_pos), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
                y_pos += 30
            
            cv.putText(status_display, f"FUSION SCORE: {fused_score:.1f}%", (10, 180), cv.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv.putText(status_display, haptic_status, (240, 40), cv.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv.putText(status_display, f"Weights: Camera {camera_weight:.2f}, Ultrasonic {ultrasonic_weight:.2f}",
                       (240, 180), cv.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            
            status_display = cv.copyMakeBorder(status_display, 3, 3, 3, 3, cv.BORDER_CONSTANT, value=border_color)
                      
            cv.imshow("Sensor Fusion Status", status_display)
            
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
                
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_file, "a") as f:
            f.write(f"--- Session ended at {timestamp} ---\n")
            
        if capture is not None:
            capture.release()
        cv.destroyAllWindows()
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        if 'haptic_pwm' in globals() and haptic_pwm is not None:
            haptic_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()