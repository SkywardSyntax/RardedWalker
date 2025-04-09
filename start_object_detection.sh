#!/bin/bash

# Start the object detection script
# Path to the Python script - adjust if needed
SCRIPT_DIR="//home/radude184/RardedWalker"
LOG_FILE="$SCRIPT_DIR/startup_log.txt"

# Log the startup time
echo "Starting object detection script at $(date)" >> "$LOG_FILE"

# Make sure we're in the right directory
cd "$SCRIPT_DIR"

# Wait a bit for the system to fully initialize and for camera/GPIO to be available
sleep 10

# Run the object detection script
# You can add any command line arguments here if needed
python3 object_detection.py >> "$LOG_FILE" 2>&1

# Log if the script exits
echo "Object detection script exited at $(date) with code $?" >> "$LOG_FILE"
